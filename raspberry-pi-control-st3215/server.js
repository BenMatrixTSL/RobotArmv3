/**
 * Robot Arm WebSocket Server (ST3215 Version)
 * 
 * This server runs on the Raspberry Pi and connects the Electron desktop app
 * to the ST3215 serial bus servo motors. It receives commands from the Electron app
 * and translates them to serial commands for the ST3215 servos.
 * 
 * Usage:
 *   node server.js
 * 
 * This server listens on port 8080 by default and communicates with:
 * - Electron desktop app (via WebSocket)
 * - ST3215 servos (via Serial/UART)
 */

const WebSocket = require('ws');
const os = require('os');
const fs = require('fs');
const path = require('path');
const { exec } = require('child_process');
const { execFile } = require('child_process');
const { promisify } = require('util');
const RobotArm = require('./robotArmST3215');
const { robotKinematics } = require('./kinematicsService');
const execFileAsync = promisify(execFile);

// Configuration
const PORT = 8080;
const JOINT_COUNT = 6; // Number of robot arm joints (ST3215 servos)
const SERVO_IDS = [1, 2, 3, 4, 5, 6];
// Serial port path:
// - Default uses Raspberry Pi UART pins via /dev/serial0
// - You can override with environment variable SERIAL_PORT, for example:
//   SERIAL_PORT=/dev/ttyUSB0 node server.js
const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/serial0';
//const SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0158625-if00'; // Serial port path (adjust as needed)

const SERIAL_BAUDRATE = 1000000; // ST3215 default baud rate

// Debug flag - set to true to enable verbose debug messages
const DEBUG = false;
// Simple performance logging flag (set to true to see timing info)
const PERF_DEBUG = false;
// Log bus diagnostics to console/debug log every N ms (0 = off)
const BUS_DIAGNOSTICS_LOG_INTERVAL_MS = parseInt(process.env.BUS_DIAGNOSTICS_LOG_INTERVAL_MS || '0', 10);

// Minimum gap between bus ticks (ms). Ticks are chained after work finishes — not a fixed overlap.
const STATUS_POLL_INTERVAL_MS = parseInt(process.env.STATUS_POLL_INTERVAL_MS || '20', 10);
const MIN_BUS_TICK_GAP_MS = parseInt(process.env.MIN_BUS_TICK_GAP_MS || '4', 10);
const JOINTS_PER_POLL_TICK = parseInt(process.env.JOINTS_PER_POLL_TICK || '6', 10);
const MAX_BUS_WRITE_QUEUE_SIZE = 100;
const MAX_BUS_WRITES_WHEN_IDLE = 0;
const MAX_BUS_WRITES_WHEN_BUSY = 4;
const SERVER_BUILD_ID = '2026-06-05-low-latency';
const BUS_QUIET_BEFORE_MOVE_MS = 12;
const BUS_QUIET_AFTER_MOVE_MS = 10;
const BUS_QUIET_JOG_MS = 4;

// Jump to the front of the bus write queue (ahead of status-related work).
const PRIORITY_BUS_COMMANDS = {
    moveJoint: true,
    stopJoint: true,
    stopAll: true,
    stopAllJoints: true,
    setTorqueAll: true
};

// Optional file log (set by install-service.sh: ROBOT_ARM_DEBUG_LOG)
const DEBUG_LOG_FILE = process.env.ROBOT_ARM_DEBUG_LOG || '';

/**
 * Write a line to the console and optionally to the debug log file.
 * @param {string} message
 * @param {boolean} isError - use stderr style prefix
 */
function debugLog(message, isError) {
    const text = String(message);
    const prefix = isError ? '[ERROR] ' : '';
    const line = '[' + new Date().toISOString() + '] ' + prefix + text;

    if (isError) {
        console.error(text);
    } else {
        console.log(text);
    }

    if (!DEBUG_LOG_FILE) {
        return;
    }

    try {
        const logDir = path.dirname(DEBUG_LOG_FILE);
        if (!fs.existsSync(logDir)) {
            fs.mkdirSync(logDir, { recursive: true });
        }
        fs.appendFileSync(DEBUG_LOG_FILE, line + '\n');
    } catch (logError) {
        console.error('Could not write debug log file:', logError.message);
    }
}

debugLog('Server process starting (pid ' + process.pid + ', build ' + SERVER_BUILD_ID + ')');
if (DEBUG_LOG_FILE) {
    debugLog('Debug log file: ' + DEBUG_LOG_FILE);
}

// Log unexpected promise rejections (bus faults should be caught by command handlers)
process.on('unhandledRejection', (reason) => {
    if (reason && reason.name === 'BusCommunicationError') {
        debugLog('Unhandled bus fault (should be caught by command): ' + reason.message, true);
        return;
    }

    const message = reason && reason.stack ? reason.stack : String(reason);
    debugLog('Unhandled promise rejection: ' + message, true);
});

/**
 * User-friendly message for WebSocket error responses.
 * @param {Error} error
 * @returns {string}
 */
function formatCommandError(error) {
    if (!error) {
        return 'Unknown error';
    }

    let message = error.message || String(error);

    if (error.isOverload) {
        message = message + ' Reduce speed, check for a mechanical limit, or relieve load on the joint.';
    }

    return message;
}

process.on('uncaughtException', (error) => {
    const message = error && error.stack ? error.stack : String(error);
    debugLog('Uncaught exception: ' + message, true);
    process.exit(1);
});

process.on('exit', (code) => {
    debugLog('Process exit, code=' + code);
});

// Array to store servo controllers
const servos = [];
let endTool = null;

// Cached joint status (updated by bus tick; served to clients via getStatus)
const jointStatusCache = [];
let busTickTimer = null;
let busTickLoopActive = false;
let busTickInProgress = false;
let busWriteQueue = [];
let statusPollJointIndex = 0;
const pendingImmediateMoves = {};

// Offline servo backoff: after SERVO_BACKOFF_FAIL_THRESHOLD consecutive read
// failures, skip that servo for SERVO_BACKOFF_DURATION_MS before retrying.
// Prevents a dead servo from consuming 50 ms of timeout every single tick.
const SERVO_BACKOFF_FAIL_THRESHOLD = 3;
const SERVO_BACKOFF_DURATION_MS = 2000;
const servoConsecFails = new Array(JOINT_COUNT).fill(0);
const servoBackoffUntilMs = new Array(JOINT_COUNT).fill(0);
let immediateMoveDrainRunning = false;
let lastStatusBroadcastAt = 0;

// Live metrics for debugging bus vs WebSocket load (see getServerDiagnostics).
const serverDiagnostics = {
    startedAt: Date.now(),
    busTickIntervalMs: STATUS_POLL_INTERVAL_MS,
    busTicks: 0,
    busTicksSkipped: 0,
    busTickErrors: 0,
    lastBusTickAt: null,
    lastBusTickDurationMs: 0,
    busWriteQueueLength: 0,
    busWritesCompleted: 0,
    busMovesCompleted: 0,
    busWritesFailed: 0,
    busWritesRejected: 0,
    buildId: SERVER_BUILD_ID,
    lastBusWriteAt: null,
    lastBusWriteDurationMs: 0,
    statusPollsCompleted: 0,
    statusPollJointIndex: 0,
    lastStatusPollAt: null,
    lastStatusPollDurationMs: 0,
    lastBroadcastAt: null,
    cacheAgeMs: null,
    wsClients: 0,
    instantCommands: 0,
    immediateBusCommands: 0,
    writeTimeouts: 0
};

// WebSocket clients (for pushing status to every app instance)
const connectedClients = new Set();
let wss = null;
let serverShuttingDown = false;
const SHUTDOWN_BUS_WAIT_MS = 2000;
const SHUTDOWN_SERIAL_CLOSE_MS = 2000;
const SHUTDOWN_FORCE_EXIT_MS = 8000;

// Cached servo discovery info (getJointConfigs — no bus read per client)
let cachedJointConfigs = null;

// Cached per-joint speed/acceleration (skip bus write when unchanged)
const jointSettingsCache = [];

// Cached global torque state
let cachedTorqueEnabled = null;

// One client at a time may send move/torque/stop commands (multi-app safety)
const CONTROL_IDLE_TIMEOUT_MS = 5 * 60 * 1000;
const CONTROL_IDLE_CHECK_INTERVAL_MS = 30 * 1000;
const controlSession = { ws: null, label: '', hostname: null, clientIp: null, since: null, lastMoveAt: null };

function normalizeClientIp(raw) {
    if (!raw || typeof raw !== 'string') {
        return null;
    }
    if (raw.startsWith('::ffff:')) {
        return raw.substring(7);
    }
    if (raw === '::1') {
        return '127.0.0.1';
    }
    return raw;
}

function isLocalClientIp(ip) {
    return !ip || ip === '127.0.0.1';
}

function getWsClientHostname(ws) {
    if (ws && ws.clientHostname) {
        return ws.clientHostname;
    }
    if (ws && ws.clientIp && isLocalClientIp(ws.clientIp)) {
        try {
            return os.hostname();
        } catch (error) {
            return null;
        }
    }
    return null;
}

function assignControlSession(ws, label) {
    controlSession.ws = ws;
    controlSession.label = (typeof label === 'string' && label) ? label : 'client';
    controlSession.clientIp = ws ? ws.clientIp : null;
    controlSession.hostname = ws ? getWsClientHostname(ws) : null;
    controlSession.since = Date.now();
    controlSession.lastMoveAt = Date.now();
}

function touchControlMoveActivity(ws) {
    if (controlSession.ws === ws) {
        controlSession.lastMoveAt = Date.now();
    }
}

function broadcastControlStatus(extraForClient) {
    connectedClients.forEach((clientWs) => {
        if (clientWs.readyState !== WebSocket.OPEN) {
            return;
        }
        const payload = getControlStatusPayload(clientWs);
        if (extraForClient && extraForClient.ws === clientWs) {
            Object.assign(payload, extraForClient.data);
        }
        clientWs.send(JSON.stringify(payload));
    });
}

function releaseControlForIdle() {
    if (!controlSession.ws || !controlSession.lastMoveAt) {
        return;
    }
    if (Date.now() - controlSession.lastMoveAt < CONTROL_IDLE_TIMEOUT_MS) {
        return;
    }

    const idleHolderWs = controlSession.ws;
    releaseControlSession(idleHolderWs);
    broadcastControlStatus({
        ws: idleHolderWs,
        data: {
            message: 'Arm control released automatically after 5 minutes with no movement'
        }
    });
    console.log('Arm control released due to 5 minute idle timeout');
}

function formatControlHolder(info) {
    if (!info || !info.ws) {
        return null;
    }

    const hostname = info.hostname || getWsClientHostname(info.ws);
    const ip = info.clientIp || (info.ws ? info.ws.clientIp : null);
    const label = info.label;

    if (hostname && ip && !isLocalClientIp(ip)) {
        return hostname + ' (' + ip + ')';
    }
    if (hostname) {
        if (isLocalClientIp(ip)) {
            return hostname + ' (this Pi)';
        }
        return hostname;
    }
    if (ip && !isLocalClientIp(ip)) {
        return ip;
    }
    if (ip && isLocalClientIp(ip)) {
        try {
            return os.hostname() + ' (this Pi)';
        } catch (error) {
            return 'this Pi';
        }
    }
    if (label && label !== 'auto' && label !== 'client' && label !== 'electron') {
        return label;
    }
    return 'another app';
}

function getControlStatusPayload(ws, extra) {
    const payload = {
        type: 'controlStatus',
        hasControl: controlSession.ws === ws,
        youHaveControl: controlSession.ws === ws,
        hasHolder: !!controlSession.ws,
        holder: controlSession.ws ? formatControlHolder(controlSession) : null,
        holderHostname: controlSession.hostname || null,
        holderIp: controlSession.clientIp || null,
        holderLabel: controlSession.label || null
    };

    if (extra) {
        Object.assign(payload, extra);
    }

    return payload;
}

const RESCAN_MIN_INTERVAL_MS = 10000;
let lastRescanTime = 0;

// Commands that do not touch the servo bus — handle immediately, not via the bus queue.
const INSTANT_SERVER_COMMANDS = {
    getStatus: true,
    getJointConfigs: true,
    getServerDiagnostics: true,
    getControlStatus: true,
    takeControl: true,
    releaseControl: true,
    kinematicsLoadURDF: true,
    kinematicsForwardKinematics: true,
    kinematicsForwardKinematicsSteps: true,
    kinematicsForwardKinematicsBatch: true,
    kinematicsInverseKinematics: true,
    kinematicsRefineOrientationWithAccuracy: true,
    kinematicsGetInfo: true
};

// Commands that write to the servo bus — require control session
const BUS_WRITE_COMMANDS = {
    moveJoint: true,
    stopJoint: true,
    stopAll: true,
    stopAllJoints: true,
    setServo: true,
    setServoAngle: true,
    setSpeed: true,
    setSpeedAll: true,
    setTorqueAll: true,
    setAcceleration: true,
    rescanServos: true,
    toolPing: true,
    toolSetPwm: true,
    toolSetServoEnabled: true,
    toolSetServoPosition: true,
    toolSetServoAngle: true,
    toolSetWatchdog: true,
    toolClearFaults: true,
    toolReset: true
};

// Joint moves run immediately on the bus (not queued behind status polls).
const IMMEDIATE_BUS_COMMANDS = {
    moveJoint: true,
    stopJoint: true,
    stopAll: true,
    stopAllJoints: true
};

/**
 * Shared serial port instance (all servos use the same port)
 */
let sharedSerialPort = null;

/**
 * Array to store all servo controllers for data routing
 * This is used by the shared serial port data handler
 */
let allServoControllers = [];

/**
 * Write queue for serializing writes to the shared serial port
 * This ensures only one write happens at a time
 */
let writeQueue = [];
let isWriting = false;

function refreshDataRoutingControllers() {
    allServoControllers = servos.filter(s => s !== null);
    if (endTool) {
        allServoControllers.push(endTool);
    }
}

/**
 * Send incoming serial bytes to every active controller.
 * Uses the servos[] array (not only allServoControllers) so startup pings work.
 */
function routeIncomingSerialData(data) {
    for (let i = 0; i < servos.length; i++) {
        const servo = servos[i];
        if (servo && typeof servo.handleIncomingData === 'function') {
            servo.handleIncomingData(data);
        }
    }
    if (endTool && typeof endTool.handleIncomingData === 'function') {
        endTool.handleIncomingData(data);
    }
}

function subnetMaskToPrefix(mask) {
    if (typeof mask !== 'string') return null;
    const parts = mask.trim().split('.');
    if (parts.length !== 4) return null;
    let prefix = 0;
    for (let i = 0; i < 4; i++) {
        const num = parseInt(parts[i], 10);
        if (isNaN(num) || num < 0 || num > 255) return null;
        let bits = num.toString(2);
        while (bits.length < 8) bits = '0' + bits;
        for (let b = 0; b < bits.length; b++) {
            if (bits[b] === '1') {
                prefix++;
            }
        }
    }
    return prefix;
}

function prefixToSubnetMask(prefix) {
    const p = parseInt(prefix, 10);
    if (isNaN(p) || p < 0 || p > 32) return null;
    let bits = '';
    for (let i = 0; i < 32; i++) {
        bits += i < p ? '1' : '0';
    }
    const parts = [];
    for (let i = 0; i < 4; i++) {
        const chunk = bits.slice(i * 8, i * 8 + 8);
        parts.push(parseInt(chunk, 2));
    }
    return parts.join('.');
}

async function runNmcli(args) {
    const result = await execFileAsync('nmcli', args, { timeout: 5000 });
    return (result.stdout || '').trim();
}

async function getEthernetSettings() {
    const runningText = await runNmcli(['-t', '-f', 'RUNNING', 'general', 'status']);
    if (runningText.toLowerCase() !== 'running') {
        throw new Error('NetworkManager is not running');
    }

    const statusText = await runNmcli(['-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status']);
    const lines = statusText.split('\n').filter(line => line.trim().length > 0);

    let selected = null;
    for (let i = 0; i < lines.length; i++) {
        const parts = lines[i].split(':');
        if (parts.length < 4) continue;
        const device = parts[0];
        const type = parts[1];
        const state = parts[2];
        const connectionName = parts.slice(3).join(':');
        if (type === 'ethernet') {
            selected = { device, type, state, connectionName };
            if (device === 'eth0') break;
        }
    }

    if (!selected || !selected.connectionName || selected.connectionName === '--') {
        throw new Error('No ethernet NetworkManager connection found');
    }

    const conn = selected.connectionName;
    const connDataText = await runNmcli(['-g', 'ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.dns', 'connection', 'show', conn]);
    const connDataLines = connDataText.split('\n');
    const ipv4Method = (connDataLines[0] || '').trim();
    const ipv4Addresses = (connDataLines[1] || '').trim();
    const ipv4Gateway = (connDataLines[2] || '').trim();
    const ipv4Dns = (connDataLines[3] || '').trim();

    let ipAddress = '';
    let prefix = '';
    let subnetMask = '';
    if (ipv4Addresses) {
        const first = ipv4Addresses.split(',')[0].trim();
        const addressParts = first.split('/');
        ipAddress = (addressParts[0] || '').trim();
        prefix = (addressParts[1] || '').trim();
        const mask = prefixToSubnetMask(prefix);
        subnetMask = mask || '';
    }

    return {
        connectionName: conn,
        device: selected.device,
        state: selected.state,
        method: ipv4Method || 'auto',
        ipAddress: ipAddress,
        subnetMask: subnetMask,
        prefix: prefix,
        gateway: ipv4Gateway,
        dns: ipv4Dns
    };
}

async function setEthernetSettings(input) {
    const current = await getEthernetSettings();
    const conn = current.connectionName;

    const mode = (input && typeof input.mode === 'string') ? input.mode.trim().toLowerCase() : '';
    if (mode !== 'dhcp' && mode !== 'static') {
        throw new Error('Mode must be "dhcp" or "static"');
    }

    if (mode === 'dhcp') {
        await runNmcli(['connection', 'modify', conn, 'ipv4.method', 'auto', 'ipv4.addresses', '', 'ipv4.gateway', '', 'ipv4.dns', '']);
        await runNmcli(['connection', 'up', conn]);
        return await getEthernetSettings();
    }

    const ipAddress = (input && typeof input.ipAddress === 'string') ? input.ipAddress.trim() : '';
    const subnetMask = (input && typeof input.subnetMask === 'string') ? input.subnetMask.trim() : '';
    const gateway = (input && typeof input.gateway === 'string') ? input.gateway.trim() : '';
    const dns = (input && typeof input.dns === 'string') ? input.dns.trim() : '';

    if (!ipAddress) throw new Error('IP address is required for static mode');
    if (!subnetMask) throw new Error('Subnet mask is required for static mode');
    if (!gateway) throw new Error('Gateway is required for static mode');

    const prefix = subnetMaskToPrefix(subnetMask);
    if (prefix === null) {
        throw new Error('Invalid subnet mask');
    }
    const addressWithPrefix = `${ipAddress}/${prefix}`;

    const args = [
        'connection', 'modify', conn,
        'ipv4.method', 'manual',
        'ipv4.addresses', addressWithPrefix,
        'ipv4.gateway', gateway
    ];
    if (dns) {
        args.push('ipv4.dns', dns);
    } else {
        args.push('ipv4.dns', '');
    }
    await runNmcli(args);
    await runNmcli(['connection', 'up', conn]);
    return await getEthernetSettings();
}

/**
 * Queue a write operation to the shared serial port
 * @param {Function} writeFn - Function that performs the write
 * @returns {Promise} Promise that resolves when write completes
 */
async function queueWrite(writeFn) {
    return new Promise((resolve, reject) => {
        writeQueue.push({ writeFn, resolve, reject });
        processWriteQueue();
    });
}

/**
 * Process the write queue (one write at a time)
 */
async function processWriteQueue() {
    if (isWriting || writeQueue.length === 0) {
        return;
    }
    
    isWriting = true;
    const { writeFn, resolve, reject } = writeQueue.shift();
    
    try {
        await writeFn();
        resolve();
    } catch (error) {
        reject(error);
    } finally {
        isWriting = false;
        // Process next item in queue
        processWriteQueue();
    }
}

/**
 * Queue a servo-bus write to run on the next bus tick (WebSocket handlers do not await the bus).
 * @param {Function} commandFn - Function that executes the command
 * @param {Object} meta - { command, joint }
 * @returns {Promise}
 */
function enqueueBusWrite(commandFn, meta) {
    return new Promise((resolve, reject) => {
        if (serverShuttingDown) {
            reject(new Error('Server is shutting down'));
            return;
        }

        if (busWriteQueue.length >= MAX_BUS_WRITE_QUEUE_SIZE) {
            serverDiagnostics.busWritesRejected++;
            console.warn('Bus write queue full (' + busWriteQueue.length + ' items), rejecting command');
            reject(new Error('Bus write queue is full, server may be overloaded'));
            return;
        }

        // Drop older queued moveJoint for the same joint (pendant spam / multi-app).
        if (meta && meta.command === 'moveJoint' && meta.joint !== undefined) {
            for (let i = busWriteQueue.length - 1; i >= 0; i--) {
                const item = busWriteQueue[i];
                if (item.meta && item.meta.command === 'moveJoint' && item.meta.joint === meta.joint) {
                    item.reject(new Error('Superseded by newer move for joint ' + meta.joint));
                    busWriteQueue.splice(i, 1);
                }
            }
        }

        const item = { commandFn, resolve, reject, meta: meta || null, enqueuedAt: Date.now() };

        if (meta && PRIORITY_BUS_COMMANDS[meta.command]) {
            busWriteQueue.unshift(item);
        } else {
            busWriteQueue.push(item);
        }

        serverDiagnostics.busWriteQueueLength = busWriteQueue.length;
        kickBusTickIfIdle();
    });
}

/**
 * Run a bus tick immediately when the bus is idle (so moves are not delayed).
 */
function kickBusTickIfIdle() {
    if (!busTickLoopActive || serverShuttingDown || busTickInProgress) {
        return;
    }
    runBusTick();
}

/**
 * Run a move/stop command on the bus right away (not behind the status poll queue).
 * @param {WebSocket} ws
 * @param {Object} data
 */
function clearAllServoPendingTransactions() {
    for (let i = 0; i < servos.length; i++) {
        const servo = servos[i];
        if (servo && typeof servo.clearPendingBusTransaction === 'function') {
            servo.clearPendingBusTransaction();
        }
    }
    if (endTool && typeof endTool.clearPendingBusTransaction === 'function') {
        endTool.clearPendingBusTransaction();
    }
}

function scheduleImmediateBusCommand(ws, data) {
    if (data.command === 'moveJoint' && data.joint !== undefined) {
        const jointKey = String(data.joint);
        const prev = pendingImmediateMoves[jointKey];
        if (prev && prev.data.requestId !== undefined) {
            try {
                prev.ws.send(JSON.stringify({
                    type: 'error',
                    message: 'Superseded by newer move for joint ' + data.joint,
                    requestId: prev.data.requestId
                }));
            } catch (sendError) {
                // Client may have disconnected
            }
        }
        pendingImmediateMoves[jointKey] = { ws: ws, data: data };
        drainImmediateMoveQueue();
        return;
    }

    runImmediateBusCommand(ws, data, false);
}

async function drainImmediateMoveQueue() {
    if (immediateMoveDrainRunning) {
        return;
    }

    immediateMoveDrainRunning = true;

    try {
        while (Object.keys(pendingImmediateMoves).length > 0) {
            const jointKey = Object.keys(pendingImmediateMoves)[0];
            const item = pendingImmediateMoves[jointKey];
            delete pendingImmediateMoves[jointKey];
            const moreAfterThis = Object.keys(pendingImmediateMoves).length > 0;
            await runImmediateBusCommand(item.ws, item.data, moreAfterThis);
        }
    } finally {
        immediateMoveDrainRunning = false;
    }
}

async function runImmediateBusCommand(ws, data, moreMovesQueued) {
    await waitForBusTickToFinish();

    if (busTickTimer) {
        clearTimeout(busTickTimer);
        busTickTimer = null;
    }

    const isRapidJog = moreMovesQueued === true;
    const quietBefore = isRapidJog ? BUS_QUIET_JOG_MS : BUS_QUIET_BEFORE_MOVE_MS;
    const quietAfter = isRapidJog ? BUS_QUIET_JOG_MS : BUS_QUIET_AFTER_MOVE_MS;

    const resumeLoop = busTickLoopActive;
    busTickLoopActive = false;
    busTickInProgress = true;
    const cmdStart = Date.now();

    try {
        clearAllServoPendingTransactions();
        await new Promise((resolve) => setTimeout(resolve, quietBefore));

        await handleCommand(ws, data);
        serverDiagnostics.immediateBusCommands++;

        await new Promise((resolve) => setTimeout(resolve, quietAfter));

        debugLog('Immediate bus command OK: ' + data.command +
            (data.joint !== undefined ? ' joint=' + data.joint : '') +
            ' (' + (Date.now() - cmdStart) + ' ms)');
    } catch (error) {
        const errMsg = error && error.message ? error.message : String(error);
        if (errMsg.indexOf('timeout') >= 0 || errMsg.indexOf('Timeout') >= 0) {
            serverDiagnostics.writeTimeouts++;
        }
        debugLog('Immediate bus command failed: ' + data.command + ' — ' + errMsg, true);
        const errPayload = {
            type: 'error',
            message: formatCommandError(error)
        };
        if (data.requestId !== undefined) {
            errPayload.requestId = data.requestId;
        }
        ws.send(JSON.stringify(errPayload));
    } finally {
        busTickInProgress = false;
        busTickLoopActive = resumeLoop;
        if (resumeLoop && !serverShuttingDown) {
            scheduleNextBusTick(MIN_BUS_TICK_GAP_MS + BUS_QUIET_AFTER_MOVE_MS);
        }
    }
}

/**
 * Snapshot of bus / WebSocket diagnostics for debugging.
 * @returns {Object}
 */
function getServerDiagnosticsSnapshot() {
    return {
        ...serverDiagnostics,
        cacheAgeMs: getJointStatusCacheAgeMs(),
        wsClients: connectedClients.size,
        busWriteQueueLength: busWriteQueue.length,
        lastBroadcastAt: lastStatusBroadcastAt || serverDiagnostics.lastBroadcastAt,
        uptimeMs: Date.now() - serverDiagnostics.startedAt
    };
}

/**
 * Rebuild cached joint config list after init/rescan.
 */
function rebuildJointConfigsCache() {
    const discoveredServos = servos.filter(s => s !== null).length;
    const jointConfigs = [];

    for (let i = 0; i < servos.length; i++) {
        if (servos[i] !== null) {
            jointConfigs.push({
                jointNumber: i + 1,
                servoId: servos[i].servoIdNumber,
                available: true
            });
        }
    }

    cachedJointConfigs = {
        count: discoveredServos,
        total: servos.length,
        joints: jointConfigs
    };
}

/**
 * @returns {Object}
 */
function getJointConfigsSnapshot() {
    if (!cachedJointConfigs) {
        rebuildJointConfigsCache();
    }
    return {
        count: cachedJointConfigs.count,
        total: cachedJointConfigs.total,
        joints: cachedJointConfigs.joints.map(j => ({ ...j }))
    };
}

/**
 * Push status snapshot to every connected WebSocket client.
 */
function broadcastStatusToClients() {
    if (connectedClients.size === 0) {
        return;
    }

    const payload = JSON.stringify({
        type: 'status',
        joints: getJointStatusSnapshot(),
        cacheAgeMs: getJointStatusCacheAgeMs(),
        pushed: true
    });

    connectedClients.forEach((ws) => {
        if (ws.readyState === WebSocket.OPEN) {
            ws.send(payload);
        }
    });

    lastStatusBroadcastAt = Date.now();
    serverDiagnostics.lastBroadcastAt = lastStatusBroadcastAt;
}

/**
 * @param {WebSocket} ws
 * @returns {{ok: boolean, message?: string}}
 */
function pruneStaleControlSession() {
    if (!controlSession.ws) {
        return;
    }
    if (controlSession.ws.readyState !== WebSocket.OPEN) {
        debugLog('Releasing arm control from disconnected client');
        releaseControlSession(controlSession.ws);
    }
}

function requireControl(ws) {
    pruneStaleControlSession();

    // No holder yet — grant this client automatically (keeps old apps working)
    if (!controlSession.ws) {
        assignControlSession(ws, 'auto');
        return { ok: true };
    }
    if (controlSession.ws !== ws) {
        const who = formatControlHolder(controlSession) || 'another client';
        return {
            ok: false,
            message: 'Arm control is held by ' + who + '. Only one app can move the arm at a time.'
        };
    }
    return { ok: true };
}

/**
 * @param {WebSocket} ws
 * @param {string} command
 * @returns {{ok: boolean, message?: string}}
 */
function requireControlForCommand(ws, command) {
    if (!BUS_WRITE_COMMANDS[command]) {
        return { ok: true };
    }
    return requireControl(ws);
}

function releaseControlSession(ws) {
    if (controlSession.ws === ws) {
        controlSession.ws = null;
        controlSession.label = '';
        controlSession.hostname = null;
        controlSession.clientIp = null;
        controlSession.since = null;
        controlSession.lastMoveAt = null;
    }
}

/**
 * Apply speed only if it changed (reduces bus traffic).
 */
async function applyJointSpeedIfChanged(servo, jointIndex, speed) {
    if (!jointSettingsCache[jointIndex]) {
        jointSettingsCache[jointIndex] = {};
    }
    if (jointSettingsCache[jointIndex].speed === speed) {
        return false;
    }
    await servo.setSpeed(speed);
    jointSettingsCache[jointIndex].speed = speed;
    return true;
}

/**
 * Apply acceleration only if it changed.
 */
async function applyJointAccelerationIfChanged(servo, jointIndex, acc) {
    if (!jointSettingsCache[jointIndex]) {
        jointSettingsCache[jointIndex] = {};
    }
    if (jointSettingsCache[jointIndex].acceleration === acc) {
        return false;
    }
    await servo.setAcceleration(acc);
    jointSettingsCache[jointIndex].acceleration = acc;
    return true;
}

/**
 * One bus tick: drain pending writes first, then read one joint (round-robin) when idle.
 * Chained scheduling avoids overlapping ticks that starve moves.
 */
async function runBusTick() {
    if (serverShuttingDown || !busTickLoopActive) {
        return;
    }

    if (busTickInProgress) {
        serverDiagnostics.busTicksSkipped++;
        return;
    }

    busTickInProgress = true;
    const tickStart = Date.now();
    serverDiagnostics.busTicks++;

    try {
        const queueAtStart = busWriteQueue.length;
        const maxWrites = queueAtStart > 0 ? MAX_BUS_WRITES_WHEN_BUSY : MAX_BUS_WRITES_WHEN_IDLE;

        for (let w = 0; w < maxWrites && busWriteQueue.length > 0 && !serverShuttingDown; w++) {
            const { commandFn, resolve, reject } = busWriteQueue.shift();
            serverDiagnostics.busWriteQueueLength = busWriteQueue.length;
            const writeStart = Date.now();

            try {
                await commandFn();
                await new Promise((r) => setTimeout(r, 15));
                resolve();
                serverDiagnostics.busWritesCompleted++;
                serverDiagnostics.lastBusWriteDurationMs = Date.now() - writeStart;
                serverDiagnostics.lastBusWriteAt = Date.now();
            } catch (error) {
                reject(error);
                serverDiagnostics.busWritesFailed++;
            }
        }

        // Always poll servos each tick (even after bus writes) so status cache stays fresh.
        const pollStart = Date.now();
        const jointsToPoll = Math.min(JOINTS_PER_POLL_TICK, JOINT_COUNT);
        for (let p = 0; p < jointsToPoll; p++) {
            await refreshSingleJointStatusFromBus(statusPollJointIndex);
            statusPollJointIndex = (statusPollJointIndex + 1) % JOINT_COUNT;
            if (p < jointsToPoll - 1) {
                await new Promise((resolve) => setTimeout(resolve, 1));
            }
        }
        serverDiagnostics.statusPollJointIndex = statusPollJointIndex;
        await new Promise((resolve) => setTimeout(resolve, 1));
        broadcastStatusToClients();
        serverDiagnostics.statusPollsCompleted++;
        serverDiagnostics.lastStatusPollDurationMs = Date.now() - pollStart;
        serverDiagnostics.lastStatusPollAt = Date.now();
    } catch (error) {
        serverDiagnostics.busTickErrors++;
        debugLog('Bus tick error: ' + (error.message || error), true);
    } finally {
        serverDiagnostics.lastBusTickDurationMs = Date.now() - tickStart;
        serverDiagnostics.lastBusTickAt = Date.now();
        serverDiagnostics.cacheAgeMs = getJointStatusCacheAgeMs();
        serverDiagnostics.wsClients = connectedClients.size;
        serverDiagnostics.busWriteQueueLength = busWriteQueue.length;
        busTickInProgress = false;

        if (busTickLoopActive && !serverShuttingDown) {
            const elapsed = Date.now() - tickStart;
            const delay = Math.max(MIN_BUS_TICK_GAP_MS, STATUS_POLL_INTERVAL_MS - elapsed);
            scheduleNextBusTick(delay);
        }
    }
}

/**
 * Schedule the next bus tick after a delay (chained loop — no overlapping ticks).
 * @param {number} delayMs
 */
function scheduleNextBusTick(delayMs) {
    if (!busTickLoopActive || serverShuttingDown) {
        return;
    }

    if (busTickTimer) {
        clearTimeout(busTickTimer);
    }

    busTickTimer = setTimeout(() => {
        busTickTimer = null;
        runBusTick();
    }, delayMs);
}

/**
 * Initialize all servo controllers
 */
async function initializeServos() {
    console.log('Initializing ST3215 servo controllers...');

    if (SERVO_IDS.length < JOINT_COUNT) {
        console.error(`Error: Need ${JOINT_COUNT} servo IDs but only ${SERVO_IDS.length} provided`);
        console.error('Please configure more servo IDs in the SERVO_IDS array');
        process.exit(1);
    }
    
    // Create a single shared serial port for all servos (they're daisy-chained)
    const { SerialPort } = require('serialport');
    
    try {
        console.log(`Using serial port: ${SERIAL_PORT} @ ${SERIAL_BAUDRATE} baud`);
        console.log('Opening shared serial port...');
        sharedSerialPort = new SerialPort({
            path: SERIAL_PORT,
            baudRate: SERIAL_BAUDRATE,
            dataBits: 8,
            parity: 'none',
            stopBits: 1,
            autoOpen: false
        });
        
        // Open the shared port
        await new Promise((resolve, reject) => {
            sharedSerialPort.open((error) => {
                if (error) {
                    console.error('Failed to open shared serial port:', error.message);
                    reject(error);
                } else {
                    console.log('✓ Shared serial port opened successfully');
                    resolve();
                }
            });
        });
        
        // One handler for the shared bus; each controller filters by servo ID
        sharedSerialPort.on('data', routeIncomingSerialData);
        
        sharedSerialPort.on('error', (error) => {
            console.error('Shared serial port error:', error.message);
        });
        
        // Attach write queue function to shared port for servo controllers to use
        sharedSerialPort._writeQueue = queueWrite;
        
    } catch (error) {
        console.error('Failed to initialize shared serial port:', error.message);
        process.exit(1);
    }
    
    // Create servo controllers, all sharing the same serial port (fixed indices 0..JOINT_COUNT-1)
    for (let i = 0; i < JOINT_COUNT; i++) {
        servos[i] = null;

        if (i > 0) {
            await new Promise(resolve => setTimeout(resolve, 150));
        }

        const servo = new RobotArm.ServoController(i + 1, sharedSerialPort, SERVO_IDS[i], SERIAL_BAUDRATE);

        try {
            await servo.open();
            servos[i] = servo;

            console.log(`Pinging servo ${i + 1} (ST3215 ID: ${SERVO_IDS[i]})...`);
            const pingResult = await servo.ping();
            if (!pingResult) {
                console.log(`⚠️  Servo ${i + 1} (ID: ${SERVO_IDS[i]}) did not respond to ping - skipping`);
                servos[i] = null;
                continue;
            }
            console.log(`✓ Servo ${i + 1} (ID: ${SERVO_IDS[i]}) responded to ping`);

            await new Promise(resolve => setTimeout(resolve, 50));
            await servo.startServo();

            console.log(`Servo ${i + 1} initialized (ST3215 ID: ${SERVO_IDS[i]})`);
        } catch (error) {
            console.error(`Failed to initialize servo ${i + 1} (ST3215 ID: ${SERVO_IDS[i]}):`, error.message);
            servos[i] = null;
        }
    }
    
    console.log(`Initialized ${servos.filter(s => s !== null).length} of ${JOINT_COUNT} servos`);

    // Initialize optional ESP32 end-tool node (ID 64)
    try {
        const tool = new RobotArm.EndToolController(sharedSerialPort, SERIAL_BAUDRATE);
        await tool.open();
        endTool = tool;

        const toolPing = await endTool.pingTool();
        if (toolPing) {
            console.log('✓ End tool node (ID 64) responded to ping');
        } else {
            console.log('⚠️  End tool node (ID 64) did not respond at startup (commands will still be available)');
        }
    } catch (error) {
        console.error('Failed to initialize end tool controller:', error.message);
        endTool = null;
    }

    refreshDataRoutingControllers();
    rebuildJointConfigsCache();
    cachedTorqueEnabled = true;
}

/**
 * Try to bring a joint online if it was missing at startup (ping retry).
 * @param {number} jointIndex - 0-based
 * @param {number} jointNum - 1-based for messages
 * @returns {Promise<Object|null>}
 */
async function resolveServoForJoint(jointIndex, jointNum) {
    if (servos[jointIndex] !== null && servos[jointIndex] !== undefined) {
        return servos[jointIndex];
    }

    try {
        clearAllServoPendingTransactions();
        await new Promise((resolve) => setTimeout(resolve, 20));
        const servo = await createAndInitializeServo(jointIndex);
        refreshDataRoutingControllers();
        rebuildJointConfigsCache();
        debugLog('Recovered servo for joint ' + jointNum + ' (ID ' + SERVO_IDS[jointIndex] + ')');
        return servo;
    } catch (error) {
        debugLog('Could not recover joint ' + jointNum + ': ' + (error.message || error), true);
        return null;
    }
}

async function createAndInitializeServo(jointIndex) {
    const servoId = SERVO_IDS[jointIndex];
    const servo = new RobotArm.ServoController(jointIndex + 1, sharedSerialPort, servoId, SERIAL_BAUDRATE);
    await servo.open();
    servos[jointIndex] = servo;
    const alive = await servo.ping();
    if (!alive) {
        servos[jointIndex] = null;
        throw new Error(`Servo ${jointIndex + 1} (ID: ${servoId}) did not respond to ping`);
    }
    await servo.startServo();
    return servo;
}

async function rescanServos() {
    const results = [];

    for (let i = 0; i < JOINT_COUNT; i++) {
        const jointNumber = i + 1;
        const servoId = SERVO_IDS[i];
        const existing = servos[i] || null;

        if (existing !== null) {
            try {
                const alive = await existing.isResponsive();
                if (alive) {
                    await existing.startServo();
                    results.push({
                        joint: jointNumber,
                        servoId: servoId,
                        available: true,
                        action: 'kept_existing'
                    });
                    continue;
                }
            } catch (error) {
                // Fall through to re-create path
            }
        }

        try {
            const replacement = await createAndInitializeServo(i);
            servos[i] = replacement;
            results.push({
                joint: jointNumber,
                servoId: servoId,
                available: true,
                action: existing ? 'recreated' : 'created'
            });
        } catch (error) {
            servos[i] = null;
            results.push({
                joint: jointNumber,
                servoId: servoId,
                available: false,
                action: existing ? 'lost' : 'missing',
                error: error.message
            });
        }
    }

    refreshDataRoutingControllers();
    return results;
}

/**
 * Default status object for one joint (before first successful bus read).
 * @param {number} jointNum
 * @returns {Object}
 */
function defaultJointStatus(jointNum) {
    return {
        joint: jointNum,
        available: false,
        isMoving: false,
        angleDegrees: 0,
        position: 0,
        stepPosition: 0,
        speed: 0,
        load: 0,
        voltage: 0,
        temperature: 0,
        torqueEnabled: false,
        readStale: false,
        lastGoodAt: null
    };
}

/**
 * Read quick status with one retry (shared bus can miss a reply under load).
 * @param {Object} servo
 * @returns {Promise<Object>}
 */
async function readServoQuickStatusWithRetry(servo) {
    try {
        return await servo.readQuickStatus();
    } catch (firstError) {
        await new Promise(resolve => setTimeout(resolve, 5));
        return await servo.readQuickStatus();
    }
}

/**
 * Poll one servo and update jointStatusCache for that joint.
 * @param {number} jointIndex - 0-based joint index
 */
async function refreshSingleJointStatusFromBus(jointIndex) {
    const jointNum = jointIndex + 1;
    const servo = servos[jointIndex];
    const previous = jointStatusCache[jointIndex] || defaultJointStatus(jointNum);

    if (servo === null) {
        jointStatusCache[jointIndex] = defaultJointStatus(jointNum);
        return;
    }

    // Skip servos that are in backoff (repeated failures) to keep tick time short.
    // They will be retried once the backoff window expires.
    const nowMs = Date.now();
    if (servoBackoffUntilMs[jointIndex] > nowMs) {
        return;
    }

    const startServo = Date.now();
    try {
        const status = await readServoQuickStatusWithRetry(servo);
        servoConsecFails[jointIndex] = 0;
        jointStatusCache[jointIndex] = {
            joint: jointNum,
            available: true,
            ...status,
            stepPosition: status.position,
            readStale: false,
            lastGoodAt: Date.now()
        };
    } catch (error) {
        servoConsecFails[jointIndex]++;
        if (servoConsecFails[jointIndex] >= SERVO_BACKOFF_FAIL_THRESHOLD) {
            servoBackoffUntilMs[jointIndex] = Date.now() + SERVO_BACKOFF_DURATION_MS;
            servoConsecFails[jointIndex] = 0;
            console.warn(`Joint ${jointNum}: entering ${SERVO_BACKOFF_DURATION_MS}ms backoff after repeated read failures`);
        }
        if (previous.lastGoodAt) {
            jointStatusCache[jointIndex] = {
                ...previous,
                available: true,
                readStale: true,
                pollError: error.message
            };
        } else {
            jointStatusCache[jointIndex] = {
                ...defaultJointStatus(jointNum),
                readStale: true,
                pollError: error.message
            };
        }
        console.warn(`Status poll joint ${jointNum}: ${error.message}`);
    }

    const servoDuration = Date.now() - startServo;
    if (PERF_DEBUG && servoDuration > 50) {
        console.log(`PERF: status poll joint ${jointNum} took ${servoDuration} ms`);
    }
}

/**
 * Poll all servos on the bus and update jointStatusCache.
 * On timeout/error, keeps the last good reading instead of resetting to 0 / torque off.
 */
async function refreshJointStatusCacheFromBus() {
    const startAll = Date.now();

    for (let i = 0; i < servos.length; i++) {
        await refreshSingleJointStatusFromBus(i);
        await new Promise(resolve => setTimeout(resolve, 5));
    }

    if (PERF_DEBUG) {
        console.log(`PERF: refreshJointStatusCacheFromBus took ${Date.now() - startAll} ms`);
    }
}

/**
 * Copy of cached joint status for WebSocket clients (getStatus).
 * @returns {Array}
 */
function getJointStatusSnapshot() {
    const snapshot = [];
    for (let i = 0; i < JOINT_COUNT; i++) {
        const jointNum = i + 1;
        if (jointStatusCache[i]) {
            snapshot.push({ ...jointStatusCache[i] });
        } else {
            snapshot.push(defaultJointStatus(jointNum));
        }
    }
    return snapshot;
}

/**
 * Start the fixed-interval bus tick loop (reads + writes).
 */
function startBusTickLoop() {
    if (busTickLoopActive) {
        return;
    }

    busTickLoopActive = true;
    debugLog('Starting bus tick loop (gap ' + MIN_BUS_TICK_GAP_MS + ' ms, target period ' + STATUS_POLL_INTERVAL_MS + ' ms)');

    runBusTick();

    if (BUS_DIAGNOSTICS_LOG_INTERVAL_MS > 0) {
        setInterval(() => {
            const d = getServerDiagnosticsSnapshot();
            debugLog(
                'BUS diag: ticks=' + d.busTicks +
                ' skipped=' + d.busTicksSkipped +
                ' writeQ=' + d.busWriteQueueLength +
                ' cacheAgeMs=' + d.cacheAgeMs +
                ' lastTickMs=' + d.lastBusTickDurationMs +
                ' lastPollMs=' + d.lastStatusPollDurationMs +
                ' ws=' + d.wsClients +
                ' instant=' + d.instantCommands
            );
        }, BUS_DIAGNOSTICS_LOG_INTERVAL_MS);
    }
}

/**
 * Stop bus tick loop (shutdown).
 */
function stopBusTickLoop() {
    busTickLoopActive = false;
    if (busTickTimer) {
        clearTimeout(busTickTimer);
        busTickTimer = null;
    }
}

/**
 * Age in ms of the oldest lastGoodAt in the cache (for debugging).
 * @returns {number|null}
 */
function getJointStatusCacheAgeMs() {
    let oldest = null;
    for (let i = 0; i < jointStatusCache.length; i++) {
        const entry = jointStatusCache[i];
        if (entry && entry.lastGoodAt) {
            const age = Date.now() - entry.lastGoodAt;
            if (oldest === null || age > oldest) {
                oldest = age;
            }
        }
    }
    return oldest;
}

/**
 * Start the WebSocket server
 */
function startServer() {
    console.log(`Starting WebSocket server on port ${PORT}...`);
    
    // Create WebSocket server
    wss = new WebSocket.Server({ port: PORT });
    
    debugLog('Server listening on port ' + PORT);
    debugLog('Waiting for clients to connect...');
    
    // Handle new client connections
    wss.on('connection', function connection(ws, req) {
        const clientIp = normalizeClientIp(req.socket.remoteAddress);
        ws.clientIp = clientIp;
        ws.clientHostname = null;
        console.log(`Client connected from ${clientIp || 'unknown'}`);

        connectedClients.add(ws);
        
        // Welcome + cached snapshots (no bus read per connect)
        ws.send(JSON.stringify({
            type: 'connected',
            message: 'Connected to Robot Arm Server (ST3215)',
            pushesStatus: true,
            statusIntervalMs: STATUS_POLL_INTERVAL_MS,
            busTickIntervalMs: STATUS_POLL_INTERVAL_MS,
            serverBuildId: SERVER_BUILD_ID
        }));

        ws.send(JSON.stringify({
            type: 'status',
            joints: getJointStatusSnapshot(),
            cacheAgeMs: getJointStatusCacheAgeMs(),
            pushed: true
        }));

        const configsOnConnect = getJointConfigsSnapshot();
        ws.send(JSON.stringify({
            type: 'jointConfigs',
            count: configsOnConnect.count,
            total: configsOnConnect.total,
            joints: configsOnConnect.joints
        }));
        
        // Handle incoming messages from client
        ws.on('message', async function incoming(message) {
            try {
                const data = JSON.parse(message);
                // Cache reads, control status, and kinematics do not use the servo bus.
                if (INSTANT_SERVER_COMMANDS[data.command]) {
                    serverDiagnostics.instantCommands++;
                    await handleCommand(ws, data);
                    return;
                }
                if (IMMEDIATE_BUS_COMMANDS[data.command]) {
                    scheduleImmediateBusCommand(ws, data);
                    return;
                }
                // Other bus writes are scheduled on the tick loop — do not await here.
                enqueueBusWrite(async () => {
                    await handleCommand(ws, data);
                }, {
                    command: data.command,
                    joint: data.joint
                }).catch((error) => {
                    const msg = error && error.message ? error.message : String(error);
                    if (msg.indexOf('Superseded by newer move') >= 0) {
                        return;
                    }
                    debugLog('Bus command failed (' + data.command + '): ' + msg, true);
                    const errPayload = {
                        type: 'error',
                        message: formatCommandError(error)
                    };
                    if (data.requestId !== undefined) {
                        errPayload.requestId = data.requestId;
                    }
                    ws.send(JSON.stringify(errPayload));
                });
            } catch (error) {
                console.error('Error handling message:', error);
                ws.send(JSON.stringify({
                    type: 'error',
                    message: error.message
                }));
            }
        });
        
        // Handle client disconnect
        ws.on('close', function() {
            connectedClients.delete(ws);
            releaseControlSession(ws);
            console.log(`Client disconnected from ${clientIp}`);
        });
        
        // Handle errors
        ws.on('error', function(error) {
            console.error('WebSocket error:', error);
        });
    });

    setInterval(releaseControlForIdle, CONTROL_IDLE_CHECK_INTERVAL_MS);
}

/**
 * Handle commands from the Electron app
 */
async function handleCommand(ws, data) {
    const command = data.command;
    const requestId = data.requestId;
    const sendResponse = (payload) => {
        if (requestId !== undefined) {
            payload.requestId = requestId;
        }
        ws.send(JSON.stringify(payload));
    };

    if (command !== 'takeControl' && command !== 'releaseControl' && command !== 'getControlStatus') {
        const controlCheck = requireControlForCommand(ws, command);
        if (!controlCheck.ok) {
            sendResponse({
                type: 'error',
                message: controlCheck.message,
                controlRequired: true
            });
            return;
        }

        if (command === 'moveJoint' || command === 'setServo' || command === 'setServoAngle' || command === 'setAcceleration') {
            touchControlMoveActivity(ws);
        }
    }

    const requireEndTool = () => {
        if (!endTool) {
            throw new Error('End tool controller is not initialized');
        }
        return endTool;
    };
    
    // Handle different commands
    switch (command) {
        case 'getPiNetworkInfo': {
            // Return basic network information for display in the Electron app
            try {
                const hostname = os.hostname();
                const interfaces = os.networkInterfaces() || {};

                // Collect a simple list of IPv4 addresses with MACs
                const ifaceSummaries = [];
                Object.keys(interfaces).forEach((name) => {
                    (interfaces[name] || []).forEach((info) => {
                        if (info && info.family === 'IPv4' && !info.internal) {
                            ifaceSummaries.push({
                                name: name,
                                address: info.address,
                                mac: info.mac || null
                            });
                        }
                    });
                });

                // Try to read default gateway from /proc/net/route (Linux-specific)
                let gateway = null;
                try {
                    const routeText = fs.readFileSync('/proc/net/route', 'utf8');
                    const lines = routeText.trim().split('\n');
                    // Skip header line
                    for (let i = 1; i < lines.length; i++) {
                        const parts = lines[i].trim().split(/\s+/);
                        if (parts.length >= 3) {
                            const dest = parts[1];
                            const gwHex = parts[2];
                            const flags = parseInt(parts[3] || '0', 16);
                            // Destination 00000000 and flag 0x2 means default route
                            if (dest === '00000000' && (flags & 0x2)) {
                                const gwNum = parseInt(gwHex, 16);
                                const b1 = gwNum & 0xFF;
                                const b2 = (gwNum >> 8) & 0xFF;
                                const b3 = (gwNum >> 16) & 0xFF;
                                const b4 = (gwNum >> 24) & 0xFF;
                                gateway = `${b1}.${b2}.${b3}.${b4}`;
                                break;
                            }
                        }
                    }
                } catch (e) {
                    // If we cannot read the route table, just leave gateway as null
                    if (DEBUG) {
                        console.warn('getPiNetworkInfo: could not read /proc/net/route:', e.message || e);
                    }
                }

                sendResponse({
                    type: 'networkInfo',
                    hostname: hostname,
                    interfaces: ifaceSummaries,
                    gateway: gateway
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: 'Failed to read network info: ' + (error.message || error)
                });
            }
            break;
        }

        case 'getPiEthernetSettings': {
            try {
                const ethernet = await getEthernetSettings();
                sendResponse({
                    type: 'ethernetSettings',
                    ethernet: ethernet
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: 'Failed to get ethernet settings: ' + (error.message || error)
                });
            }
            break;
        }

        case 'setPiEthernetSettings': {
            try {
                const ethernet = await setEthernetSettings(data || {});
                sendResponse({
                    type: 'ethernetSettingsUpdated',
                    ethernet: ethernet,
                    message: 'Ethernet settings updated'
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: 'Failed to set ethernet settings: ' + (error.message || error)
                });
            }
            break;
        }

        case 'updatePiServerFromGit': {
            // Run "git pull --ff-only" in this folder (raspberry-pi-control-st3215)
            exec('git pull --ff-only', { cwd: __dirname }, (error, stdout, stderr) => {
                if (error) {
                    console.error('updatePiServerFromGit error:', error);
                    sendResponse({
                        type: 'updateResult',
                        ok: false,
                        target: 'st3215',
                        message: `git pull failed: ${stderr || error.message}`
                    });
                } else {
                    console.log('updatePiServerFromGit output:', stdout);
                    sendResponse({
                        type: 'updateResult',
                        ok: true,
                        target: 'st3215',
                        message: stdout.trim()
                    });

                    // Restart Node.js so the updated code is loaded.
                    // systemd is configured with Restart=always, so exiting here is enough.
                    setTimeout(function () {
                        try {
                            process.exit(0);
                        } catch (e) {
                            // If exit fails for some reason, we just do nothing.
                        }
                    }, 500);
                }
            });
            break;
        }
        case 'getJointConfigs': {
            const snapshot = getJointConfigsSnapshot();
            sendResponse({
                type: 'jointConfigs',
                count: snapshot.count,
                total: snapshot.total,
                joints: snapshot.joints
            });
            break;
        }

        case 'takeControl': {
            const forceTake = data.force === true;
            const previousWs = controlSession.ws;
            const previousHolder = controlSession.ws ? formatControlHolder(controlSession) : null;

            if (typeof data.hostname === 'string' && data.hostname.trim()) {
                ws.clientHostname = data.hostname.trim();
            }

            if (controlSession.ws && controlSession.ws !== ws && !forceTake) {
                sendResponse(getControlStatusPayload(ws));
                return;
            }

            const label = (typeof data.label === 'string' && data.label) ? data.label : 'client';
            assignControlSession(ws, label);

            if (previousWs && previousWs !== ws && forceTake) {
                try {
                    previousWs.send(JSON.stringify(getControlStatusPayload(previousWs, {
                        message: 'Another app took arm control'
                    })));
                } catch (notifyError) {
                    // Previous client may have disconnected
                }
            }

            sendResponse(getControlStatusPayload(ws, {
                takenFrom: (forceTake && previousWs && previousWs !== ws) ? previousHolder : null
            }));
            break;
        }

        case 'releaseControl': {
            if (controlSession.ws === ws) {
                releaseControlSession(ws);
            }
            sendResponse(getControlStatusPayload(ws));
            break;
        }

        case 'getControlStatus': {
            sendResponse(getControlStatusPayload(ws));
            break;
        }

        case 'kinematicsLoadURDF': {
            try {
                const urdfXml = data.urdfXml;
                if (typeof urdfXml !== 'string' || !urdfXml.trim()) {
                    throw new Error('URDF text is empty');
                }
                const info = robotKinematics.loadURDF(urdfXml);
                sendResponse({
                    type: 'kinematicsLoaded',
                    configured: info.configured,
                    jointCount: info.jointCount,
                    joints: info.joints,
                    urdfData: info.urdfData,
                    maxReachMm: info.maxReachMm
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to load URDF: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsForwardKinematics': {
            try {
                const angles = data.jointAngles;
                const result = robotKinematics.forwardKinematics(angles);
                sendResponse({
                    type: 'kinematicsForwardResult',
                    result: result
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Forward kinematics failed: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsForwardKinematicsSteps': {
            try {
                const angles = data.jointAngles;
                const result = robotKinematics.getForwardKinematicsSteps(angles);
                sendResponse({
                    type: 'kinematicsForwardStepsResult',
                    result: result
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Forward kinematics steps failed: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsForwardKinematicsBatch': {
            try {
                const jointAnglesList = data.jointAnglesList;
                if (!Array.isArray(jointAnglesList)) {
                    throw new Error('jointAnglesList must be an array');
                }

                const positions = [];
                for (let i = 0; i < jointAnglesList.length; i++) {
                    const angles = jointAnglesList[i];
                    const fk = robotKinematics.forwardKinematics(angles);
                    positions.push(fk.position);
                }

                sendResponse({
                    type: 'kinematicsForwardBatchResult',
                    positions: positions
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Forward kinematics batch failed: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsInverseKinematics': {
            try {
                const targetPose = data.targetPose;
                const initialAngles = data.initialAngles;
                const result = robotKinematics.inverseKinematics(targetPose, initialAngles);
                sendResponse({
                    type: 'kinematicsInverseResult',
                    result: result
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Inverse kinematics failed: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsRefineOrientationWithAccuracy': {
            try {
                const targetPose = data.targetPose;
                const baseAngles = data.baseAngles;
                const desiredOrientation = data.desiredOrientation;
                const result = robotKinematics.refineOrientationWithAccuracy(targetPose, baseAngles, desiredOrientation);
                sendResponse({
                    type: 'kinematicsRefineOrientationResult',
                    result: result
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Refine orientation failed: ${error.message}`
                });
            }
            break;
        }

        case 'kinematicsGetInfo': {
            try {
                const info = robotKinematics.getKinematicsInfo();
                sendResponse({
                    type: 'kinematicsInfo',
                    configured: info.configured,
                    jointCount: info.jointCount,
                    joints: info.joints,
                    urdfData: info.urdfData,
                    maxReachMm: info.maxReachMm
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to get kinematics info: ${error.message}`
                });
            }
            break;
        }
            
        case 'getStatus': {
            // Return cached values (updated by bus tick) — no bus read here.
            sendResponse({
                type: 'status',
                joints: getJointStatusSnapshot(),
                cacheAgeMs: getJointStatusCacheAgeMs()
            });
            break;
        }

        case 'getServerDiagnostics': {
            sendResponse({
                type: 'serverDiagnostics',
                diagnostics: getServerDiagnosticsSnapshot()
            });
            break;
        }

        case 'rescanServos': {
            const now = Date.now();
            if (now - lastRescanTime < RESCAN_MIN_INTERVAL_MS) {
                sendResponse({
                    type: 'error',
                    message: 'Rescan rate limited — wait ' + Math.ceil((RESCAN_MIN_INTERVAL_MS - (now - lastRescanTime)) / 1000) + ' s'
                });
                return;
            }
            lastRescanTime = now;
            try {
                const scanResults = await rescanServos();
                rebuildJointConfigsCache();
                sendResponse({
                    type: 'servoRescan',
                    joints: scanResults
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to rescan servos: ${error.message}`
                });
            }
            break;
        }

        case 'toolPing': {
            try {
                const tool = requireEndTool();
                const ok = await tool.pingTool();
                sendResponse({
                    type: 'toolPing',
                    ok: ok
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to ping tool: ${error.message}`
                });
            }
            break;
        }

        case 'toolGetIdentity': {
            try {
                const tool = requireEndTool();
                const identity = await tool.getToolIdentity();
                sendResponse({
                    type: 'toolIdentity',
                    ...identity
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool identity: ${error.message}`
                });
            }
            break;
        }

        case 'toolGetStatus': {
            try {
                const tool = requireEndTool();
                const status = await tool.getToolStatus();
                sendResponse({
                    type: 'toolStatus',
                    ...status
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool status: ${error.message}`
                });
            }
            break;
        }

        case 'toolSetPwm': {
            try {
                const tool = requireEndTool();
                const pwm1Duty = Number.isFinite(data.pwm1Duty) ? data.pwm1Duty : 0;
                const pwm2Duty = Number.isFinite(data.pwm2Duty) ? data.pwm2Duty : 0;
                const enable1 = data.enable1 !== false;
                const enable2 = data.enable2 !== false;

                await tool.setPwmOutputs(pwm1Duty, pwm2Duty, enable1, enable2);
                sendResponse({
                    type: 'success',
                    message: 'Tool PWM outputs updated'
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set tool PWM: ${error.message}`
                });
            }
            break;
        }

        case 'toolGetPwmState': {
            try {
                const tool = requireEndTool();
                const pwm = await tool.getPwmState();
                sendResponse({
                    type: 'toolPwmState',
                    ...pwm
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool PWM state: ${error.message}`
                });
            }
            break;
        }

        case 'toolReadCurrents': {
            try {
                const tool = requireEndTool();
                const currents = await tool.readPwmCurrents();
                sendResponse({
                    type: 'toolCurrents',
                    ...currents
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool currents: ${error.message}`
                });
            }
            break;
        }

        case 'toolReadAdc': {
            try {
                const tool = requireEndTool();
                const adc = await tool.readAdcData();
                sendResponse({
                    type: 'toolAdc',
                    ...adc
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool ADC data: ${error.message}`
                });
            }
            break;
        }

        case 'toolSetServoEnabled': {
            try {
                const tool = requireEndTool();
                const enabled = data.enabled !== false;
                await tool.setHobbyServoEnabled(enabled);
                sendResponse({
                    type: 'success',
                    message: `Tool hobby servo ${enabled ? 'enabled' : 'disabled'}`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set tool servo enable: ${error.message}`
                });
            }
            break;
        }

        case 'toolSetServoPosition': {
            try {
                const tool = requireEndTool();
                const position = Number.isFinite(data.position) ? data.position : 0;
                await tool.setHobbyServoPosition(position);
                sendResponse({
                    type: 'success',
                    message: `Tool hobby servo position set to ${position}`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set tool servo position: ${error.message}`
                });
            }
            break;
        }

        case 'toolSetServoAngle': {
            try {
                const tool = requireEndTool();
                const angleValue = Number.isFinite(data.angle) ? data.angle : 0;
                await tool.setHobbyServoAngle(angleValue);
                sendResponse({
                    type: 'success',
                    message: `Tool hobby servo angle set to ${angleValue}`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set tool servo angle: ${error.message}`
                });
            }
            break;
        }

        case 'toolGetServoState': {
            try {
                const tool = requireEndTool();
                const servoState = await tool.getHobbyServoState();
                sendResponse({
                    type: 'toolServoState',
                    ...servoState
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to read tool servo state: ${error.message}`
                });
            }
            break;
        }

        case 'toolSetWatchdog': {
            try {
                const tool = requireEndTool();
                const timeoutMs = Number.isFinite(data.timeoutMs) ? data.timeoutMs : 0;
                await tool.setWatchdogTimeout(timeoutMs);
                sendResponse({
                    type: 'success',
                    message: `Tool watchdog timeout set to ${timeoutMs} ms`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set tool watchdog: ${error.message}`
                });
            }
            break;
        }

        case 'toolClearFaults': {
            try {
                const tool = requireEndTool();
                await tool.clearToolFaults();
                sendResponse({
                    type: 'success',
                    message: 'Tool faults clear command sent'
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to clear tool faults: ${error.message}`
                });
            }
            break;
        }

        case 'toolReset': {
            try {
                const tool = requireEndTool();
                await tool.resetTool();
                sendResponse({
                    type: 'success',
                    message: 'Tool reset command sent'
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to reset tool: ${error.message}`
                });
            }
            break;
        }
            
        case 'moveJoint':
            // Move a servo to a specific angle
            const jointNumber = data.joint - 1; // Convert to 0-based index
            const angle = data.angle;
            // Ensure speed is a valid number (default to 1500 if not provided or invalid)
            const moveSpeed = (typeof data.speed === 'number' && !isNaN(data.speed) && data.speed >= 0) ? data.speed : 1500;
            
            if (jointNumber < 0 || jointNumber >= servos.length) {
                sendResponse({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                });
                return;
            }
            
            const servo = await resolveServoForJoint(jointNumber, data.joint);
            if (!servo) {
                sendResponse({
                    type: 'error',
                    message: `Servo ${data.joint} is not available (no reply from bus servo ID ${SERVO_IDS[jointNumber]})`
                });
                return;
            }
            
            try {
                const cachedStatus = jointStatusCache[jointNumber];
                if (!cachedStatus || cachedStatus.torqueEnabled !== true) {
                    await servo.startServo();
                }

                debugLog('moveJoint: joint ' + data.joint + ' -> ' + angle + ' deg, speed ' + moveSpeed + ' step/s');
                await servo.moveToAngle(angle, moveSpeed);
                serverDiagnostics.busMovesCompleted++;

                await refreshSingleJointStatusFromBus(jointNumber);
                broadcastStatusToClients();
                sendResponse({
                    type: 'success',
                    message: `Servo ${data.joint} moving to ${angle}° at ${moveSpeed} step/s`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to move joint ${data.joint}: ${formatCommandError(error)}`,
                    servoFault: !!(error && error.isOverload),
                    joint: data.joint
                });
            }
            break;
            
        case 'stopJoint':
            // Stop a specific servo
            const stopJointNumber = data.joint - 1;
            
            if (stopJointNumber < 0 || stopJointNumber >= servos.length) {
                sendResponse({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                });
                return;
            }
            
            const stopServo = servos[stopJointNumber];
            if (stopServo === null) {
                sendResponse({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                });
                return;
            }
            
            try {
                const stopped = await stopServo.stopServo();
                if (stopped) {
                    sendResponse({
                        type: 'success',
                        message: `Servo ${data.joint} stopped`
                    });
                } else {
                    sendResponse({
                        type: 'error',
                        message: `Could not disable torque on joint ${data.joint} (servo may be in fault — check load or power cycle the joint)`,
                        joint: data.joint
                    });
                }
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to stop joint ${data.joint}: ${formatCommandError(error)}`,
                    joint: data.joint
                });
            }
            break;
            
        case 'stopAll':
        case 'stopAllJoints':
            // Stop all servos
            try {
                let failedJoints = [];

                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        const stopped = await servos[i].stopServo();
                        if (!stopped) {
                            failedJoints.push(i + 1);
                        }
                    }
                }

                if (failedJoints.length === 0) {
                    sendResponse({
                        type: 'success',
                        message: 'All servos stopped'
                    });
                } else {
                    sendResponse({
                        type: 'error',
                        message: 'Could not disable torque on joint(s): ' + failedJoints.join(', ')
                    });
                }
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to stop all servos: ${formatCommandError(error)}`
                });
            }
            break;
            
        case 'setServo':
        case 'setServoAngle':
            // Set servo angle (for gripper or other servo-controlled joints)
            const servoJointNumber = data.joint - 1;
            const servoAngle = data.angle;
            
            if (servoJointNumber < 0 || servoJointNumber >= servos.length) {
                sendResponse({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                });
                return;
            }
            
            const servoJoint = servos[servoJointNumber];
            if (servoJoint === null) {
                sendResponse({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                });
                return;
            }
            
            try {
                await servoJoint.moveToAngle(servoAngle);
                sendResponse({
                    type: 'success',
                    message: `Servo ${data.joint} set to ${servoAngle}°`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set servo angle: ${error.message}`
                });
            }
            break;
            
        case 'setSpeed':
            // Set servo speed
            const speedJointNumber = data.joint - 1;
            const speed = data.speed;
            
            if (speedJointNumber < 0 || speedJointNumber >= servos.length) {
                sendResponse({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                });
                return;
            }
            
            const speedServo = servos[speedJointNumber];
            if (speedServo === null) {
                sendResponse({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                });
                return;
            }
            
            try {
                const changed = await applyJointSpeedIfChanged(speedServo, speedJointNumber, speed);
                sendResponse({
                    type: 'success',
                    message: changed
                        ? `Servo ${data.joint} speed set to ${speed} step/s`
                        : `Servo ${data.joint} speed already ${speed} step/s`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set speed on joint ${data.joint}: ${formatCommandError(error)}`,
                    servoFault: !!(error && error.isOverload),
                    joint: data.joint
                });
            }
            break;
            
        case 'setSpeedAll':
            // Set servo speed for all joints
            const speedAll = data.speed;
            
            try {
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        await applyJointSpeedIfChanged(servos[i], i, speedAll);
                    }
                }
                sendResponse({
                    type: 'success',
                    message: `All servos speed set to ${speedAll} step/s`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set servo speeds: ${error.message}`
                });
            }
            break;
            
        case 'setTorqueAll':
            // Enable or disable torque for all joints
            const torqueEnabled = data.enabled !== false; // Default to true if not specified
            
            try {
                if (cachedTorqueEnabled === torqueEnabled) {
                    sendResponse({
                        type: 'success',
                        message: `All servos torque already ${torqueEnabled ? 'enabled' : 'disabled'}`
                    });
                    return;
                }
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        if (torqueEnabled) {
                            await servos[i].startServo();
                        } else {
                            await servos[i].stopServo();
                        }
                    }
                }
                cachedTorqueEnabled = torqueEnabled;
                sendResponse({
                    type: 'success',
                    message: `All servos torque ${torqueEnabled ? 'enabled' : 'disabled'}`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to ${torqueEnabled ? 'enable' : 'disable'} torque: ${error.message}`
                });
            }
            break;
            
        case 'setAcceleration':
            // Set servo acceleration
            const accJointNumber = data.joint - 1;
            const acc = data.acceleration;
            
            if (accJointNumber < 0 || accJointNumber >= servos.length) {
                sendResponse({
                    type: 'error',
                    message: `Invalid joint number: ${data.joint}`
                });
                return;
            }
            
            const accServo = servos[accJointNumber];
            if (accServo === null) {
                sendResponse({
                    type: 'error',
                    message: `Servo ${data.joint} is not available`
                });
                return;
            }
            
            try {
                const changed = await applyJointAccelerationIfChanged(accServo, accJointNumber, acc);
                sendResponse({
                    type: 'success',
                    message: changed
                        ? `Servo ${data.joint} acceleration set to ${acc}`
                        : `Servo ${data.joint} acceleration already ${acc}`
                });
            } catch (error) {
                sendResponse({
                    type: 'error',
                    message: `Failed to set acceleration: ${error.message}`
                });
            }
            break;
            
        default:
            sendResponse({
                type: 'error',
                message: `Unknown command: ${command}`
            });
    }
}

/**
 * Cancel queued and in-flight bus work so shutdown does not wait on the serial port.
 */
function abortPendingBusWork() {
    serverShuttingDown = true;
    stopBusTickLoop();

    while (busWriteQueue.length > 0) {
        const item = busWriteQueue.shift();
        item.reject(new Error('Server shutting down'));
    }
    serverDiagnostics.busWriteQueueLength = 0;

    while (writeQueue.length > 0) {
        const item = writeQueue.shift();
        item.reject(new Error('Server shutting down'));
    }
    isWriting = false;

    for (let i = 0; i < servos.length; i++) {
        const servo = servos[i];
        if (servo && typeof servo.clearPendingBusTransaction === 'function') {
            servo.clearPendingBusTransaction();
        }
    }
    if (endTool && typeof endTool.clearPendingBusTransaction === 'function') {
        endTool.clearPendingBusTransaction();
    }
}

/**
 * Wait briefly for an in-flight bus tick to finish.
 * @returns {Promise<void>}
 */
async function waitForBusTickToFinish() {
    const waitStart = Date.now();
    while (busTickInProgress && Date.now() - waitStart < SHUTDOWN_BUS_WAIT_MS) {
        await new Promise((resolve) => setTimeout(resolve, 50));
    }
}

/**
 * Close the shared serial port with a timeout so systemctl stop does not hang.
 * @returns {Promise<void>}
 */
async function closeSharedSerialPortWithTimeout() {
    if (!sharedSerialPort || !sharedSerialPort.isOpen) {
        return;
    }

    await Promise.race([
        new Promise((resolve) => {
            sharedSerialPort.close((error) => {
                if (error) {
                    console.error('Error closing shared serial port:', error.message);
                } else {
                    console.log('Shared serial port closed');
                }
                resolve();
            });
        }),
        new Promise((resolve) => setTimeout(resolve, SHUTDOWN_SERIAL_CLOSE_MS))
    ]);
}

/**
 * Cleanup function - called on exit
 */
let shutdownInProgress = false;

async function cleanup(signalName) {
    if (shutdownInProgress) {
        return;
    }
    shutdownInProgress = true;

    debugLog('Shutting down... (signal: ' + (signalName || 'unknown') + ')');

    const forceExitTimer = setTimeout(() => {
        debugLog('Shutdown timeout — forcing exit', true);
        process.exit(0);
    }, SHUTDOWN_FORCE_EXIT_MS);

    try {
        abortPendingBusWork();
        await waitForBusTickToFinish();

        if (wss) {
            try {
                wss.close();
            } catch (error) {
                console.error('Error closing WebSocket server:', error.message);
            }
        }

        // Close the serial port only — skip per-servo stop commands that can block on bus timeouts.
        await closeSharedSerialPortWithTimeout();
    } catch (error) {
        debugLog('Shutdown error: ' + (error.message || error), true);
    } finally {
        clearTimeout(forceExitTimer);
        process.exit(0);
    }
}

// Handle process termination (systemctl stop sends SIGTERM)
process.on('SIGINT', function () {
    cleanup('SIGINT');
});
process.on('SIGTERM', function () {
    cleanup('SIGTERM');
});

// Start the server
async function main() {
    try {
        // Initialize servos
        await initializeServos();

        // Prime cache and run servo bus on a fixed tick interval
        await refreshJointStatusCacheFromBus();
        startBusTickLoop();
        
        // Start WebSocket server
        startServer();
    } catch (error) {
        const message = error && error.stack ? error.stack : String(error);
        debugLog('Failed to start server: ' + message, true);
        process.exit(1);
    }
}

// Run the main function
main();

