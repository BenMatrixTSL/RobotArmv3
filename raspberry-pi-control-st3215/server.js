'use strict';
/**
 * Robot Arm WebSocket Server (ST3215 Version)
 *
 * Main thread: WebSocket server, control-session management, instant commands, kinematics.
 * Servo bus: delegated entirely to servoWorker.js (worker_threads).
 *
 * Usage:  node server.js
 * Env:    SERIAL_PORT, STATUS_POLL_INTERVAL_MS, BUS_DIAGNOSTICS_LOG_INTERVAL_MS,
 *         ROBOT_ARM_DEBUG_LOG
 */

const WebSocket   = require('ws');
const os          = require('os');
const fs          = require('fs');
const path        = require('path');
const { exec }    = require('child_process');
const { execFile }  = require('child_process');
const { Worker }    = require('worker_threads');
const { promisify } = require('util');
const { robotKinematics } = require('./kinematicsService');
const execFileAsync = promisify(execFile);

// ===== Configuration =====
const PORT             = 8080;
const SERVER_BUILD_ID  = '2026-06-05-worker-thread';
const DEBUG_LOG_FILE   = process.env.ROBOT_ARM_DEBUG_LOG || '';

// Commands that do not touch the servo bus — handled instantly in main thread.
const INSTANT_SERVER_COMMANDS = {
    getStatus: true,
    getJointConfigs: true,
    getServerDiagnostics: true,
    getControlStatus: true,
    takeControl: true,
    releaseControl: true,
    getPiNetworkInfo: true,
    getPiEthernetSettings: true,
    setPiEthernetSettings: true,
    updatePiServerFromGit: true,
    kinematicsLoadURDF: true,
    kinematicsForwardKinematics: true,
    kinematicsForwardKinematicsSteps: true,
    kinematicsForwardKinematicsBatch: true,
    kinematicsInverseKinematics: true,
    kinematicsRefineOrientationWithAccuracy: true,
    kinematicsGetInfo: true
};

// Bus commands that require an active control session.
const BUS_WRITE_COMMANDS = {
    moveJoint: true, stopJoint: true, stopAll: true, stopAllJoints: true,
    setServo: true, setServoAngle: true, setSpeed: true, setSpeedAll: true,
    setTorqueAll: true, setAcceleration: true, rescanServos: true,
    toolPing: true, toolSetPwm: true, toolSetServoEnabled: true,
    toolSetServoPosition: true, toolSetServoAngle: true,
    toolSetWatchdog: true, toolClearFaults: true, toolReset: true
};

// Moves/stops bypass the write queue and run on the bus immediately.
const IMMEDIATE_BUS_COMMANDS = {
    moveJoint: true, stopJoint: true, stopAll: true, stopAllJoints: true
};

const CONTROL_IDLE_TIMEOUT_MS       = 5 * 60 * 1000;
const CONTROL_IDLE_CHECK_INTERVAL_MS = 30 * 1000;

// ===== Logging =====
function debugLog(message, isError) {
    const text = String(message);
    const line = '[' + new Date().toISOString() + '] ' + (isError ? '[ERROR] ' : '') + text;
    if (isError) { console.error(text); } else { console.log(text); }
    if (!DEBUG_LOG_FILE) return;
    try {
        const dir = path.dirname(DEBUG_LOG_FILE);
        if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
        fs.appendFileSync(DEBUG_LOG_FILE, line + '\n');
    } catch (e) { console.error('Could not write debug log:', e.message); }
}

debugLog('Server process starting (pid ' + process.pid + ', build ' + SERVER_BUILD_ID + ')');
if (DEBUG_LOG_FILE) debugLog('Debug log file: ' + DEBUG_LOG_FILE);

process.on('unhandledRejection', (reason) => {
    const msg = reason && reason.stack ? reason.stack : String(reason);
    debugLog('Unhandled promise rejection: ' + msg, true);
});
process.on('uncaughtException', (error) => {
    debugLog('Uncaught exception: ' + (error.stack || error), true);
    process.exit(1);
});
process.on('exit', (code) => { debugLog('Process exit, code=' + code); });

// ===== Shared State =====
let wss                   = null;
let servoWorker           = null;
let serverShuttingDown    = false;
let shutdownInProgress    = false;
const serverStartedAt     = Date.now();

const connectedClients    = new Set();
let nextClientId          = 1;
const clientMap           = new Map(); // clientId → WebSocket

// Caches populated by the worker; served to clients without touching the bus.
let lastKnownStatusJoints  = [];
let lastKnownCacheAgeMs    = null;
let lastKnownJointConfigs  = { count: 0, total: 6, joints: [] };
let lastWorkerDiagnostics  = {};

// Control session (one app at a time can move the arm).
const controlSession = { ws: null, label: '', hostname: null, clientIp: null, since: null, lastMoveAt: null };

// ===== Control Session =====
function normalizeClientIp(raw) {
    if (!raw || typeof raw !== 'string') return null;
    if (raw.startsWith('::ffff:')) return raw.substring(7);
    if (raw === '::1') return '127.0.0.1';
    return raw;
}
function isLocalClientIp(ip) { return !ip || ip === '127.0.0.1'; }
function getWsClientHostname(ws) {
    if (ws && ws.clientHostname) return ws.clientHostname;
    if (ws && ws.clientIp && isLocalClientIp(ws.clientIp)) {
        try { return os.hostname(); } catch (e) { return null; }
    }
    return null;
}
function assignControlSession(ws, label) {
    controlSession.ws        = ws;
    controlSession.label     = (typeof label === 'string' && label) ? label : 'client';
    controlSession.clientIp  = ws ? ws.clientIp : null;
    controlSession.hostname  = ws ? getWsClientHostname(ws) : null;
    controlSession.since     = Date.now();
    controlSession.lastMoveAt = Date.now();
}
function touchControlMoveActivity(ws) {
    if (controlSession.ws === ws) controlSession.lastMoveAt = Date.now();
}
function releaseControlSession(ws) {
    if (controlSession.ws === ws) {
        controlSession.ws = null; controlSession.label = '';
        controlSession.hostname = null; controlSession.clientIp = null;
        controlSession.since = null; controlSession.lastMoveAt = null;
    }
}
function pruneStaleControlSession() {
    if (controlSession.ws && controlSession.ws.readyState !== WebSocket.OPEN) {
        debugLog('Releasing arm control from disconnected client');
        releaseControlSession(controlSession.ws);
    }
}
function requireControl(ws) {
    pruneStaleControlSession();
    if (!controlSession.ws) { assignControlSession(ws, 'auto'); return { ok: true }; }
    if (controlSession.ws !== ws) {
        const who = formatControlHolder(controlSession) || 'another client';
        return { ok: false, message: 'Arm control is held by ' + who + '. Only one app can move the arm at a time.' };
    }
    return { ok: true };
}
function requireControlForCommand(ws, command) {
    if (!BUS_WRITE_COMMANDS[command]) return { ok: true };
    return requireControl(ws);
}
function formatControlHolder(info) {
    if (!info || !info.ws) return null;
    const hostname = info.hostname || getWsClientHostname(info.ws);
    const ip       = info.clientIp || (info.ws ? info.ws.clientIp : null);
    const label    = info.label;
    if (hostname && ip && !isLocalClientIp(ip)) return hostname + ' (' + ip + ')';
    if (hostname) return isLocalClientIp(ip) ? hostname + ' (this Pi)' : hostname;
    if (ip && !isLocalClientIp(ip)) return ip;
    if (ip && isLocalClientIp(ip)) { try { return os.hostname() + ' (this Pi)'; } catch (e) { return 'this Pi'; } }
    if (label && label !== 'auto' && label !== 'client' && label !== 'electron') return label;
    return 'another app';
}
function getControlStatusPayload(ws, extra) {
    const payload = {
        type: 'controlStatus',
        hasControl:      controlSession.ws === ws,
        youHaveControl:  controlSession.ws === ws,
        hasHolder:       !!controlSession.ws,
        holder:          controlSession.ws ? formatControlHolder(controlSession) : null,
        holderHostname:  controlSession.hostname || null,
        holderIp:        controlSession.clientIp || null,
        holderLabel:     controlSession.label || null
    };
    if (extra) Object.assign(payload, extra);
    return payload;
}
function broadcastControlStatus(extraForClient) {
    connectedClients.forEach((clientWs) => {
        if (clientWs.readyState !== WebSocket.OPEN) return;
        const payload = getControlStatusPayload(clientWs);
        if (extraForClient && extraForClient.ws === clientWs) Object.assign(payload, extraForClient.data);
        clientWs.send(JSON.stringify(payload));
    });
}
function releaseControlForIdle() {
    if (!controlSession.ws || !controlSession.lastMoveAt) return;
    if (Date.now() - controlSession.lastMoveAt < CONTROL_IDLE_TIMEOUT_MS) return;
    const idleWs = controlSession.ws;
    releaseControlSession(idleWs);
    broadcastControlStatus({ ws: idleWs, data: { message: 'Arm control released automatically after 5 minutes with no movement' } });
    debugLog('Arm control released due to idle timeout');
}

// ===== Worker Status Broadcast =====
function broadcastStatusToClients() {
    if (connectedClients.size === 0) return;
    const payload = JSON.stringify({ type: 'status', joints: lastKnownStatusJoints, cacheAgeMs: lastKnownCacheAgeMs, pushed: true });
    connectedClients.forEach((ws) => {
        if (ws.readyState === WebSocket.OPEN) ws.send(payload);
    });
}
function broadcastJointConfigs() {
    if (connectedClients.size === 0) return;
    const payload = JSON.stringify({ type: 'jointConfigs', count: lastKnownJointConfigs.count, total: lastKnownJointConfigs.total, joints: lastKnownJointConfigs.joints });
    connectedClients.forEach((ws) => {
        if (ws.readyState === WebSocket.OPEN) ws.send(payload);
    });
}

// ===== Worker Message Routing =====
function handleWorkerMessage(msg) {
    if (!msg || !msg.type) return;

    if (msg.type === 'status') {
        lastKnownStatusJoints = msg.joints || [];
        lastKnownCacheAgeMs   = msg.cacheAgeMs;
        if (msg.diagnostics)  lastWorkerDiagnostics = msg.diagnostics;
        broadcastStatusToClients();
        return;
    }

    if (msg.type === 'ready') {
        if (msg.jointConfigs) lastKnownJointConfigs = msg.jointConfigs;
        debugLog('Servo worker ready — starting WebSocket server');
        startServer();
        return;
    }

    if (msg.type === 'jointConfigs') {
        lastKnownJointConfigs = { count: msg.count, total: msg.total, joints: msg.joints };
        broadcastJointConfigs();
        return;
    }

    if (msg.type === 'commandResponse') {
        const ws = clientMap.get(msg.clientId);
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(msg.payload));
        }
        return;
    }

    if (msg.type === 'initError') {
        debugLog('Servo worker failed to initialize: ' + msg.message, true);
        process.exit(1);
    }
}

// ===== Worker Thread Setup =====
function startServoWorker() {
    servoWorker = new Worker(path.join(__dirname, 'servoWorker.js'));
    servoWorker.on('message', handleWorkerMessage);
    servoWorker.on('error', (err) => {
        debugLog('Servo worker error: ' + (err.message || err), true);
    });
    servoWorker.on('exit', (code) => {
        if (!serverShuttingDown) {
            debugLog('Servo worker exited unexpectedly with code ' + code, true);
        }
    });
}

// ===== Ethernet / Network Helpers =====
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
        for (const b of bits) { if (b === '1') prefix++; }
    }
    return prefix;
}
function prefixToSubnetMask(prefix) {
    const p = parseInt(prefix, 10);
    if (isNaN(p) || p < 0 || p > 32) return null;
    let bits = '';
    for (let i = 0; i < 32; i++) bits += i < p ? '1' : '0';
    const parts = [];
    for (let i = 0; i < 4; i++) parts.push(parseInt(bits.slice(i * 8, i * 8 + 8), 2));
    return parts.join('.');
}
async function runNmcli(args) {
    const result = await execFileAsync('nmcli', args, { timeout: 5000 });
    return (result.stdout || '').trim();
}
async function getEthernetSettings() {
    const runningText = await runNmcli(['-t', '-f', 'RUNNING', 'general', 'status']);
    if (runningText.toLowerCase() !== 'running') throw new Error('NetworkManager is not running');
    const statusText = await runNmcli(['-t', '-f', 'DEVICE,TYPE,STATE,CONNECTION', 'device', 'status']);
    const lines = statusText.split('\n').filter(l => l.trim().length > 0);
    let selected = null;
    for (const line of lines) {
        const parts = line.split(':');
        if (parts.length < 4) continue;
        const [device, type, state] = parts;
        const connectionName = parts.slice(3).join(':');
        if (type === 'ethernet') { selected = { device, type, state, connectionName }; if (device === 'eth0') break; }
    }
    if (!selected || !selected.connectionName || selected.connectionName === '--') throw new Error('No ethernet NetworkManager connection found');
    const conn = selected.connectionName;
    const connDataText = await runNmcli(['-g', 'ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.dns', 'connection', 'show', conn]);
    const connLines = connDataText.split('\n');
    const ipv4Method    = (connLines[0] || '').trim();
    const ipv4Addresses = (connLines[1] || '').trim();
    const ipv4Gateway   = (connLines[2] || '').trim();
    const ipv4Dns       = (connLines[3] || '').trim();
    let ipAddress = '', prefix = '', subnetMask = '';
    if (ipv4Addresses) {
        const first = ipv4Addresses.split(',')[0].trim();
        const ap = first.split('/');
        ipAddress = (ap[0] || '').trim();
        prefix    = (ap[1] || '').trim();
        subnetMask = prefixToSubnetMask(prefix) || '';
    }
    return { connectionName: conn, device: selected.device, state: selected.state, method: ipv4Method || 'auto', ipAddress, subnetMask, prefix, gateway: ipv4Gateway, dns: ipv4Dns };
}
async function setEthernetSettings(input) {
    const current = await getEthernetSettings();
    const conn = current.connectionName;
    const mode = (input && typeof input.mode === 'string') ? input.mode.trim().toLowerCase() : '';
    if (mode !== 'dhcp' && mode !== 'static') throw new Error('Mode must be "dhcp" or "static"');
    if (mode === 'dhcp') {
        await runNmcli(['connection', 'modify', conn, 'ipv4.method', 'auto', 'ipv4.addresses', '', 'ipv4.gateway', '', 'ipv4.dns', '']);
        await runNmcli(['connection', 'up', conn]);
        return await getEthernetSettings();
    }
    const ipAddress  = (input && typeof input.ipAddress  === 'string') ? input.ipAddress.trim()  : '';
    const subnetMask = (input && typeof input.subnetMask === 'string') ? input.subnetMask.trim() : '';
    const gateway    = (input && typeof input.gateway    === 'string') ? input.gateway.trim()    : '';
    const dns        = (input && typeof input.dns        === 'string') ? input.dns.trim()        : '';
    if (!ipAddress)  throw new Error('IP address is required for static mode');
    if (!subnetMask) throw new Error('Subnet mask is required for static mode');
    if (!gateway)    throw new Error('Gateway is required for static mode');
    const prefix = subnetMaskToPrefix(subnetMask);
    if (prefix === null) throw new Error('Invalid subnet mask');
    const args = ['connection', 'modify', conn, 'ipv4.method', 'manual', 'ipv4.addresses', `${ipAddress}/${prefix}`, 'ipv4.gateway', gateway];
    if (dns) args.push('ipv4.dns', dns); else args.push('ipv4.dns', '');
    await runNmcli(args);
    await runNmcli(['connection', 'up', conn]);
    return await getEthernetSettings();
}

// ===== Instant Command Handler =====
async function handleCommand(ws, data) {
    const command   = data.command;
    const requestId = data.requestId;
    const sendResponse = (payload) => {
        if (requestId !== undefined) payload.requestId = requestId;
        ws.send(JSON.stringify(payload));
    };

    switch (command) {

        case 'getStatus': {
            sendResponse({ type: 'status', joints: lastKnownStatusJoints, cacheAgeMs: lastKnownCacheAgeMs });
            break;
        }

        case 'getJointConfigs': {
            sendResponse({ type: 'jointConfigs', count: lastKnownJointConfigs.count, total: lastKnownJointConfigs.total, joints: lastKnownJointConfigs.joints });
            break;
        }

        case 'getServerDiagnostics': {
            sendResponse({ type: 'serverDiagnostics', diagnostics: { ...lastWorkerDiagnostics, wsClients: connectedClients.size, uptimeMs: Date.now() - serverStartedAt, serverBuildId: SERVER_BUILD_ID } });
            break;
        }

        case 'takeControl': {
            const forceTake      = data.force === true;
            const previousWs     = controlSession.ws;
            const previousHolder = controlSession.ws ? formatControlHolder(controlSession) : null;
            if (typeof data.hostname === 'string' && data.hostname.trim()) ws.clientHostname = data.hostname.trim();
            if (controlSession.ws && controlSession.ws !== ws && !forceTake) { sendResponse(getControlStatusPayload(ws)); return; }
            const label = (typeof data.label === 'string' && data.label) ? data.label : 'client';
            assignControlSession(ws, label);
            if (previousWs && previousWs !== ws && forceTake) {
                try { previousWs.send(JSON.stringify(getControlStatusPayload(previousWs, { message: 'Another app took arm control' }))); } catch (e) { /* disconnected */ }
            }
            sendResponse(getControlStatusPayload(ws, { takenFrom: (forceTake && previousWs && previousWs !== ws) ? previousHolder : null }));
            break;
        }

        case 'releaseControl': {
            if (controlSession.ws === ws) releaseControlSession(ws);
            sendResponse(getControlStatusPayload(ws));
            break;
        }

        case 'getControlStatus': {
            sendResponse(getControlStatusPayload(ws));
            break;
        }

        case 'getPiNetworkInfo': {
            try {
                const hostname   = os.hostname();
                const interfaces = os.networkInterfaces() || {};
                const ifaceSummaries = [];
                Object.keys(interfaces).forEach((name) => {
                    (interfaces[name] || []).forEach((info) => {
                        if (info && info.family === 'IPv4' && !info.internal) ifaceSummaries.push({ name, address: info.address, mac: info.mac || null });
                    });
                });
                let gateway = null;
                try {
                    const routeText = fs.readFileSync('/proc/net/route', 'utf8');
                    for (const line of routeText.trim().split('\n').slice(1)) {
                        const parts = line.trim().split(/\s+/);
                        if (parts.length >= 3 && parts[1] === '00000000' && (parseInt(parts[3] || '0', 16) & 0x2)) {
                            const n = parseInt(parts[2], 16);
                            gateway = `${n & 0xFF}.${(n >> 8) & 0xFF}.${(n >> 16) & 0xFF}.${(n >> 24) & 0xFF}`;
                            break;
                        }
                    }
                } catch (e) { /* not Linux or no route file */ }
                sendResponse({ type: 'networkInfo', hostname, interfaces: ifaceSummaries, gateway });
            } catch (error) {
                sendResponse({ type: 'error', message: 'Failed to read network info: ' + (error.message || error) });
            }
            break;
        }

        case 'getPiEthernetSettings': {
            try {
                sendResponse({ type: 'ethernetSettings', ethernet: await getEthernetSettings() });
            } catch (error) {
                sendResponse({ type: 'error', message: 'Failed to get ethernet settings: ' + (error.message || error) });
            }
            break;
        }

        case 'setPiEthernetSettings': {
            try {
                sendResponse({ type: 'ethernetSettingsUpdated', ethernet: await setEthernetSettings(data || {}), message: 'Ethernet settings updated' });
            } catch (error) {
                sendResponse({ type: 'error', message: 'Failed to set ethernet settings: ' + (error.message || error) });
            }
            break;
        }

        case 'updatePiServerFromGit': {
            exec('git pull --ff-only', { cwd: __dirname }, (error, stdout, stderr) => {
                if (error) {
                    sendResponse({ type: 'updateResult', ok: false, target: 'st3215', message: 'git pull failed: ' + (stderr || error.message) });
                } else {
                    sendResponse({ type: 'updateResult', ok: true, target: 'st3215', message: stdout.trim() });
                    setTimeout(() => { try { process.exit(0); } catch (e) { /* ignore */ } }, 500);
                }
            });
            break;
        }

        case 'kinematicsLoadURDF': {
            try {
                const urdfXml = data.urdfXml;
                if (typeof urdfXml !== 'string' || !urdfXml.trim()) throw new Error('URDF text is empty');
                const info = robotKinematics.loadURDF(urdfXml);
                sendResponse({ type: 'kinematicsLoaded', configured: info.configured, jointCount: info.jointCount, joints: info.joints, urdfData: info.urdfData, maxReachMm: info.maxReachMm });
            } catch (error) {
                sendResponse({ type: 'error', message: `Failed to load URDF: ${error.message}` });
            }
            break;
        }

        case 'kinematicsForwardKinematics': {
            try {
                sendResponse({ type: 'kinematicsForwardResult', result: robotKinematics.forwardKinematics(data.jointAngles) });
            } catch (error) {
                sendResponse({ type: 'error', message: `Forward kinematics failed: ${error.message}` });
            }
            break;
        }

        case 'kinematicsForwardKinematicsSteps': {
            try {
                sendResponse({ type: 'kinematicsForwardStepsResult', result: robotKinematics.getForwardKinematicsSteps(data.jointAngles) });
            } catch (error) {
                sendResponse({ type: 'error', message: `Forward kinematics steps failed: ${error.message}` });
            }
            break;
        }

        case 'kinematicsForwardKinematicsBatch': {
            try {
                if (!Array.isArray(data.jointAnglesList)) throw new Error('jointAnglesList must be an array');
                const positions = data.jointAnglesList.map(a => robotKinematics.forwardKinematics(a).position);
                sendResponse({ type: 'kinematicsForwardBatchResult', positions });
            } catch (error) {
                sendResponse({ type: 'error', message: `Forward kinematics batch failed: ${error.message}` });
            }
            break;
        }

        case 'kinematicsInverseKinematics': {
            try {
                sendResponse({ type: 'kinematicsInverseResult', result: robotKinematics.inverseKinematics(data.targetPose, data.initialAngles) });
            } catch (error) {
                sendResponse({ type: 'error', message: `Inverse kinematics failed: ${error.message}` });
            }
            break;
        }

        case 'kinematicsRefineOrientationWithAccuracy': {
            try {
                sendResponse({ type: 'kinematicsRefineOrientationResult', result: robotKinematics.refineOrientationWithAccuracy(data.targetPose, data.baseAngles, data.desiredOrientation) });
            } catch (error) {
                sendResponse({ type: 'error', message: `Refine orientation failed: ${error.message}` });
            }
            break;
        }

        case 'kinematicsGetInfo': {
            try {
                const info = robotKinematics.getKinematicsInfo();
                sendResponse({ type: 'kinematicsInfo', configured: info.configured, jointCount: info.jointCount, joints: info.joints, urdfData: info.urdfData, maxReachMm: info.maxReachMm });
            } catch (error) {
                sendResponse({ type: 'error', message: `Failed to get kinematics info: ${error.message}` });
            }
            break;
        }

        default:
            ws.send(JSON.stringify({ type: 'error', message: `Unknown command: ${command}` }));
    }
}

// ===== WebSocket Server =====
function startServer() {
    debugLog('Starting WebSocket server on port ' + PORT + '...');
    wss = new WebSocket.Server({ port: PORT });

    wss.on('connection', function connection(ws, req) {
        const clientIp = normalizeClientIp(req.socket.remoteAddress);
        ws.clientIp       = clientIp;
        ws.clientHostname = null;
        ws.clientId       = nextClientId++;
        connectedClients.add(ws);
        clientMap.set(ws.clientId, ws);
        debugLog('Client connected from ' + (clientIp || 'unknown') + ' (id=' + ws.clientId + ')');

        // Welcome message
        ws.send(JSON.stringify({ type: 'connected', message: 'Connected to Robot Arm Server (ST3215)', pushesStatus: true, statusIntervalMs: parseInt(process.env.STATUS_POLL_INTERVAL_MS || '20', 10), busTickIntervalMs: parseInt(process.env.STATUS_POLL_INTERVAL_MS || '20', 10), serverBuildId: SERVER_BUILD_ID }));

        // Send cached state immediately (no bus hit)
        ws.send(JSON.stringify({ type: 'status', joints: lastKnownStatusJoints, cacheAgeMs: lastKnownCacheAgeMs, pushed: true }));
        ws.send(JSON.stringify({ type: 'jointConfigs', count: lastKnownJointConfigs.count, total: lastKnownJointConfigs.total, joints: lastKnownJointConfigs.joints }));
        ws.send(JSON.stringify(getControlStatusPayload(ws)));

        ws.on('message', async function incoming(message) {
            try {
                const data = JSON.parse(message);
                if (!data || typeof data.command !== 'string') return;

                // Instant commands are handled entirely in this thread.
                if (INSTANT_SERVER_COMMANDS[data.command]) {
                    await handleCommand(ws, data);
                    return;
                }

                // Control check before forwarding any bus-write command.
                if (BUS_WRITE_COMMANDS[data.command]) {
                    const controlCheck = requireControlForCommand(ws, data.command);
                    if (!controlCheck.ok) {
                        const errPayload = { type: 'error', message: controlCheck.message, controlRequired: true };
                        if (data.requestId !== undefined) errPayload.requestId = data.requestId;
                        ws.send(JSON.stringify(errPayload));
                        return;
                    }
                    if (data.command === 'moveJoint' || data.command === 'setServo' || data.command === 'setServoAngle' || data.command === 'setAcceleration') {
                        touchControlMoveActivity(ws);
                    }
                }

                if (!servoWorker) {
                    const errPayload = { type: 'error', message: 'Servo worker not ready' };
                    if (data.requestId !== undefined) errPayload.requestId = data.requestId;
                    ws.send(JSON.stringify(errPayload));
                    return;
                }

                // Moves and stops bypass the bus write queue.
                const msgType = IMMEDIATE_BUS_COMMANDS[data.command] ? 'immediateBusCommand' : 'busCommand';
                servoWorker.postMessage({ type: msgType, clientId: ws.clientId, ...data });

            } catch (error) {
                ws.send(JSON.stringify({ type: 'error', message: error.message }));
            }
        });

        ws.on('close', () => {
            connectedClients.delete(ws);
            clientMap.delete(ws.clientId);
            releaseControlSession(ws);
            debugLog('Client disconnected from ' + clientIp + ' (id=' + ws.clientId + ')');
        });

        ws.on('error', (error) => {
            console.error('WebSocket error:', error.message || error);
        });
    });

    setInterval(releaseControlForIdle, CONTROL_IDLE_CHECK_INTERVAL_MS);
    debugLog('WebSocket server listening on port ' + PORT);
}

// ===== Shutdown =====
async function cleanup(signalName) {
    if (shutdownInProgress) return;
    shutdownInProgress = true;
    serverShuttingDown = true;
    debugLog('Shutting down... (signal: ' + (signalName || 'unknown') + ')');

    const forceExitTimer = setTimeout(() => {
        debugLog('Shutdown timeout — forcing exit', true);
        process.exit(0);
    }, 8000);

    try {
        if (servoWorker) {
            servoWorker.postMessage({ type: 'shutdown' });
            await new Promise(resolve => servoWorker.once('exit', resolve));
        }
        if (wss) { try { wss.close(); } catch (e) { /* ignore */ } }
    } catch (error) {
        debugLog('Shutdown error: ' + (error.message || error), true);
    } finally {
        clearTimeout(forceExitTimer);
        process.exit(0);
    }
}

process.on('SIGINT',  () => cleanup('SIGINT'));
process.on('SIGTERM', () => cleanup('SIGTERM'));

// ===== Main =====
startServoWorker();
// WebSocket server starts once the worker signals 'ready'.
