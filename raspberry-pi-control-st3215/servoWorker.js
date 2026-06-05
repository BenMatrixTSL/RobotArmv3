'use strict';
/**
 * servoWorker.js — Dedicated child process for ST3215 serial bus communication.
 *
 * Runs as a forked child process (child_process.fork) so native serialport
 * bindings stay in their own process and cannot SIGABRT the WebSocket server.
 * Communicates with server.js via process.send / process.on('message'):
 *
 *   Child → Parent:
 *     { type: 'ready',           jointConfigs: {...} }
 *     { type: 'status',          joints, cacheAgeMs, diagnostics }
 *     { type: 'jointConfigs',    count, total, joints }
 *     { type: 'commandResponse', clientId, requestId, payload }
 *     { type: 'initError',       message }
 *
 *   Parent → Child:
 *     { type: 'busCommand',          clientId, command, requestId, ...data }
 *     { type: 'immediateBusCommand', clientId, command, requestId, ...data }
 *     { type: 'shutdown' }
 */

const { SerialPort } = require('serialport');
const RobotArm = require('./robotArmST3215');

// ===== Configuration (mirrors server.js constants) =====
const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/serial0';
const SERIAL_BAUDRATE = 1000000;
const JOINT_COUNT = 6;
const SERVO_IDS = [1, 2, 3, 4, 5, 6];
const DEBUG = false;
const PERF_DEBUG = false;
const BUS_DIAGNOSTICS_LOG_INTERVAL_MS = parseInt(process.env.BUS_DIAGNOSTICS_LOG_INTERVAL_MS || '0', 10);

const STATUS_POLL_INTERVAL_MS = parseInt(process.env.STATUS_POLL_INTERVAL_MS || '20', 10);
const MIN_BUS_TICK_GAP_MS     = parseInt(process.env.MIN_BUS_TICK_GAP_MS     || '4',  10);
const JOINTS_PER_POLL_TICK    = parseInt(process.env.JOINTS_PER_POLL_TICK    || '6',  10);
const MAX_BUS_WRITE_QUEUE_SIZE  = 100;
const MAX_BUS_WRITES_WHEN_IDLE  = 0;
const MAX_BUS_WRITES_WHEN_BUSY  = 4;
const BUS_QUIET_BEFORE_MOVE_MS  = 12;
const BUS_QUIET_AFTER_MOVE_MS   = 10;
const BUS_QUIET_JOG_MS          = 4;
const RESCAN_MIN_INTERVAL_MS    = 10000;

const SERVO_BACKOFF_FAIL_THRESHOLD = 3;
const SERVO_BACKOFF_DURATION_MS    = 2000;

const PRIORITY_BUS_COMMANDS = { moveJoint: true, stopJoint: true, stopAll: true, stopAllJoints: true, setTorqueAll: true };

// ===== State =====
let sharedSerialPort     = null;
const servos             = [];
let endTool              = null;
let allServoControllers  = [];
const jointStatusCache   = [];
const jointSettingsCache = [];
let cachedTorqueEnabled  = null;
let busTickTimer         = null;
let busTickLoopActive    = false;
let busTickInProgress    = false;
let busWriteQueue        = [];
let statusPollJointIndex = 0;
const pendingImmediateMoves = {};
let immediateMoveDrainRunning = false;
let workerShuttingDown  = false;
let lastRescanTime      = 0;
let cachedJointConfigs  = null;
let writeQueue          = [];
let isWriting           = false;

const servoConsecFails    = new Array(JOINT_COUNT).fill(0);
const servoBackoffUntilMs = new Array(JOINT_COUNT).fill(0);

const diag = {
    startedAt: Date.now(),
    busTickIntervalMs: STATUS_POLL_INTERVAL_MS,
    busTicks: 0, busTicksSkipped: 0, busTickErrors: 0,
    lastBusTickAt: null, lastBusTickDurationMs: 0,
    busWriteQueueLength: 0, busWritesCompleted: 0, busMovesCompleted: 0,
    busWritesFailed: 0, busWritesRejected: 0,
    lastBusWriteAt: null, lastBusWriteDurationMs: 0,
    statusPollsCompleted: 0, statusPollJointIndex: 0,
    lastStatusPollAt: null, lastStatusPollDurationMs: 0,
    cacheAgeMs: null, immediateBusCommands: 0, writeTimeouts: 0
};

// ===== Helpers =====
function log(msg, isError) {
    if (isError) {
        console.error(msg);
    } else {
        console.log(msg);
    }
}

function sendResponse(clientId, requestId, payload) {
    if (requestId !== undefined && requestId !== null) {
        payload.requestId = requestId;
    }
    process.send({ type: 'commandResponse', clientId, payload });
}

function formatCommandError(error) {
    if (!error) return 'Unknown error';
    let msg = error.message || String(error);
    if (error.isOverload) msg += ' Reduce speed, check for a mechanical limit, or relieve load on the joint.';
    return msg;
}

// ===== Serial Write Queue =====
async function queueWrite(writeFn) {
    return new Promise((resolve, reject) => {
        writeQueue.push({ writeFn, resolve, reject });
        processWriteQueue();
    });
}

async function processWriteQueue() {
    if (isWriting || writeQueue.length === 0) return;
    isWriting = true;
    const { writeFn, resolve, reject } = writeQueue.shift();
    try {
        await writeFn();
        resolve();
    } catch (error) {
        reject(error);
    } finally {
        isWriting = false;
        processWriteQueue();
    }
}

// ===== Serial Data Routing =====
function routeIncomingSerialData(data) {
    for (let i = 0; i < servos.length; i++) {
        if (servos[i] && typeof servos[i].handleIncomingData === 'function') {
            servos[i].handleIncomingData(data);
        }
    }
    if (endTool && typeof endTool.handleIncomingData === 'function') {
        endTool.handleIncomingData(data);
    }
}

function refreshDataRoutingControllers() {
    allServoControllers = servos.filter(s => s !== null);
    if (endTool) allServoControllers.push(endTool);
}

// ===== Joint Status Cache =====
function defaultJointStatus(jointNum) {
    return { joint: jointNum, available: false, angleDegrees: 0, position: 2048, isMoving: false, speed: 0, load: 0, voltage: 0, temperature: 0, torqueEnabled: false, readStale: false, lastGoodAt: null };
}

function getJointStatusSnapshot() {
    const snapshot = [];
    for (let i = 0; i < JOINT_COUNT; i++) {
        snapshot.push(jointStatusCache[i] ? { ...jointStatusCache[i] } : defaultJointStatus(i + 1));
    }
    return snapshot;
}

function getJointStatusCacheAgeMs() {
    let oldest = null;
    for (let i = 0; i < jointStatusCache.length; i++) {
        const entry = jointStatusCache[i];
        if (entry && entry.lastGoodAt) {
            const age = Date.now() - entry.lastGoodAt;
            if (oldest === null || age > oldest) oldest = age;
        }
    }
    return oldest;
}

// ===== Joint Config Cache =====
function rebuildJointConfigsCache() {
    const discovered = servos.filter(s => s !== null).length;
    const joints = [];
    for (let i = 0; i < servos.length; i++) {
        joints.push(servos[i] !== null
            ? { jointNumber: i + 1, servoId: servos[i].servoIdNumber, available: true }
            : { jointNumber: i + 1, servoId: SERVO_IDS[i], available: false });
    }
    cachedJointConfigs = { count: discovered, total: JOINT_COUNT, joints };
    process.send({ type: 'jointConfigs', count: discovered, total: JOINT_COUNT, joints });
}

function getJointConfigsSnapshot() {
    if (!cachedJointConfigs) return { count: 0, total: JOINT_COUNT, joints: [] };
    return { count: cachedJointConfigs.count, total: cachedJointConfigs.total, joints: cachedJointConfigs.joints.map(j => ({ ...j })) };
}

// ===== Status Push to Main Thread =====
function postStatusToMain() {
    diag.cacheAgeMs = getJointStatusCacheAgeMs();
    process.send({
        type: 'status',
        joints: getJointStatusSnapshot(),
        cacheAgeMs: diag.cacheAgeMs,
        diagnostics: { ...diag, busWriteQueueLength: busWriteQueue.length }
    });
}

// ===== Servo Polling =====
async function readServoQuickStatusWithRetry(servo) {
    try {
        return await servo.readQuickStatus();
    } catch (firstError) {
        await new Promise(r => setTimeout(r, 5));
        return await servo.readQuickStatus();
    }
}

async function refreshSingleJointStatusFromBus(jointIndex) {
    const jointNum = jointIndex + 1;
    const servo    = servos[jointIndex];
    const previous = jointStatusCache[jointIndex] || defaultJointStatus(jointNum);

    if (servo === null) {
        jointStatusCache[jointIndex] = defaultJointStatus(jointNum);
        return;
    }

    const nowMs = Date.now();
    if (servoBackoffUntilMs[jointIndex] > nowMs) return;

    const startServo = Date.now();
    try {
        const status = await readServoQuickStatusWithRetry(servo);
        servoConsecFails[jointIndex] = 0;
        jointStatusCache[jointIndex] = { joint: jointNum, available: true, ...status, stepPosition: status.position, readStale: false, lastGoodAt: Date.now() };
    } catch (error) {
        servoConsecFails[jointIndex]++;
        if (servoConsecFails[jointIndex] >= SERVO_BACKOFF_FAIL_THRESHOLD) {
            servoBackoffUntilMs[jointIndex] = Date.now() + SERVO_BACKOFF_DURATION_MS;
            servoConsecFails[jointIndex] = 0;
            log(`Joint ${jointNum}: entering ${SERVO_BACKOFF_DURATION_MS}ms backoff after repeated read failures`);
        }
        if (previous.lastGoodAt) {
            jointStatusCache[jointIndex] = { ...previous, available: true, readStale: true, pollError: error.message };
        } else {
            jointStatusCache[jointIndex] = { ...defaultJointStatus(jointNum), readStale: true, pollError: error.message };
        }
        log(`Status poll joint ${jointNum}: ${error.message}`);
    }

    const dur = Date.now() - startServo;
    if (PERF_DEBUG && dur > 50) log(`PERF: status poll joint ${jointNum} took ${dur} ms`);
}

async function refreshJointStatusCacheFromBus() {
    for (let i = 0; i < servos.length; i++) {
        await refreshSingleJointStatusFromBus(i);
        await new Promise(r => setTimeout(r, 5));
    }
}

// ===== Bus Tick Loop =====
async function runBusTick() {
    if (workerShuttingDown || !busTickLoopActive) return;
    if (busTickInProgress) { diag.busTicksSkipped++; return; }

    busTickInProgress = true;
    const tickStart = Date.now();
    diag.busTicks++;

    try {
        const queueAtStart = busWriteQueue.length;
        const maxWrites = queueAtStart > 0 ? MAX_BUS_WRITES_WHEN_BUSY : MAX_BUS_WRITES_WHEN_IDLE;

        for (let w = 0; w < maxWrites && busWriteQueue.length > 0 && !workerShuttingDown; w++) {
            const { commandFn, resolve, reject } = busWriteQueue.shift();
            diag.busWriteQueueLength = busWriteQueue.length;
            const writeStart = Date.now();
            try {
                await commandFn();
                await new Promise(r => setTimeout(r, 15));
                resolve();
                diag.busWritesCompleted++;
                diag.lastBusWriteDurationMs = Date.now() - writeStart;
                diag.lastBusWriteAt = Date.now();
            } catch (error) {
                reject(error);
                diag.busWritesFailed++;
            }
        }

        const pollStart = Date.now();
        const jointsToPoll = Math.min(JOINTS_PER_POLL_TICK, JOINT_COUNT);
        for (let p = 0; p < jointsToPoll; p++) {
            await refreshSingleJointStatusFromBus(statusPollJointIndex);
            statusPollJointIndex = (statusPollJointIndex + 1) % JOINT_COUNT;
            if (p < jointsToPoll - 1) await new Promise(r => setTimeout(r, 1));
        }
        diag.statusPollJointIndex = statusPollJointIndex;
        await new Promise(r => setTimeout(r, 1));

        postStatusToMain();
        diag.statusPollsCompleted++;
        diag.lastStatusPollDurationMs = Date.now() - pollStart;
        diag.lastStatusPollAt = Date.now();
    } catch (error) {
        diag.busTickErrors++;
        log('Bus tick error: ' + (error.message || error), true);
    } finally {
        diag.lastBusTickDurationMs = Date.now() - tickStart;
        diag.lastBusTickAt = Date.now();
        diag.busWriteQueueLength = busWriteQueue.length;
        busTickInProgress = false;

        if (busTickLoopActive && !workerShuttingDown) {
            const elapsed = Date.now() - tickStart;
            const delay = Math.max(MIN_BUS_TICK_GAP_MS, STATUS_POLL_INTERVAL_MS - elapsed);
            scheduleNextBusTick(delay);
        }
    }
}

function scheduleNextBusTick(delayMs) {
    if (!busTickLoopActive || workerShuttingDown) return;
    if (busTickTimer) clearTimeout(busTickTimer);
    busTickTimer = setTimeout(() => { busTickTimer = null; runBusTick(); }, delayMs);
}

function startBusTickLoop() {
    if (busTickLoopActive) return;
    busTickLoopActive = true;
    log('Starting bus tick loop (gap ' + MIN_BUS_TICK_GAP_MS + ' ms, target ' + STATUS_POLL_INTERVAL_MS + ' ms)');
    runBusTick();

    if (BUS_DIAGNOSTICS_LOG_INTERVAL_MS > 0) {
        setInterval(() => {
            log('BUS diag: ticks=' + diag.busTicks + ' skipped=' + diag.busTicksSkipped + ' writeQ=' + busWriteQueue.length + ' cacheAgeMs=' + diag.cacheAgeMs + ' lastTickMs=' + diag.lastBusTickDurationMs);
        }, BUS_DIAGNOSTICS_LOG_INTERVAL_MS);
    }
}

function stopBusTickLoop() {
    busTickLoopActive = false;
    if (busTickTimer) { clearTimeout(busTickTimer); busTickTimer = null; }
}

// ===== Bus Write Queue =====
function enqueueBusWrite(commandFn, meta) {
    return new Promise((resolve, reject) => {
        if (workerShuttingDown) { reject(new Error('Worker is shutting down')); return; }
        if (busWriteQueue.length >= MAX_BUS_WRITE_QUEUE_SIZE) {
            diag.busWritesRejected++;
            reject(new Error('Bus write queue is full'));
            return;
        }
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
        diag.busWriteQueueLength = busWriteQueue.length;
        kickBusTickIfIdle();
    });
}

function kickBusTickIfIdle() {
    if (!busTickLoopActive || workerShuttingDown || busTickInProgress) return;
    runBusTick();
}

// ===== Immediate Bus Commands =====
function clearAllServoPendingTransactions() {
    for (const servo of servos) {
        if (servo && typeof servo.clearPendingBusTransaction === 'function') servo.clearPendingBusTransaction();
    }
    if (endTool && typeof endTool.clearPendingBusTransaction === 'function') endTool.clearPendingBusTransaction();
}

function scheduleImmediateCommand(msg) {
    if (msg.command === 'moveJoint' && msg.joint !== undefined) {
        const key = String(msg.joint);
        const prev = pendingImmediateMoves[key];
        if (prev && prev.requestId !== undefined) {
            sendResponse(prev.clientId, prev.requestId, { type: 'error', message: 'Superseded by newer move for joint ' + msg.joint });
        }
        pendingImmediateMoves[key] = msg;
        drainImmediateMoveQueue();
        return;
    }
    runImmediateCommand(msg, false);
}

async function drainImmediateMoveQueue() {
    if (immediateMoveDrainRunning) return;
    immediateMoveDrainRunning = true;
    try {
        while (Object.keys(pendingImmediateMoves).length > 0) {
            const key = Object.keys(pendingImmediateMoves)[0];
            const item = pendingImmediateMoves[key];
            delete pendingImmediateMoves[key];
            const moreAfter = Object.keys(pendingImmediateMoves).length > 0;
            await runImmediateCommand(item, moreAfter);
        }
    } finally {
        immediateMoveDrainRunning = false;
    }
}

async function waitForBusTickToFinish() {
    const waitStart = Date.now();
    while (busTickInProgress && Date.now() - waitStart < 2000) {
        await new Promise(r => setTimeout(r, 50));
    }
}

async function runImmediateCommand(msg, moreMovesQueued) {
    await waitForBusTickToFinish();

    if (busTickTimer) { clearTimeout(busTickTimer); busTickTimer = null; }

    const isRapidJog = moreMovesQueued === true;
    const quietBefore = isRapidJog ? BUS_QUIET_JOG_MS : BUS_QUIET_BEFORE_MOVE_MS;
    const quietAfter  = isRapidJog ? BUS_QUIET_JOG_MS : BUS_QUIET_AFTER_MOVE_MS;

    const resumeLoop = busTickLoopActive;
    busTickLoopActive = false;
    busTickInProgress = true;
    const cmdStart = Date.now();

    try {
        clearAllServoPendingTransactions();
        await new Promise(r => setTimeout(r, quietBefore));
        await handleBusCommand(msg.clientId, msg);
        diag.immediateBusCommands++;
        await new Promise(r => setTimeout(r, quietAfter));
        if (DEBUG) log('Immediate bus command OK: ' + msg.command + ' (' + (Date.now() - cmdStart) + ' ms)');
    } catch (error) {
        const errMsg = error && error.message ? error.message : String(error);
        if (errMsg.toLowerCase().includes('timeout')) diag.writeTimeouts++;
        log('Immediate bus command failed: ' + msg.command + ' — ' + errMsg, true);
        sendResponse(msg.clientId, msg.requestId, { type: 'error', message: formatCommandError(error) });
    } finally {
        busTickInProgress = false;
        busTickLoopActive = resumeLoop;
        if (resumeLoop && !workerShuttingDown) scheduleNextBusTick(MIN_BUS_TICK_GAP_MS + BUS_QUIET_AFTER_MOVE_MS);
    }
}

// ===== Servo Initialization =====
async function initializeServos() {
    log('Initializing ST3215 servo controllers...');

    log(`Using serial port: ${SERIAL_PORT} @ ${SERIAL_BAUDRATE} baud`);
    sharedSerialPort = new SerialPort({ path: SERIAL_PORT, baudRate: SERIAL_BAUDRATE, dataBits: 8, parity: 'none', stopBits: 1, autoOpen: false });

    await new Promise((resolve, reject) => {
        sharedSerialPort.open((error) => {
            if (error) reject(error); else resolve();
        });
    });
    log('Shared serial port opened');

    sharedSerialPort.on('data', routeIncomingSerialData);
    sharedSerialPort.on('error', (error) => log('Serial port error: ' + error.message, true));
    sharedSerialPort._writeQueue = queueWrite;

    for (let i = 0; i < JOINT_COUNT; i++) {
        servos[i] = null;
        if (i > 0) await new Promise(r => setTimeout(r, 150));
        const servo = new RobotArm.ServoController(i + 1, sharedSerialPort, SERVO_IDS[i], SERIAL_BAUDRATE);
        try {
            await servo.open();
            servos[i] = servo;
            log(`Pinging servo ${i + 1} (ID: ${SERVO_IDS[i]})...`);
            const alive = await servo.ping();
            if (!alive) {
                log(`Servo ${i + 1} did not respond to ping — skipping`);
                servos[i] = null;
                continue;
            }
            log(`Servo ${i + 1} (ID: ${SERVO_IDS[i]}) OK`);
            await new Promise(r => setTimeout(r, 50));
            await servo.startServo();
        } catch (error) {
            log(`Failed to initialize servo ${i + 1}: ${error.message}`, true);
            servos[i] = null;
        }
    }

    log(`Initialized ${servos.filter(s => s !== null).length} of ${JOINT_COUNT} servos`);

    try {
        const tool = new RobotArm.EndToolController(sharedSerialPort, SERIAL_BAUDRATE);
        await tool.open();
        endTool = tool;
        const toolAlive = await endTool.pingTool();
        log(toolAlive ? 'End tool node (ID 64) OK' : 'End tool node did not respond at startup');
    } catch (error) {
        log('End tool init failed: ' + error.message, true);
        endTool = null;
    }

    refreshDataRoutingControllers();
    rebuildJointConfigsCache();
    cachedTorqueEnabled = true;
}

async function createAndInitializeServo(jointIndex) {
    const servoId = SERVO_IDS[jointIndex];
    const servo   = new RobotArm.ServoController(jointIndex + 1, sharedSerialPort, servoId, SERIAL_BAUDRATE);
    await servo.open();
    servos[jointIndex] = servo;
    const alive = await servo.ping();
    if (!alive) {
        servos[jointIndex] = null;
        throw new Error(`Servo ${jointIndex + 1} (ID: ${servoId}) did not respond`);
    }
    await servo.startServo();
    return servo;
}

async function resolveServoForJoint(jointIndex, jointNum) {
    if (servos[jointIndex] !== null && servos[jointIndex] !== undefined) return servos[jointIndex];
    try {
        clearAllServoPendingTransactions();
        await new Promise(r => setTimeout(r, 20));
        const servo = await createAndInitializeServo(jointIndex);
        servoConsecFails[jointIndex] = 0;
        servoBackoffUntilMs[jointIndex] = 0;
        refreshDataRoutingControllers();
        rebuildJointConfigsCache();
        log('Recovered servo for joint ' + jointNum);
        return servo;
    } catch (error) {
        log('Could not recover joint ' + jointNum + ': ' + error.message, true);
        return null;
    }
}

async function rescanServos() {
    const results = [];
    for (let i = 0; i < JOINT_COUNT; i++) {
        const jointNumber = i + 1;
        const servoId     = SERVO_IDS[i];
        const existing    = servos[i] || null;

        if (existing !== null) {
            try {
                const alive = await existing.isResponsive();
                if (alive) {
                    await existing.startServo();
                    results.push({ joint: jointNumber, servoId, available: true, action: 'kept_existing' });
                    continue;
                }
            } catch (e) { /* fall through to replacement */ }
            servos[i] = null;
        }

        try {
            await createAndInitializeServo(i);
            servoConsecFails[i] = 0;
            servoBackoffUntilMs[i] = 0;
            results.push({ joint: jointNumber, servoId, available: true, action: 'rediscovered' });
        } catch (e) {
            servos[i] = null;
            results.push({ joint: jointNumber, servoId, available: false, action: 'not_found', error: e.message });
        }

        if (i < JOINT_COUNT - 1) await new Promise(r => setTimeout(r, 100));
    }
    return results;
}

async function applyJointSpeedIfChanged(servo, jointIndex, speed) {
    if (!jointSettingsCache[jointIndex]) jointSettingsCache[jointIndex] = {};
    if (jointSettingsCache[jointIndex].speed === speed) return false;
    await servo.setSpeed(speed);
    jointSettingsCache[jointIndex].speed = speed;
    return true;
}

async function applyJointAccelerationIfChanged(servo, jointIndex, acc) {
    if (!jointSettingsCache[jointIndex]) jointSettingsCache[jointIndex] = {};
    if (jointSettingsCache[jointIndex].acceleration === acc) return false;
    await servo.setAcceleration(acc);
    jointSettingsCache[jointIndex].acceleration = acc;
    return true;
}

// ===== Bus Command Handler =====
async function handleBusCommand(clientId, data) {
    const command   = data.command;
    const requestId = data.requestId;

    const reply = (payload) => sendResponse(clientId, requestId, payload);

    const requireEndTool = () => {
        if (!endTool) throw new Error('End tool controller is not initialized');
        return endTool;
    };

    switch (command) {

        case 'rescanServos': {
            const now = Date.now();
            if (now - lastRescanTime < RESCAN_MIN_INTERVAL_MS) {
                reply({ type: 'error', message: 'Rescan rate limited — wait ' + Math.ceil((RESCAN_MIN_INTERVAL_MS - (now - lastRescanTime)) / 1000) + ' s' });
                return;
            }
            lastRescanTime = now;
            try {
                const results = await rescanServos();
                refreshDataRoutingControllers();
                rebuildJointConfigsCache();
                reply({ type: 'servoRescan', joints: results });
            } catch (error) {
                reply({ type: 'error', message: 'Failed to rescan servos: ' + error.message });
            }
            break;
        }

        case 'moveJoint': {
            const jointIndex = data.joint - 1;
            const angle      = data.angle;
            const moveSpeed  = (typeof data.speed === 'number' && !isNaN(data.speed) && data.speed >= 0) ? data.speed : 1500;

            if (jointIndex < 0 || jointIndex >= servos.length) {
                reply({ type: 'error', message: `Invalid joint number: ${data.joint}` });
                return;
            }
            const servo = await resolveServoForJoint(jointIndex, data.joint);
            if (!servo) {
                reply({ type: 'error', message: `Servo ${data.joint} is not available` });
                return;
            }
            try {
                const cached = jointStatusCache[jointIndex];
                if (!cached || cached.torqueEnabled !== true) await servo.startServo();
                await servo.moveToAngle(angle, moveSpeed);
                diag.busMovesCompleted++;
                await refreshSingleJointStatusFromBus(jointIndex);
                postStatusToMain();
                reply({ type: 'success', message: `Servo ${data.joint} moving to ${angle}° at ${moveSpeed} step/s` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to move joint ${data.joint}: ${formatCommandError(error)}`, servoFault: !!(error && error.isOverload), joint: data.joint });
            }
            break;
        }

        case 'stopJoint': {
            const idx = data.joint - 1;
            if (idx < 0 || idx >= servos.length) { reply({ type: 'error', message: `Invalid joint number: ${data.joint}` }); return; }
            const sv = servos[idx];
            if (!sv) { reply({ type: 'error', message: `Servo ${data.joint} is not available` }); return; }
            try {
                const stopped = await sv.stopServo();
                if (stopped) {
                    reply({ type: 'success', message: `Servo ${data.joint} stopped` });
                } else {
                    reply({ type: 'error', message: `Could not disable torque on joint ${data.joint}`, joint: data.joint });
                }
            } catch (error) {
                reply({ type: 'error', message: `Failed to stop joint ${data.joint}: ${formatCommandError(error)}`, joint: data.joint });
            }
            break;
        }

        case 'stopAll':
        case 'stopAllJoints': {
            try {
                const failed = [];
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        const ok = await servos[i].stopServo();
                        if (!ok) failed.push(i + 1);
                    }
                }
                if (failed.length === 0) {
                    reply({ type: 'success', message: 'All servos stopped' });
                } else {
                    reply({ type: 'error', message: 'Could not disable torque on joint(s): ' + failed.join(', ') });
                }
            } catch (error) {
                reply({ type: 'error', message: `Failed to stop all servos: ${formatCommandError(error)}` });
            }
            break;
        }

        case 'setServo':
        case 'setServoAngle': {
            const idx = data.joint - 1;
            if (idx < 0 || idx >= servos.length) { reply({ type: 'error', message: `Invalid joint number: ${data.joint}` }); return; }
            const sv = servos[idx];
            if (!sv) { reply({ type: 'error', message: `Servo ${data.joint} is not available` }); return; }
            try {
                await sv.moveToAngle(data.angle);
                reply({ type: 'success', message: `Servo ${data.joint} set to ${data.angle}°` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to set servo angle: ${error.message}` });
            }
            break;
        }

        case 'setSpeed': {
            const idx = data.joint - 1;
            if (idx < 0 || idx >= servos.length) { reply({ type: 'error', message: `Invalid joint number: ${data.joint}` }); return; }
            const sv = servos[idx];
            if (!sv) { reply({ type: 'error', message: `Servo ${data.joint} is not available` }); return; }
            try {
                const changed = await applyJointSpeedIfChanged(sv, idx, data.speed);
                reply({ type: 'success', message: changed ? `Servo ${data.joint} speed set to ${data.speed}` : `Servo ${data.joint} speed already ${data.speed}` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to set speed on joint ${data.joint}: ${formatCommandError(error)}`, servoFault: !!(error && error.isOverload), joint: data.joint });
            }
            break;
        }

        case 'setSpeedAll': {
            try {
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) await applyJointSpeedIfChanged(servos[i], i, data.speed);
                }
                reply({ type: 'success', message: `All servos speed set to ${data.speed}` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to set servo speeds: ${error.message}` });
            }
            break;
        }

        case 'setTorqueAll': {
            const torqueEnabled = data.enabled !== false;
            try {
                if (cachedTorqueEnabled === torqueEnabled) {
                    reply({ type: 'success', message: `All servos torque already ${torqueEnabled ? 'enabled' : 'disabled'}` });
                    return;
                }
                for (let i = 0; i < servos.length; i++) {
                    if (servos[i] !== null) {
                        if (torqueEnabled) await servos[i].startServo(); else await servos[i].stopServo();
                    }
                }
                cachedTorqueEnabled = torqueEnabled;
                reply({ type: 'success', message: `All servos torque ${torqueEnabled ? 'enabled' : 'disabled'}` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to set torque: ${error.message}` });
            }
            break;
        }

        case 'setAcceleration': {
            const idx = data.joint - 1;
            if (idx < 0 || idx >= servos.length) { reply({ type: 'error', message: `Invalid joint number: ${data.joint}` }); return; }
            const sv = servos[idx];
            if (!sv) { reply({ type: 'error', message: `Servo ${data.joint} is not available` }); return; }
            try {
                const changed = await applyJointAccelerationIfChanged(sv, idx, data.acceleration);
                reply({ type: 'success', message: changed ? `Servo ${data.joint} acceleration set to ${data.acceleration}` : `Servo ${data.joint} acceleration already ${data.acceleration}` });
            } catch (error) {
                reply({ type: 'error', message: `Failed to set acceleration: ${error.message}` });
            }
            break;
        }

        // ===== End Tool Commands =====
        case 'toolPing': {
            try { reply({ type: 'toolPing', ok: await requireEndTool().pingTool() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to ping tool: ' + e.message }); }
            break;
        }
        case 'toolGetIdentity': {
            try { reply({ type: 'toolIdentity', ...await requireEndTool().getToolIdentity() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool identity: ' + e.message }); }
            break;
        }
        case 'toolGetStatus': {
            try { reply({ type: 'toolStatus', ...await requireEndTool().getToolStatus() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool status: ' + e.message }); }
            break;
        }
        case 'toolSetPwm': {
            try {
                await requireEndTool().setPwmOutputs(
                    Number.isFinite(data.pwm1Duty) ? data.pwm1Duty : 0,
                    Number.isFinite(data.pwm2Duty) ? data.pwm2Duty : 0,
                    data.enable1 !== false, data.enable2 !== false);
                reply({ type: 'success', message: 'Tool PWM outputs updated' });
            } catch (e) { reply({ type: 'error', message: 'Failed to set tool PWM: ' + e.message }); }
            break;
        }
        case 'toolGetPwmState': {
            try { reply({ type: 'toolPwmState', ...await requireEndTool().getPwmState() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool PWM state: ' + e.message }); }
            break;
        }
        case 'toolReadCurrents': {
            try { reply({ type: 'toolCurrents', ...await requireEndTool().readPwmCurrents() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool currents: ' + e.message }); }
            break;
        }
        case 'toolReadAdc': {
            try { reply({ type: 'toolAdc', ...await requireEndTool().readAdcData() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool ADC data: ' + e.message }); }
            break;
        }
        case 'toolSetServoEnabled': {
            try {
                await requireEndTool().setHobbyServoEnabled(data.enabled !== false);
                reply({ type: 'success', message: `Tool hobby servo ${data.enabled !== false ? 'enabled' : 'disabled'}` });
            } catch (e) { reply({ type: 'error', message: 'Failed to set tool servo enable: ' + e.message }); }
            break;
        }
        case 'toolSetServoPosition': {
            try {
                await requireEndTool().setHobbyServoPosition(Number.isFinite(data.position) ? data.position : 0);
                reply({ type: 'success', message: `Tool hobby servo position set to ${data.position}` });
            } catch (e) { reply({ type: 'error', message: 'Failed to set tool servo position: ' + e.message }); }
            break;
        }
        case 'toolSetServoAngle': {
            try {
                await requireEndTool().setHobbyServoAngle(Number.isFinite(data.angle) ? data.angle : 0);
                reply({ type: 'success', message: `Tool hobby servo angle set to ${data.angle}` });
            } catch (e) { reply({ type: 'error', message: 'Failed to set tool servo angle: ' + e.message }); }
            break;
        }
        case 'toolGetServoState': {
            try { reply({ type: 'toolServoState', ...await requireEndTool().getHobbyServoState() }); }
            catch (e) { reply({ type: 'error', message: 'Failed to read tool servo state: ' + e.message }); }
            break;
        }
        case 'toolSetWatchdog': {
            try {
                await requireEndTool().setWatchdogTimeout(Number.isFinite(data.timeoutMs) ? data.timeoutMs : 0);
                reply({ type: 'success', message: `Tool watchdog set to ${data.timeoutMs} ms` });
            } catch (e) { reply({ type: 'error', message: 'Failed to set tool watchdog: ' + e.message }); }
            break;
        }
        case 'toolClearFaults': {
            try { await requireEndTool().clearToolFaults(); reply({ type: 'success', message: 'Tool faults cleared' }); }
            catch (e) { reply({ type: 'error', message: 'Failed to clear tool faults: ' + e.message }); }
            break;
        }
        case 'toolReset': {
            try { await requireEndTool().resetTool(); reply({ type: 'success', message: 'Tool reset command sent' }); }
            catch (e) { reply({ type: 'error', message: 'Failed to reset tool: ' + e.message }); }
            break;
        }

        default:
            reply({ type: 'error', message: `Unknown bus command: ${command}` });
    }
}

// ===== Shutdown =====
function abortPendingBusWork() {
    workerShuttingDown = true;
    stopBusTickLoop();
    while (busWriteQueue.length > 0) busWriteQueue.shift().reject(new Error('Worker shutting down'));
    while (writeQueue.length > 0) writeQueue.shift().reject(new Error('Worker shutting down'));
    isWriting = false;
    clearAllServoPendingTransactions();
}

async function closeSerialPort() {
    if (!sharedSerialPort || !sharedSerialPort.isOpen) return;
    await Promise.race([
        new Promise(resolve => sharedSerialPort.close(err => { if (err) log('Serial close error: ' + err.message, true); resolve(); })),
        new Promise(resolve => setTimeout(resolve, 2000))
    ]);
}

async function shutdown() {
    log('Servo worker shutting down...');
    abortPendingBusWork();
    await waitForBusTickToFinish();
    await closeSerialPort();
    log('Servo worker shutdown complete');
    process.exit(0);
}

// ===== Message Handler (Main → Worker) =====
process.on('message', (msg) => {
    if (!msg || !msg.type) return;

    if (msg.type === 'shutdown') {
        shutdown();
        return;
    }

    if (msg.type === 'immediateBusCommand') {
        scheduleImmediateCommand(msg);
        return;
    }

    if (msg.type === 'busCommand') {
        enqueueBusWrite(async () => {
            await handleBusCommand(msg.clientId, msg);
        }, { command: msg.command, joint: msg.joint }).catch((error) => {
            const errMsg = error && error.message ? error.message : String(error);
            if (!errMsg.includes('Superseded')) {
                log('Bus command failed (' + msg.command + '): ' + errMsg, true);
            }
            sendResponse(msg.clientId, msg.requestId, { type: 'error', message: errMsg });
        });
        return;
    }
});

// ===== Start =====
initializeServos().then(async () => {
    await refreshJointStatusCacheFromBus();
    startBusTickLoop();
    process.send({ type: 'ready', jointConfigs: getJointConfigsSnapshot() });
    log('Servo worker ready');
}).catch((error) => {
    log('Servo worker init failed: ' + (error.message || error), true);
    process.send({ type: 'initError', message: error.message || String(error) });
    process.exit(1);
});
