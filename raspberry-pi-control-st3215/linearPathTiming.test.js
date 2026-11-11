'use strict';
// Offline tests for the linear-path timing maths, the one-packet servo write, the
// status decoding and the reply parser. No arm or serial port needed.
// Run: node linearPathTiming.test.js
const assert = require('assert');
// No port is opened here, so a stub will do where serialport is not installed (e.g. a dev PC)
const Module = require('module');
const realLoad = Module._load;
Module._load = function (request, ...rest) {
    if (request === 'serialport') {
        try { return realLoad.call(this, request, ...rest); } catch (_) { return { SerialPort: class {} }; }
    }
    return realLoad.call(this, request, ...rest);
};
const T = require('./linearPathTiming');
const { ServoController } = require('./robotArmST3215');

const tests = [];
function test(name, fn) { tests.push({ name, fn }); }

// ---- helpers ------------------------------------------------------------------------------
function checksum(bytes) { return (~bytes.reduce((a, b) => a + b, 0)) & 0xFF; }
/** A reply packet as the servo sends it: FF FF id len err params... cs */
function reply(id, params, err = 0) {
    const body = [id, params.length + 2, err, ...params];
    return Buffer.from([0xFF, 0xFF, ...body, checksum(body)]);
}
/** A controller on a fake shared port; respond(packet) returns the bytes to feed back (or null). */
function fakeServo(id, respond) {
    const port = {
        writes: [],
        write(packet, cb) {
            this.writes.push(Buffer.from(packet));
            cb(null);
            const rx = respond ? respond(packet, this.writes.length) : null;
            if (rx) (Array.isArray(rx) ? rx : [rx]).forEach((chunk, i) => setTimeout(() => servo.handleIncomingData(chunk), 2 + i * 15));
        }
    };
    const servo = new ServoController(id, port, id);
    return servo;
}
function statusBlock({ pos = 2048, speed = 0, load = 0, volt = 120, temp = 30, moving = 0, torque = 1 } = {}) {
    const b = new Array(27).fill(0);
    b[0] = torque;
    b[16] = pos & 0xFF; b[17] = (pos >> 8) & 0xFF;      // 56-57
    b[18] = speed & 0xFF; b[19] = (speed >> 8) & 0xFF;  // 58-59
    b[20] = load & 0xFF; b[21] = (load >> 8) & 0xFF;    // 60-61
    b[22] = volt; b[23] = temp; b[26] = moving;         // 62, 63, 66
    return b;
}

// ---- planWaypointTiming --------------------------------------------------------------------
test('steps that already fit keep the nominal interval', () => {
    const p = T.planWaypointTiming([0, 0, 0, 0, 0, 0], [[1, 0, 0, 0, 0, 0], [2, 0, 0, 0, 0, 0], [3, 0, 0, 0, 0, 0]], 33, { maxJointDegPerSec: 110 });
    assert.deepStrictEqual(p.dtMs, [33, 33, 33]);
    assert.strictEqual(p.slowedSteps, 0);
});

test('only the step with a big joint jump is stretched', () => {
    const p = T.planWaypointTiming([0, 0, 0, 0, 0, 0], [[1, 0, 0, 0, 0, 0], [1, 0, 22, 0, 0, 0], [2, 0, 22, 0, 0, 0]], 33, { maxJointDegPerSec: 110 });
    assert.deepStrictEqual(p.dtMs, [33, 200, 33]);       // ceil(22 / 110 * 1000) = 200
    assert.strictEqual(p.slowedSteps, 1);
    assert.ok(p.peakJointDegPerSec <= 110.01);
    assert.strictEqual(p.totalMs, 266);
});

// ---- jointSpeedStepsPerSec -----------------------------------------------------------------
test('speed follows the real distance over the real time', () => {
    const oneStep = T.jointSpeedStepsPerSec(1, 33, 1.25);
    assert.ok(Math.abs(T.jointSpeedStepsPerSec(4, 33, 1.25) / oneStep - 4) < 0.1);
    assert.ok(Math.abs(T.jointSpeedStepsPerSec(4, 132, 1.25) - oneStep) <= 1);
});

test('speed is clamped to the servo range', () => {
    assert.strictEqual(T.jointSpeedStepsPerSec(0, 33, 1.25), 50);
    assert.strictEqual(T.jointSpeedStepsPerSec(80, 20, 1.25), 3400);
});

// ---- settleSpeeds --------------------------------------------------------------------------
test('settle brings every joint in together', () => {
    const remaining = [0.4, 41, 19, 0, 52, 1];
    const { seconds, speeds } = T.settleSpeeds(remaining, { settleDegPerSec: 60 });
    for (let j = 0; j < remaining.length; j++) {
        if (remaining[j] < 5) continue;                 // tiny ones sit on the 50 steps/s floor
        const t = (remaining[j] * T.STEPS_PER_DEG) / speeds[j];
        assert.ok(Math.abs(t - seconds) < 0.05 * seconds + 0.02, `joint ${j + 1} would take ${t.toFixed(2)} s, expected ${seconds.toFixed(2)} s`);
    }
    assert.strictEqual(Math.max(...speeds), speeds[4]);
});

// ---- the one-packet servo write ------------------------------------------------------------
test('moveToAngle writes position, time and speed in ONE packet (registers 42-47)', async () => {
    const calls = [];
    const servo = Object.create(ServoController.prototype);
    servo.servoId = 5; servo.currentSpeed = 1500; servo.zeroTrimSteps = 0;
    servo.writeData = async (address, data) => { calls.push({ address, data }); return true; };
    servo.logServoCommandError = () => {};
    await servo.moveToAngle(30, 300);
    assert.strictEqual(calls.length, 1);
    assert.strictEqual(calls[0].address, 42);
    assert.strictEqual(calls[0].data[0] | (calls[0].data[1] << 8), servo.angleToSteps(30));
    assert.deepStrictEqual(calls[0].data.slice(2, 4), [0, 0]);
    assert.strictEqual(calls[0].data[4] | (calls[0].data[5] << 8), 300);
    assert.strictEqual(servo.currentSpeed, 300);
});

test('a failed move is an error and the speed is not recorded as applied', async () => {
    const servo = Object.create(ServoController.prototype);
    servo.servoId = 3; servo.currentSpeed = 1500; servo.zeroTrimSteps = 0;
    servo.writeData = async () => { throw new Error('Write timeout'); };
    servo.logServoCommandError = () => {};
    await assert.rejects(() => servo.moveToAngle(10, 400), /Write timeout/);
    assert.strictEqual(servo.currentSpeed, 1500);
});

test('speed is clamped and rounded before it is written', async () => {
    const calls = [];
    const servo = Object.create(ServoController.prototype);
    servo.servoId = 1; servo.currentSpeed = 100; servo.zeroTrimSteps = 0;
    servo.writeData = async (a, d) => { calls.push(d); return true; };
    servo.logServoCommandError = () => {};
    await servo.moveToAngle(0, 99999);
    await servo.moveToAngle(0, 123.6);
    assert.strictEqual(calls[0][4] | (calls[0][5] << 8), 3400);
    assert.strictEqual(calls[1][4] | (calls[1][5] << 8), 124);
});

// ---- status decoding -----------------------------------------------------------------------
test('load uses both bytes and its direction bit (was low byte only)', () => {
    const s = Object.create(ServoController.prototype); s.zeroTrimSteps = 0;
    const st = s.quickStatusFromBuffer(statusBlock({ load: 300 }));           // 30.0 %
    assert.strictEqual(st.load, 30);
    const neg = s.quickStatusFromBuffer(statusBlock({ load: 0x400 | 655 }));   // -65.5 %
    assert.strictEqual(neg.load, 65.5);
    assert.strictEqual(neg.loadSigned, -65.5);
});

test('speed decodes its direction bit (negative used to read as 32768 + speed)', () => {
    const s = Object.create(ServoController.prototype); s.zeroTrimSteps = 0;
    const st = s.quickStatusFromBuffer(statusBlock({ speed: 0x8000 | 400 }));
    assert.strictEqual(st.speed, 400);
    assert.strictEqual(st.speedSigned, -400);
});

// ---- reply parser --------------------------------------------------------------------------
test('a stray byte before the reply no longer loses it', async () => {
    const block = statusBlock({ pos: 2500, temp: 41 });
    const servo = fakeServo(2, () => Buffer.concat([Buffer.from([0x00]), reply(2, block)]));
    const st = await servo.readQuickStatus();
    assert.strictEqual(st.position, 2500);
    assert.strictEqual(st.temperature, 41);
});

test('a leading FF before the header is skipped', async () => {
    const servo = fakeServo(2, () => Buffer.concat([Buffer.from([0xFF]), reply(2, statusBlock({ pos: 1800 }))]));
    assert.strictEqual((await servo.readQuickStatus()).position, 1800);
});

test('a corrupted packet is skipped and the good one behind it is used', async () => {
    const bad = reply(2, statusBlock({ pos: 3000 })); bad[10] ^= 0x55;     // checksum now wrong
    const good = reply(2, statusBlock({ pos: 2100 }));
    const servo = fakeServo(2, () => Buffer.concat([bad, good]));
    assert.strictEqual((await servo.readQuickStatus()).position, 2100);
});

test('a reply split across chunks is reassembled', async () => {
    const pkt = reply(4, statusBlock({ pos: 2222 }));
    const servo = fakeServo(4, () => [pkt.subarray(0, 7), pkt.subarray(7, 20), pkt.subarray(20)]);
    assert.strictEqual((await servo.readQuickStatus()).position, 2222);
});

test('a late reply of the wrong length is not taken as this read', async () => {
    // A stale 2-byte reply (e.g. a timed-out position read) arrives first, then the real one
    const servo = fakeServo(3, () => [reply(3, [0x01, 0x00]), reply(3, statusBlock({ pos: 2600, temp: 35 }))]);
    const st = await servo.readQuickStatus();
    assert.strictEqual(st.position, 2600);
    assert.strictEqual(st.temperature, 35);
});

test('a late read reply is not taken as a write ack', async () => {
    let acked = false;
    const servo = fakeServo(5, () => [reply(5, statusBlock()), reply(5, [])]);
    servo.writeData(42, [0, 8, 0, 0, 100, 0]).then(() => { acked = true; });
    await new Promise(r => setTimeout(r, 9));      // only the stale read reply has arrived
    assert.strictEqual(acked, false);
    await new Promise(r => setTimeout(r, 30));     // now the real ack
    assert.strictEqual(acked, true);
});

(async () => {
    let passed = 0;
    for (const t of tests) {
        try { await t.fn(); passed++; console.log('ok   - ' + t.name); }
        catch (e) { console.log('FAIL - ' + t.name + '\n       ' + e.message); process.exitCode = 1; }
    }
    console.log(`\n${passed} of ${tests.length} tests passed`);
})();
