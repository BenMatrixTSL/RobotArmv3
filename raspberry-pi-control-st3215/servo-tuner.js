'use strict';
/**
 * servo-tuner.js — Automated PID tuner for ST3215 servo joints.
 *
 * Sweeps P and D gain values for each joint, measures position tracking error
 * on step-response moves, and writes the best combination to EEPROM.
 * Results are saved to servo-pid-config.json for auto-apply at startup.
 *
 * BEFORE RUNNING:
 *   sudo systemctl stop st3215-server.service
 *   node servo-tuner.js
 *   sudo systemctl start st3215-server.service
 *
 * The arm must be in a safe resting position (near home / all joints ~0°).
 * Only one joint moves at a time; all others stay at whatever angle they're at.
 */

const { ServoController } = require('./robotArmST3215');
const { SerialPort }       = require('serialport');
const fs                   = require('fs');
const path                 = require('path');

const SERIAL_PORT  = process.env.SERIAL_PORT  || '/dev/serial0';
const BAUD_RATE    = 1_000_000;
const CONFIG_PATH  = path.join(__dirname, 'servo-pid-config.json');

// Same state directory servoWorker.js uses for per-joint recentering
// (see setJointCenter). A joint that's had its servo swapped may have its
// 0° redefined well away from the factory 2048 — homing it to raw 2048
// here would ignore that and could drive it toward a position that's no
// longer mechanically safe for the new servo's mounting.
const CENTER_OVERRIDES_PATH = process.env.ROBOT_ARM_STATE_DIR
    ? path.join(process.env.ROBOT_ARM_STATE_DIR, 'servo-joint-centers.json')
    : '/var/lib/robot-arm-st3215/servo-joint-centers.json';

function loadCenterOverrides() {
    if (!fs.existsSync(CENTER_OVERRIDES_PATH)) return {};
    try {
        const cfg = JSON.parse(fs.readFileSync(CENTER_OVERRIDES_PATH, 'utf8'));
        return (cfg && cfg.joints) || {};
    } catch (e) {
        console.warn(`Could not read ${CENTER_OVERRIDES_PATH}: ${e.message} — using factory center (2048) for all joints`);
        return {};
    }
}

// ── EEPROM register addresses ──────────────────────────────────────────────
const REG_EEPROM_LOCK    = 0x37;  // 0 = unlocked
const REG_MAX_TORQUE_L   = 0x10;  // 2-byte word, 0–1000
const REG_UNLOADING_COND = 0x13;  // overload-protection flags
const REG_P_COEF         = 0x15;  // position P gain  (byte, default 32)
const REG_D_COEF         = 0x16;  // position D gain  (byte, default 32)
const REG_I_COEF         = 0x17;  // position I gain  (byte, default  0)
const REG_MIN_STARTUP    = 0x18;  // min startup force(byte, default 16)
const REG_PROT_TORQUE    = 0x21;  // protection torque% after overload

// ── Tuning parameters ──────────────────────────────────────────────────────
const P_CANDIDATES  = [24, 32, 40, 48, 64, 80];
const D_CANDIDATES  = [16, 24, 32, 48, 64];
const I_CANDIDATES  = [0, 1, 2, 4, 8];
const TEST_SPEED    = 500;   // steps/s (~44 °/s) — moderate speed for testing
const SETTLE_POLL_MS   = 60;   // ms between position samples during stability check
const SETTLE_THRESHOLD = 2;    // steps — position must change less than this to be "settled"
const SETTLE_STABLE_NEEDED = 12; // consecutive stable readings required (~720ms of true stillness)
const SETTLE_TIMEOUT_MS = 8000; // max wait before giving up
// Extra pause AFTER waitSettle() reports stable, before the next bus command.
// waitSettle() only confirms the *position sensor* has stopped changing —
// the arm can still be mechanically vibrating/oscillating for a bit after
// that, and issuing the next command into that residual motion was causing
// write timeouts. Give it real time to fully come to rest.
const POST_SETTLE_DWELL_MS = 1200;

// Per-joint test: move this many degrees from current position, then back.
// Signs are chosen to be safe from ~0° home for each joint's limits.
const JOINT_CONFIG = {
    1: { label: 'J1 base yaw',       testOffsetDeg:  15 },
    2: { label: 'J2 shoulder pitch', testOffsetDeg: -15 },
    3: { label: 'J3 elbow pitch',    testOffsetDeg:  15 },
    4: { label: 'J4 wrist roll 1',   testOffsetDeg:  15 },
    5: { label: 'J5 wrist pitch',    testOffsetDeg:  15 },
    6: { label: 'J6 wrist roll 2',   testOffsetDeg:  15 },
};

// ── Helpers ────────────────────────────────────────────────────────────────
// Angle conversion uses each controller's own stepsToAngle()/angleToSteps()
// (per-instance, center-aware — see robotArmST3215.js) rather than a
// standalone helper, so a recentered joint's console output reads correctly
// relative to its actual saved 0°, not always the factory 2048.
const sleep = ms => new Promise(r => setTimeout(r, ms));

// servoWorker.js (the production bus owner) never issues a move without first
// clearing any pending bus transaction on that controller and giving the bus
// a brief quiet moment first (BUS_QUIET_BEFORE_MOVE_MS) — this script called
// ctrl.moveToPosition() directly with neither, and that gap is the most
// likely explanation for a servo that's reliable through the live app
// failing bus writes repeatedly here. Route every move through this instead.
const QUIET_BEFORE_MOVE_MS = 12; // matches servoWorker.js's BUS_QUIET_BEFORE_MOVE_MS
async function moveTo(ctrl, steps) {
    if (typeof ctrl.clearPendingBusTransaction === 'function') ctrl.clearPendingBusTransaction();
    await sleep(QUIET_BEFORE_MOVE_MS);
    await ctrl.moveToPosition(Math.round(steps));
}

async function avgPosition(ctrl, samples = 3) {
    let sum = 0;
    for (let n = 0; n < samples; n++) {
        sum += await ctrl.getPosition();
        if (n < samples - 1) await sleep(15);
    }
    return sum / samples;
}

// Wait until position stops changing for SETTLE_STABLE_NEEDED consecutive reads.
// Much more reliable than isMoving() which clears before mechanical settling.
async function waitSettle(ctrl) {
    let stableCount = 0;
    let lastPos = await ctrl.getPosition().catch(() => -1);
    const deadline = Date.now() + SETTLE_TIMEOUT_MS;
    while (Date.now() < deadline) {
        await sleep(SETTLE_POLL_MS);
        const pos = await ctrl.getPosition().catch(() => lastPos);
        if (Math.abs(pos - lastPos) <= SETTLE_THRESHOLD) {
            stableCount++;
            if (stableCount >= SETTLE_STABLE_NEEDED) return;
        } else {
            stableCount = 0;
        }
        lastPos = pos;
    }
    // Timed out — proceed anyway but add a safety pause
    await sleep(200);
}

async function unlockAndWritePID(ctrl, p, d, i, startup) {
    await sleep(200);  // ensure servo is fully settled before touching EEPROM
    await ctrl.writeData(REG_EEPROM_LOCK, [0]);
    await sleep(50);
    await ctrl.writeData(REG_P_COEF, [p, d, i, startup]);
    await sleep(100); // give EEPROM write time to complete
}

/**
 * Move to targetSteps and measure the final steady-state error after a longer dwell.
 * Used for I-sweep: waits for the arm to truly stop (not just the position
 * sensor) so integral has time to drive out residual error.
 * Returns absolute error in degrees at the settled position.
 */
async function settledError(ctrl, homeSteps, targetSteps) {
    await ctrl.setSpeed(TEST_SPEED);
    await moveTo(ctrl, targetSteps);
    await waitSettle(ctrl);
    await sleep(POST_SETTLE_DWELL_MS);
    const atTarget = await avgPosition(ctrl, 5);
    const fwdErr   = Math.abs(ctrl.stepsToAngle(atTarget) - ctrl.stepsToAngle(targetSteps));

    await ctrl.setSpeed(TEST_SPEED);
    await moveTo(ctrl, homeSteps);
    await waitSettle(ctrl);
    await sleep(POST_SETTLE_DWELL_MS);
    const atHome   = await avgPosition(ctrl, 5);
    const retErr   = Math.abs(ctrl.stepsToAngle(atHome) - ctrl.stepsToAngle(homeSteps));

    return { fwdErr, retErr, score: fwdErr + retErr * 0.5 };
}

/**
 * Move to targetSteps, settle, read actual; return to homeSteps, settle, read actual.
 * Returns { forwardErr, returnErr, score } all in degrees.
 */
async function stepResponse(ctrl, homeSteps, targetSteps) {
    await ctrl.setSpeed(TEST_SPEED);
    await moveTo(ctrl, targetSteps);
    await waitSettle(ctrl);
    await sleep(POST_SETTLE_DWELL_MS);
    const atTarget  = await avgPosition(ctrl);
    const forwardErr = Math.abs(ctrl.stepsToAngle(atTarget) - ctrl.stepsToAngle(targetSteps));

    await ctrl.setSpeed(TEST_SPEED);
    await moveTo(ctrl, homeSteps);
    await waitSettle(ctrl);
    await sleep(POST_SETTLE_DWELL_MS);
    const atHome   = await avgPosition(ctrl);
    const returnErr = Math.abs(ctrl.stepsToAngle(atHome) - ctrl.stepsToAngle(homeSteps));

    // Score: forward tracking weighted higher than return (gravity affects one direction)
    return { forwardErr, returnErr, score: forwardErr + returnErr * 0.5 };
}

// ── Per-joint tuning ───────────────────────────────────────────────────────
async function tuneJoint(ctrl, jointId) {
    const cfg = JOINT_CONFIG[jointId];
    console.log(`\n${'─'.repeat(56)}`);
    console.log(`  ${cfg.label}  (servo ID ${jointId})`);
    console.log('─'.repeat(56));

    // ── Read diagnostics ──
    const maxTorqueRaw  = await ctrl.readData(REG_MAX_TORQUE_L, 2);
    const maxTorque     = maxTorqueRaw[0] | (maxTorqueRaw[1] << 8);
    const unloading     = (await ctrl.readData(REG_UNLOADING_COND, 1))[0];
    const protTorque    = (await ctrl.readData(REG_PROT_TORQUE, 1))[0];
    console.log(`  Diagnostics: MaxTorque=${maxTorque}/1000  UnloadingCond=0b${unloading.toString(2).padStart(8,'0')}  ProtTorque=${protTorque}%`);
    if (unloading & 0x08) console.log('  ⚠  Stall-detection bit set in UnloadingCond — servo may cut torque when stalled');

    // ── Read current PID ──
    const pidData = await ctrl.readData(REG_P_COEF, 4);
    const [origP, origD, origI, origStartup] = pidData;
    console.log(`  Current PID: P=${origP}  D=${origD}  I=${origI}  MinStartup=${origStartup}`);

    // ── Capture home ──
    const homeSteps  = await avgPosition(ctrl);
    const homeDeg    = ctrl.stepsToAngle(homeSteps);
    const targetDeg  = homeDeg + cfg.testOffsetDeg;
    const targetSteps = ctrl.angleToSteps(targetDeg);
    console.log(`  Home: ${homeDeg.toFixed(2)}°   Target: ${targetDeg.toFixed(2)}°   (offset ${cfg.testOffsetDeg > 0 ? '+' : ''}${cfg.testOffsetDeg}°)`);

    // ── Baseline with current settings ──
    const baseline = await stepResponse(ctrl, homeSteps, targetSteps);
    console.log(`\n  Baseline (P=${origP} D=${origD}): fwd=${baseline.forwardErr.toFixed(3)}°  ret=${baseline.returnErr.toFixed(3)}°  score=${baseline.score.toFixed(3)}`);

    let bestP = origP, bestD = origD, bestScore = baseline.score;

    // ── Sweep P (D held at original) ──
    console.log(`\n  P sweep (D=${origD} fixed):`);
    for (const p of P_CANDIDATES) {
        if (p === origP) {
            console.log(`    P=${p.toString().padStart(2)}: (baseline — already measured)`);
            continue;
        }
        await unlockAndWritePID(ctrl, p, origD, origI, origStartup);
        const r = await stepResponse(ctrl, homeSteps, targetSteps);
        const flag = r.score < bestScore ? ' ←best' : '';
        console.log(`    P=${p.toString().padStart(2)}: fwd=${r.forwardErr.toFixed(3)}°  ret=${r.returnErr.toFixed(3)}°  score=${r.score.toFixed(3)}${flag}`);
        if (r.score < bestScore) { bestScore = r.score; bestP = p; }
    }

    // ── Sweep D (best P held fixed) ──
    console.log(`\n  D sweep (P=${bestP} fixed):`);
    for (const d of D_CANDIDATES) {
        if (d === origD && bestP === origP) {
            console.log(`    D=${d.toString().padStart(2)}: (baseline — already measured)`);
            continue;
        }
        await unlockAndWritePID(ctrl, bestP, d, origI, origStartup);
        const r = await stepResponse(ctrl, homeSteps, targetSteps);
        const flag = r.score < bestScore ? ' ←best' : '';
        console.log(`    D=${d.toString().padStart(2)}: fwd=${r.forwardErr.toFixed(3)}°  ret=${r.returnErr.toFixed(3)}°  score=${r.score.toFixed(3)}${flag}`);
        if (r.score < bestScore) { bestScore = r.score; bestD = d; }
    }

    // ── Detect if EEPROM writes had no effect (all scores identical) ──
    const noEffect = Math.abs(bestScore - baseline.score) < 0.05;
    if (noEffect) {
        console.log('\n  ⚠  No score variation detected — EEPROM writes may not take effect immediately.');
        console.log('     Keeping original values. A power-cycle may be needed after writing.');
        await unlockAndWritePID(ctrl, origP, origD, origI, origStartup);
        bestP = origP; bestD = origD;
    }

    // ── Sweep I (best P+D fixed) — measures steady-state settled error ──
    // Baseline for I sweep uses the settled-error metric, not the motion score.
    console.log(`\n  I sweep (P=${bestP} D=${bestD} fixed) — steady-state dwell test:`);
    const iBase = await settledError(ctrl, homeSteps, targetSteps);
    console.log(`    I= 0: fwd=${iBase.fwdErr.toFixed(3)}°  ret=${iBase.retErr.toFixed(3)}°  score=${iBase.score.toFixed(3)}  (baseline)`);
    let bestI = 0, bestIScore = iBase.score;

    for (const iVal of I_CANDIDATES) {
        if (iVal === 0) continue;  // already measured above
        await unlockAndWritePID(ctrl, bestP, bestD, iVal, origStartup);
        const r = await settledError(ctrl, homeSteps, targetSteps);
        const flag = r.score < bestIScore ? ' ←best' : '';
        console.log(`    I= ${iVal}: fwd=${r.fwdErr.toFixed(3)}°  ret=${r.retErr.toFixed(3)}°  score=${r.score.toFixed(3)}${flag}`);
        if (r.score < bestIScore) { bestIScore = r.score; bestI = iVal; }
    }

    const iImprove = iBase.score > 0 ? ((iBase.score - bestIScore) / iBase.score * 100) : 0;
    console.log(`  I result: 0→${bestI}  steady-state score: ${iBase.score.toFixed(3)}→${bestIScore.toFixed(3)}  (${iImprove.toFixed(1)}% improvement)`);
    if (bestI > 0 && iBase.score - bestIScore < 0.05) {
        console.log('  ⚠  I improvement negligible (<0.05°) — keeping I=0 to avoid windup risk');
        bestI = 0;
    }

    // ── Write final best values ──
    await unlockAndWritePID(ctrl, bestP, bestD, bestI, origStartup);
    await sleep(50);

    const improvement = baseline.score > 0 ? ((baseline.score - bestScore) / baseline.score * 100) : 0;
    console.log(`\n  Result: P=${origP}→${bestP}  D=${origD}→${bestD}  I=${origI}→${bestI}  score: ${baseline.score.toFixed(3)}→${bestScore.toFixed(3)}  (${improvement.toFixed(1)}% improvement)`);

    return {
        p: bestP, d: bestD, i: bestI, minStartupForce: origStartup,
        diagnostics: { maxTorque, unloadingCond: unloading, protTorquePct: protTorque },
    };
}

// ── Main ───────────────────────────────────────────────────────────────────
async function main() {
    console.log('═'.repeat(56));
    console.log('  ST3215 Servo PID Tuner');
    console.log('═'.repeat(56));
    console.log(`Port: ${SERIAL_PORT}  Baud: ${BAUD_RATE}`);
    console.log('Ensure st3215-server.service is STOPPED and the arm is in a safe position.\n');

    const port = new SerialPort({
        path: SERIAL_PORT, baudRate: BAUD_RATE,
        dataBits: 8, parity: 'none', stopBits: 1, autoOpen: false,
    });
    await new Promise((res, rej) => port.open(e => e ? rej(e) : res()));
    console.log('Serial port open.\n');

    // Create all controllers
    const centerOverrides = loadCenterOverrides();
    const homeStepsFor = {};
    const controllers = {};
    for (const idStr of Object.keys(JOINT_CONFIG)) {
        const id   = parseInt(idStr);
        const ctrl = new ServoController(id, port, id, BAUD_RATE);
        await ctrl.open();
        controllers[id] = ctrl;

        const override = centerOverrides[idStr];
        if (Number.isFinite(override)) {
            ctrl.setCenterPosition(override);
            homeStepsFor[id] = override;
            console.log(`  J${id}: using saved center ${override} (not factory 2048) — see setJointCenter in the app`);
        } else {
            homeStepsFor[id] = 2048;
        }
    }

    // Route all bus data to every controller — each filters its own ID
    port.on('data', data => {
        for (const c of Object.values(controllers)) c.handleIncomingData(data);
    });

    await sleep(500);

    // ── Home all joints to 0° before tuning ───────────────────────────────
    // "0°" per-joint: the saved center override if this joint has one,
    // otherwise the factory 2048 — see homeStepsFor above.
    console.log('Homing all joints to 0° — please stand clear...');
    const HOME_SPEED = 300;   // slow, safe homing speed (~26 °/s)
    for (const idStr of Object.keys(JOINT_CONFIG)) {
        const id   = parseInt(idStr);
        const ctrl = controllers[id];
        const alive = await ctrl.ping().catch(() => false);
        if (!alive) { console.log(`  J${id}: no response — skipping home`); continue; }
        await ctrl.startServo();
        await ctrl.setSpeed(HOME_SPEED);
        await moveTo(ctrl, homeStepsFor[id]);
        console.log(`  J${id}: moving to 0°`);
    }
    // Wait for all joints to fully settle at home
    console.log('Waiting for all joints to reach home...');
    await sleep(2000);
    for (const idStr of Object.keys(JOINT_CONFIG)) {
        const ctrl = controllers[parseInt(idStr)];
        await waitSettle(ctrl).catch(() => {});
        await sleep(POST_SETTLE_DWELL_MS);
        console.log(`  J${idStr} settled`);
    }
    console.log('All joints at home. Starting tuning sweep.\n');

    // Optional: node servo-tuner.js --joint 1,3  to tune specific joints only
    const jointArg = process.argv.find(a => a.startsWith('--joint=') || a.startsWith('--joint'));
    const jointFilter = jointArg
        ? (jointArg.includes('=') ? jointArg.split('=')[1] : process.argv[process.argv.indexOf(jointArg) + 1])
              .split(',').map(s => s.trim())
        : null;

    const results = {};

    for (const idStr of Object.keys(JOINT_CONFIG)) {
        const id   = parseInt(idStr);
        const ctrl = controllers[id];

        if (jointFilter && !jointFilter.includes(idStr)) {
            console.log(`J${id}: skipped (not in --joint filter)`);
            continue;
        }

        const alive = await ctrl.ping().catch(() => false);
        if (!alive) {
            console.log(`\nJ${id}: no response to ping — skipping.`);
            continue;
        }

        await ctrl.startServo();
        await sleep(100);

        try {
            results[id] = await tuneJoint(ctrl, id);
        } catch (e) {
            console.error(`\nJ${id} error: ${e.message}`);
        }

        // Return tuned joint to 0° before moving to next joint. Wrapped in its
        // own try/catch — this used to be able to throw uncaught (e.g. a
        // write timeout) and crash the whole run, silently discarding every
        // joint's results gathered so far even though they'd already been
        // tuned successfully and just hadn't been written to disk yet.
        try {
            await ctrl.setSpeed(HOME_SPEED);
            await moveTo(ctrl, homeStepsFor[id]);
            await waitSettle(ctrl).catch(() => {});
            await sleep(POST_SETTLE_DWELL_MS);
        } catch (e) {
            console.error(`\nJ${id}: failed to return to 0° after tuning: ${e.message}`);
        }
    }

    // ── Summary ────────────────────────────────────────────────────────────
    console.log('\n\n' + '═'.repeat(56));
    console.log('  SUMMARY — recommended servo-pid-config.json values');
    console.log('═'.repeat(56));
    for (const [id, r] of Object.entries(results)) {
        const d = r.diagnostics;
        console.log(`  J${id}: P=${r.p}  D=${r.d}  I=${r.i}${r.i > 0 ? ' ⚡' : ''}  MinStartup=${r.minStartupForce}`);
        console.log(`       MaxTorque=${d.maxTorque}  UnloadingCond=0b${d.unloadingCond.toString(2).padStart(8,'0')}  ProtTorque=${d.protTorquePct}%`);
    }

    // ── Write config file — merge with existing so partial runs don't clobber ──
    let config = { _comment: 'Auto-tuned PID values per joint. Applied by servoWorker.js at startup.', joints: {} };
    if (fs.existsSync(CONFIG_PATH)) {
        try { config = JSON.parse(fs.readFileSync(CONFIG_PATH, 'utf8')); } catch (_) {}
        if (!config.joints) config.joints = {};
    }
    config._tuned = new Date().toISOString();
    for (const [id, r] of Object.entries(results)) {
        config.joints[id] = { p: r.p, d: r.d, i: r.i, minStartupForce: r.minStartupForce };
    }
    fs.writeFileSync(CONFIG_PATH, JSON.stringify(config, null, 2));
    console.log(`\nConfig written → ${CONFIG_PATH}`);
    console.log('Restart the st3215-server to apply these values automatically at every startup.\n');

    for (const c of Object.values(controllers)) await c.close().catch(() => {});
    port.close(() => {});
}

main().catch(e => {
    console.error('\nFatal error:', e.message || e);
    process.exit(1);
});
