'use strict';
/**
 * Timing maths for server-side linear paths (executeLinearMove). Pure functions,
 * no I/O, so they can be tested without the arm (see linearPathTiming.test.js).
 * Based on the Smart Factory 2 team's analysis and patch, 2026-11-11.
 *
 * Why this exists. executeLinearMove turns a Cartesian line into waypoints and
 * streams one all-joint batch per waypoint. On the arm, some joints (J5, J3)
 * arrived seconds after the others:
 *
 *  1. Speeds were computed from ONE waypoint's travel. When the bus fell behind,
 *     the worker skipped to the newest waypoint (positions are absolute, so that
 *     is fine) but wrote it with the speed for a one-step move, far too slow.
 *  2. Nothing rewrote the speed after the last waypoint, so a joint still behind
 *     finished its leftover distance at the speed of the path's last, tiny step.
 *  3. The plan could ask for more joint speed than a servo can deliver, so the
 *     capped joint lagged while the others carried on.
 *
 * planWaypointTiming() fixes (3) by stretching only the steps that need it.
 * jointSpeedStepsPerSec() fixes (1): the speed comes from the real distance to
 * the new goal over the real time available. settleSpeeds() fixes (2): the final
 * goal is re-written so every joint finishes together.
 */

const STEPS_PER_DEG = 4096 / 360;  // ST3215: 11.378 steps per degree
const SERVO_MAX_STEPS_PER_SEC = 3400;
const SERVO_MIN_STEPS_PER_SEC = 50;

/**
 * Per-waypoint delays so that no joint has to exceed maxJointDegPerSec.
 * Steps that already fit keep the nominal interval; only steps with a big joint
 * jump are stretched. Returns the delay AFTER each waypoint is sent (ms).
 *
 * @param {number[]} startAngles   joint angles (deg) at the start of the path
 * @param {number[][]} steps       waypoint joint angles (deg)
 * @param {number} intervalMs      nominal interval from the requested Cartesian speed
 * @param {{maxJointDegPerSec?: number}} [opts]
 * @returns {{dtMs: number[], totalMs: number, slowedSteps: number, peakJointDegPerSec: number}}
 */
function planWaypointTiming(startAngles, steps, intervalMs, opts) {
    const maxRate = (opts && opts.maxJointDegPerSec > 0) ? opts.maxJointDegPerSec : 110;
    const dtMs = [];
    let prev = startAngles || [];
    let totalMs = 0;
    let slowedSteps = 0;
    let peakJointDegPerSec = 0;
    for (let i = 0; i < steps.length; i++) {
        const a = steps[i];
        let maxTravel = 0;
        for (let j = 0; j < a.length; j++) {
            maxTravel = Math.max(maxTravel, Math.abs(a[j] - (prev[j] || 0)));
        }
        const needMs = Math.ceil((maxTravel / maxRate) * 1000);
        const dt = Math.max(intervalMs, needMs);
        if (dt > intervalMs) slowedSteps++;
        dtMs.push(dt);
        totalMs += dt;
        peakJointDegPerSec = Math.max(peakJointDegPerSec, maxTravel / (dt / 1000));
        prev = a;
    }
    return { dtMs, totalMs, slowedSteps, peakJointDegPerSec };
}

/**
 * Speed (steps/s) to write with a goal so the servo covers travelDeg in about
 * dtMs, with a margin for its acceleration ramp. Clamped to the servo's range.
 */
function jointSpeedStepsPerSec(travelDeg, dtMs, margin) {
    const m = margin > 0 ? margin : 1.25;
    const seconds = Math.max(dtMs, 1) / 1000;
    const raw = Math.round(m * Math.abs(travelDeg) * STEPS_PER_DEG / seconds);
    return Math.min(SERVO_MAX_STEPS_PER_SEC, Math.max(SERVO_MIN_STEPS_PER_SEC, raw));
}

/**
 * Coordinated finish. Given each joint's remaining distance to the final goal,
 * choose one common time so every joint arrives together, never faster than
 * settleDegPerSec for the joint with furthest to go.
 *
 * @param {number[]} remainingDeg  remaining distance per joint (deg, sign ignored)
 * @param {{settleDegPerSec?: number, minSeconds?: number, maxSeconds?: number}} [opts]
 * @returns {{seconds: number, speeds: number[]}}
 */
function settleSpeeds(remainingDeg, opts) {
    const rate = (opts && opts.settleDegPerSec > 0) ? opts.settleDegPerSec : 60;
    const minS = (opts && opts.minSeconds > 0) ? opts.minSeconds : 0.3;
    const maxS = (opts && opts.maxSeconds > 0) ? opts.maxSeconds : 4.0;
    const furthest = Math.max(0, ...remainingDeg.map(Math.abs));
    const seconds = Math.min(maxS, Math.max(minS, furthest / rate));
    const speeds = remainingDeg.map(d => jointSpeedStepsPerSec(d, seconds * 1000, 1.0));
    return { seconds, speeds };
}

module.exports = {
    STEPS_PER_DEG,
    SERVO_MAX_STEPS_PER_SEC,
    SERVO_MIN_STEPS_PER_SEC,
    planWaypointTiming,
    jointSpeedStepsPerSec,
    settleSpeeds
};
