'use strict';
/**
 * IK diagnostic: simulates what moveToXYZ does for common jog moves.
 * Run on Pi: node ik_test.js
 */

const fs = require('fs');
const path = require('path');
const { robotKinematics } = require('./kinematicsService');

// ---- helper copied from kinematics.js (toolZAxisFromMatrix) ----
function toolZAxis(T) {
    // Third column of the 3x3 rotation block (= Z axis of tool frame in world)
    return { x: T[0][2], y: T[1][2], z: T[2][2] };
}

// ---- load URDF ----
const urdf = fs.readFileSync(path.join(__dirname, 'kinematics.urdf'), 'utf8');
const info = robotKinematics.loadURDF(urdf);
console.log(`URDF loaded: ${info.jointCount} revolute joints, maxReach=${info.maxReachMm} mm\n`);

// ---- print joint limits ----
console.log('Joint limits from URDF:');
if (info.joints) {
    info.joints.forEach((j, i) => {
        const lo = j.limits ? j.limits.lowerDegrees.toFixed(1) : '?';
        const hi = j.limits ? j.limits.upperDegrees.toFixed(1) : '?';
        console.log(`  J${i+1} (${j.name}): [${lo}°, ${hi}°]  axis=${JSON.stringify(j.axis)}`);
    });
}
console.log();

// ---- FK at home ----
const homeAngles = [0, 0, 0, 0, 0, 0];
const homeFk = robotKinematics.forwardKinematics(homeAngles);
const homePos = homeFk.position;
const homeToolZ = toolZAxis(homeFk.rotation);
console.log('=== FK at home (all joints 0°) ===');
console.log(`  Position : X=${homePos.x.toFixed(2)}  Y=${homePos.y.toFixed(2)}  Z=${homePos.z.toFixed(2)} mm`);
console.log(`  Tool Z   : x=${homeToolZ.x.toFixed(3)}  y=${homeToolZ.y.toFixed(3)}  z=${homeToolZ.z.toFixed(3)}`);
console.log();

// ---- helper: run IK + refine, print result ----
function testJog(label, targetPos, initialAngles, desiredOrientation) {
    const refAngles = initialAngles;
    console.log(`--- ${label} ---`);
    console.log(`  Target: X=${targetPos.x.toFixed(1)}  Y=${targetPos.y.toFixed(1)}  Z=${targetPos.z.toFixed(1)}`);
    console.log(`  Desired tool Z: x=${desiredOrientation.x.toFixed(3)} y=${desiredOrientation.y.toFixed(3)} z=${desiredOrientation.z.toFixed(3)}`);

    const base = desiredOrientation
        ? robotKinematics.inverseKinematics(
            { x: targetPos.x, y: targetPos.y, z: targetPos.z, orientation: desiredOrientation },
            initialAngles)
        : robotKinematics.inverseKinematics(
            { x: targetPos.x, y: targetPos.y, z: targetPos.z },
            initialAngles);
    if (!base) { console.log('  IK: NO SOLUTION\n'); return; }
    console.log(`  IK result: ${base.map((a,i) => `J${i+1}:${a.toFixed(1)}°`).join('  ')}`);

    // Verify with FK
    const checkFk = robotKinematics.forwardKinematics(base);
    const checkPos = checkFk.position;
    const checkToolZ = toolZAxis(checkFk.rotation);
    console.log(`  FK check : X=${checkPos.x.toFixed(2)}  Y=${checkPos.y.toFixed(2)}  Z=${checkPos.z.toFixed(2)}`);
    console.log(`  FK tool Z: x=${checkToolZ.x.toFixed(3)}  y=${checkToolZ.y.toFixed(3)}  z=${checkToolZ.z.toFixed(3)}`);
    if (desiredOrientation) {
        const dot = desiredOrientation.x*checkToolZ.x + desiredOrientation.y*checkToolZ.y + desiredOrientation.z*checkToolZ.z;
        const oriErr = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
        console.log(`  Ori err  : ${oriErr.toFixed(1)}°`);
    }
    const posErr = Math.sqrt((checkPos.x-targetPos.x)**2+(checkPos.y-targetPos.y)**2+(checkPos.z-targetPos.z)**2);
    console.log(`  Pos err  : ${posErr.toFixed(2)} mm`);

    // Report any big joint jumps
    base.forEach((a, i) => {
        const delta = Math.abs(a - initialAngles[i]);
        if (delta > 10) console.log(`  !! J${i+1} moved ${delta.toFixed(1)}° from ${initialAngles[i].toFixed(1)}° to ${a.toFixed(1)}°`);
    });
    console.log();
    return base;
}

// ---- Test 1: Z+10 jog (position-only IK, no orientation) ----
let angles = testJog('Z+10mm JOG (position-only)', { x: homePos.x, y: homePos.y, z: homePos.z + 10 }, homeAngles, null);

// ---- Test 2: X+10 jog (position-only IK) ----
angles = testJog('X+10mm JOG (position-only)', { x: homePos.x + 10, y: homePos.y, z: homePos.z }, homeAngles, null);

// ---- Test 3: Y+10 jog (position-only IK) ----
angles = testJog('Y+10mm JOG (position-only)', { x: homePos.x, y: homePos.y + 10, z: homePos.z }, homeAngles, null);

// ---- Test 4: Move to XYZ with tool-down orientation (IK with orientation) ----
angles = testJog(
    'Move to home+Z10, tool-down {0,0,-1} (IK with orientation)',
    { x: homePos.x, y: homePos.y, z: homePos.z + 10 },
    homeAngles,
    { x: 0, y: 0, z: -1 }
);

// ---- Test 5: Move to XYZ with tool-up (home) orientation ----
angles = testJog(
    'Move to home+X10, tool-up {0,0,1} (IK with orientation)',
    { x: homePos.x + 10, y: homePos.y, z: homePos.z },
    homeAngles,
    { x: 0, y: 0, z: 1 }
);

// ---- Test 6: Chained Y jogs (position-only, simulating quickMoveXYZ) ----
console.log('=== Chained Y+ jogs x5 (position-only IK, simulating quickMoveXYZ) ===');
let pos = { x: homePos.x, y: homePos.y, z: homePos.z };
let curAngles = homeAngles.slice();
for (let step = 1; step <= 5; step++) {
    pos = { x: pos.x, y: pos.y + 10, z: pos.z };
    curAngles = testJog(`Y+10 step ${step}`, pos, curAngles, null) || curAngles;
}
