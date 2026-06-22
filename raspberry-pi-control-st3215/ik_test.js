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

    const base = robotKinematics.inverseKinematics(
        { x: targetPos.x, y: targetPos.y, z: targetPos.z, orientation: desiredOrientation },
        initialAngles
    );
    if (!base) { console.log('  IK: NO SOLUTION\n'); return; }
    console.log(`  IK base  : ${base.map((a,i) => `J${i+1}:${a.toFixed(1)}°`).join('  ')}`);

    const refined = robotKinematics.refineOrientationWithAccuracy(
        targetPos, base, desiredOrientation, refAngles
    );
    console.log(`  Refined  : ${refined.angles.map((a,i) => `J${i+1}:${a.toFixed(1)}°`).join('  ')}`);
    console.log(`  Pos err  : ${refined.positionErrorMm.toFixed(2)} mm`);
    console.log(`  Ori err  : ${refined.orientationErrorDeg.toFixed(1)}°`);

    // Verify with FK
    const checkFk = robotKinematics.forwardKinematics(refined.angles);
    const checkPos = checkFk.position;
    const checkToolZ = toolZAxis(checkFk.rotation);
    console.log(`  FK check : X=${checkPos.x.toFixed(2)}  Y=${checkPos.y.toFixed(2)}  Z=${checkPos.z.toFixed(2)}`);
    console.log(`  FK tool Z: x=${checkToolZ.x.toFixed(3)}  y=${checkToolZ.y.toFixed(3)}  z=${checkToolZ.z.toFixed(3)}`);

    // Report any big joint jumps
    refined.angles.forEach((a, i) => {
        const delta = Math.abs(a - initialAngles[i]);
        if (delta > 10) console.log(`  !! J${i+1} moved ${delta.toFixed(1)}° from ${initialAngles[i].toFixed(1)}° to ${a.toFixed(1)}°`);
    });
    console.log();
    return refined.angles;
}

// ---- Test 1: Z+10 from home, maintaining home tool orientation ----
let angles = testJog(
    'Z+10mm from home, maintain home orientation',
    { x: homePos.x, y: homePos.y, z: homePos.z + 10 },
    homeAngles,
    homeToolZ
);

// ---- Test 2: Z+10 from home, forcing tool-down ----
angles = testJog(
    'Z+10mm from home, forced tool-down {0,0,-1}',
    { x: homePos.x, y: homePos.y, z: homePos.z + 10 },
    homeAngles,
    { x: 0, y: 0, z: -1 }
);

// ---- Test 3: X+10 from home, maintain home orientation ----
angles = testJog(
    'X+10mm from home, maintain home orientation',
    { x: homePos.x + 10, y: homePos.y, z: homePos.z },
    homeAngles,
    homeToolZ
);

// ---- Test 4: chain — Z+10 then Z+10 again ----
console.log('=== Chained jog: Z+10 twice (each time maintain prev orientation) ===');
let pos = { x: homePos.x, y: homePos.y, z: homePos.z };
let curAngles = homeAngles.slice();
for (let step = 1; step <= 3; step++) {
    pos = { x: pos.x, y: pos.y, z: pos.z + 10 };
    const fk0 = robotKinematics.forwardKinematics(curAngles);
    const ori = toolZAxis(fk0.rotation);
    curAngles = testJog(`Step ${step}: Z+10`, pos, curAngles, ori) || curAngles;
}
