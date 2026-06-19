/**
 * Kinematics Service (Server Side)
 *
 * This file moves URDF parsing + forward/inverse kinematics to the Pi server.
 * It intentionally uses simple JavaScript and a small URDF parser that supports
 * the structure used in kinematics.urdf (Electron app).
 */

function radiansToDegrees(radians) {
    return (radians * 180) / Math.PI;
}

function parseAttributes(tagText) {
    const attrs = {};
    const attrRegex = /([a-zA-Z_][a-zA-Z0-9_\-]*)\s*=\s*"([^"]*)"/g;
    let match = null;
    while ((match = attrRegex.exec(tagText)) !== null) {
        attrs[match[1]] = match[2];
    }
    return attrs;
}

function parseVector3(text, fallback) {
    if (!text || typeof text !== 'string') {
        return { x: fallback.x, y: fallback.y, z: fallback.z };
    }
    const parts = text.trim().split(/\s+/);
    return {
        x: parseFloat(parts[0]) || 0,
        y: parseFloat(parts[1]) || 0,
        z: parseFloat(parts[2]) || 0
    };
}

function parseURDF(urdfXml) {
    if (typeof urdfXml !== 'string' || !urdfXml.trim()) {
        throw new Error('URDF is empty');
    }

    const robotMatch = urdfXml.match(/<robot\b([^>]*)>/i);
    if (!robotMatch) {
        throw new Error('No robot element found in URDF');
    }
    const robotAttrs = parseAttributes(robotMatch[1] || '');
    const robotName = robotAttrs.name || 'robot';

    const links = [];
    const linkRegex = /<link\b([^>]*)\/>/gi;
    let linkMatch = null;
    while ((linkMatch = linkRegex.exec(urdfXml)) !== null) {
        const attrs = parseAttributes(linkMatch[1] || '');
        links.push({ name: attrs.name || null });
    }

    const joints = [];
    const jointRegex = /<joint\b([^>]*)>([\s\S]*?)<\/joint>/gi;
    let jointMatch = null;
    while ((jointMatch = jointRegex.exec(urdfXml)) !== null) {
        const jointAttrs = parseAttributes(jointMatch[1] || '');
        const body = jointMatch[2] || '';
        const joint = {
            name: jointAttrs.name || null,
            type: jointAttrs.type || null,
            parent: null,
            child: null,
            origin: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
            axis: { x: 0, y: 0, z: 1 },
            zeroOffsetDegrees: null,
            limits: {
                lowerRadians: null,
                upperRadians: null,
                lowerDegrees: null,
                upperDegrees: null,
                effort: 0,
                velocity: 0
            }
        };

        if (jointAttrs.zero_offset_degrees !== undefined) {
            const offset = parseFloat(jointAttrs.zero_offset_degrees);
            if (!isNaN(offset)) {
                joint.zeroOffsetDegrees = offset;
            }
        }

        const parentMatch = body.match(/<parent\b([^>]*)\/>/i);
        if (parentMatch) {
            const parentAttrs = parseAttributes(parentMatch[1] || '');
            joint.parent = parentAttrs.link || null;
        }

        const childMatch = body.match(/<child\b([^>]*)\/>/i);
        if (childMatch) {
            const childAttrs = parseAttributes(childMatch[1] || '');
            joint.child = childAttrs.link || null;
        }

        const originMatch = body.match(/<origin\b([^>]*)\/>/i);
        if (originMatch) {
            const originAttrs = parseAttributes(originMatch[1] || '');
            const xyz = parseVector3(originAttrs.xyz, { x: 0, y: 0, z: 0 });
            const rpy = parseVector3(originAttrs.rpy, { x: 0, y: 0, z: 0 });
            joint.origin.x = xyz.x;
            joint.origin.y = xyz.y;
            joint.origin.z = xyz.z;
            joint.origin.roll = rpy.x;
            joint.origin.pitch = rpy.y;
            joint.origin.yaw = rpy.z;
        }

        const axisMatch = body.match(/<axis\b([^>]*)\/>/i);
        if (axisMatch) {
            const axisAttrs = parseAttributes(axisMatch[1] || '');
            joint.axis = parseVector3(axisAttrs.xyz, { x: 0, y: 0, z: 1 });
        }

        const limitMatch = body.match(/<limit\b([^>]*)\/>/i);
        if (limitMatch) {
            const limitAttrs = parseAttributes(limitMatch[1] || '');
            if (limitAttrs.lower !== undefined) {
                const lowerRad = parseFloat(limitAttrs.lower);
                if (!isNaN(lowerRad)) {
                    joint.limits.lowerRadians = lowerRad;
                    joint.limits.lowerDegrees = radiansToDegrees(lowerRad);
                }
            }
            if (limitAttrs.upper !== undefined) {
                const upperRad = parseFloat(limitAttrs.upper);
                if (!isNaN(upperRad)) {
                    joint.limits.upperRadians = upperRad;
                    joint.limits.upperDegrees = radiansToDegrees(upperRad);
                }
            }
            if (limitAttrs.effort !== undefined) {
                const effort = parseFloat(limitAttrs.effort);
                if (!isNaN(effort)) {
                    joint.limits.effort = effort;
                }
            }
            if (limitAttrs.velocity !== undefined) {
                const velocity = parseFloat(limitAttrs.velocity);
                if (!isNaN(velocity)) {
                    joint.limits.velocity = velocity;
                }
            }
        }

        joints.push(joint);
    }

    return {
        name: robotName,
        joints: joints,
        links: links
    };
}

function multiplyMatrices(A, B) {
    const result = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ];
    for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
            for (let k = 0; k < 4; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

function identity4x4() {
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}

function positionFromMatrix(T) {
    return { x: T[0][3], y: T[1][3], z: T[2][3] };
}

function toolZAxisFromMatrix(T) {
    return { x: T[0][0], y: T[1][0], z: T[2][0] };
}

function normalizeVector(v) {
    const x = typeof v.x === 'number' ? v.x : 0;
    const y = typeof v.y === 'number' ? v.y : 0;
    const z = typeof v.z === 'number' ? v.z : 0;
    const len = Math.sqrt(x * x + y * y + z * z);
    if (len < 1e-6) return { x: 0, y: 0, z: -1 };
    return { x: x / len, y: y / len, z: z / len };
}

function originToMatrix(origin) {
    const x = origin.x || 0;
    const y = origin.y || 0;
    const z = origin.z || 0;
    const roll = origin.roll || 0;
    const pitch = origin.pitch || 0;
    const yaw = origin.yaw || 0;

    const cosR = Math.cos(roll);
    const sinR = Math.sin(roll);
    const cosP = Math.cos(pitch);
    const sinP = Math.sin(pitch);
    const cosY = Math.cos(yaw);
    const sinY = Math.sin(yaw);

    const r11 = cosY * cosP;
    const r12 = cosY * sinP * sinR - sinY * cosR;
    const r13 = cosY * sinP * cosR + sinY * sinR;
    const r21 = sinY * cosP;
    const r22 = sinY * sinP * sinR + cosY * cosR;
    const r23 = sinY * sinP * cosR - cosY * sinR;
    const r31 = -sinP;
    const r32 = cosP * sinR;
    const r33 = cosP * cosR;

    return [
        [r11, r12, r13, x],
        [r21, r22, r23, y],
        [r31, r32, r33, z],
        [0, 0, 0, 1]
    ];
}

function axisRotationMatrix(axis, angle) {
    const ax = (axis && typeof axis.x === 'number') ? axis.x : 0;
    const ay = (axis && typeof axis.y === 'number') ? axis.y : 0;
    const az = (axis && typeof axis.z === 'number') ? axis.z : 1;

    const length = Math.sqrt(ax * ax + ay * ay + az * az);
    const nx = length === 0 ? 0 : ax / length;
    const ny = length === 0 ? 0 : ay / length;
    const nz = length === 0 ? 1 : az / length;

    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    const oneMinusCos = 1 - cosA;

    const r11 = cosA + nx * nx * oneMinusCos;
    const r12 = nx * ny * oneMinusCos - nz * sinA;
    const r13 = nx * nz * oneMinusCos + ny * sinA;
    const r21 = ny * nx * oneMinusCos + nz * sinA;
    const r22 = cosA + ny * ny * oneMinusCos;
    const r23 = ny * nz * oneMinusCos - nx * sinA;
    const r31 = nz * nx * oneMinusCos - ny * sinA;
    const r32 = nz * ny * oneMinusCos + nx * sinA;
    const r33 = cosA + nz * nz * oneMinusCos;

    return [
        [r11, r12, r13, 0],
        [r21, r22, r23, 0],
        [r31, r32, r33, 0],
        [0, 0, 0, 1]
    ];
}

class RobotKinematics {
    constructor() {
        this.urdfData = null;
        this.joints = [];
        this.fixedToolJoints = [];
        this.maxReachMm = null;
    }

    loadURDF(urdfXml) {
        this.urdfData = parseURDF(urdfXml);
        this.joints = this.urdfData.joints.filter(joint => joint.type === 'revolute');
        const lastRevoluteChild = this.joints.length > 0 ? this.joints[this.joints.length - 1].child : null;
        this.fixedToolJoints = (this.urdfData.joints || []).filter(
            (j) => j.type === 'fixed' && j.parent === lastRevoluteChild
        );

        let totalLengthMm = 0;
        this.joints.forEach((joint) => {
            const ox = joint.origin.x || 0;
            const oy = joint.origin.y || 0;
            const oz = joint.origin.z || 0;
            totalLengthMm += Math.sqrt(ox * ox + oy * oy + oz * oz) * 1000;
        });
        this.fixedToolJoints.forEach((joint) => {
            const ox = joint.origin.x || 0;
            const oy = joint.origin.y || 0;
            const oz = joint.origin.z || 0;
            totalLengthMm += Math.sqrt(ox * ox + oy * oy + oz * oz) * 1000;
        });
        this.maxReachMm = totalLengthMm * 1.05;
        return this.getKinematicsInfo();
    }

    isConfigured() {
        return this.joints.length > 0;
    }

    getJointCount() {
        return this.joints.length;
    }

    getJointConfigs() {
        return this.joints;
    }

    getKinematicsInfo() {
        return {
            configured: this.isConfigured(),
            jointCount: this.getJointCount(),
            urdfData: this.urdfData,
            joints: this.getJointConfigs(),
            maxReachMm: this.maxReachMm
        };
    }

    forwardKinematics(jointAngles) {
        if (!this.isConfigured()) {
            throw new Error('URDF not loaded');
        }
        if (!Array.isArray(jointAngles) || jointAngles.length !== this.joints.length) {
            throw new Error('Invalid joint angles length');
        }

        let T = identity4x4();
        for (let i = 0; i < this.joints.length; i++) {
            const joint = this.joints[i];
            let angleDeg = jointAngles[i] || 0;
            if (typeof joint.zeroOffsetDegrees === 'number') {
                angleDeg += joint.zeroOffsetDegrees;
            }
            const angleRad = (angleDeg * Math.PI) / 180;
            const T_joint = multiplyMatrices(originToMatrix(joint.origin), axisRotationMatrix(joint.axis, angleRad));
            T = multiplyMatrices(T, T_joint);
        }

        for (let i = 0; i < this.fixedToolJoints.length; i++) {
            T = multiplyMatrices(T, originToMatrix(this.fixedToolJoints[i].origin));
        }

        const p = positionFromMatrix(T);
        return {
            position: { x: p.x * 1000, y: p.y * 1000, z: p.z * 1000 },
            rotation: T
        };
    }

    getForwardKinematicsSteps(jointAngles) {
        if (!this.isConfigured()) throw new Error('URDF not loaded');
        if (!Array.isArray(jointAngles) || jointAngles.length !== this.joints.length) {
            throw new Error('Invalid joint angles length');
        }

        const steps = [];
        let T = identity4x4();

        for (let i = 0; i < this.joints.length; i++) {
            const joint = this.joints[i];
            const inputAngleDeg = jointAngles[i] || 0;
            let angleDeg = inputAngleDeg;
            if (typeof joint.zeroOffsetDegrees === 'number') {
                angleDeg += joint.zeroOffsetDegrees;
            }
            const angleRad = (angleDeg * Math.PI) / 180;
            const T_joint = multiplyMatrices(originToMatrix(joint.origin), axisRotationMatrix(joint.axis, angleRad));
            T = multiplyMatrices(T, T_joint);
            steps.push({
                index: i,
                name: joint.name || `Joint ${i + 1}`,
                angleInput: inputAngleDeg,
                angleUsed: angleDeg,
                origin: joint.origin,
                axis: joint.axis,
                transform: T
            });
        }

        for (let f = 0; f < this.fixedToolJoints.length; f++) {
            const toolJoint = this.fixedToolJoints[f];
            T = multiplyMatrices(T, originToMatrix(toolJoint.origin));
            steps.push({
                index: this.joints.length + f,
                name: toolJoint.name || 'tool',
                angleInput: 0,
                angleUsed: 0,
                origin: toolJoint.origin,
                axis: { x: 0, y: 0, z: 0 },
                transform: T
            });
        }

        return { steps: steps, finalTransform: T };
    }

    inverseKinematics(targetPose, initialAngles) {
        if (!this.isConfigured()) throw new Error('URDF not loaded');
        if (!targetPose || typeof targetPose.x !== 'number' || typeof targetPose.y !== 'number' || typeof targetPose.z !== 'number') {
            throw new Error('Invalid target pose');
        }

        const numJoints = this.joints.length;
        const hasOrientationTarget = !!(targetPose && targetPose.orientation);
        const desiredToolZ = hasOrientationTarget ? normalizeVector(targetPose.orientation) : { x: 0, y: 0, z: -1 };

        if (this.maxReachMm) {
            const distance = Math.sqrt(targetPose.x * targetPose.x + targetPose.y * targetPose.y + targetPose.z * targetPose.z);
            if (distance > this.maxReachMm + 10) return null;
        }

        const angles = [];
        for (let i = 0; i < numJoints; i++) {
            if (initialAngles && Array.isArray(initialAngles) && typeof initialAngles[i] === 'number') {
                angles.push(initialAngles[i]);
            } else {
                angles.push(0);
            }
        }

        const maxIterations = 500;
        const positionToleranceMm = 0.3;
        const finiteDifferenceDeg = 0.25;   // finer Jacobian → better accuracy
        const baseStepSize = 0.025;
        const orientationWeight = 8.0;
        const maxDeltaPerIterDeg = 4.0;     // cap per-joint step to prevent singularity blow-up

        for (let iter = 0; iter < maxIterations; iter++) {
            const fkResult = this.forwardKinematics(angles);
            const currentPos = fkResult.position;
            const currentToolZ = toolZAxisFromMatrix(fkResult.rotation);

            const errX = targetPose.x - currentPos.x;
            const errY = targetPose.y - currentPos.y;
            const errZ = targetPose.z - currentPos.z;
            const positionErrorLength = Math.sqrt(errX * errX + errY * errY + errZ * errZ);

            let oriErrX = 0;
            let oriErrY = 0;
            let oriErrZ = 0;
            let orientationErrorLength = 0;
            if (hasOrientationTarget) {
                oriErrX = desiredToolZ.x - currentToolZ.x;
                oriErrY = desiredToolZ.y - currentToolZ.y;
                oriErrZ = desiredToolZ.z - currentToolZ.z;
                orientationErrorLength = Math.sqrt(oriErrX * oriErrX + oriErrY * oriErrY + oriErrZ * oriErrZ);
            }

            // Combined convergence: both position AND orientation must be within tolerance
            const posConverged = positionErrorLength < positionToleranceMm;
            const oriConverged = !hasOrientationTarget || orientationErrorLength < 0.05;
            if (posConverged && oriConverged) break;

            const Jpos = [
                new Array(numJoints).fill(0),
                new Array(numJoints).fill(0),
                new Array(numJoints).fill(0)
            ];
            const Jori = hasOrientationTarget ? [
                new Array(numJoints).fill(0),
                new Array(numJoints).fill(0),
                new Array(numJoints).fill(0)
            ] : null;

            for (let j = 0; j < numJoints; j++) {
                const originalAngle = angles[j];
                angles[j] = originalAngle + finiteDifferenceDeg;
                const fkPlus = this.forwardKinematics(angles);
                angles[j] = originalAngle;

                Jpos[0][j] = (fkPlus.position.x - currentPos.x) / finiteDifferenceDeg;
                Jpos[1][j] = (fkPlus.position.y - currentPos.y) / finiteDifferenceDeg;
                Jpos[2][j] = (fkPlus.position.z - currentPos.z) / finiteDifferenceDeg;

                if (hasOrientationTarget && Jori) {
                    const toolZPlus = toolZAxisFromMatrix(fkPlus.rotation);
                    Jori[0][j] = (toolZPlus.x - currentToolZ.x) / finiteDifferenceDeg;
                    Jori[1][j] = (toolZPlus.y - currentToolZ.y) / finiteDifferenceDeg;
                    Jori[2][j] = (toolZPlus.z - currentToolZ.z) / finiteDifferenceDeg;
                }
            }

            // Adaptive step: smaller when close (fine tuning), larger when far (fast approach)
            const stepSize = baseStepSize / (1 + positionErrorLength / 60);
            for (let j = 0; j < numJoints; j++) {
                let grad = Jpos[0][j] * errX + Jpos[1][j] * errY + Jpos[2][j] * errZ;
                if (hasOrientationTarget && Jori) {
                    grad += orientationWeight * (Jori[0][j] * oriErrX + Jori[1][j] * oriErrY + Jori[2][j] * oriErrZ);
                }
                // Cap per-iteration delta to prevent divergence near singularities
                const delta = stepSize * grad;
                angles[j] += Math.max(-maxDeltaPerIterDeg, Math.min(maxDeltaPerIterDeg, delta));

                const joint = this.joints[j];
                if (joint && joint.limits) {
                    if (typeof joint.limits.lowerDegrees === 'number' && angles[j] < joint.limits.lowerDegrees) {
                        angles[j] = joint.limits.lowerDegrees;
                    }
                    if (typeof joint.limits.upperDegrees === 'number' && angles[j] > joint.limits.upperDegrees) {
                        angles[j] = joint.limits.upperDegrees;
                    }
                }
            }
        }

        for (let i = 0; i < numJoints; i++) {
            if (!isFinite(angles[i])) return null;
        }

        const finalFk = this.forwardKinematics(angles);
        const dx = finalFk.position.x - targetPose.x;
        const dy = finalFk.position.y - targetPose.y;
        const dz = finalFk.position.z - targetPose.z;
        const finalError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (finalError > 8.0) return null;

        return angles;
    }

    /**
     * Refine solution using both position and tool-Z orientation.
     * This is a beginner-friendly grid-search refinement copied from the
     * Electron app's kinematics logic.
     *
     * @param {{ x:number, y:number, z:number }} targetPose - Desired XYZ in mm
     * @param {Array<number>} baseAngles - Base joint angles in degrees
     * @param {{ x:number, y:number, z:number }} desiredOrientation - Desired tool Z axis direction
     * @returns {{ angles: Array<number>, positionErrorMm: number, orientationErrorDeg: number, achievedPosition: {x,y,z}|null }}
     */
    refineOrientationWithAccuracy(targetPose, baseAngles, desiredOrientation, referenceAngles) {
        if (!this.isConfigured() || !baseAngles || !Array.isArray(baseAngles) || !desiredOrientation) {
            return {
                angles: baseAngles || [],
                positionErrorMm: Infinity,
                orientationErrorDeg: Infinity,
                achievedPosition: null
            };
        }

        const numJoints = this.joints.length;
        if (numJoints < 3) {
            return {
                angles: baseAngles.slice(),
                positionErrorMm: Infinity,
                orientationErrorDeg: Infinity,
                achievedPosition: null
            };
        }

        // referenceAngles = arm's current position before the move.
        // Used to penalise large joint travel and avoid unnecessary flips.
        const refAngles = (Array.isArray(referenceAngles) && referenceAngles.length === numJoints)
            ? referenceAngles : null;

        // Weight for joint-travel penalty in mm-equivalent per degree of total travel.
        // Small enough not to prevent reaching the target, large enough to prefer
        // nearby solutions over equivalent-accuracy solutions that flip a joint.
        const jointTravelWeight = 0.02;

        const desiredZ = normalizeVector(desiredOrientation);
        const clampToLimits = (angleDeg, joint) => {
            let a = angleDeg;
            if (joint && joint.limits) {
                if (typeof joint.limits.lowerDegrees === 'number' && a < joint.limits.lowerDegrees) a = joint.limits.lowerDegrees;
                if (typeof joint.limits.upperDegrees === 'number' && a > joint.limits.upperDegrees) a = joint.limits.upperDegrees;
            }
            return a;
        };

        let orientationWeightForScore = 1.0;
        const evaluateCandidate = (angles) => {
            try {
                const fk = this.forwardKinematics(angles);
                const pos = fk.position;
                const toolZ = toolZAxisFromMatrix(fk.rotation);

                const dx = pos.x - targetPose.x;
                const dy = pos.y - targetPose.y;
                const dz = pos.z - targetPose.z;
                const positionErrorMm = Math.sqrt(dx * dx + dy * dy + dz * dz);

                const dot = desiredZ.x * toolZ.x + desiredZ.y * toolZ.y + desiredZ.z * toolZ.z;
                const clampedDot = Math.max(-1, Math.min(1, dot));
                const orientationErrorDeg = (Math.acos(clampedDot) * 180) / Math.PI;

                // Joint travel penalty: sum of |candidate - reference| across all joints.
                // Prefers solutions near the arm's current position over equivalent
                // solutions that require large joint movements or flips.
                let travelPenalty = 0;
                if (refAngles) {
                    for (let i = 0; i < numJoints; i++) {
                        travelPenalty += Math.abs(angles[i] - refAngles[i]);
                    }
                    travelPenalty *= jointTravelWeight;
                }

                const score = positionErrorMm + orientationWeightForScore * orientationErrorDeg + travelPenalty;
                return {
                    score,
                    positionErrorMm,
                    orientationErrorDeg,
                    achievedPosition: { x: pos.x, y: pos.y, z: pos.z }
                };
            } catch (e) {
                return null;
            }
        };

        // Early exit: skip expensive refinement if base IK is already accurate
        const baseEval = evaluateCandidate(baseAngles);
        if (baseEval && baseEval.positionErrorMm < 1.5 && baseEval.orientationErrorDeg < 3.0) {
            return {
                angles: baseAngles.slice(),
                positionErrorMm: baseEval.positionErrorMm,
                orientationErrorDeg: baseEval.orientationErrorDeg,
                achievedPosition: baseEval.achievedPosition
            };
        }

        // runOnePass: search a grid of offsets for joints 0-5 (all 6 joints).
        // wristRoll2Offs controls joint 5 (J6, the final wrist roll).
        const runOnePass = (currentBase, maxPositionErrorMm, baseYawOffs, shoulderOffs, elbowOffs, wristRollOffs, wristPitchOffs, wristRoll2Offs) => {
            const baseOffs = Array.isArray(baseYawOffs) && baseYawOffs.length > 0 ? baseYawOffs : [0];
            const wr2Offs = Array.isArray(wristRoll2Offs) && wristRoll2Offs.length > 0 ? wristRoll2Offs : [0];
            let bestAngles = currentBase.slice();
            let bestEval = evaluateCandidate(bestAngles);

            if (!bestEval) {
                return {
                    angles: bestAngles,
                    positionErrorMm: Infinity,
                    orientationErrorDeg: Infinity,
                    achievedPosition: null
                };
            }

            for (let b = 0; b < baseOffs.length; b++) {
                for (let s = 0; s < shoulderOffs.length; s++) {
                    for (let e = 0; e < elbowOffs.length; e++) {
                        for (let r = 0; r < wristRollOffs.length; r++) {
                            for (let p = 0; p < wristPitchOffs.length; p++) {
                                for (let r2 = 0; r2 < wr2Offs.length; r2++) {
                                    const candidateAngles = currentBase.slice();

                                    if (numJoints > 0) candidateAngles[0] = clampToLimits(currentBase[0] + baseOffs[b], this.joints[0]);
                                    if (numJoints > 1) candidateAngles[1] = clampToLimits(currentBase[1] + shoulderOffs[s], this.joints[1]);
                                    if (numJoints > 2) candidateAngles[2] = clampToLimits(currentBase[2] + elbowOffs[e], this.joints[2]);
                                    if (numJoints > 3) candidateAngles[3] = clampToLimits(currentBase[3] + wristRollOffs[r], this.joints[3]);
                                    if (numJoints > 4) candidateAngles[4] = clampToLimits(currentBase[4] + wristPitchOffs[p], this.joints[4]);
                                    if (numJoints > 5) candidateAngles[5] = clampToLimits(currentBase[5] + wr2Offs[r2], this.joints[5]);

                                    const evalResult = evaluateCandidate(candidateAngles);
                                    if (!evalResult || evalResult.positionErrorMm > maxPositionErrorMm) continue;

                                    if (evalResult.score < bestEval.score) {
                                        bestEval = evalResult;
                                        bestAngles = candidateAngles.slice();
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return {
                angles: bestAngles,
                positionErrorMm: bestEval.positionErrorMm,
                orientationErrorDeg: bestEval.orientationErrorDeg,
                achievedPosition: bestEval.achievedPosition
            };
        };

        const wristRollFull = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180];
        const wristPitchFull = [0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90];
        // J6 full sweep — 30° steps cover full 360°
        const wristRoll2Full = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180];

        const wristRollFine = [];
        for (let d = -180; d <= 180; d += 15) wristRollFine.push(d);
        const wristPitchFine = [];
        for (let d = -90; d <= 90; d += 10) wristPitchFine.push(d);

        const baseYawTryBothSides = [0, -180, 180, -90, 90];

        orientationWeightForScore = 5.0;
        const pass0 = runOnePass(
            baseAngles,
            25,
            baseYawTryBothSides,
            [0, -10, 10, -20, 20, -30, 30],
            [0, -10, 10, -20, 20, -30, 30],
            wristRollFull,
            wristPitchFull,
            wristRoll2Full
        );

        const pass0b = runOnePass(pass0.angles, 25, [0], [0], [0], wristRollFine, wristPitchFine, wristRoll2Full);

        orientationWeightForScore = 1.0;

        const pass1 = runOnePass(pass0b.angles, 20, [0], [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20, -30, 30], wristRollFull, wristPitchFull, wristRoll2Full);
        const pass2 = runOnePass(pass1.angles, 12, [0], [0, -8, 8, -15, 15], [0, -8, 8, -15, 15], [0, -15, 15, -30, 30, -45, 45], [0, -15, 15, -30, 30, -45, 45], [0, -15, 15, -30, 30, -45, 45]);
        const pass3 = runOnePass(pass2.angles, 6, [0], [0, -5, 5, -10, 10], [0, -5, 5, -10, 10], [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20]);
        const pass4 = runOnePass(pass3.angles, 3, [0], [0, -2, 2, -5, 5], [0, -2, 2, -5, 5], [0, -5, 5, -10, 10], [0, -5, 5, -10, 10], [0, -5, 5, -10, 10]);
        const pass5 = runOnePass(pass4.angles, 12, [0, -5, 5], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6]);
        const pass6 = runOnePass(pass5.angles, 15, [0, -1, 1, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2]);

        return {
            angles: pass6.angles,
            positionErrorMm: pass6.positionErrorMm,
            orientationErrorDeg: pass6.orientationErrorDeg,
            achievedPosition: pass6.achievedPosition
        };
    }
}

const robotKinematics = new RobotKinematics();

module.exports = {
    robotKinematics: robotKinematics
};

