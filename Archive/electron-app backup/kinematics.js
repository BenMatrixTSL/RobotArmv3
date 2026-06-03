/**
 * Robot Arm Kinematics Module
 *
 * Forward kinematics uses standard 4x4 homogeneous transformation matrices:
 *   T_base_to_ee = T_0_1 * T_1_2 * ... * T_(n-1)_n
 * Each T_i is: [ R(3x3)  p(3x1) ]   where R = rotation, p = position in meters (URDF).
 *              [ 0 0 0    1     ]
 * XYZ position in mm = first three elements of the last column (p), scaled by 1000.
 */

/**
 * Multiplies two 4x4 homogeneous transformation matrices (row-major).
 * Standard formula: (A*B)[i][j] = sum_k A[i][k] * B[k][j]
 *
 * @param {Array} A - 4x4 matrix (array of 4 rows)
 * @param {Array} B - 4x4 matrix
 * @returns {Array} 4x4 product matrix
 */
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

/**
 * Build 4x4 identity matrix
 * @returns {Array} 4x4 identity
 */
function identity4x4() {
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
}

/**
 * Extract XYZ position from a 4x4 transformation matrix (in meters).
 * Standard convention: position is the last column, rows 0..2.
 * @param {Array} T - 4x4 matrix
 * @returns {{ x: number, y: number, z: number }} in meters
 */
function positionFromMatrix(T) {
    return {
        x: T[0][3],
        y: T[1][3],
        z: T[2][3]
    };
}

/**
 * Extracts the tool pointing direction from a 4x4 transform matrix.
 * In our URDF the tool is at origin xyz="0.042 0 0" (along link5 +X), so the
 * tool direction is the frame's X-axis (first column), not Z. Using Z would
 * put joint 4 (wrist roll) 90° out of alignment.
 * @param {Array} T - 4x4 matrix (row-major, upper-left 3x3 = rotation)
 * @returns {{ x: number, y: number, z: number }}
 */
function toolZAxisFromMatrix(T) {
    return {
        x: T[0][0],
        y: T[1][0],
        z: T[2][0]
    };
}

/**
 * Normalises a 3D vector. If the length is very small, returns a default
 * pointing straight down (0, 0, -1) so we never divide by zero.
 * @param {{ x: number, y: number, z: number }} v
 * @returns {{ x: number, y: number, z: number }}
 */
function normalizeVector(v) {
    const x = typeof v.x === 'number' ? v.x : 0;
    const y = typeof v.y === 'number' ? v.y : 0;
    const z = typeof v.z === 'number' ? v.z : 0;
    const len = Math.sqrt(x * x + y * y + z * z);
    if (len < 1e-6) {
        return { x: 0, y: 0, z: -1 };
    }
    return { x: x / len, y: y / len, z: z / len };
}

/**
 * Robot Arm Kinematics Class
 * Handles forward and inverse kinematics for a multi-joint robot arm using URDF
 */
class RobotKinematics {
    constructor() {
        // URDF robot structure
        this.urdfData = null;
        // Array of revolute joints (excluding fixed joints) in order
        this.joints = [];
        // Controls whether forwardKinematics prints debug logs
        this.enableDebugLogging = true;
        // Approximate maximum reach of the robot in mm (computed from URDF)
        this.maxReachMm = null;
    }

    /**
     * Loads URDF XML and parses it
     * @param {string} urdfXml - URDF XML string
     */
    loadURDF(urdfXml) {
        if (typeof parseURDF === 'undefined') {
            throw new Error('URDF parser not loaded. Make sure urdfParser.js is included.');
        }
        
        this.urdfData = parseURDF(urdfXml);
        
        // Extract only revolute joints (in order, excluding fixed joints)
        this.joints = this.urdfData.joints.filter(joint => joint.type === 'revolute');
        
        // Fixed joints after the last revolute (e.g. tool_fixed) are applied so FK returns tool-tip position
        const lastRevoluteChild = this.joints.length > 0 ? this.joints[this.joints.length - 1].child : null;
        this.fixedToolJoints = (this.urdfData.joints || []).filter(
            (j) => j.type === 'fixed' && j.parent === lastRevoluteChild
        );
        
        console.log(`Kinematics: Loaded URDF with ${this.joints.length} revolute joints`);
        this.joints.forEach(function (j, idx) {
            if (j && typeof j.zeroOffsetDegrees === 'number') {
                console.log('Kinematics: joint', idx, '"' + (j.name || '?') + '" zeroOffsetDegrees =', j.zeroOffsetDegrees);
            }
        });

        // Estimate maximum reach from joint origins (very simple approximation)
        let totalLengthMm = 0;
        this.joints.forEach(function (joint) {
            if (joint && joint.origin) {
                const ox = joint.origin.x || 0;
                const oy = joint.origin.y || 0;
                const oz = joint.origin.z || 0;
                const segMeters = Math.sqrt(ox * ox + oy * oy + oz * oz);
                totalLengthMm += segMeters * 1000;
            }
        });
        if (this.fixedToolJoints && this.fixedToolJoints.length > 0) {
            this.fixedToolJoints.forEach(function (joint) {
                if (joint && joint.origin) {
                    const ox = joint.origin.x || 0;
                    const oy = joint.origin.y || 0;
                    const oz = joint.origin.z || 0;
                    const segMeters = Math.sqrt(ox * ox + oy * oy + oz * oz);
                    totalLengthMm += segMeters * 1000;
                }
            });
        }
        // Add a small safety margin
        this.maxReachMm = totalLengthMm * 1.05;
        console.log('Kinematics: approximate max reach =', this.maxReachMm.toFixed(1), 'mm');
    }

    /**
     * Sets joint configurations from URDF data
     * This is a compatibility method - it accepts the old DH-style configs
     * but converts them to URDF format internally
     * 
     * @param {Array} configs - Array of joint configurations (for backward compatibility)
     */
    setJointConfigurations(configs) {
        // This method is kept for backward compatibility
        // If configs are provided, we'll create a simple URDF structure
        console.warn('setJointConfigurations() called - please use loadURDF() instead for URDF-based kinematics');
        this.jointConfigs = configs; // Keep for backward compatibility
    }

    /**
     * Gets the number of revolute joints
     * @returns {number} Number of revolute joints
     */
    getJointCount() {
        return this.joints.length;
    }

    /**
     * Checks if kinematics is configured
     * @returns {boolean} True if URDF is loaded
     */
    isConfigured() {
        return this.joints.length > 0;
    }

    /**
     * Forward kinematics: compute end effector pose from joint angles using
     * standard 4x4 matrix chain. T_world_ee = T_0_1 * T_1_2 * ... * T_(n-1)_n.
     * Each joint: T_i = Origin_i * Rotation_i(axis, angle). XYZ from last column.
     *
     * @param {Array} jointAngles - Joint angles in degrees (one per revolute joint)
     * @returns {Object} { position: { x, y, z } in mm, rotation: 4x4 matrix }
     */
    forwardKinematics(jointAngles) {
        if (!this.isConfigured()) {
            throw new Error('URDF not loaded. Call loadURDF() first.');
        }
        if (jointAngles.length !== this.joints.length) {
            throw new Error(`Number of joint angles (${jointAngles.length}) doesn't match number of joints (${this.joints.length})`);
        }

        // --- Console logging for debugging / comparison (can be turned off by inverseKinematics) ---
        if (this.enableDebugLogging) {
            console.log('=== Forward Kinematics ===');
            console.log('Input angles (degrees):', jointAngles.map((a, i) => {
                const name = this.joints[i] && this.joints[i].name ? this.joints[i].name : `J${i + 1}`;
                const offset = (this.joints[i] && typeof this.joints[i].zeroOffsetDegrees === 'number')
                    ? this.joints[i].zeroOffsetDegrees : 0;
                const used = (jointAngles[i] || 0) + offset;
                return `${name}=${(jointAngles[i] || 0).toFixed(1)}° (used: ${used.toFixed(1)}°)`;
            }).join(', '));
        }

        // T = cumulative transform from base to current link (row-major 4x4)
        let T = identity4x4();

        for (let i = 0; i < this.joints.length; i++) {
            const joint = this.joints[i];
            let angleDeg = jointAngles[i] || 0;
            if (typeof joint.zeroOffsetDegrees === 'number') {
                angleDeg = angleDeg + joint.zeroOffsetDegrees;
            }
            const angleRad = (angleDeg * Math.PI) / 180;

            // T_joint = origin (link offset) then rotation about joint axis
            const T_origin = originToMatrix(joint.origin);
            const T_rotation = axisRotationMatrix(joint.axis, angleRad);
            const T_joint = multiplyMatrices(T_origin, T_rotation);

            // Chain: T = T * T_joint
            T = multiplyMatrices(T, T_joint);

            // Log cumulative position after each joint (in mm)
            if (this.enableDebugLogging) {
                const pStep = positionFromMatrix(T);
                const name = joint.name || `J${i + 1}`;
                const ax = (joint.axis && typeof joint.axis.x === 'number') ? joint.axis.x : 0;
                const ay = (joint.axis && typeof joint.axis.y === 'number') ? joint.axis.y : 0;
                const az = (joint.axis && typeof joint.axis.z === 'number') ? joint.axis.z : 0;
                console.log(
                    `  After ${name}: origin=(${(joint.origin.x || 0).toFixed(4)}, ` +
                    `${(joint.origin.y || 0).toFixed(4)}, ${(joint.origin.z || 0).toFixed(4)}) m, ` +
                    `axis=(${ax}, ${ay}, ${az}), angle=${angleDeg.toFixed(2)}° → ` +
                    `position = (${(pStep.x * 1000).toFixed(2)}, ${(pStep.y * 1000).toFixed(2)}, ` +
                    `${(pStep.z * 1000).toFixed(2)}) mm`
                );
            }
        }

        // Apply fixed tool joint(s) so the reported position is the actual end tool tip (e.g. tool_link)
        if (this.fixedToolJoints && this.fixedToolJoints.length > 0) {
            for (let f = 0; f < this.fixedToolJoints.length; f++) {
                const toolOrigin = this.fixedToolJoints[f].origin;
                if (this.enableDebugLogging) {
                    console.log(`  Fixed tool "${this.fixedToolJoints[f].name || 'tool'}": origin=(${(toolOrigin.x || 0).toFixed(4)}, ${(toolOrigin.y || 0).toFixed(4)}, ${(toolOrigin.z || 0).toFixed(4)}) m`);
                }
                const T_tool = originToMatrix(toolOrigin);
                T = multiplyMatrices(T, T_tool);
            }
        }

        // Standard: position in meters is column 3 (rows 0,1,2). Convert to mm.
        const pMeters = positionFromMatrix(T);
        const position = {
            x: pMeters.x * 1000,
            y: pMeters.y * 1000,
            z: pMeters.z * 1000
        };

        return {
            position: position,
            rotation: T
        };
    }

    /**
     * Returns step-by-step forward kinematics data for each joint.
     * This is used by the UI to display the cumulative transform matrix at each stage.
     *
     * @param {Array} jointAngles - Joint angles in degrees (one per revolute joint)
     * @returns {Object} { steps: Array<{ index, name, angleInput, angleUsed, transform }>, finalTransform }
     */
    getForwardKinematicsSteps(jointAngles) {
        if (!this.isConfigured()) {
            throw new Error('URDF not loaded. Call loadURDF() first.');
        }
        if (!jointAngles || jointAngles.length !== this.joints.length) {
            throw new Error(`Number of joint angles (${jointAngles ? jointAngles.length : 0}) doesn't match number of joints (${this.joints.length})`);
        }

        const steps = [];
        let T = identity4x4();

        for (let i = 0; i < this.joints.length; i++) {
            const joint = this.joints[i];
            const inputAngleDeg = jointAngles[i] || 0;
            let angleDeg = inputAngleDeg;
            if (typeof joint.zeroOffsetDegrees === 'number') {
                angleDeg = angleDeg + joint.zeroOffsetDegrees;
            }
            const angleRad = (angleDeg * Math.PI) / 180;

            const T_origin = originToMatrix(joint.origin);
            const T_rotation = axisRotationMatrix(joint.axis, angleRad);
            const T_joint = multiplyMatrices(T_origin, T_rotation);

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

        // Apply fixed tool joints as additional step(s)
        if (this.fixedToolJoints && this.fixedToolJoints.length > 0) {
            for (let f = 0; f < this.fixedToolJoints.length; f++) {
                const toolJoint = this.fixedToolJoints[f];
                const T_tool = originToMatrix(toolJoint.origin);
                T = multiplyMatrices(T, T_tool);

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
        }

        return {
            steps: steps,
            finalTransform: T
        };
    }

    /**
     * Inverse Kinematics
     * Calculates joint angles from end effector position
     * 
     * This is a placeholder - full IK implementation would require
     * solving the inverse kinematics equations for the specific robot geometry
     * 
     * @param {Object} targetPose - Target pose: { x: mm, y: mm, z: mm, orientation?: { x, y, z } }
     * @param {Array|null} initialAngles - Optional starting guess for joint angles (degrees)
     * @returns {Array|null} Array of joint angles in degrees, or null if it fails to find a solution
     */
    inverseKinematics(targetPose, initialAngles) {
        if (!this.isConfigured()) {
            throw new Error('URDF not loaded. Call loadURDF() first.');
        }

        const numJoints = this.joints.length;

        // Decide if we also have a desired tool orientation (direction vector)
        let hasOrientationTarget = false;
        let desiredToolZ = { x: 0, y: 0, z: -1 };
        if (targetPose && targetPose.orientation) {
            desiredToolZ = normalizeVector(targetPose.orientation);
            hasOrientationTarget = true;
        }

        // Quick reachability check using approximate maximum reach
        if (this.maxReachMm && targetPose && typeof targetPose.x === 'number') {
            const tx = targetPose.x || 0;
            const ty = targetPose.y || 0;
            const tz = targetPose.z || 0;
            const distance = Math.sqrt(tx * tx + ty * ty + tz * tz);
            if (distance > this.maxReachMm + 10) {
                console.warn('Inverse kinematics: target outside approximate reach. Distance =', distance.toFixed(1), 'mm, maxReach ≈', this.maxReachMm.toFixed(1), 'mm');
                return null;
            }
        }

        // Simple numeric IK using Jacobian transpose.
        // This is a beginner-friendly implementation:
        // 1. Start from an initial guess for the joint angles.
        // 2. Use forward kinematics to see where the end effector is.
        // 3. Estimate how changing each joint changes XYZ (Jacobian).
        // 4. Nudge the angles a little bit in the direction that reduces the XYZ error.
        // 5. Repeat until the error is small or we hit a maximum number of steps.

        // Build starting angles array
        const angles = [];
        for (let i = 0; i < numJoints; i++) {
            if (initialAngles && Array.isArray(initialAngles) && typeof initialAngles[i] === 'number') {
                angles.push(initialAngles[i]);
            } else {
                angles.push(0);
            }
        }

        // Settings for the solver
        const maxIterations = 400;
        const positionToleranceMm = 1.0; // stop main loop when we are within 1mm
        const finiteDifferenceDeg = 1.0; // small angle change when estimating Jacobian
        const baseStepSize = 0.03;       // baseline strength for the Jacobian transpose update
        const orientationWeight = 10.0;  // how strongly to try to match orientation (reduced for stability)

        // We temporarily disable verbose FK logging while we iterate
        const previousLogging = this.enableDebugLogging;
        this.enableDebugLogging = false;

        try {
            for (let iter = 0; iter < maxIterations; iter++) {
                // Where are we now?
                const fkResult = this.forwardKinematics(angles);
                const currentPos = fkResult.position; // in mm
                const currentToolZ = toolZAxisFromMatrix(fkResult.rotation);

                // Calculate XYZ error (target - current)
                const errX = targetPose.x - currentPos.x;
                const errY = targetPose.y - currentPos.y;
                const errZ = targetPose.z - currentPos.z;

                const positionErrorLength = Math.sqrt(errX * errX + errY * errY + errZ * errZ);

                // Optional orientation error (desired tool Z direction - current)
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

                // If position is close enough, and either we don't care about orientation
                // or the orientation error is already small enough, stop iterating.
                if (positionErrorLength < positionToleranceMm) {
                    if (!hasOrientationTarget || orientationErrorLength < 0.1) {
                        break;
                    }
                }

                // Build numeric Jacobian for position (3 x numJoints): how XYZ changes per degree
                const Jpos = [];
                // Row 0: dX/dTheta_j, Row 1: dY/dTheta_j, Row 2: dZ/dTheta_j
                Jpos[0] = new Array(numJoints).fill(0);
                Jpos[1] = new Array(numJoints).fill(0);
                Jpos[2] = new Array(numJoints).fill(0);

                // If we care about orientation, we also build a simple numeric Jacobian
                // for the tool Z-axis direction: how (Zx, Zy, Zz) change per degree.
                const Jori = hasOrientationTarget ? [
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0)
                ] : null;

                for (let j = 0; j < numJoints; j++) {
                    const originalAngle = angles[j];
                    // Slightly change this one joint
                    angles[j] = originalAngle + finiteDifferenceDeg;
                    const fkPlus = this.forwardKinematics(angles);
                    const posPlus = fkPlus.position;
                    const toolZPlus = toolZAxisFromMatrix(fkPlus.rotation);
                    // Restore
                    angles[j] = originalAngle;

                    // Approximate position derivative: (f(theta+delta) - f(theta)) / delta
                    Jpos[0][j] = (posPlus.x - currentPos.x) / finiteDifferenceDeg;
                    Jpos[1][j] = (posPlus.y - currentPos.y) / finiteDifferenceDeg;
                    Jpos[2][j] = (posPlus.z - currentPos.z) / finiteDifferenceDeg;

                    // Approximate orientation derivative (tool Z axis) if needed
                    if (hasOrientationTarget && Jori) {
                        Jori[0][j] = (toolZPlus.x - currentToolZ.x) / finiteDifferenceDeg;
                        Jori[1][j] = (toolZPlus.y - currentToolZ.y) / finiteDifferenceDeg;
                        Jori[2][j] = (toolZPlus.z - currentToolZ.z) / finiteDifferenceDeg;
                    }
                }

                // Use Jacobian transpose to compute angle updates
                // We scale the step by the error length so that:
                // - when we are far away, the step is gentle
                // - when we get close, the step can be a bit stronger
                const stepSize = baseStepSize / (1 + positionErrorLength / 50);
                // deltaTheta_j = stepSize * (Jpos^T * positionError + orientation term)
                for (let j = 0; j < numJoints; j++) {
                    let grad =
                        Jpos[0][j] * errX +
                        Jpos[1][j] * errY +
                        Jpos[2][j] * errZ;

                    if (hasOrientationTarget && Jori) {
                        grad += orientationWeight * (
                            Jori[0][j] * oriErrX +
                            Jori[1][j] * oriErrY +
                            Jori[2][j] * oriErrZ
                        );
                    }

                    const deltaAngle = stepSize * grad;
                    angles[j] += deltaAngle;

                    // Clamp to joint limits if provided (in degrees)
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
        } finally {
            // Restore logging setting
            this.enableDebugLogging = previousLogging;
        }

        // Simple sanity check: make sure we got finite numbers
        for (let i = 0; i < numJoints; i++) {
            if (!isFinite(angles[i])) {
                console.warn('Inverse kinematics failed: non-finite angle found');
                return null;
            }
        }

        // Final accuracy check: if we are still far away, report failure
        try {
            const finalFk = this.forwardKinematics(angles);
            const p = finalFk.position;
            const dx = p.x - targetPose.x;
            const dy = p.y - targetPose.y;
            const dz = p.z - targetPose.z;
            const finalError = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (finalError > 10.0) {
                console.warn('Inverse kinematics could not reach target within 10mm. Final error =', finalError.toFixed(2), 'mm');
                return null;
            }

            // If we also had an orientation target, log how close we got.
            if (hasOrientationTarget) {
                const finalToolZ = toolZAxisFromMatrix(finalFk.rotation);
                const dZx = desiredToolZ.x - finalToolZ.x;
                const dZy = desiredToolZ.y - finalToolZ.y;
                const dZz = desiredToolZ.z - finalToolZ.z;
                const oriErrorLength = Math.sqrt(dZx * dZx + dZy * dZy + dZz * dZz);

                // Cosine of angle between desired and achieved Z axes
                const dot =
                    desiredToolZ.x * finalToolZ.x +
                    desiredToolZ.y * finalToolZ.y +
                    desiredToolZ.z * finalToolZ.z;
                const clampedDot = Math.max(-1, Math.min(1, dot));
                const angleDeg = (Math.acos(clampedDot) * 180) / Math.PI;

                console.log(
                    'IK orientation summary: desiredZ=',
                    desiredToolZ,
                    ' finalZ=',
                    finalToolZ,
                    ' |ΔZ|=',
                    oriErrorLength.toFixed(3),
                    ' angle error≈',
                    angleDeg.toFixed(1),
                    'deg'
                );
            }
        } catch (e) {
            console.warn('Inverse kinematics final error check failed:', e);
        }

        return angles;
    }

    /**
     * Refines the pose by searching around the base joint angles for a better
     * compromise between position and tool orientation.
     *
     * For a 5-axis arm we primarily adjust joints 2, 3, 4, and 5 (shoulder,
     * elbow, wrist roll, wrist pitch) in small steps around the base solution.
     * This keeps the code simple and beginner-friendly while still allowing
     * the solver to "meet in the middle" when both XYZ and orientation matter.
     *
     * @param {{ x:number, y:number, z:number }} targetPose - desired XYZ in mm
     * @param {Array<number>} baseAngles - joint angles from a position-only IK (degrees)
     * @param {{ x:number, y:number, z:number }} desiredOrientation - desired tool Z-axis direction
     * @returns {Array<number>} refined joint angles (may be the same as baseAngles)
     */
    refineOrientationWithWrist(targetPose, baseAngles, desiredOrientation) {
        if (!this.isConfigured()) {
            return baseAngles;
        }
        if (!baseAngles || !Array.isArray(baseAngles)) {
            return baseAngles;
        }
        if (!desiredOrientation) {
            return baseAngles;
        }

        const numJoints = this.joints.length;
        // We need at least a shoulder, elbow, and two wrist joints to adjust
        if (numJoints < 3) {
            return baseAngles;
        }

        // For a typical 5-axis arm:
        //  index 0 = base yaw
        //  index 1 = shoulder pitch
        //  index 2 = elbow pitch
        //  index 3 = wrist roll
        //  index 4 = wrist pitch
        // We will adjust joints 1–(numJoints-1), leaving only the base fixed.
        const firstAdjustable = 1;
        const lastAdjustable = numJoints - 1;

        const desiredZ = normalizeVector(desiredOrientation);

        // Helper: clamp an angle to joint limits if present
        const clampToLimits = (angleDeg, joint) => {
            let a = angleDeg;
            if (joint && joint.limits) {
                if (typeof joint.limits.lowerDegrees === 'number' && a < joint.limits.lowerDegrees) {
                    a = joint.limits.lowerDegrees;
                }
                if (typeof joint.limits.upperDegrees === 'number' && a > joint.limits.upperDegrees) {
                    a = joint.limits.upperDegrees;
                }
            }
            return a;
        };

        // Helper: evaluate how good a particular set of angles is
        const evaluateCandidate = (angles) => {
            try {
                const fk = this.forwardKinematics(angles);
                const pos = fk.position;
                const toolZ = toolZAxisFromMatrix(fk.rotation);

                const dx = pos.x - targetPose.x;
                const dy = pos.y - targetPose.y;
                const dz = pos.z - targetPose.z;
                const positionErrorMm = Math.sqrt(dx * dx + dy * dy + dz * dz);

                const dot =
                    desiredZ.x * toolZ.x +
                    desiredZ.y * toolZ.y +
                    desiredZ.z * toolZ.z;
                const clampedDot = Math.max(-1, Math.min(1, dot));
                const orientationErrorDeg = (Math.acos(clampedDot) * 180) / Math.PI;

                // We mainly care about position, but we also reward better orientation.
                // Here we treat 1 degree of orientation error as roughly 1mm.
                const orientationWeightMmPerDeg = 1.0;
                const score = positionErrorMm + orientationWeightMmPerDeg * orientationErrorDeg;

                return {
                    score: score,
                    positionErrorMm: positionErrorMm,
                    orientationErrorDeg: orientationErrorDeg,
                    toolZ: toolZ,
                    achievedPosition: { x: pos.x, y: pos.y, z: pos.z }
                };
            } catch (e) {
                console.warn('refineOrientationWithWrist: FK failed for candidate angles:', e);
                return null;
            }
        };

        // One pass of grid search with given offset arrays and position cap.
        const runOnePass = (currentBase, maxPositionErrorMm, shoulderOffs, elbowOffs, wristRollOffs, wristPitchOffs) => {
            let bestAngles = currentBase.slice();
            let bestEval = evaluateCandidate(bestAngles);
            if (!bestEval) {
                return { angles: bestAngles, positionErrorMm: Infinity, orientationErrorDeg: Infinity, achievedPosition: null };
            }

            for (let s = 0; s < shoulderOffs.length; s++) {
                for (let e = 0; e < elbowOffs.length; e++) {
                    for (let r = 0; r < wristRollOffs.length; r++) {
                        for (let p = 0; p < wristPitchOffs.length; p++) {
                            const candidateAngles = currentBase.slice();
                            if (numJoints > 1) {
                                candidateAngles[1] = clampToLimits(currentBase[1] + shoulderOffs[s], this.joints[1]);
                            }
                            if (numJoints > 2) {
                                candidateAngles[2] = clampToLimits(currentBase[2] + elbowOffs[e], this.joints[2]);
                            }
                            if (numJoints > 3) {
                                candidateAngles[3] = clampToLimits(currentBase[3] + wristRollOffs[r], this.joints[3]);
                            }
                            if (numJoints > 4) {
                                candidateAngles[4] = clampToLimits(currentBase[4] + wristPitchOffs[p], this.joints[4]);
                            }

                            const evalResult = evaluateCandidate(candidateAngles);
                            if (!evalResult || evalResult.positionErrorMm > maxPositionErrorMm) {
                                continue;
                            }
                            if (evalResult.score < bestEval.score) {
                                bestEval = evalResult;
                                bestAngles = candidateAngles.slice();
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

        // Use the shared iterative refinement; return only angles for backward compatibility.
        const result = this.refineOrientationWithAccuracy(targetPose, baseAngles, desiredOrientation);
        console.log(
            'refineOrientationWithWrist: positionError=',
            result.positionErrorMm.toFixed(2),
            'mm, orientationError≈',
            result.orientationErrorDeg.toFixed(1),
            'deg, achieved XYZ=',
            result.achievedPosition ? `(${result.achievedPosition.x.toFixed(1)}, ${result.achievedPosition.y.toFixed(1)}, ${result.achievedPosition.z.toFixed(1)})` : '-'
        );
        return result.angles;
    }

    /**
     * Iterative refinement: starts coarse (tool down, ~20mm) then tightens over 4 passes
     * down to ~2mm position cap. Returns angles and accuracy for reporting.
     *
     * @param {{ x:number, y:number, z:number }} targetPose - desired XYZ in mm
     * @param {Array<number>} baseAngles - joint angles from position-only IK (degrees)
     * @param {{ x:number, y:number, z:number }} desiredOrientation - desired tool Z-axis direction
     * @returns {{ angles: Array<number>, positionErrorMm: number, orientationErrorDeg: number, achievedPosition: {x,y,z}|null }}
     */
    refineOrientationWithAccuracy(targetPose, baseAngles, desiredOrientation) {
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
            return { angles: baseAngles.slice(), positionErrorMm: Infinity, orientationErrorDeg: Infinity, achievedPosition: null };
        }

        const desiredZ = normalizeVector(desiredOrientation);
        const clampToLimits = (angleDeg, joint) => {
            let a = angleDeg;
            if (joint && joint.limits) {
                if (typeof joint.limits.lowerDegrees === 'number' && a < joint.limits.lowerDegrees) {
                    a = joint.limits.lowerDegrees;
                }
                if (typeof joint.limits.upperDegrees === 'number' && a > joint.limits.upperDegrees) {
                    a = joint.limits.upperDegrees;
                }
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
                const score = positionErrorMm + orientationWeightForScore * orientationErrorDeg;
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

        // baseYawOffs: optional array of offsets (degrees) for joint 0 so we try both sides of the workspace (e.g. [0, -180, 180, -90, 90])
        const runOnePass = (currentBase, maxPositionErrorMm, baseYawOffs, shoulderOffs, elbowOffs, wristRollOffs, wristPitchOffs) => {
            const baseOffs = Array.isArray(baseYawOffs) && baseYawOffs.length > 0 ? baseYawOffs : [0];
            let bestAngles = currentBase.slice();
            let bestEval = evaluateCandidate(bestAngles);
            if (!bestEval) {
                return { angles: bestAngles, positionErrorMm: Infinity, orientationErrorDeg: Infinity, achievedPosition: null };
            }
            for (let b = 0; b < baseOffs.length; b++) {
                for (let s = 0; s < shoulderOffs.length; s++) {
                    for (let e = 0; e < elbowOffs.length; e++) {
                        for (let r = 0; r < wristRollOffs.length; r++) {
                            for (let p = 0; p < wristPitchOffs.length; p++) {
                                const candidateAngles = currentBase.slice();
                                if (numJoints > 0) candidateAngles[0] = clampToLimits(currentBase[0] + baseOffs[b], this.joints[0]);
                                if (numJoints > 1) candidateAngles[1] = clampToLimits(currentBase[1] + shoulderOffs[s], this.joints[1]);
                                if (numJoints > 2) candidateAngles[2] = clampToLimits(currentBase[2] + elbowOffs[e], this.joints[2]);
                                if (numJoints > 3) candidateAngles[3] = clampToLimits(currentBase[3] + wristRollOffs[r], this.joints[3]);
                                if (numJoints > 4) candidateAngles[4] = clampToLimits(currentBase[4] + wristPitchOffs[p], this.joints[4]);
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
            return {
                angles: bestAngles,
                positionErrorMm: bestEval.positionErrorMm,
                orientationErrorDeg: bestEval.orientationErrorDeg,
                achievedPosition: bestEval.achievedPosition
            };
        };

        // Wrist roll (joint 4) has ±180° in URDF; wrist pitch (joint 5) ±90°. Search full range so tool-down is findable.
        const wristRollFull = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180];
        const wristPitchFull = [0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90];
        // Fine wrist-only grid (degrees) for a dedicated orientation sweep
        const wristRollFine = [];
        for (let d = -180; d <= 180; d += 15) wristRollFine.push(d);
        const wristPitchFine = [];
        for (let d = -90; d <= 90; d += 10) wristPitchFine.push(d);

        // Base yaw (joint 0) offsets so we try both sides of the workspace — avoids getting stuck when moving e.g. 250,0,140 → -250,0,140
        const baseYawTryBothSides = [0, -180, 180, -90, 90];

        // Pass 0: orientation-first; include base yaw so we pick the correct side (e.g. base ~180° for X < 0)
        orientationWeightForScore = 5.0;
        const pass0 = runOnePass(baseAngles, 25, baseYawTryBothSides, [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20, -30, 30], wristRollFull, wristPitchFull);
        // Pass 0b: wrist-only fine search (shoulder/elbow fixed) to nail tool orientation; base fixed
        const pass0b = runOnePass(pass0.angles, 25, [0], [0], [0], wristRollFine, wristPitchFine);
        orientationWeightForScore = 1.0;
        // Passes 1–4: refine position from that seed; base fixed so we don't flip side
        const pass1 = runOnePass(pass0b.angles, 20, [0], [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20, -30, 30], wristRollFull, wristPitchFull);
        const pass2 = runOnePass(pass1.angles, 12, [0], [0, -8, 8, -15, 15], [0, -8, 8, -15, 15], [0, -15, 15, -30, 30, -45, 45], [0, -15, 15, -30, 30, -45, 45]);
        const pass3 = runOnePass(pass2.angles, 6, [0], [0, -5, 5, -10, 10], [0, -5, 5, -10, 10], [0, -10, 10, -20, 20, -30, 30], [0, -10, 10, -20, 20, -30, 30]);
        const pass4 = runOnePass(pass3.angles, 2, [0], [0, -2, 2, -5, 5], [0, -2, 2, -5, 5], [0, -5, 5, -10, 10], [0, -5, 5, -10, 10]);
        // Pass 5: position polish — fine local search; include small base yaw nudge (±5°) in case we're a few degrees off
        const pass5 = runOnePass(pass4.angles, 12, [0, -5, 5], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10], [0, -3, 3, -6, 6, -10, 10]);
        // Pass 6: really fine adjustment — ±0.5°, ±1°, ±1.5°, ±2°; include base ±1°, ±2°
        const pass6 = runOnePass(pass5.angles, 15, [0, -1, 1, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2], [0, -0.5, 0.5, -1, 1, -1.5, 1.5, -2, 2]);

        return {
            angles: pass6.angles,
            positionErrorMm: pass6.positionErrorMm,
            orientationErrorDeg: pass6.orientationErrorDeg,
            achievedPosition: pass6.achievedPosition
        };
    }

    /**
     * Gets joint configuration (for backward compatibility)
     * @param {number} jointIndex - Joint index (0-based)
     * @returns {Object|null} Joint configuration or null if not found
     */
    getJointConfiguration(jointIndex) {
        if (jointIndex < 0 || jointIndex >= this.joints.length) {
            return null;
        }
        return this.joints[jointIndex];
    }

    /**
     * Gets all joint configurations (for backward compatibility)
     * @returns {Array} Array of joint configurations
     */
    getJointConfigs() {
        return this.joints;
    }
}

// Create a global instance
const robotKinematics = new RobotKinematics();
