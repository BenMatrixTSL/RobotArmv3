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
    // Tool mounts along -Z of the J6 frame (xyz="0 0 -0.035" in URDF tool_mount).
    // At home (identity rotation) this gives {0,0,-1} = tool pointing down, matching physical reality.
    return {
        x: -T[0][2],
        y: -T[1][2],
        z: -T[2][2]
    };
}

/**
 * Extracts the tool X-axis from a 4x4 transform matrix (first column of rotation part).
 * @param {Array} T - 4x4 matrix (row-major, upper-left 3x3 = rotation)
 * @returns {{ x: number, y: number, z: number }}
 */
function toolXAxisFromMatrix(T) {
    return { x: T[0][0], y: T[1][0], z: T[2][0] };
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
 * Builds a consistent tool orientation frame from a desired Z-axis and a rotation angle.
 * Rotation=0 gives a repeatable reference X-axis regardless of where the arm is.
 * Increasing rotation spins the tool around its pointing axis (right-hand rule).
 *
 * @param {{ x, y, z }} desiredZ - normalised tool Z-axis (pointing direction)
 * @param {number} rotationDeg - rotation around the tool Z-axis in degrees
 * @returns {{ xAxis: {x,y,z}, yAxis: {x,y,z}, zAxis: {x,y,z} }}
 */
function buildToolFrame(desiredZ, rotationDeg) {
    const z = normalizeVector(desiredZ);
    // Choose a world reference not parallel to z, then Gram-Schmidt orthogonalise
    const ref = (Math.abs(z.z) < 0.9) ? { x: 0, y: 0, z: 1 } : { x: 1, y: 0, z: 0 };
    const dot = ref.x*z.x + ref.y*z.y + ref.z*z.z;
    const xRef = normalizeVector({ x: ref.x - dot*z.x, y: ref.y - dot*z.y, z: ref.z - dot*z.z });
    // yRef = z × xRef
    const yRef = {
        x: z.y*xRef.z - z.z*xRef.y,
        y: z.z*xRef.x - z.x*xRef.z,
        z: z.x*xRef.y - z.y*xRef.x
    };
    // Rotate xRef around z by rotationDeg
    const c = Math.cos(rotationDeg * Math.PI / 180);
    const s = Math.sin(rotationDeg * Math.PI / 180);
    return {
        xAxis: { x: c*xRef.x - s*yRef.x, y: c*xRef.y - s*yRef.y, z: c*xRef.z - s*yRef.z },
        yAxis: { x: s*xRef.x + c*yRef.x, y: s*xRef.y + c*yRef.y, z: s*xRef.z + c*yRef.z },
        zAxis: z
    };
}

// ---------------------------------------------------------------------------
// Null-space IK helpers
// ---------------------------------------------------------------------------

/**
 * Inverts a 3×3 matrix using cofactors. Returns null if singular.
 * @param {number[][]} m - 3×3 matrix (array of 3 rows)
 * @returns {number[][]|null}
 */
function invertMat3(m) {
    const a=m[0][0], b=m[0][1], c=m[0][2];
    const d=m[1][0], e=m[1][1], f=m[1][2];
    const g=m[2][0], h=m[2][1], k=m[2][2];
    const det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g);
    if (Math.abs(det) < 1e-9) return null;
    const inv = 1.0 / det;
    return [
        [(e*k-f*h)*inv, (c*h-b*k)*inv, (b*f-c*e)*inv],
        [(f*g-d*k)*inv, (a*k-c*g)*inv, (c*d-a*f)*inv],
        [(d*h-e*g)*inv, (b*g-a*h)*inv, (a*e-b*d)*inv]
    ];
}

/**
 * Damped-least-squares pseudoinverse of a 3×N Jacobian.
 * J_pinv = J^T (J J^T + λ²I)^{-1}
 *
 * @param {number[][]} J  - 3-element array, each row is an N-element array
 * @param {number}     n  - number of columns (joints)
 * @param {number}     lambda - damping factor
 * @returns {number[][]|null} N×3 matrix, or null if inversion fails
 */
function dampedPseudoinverse3xN(J, n, lambda) {
    // JJT = J * J^T  (3×3)
    const JJT = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++) {
        for (let k = 0; k < 3; k++) {
            let s = 0;
            for (let j = 0; j < n; j++) s += J[i][j] * J[k][j];
            JJT[i][k] = s;
        }
    }
    const lsq = lambda * lambda;
    JJT[0][0] += lsq; JJT[1][1] += lsq; JJT[2][2] += lsq;

    const inv = invertMat3(JJT);
    if (!inv) return null;

    // J_pinv = J^T * inv(JJT)  (N×3)
    const pinv = [];
    for (let j = 0; j < n; j++) {
        pinv.push([
            J[0][j]*inv[0][0] + J[1][j]*inv[1][0] + J[2][j]*inv[2][0],
            J[0][j]*inv[0][1] + J[1][j]*inv[1][1] + J[2][j]*inv[2][1],
            J[0][j]*inv[0][2] + J[1][j]*inv[1][2] + J[2][j]*inv[2][2]
        ]);
    }
    return pinv;
}

/**
 * Multiplies an N×3 matrix by a 3-element vector, returning an N-element vector.
 * @param {number[][]} M - N×3
 * @param {number[]}   v - length 3
 * @returns {number[]} length N
 */
function matVec3(M, v) {
    return M.map(row => row[0]*v[0] + row[1]*v[1] + row[2]*v[2]);
}

/**
 * Projects vector g (length N) into the null space of J_pos (3×N).
 * (I - J_pos_pinv * J_pos) * g  — computed without forming the N×N matrix:
 *   result = g - J_pos_pinv * (J_pos * g)
 *
 * @param {number[][]} Jpos     - 3×N position Jacobian
 * @param {number[][]} Jpos_pinv - N×3 pseudoinverse
 * @param {number[]}   g        - length-N vector to project
 * @param {number}     n        - number of joints
 * @returns {number[]} length-N null-space projected vector
 */
function nullSpaceProject(Jpos, Jpos_pinv, g, n) {
    // h = J_pos * g  (3×1)
    const h = [0, 0, 0];
    for (let j = 0; j < n; j++) {
        h[0] += Jpos[0][j] * g[j];
        h[1] += Jpos[1][j] * g[j];
        h[2] += Jpos[2][j] * g[j];
    }
    // J_pos_pinv * h  (N×1)
    const Jph = matVec3(Jpos_pinv, h);
    // g - J_pos_pinv * J_pos * g
    return g.map((gi, i) => gi - Jph[i]);
}

// ---------------------------------------------------------------------------

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
        // End tools declared in the URDF, keyed by the tool type ID the ESP32
        // end tool reports in register 3. activeEndToolId picks which one FK
        // applies, and the Pi is what tells us that ID.
        this.endTools = {};
        this.activeEndToolId = 0;
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
        this.baseLengthMm = totalLengthMm;

        // Collect the end tools the URDF describes
        this.endTools = {};
        (this.urdfData.joints || []).forEach((j) => {
            if (j.endTool && typeof j.endTool.id === 'number') {
                const ox = j.origin.x || 0;
                const oy = j.origin.y || 0;
                const oz = j.origin.z || 0;
                this.endTools[j.endTool.id] = {
                    id: j.endTool.id,
                    label: j.endTool.label,
                    controls: j.endTool.controls,
                    provisional: !!j.endTool.provisional,
                    jointName: j.name,
                    origin: j.origin,
                    offsetMm: { x: ox * 1000, y: oy * 1000, z: oz * 1000 },
                    lengthMm: Math.sqrt(ox * ox + oy * oy + oz * oz) * 1000
                };
            }
        });
        const toolCount = Object.keys(this.endTools).length;
        if (toolCount > 0) {
            console.log('Kinematics: URDF describes ' + toolCount + ' end tool(s): ' +
                this.getEndTools().map(t => t.id + '=' + t.label).join(', '));
        }

        // Keep the selected tool if the new URDF still describes it
        if (!this.endTools[this.activeEndToolId]) {
            this.activeEndToolId = this.endTools[0] ? 0 : null;
        }

        this.updateMaxReach();
        console.log('Kinematics: approximate max reach =', this.maxReachMm.toFixed(1), 'mm');
    }

    /**
     * Reach is the arm plus whatever tool is fitted.
     */
    updateMaxReach() {
        const tool = this.getActiveEndTool();
        const toolMm = tool ? tool.lengthMm : 0;
        // Add a small safety margin
        this.maxReachMm = ((this.baseLengthMm || 0) + toolMm) * 1.05;
    }

    /**
     * Selects the end tool whose kinematics apply, by the tool type ID read
     * from register 3 of the ESP32 end tool. An ID the URDF does not describe
     * leaves the TCP at the bare mount face rather than guessing.
     * @param {number|null} toolTypeId - Tool type ID, or null for none
     * @returns {Object|null} The tool that is now active, or null
     */
    setActiveEndTool(toolTypeId) {
        const id = (typeof toolTypeId === 'number' && isFinite(toolTypeId)) ? toolTypeId : null;
        const previous = this.activeEndToolId;
        this.activeEndToolId = id;
        this.updateMaxReach();
        const active = this.getActiveEndTool();
        if (previous !== id) {
            console.log('Kinematics: end tool ' + (active
                ? ('#' + active.id + ' "' + active.label + '" (' + active.lengthMm.toFixed(1) + ' mm)')
                : ('#' + id + ' — not described in the URDF, using the bare mount')));
        }
        return active;
    }

    /**
     * @returns {Object|null} The active tool definition, or null when the ID
     *   is unknown to the URDF
     */
    getActiveEndTool() {
        if (this.activeEndToolId === null || this.activeEndToolId === undefined) {
            return null;
        }
        return this.endTools[this.activeEndToolId] || null;
    }

    /**
     * @returns {Array} Every end tool the URDF describes, by ID
     */
    getEndTools() {
        return Object.keys(this.endTools)
            .map((k) => this.endTools[k])
            .sort((a, b) => a.id - b.id);
    }

    /**
     * Tool state for the UI.
     * @returns {Object}
     */
    getEndToolInfo() {
        const active = this.getActiveEndTool();
        return {
            activeToolTypeId: this.activeEndToolId,
            known: !!active,
            tool: active,
            tools: this.getEndTools()
        };
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

        // ...then the fitted end tool, so the reported point is its working tip
        const activeTool = this.getActiveEndTool();
        if (activeTool) {
            if (this.enableDebugLogging) {
                console.log(`  End tool "${activeTool.label}": offset=(${activeTool.offsetMm.x.toFixed(1)}, ${activeTool.offsetMm.y.toFixed(1)}, ${activeTool.offsetMm.z.toFixed(1)}) mm`);
            }
            T = multiplyMatrices(T, originToMatrix(activeTool.origin));
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

        // Apply fixed tool joints, then the fitted end tool, as extra step(s)
        const activeToolForSteps = this.getActiveEndTool();
        const toolChain = (this.fixedToolJoints || []).concat(
            activeToolForSteps
                ? [{ name: activeToolForSteps.jointName, origin: activeToolForSteps.origin }]
                : []
        );
        if (toolChain.length > 0) {
            for (let f = 0; f < toolChain.length; f++) {
                const toolJoint = toolChain[f];
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

        // Decide if we also have a desired tool orientation (direction vector + optional spin)
        let hasOrientationTarget = false;
        let hasRotationTarget = false;
        let desiredToolZ = { x: 0, y: 0, z: -1 };
        let desiredToolX = null;
        if (targetPose && targetPose.orientation) {
            desiredToolZ = normalizeVector(targetPose.orientation);
            hasOrientationTarget = true;
            if (typeof targetPose.orientation.rotation === 'number') {
                const frame = buildToolFrame(desiredToolZ, targetPose.orientation.rotation);
                desiredToolX = frame.xAxis;
                hasRotationTarget = true;
            }
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

        // Capture seed angles as posture reference — used by the null-space posture task
        // to keep the arm in its current configuration when no orientation is locked.
        const seedAngles = angles.slice();

        // Settings for the solver
        const maxIterations = 800;
        const positionToleranceMm = 0.3;  // stop when within 0.3 mm
        const finiteDifferenceDeg = 0.25; // finite-difference step for numeric Jacobian
        const lambda = 0.5;               // DLS damping factor (singularity robustness)
        const posStepSize = 0.7;          // fraction of pseudoinverse step (handles nonlinearity)
        const nullSpaceGain = 0.3;        // orientation correction gain in null space
        const maxDeltaPerIterDeg = 4.0;   // per-joint safety cap

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
                let oriErrX = 0, oriErrY = 0, oriErrZ = 0, orientationErrorLength = 0;
                if (hasOrientationTarget) {
                    oriErrX = desiredToolZ.x - currentToolZ.x;
                    oriErrY = desiredToolZ.y - currentToolZ.y;
                    oriErrZ = desiredToolZ.z - currentToolZ.z;
                    orientationErrorLength = Math.sqrt(oriErrX * oriErrX + oriErrY * oriErrY + oriErrZ * oriErrZ);
                }

                // Optional rotation error (desired tool X axis - current, spins tool around its Z)
                let rotErrX = 0, rotErrY = 0, rotErrZ = 0, rotationErrorLength = 0;
                let currentToolX = null;
                if (hasRotationTarget) {
                    currentToolX = toolXAxisFromMatrix(fkResult.rotation);
                    rotErrX = desiredToolX.x - currentToolX.x;
                    rotErrY = desiredToolX.y - currentToolX.y;
                    rotErrZ = desiredToolX.z - currentToolX.z;
                    rotationErrorLength = Math.sqrt(rotErrX*rotErrX + rotErrY*rotErrY + rotErrZ*rotErrZ);
                }

                // Combined convergence: both position AND orientation (and rotation if set) within tolerance
                const posConverged = positionErrorLength < positionToleranceMm;
                const oriConverged = !hasOrientationTarget || orientationErrorLength < 0.05;
                const rotConverged = !hasRotationTarget || rotationErrorLength < 0.1;
                if (posConverged && oriConverged && rotConverged) break;

                // Build numeric Jacobian for position (3 x numJoints): how XYZ changes per degree
                const Jpos = [];
                // Row 0: dX/dTheta_j, Row 1: dY/dTheta_j, Row 2: dZ/dTheta_j
                Jpos[0] = new Array(numJoints).fill(0);
                Jpos[1] = new Array(numJoints).fill(0);
                Jpos[2] = new Array(numJoints).fill(0);

                // If we care about orientation, build numeric Jacobians for tool Z and X axes.
                const Jori = hasOrientationTarget ? [
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0)
                ] : null;
                const Jori_x = hasRotationTarget ? [
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0),
                    new Array(numJoints).fill(0)
                ] : null;

                for (let j = 0; j < numJoints; j++) {
                    const originalAngle = angles[j];
                    angles[j] = originalAngle + finiteDifferenceDeg;
                    const fkPlus = this.forwardKinematics(angles);
                    const posPlus = fkPlus.position;
                    const toolZPlus = toolZAxisFromMatrix(fkPlus.rotation);
                    angles[j] = originalAngle;

                    Jpos[0][j] = (posPlus.x - currentPos.x) / finiteDifferenceDeg;
                    Jpos[1][j] = (posPlus.y - currentPos.y) / finiteDifferenceDeg;
                    Jpos[2][j] = (posPlus.z - currentPos.z) / finiteDifferenceDeg;

                    if (hasOrientationTarget && Jori) {
                        Jori[0][j] = (toolZPlus.x - currentToolZ.x) / finiteDifferenceDeg;
                        Jori[1][j] = (toolZPlus.y - currentToolZ.y) / finiteDifferenceDeg;
                        Jori[2][j] = (toolZPlus.z - currentToolZ.z) / finiteDifferenceDeg;
                    }
                    if (hasRotationTarget && Jori_x) {
                        const toolXPlus = toolXAxisFromMatrix(fkPlus.rotation);
                        Jori_x[0][j] = (toolXPlus.x - currentToolX.x) / finiteDifferenceDeg;
                        Jori_x[1][j] = (toolXPlus.y - currentToolX.y) / finiteDifferenceDeg;
                        Jori_x[2][j] = (toolXPlus.z - currentToolX.z) / finiteDifferenceDeg;
                    }
                }

                // --- Null-space IK update ---
                // Primary task: position via damped pseudoinverse (J_pos^+)
                // Secondary task: orientation (Z-axis + optional X-axis spin) projected into
                // null space of J_pos so position is unaffected.

                const Jpos_pinv = dampedPseudoinverse3xN(Jpos, numJoints, lambda);
                if (!Jpos_pinv) continue; // degenerate — skip this iteration

                // Primary update: move toward target position
                const dq_primary = matVec3(Jpos_pinv, [errX, errY, errZ]);

                // Secondary update: orientation lock OR posture control, projected into
                // the null space of J_pos so the primary position task is unaffected.
                let dq_null = null;
                if (hasOrientationTarget && Jori) {
                    // Orientation is locked — drive tool Z (and optionally X) toward target.
                    const g_ori = [];
                    for (let j = 0; j < numJoints; j++) {
                        let g = Jori[0][j]*oriErrX + Jori[1][j]*oriErrY + Jori[2][j]*oriErrZ;
                        // Add X-axis (spin) gradient with half weight — it has 1 DOF vs 2 for Z
                        if (hasRotationTarget && Jori_x) {
                            g += 0.5 * (Jori_x[0][j]*rotErrX + Jori_x[1][j]*rotErrY + Jori_x[2][j]*rotErrZ);
                        }
                        g_ori.push(g);
                    }
                    dq_null = nullSpaceProject(Jpos, Jpos_pinv, g_ori, numJoints);
                } else {
                    // No orientation locked — use null space to hold the arm's starting
                    // configuration (posture control). Without this the 3 redundant DOF
                    // drift freely over 800 iterations, causing cross-axis position errors
                    // (e.g. Z shifts when jogging X).
                    const g_posture = seedAngles.map((a, j) => a - angles[j]);
                    dq_null = nullSpaceProject(Jpos, Jpos_pinv, g_posture, numJoints);
                }

                for (let j = 0; j < numJoints; j++) {
                    let delta = posStepSize * dq_primary[j];
                    if (dq_null) delta += nullSpaceGain * dq_null[j];

                    // Cap per-iteration delta to prevent divergence near singularities
                    angles[j] += Math.max(-maxDeltaPerIterDeg, Math.min(maxDeltaPerIterDeg, delta));

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
                const dot = Math.max(-1, Math.min(1,
                    desiredToolZ.x * finalToolZ.x + desiredToolZ.y * finalToolZ.y + desiredToolZ.z * finalToolZ.z));
                const angleDeg = (Math.acos(dot) * 180) / Math.PI;
                let rotLog = '';
                if (hasRotationTarget) {
                    const finalToolX = toolXAxisFromMatrix(finalFk.rotation);
                    const dotX = Math.max(-1, Math.min(1,
                        desiredToolX.x * finalToolX.x + desiredToolX.y * finalToolX.y + desiredToolX.z * finalToolX.z));
                    rotLog = ` rotError≈${(Math.acos(dotX) * 180 / Math.PI).toFixed(1)}deg`;
                }
                console.log(
                    'IK orientation summary: desiredZ=', desiredToolZ,
                    ' finalZ=', finalToolZ,
                    ' |ΔZ|=', oriErrorLength.toFixed(3),
                    ' angle error≈', angleDeg.toFixed(1), 'deg' + rotLog
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
     * Orientation refinement: takes a position-only IK solution and adjusts the wrist
     * joints so the tool direction matches desiredOrientation, keeping position within
     * a few mm of targetPose.
     *
     * Strategy (fast-path first, grid fallback):
     *   Phase 1 – Jacobian null-space optimizer: drives orientation error in the null
     *             space of the position Jacobian (position stays locked, orientation
     *             improves). Typically converges in <20 ms.
     *   Phase 2 – Wrist-only grid search: only if Phase 1 left orientation error > 12°
     *             or position drifted > 4 mm. Sweeps J4/J5/J6 at coarse intervals then
     *             refines. Much cheaper than the old 7-pass full-arm grid.
     *
     * @param {{ x:number, y:number, z:number }} targetPose - desired XYZ in mm
     * @param {Array<number>} baseAngles - joint angles from position-only IK (degrees)
     * @param {{ x:number, y:number, z:number }} desiredOrientation - desired tool Z-axis direction
     * @param {Array<number>} [referenceAngles] - current arm angles (used to penalise large travel)
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
            return { angles: baseAngles.slice(), positionErrorMm: Infinity, orientationErrorDeg: Infinity, achievedPosition: null };
        }

        const desiredZ = normalizeVector(desiredOrientation);

        // Optional spin (rotation around tool Z-axis)
        const hasRot = typeof desiredOrientation.rotation === 'number';
        const desiredX = hasRot ? buildToolFrame(desiredZ, desiredOrientation.rotation).xAxis : null;

        const clampToLimits = (angleDeg, joint) => {
            let a = angleDeg;
            if (joint && joint.limits) {
                if (typeof joint.limits.lowerDegrees === 'number' && a < joint.limits.lowerDegrees) a = joint.limits.lowerDegrees;
                if (typeof joint.limits.upperDegrees === 'number' && a > joint.limits.upperDegrees) a = joint.limits.upperDegrees;
            }
            return a;
        };

        const evalAngles = (angles) => {
            try {
                const fk = this.forwardKinematics(angles);
                const pos = fk.position;
                const tz = toolZAxisFromMatrix(fk.rotation);
                const dx = pos.x - targetPose.x, dy = pos.y - targetPose.y, dz = pos.z - targetPose.z;
                const posErr = Math.sqrt(dx * dx + dy * dy + dz * dz);
                const dotZ = Math.max(-1, Math.min(1, desiredZ.x*tz.x + desiredZ.y*tz.y + desiredZ.z*tz.z));
                let oriDeg = (Math.acos(dotZ) * 180) / Math.PI;
                if (hasRot) {
                    const tx = toolXAxisFromMatrix(fk.rotation);
                    const dotX = Math.max(-1, Math.min(1, desiredX.x*tx.x + desiredX.y*tx.y + desiredX.z*tx.z));
                    oriDeg += 0.5 * (Math.acos(dotX) * 180) / Math.PI;
                }
                return { positionErrorMm: posErr, orientationErrorDeg: oriDeg, achievedPosition: { x: pos.x, y: pos.y, z: pos.z } };
            } catch (e) {
                return null;
            }
        };

        // Run one Jacobian null-space pass from a given starting point.
        // Returns the updated angles array.
        const runJacobian = (startAngles, maxIter, posStepSize, oriGain, maxDelta) => {
            const a = startAngles.slice();
            const lambda = 0.4, fdDeg = 0.25, posTol = 0.5, oriTol = 0.1;
            const prevLog = this.enableDebugLogging;
            this.enableDebugLogging = false;
            try {
                for (let iter = 0; iter < maxIter; iter++) {
                    const fk = this.forwardKinematics(a);
                    const cp = fk.position;
                    const ctz = toolZAxisFromMatrix(fk.rotation);
                    const ctx = hasRot ? toolXAxisFromMatrix(fk.rotation) : null;
                    const ex = targetPose.x - cp.x, ey = targetPose.y - cp.y, ez = targetPose.z - cp.z;
                    const posErr = Math.sqrt(ex*ex + ey*ey + ez*ez);
                    const oriErrX = desiredZ.x - ctz.x, oriErrY = desiredZ.y - ctz.y, oriErrZ = desiredZ.z - ctz.z;
                    const oriErr = Math.sqrt(oriErrX*oriErrX + oriErrY*oriErrY + oriErrZ*oriErrZ);
                    let rotErrX = 0, rotErrY = 0, rotErrZ = 0, rotErr = 0;
                    if (hasRot) {
                        rotErrX = desiredX.x - ctx.x; rotErrY = desiredX.y - ctx.y; rotErrZ = desiredX.z - ctx.z;
                        rotErr = Math.sqrt(rotErrX*rotErrX + rotErrY*rotErrY + rotErrZ*rotErrZ);
                    }
                    if (posErr < posTol && oriErr < oriTol && rotErr < 0.15) break;
                    const Jpos = [new Array(numJoints).fill(0), new Array(numJoints).fill(0), new Array(numJoints).fill(0)];
                    const Jori = [new Array(numJoints).fill(0), new Array(numJoints).fill(0), new Array(numJoints).fill(0)];
                    const Jori_x = hasRot ? [new Array(numJoints).fill(0), new Array(numJoints).fill(0), new Array(numJoints).fill(0)] : null;
                    for (let j = 0; j < numJoints; j++) {
                        const orig = a[j]; a[j] = orig + fdDeg;
                        const fp = this.forwardKinematics(a); a[j] = orig;
                        Jpos[0][j] = (fp.position.x - cp.x) / fdDeg;
                        Jpos[1][j] = (fp.position.y - cp.y) / fdDeg;
                        Jpos[2][j] = (fp.position.z - cp.z) / fdDeg;
                        const tz2 = toolZAxisFromMatrix(fp.rotation);
                        Jori[0][j] = (tz2.x - ctz.x) / fdDeg;
                        Jori[1][j] = (tz2.y - ctz.y) / fdDeg;
                        Jori[2][j] = (tz2.z - ctz.z) / fdDeg;
                        if (hasRot && Jori_x) {
                            const tx2 = toolXAxisFromMatrix(fp.rotation);
                            Jori_x[0][j] = (tx2.x - ctx.x) / fdDeg;
                            Jori_x[1][j] = (tx2.y - ctx.y) / fdDeg;
                            Jori_x[2][j] = (tx2.z - ctx.z) / fdDeg;
                        }
                    }
                    const pinv = dampedPseudoinverse3xN(Jpos, numJoints, lambda);
                    if (!pinv) continue;
                    const dq_pos = matVec3(pinv, [ex, ey, ez]);
                    const g_ori = [];
                    for (let j = 0; j < numJoints; j++) {
                        let g = Jori[0][j]*oriErrX + Jori[1][j]*oriErrY + Jori[2][j]*oriErrZ;
                        if (hasRot && Jori_x) g += 0.5 * (Jori_x[0][j]*rotErrX + Jori_x[1][j]*rotErrY + Jori_x[2][j]*rotErrZ);
                        g_ori.push(g);
                    }
                    const dq_null = nullSpaceProject(Jpos, pinv, g_ori, numJoints);
                    for (let j = 0; j < numJoints; j++) {
                        let d = posStepSize * dq_pos[j] + oriGain * dq_null[j];
                        d = Math.max(-maxDelta, Math.min(maxDelta, d));
                        a[j] = clampToLimits(a[j] + d, this.joints[j]);
                    }
                }
            } finally {
                this.enableDebugLogging = prevLog;
            }
            return a;
        };

        // ------------------------------------------------------------------
        // Build analytically-informed wrist seeds.
        // J5 must compensate for the accumulated arm pitch from J2 and J3
        // (both rotate about the -Y axis). J4 (wrist roll) determines which
        // direction J5's axis points in the world frame.  We try several J4
        // values to cover left/right workspace and the "behind" configuration.
        // ------------------------------------------------------------------
        const j2 = baseAngles[1] || 0;
        const j3 = baseAngles[2] || 0;
        const j5Hint = clampToLimits(-(j2 + j3), numJoints > 4 ? this.joints[4] : null);

        // Wrist seed configurations: [j4_absolute, j5_absolute, j6_absolute]
        // When a spin rotation is specified, seed j6 with the target rotation angle as a hint.
        const rot6 = hasRot ? (desiredOrientation.rotation || 0) : 0;
        const wristSeeds = [
            [baseAngles[3] || 0, baseAngles[4] || 0, baseAngles[5] || 0], // current (from pos-IK)
            [0,   j5Hint,  rot6],   // no roll, compensated pitch, rotation hint
            [90,  j5Hint,  rot6],   // +90° roll
            [-90, j5Hint,  rot6],   // -90° roll
            [0,   90,      rot6],   // max pitch (for very low positions)
            [90,  90,      rot6],   // roll + max pitch
            [-90, 90,      rot6],   // -roll + max pitch
        ];

        // For each seed run a short Jacobian to locally converge it, then
        // keep the best result for a final longer refinement.
        let bestAngles = null;
        let bestScore = Infinity;

        for (const [j4seed, j5seed, j6seed] of wristSeeds) {
            const seed = baseAngles.slice();
            if (numJoints > 3) seed[3] = clampToLimits(j4seed, this.joints[3]);
            if (numJoints > 4) seed[4] = clampToLimits(j5seed, this.joints[4]);
            if (numJoints > 5) seed[5] = clampToLimits(j6seed, this.joints[5]);

            const refined = runJacobian(seed, 80, 0.5, 1.8, 5.0);
            const ev = evalAngles(refined);
            if (!ev) continue;
            const score = ev.positionErrorMm + ev.orientationErrorDeg;
            if (score < bestScore) {
                bestScore = score;
                bestAngles = refined;
            }
        }

        if (!bestAngles) bestAngles = baseAngles.slice();

        // Final long Jacobian pass from best seed to nail accuracy.
        const finalAngles = runJacobian(bestAngles, 400, 0.5, 1.8, 5.0);
        const finalEv = evalAngles(finalAngles);

        return {
            angles: finalAngles,
            positionErrorMm: finalEv ? finalEv.positionErrorMm : Infinity,
            orientationErrorDeg: finalEv ? finalEv.orientationErrorDeg : Infinity,
            achievedPosition: finalEv ? finalEv.achievedPosition : null
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

    /**
     * Filters a raw URDF joints array (e.g. urdfData.joints, or the
     * equivalent from the Pi's parsed URDF) down to the joints that form
     * the single kinematic chain to draw or compute forward kinematics
     * through: every revolute joint, ordinary fixed joints (e.g.
     * tool_mount), and — only if one is fitted — the active end tool's
     * fixed joint. The URDF can describe several end tools as sibling
     * branches off the same mount link (only one <end_tool> joint per
     * tool type); passing the raw array straight to code that assumes a
     * single linear chain (like the 3D view) draws every possible tool
     * stacked one after another instead of just the one that's fitted.
     * @param {Array} joints - Raw joints array
     * @returns {Array}
     */
    filterChainJoints(joints) {
        if (!Array.isArray(joints)) return [];
        const activeId = this.activeEndToolId;
        return joints.filter((j) => {
            if (!j) return false;
            if (j.endTool) {
                return activeId !== null && activeId !== undefined && j.endTool.id === activeId;
            }
            return true;
        });
    }
}

// Create a global instance
const robotKinematics = new RobotKinematics();
