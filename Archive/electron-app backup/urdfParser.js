/**
 * URDF Parser
 * 
 * Simple parser for URDF (Unified Robot Description Format) robot descriptions.
 * Parses URDF XML and extracts joint and link information for kinematics calculations.
 * 
 * Beginner-friendly implementation.
 */

/**
 * Converts radians to degrees
 * @param {number} radians - Angle in radians
 * @returns {number} Angle in degrees
 */
function radiansToDegrees(radians) {
    return (radians * 180) / Math.PI;
}

/**
 * Parses a URDF XML string and extracts robot structure
 * @param {string} urdfXml - URDF XML string
 * @returns {Object} Parsed robot structure with joints and links
 */
function parseURDF(urdfXml) {
    // Create a simple XML parser (using DOMParser)
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(urdfXml, 'text/xml');
    
    // Check for parsing errors
    const parserError = xmlDoc.querySelector('parsererror');
    if (parserError) {
        throw new Error('URDF XML parsing error: ' + parserError.textContent);
    }
    
    const robot = xmlDoc.querySelector('robot');
    if (!robot) {
        throw new Error('No robot element found in URDF');
    }
    
    const robotName = robot.getAttribute('name') || 'robot';
    
    // Extract all joints
    const joints = [];
    const jointElements = robot.querySelectorAll('joint');
    
    jointElements.forEach((jointElement) => {
        const joint = {
            name: jointElement.getAttribute('name'),
            type: jointElement.getAttribute('type'), // 'revolute', 'fixed', etc.
            parent: null,
            child: null,
            origin: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
            axis: { x: 0, y: 0, z: 1 },
            // Optional zero offset (in degrees) to shift the "zero" angle
            // If provided in the URDF, the kinematics will add this to the angle
            zeroOffsetDegrees: null,
            // limits will hold both radians and degrees so other parts
            // of the app can easily work in degrees
            limits: {
                lowerRadians: null,
                upperRadians: null,
                lowerDegrees: null,
                upperDegrees: null,
                effort: 0,
                velocity: 0
            }
        };
        
        // Get parent link
        const parentElement = jointElement.querySelector('parent');
        if (parentElement) {
            joint.parent = parentElement.getAttribute('link');
        }
        
        // Get child link
        const childElement = jointElement.querySelector('child');
        if (childElement) {
            joint.child = childElement.getAttribute('link');
        }
        
        // Get origin (xyz and rpy)
        const originElement = jointElement.querySelector('origin');
        if (originElement) {
            const xyz = originElement.getAttribute('xyz');
            const rpy = originElement.getAttribute('rpy');
            
            if (xyz) {
                const xyzParts = xyz.trim().split(/\s+/);
                joint.origin.x = parseFloat(xyzParts[0]) || 0;
                joint.origin.y = parseFloat(xyzParts[1]) || 0;
                joint.origin.z = parseFloat(xyzParts[2]) || 0;
            }
            
            if (rpy) {
                const rpyParts = rpy.trim().split(/\s+/);
                joint.origin.roll = parseFloat(rpyParts[0]) || 0;
                joint.origin.pitch = parseFloat(rpyParts[1]) || 0;
                joint.origin.yaw = parseFloat(rpyParts[2]) || 0;
            }
        }
        
        // Get axis (for revolute joints)
        const axisElement = jointElement.querySelector('axis');
        if (axisElement) {
            const xyz = axisElement.getAttribute('xyz');
            if (xyz) {
                const xyzParts = xyz.trim().split(/\s+/);
                joint.axis.x = parseFloat(xyzParts[0]) || 0;
                joint.axis.y = parseFloat(xyzParts[1]) || 0;
                joint.axis.z = parseFloat(xyzParts[2]) || 0;
            }
        }
        
        // Optional zero offset (custom attribute on the joint)
        const zeroOffsetAttr = jointElement.getAttribute('zero_offset_degrees');
        if (zeroOffsetAttr !== null) {
            const offsetDeg = parseFloat(zeroOffsetAttr);
            if (!isNaN(offsetDeg)) {
                joint.zeroOffsetDegrees = offsetDeg;
                console.log('[URDF parser] joint "' + (joint.name || '?') + '" zero_offset_degrees =', offsetDeg);
            }
        }
        
        // Get limits (for revolute joints)
        const limitElement = jointElement.querySelector('limit');
        if (limitElement) {
            // Read lower/upper in radians (URDF convention) and also store in degrees
            const lowerAttr = limitElement.getAttribute('lower');
            const upperAttr = limitElement.getAttribute('upper');
            const effortAttr = limitElement.getAttribute('effort');
            const velocityAttr = limitElement.getAttribute('velocity');

            if (lowerAttr !== null) {
                const lowerRad = parseFloat(lowerAttr);
                if (!isNaN(lowerRad)) {
                    joint.limits.lowerRadians = lowerRad;
                    joint.limits.lowerDegrees = radiansToDegrees(lowerRad);
                }
            }

            if (upperAttr !== null) {
                const upperRad = parseFloat(upperAttr);
                if (!isNaN(upperRad)) {
                    joint.limits.upperRadians = upperRad;
                    joint.limits.upperDegrees = radiansToDegrees(upperRad);
                }
            }

            if (effortAttr !== null) {
                const effortVal = parseFloat(effortAttr);
                if (!isNaN(effortVal)) {
                    joint.limits.effort = effortVal;
                }
            }

            if (velocityAttr !== null) {
                const velVal = parseFloat(velocityAttr);
                if (!isNaN(velVal)) {
                    joint.limits.velocity = velVal;
                }
            }
        }
        
        joints.push(joint);
    });
    
    // Extract all links (for reference, though we mainly use joints)
    const links = [];
    const linkElements = robot.querySelectorAll('link');
    linkElements.forEach((linkElement) => {
        links.push({
            name: linkElement.getAttribute('name')
        });
    });
    
    return {
        name: robotName,
        joints: joints,
        links: links
    };
}

/**
 * Creates a transformation matrix from URDF origin (xyz + rpy)
 * @param {Object} origin - Origin object with x, y, z, roll, pitch, yaw
 * @returns {Array} 4x4 transformation matrix (row-major)
 */
function originToMatrix(origin) {
    const x = origin.x || 0;
    const y = origin.y || 0;
    const z = origin.z || 0;
    const roll = origin.roll || 0;
    const pitch = origin.pitch || 0;
    const yaw = origin.yaw || 0;
    
    // Calculate rotation matrix from RPY (Roll-Pitch-Yaw)
    const cosR = Math.cos(roll);
    const sinR = Math.sin(roll);
    const cosP = Math.cos(pitch);
    const sinP = Math.sin(pitch);
    const cosY = Math.cos(yaw);
    const sinY = Math.sin(yaw);
    
    // Rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const r11 = cosY * cosP;
    const r12 = cosY * sinP * sinR - sinY * cosR;
    const r13 = cosY * sinP * cosR + sinY * sinR;
    const r21 = sinY * cosP;
    const r22 = sinY * sinP * sinR + cosY * cosR;
    const r23 = sinY * sinP * cosR - cosY * sinR;
    const r31 = -sinP;
    const r32 = cosP * sinR;
    const r33 = cosP * cosR;
    
    // Build 4x4 transformation matrix
    return [
        [r11, r12, r13, x],
        [r21, r22, r23, y],
        [r31, r32, r33, z],
        [0, 0, 0, 1]
    ];
}

/**
 * Creates a rotation matrix around an arbitrary axis
 * @param {Object} axis - Axis object with x, y, z components
 * @param {number} angle - Rotation angle in radians
 * @returns {Array} 4x4 rotation matrix (row-major)
 */
function axisRotationMatrix(axis, angle) {
    // IMPORTANT: 0 is a valid component and must NOT be replaced by defaults.
    // Only fall back when a component is missing or not a number.
    const ax = (axis && typeof axis.x === 'number') ? axis.x : 0;
    const ay = (axis && typeof axis.y === 'number') ? axis.y : 0;
    const az = (axis && typeof axis.z === 'number') ? axis.z : 1; // default Z if nothing provided
    
    // Normalize axis; if length is 0, fall back to Z axis
    let length = Math.sqrt(ax * ax + ay * ay + az * az);
    let nx, ny, nz;
    if (length === 0) {
        nx = 0; ny = 0; nz = 1;
    } else {
        nx = ax / length;
        ny = ay / length;
        nz = az / length;
    }
    
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    const oneMinusCos = 1 - cosA;
    
    // Rodrigues' rotation formula
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





