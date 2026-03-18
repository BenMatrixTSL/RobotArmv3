/**
 * Robot Arm 3D Visualization
 * 
 * This module creates a 3D visualization of the robot arm using Three.js.
 * It displays the arm based on kinematics parameters and updates in real-time
 * as joint angles change.
 * 
 * Simple, beginner-friendly 3D rendering.
 */

class RobotArm3D {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        // Point in space the camera looks at (used for rotate and pan)
        this.cameraTarget = new THREE.Vector3(0, 0, 0);
        this.robotArm = null;
        this.urdfRobot = null;       // Robot loaded via URDFLoader
        this.urdfJointNames = [      // Mapping from joint index to URDF joint name
            'joint1_base_yaw',
            'joint2_shoulder_pitch',
            'joint3_elbow_pitch',
            'joint4_wrist_roll',
            'joint5_wrist_pitch'
        ];
        this.jointAngles = [];
        this.targetAngles = []; // Target angles for animation
        this.currentAnimatedAngles = []; // Current animated angles (for smooth transitions)
        this.jointConfigs = [];
        this.animationId = null;
        this.controls = null; // Camera controls
        this.animationSpeed = 0.1; // Speed of animation (0.1 = 10% per frame, adjust for faster/slower)
        this.isAnimating = false; // Track if animation is in progress
        this.hasLoggedSizeWarning = false; // Track if we've already logged the size warning
        this.deadZoneGroup = null; // Group for dead zone boxes
        this.storedPositionsGroup = null; // Group for stored position markers
        this.movementTraceEndEffectorGroup = null; // Group for end effector movement trace
        this.movementTraceJointsGroup = null;      // Group for joint movement traces
        
        // Initialize Three.js
        this.init();
    }

    /**
     * Initializes the Three.js scene, camera, and renderer
     */
    init() {
        if (!this.container) {
            console.error('Container element not found');
            return;
        }

        // Check container size
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        
        if (width === 0 || height === 0) {
            // Only log warning once to avoid console spam
            if (!this.hasLoggedSizeWarning) {
                this.hasLoggedSizeWarning = true;
                // Don't log - this is expected when tab is not visible
            }
            // Retry after a short delay
            setTimeout(() => this.init(), 100);
            return;
        }
        
        // Reset warning flag once container is visible
        this.hasLoggedSizeWarning = false;

        console.log(`Initializing 3D visualization with container size: ${width}x${height}`);

        // Create scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x222222);

        // Create camera
        // Parameters: field of view, aspect ratio, near clipping plane, far clipping plane
        // Increased far plane from 1000 to 5000 to allow viewing from further away
        this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 5000);
        this.camera.position.set(300, 300, 300);
        this.camera.lookAt(this.cameraTarget);

        // Create renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.container.appendChild(this.renderer.domElement);

        // Add lights
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(200, 200, 200);
        this.scene.add(directionalLight);

        // Add grid helper
        const gridHelper = new THREE.GridHelper(500, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // Add a simple table top at Z = 0 (URDF),
        // which corresponds to Y = 0 in this Three.js scene.
        const tableGeometry = new THREE.BoxGeometry(600, 10, 600);
        const tableMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
        const tableMesh = new THREE.Mesh(tableGeometry, tableMaterial);
        // Place the table so its top surface sits at Y = 0
        tableMesh.position.y = -5;
        this.scene.add(tableMesh);

        // Add axes helper
        const axesHelper = new THREE.AxesHelper(100);
        this.scene.add(axesHelper);

        // Groups for overlays
        this.deadZoneGroup = new THREE.Group();
        this.scene.add(this.deadZoneGroup);

        this.storedPositionsGroup = new THREE.Group();
        this.scene.add(this.storedPositionsGroup);

        // Groups for movement traces
        this.movementTraceEndEffectorGroup = new THREE.Group();
        this.movementTraceEndEffectorGroup.visible = false;
        this.scene.add(this.movementTraceEndEffectorGroup);

        this.movementTraceJointsGroup = new THREE.Group();
        this.movementTraceJointsGroup.visible = false;
        this.scene.add(this.movementTraceJointsGroup);

        // Group for workspace / envelope visuals
        this.workspaceGroup = new THREE.Group();
        this.workspaceGroup.visible = true;
        this.scene.add(this.workspaceGroup);

        // Add simple orbit controls (mouse drag to rotate)
        this.setupControls();

        // Create robot arm
        this.createRobotArm();

        // If URDFLoader is available, load the demo URDF robot for accurate kinematics
        if (typeof URDFLoader !== 'undefined') {
            try {
                const loader = new URDFLoader();
                // Load the demo URDF that lives alongside index.html
                loader.load('demo-kinematics.urdf', (robot) => {
                    console.log('URDF robot loaded for 3D visualization');
                    this.urdfRobot = robot;
                    // Add robot to the scene
                    this.scene.add(robot);
                    // Optionally hide the placeholder arm
                    if (this.robotArm) {
                        this.robotArm.visible = false;
                    }
                });
            } catch (error) {
                console.error('Failed to load URDF robot:', error);
            }
        } else {
            console.warn('URDFLoader not available. Falling back to simple placeholder arm.');
        }

        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());

        // Start animation loop
        this.animate();
    }

    /**
     * Sets up simple mouse controls for camera rotation and panning
     */
    setupControls() {
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };
        let currentAction = null; // 'rotate' or 'pan'
        
        const onMouseDown = (event) => {
            isDragging = true;
            // Left button (0) = pan, others (1, 2) = rotate
            if (event.button === 0) {
                currentAction = 'pan';
            } else {
                currentAction = 'rotate';
            }
        };
        
        const onMouseUp = () => {
            isDragging = false;
            currentAction = null;
        };
        
        const onMouseMove = (event) => {
            if (!isDragging || !currentAction) return;
            
            const deltaX = event.clientX - previousMousePosition.x;
            const deltaY = event.clientY - previousMousePosition.y;
            
            if (currentAction === 'rotate') {
                // Rotate camera around the cameraTarget
                const spherical = new THREE.Spherical();
                const offset = this.camera.position.clone().sub(this.cameraTarget);
                spherical.setFromVector3(offset);
                
                // Adjust angles based on mouse movement
                spherical.theta -= deltaX * 0.01; // Horizontal rotation
                spherical.phi += deltaY * 0.01;    // Vertical rotation
                
                // Clamp phi to prevent flipping
                spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
                
                // Update camera position relative to target
                const newOffset = new THREE.Vector3();
                newOffset.setFromSpherical(spherical);
                this.camera.position.copy(this.cameraTarget.clone().add(newOffset));
                this.camera.lookAt(this.cameraTarget);
            } else if (currentAction === 'pan') {
                // Pan: move camera and target together, so the scene appears to slide
                const panSpeed = 0.5; // Adjust this to make panning faster/slower

                // Get camera basis vectors
                const forward = new THREE.Vector3();
                this.camera.getWorldDirection(forward);
                const right = new THREE.Vector3().crossVectors(forward, this.camera.up).normalize();
                const up = this.camera.up.clone().normalize();

                const panRight = right.multiplyScalar(-deltaX * panSpeed);
                const panUp = up.multiplyScalar(deltaY * panSpeed);

                const panOffset = new THREE.Vector3().addVectors(panRight, panUp);

                this.camera.position.add(panOffset);
                this.cameraTarget.add(panOffset);

                this.camera.lookAt(this.cameraTarget);
            }
            
            previousMousePosition = { x: event.clientX, y: event.clientY };
        };
        
        const onMouseWheel = (event) => {
            // Zoom in/out
            const distance = this.camera.position.length();
            const newDistance = distance + event.deltaY * 0.1;
            
            // Increased max zoom distance from 2000 to 5000 to match increased far clipping plane
            if (newDistance > 50 && newDistance < 5000) {
                this.camera.position.normalize().multiplyScalar(newDistance);
            }
            
            event.preventDefault();
        };
        
        // Add event listeners to the renderer canvas
        this.renderer.domElement.addEventListener('mousedown', (e) => {
            previousMousePosition = { x: e.clientX, y: e.clientY };
            onMouseDown(e);
        });
        
        this.renderer.domElement.addEventListener('mouseup', onMouseUp);
        this.renderer.domElement.addEventListener('mousemove', onMouseMove);
        this.renderer.domElement.addEventListener('wheel', onMouseWheel);
        
        // Prevent context menu on right click
        this.renderer.domElement.addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });
    }

    /**
     * Creates the robot arm visualization
     */
    createRobotArm() {
        // Remove existing arm if any
        if (this.robotArm) {
            this.scene.remove(this.robotArm);
        }

        // Create a group to hold all arm parts
        this.robotArm = new THREE.Group();
        this.scene.add(this.robotArm);
        
        // Show placeholder until configurations are loaded
        this.showPlaceholder();
    }

    /**
     * Shows a placeholder message/arm when no configurations are loaded
     */
    showPlaceholder() {
        if (!this.robotArm) {
            return;
        }

        // Clear existing arm
        while (this.robotArm.children.length > 0) {
            this.robotArm.remove(this.robotArm.children[0]);
        }

        // Create a simple base to show something is working
        const baseGeometry = new THREE.CylinderGeometry(30, 30, 20, 32);
        const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x666666 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = 10;
        this.robotArm.add(base);

        // Add a simple placeholder link
        const linkGeometry = new THREE.CylinderGeometry(8, 8, 100, 16);
        const linkMaterial = new THREE.MeshStandardMaterial({ color: 0x95a5a6, opacity: 0.5, transparent: true });
        const link = new THREE.Mesh(linkGeometry, linkMaterial);
        link.position.y = 70;
        link.rotateX(Math.PI / 2);
        this.robotArm.add(link);

        // Add a placeholder joint
        const jointGeometry = new THREE.SphereGeometry(15, 16, 16);
        const jointMaterial = new THREE.MeshStandardMaterial({ color: 0x3498db, opacity: 0.5, transparent: true });
        const joint = new THREE.Mesh(jointGeometry, jointMaterial);
        joint.position.y = 120;
        this.robotArm.add(joint);
    }

    /**
     * Updates the robot arm visualization based on URDF joint configurations and angles
     * @param {Array} jointConfigs - Array of URDF joint configurations
     * @param {Array} jointAngles - Array of current joint angles in degrees (only for revolute joints)
     * @param {boolean} animate - Whether to animate the transition (default: true)
     */
    update(jointConfigs, jointAngles, animate = true) {
        if (!this.scene || !this.robotArm) {
            console.error('3D visualization not initialized. Scene or robotArm is null.');
            return;
        }

        // Store latest angles
        this.targetAngles = (jointAngles || []).slice(); // Copy the array
        this.jointAngles = this.targetAngles.slice();

        // If a URDF robot is loaded, drive its joints directly using URDFLoader
        if (this.urdfRobot && this.urdfRobot.setJointValue) {
            const kinematicsJointsForUrdf = (typeof robotKinematics !== 'undefined' && robotKinematics.isConfigured())
                ? robotKinematics.getJointConfigs() : null;
            for (let i = 0; i < this.urdfJointNames.length; i++) {
                const jointName = this.urdfJointNames[i];
                let angleDeg = this.jointAngles[i] || 0;
                const kinJoint = kinematicsJointsForUrdf && kinematicsJointsForUrdf[i];
                const offset = (kinJoint && typeof kinJoint.zeroOffsetDegrees === 'number') ? kinJoint.zeroOffsetDegrees : 0;
                angleDeg = angleDeg + offset;
                if (offset !== 0) {
                    console.log('[3D view URDF]', jointName, ': UI angle =', this.jointAngles[i], ', offset =', offset, ', effective =', angleDeg);
                }
                const angleRad = (angleDeg * Math.PI) / 180;
                try {
                    this.urdfRobot.setJointValue(jointName, angleRad);
                } catch (error) {
                    // If a joint name is missing, just skip it
                }
            }
            // No custom arm geometry needed when URDF robot is present
            return;
        }

        // Update working envelope visual if kinematics is available
        if (typeof robotKinematics !== 'undefined' && robotKinematics.isConfigured() && robotKinematics.maxReachMm) {
            this.updateWorkspaceEnvelope(robotKinematics.maxReachMm);
        }

        // Fallback: if no URDF robot, use the simple custom arm geometry
        if (!jointConfigs || jointConfigs.length === 0) {
            console.warn('No joint configurations provided. Showing placeholder.');
            this.showPlaceholder();
            return;
        }
        
        this.jointConfigs = jointConfigs;
        this.updateArmGeometry();
    }

    /**
     * Draws or updates a simple spherical working envelope based on max reach (in mm).
     * The sphere is centered at the base (0,0,0) in URDF space, which corresponds to
     * (0,0,0) in this Three.js scene as well.
     * @param {number} maxReachMm - Approximate maximum reach radius in millimetres
     */
    updateWorkspaceEnvelope(maxReachMm) {
        if (!this.workspaceGroup) {
            return;
        }

        // Clear previous envelope
        while (this.workspaceGroup.children.length > 0) {
            const child = this.workspaceGroup.children[0];
            this.workspaceGroup.remove(child);
        }

        if (!maxReachMm || maxReachMm <= 0) {
            return;
        }

        // In this scene, 1 unit ≈ 1 mm (grid is 500 units wide).
        const radius = maxReachMm;

        const geometry = new THREE.SphereGeometry(radius, 32, 24);
        const material = new THREE.MeshBasicMaterial({
            color: 0x3498db,
            opacity: 0.08,
            transparent: true,
            wireframe: false
        });
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.set(0, 0, 0);
        this.workspaceGroup.add(sphere);
    }

    /**
     * Shows or hides the working envelope group.
     * @param {boolean} visible
     */
    setEnvelopeVisible(visible) {
        if (this.workspaceGroup) {
            this.workspaceGroup.visible = !!visible;
        }
    }
    
    /**
     * Checks if two angle arrays are approximately equal
     * @param {Array} angles1 - First angle array
     * @param {Array} angles2 - Second angle array
     * @returns {boolean} True if angles are approximately equal
     */
    anglesAreEqual(angles1, angles2) {
        if (angles1.length !== angles2.length) {
            return false;
        }
        const threshold = 0.1; // 0.1 degree threshold
        for (let i = 0; i < angles1.length; i++) {
            if (Math.abs(angles1[i] - angles2[i]) > threshold) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * Updates the arm geometry based on current animated angles using URDF
     */
    updateArmGeometry() {
        if (!this.scene || !this.robotArm) {
            return;
        }

        // Clear existing arm
        while (this.robotArm.children.length > 0) {
            this.robotArm.remove(this.robotArm.children[0]);
        }

        // Build the arm from base to end effector using URDF transformations
        let currentTransform = new THREE.Matrix4();
        currentTransform.identity();

        // Create base
        const baseGeometry = new THREE.CylinderGeometry(30, 30, 20, 32);
        const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x666666 });
        const base = new THREE.Mesh(baseGeometry, baseMaterial);
        base.position.y = 10;
        this.robotArm.add(base);

        // Track positions for drawing links
        // Start with base position (top of base cylinder at y=20mm)
        const baseTopPosition = new THREE.Vector3(0, 20, 0); // Base top in mm
        const positions = [baseTopPosition.clone()];

        // Start transformation from base_link origin
        // Translate to base top
        // Note: We'll handle coordinate system mapping in the joint transformations
        const baseOffset = new THREE.Matrix4();
        baseOffset.makeTranslation(0, 20, 0); // Start from base top (20mm up)
        currentTransform.multiply(baseOffset);

        // Process each joint using URDF transformations
        // Standard URDF forward kinematics: work in URDF coordinate space, then transform to Three.js at the end
        // Track which revolute joint index we're on (for angle array)
        let revoluteJointIndex = 0;

        // Get canonical kinematics joints (these always include zeroOffsetDegrees)
        let kinematicsJoints = null;
        if (typeof robotKinematics !== 'undefined' && robotKinematics.isConfigured()) {
            kinematicsJoints = robotKinematics.getJointConfigs() || null;
        }
        console.log('[3D view] zero_offset check: robotKinematics defined=', typeof robotKinematics !== 'undefined', ', configured=', (robotKinematics && robotKinematics.isConfigured()), ', kinematicsJoints count=', kinematicsJoints ? kinematicsJoints.length : 0);
        if (kinematicsJoints) {
            kinematicsJoints.forEach(function (j, idx) {
                if (j && typeof j.zeroOffsetDegrees === 'number') {
                    console.log('[3D view] kinematics joint', idx, '"' + (j.name || '?') + '" zeroOffsetDegrees =', j.zeroOffsetDegrees);
                }
            });
        }
        
        // Work in URDF coordinate space first (using row-major matrices like urdfpy)
        // We'll use arrays and convert to Three.js matrices at the end
        let urdfTransform = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
        
        // Helper function to multiply two 4x4 matrices (row-major)
        const multiplyMatrices = (m1, m2) => {
            const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]];
            for (let i = 0; i < 4; i++) {
                for (let j = 0; j < 4; j++) {
                    for (let k = 0; k < 4; k++) {
                        result[i][j] += m1[i][k] * m2[k][j];
                    }
                }
            }
            return result;
        };
        
        for (let i = 0; i < this.jointConfigs.length; i++) {
            const joint = this.jointConfigs[i];
            if (!joint) continue;

            // Get current joint angle (only for revolute joints)
            let jointAngle = 0;
            if (joint.type === 'revolute') {
                const uiAngle = this.jointAngles[revoluteJointIndex] || 0;
                revoluteJointIndex++;
                // Apply zero-offset from kinematics (parsed URDF); use canonical list so offset is never lost
                const kinJoint = kinematicsJoints && kinematicsJoints[revoluteJointIndex - 1];
                const offset = (kinJoint && typeof kinJoint.zeroOffsetDegrees === 'number') ? kinJoint.zeroOffsetDegrees : 0;
                jointAngle = uiAngle + offset;
                if (offset !== 0) {
                    console.log('[3D view] revolute', (joint.name || ('J' + revoluteJointIndex)), ': UI angle =', uiAngle, ', offset =', offset, ', effective angle =', jointAngle);
                }
            }
            const jointAngleRad = (jointAngle * Math.PI) / 180;

            // Step 1: Apply origin transformation (xyz + rpy from URDF)
            // Use the standard URDF transformation functions
            const originMatrix = originToMatrix(joint.origin);
            urdfTransform = multiplyMatrices(urdfTransform, originMatrix);

            // Step 2: Apply joint rotation around the axis (only for revolute joints)
            if (joint.type === 'revolute') {
                const rotationMatrix = axisRotationMatrix(joint.axis, jointAngleRad);
                urdfTransform = multiplyMatrices(urdfTransform, rotationMatrix);
            }
            
            // Convert URDF transform (row-major) to Three.js Matrix4 (column-major)
            // URDF: X=forward, Y=left, Z=up
            // Three.js: X=right, Y=up, Z=forward
            // Mapping: URDF(X,Y,Z) -> Three.js(-Y,Z,X)
            const urdfPos = {
                x: urdfTransform[0][3],
                y: urdfTransform[1][3],
                z: urdfTransform[2][3]
            };
            
            // Map position: URDF (X,Y,Z) -> Three.js (-Y, Z, X) in millimeters
            const threePos = new THREE.Vector3(
                -urdfPos.y * 1000,  // URDF Y -> Three.js -X (mm)
                urdfPos.z * 1000,   // URDF Z -> Three.js Y (mm)
                urdfPos.x * 1000    // URDF X -> Three.js Z (mm)
            );

            // Extract rotation matrix from URDF transform
            const urdfRot = [
                [urdfTransform[0][0], urdfTransform[0][1], urdfTransform[0][2]],
                [urdfTransform[1][0], urdfTransform[1][1], urdfTransform[1][2]],
                [urdfTransform[2][0], urdfTransform[2][1], urdfTransform[2][2]]
            ];
            
            // Map rotation matrix: URDF -> Three.js coordinate system
            // URDF basis vectors -> Three.js basis vectors
            // URDF X (1,0,0) -> Three.js Z (0,0,1)
            // URDF Y (0,1,0) -> Three.js -X (-1,0,0)
            // URDF Z (0,0,1) -> Three.js Y (0,1,0)
            const threeRot = new THREE.Matrix4();
            threeRot.set(
                -urdfRot[1][0], -urdfRot[1][1], -urdfRot[1][2], 0,  // URDF Y -> Three.js -X
                urdfRot[2][0],  urdfRot[2][1],  urdfRot[2][2],  0,  // URDF Z -> Three.js Y
                urdfRot[0][0],  urdfRot[0][1],  urdfRot[0][2],  0,  // URDF X -> Three.js Z
                0,              0,              0,              1
            );
            
            // Combine position and rotation
            currentTransform = new THREE.Matrix4();
            currentTransform.copy(threeRot);
            currentTransform.setPosition(threePos);
            
            // Also add base offset
            const baseOffset = new THREE.Matrix4();
            baseOffset.makeTranslation(0, 20, 0);
            currentTransform.premultiply(baseOffset);

            // Extract position from transformation matrix
            const jointPosition = new THREE.Vector3();
            jointPosition.setFromMatrixPosition(currentTransform);
            
            // Store this position
            positions.push(jointPosition.clone());

            // Get the previous position
            const prevPosition = positions[i];
            const currentPosition = positions[i + 1];

            // Create joint (sphere) at current position
            const jointGeometry = new THREE.SphereGeometry(15, 16, 16);
            const jointMaterial = new THREE.MeshStandardMaterial({ 
                color: i === 0 ? 0x3498db : (joint.type === 'fixed' ? 0xe74c3c : 0x2ecc71)
            });
            const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial);
            jointMesh.position.copy(currentPosition);
            this.robotArm.add(jointMesh);

            // Create link (cylinder) from previous position to current position.
            // If the joint origins are the same (very short segment), we still draw
            // a small stub so all joints are visible.
            const linkDirection = new THREE.Vector3().subVectors(currentPosition, prevPosition);
            let linkLength = linkDirection.length();
            
            let linkGeometry;
            const linkMaterial = new THREE.MeshStandardMaterial({ color: 0x95a5a6 });
            const link = new THREE.Mesh(undefined, linkMaterial);
            
            if (linkLength > 1.0) {
                // Normal link between two different points
                linkGeometry = new THREE.CylinderGeometry(8, 8, linkLength, 16);
                link.geometry = linkGeometry;
                
                // Position link at the midpoint between previous and current position
                link.position.copy(prevPosition);
                link.position.add(linkDirection.clone().multiplyScalar(0.5));
                
                // Rotate link to point from previous to current position
                link.lookAt(currentPosition);
                link.rotateX(Math.PI / 2);
            } else {
                // Very short / zero-length segment: draw a small vertical stub
                const stubLength = 20;
                linkGeometry = new THREE.CylinderGeometry(8, 8, stubLength, 16);
                link.geometry = linkGeometry;
                
                link.position.copy(currentPosition);
                // Default cylinder is along Y axis, so no extra rotation needed
            }
            
            this.robotArm.add(link);
        }

        // Create end effector
        if (positions.length > 1) {
            const endEffectorPos = positions[positions.length - 1];
            const endEffectorGeometry = new THREE.ConeGeometry(10, 20, 16);
            const endEffectorMaterial = new THREE.MeshStandardMaterial({ color: 0xe74c3c });
            const endEffector = new THREE.Mesh(endEffectorGeometry, endEffectorMaterial);
            endEffector.position.copy(endEffectorPos);
            endEffector.rotateX(Math.PI);
            this.robotArm.add(endEffector);

            // Add a small sphere at end effector tip
            const tipGeometry = new THREE.SphereGeometry(5, 16, 16);
            const tipMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
            const tip = new THREE.Mesh(tipGeometry, tipMaterial);
            tip.position.copy(endEffectorPos);
            tip.position.y += 10;
            this.robotArm.add(tip);
        }

        console.log(`3D visualization updated with ${this.jointConfigs.length} joints`);
    }


    /**
     * Handles window resize
     */
    onWindowResize() {
        if (!this.container || !this.camera || !this.renderer) {
            return;
        }

        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * Animation loop
     */
    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        // For URDF-based visualization we don't need extra interpolation here;
        // joints are updated directly in update(). The existing interpolation
        // logic is kept only for the simple fallback arm.
        
        this.renderer.render(this.scene, this.camera);
    }

    /**
     * Sets camera position (and optionally the camera target height)
     * @param {number} x - X position
     * @param {number} y - Y position
     * @param {number} z - Z position
     * @param {number} [targetY] - Optional Y position for the point the camera looks at
     */
    setCameraPosition(x, y, z, targetY) {
        if (this.camera) {
            this.camera.position.set(x, y, z);
            if (typeof targetY === 'number') {
                this.cameraTarget.set(0, targetY, 0);
            }
            this.camera.lookAt(this.cameraTarget);
        }
    }

    /**
     * Updates dead zone boxes in the 3D scene.
     * @param {Array} zones - Array of {name, minX, maxX, minY, maxY, minZ, maxZ, enabled}
     * Coordinates are in URDF mm (X forward, Y left, Z up).
     */
    updateDeadZones(zones) {
        // Ensure deadZoneGroup is initialized
        if (!this.deadZoneGroup) {
            // Try to initialize it if scene exists
            if (this.scene) {
                this.deadZoneGroup = new THREE.Group();
                this.scene.add(this.deadZoneGroup);
            } else {
                console.warn('updateDeadZones: Scene not initialized yet, cannot update dead zones');
                return;
            }
        }

        // Clear existing boxes
        while (this.deadZoneGroup.children.length > 0) {
            const child = this.deadZoneGroup.children[0];
            this.deadZoneGroup.remove(child);
            // Dispose of geometry and material to prevent memory leaks
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }

        if (!zones || zones.length === 0) {
            return;
        }

        let addedCount = 0;
        zones.forEach((zone, index) => {
            if (!zone) {
                console.warn(`updateDeadZones: Zone at index ${index} is null or undefined`);
                return;
            }
            
            if (!zone.enabled) {
                // Skip disabled zones but log for debugging
                console.log(`updateDeadZones: Skipping disabled zone "${zone.name || 'unnamed'}"`);
                return;
            }

            const minX = zone.minX;
            const maxX = zone.maxX;
            const minY = zone.minY;
            const maxY = zone.maxY;
            const minZ = zone.minZ;
            const maxZ = zone.maxZ;

            // Check for null/undefined values
            if (
                minX == null || maxX == null ||
                minY == null || maxY == null ||
                minZ == null || maxZ == null
            ) {
                console.warn(`updateDeadZones: Zone "${zone.name || 'unnamed'}" has null/undefined values. minX=${minX}, maxX=${maxX}, minY=${minY}, maxY=${maxY}, minZ=${minZ}, maxZ=${maxZ}`);
                return;
            }

            // Check for valid ranges (max should be >= min)
            if (maxX < minX || maxY < minY || maxZ < minZ) {
                console.warn(`updateDeadZones: Zone "${zone.name || 'unnamed'}" has invalid range (max < min)`);
                return;
            }

            const centerX = (minX + maxX) / 2;
            const centerY = (minY + maxY) / 2;
            const centerZ = (minZ + maxZ) / 2;

            const sizeX_urdf = Math.abs(maxX - minX);
            const sizeY_urdf = Math.abs(maxY - minY);
            const sizeZ_urdf = Math.abs(maxZ - minZ);

            // Skip zones with zero or negative size
            if (sizeX_urdf <= 0 || sizeY_urdf <= 0 || sizeZ_urdf <= 0) {
                console.warn(`updateDeadZones: Zone "${zone.name || 'unnamed'}" has zero or negative size`);
                return;
            }

            // Map URDF (X,Y,Z) -> Three.js (-Y,Z,X)
            const threeCenter = new THREE.Vector3(
                -centerY,   // X (right) = -URDF Y
                centerZ,    // Y (up)    =  URDF Z
                centerX     // Z (forward) = URDF X
            );

            const threeSize = {
                x: sizeY_urdf, // scene X span corresponds to URDF Y
                y: sizeZ_urdf, // scene Y span corresponds to URDF Z
                z: sizeX_urdf  // scene Z span corresponds to URDF X
            };

            try {
                const geometry = new THREE.BoxGeometry(threeSize.x, threeSize.y, threeSize.z);
                const material = new THREE.MeshStandardMaterial({
                    color: 0xff0000,
                    transparent: true,
                    opacity: 0.2
                });

                const box = new THREE.Mesh(geometry, material);
                box.position.copy(threeCenter);
                // Store zone info for debugging
                box.userData = { zoneName: zone.name || 'unnamed', zoneId: zone.id };
                this.deadZoneGroup.add(box);
                addedCount++;
            } catch (error) {
                console.error(`updateDeadZones: Error creating box for zone "${zone.name || 'unnamed'}":`, error);
            }
        });
        
        console.log(`updateDeadZones: Added ${addedCount} dead zone(s) to 3D visualization`);
    }

    /**
     * Show/hide dead zones group
     */
    setDeadZonesVisible(visible) {
        if (this.deadZoneGroup) {
            this.deadZoneGroup.visible = !!visible;
        }
    }

    /**
     * Updates stored position markers in the 3D scene.
     * @param {Array} points - Array of {label, x, y, z} in URDF mm.
     */
    updateStoredPositions(points) {
        if (!this.storedPositionsGroup) return;

        // Clear existing markers
        while (this.storedPositionsGroup.children.length > 0) {
            const child = this.storedPositionsGroup.children[0];
            this.storedPositionsGroup.remove(child);
        }

        if (!points || points.length === 0) {
            return;
        }

        points.forEach(point => {
            if (!point) return;
            const x = point.x;
            const y = point.y;
            const z = point.z;
            if (x == null || y == null || z == null) return;

            // Map URDF (X,Y,Z) -> Three.js (-Y,Z,X)
            const threePos = new THREE.Vector3(
                -y, // X
                z,  // Y
                x   // Z
            );

            const geom = new THREE.SphereGeometry(5, 12, 12);
            const mat = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
            const marker = new THREE.Mesh(geom, mat);
            marker.position.copy(threePos);
            this.storedPositionsGroup.add(marker);
        });
    }

    /**
     * Show/hide stored positions group
     */
    setStoredPositionsVisible(visible) {
        if (this.storedPositionsGroup) {
            this.storedPositionsGroup.visible = !!visible;
        }
    }

    /**
     * Updates the end effector movement trace.
     * @param {Array} points - Array of { x, y, z } in URDF mm.
     */
    updateEndEffectorTrace(points) {
        if (!this.movementTraceEndEffectorGroup) {
            return;
        }

        // Clear existing trace
        while (this.movementTraceEndEffectorGroup.children.length > 0) {
            const child = this.movementTraceEndEffectorGroup.children[0];
            this.movementTraceEndEffectorGroup.remove(child);
        }

        if (!points || points.length < 2) {
            return;
        }

        // Build a line through all points
        const positionsArray = [];

        for (let i = 0; i < points.length; i++) {
            const p = points[i];
            if (!p) {
                continue;
            }

            const x = typeof p.x === 'number' ? p.x : 0;
            const y = typeof p.y === 'number' ? p.y : 0;
            const z = typeof p.z === 'number' ? p.z : 0;

            // Map URDF (X,Y,Z) -> Three.js (-Y,Z,X)
            const threeX = -y;
            const threeY = z;
            const threeZ = x;

            positionsArray.push(threeX, threeY, threeZ);
        }

        if (positionsArray.length < 6) {
            // Not enough points to draw a line
            return;
        }

        const geometry = new THREE.BufferGeometry();
        const positionsFloat32 = new Float32Array(positionsArray);
        geometry.setAttribute('position', new THREE.BufferAttribute(positionsFloat32, 3));

        const material = new THREE.LineBasicMaterial({
            color: 0xffff00 // Yellow line for end effector
        });

        const line = new THREE.Line(geometry, material);
        this.movementTraceEndEffectorGroup.add(line);
    }

    /**
     * Updates the per‑joint movement traces.
     * @param {Array<Array>} jointPoints - Array of arrays. Each inner array is a list of { x, y, z } in URDF mm for one joint.
     */
    updateJointTraces(jointPoints) {
        if (!this.movementTraceJointsGroup) {
            return;
        }

        // Clear existing traces
        while (this.movementTraceJointsGroup.children.length > 0) {
            const child = this.movementTraceJointsGroup.children[0];
            this.movementTraceJointsGroup.remove(child);
        }

        if (!jointPoints || !Array.isArray(jointPoints) || jointPoints.length === 0) {
            return;
        }

        // Simple colour list for different joints
        const colors = [
            0xff0000, // red
            0x00ff00, // green
            0x0000ff, // blue
            0xff00ff, // magenta
            0x00ffff, // cyan
            0xffff00  // yellow
        ];

        for (let j = 0; j < jointPoints.length; j++) {
            const points = jointPoints[j];
            if (!points || !Array.isArray(points) || points.length < 2) {
                continue;
            }

            const positionsArray = [];

            for (let i = 0; i < points.length; i++) {
                const p = points[i];
                if (!p) {
                    continue;
                }

                const x = typeof p.x === 'number' ? p.x : 0;
                const y = typeof p.y === 'number' ? p.y : 0;
                const z = typeof p.z === 'number' ? p.z : 0;

                // Map URDF (X,Y,Z) -> Three.js (-Y,Z,X)
                const threeX = -y;
                const threeY = z;
                const threeZ = x;

                positionsArray.push(threeX, threeY, threeZ);
            }

            if (positionsArray.length < 6) {
                continue;
            }

            const geometry = new THREE.BufferGeometry();
            const positionsFloat32 = new Float32Array(positionsArray);
            geometry.setAttribute('position', new THREE.BufferAttribute(positionsFloat32, 3));

            const colorIndex = j % colors.length;
            const material = new THREE.LineBasicMaterial({
                color: colors[colorIndex]
            });

            const line = new THREE.Line(geometry, material);
            this.movementTraceJointsGroup.add(line);
        }
    }

    /**
     * Show/hide end effector trace.
     * @param {boolean} visible
     */
    setEndEffectorTraceVisible(visible) {
        if (this.movementTraceEndEffectorGroup) {
            this.movementTraceEndEffectorGroup.visible = !!visible;
        }
    }

    /**
     * Show/hide joint traces.
     * @param {boolean} visible
     */
    setJointTracesVisible(visible) {
        if (this.movementTraceJointsGroup) {
            this.movementTraceJointsGroup.visible = !!visible;
        }
    }

    /**
     * Clears all stored movement trace geometry (end effector and joint traces).
     */
    clearMovementTraces() {
        if (this.movementTraceEndEffectorGroup) {
            while (this.movementTraceEndEffectorGroup.children.length > 0) {
                const child = this.movementTraceEndEffectorGroup.children[0];
                this.movementTraceEndEffectorGroup.remove(child);
            }
        }
        if (this.movementTraceJointsGroup) {
            while (this.movementTraceJointsGroup.children.length > 0) {
                const child = this.movementTraceJointsGroup.children[0];
                this.movementTraceJointsGroup.remove(child);
            }
        }
    }

    /**
     * Resets camera to default view
     */
    resetCamera() {
        this.setCameraPosition(300, 300, 300, 150);
    }

    /**
     * Cleans up resources
     */
    dispose() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
        if (this.renderer) {
            this.renderer.dispose();
        }
    }
}

