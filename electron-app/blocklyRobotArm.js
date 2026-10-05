/**
 * Blockly Visual Programming for Robot Arm
 * 
 * This file sets up Blockly with custom blocks for robot arm control.
 * Simple, beginner-friendly visual programming interface.
 */

/**
 * Speed conversion functions
 * These functions convert between degrees per second and steps per second
 * Conversion factor: 1 degree ≈ 11.37 steps
 * 
 * These functions are globally accessible for use throughout the codebase
 */
const DEGREES_TO_STEPS_RATIO = 11.37;

/**
 * Converts degrees per second to steps per second
 * @param {number} degreesPerSecond - Speed in degrees per second
 * @returns {number} Speed in steps per second
 */
function degreesPerSecondToStepsPerSecond(degreesPerSecond) {
    return Math.round(degreesPerSecond * DEGREES_TO_STEPS_RATIO);
}

/**
 * Converts steps per second to degrees per second
 * @param {number} stepsPerSecond - Speed in steps per second
 * @returns {number} Speed in degrees per second (rounded to 2 decimal places)
 */
function stepsPerSecondToDegreesPerSecond(stepsPerSecond) {
    return Math.round((stepsPerSecond / DEGREES_TO_STEPS_RATIO) * 100) / 100;
}

// Make conversion functions globally accessible for use in other parts of the codebase
if (typeof window !== 'undefined') {
    window.degreesPerSecondToStepsPerSecond = degreesPerSecondToStepsPerSecond;
    window.stepsPerSecondToDegreesPerSecond = stepsPerSecondToDegreesPerSecond;
    window.DEGREES_TO_STEPS_RATIO = DEGREES_TO_STEPS_RATIO;
}

/**
 * Calculates scaled speeds for multiple joints using linear interpolation
 * so that all joints arrive at their target positions simultaneously
 * 
 * @param {Array<number>} currentAngles - Current angles for each joint (degrees)
 * @param {Array<number>} targetAngles - Target angles for each joint (degrees)
 * @param {number} baseSpeedDegreesPerSecond - Base speed in degrees/s
 * @returns {Array<number>} Array of scaled speeds in degrees/s for each joint
 */
function calculateScaledSpeeds(currentAngles, targetAngles, baseSpeedDegreesPerSecond) {
    if (currentAngles.length !== targetAngles.length) {
        console.warn('Current and target angles arrays have different lengths, using base speed for all joints');
        return Array(currentAngles.length).fill(baseSpeedDegreesPerSecond);
    }
    
    // Calculate distances for each joint
    const distances = currentAngles.map((current, index) => {
        const target = targetAngles[index];
        return Math.abs(target - current);
    });
    
    // Find the maximum distance (this joint will use the base speed)
    const maxDistance = Math.max(...distances);
    
    // If max distance is 0, all joints are already at target
    if (maxDistance === 0) {
        return Array(currentAngles.length).fill(baseSpeedDegreesPerSecond);
    }
    
    // Minimum speed floor (matches the 50 steps/s floor in computeCoordinatedSpeeds).
    // Without this, joints with very small travel get near-zero speeds that the
    // servo cannot accurately track, causing them to appear to move too slowly.
    const MIN_SPEED_DEG_PER_SEC = 50 / 11.37; // ~4.4 deg/s

    // Calculate scaled speeds: speed = baseSpeed * (distance / maxDistance)
    // This ensures all joints finish at the same time
    const scaledSpeeds = distances.map(distance => {
        if (distance === 0) {
            return 0; // Joint already at target, no movement needed
        }
        return Math.max(baseSpeedDegreesPerSecond * (distance / maxDistance), MIN_SPEED_DEG_PER_SEC);
    });
    
    return scaledSpeeds;
}

// Make globally accessible
if (typeof window !== 'undefined') {
    window.calculateScaledSpeeds = calculateScaledSpeeds;
}

let blocklyWorkspace = null;
let blocklyProgramRunning = false;
let blocklyProgramStopRequested = false;
let blocklyProgramPaused = false;
let blocklyProgramResumeResolve = null;
let currentHighlightedBlock = null; // Track currently highlighted block

/**
 * Initializes the Blockly workspace
 * Call this when the Blockly tab is opened
 */
function initializeBlockly() {
    // Hide loading message
    const loadingDiv = document.getElementById('blockly-loading');
    if (loadingDiv) {
        loadingDiv.style.display = 'none';
    }
    
        // Show controls
        const controlsDiv = document.getElementById('blocklyControls');
        if (controlsDiv) {
            controlsDiv.style.display = 'flex';
        }
    
    // Check if Blockly is loaded
    if (typeof Blockly === 'undefined') {
        console.error('Blockly library not loaded');
        const statusDiv = document.getElementById('blocklyStatus');
        if (statusDiv) {
            statusDiv.textContent = 'Error: Blockly library not loaded. Please check your internet connection or refresh the page.';
            statusDiv.style.color = '#e74c3c';
        }
        
        // Show error in workspace area
        const workspaceDiv = document.getElementById('blockly-workspace');
        if (workspaceDiv) {
            workspaceDiv.innerHTML = '<div style="padding: 20px; text-align: center; color: #e74c3c; background: #ffe6e6; border-radius: 4px;"><h3>Blockly Library Not Loaded</h3><p>Please check your internet connection and refresh the page.</p><p>If the problem persists, the Blockly CDN may be unavailable.</p><p>Check the browser console (F12) for more details.</p></div>';
        }
        return;
    }

    // Check if JavaScript generator is loaded
    if (typeof Blockly.JavaScript === 'undefined') {
        console.error('Blockly JavaScript generator not loaded');
        const statusDiv = document.getElementById('blocklyStatus');
        if (statusDiv) {
            statusDiv.textContent = 'Error: Blockly JavaScript generator not loaded';
            statusDiv.style.color = '#e74c3c';
        }
        return;
    }

    // Check if XML module is loaded (needed for save/load)
    if (typeof Blockly.Xml === 'undefined' || typeof Blockly.Xml.textToDom !== 'function') {
        console.warn('Blockly XML module not loaded - save/load functionality will not work');
        const statusDiv = document.getElementById('blocklyStatus');
        if (statusDiv) {
            statusDiv.textContent = 'Warning: XML module not loaded. Save/load may not work. Refresh the page.';
            statusDiv.style.color = '#f39c12';
        }
        // Don't return - allow workspace to initialize, but save/load won't work
    }

    // Get the workspace container
    const workspaceDiv = document.getElementById('blockly-workspace');
    if (!workspaceDiv) {
        console.error('Blockly workspace container not found');
        return;
    }

    // Check if Blockly workspace is already initialized
    if (blocklyWorkspace) {
        console.log('Blockly workspace already initialized, skipping re-initialization');
        return;
    }

    // Check if the container already has a Blockly workspace (prevent duplicate initialization)
    // Look for Blockly-injected SVG elements
    if (workspaceDiv.querySelector('svg.blocklyMainBackground') || 
        workspaceDiv.querySelector('svg.blocklySvg') ||
        workspaceDiv.querySelector('.blocklyToolboxDiv') ||
        workspaceDiv.querySelector('.blocklyFlyout')) {
        console.warn('Blockly workspace already exists in container, skipping initialization');
        // Try to get the existing workspace
        if (typeof Blockly.getMainWorkspace === 'function') {
            blocklyWorkspace = Blockly.getMainWorkspace();
            console.log('Retrieved existing Blockly workspace');
        }
        return;
    }

    try {
        // Create the Blockly workspace
        blocklyWorkspace = Blockly.inject(workspaceDiv, {
            toolbox: getBlocklyToolbox(),
            grid: {
                spacing: 20,
                length: 3,
                colour: '#ccc',
                snap: true
            },
            zoom: {
                controls: true,
                wheel: true,
                // Blocks need to be bigger to be draggable with a finger
                startScale: document.body.classList.contains('touch-mode') ? 1.25 : 1.0,
                maxScale: 3,
                minScale: 0.3,
                scaleSpeed: 1.1
            },
            trashcan: true,
            // Blockly's click/delete sounds are the only audio the app plays.
            // On the Pi kiosk, opening the headphone audio device reprograms
            // the PWM clock the WS2812B status LED uses and freezes the strip.
            sounds: false,
            // Enable variables
            variables: true
        });

        // Define custom blocks
        defineCustomBlocks();

        // Auto-load saved program from localStorage (if available)
        autoLoadBlocklyProgram();
        
        // If no saved program, load example
        if (!localStorage.getItem('blocklyProgram')) {
            loadBlocklyExample();
        }

        console.log('Blockly workspace initialized');
        const statusDiv = document.getElementById('blocklyStatus');
        if (statusDiv) {
            statusDiv.textContent = 'Ready - Drag blocks from the toolbox on the left';
            statusDiv.style.color = '#27ae60';
        }
    } catch (error) {
        console.error('Error initializing Blockly:', error);
        const statusDiv = document.getElementById('blocklyStatus');
        if (statusDiv) {
            statusDiv.textContent = 'Error initializing Blockly: ' + error.message;
            statusDiv.style.color = '#e74c3c';
        }
    }
}

/**
 * Defines the toolbox with all available blocks
 */
function getBlocklyToolbox() {
    return {
        kind: 'categoryToolbox',
        contents: [
            {
                kind: 'category',
                name: 'Robot Control',
                colour: '#5C81A6',
                contents: [
                    {
                        kind: 'block',
                        type: 'move_joint'
                    },
                    {
                        kind: 'block',
                        type: 'move_all_joints'
                    },
                    {
                        kind: 'block',
                        type: 'stop_joint'
                    },
                    {
                        kind: 'block',
                        type: 'stop_all'
                    },
                    {
                        kind: 'block',
                        type: 'set_servo'
                    },
                    {
                        kind: 'block',
                        type: 'move_to_position'
                    },
                    {
                        kind: 'block',
                        type: 'set_acceleration'
                    },
                    {
                        kind: 'block',
                        type: 'move_xyz',
                        inputs: {
                            X: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                            Y: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                            Z: { shadow: { type: 'math_number', fields: { NUM: 0 } } }
                        }
                    },
                    {
                        kind: 'block',
                        type: 'move_xyz_offset',
                        inputs: {
                            DX: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                            DY: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                            DZ: { shadow: { type: 'math_number', fields: { NUM: 0 } } }
                        }
                    },
                    {
                        kind: 'block',
                        type: 'set_tool_orientation'
                    }
                ]
            },
            {
                kind: 'category',
                name: 'Tool',
                colour: '#A5745B',
                contents: [
                    { kind: 'block', type: 'gripper_open' },
                    { kind: 'block', type: 'gripper_close' },
                    { kind: 'block', type: 'end_tool_servo' },
                    { kind: 'block', type: 'pump_on' },
                    { kind: 'block', type: 'pump_off' },
                    { kind: 'block', type: 'solenoid_on' },
                    { kind: 'block', type: 'solenoid_off' }
                ]
            },
            {
                kind: 'category',
                name: 'Wait',
                colour: '#5BA55B',
                contents: [
                    {
                        kind: 'block',
                        type: 'wait_seconds'
                    },
                    {
                        kind: 'block',
                        type: 'wait_until_stopped'
                    },
                    {
                        kind: 'block',
                        type: 'wait_until_all_stopped'
                    }
                ]
            },
            {
                kind: 'category',
                name: 'Vision',
                colour: '#A6945C',
                contents: [
                    { kind: 'block', type: 'block_count' },
                    { kind: 'block', type: 'block_x_at' },
                    { kind: 'block', type: 'block_y_at' },
                    { kind: 'block', type: 'block_color_at' },
                    { kind: 'block', type: 'save_block_to_position' }
                ]
            },
            {
                kind: 'category',
                name: 'Loops',
                colour: '#5C68A6',
                contents: [
                    {
                        kind: 'block',
                        type: 'controls_repeat_ext'
                    },
                    {
                        kind: 'block',
                        type: 'controls_whileUntil'
                    }
                ]
            },
            {
                kind: 'category',
                name: 'Logic',
                colour: '#5C81A6',
                contents: [
                    {
                        kind: 'block',
                        type: 'controls_if'
                    },
                    {
                        kind: 'block',
                        type: 'logic_compare'
                    },
                    {
                        kind: 'block',
                        type: 'logic_operation'
                    }
                ]
            },
            {
                kind: 'category',
                name: 'Variables',
                colour: '#A65C81',
                custom: 'VARIABLE',
                variables: true
            },
            {
                kind: 'category',
                name: 'Math',
                colour: '#5C68A6',
                contents: [
                    {
                        kind: 'block',
                        type: 'math_number'
                    },
                    {
                        kind: 'block',
                        type: 'math_arithmetic'
                    }
                ]
            }
        ]
    };
}

/**
 * Updates the position dropdown in Blockly blocks
 */
function updateBlocklyPositionBlocks() {
    if (!blocklyWorkspace || typeof Blockly === 'undefined') {
        return;
    }
    
    // Find all move_to_position blocks and update their dropdowns
    const blocks = blocklyWorkspace.getAllBlocks();
    blocks.forEach(block => {
        if (block.type === 'move_to_position') {
            const dropdown = block.getField('POSITION');
            if (dropdown && typeof getPositionsForBlockly === 'function') {
                const positions = getPositionsForBlockly();
                if (positions.length > 0) {
                    dropdown.menuGenerator_ = function() {
                        return positions;
                    };
                }
            }
        }
    });
}

/**
 * Defines custom blocks for robot arm control
 */
function defineCustomBlocks() {
    // Move joint to angle
    Blockly.Blocks['move_joint'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Move Joint')
                .appendField(new Blockly.FieldNumber(1, 1, 6, 1), 'JOINT')
                .appendField('to')
                .appendField(new Blockly.FieldNumber(0, -160, 160, 0.1), 'ANGLE')
                .appendField('degrees');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(40, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Move a joint to a specific angle at a given speed in degrees per second');
        }
    };

    // Set tool orientation vector (direction of tool Z-axis) with optional spin rotation
    Blockly.Blocks['set_tool_orientation'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Set tool orientation to vector')
                .appendField('X')
                .appendField(new Blockly.FieldNumber(0), 'ORI_X')
                .appendField('Y')
                .appendField(new Blockly.FieldNumber(0), 'ORI_Y')
                .appendField('Z')
                .appendField(new Blockly.FieldNumber(-1), 'ORI_Z')
                .appendField('Rotation (°)')
                .appendField(new Blockly.FieldNumber(0), 'ORI_ROTATION')
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(90, 1, 360), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Set the tool orientation vector and spin rotation, and turn the tool to it now, in place, at the given joint speed. Rotation spins the tool around its pointing axis — 0° keeps a consistent world-aligned reference regardless of position. Later Move TCP blocks keep this orientation.');
        }
    };

    // Move all joints to angles
    Blockly.Blocks['move_all_joints'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Move All Joints to');
            this.appendValueInput('JOINT1')
                .setCheck('Number')
                .appendField('Joint 1 (degrees)');
            this.appendValueInput('JOINT2')
                .setCheck('Number')
                .appendField('Joint 2 (degrees)');
            this.appendValueInput('JOINT3')
                .setCheck('Number')
                .appendField('Joint 3 (degrees)');
            this.appendValueInput('JOINT4')
                .setCheck('Number')
                .appendField('Joint 4 (degrees)');
            this.appendValueInput('JOINT5')
                .setCheck('Number')
                .appendField('Joint 5 (degrees)');
            this.appendValueInput('JOINT6')
                .setCheck('Number')
                .appendField('Joint 6 (degrees)');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(40, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Move all joints to specified angles simultaneously at a given speed in degrees per second');
        }
    };

    // Stop joint
    Blockly.Blocks['stop_joint'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Stop Joint')
                .appendField(new Blockly.FieldNumber(1, 1, 6, 1), 'JOINT');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Stop a specific joint');
        }
    };

    // Stop all joints
    Blockly.Blocks['stop_all'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Stop All Joints');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Stop all joints immediately');
        }
    };

    // Set servo angle
    Blockly.Blocks['set_servo'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Set Servo on Joint')
                .appendField(new Blockly.FieldNumber(1, 1, 6, 1), 'JOINT')
                .appendField('to')
                .appendField(new Blockly.FieldNumber(90, 0, 180, 1), 'ANGLE')
                .appendField('degrees');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Set servo angle on a joint');
        }
    };

    // Move to stored position
    Blockly.Blocks['move_to_position'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Move to Position')
                .appendField(new Blockly.FieldDropdown(this.getPositions), 'POSITION');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(40, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Move all joints to a stored position at a given speed in degrees per second');
        },
        getPositions: function() {
            // Get positions from positionsManager
            if (typeof getPositionsForBlockly === 'function') {
                const positions = getPositionsForBlockly();
                if (positions.length > 0) {
                    return positions;
                }
            }
            // Default if no positions available
            return [['No positions', '0']];
        },
        onchange: function() {
            // Update dropdown when positions change
            if (typeof getPositionsForBlockly === 'function') {
                const positions = getPositionsForBlockly();
                const dropdown = this.getField('POSITION');
                if (dropdown && positions.length > 0) {
                    dropdown.menuGenerator_ = function() {
                        return positions;
                    };
                }
            }
        }
    };

    // Move TCP to an absolute XYZ position (uses kinematics / inverseKinematics)
    Blockly.Blocks['move_xyz'] = {
        init: function() {
            this.appendValueInput('X')
                .setCheck('Number')
                .appendField('Move TCP to X (mm)');
            this.appendValueInput('Y')
                .setCheck('Number')
                .appendField('Y (mm)');
            this.appendValueInput('Z')
                .setCheck('Number')
                .appendField('Z (mm)');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(40, 0, 300, 1), 'SPEED')
                .appendField('mm/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(200);
            this.setTooltip('Move the end effector to an absolute XYZ position using kinematics. Plug in a number, variable, or math block for X/Y/Z.');
        }
    };

    // Move TCP by an XYZ offset relative to the current position
    Blockly.Blocks['move_xyz_offset'] = {
        init: function() {
            this.appendValueInput('DX')
                .setCheck('Number')
                .appendField('Move TCP by dX (mm)');
            this.appendValueInput('DY')
                .setCheck('Number')
                .appendField('dY (mm)');
            this.appendValueInput('DZ')
                .setCheck('Number')
                .appendField('dZ (mm)');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(40, 0, 300, 1), 'SPEED')
                .appendField('mm/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(200);
            this.setTooltip('Move the end effector by an XYZ offset from the current position using kinematics. Plug in a number, variable, or math block for dX/dY/dZ.');
        }
    };

    // Wait for seconds
    Blockly.Blocks['wait_seconds'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Wait')
                .appendField(new Blockly.FieldNumber(1, 0, 60, 0.1), 'SECONDS')
                .appendField('seconds');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip('Wait for a specified number of seconds');
        }
    };

    // Wait until joint stops moving
    Blockly.Blocks['wait_until_stopped'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Wait until Joint')
                .appendField(new Blockly.FieldNumber(1, 1, 6, 1), 'JOINT')
                .appendField('stops moving');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip('Wait until a joint finishes moving');
        }
    };

    // Wait until all joints stop moving
    Blockly.Blocks['wait_until_all_stopped'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Wait until all joints stop moving');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
            this.setTooltip('Wait until all joints finish moving');
        }
    };

    // Set acceleration for a joint
    Blockly.Blocks['set_acceleration'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Set Acceleration for Joint')
                .appendField(new Blockly.FieldNumber(1, 1, 6, 1), 'JOINT')
                .appendField('to')
                .appendField(new Blockly.FieldNumber(5, 0, 254, 1), 'ACCELERATION')
                .appendField('(0-254)');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Set the acceleration for a specific joint. Range: 0-254 (unit: 100 step/s²). Higher values = faster acceleration.');
        }
    };

    // Tool: Gripper open
    Blockly.Blocks['gripper_open'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Open Gripper');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Open the gripper (moves end-tool servo to the shared open position, 141°)');
        }
    };

    // Tool: Gripper close
    Blockly.Blocks['gripper_close'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Close Gripper');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Close the gripper (moves end-tool servo to 0°)');
        }
    };

    // Tool: Pump on (vacuum = pump + solenoid)
    Blockly.Blocks['pump_on'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Vacuum On');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Turn the vacuum on (pump + solenoid valve both on)');
        }
    };

    // Tool: Pump off
    Blockly.Blocks['pump_off'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Vacuum Off');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Turn the vacuum off (pump + solenoid valve both off)');
        }
    };

    // Tool: Solenoid on
    Blockly.Blocks['solenoid_on'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Solenoid On');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Open the solenoid valve (independent of pump)');
        }
    };

    // Tool: Solenoid off
    Blockly.Blocks['solenoid_off'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Solenoid Off');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Close the solenoid valve (independent of pump)');
        }
    };

    // Tool: End tool servo angle
    Blockly.Blocks['end_tool_servo'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Set End Tool Servo to')
                .appendField(new Blockly.FieldNumber(90, 0, 180, 1), 'ANGLE')
                .appendField('degrees');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(165);
            this.setTooltip('Set the end tool servo to a specific angle (0° = closed, 180° = open)');
        }
    };

    // Vision: how many blocks the camera currently sees. Blocks are indexed
    // 0, 1, 2… by position in the camera's view, nearest the top first — a
    // block lower down in view always gets a higher index.
    Blockly.Blocks['block_count'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('detected block count');
            this.setOutput(true, 'Number');
            this.setColour(20);
            this.setTooltip('Number of coloured blocks the camera currently sees');
        }
    };

    Blockly.Blocks['block_x_at'] = {
        init: function() {
            this.appendValueInput('INDEX')
                .setCheck('Number')
                .appendField('block X (mm) at index');
            this.setInputsInline(true);
            this.setOutput(true, 'Number');
            this.setColour(20);
            this.setTooltip('World X position in mm of the detected block at this index. Requires ArUco markers to be visible.');
        }
    };

    Blockly.Blocks['block_y_at'] = {
        init: function() {
            this.appendValueInput('INDEX')
                .setCheck('Number')
                .appendField('block Y (mm) at index');
            this.setInputsInline(true);
            this.setOutput(true, 'Number');
            this.setColour(20);
            this.setTooltip('World Y position in mm of the detected block at this index. Requires ArUco markers to be visible.');
        }
    };

    Blockly.Blocks['block_color_at'] = {
        init: function() {
            this.appendValueInput('INDEX')
                .setCheck('Number')
                .appendField('block colour at index');
            this.setInputsInline(true);
            this.setOutput(true, 'String');
            this.setColour(20);
            this.setTooltip('Colour name of the detected block at this index');
        }
    };

    Blockly.Blocks['save_block_to_position'] = {
        init: function() {
            this.appendValueInput('INDEX')
                .setCheck('Number')
                .appendField('save block at index');
            this.appendValueInput('SLOT')
                .setCheck('Number')
                .appendField('to position slot');
            this.appendValueInput('Z')
                .setCheck('Number')
                .appendField('at height (mm)');
            this.setInputsInline(true);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
            this.setTooltip("Saves the detected block's position into a Stored Position slot (0-99) so it can be moved to later");
        }
    };
}

/**
 * Generates JavaScript code from Blockly workspace
 */
function generateBlocklyCode() {
    if (!blocklyWorkspace) {
        return '';
    }

    // Generate JavaScript code
    const code = Blockly.JavaScript.workspaceToCode(blocklyWorkspace);
    return code;
}

/**
 * Runs the Blockly program
 */
async function runBlocklyProgram() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi. Please connect first.');
        return;
    }

    if (blocklyProgramRunning) {
        showAppMessage('Program is already running');
        return;
    }

    if (!blocklyWorkspace) {
        showAppMessage('Blockly workspace not initialized');
        return;
    }

    // Generate code from blocks
    const code = generateBlocklyCode();
    
    if (!code || code.trim() === '') {
        showAppMessage('No blocks in workspace. Add some blocks to create a program.');
        return;
    }

    // Clear output
    document.getElementById('blocklyOutput').textContent = '';
    document.getElementById('blocklyStatus').textContent = 'Running...';
    blocklyProgramRunning = true;
    blocklyProgramStopRequested = false;
    blocklyProgramPaused = false;
    
    // Update button states
    updateBlocklyButtonStates();

    try {
        // Start program execution
        appendBlocklyOutput('Program started');
        
        // Create a helper function to check for pause/stop
        const checkBlocklyPauseStop = async () => {
            // Check for stop request
            if (blocklyProgramStopRequested) {
                throw new Error('Program stopped by user');
            }
            
            // Check for pause
            while (blocklyProgramPaused && !blocklyProgramStopRequested) {
                await new Promise(resolve => {
                    blocklyProgramResumeResolve = resolve;
                });
            }
            
            // Check for stop again after resume
            if (blocklyProgramStopRequested) {
                throw new Error('Program stopped by user');
            }
        };
        
        // Create an async function from the generated code
        // We need to provide robotArmClient, helper functions, pause/stop checking, block highlighting, and conversion functions in the scope
        // Variables are automatically handled by Blockly's JavaScript generator
        const asyncFunction = new Function('robotArmClient', 'getNumJoints', 'appendBlocklyOutput', 'checkBlocklyPauseStop', 'highlightBlocklyBlock', 'degreesPerSecondToStepsPerSecond', 'calculateScaledSpeeds', 
            `return (async function() {
                ${code}
            })();`
        );
        
        // Execute the async function
        await asyncFunction(robotArmClient, getNumJoints, appendBlocklyOutput, checkBlocklyPauseStop, highlightBlocklyBlock, degreesPerSecondToStepsPerSecond, calculateScaledSpeeds);

        if (!blocklyProgramStopRequested) {
            document.getElementById('blocklyStatus').textContent = 'Program completed';
            appendBlocklyOutput('Program completed');
        } else {
            document.getElementById('blocklyStatus').textContent = 'Program stopped';
            appendBlocklyOutput('Program stopped');
        }
    } catch (error) {
        console.error('Blockly program error:', error);
        if (error.message === 'Program stopped by user') {
            document.getElementById('blocklyStatus').textContent = 'Program stopped';
            appendBlocklyOutput('Program stopped');
        } else {
            document.getElementById('blocklyStatus').textContent = 'Error: ' + error.message;
            appendBlocklyOutput(`Error: ${error.message}`);
            showAppMessage('Program error: ' + error.message);
        }
    } finally {
        blocklyProgramRunning = false;
        blocklyProgramPaused = false;
        blocklyProgramResumeResolve = null;
        clearBlocklyHighlight(); // Clear any highlighting when program ends
        updateBlocklyButtonStates();
    }
}

/**
 * Stops the running Blockly program
 */
function stopBlocklyProgram() {
    if (!blocklyProgramRunning) {
        return;
    }

    blocklyProgramStopRequested = true;
    blocklyProgramPaused = false;
    
    // Resume if paused so stop can take effect immediately
    if (blocklyProgramResumeResolve) {
        blocklyProgramResumeResolve();
        blocklyProgramResumeResolve = null;
    }
    
    // Stop all joints immediately
    robotArmClient.stopAllJoints();
    document.getElementById('blocklyStatus').textContent = 'Stopping...';
    
    // Clear block highlighting
    clearBlocklyHighlight();
    
    updateBlocklyButtonStates();
}

/**
 * Toggles between pause and resume for Blockly program execution
 */
function toggleBlocklyPauseResume() {
    const pauseButton = document.getElementById('pauseBlocklyButton');
    
    if (blocklyProgramPaused) {
        // Resume execution
        blocklyProgramPaused = false;
        document.getElementById('blocklyStatus').textContent = 'Running...';
        pauseButton.textContent = 'Pause';
        pauseButton.classList.remove('btn-success');
        pauseButton.classList.add('btn-warning');
        
        // Resume the program
        if (blocklyProgramResumeResolve) {
            blocklyProgramResumeResolve();
            blocklyProgramResumeResolve = null;
        }
    } else {
        // Pause execution
        blocklyProgramPaused = true;
        document.getElementById('blocklyStatus').textContent = 'Paused';
        pauseButton.textContent = 'Resume';
        pauseButton.classList.remove('btn-warning');
        pauseButton.classList.add('btn-success');
    }
}

/**
 * Updates Blockly control button states based on program status
 */
function updateBlocklyButtonStates() {
    const runButton = document.getElementById('runBlocklyButton');
    const pauseButton = document.getElementById('pauseBlocklyButton');
    const stopButton = document.getElementById('stopBlocklyButton');
    
    if (blocklyProgramRunning) {
        if (runButton) runButton.disabled = true;
        if (pauseButton) pauseButton.disabled = false;
        if (stopButton) stopButton.disabled = false;
        
        // Update pause button appearance
        if (pauseButton) {
            if (blocklyProgramPaused) {
                pauseButton.textContent = 'Resume';
                pauseButton.classList.remove('btn-warning');
                pauseButton.classList.add('btn-success');
            } else {
                pauseButton.textContent = 'Pause';
                pauseButton.classList.remove('btn-success');
                pauseButton.classList.add('btn-warning');
            }
        }
    } else {
        if (runButton) runButton.disabled = false;
        if (pauseButton) pauseButton.disabled = true;
        if (stopButton) stopButton.disabled = true;
        
        // Reset pause button
        if (pauseButton) {
            pauseButton.textContent = 'Pause';
            pauseButton.classList.remove('btn-success');
            pauseButton.classList.add('btn-warning');
        }
    }
}

/**
 * Clears the Blockly workspace
 */
function clearBlocklyWorkspace() {
    if (blocklyWorkspace) {
        blocklyWorkspace.clear();
        document.getElementById('blocklyOutput').textContent = '';
        document.getElementById('blocklyStatus').textContent = 'Workspace cleared';
    }
}

/**
 * Loads an example program
 */
function loadBlocklyExample() {
    if (!blocklyWorkspace) {
        return;
    }

    clearBlocklyWorkspace();

    // Create example blocks programmatically
    // This example demonstrates: move multiple joints simultaneously, wait, move individual joint, and home
    const moveAll1 = blocklyWorkspace.newBlock('move_all_joints');
    
    // Create number blocks for each joint angle
    const num1_1 = blocklyWorkspace.newBlock('math_number');
    num1_1.setFieldValue('30', 'NUM');
    num1_1.initSvg();
    num1_1.render();
    moveAll1.getInput('JOINT1').connection.connect(num1_1.outputConnection);
    
    const num1_2 = blocklyWorkspace.newBlock('math_number');
    num1_2.setFieldValue('20', 'NUM');
    num1_2.initSvg();
    num1_2.render();
    moveAll1.getInput('JOINT2').connection.connect(num1_2.outputConnection);
    
    const num1_3 = blocklyWorkspace.newBlock('math_number');
    num1_3.setFieldValue('15', 'NUM');
    num1_3.initSvg();
    num1_3.render();
    moveAll1.getInput('JOINT3').connection.connect(num1_3.outputConnection);
    
    const num1_4 = blocklyWorkspace.newBlock('math_number');
    num1_4.setFieldValue('10', 'NUM');
    num1_4.initSvg();
    num1_4.render();
    moveAll1.getInput('JOINT4').connection.connect(num1_4.outputConnection);
    
    const num1_5 = blocklyWorkspace.newBlock('math_number');
    num1_5.setFieldValue('0', 'NUM');
    num1_5.initSvg();
    num1_5.render();
    moveAll1.getInput('JOINT5').connection.connect(num1_5.outputConnection);
    
    const num1_6 = blocklyWorkspace.newBlock('math_number');
    num1_6.setFieldValue('0', 'NUM');
    num1_6.initSvg();
    num1_6.render();
    moveAll1.getInput('JOINT6').connection.connect(num1_6.outputConnection);
    
    moveAll1.setFieldValue('40', 'SPEED');
    moveAll1.initSvg();
    moveAll1.render();

    const wait1 = blocklyWorkspace.newBlock('wait_seconds');
    wait1.setFieldValue('2', 'SECONDS');
    wait1.initSvg();
    wait1.render();
    moveAll1.nextConnection.connect(wait1.previousConnection);

    const moveJoint1 = blocklyWorkspace.newBlock('move_joint');
    moveJoint1.setFieldValue('1', 'JOINT');
    moveJoint1.setFieldValue('45', 'ANGLE');
    moveJoint1.setFieldValue('40', 'SPEED');
    moveJoint1.initSvg();
    moveJoint1.render();
    wait1.nextConnection.connect(moveJoint1.previousConnection);

    const wait2 = blocklyWorkspace.newBlock('wait_seconds');
    wait2.setFieldValue('1', 'SECONDS');
    wait2.initSvg();
    wait2.render();
    moveJoint1.nextConnection.connect(wait2.previousConnection);

    const moveAll2 = blocklyWorkspace.newBlock('move_all_joints');
    
    // Create number blocks for home position (all zeros)
    const num2_1 = blocklyWorkspace.newBlock('math_number');
    num2_1.setFieldValue('0', 'NUM');
    num2_1.initSvg();
    num2_1.render();
    moveAll2.getInput('JOINT1').connection.connect(num2_1.outputConnection);
    
    const num2_2 = blocklyWorkspace.newBlock('math_number');
    num2_2.setFieldValue('0', 'NUM');
    num2_2.initSvg();
    num2_2.render();
    moveAll2.getInput('JOINT2').connection.connect(num2_2.outputConnection);
    
    const num2_3 = blocklyWorkspace.newBlock('math_number');
    num2_3.setFieldValue('0', 'NUM');
    num2_3.initSvg();
    num2_3.render();
    moveAll2.getInput('JOINT3').connection.connect(num2_3.outputConnection);
    
    const num2_4 = blocklyWorkspace.newBlock('math_number');
    num2_4.setFieldValue('0', 'NUM');
    num2_4.initSvg();
    num2_4.render();
    moveAll2.getInput('JOINT4').connection.connect(num2_4.outputConnection);
    
    const num2_5 = blocklyWorkspace.newBlock('math_number');
    num2_5.setFieldValue('0', 'NUM');
    num2_5.initSvg();
    num2_5.render();
    moveAll2.getInput('JOINT5').connection.connect(num2_5.outputConnection);
    
    const num2_6 = blocklyWorkspace.newBlock('math_number');
    num2_6.setFieldValue('0', 'NUM');
    num2_6.initSvg();
    num2_6.render();
    moveAll2.getInput('JOINT6').connection.connect(num2_6.outputConnection);
    
    moveAll2.setFieldValue('40', 'SPEED');
    moveAll2.initSvg();
    moveAll2.render();
    wait2.nextConnection.connect(moveAll2.previousConnection);

    // Position blocks - only move the top block (moveAll1), connected blocks move with it
    // Connected blocks cannot be moved individually
    moveAll1.moveBy(20, 20);

    document.getElementById('blocklyStatus').textContent = 'Example loaded';
}

/**
 * Saves the current Blockly program to localStorage and optionally downloads as file
 */
function saveBlocklyProgram() {
    if (!blocklyWorkspace) {
        appendBlocklyOutput('Error: Blockly workspace not initialized');
        return;
    }

    try {
        if (typeof Blockly === 'undefined' || !Blockly.Xml || typeof Blockly.Xml.workspaceToDom !== 'function') {
            throw new Error('Blockly XML support is not available. Please refresh the page.');
        }

        // Get DOM from workspace
        const xmlDom = Blockly.Xml.workspaceToDom(blocklyWorkspace);
        // Convert DOM to text using the browser's XMLSerializer (works across Blockly versions)
        const serializer = new XMLSerializer();
        const xmlText = serializer.serializeToString(xmlDom);

        // Save to localStorage
        localStorage.setItem('blocklyProgram', xmlText);
        appendBlocklyOutput('Program saved');

        // Also offer to download as file
        const blob = new Blob([xmlText], { type: 'application/xml' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'robot-arm-program.xml';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        document.getElementById('blocklyStatus').textContent = 'Program saved';
    } catch (error) {
        console.error('Error saving Blockly program:', error);
        appendBlocklyOutput('Error saving program: ' + error.message);
        document.getElementById('blocklyStatus').textContent = 'Error saving program';
    }
}

/**
 * Loads a Blockly program from a file
 * Always shows file dialog when called
 */
function loadBlocklyProgram() {
    if (!blocklyWorkspace) {
        appendBlocklyOutput('Error: Blockly workspace not initialized');
        return;
    }

    // Always show file dialog - we'll check XML module when file is selected
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.xml';
    input.onchange = function(event) {
        const file = event.target.files[0];
        if (!file) {
            return;
        }

        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const xmlText = e.target.result;
                
                // Check if we have valid XML content
                if (!xmlText || xmlText.trim().length === 0) {
                    throw new Error('File is empty');
                }
                
                // Check basic Blockly XML support
                if (typeof Blockly === 'undefined' || !Blockly.Xml || typeof Blockly.Xml.domToWorkspace !== 'function') {
                    showAppMessage('Blockly XML support is not available. Please refresh the page.');
                    throw new Error('Blockly XML support is not available.');
                }

                // Parse XML text into a DOM document
                let xmlDoc;
                try {
                    const parser = new DOMParser();
                    xmlDoc = parser.parseFromString(xmlText, 'text/xml');
                } catch (parseError) {
                    console.error('XML parsing error:', parseError);
                    throw new Error('Failed to parse XML: ' + parseError.message);
                }
                
                const xmlRoot = xmlDoc && xmlDoc.documentElement ? xmlDoc.documentElement : null;
                if (!xmlRoot) {
                    throw new Error('Failed to parse XML document - no root element found');
                }
                
                // Clear workspace before loading
                blocklyWorkspace.clear();
                
                // Load the XML into the workspace
                try {
                    Blockly.Xml.domToWorkspace(xmlRoot, blocklyWorkspace);
                } catch (loadError) {
                    console.error('Error loading blocks into workspace:', loadError);
                    throw new Error('Failed to load blocks into workspace: ' + loadError.message);
                }
                
                // Verify blocks were loaded
                const blocks = blocklyWorkspace.getAllBlocks(false);
                if (blocks.length === 0) {
                    console.warn('No blocks found in loaded XML');
                    appendBlocklyOutput('Warning: File loaded but no blocks found. The file might be empty or invalid.');
                } else {
                    appendBlocklyOutput(`Program loaded from file: ${file.name} (${blocks.length} block${blocks.length !== 1 ? 's' : ''})`);
                }
                
                // Also save to localStorage for next time
                localStorage.setItem('blocklyProgram', xmlText);
                
                document.getElementById('blocklyStatus').textContent = 'Program loaded';
            } catch (error) {
                console.error('Error loading file:', error);
                appendBlocklyOutput('Error loading file: ' + error.message);
                document.getElementById('blocklyStatus').textContent = 'Error loading program';
                showAppMessage('Failed to load Blockly program: ' + error.message);
            }
        };
        reader.readAsText(file);
    };
    input.click();
}

/**
 * Auto-loads a Blockly program from localStorage on startup
 * This is called automatically, not by user action
 */
function autoLoadBlocklyProgram() {
    if (!blocklyWorkspace) {
        return;
    }

    // Try to load from localStorage (auto-load on startup)
    const savedProgram = localStorage.getItem('blocklyProgram');
    if (savedProgram) {
        try {
            if (typeof Blockly === 'undefined' || !Blockly.Xml || typeof Blockly.Xml.domToWorkspace !== 'function') {
                console.warn('Blockly XML support not available yet, skipping auto-load from localStorage');
                return;
            }
            
            const parser = new DOMParser();
            const xmlDoc = parser.parseFromString(savedProgram, 'text/xml');
            const xmlRoot = xmlDoc && xmlDoc.documentElement ? xmlDoc.documentElement : null;
            if (!xmlRoot) {
                console.warn('Saved Blockly XML has no root element, skipping auto-load');
                return;
            }
            
            blocklyWorkspace.clear();
            Blockly.Xml.domToWorkspace(xmlRoot, blocklyWorkspace);
            // Don't show output for auto-load
        } catch (error) {
            console.error('Error auto-loading from localStorage:', error);
            // Silently fail for auto-load
        }
    }
}

/**
 * Appends text to the output area
 */
// Surface move retries (bus faults) in the program log so a pause is explained.
if (typeof robotArmClient !== 'undefined' && robotArmClient) {
    robotArmClient.onMoveRetry = (msg) => appendBlocklyOutput(msg);
}

/**
 * Turns the tool to currentToolOrientation without moving the tool tip.
 * Works in joint space so nothing is lost to a rounded XYZ round trip:
 *  - reads the current joint angles and the server's own FK of them;
 *  - if the requested pointing vector already matches the tool's current
 *    Z axis, only joint 6 is turned (server applyToolSpin); joints 1-5 are
 *    not even re-sent;
 *  - otherwise the pose is re-solved at the server-FK position with the
 *    current angles as seed and reference, so the wrist re-poses in place.
 * Shared by the Blockly "Set tool orientation" block, G-code "G1 I J K [R] [F]"
 * and RAPID "SetToolOri" so all three turn the tool when the command runs, at
 * its own speed, rather than folding it into the next move.
 * @param {number} speedStepsPerSecond
 * @param {function(string)} [log] - where progress messages go (default: Blockly output)
 */
async function applyToolOrientationInPlace(speedStepsPerSecond, log) {
    const appendOutput = typeof log === 'function' ? log : appendBlocklyOutput;
    if (!currentToolOrientation) return;
    if (!robotArmClient || !robotArmClient.isConnected) {
        appendOutput('Not connected \u2014 orientation stored for the next move.');
        return;
    }
    if (typeof robotKinematics === 'undefined' || !robotKinematics.isConfigured()) {
        appendOutput('Kinematics not configured \u2014 orientation stored for the next move.');
        return;
    }

    let currentAngles;
    try {
        const status = await robotArmClient.getStatus();
        const numJoints = getNumJoints();
        currentAngles = [];
        for (let i = 0; i < numJoints; i++) {
            if (!status[i] || typeof status[i].angleDegrees !== 'number') throw new Error('joint ' + (i + 1) + ' angle unknown');
            currentAngles.push(status[i].angleDegrees);
        }
    } catch (e) {
        appendOutput('Current joint angles unknown (' + e.message + ') \u2014 orientation stored for the next move.');
        return;
    }

    const fk = await robotArmClient.forwardKinematics(currentAngles);
    const T = fk && fk.rotation;
    // Tool points along -Z of the last joint frame (see toolZAxisFromMatrix on the server).
    const toolZ = T ? { x: -T[0][2], y: -T[1][2], z: -T[2][2] } : null;
    const want = currentToolOrientation;
    const wantLen = Math.sqrt(want.x * want.x + want.y * want.y + want.z * want.z) || 1;
    const dot = toolZ ? (want.x * toolZ.x + want.y * toolZ.y + want.z * toolZ.z) / wantLen : -2;
    const pointingErrorDeg = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;

    let targetAngles;
    let spinErrorDeg = null;
    if (pointingErrorDeg <= 2.0 && currentAngles.length >= 6) {
        // Pointing direction already right: spin joint 6 only.
        const spun = await robotArmClient.applyToolSpin(currentAngles, want);
        targetAngles = spun.angles;
        spinErrorDeg = spun.spinErrorDeg;
        const j6 = targetAngles.length - 1;
        if (Math.abs(targetAngles[j6] - currentAngles[j6]) < 0.2) {
            appendOutput('Tool orientation already set (joint 6 at ' + currentAngles[j6].toFixed(1) + '\u00b0)');
            return;
        }
        await robotArmClient.moveJoint(j6 + 1, targetAngles[j6], speedStepsPerSecond);
    } else {
        // Pointing direction changes: re-solve at the server's own FK position
        // so the tip stays put, seeded and referenced by the current angles.
        const target = { x: fk.position.x, y: fk.position.y, z: fk.position.z };
        const baseAngles = await robotArmClient.inverseKinematics({ ...target, orientation: want }, currentAngles);
        if (!baseAngles) {
            appendOutput('Could not reach that orientation at the current position \u2014 stored for the next move.');
            return;
        }
        const refined = await robotArmClient.refineOrientationWithAccuracy(target, baseAngles, want, currentAngles);
        targetAngles = refined.angles;
        spinErrorDeg = typeof refined.spinErrorDeg === 'number' ? refined.spinErrorDeg : null;
        for (let i = 0; i < targetAngles.length; i++) {
            await robotArmClient.moveJoint(i + 1, targetAngles[i], speedStepsPerSecond);
        }
    }
    await robotArmClient.waitForMotionComplete(30000);

    const spinStr = typeof spinErrorDeg === 'number' ? ', spin error ' + formatFiniteNumber(spinErrorDeg, 1) + '\u00b0' : '';
    appendOutput('Tool orientation applied (joint 6 to ' + targetAngles[targetAngles.length - 1].toFixed(1) + '\u00b0' + spinStr + ')');
}

/** Blockly wrapper — kept so generated block code keeps working. */
async function blocklyApplyToolOrientationInPlace(speedStepsPerSecond) {
    return applyToolOrientationInPlace(speedStepsPerSecond, appendBlocklyOutput);
}

function appendBlocklyOutput(text) {
    const output = document.getElementById('blocklyOutput');
    const timestamp = new Date().toLocaleTimeString();
    // Format similar to G-code log: [HH:MM:SS] Message
    const line = `[${timestamp}] ${text}`;
    if (typeof appendCappedLog === 'function') {
        appendCappedLog(output, line);
    } else if (output) {
        // app.js not loaded (shouldn't happen in the real app) — fall back
        // to the old unbounded behavior rather than throwing.
        output.textContent += line + '\n';
        output.scrollTop = output.scrollHeight;
    }
}

/**
 * Sets the acceleration value for all joints from the Visual Programming tab
 * This is a simple helper that asks for one acceleration value (0-254)
 * and applies it to every joint using the existing setAcceleration API.
 */
function setAllJointAccelerations() {
    // Make sure we are connected before sending commands
    if (!robotArmClient || !robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi. Please connect first.');
        return;
    }

    // Ask for a single acceleration value. showPrompt is the in-app dialog:
    // window.prompt draws OS buttons that cannot be made touch-sized.
    showPrompt('Enter acceleration value for all joints (0-254):', '5').then(input => {
        // If the user cancelled, do nothing
        if (input === null) {
            return;
        }

        const parsed = parseInt(input, 10);
        if (isNaN(parsed) || parsed < 0 || parsed > 254) {
            showAppMessage('Please enter a whole number between 0 and 254.');
            return;
        }

        applyAccelerationToAllJoints(parsed);
    });
}

/**
 * Sends one acceleration value to every joint.
 * @param {number} accelerationValue - Acceleration, 0-254 (unit: 100 step/s²)
 */
function applyAccelerationToAllJoints(accelerationValue) {
    // Get number of joints (falls back to 6 if function not available)
    let numJoints = 6;
    if (typeof getNumJoints === 'function') {
        const n = getNumJoints();
        if (!isNaN(n) && n > 0) {
            numJoints = n;
        }
    }

    appendBlocklyOutput(`Setting acceleration for all joints to ${accelerationValue} (unit: 100 step/s²)`);

    // Apply acceleration to each joint
    for (let i = 1; i <= numJoints; i++) {
        try {
            if (typeof robotArmClient.setAcceleration === 'function') {
                robotArmClient.setAcceleration(i, accelerationValue);
            }
        } catch (error) {
            console.error(`Failed to set acceleration for Joint ${i}:`, error);
        }
    }
}

/**
 * Highlights a Blockly block by its ID
 * @param {string} blockId - The ID of the block to highlight
 */
function highlightBlocklyBlock(blockId) {
    if (!blocklyWorkspace || !blockId) return;
    
    // Clear previous highlight
    if (currentHighlightedBlock) {
        currentHighlightedBlock.setHighlighted(false);
    }
    
    // Get the block and highlight it
    const block = blocklyWorkspace.getBlockById(blockId);
    if (block) {
        block.setHighlighted(true);
        currentHighlightedBlock = block;
        
        // Scroll the block into view
        blocklyWorkspace.centerOnBlock(blockId);
    }
}

/**
 * Clears the current block highlight
 */
function clearBlocklyHighlight() {
    if (currentHighlightedBlock) {
        currentHighlightedBlock.setHighlighted(false);
        currentHighlightedBlock = null;
    }
}

/**
 * Generates JavaScript code for custom blocks
 * This function registers all Blockly code generators
 * It's called when Blockly is loaded to avoid "Blockly is not defined" errors
 */
function registerBlocklyGenerators() {
    // Only register if Blockly is loaded
    if (typeof Blockly === 'undefined' || typeof Blockly.JavaScript === 'undefined') {
        console.warn('Blockly not loaded yet, deferring generator registration');
        // Try again after a short delay
        setTimeout(registerBlocklyGenerators, 100);
        return;
    }
    
    // Register all code generators
    Blockly.JavaScript['move_joint'] = function(block) {
        const blockId = block.id;
        const joint = block.getFieldValue('JOINT');
        const angle = block.getFieldValue('ANGLE');
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 40;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Moving Joint ${joint} to ${angle}° at speed ${speedDegreesPerSecond} degrees/s (dead-zone aware)');
        {
            const numJoints = getNumJoints();
            const status = await robotArmClient.getStatus();
            const targetAngles = [];
            for (let i = 0; i < numJoints; i++) {
                if (status[i] && typeof status[i].angleDegrees === 'number') {
                    targetAngles.push(status[i].angleDegrees);
                } else {
                    targetAngles.push(0);
                }
            }
            targetAngles[${joint} - 1] = ${angle};
            await moveJointsToAnglesWithDeadZones(targetAngles, ${speedDegreesPerSecond});
        }
        `;
    };

    Blockly.JavaScript['move_all_joints'] = function(block) {
        const blockId = block.id;
        const joint1 = Blockly.JavaScript.valueToCode(block, 'JOINT1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const joint2 = Blockly.JavaScript.valueToCode(block, 'JOINT2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const joint3 = Blockly.JavaScript.valueToCode(block, 'JOINT3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const joint4 = Blockly.JavaScript.valueToCode(block, 'JOINT4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const joint5 = Blockly.JavaScript.valueToCode(block, 'JOINT5', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const joint6 = Blockly.JavaScript.valueToCode(block, 'JOINT6', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 40;
        // Sanitize block ID to make it a valid JavaScript identifier
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        // Use unique variable names based on block ID to avoid conflicts
        const targetAnglesVar = 'targetAngles_all_' + sanitizedId;
        
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        {
            const ${targetAnglesVar} = [${joint1}, ${joint2}, ${joint3}, ${joint4}, ${joint5}, ${joint6}];
            appendBlocklyOutput('Moving all joints to [' + ${targetAnglesVar}.join(', ') + '] at speed ${speedDegreesPerSecond} degrees/s (dead-zone aware)');
            await moveJointsToAnglesWithDeadZones(${targetAnglesVar}, ${speedDegreesPerSecond});
        }
    `;
    };

    Blockly.JavaScript['stop_joint'] = function(block) {
        const blockId = block.id;
        const joint = block.getFieldValue('JOINT');
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Stopping Joint ${joint}');
        await robotArmClient.stopJoint(${joint});
        `;
    };

    Blockly.JavaScript['stop_all'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Stopping all joints');
        robotArmClient.stopAllJoints();
        `;
    };

    Blockly.JavaScript['set_acceleration'] = function(block) {
        const blockId = block.id;
        const joint = block.getFieldValue('JOINT');
        const acceleration = block.getFieldValue('ACCELERATION') || 5;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Setting acceleration for Joint ${joint} to ${acceleration} (unit: 100 step/s²)');
        await robotArmClient.setAcceleration(${joint}, ${acceleration});
        `;
    };

    Blockly.JavaScript['set_servo'] = function(block) {
        const blockId = block.id;
        const joint = block.getFieldValue('JOINT');
        const angle = block.getFieldValue('ANGLE');
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Setting servo on Joint ${joint} to ${angle}°');
        await robotArmClient.setServoAngle(${joint}, ${angle});
        `;
    };

    Blockly.JavaScript['wait_seconds'] = function(block) {
        const blockId = block.id;
        const seconds = block.getFieldValue('SECONDS');
        // Sanitize block ID to make it a valid JavaScript identifier (remove invalid characters)
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        // Use unique variable names based on block ID to avoid conflicts with multiple wait blocks
        const waitTimeVar = `waitTime_${sanitizedId}`;
        const elapsedVar = `elapsed_${sanitizedId}`;
        const chunkVar = `chunk_${sanitizedId}`;
        const checkIntervalVar = `checkInterval_${sanitizedId}`;
        // Check for pause/stop during the wait by breaking it into smaller intervals
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Waiting ${seconds} seconds');
        const ${waitTimeVar} = ${seconds * 1000};
        const ${checkIntervalVar} = 100; // Check every 100ms
        let ${elapsedVar} = 0;
        while (${elapsedVar} < ${waitTimeVar}) {
            await checkBlocklyPauseStop();
            const ${chunkVar} = Math.min(${checkIntervalVar}, ${waitTimeVar} - ${elapsedVar});
            await new Promise(resolve => setTimeout(resolve, ${chunkVar}));
            ${elapsedVar} += ${chunkVar};
        }
        `;
    };

    Blockly.JavaScript['wait_until_stopped'] = function(block) {
        const blockId = block.id;
        const joint = block.getFieldValue('JOINT');
        // Sanitize block ID to make it a valid JavaScript identifier (remove invalid characters)
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        // Use unique variable names based on block ID to avoid conflicts with multiple wait blocks
        const maxWaitTimeVar = `maxWaitTime_${sanitizedId}`;
        const startTimeVar = `startTime_${sanitizedId}`;
        const statusVar = `status_${sanitizedId}`;
        const checkIntervalVar = `checkInterval_${sanitizedId}`;
        const initialDelayVar = `initialDelay_${sanitizedId}`;
        // Generate inline code to wait for joint to stop, checking for pause/stop periodically
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Waiting for Joint ${joint} to stop moving');
        // Small delay to allow joint to start moving before we check if it has stopped
        const ${initialDelayVar} = 200; // 200ms initial delay
        await new Promise(resolve => setTimeout(resolve, ${initialDelayVar}));
        await checkBlocklyPauseStop();
        const ${maxWaitTimeVar} = 30000; // 30 seconds max
        const ${checkIntervalVar} = 500; // Check every 500ms
        const ${startTimeVar} = Date.now();
        while (Date.now() - ${startTimeVar} < ${maxWaitTimeVar}) {
            await checkBlocklyPauseStop();
            try {
                const ${statusVar} = await robotArmClient.getStatus();
                if (${statusVar} && ${statusVar}.length >= ${joint} && ${statusVar}[${joint} - 1] && !${statusVar}[${joint} - 1].isMoving) {
                    appendBlocklyOutput('Joint ${joint} has stopped');
                    break; // Joint has stopped
                }
            } catch (error) {
                // If status request fails, continue waiting
                console.warn('Failed to get status:', error);
            }
            await new Promise(resolve => setTimeout(resolve, ${checkIntervalVar}));
        }
        `;
    };

    Blockly.JavaScript['wait_until_all_stopped'] = function(block) {
        const blockId = block.id;
        // Sanitize block ID to make it a valid JavaScript identifier (remove invalid characters)
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        // Use unique variable names based on block ID to avoid conflicts with multiple wait blocks
        const maxWaitTimeVar = `maxWaitTime_${sanitizedId}`;
        const startTimeVar = `startTime_${sanitizedId}`;
        const statusVar = `status_${sanitizedId}`;
        const checkIntervalVar = `checkInterval_${sanitizedId}`;
        const initialDelayVar = `initialDelay_${sanitizedId}`;
        const allStoppedVar = `allStopped_${sanitizedId}`;
        const numJointsVar = `numJoints_${sanitizedId}`;
        // Generate inline code to wait for all joints to stop, checking for pause/stop periodically
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Waiting for all joints to stop moving');
        // Small delay to allow joints to start moving before we check if they have stopped
        const ${initialDelayVar} = 200; // 200ms initial delay
        await new Promise(resolve => setTimeout(resolve, ${initialDelayVar}));
        await checkBlocklyPauseStop();
        const ${maxWaitTimeVar} = 30000; // 30 seconds max
        const ${checkIntervalVar} = 500; // Check every 500ms
        const ${startTimeVar} = Date.now();
        const ${numJointsVar} = getNumJoints();
        while (Date.now() - ${startTimeVar} < ${maxWaitTimeVar}) {
            await checkBlocklyPauseStop();
            try {
                const ${statusVar} = await robotArmClient.getStatus();
                if (${statusVar} && ${statusVar}.length >= ${numJointsVar}) {
                    // Check if all joints have stopped
                    let ${allStoppedVar} = true;
                    for (let i = 0; i < ${numJointsVar}; i++) {
                        if (${statusVar}[i] && ${statusVar}[i].isMoving) {
                            ${allStoppedVar} = false;
                            break;
                        }
                    }
                    if (${allStoppedVar}) {
                        appendBlocklyOutput('All joints have stopped');
                        break; // All joints have stopped
                    }
                }
            } catch (error) {
                // If status request fails, continue waiting
                console.warn('Failed to get status:', error);
            }
            await new Promise(resolve => setTimeout(resolve, ${checkIntervalVar}));
        }
        `;
    };

    Blockly.JavaScript['move_to_position'] = function(block) {
        const blockId = block.id;
        // Sanitize block ID to make it a valid JavaScript identifier
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        const positionNumber = block.getFieldValue('POSITION');
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 40;
        // Best-effort label for the log message only — resolved fresh again at
        // runtime below via resolveStoredPositionAngles(), so this doesn't need
        // to be accurate if the position is edited/added after this block was built.
        const positionLabel = (typeof getPosition === 'function' && getPosition(parseInt(positionNumber)) &&
            getPosition(parseInt(positionNumber)).label) || `Position ${positionNumber}`;

        // Target angles are resolved at RUN TIME, not baked in here — a stored
        // position saved as XYZ needs IK against whichever tool is attached when
        // the program actually runs (which may differ from build time), and this
        // also means an angles-type position edited after building the blocks
        // picks up the new values without needing to rebuild.
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();

        // Get current joint angles
        const status_pos_${sanitizedId} = await robotArmClient.getStatus();
        const currentAngles_pos_${sanitizedId} = [];
        const numJoints_pos_${sanitizedId} = getNumJoints();
        for (let i = 0; i < numJoints_pos_${sanitizedId}; i++) {
            if (status_pos_${sanitizedId}[i] && typeof status_pos_${sanitizedId}[i].angleDegrees === 'number') {
                currentAngles_pos_${sanitizedId}.push(status_pos_${sanitizedId}[i].angleDegrees);
            } else {
                currentAngles_pos_${sanitizedId}.push(0);
            }
        }

        // Resolve the stored position to joint angles now (works for angles-type
        // and XYZ-type positions alike).
        const targetAngles_pos_${sanitizedId} = resolveStoredPositionAngles(${positionNumber});
        if (!targetAngles_pos_${sanitizedId}) {
            appendBlocklyOutput('Could not move to Position ${positionNumber} ("${positionLabel}") — it could not be resolved to joint angles (missing, or XYZ target unreachable with the current tool).');
        } else {
            appendBlocklyOutput('Moving to ${positionLabel} at speed ${speedDegreesPerSecond} degrees/s (scaled speeds for synchronized arrival)');
            // Coordinated joint move; a descent is approached from 30 mm above at a
            // capped tip speed (moveToStoredAnglesWithApproach in app.js). Waits for
            // the motion to finish before the next block runs.
            await moveToStoredAnglesWithApproach(currentAngles_pos_${sanitizedId}, targetAngles_pos_${sanitizedId}, ${speedDegreesPerSecond}, appendBlocklyOutput);
        }
        `;
    };

    // Move TCP to absolute XYZ using kinematics
    Blockly.JavaScript['move_xyz'] = function(block) {
        const blockId = block.id;
        const x = Blockly.JavaScript.valueToCode(block, 'X', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const y = Blockly.JavaScript.valueToCode(block, 'Y', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const z = Blockly.JavaScript.valueToCode(block, 'Z', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const mmPerSec = block.getFieldValue('SPEED') || 40;

        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();

        if (!robotKinematics.isConfigured()) {
            appendBlocklyOutput('Kinematics not configured. Cannot run "Move TCP to XYZ" block.');
        } else {
            const targetPose = { x: ${x}, y: ${y}, z: ${z} };

            // Read current XYZ from the UI as start pose
            const _bcp1 = getCurrentDisplayXYZ();
            const startPose = {
                x: isFinite(_bcp1.x) ? _bcp1.x : targetPose.x,
                y: isFinite(_bcp1.y) ? _bcp1.y : targetPose.y,
                z: isFinite(_bcp1.z) ? _bcp1.z : targetPose.z,
            };
            const waypoints = insertApproachWaypoints(planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight), startPose);
            if (!waypoints) {
                appendBlocklyOutput('Move TCP to XYZ cancelled: target lies inside a dead zone.');
            } else {
                appendBlocklyOutput('Moving TCP to X=' + targetPose.x + ' Y=' + targetPose.y + ' Z=' + targetPose.z + ' at ' + ${mmPerSec} + ' mm/s (dead-zone aware path, orientation-aware)');

                for (let w = 0; w < waypoints.length; w++) {
                    const wp = waypoints[w];

                    // Get a starting guess from current joint status if possible
                    let initialAngles = null;
                    try {
                        const status = await robotArmClient.getStatus();
                        const numJoints = getNumJoints();
                        initialAngles = [];
                        for (let i = 0; i < numJoints; i++) {
                            if (status[i] && typeof status[i].angleDegrees === 'number') {
                                initialAngles.push(status[i].angleDegrees);
                            } else {
                                initialAngles.push(0);
                            }
                        }
                    } catch (e) {
                        console.warn('Blockly Move TCP: failed to read status for IK starting guess, using zeros:', e);
                        initialAngles = null;
                    }

                    // Server-only kinematics
                    if (!robotArmClient || !robotArmClient.isConnected || typeof robotArmClient.inverseKinematics !== 'function' || typeof robotArmClient.refineOrientationWithAccuracy !== 'function') {
                        appendBlocklyOutput('Server kinematics is not available. Connect to Raspberry Pi and reload URDF.');
                        break;
                    }
                    const baseAngles = await robotArmClient.inverseKinematics(
                        { x: wp.x, y: wp.y, z: wp.z, orientation: currentToolOrientation },
                        initialAngles
                    );
                    if (!baseAngles) {
                        appendBlocklyOutput('Move TCP to XYZ failed: waypoint unreachable.');
                        break;
                    }

                    // Iterative refinement (coarse to fine) and get accuracy for reporting
                    const refined = await robotArmClient.refineOrientationWithAccuracy(
                        { x: wp.x, y: wp.y, z: wp.z },
                        baseAngles,
                        currentToolOrientation
                    );
                    const refinedAngles = refined.angles;

                    // Each joint gets the speed that makes the tool tip cover this
                    // segment at the block's mm/s with all joints arriving together.
                    const segStart = w === 0 ? startPose : waypoints[w - 1];
                    const segMm = Math.hypot(wp.x - segStart.x, wp.y - segStart.y, wp.z - segStart.z);
                    const segMmPerSec = wp.approach ? Math.min(${mmPerSec}, APPROACH_SPEED_MM_PER_S) : ${mmPerSec};
                    if (wp.approach) {
                        appendBlocklyOutput('Final approach: descending ' + segMm.toFixed(0) + ' mm at ' + segMmPerSec.toFixed(0) + ' mm/s');
                        await new Promise(resolve => setTimeout(resolve, APPROACH_SETTLE_MS));
                    }
                    const tipSpeeds = computeTipSpeeds(initialAngles, refinedAngles, segMm, segMmPerSec);
                    const movePromises = [];
                    for (let i = 0; i < refinedAngles.length; i++) {
                        movePromises.push(robotArmClient.moveJoint(i + 1, refinedAngles[i], tipSpeeds[i]));
                    }
                    // Wait for the bus drain to finish before listening for motion
                    // complete, then for the arm to actually stop. Without this the
                    // next block (gripper, next waypoint) ran while still moving.
                    await Promise.allSettled(movePromises);
                    await robotArmClient.waitForMotionComplete(30000);

                    const posErr = formatFiniteNumber(refined.positionErrorMm, 2);
                    const oriErr = formatFiniteNumber(refined.orientationErrorDeg, 1);
                    const ach = refined.achievedPosition;
                    const xyzStr = ach ? ' X=' + ach.x.toFixed(1) + ' Y=' + ach.y.toFixed(1) + ' Z=' + ach.z.toFixed(1) + ' mm' : '';
                    appendBlocklyOutput('Accuracy: position ' + posErr + ' mm, orientation ' + oriErr + '°' + (xyzStr ? '; achieved' + xyzStr : ''));
                }
            }
        }
        `;
    };

    // Move TCP by XYZ offset using kinematics
    Blockly.JavaScript['move_xyz_offset'] = function(block) {
        const blockId = block.id;
        const dx = Blockly.JavaScript.valueToCode(block, 'DX', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const dy = Blockly.JavaScript.valueToCode(block, 'DY', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const dz = Blockly.JavaScript.valueToCode(block, 'DZ', Blockly.JavaScript.ORDER_ATOMIC) || '0';
        const mmPerSec = block.getFieldValue('SPEED') || 40;

        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();

        if (!robotKinematics.isConfigured()) {
            appendBlocklyOutput('Kinematics not configured. Cannot run "Move TCP by offset" block.');
        } else {
            // Read current XYZ from the UI labels
            const _bcp2 = getCurrentDisplayXYZ();
            const startPose = {
                x: isFinite(_bcp2.x) ? _bcp2.x : 0,
                y: isFinite(_bcp2.y) ? _bcp2.y : 0,
                z: isFinite(_bcp2.z) ? _bcp2.z : 0,
            };
            const targetPose = {
                x: currentX + (${dx}),
                y: currentY + (${dy}),
                z: currentZ + (${dz})
            };

            const waypoints = insertApproachWaypoints(planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight), startPose);
            if (!waypoints) {
                appendBlocklyOutput('Move TCP by offset cancelled: target lies inside a dead zone.');
            } else {
                appendBlocklyOutput(
                    'Moving TCP by dX=' + ${dx} + ' dY=' + ${dy} + ' dZ=' + ${dz} +
                    ' to X=' + targetPose.x + ' Y=' + targetPose.y + ' Z=' + targetPose.z +
                    ' at ' + ${mmPerSec} + ' mm/s (dead-zone aware path, orientation-aware)'
                );

                for (let w = 0; w < waypoints.length; w++) {
                    const wp = waypoints[w];

                    let initialAngles = null;
                    try {
                        const status = await robotArmClient.getStatus();
                        const numJoints = getNumJoints();
                        initialAngles = [];
                        for (let i = 0; i < numJoints; i++) {
                            if (status[i] && typeof status[i].angleDegrees === 'number') {
                                initialAngles.push(status[i].angleDegrees);
                            } else {
                                initialAngles.push(0);
                            }
                        }
                    } catch (e) {
                        console.warn('Blockly Move TCP offset: failed to read status for IK starting guess, using zeros:', e);
                        initialAngles = null;
                    }

                    // Server-only kinematics
                    if (!robotArmClient || !robotArmClient.isConnected || typeof robotArmClient.inverseKinematics !== 'function' || typeof robotArmClient.refineOrientationWithAccuracy !== 'function') {
                        appendBlocklyOutput('Server kinematics is not available. Connect to Raspberry Pi and reload URDF.');
                        break;
                    }
                    const baseAngles = await robotArmClient.inverseKinematics(
                        { x: wp.x, y: wp.y, z: wp.z, orientation: currentToolOrientation },
                        initialAngles
                    );
                    if (!baseAngles) {
                        appendBlocklyOutput('Move TCP by offset failed: waypoint unreachable.');
                        break;
                    }

                    // Iterative refinement (coarse to fine) and get accuracy for reporting
                    const refined = await robotArmClient.refineOrientationWithAccuracy(
                        { x: wp.x, y: wp.y, z: wp.z },
                        baseAngles,
                        currentToolOrientation
                    );
                    const refinedAngles = refined.angles;

                    // Each joint gets the speed that makes the tool tip cover this
                    // segment at the block's mm/s with all joints arriving together.
                    const segStart = w === 0 ? startPose : waypoints[w - 1];
                    const segMm = Math.hypot(wp.x - segStart.x, wp.y - segStart.y, wp.z - segStart.z);
                    const segMmPerSec = wp.approach ? Math.min(${mmPerSec}, APPROACH_SPEED_MM_PER_S) : ${mmPerSec};
                    if (wp.approach) {
                        appendBlocklyOutput('Final approach: descending ' + segMm.toFixed(0) + ' mm at ' + segMmPerSec.toFixed(0) + ' mm/s');
                        await new Promise(resolve => setTimeout(resolve, APPROACH_SETTLE_MS));
                    }
                    const tipSpeeds = computeTipSpeeds(initialAngles, refinedAngles, segMm, segMmPerSec);
                    const movePromises = [];
                    for (let i = 0; i < refinedAngles.length; i++) {
                        movePromises.push(robotArmClient.moveJoint(i + 1, refinedAngles[i], tipSpeeds[i]));
                    }
                    // Wait for the bus drain to finish before listening for motion
                    // complete, then for the arm to actually stop. Without this the
                    // next block (gripper, next waypoint) ran while still moving.
                    await Promise.allSettled(movePromises);
                    await robotArmClient.waitForMotionComplete(30000);

                    const posErr = formatFiniteNumber(refined.positionErrorMm, 2);
                    const oriErr = formatFiniteNumber(refined.orientationErrorDeg, 1);
                    const ach = refined.achievedPosition;
                    const xyzStr = ach ? ' X=' + ach.x.toFixed(1) + ' Y=' + ach.y.toFixed(1) + ' Z=' + ach.z.toFixed(1) + ' mm' : '';
                    appendBlocklyOutput('Accuracy: position ' + posErr + ' mm, orientation ' + oriErr + '°' + (xyzStr ? '; achieved' + xyzStr : ''));
                }
            }
        }
        `;
    };

    Blockly.JavaScript['set_tool_orientation'] = function(block) {
        const blockId = block.id;
        const ox = block.getFieldValue('ORI_X');
        const oy = block.getFieldValue('ORI_Y');
        const oz = block.getFieldValue('ORI_Z');
        const rot = block.getFieldValue('ORI_ROTATION');
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 90;
        const speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Setting tool orientation to vector (${ox}, ${oy}, ${oz}) rotation ${rot}° at ${speedDegreesPerSecond} deg/s');
        setToolOrientationVector(${ox}, ${oy}, ${oz}, ${rot});
        await blocklyApplyToolOrientationInPlace(${speedStepsPerSecond});
        `;
    };

    Blockly.JavaScript['gripper_open'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Opening gripper');
        openGripper();
        await new Promise(resolve => setTimeout(resolve, 500));
        `;
    };

    Blockly.JavaScript['gripper_close'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Closing gripper');
        closeGripper();
        await new Promise(resolve => setTimeout(resolve, 500));
        `;
    };

    Blockly.JavaScript['pump_on'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Vacuum on');
        setVacuum(true);
        `;
    };

    Blockly.JavaScript['pump_off'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Vacuum off');
        setVacuum(false);
        `;
    };

    Blockly.JavaScript['solenoid_on'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Solenoid on');
        setEndToolSolenoidEnabled(true);
        `;
    };

    Blockly.JavaScript['solenoid_off'] = function(block) {
        const blockId = block.id;
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Solenoid off');
        setEndToolSolenoidEnabled(false);
        `;
    };

    Blockly.JavaScript['end_tool_servo'] = function(block) {
        const blockId = block.id;
        const angle = block.getFieldValue('ANGLE');
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('End tool servo to ${angle}°');
        moveEndToolServoTo(${angle});
        await new Promise(resolve => setTimeout(resolve, 500));
        `;
    };

    Blockly.JavaScript['block_count'] = function(block) {
        return ['(await getDetectedBlockCount())', Blockly.JavaScript.ORDER_ATOMIC];
    };

    Blockly.JavaScript['block_x_at'] = function(block) {
        const index = Blockly.JavaScript.valueToCode(block, 'INDEX', Blockly.JavaScript.ORDER_NONE) || '0';
        return [`(await getDetectedBlockXAt(${index}))`, Blockly.JavaScript.ORDER_ATOMIC];
    };

    Blockly.JavaScript['block_y_at'] = function(block) {
        const index = Blockly.JavaScript.valueToCode(block, 'INDEX', Blockly.JavaScript.ORDER_NONE) || '0';
        return [`(await getDetectedBlockYAt(${index}))`, Blockly.JavaScript.ORDER_ATOMIC];
    };

    Blockly.JavaScript['block_color_at'] = function(block) {
        const index = Blockly.JavaScript.valueToCode(block, 'INDEX', Blockly.JavaScript.ORDER_NONE) || '0';
        return [`(await getDetectedBlockColorAt(${index}))`, Blockly.JavaScript.ORDER_ATOMIC];
    };

    Blockly.JavaScript['save_block_to_position'] = function(block) {
        const blockId = block.id;
        const index = Blockly.JavaScript.valueToCode(block, 'INDEX', Blockly.JavaScript.ORDER_NONE) || '0';
        const slot = Blockly.JavaScript.valueToCode(block, 'SLOT', Blockly.JavaScript.ORDER_NONE) || '0';
        const z = Blockly.JavaScript.valueToCode(block, 'Z', Blockly.JavaScript.ORDER_NONE) || '0';
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        {
            const blockIndex = ${index};
            const slotNumber = ${slot};
            const result = await saveDetectedBlockToPositionSlot(blockIndex, slotNumber, ${z});
            appendBlocklyOutput('Saved block ' + blockIndex + ' (' + result.block.color + ') to position ' + slotNumber);
        }
        `;
    };

    console.log('Blockly code generators registered successfully');
}

// Try to register generators immediately (if Blockly is already loaded)
registerBlocklyGenerators();

// Also try to register when page loads (for async loading)
if (typeof window !== 'undefined') {
    window.addEventListener('load', function() {
        setTimeout(registerBlocklyGenerators, 500);
    });
}

// ===== Blockly to G-code / RAPID Conversion Functions =====

// Loops whose condition can't be expressed in G-code (e.g. "repeat while
// true") are converted to a counted loop of this many passes, with a comment
// telling the user how to change it.
const GCODE_FALLBACK_LOOP_COUNT = 10;

function gcodeNum(v, decimals) {
    const d = decimals === undefined ? 1 : decimals;
    return Number(v).toFixed(d).replace(/\.0+$/, '').replace(/(\.\d*?)0+$/, '$1');
}

/**
 * Converts the current Blockly workspace to G-code by walking the block
 * tree directly (not by scraping the generated JavaScript, which changes
 * whenever a block's runtime implementation does).
 *
 * Emits one G-code line per block where the G-code executor has an
 * equivalent (see executeGCodeCommand in app.js). Control flow uses the
 * processor's N-labels, GOTO, IF and #variables (see gcodeProcessor.js):
 * loops become label/IF/GOTO structures, Blockly variables become #n, and
 * values that only exist at run time (block count, block X/Y) are fetched
 * into a #variable with M780/M782/M783 just before they're used.
 *
 * Anything that still can't be expressed is left as a "; NOT CONVERTED:"
 * comment so the gap is visible in the editor rather than silently dropped.
 *
 * @param {Blockly.Workspace} [workspace] - defaults to the live Blockly workspace
 * @returns {string} G-code program
 */
function convertBlocklyToGCode(workspace) {
    const ws = workspace || blocklyWorkspace;
    if (!ws) return '; No blocks in workspace\n';

    const topBlocks = ws.getTopBlocks(true).filter(b =>
        !b.outputConnection && (b.previousConnection || b.nextConnection));
    if (topBlocks.length === 0) return '; No blocks in workspace\n';

    const out = [];
    const notConverted = [];
    const warnings = [];
    let indent = '';

    function emit(line) { out.push(line === '' ? '' : indent + line); }
    function skip(reason) {
        notConverted.push(reason);
        emit(`; NOT CONVERTED: ${reason}`);
    }
    function warn(reason) {
        warnings.push(reason);
        emit(`; WARNING: ${reason}`);
    }
    function isEnabled(b) {
        return typeof b.isEnabled === 'function' ? b.isEnabled() : !b.disabled;
    }

    // ── Labels and variables ─────────────────────────────────────────────
    let nextLabel = 100;
    function newLabel() { const n = nextLabel; nextLabel += 10; return n; }

    // Blockly variable name → #n. User variables are numbered first so
    // they're easy to find; loop counters and temporaries follow.
    const varNumbers = {};
    let nextVar = 1;
    function varFor(name) {
        if (varNumbers[name] === undefined) varNumbers[name] = nextVar++;
        return varNumbers[name];
    }
    function tempVar(purpose) { return varFor(`(${purpose} ${nextVar})`); }
    function blockVarName(b, field) {
        const f = b.getField(field || 'VAR');
        return f ? f.getText() : 'var';
    }

    // ── Loop context for break/continue ──────────────────────────────────
    const loopStack = []; // { continueLabel, breakLabel }

    // ── Values ───────────────────────────────────────────────────────────
    // Converts a value block to a G-code expression string ("45", "#1",
    // "[#1 + 10]"). Returns null if it can't be expressed. May emit prelude
    // lines (M780/M782/M783) that load run-time values into a variable.
    function valueToGCode(b) {
        if (!b) return null;
        switch (b.type) {
            case 'math_number': {
                const v = parseFloat(b.getFieldValue('NUM'));
                return isFinite(v) ? gcodeNum(v, 3) : null;
            }
            case 'variables_get':
                return `#${varFor(blockVarName(b))}`;
            case 'math_negate': {
                const v = inputToGCode(b, 'NUM');
                return v === null ? null : `[0 - ${v}]`;
            }
            case 'math_arithmetic': {
                const a = inputToGCode(b, 'A');
                const c = inputToGCode(b, 'B');
                if (a === null || c === null) return null;
                const op = { ADD: '+', MINUS: '-', MULTIPLY: '*', DIVIDE: '/' }[b.getFieldValue('OP')];
                if (!op) return null; // POWER has no G-code operator
                return `[${a} ${op} ${c}]`;
            }
            case 'block_count': {
                const v = tempVar('block count');
                emit(`M780 V${v} ; detected block count → #${v}`);
                return `#${v}`;
            }
            case 'block_x_at':
            case 'block_y_at': {
                const idx = inputToGCode(b, 'INDEX');
                if (idx === null) return null;
                const v = tempVar(b.type === 'block_x_at' ? 'block X' : 'block Y');
                emit(`${b.type === 'block_x_at' ? 'M782' : 'M783'} P${idx} V${v} ; block ${idx} ${b.type === 'block_x_at' ? 'X' : 'Y'} (mm) → #${v}`);
                return `#${v}`;
            }
        }
        return null;
    }
    function inputToGCode(b, inputName) {
        return valueToGCode(b.getInputTargetBlock(inputName));
    }
    function describeValue(b, inputName) {
        const t = b.getInputTargetBlock(inputName);
        return t ? `"${t.type}" block` : 'empty input';
    }

    // ── Conditions ───────────────────────────────────────────────────────
    // Converts a boolean block to { always: true|false } or
    // { cond: 'A OP B', negated: 'A OP' B' }. Returns null if not expressible.
    const INVERT = { EQ: 'NE', NE: 'EQ', LT: 'GE', GE: 'LT', GT: 'LE', LE: 'GT' };
    function conditionToGCode(b) {
        if (!b) return null;
        switch (b.type) {
            case 'logic_boolean':
                return { always: b.getFieldValue('BOOL') === 'TRUE' };
            case 'logic_negate': {
                const inner = conditionToGCode(b.getInputTargetBlock('BOOL'));
                if (!inner) return null;
                if (inner.always !== undefined) return { always: !inner.always };
                return { cond: inner.negated, negated: inner.cond };
            }
            case 'logic_compare': {
                const a = inputToGCode(b, 'A');
                const c = inputToGCode(b, 'B');
                if (a === null || c === null) return null;
                const op = { EQ: 'EQ', NEQ: 'NE', LT: 'LT', LTE: 'LE', GT: 'GT', GTE: 'GE' }[b.getFieldValue('OP')];
                if (!op) return null;
                return { cond: `${a} ${op} ${c}`, negated: `${a} ${INVERT[op]} ${c}` };
            }
        }
        return null;
    }

    // Emits "jump to `label` unless the condition holds" (used at the top of
    // loops and if-branches).
    function emitJumpUnless(condition, label) {
        if (condition.always === true) return;            // never jump
        if (condition.always === false) { emit(`GOTO ${label}`); return; }
        emit(`IF [${condition.negated}] GOTO ${label}`);
    }

    // ── Statement walking ────────────────────────────────────────────────
    function walkChain(block) {
        for (let b = block; b; b = b.getNextBlock()) {
            if (!isEnabled(b)) { emit(`; (disabled block skipped: ${b.type})`); continue; }
            convertBlock(b);
        }
    }

    function walkBody(block) {
        const saved = indent;
        indent += '  ';
        walkChain(block);
        indent = saved;
    }

    // Standard counted loop: #c = 0 / N top / IF [#c GE count] GOTO end /
    // body / N cont / #c = #c + 1 / GOTO top / N end
    function emitCountedLoop(countExpr, body, title) {
        const c = tempVar('loop counter');
        const top = newLabel(), cont = newLabel(), end = newLabel();
        emit(`; ${title}`);
        emit(`#${c} = 0`);
        emit(`N${top}`);
        emit(`IF [#${c} GE ${countExpr}] GOTO ${end}`);
        loopStack.push({ continueLabel: cont, breakLabel: end });
        walkBody(body);
        loopStack.pop();
        emit(`N${cont}`);
        emit(`#${c} = #${c} + 1`);
        emit(`GOTO ${top}`);
        emit(`N${end}`);
    }

    function convertBlock(b) {
        switch (b.type) {
            // ── Joint-space moves ────────────────────────────────────────
            case 'move_joint': {
                const joint = b.getFieldValue('JOINT');
                const angle = parseFloat(b.getFieldValue('ANGLE'));
                const speed = parseFloat(b.getFieldValue('SPEED')) || 40;
                emit(`G1 J${joint}=${gcodeNum(angle)} F${gcodeNum(speed)}`);
                break;
            }
            case 'move_all_joints': {
                const parts = [];
                const bad = [];
                for (let j = 1; j <= 6; j++) {
                    if (!b.getInputTargetBlock('JOINT' + j)) continue; // empty = leave joint where it is
                    const v = inputToGCode(b, 'JOINT' + j);
                    if (v === null) { bad.push(`joint ${j} (${describeValue(b, 'JOINT' + j)})`); continue; }
                    parts.push(`J${j}=${v}`);
                }
                if (bad.length) { skip(`Move All Joints — can't convert ${bad.join(', ')}`); break; }
                if (parts.length === 0) { emit('; Move All Joints with no joint values'); break; }
                const speed = parseFloat(b.getFieldValue('SPEED')) || 40;
                emit(`G1 ${parts.join(' ')} F${gcodeNum(speed)}`);
                break;
            }
            case 'move_to_position': {
                const slot = parseInt(b.getFieldValue('POSITION'), 10);
                const speed = parseFloat(b.getFieldValue('SPEED')) || 40;
                const pos = (typeof getPosition === 'function') ? getPosition(slot) : null;
                const label = pos && pos.label ? ` ; ${pos.label}` : '';
                emit(`G1 P${slot} F${gcodeNum(speed)}${label}`);
                break;
            }

            // ── Cartesian moves ──────────────────────────────────────────
            case 'move_xyz': {
                const x = inputToGCode(b, 'X');
                const y = inputToGCode(b, 'Y');
                const z = inputToGCode(b, 'Z');
                if (x === null || y === null || z === null) {
                    const bad = ['X', 'Y', 'Z'].filter((n, i) => [x, y, z][i] === null).map(n => `${n} (${describeValue(b, n)})`);
                    skip(`Move TCP to XYZ — can't convert ${bad.join(', ')}`);
                    break;
                }
                // G-code F for Cartesian moves is mm/min; the block is mm/s.
                const mmPerSec = parseFloat(b.getFieldValue('SPEED')) || 40;
                emit(`G1 X${x} Y${y} Z${z} F${gcodeNum(mmPerSec * 60, 0)}`);
                break;
            }
            case 'move_xyz_offset': {
                const dx = inputToGCode(b, 'DX'), dy = inputToGCode(b, 'DY'), dz = inputToGCode(b, 'DZ');
                const desc = (dx === null || dy === null || dz === null) ? 'values can\'t be converted' : `dX${dx} dY${dy} dZ${dz}`;
                skip(`Move TCP by offset (${desc}) — G-code moves are absolute only; use Move TCP to X/Y/Z`);
                break;
            }
            case 'set_tool_orientation': {
                const ox = parseFloat(b.getFieldValue('ORI_X')) || 0;
                const oy = parseFloat(b.getFieldValue('ORI_Y')) || 0;
                const oz = parseFloat(b.getFieldValue('ORI_Z')) || 0;
                const rot = parseFloat(b.getFieldValue('ORI_ROTATION')) || 0;
                const oriSpeed = parseFloat(b.getFieldValue('SPEED')) || 90;
                emit(`G1 I${gcodeNum(ox, 3)} J${gcodeNum(oy, 3)} K${gcodeNum(oz, 3)} R${gcodeNum(rot)} F${gcodeNum(oriSpeed)} ; turn tool to this orientation now`);
                break;
            }

            // ── Timing ───────────────────────────────────────────────────
            case 'wait_seconds': {
                const secs = parseFloat(b.getFieldValue('SECONDS')) || 0;
                emit(`M0 P${Math.round(secs * 1000)}`);
                break;
            }
            case 'wait_until_stopped':
            case 'wait_until_all_stopped':
                emit('; (G-code moves already wait for motion to finish)');
                break;

            // ── Motion control ───────────────────────────────────────────
            case 'set_acceleration': {
                const joint = b.getFieldValue('JOINT');
                const accel = parseInt(b.getFieldValue('ACCELERATION'), 10);
                emit(`M204 J${joint} A${isFinite(accel) ? accel : 5}`);
                break;
            }
            case 'stop_joint':
                emit(`; Stop Joint ${b.getFieldValue('JOINT')} — not needed, G-code moves run to completion`);
                break;
            case 'stop_all':
                emit('; Stop All Joints — not needed, G-code moves run to completion');
                break;
            case 'set_servo':
                skip(`Set Servo on Joint ${b.getFieldValue('JOINT')} has no G-code equivalent`);
                break;

            // ── End tool ─────────────────────────────────────────────────
            case 'gripper_open':  emit('M10'); break;
            case 'gripper_close': emit('M11'); break;
            case 'pump_on':       emit('M62'); break;
            case 'pump_off':      emit('M63'); break;
            case 'solenoid_on':   emit('M64'); break;
            case 'solenoid_off':  emit('M65'); break;
            case 'end_tool_servo': {
                const angle = parseInt(b.getFieldValue('ANGLE'), 10);
                emit(`M12 P${isFinite(angle) ? angle : 90}`);
                break;
            }

            // ── Vision ───────────────────────────────────────────────────
            case 'save_block_to_position': {
                const idx  = inputToGCode(b, 'INDEX');
                const slot = inputToGCode(b, 'SLOT');
                const z    = inputToGCode(b, 'Z');
                if (idx === null || slot === null) {
                    skip('Save block to position — index and slot can\'t be converted');
                    break;
                }
                emit(`M781 P${idx} L${slot}${z === null ? '' : ' Z' + z}`);
                break;
            }

            // ── Variables ────────────────────────────────────────────────
            case 'variables_set': {
                const name = blockVarName(b);
                const v = inputToGCode(b, 'VALUE');
                if (v === null) { skip(`set ${name} — value (${describeValue(b, 'VALUE')}) can't be converted`); break; }
                emit(`#${varFor(name)} = ${v} ; ${name}`);
                break;
            }
            case 'math_change': {
                const name = blockVarName(b);
                const v = inputToGCode(b, 'DELTA');
                if (v === null) { skip(`change ${name} — amount (${describeValue(b, 'DELTA')}) can't be converted`); break; }
                emit(`#${varFor(name)} = #${varFor(name)} + ${v} ; ${name}`);
                break;
            }

            // ── Loops ────────────────────────────────────────────────────
            case 'controls_repeat_ext':
            case 'controls_repeat': {
                const body = b.getInputTargetBlock('DO');
                let count = b.type === 'controls_repeat'
                    ? gcodeNum(parseInt(b.getFieldValue('TIMES'), 10) || 0, 0)
                    : inputToGCode(b, 'TIMES');
                if (count === null) {
                    warn(`repeat count (${describeValue(b, 'TIMES')}) can't be converted — using ${GCODE_FALLBACK_LOOP_COUNT} passes`);
                    count = String(GCODE_FALLBACK_LOOP_COUNT);
                }
                emitCountedLoop(count, body, `Repeat ${count} times`);
                break;
            }
            case 'controls_whileUntil': {
                const until = b.getFieldValue('MODE') === 'UNTIL';
                const body = b.getInputTargetBlock('DO');
                const condBlock = b.getInputTargetBlock('BOOL');
                let condition = conditionToGCode(condBlock);
                if (condition && until) {
                    condition = condition.always !== undefined
                        ? { always: !condition.always }
                        : { cond: condition.negated, negated: condition.cond };
                }
                if (!condition) {
                    warn(`${until ? 'repeat until' : 'repeat while'} condition (${condBlock ? `"${condBlock.type}" block` : 'empty'}) can't be converted — looping ${GCODE_FALLBACK_LOOP_COUNT} times instead`);
                    emitCountedLoop(String(GCODE_FALLBACK_LOOP_COUNT), body, `Loop ${GCODE_FALLBACK_LOOP_COUNT} times (was a conditional loop)`);
                    break;
                }
                if (condition.always === true) {
                    // "repeat while true" — an endless loop. Run a fixed number of
                    // passes so the program finishes, and say how to change that.
                    emit(`; Blockly "${until ? 'repeat until false' : 'repeat while true'}" is an endless loop.`);
                    emit(`; Converted to ${GCODE_FALLBACK_LOOP_COUNT} passes — change the number in the IF line below,`);
                    emit('; or delete that IF line to loop forever (use Stop to end the program).');
                    emitCountedLoop(String(GCODE_FALLBACK_LOOP_COUNT), body, `Loop ${GCODE_FALLBACK_LOOP_COUNT} times`);
                    break;
                }
                if (condition.always === false) {
                    emit(`; ${until ? 'repeat until true' : 'repeat while false'} — body never runs, skipped`);
                    break;
                }
                const top = newLabel(), end = newLabel();
                emit(`; ${until ? 'Repeat until' : 'Repeat while'} [${until ? condition.negated : condition.cond}]`);
                emit(`N${top}`);
                emitJumpUnless(condition, end);
                loopStack.push({ continueLabel: top, breakLabel: end });
                walkBody(body);
                loopStack.pop();
                emit(`GOTO ${top}`);
                emit(`N${end}`);
                break;
            }
            case 'controls_for': {
                const name = blockVarName(b);
                const v = varFor(name);
                const from = inputToGCode(b, 'FROM');
                const to   = inputToGCode(b, 'TO');
                const by   = inputToGCode(b, 'BY');
                const body = b.getInputTargetBlock('DO');
                if (from === null || to === null || by === null) {
                    warn(`count with ${name} — from/to/by can't all be converted — looping ${GCODE_FALLBACK_LOOP_COUNT} times instead`);
                    emitCountedLoop(String(GCODE_FALLBACK_LOOP_COUNT), body, `Loop ${GCODE_FALLBACK_LOOP_COUNT} times (was count with ${name})`);
                    break;
                }
                // Blockly counts downwards if "by" is negative; G-code can only
                // check one direction, so pick it from the sign when it's a constant.
                const descending = parseFloat(by) < 0;
                if (isNaN(parseFloat(by))) warn(`count with ${name} — step "${by}" isn't a plain number, assuming it counts upwards`);
                const top = newLabel(), cont = newLabel(), end = newLabel();
                emit(`; Count with ${name} from ${from} to ${to} by ${by}`);
                emit(`#${v} = ${from}`);
                emit(`N${top}`);
                emit(`IF [#${v} ${descending ? 'LT' : 'GT'} ${to}] GOTO ${end}`);
                loopStack.push({ continueLabel: cont, breakLabel: end });
                walkBody(body);
                loopStack.pop();
                emit(`N${cont}`);
                emit(`#${v} = #${v} + ${by}`);
                emit(`GOTO ${top}`);
                emit(`N${end}`);
                break;
            }
            case 'controls_forEach':
                skip('for each item in list — G-code has no lists; its contents were skipped');
                break;
            case 'controls_flow_statements': {
                const flow = b.getFieldValue('FLOW');
                const ctx = loopStack[loopStack.length - 1];
                if (!ctx) { skip(`${flow === 'BREAK' ? 'break' : 'continue'} outside a loop`); break; }
                emit(`GOTO ${flow === 'BREAK' ? ctx.breakLabel : ctx.continueLabel} ; ${flow === 'BREAK' ? 'break out of loop' : 'continue with next iteration'}`);
                break;
            }

            // ── Conditionals ─────────────────────────────────────────────
            case 'controls_if':
            case 'controls_ifelse': {
                const end = newLabel();
                let n = 0;
                while (b.getInput('IF' + n)) {
                    const condBlock = b.getInputTargetBlock('IF' + n);
                    const condition = conditionToGCode(condBlock);
                    const next = newLabel();
                    emit(`; ${n === 0 ? 'if' : 'else if'} ${condition && condition.cond ? `[${condition.cond}]` : ''}`);
                    if (!condition) {
                        skip(`${n === 0 ? 'if' : 'else if'} condition (${condBlock ? `"${condBlock.type}" block` : 'empty'}) can't be converted — this branch was skipped`);
                        emit(`GOTO ${next}`);
                    } else {
                        emitJumpUnless(condition, next);
                        walkBody(b.getInputTargetBlock('DO' + n));
                        emit(`GOTO ${end}`);
                    }
                    emit(`N${next}`);
                    n++;
                }
                if (b.getInput('ELSE')) {
                    emit('; else');
                    walkBody(b.getInputTargetBlock('ELSE'));
                }
                emit(`N${end}`);
                break;
            }

            case 'text_print': {
                const t = b.getInputTargetBlock('TEXT');
                emit(`; print: ${t && t.type === 'text' ? t.getFieldValue('TEXT') : '(expression)'}`);
                break;
            }

            default:
                skip(`"${b.type}" block has no G-code equivalent`);
                break;
        }
    }

    for (const top of topBlocks) {
        if (topBlocks.length > 1) emit(`; --- block stack ${topBlocks.indexOf(top) + 1} of ${topBlocks.length} ---`);
        walkChain(top);
    }

    let gcode = '; G-code converted from Blockly program\n';
    gcode += '; Generated automatically - review before running\n';
    gcode += '; Joint moves: G1 Jn=deg F=deg/s   Stored positions: G1 Pn   TCP moves: G1 X Y Z F=mm/min\n';
    gcode += '; Loops/ifs use N-labels, GOTO, IF [..] GOTO and #variables (see the G-code cheat sheet)\n';
    const userVars = Object.keys(varNumbers).filter(k => !k.startsWith('('));
    if (userVars.length) {
        gcode += `; Variables: ${userVars.map(k => `${k} = #${varNumbers[k]}`).join(', ')}\n`;
    }
    if (warnings.length) {
        gcode += `; ${warnings.length} WARNING(S) — search for "WARNING"\n`;
    }
    if (notConverted.length) {
        gcode += `; ${notConverted.length} block(s) could not be converted — search for "NOT CONVERTED"\n`;
    }
    gcode += '\n' + out.join('\n') + '\n\nM30\n';
    return gcode;
}

/**
 * Converts the current Blockly workspace to the app's RAPID subset by
 * walking the block tree (see convertBlocklyToGCode for the G-code twin).
 *
 * RAPID has real structured control flow, so loops and ifs convert
 * directly: repeat → FOR, while/until → WHILE, if/else → IF/ELSEIF/ELSE,
 * Blockly variables → VAR num, break/continue → GOTO labels. Vision values
 * become GetBlockCount()/GetBlockX(i)/GetBlockY(i) calls inside expressions.
 *
 * Anything that still can't be expressed is left as a "! NOT CONVERTED:"
 * comment so the gap is visible in the editor rather than silently dropped.
 *
 * @param {Blockly.Workspace} [workspace] - defaults to the live Blockly workspace
 * @returns {string} RAPID program
 */
function convertBlocklyToRapid(workspace) {
    const ws = workspace || blocklyWorkspace;
    if (!ws) return '! No blocks in workspace\n';

    const topBlocks = ws.getTopBlocks(true).filter(b =>
        !b.outputConnection && (b.previousConnection || b.nextConnection));
    if (topBlocks.length === 0) return '! No blocks in workspace\n';

    const out = [];
    const notConverted = [];
    const warnings = [];
    let indent = '';

    function emit(line) { out.push(indent + line); }
    function skip(reason) {
        notConverted.push(reason);
        emit(`! NOT CONVERTED: ${reason}`);
    }
    function warn(reason) {
        warnings.push(reason);
        emit(`! WARNING: ${reason}`);
    }
    function isEnabled(b) {
        return typeof b.isEnabled === 'function' ? b.isEnabled() : !b.disabled;
    }
    function num(v, decimals) { return gcodeNum(v, decimals === undefined ? 1 : decimals); }

    // ── Variables ────────────────────────────────────────────────────────
    // Blockly variable names can contain anything; RAPID identifiers can't.
    const rapidNames = {};      // Blockly name → RAPID identifier
    const usedNames = new Set();
    const RESERVED = new Set(['if', 'then', 'else', 'elseif', 'endif', 'while', 'do', 'endwhile', 'for',
        'from', 'to', 'step', 'endfor', 'var', 'num', 'bool', 'goto', 'exit', 'stop', 'true', 'false',
        'and', 'or', 'not', 'div', 'mod', 'tpwrite']);
    function rapidName(name) {
        if (rapidNames[name]) return rapidNames[name];
        let id = String(name).replace(/[^A-Za-z0-9_]/g, '_').replace(/^(\d)/, 'v$1') || 'v';
        if (RESERVED.has(id.toLowerCase())) id = id + '_';
        let candidate = id, n = 2;
        while (usedNames.has(candidate.toLowerCase())) candidate = `${id}${n++}`;
        usedNames.add(candidate.toLowerCase());
        rapidNames[name] = candidate;
        return candidate;
    }
    function tempName(base) {
        let n = 1;
        while (usedNames.has(`${base}${n}`.toLowerCase())) n++;
        usedNames.add(`${base}${n}`.toLowerCase());
        return `${base}${n}`;
    }
    function blockVarName(b, field) {
        const f = b.getField(field || 'VAR');
        return f ? f.getText() : 'var';
    }
    let labelCounter = 0;
    function newLabel(base) { return `${base}${++labelCounter}`; }

    // ── Loop context for break/continue ──────────────────────────────────
    const loopStack = []; // { breakLabel, continueLabel, breakUsed, continueUsed }

    // ── Values ───────────────────────────────────────────────────────────
    function valueToRapid(b) {
        if (!b) return null;
        switch (b.type) {
            case 'math_number': {
                const v = parseFloat(b.getFieldValue('NUM'));
                return isFinite(v) ? num(v, 3) : null;
            }
            case 'variables_get':
                return rapidName(blockVarName(b));
            case 'math_negate': {
                const v = inputToRapid(b, 'NUM');
                return v === null ? null : `(-${v})`;
            }
            case 'math_arithmetic': {
                const a = inputToRapid(b, 'A');
                const c = inputToRapid(b, 'B');
                if (a === null || c === null) return null;
                const op = { ADD: '+', MINUS: '-', MULTIPLY: '*', DIVIDE: '/' }[b.getFieldValue('OP')];
                if (!op) return null; // POWER has no RAPID operator in this subset
                return `(${a} ${op} ${c})`;
            }
            case 'block_count':
                return 'GetBlockCount()';
            case 'block_x_at':
            case 'block_y_at': {
                const idx = inputToRapid(b, 'INDEX');
                if (idx === null) return null;
                return `${b.type === 'block_x_at' ? 'GetBlockX' : 'GetBlockY'}(${idx})`;
            }
        }
        return null;
    }
    function inputToRapid(b, inputName) {
        return valueToRapid(b.getInputTargetBlock(inputName));
    }
    function describeValue(b, inputName) {
        const t = b.getInputTargetBlock(inputName);
        return t ? `"${t.type}" block` : 'empty input';
    }

    // ── Conditions ───────────────────────────────────────────────────────
    function conditionToRapid(b) {
        if (!b) return null;
        switch (b.type) {
            case 'logic_boolean':
                return b.getFieldValue('BOOL') === 'TRUE' ? 'TRUE' : 'FALSE';
            case 'logic_negate': {
                const inner = conditionToRapid(b.getInputTargetBlock('BOOL'));
                return inner === null ? null : `NOT (${inner})`;
            }
            case 'logic_operation': {
                const a = conditionToRapid(b.getInputTargetBlock('A'));
                const c = conditionToRapid(b.getInputTargetBlock('B'));
                if (a === null || c === null) return null;
                return `(${a} ${b.getFieldValue('OP') === 'AND' ? 'AND' : 'OR'} ${c})`;
            }
            case 'logic_compare': {
                const a = inputToRapid(b, 'A');
                const c = inputToRapid(b, 'B');
                if (a === null || c === null) return null;
                const op = { EQ: '=', NEQ: '<>', LT: '<', LTE: '<=', GT: '>', GTE: '>=' }[b.getFieldValue('OP')];
                if (!op) return null;
                return `${a} ${op} ${c}`;
            }
        }
        return null;
    }

    // ── Statement walking ────────────────────────────────────────────────
    function walkChain(block) {
        for (let b = block; b; b = b.getNextBlock()) {
            if (!isEnabled(b)) { emit(`! (disabled block skipped: ${b.type})`); continue; }
            convertBlock(b);
        }
    }
    function walkBody(block) {
        const saved = indent;
        indent += '  ';
        walkChain(block);
        indent = saved;
    }

    // Emits a loop body with break/continue support. `open` and `close` are
    // the loop's opening and closing lines.
    function emitLoop(open, body, close, title) {
        const ctx = { breakLabel: newLabel('loop_end'), continueLabel: newLabel('loop_next'), breakUsed: false, continueUsed: false };
        if (title) emit(`! ${title}`);
        emit(open);
        loopStack.push(ctx);
        walkBody(body);
        loopStack.pop();
        if (ctx.continueUsed) emit(`  ${ctx.continueLabel}:`);
        emit(close);
        if (ctx.breakUsed) emit(`${ctx.breakLabel}:`);
    }

    function speedSuffix(b, field, dflt) {
        const v = parseFloat(b.getFieldValue(field || 'SPEED')) || dflt;
        return `, v${num(v)}`;
    }

    function convertBlock(b) {
        switch (b.type) {
            // ── Joint-space moves ────────────────────────────────────────
            case 'move_joint': {
                emit(`MoveJoint ${b.getFieldValue('JOINT')}, ${num(parseFloat(b.getFieldValue('ANGLE')))}${speedSuffix(b, 'SPEED', 40)};`);
                break;
            }
            case 'move_all_joints': {
                const values = [];
                const bad = [];
                let missing = 0;
                for (let j = 1; j <= 6; j++) {
                    if (!b.getInputTargetBlock('JOINT' + j)) { values.push(null); missing++; continue; }
                    const v = inputToRapid(b, 'JOINT' + j);
                    if (v === null) bad.push(`joint ${j} (${describeValue(b, 'JOINT' + j)})`);
                    values.push(v);
                }
                if (bad.length) { skip(`Move All Joints — can't convert ${bad.join(', ')}`); break; }
                if (missing === 6) { emit('! Move All Joints with no joint values'); break; }
                if (missing === 0) {
                    emit(`MoveAbsJ [[${values.join(', ')}]]${speedSuffix(b, 'SPEED', 40)};`);
                } else {
                    // MoveAbsJ needs all six angles; move the given joints one at a time instead
                    emit(`! Move All Joints with ${6 - missing} joint(s) set — moved one joint at a time`);
                    values.forEach((v, i) => { if (v !== null) emit(`MoveJoint ${i + 1}, ${v}${speedSuffix(b, 'SPEED', 40)};`); });
                }
                break;
            }
            case 'move_to_position': {
                const slot = parseInt(b.getFieldValue('POSITION'), 10);
                const pos = (typeof getPosition === 'function') ? getPosition(slot) : null;
                emit(`MoveToPos ${slot}${speedSuffix(b, 'SPEED', 40)};${pos && pos.label ? ` ! ${pos.label}` : ''}`);
                break;
            }

            // ── Cartesian moves ──────────────────────────────────────────
            case 'move_xyz':
            case 'move_xyz_offset': {
                const names = b.type === 'move_xyz' ? ['X', 'Y', 'Z'] : ['DX', 'DY', 'DZ'];
                const vals = names.map(n => inputToRapid(b, n));
                if (vals.some(v => v === null)) {
                    const bad = names.filter((n, i) => vals[i] === null).map(n => `${n} (${describeValue(b, n)})`);
                    skip(`${b.type === 'move_xyz' ? 'Move TCP to XYZ' : 'Move TCP by offset'} — can't convert ${bad.join(', ')}`);
                    break;
                }
                emit(`${b.type === 'move_xyz' ? 'MoveLXYZ' : 'MoveLOffs'} [[${vals.join(', ')}]]${speedSuffix(b, 'SPEED', 40)};`);
                break;
            }
            case 'set_tool_orientation': {
                const ox = parseFloat(b.getFieldValue('ORI_X')) || 0;
                const oy = parseFloat(b.getFieldValue('ORI_Y')) || 0;
                const oz = parseFloat(b.getFieldValue('ORI_Z')) || 0;
                const rot = parseFloat(b.getFieldValue('ORI_ROTATION')) || 0;
                emit(`SetToolOri [[${num(ox, 3)}, ${num(oy, 3)}, ${num(oz, 3)}], ${num(rot)}]${speedSuffix(b, 'SPEED', 90)};`);
                break;
            }

            // ── Timing ───────────────────────────────────────────────────
            case 'wait_seconds':
                emit(`WaitTime ${num(parseFloat(b.getFieldValue('SECONDS')) || 0)};`);
                break;
            case 'wait_until_stopped':
            case 'wait_until_all_stopped':
                emit('! (RAPID moves already wait for motion to finish)');
                break;

            // ── Motion control ───────────────────────────────────────────
            case 'set_acceleration':
                emit(`SetAcc ${b.getFieldValue('JOINT')}, ${parseInt(b.getFieldValue('ACCELERATION'), 10) || 5};`);
                break;
            case 'stop_joint':
                emit(`! Stop Joint ${b.getFieldValue('JOINT')} — not needed, RAPID moves run to completion`);
                break;
            case 'stop_all':
                emit('! Stop All Joints — not needed, RAPID moves run to completion');
                break;
            case 'set_servo':
                skip(`Set Servo on Joint ${b.getFieldValue('JOINT')} has no RAPID equivalent`);
                break;

            // ── End tool ─────────────────────────────────────────────────
            case 'gripper_open':  emit('GripperOpen;'); break;
            case 'gripper_close': emit('GripperClose;'); break;
            case 'pump_on':       emit('PumpOn;'); break;
            case 'pump_off':      emit('PumpOff;'); break;
            case 'solenoid_on':   emit('SolenoidOn;'); break;
            case 'solenoid_off':  emit('SolenoidOff;'); break;
            case 'end_tool_servo':
                emit(`ServoTo ${parseInt(b.getFieldValue('ANGLE'), 10) || 0};`);
                break;

            // ── Vision ───────────────────────────────────────────────────
            case 'save_block_to_position': {
                const idx  = inputToRapid(b, 'INDEX');
                const slot = inputToRapid(b, 'SLOT');
                const z    = inputToRapid(b, 'Z');
                if (idx === null || slot === null) { skip('Save block to position — index and slot can\'t be converted'); break; }
                emit(`SaveBlockToPos ${idx}, ${slot}${z === null ? '' : ', ' + z};`);
                break;
            }

            // ── Variables ────────────────────────────────────────────────
            case 'variables_set': {
                const name = blockVarName(b);
                const v = inputToRapid(b, 'VALUE');
                if (v === null) { skip(`set ${name} — value (${describeValue(b, 'VALUE')}) can't be converted`); break; }
                emit(`${rapidName(name)} := ${v};`);
                break;
            }
            case 'math_change': {
                const name = blockVarName(b);
                const v = inputToRapid(b, 'DELTA');
                if (v === null) { skip(`change ${name} — amount (${describeValue(b, 'DELTA')}) can't be converted`); break; }
                emit(`${rapidName(name)} := ${rapidName(name)} + ${v};`);
                break;
            }

            // ── Loops ────────────────────────────────────────────────────
            case 'controls_repeat_ext':
            case 'controls_repeat': {
                let count = b.type === 'controls_repeat'
                    ? String(parseInt(b.getFieldValue('TIMES'), 10) || 0)
                    : inputToRapid(b, 'TIMES');
                if (count === null) {
                    warn(`repeat count (${describeValue(b, 'TIMES')}) can't be converted — using 10`);
                    count = '10';
                }
                const counter = tempName('rep');
                emitLoop(`FOR ${counter} FROM 1 TO ${count} DO`, b.getInputTargetBlock('DO'), 'ENDFOR', `Repeat ${count} times`);
                break;
            }
            case 'controls_whileUntil': {
                const until = b.getFieldValue('MODE') === 'UNTIL';
                const condBlock = b.getInputTargetBlock('BOOL');
                let cond = conditionToRapid(condBlock);
                if (cond === null) {
                    warn(`${until ? 'repeat until' : 'repeat while'} condition (${condBlock ? `"${condBlock.type}" block` : 'empty'}) can't be converted — looping 10 times instead`);
                    emitLoop(`FOR ${tempName('rep')} FROM 1 TO 10 DO`, b.getInputTargetBlock('DO'), 'ENDFOR');
                    break;
                }
                if (until) cond = cond === 'TRUE' ? 'FALSE' : cond === 'FALSE' ? 'TRUE' : `NOT (${cond})`;
                if (cond === 'TRUE') {
                    emit('! Endless loop (Blockly "repeat while true") — press Stop to end it,');
                    emit('! or change TRUE to a condition such as count < 10.');
                }
                emitLoop(`WHILE ${cond} DO`, b.getInputTargetBlock('DO'), 'ENDWHILE');
                break;
            }
            case 'controls_for': {
                const name = rapidName(blockVarName(b));
                const from = inputToRapid(b, 'FROM');
                const to   = inputToRapid(b, 'TO');
                const by   = inputToRapid(b, 'BY');
                if (from === null || to === null || by === null) {
                    warn(`count with ${name} — from/to/by can't all be converted — looping 10 times instead`);
                    emitLoop(`FOR ${name} FROM 1 TO 10 DO`, b.getInputTargetBlock('DO'), 'ENDFOR');
                    break;
                }
                const step = (by === '1') ? '' : ` STEP ${by}`;
                emitLoop(`FOR ${name} FROM ${from} TO ${to}${step} DO`, b.getInputTargetBlock('DO'), 'ENDFOR');
                break;
            }
            case 'controls_forEach':
                skip('for each item in list — RAPID subset has no lists; its contents were skipped');
                break;
            case 'controls_flow_statements': {
                const flow = b.getFieldValue('FLOW');
                const ctx = loopStack[loopStack.length - 1];
                if (!ctx) { skip(`${flow === 'BREAK' ? 'break' : 'continue'} outside a loop`); break; }
                if (flow === 'BREAK') { ctx.breakUsed = true; emit(`GOTO ${ctx.breakLabel}; ! break out of loop`); }
                else { ctx.continueUsed = true; emit(`GOTO ${ctx.continueLabel}; ! continue with next iteration`); }
                break;
            }

            // ── Conditionals ─────────────────────────────────────────────
            case 'controls_if':
            case 'controls_ifelse': {
                let n = 0;
                let opened = false;
                while (b.getInput('IF' + n)) {
                    const condBlock = b.getInputTargetBlock('IF' + n);
                    const cond = conditionToRapid(condBlock);
                    if (cond === null) {
                        skip(`${n === 0 ? 'if' : 'else if'} condition (${condBlock ? `"${condBlock.type}" block` : 'empty'}) can't be converted — this branch was skipped`);
                    } else {
                        emit(`${opened ? 'ELSEIF' : 'IF'} ${cond} THEN`);
                        opened = true;
                        walkBody(b.getInputTargetBlock('DO' + n));
                    }
                    n++;
                }
                if (b.getInput('ELSE')) {
                    if (opened) { emit('ELSE'); walkBody(b.getInputTargetBlock('ELSE')); }
                    else { emit('! else (no convertible condition before it — runs unconditionally)'); walkChain(b.getInputTargetBlock('ELSE')); }
                }
                if (opened) emit('ENDIF');
                break;
            }

            case 'text_print': {
                const t = b.getInputTargetBlock('TEXT');
                if (t && t.type === 'text') emit(`TPWrite "${t.getFieldValue('TEXT').replace(/"/g, '\'')}";`);
                else {
                    const v = valueToRapid(t);
                    if (v === null) skip('print — value can\'t be converted');
                    else emit(`TPWrite "" \\Num:=${v};`);
                }
                break;
            }

            default:
                skip(`"${b.type}" block has no RAPID equivalent`);
                break;
        }
    }

    for (const top of topBlocks) {
        if (topBlocks.length > 1) emit(`! --- block stack ${topBlocks.indexOf(top) + 1} of ${topBlocks.length} ---`);
        walkChain(top);
    }

    let rapid = '! RAPID program converted from Blockly\n';
    rapid += '! Generated automatically - review before running\n';
    if (warnings.length) rapid += `! ${warnings.length} WARNING(S) — search for "WARNING"\n`;
    if (notConverted.length) rapid += `! ${notConverted.length} block(s) could not be converted — search for "NOT CONVERTED"\n`;

    // Declare every Blockly variable up front (RAPID needs VAR before use).
    // Loop counters are declared by their FOR statements.
    const userVars = Object.keys(rapidNames).filter(k => !k.startsWith('('));
    if (userVars.length) {
        rapid += '\n! Variables\n';
        for (const k of userVars) {
            rapid += `VAR num ${rapidNames[k]} := 0;${rapidNames[k] !== k ? ` ! Blockly variable "${k}"` : ''}\n`;
        }
    }
    rapid += '\n' + out.join('\n') + '\n\n! Program end\n';
    return rapid;
}

/**
 * Converts current Blockly program to G-code and opens G-code tab
 */
function convertBlocklyToGCodeAndOpen() {
    if (!blocklyWorkspace) {
        showAppMessage('Blockly workspace not initialized');
        return;
    }

    if (blocklyWorkspace.getTopBlocks(false).length === 0) {
        showAppMessage('No blocks in workspace. Add some blocks to create a program.');
        return;
    }

    // Convert to G-code (walks the block tree directly)
    const gcode = convertBlocklyToGCode(blocklyWorkspace);
    const notConverted = (gcode.match(/; NOT CONVERTED:/g) || []).length;

    // Switch to G-code tab
    switchToTab('gcode');

    // Load G-code into editor after a short delay to ensure tab is visible
    setTimeout(() => {
        const gcodeTextarea = document.getElementById('gcodeContent');
        if (gcodeTextarea) {
            gcodeTextarea.value = gcode;
            // Update line count
            const lines = gcode.split('\n').filter(l => l.trim() !== '');
            document.getElementById('gcodeLineCount').textContent = lines.length;
            // Apply changes to processor
            applyGCodeChanges();
            showAppMessage(notConverted
                ? `Converted to G-code — ${notConverted} block(s) could not be converted, see "NOT CONVERTED" comments`
                : 'Blockly program converted to G-code and loaded');
        }
    }, 100);
}

/**
 * Converts current Blockly program to RAPID and opens RAPID tab
 */
function convertBlocklyToRapidAndOpen() {
    if (!blocklyWorkspace) {
        showAppMessage('Blockly workspace not initialized');
        return;
    }

    if (blocklyWorkspace.getTopBlocks(false).length === 0) {
        showAppMessage('No blocks in workspace. Add some blocks to create a program.');
        return;
    }

    // Convert to RAPID (walks the block tree directly)
    const rapid = convertBlocklyToRapid(blocklyWorkspace);
    const notConverted = (rapid.match(/! NOT CONVERTED:/g) || []).length;

    // Switch to RAPID tab
    switchToTab('rapid');

    // Load RAPID into editor after a short delay to ensure tab is visible
    setTimeout(() => {
        const rapidTextarea = document.getElementById('rapidContent');
        if (rapidTextarea) {
            rapidTextarea.value = rapid;
            showAppMessage(notConverted
                ? `Converted to RAPID — ${notConverted} block(s) could not be converted, see "NOT CONVERTED" comments`
                : 'Blockly program converted to RAPID and loaded');
        }
    }, 100);
}

/**
 * Switches to a specific tab programmatically
 * @param {string} tabName - Name of the tab (e.g., 'gcode', 'rapid', 'blockly')
 */
function switchToTab(tabName) {
    const tabButtons = document.querySelectorAll('.tab-button');
    const tabContents = document.querySelectorAll('.tab-content');
    
    // Remove active class from all buttons and contents
    tabButtons.forEach(btn => btn.classList.remove('active'));
    tabContents.forEach(content => content.classList.remove('active'));
    
    // Find and activate the target tab button
    let targetButton = null;
    tabButtons.forEach(btn => {
        if (btn.getAttribute('data-tab') === tabName) {
            targetButton = btn;
            btn.classList.add('active');
        }
    });
    
    // Activate the target tab content
    const targetContent = document.getElementById(tabName + '-tab');
    if (targetContent) {
        targetContent.classList.add('active');
    } else {
        console.error(`Tab content not found: ${tabName}-tab`);
    }
    
    // Handle special tab initialization if needed
    if (tabName === 'visualization') {
        setTimeout(() => {
            if (!robotArm3D) {
                initialize3DVisualization();
            }
        }, 100);
    }
}
