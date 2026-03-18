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
    
    // Calculate scaled speeds: speed = baseSpeed * (distance / maxDistance)
    // This ensures all joints finish at the same time
    const scaledSpeeds = distances.map(distance => {
        if (distance === 0) {
            return 0; // Joint already at target, no movement needed
        }
        return baseSpeedDegreesPerSecond * (distance / maxDistance);
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
                startScale: 1.0,
                maxScale: 3,
                minScale: 0.3,
                scaleSpeed: 1.1
            },
            trashcan: true,
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
                        type: 'move_xyz'
                    },
                    {
                        kind: 'block',
                        type: 'move_xyz_offset'
                    },
                    {
                        kind: 'block',
                        type: 'set_tool_orientation'
                    }
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
                .appendField(new Blockly.FieldNumber(45, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Move a joint to a specific angle at a given speed in degrees per second');
        }
    };

    // Set tool orientation vector (direction of tool Z-axis)
    Blockly.Blocks['set_tool_orientation'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Set tool orientation to vector')
                .appendField('X')
                .appendField(new Blockly.FieldNumber(0), 'ORI_X')
                .appendField('Y')
                .appendField(new Blockly.FieldNumber(0), 'ORI_Y')
                .appendField('Z')
                .appendField(new Blockly.FieldNumber(-1), 'ORI_Z');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Set the tool orientation vector (direction of the tool Z-axis)');
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
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(45, 0, 300, 1), 'SPEED')
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
                .appendField(new Blockly.FieldNumber(45, 0, 300, 1), 'SPEED')
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
            this.appendDummyInput()
                .appendField('Move TCP to X')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'X')
                .appendField('mm  Y')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'Y')
                .appendField('mm  Z')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'Z')
                .appendField('mm');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(45, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(200);
            this.setTooltip('Move the end effector to an absolute XYZ position using kinematics');
        }
    };

    // Move TCP by an XYZ offset relative to the current position
    Blockly.Blocks['move_xyz_offset'] = {
        init: function() {
            this.appendDummyInput()
                .appendField('Move TCP by dX')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'DX')
                .appendField('mm  dY')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'DY')
                .appendField('mm  dZ')
                .appendField(new Blockly.FieldNumber(0, -500, 500, 1), 'DZ')
                .appendField('mm');
            this.appendDummyInput()
                .appendField('at speed')
                .appendField(new Blockly.FieldNumber(45, 0, 300, 1), 'SPEED')
                .appendField('degrees/s');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(200);
            this.setTooltip('Move the end effector by an XYZ offset from the current position using kinematics');
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
                .appendField(new Blockly.FieldNumber(50, 0, 254, 1), 'ACCELERATION')
                .appendField('(0-254)');
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(230);
            this.setTooltip('Set the acceleration for a specific joint. Range: 0-254 (unit: 100 step/s²). Higher values = faster acceleration.');
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
    
    moveAll1.setFieldValue('45', 'SPEED');
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
    moveJoint1.setFieldValue('45', 'SPEED');
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
    
    moveAll2.setFieldValue('45', 'SPEED');
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
function appendBlocklyOutput(text) {
    const output = document.getElementById('blocklyOutput');
    const timestamp = new Date().toLocaleTimeString();
    // Format similar to G-code log: [HH:MM:SS] Message
    output.textContent += `[${timestamp}] ${text}\n`;
    output.scrollTop = output.scrollHeight;
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

    // Ask the user for a single acceleration value
    const input = prompt('Enter acceleration value for all joints (0-254):', '50');

    // If the user clicked Cancel, do nothing
    if (input === null) {
        return;
    }

    const parsed = parseInt(input, 10);
    if (isNaN(parsed) || parsed < 0 || parsed > 254) {
        showAppMessage('Please enter a whole number between 0 and 254.');
        return;
    }

    const accelerationValue = parsed;

    // Get number of joints (falls back to 5 if function not available)
    let numJoints = 5;
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
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 45;
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
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 45;
        // Sanitize block ID to make it a valid JavaScript identifier
        const sanitizedId = blockId.replace(/[^a-zA-Z0-9_]/g, '_');
        // Use unique variable names based on block ID to avoid conflicts
        const targetAnglesVar = 'targetAngles_all_' + sanitizedId;
        
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        {
            const ${targetAnglesVar} = [${joint1}, ${joint2}, ${joint3}, ${joint4}, ${joint5}];
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
        const acceleration = block.getFieldValue('ACCELERATION') || 50;
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
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 45;
        
        // Get the position data
        if (typeof getPosition === 'function') {
            const position = getPosition(parseInt(positionNumber));
            if (position && position.angles) {
                const positionLabel = position.label || `Position ${positionNumber}`;
                // Move all joints to the stored angles at the specified speed with linear interpolation
                const targetAnglesArray = position.angles.join(', ');
                let code = `
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
                
                // Target angles from stored position
                const targetAngles_pos_${sanitizedId} = [${targetAnglesArray}];
                
                // Calculate scaled speeds using linear interpolation
                const scaledSpeeds_pos_${sanitizedId} = calculateScaledSpeeds(currentAngles_pos_${sanitizedId}, targetAngles_pos_${sanitizedId}, ${speedDegreesPerSecond});
                
                // Convert speeds to steps/s and move joints
                appendBlocklyOutput('Moving to ${positionLabel} at speed ${speedDegreesPerSecond} degrees/s (scaled speeds for synchronized arrival)');
                `;
                for (let i = 0; i < position.angles.length; i++) {
                    code += `
                    if (scaledSpeeds_pos_${sanitizedId}[${i}] > 0) {
                        const speedStepsPerSecond_pos_${sanitizedId}_${i} = degreesPerSecondToStepsPerSecond(scaledSpeeds_pos_${sanitizedId}[${i}]);
                        await robotArmClient.moveJoint(${i + 1}, targetAngles_pos_${sanitizedId}[${i}], speedStepsPerSecond_pos_${sanitizedId}_${i});
                    }
                    `;
                }
                return code;
            }
        }
        
        // Fallback if position not found
        return `highlightBlocklyBlock('${blockId}');\nawait checkBlocklyPauseStop();\n// Position ${positionNumber} not found\n`;
    };

    // Move TCP to absolute XYZ using kinematics
    Blockly.JavaScript['move_xyz'] = function(block) {
        const blockId = block.id;
        const x = block.getFieldValue('X') || 0;
        const y = block.getFieldValue('Y') || 0;
        const z = block.getFieldValue('Z') || 0;
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 45;
        const speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);

        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();

        if (!robotKinematics.isConfigured()) {
            appendBlocklyOutput('Kinematics not configured. Cannot run "Move TCP to XYZ" block.');
        } else {
            const targetPose = { x: ${x}, y: ${y}, z: ${z} };

            // Read current XYZ from the UI as start pose
            let currentX = parseFloat(document.getElementById('currentX').textContent);
            let currentY = parseFloat(document.getElementById('currentY').textContent);
            let currentZ = parseFloat(document.getElementById('currentZ').textContent);
            if (!isFinite(currentX)) currentX = targetPose.x;
            if (!isFinite(currentY)) currentY = targetPose.y;
            if (!isFinite(currentZ)) currentZ = targetPose.z;

            const startPose = { x: currentX, y: currentY, z: currentZ };
            const waypoints = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
            if (!waypoints) {
                appendBlocklyOutput('Move TCP to XYZ cancelled: target lies inside a dead zone.');
            } else {
                appendBlocklyOutput('Moving TCP to X=' + targetPose.x + ' Y=' + targetPose.y + ' Z=' + targetPose.z + ' at ' + ${speedDegreesPerSecond} + ' deg/s (dead-zone aware path, orientation-aware)');

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

                    // First solve for position only
                    const baseAngles = robotKinematics.inverseKinematics({
                        x: wp.x,
                        y: wp.y,
                        z: wp.z
                    }, initialAngles);
                    if (!baseAngles) {
                        appendBlocklyOutput('Move TCP to XYZ failed: waypoint unreachable.');
                        break;
                    }

                    // Iterative refinement (coarse to fine) and get accuracy for reporting
                    const refined = robotKinematics.refineOrientationWithAccuracy(
                        { x: wp.x, y: wp.y, z: wp.z },
                        baseAngles,
                        currentToolOrientation
                    );
                    const refinedAngles = refined.angles;

                    const numJoints = refinedAngles.length;
                    for (let i = 0; i < numJoints; i++) {
                        await robotArmClient.moveJoint(i + 1, refinedAngles[i], ${speedStepsPerSecond});
                    }

                    const posErr = refined.positionErrorMm.toFixed(2);
                    const oriErr = refined.orientationErrorDeg.toFixed(1);
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
        const dx = block.getFieldValue('DX') || 0;
        const dy = block.getFieldValue('DY') || 0;
        const dz = block.getFieldValue('DZ') || 0;
        const speedDegreesPerSecond = block.getFieldValue('SPEED') || 45;
        const speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);

        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();

        if (!robotKinematics.isConfigured()) {
            appendBlocklyOutput('Kinematics not configured. Cannot run "Move TCP by offset" block.');
        } else {
            // Read current XYZ from the UI labels
            let currentX = parseFloat(document.getElementById('currentX').textContent);
            let currentY = parseFloat(document.getElementById('currentY').textContent);
            let currentZ = parseFloat(document.getElementById('currentZ').textContent);
            if (!isFinite(currentX)) currentX = 0;
            if (!isFinite(currentY)) currentY = 0;
            if (!isFinite(currentZ)) currentZ = 0;

            const startPose = { x: currentX, y: currentY, z: currentZ };
            const targetPose = {
                x: currentX + (${dx}),
                y: currentY + (${dy}),
                z: currentZ + (${dz})
            };

            const waypoints = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
            if (!waypoints) {
                appendBlocklyOutput('Move TCP by offset cancelled: target lies inside a dead zone.');
            } else {
                appendBlocklyOutput(
                    'Moving TCP by dX=' + ${dx} + ' dY=' + ${dy} + ' dZ=' + ${dz} +
                    ' to X=' + targetPose.x + ' Y=' + targetPose.y + ' Z=' + targetPose.z +
                    ' (dead-zone aware path, orientation-aware)'
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

                    // First solve for position only
                    const baseAngles = robotKinematics.inverseKinematics({
                        x: wp.x,
                        y: wp.y,
                        z: wp.z
                    }, initialAngles);
                    if (!baseAngles) {
                        appendBlocklyOutput('Move TCP by offset failed: waypoint unreachable.');
                        break;
                    }

                    // Iterative refinement (coarse to fine) and get accuracy for reporting
                    const refined = robotKinematics.refineOrientationWithAccuracy(
                        { x: wp.x, y: wp.y, z: wp.z },
                        baseAngles,
                        currentToolOrientation
                    );
                    const refinedAngles = refined.angles;

                    const numJoints = refinedAngles.length;
                    for (let i = 0; i < numJoints; i++) {
                        await robotArmClient.moveJoint(i + 1, refinedAngles[i], ${speedStepsPerSecond});
                    }

                    const posErr = refined.positionErrorMm.toFixed(2);
                    const oriErr = refined.orientationErrorDeg.toFixed(1);
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
        return `
        highlightBlocklyBlock('${blockId}');
        await checkBlocklyPauseStop();
        appendBlocklyOutput('Setting tool orientation to vector (${ox}, ${oy}, ${oz})');
        setToolOrientationVector(${ox}, ${oy}, ${oz});
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

/**
 * Converts Blockly JavaScript code to G-code format
 * This is a simple converter that handles common robot arm commands
 * @param {string} blocklyCode - JavaScript code generated from Blockly
 * @returns {string} G-code program
 */
function convertBlocklyToGCode(blocklyCode) {
    if (!blocklyCode || blocklyCode.trim() === '') {
        return '; No blocks in workspace\n';
    }

    let gcode = '; G-code converted from Blockly program\n';
    gcode += '; Generated automatically - review before running\n\n';

    // Split code into lines and process each
    const lines = blocklyCode.split('\n');
    let inComment = false;

    for (let i = 0; i < lines.length; i++) {
        let line = lines[i].trim();
        if (line === '') continue;

        // Skip Blockly-specific function calls
        if (line.includes('highlightBlocklyBlock') || 
            line.includes('checkBlocklyPauseStop') ||
            line.includes('appendBlocklyOutput') ||
            line.includes('getNumJoints') ||
            line.includes('robotArmClient.getStatus') ||
            line.includes('moveJointsToAnglesWithDeadZones') ||
            line.includes('planSafePathAroundDeadZones') ||
            line.includes('robotKinematics.inverseKinematics')) {
            // These are Blockly runtime functions, skip them
            continue;
        }

        // Convert move_joint: robotArmClient.moveJoint(joint, angle, speed)
        const moveJointMatch = line.match(/robotArmClient\.moveJoint\((\d+),\s*([-\d.]+),\s*([-\d.]+)\)/);
        if (moveJointMatch) {
            const joint = moveJointMatch[1];
            const angle = parseFloat(moveJointMatch[2]);
            const speedStepsPerSecond = parseFloat(moveJointMatch[3]);
            // Convert steps/s back to degrees/s (approximate)
            const speedDegreesPerSecond = Math.round((speedStepsPerSecond / 11.37) * 10) / 10;
            gcode += `G1 J${joint}=${angle.toFixed(1)} F${speedDegreesPerSecond.toFixed(1)}\n`;
            continue;
        }

        // Convert stop_joint: robotArmClient.stopJoint(joint)
        const stopJointMatch = line.match(/robotArmClient\.stopJoint\((\d+)\)/);
        if (stopJointMatch) {
            const joint = stopJointMatch[1];
            gcode += `; Stop Joint ${joint}\n`;
            gcode += `M0 P100\n`; // Pause for 100ms
            continue;
        }

        // Convert stop_all: robotArmClient.stopAllJoints()
        if (line.includes('robotArmClient.stopAllJoints()')) {
            gcode += '; Stop all joints\n';
            gcode += 'M0 P100\n';
            continue;
        }

        // Convert set_acceleration: robotArmClient.setAcceleration(joint, accel)
        const setAccelMatch = line.match(/robotArmClient\.setAcceleration\((\d+),\s*(\d+)\)/);
        if (setAccelMatch) {
            const joint = setAccelMatch[1];
            const accel = setAccelMatch[2];
            gcode += `M204 J${joint} A${accel}\n`;
            continue;
        }

        // Convert set_servo: robotArmClient.setServoAngle(joint, angle)
        const setServoMatch = line.match(/robotArmClient\.setServoAngle\((\d+),\s*([-\d.]+)\)/);
        if (setServoMatch) {
            const joint = setServoMatch[1];
            const angle = setServoMatch[2];
            gcode += `; Set servo on Joint ${joint} to ${angle}°\n`;
            gcode += `M0 P100\n`; // Pause to allow servo to move
            continue;
        }

        // Convert wait: setTimeout or Promise delay
        const waitMatch = line.match(/setTimeout\(resolve,\s*(\d+)\)/);
        if (waitMatch) {
            const ms = parseInt(waitMatch[1]);
            const seconds = (ms / 1000).toFixed(1);
            gcode += `M0 P${ms}\n`; // Pause in milliseconds
            continue;
        }

        // Convert wait loops (simplified)
        if (line.includes('while') && line.includes('elapsed') && line.includes('waitTime')) {
            // This is a wait block, try to extract duration
            const waitTimeMatch = blocklyCode.match(/const\s+waitTime_\w+\s*=\s*(\d+)/);
            if (waitTimeMatch) {
                const ms = parseInt(waitTimeMatch[1]);
                gcode += `M0 P${ms}\n`;
            }
        }
    }

    // Add program end
    gcode += '\nM30\n';

    return gcode;
}

/**
 * Converts Blockly JavaScript code to RAPID format
 * This is a simple converter that handles common robot arm commands
 * @param {string} blocklyCode - JavaScript code generated from Blockly
 * @returns {string} RAPID program
 */
function convertBlocklyToRapid(blocklyCode) {
    if (!blocklyCode || blocklyCode.trim() === '') {
        return '! No blocks in workspace\n';
    }

    let rapid = '! RAPID program converted from Blockly\n';
    rapid += '! Generated automatically - review before running\n\n';

    // Track current joint angles for relative moves
    let currentAngles = [0, 0, 0, 0, 0];
    let lineNumber = 0;

    // Split code into lines and process each
    const lines = blocklyCode.split('\n');

    for (let i = 0; i < lines.length; i++) {
        let line = lines[i].trim();
        if (line === '') continue;

        // Skip Blockly-specific function calls
        if (line.includes('highlightBlocklyBlock') || 
            line.includes('checkBlocklyPauseStop') ||
            line.includes('appendBlocklyOutput') ||
            line.includes('getNumJoints') ||
            line.includes('robotArmClient.getStatus') ||
            line.includes('moveJointsToAnglesWithDeadZones') ||
            line.includes('planSafePathAroundDeadZones') ||
            line.includes('robotKinematics.inverseKinematics')) {
            continue;
        }

        // Convert move_joint: robotArmClient.moveJoint(joint, angle, speed)
        const moveJointMatch = line.match(/robotArmClient\.moveJoint\((\d+),\s*([-\d.]+),\s*([-\d.]+)\)/);
        if (moveJointMatch) {
            const jointIndex = parseInt(moveJointMatch[1]) - 1; // Convert to 0-based
            const angle = parseFloat(moveJointMatch[2]);
            currentAngles[jointIndex] = angle;
            rapid += `MoveJ [[${currentAngles[0]},${currentAngles[1]},${currentAngles[2]},${currentAngles[3]},${currentAngles[4]}]];\n`;
            continue;
        }

        // Convert move_all_joints: targetAngles array
        const moveAllMatch = line.match(/const\s+targetAngles\s*=\s*\[([^\]]+)\]/);
        if (moveAllMatch) {
            const anglesStr = moveAllMatch[1];
            const angles = anglesStr.split(',').map(a => parseFloat(a.trim()));
            currentAngles = angles.slice(0, 5); // Take first 5
            while (currentAngles.length < 5) currentAngles.push(0);
            rapid += `MoveJ [[${currentAngles[0]},${currentAngles[1]},${currentAngles[2]},${currentAngles[3]},${currentAngles[4]}]];\n`;
            continue;
        }

        // Convert stop_joint: robotArmClient.stopJoint(joint)
        const stopJointMatch = line.match(/robotArmClient\.stopJoint\((\d+)\)/);
        if (stopJointMatch) {
            rapid += `! Stop Joint ${stopJointMatch[1]}\n`;
            rapid += `WaitTime 0.1;\n`;
            continue;
        }

        // Convert stop_all: robotArmClient.stopAllJoints()
        if (line.includes('robotArmClient.stopAllJoints()')) {
            rapid += '! Stop all joints\n';
            rapid += 'WaitTime 0.1;\n';
            continue;
        }

        // Convert set_acceleration: robotArmClient.setAcceleration(joint, accel)
        const setAccelMatch = line.match(/robotArmClient\.setAcceleration\((\d+),\s*(\d+)\)/);
        if (setAccelMatch) {
            rapid += `! Set acceleration for Joint ${setAccelMatch[1]} to ${setAccelMatch[2]}\n`;
            rapid += `WaitTime 0.1;\n`;
            continue;
        }

        // Convert set_servo: robotArmClient.setServoAngle(joint, angle)
        const setServoMatch = line.match(/robotArmClient\.setServoAngle\((\d+),\s*([-\d.]+)\)/);
        if (setServoMatch) {
            rapid += `! Set servo on Joint ${setServoMatch[1]} to ${setServoMatch[2]}°\n`;
            rapid += `WaitTime 0.1;\n`;
            continue;
        }

        // Convert wait: setTimeout or Promise delay
        const waitMatch = line.match(/setTimeout\(resolve,\s*(\d+)\)/);
        if (waitMatch) {
            const ms = parseInt(waitMatch[1]);
            const seconds = (ms / 1000).toFixed(1);
            rapid += `WaitTime ${seconds};\n`;
            continue;
        }

        // Convert wait loops (simplified)
        if (line.includes('while') && line.includes('elapsed') && line.includes('waitTime')) {
            const waitTimeMatch = blocklyCode.match(/const\s+waitTime_\w+\s*=\s*(\d+)/);
            if (waitTimeMatch) {
                const ms = parseInt(waitTimeMatch[1]);
                const seconds = (ms / 1000).toFixed(1);
                rapid += `WaitTime ${seconds};\n`;
            }
        }

        // Convert move to position (stored position)
        const moveToPosMatch = blocklyCode.match(/targetAngles_pos_\w+\s*=\s*\[([^\]]+)\]/);
        if (moveToPosMatch && !rapid.includes('targetAngles_pos_')) {
            const anglesStr = moveToPosMatch[1];
            const angles = anglesStr.split(',').map(a => parseFloat(a.trim()));
            currentAngles = angles.slice(0, 5);
            while (currentAngles.length < 5) currentAngles.push(0);
            rapid += `MoveJ [[${currentAngles[0]},${currentAngles[1]},${currentAngles[2]},${currentAngles[3]},${currentAngles[4]}]];\n`;
        }
    }

    // Add program end comment
    rapid += '\n! Program end\n';

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

    // Generate JavaScript code from blocks
    const blocklyCode = generateBlocklyCode();
    
    if (!blocklyCode || blocklyCode.trim() === '') {
        showAppMessage('No blocks in workspace. Add some blocks to create a program.');
        return;
    }

    // Convert to G-code
    const gcode = convertBlocklyToGCode(blocklyCode);

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
            showAppMessage('Blockly program converted to G-code and loaded');
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

    // Generate JavaScript code from blocks
    const blocklyCode = generateBlocklyCode();
    
    if (!blocklyCode || blocklyCode.trim() === '') {
        showAppMessage('No blocks in workspace. Add some blocks to create a program.');
        return;
    }

    // Convert to RAPID
    const rapid = convertBlocklyToRapid(blocklyCode);

    // Switch to RAPID tab
    switchToTab('rapid');

    // Load RAPID into editor after a short delay to ensure tab is visible
    setTimeout(() => {
        const rapidTextarea = document.getElementById('rapidContent');
        if (rapidTextarea) {
            rapidTextarea.value = rapid;
            showAppMessage('Blockly program converted to RAPID and loaded');
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
