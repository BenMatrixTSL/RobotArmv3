/**
 * Sets the global tool orientation vector.
 * The vector describes the desired direction of the tool's local Z-axis
 * in world coordinates. The vector is normalised internally.
 *
 * @param {number} x - X component
 * @param {number} y - Y component
 * @param {number} z - Z component
 */
function setToolOrientationVector(x, y, z) {
    const vx = typeof x === 'number' ? x : 0;
    const vy = typeof y === 'number' ? y : 0;
    const vz = typeof z === 'number' ? z : -1;
    const length = Math.sqrt(vx * vx + vy * vy + vz * vz);

    if (length < 1e-6) {
        console.warn('setToolOrientationVector: vector too small, keeping previous orientation');
        return;
    }

    currentToolOrientation = {
        x: vx / length,
        y: vy / length,
        z: vz / length
    };

    console.log('Tool orientation set to:', currentToolOrientation);
}

/**
 * Main Application Logic
 * 
 * This file ties everything together and handles the UI interactions.
 * It's simple and beginner-friendly with clear step-by-step code.
 */

// Global variables
let statusUpdateInterval = null;
let robotArm3D = null; // 3D visualization instance
let useSimulatedAngles = false; // Whether to use simulated angles instead of real robot angles
let simulatedAngles = [0, 0, 0, 0]; // Simulated joint angles

// If the preload bridge did not run, we provide a simple fallback here
// so that window.electronAPI is still available when nodeIntegration is enabled.
try {
    if (typeof window !== 'undefined' && typeof window.electronAPI === 'undefined' && typeof require === 'function') {
        const { ipcRenderer } = require('electron');
        const fs = require('fs');
        const path = require('path');

        window.electronAPI = {
            platform: process.platform,
            version: process.versions.electron,
            readTextFile: function(relativePath) {
                const bases = [
                    __dirname,
                    process.cwd(),
                    path.join(process.cwd(), 'electron-app')
                ];
                for (let i = 0; i < bases.length; i++) {
                    const fullPath = path.join(bases[i], relativePath);
                    try {
                        if (fs.existsSync(fullPath)) {
                            const content = fs.readFileSync(fullPath, 'utf8');
                            if (i > 0) {
                                console.log('[renderer fallback] readTextFile: loaded from', fullPath);
                            }
                            return content;
                        }
                    } catch (e) {
                        // try next base
                    }
                }
                console.error('[renderer fallback] readTextFile: not found for', relativePath);
                return null;
            },
            openVisualisationWindow: function() {
                try {
                    ipcRenderer.invoke('open-visualisation-window');
                } catch (e) {
                    console.error('[renderer fallback] openVisualisationWindow failed:', e);
                }
            }
        };

        console.log('[renderer] electronAPI fallback initialised.');
    }
} catch (e) {
    console.warn('[renderer] Failed to initialise electronAPI fallback:', e);
}

// Movement trace configuration (for 3D visualisation)
// We keep the last 100 samples of the end effector and each joint position.
const MOVEMENT_TRACE_MAX_POINTS = 100;
let endEffectorTraceEnabled = false;
let jointTracesEnabled = false;
let endEffectorTracePoints = []; // Array of { x, y, z } in mm
let jointTracesPoints = [];      // Array of arrays: one array per joint, each with { x, y, z } in mm

// Dead zone configuration (stored in memory and persisted to localStorage)
const DEAD_ZONES_STORAGE_KEY = 'robotArmDeadZones';
const SAFE_Z_STORAGE_KEY = 'robotArmSafeZHeight';
let deadZones = [];
let nextDeadZoneId = 1;
let safeZHeight = 300; // Default safe Z height in mm for routing over dead zones

// Tool orientation (direction vector for the tool's local Z-axis)
// We keep this simple: a single global direction that all Cartesian moves try to follow.
// Default: point straight down along negative Z in the world frame.
let currentToolOrientation = { x: 0, y: 0, z: -1 };

/**
 * Updates stored position markers in 3D view if visualization and kinematics are ready.
 */
function update3DStoredPositionsIfAvailable() {
    try {
        if (!robotArm3D || typeof robotArm3D.updateStoredPositions !== 'function') {
            return;
        }
        if (typeof robotKinematics === 'undefined' || !robotKinematics.isConfigured || !robotKinematics.isConfigured()) {
            return;
        }
        if (typeof getAllPositions !== 'function') {
            return;
        }

        const positions = getAllPositions();
        const points = [];

        const keys = Object.keys(positions);
        for (let i = 0; i < keys.length; i++) {
            const num = keys[i];
            const pos = positions[num];
            if (!pos || !Array.isArray(pos.angles) || pos.angles.length === 0) {
                continue;
            }
            try {
                const pose = robotKinematics.forwardKinematics(pos.angles);
                if (!pose || !pose.position) continue;
                points.push({
                    label: pos.label || `P${num}`,
                    x: pose.position.x,
                    y: pose.position.y,
                    z: pose.position.z
                });
            } catch (e) {
                console.warn('Error computing FK for stored position', num, e);
            }
        }

        robotArm3D.updateStoredPositions(points);
    } catch (error) {
        console.error('update3DStoredPositionsIfAvailable error:', error);
    }
}

/**
 * Updates the kinematics matrices panel for the current pose.
 * If jointAnglesOverride is provided, uses that; otherwise tries to use the latest joint angles.
 * @param {Array} jointAnglesOverride - Optional array of joint angles in degrees
 */
function updateKinematicsMatrices(jointAnglesOverride) {
    const display = document.getElementById('kinematicsMatricesDisplay');
    if (!display) {
        return;
    }

        if (!robotKinematics || typeof robotKinematics.isConfigured !== 'function' || !robotKinematics.isConfigured()) {
            display.textContent = 'Kinematics not configured. Load a URDF configuration first.';
            return;
        }

    let angles = jointAnglesOverride;

    // If not provided, try to use current joint angles from the visualization
    if (!angles || !Array.isArray(angles) || angles.length === 0) {
        if (robotArm3D && Array.isArray(robotArm3D.jointAngles) && robotArm3D.jointAngles.length > 0) {
            angles = robotArm3D.jointAngles.slice();
        } else {
            display.textContent = 'No joint angles available. Move the robot or update the visualization first.';
            return;
        }
    }

    try {
        const fkSteps = robotKinematics.getForwardKinematicsSteps(angles);
        const steps = fkSteps.steps || [];

        if (steps.length === 0) {
            display.textContent = 'No steps available.';
            return;
        }

        let html = '';
        html += '<div class="kinematics-steps">';
        html += '<p><strong>Forward Kinematics Matrices (Base → Joint / Tool)</strong><br>';
        html += 'Angles shown include any zero offsets defined in the URDF. ';
        html += 'The top-left 3×3 of each matrix is the rotation; the last column is translation in millimetres.</p>';

        steps.forEach(step => {
            const name = step.name || `Step ${step.index}`;
            const input = step.angleInput != null ? step.angleInput.toFixed(2) : '0.00';
            const used = step.angleUsed != null ? step.angleUsed.toFixed(2) : '0.00';
            const origin = step.origin || { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
            const axis = step.axis || { x: 0, y: 0, z: 0 };

            const T = step.transform || [];
            const px_m = (T[0] && T[0][3]) || 0;
            const py_m = (T[1] && T[1][3]) || 0;
            const pz_m = (T[2] && T[2][3]) || 0;
            const px = px_m * 1000;
            const py = py_m * 1000;
            const pz = pz_m * 1000;

            html += '<div class="kinematics-step">';
            html += `<h4>${name} (step ${step.index})</h4>`;
            html += '<table class="kinematics-step-info"><tbody>';
            html += `<tr><td>Input angle</td><td>${input}°</td></tr>`;
            html += `<tr><td>Used angle (with zero offset)</td><td>${used}°</td></tr>`;
            html += `<tr><td>Origin (m)</td><td>(${(origin.x || 0).toFixed(3)}, ${(origin.y || 0).toFixed(3)}, ${(origin.z || 0).toFixed(3)})</td></tr>`;
            html += `<tr><td>Axis</td><td>(${(axis.x || 0).toFixed(3)}, ${(axis.y || 0).toFixed(3)}, ${(axis.z || 0).toFixed(3)})</td></tr>`;
            html += `<tr><td>Position (mm)</td><td>X=${px.toFixed(1)}, Y=${py.toFixed(1)}, Z=${pz.toFixed(1)}</td></tr>`;
            html += '</tbody></table>';

            html += '<table class="kinematics-matrix"><tbody>';
            for (let r = 0; r < 4; r++) {
                const row = T[r] || [0, 0, 0, 0];
                html += '<tr>';
                for (let c = 0; c < 4; c++) {
                    const v = row[c] || 0;
                    if (c === 3) {
                        const mm = v * 1000;
                        html += `<td>${mm.toFixed(1)}</td>`;
                    } else {
                        html += `<td>${v.toFixed(3)}</td>`;
                    }
                }
                html += '</tr>';
            }
            html += '</tbody></table>';
            html += '</div>';
        });

        html += '</div>';
        display.innerHTML = html;
    } catch (error) {
        console.error('updateKinematicsMatrices error:', error);
        display.textContent = 'Error calculating matrices: ' + error.message;
    }
}

// ===== Helper Functions =====

/**
 * Gets the number of joints from settings
 * @returns {number} Number of joints (default: 4)
 */
function getNumJoints() {
    const numJointsInput = document.getElementById('numJoints');
    if (numJointsInput) {
        const num = parseInt(numJointsInput.value);
        return isNaN(num) || num < 1 ? 4 : num; // Default to 4 if invalid
    }
    return 4; // Default
}

/**
 * Updates the UI to reflect the current number of joints setting
 * This regenerates all joint-related UI elements
 */
function updateJointUI() {
    const numJoints = getNumJoints();
    
    // Update simulated angles array to match number of joints
    while (simulatedAngles.length < numJoints) {
        simulatedAngles.push(0);
    }
    while (simulatedAngles.length > numJoints) {
        simulatedAngles.pop();
    }
    
    // Regenerate joint status cards
    generateJointStatusCards();
    
    // Regenerate joint controls
    generateJointControls();
    
    // Regenerate quick movement buttons
    generateQuickMovementButtons();
    
    // Regenerate simulated angle controls
    generateSimulatedAngleControls();
}

/**
 * Generates combined joint status + control cards dynamically
 * Uses joint limits from the URDF (via robotKinematics) when available,
 * so each joint's input range matches its real allowed angles.
 */
function generateJointStatusCards() {
    const container = document.getElementById('jointStatusGrid');
    if (!container) return;
    
    const numJoints = getNumJoints();

    // Try to read joint limits from kinematics (URDF) so we can
    // show them in the UI and also use them when clamping in moveJoint.
    let jointConfigsForLimits = [];
    try {
        if (typeof robotKinematics !== 'undefined' &&
            robotKinematics &&
            typeof robotKinematics.isConfigured === 'function' &&
            robotKinematics.isConfigured() &&
            typeof robotKinematics.getJointConfigs === 'function') {
            jointConfigsForLimits = robotKinematics.getJointConfigs() || [];
        }
    } catch (e) {
        console.warn('generateJointStatusCards: could not read kinematics joint limits, falling back to open range', e);
    }

    let html = '';
    
    for (let i = 1; i <= numJoints; i++) {
        // Default to an open range; we'll just display URDF limits if we have them.
        let limitsLabel = '';
        const cfg = jointConfigsForLimits[i - 1];
        if (cfg && cfg.limits) {
            const lower = (typeof cfg.limits.lowerDegrees === 'number') ? cfg.limits.lowerDegrees.toFixed(1) : null;
            const upper = (typeof cfg.limits.upperDegrees === 'number') ? cfg.limits.upperDegrees.toFixed(1) : null;
            if (lower !== null && upper !== null) {
                limitsLabel = ` (URDF: ${lower}° to ${upper}°)`;
            }
        }

        html += `
            <div class="joint-status-card">
                <h3>Joint ${i}</h3>
                <div class="joint-status-values">
                    <div class="status-value">Angle: <span id="joint${i}Angle">0.0</span>°</div>
                    <div class="status-value">Position: <span id="joint${i}Position">0</span></div>
                    <div class="status-value">Moving: <span id="joint${i}Moving">No</span></div>
                    <div class="status-value">Speed: <span id="joint${i}SpeedDisplay">0</span> degrees/s</div>
                    <div class="status-value">Load: <span id="joint${i}Load">0.0</span>%</div>
                    <div class="status-value">Voltage: <span id="joint${i}Voltage">0.0</span>V</div>
                    <div class="status-value">Temperature: <span id="joint${i}Temperature">0</span>°C</div>
                    <div class="status-value">Torque: <span id="joint${i}Torque">Off</span></div>
                </div>
                <div class="joint-control-fields">
                    <div class="joint-field-group">
                        <label for="joint${i}Target">Angle (degrees)${limitsLabel}:</label>
                        <input type="number" id="joint${i}Target" value="0" step="0.1">
                    </div>
                    <div class="joint-field-group">
                        <label for="joint${i}Speed">Speed (degrees/s):</label>
                        <input type="number" id="joint${i}Speed" value="45" min="0" max="300" step="1">
                    </div>
                    <div class="joint-field-group">
                        <label for="joint${i}Acceleration">Acceleration (0-254):</label>
                        <input type="number" id="joint${i}Acceleration" value="50" min="0" max="254" step="1">
                    </div>
                    <button class="btn btn-small" onclick="moveJoint(${i})">Move</button>
                </div>
            </div>
        `;
    }
    
    container.innerHTML = html;
}

/**
 * Legacy joint controls container
 * Now just shows an information message, because controls are combined with status cards.
 */
function generateJointControls() {
    const container = document.getElementById('jointControlsContainer');
    if (!container) return;
    
    container.innerHTML = '<p class="info-text">Joint controls are now combined with the status cards above.</p>';
}

/**
 * Generates quick movement buttons dynamically
 */
function generateQuickMovementButtons() {
    const container = document.getElementById('quickMovementButtons');
    if (!container) return;
    
    const numJoints = getNumJoints();
    let html = '';
    
    for (let i = 1; i <= numJoints; i++) {
        html += `
            <button class="direction-btn" onclick="quickMove(${i}, -1)">Joint ${i} -</button>
            <button class="direction-btn" onclick="quickMove(${i}, 1)">Joint ${i} +</button>
        `;
    }
    
    container.innerHTML = html;
}

/**
 * Generates simulated angle controls dynamically
 */
function generateSimulatedAngleControls() {
    const container = document.getElementById('simulatedAnglesGrid');
    if (!container) return;
    
    // Use kinematics configuration count if available, otherwise use numJoints setting
    // For 5-axis robot: 5 joints (tool offset doesn't need angle control)
    let numJoints = 5; // Default to 5 for 5-axis robot
    
    // Try to get count from kinematics configuration
    if (robotKinematics && robotKinematics.jointConfigs && robotKinematics.jointConfigs.length > 0) {
        // If we have 6 configs (5 joints + tool), use 5; otherwise use the count
        const configCount = robotKinematics.jointConfigs.length;
        if (configCount === 6) {
            numJoints = 5; // 5 joints + 1 tool offset
        } else {
            numJoints = configCount;
        }
    } else {
        // Fallback to input field value
        numJoints = getNumJoints();
        // Ensure at least 5 for 5-axis robot
        if (numJoints < 5) {
            numJoints = 5;
        }
    }
    
    let html = '';
    
    for (let i = 1; i <= numJoints; i++) {
        const currentValue = simulatedAngles[i - 1] || 0;
        html += `
            <div class="simulated-angle-control">
                <label>Joint ${i} (degrees):</label>
                <input type="number" id="simJoint${i}" value="${currentValue}" min="-180" max="180" step="0.1" onchange="updateSimulatedVisualization()">
            </div>
        `;
    }
    
    container.innerHTML = html;
}

// ===== Initialization =====

// Wait for the page to load completely
document.addEventListener('DOMContentLoaded', function() {
    console.log('Application loaded');
    
    // Detect screen size and apply optimizations
    detectScreenSize();
    
    // Initialize the UI
    initializeTabs();
    initializeConnection();
    initializeFileInput();
    initializeStatusUpdates();
    initialize3DVisualization();
    initializePositions();
    initializeGCodeEditor();
    initializeDeadZones();
    initializeKinematicsTab();
    
    // Load saved settings
    loadSettings();
    
    // Initialize joint UI based on settings
    updateJointUI();
    
    // Automatically load demo kinematics (URDF) on startup
    try {
        if (typeof loadDemoConfig === 'function') {
            loadDemoConfig();
        }
    } catch (error) {
        console.error('Failed to auto-load demo kinematics:', error);
    }

    // Local Raspberry Pi panel (only shown on the Pi)
    try {
        // Give the DOM a moment to settle, then check local server.
        setTimeout(function () {
            if (typeof checkLocalPiServer === 'function') {
                checkLocalPiServer();
            }
        }, 200);
    } catch (e) {
        console.warn('Failed to check local Pi server:', e);
    }
    
    // Re-check screen size on window resize
    window.addEventListener('resize', detectScreenSize);
});

// ===== Screen Size Detection =====

/**
 * Detects screen size and applies optimizations for small displays
 */
function detectScreenSize() {
    const width = window.innerWidth;
    const height = window.innerHeight;
    
    // Check if this is a small display (800x480 or similar)
    const isSmallDisplay = width <= 800 || height <= 600;
    
    // Add class to body for CSS targeting
    if (isSmallDisplay) {
        document.body.classList.add('small-display');
        console.log(`Small display detected: ${width}x${height}`);
    } else {
        document.body.classList.remove('small-display');
    }
    
    // Adjust 3D visualization height for small screens
    if (robotArm3D && isSmallDisplay) {
        const container = document.getElementById('robotArm3DContainer');
        if (container) {
            // Make 3D visualization smaller on small screens
            container.style.height = '250px';
        }
    }
}

// ===== Tab Management =====

/**
 * Initializes the tab switching functionality
 */
function initializeTabs() {
    const tabButtons = document.querySelectorAll('.tab-button');
    const tabContents = document.querySelectorAll('.tab-content');
    
    // Add click handlers to all tab buttons
    tabButtons.forEach(button => {
        button.addEventListener('click', function() {
            const targetTab = this.getAttribute('data-tab');
            
            // Remove active class from all buttons and contents
            tabButtons.forEach(btn => btn.classList.remove('active'));
            tabContents.forEach(content => content.classList.remove('active'));
            
            // Add active class to clicked button and corresponding content
            this.classList.add('active');
            const targetContent = document.getElementById(targetTab + '-tab');
            if (targetContent) {
            targetContent.classList.add('active');
            } else {
                console.error(`Tab content not found: ${targetTab}-tab`);
            }
            
            // If visualization tab is opened, ensure it's initialized and updated
            if (targetTab === 'visualization') {
                // Wait a bit for tab to be visible, then check/update visualization
                setTimeout(() => {
                    if (!robotArm3D) {
                        console.log('Initializing 3D visualization (tab opened)');
                        initialize3DVisualization();
                    } else {
                        // Tab is now visible, update visualization if configs are loaded
                        const configs = robotKinematics.jointConfigs || [];
                        if (configs.length > 0) {
                            const numJoints = getNumJoints();
                            const angles = useSimulatedAngles && simulatedAngles.length === numJoints 
                                ? simulatedAngles 
                                : Array(numJoints).fill(0);
                            robotArm3D.update(configs, angles);
                            console.log('Updated visualization when tab opened');
                        }
                        
                        // Always refresh dead zones when tab is opened
                        if (typeof robotArm3D.updateDeadZones === 'function') {
                            console.log('Refreshing dead zones in 3D visualization');
                            robotArm3D.updateDeadZones(deadZones);
                        }
                    }
                }, 100);
            }
            
            // If Positions tab is opened, ensure it's initialized
            if (targetTab === 'positions') {
                setTimeout(() => {
                    if (typeof initializePositions === 'function') {
                        initializePositions();
                    }
                }, 100);
            }
            
            // If Blockly tab is opened, ensure it's initialized
            if (targetTab === 'blockly') {
                setTimeout(() => {
                    // Check if Blockly is already initialized
                    const workspaceDiv = document.getElementById('blockly-workspace');
                    const alreadyInitialized = workspaceDiv && (
                        workspaceDiv.querySelector('.blocklyMainBackground') || 
                        workspaceDiv.querySelector('.blocklyWorkspace') ||
                        (typeof blocklyWorkspace !== 'undefined' && blocklyWorkspace !== null) ||
                        (typeof window !== 'undefined' && typeof window.blocklyWorkspace !== 'undefined' && window.blocklyWorkspace !== null)
                    );
                    
                    if (alreadyInitialized) {
                        console.log('Blockly workspace already initialized, skipping');
                        return;
                    }
                    
                    // Wait a bit longer for Blockly library to load if needed
                    if (typeof Blockly === 'undefined') {
                        console.log('Waiting for Blockly library to load...');
                        // Try again after a delay
                        setTimeout(() => {
                            if (typeof Blockly !== 'undefined') {
                                console.log('Initializing Blockly workspace (tab opened)');
                                // Try both global and window.initializeBlockly
                                const initFunc = typeof initializeBlockly !== 'undefined' ? initializeBlockly : 
                                                (typeof window !== 'undefined' && typeof window.initializeBlockly !== 'undefined' ? window.initializeBlockly : null);
                                if (typeof initFunc === 'function') {
                                    initFunc();
                                } else {
                                    console.error('initializeBlockly function not available. blocklyRobotArm.js may not have loaded.');
                                }
                            } else {
                                console.error('Blockly library still not loaded after delay');
                                const statusDiv = document.getElementById('blocklyStatus');
                                if (statusDiv) {
                                    statusDiv.textContent = 'Error: Blockly library failed to load. Please refresh the page.';
                                    statusDiv.style.color = '#e74c3c';
                                }
                            }
                        }, 500);
                    } else {
                        console.log('Initializing Blockly workspace (tab opened)');
                        // Try both global and window.initializeBlockly
                        const initFunc = typeof initializeBlockly !== 'undefined' ? initializeBlockly : 
                                        (typeof window !== 'undefined' && typeof window.initializeBlockly !== 'undefined' ? window.initializeBlockly : null);
                        if (typeof initFunc === 'function') {
                            initFunc();
                        } else {
                            console.error('initializeBlockly function not available. blocklyRobotArm.js may not have loaded.');
                            // Try to initialize after a short delay
                            setTimeout(() => {
                                const retryFunc = typeof initializeBlockly !== 'undefined' ? initializeBlockly : 
                                                 (typeof window !== 'undefined' && typeof window.initializeBlockly !== 'undefined' ? window.initializeBlockly : null);
                                if (typeof retryFunc === 'function') {
                                    retryFunc();
                                } else {
                                    console.error('initializeBlockly still not available after delay');
                                }
                            }, 200);
                        }
                    }
                }, 100);
            }
        });
    });

    // If this window was opened as a visualisation-only popup, auto-select
    // the 3D Visualisation tab when the page loads.
    try {
        const params = new URLSearchParams(window.location.search || '');
        if (params.get('visualisationOnly') === '1') {
            const vizButton = document.querySelector('.tab-button[data-tab="visualization"]');
            if (vizButton) {
                vizButton.click();
            }
        }
    } catch (e) {
        console.warn('initializeTabs: failed to auto-select visualisation tab from query string:', e);
    }
}

// ===== Visualisation Popup =====

/**
 * Opens the 3D visualisation in a separate always-on-top window.
 * Uses the Electron preload bridge (electronAPI) to ask the main
 * process to create the extra window.
 */
function openVisualisationPopup() {
    try {
        if (window.electronAPI && typeof window.electronAPI.openVisualisationWindow === 'function') {
            window.electronAPI.openVisualisationWindow();
        } else {
            showAppMessage('Opening extra 3D window is not available in this build.');
            console.warn('openVisualisationPopup: electronAPI.openVisualisationWindow is not available.');
        }
    } catch (e) {
        console.error('openVisualisationPopup failed:', e);
        showAppMessage('Failed to open 3D window.');
    }
}

// ===== Simple Message Helper =====

let appMessageTimeoutId = null;

/**
 * Shows a small non-blocking message in the bottom-right corner.
 * This replaces most uses of window.alert so that popups do not
 * interfere with keyboard focus.
 * @param {string} message - Message to show
 * @param {number} [durationMs] - How long to show it (default 3000ms)
 */
function showAppMessage(message, durationMs) {
    const box = document.getElementById('appMessage');
    if (!box) {
        // Fallback: if the box is missing, use standard alert
        window.alert(message);
        return;
    }

    box.textContent = message;
    box.style.display = 'block';

    if (appMessageTimeoutId) {
        clearTimeout(appMessageTimeoutId);
    }

    const timeout = typeof durationMs === 'number' ? durationMs : 3000;
    appMessageTimeoutId = setTimeout(function () {
        box.style.display = 'none';
    }, timeout);
}

// ===== Dead Zones UI =====

/**
 * Initializes the Dead Zones tab UI
 */
function initializeDeadZones() {
    // Safe Z height input
    const safeZInput = document.getElementById('deadZoneSafeZ');
    if (safeZInput) {
        // Load saved safe Z height from localStorage if available
        const savedSafeZ = localStorage.getItem(SAFE_Z_STORAGE_KEY);
        if (savedSafeZ !== null) {
            const parsed = parseFloat(savedSafeZ);
            if (!isNaN(parsed)) {
                safeZHeight = parsed;
            }
        }
        // Set initial value from global safeZHeight
        safeZInput.value = safeZHeight;
        safeZInput.addEventListener('change', function () {
            const value = parseFloat(this.value);
            if (!isNaN(value)) {
                safeZHeight = value;
                try {
                    localStorage.setItem(SAFE_Z_STORAGE_KEY, String(value));
                } catch (e) {
                    console.warn('Failed to save safe Z height:', e);
                }
            }
        });
    }

    // Add / update button
    const addButton = document.getElementById('addDeadZoneButton');
    if (addButton) {
        addButton.addEventListener('click', addDeadZoneFromForm);
    }

    // Load dead zones from localStorage if available
    try {
        const savedZones = localStorage.getItem(DEAD_ZONES_STORAGE_KEY);
        if (savedZones) {
            const parsedZones = JSON.parse(savedZones);
            if (Array.isArray(parsedZones)) {
                deadZones = parsedZones;
                // Ensure nextDeadZoneId is higher than any existing id
                const maxId = deadZones.reduce((max, z) => (z && typeof z.id === 'number' && z.id > max) ? z.id : max, 0);
                nextDeadZoneId = maxId + 1;
            }
        }
    } catch (e) {
        console.warn('Failed to load dead zones from storage:', e);
        deadZones = [];
        nextDeadZoneId = 1;
    }

    // Render any existing zones
    renderDeadZonesTable();

    // Update 3D visualization if available
    if (robotArm3D && typeof robotArm3D.updateDeadZones === 'function') {
        robotArm3D.updateDeadZones(deadZones);
    }
}

/**
 * Initialises the Kinematics tab (angle controls + first matrices view)
 */
function initializeKinematicsTab() {
    generateKinematicsAngleControls();
    // Show matrices for current simulated angles (or real angles if no sim yet)
    updateKinematicsMatrices();
}

/**
 * Generates joint angle controls for the Kinematics tab
 */
function generateKinematicsAngleControls() {
    const container = document.getElementById('kinematicsAnglesGrid');
    if (!container) return;

    // Decide how many joints to show
    let numJoints = 5; // Default
    if (robotKinematics && robotKinematics.jointConfigs && robotKinematics.jointConfigs.length > 0) {
        const configCount = robotKinematics.jointConfigs.length;
        // Ignore fixed tool joint if present
        numJoints = configCount === 6 ? 5 : configCount;
    } else {
        numJoints = getNumJoints();
        if (numJoints < 5) numJoints = 5;
    }

    // Ensure simulatedAngles length matches
    while (simulatedAngles.length < numJoints) {
        simulatedAngles.push(0);
    }
    while (simulatedAngles.length > numJoints) {
        simulatedAngles.pop();
    }

    let html = '';
    for (let i = 1; i <= numJoints; i++) {
        const currentValue = simulatedAngles[i - 1] || 0;
        html += `
            <div class="simulated-angle-control">
                <label>Joint ${i} (degrees):</label>
                <input type="text" id="kinematicsJoint${i}" value="${currentValue}" onchange="updateKinematicsFromAngles()">
            </div>
        `;
    }

    container.innerHTML = html;
}

/**
 * Reads angles from Kinematics tab inputs into simulatedAngles and updates matrices (and optionally 3D)
 */
function updateKinematicsFromAngles() {
    const angles = [];
    const numJoints = simulatedAngles.length || getNumJoints();

    for (let i = 1; i <= numJoints; i++) {
        const input = document.getElementById(`kinematicsJoint${i}`);
        if (input) {
            angles.push(parseFloat(input.value) || 0);
        }
    }

    if (angles.length > 0) {
        simulatedAngles = angles.slice();
    }

    // Update the matrices using these angles
    updateKinematicsMatrices(simulatedAngles);

    // If simulation mode is enabled in 3D, keep it in sync
    if (useSimulatedAngles && typeof updateSimulatedVisualization === 'function') {
        updateSimulatedVisualization();
    }
}

/**
 * Resets Kinematics simulated angles to 0°
 */
function resetSimulatedAnglesForKinematics() {
    const numJoints = simulatedAngles.length || getNumJoints();
    simulatedAngles = Array(numJoints).fill(0);
    generateKinematicsAngleControls();
    updateKinematicsMatrices(simulatedAngles);
    if (useSimulatedAngles && typeof updateSimulatedVisualization === 'function') {
        updateSimulatedVisualization();
    }
}

/**
 * Copies real robot angles into Kinematics simulated angles, then updates matrices
 */
function copyRealAnglesToSimForKinematics() {
    if (typeof copyRealAnglesToSim === 'function') {
        copyRealAnglesToSim();
    }
    // Refresh our local controls from the global simulatedAngles
    generateKinematicsAngleControls();
    updateKinematicsMatrices(simulatedAngles);
}

/**
 * Reads form inputs and adds a new dead zone
 */
function addDeadZoneFromForm() {
    const nameInput = document.getElementById('deadZoneName');
    const minXInput = document.getElementById('deadZoneMinX');
    const maxXInput = document.getElementById('deadZoneMaxX');
    const minYInput = document.getElementById('deadZoneMinY');
    const maxYInput = document.getElementById('deadZoneMaxY');
    const minZInput = document.getElementById('deadZoneMinZ');
    const maxZInput = document.getElementById('deadZoneMaxZ');

    if (!nameInput || !minXInput || !maxXInput || !minYInput || !maxYInput || !minZInput || !maxZInput) {
        return;
    }

    const name = nameInput.value.trim() || `Zone ${nextDeadZoneId}`;
    const minX = parseFloat(minXInput.value);
    const maxX = parseFloat(maxXInput.value);
    const minY = parseFloat(minYInput.value);
    const maxY = parseFloat(maxYInput.value);
    const minZ = parseFloat(minZInput.value);
    const maxZ = parseFloat(maxZInput.value);

    if (
        isNaN(minX) || isNaN(maxX) ||
        isNaN(minY) || isNaN(maxY) ||
        isNaN(minZ) || isNaN(maxZ)
    ) {
        showAppMessage('Please enter valid numeric values for all min/max fields.');
        return;
    }

    const zone = {
        id: nextDeadZoneId++,
        name: name,
        minX: minX,
        maxX: maxX,
        minY: minY,
        maxY: maxY,
        minZ: minZ,
        maxZ: maxZ,
        enabled: true
    };

    deadZones.push(zone);

    // Persist to localStorage
    try {
        localStorage.setItem(DEAD_ZONES_STORAGE_KEY, JSON.stringify(deadZones));
    } catch (e) {
        console.warn('Failed to save dead zones:', e);
    }

    // Clear name so the user can enter a new one easily
    nameInput.value = '';

    renderDeadZonesTable();

    // Update 3D visualization if available
    if (robotArm3D && typeof robotArm3D.updateDeadZones === 'function') {
        robotArm3D.updateDeadZones(deadZones);
    }
}

/**
 * Renders the dead zones table
 */
function renderDeadZonesTable() {
    const tbody = document.getElementById('deadZonesTableBody');
    if (!tbody) {
        return;
    }

    if (!deadZones || deadZones.length === 0) {
        tbody.innerHTML = '<tr><td colspan="9" style="text-align: center; color: #888;">No dead zones defined.</td></tr>';
        return;
    }

    let html = '';
    deadZones.forEach(zone => {
        html += `
            <tr>
                <td>${zone.name}</td>
                <td>${zone.minX}</td>
                <td>${zone.maxX}</td>
                <td>${zone.minY}</td>
                <td>${zone.maxY}</td>
                <td>${zone.minZ}</td>
                <td>${zone.maxZ}</td>
                <td>
                    <input type="checkbox" ${zone.enabled ? 'checked' : ''} onclick="toggleDeadZoneEnabled(${zone.id}, this.checked)">
                </td>
                <td>
                    <button class="btn btn-small" onclick="editDeadZone(${zone.id})">Edit</button>
                    <button class="btn btn-small btn-danger" onclick="deleteDeadZone(${zone.id})">Delete</button>
                </td>
            </tr>
        `;
    });

    tbody.innerHTML = html;
}

/**
 * Enables or disables a dead zone
 * @param {number} id - Dead zone ID
 * @param {boolean} enabled - Whether the zone is enabled
 */
function toggleDeadZoneEnabled(id, enabled) {
    const zone = deadZones.find(z => z.id === id);
    if (zone) {
        zone.enabled = enabled;
        try {
            localStorage.setItem(DEAD_ZONES_STORAGE_KEY, JSON.stringify(deadZones));
        } catch (e) {
            console.warn('Failed to save dead zones:', e);
        }
        if (robotArm3D && typeof robotArm3D.updateDeadZones === 'function') {
            robotArm3D.updateDeadZones(deadZones);
        }
    }
}

/**
 * Loads a dead zone into the form for editing
 * @param {number} id - Dead zone ID
 */
function editDeadZone(id) {
    const zone = deadZones.find(z => z.id === id);
    if (!zone) return;

    const nameInput = document.getElementById('deadZoneName');
    const minXInput = document.getElementById('deadZoneMinX');
    const maxXInput = document.getElementById('deadZoneMaxX');
    const minYInput = document.getElementById('deadZoneMinY');
    const maxYInput = document.getElementById('deadZoneMaxY');
    const minZInput = document.getElementById('deadZoneMinZ');
    const maxZInput = document.getElementById('deadZoneMaxZ');

    if (!nameInput || !minXInput || !maxXInput || !minYInput || !maxYInput || !minZInput || !maxZInput) {
        return;
    }

    nameInput.value = zone.name;
    minXInput.value = zone.minX;
    maxXInput.value = zone.maxX;
    minYInput.value = zone.minY;
    maxYInput.value = zone.maxY;
    minZInput.value = zone.minZ;
    maxZInput.value = zone.maxZ;
}

/**
 * Deletes a dead zone
 * @param {number} id - Dead zone ID
 */
function deleteDeadZone(id) {
    deadZones = deadZones.filter(z => z.id !== id);
    renderDeadZonesTable();
    if (robotArm3D && typeof robotArm3D.updateDeadZones === 'function') {
        robotArm3D.updateDeadZones(deadZones);
    }

    try {
        localStorage.setItem(DEAD_ZONES_STORAGE_KEY, JSON.stringify(deadZones));
    } catch (e) {
        console.warn('Failed to save dead zones:', e);
    }
}

// ===== Connection Management =====

/**
 * Initializes connection controls
 */
function initializeConnection() {
    console.log('Initializing connection handlers...');
    const connectButton = document.getElementById('connectButton');
    const autoDetectButton = document.getElementById('autoDetectButton');
    const disconnectButton = document.getElementById('disconnectButton');
    
    console.log('Buttons found:', {
        connect: !!connectButton,
        autoDetect: !!autoDetectButton,
        disconnect: !!disconnectButton
    });
    
    if (!connectButton) {
        console.error('Connect button not found in DOM');
        return;
    }
    
    if (!disconnectButton) {
        console.error('Disconnect button not found in DOM');
        return;
    }
    
    // Auto-detect button handler
    if (autoDetectButton) {
        autoDetectButton.addEventListener('click', async function() {
            autoDetectButton.disabled = true;
            const statusDiv = document.getElementById('autoDetectStatus');
            statusDiv.textContent = 'Searching for Raspberry Pi...';
            statusDiv.style.display = 'block';
            
            try {
                const foundAddress = await autoDetectRaspberryPi();
                if (foundAddress) {
                    document.getElementById('piAddress').value = foundAddress;
                    statusDiv.textContent = `Found Raspberry Pi at ${foundAddress}`;
                    statusDiv.style.color = '#27ae60';
                    // Optionally auto-connect
                    setTimeout(() => {
                        connectButton.click();
                    }, 500);
                } else {
                    statusDiv.textContent = 'Raspberry Pi not found. Make sure it is on the same network.';
                    statusDiv.style.color = '#e74c3c';
                }
            } catch (error) {
                statusDiv.textContent = `Error during detection: ${error.message}`;
                statusDiv.style.color = '#e74c3c';
            } finally {
                autoDetectButton.disabled = false;
            }
        });
    } else {
        console.warn('Auto-detect button not found');
    }
    
    // Connect button handler
    if (connectButton) {
    connectButton.addEventListener('click', async function() {
            console.log('Connect button clicked');
        const address = document.getElementById('piAddress').value;
        const port = parseInt(document.getElementById('piPort').value);
            
            console.log('Connection parameters:', { address, port });
        
        if (!address) {
            showAppMessage('Please enter Raspberry Pi address');
            return;
        }
        
        // Disable connect button, enable disconnect button
        connectButton.disabled = true;
            if (disconnectButton) disconnectButton.disabled = false;
        
        try {
                console.log(`Attempting to connect to ${address}:${port}`);
            // Connect to Raspberry Pi
            await robotArmClient.connect(address, port);
            
            // Set up status update callback
            robotArmClient.onStatusUpdate = updateJointStatus;
            
            // Set up configuration update callback
            robotArmClient.onConfigUpdate = onJointConfigsLoaded;
            
            // Start requesting status updates
            startStatusUpdates();
            
            // Request joint configurations
            robotArmClient.requestJointConfigs();
            
        } catch (error) {
                console.error('Connection error:', error);
            showAppMessage('Failed to connect: ' + error.message);
            connectButton.disabled = false;
                if (disconnectButton) disconnectButton.disabled = true;
        }
    });
        console.log('Connect button handler attached successfully');
    } else {
        console.error('Connect button not found!');
    }
    
    // Disconnect button handler
    if (disconnectButton) {
    disconnectButton.addEventListener('click', function() {
        robotArmClient.disconnect();
        stopStatusUpdates();
            if (connectButton) connectButton.disabled = false;
        disconnectButton.disabled = true;
    });
    } else {
        console.warn('Disconnect button not found');
    }
    
    console.log('Connection initialization complete');
}

// ===== Status Updates =====

/**
 * Starts periodic status updates
 */
function startStatusUpdates() {
    // Get update interval from settings (default 500ms)
    let interval = parseInt(document.getElementById('updateInterval')?.value || '500', 10);
    
    // Make sure the interval is a sensible value
    // Clamp to between 100 ms and 5000 ms to avoid "too fast" or "never updates"
    if (isNaN(interval)) {
        interval = 500;
    } else if (interval < 100) {
        interval = 100;
    } else if (interval > 5000) {
        interval = 5000;
    }
    
    // Clear any existing interval
    stopStatusUpdates();
    
    // Request status immediately
    robotArmClient.requestStatus();
    
    // Set up periodic status requests
    statusUpdateInterval = setInterval(function() {
        if (robotArmClient.isConnected) {
            robotArmClient.requestStatus();
        } else {
            stopStatusUpdates();
        }
    }, interval);
}

/**
 * Stops periodic status updates
 */
function stopStatusUpdates() {
    if (statusUpdateInterval) {
        clearInterval(statusUpdateInterval);
        statusUpdateInterval = null;
    }
}

/**
 * Initializes status update system
 */
function initializeStatusUpdates() {
    // Set up G-code processor logging
    gcodeProcessor.onLog = function(message) {
        const logElement = document.getElementById('gcodeLog');
        if (logElement) {
            logElement.textContent += message + '\n';
            // Auto-scroll to bottom
            logElement.scrollTop = logElement.scrollHeight;
        }
    };
    
    gcodeProcessor.onProgress = function(progress, current, total) {
        document.getElementById('gcodeProgress').textContent = progress;
        document.getElementById('currentLine').textContent = current;
        document.getElementById('totalLines').textContent = total;
    };
    
    // Set up callback for current line changes to highlight in editor
    gcodeProcessor.onLineChange = function(lineIndex, lineText) {
        highlightCurrentGCodeLine(lineIndex, lineText);
    };
}

/**
 * Updates the joint status display in the UI
 * @param {Array} joints - Array of joint status objects
 */
function updateJointStatus(joints) {
    // Update each joint's status
    const jointAngles = [];
    const numJoints = getNumJoints();
    
    // Only process up to the configured number of joints
    for (let i = 0; i < Math.min(joints.length, numJoints); i++) {
        const jointNumber = i + 1;
        const joint = joints[i];
        
        // Collect joint angles for 3D visualization
        // Be defensive: if the server sends null/undefined/non-number, treat it as 0
        const angleDegrees =
            (joint && typeof joint.angleDegrees === 'number' && !isNaN(joint.angleDegrees))
                ? joint.angleDegrees
                : 0;
        jointAngles.push(angleDegrees);
        
        // Update angle display
        const angleElement = document.getElementById(`joint${jointNumber}Angle`);
        if (angleElement) {
            angleElement.textContent = angleDegrees.toFixed(2);
        }
        
        // Update moving status
        const movingElement = document.getElementById(`joint${jointNumber}Moving`);
        if (movingElement) {
            movingElement.textContent = joint.isMoving ? 'Yes' : 'No';
            movingElement.style.color = joint.isMoving ? '#e74c3c' : '#27ae60';
        }
        
        // Update position display
        const positionElement = document.getElementById(`joint${jointNumber}Position`);
        if (positionElement) {
            // ST3215 server sends `position`; older I2C code used `stepPosition`.
            // Support both so the UI keeps working across both backends.
            const stepPosition =
                (joint && typeof joint.stepPosition === 'number') ? joint.stepPosition
                : (joint && typeof joint.position === 'number') ? joint.position
                : 0;
            positionElement.textContent = stepPosition;
        }
        
        // Update speed display (status card)
        // Convert steps/s to degrees/s for display
        const speedDisplayElement = document.getElementById(`joint${jointNumber}SpeedDisplay`);
        if (speedDisplayElement && joint && typeof joint.speed === 'number') {
            // Convert steps/s to degrees/s for display
            let speedDegreesPerSecond;
            if (typeof stepsPerSecondToDegreesPerSecond === 'function') {
                speedDegreesPerSecond = stepsPerSecondToDegreesPerSecond(joint.speed);
            } else if (typeof window !== 'undefined' && typeof window.stepsPerSecondToDegreesPerSecond === 'function') {
                speedDegreesPerSecond = window.stepsPerSecondToDegreesPerSecond(joint.speed);
            } else {
                // Fallback: use conversion factor directly (1 degree ≈ 11.37 steps)
                speedDegreesPerSecond = Math.round((joint.speed / 11.37) * 100) / 100;
            }
            speedDisplayElement.textContent = speedDegreesPerSecond;
        }
        
        // Update load display
        const loadElement = document.getElementById(`joint${jointNumber}Load`);
        if (loadElement && joint && typeof joint.load === 'number') {
            loadElement.textContent = joint.load.toFixed(1);
        }
        
        // Update voltage display
        const voltageElement = document.getElementById(`joint${jointNumber}Voltage`);
        if (voltageElement && joint && typeof joint.voltage === 'number') {
            voltageElement.textContent = joint.voltage.toFixed(1);
        }
        
        // Update temperature display
        const temperatureElement = document.getElementById(`joint${jointNumber}Temperature`);
        if (temperatureElement && joint && typeof joint.temperature === 'number') {
            temperatureElement.textContent = joint.temperature;
        }
        
        // Update torque display
        const torqueElement = document.getElementById(`joint${jointNumber}Torque`);
        if (torqueElement && joint && typeof joint.torqueEnabled === 'boolean') {
            torqueElement.textContent = joint.torqueEnabled ? 'On' : 'Off';
            torqueElement.style.color = joint.torqueEnabled ? '#27ae60' : '#e74c3c';
            
            // Update global torque state and button if all joints have the same state
            if (jointNumber === 1) {
                // Check if all joints have the same torque state
                let allSame = true;
                for (let j = 0; j < Math.min(joints.length, numJoints); j++) {
                    if (joints[j] && typeof joints[j].torqueEnabled === 'boolean') {
                        if (joints[j].torqueEnabled !== joint.torqueEnabled) {
                            allSame = false;
                            break;
                        }
                    }
                }
                
                if (allSame) {
                    torqueEnabled = joint.torqueEnabled;
                    const button = document.getElementById('torqueToggleButton');
                    if (button) {
                        button.textContent = `Torque: ${torqueEnabled ? 'On' : 'Off'}`;
                        button.classList.remove(torqueEnabled ? 'btn-secondary' : 'btn-warning');
                        button.classList.add(torqueEnabled ? 'btn-warning' : 'btn-secondary');
                    }
                }
            }
        }
    }
    
    // Pad with zeros if we have fewer joints than configured
    while (jointAngles.length < numJoints) {
        jointAngles.push(0);
    }
    
    // When simulation mode is on, use simulated angles so position display matches the sliders
    const anglesForDisplay = (useSimulatedAngles && simulatedAngles.length >= numJoints)
        ? simulatedAngles
        : jointAngles;
    
    // Update 3D visualization if available
    update3DVisualizationWithAngles(anglesForDisplay);
    
    // Update XYZ position display
    updateXYZPosition(anglesForDisplay);
}

// ===== Joint Control Functions =====

/**
 * Moves a joint to the target angle specified in the input field
 * @param {number} jointNumber - Joint number (1, 2, 3, etc.)
 */
function moveJoint(jointNumber) {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    // Get the target angle from the input field
    const inputElement = document.getElementById(`joint${jointNumber}Target`);
    if (!inputElement) {
        return;
    }
    
    let targetAngle = parseFloat(inputElement.value);
    
    if (isNaN(targetAngle)) {
        showAppMessage('Please enter a valid angle');
        return;
    }

    // Clamp to URDF joint limits if available so we never command
    // angles outside the configured range for this joint.
    try {
        if (typeof robotKinematics !== 'undefined' &&
            robotKinematics &&
            typeof robotKinematics.isConfigured === 'function' &&
            robotKinematics.isConfigured() &&
            typeof robotKinematics.getJointConfiguration === 'function') {
            const cfg = robotKinematics.getJointConfiguration(jointNumber - 1);
            if (cfg && cfg.limits) {
                let clamped = targetAngle;
                if (typeof cfg.limits.lowerDegrees === 'number' && clamped < cfg.limits.lowerDegrees) {
                    clamped = cfg.limits.lowerDegrees;
                }
                if (typeof cfg.limits.upperDegrees === 'number' && clamped > cfg.limits.upperDegrees) {
                    clamped = cfg.limits.upperDegrees;
                }
                if (clamped !== targetAngle) {
                    console.warn(`moveJoint: requested angle ${targetAngle}° for joint ${jointNumber} clamped to URDF range [${cfg.limits.lowerDegrees}, ${cfg.limits.upperDegrees}] → ${clamped}°`);
                    targetAngle = clamped;
                    inputElement.value = clamped.toFixed(2);
                }
            }
        }
    } catch (e) {
        console.warn('moveJoint: failed to clamp to URDF limits, sending raw angle', e);
    }
    
    // Get the speed from the input field (optional, defaults to 45 degrees/s)
    const speedInputElement = document.getElementById(`joint${jointNumber}Speed`);
    let speedDegreesPerSecond = 45; // Default speed in degrees/s
    if (speedInputElement) {
        // Check if it's an input element (not a span)
        if (speedInputElement.tagName === 'INPUT') {
            const speedValue = parseFloat(speedInputElement.value);
            if (!isNaN(speedValue) && speedValue >= 0 && speedValue <= 300) {
                speedDegreesPerSecond = speedValue;
            } else if (speedInputElement.value !== '') {
                console.warn(`Invalid speed value for joint ${jointNumber}: ${speedInputElement.value}, using default 45 degrees/s`);
            }
        } else {
            console.error(`Element joint${jointNumber}Speed is not an input field (found ${speedInputElement.tagName})`);
        }
    } else {
        console.warn(`Speed input element not found for joint ${jointNumber}`);
    }
    
    // Convert degrees/s to steps/s for the API
    // Use the conversion function if available, otherwise calculate directly
    let speedStepsPerSecond;
    if (typeof degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
    } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
    } else {
        // Fallback: use conversion factor directly (1 degree ≈ 11.37 steps)
        speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
    }
    
    // Get the acceleration from the input field (optional, defaults to 50)
    let accelerationValue = 50;
    const accelerationInputElement = document.getElementById(`joint${jointNumber}Acceleration`);
    if (accelerationInputElement) {
        if (accelerationInputElement.tagName === 'INPUT') {
            const acc = parseInt(accelerationInputElement.value, 10);
            if (!isNaN(acc) && acc >= 0 && acc <= 254) {
                accelerationValue = acc;
            } else if (accelerationInputElement.value !== '') {
                console.warn(`Invalid acceleration value for joint ${jointNumber}: ${accelerationInputElement.value}, using default 50`);
            }
        } else {
            console.error(`Element joint${jointNumber}Acceleration is not an input field (found ${accelerationInputElement.tagName})`);
        }
    } else {
        console.warn(`Acceleration input element not found for joint ${jointNumber}`);
    }

    // Send acceleration command before moving the joint
    // This uses the existing setAcceleration API on the Raspberry Pi
    try {
        if (typeof robotArmClient.setAcceleration === 'function') {
            console.log(`Setting acceleration for joint ${jointNumber} to ${accelerationValue}`);
            // We do not await the result here to keep the UI responsive
            robotArmClient.setAcceleration(jointNumber, accelerationValue);
        }
    } catch (error) {
        console.error(`Failed to set acceleration for joint ${jointNumber}:`, error);
    }
    
    // For single joint movements, move directly without dead zone checking
    // Dead zone checking is only needed for multi-joint movements that might cause
    // the end effector to pass through a dead zone
    robotArmClient.moveJoint(jointNumber, targetAngle, speedStepsPerSecond);
}

/**
 * Stops all joints immediately
 */
function stopAllJoints() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    robotArmClient.stopAllJoints();
}

/**
 * Homes all joints (moves them to 0 degrees)
 */
function homeAllJoints() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    // Convert default speed from degrees/s to steps/s
    const defaultSpeedDegreesPerSecond = 45;
    let speedStepsPerSecond;
    if (typeof degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = degreesPerSecondToStepsPerSecond(defaultSpeedDegreesPerSecond);
    } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(defaultSpeedDegreesPerSecond);
    } else {
        // Fallback: use conversion factor directly (1 degree ≈ 11.37 steps)
        speedStepsPerSecond = Math.round(defaultSpeedDegreesPerSecond * 11.37);
    }
    
    // Move all joints to 0 degrees at the default speed
    const numJoints = getNumJoints();
    for (let i = 1; i <= numJoints; i++) {
        robotArmClient.moveJoint(i, 0, speedStepsPerSecond);
    }
}

/**
 * Initialize G-Code editor functionality
 */
let gcodeOriginalContent = ''; // Store original content to detect changes

function initializeGCodeEditor() {
    const gcodeTextarea = document.getElementById('gcodeContent');
    if (!gcodeTextarea) return;
    
    // Store original content when loaded
    gcodeTextarea.addEventListener('input', function() {
        // Show modified indicator if content has changed
        const currentContent = gcodeTextarea.value;
        if (currentContent !== gcodeOriginalContent) {
            document.getElementById('gcodeModified').style.display = 'inline';
        } else {
            document.getElementById('gcodeModified').style.display = 'none';
        }
    });
}

/**
 * Applies changes from the G-Code editor to the processor
 */
function applyGCodeChanges() {
    const gcodeTextarea = document.getElementById('gcodeContent');
    if (!gcodeTextarea) {
        showAppMessage('G-Code editor not found');
        return;
    }
    
    const content = gcodeTextarea.value;
    
    // Load the updated G-code
    const result = gcodeProcessor.loadGCode(content);
    
    // Update line and command counts
    document.getElementById('gcodeLineCount').textContent = result.lines;
    document.getElementById('gcodeCommandCount').textContent = result.commands;
    
    // Update original content
    gcodeOriginalContent = content;
    
    // Clear modified indicator
    document.getElementById('gcodeModified').style.display = 'none';
    
    // Enable start button if not already enabled
    const startButton = document.getElementById('startGcodeButton');
    if (startButton) {
        startButton.disabled = false;
    }
    
    console.log(`G-Code updated: ${result.lines} lines, ${result.commands} commands`);
}

/**
 * Reloads the G-Code from the original file (if one was loaded)
 */
function reloadGCodeFromEditor() {
    if (gcodeOriginalContent) {
        document.getElementById('gcodeContent').value = gcodeOriginalContent;
        applyGCodeChanges();
    } else {
        showAppMessage('No original file loaded. Please load a G-Code file first.');
    }
}

/**
 * Highlights the current line being executed in the G-Code editor
 * @param {number} lineIndex - Index of the command being executed
 * @param {string} lineText - Text of the line being executed
 */
function highlightCurrentGCodeLine(lineIndex, lineText) {
    const textarea = document.getElementById('gcodeContent');
    if (!textarea) return;
    
    // Get all lines from the textarea
    const allLines = textarea.value.split('\n');
    
    // Get the command line from processor
    const processorLines = gcodeProcessor.getLines();
    if (lineIndex < 0 || lineIndex >= processorLines.length) {
        return;
    }
    
    const commandLine = processorLines[lineIndex];
    
    // Find this line in the textarea (accounting for comments and empty lines)
    // We need to match the command line to the actual line in the textarea
    let lineNumber = -1;
    let processedLineCount = 0;
    
    for (let i = 0; i < allLines.length; i++) {
        const textareaLine = allLines[i].trim();
        
        // Skip empty lines
        if (textareaLine.length === 0) {
            continue;
        }
        
        // Remove comments for comparison
        const textareaLineNoComment = textareaLine.split(';')[0].trim();
        
        // Skip if line is now empty after removing comments
        if (textareaLineNoComment.length === 0) {
            continue;
        }
        
        // Check if this processed line matches our command line
        if (processedLineCount === lineIndex) {
            // This should be our line - verify it matches
            if (textareaLineNoComment === commandLine || textareaLine.includes(commandLine)) {
                lineNumber = i;
                break;
            }
        }
        
        processedLineCount++;
    }
    
    // If we found the line, highlight it
    if (lineNumber >= 0 && lineNumber < allLines.length) {
        // Calculate the character position of the line start
        let lineStart = 0;
        for (let i = 0; i < lineNumber; i++) {
            lineStart += allLines[i].length + 1; // +1 for newline
        }
        
        // Calculate line end
        const lineEnd = lineStart + allLines[lineNumber].length;
        
        // Scroll to the line (center it in the viewport)
        const lineHeight = parseInt(window.getComputedStyle(textarea).lineHeight) || 20;
        const scrollTop = lineNumber * lineHeight - (textarea.clientHeight / 2) + lineHeight;
        textarea.scrollTop = Math.max(0, scrollTop);
        
        // Select the entire line to highlight it
        // This provides a visual indicator of the current line
        textarea.setSelectionRange(lineStart, lineEnd);
        
        // Focus the textarea so the selection is visible
        textarea.focus();
    }
}

/**
 * Toggles torque on/off for all joints
 */
let torqueEnabled = true; // Track current torque state

function updateTorqueButtons() {
    const buttons = document.querySelectorAll('.torque-toggle-button');
    buttons.forEach(function (button) {
        button.textContent = `Torque: ${torqueEnabled ? 'On' : 'Off'}`;
        button.classList.remove(torqueEnabled ? 'btn-secondary' : 'btn-warning');
        button.classList.add(torqueEnabled ? 'btn-warning' : 'btn-secondary');
    });
}

function toggleTorqueAll() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    // Toggle torque state
    torqueEnabled = !torqueEnabled;
    
    // Update all torque buttons
    updateTorqueButtons();
    
    // Send command to server
    robotArmClient.setTorqueAll(torqueEnabled)
        .then(() => {
            console.log(`Torque ${torqueEnabled ? 'enabled' : 'disabled'} for all joints`);
        })
        .catch(error => {
            console.error('Failed to toggle torque:', error);
            showAppMessage('Failed to toggle torque: ' + error.message);
            // Revert button state on error
            torqueEnabled = !torqueEnabled;
            updateTorqueButtons();
        });
}

/**
 * Moves the robot to the stored position selected on the Pendant tab.
 * Uses the joint angles saved in the Stored Positions page.
 */
async function moveToStoredPositionFromPendant() {
    const select = document.getElementById('pendantPositionSelect');
    if (!select) {
        showAppMessage('Stored position selector not found.');
        return;
    }

    const value = select.value;
    if (value === '') {
        showAppMessage('Please select a stored position first.');
        return;
    }

    const positionNumber = parseInt(value, 10);
    if (isNaN(positionNumber) || positionNumber < 0 || positionNumber > 99) {
        showAppMessage('Invalid stored position number.');
        return;
    }

    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }

    if (typeof getPosition !== 'function') {
        showAppMessage('Stored positions are not available.');
        return;
    }

    const position = getPosition(positionNumber);
    if (!position || !Array.isArray(position.angles) || position.angles.length === 0) {
        showAppMessage(`Stored position ${positionNumber} has no joint angles.`);
        return;
    }

    const numJoints = getNumJoints();
    const speedDegreesPerSecond = 45;
    showAppMessage(`Moving to stored position ${positionNumber}...`);

    // Use dead-zone-aware joint movement
    const targetAngles = [];
    for (let i = 0; i < numJoints; i++) {
        if (position.angles[i] !== undefined && typeof position.angles[i] === 'number') {
            targetAngles.push(position.angles[i]);
        } else {
            targetAngles.push(0);
        }
    }

    await moveJointsToAnglesWithDeadZones(targetAngles, speedDegreesPerSecond);
}

/**
 * Performs a quick movement (increment or decrement)
 * @param {number} jointNumber - Joint number
 * @param {number} direction - 1 for positive, -1 for negative
 */
function quickMove(jointNumber, direction) {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    // Get current angle from status display
    const angleElement = document.getElementById(`joint${jointNumber}Angle`);
    if (!angleElement) {
        return;
    }
    
    const currentAngle = parseFloat(angleElement.textContent) || 0;
    
    // Get step size
    const stepSizeInput = document.getElementById('stepSize');
    const stepSize = parseFloat(stepSizeInput?.value || '1');
    
    // Calculate new angle
    const newAngle = currentAngle + (direction * stepSize);

    // For single joint quick moves, move directly without dead zone checking
    // This allows multiple joints to move simultaneously
    const speedStepsPerSecond = (typeof degreesPerSecondToStepsPerSecond === 'function') ? 
        degreesPerSecondToStepsPerSecond(45) : 
        (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function' ?
            window.degreesPerSecondToStepsPerSecond(45) : Math.round(45 * 11.37));
    robotArmClient.moveJoint(jointNumber, newAngle, speedStepsPerSecond);
}

// ===== XYZ Position Control Functions =====

/**
 * Updates the XYZ position display based on current joint angles
 * @param {Array} jointAngles - Array of joint angles in degrees
 */
function updateXYZPosition(jointAngles) {
    // Check if kinematics is configured
    if (!robotKinematics.isConfigured() || !jointAngles || jointAngles.length === 0) {
        const currentXSpan = document.getElementById('currentX');
        const currentYSpan = document.getElementById('currentY');
        const currentZSpan = document.getElementById('currentZ');
        if (currentXSpan) currentXSpan.textContent = '-';
        if (currentYSpan) currentYSpan.textContent = '-';
        if (currentZSpan) currentZSpan.textContent = '-';

        const pendantXSpan = document.getElementById('pendantX');
        const pendantYSpan = document.getElementById('pendantY');
        const pendantZSpan = document.getElementById('pendantZ');
        if (pendantXSpan) pendantXSpan.textContent = '-';
        if (pendantYSpan) pendantYSpan.textContent = '-';
        if (pendantZSpan) pendantZSpan.textContent = '-';
        return;
    }
    
    try {
        // Calculate forward kinematics to get current end effector position
        const pose = robotKinematics.forwardKinematics(jointAngles);
        const pos = pose.position;
        
        // Update main XYZ display (Joint Control tab)
        const posX = pos.x.toFixed(1);
        const posY = pos.y.toFixed(1);
        const posZ = pos.z.toFixed(1);

        const currentXSpan = document.getElementById('currentX');
        const currentYSpan = document.getElementById('currentY');
        const currentZSpan = document.getElementById('currentZ');
        if (currentXSpan) currentXSpan.textContent = posX;
        if (currentYSpan) currentYSpan.textContent = posY;
        if (currentZSpan) currentZSpan.textContent = posZ;

        // Also update the Pendant touchscreen display if present
        const pendantXSpan = document.getElementById('pendantX');
        const pendantYSpan = document.getElementById('pendantY');
        const pendantZSpan = document.getElementById('pendantZ');
        if (pendantXSpan) pendantXSpan.textContent = posX;
        if (pendantYSpan) pendantYSpan.textContent = posY;
        if (pendantZSpan) pendantZSpan.textContent = posZ;

        // ===== Movement trace data (end effector and per-joint) =====
        // End effector trace: store last MOVEMENT_TRACE_MAX_POINTS XYZ samples
        if (endEffectorTraceEnabled) {
            const newPoint = {
                x: pos.x,
                y: pos.y,
                z: pos.z
            };
            endEffectorTracePoints.push(newPoint);
            // Trim to maximum number of points
            if (endEffectorTracePoints.length > MOVEMENT_TRACE_MAX_POINTS) {
                endEffectorTracePoints.shift();
            }

            // Update 3D end effector trace if available
            if (robotArm3D && typeof robotArm3D.updateEndEffectorTrace === 'function') {
                robotArm3D.updateEndEffectorTrace(endEffectorTracePoints);
            }
        }

        // Joint traces: use step-by-step forward kinematics to get each joint position
        if (jointTracesEnabled) {
            try {
                const fkSteps = robotKinematics.getForwardKinematicsSteps(jointAngles);
                const steps = fkSteps.steps || [];

                // Ensure jointTracesPoints has one array per step
                if (!Array.isArray(jointTracesPoints)) {
                    jointTracesPoints = [];
                }
                while (jointTracesPoints.length < steps.length) {
                    jointTracesPoints.push([]);
                }

                for (let i = 0; i < steps.length; i++) {
                    const T = steps[i].transform || [];
                    const px_m = (T[0] && T[0][3]) || 0;
                    const py_m = (T[1] && T[1][3]) || 0;
                    const pz_m = (T[2] && T[2][3]) || 0;
                    const px = px_m * 1000;
                    const py = py_m * 1000;
                    const pz = pz_m * 1000;

                    const jointPoint = { x: px, y: py, z: pz };

                    if (!Array.isArray(jointTracesPoints[i])) {
                        jointTracesPoints[i] = [];
                    }
                    jointTracesPoints[i].push(jointPoint);
                    if (jointTracesPoints[i].length > MOVEMENT_TRACE_MAX_POINTS) {
                        jointTracesPoints[i].shift();
                    }
                }

                // Update 3D joint traces if available
                if (robotArm3D && typeof robotArm3D.updateJointTraces === 'function') {
                    robotArm3D.updateJointTraces(jointTracesPoints);
                }
            } catch (e) {
                console.warn('Error calculating joint trace positions:', e);
            }
        }
    } catch (error) {
        console.error('Error calculating XYZ position:', error);
        const currentXSpan = document.getElementById('currentX');
        const currentYSpan = document.getElementById('currentY');
        const currentZSpan = document.getElementById('currentZ');
        if (currentXSpan) currentXSpan.textContent = 'Error';
        if (currentYSpan) currentYSpan.textContent = 'Error';
        if (currentZSpan) currentZSpan.textContent = 'Error';

        const pendantXSpan = document.getElementById('pendantX');
        const pendantYSpan = document.getElementById('pendantY');
        const pendantZSpan = document.getElementById('pendantZ');
        if (pendantXSpan) pendantXSpan.textContent = 'Error';
        if (pendantYSpan) pendantYSpan.textContent = 'Error';
        if (pendantZSpan) pendantZSpan.textContent = 'Error';
    }
}

/**
 * Moves the end effector to the specified XYZ position
 */
async function moveToXYZ() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    if (!robotKinematics.isConfigured()) {
        showAppMessage('Kinematics not configured. Please load joint configurations from the Settings tab first.');
        return;
    }
    
    // Get target position from input fields
    const x = parseFloat(document.getElementById('targetX').value);
    const y = parseFloat(document.getElementById('targetY').value);
    const z = parseFloat(document.getElementById('targetZ').value);
    
    if (isNaN(x) || isNaN(y) || isNaN(z)) {
        showAppMessage('Please enter valid X, Y, Z coordinates');
        return;
    }
    
    try {
        // Determine current position from XYZ display (used to plan around dead zones)
        let currentX = parseFloat(document.getElementById('currentX').textContent);
        let currentY = parseFloat(document.getElementById('currentY').textContent);
        let currentZ = parseFloat(document.getElementById('currentZ').textContent);
        if (!isFinite(currentX)) currentX = x;
        if (!isFinite(currentY)) currentY = y;
        if (!isFinite(currentZ)) currentZ = z;

        const startPose = { x: currentX, y: currentY, z: currentZ };
        const targetPose = { x: x, y: y, z: z };

        // Plan a simple safe path that avoids dead zones (up-over-down if needed)
        const waypoints = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
        if (!waypoints) {
            showAppMessage(`Target position (${x}, ${y}, ${z}) lies inside a dead zone. Move cancelled.`);
            return;
        }
        
        // For each waypoint, run IK with iterative refinement and move the joints.
        // Seed IK with the previous waypoint's refined angles so we don't get stuck on the wrong side (e.g. base yaw 0° when we need 180°).
        let lastAccuracy = null;
        let previousRefinedAngles = null;
        for (let w = 0; w < waypoints.length; w++) {
            const wp = waypoints[w];
            const baseAngles = robotKinematics.inverseKinematics(
                { x: wp.x, y: wp.y, z: wp.z },
                previousRefinedAngles
            );
            if (!baseAngles) {
                showAppMessage(`Target position (${wp.x}, ${wp.y}, ${wp.z}) is unreachable. Please choose a different position.`);
                return;
            }

            const refined = robotKinematics.refineOrientationWithAccuracy(
                { x: wp.x, y: wp.y, z: wp.z },
                baseAngles,
                currentToolOrientation
            );
            const jointAngles = refined.angles;
            previousRefinedAngles = jointAngles.slice();
            lastAccuracy = refined;

            console.log(`Moving to XYZ waypoint ${w + 1}/${waypoints.length}: (${wp.x}, ${wp.y}, ${wp.z})`);
            console.log(`Joint angles: ${jointAngles.map((a, i) => `J${i+1}:${a.toFixed(2)}°`).join(', ')}`);
            console.log(`Accuracy: position ${refined.positionErrorMm.toFixed(2)} mm, orientation ${refined.orientationErrorDeg.toFixed(1)}°`);

            for (let i = 0; i < jointAngles.length; i++) {
                robotArmClient.moveJoint(i + 1, jointAngles[i]);
                await new Promise(resolve => setTimeout(resolve, 50));
            }
        }

        if (lastAccuracy) {
            const msg = `Move complete. Position error: ${lastAccuracy.positionErrorMm.toFixed(2)} mm, orientation: ${lastAccuracy.orientationErrorDeg.toFixed(1)}°`;
            showAppMessage(msg);
        }
        
    } catch (error) {
        showAppMessage(`Error moving to position: ${error.message}`);
        console.error('Error in moveToXYZ:', error);
    }
}

/**
 * Performs a quick movement along X, Y, or Z axis
 * @param {string} axis - 'X', 'Y', or 'Z'
 * @param {number} direction - 1 for positive, -1 for negative
 */
async function quickMoveXYZ(axis, direction) {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    if (!robotKinematics.isConfigured()) {
        showAppMessage('Kinematics not configured. Please load joint configurations first.');
        return;
    }
    
    // Get current position
    const currentX = parseFloat(document.getElementById('currentX').textContent);
    const currentY = parseFloat(document.getElementById('currentY').textContent);
    const currentZ = parseFloat(document.getElementById('currentZ').textContent);
    
    if (isNaN(currentX) || isNaN(currentY) || isNaN(currentZ)) {
        showAppMessage('Current position not available. Please wait for status update.');
        return;
    }
    
    // Get step size
    const stepSizeInput = document.getElementById('xyzStepSize');
    const stepSize = parseFloat(stepSizeInput?.value || '5');
    
    // Calculate new position
    let newX = currentX;
    let newY = currentY;
    let newZ = currentZ;
    
    if (axis === 'X') {
        newX = currentX + (direction * stepSize);
    } else if (axis === 'Y') {
        newY = currentY + (direction * stepSize);
    } else if (axis === 'Z') {
        newZ = currentZ + (direction * stepSize);
    }
    
    // Update target inputs
    document.getElementById('targetX').value = newX.toFixed(1);
    document.getElementById('targetY').value = newY.toFixed(1);
    document.getElementById('targetZ').value = newZ.toFixed(1);
    
    // Move to new position
    await moveToXYZ();
}

/**
 * Copies current XYZ position to target inputs
 */
function copyCurrentXYZ() {
    const currentX = document.getElementById('currentX').textContent;
    const currentY = document.getElementById('currentY').textContent;
    const currentZ = document.getElementById('currentZ').textContent;
    
    if (currentX === '-' || currentY === '-' || currentZ === '-') {
        showAppMessage('Current position not available. Please wait for status update.');
        return;
    }
    
    document.getElementById('targetX').value = currentX;
    document.getElementById('targetY').value = currentY;
    document.getElementById('targetZ').value = currentZ;
}

// ===== Dead Zone Path Planning =====

/**
 * Returns true if a point (x,y,z) lies inside an enabled dead zone.
 * @param {{x:number,y:number,z:number}} p
 * @param {Object} zone
 */
function pointInZone(p, zone) {
    if (!zone || !zone.enabled) return false;
    return (
        p.x >= zone.minX && p.x <= zone.maxX &&
        p.y >= zone.minY && p.y <= zone.maxY &&
        p.z >= zone.minZ && p.z <= zone.maxZ
    );
}

/**
 * Returns true if a point lies in any enabled zone.
 * @param {{x:number,y:number,z:number}} p
 * @param {Array} zones
 */
function pointInAnyZone(p, zones) {
    if (!zones || zones.length === 0) return false;
    for (let i = 0; i < zones.length; i++) {
        if (pointInZone(p, zones[i])) {
            return true;
        }
    }
    return false;
}

/**
 * Returns true if the straight segment from start to end passes through any enabled dead zone.
 * This uses simple sampling along the line for robustness.
 * @param {{x:number,y:number,z:number}} start
 * @param {{x:number,y:number,z:number}} end
 * @param {Array} zones
 */
function segmentIntersectsAnyZone(start, end, zones) {
    if (!zones || zones.length === 0) return false;
    const steps = 40;
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const p = {
            x: start.x + (end.x - start.x) * t,
            y: start.y + (end.y - start.y) * t,
            z: start.z + (end.z - start.z) * t
        };
        for (let zIndex = 0; zIndex < zones.length; zIndex++) {
            if (pointInZone(p, zones[zIndex])) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Plans a simple safe path from start to target that avoids dead zones.
 * If a straight line does not intersect any enabled zone, returns [target].
 * If it does, returns up-over-down waypoints using the provided safeZ height.
 * If the final target lies inside a dead zone, returns null to indicate
 * that the move is not allowed.
 * Validates that all waypoints are reachable before returning them.
 * @param {{x:number,y:number,z:number}} start
 * @param {{x:number,y:number,z:number}} target
 * @param {Array} zones
 * @param {number} safeZMm
 * @returns {Array<{x:number,y:number,z:number}>}
 */
function planSafePathAroundDeadZones(start, target, zones, safeZMm) {
    if (!zones || zones.length === 0) {
        return [target];
    }

    // If the final target is inside any enabled dead zone, refuse the move
    if (pointInAnyZone(target, zones)) {
        console.warn('planSafePathAroundDeadZones: Target position', target, 'is inside a dead zone');
        // Log which zone it's in for debugging
        zones.forEach(function(zone, idx) {
            if (zone && zone.enabled && pointInZone(target, zone)) {
                console.warn(`  Inside zone ${idx}: X[${zone.minX},${zone.maxX}] Y[${zone.minY},${zone.maxY}] Z[${zone.minZ},${zone.maxZ}]`);
            }
        });
        return null;
    }

    // If straight line is clear, just go there directly
    if (!segmentIntersectsAnyZone(start, target, zones)) {
        return [target];
    }

    // Compute a safe Z that is at least above all zone maxZ, plus a small margin
    let highestZoneZ = safeZMm;
    zones.forEach(function (zone) {
        if (zone && zone.enabled && typeof zone.maxZ === 'number') {
            if (zone.maxZ + 20 > highestZoneZ) {
                highestZoneZ = zone.maxZ + 20;
            }
        }
    });

    const safeZ = highestZoneZ;

    // Strategy: Try multiple safe Z heights to find reachable waypoints
    // Start with the minimum safe Z (dead zone max + margin)
    // Then try progressively higher Z values if waypoints are unreachable
    const minReachableZ = Math.max(start.z || 0, target.z || 0);
    
    // Try different safe Z heights, starting from the minimum required
    const safeZOptions = [
        Math.max(highestZoneZ, minReachableZ + 10),  // Minimum safe Z
        Math.max(highestZoneZ, minReachableZ + 30),  // Medium safe Z
        Math.max(highestZoneZ, minReachableZ + 50),  // Higher safe Z
        highestZoneZ + 50,  // Maximum safe Z
    ];

    // Helper function to check if a waypoint is reachable
    const isReachable = function(wp) {
        if (!robotKinematics || !robotKinematics.isConfigured()) {
            return true; // Can't check, assume reachable
        }
        const ikResult = robotKinematics.inverseKinematics({
            x: wp.x,
            y: wp.y,
            z: wp.z
        });
        return ikResult !== null;
    };

    // Try each safe Z option until we find reachable waypoints
    for (let zIdx = 0; zIdx < safeZOptions.length; zIdx++) {
        const finalSafeZ = safeZOptions[zIdx];
        
        // Waypoints: up from start, over in XY at safe Z, then down to target
        const wp1 = { x: start.x, y: start.y, z: finalSafeZ };
        const wp2 = { x: target.x, y: target.y, z: finalSafeZ };

        const waypoints = [];

        // Only include wp1 if we are not already near safe Z
        if (Math.abs(start.z - finalSafeZ) > 1) {
            // Check if wp1 is reachable
            if (!isReachable(wp1)) {
                continue; // Try next safe Z option
            }
            waypoints.push(wp1);
        }
        
        // Only include wp2 if we need to move horizontally and target is not already at safe Z
        if ((Math.abs(target.x - start.x) > 0.1 || Math.abs(target.y - start.y) > 0.1) &&
            Math.abs(target.z - finalSafeZ) > 1) {
            // Check if wp2 is reachable
            if (!isReachable(wp2)) {
                // wp2 (target.x, target.y, safeZ) is unreachable
                // Check if we can go directly from lifted position to target without hitting dead zone
                const lastWp = waypoints.length > 0 ? waypoints[waypoints.length - 1] : start;
                if (segmentIntersectsAnyZone(lastWp, target, zones)) {
                    // Path would hit dead zone, try next safe Z option
                    continue;
                }
                // Path is clear, skip wp2 and go directly to target
            } else {
                waypoints.push(wp2);
            }
        }
        
        // Final target - always check if it's reachable
        if (!isReachable(target)) {
            continue; // Try next safe Z option
        }
        
        // Check if path from last waypoint to target avoids dead zone
        const lastWp = waypoints.length > 0 ? waypoints[waypoints.length - 1] : start;
        if (segmentIntersectsAnyZone(lastWp, target, zones)) {
            // Path would hit dead zone, try next safe Z option
            continue;
        }
        
        // Final target (only if it's different from the last waypoint)
        if (waypoints.length === 0 || 
            Math.abs(target.x - waypoints[waypoints.length - 1].x) > 0.1 ||
            Math.abs(target.y - waypoints[waypoints.length - 1].y) > 0.1 ||
            Math.abs(target.z - waypoints[waypoints.length - 1].z) > 0.1) {
            waypoints.push(target);
        }

        // If we got here, all waypoints are reachable
        return waypoints;
    }

    // If we couldn't find any reachable path, check if target is actually in a zone
    // If target is not in a zone, try smarter path planning with intermediate waypoints
    if (!pointInAnyZone(target, zones)) {
        console.warn('planSafePathAroundDeadZones: Could not find reachable waypoints with complex path, but target is not in a zone. Trying smarter path planning.');
        
        // Strategy: Find intermediate waypoints that:
        // 1. Are reachable
        // 2. Avoid the dead zone
        // 3. Help navigate around the dead zone
        
        // Calculate the center of the dead zone (for reference)
        let deadZoneCenterX = 0, deadZoneCenterY = 0, deadZoneCenterZ = 0;
        let deadZoneCount = 0;
        zones.forEach(function(zone) {
            if (zone && zone.enabled) {
                deadZoneCenterX += (zone.minX + zone.maxX) / 2;
                deadZoneCenterY += (zone.minY + zone.maxY) / 2;
                deadZoneCenterZ += (zone.minZ + zone.maxZ) / 2;
                deadZoneCount++;
            }
        });
        if (deadZoneCount > 0) {
            deadZoneCenterX /= deadZoneCount;
            deadZoneCenterY /= deadZoneCount;
            deadZoneCenterZ /= deadZoneCount;
        }
        
        // Try incremental Z heights, starting from just above the dead zone
        const minZ = Math.max(start.z || 0, target.z || 0);
        const maxZ = highestZoneZ + 100; // Try up to 100mm above dead zone
        
        // Try Z heights in increments of 20mm
        for (let testZ = highestZoneZ; testZ <= maxZ; testZ += 20) {
            // Strategy 1: Try lifting straight up from start, then moving to target
            const liftWp = { x: start.x, y: start.y, z: testZ };
            
            if (robotKinematics && robotKinematics.isConfigured()) {
                const liftIk = robotKinematics.inverseKinematics({
                    x: liftWp.x,
                    y: liftWp.y,
                    z: liftWp.z
                });
                if (liftIk) {
                    // Found a reachable lift height!
                    // Check if path from lift to target avoids dead zone
                    if (!segmentIntersectsAnyZone(liftWp, target, zones)) {
                        const simpleWaypoints = [];
                        if (Math.abs(start.z - testZ) > 1) {
                            simpleWaypoints.push(liftWp);
                        }
                        simpleWaypoints.push(target);
                        console.log(`planSafePathAroundDeadZones: Using simple lift-and-move path at Z=${testZ}:`, simpleWaypoints);
                        return simpleWaypoints;
                    }
                    
                    // Path from lift to target would hit dead zone
                    // Strategy 2: Find reachable intermediate waypoint by scaling back X,Y towards start
                    // Calculate direction from start to target
                    const dx = target.x - start.x;
                    const dy = target.y - start.y;
                    const distance = Math.sqrt(dx * dx + dy * dy);
                    
                    if (distance > 0) {
                        // Try to find a reachable intermediate waypoint
                        // Start with target X,Y at safe Z, then scale back towards start if unreachable
                        let bestIntermediateWp = null;
                        
                        // Try scale factors from 1.0 (target position) down to 0.1 (close to start)
                        // Start with target X,Y at safe Z, then scale back if unreachable
                        // This finds the furthest reachable point towards the target
                        const scaleFactors = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1];
                        let bestScale = 0;
                        
                        for (let scaleIdx = 0; scaleIdx < scaleFactors.length; scaleIdx++) {
                            const scale = scaleFactors[scaleIdx];
                            const intermediateX = start.x + dx * scale;
                            const intermediateY = start.y + dy * scale;
                            const intermediateWp = { x: intermediateX, y: intermediateY, z: testZ };
                            
                            // Check if intermediate waypoint avoids dead zone
                            if (!pointInAnyZone(intermediateWp, zones)) {
                                const intermediateIk = robotKinematics.inverseKinematics({
                                    x: intermediateWp.x,
                                    y: intermediateWp.y,
                                    z: intermediateWp.z
                                });
                                if (intermediateIk) {
                                    // Found a reachable intermediate waypoint!
                                    // Check if paths avoid dead zone
                                    if (!segmentIntersectsAnyZone(liftWp, intermediateWp, zones) &&
                                        !segmentIntersectsAnyZone(intermediateWp, target, zones)) {
                                        // Use the furthest reachable waypoint (closest to target)
                                        if (!bestIntermediateWp || scale > bestScale) {
                                            bestIntermediateWp = intermediateWp;
                                            bestScale = scale;
                                        }
                                    }
                                }
                            }
                        }
                        
                        if (bestIntermediateWp) {
                            const waypoints = [];
                            if (Math.abs(start.z - testZ) > 1) {
                                waypoints.push(liftWp);
                            }
                            waypoints.push(bestIntermediateWp);
                            waypoints.push(target);
                            console.log(`planSafePathAroundDeadZones: Using scaled intermediate waypoint at Z=${testZ}, scale=${bestScale.toFixed(2)}:`, waypoints);
                            return waypoints;
                        }
                    }
                    
                    // Strategy 3: Try moving horizontally at this Z first, then down to target
                    const horizontalWp = { x: target.x, y: target.y, z: testZ };
                    const horizontalIk = robotKinematics.inverseKinematics({
                        x: horizontalWp.x,
                        y: horizontalWp.y,
                        z: horizontalWp.z
                    });
                    if (horizontalIk && !segmentIntersectsAnyZone(horizontalWp, target, zones)) {
                        const simpleWaypoints = [];
                        if (Math.abs(start.z - testZ) > 1) {
                            simpleWaypoints.push(liftWp);
                        }
                        simpleWaypoints.push(horizontalWp);
                        simpleWaypoints.push(target);
                        console.log(`planSafePathAroundDeadZones: Using lift-horizontal-down path at Z=${testZ}:`, simpleWaypoints);
                        return simpleWaypoints;
                    }
                }
            }
        }
        
        // If we still couldn't find a reachable path, try going directly to target
        // (target is not in a dead zone, so this is safe even if path intersects)
        console.warn('planSafePathAroundDeadZones: Could not find any reachable safe path. Target is safe, but path may intersect dead zone.');
    }
    
    // If we still couldn't find a path, return null
    console.warn('planSafePathAroundDeadZones: Could not find reachable waypoints to avoid dead zone');
    return null;
}

/**
 * Checks whether the interpolated joint path between two joint-angle vectors
 * causes the TCP to pass through any enabled dead zone.
 * This samples along the joint-space path and uses forward kinematics to test
 * each intermediate TCP position against all zones.
 *
 * @param {Array<number>} currentAngles - Current joint angles in degrees
 * @param {Array<number>} targetAngles - Target joint angles in degrees
 * @param {Array} zones - Dead zone list
 * @param {number} [samples=40] - Number of interpolation samples
 * @returns {boolean} True if any sample lies inside a dead zone
 */
function jointPathIntersectsDeadZone(currentAngles, targetAngles, zones, samples) {
    if (!zones || zones.length === 0) return false;
    if (!robotKinematics || !robotKinematics.isConfigured()) return false;

    const numJoints = robotKinematics.getJointCount();
    const steps = typeof samples === 'number' && samples > 0 ? samples : 40;

    // Ensure angle arrays have the correct length
    const cur = [];
    const tgt = [];
    for (let i = 0; i < numJoints; i++) {
        cur.push(typeof currentAngles[i] === 'number' ? currentAngles[i] : 0);
        tgt.push(typeof targetAngles[i] === 'number' ? targetAngles[i] : 0);
    }

    for (let s = 0; s <= steps; s++) {
        const t = s / steps;
        const interpAngles = [];
        for (let j = 0; j < numJoints; j++) {
            interpAngles.push(cur[j] + (tgt[j] - cur[j]) * t);
        }

        try {
            const fk = robotKinematics.forwardKinematics(interpAngles);
            const p = fk.position;
            if (pointInAnyZone(p, zones)) {
                return true;
            }
        } catch (e) {
            console.warn('jointPathIntersectsDeadZone: FK failed at sample', s, e);
        }
    }

    return false;
}

/**
 * Moves all joints to the specified target angles, respecting dead zones.
 * If the joint-space path does not intersect any dead zone, this will move
 * directly in joint space. If it does, the move is converted into a
 * dead-zone-aware Cartesian path using the up-over-down planner.
 *
 * @param {Array<number>} targetAngles - Target joint angles in degrees
 * @param {number} speedDegreesPerSecond - Base speed in degrees/s
 */
async function moveJointsToAnglesWithDeadZones(targetAngles, speedDegreesPerSecond) {
    if (!robotArmClient || !robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }

    const numJoints = getNumJoints();
    const baseSpeed = speedDegreesPerSecond || 45;

    // Convert speed to steps/s
    let speedStepsPerSecond;
    if (typeof degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = degreesPerSecondToStepsPerSecond(baseSpeed);
    } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(baseSpeed);
    } else {
        speedStepsPerSecond = Math.round(baseSpeed * 11.37);
    }

    // Pad / trim target angles
    const tgt = [];
    for (let i = 0; i < numJoints; i++) {
        const a = typeof targetAngles[i] === 'number' ? targetAngles[i] : 0;
        tgt.push(a);
    }

    // If we have no kinematics or no dead zones, move directly in joint space
    if (!robotKinematics || !robotKinematics.isConfigured() || !deadZones || deadZones.length === 0) {
        for (let i = 0; i < numJoints; i++) {
            await robotArmClient.moveJoint(i + 1, tgt[i], speedStepsPerSecond);
        }
        return;
    }

    // Get current joint angles from status
    let currentAngles = [];
    try {
        const status = await robotArmClient.getStatus();
        for (let i = 0; i < numJoints; i++) {
            if (status[i] && typeof status[i].angleDegrees === 'number') {
                currentAngles.push(status[i].angleDegrees);
            } else {
                currentAngles.push(0);
            }
        }
    } catch (e) {
        console.warn('moveJointsToAnglesWithDeadZones: failed to read current status; moving directly.', e);
        for (let i = 0; i < numJoints; i++) {
            await robotArmClient.moveJoint(i + 1, tgt[i], speedStepsPerSecond);
        }
        return;
    }

    // Check if the joint-space path intersects any dead zone
    const intersects = jointPathIntersectsDeadZone(currentAngles, tgt, deadZones, 40);

    if (!intersects) {
        // Straight joint-space move is safe
        for (let i = 0; i < numJoints; i++) {
            await robotArmClient.moveJoint(i + 1, tgt[i], speedStepsPerSecond);
        }
        return;
    }

    // Joint path would go through a dead zone: convert to a Cartesian safe path
    try {
        const startFk = robotKinematics.forwardKinematics(currentAngles);
        const targetFk = robotKinematics.forwardKinematics(tgt);
        const startPose = startFk.position;
        const targetPose = targetFk.position;

        console.log('moveJointsToAnglesWithDeadZones: Planning path from', startPose, 'to', targetPose);
        console.log('moveJointsToAnglesWithDeadZones: Dead zones:', deadZones);
        
        const waypoints = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
        if (!waypoints) {
            console.error('moveJointsToAnglesWithDeadZones: planSafePathAroundDeadZones returned null');
            console.error('  Start pose:', startPose);
            console.error('  Target pose:', targetPose);
            console.error('  Dead zones:', deadZones);
            // Check if target is actually in a zone
            if (pointInAnyZone(targetPose, deadZones)) {
                console.error('  Target IS inside a dead zone according to pointInAnyZone');
                deadZones.forEach(function(zone, idx) {
                    if (zone && zone.enabled && pointInZone(targetPose, zone)) {
                        console.error(`  Inside zone ${idx}: X[${zone.minX},${zone.maxX}] Y[${zone.minY},${zone.maxY}] Z[${zone.minZ},${zone.maxZ}]`);
                    }
                });
                showAppMessage(`Target joint configuration would place the TCP inside a dead zone. Move cancelled. Target: (${targetPose.x.toFixed(1)}, ${targetPose.y.toFixed(1)}, ${targetPose.z.toFixed(1)})`);
                return;
            } else {
                // Target is NOT in a dead zone, but we couldn't find a safe path
                // Check if the direct path would go through the dead zone
                if (segmentIntersectsAnyZone(startPose, targetPose, deadZones)) {
                    // Direct path would go through dead zone - we must find a safe path
                    console.error('  Target is NOT inside a dead zone, but direct path would intersect dead zone.');
                    console.error('  Cannot proceed - no reachable safe path found.');
                    showAppMessage(`Cannot move: Target position is safe, but no reachable path found that avoids the dead zone. Try adjusting the target position or dead zone settings.`);
                    return;
                } else {
                    // Direct path is clear - safe to proceed
                    console.warn('  Target is NOT inside a dead zone, and direct path is clear.');
                    console.warn('  Proceeding with direct move.');
                    showAppMessage('Moving directly to target (path is clear).');
                    // Fall back to direct joint movement
                    for (let i = 0; i < numJoints; i++) {
                        await robotArmClient.moveJoint(i + 1, tgt[i], speedStepsPerSecond);
                    }
                    return;
                }
            }
        }
        console.log('moveJointsToAnglesWithDeadZones: Generated waypoints:', waypoints);

        let initialAngles = currentAngles.slice();

        for (let w = 0; w < waypoints.length; w++) {
            const wp = waypoints[w];
            const baseAngles = robotKinematics.inverseKinematics({
                x: wp.x,
                y: wp.y,
                z: wp.z
            }, initialAngles);
            const refined = baseAngles
                ? robotKinematics.refineOrientationWithAccuracy({ x: wp.x, y: wp.y, z: wp.z }, baseAngles, currentToolOrientation)
                : null;
            const ikAngles = refined ? refined.angles : null;
            if (!ikAngles) {
                if (w < waypoints.length - 1) {
                    console.warn(`Waypoint ${w + 1} (${wp.x}, ${wp.y}, ${wp.z}) is unreachable, trying to skip to target`);
                    const finalWp = waypoints[waypoints.length - 1];
                    const finalBaseAngles = robotKinematics.inverseKinematics({
                        x: finalWp.x,
                        y: finalWp.y,
                        z: finalWp.z
                    }, initialAngles);
                    const finalRefined = finalBaseAngles
                        ? robotKinematics.refineOrientationWithAccuracy({ x: finalWp.x, y: finalWp.y, z: finalWp.z }, finalBaseAngles, currentToolOrientation)
                        : null;
                    const finalIk = finalRefined ? finalRefined.angles : null;
                    if (finalIk) {
                        for (let i = 0; i < numJoints && i < finalIk.length; i++) {
                            await robotArmClient.moveJoint(i + 1, finalIk[i], speedStepsPerSecond);
                        }
                        return;
                    }
                }
                showAppMessage(`Failed to find IK solution for waypoint ${w + 1} (${wp.x}, ${wp.y}, ${wp.z}). Move cancelled.`);
                return;
            }

            for (let i = 0; i < numJoints && i < ikAngles.length; i++) {
                await robotArmClient.moveJoint(i + 1, ikAngles[i], speedStepsPerSecond);
            }

            initialAngles = ikAngles.slice();

            // Brief pause between waypoints
            await new Promise(resolve => setTimeout(resolve, 500));
        }
    } catch (e) {
        console.warn('moveJointsToAnglesWithDeadZones: error during safe path execution; falling back to direct move.', e);
        for (let i = 0; i < numJoints; i++) {
            await robotArmClient.moveJoint(i + 1, tgt[i], speedStepsPerSecond);
        }
    }
}

// ===== File Input =====

/**
 * Loads the default G-code example script
 */
function loadDefaultGCode() {
    // Default G-code script demonstrating joint movements and speed control
    const defaultGCode = `; Default G-Code Example for Robot Arm
; This script demonstrates basic joint movements and speed control
; 
; Commands used:
;   J1=angle J2=angle J3=angle J4=angle J5=angle - Move individual or multiple joints simultaneously
;   F speed - Set movement speed in degrees/s (0-300, default: 45)
;   G28 - Home all joints to 0 degrees
;   M0 - Pause (default: 2 seconds)
;   M0 P milliseconds - Pause for specified time (e.g., M0 P5000 = pause 5 seconds)
;   M204 A<value> - Set acceleration for all joints (0-254, unit: 100 step/s²)
;   M204 J<joint> A<value> - Set acceleration for specific joint (0-254, unit: 100 step/s²)
;   M30 - Program end

; Start at home position
G28 F45
M0

; Set acceleration for all joints to medium value (50 = 5000 step/s²)
M204 A50
M0

; Move Joint 1 to 45 degrees at slow speed
J1=45 F30
M0

; Move Joint 1 back to 0 degrees
J1=0 F30
M0

; Move Joint 2 to 30 degrees at medium speed
J2=30 F45
M0

; Move Joint 2 back to 0 degrees
J2=0 F45
M0

; Move multiple joints simultaneously at medium speed
J1=45 J2=30 F45
M0

; Move all 5 joints simultaneously at fast speed
J1=30 J2=20 J3=15 J4=10 J5=0 F60
M0

; Move joints to negative angles
J1=-30 J2=-20 J3=-15 F50
M0

; Return to home position
G28 F45
M0

; Program complete
M30`;

    // Load the default G-code
    const result = gcodeProcessor.loadGCode(defaultGCode);
    
    // Update UI
    document.getElementById('gcodeContent').value = defaultGCode;
    document.getElementById('gcodeLineCount').textContent = result.lines;
    document.getElementById('gcodeCommandCount').textContent = result.commands;
    
    // Store original content for change detection
    gcodeOriginalContent = defaultGCode;
    
    // Clear modified indicator
    document.getElementById('gcodeModified').style.display = 'none';
    document.getElementById('fileName').textContent = 'Default Example (loaded)';
    document.getElementById('startGcodeButton').disabled = false;
    
    gcodeProcessor.log('Default G-code example loaded');
}

/**
 * Initializes the G-code file input
 */
function initializeFileInput() {
    const fileInput = document.getElementById('gcodeFileInput');
    
    fileInput.addEventListener('change', function(event) {
        const file = event.target.files[0];
        
        if (!file) {
            return;
        }
        
        // Display file name
        document.getElementById('fileName').textContent = file.name;
        
        // Read the file
        const reader = new FileReader();
        
        reader.onload = function(e) {
            const content = e.target.result;
            
            // Load the G-code
            const result = gcodeProcessor.loadGCode(content);
            
            // Display the content
            document.getElementById('gcodeContent').value = content;
            document.getElementById('gcodeLineCount').textContent = result.lines;
            document.getElementById('gcodeCommandCount').textContent = result.commands;
            
            // Store original content for change detection
            gcodeOriginalContent = content;
            
            // Clear modified indicator
            document.getElementById('gcodeModified').style.display = 'none';
            
            // Enable start button
            document.getElementById('startGcodeButton').disabled = false;
        };
        
        reader.readAsText(file);
    });
}

// ===== G-Code Control Functions =====

/**
 * Starts G-code execution
 */
async function startGcode() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    if (gcodeProcessor.getCommands().length === 0) {
        showAppMessage('No G-code loaded');
        return;
    }
    
    // Clear the log
    document.getElementById('gcodeLog').textContent = '';
    
    // Update UI buttons
    document.getElementById('startGcodeButton').disabled = true;
    const pauseButton = document.getElementById('pauseGcodeButton');
    pauseButton.disabled = false;
    pauseButton.textContent = 'Pause';
    pauseButton.classList.remove('btn-success');
    pauseButton.classList.add('btn-warning');
    document.getElementById('stopGcodeButton').disabled = false;
    
    // Update status
    document.getElementById('gcodeStatus').textContent = 'Running';
    
    // Execute the G-code
    await gcodeProcessor.start(executeGCodeCommand);
    
    // Reset UI when done
    document.getElementById('startGcodeButton').disabled = false;
    pauseButton.disabled = true;
    pauseButton.textContent = 'Pause';
    pauseButton.classList.remove('btn-success');
    pauseButton.classList.add('btn-warning');
    document.getElementById('stopGcodeButton').disabled = true;
    document.getElementById('gcodeStatus').textContent = 'Ready';
    
    // Clear line highlighting
    const textarea = document.getElementById('gcodeContent');
    if (textarea) {
        textarea.setSelectionRange(0, 0);
    }
}

/**
 * Toggles between pause and resume for G-code execution
 */
function togglePauseResume() {
    const pauseButton = document.getElementById('pauseGcodeButton');
    const status = gcodeProcessor.getStatus();
    
    if (status.isPaused) {
        // Resume execution
        gcodeProcessor.resume();
        document.getElementById('gcodeStatus').textContent = 'Running';
        pauseButton.textContent = 'Pause';
        pauseButton.classList.remove('btn-success');
        pauseButton.classList.add('btn-warning');
    } else {
        // Pause execution
    gcodeProcessor.pause();
    document.getElementById('gcodeStatus').textContent = 'Paused';
        pauseButton.textContent = 'Resume';
        pauseButton.classList.remove('btn-warning');
        pauseButton.classList.add('btn-success');
    }
}

/**
 * Pauses G-code execution (kept for backwards compatibility, now uses toggle)
 */
function pauseGcode() {
    togglePauseResume();
}

/**
 * Stops G-code execution
 */
function stopGcode() {
    gcodeProcessor.stop();
    document.getElementById('gcodeStatus').textContent = 'Ready';
    document.getElementById('startGcodeButton').disabled = false;
    const pauseButton = document.getElementById('pauseGcodeButton');
    pauseButton.disabled = true;
    pauseButton.textContent = 'Pause';
    pauseButton.classList.remove('btn-success');
    pauseButton.classList.add('btn-warning');
    document.getElementById('stopGcodeButton').disabled = true;
    
    // Clear line highlighting
    const textarea = document.getElementById('gcodeContent');
    if (textarea) {
        textarea.setSelectionRange(0, 0);
    }
}

/**
 * Executes a single G-code command
 * This function is called by the G-code processor for each command
 * @param {Object} command - Parsed G-code command object
 */
async function executeGCodeCommand(command) {
    // Handle different G-code commands
    if (command.code === 'G0' || command.code === 'G1') {
        // G0 = rapid movement, G1 = linear movement
        // For now we support:
        //   - Stored position moves using P parameter (e.g., G1 P5 F45)
        //   - Joint-space moves using J1..J5 parameters (same as "J" commands)
        //   - Simple Cartesian X/Y/Z moves using a numeric inverse kinematics placeholder
        
        // Check if P parameter (position number or name) is specified
        if (command.params.P !== undefined) {
            let position = null;
            let positionNumber = null;
            let positionLabel = null;
            
            // Check if getPosition function is available
            if (typeof getPosition !== 'function') {
                gcodeProcessor.log(`Error: Stored positions are not available. Cannot execute G1 P${command.params.P}.`);
                return;
            }
            
            // Check if P is a string (position name) or number (position index)
            if (typeof command.params.P === 'string') {
                // P parameter is a position name (e.g., P"Home" or P'Home')
                const positionName = command.params.P.trim();
                
                // Check if getPositionByName function is available
                if (typeof getPositionByName !== 'function') {
                    gcodeProcessor.log(`Error: Position lookup by name is not available. Cannot execute G1 P"${positionName}".`);
                    return;
                }
                
                // Look up position by name
                const positionResult = getPositionByName(positionName);
                if (!positionResult || !positionResult.position) {
                    gcodeProcessor.log(`Error: Stored position "${positionName}" not found.`);
                    return;
                }
                
                position = positionResult.position;
                positionNumber = positionResult.number;
                positionLabel = position.label || positionName;
            } else {
                // P parameter is a number (position index)
                positionNumber = Math.round(command.params.P);
                
                // Get the stored position
                position = getPosition(positionNumber);
                if (!position || !Array.isArray(position.angles) || position.angles.length === 0) {
                    gcodeProcessor.log(`Error: Stored position ${positionNumber} not found or has no joint angles.`);
                    return;
                }
                
                positionLabel = position.label || `Position ${positionNumber}`;
            }
            
            // Get speed from F parameter (feed rate) - F parameter is in degrees/s
            const speedDegreesPerSecond = command.params.F || 45; // Default speed in degrees/s
            
            gcodeProcessor.log(`Moving to stored position ${positionNumber} (${positionLabel}) at speed ${speedDegreesPerSecond} degrees/s`);
            
            // Use the stored position's joint angles
            const targetAngles = position.angles.slice(); // Copy the array
            
            // Use the existing moveJointsToAnglesWithDeadZones function for dead-zone aware movement
            if (typeof moveJointsToAnglesWithDeadZones === 'function') {
                await moveJointsToAnglesWithDeadZones(targetAngles, speedDegreesPerSecond);
            } else {
                // Fallback: direct joint movement
                const numJoints = getNumJoints();
                let speedStepsPerSecond;
                if (typeof degreesPerSecondToStepsPerSecond === 'function') {
                    speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
                } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
                    speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
                } else {
                    speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
                }
                
                for (let i = 0; i < Math.min(targetAngles.length, numJoints); i++) {
                    if (robotArmClient.isConnected) {
                        robotArmClient.moveJoint(i + 1, targetAngles[i], speedStepsPerSecond);
                        await new Promise(resolve => setTimeout(resolve, 50));
                    }
                }
                await new Promise(resolve => setTimeout(resolve, 1000));
            }
            
            return;
        }
        
        // If the command specifies joint angles (J1..J5), treat this as a joint move.
        const hasJointParams =
            command.params.J1 !== undefined ||
            command.params.J2 !== undefined ||
            command.params.J3 !== undefined ||
            command.params.J4 !== undefined ||
            command.params.J5 !== undefined;
        
        if (hasJointParams) {
            // Reuse the existing joint-move handler by creating a "J" command
            const jointCommand = {
                code: 'J',
                params: Object.assign({}, command.params),
                line: command.line
            };
            await executeGCodeCommand(jointCommand);
            return;
        }
        
        // Optional orientation vector (I, J, K) for the tool's Z-axis
        if (typeof command.params.I === 'number' ||
            typeof command.params.J === 'number' ||
            typeof command.params.K === 'number') {
            const oriX = typeof command.params.I === 'number' ? command.params.I : 0;
            const oriY = typeof command.params.J === 'number' ? command.params.J : 0;
            const oriZ = typeof command.params.K === 'number' ? command.params.K : 0;
            setToolOrientationVector(oriX, oriY, oriZ);
        }

        // Check if kinematics is configured
        if (!robotKinematics.isConfigured()) {
            gcodeProcessor.log(`Warning: Kinematics not configured. Skipping ${command.code} command.`);
            return;
        }
        
        // Get target position from G-code parameters
        const x = command.params.X;
        const y = command.params.Y;
        const z = command.params.Z;
        
        // If no position is specified at all, there is nothing to do
        if (x === undefined && y === undefined && z === undefined) {
            gcodeProcessor.log(`No position specified in ${command.code} command`);
            return;
        }

        // Get speed from F parameter (feed rate) - F parameter is in degrees/s
        const speedDegreesPerSecond = command.params.F || 45; // Default speed in degrees/s
        // Convert degrees/s to steps/s for the API
        let speedStepsPerSecond;
        if (typeof degreesPerSecondToStepsPerSecond === 'function') {
            speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
        } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
            speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
        } else {
            speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
        }
        const speed = speedStepsPerSecond;
        
        try {
            // Work out the target XYZ in mm.
            // If any axis is not specified, we try to keep the current value from the UI.
            let currentX = parseFloat(document.getElementById('currentX').textContent);
            let currentY = parseFloat(document.getElementById('currentY').textContent);
            let currentZ = parseFloat(document.getElementById('currentZ').textContent);
            if (!isFinite(currentX)) currentX = 0;
            if (!isFinite(currentY)) currentY = 0;
            if (!isFinite(currentZ)) currentZ = 0;

            const startPose = { x: currentX, y: currentY, z: currentZ };
            const targetPose = {
                x: x !== undefined ? x : currentX,
                y: y !== undefined ? y : currentY,
                z: z !== undefined ? z : currentZ
            };

            // Plan dead-zone-aware path
            const waypoints = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
            if (!waypoints) {
                gcodeProcessor.log(`Error: Target position (${targetPose.x}, ${targetPose.y}, ${targetPose.z}) lies inside a dead zone. Move cancelled.`);
                return;
            }

            // For each waypoint, run IK and move joints
            for (let w = 0; w < waypoints.length; w++) {
                const wp = waypoints[w];

                // Get a starting guess for IK from the real robot status if available
                let initialAngles = null;
                if (robotArmClient && robotArmClient.isConnected) {
                    try {
                        const status = await robotArmClient.getStatus();
                        const numJoints = robotKinematics.getJointCount();
                        initialAngles = [];
                        for (let i = 0; i < numJoints; i++) {
                            if (status[i] && typeof status[i].angleDegrees === 'number') {
                                initialAngles.push(status[i].angleDegrees);
                            } else {
                                initialAngles.push(0);
                            }
                        }
                    } catch (statusError) {
                        console.warn('G-code: Failed to get status for IK starting guess, using zeros:', statusError);
                        initialAngles = null;
                    }
                }

                // First solve for position only
                const baseAngles = robotKinematics.inverseKinematics({
                    x: wp.x,
                    y: wp.y,
                    z: wp.z
                }, initialAngles);
                if (!baseAngles) {
                    gcodeProcessor.log(`Error: Target waypoint (${wp.x}, ${wp.y}, ${wp.z}) is unreachable`);
                    return;
                }

                // Then refine wrist joints to get closer to the desired tool orientation
                const jointAngles = robotKinematics.refineOrientationWithWrist(
                    { x: wp.x, y: wp.y, z: wp.z },
                    baseAngles,
                    currentToolOrientation
                );

                gcodeProcessor.log(`Moving to waypoint ${w + 1}/${waypoints.length}: X${wp.x} Y${wp.y} Z${wp.z} at speed ${speedDegreesPerSecond} degrees/s`);
                gcodeProcessor.log(`Joint angles: ${jointAngles.map((a, i) => `J${i+1}:${a.toFixed(2)}°`).join(', ')}`);

                // Move each joint with the specified speed
                for (let i = 0; i < jointAngles.length; i++) {
                    if (robotArmClient.isConnected) {
                        robotArmClient.moveJoint(i + 1, jointAngles[i], speed);
                    }
                    // Small delay between joint commands
                    await new Promise(resolve => setTimeout(resolve, 50));
                }

                // Brief pause between waypoints
                await new Promise(resolve => setTimeout(resolve, 500));
            }
        } catch (error) {
            gcodeProcessor.log(`Error executing ${command.code}: ${error.message}`);
        }
        
    } else if (command.code === 'G28') {
        // G28 = Home all joints (move to 0 degrees)
        gcodeProcessor.log('Homing all joints to 0 degrees');
        
        // Get speed from F parameter (in degrees/s)
        const speedDegreesPerSecond = command.params.F || 45; // Default speed in degrees/s
        // Convert degrees/s to steps/s for the API
        let speedStepsPerSecond;
        if (typeof degreesPerSecondToStepsPerSecond === 'function') {
            speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
        } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
            speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
        } else {
            speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
        }
        const speed = speedStepsPerSecond;
        
        if (robotArmClient.isConnected) {
            const numJoints = getNumJoints();
            for (let i = 1; i <= numJoints; i++) {
                robotArmClient.moveJoint(i, 0, speed);
                await new Promise(resolve => setTimeout(resolve, 50));
            }
            await new Promise(resolve => setTimeout(resolve, 1000));
        }
        
    } else if (command.code.startsWith('J') || command.params.J1 !== undefined || command.params.J2 !== undefined || command.params.J3 !== undefined || command.params.J4 !== undefined || command.params.J5 !== undefined) {
        // Joint command: J1, J2, J3, J4, J5 to move individual joints or multiple joints simultaneously
        // Format: J1=angle J2=angle J3=angle F=speed or J1=45 F45
        // Example: J1=45 J2=-30 F45 (move joints 1 and 2 simultaneously at 45 degrees/s)
        // Example: J1=45 J2=30 J3=20 J4=10 J5=0 F60 (move all 5 joints simultaneously)
        // Uses linear interpolation to scale speeds so all joints arrive simultaneously
        
        // Get speed from F parameter (in degrees/s)
        const speedDegreesPerSecond = command.params.F || 45; // Default speed in degrees/s
        const numJoints = getNumJoints();
        let movedAny = false;
        
        // Collect target angles for joints that need to move
        const targetAngles = [];
        const jointsToMove = [];
        for (let i = 1; i <= numJoints; i++) {
            const jParam = `J${i}`;
            const angle = command.params[jParam];
            if (angle !== undefined) {
                targetAngles.push(angle);
                jointsToMove.push(i);
                movedAny = true;
            } else {
                targetAngles.push(null); // Joint not specified in command
            }
        }
        
        if (movedAny) {
            // Get current joint angles for linear interpolation
            try {
                const status = await robotArmClient.getStatus();
                const currentAngles = [];
                for (let i = 0; i < numJoints; i++) {
                    if (status[i] && typeof status[i].angleDegrees === 'number') {
                        currentAngles.push(status[i].angleDegrees);
                    } else {
                        currentAngles.push(0);
                    }
                }
                
                // Filter to only joints that need to move
                const currentAnglesToMove = jointsToMove.map(jointNum => currentAngles[jointNum - 1]);
                const targetAnglesToMove = jointsToMove.map(jointNum => targetAngles[jointNum - 1]);
                
                // Calculate scaled speeds using linear interpolation
                let scaledSpeeds;
                if (typeof calculateScaledSpeeds === 'function') {
                    scaledSpeeds = calculateScaledSpeeds(currentAnglesToMove, targetAnglesToMove, speedDegreesPerSecond);
                } else if (typeof window !== 'undefined' && typeof window.calculateScaledSpeeds === 'function') {
                    scaledSpeeds = window.calculateScaledSpeeds(currentAnglesToMove, targetAnglesToMove, speedDegreesPerSecond);
                } else {
                    // Fallback: use base speed for all joints
                    scaledSpeeds = Array(jointsToMove.length).fill(speedDegreesPerSecond);
                }
                
                // Move joints with scaled speeds
                gcodeProcessor.log(`Moving joints ${jointsToMove.join(', ')} simultaneously (scaled speeds for synchronized arrival)`);
                for (let i = 0; i < jointsToMove.length; i++) {
                    const jointNum = jointsToMove[i];
                    const targetAngle = targetAnglesToMove[i];
                    const scaledSpeedDegreesPerSecond = scaledSpeeds[i];
                    
                    if (scaledSpeedDegreesPerSecond > 0) {
                        // Convert degrees/s to steps/s
                        let speedStepsPerSecond;
                        if (typeof degreesPerSecondToStepsPerSecond === 'function') {
                            speedStepsPerSecond = degreesPerSecondToStepsPerSecond(scaledSpeedDegreesPerSecond);
                        } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
                            speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(scaledSpeedDegreesPerSecond);
                        } else {
                            speedStepsPerSecond = Math.round(scaledSpeedDegreesPerSecond * 11.37);
                        }
                        
                        gcodeProcessor.log(`  Joint ${jointNum}: ${targetAngle}° at ${scaledSpeedDegreesPerSecond.toFixed(1)} degrees/s`);
                        if (robotArmClient.isConnected) {
                            robotArmClient.moveJoint(jointNum, targetAngle, speedStepsPerSecond);
                        }
                    }
                }
            } catch (error) {
                // Fallback: use base speed if status retrieval fails
                console.warn('Failed to get joint status for linear interpolation, using base speed:', error);
                gcodeProcessor.log(`Moving joints ${jointsToMove.join(', ')} simultaneously at ${speedDegreesPerSecond} degrees/s`);
                let speedStepsPerSecond;
                if (typeof degreesPerSecondToStepsPerSecond === 'function') {
                    speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
                } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
                    speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
                } else {
                    speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
                }
                for (let i = 0; i < jointsToMove.length; i++) {
                    const jointNum = jointsToMove[i];
                    const targetAngle = targetAnglesToMove[i];
                    gcodeProcessor.log(`Moving Joint ${jointNum} to ${targetAngle}° at speed ${speedDegreesPerSecond} degrees/s`);
                    if (robotArmClient.isConnected) {
                        robotArmClient.moveJoint(jointNum, targetAngle, speedStepsPerSecond);
                    }
                }
            }
        }
        
        // If no joint parameters found, check if the command code itself is a joint command (e.g., J1=45)
        if (!movedAny && command.code.startsWith('J')) {
            const jointMatch = command.code.match(/J(\d+)/i);
            if (jointMatch) {
                const jointNumber = parseInt(jointMatch[1]);
                // Try to get angle from the command line directly
                const angleMatch = command.line.match(/J\d+\s*=\s*([-+]?\d*\.?\d+)/i);
                if (angleMatch) {
                    const angle = parseFloat(angleMatch[1]);
                    gcodeProcessor.log(`Moving Joint ${jointNumber} to ${angle}° at speed ${speedDegreesPerSecond} degrees/s`);
                    if (robotArmClient.isConnected) {
                        robotArmClient.moveJoint(jointNumber, angle, speed);
                        movedAny = true;
                    }
                }
            }
        }
        
        if (movedAny) {
            // Wait for all movements to complete
            await new Promise(resolve => setTimeout(resolve, 1000));
        } else {
            gcodeProcessor.log(`Warning: No joint angles specified in ${command.code} command`);
        }
        
    } else if (command.code.startsWith('M')) {
        // M codes (miscellaneous commands)
        if (command.code === 'M0' || command.code === 'M1') {
            // M0 = Program pause (wait for user), M1 = Optional stop
            const pauseTime = command.params.P || 2000; // P parameter = pause time in milliseconds (default 2 seconds)
            gcodeProcessor.log(`M-code: ${command.code} (pausing for ${pauseTime}ms)`);
            await new Promise(resolve => setTimeout(resolve, pauseTime));
        } else if (command.code === 'M3' || command.code === 'M4') {
            // Spindle on - not applicable to our robot
            gcodeProcessor.log(`M-code: ${command.code} (spindle on - ignored)`);
        } else if (command.code === 'M5') {
            // Spindle off - not applicable
            gcodeProcessor.log(`M-code: ${command.code} (spindle off - ignored)`);
        } else if (command.code === 'M204') {
            // M204 = Set acceleration
            // Format: M204 A<value> - Set acceleration for all joints
            // Format: M204 J<joint> A<value> - Set acceleration for specific joint
            // Acceleration range: 0-254 (unit: 100 step/s²)
            
            const acceleration = command.params.A;
            const joint = command.params.J;
            
            if (acceleration === undefined) {
                gcodeProcessor.log('Warning: M204 requires A parameter (acceleration value 0-254)');
                return;
            }
            
            // Validate acceleration range
            const accValue = Math.max(0, Math.min(254, Math.round(acceleration)));
            
            if (joint !== undefined) {
                // Set acceleration for specific joint
                const jointNumber = Math.round(joint);
                if (jointNumber >= 1 && jointNumber <= getNumJoints()) {
                    gcodeProcessor.log(`Setting acceleration for Joint ${jointNumber} to ${accValue} (unit: 100 step/s²)`);
                    if (robotArmClient.isConnected) {
                        robotArmClient.setAcceleration(jointNumber, accValue);
                    }
                } else {
                    gcodeProcessor.log(`Error: Invalid joint number ${jointNumber} in M204 command`);
                }
            } else {
                // Set acceleration for all joints
                gcodeProcessor.log(`Setting acceleration for all joints to ${accValue} (unit: 100 step/s²)`);
                if (robotArmClient.isConnected) {
                    const numJoints = getNumJoints();
                    for (let i = 1; i <= numJoints; i++) {
                        robotArmClient.setAcceleration(i, accValue);
                        await new Promise(resolve => setTimeout(resolve, 50));
                    }
                }
            }
            
        } else if (command.code === 'M30' || command.code === 'M2') {
            // Program end
            gcodeProcessor.log(`M-code: ${command.code} (program end)`);
        } else {
            gcodeProcessor.log(`M-code: ${command.code} (not implemented)`);
        }
    } else {
        // Unknown command
        gcodeProcessor.log(`Unknown command: ${command.code}`);
    }
}

// ===== RAPID Control Functions (Simple Subset) =====

/**
 * Loads a simple example RAPID program into the editor
 */
function loadDefaultRapid() {
    const example = `! Simple RAPID example for the robot arm
! Lines starting with "!" are comments and are ignored.

! Move to a safe starting pose (same as Home)
Home;

! Raise the arm a little with an absolute joint move
MoveAbsJ [[0,10,20,0,0]];

! Move by joint offsets relative to the current pose
! (Here we lower joint 2 by 5 degrees and joint 3 by 10)
MoveJOffs [[0,-5,-10,0,0]];

! Wait for 2 seconds
WaitTime 2;

! Turn on a (simulated) digital output 1
SetDO 1, 1;

! Return to home
Home;`;

    const textarea = document.getElementById('rapidContent');
    if (textarea) {
        textarea.value = example;
    }
}

/**
 * Parses a simple RAPID MoveJ line of the form:
 *   MoveJ [[J1,J2,J3,J4,J5]];
 * Returns an array of joint angles in degrees, or null if parsing fails.
 * @param {string} line - RAPID source line
 * @returns {Array<number>|null}
 */
function parseRapidMoveJ(line) {
    if (!line) {
        return null;
    }

    // Basic format: MoveJ [[a,b,c,d,e]];
    const moveMatch = line.match(/MoveJ\s*\[\s*\[\s*([^\]]+)\s*\]\s*\]/i);
    if (!moveMatch) {
        return null;
    }

    const inside = moveMatch[1];
    const parts = inside.split(',').map(function (p) {
        return parseFloat(p.trim());
    });

    if (parts.length === 0) {
        return null;
    }

    const numJoints = getNumJoints();
    const angles = [];

    for (let i = 0; i < numJoints; i++) {
        if (typeof parts[i] === 'number' && !isNaN(parts[i])) {
            angles.push(parts[i]);
        } else {
            angles.push(0);
        }
    }

    return angles;
}

/**
 * Parses a simple RAPID XYZ line of the form:
 *   MoveLXYZ [[X,Y,Z]];
 * or:
 *   MoveLOffs [[dX,dY,dZ]];
 * Returns an array [X, Y, Z] (in mm) or null if parsing fails.
 * @param {string} line - RAPID source line
 * @param {string} keyword - Command name to match (e.g. 'MoveLXYZ', 'MoveLOffs')
 * @returns {Array<number>|null}
 */
function parseRapidXYZ(line, keyword) {
    if (!line) {
        return null;
    }

    const regex = new RegExp(keyword + '\\s*\\[\\s*\\[\\s*([^\\]]+)\\s*\\]\\s*\\]', 'i');
    const match = line.match(regex);
    if (!match) {
        return null;
    }

    const inside = match[1];
    const parts = inside.split(',').map(function (p) {
        return parseFloat(p.trim());
    });

    if (parts.length < 3) {
        return null;
    }

    const values = [];
    for (let i = 0; i < 3; i++) {
        if (typeof parts[i] === 'number' && !isNaN(parts[i])) {
            values.push(parts[i]);
        } else {
            return null;
        }
    }

    return values;
}

/**
 * Runs the RAPID program written in the RAPID editor.
 * Supports:
 *   - Comment lines starting with "!"
 *   - MoveJ [[J1,J2,J3,J4,J5]];
 *   - MoveAbsJ [[J1,J2,J3,J4,J5]];
 *   - MoveJOffs [[dJ1,dJ2,dJ3,dJ4,dJ5]];
 *   - MoveLXYZ [[X,Y,Z]];
 *   - MoveLOffs [[dX,dY,dZ]];
 *   - WaitTime t;
 *   - SetDO n, v;
 *   - Home;
 */
async function runRapidProgram() {
    if (!robotArmClient || !robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }

    const textarea = document.getElementById('rapidContent');
    if (!textarea) {
        showAppMessage('RAPID editor not found in the page.');
        return;
    }

    const source = textarea.value || '';
    const lines = source.split(/\r?\n/);

    if (lines.length === 0) {
        showAppMessage('No RAPID code to run.');
        return;
    }

    const numJoints = getNumJoints();

    // Fixed speed for now (degrees per second)
    const speedDegreesPerSecond = 45;
    let speedStepsPerSecond;
    if (typeof degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
    } else if (typeof window !== 'undefined' && typeof window.degreesPerSecondToStepsPerSecond === 'function') {
        speedStepsPerSecond = window.degreesPerSecondToStepsPerSecond(speedDegreesPerSecond);
    } else {
        speedStepsPerSecond = Math.round(speedDegreesPerSecond * 11.37);
    }

    console.log('Starting RAPID program...');

    for (let i = 0; i < lines.length; i++) {
        let line = lines[i];
        if (!line) {
            continue;
        }

        line = line.trim();

        // Skip empty lines and comments
        if (line.length === 0 || line.startsWith('!')) {
            continue;
        }

        // Absolute joint move: MoveJ and MoveAbsJ
        if (/^MoveJ\b/i.test(line) || /^MoveAbsJ\b/i.test(line)) {
            const angles = parseRapidMoveJ(line);
            if (!angles) {
                console.warn('RAPID: Could not parse MoveJ/MoveAbsJ on line', i + 1, ':', line);
                continue;
            }

            console.log('RAPID: MoveJ/MoveAbsJ on line', i + 1, 'angles:', angles);
            
            // Use dead-zone-aware joint movement helper
            const targetAngles = [];
            for (let j = 0; j < numJoints; j++) {
                if (typeof angles[j] === 'number' && !isNaN(angles[j])) {
                    targetAngles.push(angles[j]);
                } else {
                    targetAngles.push(0);
                }
            }
            await moveJointsToAnglesWithDeadZones(targetAngles, speedDegreesPerSecond);
        } else if (/^MoveJOffs\b/i.test(line)) {
            // Incremental joint move: offsets added to current joint angles
            const offsets = parseRapidMoveJ(line);
            if (!offsets) {
                console.warn('RAPID: Could not parse MoveJOffs on line', i + 1, ':', line);
                continue;
            }

            console.log('RAPID: MoveJOffs on line', i + 1, 'offsets:', offsets);

            let currentAngles = new Array(numJoints).fill(0);
            try {
                const status = await robotArmClient.getStatus();
                for (let j = 0; j < numJoints; j++) {
                    if (status[j] && typeof status[j].angleDegrees === 'number') {
                        currentAngles[j] = status[j].angleDegrees;
                    }
                }
            } catch (err) {
                console.warn('RAPID: Failed to read current joint angles for MoveJOffs, using zeros:', err);
            }

            const targetAngles = [];
            for (let j = 0; j < numJoints; j++) {
                const offset = typeof offsets[j] === 'number' && !isNaN(offsets[j]) ? offsets[j] : 0;
                targetAngles.push(currentAngles[j] + offset);
            }
            
            // Use dead-zone-aware joint movement helper
            await moveJointsToAnglesWithDeadZones(targetAngles, speedDegreesPerSecond);
        } else if (/^MoveLXYZ\b/i.test(line)) {
            // Cartesian move to an absolute XYZ position in mm
            if (!robotKinematics.isConfigured()) {
                console.warn('RAPID: Kinematics not configured, cannot run MoveLXYZ.');
                continue;
            }

            const xyz = parseRapidXYZ(line, 'MoveLXYZ');
            if (!xyz) {
                console.warn('RAPID: Could not parse MoveLXYZ on line', i + 1, ':', line);
                continue;
            }

            const targetPose = { x: xyz[0], y: xyz[1], z: xyz[2] };
            console.log('RAPID: MoveLXYZ on line', i + 1, 'target XYZ:', targetPose);

            // Get current XYZ from the UI as a starting pose
            let currentX = parseFloat(document.getElementById('currentX').textContent);
            let currentY = parseFloat(document.getElementById('currentY').textContent);
            let currentZ = parseFloat(document.getElementById('currentZ').textContent);
            if (!isFinite(currentX)) currentX = targetPose.x;
            if (!isFinite(currentY)) currentY = targetPose.y;
            if (!isFinite(currentZ)) currentZ = targetPose.z;
            const startPose = { x: currentX, y: currentY, z: currentZ };

            const waypointsRapid = planSafePathAroundDeadZones(startPose, targetPose, deadZones, safeZHeight);
            if (!waypointsRapid) {
                console.warn('RAPID: MoveLXYZ target lies inside a dead zone. Move cancelled.');
                continue;
            }

            for (let w = 0; w < waypointsRapid.length; w++) {
                const wp = waypointsRapid[w];

                // Try to get a starting guess from the real robot
                let initialAngles = null;
                try {
                    const status = await robotArmClient.getStatus();
                    initialAngles = [];
                    for (let j = 0; j < numJoints; j++) {
                        if (status[j] && typeof status[j].angleDegrees === 'number') {
                            initialAngles.push(status[j].angleDegrees);
                        } else {
                            initialAngles.push(0);
                        }
                    }
                } catch (err) {
                    console.warn('RAPID: Failed to get status for MoveLXYZ starting guess, using zeros:', err);
                    initialAngles = null;
                }

                // First solve for position only
                const baseAngles = robotKinematics.inverseKinematics({
                    x: wp.x,
                    y: wp.y,
                    z: wp.z
                }, initialAngles);
                if (!baseAngles) {
                    console.warn('RAPID: MoveLXYZ IK failed for waypoint', wp);
                    break;
                }

                // Then refine wrist joints to get closer to the desired tool orientation
                const jointAngles = robotKinematics.refineOrientationWithWrist(
                    { x: wp.x, y: wp.y, z: wp.z },
                    baseAngles,
                    currentToolOrientation
                );

                for (let j = 0; j < numJoints; j++) {
                    const targetAngle = jointAngles[j];
                    if (typeof targetAngle === 'number' && !isNaN(targetAngle)) {
                        robotArmClient.moveJoint(j + 1, targetAngle, speedStepsPerSecond);
                    }
                }

                await new Promise(function (resolve) {
                    setTimeout(resolve, 800);
                });
            }
        } else if (/^MoveLOffs\b/i.test(line)) {
            // Cartesian offset move: offsets applied in XYZ space in mm
            if (!robotKinematics.isConfigured()) {
                console.warn('RAPID: Kinematics not configured, cannot run MoveLOffs.');
                continue;
            }

            const offsets = parseRapidXYZ(line, 'MoveLOffs');
            if (!offsets) {
                console.warn('RAPID: Could not parse MoveLOffs on line', i + 1, ':', line);
                continue;
            }

            // Get current XYZ from the UI
            let currentX = parseFloat(document.getElementById('currentX').textContent);
            let currentY = parseFloat(document.getElementById('currentY').textContent);
            let currentZ = parseFloat(document.getElementById('currentZ').textContent);
            if (!isFinite(currentX)) currentX = 0;
            if (!isFinite(currentY)) currentY = 0;
            if (!isFinite(currentZ)) currentZ = 0;

            const startPoseRapid = { x: currentX, y: currentY, z: currentZ };
            const targetPose = {
                x: currentX + offsets[0],
                y: currentY + offsets[1],
                z: currentZ + offsets[2]
            };

            console.log('RAPID: MoveLOffs on line', i + 1, 'offsets:', offsets, 'target XYZ:', targetPose);

            const waypointsRapidOffs = planSafePathAroundDeadZones(startPoseRapid, targetPose, deadZones, safeZHeight);
            if (!waypointsRapidOffs) {
                console.warn('RAPID: MoveLOffs target lies inside a dead zone. Move cancelled.');
                continue;
            }

            for (let w = 0; w < waypointsRapidOffs.length; w++) {
                const wp = waypointsRapidOffs[w];

                // Starting guess from current joint angles
                let initialAngles2 = null;
                try {
                    const status2 = await robotArmClient.getStatus();
                    initialAngles2 = [];
                    for (let j = 0; j < numJoints; j++) {
                        if (status2[j] && typeof status2[j].angleDegrees === 'number') {
                            initialAngles2.push(status2[j].angleDegrees);
                        } else {
                            initialAngles2.push(0);
                        }
                    }
                } catch (err2) {
                    console.warn('RAPID: Failed to get status for MoveLOffs starting guess, using zeros:', err2);
                    initialAngles2 = null;
                }

                // First solve for position only
                const baseAngles2 = robotKinematics.inverseKinematics({
                    x: wp.x,
                    y: wp.y,
                    z: wp.z
                }, initialAngles2);
                if (!baseAngles2) {
                    console.warn('RAPID: MoveLOffs IK failed for waypoint', wp);
                    break;
                }

                // Then refine wrist joints to get closer to the desired tool orientation
                const jointAngles2 = robotKinematics.refineOrientationWithWrist(
                    { x: wp.x, y: wp.y, z: wp.z },
                    baseAngles2,
                    currentToolOrientation
                );

                for (let j = 0; j < numJoints; j++) {
                    const targetAngle = jointAngles2[j];
                    if (typeof targetAngle === 'number' && !isNaN(targetAngle)) {
                        robotArmClient.moveJoint(j + 1, targetAngle, speedStepsPerSecond);
                    }
                }

                await new Promise(function (resolve) {
                    setTimeout(resolve, 800);
                });
            }
        } else if (/^SetToolOri\b/i.test(line)) {
            // SetToolOri [[ux,uy,uz]];  → set global tool orientation vector
            const xyz = parseRapidXYZ(line, 'SetToolOri');
            if (!xyz) {
                console.warn('RAPID: Could not parse SetToolOri on line', i + 1, ':', line);
                continue;
            }
            setToolOrientationVector(xyz[0], xyz[1], xyz[2]);
            console.log('RAPID: SetToolOri on line', i + 1, 'orientation:', xyz);
        } else if (/^WaitTime\b/i.test(line)) {
            // WaitTime t; where t is seconds
            const match = line.match(/WaitTime\s+([0-9]*\.?[0-9]+)/i);
            if (match) {
                const seconds = parseFloat(match[1]);
                const ms = isNaN(seconds) ? 0 : Math.max(0, seconds * 1000);
                console.log('RAPID: WaitTime', seconds, 'seconds');
                await new Promise(function (resolve) {
                    setTimeout(resolve, ms);
                });
            } else {
                console.warn('RAPID: Could not parse WaitTime value on line', i + 1, ':', line);
            }
        } else if (/^SetDO\b/i.test(line)) {
            // SetDO n, v; where n is channel, v is 0 or 1
            const match = line.match(/SetDO\s+(\d+)\s*,\s*(\d+)/i);
            if (match) {
                const channel = parseInt(match[1], 10);
                const value = parseInt(match[2], 10) ? 1 : 0;
                console.log('RAPID: SetDO channel', channel, 'to', value);
                if (typeof robotArmClient.setOutput === 'function') {
                    try {
                        robotArmClient.setOutput(channel, value);
                    } catch (err) {
                        console.warn('RAPID: setOutput failed:', err);
                    }
                }
            } else {
                console.warn('RAPID: Could not parse SetDO on line', i + 1, ':', line);
            }
        } else if (/^Home\b/i.test(line)) {
            // Home; is just a MoveAbsJ to all zeros
            const homeAngles = new Array(numJoints).fill(0);
            console.log('RAPID: Home on line', i + 1);
            for (let j = 0; j < numJoints; j++) {
                robotArmClient.moveJoint(j + 1, homeAngles[j], speedStepsPerSecond);
            }
            await new Promise(function (resolve) {
                setTimeout(resolve, 1000);
            });
        } else {
            console.warn('RAPID: Unsupported line (only MoveJ/MoveAbsJ/MoveJOffs/MoveLXYZ/MoveLOffs/WaitTime/SetDO/Home are implemented):', line);
        }
    }

    console.log('RAPID program finished.');
}

// ===== Settings =====

/**
 * Returns true when the app is likely running on a Raspberry Pi.
 * We keep this simple: Raspberry Pi usually reports linux + arm/arm64.
 * If we cannot read arch, we fall back to linux only.
 */
function isLikelyRaspberryPi() {
    try {
        const platform = (window.electronAPI && window.electronAPI.platform) ? window.electronAPI.platform : null;
        const isLinux = platform === 'linux';
        let arch = null;
        if (typeof process !== 'undefined' && process && typeof process.arch === 'string') {
            arch = process.arch;
        }
        const isArm = arch === 'arm' || arch === 'arm64';
        if (arch === null) {
            return isLinux;
        }
        return isLinux && isArm;
    } catch (e) {
        return false;
    }
}

/**
 * Checks whether the local ST3215 WebSocket server is reachable on 127.0.0.1.
 * The ST3215 server is WebSocket-only (no HTTP), so we try opening a WS connection.
 *
 * @param {number} port
 * @returns {Promise<{ ok: boolean, message: string }>}
 */
function probeLocalSt3215Server(port) {
    return new Promise(function (resolve) {
        const portNumber = (typeof port === 'number' && isFinite(port)) ? port : 8080;
        const url = `ws://127.0.0.1:${portNumber}`;

        let finished = false;
        const finish = function (ok, message, info) {
            if (finished) return;
            finished = true;
            resolve({ ok: ok, message: message, info: info || null });
        };

        let ws = null;
        const timeoutMs = 900;
        const timer = setTimeout(function () {
            try {
                if (ws && ws.readyState === WebSocket.OPEN) {
                    ws.close();
                }
            } catch (e) {
                // ignore
            }
            finish(false, `No response from local server at ${url}`);
        }, timeoutMs);

        try {
            ws = new WebSocket(url);
        } catch (e) {
            clearTimeout(timer);
            finish(false, `Could not create WebSocket to ${url}: ${e.message || e}`);
            return;
        }

        ws.onopen = function () {
            // Once connected, ask the server for network information.
            try {
                ws.send(JSON.stringify({ command: 'getPiNetworkInfo' }));
            } catch (e) {
                // If we cannot send the command, we still know the server is up.
            }
        };

        ws.onmessage = function (event) {
            try {
                const data = JSON.parse(event.data);
                if (data && data.type === 'networkInfo') {
                    clearTimeout(timer);
                    try { ws.close(); } catch (e) { /* ignore */ }
                    finish(true, `Local ST3215 server detected at ${url}`, {
                        hostname: data.hostname || null,
                        interfaces: Array.isArray(data.interfaces) ? data.interfaces : [],
                        gateway: data.gateway || null
                    });
                    return;
                }
                if (data && data.type === 'connected') {
                    // Welcome message; we know the server is up even if networkInfo is not implemented.
                    clearTimeout(timer);
                    try { ws.close(); } catch (e) { /* ignore */ }
                    finish(true, `Local ST3215 server detected at ${url}`, null);
                }
            } catch (e) {
                // If we got any message at all, the server is up.
                clearTimeout(timer);
                try { ws.close(); } catch (e2) { /* ignore */ }
                finish(true, `Local ST3215 server responded at ${url}`, null);
            }
        };

        ws.onerror = function () {
            clearTimeout(timer);
            finish(false, `Could not connect to local server at ${url}`);
        };
    });
}

/**
 * Updates the Settings tab "Local Raspberry Pi" panel visibility and status text.
 * - Hidden on non-Pi systems.
 * - Shown on Pi systems, with a status line indicating whether the local server is reachable.
 */
async function checkLocalPiServer() {
    const panel = document.getElementById('localPiSettingsPanel');
    const statusText = document.getElementById('localPiServerStatusText');
    if (!panel || !statusText) {
        return;
    }

    if (!isLikelyRaspberryPi()) {
        panel.style.display = 'none';
        return;
    }

    panel.style.display = 'block';
    statusText.textContent = 'Checking...';

    const portValue = parseInt(document.getElementById('piPort')?.value || '8080', 10);
    const result = await probeLocalSt3215Server(isNaN(portValue) ? 8080 : portValue);
    statusText.textContent = result.message;
    statusText.style.color = result.ok ? '#27ae60' : '#e67e22';

    // If the server returned network information, populate the extra fields.
    try {
        const hostnameSpan = document.getElementById('localPiHostname');
        const ipSpan = document.getElementById('localPiIp');
        const macSpan = document.getElementById('localPiMac');
        const gwSpan = document.getElementById('localPiGateway');

        if (hostnameSpan) {
            hostnameSpan.textContent = (result.info && result.info.hostname) ? result.info.hostname : '-';
        }

        if (ipSpan || macSpan) {
            let ipText = '-';
            let macText = '-';
            if (result.info && Array.isArray(result.info.interfaces) && result.info.interfaces.length > 0) {
                // Prefer eth0, then wlan0, then first interface.
                let chosen = null;
                for (let i = 0; i < result.info.interfaces.length; i++) {
                    const iface = result.info.interfaces[i];
                    if (iface && iface.name === 'eth0') {
                        chosen = iface;
                        break;
                    }
                }
                if (!chosen) {
                    for (let i = 0; i < result.info.interfaces.length; i++) {
                        const iface = result.info.interfaces[i];
                        if (iface && iface.name === 'wlan0') {
                            chosen = iface;
                            break;
                        }
                    }
                }
                if (!chosen) {
                    chosen = result.info.interfaces[0];
                }

                if (chosen && chosen.address) {
                    ipText = chosen.address;
                }
                if (chosen && chosen.mac) {
                    macText = chosen.mac;
                }
            }
            if (ipSpan) ipSpan.textContent = ipText;
            if (macSpan) macSpan.textContent = macText;
        }

        if (gwSpan) {
            gwSpan.textContent = (result.info && result.info.gateway) ? result.info.gateway : '-';
        }
    } catch (e) {
        console.warn('checkLocalPiServer: failed to update network info fields:', e);
    }
}

/**
 * Convenience button for when the app is running on the Pi:
 * Set the connection address to localhost (127.0.0.1) and connect.
 */
function useLocalPiConnectionSettings() {
    const addrInput = document.getElementById('piAddress');
    if (addrInput) {
        addrInput.value = '127.0.0.1';
    }
    const connectButton = document.getElementById('connectButton');
    if (connectButton) {
        connectButton.click();
    }
}

/**
 * Settings tab helper: update the Electron app's own git repo on the Pi.
 * Calls into the main process which runs "git pull --ff-only" in the electron-app folder.
 */
async function updateElectronAppFromGit() {
    const statusSpan = document.getElementById('localPiUpdateStatus');
    if (statusSpan) {
        statusSpan.textContent = 'Updating Electron app from git...';
        statusSpan.style.color = '#e67e22';
    }

    if (!window.electronAPI || typeof window.electronAPI.updateFromGit !== 'function') {
        if (statusSpan) {
            statusSpan.textContent = 'Electron update API is not available in this build.';
            statusSpan.style.color = '#e74c3c';
        }
        return;
    }

    const result = await window.electronAPI.updateFromGit('electron');
    if (statusSpan) {
        statusSpan.textContent = result.message || (result.ok ? 'Update finished.' : 'Update failed.');
        statusSpan.style.color = result.ok ? '#27ae60' : '#e74c3c';
    }
}

/**
 * Settings tab helper: ask the ST3215 Pi server to update itself from git.
 * Sends a WebSocket command "updatePiServerFromGit" and displays the result.
 */
async function updateSt3215FromGit() {
    const statusSpan = document.getElementById('localPiUpdateStatus');
    if (statusSpan) {
        statusSpan.textContent = 'Updating ST3215 server from git...';
        statusSpan.style.color = '#e67e22';
    }

    try {
        if (!robotArmClient || typeof robotArmClient.sendRawCommand !== 'function') {
            if (statusSpan) {
                statusSpan.textContent = 'ST3215 client does not support raw commands in this version.';
                statusSpan.style.color = '#e74c3c';
            }
            return;
        }

        // Wrap the raw command/send in a small promise so we can await it here.
        const message = await new Promise((resolve) => {
            try {
                robotArmClient.sendRawCommand(
                    { command: 'updatePiServerFromGit' },
                    function onMessage(data) {
                        // Expect an "updateResult" response
                        if (data && data.type === 'updateResult') {
                            resolve(
                                (data.ok ? 'ST3215 update OK: ' : 'ST3215 update failed: ') +
                                (data.message || '')
                            );
                            return true; // tell client this message was handled
                        }
                        return false;
                    },
                    15000 // 15s timeout
                );
            } catch (e) {
                resolve('Failed to send update command to ST3215 server: ' + (e.message || e));
            }
        });

        if (statusSpan) {
            const ok = message && message.indexOf('failed') === -1;
            statusSpan.textContent = message;
            statusSpan.style.color = ok ? '#27ae60' : '#e74c3c';
        }
    } catch (e) {
        if (statusSpan) {
            statusSpan.textContent = 'Error updating ST3215 server: ' + (e.message || e);
            statusSpan.style.color = '#e74c3c';
        }
    }
}

/**
 * Saves settings to localStorage
 */
function saveSettings() {
    const settings = {
        numJoints: parseInt(document.getElementById('numJoints').value),
        updateInterval: parseInt(document.getElementById('updateInterval').value),
        defaultStepSize: parseFloat(document.getElementById('defaultStepSize').value),
        piAddress: document.getElementById('piAddress').value,
        piPort: parseInt(document.getElementById('piPort').value)
    };
    
    localStorage.setItem('robotArmSettings', JSON.stringify(settings));
    
    // Update step size if changed
    document.getElementById('stepSize').value = settings.defaultStepSize;
    
    // Update joint UI if number of joints changed
    updateJointUI();
    
    // Restart status updates with new interval
    if (robotArmClient.isConnected) {
        stopStatusUpdates();
        startStatusUpdates();
    }
    
    showAppMessage('Settings saved!');
}

/**
 * Loads settings from localStorage
 */
function loadSettings() {
    const saved = localStorage.getItem('robotArmSettings');
    
    if (saved) {
        try {
            const settings = JSON.parse(saved);
            
            if (settings.numJoints) {
                document.getElementById('numJoints').value = settings.numJoints;
            }
            if (settings.updateInterval) {
                document.getElementById('updateInterval').value = settings.updateInterval;
            }
            if (settings.defaultStepSize) {
                document.getElementById('defaultStepSize').value = settings.defaultStepSize;
                document.getElementById('stepSize').value = settings.defaultStepSize;
            }
            if (settings.piAddress) {
                document.getElementById('piAddress').value = settings.piAddress;
            }
            if (settings.piPort) {
                document.getElementById('piPort').value = settings.piPort;
            }
        } catch (error) {
            console.error('Error loading settings:', error);
        }
    }
}

// ===== Kinematics Functions =====

/**
 * Called when joint configurations are loaded from the robot
 * @param {Array} configs - Array of joint configurations
 */
function onJointConfigsLoaded(configs) {
    console.log('Joint configurations loaded:', configs);
    
    // Check if this is ST3215 version (array of joint info objects) or I2C version (kinematics data)
    if (configs.length > 0 && configs[0].jointNumber !== undefined) {
        // ST3215 version - just basic servo info
        const servoCount = configs.length;
        console.log(`ST3215: ${servoCount} servos discovered`);
        
        // Update the number of joints setting based on discovered servos
        const numJointsInput = document.getElementById('numJoints');
        if (numJointsInput && servoCount > 0) {
            numJointsInput.value = servoCount;
            // Trigger change event to update UI
            numJointsInput.dispatchEvent(new Event('change'));
        }
        
        // Update kinematics status
        document.getElementById('kinematicsStatus').textContent = `✓ ${servoCount} servos discovered`;
        document.getElementById('kinematicsStatus').style.color = '#27ae60';
    } else {
        // I2C version with kinematics data
    // Update the display
    displayJointConfigs(configs);
    
    // Apply to kinematics module if all joints are configured
    if (configs.every(c => c !== null)) {
        robotKinematics.setJointConfigurations(configs);
        document.getElementById('kinematicsStatus').textContent = '✓ Kinematics configured';
        document.getElementById('kinematicsStatus').style.color = '#27ae60';
        
        // Update 3D visualization with new configurations
        if (robotArm3D) {
            const currentAngles = robotArm3D.jointAngles || [];
            robotArm3D.update(configs, currentAngles);
            }
        }
    }
}

/**
 * Loads joint configurations from the robot
 */
function loadJointConfigs() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    document.getElementById('kinematicsStatus').textContent = 'Loading...';
    robotArmClient.requestJointConfigs();
}

/**
 * Default URDF for the 5-axis robot arm
 */
const DEFAULT_URDF = `<?xml version="1.0"?>
<robot name="five_dof_arm">

  <!-- ================= LINKS ================= -->

  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <link name="link4"/>
  <link name="link5"/>
  <link name="tool_link"/>

  <!-- ================= JOINT 1 ================= -->
  <!-- Base Yaw -->

  <joint name="joint1_base_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.122" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
  </joint>

  <!-- ================= JOINT 2 ================= -->
  <!-- Shoulder Pitch (set zero_offset_degrees in demo-kinematics.urdf; fallback has none) -->

  <joint name="joint2_shoulder_pitch" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <!-- ================= JOINT 3 ================= -->
  <!-- Elbow Pitch -->

  <joint name="joint3_elbow_pitch" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0.16178 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="10" velocity="2"/>
  </joint>

  <!-- ================= JOINT 4 ================= -->
  <!-- Wrist Roll (inline with forearm) -->

  <joint name="joint4_wrist_roll" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0.14820 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="5" velocity="3"/>
  </joint>

  <!-- ================= JOINT 5 ================= -->
  <!-- Wrist Pitch -->

  <joint name="joint5_wrist_pitch" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="5" velocity="3"/>
  </joint>

  <!-- ================= TOOL (Fixed) ================= -->

  <joint name="tool_fixed" type="fixed">
    <parent link="link5"/>
    <child link="tool_link"/>
    <origin xyz="0.030 0 0.04189" rpy="0 0 0"/>
  </joint>

</robot>`;

/**
 * Applies loaded URDF text to kinematics and updates the UI (used by loadDemoConfig).
 * @param {string} urdfText - Full URDF XML string
 * @param {string} source - Description for logging (e.g. 'file' or 'DEFAULT_URDF')
 */
function applyLoadedUrdf(urdfText, source) {
    if (!urdfText || !urdfText.trim()) {
        return;
    }
    const zeroOffsetMatch = urdfText.match(/zero_offset_degrees="(-?\d+)/);
    const zeroOffsetValue = zeroOffsetMatch ? zeroOffsetMatch[1] : 'none';
    if (source !== 'DEFAULT_URDF') {
        console.log('Loaded URDF from', source, '(zero_offset in file:', zeroOffsetValue, ')');
    }
    robotKinematics.loadURDF(urdfText);
    if (!robotKinematics.urdfData || !robotKinematics.urdfData.joints) {
        throw new Error('URDF data not properly loaded');
    }
    const allJoints = robotKinematics.urdfData.joints;
    const revoluteJoints = robotKinematics.getJointConfigs();
    robotArmClient.jointConfigs = revoluteJoints.map(joint => ({
        name: joint.name,
        type: joint.type,
        origin: joint.origin,
        axis: joint.axis
    }));
    displayJointConfigs(allJoints);
    document.getElementById('kinematicsStatus').textContent = '✓ URDF demo configuration loaded';
    document.getElementById('kinematicsStatus').style.color = '#27ae60';

    // Now that kinematics are configured, regenerate joint UI so the
    // per-joint angle inputs use the real URDF limits (lowerDegrees / upperDegrees)
    // instead of the default -90..90 range.
    if (typeof updateJointUI === 'function') {
        updateJointUI();
    }

    generateSimulatedAngleControls();
    if (robotArm3D) {
        const numRevoluteJoints = robotKinematics.getJointCount();
        const currentAngles = Array(numRevoluteJoints).fill(0);
        try {
            robotArm3D.update(allJoints, currentAngles);
            console.log('Updated 3D visualization with URDF config');
        } catch (error) {
            console.error('Error updating 3D visualization with URDF config:', error);
        }
        simulatedAngles = [...currentAngles];
        for (let i = 1; i <= getNumJoints(); i++) {
            const simInput = document.getElementById('simJoint' + i);
            if (simInput) {
                simInput.value = currentAngles[i - 1] || 0;
            }
        }
        try {
            const pose = robotKinematics.forwardKinematics(currentAngles);
            const pos = pose.position;
            document.getElementById('endEffectorPos').textContent =
                'X: ' + pos.x.toFixed(1) + 'mm, Y: ' + pos.y.toFixed(1) + 'mm, Z: ' + pos.z.toFixed(1) + 'mm';
        } catch (error) {
            document.getElementById('endEffectorPos').textContent = 'Error calculating position';
            console.error('Error calculating end effector position:', error);
        }
    } else {
        console.warn('robotArm3D not initialized yet. Visualization will update when initialized.');
    }
    console.log('URDF demo configuration loaded:', allJoints);
}

/**
 * Loads a demo URDF configuration for testing.
 * Tries: 1) electronAPI.readTextFile (preload), 2) fetch (same folder as page), 3) DEFAULT_URDF.
 */
function loadDemoConfig() {
    const urdfFilename = 'demo-kinematics.urdf';
    let urdfText = null;

    // 1) Try preload file read (Electron)
    if (window.electronAPI && typeof window.electronAPI.readTextFile === 'function') {
        urdfText = window.electronAPI.readTextFile(urdfFilename);
    }

    if (urdfText && urdfText.trim()) {
        try {
            applyLoadedUrdf(urdfText, urdfFilename);
            return;
        } catch (error) {
            console.error('Error applying URDF from preload:', error);
        }
    }

    // 2) Try fetch (relative to the loaded page; works when page is file:// from app folder)
    fetch(urdfFilename)
        .then(function (response) {
            if (!response.ok) {
                throw new Error('HTTP ' + response.status);
            }
            return response.text();
        })
        .then(function (text) {
            if (text && text.trim()) {
                console.log('Loaded URDF via fetch from', urdfFilename);
                applyLoadedUrdf(text, urdfFilename);
            } else {
                useDefaultUrdf();
            }
        })
        .catch(function (err) {
            console.warn('Could not load demo-kinematics.urdf (fetch failed: ' + (err.message || err) + '), falling back to built-in DEFAULT_URDF.');
            useDefaultUrdf();
        });
    return;

    function useDefaultUrdf() {
        try {
            applyLoadedUrdf(DEFAULT_URDF, 'DEFAULT_URDF');
        } catch (error) {
            console.error('Error loading URDF:', error);
            document.getElementById('kinematicsStatus').textContent = '✗ Error loading URDF: ' + error.message;
            document.getElementById('kinematicsStatus').style.color = '#e74c3c';
        }
    }
}

/**
 * Displays joint configurations in the UI
 * Supports both URDF joint structures and legacy DH parameters
 * @param {Array} configs - Array of joint configurations (URDF joints or DH parameters)
 */
function displayJointConfigs(configs) {
    const displayDiv = document.getElementById('kinematicsConfigDisplay');
    
    if (!configs || configs.length === 0) {
        displayDiv.innerHTML = '<p class="info-text">No configurations loaded.</p>';
        return;
    }
    
    let html = '<div class="kinematics-grid">';
    
    for (let i = 0; i < configs.length; i++) {
        const config = configs[i];
        
        if (config === null) {
            html += `
                <div class="joint-kinematics-card">
                    <h4>Joint ${i + 1}</h4>
                    <p class="error-text">Not available</p>
                </div>
            `;
        } else {
            // Check if this is a URDF joint (has origin and axis) or legacy DH (has d, a, theta, alpha)
            const isURDF = config.origin !== undefined && config.axis !== undefined;
            
            if (isURDF) {
                // Display URDF joint information
                const origin = config.origin || { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
                const axis = config.axis || { x: 0, y: 0, z: 1 };
                const jointName = config.name || `Joint ${i + 1}`;
                const jointType = config.type || 'revolute';
                
                html += `
                    <div class="joint-kinematics-card">
                        <h4>${jointName}</h4>
                        <div class="kinematics-params">
                            <div class="param-row">
                                <label>Type:</label>
                                <span>${jointType}</span>
                            </div>
                            <div class="param-row">
                                <label>Origin X:</label>
                                <span>${(origin.x * 1000).toFixed(2)} mm</span>
                            </div>
                            <div class="param-row">
                                <label>Origin Y:</label>
                                <span>${(origin.y * 1000).toFixed(2)} mm</span>
                            </div>
                            <div class="param-row">
                                <label>Origin Z:</label>
                                <span>${(origin.z * 1000).toFixed(2)} mm</span>
                            </div>
                            <div class="param-row">
                                <label>Axis:</label>
                                <span>(${axis.x.toFixed(2)}, ${axis.y.toFixed(2)}, ${axis.z.toFixed(2)})</span>
                            </div>
                        </div>
                    </div>
                `;
            } else {
                // Display legacy DH parameters
            html += `
                <div class="joint-kinematics-card">
                    <h4>Joint ${i + 1}</h4>
                    <div class="kinematics-params">
                        <div class="param-row">
                            <label>d (offset):</label>
                                <input type="number" id="joint${i}_d" value="${(config.d || 0).toFixed(2)}" step="0.01">
                            <span>mm</span>
                        </div>
                        <div class="param-row">
                            <label>a (length):</label>
                                <input type="number" id="joint${i}_a" value="${(config.a || 0).toFixed(2)}" step="0.01">
                            <span>mm</span>
                        </div>
                        <div class="param-row">
                            <label>θ (angle):</label>
                                <input type="number" id="joint${i}_theta" value="${(config.theta || 0).toFixed(2)}" step="0.01">
                            <span>°</span>
                        </div>
                        <div class="param-row">
                            <label>α (twist):</label>
                                <input type="number" id="joint${i}_alpha" value="${(config.alpha || 0).toFixed(2)}" step="0.01">
                            <span>°</span>
                        </div>
                        <button class="btn btn-small" onclick="saveJointKinematics(${i + 1})">Save to Robot</button>
                    </div>
                </div>
            `;
            }
        }
    }
    
    html += '</div>';
    displayDiv.innerHTML = html;
}

/**
 * Applies loaded configurations to the kinematics module
 */
function applyKinematicsConfig() {
    const configs = robotArmClient.getJointConfigs();
    
    if (!configs || configs.length === 0) {
        showAppMessage('No joint configurations loaded. Click "Load from Robot" or "Load Demo Configuration" first.');
        return;
    }
    
    // Filter out null configurations
    const validConfigs = configs.filter(c => c !== null);
    
    if (validConfigs.length === 0) {
        showAppMessage('No valid joint configurations found.');
        return;
    }
    
    robotKinematics.setJointConfigurations(validConfigs);
    document.getElementById('kinematicsStatus').textContent = `✓ ${validConfigs.length} joints configured`;
    document.getElementById('kinematicsStatus').style.color = '#27ae60';
    
    // Regenerate simulated angle controls to match the number of joints
    generateSimulatedAngleControls();
    
    // Update 3D visualization if available
    if (robotArm3D) {
        const currentAngles = robotArm3D.jointAngles || [];
        robotArm3D.update(validConfigs, currentAngles);
        
        // Calculate and display end effector position if we have angles
        if (currentAngles.length > 0) {
            try {
                const pose = robotKinematics.forwardKinematics(currentAngles);
                const pos = pose.position;
                document.getElementById('endEffectorPos').textContent = 
                    `X: ${pos.x.toFixed(1)}mm, Y: ${pos.y.toFixed(1)}mm, Z: ${pos.z.toFixed(1)}mm`;
            } catch (error) {
                // Ignore errors if angles don't match config count
            }
        }
    }
}

/**
 * Saves kinematics parameters for a joint to the robot
 * @param {number} jointNumber - Joint number (1-based)
 */
async function saveJointKinematics(jointNumber) {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    const jointIndex = jointNumber - 1;
    
    // Read values from input fields
    const d = parseFloat(document.getElementById(`joint${jointIndex}_d`).value);
    const a = parseFloat(document.getElementById(`joint${jointIndex}_a`).value);
    const theta = parseFloat(document.getElementById(`joint${jointIndex}_theta`).value);
    const alpha = parseFloat(document.getElementById(`joint${jointIndex}_alpha`).value);
    
    try {
        // Save each parameter
        await robotArmClient.setKinematics(jointNumber, 'd', d, 'mm');
        await new Promise(resolve => setTimeout(resolve, 100));
        await robotArmClient.setKinematics(jointNumber, 'a', a, 'mm');
        await new Promise(resolve => setTimeout(resolve, 100));
        await robotArmClient.setKinematics(jointNumber, 'theta', theta, 'degrees');
        await new Promise(resolve => setTimeout(resolve, 100));
        await robotArmClient.setKinematics(jointNumber, 'alpha', alpha, 'degrees');
        
        showAppMessage(`Joint ${jointNumber} kinematics saved successfully!`);
    } catch (error) {
        showAppMessage(`Error saving kinematics: ${error.message}`);
    }
}

/**
 * Tests forward kinematics
 */
function testForwardKinematics() {
    if (!robotKinematics.isConfigured()) {
        showAppMessage('Kinematics not configured. Load joint configurations first.');
        return;
    }
    
    // Get joint angles from input
    const anglesText = document.getElementById('fkAngles').value;
    const angles = anglesText.split(',').map(a => parseFloat(a.trim()));
    
    if (angles.length !== robotKinematics.getJointCount()) {
        showAppMessage(`Please enter ${robotKinematics.getJointCount()} angles (comma-separated)`);
        return;
    }
    
    try {
        const result = robotKinematics.forwardKinematics(angles);
        const pos = result.position;
        
        document.getElementById('fkResult').innerHTML = `
            <strong>End Effector Position:</strong><br>
            X: ${pos.x.toFixed(2)} mm<br>
            Y: ${pos.y.toFixed(2)} mm<br>
            Z: ${pos.z.toFixed(2)} mm
        `;
    } catch (error) {
        document.getElementById('fkResult').innerHTML = `<span class="error-text">Error: ${error.message}</span>`;
    }
}

/**
 * Tests inverse kinematics
 */
function testInverseKinematics() {
    if (!robotKinematics.isConfigured()) {
        showAppMessage('Kinematics not configured. Load joint configurations first.');
        return;
    }
    
    // Get target position from inputs
    const x = parseFloat(document.getElementById('ikX').value);
    const y = parseFloat(document.getElementById('ikY').value);
    const z = parseFloat(document.getElementById('ikZ').value);
    
    if (isNaN(x) || isNaN(y) || isNaN(z)) {
        showAppMessage('Please enter valid X, Y, Z coordinates');
        return;
    }
    
    try {
        // First solve for position only
        const baseAngles = robotKinematics.inverseKinematics({
            x: x,
            y: y,
            z: z
        });

        const refined = baseAngles
            ? robotKinematics.refineOrientationWithAccuracy({ x: x, y: y, z: z }, baseAngles, currentToolOrientation)
            : null;
        const angles = refined ? refined.angles : null;

        if (angles === null) {
            document.getElementById('ikResult').innerHTML = `<span class="error-text">Target position is unreachable</span>`;
        } else {
            const anglesText = angles.map((a, i) => `Joint ${i + 1}: ${a.toFixed(2)}°`).join('<br>');
            const accLine = refined && refined.achievedPosition
                ? `<br><strong>Achieved position:</strong> X: ${refined.achievedPosition.x.toFixed(2)} mm, Y: ${refined.achievedPosition.y.toFixed(2)} mm, Z: ${refined.achievedPosition.z.toFixed(2)} mm<br><strong>Accuracy:</strong> position ${refined.positionErrorMm.toFixed(2)} mm, orientation ${refined.orientationErrorDeg.toFixed(1)}°`
                : '';
            document.getElementById('ikResult').innerHTML = `
                <strong>Joint Angles:</strong><br>
                ${anglesText}
                ${accLine}
            `;
        }
    } catch (error) {
        document.getElementById('ikResult').innerHTML = `<span class="error-text">Error: ${error.message}</span>`;
    }
}

// ===== 3D Visualization Functions =====

/**
 * Initializes the 3D visualization
 */
function initialize3DVisualization() {
    // Wait a bit for Three.js to load and container to be visible
    setTimeout(() => {
        const container = document.getElementById('robotArm3DContainer');
        if (!container) {
            console.error('3D container element not found');
            document.getElementById('visualizationStatus').textContent = 'Error: Container not found';
            return;
        }

        if (typeof THREE === 'undefined') {
            console.error('Three.js library not loaded');
            document.getElementById('visualizationStatus').textContent = 'Error: Three.js not loaded';
            return;
        }

        try {
            robotArm3D = new RobotArm3D('robotArm3DContainer');
            document.getElementById('visualizationStatus').textContent = 'Initialized';
            console.log('3D visualization initialized');
            
            // Try to update with demo config if available
            const configs = robotKinematics.jointConfigs || [];
            if (configs.length > 0) {
                const numJoints = getNumJoints();
                const angles = simulatedAngles.length === numJoints ? simulatedAngles : Array(numJoints).fill(0);
                robotArm3D.update(configs, angles);
                console.log('Updated visualization with existing configs');
            }

            // Apply current dead zones and stored positions if available
            if (typeof robotArm3D.updateDeadZones === 'function') {
                robotArm3D.updateDeadZones(deadZones);
            }
            if (typeof update3DStoredPositionsIfAvailable === 'function') {
                update3DStoredPositionsIfAvailable();
            }

            // Hook up visibility checkboxes
            const dzCheckbox = document.getElementById('showDeadZonesCheckbox');
            if (dzCheckbox && typeof robotArm3D.setDeadZonesVisible === 'function') {
                robotArm3D.setDeadZonesVisible(dzCheckbox.checked);
                dzCheckbox.addEventListener('change', function () {
                    robotArm3D.setDeadZonesVisible(this.checked);
                });
            }
            const posCheckbox = document.getElementById('showStoredPositionsCheckbox');
            if (posCheckbox && typeof robotArm3D.setStoredPositionsVisible === 'function') {
                robotArm3D.setStoredPositionsVisible(posCheckbox.checked);
                posCheckbox.addEventListener('change', function () {
                    robotArm3D.setStoredPositionsVisible(this.checked);
                });
            }
            const envCheckbox = document.getElementById('showEnvelopeCheckbox');
            if (envCheckbox && typeof robotArm3D.setEnvelopeVisible === 'function') {
                robotArm3D.setEnvelopeVisible(envCheckbox.checked);
                envCheckbox.addEventListener('change', function () {
                    robotArm3D.setEnvelopeVisible(this.checked);
                });
            }

            // End effector movement trace visibility
            const eeTraceCheckbox = document.getElementById('showEndEffectorTraceCheckbox');
            if (eeTraceCheckbox && typeof robotArm3D.setEndEffectorTraceVisible === 'function') {
                robotArm3D.setEndEffectorTraceVisible(eeTraceCheckbox.checked);
                endEffectorTraceEnabled = eeTraceCheckbox.checked;
                eeTraceCheckbox.addEventListener('change', function () {
                    endEffectorTraceEnabled = this.checked;
                    robotArm3D.setEndEffectorTraceVisible(this.checked);
                    if (!this.checked && typeof robotArm3D.clearMovementTraces === 'function') {
                        // Clear end effector trace when disabling (joint traces may also be cleared)
                        robotArm3D.clearMovementTraces();
                        endEffectorTracePoints = [];
                    }
                });
            }

            // Joint movement traces visibility
            const jointTracesCheckbox = document.getElementById('showJointTracesCheckbox');
            if (jointTracesCheckbox && typeof robotArm3D.setJointTracesVisible === 'function') {
                robotArm3D.setJointTracesVisible(jointTracesCheckbox.checked);
                jointTracesEnabled = jointTracesCheckbox.checked;
                jointTracesCheckbox.addEventListener('change', function () {
                    jointTracesEnabled = this.checked;
                    robotArm3D.setJointTracesVisible(this.checked);
                    if (!this.checked && typeof robotArm3D.clearMovementTraces === 'function') {
                        // Clear joint traces when disabling (end effector trace may also be cleared)
                        robotArm3D.clearMovementTraces();
                        jointTracesPoints = [];
                    }
                });
            }
        } catch (error) {
            console.error('Failed to initialize 3D visualization:', error);
            document.getElementById('visualizationStatus').textContent = 'Error: ' + error.message;
        }
    }, 1000); // Increased delay to ensure container is visible
}

/**
 * Updates the 3D visualization with current joint angles
 * @param {Array} jointAngles - Array of joint angles in degrees
 */
function update3DVisualizationWithAngles(jointAngles) {
    if (!robotArm3D) {
        return;
    }

    // Get joint configurations from kinematics module.
    // Prefer full URDF joint list (including fixed tool joint) for visualisation,
    // fall back to revolute-only configs if needed.
    let configs = [];
    if (robotKinematics && robotKinematics.urdfData && Array.isArray(robotKinematics.urdfData.joints)) {
        configs = robotKinematics.urdfData.joints;
    } else {
        configs = robotKinematics.getJointConfigs() || [];
    }
    
    if (!configs || configs.length === 0) {
        return; // No configurations loaded yet
    }

    try {
        // Ensure we have the correct number of angles (pad with zeros if needed)
        const expectedJointCount = robotKinematics.isConfigured() ? robotKinematics.getJointCount() : configs.length;
        // When simulation mode is on, use simulated angles so 3D and End Effector Position match the sliders
        const anglesToUse = (useSimulatedAngles && simulatedAngles.length >= expectedJointCount)
            ? [...simulatedAngles]
            : [...jointAngles];
        const adjustedAngles = anglesToUse;
        while (adjustedAngles.length < expectedJointCount) {
            adjustedAngles.push(0);
        }
        // Trim if too many
        while (adjustedAngles.length > expectedJointCount) {
            adjustedAngles.pop();
        }
        
        robotArm3D.update(configs, adjustedAngles, false); // No animation for real robot updates
        
        // Calculate and display end effector position
        // Only calculate if we have the correct number of angles
        if (robotKinematics.isConfigured() && adjustedAngles.length === robotKinematics.getJointCount()) {
            try {
                const pose = robotKinematics.forwardKinematics(adjustedAngles);
                const pos = pose.position;
                const endEffectorElement = document.getElementById('endEffectorPos');
                if (endEffectorElement) {
                    endEffectorElement.textContent = 
                        `X: ${pos.x.toFixed(1)}mm, Y: ${pos.y.toFixed(1)}mm, Z: ${pos.z.toFixed(1)}mm`;
                }
            } catch (error) {
                const endEffectorElement = document.getElementById('endEffectorPos');
                if (endEffectorElement) {
                    endEffectorElement.textContent = 'Error calculating position';
                }
                console.error('Error calculating end effector position:', error);
            }
        } else {
            const endEffectorElement = document.getElementById('endEffectorPos');
            if (endEffectorElement) {
                endEffectorElement.textContent = 'No configuration or angles';
            }
        }

        // Also update the kinematics matrices display (if the user has that panel open)
        if (typeof updateKinematicsMatrices === 'function') {
            updateKinematicsMatrices(adjustedAngles);
        }
    } catch (error) {
        console.error('Error updating 3D visualization:', error);
    }
}

/**
 * Updates the 3D visualization manually
 */
function update3DVisualization() {
    if (!robotArm3D) {
        showAppMessage('3D visualization not initialized. Please refresh the page.');
        return;
    }

    // Get joint configurations from URDF
    const configs = robotKinematics.getJointConfigs() || [];
    
    if (configs.length === 0) {
        showAppMessage('No joint configurations loaded. Please load configurations from the Settings tab first.');
        return;
    }

    // If simulation mode is enabled, use simulated angles
    if (useSimulatedAngles) {
        updateSimulatedVisualization();
        return;
    }

    // Get current joint angles from status displays
    // Use URDF joint count if available, otherwise fall back to getNumJoints()
    let numJoints = robotKinematics.isConfigured() ? robotKinematics.getJointCount() : getNumJoints();
    const jointAngles = [];
    
    for (let i = 1; i <= numJoints; i++) {
        const angleElement = document.getElementById(`joint${i}Angle`);
        if (angleElement) {
            const angle = parseFloat(angleElement.textContent) || 0;
            jointAngles.push(angle);
        } else {
            jointAngles.push(0);
        }
    }

    // Update visualization
    robotArm3D.update(configs, jointAngles);
    document.getElementById('visualizationStatus').textContent = 'Updated';
    
    // Calculate and display end effector position
    // Only calculate if we have the correct number of angles
    if (robotKinematics.isConfigured() && jointAngles.length === robotKinematics.getJointCount()) {
        try {
            const pose = robotKinematics.forwardKinematics(jointAngles);
            const pos = pose.position;
            document.getElementById('endEffectorPos').textContent = 
                `X: ${pos.x.toFixed(1)}mm, Y: ${pos.y.toFixed(1)}mm, Z: ${pos.z.toFixed(1)}mm`;
        } catch (error) {
            document.getElementById('endEffectorPos').textContent = 'Error calculating position';
            console.error('Error calculating end effector position:', error);
        }
    } else {
        document.getElementById('endEffectorPos').textContent = 'No configuration loaded';
    }
}

/**
 * Resets the 3D camera to default position
 */
function reset3DCamera() {
    if (robotArm3D) {
        robotArm3D.resetCamera();
    }
}

/**
 * Sets the 3D camera to a specific view
 * @param {string} view - 'front', 'side', 'top', or 'iso'
 */
function set3DView(view) {
    if (!robotArm3D) {
        return;
    }

    switch (view) {
        case 'front':
            // Front view: look along -Z, centered roughly halfway up the arm
            robotArm3D.setCameraPosition(0, 250, 700, 250);
            break;
        case 'side':
            // Side view: look along +X, centered roughly halfway up the arm
            robotArm3D.setCameraPosition(700, 250, 0, 250);
            break;
        case 'top':
            // Top view: look straight down
            robotArm3D.setCameraPosition(0, 700, 0, 0);
            break;
        case 'iso':
            // Isometric view: default angled view
            robotArm3D.setCameraPosition(400, 350, 400, 150);
            break;
        default:
            robotArm3D.resetCamera();
    }
}

/**
 * Debug function to check 3D visualization status
 */
function debug3DVisualization() {
    const info = {
        robotArm3D: robotArm3D !== null,
        scene: robotArm3D ? robotArm3D.scene !== null : false,
        camera: robotArm3D ? robotArm3D.camera !== null : false,
        renderer: robotArm3D ? robotArm3D.renderer !== null : false,
        robotArm: robotArm3D ? robotArm3D.robotArm !== null : false,
        container: document.getElementById('robotArm3DContainer') !== null,
        containerSize: {
            width: document.getElementById('robotArm3DContainer')?.clientWidth || 0,
            height: document.getElementById('robotArm3DContainer')?.clientHeight || 0
        },
        jointConfigs: robotKinematics.jointConfigs?.length || 0,
        threeJSLoaded: typeof THREE !== 'undefined',
        jointAngles: robotArm3D ? robotArm3D.jointAngles : [],
        useSimulated: useSimulatedAngles
    };
    
    console.log('3D Visualization Debug Info:', info);
    
    let message = 'Debug Info:\n';
    message += `robotArm3D initialized: ${info.robotArm3D}\n`;
    message += `Scene: ${info.scene}\n`;
    message += `Camera: ${info.camera}\n`;
    message += `Renderer: ${info.renderer}\n`;
    message += `Robot Arm Group: ${info.robotArm}\n`;
    message += `Container exists: ${info.container}\n`;
    message += `Container size: ${info.containerSize.width}x${info.containerSize.height}\n`;
    message += `Joint configs loaded: ${info.jointConfigs}\n`;
    message += `Three.js loaded: ${info.threeJSLoaded}\n`;
    message += `Joint angles: ${info.jointAngles.join(', ')}\n`;
    message += `Using simulated: ${info.useSimulated}`;
    
    showAppMessage(message, 6000);
    
    // Try to force update if everything looks good
    if (info.robotArm3D && info.jointConfigs > 0) {
        console.log('Attempting to force update...');
        const configs = robotKinematics.jointConfigs || [];
        const numJoints = getNumJoints();
        const angles = useSimulatedAngles && simulatedAngles.length === numJoints 
            ? simulatedAngles 
            : Array(numJoints).fill(0);
        if (robotArm3D) {
            robotArm3D.update(configs, angles);
            document.getElementById('visualizationStatus').textContent = 'Force updated';
        }
    }
}

// ===== Simulation Mode Functions =====

/**
 * Toggles between real robot angles and simulated angles
 */
function toggleSimulationMode() {
    const checkbox = document.getElementById('useSimulatedAngles');
    useSimulatedAngles = checkbox.checked;
    
    const panel = document.getElementById('simulatedAnglesPanel');
    const modeText = document.getElementById('simulationModeText');
    const modeDisplay = document.getElementById('visualizationMode');
    
    if (useSimulatedAngles) {
        // Show simulation controls
        panel.style.display = 'block';
        modeText.textContent = 'Simulation mode enabled. Visualization uses your manual angle settings.';
        if (modeDisplay) {
            modeDisplay.textContent = 'Simulated';
        }
        
        // Update visualization with simulated angles
        updateSimulatedVisualization();
    } else {
        // Hide simulation controls
        panel.style.display = 'none';
        modeText.textContent = 'Currently showing real robot joint angles. Enable simulation to manually set angles.';
        if (modeDisplay) {
            modeDisplay.textContent = 'Real Robot';
        }
        
        // Update visualization with real angles
        const realAngles = [];
        const numJoints = getNumJoints();
        for (let i = 1; i <= numJoints; i++) {
            const angleElement = document.getElementById(`joint${i}Angle`);
            if (angleElement) {
                const angle = parseFloat(angleElement.textContent) || 0;
                realAngles.push(angle);
            } else {
                realAngles.push(0);
            }
        }
        update3DVisualizationWithAngles(realAngles);
    }
}

/**
 * Updates the visualization with simulated angles
 */
function updateSimulatedVisualization() {
    if (!robotArm3D) {
        return;
    }

    // Get joint configurations
    const configs = robotKinematics.jointConfigs || [];
    
    if (configs.length === 0) {
        showAppMessage('No joint configurations loaded. Please load configurations from the Settings tab first.');
        return;
    }

    // Get simulated angles from input fields
    // For 5-axis robot: collect angles for 5 joints, then add 0 for tool offset
    simulatedAngles = [];
    
    // Determine how many joint angle controls exist (should be 5 for 5-axis robot)
    let numJointControls = 0;
    for (let i = 1; i <= 10; i++) { // Check up to 10 joints
        const inputElement = document.getElementById(`simJoint${i}`);
        if (inputElement) {
            numJointControls = i;
        } else {
            break;
        }
    }
    
    // Collect angles from all available input fields
    for (let i = 1; i <= numJointControls; i++) {
        const inputElement = document.getElementById(`simJoint${i}`);
        if (inputElement) {
            const angle = parseFloat(inputElement.value) || 0;
            simulatedAngles.push(angle);
        } else {
            simulatedAngles.push(0);
        }
    }
    
    // If we have 6 configs (5 joints + tool), ensure we have 6 angles (tool angle is always 0)
    // If we have 5 configs, use 5 angles
    // Otherwise, pad or trim to match config count
    while (simulatedAngles.length < configs.length) {
        // Add 0 for tool offset or missing joints
        simulatedAngles.push(0);
    }
    while (simulatedAngles.length > configs.length) {
        // Remove extra angles if we have too many
        simulatedAngles.pop();
    }

    // Debug: log collected angles
    console.log(`Collected ${simulatedAngles.length} angles for ${configs.length} configs:`, simulatedAngles);

    // Update visualization with animation enabled for smooth transitions
    try {
        robotArm3D.update(configs, simulatedAngles, true);
        
        // Calculate and display end effector position
        // Forward kinematics should work with all angles (tool angle is 0, so it's just an offset)
        if (robotKinematics.isConfigured() && simulatedAngles.length > 0) {
            try {
                const pose = robotKinematics.forwardKinematics(simulatedAngles);
                const pos = pose.position;
                document.getElementById('endEffectorPos').textContent = 
                    `X: ${pos.x.toFixed(1)}mm, Y: ${pos.y.toFixed(1)}mm, Z: ${pos.z.toFixed(1)}mm`;
            } catch (error) {
                document.getElementById('endEffectorPos').textContent = 'Error calculating position';
                console.error('Error calculating end effector position:', error);
            }
        }
        
        document.getElementById('visualizationStatus').textContent = 'Updated (Simulated)';
    } catch (error) {
        console.error('Error updating simulated visualization:', error);
    }
}

/**
 * Resets all simulated angles to 0
 */
function resetSimulatedAngles() {
    const numJoints = getNumJoints();
    for (let i = 1; i <= numJoints; i++) {
        const inputElement = document.getElementById(`simJoint${i}`);
        if (inputElement) {
            inputElement.value = 0;
        }
    }
    simulatedAngles = Array(numJoints).fill(0);
    updateSimulatedVisualization();
}

/**
 * Copies real robot angles to simulated angles
 */
function copyRealAnglesToSim() {
    const numJoints = getNumJoints();
    for (let i = 1; i <= numJoints; i++) {
        const realAngleElement = document.getElementById(`joint${i}Angle`);
        const simAngleElement = document.getElementById(`simJoint${i}`);
        
        if (realAngleElement && simAngleElement) {
            const realAngle = parseFloat(realAngleElement.textContent) || 0;
            simAngleElement.value = realAngle.toFixed(1);
        }
    }
    
    // Update simulated angles array
    simulatedAngles = [];
    for (let i = 1; i <= numJoints; i++) {
        const simAngleElement = document.getElementById(`simJoint${i}`);
        if (simAngleElement) {
            simulatedAngles.push(parseFloat(simAngleElement.value) || 0);
        } else {
            simulatedAngles.push(0);
        }
    }
    
    updateSimulatedVisualization();
}

// ===== AS5600 Calibration Functions =====

let calibrationInProgress = false;
let calibrationStopRequested = false;

/**
 * Stops the calibration process
 */
function stopCalibration() {
    calibrationStopRequested = true;
    if (robotArmClient.isConnected) {
        const jointNumber = parseInt(document.getElementById('calibrationJoint').value);
        robotArmClient.stopJoint(jointNumber);
    }
    document.getElementById('calibrationStatus').textContent = 'Calibration stopped by user';
    document.getElementById('startCalibrationButton').disabled = false;
    document.getElementById('stopCalibrationButton').style.display = 'none';
    calibrationInProgress = false;
}

/**
 * Starts the AS5600 sensor calibration routine for a joint
 */
async function startCalibration() {
    if (!robotArmClient.isConnected) {
        showAppMessage('Not connected to Raspberry Pi');
        return;
    }
    
    if (calibrationInProgress) {
        showAppMessage('Calibration already in progress');
        return;
    }
    
    const jointNumber = parseInt(document.getElementById('calibrationJoint').value);
    if (isNaN(jointNumber) || jointNumber < 1) {
        showAppMessage('Please enter a valid joint number');
        return;
    }
    
    calibrationInProgress = true;
    calibrationStopRequested = false;
    
    // Update UI
    document.getElementById('startCalibrationButton').disabled = true;
    document.getElementById('stopCalibrationButton').style.display = 'inline-block';
    document.getElementById('calibrationStatus').textContent = 'Starting calibration...';
    document.getElementById('calibrationProgress').textContent = '';
    
    try {
        // Step 1: Move forward until hitting end stop
        document.getElementById('calibrationStatus').textContent = 'Step 1: Moving forward to find end stop...';
        const forwardEndStopAngle = await findEndStop(jointNumber, 1); // 1 = forward direction
        
        if (calibrationStopRequested) {
            return;
        }
        
        // Step 2: Move backward until hitting other end stop
        document.getElementById('calibrationStatus').textContent = 'Step 2: Moving backward to find end stop...';
        const backwardEndStopAngle = await findEndStop(jointNumber, -1); // -1 = backward direction
        
        if (calibrationStopRequested) {
            return;
        }
        
        // Step 3: Calculate middle position
        const middleAngle = (forwardEndStopAngle + backwardEndStopAngle) / 2;
        document.getElementById('calibrationStatus').textContent = 
            `Step 3: Moving to middle position (${middleAngle.toFixed(2)}°)...`;
        document.getElementById('calibrationProgress').textContent = 
            `Forward end: ${forwardEndStopAngle.toFixed(2)}°, Backward end: ${backwardEndStopAngle.toFixed(2)}°, Middle: ${middleAngle.toFixed(2)}°`;
        
        // Move to middle position
        robotArmClient.moveJoint(jointNumber, middleAngle);
        
        // Wait for movement to complete (check status until not moving)
        await waitForJointToStop(jointNumber);
        
        if (calibrationStopRequested) {
            return;
        }
        
        // Step 4: Get current AS5600 angle and set offset to make it 0
        const statuses = await getCurrentJointStatus(jointNumber);
        if (!statuses || statuses.length === 0) {
            throw new Error('Failed to get joint status');
        }
        
        const currentAngle = statuses[0].angleDegrees;
        const offsetNeeded = -currentAngle; // Negative to make current angle become 0
        
        document.getElementById('calibrationStatus').textContent = 
            `Step 4: Setting sensor offset to ${offsetNeeded.toFixed(2)}°...`;
        
        // Set the sensor offset (this calls I2C command 0x05)
        robotArmClient.setSensorOffset(jointNumber, offsetNeeded);
        
        // Wait a bit for the command to complete
        await new Promise(resolve => setTimeout(resolve, 500));
        
        // Verify by reading status again
        const newStatuses = await getCurrentJointStatus(jointNumber);
        if (newStatuses && newStatuses.length > 0) {
            const newAngle = newStatuses[0].angleDegrees;
            document.getElementById('calibrationStatus').textContent = 
                `Calibration complete! Sensor angle is now ${newAngle.toFixed(2)}° (should be close to 0°)`;
            document.getElementById('calibrationProgress').textContent = 
                `Offset set to ${offsetNeeded.toFixed(2)}°. Forward end: ${forwardEndStopAngle.toFixed(2)}°, Backward end: ${backwardEndStopAngle.toFixed(2)}°`;
        } else {
            document.getElementById('calibrationStatus').textContent = 'Calibration complete (unable to verify)';
        }
        
    } catch (error) {
        console.error('Calibration error:', error);
        document.getElementById('calibrationStatus').textContent = `Error: ${error.message}`;
        showAppMessage(`Calibration failed: ${error.message}`);
    } finally {
        calibrationInProgress = false;
        document.getElementById('startCalibrationButton').disabled = false;
        document.getElementById('stopCalibrationButton').style.display = 'none';
    }
}

/**
 * Finds an end stop by moving in a direction until the AS5600 angle stops changing
 * @param {number} jointNumber - Joint number (1-based)
 * @param {number} direction - 1 for forward, -1 for backward
 * @returns {Promise<number>} The AS5600 angle when end stop is reached
 */
async function findEndStop(jointNumber, direction) {
    const STABLE_READINGS_REQUIRED = 5; // Number of consecutive stable readings needed
    const STABLE_THRESHOLD = 0.5; // Degrees - angle must change less than this to be considered stable
    const MOVE_STEP = direction > 0 ? 5 : -5; // Degrees to move each step
    const MAX_MOVEMENT = 270; // Maximum degrees to move before giving up
    
    let stableCount = 0;
    let lastAngle = null;
    let totalMovement = 0;
    let previousAngle = null;
    
    // Get initial angle
    const initialStatus = await getCurrentJointStatus(jointNumber);
    if (!initialStatus || initialStatus.length === 0) {
        throw new Error('Failed to get initial joint status');
    }
    previousAngle = initialStatus[0].angleDegrees;
    lastAngle = previousAngle;
    
    while (stableCount < STABLE_READINGS_REQUIRED && totalMovement < MAX_MOVEMENT && !calibrationStopRequested) {
        // Move the joint
        const currentStatus = await getCurrentJointStatus(jointNumber);
        if (!currentStatus || currentStatus.length === 0) {
            throw new Error('Failed to get joint status during calibration');
        }
        
        const currentAngle = currentStatus[0].angleDegrees;
        const targetAngle = currentAngle + MOVE_STEP;
        
        robotArmClient.moveJoint(jointNumber, targetAngle);
        
        // Wait a bit for movement to start
        await new Promise(resolve => setTimeout(resolve, 200));
        
        // Monitor angle changes
        let readingsWithoutChange = 0;
        const maxReadings = 20; // Maximum readings to check
        
        for (let i = 0; i < maxReadings && !calibrationStopRequested; i++) {
            await new Promise(resolve => setTimeout(resolve, 300)); // Wait 300ms between readings
            
            const status = await getCurrentJointStatus(jointNumber);
            if (!status || status.length === 0) {
                continue;
            }
            
            const newAngle = status[0].angleDegrees;
            const angleChange = Math.abs(newAngle - lastAngle);
            
            if (angleChange < STABLE_THRESHOLD) {
                readingsWithoutChange++;
                if (readingsWithoutChange >= STABLE_READINGS_REQUIRED) {
                    // Angle has been stable - we've hit the end stop
                    stableCount = STABLE_READINGS_REQUIRED;
                    lastAngle = newAngle;
                    break;
                }
            } else {
                // Angle is still changing, reset counter
                readingsWithoutChange = 0;
                lastAngle = newAngle;
            }
        }
        
        totalMovement += Math.abs(MOVE_STEP);
        
        // Update progress
        document.getElementById('calibrationProgress').textContent = 
            `Current angle: ${lastAngle.toFixed(2)}°, Stable readings: ${readingsWithoutChange}/${STABLE_READINGS_REQUIRED}`;
    }
    
    if (calibrationStopRequested) {
        throw new Error('Calibration stopped by user');
    }
    
    if (totalMovement >= MAX_MOVEMENT) {
        throw new Error('Reached maximum movement without finding end stop');
    }
    
    return lastAngle;
}

/**
 * Waits for a joint to stop moving
 * @param {number} jointNumber - Joint number (1-based)
 */
async function waitForJointToStop(jointNumber) {
    const maxWaitTime = 30000; // 30 seconds max
    const checkInterval = 500; // Check every 500ms
    const startTime = Date.now();
    
    while (Date.now() - startTime < maxWaitTime && !calibrationStopRequested) {
        const status = await getCurrentJointStatus(jointNumber);
        if (status && status.length > 0 && !status[0].isMoving) {
            return; // Joint has stopped
        }
        await new Promise(resolve => setTimeout(resolve, checkInterval));
    }
    
    if (calibrationStopRequested) {
        throw new Error('Calibration stopped by user');
    }
}

/**
 * Gets the current status of a specific joint
 * @param {number} jointNumber - Joint number (1-based)
 * @returns {Promise<Array>} Array with single joint status object
 */
async function getCurrentJointStatus(jointNumber) {
    return new Promise((resolve, reject) => {
        // Request status
        robotArmClient.requestStatus();
        
        // Wait for status update
        const originalCallback = robotArmClient.onStatusUpdate;
        let timeoutId = null;
        
        robotArmClient.onStatusUpdate = function(joints) {
            // Restore original callback
            robotArmClient.onStatusUpdate = originalCallback;
            
            if (timeoutId) {
                clearTimeout(timeoutId);
            }
            
            // Find the requested joint
            const joint = joints.find(j => j.joint === jointNumber);
            if (joint) {
                resolve([joint]);
            } else {
                reject(new Error(`Joint ${jointNumber} not found in status update`));
            }
        };
        
        // Timeout after 5 seconds
        timeoutId = setTimeout(() => {
            robotArmClient.onStatusUpdate = originalCallback;
            reject(new Error('Timeout waiting for joint status'));
        }, 5000);
    });
}

// ===== Auto-Detection Functions =====

/**
 * Automatically detects the Raspberry Pi on the network
 * Tries multiple methods: mDNS hostname, common IP ranges
 * @returns {Promise<string|null>} IP address if found, null otherwise
 */
async function autoDetectRaspberryPi() {
    const port = parseInt(document.getElementById('piPort').value) || 8080;
    const timeout = 2000; // 2 seconds timeout per attempt
    
    // Method 1: Try mDNS hostname (raspberrypi.local)
    const mDNSHostnames = [
        'raspberrypi.local',
        'robot-arm.local',
        'robotarm.local'
    ];
    
    for (const hostname of mDNSHostnames) {
        try {
            if (await testConnection(hostname, port, timeout)) {
                return hostname;
            }
        } catch (error) {
            // Continue to next method
        }
    }
    
    // Method 2: Try common IP ranges
    const commonRanges = [
        { base: '192.168.1', start: 1, end: 254 },
        { base: '192.168.0', start: 1, end: 254 },
        { base: '192.168.2', start: 1, end: 254 },
        { base: '10.0.0', start: 1, end: 254 }
    ];
    
    // Update status
    const statusDiv = document.getElementById('autoDetectStatus');
    
    for (const range of commonRanges) {
        statusDiv.textContent = `Scanning ${range.base}.x...`;
        
        // Test a few common addresses first (faster)
        const commonIPs = [100, 101, 102, 103, 104, 105];
        for (const lastOctet of commonIPs) {
            const ip = `${range.base}.${lastOctet}`;
            try {
                if (await testConnection(ip, port, timeout)) {
                    return ip;
                }
            } catch (error) {
                // Continue
            }
        }
        
        // If common IPs didn't work, scan the range (but limit to avoid taking too long)
        const scanLimit = 20; // Only scan first 20 addresses to keep it fast
        for (let i = range.start; i <= Math.min(range.end, range.start + scanLimit); i++) {
            // Skip already tested common IPs
            if (commonIPs.includes(i)) continue;
            
            const ip = `${range.base}.${i}`;
            try {
                if (await testConnection(ip, port, timeout)) {
                    return ip;
                }
            } catch (error) {
                // Continue
            }
        }
    }
    
    return null;
}

/**
 * Tests if a connection can be made to the given address and port
 * @param {string} address - IP address or hostname
 * @param {number} port - Port number
 * @param {number} timeout - Timeout in milliseconds
 * @returns {Promise<boolean>} True if connection successful
 */
function testConnection(address, port, timeout) {
    return new Promise((resolve) => {
        const ws = new WebSocket(`ws://${address}:${port}`);
        let resolved = false;
        
        const timer = setTimeout(() => {
            if (!resolved) {
                resolved = true;
                try {
                    ws.close();
                } catch (e) {
                    // Ignore
                }
                resolve(false);
            }
        }, timeout);
        
        ws.onopen = () => {
            if (!resolved) {
                resolved = true;
                clearTimeout(timer);
                ws.close();
                resolve(true);
            }
        };
        
        ws.onerror = () => {
            if (!resolved) {
                resolved = true;
                clearTimeout(timer);
                resolve(false);
            }
        };
    });
}

