/**
 * Stored Positions Manager
 * Manages preset positions (0-99) with labels for the robot arm
 */

// Storage key for positions
const POSITIONS_STORAGE_KEY = 'robotArmPositions';

/**
 * Initialize positions management
 */
function initializePositions() {
    // Generate joint angle inputs based on number of joints
    updatePositionJointsGrid();
    
    // Refresh the positions list
    refreshPositionsList();
    
    // Update joints grid when number of joints changes
    const numJointsInput = document.getElementById('numJoints');
    if (numJointsInput) {
        numJointsInput.addEventListener('change', updatePositionJointsGrid);
    }
}

/**
 * Updates the grid of joint angle inputs for position editing
 */
function updatePositionJointsGrid() {
    const grid = document.getElementById('positionJointsGrid');
    if (!grid) return;
    
    const numJoints = getNumJoints();
    grid.innerHTML = '';
    
    for (let i = 1; i <= numJoints; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'position-joint-item';
        jointDiv.innerHTML = `
            <label>Joint ${i}:</label>
            <input type="number" id="positionJoint${i}" value="0" step="0.1" style="width: 100px;">
            <span>°</span>
        `;
        grid.appendChild(jointDiv);
    }
}

/**
 * Gets all joint angles from the position editor
 */
function getPositionAngles() {
    const numJoints = getNumJoints();
    const angles = [];
    
    for (let i = 1; i <= numJoints; i++) {
        const input = document.getElementById(`positionJoint${i}`);
        if (input) {
            angles.push(parseFloat(input.value) || 0);
        } else {
            angles.push(0);
        }
    }
    
    return angles;
}

/**
 * Sets all joint angles in the position editor
 */
function setPositionAngles(angles) {
    for (let i = 0; i < angles.length; i++) {
        const input = document.getElementById(`positionJoint${i + 1}`);
        if (input) {
            input.value = angles[i].toFixed(1);
        }
    }
}

/**
 * Gets all stored positions from localStorage
 */
function getAllPositions() {
    const stored = localStorage.getItem(POSITIONS_STORAGE_KEY);
    if (!stored) {
        return {};
    }
    
    try {
        return JSON.parse(stored);
    } catch (error) {
        console.error('Error loading positions:', error);
        return {};
    }
}

/**
 * Saves all positions to localStorage
 */
function saveAllPositions(positions) {
    try {
        localStorage.setItem(POSITIONS_STORAGE_KEY, JSON.stringify(positions));
        return true;
    } catch (error) {
        console.error('Error saving positions:', error);
        return false;
    }
}

/**
 * Gets a specific position by number
 */
function getPosition(number) {
    const positions = getAllPositions();
    return positions[number] || null;
}

/**
 * Gets a specific position by name/label (case-insensitive)
 * @param {string} name - Position label/name to search for
 * @returns {Object|null} Position object with number and data, or null if not found
 */
function getPositionByName(name) {
    if (!name || typeof name !== 'string') {
        return null;
    }
    
    const positions = getAllPositions();
    const searchName = name.trim().toLowerCase();
    
    // Search through all positions
    const keys = Object.keys(positions);
    for (let i = 0; i < keys.length; i++) {
        const num = keys[i];
        const pos = positions[num];
        if (pos && pos.label) {
            const posLabel = pos.label.toLowerCase();
            if (posLabel === searchName) {
                return {
                    number: parseInt(num),
                    position: pos
                };
            }
        }
    }
    
    return null;
}

/**
 * Saves the current position editor values to a position number
 */
function savePosition() {
    const positionNumber = parseInt(document.getElementById('positionNumber').value);
    const label = document.getElementById('positionLabel').value.trim();
    const angles = getPositionAngles();
    
    if (isNaN(positionNumber) || positionNumber < 0 || positionNumber > 99) {
        updatePositionsStatus('Error: Position number must be between 0 and 99', 'error');
        return;
    }
    
    const positions = getAllPositions();
    positions[positionNumber] = {
        label: label || `Position ${positionNumber}`,
        angles: angles,
        timestamp: new Date().toISOString()
    };
    
    if (saveAllPositions(positions)) {
        updatePositionsStatus(`Position ${positionNumber} saved: "${positions[positionNumber].label}"`, 'success');
        refreshPositionsList();
        if (typeof update3DStoredPositionsIfAvailable === 'function') {
            update3DStoredPositionsIfAvailable();
        }
        // Update Blockly blocks if available
        if (typeof updateBlocklyPositionBlocks === 'function') {
            setTimeout(updateBlocklyPositionBlocks, 100);
        }

        // After saving, auto-advance to the next free position number
        let nextNumber = null;
        // First, look from the current number upwards
        for (let n = positionNumber + 1; n <= 99; n++) {
            if (!positions.hasOwnProperty(n)) {
                nextNumber = n;
                break;
            }
        }
        // If none found above, look from 0 upwards
        if (nextNumber === null) {
            for (let n = 0; n <= 99; n++) {
                if (!positions.hasOwnProperty(n)) {
                    nextNumber = n;
                    break;
                }
            }
        }
        // If we found a free slot, update the UI to point at it
        if (nextNumber !== null) {
            const numberInput = document.getElementById('positionNumber');
            if (numberInput) {
                numberInput.value = nextNumber;
            }
            // Clear label so the user can type a new one
            const labelInput = document.getElementById('positionLabel');
            if (labelInput) {
                labelInput.value = '';
            }
        }
    } else {
        updatePositionsStatus('Error: Failed to save position', 'error');
    }
}

/**
 * Loads a position into the editor
 */
function loadPosition() {
    const positionNumber = parseInt(document.getElementById('positionNumber').value);
    
    if (isNaN(positionNumber) || positionNumber < 0 || positionNumber > 99) {
        updatePositionsStatus('Error: Position number must be between 0 and 99', 'error');
        return;
    }
    
    const position = getPosition(positionNumber);
    if (!position) {
        updatePositionsStatus(`Position ${positionNumber} not found`, 'error');
        return;
    }
    
    // Load into editor
    document.getElementById('positionLabel').value = position.label || '';
    setPositionAngles(position.angles || []);
    
    updatePositionsStatus(`Position ${positionNumber} loaded: "${position.label}"`, 'success');
}

/**
 * Deletes a position
 */
function deletePosition() {
    const positionNumber = parseInt(document.getElementById('positionNumber').value);
    
    if (isNaN(positionNumber) || positionNumber < 0 || positionNumber > 99) {
        updatePositionsStatus('Error: Position number must be between 0 and 99', 'error');
        return;
    }
    
    if (!confirm(`Delete position ${positionNumber}?`)) {
        return;
    }
    
    const positions = getAllPositions();
    if (positions[positionNumber]) {
        delete positions[positionNumber];
        if (saveAllPositions(positions)) {
            updatePositionsStatus(`Position ${positionNumber} deleted`, 'success');
            refreshPositionsList();
            // Clear editor
            document.getElementById('positionLabel').value = '';
            setPositionAngles(Array(getNumJoints()).fill(0));
            if (typeof update3DStoredPositionsIfAvailable === 'function') {
                update3DStoredPositionsIfAvailable();
            }
        } else {
            updatePositionsStatus('Error: Failed to delete position', 'error');
        }
    } else {
        updatePositionsStatus(`Position ${positionNumber} not found`, 'error');
    }
}

/**
 * Refreshes the positions list display
 */
function refreshPositionsList() {
    const listDiv = document.getElementById('positionsList');
    if (!listDiv) return;
    
    const positions = getAllPositions();
    const positionNumbers = Object.keys(positions).map(Number).sort((a, b) => a - b);
    
    if (positionNumbers.length === 0) {
        listDiv.innerHTML = '<p style="color: #666; font-style: italic;">No positions saved yet</p>';
        return;
    }
    
    let html = '<div class="positions-list-items">';
    positionNumbers.forEach(num => {
        const pos = positions[num];
        html += `
            <div class="positions-list-item" onclick="selectPosition(${num})">
                <div class="position-item-number">${num}</div>
                <div class="position-item-label">${pos.label || `Position ${num}`}</div>
                <div class="position-item-angles">${pos.angles ? pos.angles.map((a, i) => `J${i+1}:${a.toFixed(1)}°`).join(', ') : 'No angles'}</div>
                <button class="btn btn-small" onclick="event.stopPropagation(); loadPositionToEditor(${num})">Load</button>
                <button class="btn btn-small btn-danger" onclick="event.stopPropagation(); deletePositionByNumber(${num})">Delete</button>
            </div>
        `;
    });
    html += '</div>';
    
    listDiv.innerHTML = html;

    // Update 3D visualization markers if available
    if (typeof update3DStoredPositionsIfAvailable === 'function') {
        update3DStoredPositionsIfAvailable();
    }

    // Also update the pendant stored-position dropdown if it exists
    const pendantSelect = document.getElementById('pendantPositionSelect');
    if (pendantSelect) {
        pendantSelect.innerHTML = '';
        const defaultOption = document.createElement('option');
        defaultOption.value = '';
        defaultOption.textContent = 'Select position';
        pendantSelect.appendChild(defaultOption);

        positionNumbers.forEach(num => {
            const pos = positions[num];
            const option = document.createElement('option');
            option.value = String(num);
            option.textContent = `${num} - ${pos.label || `Position ${num}`}`;
            pendantSelect.appendChild(option);
        });
    }
}

/**
 * Selects a position (loads it into the editor)
 */
function selectPosition(number) {
    document.getElementById('positionNumber').value = number;
    loadPosition();
}

/**
 * Loads a position into the editor by number
 */
function loadPositionToEditor(number) {
    document.getElementById('positionNumber').value = number;
    loadPosition();
}

/**
 * Deletes a position by number
 */
function deletePositionByNumber(number) {
    if (!confirm(`Delete position ${number}?`)) {
        return;
    }
    
    const positions = getAllPositions();
    if (positions[number]) {
        delete positions[number];
        if (saveAllPositions(positions)) {
            updatePositionsStatus(`Position ${number} deleted`, 'success');
            refreshPositionsList();
            if (typeof update3DStoredPositionsIfAvailable === 'function') {
                update3DStoredPositionsIfAvailable();
            }
        } else {
            updatePositionsStatus('Error: Failed to delete position', 'error');
        }
    }
}

/**
 * Updates the positions status message
 */
function updatePositionsStatus(message, type = 'info') {
    const statusDiv = document.getElementById('positionsStatus');
    if (!statusDiv) return;
    
    statusDiv.textContent = message;
    statusDiv.className = `positions-status-${type}`;
    
    // Reset to default after 3 seconds
    setTimeout(() => {
        statusDiv.textContent = 'Ready';
        statusDiv.className = '';
    }, 3000);
}

/**
 * Exports all positions to a JSON file
 */
function exportPositions() {
    const positions = getAllPositions();
    const exportData = {
        version: '1.0',
        timestamp: new Date().toISOString(),
        positions: positions
    };
    
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'robot-arm-positions.json';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    
    updatePositionsStatus('Positions exported', 'success');
}

/**
 * Imports positions from a JSON file
 */
function importPositions() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = function(event) {
        const file = event.target.files[0];
        if (!file) {
            return;
        }
        
        const reader = new FileReader();
        reader.onload = function(e) {
            try {
                const importData = JSON.parse(e.target.result);
                const positions = importData.positions || importData;
                
                // Merge with existing positions (user can choose to overwrite)
                const existing = getAllPositions();
                const merged = { ...existing, ...positions };
                
                if (saveAllPositions(merged)) {
                    updatePositionsStatus('Positions imported successfully', 'success');
                    refreshPositionsList();
                    if (typeof update3DStoredPositionsIfAvailable === 'function') {
                        update3DStoredPositionsIfAvailable();
                    }
                } else {
                    updatePositionsStatus('Error: Failed to import positions', 'error');
                }
            } catch (error) {
                console.error('Error importing positions:', error);
                updatePositionsStatus('Error: Invalid file format', 'error');
            }
        };
        reader.readAsText(file);
    };
    input.click();
}

/**
 * Gets position data for Blockly (returns array of {number, label} objects)
 */
function getPositionsForBlockly() {
    const positions = getAllPositions();
    const result = [];
    
    for (let num = 0; num <= 99; num++) {
        if (positions[num]) {
            result.push([positions[num].label || `Position ${num}`, num.toString()]);
        }
    }
    
    return result;
}

/**
 * Loads current robot joint angles into the position editor
 */
async function loadCurrentRobotAngles() {
    if (!robotArmClient || !robotArmClient.isConnected) {
        updatePositionsStatus('Error: Not connected to robot', 'error');
        return;
    }
    
    try {
        updatePositionsStatus('Loading current robot angles...', 'info');
        const numJoints = getNumJoints();
        const angles = [];
        
        // Get current angles for all joints in a single status request
        const statusArray = await robotArmClient.getStatus();
        for (let i = 0; i < numJoints; i++) {
            const s = statusArray[i];
            if (s && typeof s.angleDegrees === 'number') {
                angles.push(s.angleDegrees);
            } else {
                angles.push(0);
            }
        }
        
        // Set angles in editor
        setPositionAngles(angles);
        updatePositionsStatus('Current robot angles loaded', 'success');
    } catch (error) {
        console.error('Error loading robot angles:', error);
        updatePositionsStatus('Error loading robot angles: ' + error.message, 'error');
    }
}

