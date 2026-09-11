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

    // XYZ inputs are authoritative when position type is 'xyz' — recompute the
    // angle preview whenever they change.
    ['positionX', 'positionY', 'positionZ'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.addEventListener('input', updatePositionEditorPreview);
    });

    updatePositionTypeUI();
}

/**
 * Which type the position editor is currently set to save as.
 * @returns {'angles'|'xyz'}
 */
function getSelectedPositionType() {
    const checked = document.querySelector('input[name="positionType"]:checked');
    return (checked && checked.value === 'xyz') ? 'xyz' : 'angles';
}

/**
 * Sets the editor's angles/XYZ radio selection and updates the UI to match.
 * @param {'angles'|'xyz'} type
 */
function setPositionType(type) {
    const radio = document.querySelector(`input[name="positionType"][value="${type === 'xyz' ? 'xyz' : 'angles'}"]`);
    if (radio) radio.checked = true;
    updatePositionTypeUI();
}

/**
 * Enables the authoritative field group for the selected type (angles or XYZ)
 * and turns the other group into a read-only, tool-aware preview, then
 * recomputes that preview.
 */
function updatePositionTypeUI() {
    const type = getSelectedPositionType();
    const numJoints = getNumJoints();
    const anglesAuthoritative = type === 'angles';

    for (let i = 1; i <= numJoints; i++) {
        const input = document.getElementById(`positionJoint${i}`);
        if (input) input.disabled = !anglesAuthoritative;
    }
    ['positionX', 'positionY', 'positionZ'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = anglesAuthoritative;
    });

    const previewNote = document.getElementById('positionPreviewNote');
    if (previewNote) {
        previewNote.textContent = anglesAuthoritative
            ? 'XYZ shown below is a live preview (forward kinematics) — not saved.'
            : 'Joint angles shown below are a live preview (inverse kinematics for the CURRENT tool) — not saved. A different tool will resolve this XYZ to different angles at move time.';
    }

    updatePositionEditorPreview();
}

/**
 * Recomputes whichever field group is currently a read-only preview
 * (XYZ via FK when angles are authoritative, or angles via IK when XYZ is
 * authoritative) from the other, authoritative group.
 */
function updatePositionEditorPreview() {
    if (typeof robotKinematics === 'undefined' || !robotKinematics.isConfigured()) return;

    if (getSelectedPositionType() === 'angles') {
        updateXYZFromAngles();
    } else {
        const x = parseFloat(document.getElementById('positionX').value);
        const y = parseFloat(document.getElementById('positionY').value);
        const z = parseFloat(document.getElementById('positionZ').value);
        if (isNaN(x) || isNaN(y) || isNaN(z)) return;
        try {
            const seed = getPositionAngles();
            const angles = robotKinematics.inverseKinematics({ x, y, z }, seed);
            if (angles) {
                setPositionAngles(angles);
                updatePositionsXYZStatus(`Preview OK`, 'success');
            } else {
                updatePositionsXYZStatus(`Position (${x}, ${y}, ${z}) mm is unreachable with the current tool`, 'error');
            }
        } catch (e) {
            updatePositionsXYZStatus('IK preview error: ' + e.message, 'error');
        }
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
            <input type="number" id="positionJoint${i}" value="0" step="0.1" style="width: 100px;" oninput="updatePositionEditorPreview()">
            <span>°</span>
        `;
        grid.appendChild(jointDiv);
    }

    // These inputs are rebuilt whenever the joint count changes, so the
    // steppers have to be re-applied to the new ones, and the disabled state
    // has to be reapplied since fresh elements default to enabled.
    if (document.body.classList.contains('touch-mode') &&
        typeof enhanceNumberInputs === 'function') {
        enhanceNumberInputs();
    }
    if (typeof updatePositionTypeUI === 'function') {
        updatePositionTypeUI();
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
 * Sets the XYZ display fields
 */
function setPositionXYZ(xyz) {
    const xInput = document.getElementById('positionX');
    const yInput = document.getElementById('positionY');
    const zInput = document.getElementById('positionZ');
    if (xInput) xInput.value = xyz.x.toFixed(1);
    if (yInput) yInput.value = xyz.y.toFixed(1);
    if (zInput) zInput.value = xyz.z.toFixed(1);
}

/**
 * Runs FK on the current joint angle inputs and updates the XYZ fields
 */
function updateXYZFromAngles() {
    if (typeof robotKinematics === 'undefined' || !robotKinematics.isConfigured()) {
        updatePositionsXYZStatus('Kinematics not configured', 'error');
        return;
    }
    const angles = getPositionAngles();
    try {
        const fk = robotKinematics.forwardKinematics(angles);
        if (fk && fk.position) {
            setPositionXYZ(fk.position);
            updatePositionsXYZStatus(`FK: (${fk.position.x.toFixed(1)}, ${fk.position.y.toFixed(1)}, ${fk.position.z.toFixed(1)}) mm`, 'success');
        }
    } catch (e) {
        updatePositionsXYZStatus('FK error: ' + e.message, 'error');
    }
}

/**
 * Updates the XYZ section status message
 */
function updatePositionsXYZStatus(message, type = 'info') {
    const el = document.getElementById('positionsXYZStatus');
    if (!el) return;
    el.textContent = message;
    el.className = `positions-xyz-status positions-xyz-status-${type}`;
    setTimeout(() => { el.textContent = ''; el.className = 'positions-xyz-status'; }, 4000);
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
    const type = getSelectedPositionType();
    // Both field groups are read at save time: the authoritative one is what
    // gets saved as the position's real data, the other is just the live
    // preview (cached alongside for the list/3D view — not used to move).
    const angles = getPositionAngles();
    const xInput = parseFloat(document.getElementById('positionX').value);
    const yInput = parseFloat(document.getElementById('positionY').value);
    const zInput = parseFloat(document.getElementById('positionZ').value);

    if (isNaN(positionNumber) || positionNumber < 0 || positionNumber > 99) {
        updatePositionsStatus('Error: Position number must be between 0 and 99', 'error');
        return;
    }
    if (type === 'xyz' && (isNaN(xInput) || isNaN(yInput) || isNaN(zInput))) {
        updatePositionsStatus('Error: Enter valid X, Y, Z values', 'error');
        return;
    }

    let xyz = (!isNaN(xInput) && !isNaN(yInput) && !isNaN(zInput)) ? { x: xInput, y: yInput, z: zInput } : null;
    if (type === 'angles' && typeof robotKinematics !== 'undefined' && robotKinematics.isConfigured()) {
        try {
            const fk = robotKinematics.forwardKinematics(angles);
            if (fk && fk.position) {
                xyz = { x: fk.position.x, y: fk.position.y, z: fk.position.z };
            }
        } catch (e) {}
    }

    const positions = getAllPositions();
    positions[positionNumber] = {
        label: label || `Position ${positionNumber}`,
        type: type,
        angles: angles,   // authoritative for 'angles', a cached IK preview for 'xyz'
        xyz: xyz,          // authoritative for 'xyz', a cached FK preview for 'angles'
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
    
    // Load into editor. Missing type = legacy position saved before this field
    // existed — treat as 'angles', matching its actual data shape.
    document.getElementById('positionLabel').value = position.label || '';
    setPositionType(position.type === 'xyz' ? 'xyz' : 'angles');
    setPositionAngles(position.angles || []);
    if (position.xyz) setPositionXYZ(position.xyz);
    updatePositionTypeUI(); // (re)computes whichever field group is the read-only preview

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

    const positions = getAllPositions();
    if (!positions[positionNumber]) {
        updatePositionsStatus(`Position ${positionNumber} not found`, 'error');
        return;
    }

    const label = positions[positionNumber].label;
    const name = label ? `${positionNumber} "${label}"` : `${positionNumber}`;

    showConfirm(`Delete position ${name}?`, { confirmLabel: 'Delete', danger: true })
        .then(confirmed => {
            if (!confirmed) {
                return;
            }

            delete positions[positionNumber];

            if (!saveAllPositions(positions)) {
                updatePositionsStatus('Error: Failed to delete position', 'error');
                return;
            }

            updatePositionsStatus(`Position ${positionNumber} deleted`, 'success');
            refreshPositionsList();

            // Clear editor
            document.getElementById('positionLabel').value = '';
            setPositionAngles(Array(getNumJoints()).fill(0));

            if (typeof update3DStoredPositionsIfAvailable === 'function') {
                update3DStoredPositionsIfAvailable();
            }
        });
}

/**
 * Escapes a label before it goes into innerHTML.
 * @param {string} text - Raw label
 * @returns {string} HTML-safe label
 */
function escapePositionText(text) {
    if (typeof escapeHtml === 'function') {
        return escapeHtml(text);
    }
    return String(text === null || text === undefined ? '' : text)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
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
        const type = pos.type === 'xyz' ? 'xyz' : 'angles';
        const anglesStr = pos.angles ? pos.angles.map((a, i) => `J${i+1}:${a.toFixed(1)}°`).join(', ') : 'No angles';
        const xyzStr = pos.xyz
            ? `X:${pos.xyz.x.toFixed(1)} Y:${pos.xyz.y.toFixed(1)} Z:${pos.xyz.z.toFixed(1)} mm`
            : '';
        // Whichever field isn't authoritative for this position's type is shown
        // as a "(preview)" hint — it's a cached snapshot, not what's actually used to move.
        const primaryStr = type === 'xyz' ? xyzStr : anglesStr;
        const previewStr = type === 'xyz'
            ? (anglesStr ? `${anglesStr} (preview)` : '')
            : (xyzStr ? `${xyzStr} (preview)` : '');
        const typeBadge = type === 'xyz' ? 'XYZ' : 'Angles';
        const safeLabel = escapePositionText(pos.label || `Position ${num}`);
        html += `
            <div class="positions-list-item">
                <button type="button" class="position-item-select" onclick="selectPosition(${num})"
                        aria-label="Select position ${num}">
                    <span class="position-item-number">${num}</span>
                    <span class="position-item-info">
                        <span class="position-item-label">${safeLabel} <span class="position-item-type-badge">${typeBadge}</span></span>
                        <span class="position-item-angles">${primaryStr}</span>
                        ${previewStr ? `<span class="position-item-xyz">${previewStr}</span>` : ''}
                    </span>
                </button>
                <div class="position-item-actions">
                    <button class="btn btn-secondary" onclick="loadPositionToEditor(${num})">Load</button>
                    <button class="btn btn-danger" onclick="deletePositionByNumber(${num})">Delete</button>
                </div>
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
    const positions = getAllPositions();
    if (!positions[number]) {
        return;
    }

    const label = positions[number].label;
    const name = label ? `${number} "${label}"` : `${number}`;

    showConfirm(`Delete position ${name}?`, { confirmLabel: 'Delete', danger: true })
        .then(confirmed => {
            if (!confirmed) {
                return;
            }

            delete positions[number];

            if (!saveAllPositions(positions)) {
                updatePositionsStatus('Error: Failed to delete position', 'error');
                return;
            }

            updatePositionsStatus(`Position ${number} deleted`, 'success');
            refreshPositionsList();

            if (typeof update3DStoredPositionsIfAvailable === 'function') {
                update3DStoredPositionsIfAvailable();
            }
        });
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
        
        // Set angles in editor — capturing live angles implies saving as angles-type.
        setPositionType('angles');
        setPositionAngles(angles);
        updatePositionTypeUI();
        updatePositionsStatus('Current robot angles loaded', 'success');
    } catch (error) {
        console.error('Error loading robot angles:', error);
        updatePositionsStatus('Error loading robot angles: ' + error.message, 'error');
    }
}

