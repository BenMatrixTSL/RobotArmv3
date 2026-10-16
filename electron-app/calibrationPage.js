/**
 * Calibration tab - reads a servo's full EEPROM and SRAM blocks and renders
 * them against the STS3215 memory table (stsMemoryTable.js), or - when
 * "End Tool (ID 64)" is selected in the same dropdown - the ESP32 end tool's
 * register array against endToolMemoryTable.js (identity + persisted Tool
 * type ID as "EEPROM", everything else as editable "RAM"). The EEPROM
 * table highlights anything that differs from the factory default so a
 * change made at some point in the past (e.g. during PID/overload tuning)
 * is easy to spot later. The SRAM table shows live/runtime state (position,
 * speed, load, current, fault flags, torque switch, etc.) read-only - see
 * stsMemoryTable.js for why SRAM writes aren't offered on this page.
 *
 * Editing is off by default and EEPROM-only. "Enable Editing" switches
 * writable EEPROM rows into an editable state; each write still requires the
 * control-lock password (typed once into calibrationWritePassword and sent
 * with every write - the server is the actual authority, this page doesn't
 * validate the password itself). The Position Correction and Commissioning
 * panels lower on the page are only revealed once that same password has
 * been entered via "Enable Editing".
 */

// Cache of the last-read {eeprom, sram} byte arrays per joint, so the "show
// only changed" toggle can re-render without a fresh bus read. Keyed by joint
// number (1-6) or CALIBRATION_TOOL_KEY for the end tool, whose single
// 64-byte register block is stored under both eeprom and sram (the
// endToolMemoryTable.js decoders each pick their own rows out of it).
const calibrationRawByJoint = {};

// Value of the "End Tool (ID 64)" option in calibrationJointSelect.
const CALIBRATION_TOOL_KEY = 'tool';

let calibrationWriteModeEnabled = false;

function setCalibrationStatus(message) {
    const el = document.getElementById('calibrationStatus');
    if (el) el.textContent = message;
}

/**
 * What the joint dropdown currently points at: one of the six servos or the
 * ESP32 end tool. Everything on this page that reads, renders or writes goes
 * through this so the two device types share the same table code.
 * @returns {{isTool:boolean, jointNumber:number|null, key:string|number, label:string}}
 */
function calibrationSelectedTarget() {
    const value = document.getElementById('calibrationJointSelect').value;
    if (value === CALIBRATION_TOOL_KEY) {
        return { isTool: true, jointNumber: null, key: CALIBRATION_TOOL_KEY, label: 'End Tool' };
    }
    const jointNumber = parseInt(value, 10);
    return { isTool: false, jointNumber, key: jointNumber, label: `Joint ${jointNumber}` };
}

/** Selected joint number, or null when the end tool is selected. */
function calibrationSelectedJointNumber() {
    const target = calibrationSelectedTarget();
    return target.isTool ? null : target.jointNumber;
}

async function readCalibrationTable() {
    await readCalibrationForTarget(calibrationSelectedTarget());
    renderCalibrationTable();
}

async function readCalibrationForTarget(target) {
    if (target.isTool) await readCalibrationForTool();
    else await readCalibrationForJoint(target.jointNumber);
}

async function readCalibrationForTool() {
    try {
        setCalibrationStatus('Reading End Tool registers...');
        const response = await robotArmClient.readEndToolRegisters();
        calibrationRawByJoint[CALIBRATION_TOOL_KEY] = { eeprom: response.bytes, sram: response.bytes };
        setCalibrationStatus(`End Tool registers read at ${new Date().toLocaleTimeString()}`);
    } catch (error) {
        setCalibrationStatus(`Failed to read End Tool registers: ${error.message}`);
    }
}

async function readCalibrationForJoint(jointNumber) {
    try {
        setCalibrationStatus(`Reading Joint ${jointNumber} registers...`);
        const response = await robotArmClient.readServoEepromRaw(jointNumber);
        calibrationRawByJoint[jointNumber] = { eeprom: response.eepromBytes, sram: response.sramBytes };
        setCalibrationStatus(`Joint ${jointNumber} registers read at ${new Date().toLocaleTimeString()}`);
    } catch (error) {
        setCalibrationStatus(`Failed to read Joint ${jointNumber} registers: ${error.message}`);
    }
}

/**
 * Toggles the page between read-only and edit mode. Entering edit mode asks
 * the server to verify the control-lock password first (verifyPassword is
 * side-effect free), so the write controls only appear for the real
 * password - the server still checks it again on every write.
 */
async function toggleCalibrationWriteMode() {
    const passwordInput = document.getElementById('calibrationWritePassword');
    const button = document.getElementById('calibrationUnlockWritesButton');
    const editHeader = document.getElementById('calibrationEditHeader');
    const commissioningSection = document.getElementById('calibrationCommissioningSection');

    // The Position Correction panel's visibility also depends on whether a
    // servo or the end tool is selected, so renderCalibrationTable() owns it.
    if (calibrationWriteModeEnabled) {
        calibrationWriteModeEnabled = false;
        button.textContent = 'Enable Editing';
        button.classList.remove('btn-danger');
        button.classList.add('btn-warning');
        if (editHeader) editHeader.hidden = true;
        if (commissioningSection) commissioningSection.hidden = true;
        renderCalibrationTable();
        return;
    }

    if (!passwordInput.value) {
        showAppMessage('Enter the control-lock password to enable editing.');
        return;
    }
    if (!robotArmClient || !robotArmClient.isConnected) {
        showAppMessage('Connect to the robot arm controller first - the password is checked by the server.');
        return;
    }

    button.disabled = true;
    try {
        const result = await robotArmClient.verifyPassword(passwordInput.value);
        if (!result || !result.ok) {
            showAppMessage('Incorrect password - editing stays disabled.');
            setCalibrationStatus('Incorrect password');
            passwordInput.focus();
            passwordInput.select();
            return;
        }
    } catch (error) {
        showAppMessage('Could not verify the password: ' + error.message);
        return;
    } finally {
        button.disabled = false;
    }

    calibrationWriteModeEnabled = true;
    button.textContent = 'Disable Editing';
    button.classList.remove('btn-warning');
    button.classList.add('btn-danger');
    if (editHeader) editHeader.hidden = false;
    if (commissioningSection) commissioningSection.hidden = false;
    renderCalibrationTable();
}

function renderCalibrationTable() {
    updateCalibrationServoOnlyPanels();
    renderEepromTable();
    renderSramTable();
    renderPositionCorrectionDisplay();
}

/**
 * The trainee zero trim and Position Correction panels only make sense for
 * a servo joint - hide them while the end tool is selected. Position
 * Correction additionally needs edit mode to be on.
 */
function updateCalibrationServoOnlyPanels() {
    const isTool = calibrationSelectedTarget().isTool;
    const trimSection = document.getElementById('traineeZeroTrimSection');
    const positionCorrectionSection = document.getElementById('calibrationPositionCorrectionSection');
    if (trimSection) trimSection.hidden = isTool;
    if (positionCorrectionSection) positionCorrectionSection.hidden = isTool || !calibrationWriteModeEnabled;
}

/**
 * Shows the currently selected joint's Position Correction value (decoded
 * from the cached EEPROM block, same as the main table) next to the
 * up/down nudge buttons - " - " until that joint has actually been read.
 */
function renderPositionCorrectionDisplay() {
    const valueEl = document.getElementById('posCorrectionValue');
    if (!valueEl) return;
    const jointNumber = calibrationSelectedJointNumber();
    const cached = jointNumber === null ? null : calibrationRawByJoint[jointNumber];
    if (!cached) {
        valueEl.textContent = 'Position Correction: - ';
        return;
    }
    const raw = rawValueAtAddress(cached.eeprom, STS_POSITION_CORRECTION_ADDRESS, 2);
    valueEl.textContent = `Position Correction: ${decodeSignedStepNumeric(raw)} step`;
}

/**
 * Decoded EEPROM rows for the selected target - STS3215 memory table for a
 * joint, endToolMemoryTable.js for the end tool. Null until it's been read.
 */
function decodedEepromRowsForTarget(target) {
    const cached = calibrationRawByJoint[target.key];
    if (!cached) return null;
    return target.isTool ? decodeEndToolEepromBlock(cached.eeprom) : decodeEepromBlock(cached.eeprom);
}

/** Decoded SRAM/RAM rows for the selected target, or null until read. */
function decodedSramRowsForTarget(target) {
    const cached = calibrationRawByJoint[target.key];
    if (!cached) return null;
    return target.isTool ? decodeEndToolRamBlock(cached.sram) : decodeSramBlock(cached.sram);
}

/**
 * The Edit column cell for one register row (only rendered in edit mode):
 * a value box and Write button for writable registers, a padlock otherwise.
 */
function calibrationEditCell(r) {
    if (r.writable) {
        return `
            <td>
                <input type="number" class="cal-edit-input" id="cal-edit-${r.address}" value="${decodeRegisterValue(r, r.raw)}" min="${r.min}" max="${r.max}">
                <button class="btn btn-small btn-primary" onclick="writeCalibrationRegister(${r.address})">Write</button>
            </td>
        `;
    }
    const reason = r.access === 'read' ? 'Read-only hardware field' : 'Not editable from this page';
    return `<td><span class="cal-locked" title="${escapeCalibrationText(reason)}">&#128274;</span></td>`;
}

function renderEepromTable() {
    const target = calibrationSelectedTarget();
    const tbody = document.getElementById('calibrationTableBody');
    if (!tbody) return;

    const colSpan = calibrationWriteModeEnabled ? 9 : 8;
    const decoded = decodedEepromRowsForTarget(target);
    if (!decoded) {
        tbody.innerHTML = `<tr><td colspan="${colSpan}" class="calibration-empty">Select a joint or the end tool and click &quot;Read Registers&quot;.</td></tr>`;
        return;
    }

    const diffOnly = document.getElementById('calibrationDiffOnly').checked;
    const rows = diffOnly ? decoded.filter(r => !r.isDefault) : decoded;

    if (rows.length === 0) {
        tbody.innerHTML = `<tr><td colspan="${colSpan}" class="calibration-empty">Every value matches its factory default.</td></tr>`;
        return;
    }

    tbody.innerHTML = rows.map(r => {
        const addrHex = '0x' + r.address.toString(16).toUpperCase().padStart(2, '0');
        const readOnly = r.access === 'read';
        const rangeStr = r.min === -1 && r.max === -1 ? ' - ' : `${r.min}..${r.max} ${r.unit === ' - ' ? '' : r.unit}`.trim();
        const editCell = calibrationWriteModeEnabled ? calibrationEditCell(r) : '';

        return `
            <tr class="${r.isDefault ? '' : 'cal-changed'}${readOnly ? ' cal-readonly' : ''}">
                <td>${addrHex} <span style="color:#999;">(${r.address})</span></td>
                <td class="cal-name" title="${escapeCalibrationText(r.description)}">${escapeCalibrationText(r.name)}</td>
                <td>${r.bytes}</td>
                <td class="cal-value">${r.raw}</td>
                <td class="cal-value cal-meaning">${escapeCalibrationText(r.meaningful)}</td>
                <td class="cal-default">${escapeCalibrationText(r.defaultMeaningful)}</td>
                <td class="cal-range">${escapeCalibrationText(rangeStr)}</td>
                <td>${readOnly ? 'read' : 'r/w'}</td>
                ${editCell}
            </tr>
        `;
    }).join('');
}

function renderSramTable() {
    const target = calibrationSelectedTarget();
    const tbody = document.getElementById('calibrationSramTableBody');
    if (!tbody) return;

    // Servo SRAM is deliberately read-only here (see stsMemoryTable.js); the
    // end tool's RAM is its control interface, so it gets an Edit column.
    const showEdit = target.isTool && calibrationWriteModeEnabled;
    const editHeader = document.getElementById('calibrationSramEditHeader');
    if (editHeader) editHeader.hidden = !showEdit;
    const heading = document.getElementById('calibrationSramHeading');
    if (heading) {
        heading.textContent = target.isTool
            ? 'RAM (outputs, telemetry and hobby-servo settings - resets on power-off, editable here)'
            : 'SRAM (live/runtime state - resets on power-off, read-only here)';
    }
    const colSpan = showEdit ? 7 : 6;

    const decoded = decodedSramRowsForTarget(target);
    if (!decoded) {
        tbody.innerHTML = `<tr><td colspan="${colSpan}" class="calibration-empty">Select a joint or the end tool and click &quot;Read Registers&quot;.</td></tr>`;
        return;
    }

    tbody.innerHTML = decoded.map(r => {
        const addrHex = '0x' + r.address.toString(16).toUpperCase().padStart(2, '0');
        const readOnly = r.access === 'read';
        const accessStr = readOnly ? 'read' : target.isTool ? 'r/w' : 'live (read-only here)';
        const editCell = showEdit ? calibrationEditCell(r) : '';
        return `
            <tr>
                <td>${addrHex} <span style="color:#999;">(${r.address})</span></td>
                <td class="cal-name" title="${escapeCalibrationText(r.description)}">${escapeCalibrationText(r.name)}</td>
                <td>${r.bytes}</td>
                <td class="cal-value">${r.raw}</td>
                <td class="cal-value cal-meaning">${escapeCalibrationText(r.meaningful)}</td>
                <td>${accessStr}</td>
                ${editCell}
            </tr>
        `;
    }).join('');
}

/**
 * Finds a register row by address in whichever decoded block holds it for
 * the selected target: servo EEPROM only (servo SRAM is never written from
 * this page), or either of the end tool's EEPROM/RAM blocks.
 */
function calibrationRegisterForAddress(target, address) {
    const eepromRows = decodedEepromRowsForTarget(target);
    const found = eepromRows && eepromRows.find(r => r.address === address);
    if (found || !target.isTool) return found || null;
    const ramRows = decodedSramRowsForTarget(target);
    return (ramRows && ramRows.find(r => r.address === address)) || null;
}

/**
 * Writes one register's raw value to the device currently selected in the
 * dropdown (a servo's EEPROM, or any writable end tool register), after
 * range/confirmation checks, then re-reads that device's registers so the
 * table reflects what the hardware actually accepted. Servo SRAM is never
 * writable from this page (see stsMemoryTable.js).
 */
async function writeCalibrationRegister(address) {
    const target = calibrationSelectedTarget();
    const reg = calibrationRegisterForAddress(target, address);
    if (!reg) return;

    const input = document.getElementById(`cal-edit-${address}`);
    const rawValue = parseInt(input.value, 10);
    if (isNaN(rawValue) || rawValue < reg.min || rawValue > reg.max) {
        showAppMessage(`Value must be between ${reg.min} and ${reg.max}.`);
        return;
    }

    if (reg.extraConfirm) {
        const confirmed = await showConfirm(
            `Write ${rawValue} to "${reg.name}" (address 0x${address.toString(16)}) on ${target.label}?\n\n` +
            `This can let the joint travel to a position that's mechanically unsafe if set incorrectly. Double-check the value before continuing.`
        );
        if (!confirmed) return;
    }

    const password = document.getElementById('calibrationWritePassword').value;
    const writeButton = input.nextElementSibling;
    const addrHex = '0x' + address.toString(16);
    if (writeButton) writeButton.disabled = true;
    input.disabled = true;

    // A few servo registers (Position Correction, in particular) use a bit
    // pattern that isn't standard two's complement - encode the value the
    // user typed into the actual raw integer the device expects before
    // sending it. End tool registers are all plain integers.
    const encodedValue = target.isTool ? rawValue : encodeRegisterValue(reg, rawValue);

    try {
        setCalibrationStatus(`Writing ${target.label} address ${addrHex}...`);
        if (target.isTool) {
            await robotArmClient.writeEndToolRegister(address, encodedValue, password);
        } else {
            await robotArmClient.writeServoEepromRaw(target.jointNumber, address, encodedValue, password);
        }

        // The write's own reply already waits for the device's settle time
        // server-side, but read it back explicitly rather than trusting
        // "success" alone - that's the only way to know the table reflects
        // what the hardware actually holds, not just what we asked for.
        setCalibrationStatus(`Verifying ${target.label} address ${addrHex}...`);
        await readCalibrationForTarget(target);
        renderCalibrationTable();

        const confirmedReg = calibrationRegisterForAddress(target, address);
        if (confirmedReg && confirmedReg.raw === encodedValue) {
            setCalibrationStatus(`Confirmed - ${target.label} address ${addrHex} now reads ${rawValue} (${confirmedReg.meaningful}).`);
        } else {
            const actual = confirmedReg ? confirmedReg.meaningful : '(read failed)';
            setCalibrationStatus(`Wrote ${rawValue} but the read-back shows ${actual} - the write may not have taken effect. Try again or check the ${target.isTool ? 'end tool' : 'servo'} connection.`);
        }
    } catch (error) {
        setCalibrationStatus(`Failed to write ${target.label} address ${addrHex}: ${error.message}`);
        renderCalibrationTable(); // re-enable the row's controls even on failure
    }
}

// EEPROM address of Position Correction (2 bytes, signed-step encoding -
// see encodeSignedStep/decodeSignedStepNumeric in stsMemoryTable.js).
const STS_POSITION_CORRECTION_ADDRESS = 0x1F;
// SRAM Target Location (goal position) address and the absolute step value
// that commands the servo to the mechanical center - same plain 0-4095
// encoding robotArmST3215.js's moveToPosition() uses, NOT the signed-step
// encoding stsMemoryTable.js decodes this register as for display. Driving
// the servo there is what makes a Position Correction change visible
// immediately instead of only after the servo's next power-up.
const STS_TARGET_LOCATION_ADDRESS = 0x2A;
const STS_TARGET_LOCATION_RECENTER_VALUE = 2048;

/**
 * Nudges the selected joint's Position Correction by `direction` (-1 or +1)
 * times the step size field, writes it, then commands the servo to the
 * mechanical center (Target Location = 2048) so the change is reflected in
 * the servo's live position right away - this physically moves the joint.
 * Re-reads the joint afterward and checks the read-back actually matches
 * what was written before reporting success, so the on-page value and the
 * main EEPROM table only ever show what's genuinely on the hardware.
 */
async function adjustPositionCorrection(direction) {
    const jointNumber = calibrationSelectedJointNumber();
    if (jointNumber === null) {
        showAppMessage('Position Correction only applies to a servo joint - select a joint first.');
        return;
    }
    const password = document.getElementById('calibrationWritePassword').value;
    if (!password) {
        showAppMessage('Enter the control-lock password to enable editing.');
        return;
    }

    if (!calibrationRawByJoint[jointNumber]) {
        await readCalibrationForJoint(jointNumber);
    }
    const cached = calibrationRawByJoint[jointNumber];
    if (!cached) {
        setCalibrationStatus(`Couldn't read Joint ${jointNumber}'s current Position Correction.`);
        return;
    }

    const stepInput = document.getElementById('posCorrectionStep');
    const step = Math.max(1, parseInt(stepInput.value, 10) || 1);
    const currentValue = decodeSignedStepNumeric(rawValueAtAddress(cached.eeprom, STS_POSITION_CORRECTION_ADDRESS, 2));
    const newValue = Math.max(-2047, Math.min(2047, currentValue + direction * step));
    const newRaw = encodeSignedStep(newValue);

    const upButton = document.getElementById('posCorrectionUpButton');
    const downButton = document.getElementById('posCorrectionDownButton');
    if (upButton) upButton.disabled = true;
    if (downButton) downButton.disabled = true;
    try {
        setCalibrationStatus(`Joint ${jointNumber}: writing Position Correction = ${newValue} step...`);
        await robotArmClient.writeServoEepromRaw(jointNumber, STS_POSITION_CORRECTION_ADDRESS, newRaw, password);

        setCalibrationStatus(`Joint ${jointNumber}: moving to center (Target Location = 2048)...`);
        await robotArmClient.writeServoEepromRaw(jointNumber, STS_TARGET_LOCATION_ADDRESS, STS_TARGET_LOCATION_RECENTER_VALUE, password);

        // Both writes' own replies already wait for their bus-write settle
        // time, but that move is still ongoing SRAM/bus traffic - give it a
        // moment before reading back, rather than racing it, and then
        // actually check the read-back matches what was written instead of
        // just assuming it did (readCalibrationForJoint swallows its own
        // read errors, so a failed read would otherwise look identical to
        // a successful one here).
        await new Promise(resolve => setTimeout(resolve, 200));

        setCalibrationStatus(`Joint ${jointNumber}: verifying Position Correction...`);
        await readCalibrationForJoint(jointNumber);
        renderCalibrationTable();

        const confirmedCache = calibrationRawByJoint[jointNumber];
        const confirmedRaw = confirmedCache ? rawValueAtAddress(confirmedCache.eeprom, STS_POSITION_CORRECTION_ADDRESS, 2) : null;
        if (confirmedCache && confirmedRaw === newRaw) {
            setCalibrationStatus(`Joint ${jointNumber}: Position Correction confirmed at ${newValue} step, moved to center (2048).`);
        } else {
            const actual = confirmedCache ? `${decodeSignedStepNumeric(confirmedRaw)} step` : '(read failed)';
            setCalibrationStatus(`Wrote Position Correction ${newValue} step but the read-back shows ${actual} - the write may not have taken effect. Try again or check the servo connection.`);
        }
    } catch (error) {
        setCalibrationStatus(`Failed to adjust Joint ${jointNumber}'s Position Correction: ${error.message}`);
    } finally {
        if (upButton) upButton.disabled = false;
        if (downButton) downButton.disabled = false;
    }
}

// Custom values that override the factory default for these registers,
// applied to every joint identically. Every other writable register from
// COMMISSIONING_MIN_ADDRESS through COMMISSIONING_MAX_ADDRESS is instead
// reset to its factory default - see commissioningWritesForJoint() below.
const COMMISSIONING_OVERRIDES = {
    0x0E: { value: 140, label: 'Max input voltage' },
    0x17: { value: 10, label: 'I coefficient' },
    0x1C: { value: 100, label: 'Protection current' },
    0x22: { value: 10, label: 'Protective torque' },
    0x23: { value: 50, label: 'Protection time' },
    0x24: { value: 40, label: 'Overload torque' },
};
const COMMISSIONING_MIN_ANGLE_ADDRESS = 0x09;
const COMMISSIONING_MAX_ANGLE_ADDRESS = 0x0B;
const COMMISSIONING_MIN_ADDRESS = 0x07;
const COMMISSIONING_MAX_ADDRESS = 0x27;
const COMMISSIONING_JOINT_COUNT = 6;

// Position correction (0x1F) is per-servo zero-point calibration, not a
// generic default - commissioning must never stomp it, so it's handled
// separately from this button entirely.
const COMMISSIONING_SKIP_ADDRESSES = new Set([0x1F]);

// Raw-step <-> radian conversion the ST3215 servos use: 0 rad = 2048 (center),
// full circle = 4096 steps. Same mapping as robotArmST3215.js's
// CENTER_POSITION/STEPS_PER_DEGREE, expressed per-radian instead of per-degree.
const COMMISSIONING_STEPS_PER_RADIAN = 2048 / Math.PI;
const COMMISSIONING_CENTER_STEP = 2048;

/**
 * This joint's min/max angle limit, as raw steps (0-4095, center 2048 = 0°),
 * read live from the loaded kinematics.urdf via the global robotKinematics
 * instance (kinematics.js) - not a hardcoded copy of the URDF's numbers,
 * which would silently go stale the next time someone tunes a joint limit
 * there. Throws if the URDF hasn't loaded (or doesn't describe this joint),
 * rather than writing a limit computed from missing data.
 */
function commissioningAngleLimitsForJoint(jointNumber) {
    const joint = robotKinematics.joints && robotKinematics.joints[jointNumber - 1];
    const limits = joint && joint.limits;
    if (!limits || typeof limits.lowerRadians !== 'number' || typeof limits.upperRadians !== 'number') {
        throw new Error(`No joint limits available for Joint ${jointNumber} - kinematics.urdf may not have loaded yet`);
    }
    const min = Math.round(COMMISSIONING_CENTER_STEP + limits.lowerRadians * COMMISSIONING_STEPS_PER_RADIAN);
    const max = Math.round(COMMISSIONING_CENTER_STEP + limits.upperRadians * COMMISSIONING_STEPS_PER_RADIAN);
    return {
        min: Math.max(0, Math.min(4094, min)),
        max: Math.max(1, Math.min(4095, max)),
    };
}

/**
 * Every EEPROM register one joint's servo gets written to by the
 * Commissioning button, in address order: COMMISSIONING_OVERRIDES's custom
 * values, this joint's angle limits from commissioningAngleLimitsForJoint(),
 * and every other writable register in the 0x07-0x27 range reset to its
 * factory default - skipping addresses the Calibration page itself refuses
 * to write (ID, Baud rate, Phase - STS_WRITE_BLOCKED_ADDRESSES, from
 * stsMemoryTable.js) and COMMISSIONING_SKIP_ADDRESSES.
 */
function commissioningWritesForJoint(jointNumber) {
    const angleLimits = commissioningAngleLimitsForJoint(jointNumber);
    return STS_EEPROM_REGISTERS
        .filter(reg => reg.address >= COMMISSIONING_MIN_ADDRESS && reg.address <= COMMISSIONING_MAX_ADDRESS)
        .filter(reg => !STS_WRITE_BLOCKED_ADDRESSES.has(reg.address))
        .filter(reg => !COMMISSIONING_SKIP_ADDRESSES.has(reg.address))
        .map(reg => {
            if (reg.address === COMMISSIONING_MIN_ANGLE_ADDRESS) {
                return { address: reg.address, bytes: reg.bytes, value: angleLimits.min, label: 'Min angle limit', isOverride: true };
            }
            if (reg.address === COMMISSIONING_MAX_ANGLE_ADDRESS) {
                return { address: reg.address, bytes: reg.bytes, value: angleLimits.max, label: 'Max angle limit', isOverride: true };
            }
            const override = COMMISSIONING_OVERRIDES[reg.address];
            return {
                address: reg.address,
                bytes: reg.bytes,
                value: override ? override.value : reg.default,
                label: override ? override.label : reg.name,
                isOverride: !!override,
            };
        });
}

/**
 * Reads a register's current raw value out of a raw EEPROM byte block (as
 * returned by readServoEepromRaw), for comparing against the value
 * commissioning wants to write there. Same little-endian layout as
 * decodeEepromBlock() in stsMemoryTable.js.
 */
function rawValueAtAddress(bytes, address, byteCount) {
    return byteCount === 2 ? (bytes[address] || 0) | ((bytes[address + 1] || 0) << 8) : (bytes[address] || 0);
}

/**
 * Writes commissioningWritesForJoint()'s registers to every joint's servo -
 * but reads each joint's current EEPROM first and skips any register that's
 * already at its target value, so re-running commissioning on an
 * already-commissioned arm only touches what's actually changed (a fresh or
 * factory-reset servo still gets every register written, since none of them
 * will match yet). This is a lot fewer bus round-trips than blindly writing
 * all ~150 registers every time, so a re-run is quicker and puts far less
 * traffic on a bus that's also running the 20ms status poll concurrently.
 */
async function commissionAllServos() {
    const password = document.getElementById('calibrationWritePassword').value;
    if (!password) {
        showAppMessage('Enter the control-lock password to enable editing.');
        return;
    }
    if (!robotKinematics.joints || robotKinematics.joints.length < COMMISSIONING_JOINT_COUNT) {
        showAppMessage("Joint limits aren't loaded yet (kinematics.urdf) - wait for the app to finish starting up and try again.");
        return;
    }

    const globalOverridesSummary = Object.entries(COMMISSIONING_OVERRIDES)
        .map(([address, o]) => `0x${Number(address).toString(16).toUpperCase()} (${o.label}) = ${o.value}`)
        .join('\n');
    const defaultCount = commissioningWritesForJoint(1).filter(r => !r.isOverride).length;
    const confirmed = await showConfirm(
        `Write default values to all ${COMMISSIONING_JOINT_COUNT} servos?\n\n${globalOverridesSummary}\n` +
        `Min/Max angle limit = this arm's own travel range per joint (from kinematics.urdf)\n\n` +
        `...plus ${defaultCount} other registers (addresses 0x${COMMISSIONING_MIN_ADDRESS.toString(16).toUpperCase()}-` +
        `0x${COMMISSIONING_MAX_ADDRESS.toString(16).toUpperCase()}) reset to their factory default.\n\n` +
        `Each joint is read first, and any register already at its target value is left alone.\n\n` +
        `This writes directly to EEPROM on every joint, immediately.`
    );
    if (!confirmed) return;

    const button = document.getElementById('commissionAllButton');
    if (button) button.disabled = true;
    const failures = [];
    let successCount = 0;
    let skippedCount = 0;
    try {
        for (let joint = 1; joint <= COMMISSIONING_JOINT_COUNT; joint++) {
            setCalibrationStatus(`Joint ${joint}: reading current EEPROM...`);
            let currentBytes = null;
            try {
                const response = await robotArmClient.readServoEepromRaw(joint);
                currentBytes = response.eepromBytes;
            } catch (error) {
                // Can't tell what's already correct - fall back to writing
                // every register for this joint, same as before this change.
                failures.push(`Joint ${joint}: couldn't read current EEPROM (${error.message}) - writing every register unconditionally`);
            }

            for (const { address, value, label, bytes } of commissioningWritesForJoint(joint)) {
                const addrHex = '0x' + address.toString(16).toUpperCase();
                if (currentBytes && rawValueAtAddress(currentBytes, address, bytes) === value) {
                    skippedCount++;
                    continue;
                }
                setCalibrationStatus(`Joint ${joint}: writing ${label} (${addrHex}) = ${value}...`);
                try {
                    await writeCommissioningRegisterWithRetry(joint, address, value, password);
                    successCount++;
                } catch (error) {
                    // A bulk run touches ~150 registers across 6 servos - one
                    // transient bus hiccup must not silently abort every
                    // write after it, so record the failure and keep going.
                    failures.push(`Joint ${joint} ${addrHex} (${label}): ${error.message}`);
                }
            }
        }

        // Every joint's registers just changed - drop the whole cache rather
        // than leave other joints' tables showing stale pre-commissioning
        // bytes, then re-read the one currently on screen.
        for (const key of Object.keys(calibrationRawByJoint)) delete calibrationRawByJoint[key];
        await readCalibrationForTarget(calibrationSelectedTarget());
        renderCalibrationTable();

        if (failures.length === 0) {
            setCalibrationStatus(
                `Commissioned all ${COMMISSIONING_JOINT_COUNT} servos at ${new Date().toLocaleTimeString()}: ` +
                `${successCount} register(s) written, ${skippedCount} already correct.`
            );
        } else {
            setCalibrationStatus(
                `Commissioned ${successCount} register write(s), ${skippedCount} already correct; ` +
                `${failures.length} failed (likely a transient bus error - re-run to retry just those): ${failures.join('; ')}`
            );
        }
    } catch (error) {
        setCalibrationStatus(`Commissioning failed: ${error.message}`);
    } finally {
        if (button) button.disabled = false;
    }
}

/**
 * A single commissioning register write, with its own retry on top of
 * writeServoEepromRaw's own low-level bus retry - bulk commissioning touches
 * ~150 registers in one run, so one transient bus hiccup among all of them
 * shouldn't need a whole extra pass over every joint to recover from.
 */
async function writeCommissioningRegisterWithRetry(joint, address, value, password, attemptsLeft = 2) {
    try {
        await robotArmClient.writeServoEepromRaw(joint, address, value, password);
    } catch (error) {
        if (attemptsLeft <= 1) throw error;
        await new Promise(resolve => setTimeout(resolve, 150));
        await writeCommissioningRegisterWithRetry(joint, address, value, password, attemptsLeft - 1);
    }
}

// ===== Student zero calibration (trainee trims) =====
// A software-only zero offset per joint, kept by the server and reported in
// each joint's jointConfigs entry as zeroTrimDegrees. Unlike Position
// Correction above it needs no password and never touches servo EEPROM, so
// "Reset" always gets back to the real calibration - safe for students.

/**
 * Shows every joint's current trainee trim, and warns when any are active so
 * a trim left behind by one class doesn't silently affect the next.
 */
function renderTraineeZeroTrims() {
    const summary = document.getElementById('zeroTrimSummary');
    if (!summary) return;
    const configs = (robotArmClient && robotArmClient.jointConfigs) || [];
    const trims = configs.filter(j => j && j.jointNumber !== undefined && typeof j.zeroTrimDegrees === 'number');
    if (trims.length === 0) {
        summary.textContent = 'Trainee trims: not available (connect to the arm - the server may need updating).';
        summary.classList.remove('zero-trim-active');
        return;
    }
    const active = trims.filter(j => j.zeroTrimDegrees !== 0);
    const list = trims.map(j => `J${j.jointNumber} ${j.zeroTrimDegrees > 0 ? '+' : ''}${j.zeroTrimDegrees.toFixed(2)}°`).join('   ');
    summary.textContent = active.length === 0
        ? `Trainee trims: none active - every joint is using the servo's own calibration.   (${list})`
        : `⚠ Trainee trims ACTIVE on ${active.map(j => 'J' + j.jointNumber).join(', ')} - reset them when you have finished.   (${list})`;
    summary.classList.toggle('zero-trim-active', active.length > 0);
}

async function setTraineeZeroTrim() {
    const jointNumber = calibrationSelectedJointNumber();
    if (jointNumber === null) {
        showAppMessage('Zero trims only apply to a servo joint - select a joint first.');
        return;
    }
    const confirmed = await showConfirm(
        `Make Joint ${jointNumber}'s current position read as 0°?\n\n` +
        `Only do this when the joint's alignment marks line up exactly. The joint will not move. ` +
        `You can undo it at any time with "Reset this joint".`
    );
    if (!confirmed) return;
    try {
        setCalibrationStatus(`Joint ${jointNumber}: setting trainee zero trim...`);
        const result = await robotArmClient.setJointZeroTrim(jointNumber);
        setCalibrationStatus(result && result.message ? result.message : `Joint ${jointNumber} zero trim set.`);
    } catch (error) {
        setCalibrationStatus(`Could not set Joint ${jointNumber}'s zero trim: ${error.message}`);
    }
}

async function resetTraineeZeroTrim(allJoints) {
    const jointNumber = calibrationSelectedJointNumber();
    if (!allJoints && jointNumber === null) {
        showAppMessage('Zero trims only apply to a servo joint - select a joint first.');
        return;
    }
    const target = allJoints ? 'all' : jointNumber;
    try {
        setCalibrationStatus(allJoints ? 'Resetting all trainee zero trims...' : `Joint ${jointNumber}: resetting trainee zero trim...`);
        const result = await robotArmClient.resetJointZeroTrim(target);
        setCalibrationStatus(result && result.message ? result.message : 'Zero trim reset.');
    } catch (error) {
        setCalibrationStatus(`Could not reset the zero trim: ${error.message}`);
    }
}

function escapeCalibrationText(text) {
    if (typeof escapeHtml === 'function') return escapeHtml(text);
    return String(text === null || text === undefined ? '' : text)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#39;');
}

document.addEventListener('DOMContentLoaded', () => {
    const jointSelect = document.getElementById('calibrationJointSelect');
    // Auto-read on joint change so the table always reflects that joint's
    // actual current registers rather than stale data left over from
    // whichever joint was last read (or nothing, before the first read).
    if (jointSelect) jointSelect.addEventListener('change', readCalibrationTable);
});
