/**
 * Calibration tab — reads a servo's full EEPROM and SRAM blocks and renders
 * them against the STS3215 memory table (stsMemoryTable.js). The EEPROM
 * table highlights anything that differs from the factory default so a
 * change made at some point in the past (e.g. during PID/overload tuning)
 * is easy to spot later. The SRAM table shows live/runtime state (position,
 * speed, load, current, fault flags, torque switch, etc.) read-only — see
 * stsMemoryTable.js for why SRAM writes aren't offered on this page.
 *
 * Editing is off by default and EEPROM-only. "Enable Editing" switches
 * writable EEPROM rows into an editable state; each write still requires the
 * control-lock password (typed once into calibrationWritePassword and sent
 * with every write — the server is the actual authority, this page doesn't
 * validate the password itself). The Position Correction and Commissioning
 * panels lower on the page are only revealed once that same password has
 * been entered via "Enable Editing".
 */

// Cache of the last-read {eeprom, sram} byte arrays per joint, so the "show
// only changed" toggle can re-render without a fresh bus read.
const calibrationRawByJoint = {};

let calibrationWriteModeEnabled = false;

function setCalibrationStatus(message) {
    const el = document.getElementById('calibrationStatus');
    if (el) el.textContent = message;
}

async function readCalibrationTable() {
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
    await readCalibrationForJoint(jointNumber);
    renderCalibrationTable();
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
 * Toggles the page between read-only and edit mode. Requires a non-empty
 * password to enter edit mode (the server rejects the write itself if the
 * password is wrong — this is just gating the UI, not authenticating).
 */
function toggleCalibrationWriteMode() {
    const passwordInput = document.getElementById('calibrationWritePassword');
    const button = document.getElementById('calibrationUnlockWritesButton');
    const editHeader = document.getElementById('calibrationEditHeader');
    const commissioningSection = document.getElementById('calibrationCommissioningSection');
    const positionCorrectionSection = document.getElementById('calibrationPositionCorrectionSection');

    if (calibrationWriteModeEnabled) {
        calibrationWriteModeEnabled = false;
        button.textContent = 'Enable Editing';
        button.classList.remove('btn-danger');
        button.classList.add('btn-warning');
        if (editHeader) editHeader.hidden = true;
        if (commissioningSection) commissioningSection.hidden = true;
        if (positionCorrectionSection) positionCorrectionSection.hidden = true;
        renderCalibrationTable();
        return;
    }

    if (!passwordInput.value) {
        showAppMessage('Enter the control-lock password to enable editing.');
        return;
    }

    calibrationWriteModeEnabled = true;
    button.textContent = 'Disable Editing';
    button.classList.remove('btn-warning');
    button.classList.add('btn-danger');
    if (editHeader) editHeader.hidden = false;
    if (commissioningSection) commissioningSection.hidden = false;
    if (positionCorrectionSection) positionCorrectionSection.hidden = false;
    renderCalibrationTable();
}

function renderCalibrationTable() {
    renderEepromTable();
    renderSramTable();
    renderPositionCorrectionDisplay();
}

/**
 * Shows the currently selected joint's Position Correction value (decoded
 * from the cached EEPROM block, same as the main table) next to the
 * up/down nudge buttons — "—" until that joint has actually been read.
 */
function renderPositionCorrectionDisplay() {
    const valueEl = document.getElementById('posCorrectionValue');
    if (!valueEl) return;
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
    const cached = calibrationRawByJoint[jointNumber];
    if (!cached) {
        valueEl.textContent = 'Position Correction: —';
        return;
    }
    const raw = rawValueAtAddress(cached.eeprom, STS_POSITION_CORRECTION_ADDRESS, 2);
    valueEl.textContent = `Position Correction: ${decodeSignedStepNumeric(raw)} step`;
}

function renderEepromTable() {
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
    const tbody = document.getElementById('calibrationTableBody');
    if (!tbody) return;

    const colSpan = calibrationWriteModeEnabled ? 9 : 8;
    const cached = calibrationRawByJoint[jointNumber];
    if (!cached) {
        tbody.innerHTML = `<tr><td colspan="${colSpan}" class="calibration-empty">Select a joint and click &quot;Read EEPROM&quot;.</td></tr>`;
        return;
    }

    const decoded = decodeEepromBlock(cached.eeprom);
    const diffOnly = document.getElementById('calibrationDiffOnly').checked;
    const rows = diffOnly ? decoded.filter(r => !r.isDefault) : decoded;

    if (rows.length === 0) {
        tbody.innerHTML = `<tr><td colspan="${colSpan}" class="calibration-empty">Every value matches its factory default.</td></tr>`;
        return;
    }

    tbody.innerHTML = rows.map(r => {
        const addrHex = '0x' + r.address.toString(16).toUpperCase().padStart(2, '0');
        const readOnly = r.access === 'read';
        const rangeStr = r.min === -1 && r.max === -1 ? '—' : `${r.min}..${r.max} ${r.unit === '—' ? '' : r.unit}`.trim();

        let editCell = '';
        if (calibrationWriteModeEnabled) {
            if (r.writable) {
                editCell = `
                    <td>
                        <input type="number" class="cal-edit-input" id="cal-edit-${r.address}" value="${decodeRegisterValue(r, r.raw)}" min="${r.min}" max="${r.max}">
                        <button class="btn btn-small btn-primary" onclick="writeCalibrationRegister(${r.address})">Write</button>
                    </td>
                `;
            } else {
                const reason = r.access === 'read' ? 'Read-only hardware field' : 'Not editable from this page';
                editCell = `<td><span class="cal-locked" title="${escapeCalibrationText(reason)}">&#128274;</span></td>`;
            }
        }

        return `
            <tr class="${r.isDefault ? '' : 'cal-changed'}${readOnly ? ' cal-readonly' : ''}">
                <td>${addrHex} <span style="color:#999;">(${r.address})</span></td>
                <td class="cal-name" title="${escapeCalibrationText(r.description)}">${escapeCalibrationText(r.name)}</td>
                <td>${r.bytes}</td>
                <td class="cal-value">${r.raw}</td>
                <td class="cal-value">${escapeCalibrationText(r.meaningful)}</td>
                <td class="cal-default">${escapeCalibrationText(r.defaultMeaningful)}</td>
                <td>${escapeCalibrationText(rangeStr)}</td>
                <td>${readOnly ? 'read' : 'r/w'}</td>
                ${editCell}
            </tr>
        `;
    }).join('');
}

function renderSramTable() {
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
    const tbody = document.getElementById('calibrationSramTableBody');
    if (!tbody) return;

    const cached = calibrationRawByJoint[jointNumber];
    if (!cached) {
        tbody.innerHTML = `<tr><td colspan="6" class="calibration-empty">Select a joint and click &quot;Read EEPROM&quot;.</td></tr>`;
        return;
    }

    const decoded = decodeSramBlock(cached.sram);
    tbody.innerHTML = decoded.map(r => {
        const addrHex = '0x' + r.address.toString(16).toUpperCase().padStart(2, '0');
        const readOnly = r.access === 'read';
        const rangeStr = r.min === -1 && r.max === -1 ? '—' : `${r.min}..${r.max} ${r.unit === '—' ? '' : r.unit}`.trim();
        return `
            <tr>
                <td>${addrHex} <span style="color:#999;">(${r.address})</span></td>
                <td class="cal-name" title="${escapeCalibrationText(r.description)}">${escapeCalibrationText(r.name)}</td>
                <td>${r.bytes}</td>
                <td class="cal-value">${r.raw}</td>
                <td class="cal-value">${escapeCalibrationText(r.meaningful)}</td>
                <td>${readOnly ? 'read' : 'live (read-only here)'}</td>
            </tr>
        `;
    }).join('');
}

/**
 * Writes one EEPROM register's raw value to the servo currently selected in
 * the joint dropdown, after range/confirmation checks, then re-reads that
 * joint's registers so the table reflects what the hardware actually
 * accepted. SRAM is never writable from this page (see stsMemoryTable.js).
 */
async function writeCalibrationRegister(address) {
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
    const cached = calibrationRawByJoint[jointNumber];
    const reg = cached && decodeEepromBlock(cached.eeprom).find(r => r.address === address);
    if (!reg) return;

    const input = document.getElementById(`cal-edit-${address}`);
    const rawValue = parseInt(input.value, 10);
    if (isNaN(rawValue) || rawValue < reg.min || rawValue > reg.max) {
        showAppMessage(`Value must be between ${reg.min} and ${reg.max}.`);
        return;
    }

    if (reg.extraConfirm) {
        const confirmed = await showConfirm(
            `Write ${rawValue} to "${reg.name}" (address 0x${address.toString(16)}) on Joint ${jointNumber}?\n\n` +
            `This can let the joint travel to a position that's mechanically unsafe if set incorrectly. Double-check the value before continuing.`
        );
        if (!confirmed) return;
    }

    const password = document.getElementById('calibrationWritePassword').value;
    const writeButton = input.nextElementSibling;
    const addrHex = '0x' + address.toString(16);
    if (writeButton) writeButton.disabled = true;
    input.disabled = true;

    // A few registers (Position Correction, in particular) use a bit pattern
    // that isn't standard two's complement — encode the value the user typed
    // into the actual raw integer the servo expects before sending it.
    const encodedValue = encodeRegisterValue(reg, rawValue);

    try {
        setCalibrationStatus(`Writing Joint ${jointNumber} address ${addrHex}...`);
        await robotArmClient.writeServoEepromRaw(jointNumber, address, encodedValue, password);

        // The write's own reply already waits for the servo's EEPROM-write
        // settle time server-side, but read it back explicitly rather than
        // trusting "success" alone — that's the only way to know the table
        // reflects what the hardware actually holds, not just what we asked for.
        setCalibrationStatus(`Verifying Joint ${jointNumber} address ${addrHex}...`);
        await readCalibrationForJoint(jointNumber);
        renderCalibrationTable();

        const confirmedReg = decodeEepromBlock(calibrationRawByJoint[jointNumber].eeprom).find(r => r.address === address);
        if (confirmedReg && confirmedReg.raw === encodedValue) {
            setCalibrationStatus(`Confirmed — Joint ${jointNumber} address ${addrHex} now reads ${rawValue} (${confirmedReg.meaningful}).`);
        } else {
            const actual = confirmedReg ? confirmedReg.meaningful : '(read failed)';
            setCalibrationStatus(`Wrote ${rawValue} but the read-back shows ${actual} — the write may not have taken effect. Try again or check the servo connection.`);
        }
    } catch (error) {
        setCalibrationStatus(`Failed to write Joint ${jointNumber} address ${addrHex}: ${error.message}`);
        renderCalibrationTable(); // re-enable the row's controls even on failure
    }
}

// EEPROM address of Position Correction (2 bytes, signed-step encoding —
// see encodeSignedStep/decodeSignedStepNumeric in stsMemoryTable.js).
const STS_POSITION_CORRECTION_ADDRESS = 0x1F;
// SRAM Target Location (goal position) address and the absolute step value
// that commands the servo to the mechanical center — same plain 0-4095
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
 * the servo's live position right away — this physically moves the joint.
 * Re-reads the joint afterward so the on-page value and the main EEPROM
 * table both reflect what's actually on the hardware.
 */
async function adjustPositionCorrection(direction) {
    const jointNumber = parseInt(document.getElementById('calibrationJointSelect').value, 10);
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

        await readCalibrationForJoint(jointNumber);
        renderCalibrationTable();
        setCalibrationStatus(`Joint ${jointNumber}: Position Correction is now ${newValue} step, moved to center (2048).`);
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
// reset to its factory default — see commissioningWritesForJoint() below.
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
// generic default — commissioning must never stomp it, so it's handled
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
 * instance (kinematics.js) — not a hardcoded copy of the URDF's numbers,
 * which would silently go stale the next time someone tunes a joint limit
 * there. Throws if the URDF hasn't loaded (or doesn't describe this joint),
 * rather than writing a limit computed from missing data.
 */
function commissioningAngleLimitsForJoint(jointNumber) {
    const joint = robotKinematics.joints && robotKinematics.joints[jointNumber - 1];
    const limits = joint && joint.limits;
    if (!limits || typeof limits.lowerRadians !== 'number' || typeof limits.upperRadians !== 'number') {
        throw new Error(`No joint limits available for Joint ${jointNumber} — kinematics.urdf may not have loaded yet`);
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
 * factory default — skipping addresses the Calibration page itself refuses
 * to write (ID, Baud rate, Phase — STS_WRITE_BLOCKED_ADDRESSES, from
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
 * Writes commissioningWritesForJoint()'s registers to every joint's servo —
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
        showAppMessage("Joint limits aren't loaded yet (kinematics.urdf) — wait for the app to finish starting up and try again.");
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
                // Can't tell what's already correct — fall back to writing
                // every register for this joint, same as before this change.
                failures.push(`Joint ${joint}: couldn't read current EEPROM (${error.message}) — writing every register unconditionally`);
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
                    // A bulk run touches ~150 registers across 6 servos — one
                    // transient bus hiccup must not silently abort every
                    // write after it, so record the failure and keep going.
                    failures.push(`Joint ${joint} ${addrHex} (${label}): ${error.message}`);
                }
            }
        }

        // Every joint's registers just changed — drop the whole cache rather
        // than leave other joints' tables showing stale pre-commissioning
        // bytes, then re-read the one currently on screen.
        for (const key of Object.keys(calibrationRawByJoint)) delete calibrationRawByJoint[key];
        const selectedJoint = parseInt(document.getElementById('calibrationJointSelect').value, 10);
        await readCalibrationForJoint(selectedJoint);
        renderCalibrationTable();

        if (failures.length === 0) {
            setCalibrationStatus(
                `Commissioned all ${COMMISSIONING_JOINT_COUNT} servos at ${new Date().toLocaleTimeString()}: ` +
                `${successCount} register(s) written, ${skippedCount} already correct.`
            );
        } else {
            setCalibrationStatus(
                `Commissioned ${successCount} register write(s), ${skippedCount} already correct; ` +
                `${failures.length} failed (likely a transient bus error — re-run to retry just those): ${failures.join('; ')}`
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
 * writeServoEepromRaw's own low-level bus retry — bulk commissioning touches
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
