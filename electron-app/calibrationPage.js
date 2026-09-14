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
 * validate the password itself). The Commissioning panel (per-joint PID/
 * overload/torque profile) lives at the bottom of this page and is only
 * revealed once that same password has been entered via "Enable Editing".
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

    calibrationWriteModeEnabled = true;
    button.textContent = 'Disable Editing';
    button.classList.remove('btn-warning');
    button.classList.add('btn-danger');
    if (editHeader) editHeader.hidden = false;
    if (commissioningSection) commissioningSection.hidden = false;
    renderCalibrationTable();
}

function renderCalibrationTable() {
    renderEepromTable();
    renderSramTable();
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

// Fixed defaults written to every servo by the Commissioning button — for
// bringing a freshly replaced or factory-reset servo back to the arm's
// known-good configuration in one pass.
const COMMISSIONING_DEFAULTS = [
    { address: 0x0E, value: 140, label: 'Max input voltage' },
    { address: 0x17, value: 10, label: 'I coefficient' },
    { address: 0x1C, value: 100, label: 'Protection current' },
];
const COMMISSIONING_JOINT_COUNT = 6;

/**
 * Writes the fixed COMMISSIONING_DEFAULTS registers to every joint's servo,
 * one register at a time (reusing the raw EEPROM write used by the table
 * above), so a replacement or factory-reset servo can be brought back to the
 * arm's known-good configuration with a single button press.
 */
async function commissionAllServos() {
    const password = document.getElementById('calibrationWritePassword').value;
    if (!password) {
        showAppMessage('Enter the control-lock password to enable editing.');
        return;
    }

    const summary = COMMISSIONING_DEFAULTS
        .map(d => `0x${d.address.toString(16).toUpperCase()} (${d.label}) = ${d.value}`)
        .join('\n');
    const confirmed = await showConfirm(
        `Write default values to all ${COMMISSIONING_JOINT_COUNT} servos?\n\n${summary}\n\n` +
        `This writes directly to EEPROM on every joint, immediately.`
    );
    if (!confirmed) return;

    const button = document.getElementById('commissionAllButton');
    if (button) button.disabled = true;
    try {
        for (let joint = 1; joint <= COMMISSIONING_JOINT_COUNT; joint++) {
            for (const { address, value, label } of COMMISSIONING_DEFAULTS) {
                const addrHex = '0x' + address.toString(16).toUpperCase();
                setCalibrationStatus(`Joint ${joint}: writing ${label} (${addrHex}) = ${value}...`);
                await robotArmClient.writeServoEepromRaw(joint, address, value, password);
            }
        }
        setCalibrationStatus(`Commissioned all ${COMMISSIONING_JOINT_COUNT} servos with default values at ${new Date().toLocaleTimeString()}.`);
    } catch (error) {
        setCalibrationStatus(`Commissioning failed: ${error.message}`);
    } finally {
        if (button) button.disabled = false;
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
