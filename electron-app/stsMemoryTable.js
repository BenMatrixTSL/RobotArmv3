/**
 * STS3215 memory table — transcribed from the manufacturer's datasheet
 * (ST3215 Datasheets/sts3215_memory_table.xlsx, sheet "STS3215").
 *
 * Two register blocks:
 *  - STS_EEPROM_REGISTERS: address 0x00-0x27 (0-39), persists across power-off.
 *  - STS_SRAM_REGISTERS: address 0x28-0x45 (40-69), live/runtime state —
 *    resets to defaults on power-off, and overlaps what getStatus() already
 *    surfaces for some fields (position/speed/load/voltage/temperature), but
 *    this table exposes the raw register view (including ones getStatus()
 *    doesn't, like Torque Switch, Lock Mark, and the Servo Status fault bits).
 *    Read-only on the Calibration page even where the datasheet marks a
 *    field read&write — several of these registers directly command motion
 *    (Target Location, Running Speed, Torque Switch) and writing them here
 *    would bypass the app's normal move safety checks (dead zones, control
 *    session), so this table intentionally doesn't support editing them.
 *
 * Multi-byte registers are little-endian (low byte at the given address,
 * high byte at address+1), per the datasheet's "Low Front High behind" note.
 */

// Registers the Calibration page's write UI refuses to edit, even with the
// write password — wrong values here can make a servo unreachable on the bus
// (ID, Baud rate) or violate the datasheet's own "do not modify" note (Phase).
// Mirrors EEPROM_WRITE_BLOCKED_ADDRESSES in raspberry-pi-control-st3215/servoWorker.js.
const STS_WRITE_BLOCKED_ADDRESSES = new Set([5, 6, 18]); // ID, Baud rate, Phase

// Registers where the write UI asks for an extra confirmation before sending —
// these can let a joint travel somewhere mechanically unsafe.
const STS_WRITE_EXTRA_CONFIRM_ADDRESSES = new Set([9, 11]); // Min/Max angle limit

const STS_BAUD_RATES = ['1,000,000', '500,000', '250,000', '128,000', '115,200', '76,800', '57,600', '38,400'];
const STS_RESPONSE_LEVELS = ['Reply to READ/PING only', 'Reply to all instructions'];
const STS_OPERATION_MODES = ['Position servo', 'Constant speed (velocity)', 'PWM open-loop speed', 'Step servo'];
// Bit order per datasheet: Voltage, Sensor, Temperature, Current, Angle, Overload (bits 0-5)
const STS_FAULT_BIT_NAMES = ['Voltage', 'Sensor', 'Temperature', 'Current', 'Angle', 'Overload'];

function decodeBitmask(raw) {
    const active = [];
    for (let i = 0; i < STS_FAULT_BIT_NAMES.length; i++) {
        if (raw & (1 << i)) active.push(STS_FAULT_BIT_NAMES[i]);
    }
    return active.length ? active.join(', ') : 'none';
}

function decodeSignedStep(raw) {
    // Bit 11 is the sign bit; bits 0-10 are magnitude (datasheet: "Bit11 is the
    // direction bit... other bits can represent the range of 0-2047 steps").
    const magnitude = raw & 0x7FF;
    const negative = (raw & 0x800) !== 0;
    return (negative ? -magnitude : magnitude) + ' step';
}

/**
 * Inverse of decodeSignedStep: turns a signed step count (-2047..2047) into
 * this register's actual raw bit pattern (bit11 = sign, bits0-10 = magnitude).
 * NOT standard two's complement — a plain `value & 0xFF` on a negative JS
 * number produces a completely different (wrong) bit pattern for this field.
 */
function encodeSignedStep(value) {
    const magnitude = Math.abs(value) & 0x7FF;
    return value < 0 ? (0x800 | magnitude) : magnitude;
}

function decodeSignedLoad(raw) {
    // Not documented in the datasheet's own text, but confirmed against a real
    // reading: bit 10 is a direction flag, bits 0-9 are magnitude (0-1000 =
    // 0-100%) — the datasheet's plain 0.001-unit description alone produces
    // nonsensical values like "105.2%" whenever this bit happens to be set.
    const magnitude = raw & 0x3FF;
    const negative = (raw & 0x400) !== 0;
    return `${negative ? '-' : ''}${(magnitude / 10).toFixed(1)} %`;
}

function decodeSigned16(raw) {
    // Plain two's-complement 16-bit value, used by the SRAM motion-target/
    // feedback registers (as opposed to the EEPROM Position Correction
    // register's bit11-sign-plus-11-bit-magnitude scheme above).
    const signed = raw > 0x7FFF ? raw - 0x10000 : raw;
    return `${signed} step`;
}

const STS_TORQUE_SWITCH_STATES = { 0: 'Off', 1: 'On', 128: '(write-only) reset current position to center' };
const STS_LOCK_MARK_STATES = ['Unlocked — EEPROM writes persist across power-off', 'Locked — EEPROM writes are lost on power-off'];
const STS_MOVING_STATES = ['Stopped', 'Moving'];

// unitType decoders: (raw) => human-readable string
const UNIT_DECODERS = {
    raw:              (raw) => String(raw),
    id:               (raw) => String(raw),
    'enum-baud':      (raw) => STS_BAUD_RATES[raw] || `unknown (${raw})`,
    'enum-response':  (raw) => STS_RESPONSE_LEVELS[raw] || `unknown (${raw})`,
    'enum-mode':      (raw) => STS_OPERATION_MODES[raw] || `unknown (${raw})`,
    step:             (raw) => `${raw} step (${(raw * 0.087890625).toFixed(1)}°)`,
    'signed-step':    decodeSignedStep,
    'signed16-step':  decodeSigned16,
    'signed-load-pct': decodeSignedLoad,
    '°C':             (raw) => `${raw} °C`,
    '0.1V':           (raw) => `${(raw / 10).toFixed(1)} V`,
    'permille-pct':   (raw) => `${(raw / 10).toFixed(1)} %`,   // 0-1000 range = 0-100.0%
    '2us':            (raw) => `${raw * 2} µs`,
    '6.5mA':          (raw) => `${(raw * 6.5).toFixed(0)} mA`,
    '10ms':           (raw) => `${raw * 10} ms`,
    '100steps2':      (raw) => `${raw * 100} step/s²`,
    'steps-per-sec':  (raw) => `${raw} step/s`,
    'percent-direct': (raw) => `${raw} %`,                      // value IS the percent (e.g. 20 = 20%)
    bitmask:          decodeBitmask,
    'enum-torque':    (raw) => STS_TORQUE_SWITCH_STATES[raw] !== undefined ? STS_TORQUE_SWITCH_STATES[raw] : `unknown (${raw})`,
    'enum-lock-mark': (raw) => STS_LOCK_MARK_STATES[raw] || `unknown (${raw})`,
    'enum-moving':    (raw) => STS_MOVING_STATES[raw] || `unknown (${raw})`,
};

// unitType encoders: (value) => the register's actual raw bit pattern to write.
// Only needed where that differs from the plain integer a user types in (i.e.
// where UNIT_DECODERS does something other than String(raw)) — every other
// unitType is its own inverse, so it's fine that they're absent here; look up
// with `UNIT_ENCODERS[unitType] || ((v) => v)`.
const UNIT_ENCODERS = {
    'signed-step': encodeSignedStep,
};

/**
 * Full EEPROM register list, in address order.
 * @property {number} address - decimal EEPROM address
 * @property {number} bytes - 1 or 2
 * @property {string} name
 * @property {'read'|'read&write'} access
 * @property {number} default - factory initial value (raw)
 * @property {number} min
 * @property {number} max
 * @property {string} unit - label shown in the Range column
 * @property {string} unitType - key into UNIT_DECODERS
 * @property {string} description - verbatim (lightly cleaned) from the datasheet
 */
const STS_EEPROM_REGISTERS = [
    { address: 0,  bytes: 1, name: 'Firmware major version', access: 'read', default: 3,    min: -1,    max: -1,   unit: '—',      unitType: 'raw',
      description: 'Firmware major version number.' },
    { address: 1,  bytes: 1, name: 'Firmware sub version', access: 'read', default: 6,    min: -1,    max: -1,   unit: '—',      unitType: 'raw',
      description: 'Firmware minor version number.' },
    { address: 3,  bytes: 1, name: 'Servo main version', access: 'read', default: 9,    min: -1,    max: -1,   unit: '—',      unitType: 'raw',
      description: 'Servo hardware main version number.' },
    { address: 4,  bytes: 1, name: 'Servo sub version', access: 'read', default: 3,    min: -1,    max: -1,   unit: '—',      unitType: 'raw',
      description: 'Servo hardware sub version number.' },
    { address: 5,  bytes: 1, name: 'ID', access: 'read&write', default: 1,    min: 0,     max: 253,  unit: '0-253',  unitType: 'id',
      description: 'Unique identification code on the bus. Duplicate IDs are not allowed on the same bus. 254 (0xFE) is the broadcast ID — broadcasts get no reply packet.' },
    { address: 6,  bytes: 1, name: 'Baud rate', access: 'read&write', default: 0,    min: 0,     max: 7,    unit: 'enum',   unitType: 'enum-baud',
      description: 'Serial baud rate. 0-7 selects 1,000,000 / 500,000 / 250,000 / 128,000 / 115,200 / 76,800 / 57,600 / 38,400 bps.' },
    { address: 7,  bytes: 1, name: 'Return delay', access: 'read&write', default: 0,    min: 0,     max: 254,  unit: '2µs',    unitType: '2us',
      description: 'Delay before the servo replies to an instruction. Minimum unit 2µs; max settable delay is 254 × 2 = 508µs.' },
    { address: 8,  bytes: 1, name: 'Response status level', access: 'read&write', default: 1,    min: 0,     max: 1,    unit: 'enum',   unitType: 'enum-response',
      description: '0: only READ and PING instructions get a reply packet. 1: every instruction gets a reply packet.' },
    { address: 9,  bytes: 2, name: 'Min angle limit', access: 'read&write', default: 0,    min: 0,     max: 4094, unit: 'step',   unitType: 'step',
      description: 'Minimum travel limit. Must be less than the max angle limit. Set to 0 for multi-turn absolute position control.' },
    { address: 11, bytes: 2, name: 'Max angle limit', access: 'read&write', default: 4095, min: 1,     max: 4095, unit: 'step',   unitType: 'step',
      description: 'Maximum travel limit. Must be greater than the min angle limit. Set to 0 for multi-turn absolute position control.' },
    { address: 13, bytes: 1, name: 'Max temperature limit', access: 'read&write', default: 70,   min: 0,     max: 100,  unit: '°C',     unitType: '°C',
      description: 'Maximum operating temperature. E.g. 70 = servo cuts off at 70°C. Accuracy 1°C.' },
    { address: 14, bytes: 1, name: 'Max input voltage', access: 'read&write', default: 80,   min: 0,     max: 254,  unit: '0.1V',   unitType: '0.1V',
      description: 'Maximum working voltage. E.g. 80 = 8.0V limit. Accuracy 0.1V.' },
    { address: 15, bytes: 1, name: 'Min input voltage', access: 'read&write', default: 40,   min: 0,     max: 254,  unit: '0.1V',   unitType: '0.1V',
      description: 'Minimum working voltage. E.g. 40 = 4.0V limit. Accuracy 0.1V.' },
    { address: 16, bytes: 2, name: 'Max torque', access: 'read&write', default: 1000, min: 0,     max: 1000, unit: '0-100%', unitType: 'permille-pct',
      description: 'Overall output torque ceiling. 1000 = 100% of stall torque. Copied to the runtime torque-limit register (address 48) at power-on.' },
    { address: 18, bytes: 1, name: 'Phase (special byte)', access: 'read&write', default: 12,   min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Special function byte — do not modify without a specific reason. See the datasheet\'s "special byte bit analysis" for details.' },
    { address: 19, bytes: 1, name: 'Unloading condition', access: 'read&write', default: 44,   min: 0,     max: 254,  unit: 'bitmask', unitType: 'bitmask',
      description: 'Which fault conditions cut torque when tripped. Bits 0-5 = Voltage, Sensor, Temperature, Current, Angle, Overload — setting a bit enables that protection.' },
    { address: 20, bytes: 1, name: 'LED alarm condition', access: 'read&write', default: 47,   min: 0,     max: 254,  unit: 'bitmask', unitType: 'bitmask',
      description: 'Which fault conditions flash the status LED. Same bit layout as Unloading Condition (Voltage, Sensor, Temperature, Current, Angle, Overload).' },
    { address: 21, bytes: 1, name: 'P coefficient', access: 'read&write', default: 32,   min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Position-loop proportional gain. Higher = stiffer/faster correction, but more overshoot and current draw.' },
    { address: 22, bytes: 1, name: 'D coefficient', access: 'read&write', default: 32,   min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Position-loop derivative gain. Damps oscillation from the P term.' },
    { address: 23, bytes: 1, name: 'I coefficient', access: 'read&write', default: 0,    min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Position-loop integral gain. Corrects steady-state error, but a nonzero value against a constant load (e.g. gravity) can drive sustained current draw — the most likely EEPROM setting to cause continuous overcurrent / brownout if raised from its factory 0.' },
    { address: 24, bytes: 2, name: 'Min startup force', access: 'read&write', default: 16,   min: 0,     max: 1000, unit: '0-100%', unitType: 'permille-pct',
      description: 'Minimum output torque needed to start moving. 1000 = 100% of stall torque. NOTE: this is a 2-byte register (address 24-25) — code that writes it as a single byte only ever sets the low byte.' },
    { address: 26, bytes: 1, name: 'CW insensitive area', access: 'read&write', default: 1,    min: 0,     max: 32,   unit: 'step',   unitType: 'step',
      description: 'Clockwise dead zone around the target position, in units of the minimum resolution angle.' },
    { address: 27, bytes: 1, name: 'CCW insensitive area', access: 'read&write', default: 1,    min: 0,     max: 32,   unit: 'step',   unitType: 'step',
      description: 'Counter-clockwise dead zone around the target position, in units of the minimum resolution angle.' },
    { address: 28, bytes: 2, name: 'Protection current', access: 'read&write', default: 500,  min: 0,     max: 511,  unit: '6.5mA',  unitType: '6.5mA',
      description: 'Overcurrent protection threshold. Max settable current is 511 × 6.5mA ≈ 3255mA.' },
    { address: 30, bytes: 1, name: 'Angular resolution', access: 'read&write', default: 1,    min: 1,     max: 100,  unit: '×',      unitType: 'raw',
      description: 'Multiplier on the minimum resolution angle (degrees/step). Increasing it extends the number of controllable turns.' },
    { address: 31, bytes: 2, name: 'Position correction', access: 'read&write', default: 0,    min: -2047, max: 2047, unit: '±step',  unitType: 'signed-step',
      description: 'Zero-point calibration offset. Bit 11 is the sign bit; the remaining bits give a magnitude of 0-2047 steps.' },
    { address: 33, bytes: 1, name: 'Operation mode', access: 'read&write', default: 0,    min: 0,     max: 2,    unit: 'enum',   unitType: 'enum-mode',
      description: '0 = position servo. 1 = constant speed (velocity), controlled via address 0x2E, bit15 = direction. 2 = PWM open-loop, controlled via address 0x2C, bit11 = direction. 3 = step servo, controlled via address 0x2A, bit15 = direction.' },
    { address: 34, bytes: 1, name: 'Protective torque', access: 'read&write', default: 20,   min: 0,     max: 254,  unit: '%',      unitType: 'percent-direct',
      description: 'Torque used AFTER an overload trip. E.g. 20 = 20% of max torque — the servo goes limp-ish rather than fully unpowered.' },
    { address: 35, bytes: 1, name: 'Protection time', access: 'read&write', default: 200,  min: 0,     max: 254,  unit: '10ms',   unitType: '10ms',
      description: 'How long the load must stay above the overload-torque threshold before the fault actually trips. E.g. 200 = 2 seconds; max ~2.5s.' },
    { address: 36, bytes: 1, name: 'Overload torque', access: 'read&write', default: 80,   min: 0,     max: 254,  unit: '%',      unitType: 'percent-direct',
      description: 'Load threshold (as % of max torque) that counts as a stall/overload and starts the Protection Time countdown. E.g. 80 = 80%.' },
    { address: 37, bytes: 1, name: 'Speed closed-loop P coefficient', access: 'read&write', default: 10,   min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Velocity-loop proportional gain, used only in constant-speed mode (operation mode 1).' },
    { address: 38, bytes: 1, name: 'Over-current protection time', access: 'read&write', default: 200,  min: 0,     max: 254,  unit: '10ms',   unitType: '10ms',
      description: 'How long current must exceed the Protection Current threshold before tripping. Max 254 × 10ms = 2540ms.' },
    { address: 39, bytes: 1, name: 'Velocity closed-loop I coefficient', access: 'read&write', default: 10,   min: 0,     max: 254,  unit: '—',      unitType: 'raw',
      description: 'Velocity-loop integral gain, used only in constant-speed mode (operation mode 1).' },
];

/**
 * SRAM (live/runtime) register list, in address order. See the file header
 * for why this page treats every one of these as read-only regardless of the
 * datasheet's own access column.
 */
const STS_SRAM_REGISTERS = [
    { address: 40, bytes: 1, name: 'Torque switch', access: 'read&write', default: 0, min: 0, max: 2, unit: 'enum', unitType: 'enum-torque',
      description: 'Whether the servo is actively holding/driving torque right now. Write 128 (not readable back as such) recenters the current position to 2048.' },
    { address: 41, bytes: 1, name: 'Acceleration', access: 'read&write', default: 0, min: 0, max: 254, unit: '100 step/s²', unitType: '100steps2',
      description: 'Current move\'s acceleration/deceleration rate.' },
    { address: 42, bytes: 2, name: 'Target location', access: 'read&write', default: 0, min: -32766, max: 32766, unit: '±step', unitType: 'signed16-step',
      description: 'The goal position of the current or most recent move, in position-servo mode.' },
    { address: 44, bytes: 2, name: 'Running time', access: 'read&write', default: 0, min: 0, max: 1000, unit: '0-100%', unitType: 'permille-pct',
      description: 'Run-time parameter for PWM open-loop mode (operation mode 2).' },
    { address: 46, bytes: 2, name: 'Running speed', access: 'read&write', default: 0, min: 0, max: 254, unit: 'step/s', unitType: 'steps-per-sec',
      description: 'Goal speed for the current or most recent move. 50 step/s ≈ 0.732 RPM.' },
    { address: 48, bytes: 2, name: 'Torque limit', access: 'read&write', default: 1000, min: 0, max: 1000, unit: '0-100%', unitType: 'permille-pct',
      description: 'Live torque ceiling — initialized from EEPROM Max Torque (address 0x10) at power-on, but can be changed at runtime without touching EEPROM.' },
    { address: 55, bytes: 1, name: 'Lock mark', access: 'read&write', default: 0, min: 0, max: 1, unit: 'enum', unitType: 'enum-lock-mark',
      description: 'EEPROM write-protect state. The commissioning/calibration write paths unlock this (write 0) immediately before every EEPROM write and leave it unlocked.' },
    { address: 56, bytes: 2, name: 'Current location', access: 'read', default: -1, min: -1, max: -1, unit: 'step', unitType: 'signed16-step',
      description: 'The servo\'s present position feedback (same value getStatus() reports as position/angleDegrees).' },
    { address: 58, bytes: 2, name: 'Current speed', access: 'read', default: -1, min: -1, max: -1, unit: 'step/s', unitType: 'steps-per-sec',
      description: 'The servo\'s present rotational speed feedback.' },
    { address: 60, bytes: 2, name: 'Current load', access: 'read', default: -1, min: -1, max: -1, unit: '±0-100%', unitType: 'signed-load-pct',
      description: 'Voltage duty cycle currently being applied to drive the motor, signed by direction — a proxy for how hard it\'s working (and holding, e.g. against gravity, even while stationary).' },
    { address: 62, bytes: 1, name: 'Current voltage', access: 'read', default: -1, min: -1, max: -1, unit: '0.1V', unitType: '0.1V',
      description: 'The servo\'s present supply voltage feedback.' },
    { address: 63, bytes: 1, name: 'Current temperature', access: 'read', default: -1, min: -1, max: -1, unit: '°C', unitType: '°C',
      description: 'The servo\'s present internal temperature feedback.' },
    { address: 64, bytes: 1, name: 'Asynchronous write flag', access: 'read', default: -1, min: -1, max: -1, unit: '—', unitType: 'raw',
      description: 'Set while an async-write instruction is pending action.' },
    { address: 65, bytes: 1, name: 'Servo status', access: 'read', default: -1, min: -1, max: -1, unit: 'bitmask', unitType: 'bitmask',
      description: 'Currently active fault flags. Same bit layout as Unloading Condition/LED Alarm Condition (Voltage, Sensor, Temperature, Current, Angle, Overload) — a set bit here means that fault is presently occurring.' },
    { address: 66, bytes: 1, name: 'Mobile sign', access: 'read', default: -1, min: -1, max: -1, unit: 'enum', unitType: 'enum-moving',
      description: 'Whether the servo is currently moving (same as getStatus()\'s isMoving).' },
    { address: 69, bytes: 2, name: 'Current current', access: 'read', default: -1, min: -1, max: -1, unit: '6.5mA', unitType: '6.5mA',
      description: 'The servo\'s present current-draw feedback. Max measurable is 500 × 6.5mA = 3250mA — directly relevant if you\'re chasing an overcurrent/brownout issue.' },
];

/**
 * Converts a value the user typed in (matching the register's displayed
 * min/max, e.g. -100 for Position Correction) into the actual raw integer
 * to write to the servo. Identity for every register except the handful with
 * a non-standard bit encoding (see UNIT_ENCODERS).
 * @param {object} reg - a register definition (from STS_EEPROM_REGISTERS)
 * @param {number} value
 * @returns {number} raw register value, ready to split into bytes and write
 */
function encodeRegisterValue(reg, value) {
    const encoder = UNIT_ENCODERS[reg.unitType];
    return encoder ? encoder(value) : value;
}

/**
 * Decodes a raw EEPROM byte block (as read from address 0, length 40) into
 * per-register raw values, meaningful values, and default-deviation flags.
 * @param {number[]} bytes - raw bytes for addresses 0x00-0x27 (40 bytes)
 * @returns {Array<object>} one entry per register in STS_EEPROM_REGISTERS
 */
function decodeEepromBlock(bytes) {
    return STS_EEPROM_REGISTERS.map((reg) => {
        let raw;
        if (reg.bytes === 2) {
            raw = (bytes[reg.address] || 0) | ((bytes[reg.address + 1] || 0) << 8);
        } else {
            raw = bytes[reg.address] || 0;
        }
        const decoder = UNIT_DECODERS[reg.unitType] || UNIT_DECODERS.raw;
        let meaningful;
        try {
            meaningful = decoder(raw);
        } catch (e) {
            meaningful = `(decode error: ${e.message})`;
        }
        const defaultMeaningful = reg.access === 'read' ? '—' : decoder(reg.default);
        return {
            ...reg,
            raw,
            meaningful,
            defaultMeaningful,
            // Read-only hardware identity fields (firmware/servo version numbers)
            // always legitimately differ from a generic default — that's not a
            // "changed setting", so never flag them.
            isDefault: reg.access === 'read' ? true : raw === reg.default,
            writable: reg.access === 'read&write' && !STS_WRITE_BLOCKED_ADDRESSES.has(reg.address),
            extraConfirm: STS_WRITE_EXTRA_CONFIRM_ADDRESSES.has(reg.address),
        };
    });
}

/**
 * Decodes a raw SRAM byte block (as read from address 40, length 31) into
 * per-register raw and meaningful values. No default/deviation flags — this
 * is live state, not persistent configuration, so "differs from default"
 * isn't a meaningful question (e.g. Torque Switch is expected to read 1
 * whenever the arm is armed).
 * @param {number[]} bytes - raw bytes for addresses 0x28-0x45 (31 bytes), i.e. bytes[0] is address 40
 * @returns {Array<object>} one entry per register in STS_SRAM_REGISTERS
 */
function decodeSramBlock(bytes) {
    const base = 40;
    return STS_SRAM_REGISTERS.map((reg) => {
        const offset = reg.address - base;
        let raw;
        if (reg.bytes === 2) {
            raw = (bytes[offset] || 0) | ((bytes[offset + 1] || 0) << 8);
        } else {
            raw = bytes[offset] || 0;
        }
        const decoder = UNIT_DECODERS[reg.unitType] || UNIT_DECODERS.raw;
        let meaningful;
        try {
            meaningful = decoder(raw);
        } catch (e) {
            meaningful = `(decode error: ${e.message})`;
        }
        return { ...reg, raw, meaningful, writable: false };
    });
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { STS_EEPROM_REGISTERS, STS_SRAM_REGISTERS, decodeEepromBlock, decodeSramBlock, encodeRegisterValue };
}
