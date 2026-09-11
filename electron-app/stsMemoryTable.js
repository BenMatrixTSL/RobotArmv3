/**
 * STS3215 EEPROM memory table — transcribed from the manufacturer's
 * datasheet (ST3215 Datasheets/sts3215_memory_table.xlsx, sheet "STS3215").
 *
 * Covers the full EEPROM control table, address 0x00-0x27 (0-39 decimal).
 * SRAM-only registers (target position, current load, torque switch, etc.)
 * are runtime state already surfaced by getStatus() and are deliberately
 * excluded here — this table is for servo *configuration*, not live status.
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

// unitType decoders: (raw) => human-readable string
const UNIT_DECODERS = {
    raw:              (raw) => String(raw),
    id:               (raw) => String(raw),
    'enum-baud':      (raw) => STS_BAUD_RATES[raw] || `unknown (${raw})`,
    'enum-response':  (raw) => STS_RESPONSE_LEVELS[raw] || `unknown (${raw})`,
    'enum-mode':      (raw) => STS_OPERATION_MODES[raw] || `unknown (${raw})`,
    step:             (raw) => `${raw} step (${(raw * 0.087890625).toFixed(1)}°)`,
    'signed-step':    decodeSignedStep,
    '°C':             (raw) => `${raw} °C`,
    '0.1V':           (raw) => `${(raw / 10).toFixed(1)} V`,
    'permille-pct':   (raw) => `${(raw / 10).toFixed(1)} %`,   // 0-1000 range = 0-100.0%
    '2us':            (raw) => `${raw * 2} µs`,
    '6.5mA':          (raw) => `${(raw * 6.5).toFixed(0)} mA`,
    '10ms':           (raw) => `${raw * 10} ms`,
    'percent-direct': (raw) => `${raw} %`,                      // value IS the percent (e.g. 20 = 20%)
    bitmask:          decodeBitmask,
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

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { STS_EEPROM_REGISTERS, decodeEepromBlock };
}
