/**
 * ESP32 end-tool (bus ID 64) register map - transcribed from
 * End Tool API ESP32/REGISTER_MAP.md, which documents the register layout the
 * firmware in RobotArmv3_End_Tool_API.c actually implements (the older
 * ST3215_ESP32_END_TOOL_API.md design doc lists extra registers - watchdog,
 * config flags, reset/fault-clear commands, uptime - that this firmware does
 * not implement; they aren't listed here because reading them just returns
 * 0 and writing them does nothing).
 *
 * Unlike an ST3215 servo the tool has one flat 64-byte register array in RAM
 * (addresses 0-63). Only one register persists across a power cycle: the
 * Tool type ID (address 3), which the firmware saves to its EEPROM whenever
 * it changes. The Calibration page therefore shows two blocks:
 *  - END_TOOL_EEPROM_REGISTERS: identity (read-only firmware constants) plus
 *    the persisted Tool type ID.
 *  - END_TOOL_RAM_REGISTERS: everything else - outputs, telemetry and hobby
 *    servo settings - lost on power-off. Unlike servo SRAM these ARE
 *    writable from the Calibration page (behind the same password gate):
 *    writing them is exactly how the tool is driven, and there is no
 *    arm-motion safety concern in doing so.
 *
 * Multi-byte registers are little-endian (low byte at the given address,
 * high byte at address+1), same as the servos.
 */

const END_TOOL_READ_BLOCK_LENGTH = 64;

// Address of the only register the firmware persists to EEPROM.
const END_TOOL_TOOL_TYPE_ADDRESS = 3;

const END_TOOL_TYPE_NAMES = { 0: 'unassigned', 1: 'pneumatic vacuum + valve', 2: 'servo motor', 3: 'pen' };

// Bit order per ST3215_ESP32_END_TOOL_API.md section 4.1 (status flags) and
// REGISTER_MAP.md (PWM control bits).
const END_TOOL_STATUS_BIT_NAMES = ['PWM1 overcurrent', 'PWM2 overcurrent', 'ADC1 out of range', 'ADC2 out of range',
    'Servo fault', 'Watchdog active', 'Outputs forced off', 'Reserved'];
const END_TOOL_PWM_CONTROL_BIT_NAMES = ['PWM1 enable', 'PWM2 enable'];

function decodeEndToolBits(raw, names) {
    const active = [];
    for (let i = 0; i < names.length; i++) {
        if (raw & (1 << i)) active.push(names[i]);
    }
    return active.length ? active.join(', ') : 'none';
}

const END_TOOL_UNIT_DECODERS = {
    raw:            (raw) => String(raw),
    'enum-tool':    (raw) => END_TOOL_TYPE_NAMES[raw] !== undefined ? `${raw} = ${END_TOOL_TYPE_NAMES[raw]}` : `${raw} (no URDF entry)`,
    'status-bits':  (raw) => decodeEndToolBits(raw, END_TOOL_STATUS_BIT_NAMES),
    'pwm-bits':     (raw) => decodeEndToolBits(raw, END_TOOL_PWM_CONTROL_BIT_NAMES),
    'duty-255':     (raw) => `${raw} (${(raw / 255 * 100).toFixed(0)} %)`,
    'enum-enable':  (raw) => raw === 0 ? 'Disabled' : raw === 1 ? 'Enabled' : `unknown (${raw})`,
    mA:             (raw) => `${raw} mA`,
    mV:             (raw) => `${raw} mV`,
    us:             (raw) => `${raw} µs`,
    deg:            (raw) => `${raw}°`,
    'slew-rate':    (raw) => raw === 0 ? '0 = jump immediately' : `${raw} steps/tick (full travel ≈ ${(255 / raw * 0.01).toFixed(2)} s)`,
};

/**
 * Register definitions share the same shape as STS_EEPROM_REGISTERS in
 * stsMemoryTable.js so calibrationPage.js can render either device with the
 * same table code. `default` is only meaningful for read&write registers -
 * it's the firmware's power-on value.
 */
const END_TOOL_EEPROM_REGISTERS = [
    { address: 0, bytes: 1, name: 'Protocol version', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: 'Bus protocol version implemented by the end-tool firmware.' },
    { address: 1, bytes: 1, name: 'Firmware major', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: 'End-tool firmware major version.' },
    { address: 2, bytes: 1, name: 'Firmware minor', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: 'End-tool firmware minor version. 2 or higher means hobby servo support is present; 3 or higher adds the servo slew rate register (57).' },
    { address: 3, bytes: 1, name: 'Tool type ID', access: 'read&write', default: 0, min: 0, max: 255, unit: ' - ', unitType: 'enum-tool',
      description: 'Which tool is fitted: 0 = unassigned, 1 = pneumatic vacuum + valve, 2 = servo motor, 3 = pen. Saved to the tool\'s EEPROM when changed. The Pi reads this every 15 s and applies the matching <end_tool> entry from kinematics.urdf, which sets the tool length used by FK/IK.' },
];

const END_TOOL_RAM_REGISTERS = [
    { address: 4, bytes: 1, name: 'Status flags', access: 'read&write', default: 0, min: 0, max: 255, unit: 'bits', unitType: 'status-bits',
      description: 'Fault/state bit flags: bit0 PWM1 overcurrent, bit1 PWM2 overcurrent, bit2 ADC1 out of range, bit3 ADC2 out of range, bit4 servo fault, bit5 watchdog active, bit6 outputs forced off.' },
    { address: 5, bytes: 1, name: 'PWM1 duty', access: 'read&write', default: 0, min: 0, max: 255, unit: ' - ', unitType: 'duty-255',
      description: 'PWM output 1 duty cycle, 0-255. Only drives the output when bit0 of PWM control is set.' },
    { address: 6, bytes: 1, name: 'PWM2 duty', access: 'read&write', default: 0, min: 0, max: 255, unit: ' - ', unitType: 'duty-255',
      description: 'PWM output 2 duty cycle, 0-255. Only drives the output when bit1 of PWM control is set.' },
    { address: 7, bytes: 1, name: 'PWM control', access: 'read&write', default: 0, min: 0, max: 3, unit: 'bits', unitType: 'pwm-bits',
      description: 'bit0 = PWM1 output enable, bit1 = PWM2 output enable.' },
    { address: 11, bytes: 2, name: 'PWM1 current', access: 'read', default: 0, min: -1, max: -1, unit: 'mA', unitType: 'mA',
      description: 'Measured current on PWM output 1 (ADS1220), milliamps.' },
    { address: 13, bytes: 2, name: 'PWM2 current', access: 'read', default: 0, min: -1, max: -1, unit: 'mA', unitType: 'mA',
      description: 'Measured current on PWM output 2 (ADS1220), milliamps.' },
    { address: 15, bytes: 2, name: 'Servo current', access: 'read', default: 0, min: -1, max: -1, unit: 'mA', unitType: 'mA',
      description: 'Measured hobby servo supply current (ADS1220), milliamps.' },
    { address: 17, bytes: 2, name: 'ADC0 raw', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: 'Raw ADC reading on analog input 0.' },
    { address: 19, bytes: 2, name: 'ADC1 raw', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: 'Raw ADC reading on analog input 1.' },
    { address: 21, bytes: 2, name: 'ADC0 voltage', access: 'read', default: 0, min: -1, max: -1, unit: 'mV', unitType: 'mV',
      description: 'Analog input 0, millivolts.' },
    { address: 23, bytes: 2, name: 'ADC1 voltage', access: 'read', default: 0, min: -1, max: -1, unit: 'mV', unitType: 'mV',
      description: 'Analog input 1, millivolts.' },
    { address: 48, bytes: 1, name: 'Servo enable', access: 'read&write', default: 0, min: 0, max: 1, unit: ' - ', unitType: 'enum-enable',
      description: 'Hobby servo output: 0 = disabled (no pulses), 1 = enabled. Disabled at boot until set.' },
    { address: 49, bytes: 1, name: 'Servo position (8-bit)', access: 'read&write', default: 0, min: 0, max: 255, unit: ' - ', unitType: 'raw',
      description: 'Hobby servo target as 0-255 across the pulse min..max range. Writing this moves the servo (if enabled).' },
    { address: 50, bytes: 1, name: 'Servo angle', access: 'read&write', default: 0, min: 0, max: 180, unit: '°', unitType: 'deg',
      description: 'Hobby servo target angle, 0-180°. Writing this moves the servo (if enabled).' },
    { address: 51, bytes: 2, name: 'Servo pulse min', access: 'read&write', default: 1000, min: 500, max: 2500, unit: 'µs', unitType: 'us',
      description: 'Pulse width that corresponds to position 0 / angle 0° (default 1000 µs).' },
    { address: 53, bytes: 2, name: 'Servo pulse max', access: 'read&write', default: 2000, min: 500, max: 2500, unit: 'µs', unitType: 'us',
      description: 'Pulse width that corresponds to position 255 / angle 180° (default 2000 µs).' },
    { address: 55, bytes: 1, name: 'Servo current position (8-bit)', access: 'read', default: 0, min: -1, max: -1, unit: ' - ', unitType: 'raw',
      description: '8-bit position currently applied to the hobby servo output. While a slew is in progress this walks from the old position to the new target.' },
    { address: 56, bytes: 1, name: 'Servo current angle', access: 'read', default: 0, min: -1, max: -1, unit: '°', unitType: 'deg',
      description: 'Angle currently applied to the hobby servo output (derived from register 55).' },
    { address: 57, bytes: 1, name: 'Servo slew rate', access: 'read&write', default: 0, min: 0, max: 255, unit: 'steps/tick', unitType: 'slew-rate',
      description: 'How fast the hobby servo output moves toward a new position/angle: 8-bit position steps per ~10 ms firmware tick. 0 = jump straight to the target (original behaviour). 1 = full 0-255 travel in about 2.5 s, 5 = about 0.5 s, 25 = about 0.1 s. A move made while the output is disabled always jumps (its real position is unknown). Needs end-tool firmware minor version 3 or later; older firmware reads 0 and ignores writes.' },
];

function decodeEndToolRegisters(registers, bytes) {
    return registers.map((reg) => {
        const raw = reg.bytes === 2
            ? (bytes[reg.address] || 0) | ((bytes[reg.address + 1] || 0) << 8)
            : (bytes[reg.address] || 0);
        const decoder = END_TOOL_UNIT_DECODERS[reg.unitType] || END_TOOL_UNIT_DECODERS.raw;
        let meaningful;
        try {
            meaningful = decoder(raw);
        } catch (e) {
            meaningful = `(decode error: ${e.message})`;
        }
        const readOnly = reg.access === 'read';
        return {
            ...reg,
            raw,
            meaningful,
            defaultMeaningful: readOnly ? ' - ' : decoder(reg.default),
            isDefault: readOnly ? true : raw === reg.default,
            writable: !readOnly,
            extraConfirm: false,
        };
    });
}

/**
 * Decodes the tool's full 64-byte register block (as returned by
 * readEndToolRegisters) into the EEPROM/identity rows.
 */
function decodeEndToolEepromBlock(bytes) {
    return decodeEndToolRegisters(END_TOOL_EEPROM_REGISTERS, bytes);
}

/** Same block, RAM rows. */
function decodeEndToolRamBlock(bytes) {
    return decodeEndToolRegisters(END_TOOL_RAM_REGISTERS, bytes);
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { END_TOOL_EEPROM_REGISTERS, END_TOOL_RAM_REGISTERS, END_TOOL_READ_BLOCK_LENGTH,
        decodeEndToolEepromBlock, decodeEndToolRamBlock };
}
