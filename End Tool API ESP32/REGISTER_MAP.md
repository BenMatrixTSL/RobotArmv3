# End Tool Register Map (Current Project)

This is the current byte-level register map used by `RobotArmv3_End_Tool_API.fcfx`.

All multi-byte values are little-endian:
- `*_L` = low byte
- `*_H` = high byte

## Identity and Status

- `0` - Protocol version (RO)
- `1` - Firmware major (RO)
- `2` - Firmware minor (RO)
- `3` - Tool type ID (RW)
- `4` - Status flags (RW)

## PWM Control

- `5` - PWM1 duty 0-255 (RW)
- `6` - PWM2 duty 0-255 (RW)
- `7` - PWM control flags (RW)
  - bit0 = PWM1 enable
  - bit1 = PWM2 enable

## Current Measurements (ADS1220, mA)

- `11` - PWM1 current low byte (RO)
- `12` - PWM1 current high byte (RO)
- `13` - PWM2 current low byte (RO)
- `14` - PWM2 current high byte (RO)
- `15` - Servo current low byte (RO)
- `16` - Servo current high byte (RO)

Reconstruct:
- `current1mA = (reg12 << 8) | reg11`
- `current2mA = (reg14 << 8) | reg13`
- `current3mA = (reg16 << 8) | reg15`

## ADC Telemetry

- `17` - ADC0 raw low byte (RO)
- `18` - ADC0 raw high byte (RO)
- `19` - ADC1 raw low byte (RO)
- `20` - ADC1 raw high byte (RO)
- `21` - ADC0 millivolts low byte (RO)
- `22` - ADC0 millivolts high byte (RO)
- `23` - ADC1 millivolts low byte (RO)
- `24` - ADC1 millivolts high byte (RO)

Reconstruct:
- `adc0Raw = (reg18 << 8) | reg17`
- `adc1Raw = (reg20 << 8) | reg19`
- `adc0mV = (reg22 << 8) | reg21`
- `adc1mV = (reg24 << 8) | reg23`

## Hobby Servo Output (Servo_Controller1, channel 0)

Addresses match `ST3215_ESP32_END_TOOL_API.md` (hex) and the Pi server (`robotArmST3215.js`).

| Address | Name | R/W | Description |
|--------:|------|-----|-------------|
| 48 (`0x30`) | `SERVO_ENABLE` | RW | `0` = disabled, `1` = enabled |
| 49 (`0x31`) | `SERVO_POSITION_8BIT` | RW | Position `0`–`255` |
| 50 (`0x32`) | `SERVO_ANGLE_DEG` | RW | Angle `0`–`180` degrees |
| 51 (`0x33`) | `SERVO_PULSE_MIN_L` | RW | Min pulse width low byte (default 1000 µs) |
| 52 | `SERVO_PULSE_MIN_H` | RW | Min pulse width high byte |
| 53 (`0x35`) | `SERVO_PULSE_MAX_L` | RW | Max pulse width low byte (default 2000 µs) |
| 54 | `SERVO_PULSE_MAX_H` | RW | Max pulse width high byte |
| 55 (`0x37`) | `SERVO_CURRENT_POSITION_8BIT` | R | Last applied 8-bit position |
| 56 (`0x38`) | `SERVO_CURRENT_ANGLE` | R | Last applied angle (degrees) |

### Behaviour

- **READ** supports 1–8 bytes in one command (for example read 3 bytes from address 48).
- **WRITE** copies every data byte into consecutive registers (for example enable + angle in one packet).
- If the last write touched register **50**, `MoveToAngle` runs; if **49**, `SetPosition` runs; otherwise the current angle is applied.
- Servo is **disabled** at boot until `SERVO_ENABLE = 1`.
- Firmware minor version is **2** when hobby servo support is present.

### WebSocket commands (Pi server)

- `toolSetServoEnabled` → register 48
- `toolSetServoPosition` → register 49
- `toolSetServoAngle` → register 50
- `toolGetServoState` → read registers 48–50 (enable, position, angle)
