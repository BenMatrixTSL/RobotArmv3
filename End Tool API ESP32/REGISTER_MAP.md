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
