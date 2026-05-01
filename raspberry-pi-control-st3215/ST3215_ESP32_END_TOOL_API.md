# ST3215-Compatible ESP32 End Tool API (ID 64)

This document defines a bus protocol for an ESP32 end-tool node that shares the same serial bus as ST3215 servos.

Goal:

- keep wire protocol compatible with existing ST3215 packet framing
- add end-tool functionality (PWM FETs, current sensing, ADC, hobby servo output, tool ID)
- avoid affecting any ST3215 servo behavior on the bus

---

## 1) Bus Compatibility Rules

### 1.1 Device ID

- Fixed ESP32 end-tool ID: `64` (`0x40`)
- This provides separation from your current servo IDs.

### 1.2 Packet format

Use the same packet frame as ST3215:

1. Header 1: `0xFF`
2. Header 2: `0xFF`
3. ID
4. LENGTH
5. INSTRUCTION (TX) / ERROR (RX)
6. PARAMETERS...
7. CHECKSUM

### 1.3 Instructions

Use only standard instructions:

- `PING = 0x01`
- `READ = 0x02`
- `WRITE = 0x03`

### 1.4 Endianness

All multi-byte fields are **little-endian**:

- 16-bit value = `[lowByte, highByte]`

### 1.5 Checksum

Same as ST3215:

- `checksum = (~sum(ID..lastDataByte)) & 0xFF`

---

## 2) Why This Will Not Affect Servos

1. Servos only respond to their own IDs.
2. ESP32 only responds to ID `64`.
3. Existing servo register map is untouched.
4. Host can route commands by ID:
   - servo commands -> IDs `1..6` (or your configured set)
   - tool commands -> ID `64` only

---

## 3) Register Map (ESP32 Tool Node, ID 64)

Addresses below are local to the ESP32 node implementation.

## 3.1 EEPROM-like / identity/config region

| Address | Name | Size | R/W | Description |
|---|---|---:|---|---|
| `0x00` | `TOOL_PROTOCOL_VERSION` | 1 | R | Protocol version byte |
| `0x01` | `TOOL_FIRMWARE_MAJOR` | 1 | R | Firmware major |
| `0x02` | `TOOL_FIRMWARE_MINOR` | 1 | R | Firmware minor |
| `0x03` | `TOOL_TYPE_ID` | 1 | R/W | End-tool type identifier |
| `0x04` | `TOOL_STATUS_FLAGS` | 1 | R | Bit flags for faults/state |
| `0x05` | `TOOL_CONFIG_FLAGS` | 1 | R/W | Behavior flags (startup, watchdog, etc.) |
| `0x06` | `WATCHDOG_TIMEOUT_L` | 2 | R/W | Tool-safe timeout in ms (`0` disables) |

## 3.2 PWM FET outputs and current sensing

| Address | Name | Size | R/W | Description |
|---|---|---:|---|---|
| `0x10` | `PWM1_DUTY` | 1 | R/W | PWM1 duty `0..255` |
| `0x11` | `PWM2_DUTY` | 1 | R/W | PWM2 duty `0..255` |
| `0x12` | `PWM_CONTROL` | 1 | R/W | Output enable and mode bits |
| `0x13` | `PWM_FREQ_DIV` | 1 | R/W | Optional coarse frequency selector |
| `0x14` | `PWM1_CURRENT_RAW_L` | 2 | R | Current sensor 1 raw ADC (or scaled) |
| `0x16` | `PWM2_CURRENT_RAW_L` | 2 | R | Current sensor 2 raw ADC (or scaled) |
| `0x18` | `PWM1_CURRENT_LIMIT_L` | 2 | R/W | Overcurrent limit channel 1 |
| `0x1A` | `PWM2_CURRENT_LIMIT_L` | 2 | R/W | Overcurrent limit channel 2 |

## 3.3 ADC inputs

| Address | Name | Size | R/W | Description |
|---|---|---:|---|---|
| `0x20` | `ADC1_RAW_L` | 2 | R | ADC1 raw reading |
| `0x22` | `ADC2_RAW_L` | 2 | R | ADC2 raw reading |
| `0x24` | `ADC1_MV_L` | 2 | R | ADC1 voltage in mV |
| `0x26` | `ADC2_MV_L` | 2 | R | ADC2 voltage in mV |
| `0x28` | `ADC1_RESISTANCE_L` | 2 | R | ADC1 computed resistance (optional) |
| `0x2A` | `ADC2_RESISTANCE_L` | 2 | R | ADC2 computed resistance (optional) |
| `0x2C` | `ADC_CONFIG` | 1 | R/W | Input mode/averaging/range bits |

## 3.4 Hobby servo output

| Address | Name | Size | R/W | Description |
|---|---|---:|---|---|
| `0x30` | `SERVO_ENABLE` | 1 | R/W | `0` disabled, `1` enabled |
| `0x31` | `SERVO_POSITION_8BIT` | 1 | R/W | Position `0..255` |
| `0x32` | `SERVO_ANGLE_DEG` | 1 | R/W | Angle in degrees (`0..180`) |
| `0x33` | `SERVO_PULSE_MIN_L` | 2 | R/W | Min pulse width in us |
| `0x35` | `SERVO_PULSE_MAX_L` | 2 | R/W | Max pulse width in us |
| `0x37` | `SERVO_CURRENT_POSITION_8BIT` | 1 | R | Last applied 8-bit position |
| `0x38` | `SERVO_CURRENT_ANGLE` | 1 | R | Last applied angle |

## 3.5 Device control / diagnostics

| Address | Name | Size | R/W | Description |
|---|---|---:|---|---|
| `0x40` | `DEVICE_RESET_CMD` | 1 | W | Write magic value to soft-reset tool |
| `0x41` | `FAULT_CLEAR_CMD` | 1 | W | Write magic value to clear latched faults |
| `0x42` | `UPTIME_SEC_L` | 2 | R | Uptime seconds (low 16 bits) |
| `0x44` | `LAST_ERROR_CODE` | 1 | R | Last internal error code |
| `0x45` | `BUILD_ID` | 1 | R | Build/revision byte |

---

## 4) Bit Definitions

## 4.1 `TOOL_STATUS_FLAGS` (`0x04`)

- Bit 0: `PWM1_OVERCURRENT`
- Bit 1: `PWM2_OVERCURRENT`
- Bit 2: `ADC1_OUT_OF_RANGE`
- Bit 3: `ADC2_OUT_OF_RANGE`
- Bit 4: `SERVO_FAULT`
- Bit 5: `WATCHDOG_ACTIVE`
- Bit 6: `OUTPUTS_FORCED_OFF`
- Bit 7: `RESERVED`

## 4.2 `PWM_CONTROL` (`0x12`)

- Bit 0: PWM1 enable
- Bit 1: PWM2 enable
- Bit 2: PWM1 polarity (`0` normal, `1` inverted)
- Bit 3: PWM2 polarity (`0` normal, `1` inverted)
- Bit 4: PWM sync update (`1` apply both channels together)
- Bit 5..7: reserved

## 4.3 `TOOL_CONFIG_FLAGS` (`0x05`)

- Bit 0: apply safe outputs on boot
- Bit 1: watchdog enabled
- Bit 2: servo enable allowed
- Bit 3: publish computed resistance values
- Bit 4..7: reserved

---

## 5) Data Scaling Rules

To keep firmware simple and explicit:

1. `PWMx_DUTY` is raw `0..255`.
2. `PWMx_CURRENT_RAW_L` can be either:
   - raw ADC counts, or
   - current in mA (recommended).
   Pick one and document in firmware release notes.
3. `ADCx_MV_L` is millivolts.
4. `ADCx_RESISTANCE_L` is ohms (clamped to `0..65535`).
5. `SERVO_POSITION_8BIT` maps linearly to pulse range:
   - `0` -> `SERVO_PULSE_MIN`
   - `255` -> `SERVO_PULSE_MAX`
6. `SERVO_ANGLE_DEG` maps `0..180` to same pulse range.

If both angle and position are written, apply the most recent write.

---

## 6) Safety Behavior (Recommended)

1. On watchdog timeout:
   - set `PWM1_DUTY=0`, `PWM2_DUTY=0`
   - clear PWM enable bits
   - optionally disable hobby servo
   - set status bits `WATCHDOG_ACTIVE` and `OUTPUTS_FORCED_OFF`
2. On overcurrent channel N:
   - force that channel duty to `0`
   - clear enable bit
   - set corresponding overcurrent status bit
3. Faults remain latched until `FAULT_CLEAR_CMD` write.

---

## 7) ST3215-Compatible Command Usage

## 7.1 PING tool node

- ID must be `0x40`
- Standard ping with no parameters

TX:

`FF FF 40 02 01 BC`

Checksum check:

- sum = `0x40 + 0x02 + 0x01 = 0x43`
- checksum = `~0x43 & 0xFF = 0xBC`

## 7.2 READ single byte (tool type ID at `0x03`)

READ parameters:

- address = `0x03`
- length = `0x01`

TX:

`FF FF 40 04 02 03 01 B5`

Checksum:

- sum = `0x40 + 0x04 + 0x02 + 0x03 + 0x01 = 0x4A`
- checksum = `0xB5`

## 7.3 WRITE PWM1 duty = 128 to `0x10`

WRITE parameters:

- address = `0x10`
- data = `0x80`

TX:

`FF FF 40 04 03 10 80 28`

Checksum:

- sum = `0x40 + 0x04 + 0x03 + 0x10 + 0x80 = 0xD7`
- checksum = `0x28`

## 7.4 READ PWM1 current 16-bit at `0x14` (length 2)

TX:

`FF FF 40 04 02 14 02 A3`

Checksum:

- sum = `0x40 + 0x04 + 0x02 + 0x14 + 0x02 = 0x5C`
- checksum = `0xA3`

## 7.5 WRITE hobby servo angle = 90 deg to `0x32`

TX:

`FF FF 40 04 03 32 5A 2C`

Checksum:

- sum = `0x40 + 0x04 + 0x03 + 0x32 + 0x5A = 0xD3`
- checksum = `0x2C`

---

## 8) Host-Side API (JSON layer suggestion)

These are optional higher-level commands for your WebSocket server.
Under the hood they use STS `READ`/`WRITE` with ID `64`.

### 8.1 Identify tool

Request:

```json
{ "command": "toolGetIdentity" }
```

Response:

```json
{
  "type": "toolIdentity",
  "id": 64,
  "toolTypeId": 7,
  "protocolVersion": 1,
  "firmwareMajor": 1,
  "firmwareMinor": 0
}
```

### 8.2 Set PWM outputs

Request:

```json
{
  "command": "toolSetPwm",
  "pwm1Duty": 128,
  "pwm2Duty": 64,
  "enable1": true,
  "enable2": true
}
```

### 8.3 Read ADCs

Request:

```json
{ "command": "toolReadAdc" }
```

Response:

```json
{
  "type": "toolAdc",
  "adc1Raw": 2345,
  "adc2Raw": 1988,
  "adc1mV": 1890,
  "adc2mV": 1602,
  "adc1Resistance": 4700,
  "adc2Resistance": 10000
}
```

### 8.4 Set hobby servo output

Request (position):

```json
{ "command": "toolSetServoPosition", "position": 200 }
```

Request (angle):

```json
{ "command": "toolSetServoAngle", "angle": 90 }
```

### 8.5 Read tool status

Request:

```json
{ "command": "toolGetStatus" }
```

Response:

```json
{
  "type": "toolStatus",
  "statusFlags": 0,
  "lastErrorCode": 0,
  "uptimeSec": 1234,
  "pwm1Current": 120,
  "pwm2Current": 95
}
```

---

## 9) Integration Notes for Existing Node.js Stack

1. Reuse current packet builder/parser exactly.
2. Create one controller instance with servo ID `64`.
3. Add guard rails so joint commands never target ID `64`.
4. Add separate tool command handlers in `server.js`.
5. Poll tool status at a slower rate than servo motion loop if needed.

---

## 10) Validation Checklist

Before declaring protocol stable:

1. PING ID `64` repeatedly while servos move -> no cross-response.
2. Read/write PWM registers while reading servo statuses -> no parser confusion.
3. Verify all 16-bit fields decode little-endian correctly.
4. Force watchdog timeout and confirm outputs go safe.
5. Force overcurrent test condition and confirm latch + clear workflow.
6. Power-cycle and verify config persistence behavior is as intended.

---

## 11) Versioning

Start with:

- `TOOL_PROTOCOL_VERSION = 1`

If register meanings change in future:

- keep old addresses stable where possible
- increment protocol version
- add compatibility handling in host code

