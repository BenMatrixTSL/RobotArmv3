# ST3215 Protocol Reference (From Local Working Code)

This document is derived from the working implementation in:

- `raspberry-pi-control-st3215/robotArmST3215.js`
- `raspberry-pi-control-st3215/server.js`
- `raspberry-pi-control-st3215/test-st3215.js`

No online protocol sources were used to build this reference.

---

## 1) Serial Link Settings Used

- Baud rate: `1000000` (1 Mbps)
- Data bits: `8`
- Parity: `none`
- Stop bits: `1`
- Shared bus model: one UART line with multiple servos, addressed by servo ID

---

## 2) Packet Layouts (Exact Byte Positions)

The implementation uses these fixed index meanings:

- `PKT_HEADER_0 = 0`
- `PKT_HEADER_1 = 1`
- `PKT_ID = 2`
- `PKT_LENGTH = 3`
- `PKT_INSTRUCTION = 4` (TX packet)
- `PKT_ERROR = 4` (RX status packet)
- `PKT_PARAMETER0 = 5`

### 2.1 Master -> Servo (TX Command Packet)

Byte layout:

1. `0xFF` (header 1)
2. `0xFF` (header 2)
3. `ID` (servo ID)
4. `LENGTH`
5. `INSTRUCTION`
6. `PARAMETER 0` (optional)
7. `PARAMETER 1` (optional)
8. `...`
9. `CHECKSUM` (last byte)

Length rule in this code:

- `LENGTH = 2 + parameterCount`
- Why: `INSTRUCTION(1) + PARAMETERS(N) + CHECKSUM(1)`

Total TX packet bytes:

- `2(header) + 1(id) + 1(length) + LENGTH`
- Equivalent to `4 + LENGTH`

### 2.2 Servo -> Master (RX Status Packet)

Byte layout:

1. `0xFF` (header 1)
2. `0xFF` (header 2)
3. `ID`
4. `LENGTH`
5. `ERROR`
6. `PARAMETER 0` (optional)
7. `PARAMETER 1` (optional)
8. `...`
9. `CHECKSUM` (last byte)

Length rule in this code:

- `LENGTH` includes: `ERROR(1) + PARAMETERS(N) + CHECKSUM(1)`
- Therefore `parameterCount = LENGTH - 2`

Expected total RX bytes (exact parser rule):

- `expectedLength = LENGTH + 4`
- The parser waits until at least this many bytes are present.

---

## 3) Header, ID, and Routing Behavior

- Header must be exactly `0xFF 0xFF` at bytes `0` and `1`.
- Response packet is accepted only when response `ID` equals the target servo ID for that controller.
- If complete packet is received for a different ID, that controller ignores it.
- This is required because all servos share one serial line.

---

## 4) Checksum (Exact Algorithm Used)

Checksum is identical in TX and RX verification style:

1. Sum bytes from `ID` through the byte before checksum.
2. Bitwise NOT the sum.
3. Keep low 8 bits.

Formula:

`checksum = (~sum) & 0xFF`

### 4.1 TX checksum range

For TX command packet, sum includes:

- `ID`
- `LENGTH`
- `INSTRUCTION`
- all parameters

### 4.2 RX checksum range

For RX status packet, sum includes:

- `ID`
- `LENGTH`
- `ERROR`
- all returned parameters

If checksum does not match, parser reports `"Checksum mismatch"` and packet is rejected.

---

## 5) Instructions Implemented

From the working code:

- `PING = 0x01`
- `READ = 0x02`
- `WRITE = 0x03`

### 5.1 PING (`0x01`)

TX parameters: none

- `LENGTH = 2`

Expected RX:

- status packet with same ID
- `ERROR = 0` means success

### 5.2 READ (`0x02`)

TX parameters are exactly 2 bytes:

1. `address` (1 byte)
2. `readLength` (1 byte)

So:

- `parameterCount = 2`
- `LENGTH = 4`

Expected RX:

- `ERROR` byte
- `readLength` data bytes in `PARAMETER` region
- Parser extracts exactly `LENGTH - 2` parameter bytes

### 5.3 WRITE (`0x03`)

TX parameters are:

1. `address` (1 byte)
2. one or more data bytes

So:

- `parameterCount = 1 + dataByteCount`
- `LENGTH = 2 + parameterCount`

Expected RX for write ack:

- typically no returned data bytes
- `LENGTH = 2` (`ERROR + CHECKSUM`)
- code handles this as a normal write response

---

## 6) Endianness Rules (Important)

### 6.1 Multi-byte values are little-endian

The code always sends and interprets 16-bit values as:

- low byte first
- high byte second

This is used for:

- goal speed (`STS_GOAL_SPEED_L/H`)
- goal position (`STS_GOAL_POSITION_L/H`)
- present position (`STS_PRESENT_POSITION_L/H`)
- present speed (`STS_PRESENT_SPEED_L/H`)

### 6.2 Exact helper behavior from code

- low byte: `value & 0xFF`
- high byte: `(value >> 8) & 0xFF`
- reconstruct word: `(low & 0xFF) | ((high & 0xFF) << 8)`

### 6.3 Example (16-bit value)

If value is `1500` decimal:

- hex value = `0x05DC`
- low byte = `0xDC`
- high byte = `0x05`
- transmitted order = `[0xDC, 0x05]`

---

## 7) Register Map Actually Used by This Code

Only registers used in local working implementation are listed.

- `0x28` (`40`) `STS_TORQUE_ENABLE` (1 byte)
- `0x29` (`41`) `STS_ACC` (1 byte)
- `0x2A` (`42`) `STS_GOAL_POSITION_L`
- `0x2B` (`43`) `STS_GOAL_POSITION_H`
- `0x2E` (`46`) `STS_GOAL_SPEED_L`
- `0x2F` (`47`) `STS_GOAL_SPEED_H`
- `0x38` (`56`) `STS_PRESENT_POSITION_L`
- `0x39` (`57`) `STS_PRESENT_POSITION_H`
- `0x3A` (`58`) `STS_PRESENT_SPEED_L`
- `0x3B` (`59`) `STS_PRESENT_SPEED_H`
- `0x3C` (`60`) `STS_PRESENT_LOAD_L` (read as 1 byte in this code)
- `0x3E` (`62`) `STS_PRESENT_VOLTAGE` (1 byte)
- `0x3F` (`63`) `STS_PRESENT_TEMPERATURE` (1 byte)
- `0x42` (`66`) `STS_MOVING` (1 byte)

---

## 8) Exact Command Byte Examples

All examples below are computed with the same checksum algorithm used in code.

## 8.1 Ping servo ID 5

Fields:

- Header: `FF FF`
- ID: `05`
- LENGTH: `02`
- INST: `01`
- CHECKSUM over `05 02 01`

Sum: `0x05 + 0x02 + 0x01 = 0x08`  
Checksum: `~0x08 & 0xFF = 0xF7`

TX bytes:

`FF FF 05 02 01 F7`

## 8.2 Read 2 bytes from present position low (`address 0x38`) on ID 5

READ params:

- address: `38`
- length: `02`

Fields before checksum:

`FF FF 05 04 02 38 02`

Checksum over `05 04 02 38 02`:

- Sum = `0x45`
- Checksum = `~0x45 & 0xFF = 0xBA`

TX bytes:

`FF FF 05 04 02 38 02 BA`

If servo position were `2048` (`0x0800`, low/high = `00 08`) and no error:

RX bytes would be:

`FF FF 05 04 00 00 08 EE`

Why:

- `LENGTH = 4` (`ERROR + 2 data bytes + CHECKSUM`)
- checksum over `05 04 00 00 08`:
  - sum = `0x11`
  - checksum = `0xEE`

## 8.3 Write speed = 1500 (`0x05DC`) to ID 5 at `GOAL_SPEED_L` (`0x2E`)

Data bytes (little-endian): `DC 05`

WRITE parameters:

- address: `2E`
- data0: `DC`
- data1: `05`

Fields before checksum:

`FF FF 05 05 03 2E DC 05`

Checksum over `05 05 03 2E DC 05`:

- Sum = `0x1C`
- Checksum = `0xE3`

TX bytes:

`FF FF 05 05 03 2E DC 05 E3`

Typical write ACK RX (no returned data, no error):

`FF FF 05 02 00 F8`

Checksum over `05 02 00`:

- Sum = `0x07`
- Checksum = `0xF8`

## 8.4 Write goal position = 3072 (`0x0C00`) to ID 5 at `GOAL_POSITION_L` (`0x2A`)

Data bytes (little-endian): `00 0C`

Fields before checksum:

`FF FF 05 05 03 2A 00 0C`

Checksum over `05 05 03 2A 00 0C`:

- Sum = `0x43`
- Checksum = `0xBC`

TX bytes:

`FF FF 05 05 03 2A 00 0C BC`

---

## 9) Data Interpretation Used by Application Code

### 9.1 Position and angle mapping

Application mapping in `robotArmST3215.js`:

- center position: `2048` steps = `0°`
- conversion: `steps = round(2048 + angleDeg * (2048/180))`
- inverse: `angleDeg = (steps - 2048) / (2048/180)`

Limits used:

- position clamped `0..4095`
- angle clamped `-180..180`

### 9.2 Status scaling used

- load byte interpreted as: `load = raw * 0.1`
- voltage byte interpreted as: `voltage = raw * 0.1`
- temperature byte interpreted directly as Celsius
- moving flag: non-zero means moving
- torque enabled: non-zero means enabled

---

## 10) Bulk Read Pattern Used for Fast Status

`readQuickStatus()` performs one READ:

- start address: `STS_TORQUE_ENABLE` (`40`)
- length: `(STS_MOVING - STS_TORQUE_ENABLE) + 1 = 27`

So READ request parameters are:

- address = `0x28`
- length = `0x1B`

The returned block is indexed by offset from address `40`:

- offset `0` -> torque enabled (`40`)
- offset `16` -> present position low (`56`)
- offset `17` -> present position high (`57`)
- offset `26` -> moving (`66`)

---

## 11) Timeout and Validation Behavior in Code

Used in local implementation:

- read timeout: `10 ms`
- write timeout: `200 ms`
- ping timeout: `200 ms`
- pending response buffer max: `256 bytes`

Validation steps:

1. Minimum packet size must be at least 6 bytes.
2. Header must be `FF FF`.
3. Full packet length must satisfy `expectedLength = LENGTH + 4`.
4. Checksum must match.
5. Response `ID` must match target servo ID.

Special case handled:

- During reads, if a write ACK packet (`responseLength = 2`) is received first, code ignores it and keeps waiting for actual read data response.

---

## 12) Practical Rules to Rely On

If you build other clients (Python/C++/MCU), match these exact rules to be compatible with this working stack:

1. Always send `FF FF` header.
2. Use one-byte servo ID.
3. For TX, set `LENGTH = 2 + parameterCount`.
4. For RX parse, expect total bytes as `LENGTH + 4`.
5. Multi-byte values are little-endian (`low, high`).
6. Checksum is `(~sum(ID..lastDataByte)) & 0xFF`.
7. READ parameters are exactly `[address, byteCount]`.
8. WRITE parameters are `[address, data0, data1, ...]`.

---

## 13) Source of Truth Note

When in doubt, treat these files as authoritative for your system behavior:

- `raspberry-pi-control-st3215/robotArmST3215.js` (packet build/parse, byte order, checksums)
- `raspberry-pi-control-st3215/server.js` (bus sharing, command queueing, usage patterns)
- `raspberry-pi-control-st3215/test-st3215.js` (real-world test flow)

This reference is intentionally scoped to what your local code actually implements and uses.

---

## 14) Untested Possible Additions (Do Not Assume Working Yet)

This section is a roadmap only. These items were seen in external ST3215/SCServo libraries, but are **not implemented or validated** in your local codebase.

Treat each item as experimental until hardware-tested on your setup.

### 14.0 Validation tracker

Use this table as a simple checklist while testing.  
Suggested status values: `Not in code`, `Untested`, `In progress`, `Validated`, `Rejected`.

| Feature | Current status | Last test date | Notes |
|---|---|---|---|
| Bus discovery / scan helper | Not in code | - | |
| Read present current (`0x45/0x46`) | Not in code | - | |
| Operating mode control (`0x21`) | Not in code | - | |
| Continuous rotation helper | Not in code | - | |
| PWM/open-loop control | Not in code | - | |
| EEPROM lock/unlock (`0x37`) | Not in code | - | |
| ID reconfiguration (`0x05`) | Not in code | - | |
| Baudrate reconfiguration (`0x06`) | Not in code | - | |
| Offset / midpoint calibration | Not in code | - | |
| Status/error bitfield read (`0x41`) | Not in code | - | |
| Sync write / broadcast operations | Not in code | - | |

### 14.1 Bus discovery / scan helper

Potential function:

- `scanServos(startId = 0, endId = 253)` -> list IDs that answer `PING`

Likely packet behavior:

- send `PING` (`0x01`) to each ID
- add ID to results only if valid response packet with matching ID and `ERROR=0`

Risk:

- scans can take time and may collide with normal motion commands unless queued.

### 14.2 Read present current

Potential registers seen in external libraries:

- current low: `0x45` (`69`)
- current high: `0x46` (`70`)

Likely packet behavior:

- READ address `0x45`, length `0x02`
- parse as little-endian 16-bit value (`low, high`)

Risk:

- scaling/sign interpretation may vary by firmware/model and must be confirmed by real measurements.

### 14.3 Operating mode control

Potential register:

- mode register: `0x21` (`33`)

Common mode values in external code:

- `0` position mode
- `1` closed-loop speed mode
- `2` PWM/open-loop mode
- `3` step mode

Risk:

- mode transitions may require torque disable/enable sequence and may affect safety.

### 14.4 Continuous rotation helper

Potential function:

- `rotate(speedSigned)` in speed mode

Likely behavior in external libs:

- set mode to speed mode (`1`)
- write absolute speed to goal speed register
- encode direction in a high bit of the 16-bit speed field

Risk:

- direction bit conventions are easy to get wrong without direct hardware verification.

### 14.5 PWM/open-loop control

Potential function:

- `setPwm(pwmSigned)` in PWM mode

Likely behavior:

- set mode to PWM mode (`2`)
- write 16-bit PWM value and direction flag encoding

Risk:

- incorrect encoding can cause unexpected motor behavior.

### 14.6 EEPROM lock/unlock

Potential register:

- lock flag: `0x37` (`55`)

Likely usage:

- unlock before editing persistent config (ID, baud, limits)
- relock afterwards

Risk:

- wrong write order can leave settings half-applied across power cycles.

### 14.7 ID and baudrate reconfiguration tools

Potential registers:

- ID: `0x05`
- baud: `0x06`

Potential functions:

- `changeId(oldId, newId)`
- `changeBaud(oldId, baudCode)`

Risk:

- one wrong write can make a servo "disappear" until recovered with a scan/config tool.

### 14.8 Offset / midpoint calibration helpers

Potential registers:

- position correction low/high: `0x1F` / `0x20`
- torque special value `128` at `0x28` seen in external implementations for midpoint define

Potential functions:

- `setPositionCorrection(correctionSteps)`
- `defineMiddle()`

Risk:

- calibration changes can alter all future angle mappings unexpectedly.

### 14.9 Status/error bitfield read

Potential register:

- status flags: `0x41` (`65`)

Potential function:

- `readStatusFlags()` -> decode voltage/sensor/temperature/current/angle/overload bits

Risk:

- bit meaning should be validated against actual servo responses on your hardware.

### 14.10 Sync write / broadcast operations

Common protocol extensions in external SDKs:

- `REG_WRITE` + `ACTION`
- `SYNC_WRITE`
- broadcast ID `0xFE`

Potential benefit:

- lower bus overhead for multi-servo coordinated motion

Risk:

- your current parser/queue model is built around per-servo request/response; broadcast and sync patterns need careful integration/testing.

### 14.11 Safe adoption checklist (recommended)

Before trusting any addition above:

1. Add one function at a time.
2. Capture and log full TX/RX hex packets.
3. Verify checksum and response length on real hardware.
4. Confirm register scaling with measured physical behavior.
5. Keep a recovery path (known baud, ID scan, single-servo test rig).
6. Mark each new feature as "validated" only after repeated tests.

---

These are deliberately documented as **untested additions** so your current stable control path remains unchanged.

---

## 15) Flowcode `ST3215.fcfx` Comparison (Documentation-Only)

This section compares this reference against:

- `D:/Dev/Flowcode V11/Components/IO_Source/mechatronics/ST3215.fcfx`

Purpose:

- identify where Flowcode matches this protocol reference
- flag potential mismatches that should **not** be trusted without hardware tests

No runtime code was changed as part of this comparison.

### 15.1 Items that match this reference

From the Flowcode macros/constants:

- Header bytes are `0xFF 0xFF` (`ST3215_HEADER1=255`, `ST3215_HEADER2=255`)
- Commands match: `PING=1`, `READ=2`, `WRITE=3`
- TX length logic matches: `Length = ParamCount + 2`
- Checksum generation matches pattern: checksum is bitwise NOT of the byte sum
- Most 16-bit write/read operations use little-endian ordering (`low` then `high`)

### 15.2 Potential mismatches in Flowcode constants/macros

The items below appear inconsistent with this reference and with common STS mappings used elsewhere in this project.

#### A) Mode register constant

- Flowcode constant: `ST3215_ADDR_MODE = 11`
- Expected mapping used in external STS-style mappings: mode register at `0x21` (`33`)

Impact:

- mode-related APIs (`SetMode`, `ReadMode`, `Rotate`) may target the wrong register if `11` is incorrect.

#### B) Baudrate register constant

- Flowcode constant: `ST3215_ADDR_BAUDRATE = 4`
- Common STS mapping: baudrate register is `0x06` (`6`)

Impact:

- baud-change API could write to the wrong location.

#### C) Correction register constant

- Flowcode constant: `ST3215_ADDR_CORRECTION = 28`
- Common STS mapping: correction low/high at `0x1F/0x20` (`31/32`)

Impact:

- correction read/write APIs may not affect intended correction register.

#### D) `ReadCorrection` byte combine order

- Flowcode `ReadPosition` combines bytes as little-endian (`low + (high << 8)`)
- Flowcode `ReadCorrection` combines in opposite order (`ResponseBuffer[3] + (ResponseBuffer[2] << 8)`)

Impact:

- correction values may decode incorrectly even if register address were correct.

### 15.3 Response parsing robustness differences

Compared to `robotArmST3215.js`, Flowcode `Prv_ReceiveResponse` appears simpler:

- reads error byte but does not expose/use it consistently in API return logic
- no explicit checksum validation step visible in response parse path
- no explicit "response ID must match requested servo ID" guard visible in parse path

Impact:

- on shared buses or noisy links, parser may be less robust than the Node.js implementation.

### 15.4 Operational defaults difference

Flowcode component default UART value in this file is `115200`, while your Node stack is configured for `1000000`.

Impact:

- mismatch in baud between controller implementations can prevent communication.

### 15.5 Confidence labels for this comparison

- **High confidence:** direct constants and explicit macro formulas visible in the `.fcfx` source.
- **Medium confidence:** behavior implications inferred from macro flow (because `.fcfx` is a visual/macro definition format, not plain C source).

### 15.6 Practical guidance

For future work, safest order is:

1. Keep using this document + local `robotArmST3215.js` behavior as primary reference.
2. Treat conflicting Flowcode constants as "suspect until bench-tested."
3. If Flowcode functionality is needed, validate one register/API at a time on hardware with packet logging.

