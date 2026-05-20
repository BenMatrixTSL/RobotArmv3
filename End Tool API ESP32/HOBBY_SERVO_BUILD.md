# Hobby Servo Firmware — Build Notes

Hobby servo support is implemented in **`RobotArmv3_End_Tool_API.fcfx`** (Flowcode source).

The generated files `RobotArmv3_End_Tool_API.c` / `.h` are **not** updated until you build the project in Flowcode.

## What was added

1. Register map entries **48–56** (`0x30`–`0x38`) for enable, position, angle, pulse limits, and read-back.
2. Multi-byte **READ** (1–8 bytes) so the Pi can read `toolGetServoState` in one bus transaction.
3. Multi-byte **WRITE** so several registers can be updated from one packet.
4. **`Servo_Controller1`** macros: `EnableServo`, `DisableServo`, `MoveToAngle`, `SetPosition`.
5. Register array expanded from 32 to **64** bytes.

## Build steps (Flowcode)

1. Open `RobotArmv3_End_Tool_API.fcfx` in Flowcode 10.
2. Review the new macros (if Flowcode reports errors on array index expressions, fix in the IDE):
   - `WritePacketBytesToRegisters`
   - `SendStatusPacketWithMultiReadData`
   - Hobby servo section inside `ApplyWritableRegistersToVariablesAndHardware`
3. Build / compile for **ESP32**.
4. Flash the ESP32 end-tool board.
5. On the Pi, pull the latest `robotArmST3215.js` (already uses addresses `0x30`–`0x32`) and restart `node server.js`.

## Quick test from Python

```bash
cd Firmware/Python
python example_end_tool_servo.py
```

## Register quick reference

| Address | Use |
|--------:|-----|
| 48 | Enable (`1` = on) |
| 49 | Position 0–255 |
| 50 | Angle 0–180° |
| 55–56 | Read-only copies of last position / angle |

See `REGISTER_MAP.md` for full details.
