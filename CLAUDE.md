# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a complete 6-axis robot arm control system targeting a Raspberry Pi + ST3215 serial bus servos, consisting of three independently runnable components:

- **`raspberry-pi-control-st3215/`** — Node.js WebSocket server running on the Pi (the authoritative control layer)
- **`electron-app/`** — Electron desktop pendant/control interface (the main UI)
- **`Python/`** — Python client library for external scripting
- **`End Tool API ESP32/`** — C firmware for an ESP32 end-tool (servo ID 64 on the bus)

The system is also used in a BTEC educational context. All third-party libs (Blockly, Three.js) are bundled locally for offline use.

## Raspberry Pi Access

- **IP:** `192.168.1.112` — user `mxadmin`, password `matrixpass_567834`
- **Hostname:** `MatrixRPI` (Linux 6.12.75, aarch64, Raspberry Pi OS Debian)
- `sshpass` is not available on Windows — use `plink` (PuTTY) instead:

```powershell
plink -ssh -pw matrixpass_567834 -batch mxadmin@192.168.1.112 "your command"
```

## Commands

### Pi Server
```bash
cd raspberry-pi-control-st3215
npm install
node server.js                    # start server (port 8080, /dev/serial0)
npm run test                      # run test-st3215.js (bus communication tests)
SERIAL_PORT=/dev/ttyUSB0 node server.js   # override serial device
```

### Electron App
```bash
cd electron-app
npm install
npm start                         # run app
npm run dev                       # run with DevTools open
```

### Python Client
```bash
cd Python
pip install -r requirements.txt
python example_basic.py
python example_end_tool_servo.py
```

### Deployment (on Pi)
```bash
sudo ./raspberry-pi-control-st3215/install-service.sh /opt/RobotArm/raspberry-pi-control-st3215
sudo ./electron-app/install-kiosk-service.sh
```

## Architecture

### Communication Stack
```
Electron UI
  → robotArmClient.js (WS client, auto-reconnects)
    → server.js (port 8080, command routing + status cache)
      → servoWorker.js (child process — owns serial port exclusively)
        → robotArmST3215.js (ST3215 binary protocol)
          → /dev/serial0 @ 1 Mbps
            → ST3215 servos (IDs 1–6, daisy-chain) + ESP32 end-tool (ID 64)
```

### server.js — Command Routing
Two command classes:
- **Instant commands** (answered from cache, no bus access): `getStatus`, `getJointConfigs`, `kinematicsLoadURDF`, etc.
- **Bus write commands** (require holding the control session lock): `moveJoint`, `setSpeed`, `rescanServos`, etc.

Only one WebSocket client may hold the control session at a time. `takeControl`/`releaseControl` messages manage this. Immediate moves bypass the command queue and go to the bus directly.

### servoWorker.js — Serial I/O Isolation
Runs as a child process to isolate serial failures from the server. Key behaviors:
- Status polling every 20 ms (configurable via `STATUS_POLL_INTERVAL_MS`)
- Write queue with priority lanes for moves vs. stops
- Servo backoff: after 3 consecutive failures, that servo is skipped for 2 s
- IPC with parent via `process.send` / `process.on('message')`

### robotArmST3215.js — Protocol Layer
Implements the ST3215 half-duplex serial bus protocol. Frame format:
`[0xFF, 0xFF, ID, LENGTH, INSTRUCTION, PARAMS..., CHECKSUM]`

Throws `BusCommunicationError` with a `retryable` flag so callers can decide whether to retry or give up. `ServoController` instances are created per servo ID.

### Electron App Modules
| File | Responsibility |
|---|---|
| `robotArmClient.js` | WebSocket wrapper, request-ID correlation, reconnect backoff |
| `kinematics.js` | Client-side FK/IK (mirrors server-side) |
| `robotArm3D.js` | Three.js 3D arm visualization |
| `gcodeProcessor.js` | G-code parsing/execution incl. N-labels, GOTO, IF, #variables |
| `rapidProcessor.js` | RAPID subset language core: VAR/WHILE/FOR/IF/GOTO, expressions (arm commands stay in app.js) |
| `blocklyRobotArm.js` | Blockly visual-programming integration |
| `positionsManager.js` | 100 stored positions (slots 0–99), persisted locally |

### Key Server Configuration (server.js top-of-file constants)
```javascript
PORT = 8080
SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERIAL_PORT = process.env.SERIAL_PORT || '/dev/serial0'
SERIAL_BAUDRATE = 1000000
STATUS_POLL_INTERVAL_MS = 20
```

Environment variables for diagnostics: `ROBOT_ARM_DEBUG_LOG`, `VERBOSE_LOG`, `BUS_DIAGNOSTICS_LOG_INTERVAL_MS`.

### End-Tool ESP32 (ID 64)
The ESP32 appears as servo ID 64 on the ST3215 bus but speaks a register-map protocol defined in `End Tool API ESP32/REGISTER_MAP.md`. It controls hobby servos and PWM outputs. The C source is in `RobotArmv3_End_Tool_API.c`; the compiled `.elf` is in the same directory.

## Key Documentation Files
- `raspberry-pi-control-st3215/API_DOCUMENTATION.md` — full WebSocket JSON API reference
- `raspberry-pi-control-st3215/ST3215_PROTOCOL_REFERENCE.md` — low-level bus protocol details
- `raspberry-pi-control-st3215/MULTI_CLIENT.md` — control session locking semantics
- `End Tool API ESP32/REGISTER_MAP.md` — ESP32 end-tool register layout
- `electron-app/KIOSK_SETUP.md` — Chromium full-screen kiosk on Pi
- `RASPBERRY_PI_SETUP_GUIDE.md` — initial Pi hardware/OS setup
