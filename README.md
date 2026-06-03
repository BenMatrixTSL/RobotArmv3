# Robot Arm v3 Firmware

This repository contains the firmware/server-side and desktop-control code for the Robot Arm v3 project.

The active control stack is now ST3215 serial-bus based (Raspberry Pi + Node.js server + Electron app).  
Legacy PIC/stepper paths have been removed from the active repository layout.

## Current Active Components

- `raspberry-pi-control-st3215/`  
  Node.js WebSocket server and ST3215 bus controller used on Raspberry Pi.
- `electron-app/`
  Electron desktop app used to control the arm and exchange commands with the Pi server.
  On a Pi with a touchscreen, use **Chromium kiosk mode** (`electron-app/KIOSK_SETUP.md`) instead of Electron.

## ST3215 Control Stack (Current)

Main runtime flow:

1. Electron app sends JSON commands over WebSocket.
2. Raspberry Pi server receives commands and serializes bus access.
3. `robotArmST3215.js` sends ST3215-compatible packets over UART at 1 Mbps.
4. Servos (and optional end-tool node) respond on the same shared bus by ID.

## New Additions Included

- Full local ST3215 packet reference:
  - `raspberry-pi-control-st3215/ST3215_PROTOCOL_REFERENCE.md`
- ESP32 end-tool extension spec (bus ID `64`):
  - `raspberry-pi-control-st3215/ST3215_ESP32_END_TOOL_API.md`
- End-tool support implemented in server/controller:
  - `toolPing`, `toolGetIdentity`, `toolGetStatus`, PWM/ADC/hobby-servo commands, watchdog/fault/reset commands
- Runtime servo recovery command:
  - `rescanServos` (re-checks configured servo IDs and reinitializes when possible)

## Quick Start

1. Go to the ST3215 server folder:
   ```bash
   cd raspberry-pi-control-st3215
   ```
2. Install dependencies:
   ```bash
   npm install
   ```
3. Run server:
   ```bash
   node server.js
   ```

You can override serial device path with:

```bash
SERIAL_PORT=/dev/ttyUSB0 node server.js
```

## API and Protocol Docs

- WebSocket API:
  - `raspberry-pi-control-st3215/API_DOCUMENTATION.md`
- ST3215 packet protocol:
  - `raspberry-pi-control-st3215/ST3215_PROTOCOL_REFERENCE.md`
- ESP32 end-tool bus API:
  - `raspberry-pi-control-st3215/ST3215_ESP32_END_TOOL_API.md`

## Notes

- The ST3215 server is the primary control path.
- If you maintain historical experiments, keep them under archive paths and avoid using them as active references.

