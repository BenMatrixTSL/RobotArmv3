# Robot Arm Control - ST3215 Servo Version

Node.js control system for robot arm using Waveshare ST3215 serial bus servo motors on Raspberry Pi.

This project is similar to the I2C-based robot arm control system, but uses ST3215 serial bus servos instead of I2C-controlled PIC joints.

## Features

- Control up to 6 ST3215 serial bus servo motors
- Optional ESP32 end-tool node support on ST3215-compatible bus ID `64`
- WebSocket server for communication with Electron desktop app
- Runtime servo rescan and reinitialize command (`rescanServos`)
- Set servo angles (-180 to +180 in current implementation)
- Read servo status (position, speed, temperature, voltage, load)
- Set speed and acceleration
- Start/stop servo torque

## Hardware Setup

1. **ST3215 Servos**:
   - Connect all servos in a daisy-chain configuration on a single serial bus
   - Each servo must have a unique ID (1-6)
   - Connect servo data line to Raspberry Pi serial port (e.g., `/dev/ttyUSB0` or `/dev/ttyAMA0`)

2. **Power Supply**:
   - Provide appropriate power supply for all servos
   - Check ST3215 specifications for voltage and current requirements
   - Ensure power supply can handle all servos simultaneously

3. **Serial Connection**:
   - Connect servo data line to Raspberry Pi serial port
   - Default baud rate: 1,000,000 (1 Mbps)
   - Common serial ports on Raspberry Pi:
     - `/dev/ttyUSB0` - USB-to-serial adapter
     - `/dev/ttyAMA0` - Hardware UART (may need configuration)
     - `/dev/ttyS0` - Alternative serial port

## Software Setup

### Prerequisites

- Node.js (v14 or higher)
- npm (Node Package Manager)
- Raspberry Pi OS (or similar Linux distribution)

### Installation

1. **Navigate to project directory**:
   ```bash
   cd raspberry-pi-control-st3215
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

   This will install:
   - `serialport` - For serial communication with ST3215 servos
   - `ws` - WebSocket library for client communication

3. **Configure serial port**:
   - Default is Raspberry Pi UART pins (`/dev/serial0`).
   - To use a different port (for example USB serial), set environment variable `SERIAL_PORT`:
     ```bash
     SERIAL_PORT=/dev/ttyUSB0 node server.js
     ```
   - Or edit `server.js` and change the fallback value:
     ```javascript
     const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/serial0';
     ```

4. **Configure servo IDs**:
   - Edit `server.js` and update the `SERVO_IDS` array:
     ```javascript
     const SERVO_IDS = [1, 2, 3, 4, 5, 6]; // ST3215 servo IDs
     ```

5. **Set serial port permissions** (if needed):
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
   Or add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   (Log out and back in for group changes to take effect)

## Usage

### Start the Server

```bash
npm run server
```

Or directly:
```bash
node server.js
```

### Auto-start on boot (systemd)

This project includes:
- `install-service.sh` — installs `npm` dependencies and registers a systemd service
- `st3215-server.service` — service template (paths filled in by the installer)
- `uninstall-service.sh` — removes the service

**Automatic restart:** The service uses `Restart=always`. If the server crashes or exits on its own, systemd waits 5 seconds and starts it again. A manual `sudo systemctl stop st3215-server` does not auto-restart until you start it again.

**Debug log files** (after install):
- `/var/log/robot-arm-st3215/server-debug.log` — startup, shutdown, signals, uncaught errors
- `/var/log/robot-arm-st3215/server.log` — normal console output from the server

Run on the Pi:
```bash
cd raspberry-pi-control-st3215
chmod +x install-service.sh uninstall-service.sh
sudo ./install-service.sh
```

If your repo lives elsewhere (for example `/opt/RobotArm/raspberry-pi-control-st3215`):
```bash
sudo ./install-service.sh /opt/RobotArm/raspberry-pi-control-st3215
```

After install:
```bash
sudo systemctl status st3215-server.service
tail -f /var/log/robot-arm-st3215/server-debug.log
sudo journalctl -u st3215-server.service -f
```

Remove the service:
```bash
sudo ./uninstall-service.sh
```

The server will:
1. Initialize all ST3215 servo controllers
2. Initialize optional end-tool controller (ID `64`)
3. Enable torque on all responsive servos
4. Start WebSocket server on port 8080
5. Wait for client connections

### Test the Servos

Create a simple test script (`test-st3215.js`):

```javascript
const RobotArm = require('./robotArmST3215');

async function test() {
    const servo = new RobotArm.ServoController(1, '/dev/ttyUSB0', 1);
    
    try {
        await servo.open();
        console.log('Servo connected');
        
        // Enable torque
        await servo.startServo();
        
        // Move to 90 degrees
        await servo.moveToAngle(90);
        
        // Wait a bit
        await new Promise(resolve => setTimeout(resolve, 2000));
        
        // Read status
        const status = await servo.readStatus();
        console.log('Status:', status);
        
        // Disable torque
        await servo.stopServo();
        
        await servo.close();
    } catch (error) {
        console.error('Error:', error);
    }
}

test();
```

Run the test:
```bash
node test-st3215.js
```

## WebSocket API

The server accepts JSON commands via WebSocket. All commands follow this format:

```json
{
  "command": "commandName",
  "joint": 1,
  "angle": 90,
  ...
}
```

### Available Commands

#### `getStatus`
Get status from all servos.

**Request**:
```json
{
  "command": "getStatus"
}
```

**Response**:
```json
{
  "type": "status",
  "joints": [
    {
      "joint": 1,
      "available": true,
      "angleDegrees": 90.5,
      "position": 1024,
      "speed": 1000,
      "load": 5.0,
      "voltage": 7.4,
      "temperature": 35,
      "isMoving": false
    },
    ...
  ]
}
```

#### `moveJoint`
Move a servo to a specific angle.

**Request**:
```json
{
  "command": "moveJoint",
  "joint": 1,
  "angle": 90.0
}
```

**Response**:
```json
{
  "type": "success",
  "message": "Servo 1 moving to 90°"
}
```

#### `stopJoint`
Stop a specific servo (disable torque).

**Request**:
```json
{
  "command": "stopJoint",
  "joint": 1
}
```

#### `stopAllJoints`
Stop all servos.

**Request**:
```json
{
  "command": "stopAllJoints"
}
```

#### `setSpeed`
Set servo speed.

**Request**:
```json
{
  "command": "setSpeed",
  "joint": 1,
  "speed": 2000
}
```

Speed range: 0-3400 step/s

#### `setAcceleration`
Set servo acceleration.

**Request**:
```json
{
  "command": "setAcceleration",
  "joint": 1,
  "acceleration": 100
}
```

Acceleration range: 0-254 (unit: 100 step/s²)

#### `rescanServos`
Rescan configured servo IDs and reinitialize unavailable/unresponsive servos without restarting the server.

**Request**:
```json
{
  "command": "rescanServos"
}
```

**Response**:
```json
{
  "type": "servoRescan",
  "joints": [
    {
      "joint": 1,
      "servoId": 1,
      "available": true,
      "action": "kept_existing"
    },
    {
      "joint": 2,
      "servoId": 2,
      "available": true,
      "action": "recreated"
    }
  ]
}
```

#### End-tool commands (ID `64`)
The server includes optional end-tool commands:

- `toolPing`
- `toolGetIdentity`
- `toolGetStatus`
- `toolSetPwm`
- `toolGetPwmState`
- `toolReadCurrents`
- `toolReadAdc`
- `toolSetServoEnabled`
- `toolSetServoPosition`
- `toolSetServoAngle`
- `toolGetServoState`
- `toolSetWatchdog`
- `toolClearFaults`
- `toolReset`

For full request/response examples, see `API_DOCUMENTATION.md`.

## Configuration

### Serial Port Settings

Edit `server.js` to configure:

```javascript
const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/serial0';  // Serial port path
const SERIAL_BAUDRATE = 1000000;      // Baud rate (1 Mbps default)
```

### Servo IDs

Edit `server.js` to configure servo IDs:

```javascript
const SERVO_IDS = [1, 2, 3, 4, 5, 6];  // ST3215 servo IDs
```

### Number of Joints

Edit `server.js` to configure number of servos:

```javascript
const JOINT_COUNT = 6;  // Number of servos
```

## Troubleshooting

### Servo Not Responding

1. **Check Serial Port**:
   - Verify serial port path is correct
   - Check permissions: `ls -l /dev/ttyUSB0`
   - Try different serial ports

2. **Check Servo IDs**:
   - Ensure each servo has a unique ID
   - Use ST3215 configuration tool to set IDs
   - Verify IDs match the `servoIds` array in `server.js`

3. **Check Power Supply**:
   - Ensure servos have adequate power
   - Check voltage levels

4. **Check Connections**:
   - Verify data line is connected correctly
   - Check for loose connections

### Serial Port Permission Errors

If you get permission errors:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Or add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

(Log out and back in)

### Communication Timeouts

If you experience timeouts:

1. Check baud rate matches servo configuration
2. Verify serial port is not in use by another program
3. Check for electrical interference
4. Try reducing baud rate (if servos support it)

## Differences from I2C Version

This ST3215 version differs from the I2C version in several ways:

1. **Communication**: Uses serial (UART) instead of I2C
2. **Protocol**: Uses ST3215 serial bus protocol instead of custom I2C commands
3. **Position Range**: 0-4095 (12-bit) instead of angle-based
4. **Multiple Servos**: All servos share the same serial port (daisy-chained)
5. **No Sensor Reading**: ST3215 servos don't have external sensors like AS5600

## Protocol and API References

- ST3215 protocol reference: `ST3215_PROTOCOL_REFERENCE.md`
- ESP32 end-tool bus/API spec: `ST3215_ESP32_END_TOOL_API.md`
- Full WebSocket API (including tool and rescan commands): `API_DOCUMENTATION.md`

## License

MIT License - See LICENSE file for details






