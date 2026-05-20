# Robot Arm Control API Documentation

## Overview

This document describes the WebSocket API for controlling the robot arm via the Node.js server running on the Raspberry Pi. The server communicates with ST3215 serial bus servo motors and provides a WebSocket interface for external control from Python, C++, MATLAB, LabVIEW, or any other language that supports WebSocket connections.

## Connection Details

- **Protocol**: WebSocket (WS)
- **Default Port**: 8080
- **URL Format**: `ws://<raspberry-pi-ip>:8080`
- **Example**: `ws://192.168.1.100:8080` or `ws://raspberrypi.local:8080`

## Message Format

All messages are JSON-encoded strings sent over WebSocket.

### Request Format

```json
{
  "command": "<command_name>",
  "<parameter1>": <value1>,
  "<parameter2>": <value2>,
  ...
}
```

### Response Format

Responses can be one of the following types:

**Success Response:**
```json
{
  "type": "success",
  "message": "Description of what happened"
}
```

**Error Response:**
```json
{
  "type": "error",
  "message": "Error description"
}
```

**Status Response:**
```json
{
  "type": "status",
  "joints": [
    {
      "joint": 1,
      "available": true,
      "isMoving": false,
      "angleDegrees": 45.5,
      "position": 2048,
      "stepPosition": 2048,
      "speed": 1500,
      "load": 50.2,
      "voltage": 7.4,
      "temperature": 35,
      "torqueEnabled": true
    },
    ...
  ]
}
```

**Joint Configs Response:**
```json
{
  "type": "jointConfigs",
  "count": 6,
  "total": 6,
  "joints": [
    {
      "jointNumber": 1,
      "servoId": 1,
      "available": true
    },
    ...
  ]
}
```

**Connection Response (on connect):**
```json
{
  "type": "connected",
  "message": "Connected to Robot Arm Server (ST3215)"
}
```

## Available Commands

### 1. Get Joint Configurations

Retrieves the number of servos discovered and their basic configuration.

**Request:**
```json
{
  "command": "getJointConfigs"
}
```

**Response:**
```json
{
  "type": "jointConfigs",
  "count": 6,
  "total": 6,
  "joints": [
    {
      "jointNumber": 1,
      "servoId": 1,
      "available": true
    },
    {
      "jointNumber": 2,
      "servoId": 2,
      "available": true
    },
    ...
  ]
}
```

### 2. Get Status

Retrieves the current status of all joints.

**Request:**
```json
{
  "command": "getStatus"
}
```

**Response:**
```json
{
  "type": "status",
  "joints": [
    {
      "joint": 1,
      "available": true,
      "isMoving": false,
      "angleDegrees": 45.5,
      "position": 2048,
      "stepPosition": 2048,
      "speed": 1500,
      "load": 50.2,
      "voltage": 7.4,
      "temperature": 35,
      "torqueEnabled": true
    },
    {
      "joint": 2,
      "available": false,
      "isMoving": false,
      "angleDegrees": 0,
      "position": 0,
      "stepPosition": 0,
      "speed": 0,
      "load": 0,
      "voltage": 0,
      "temperature": 0,
      "torqueEnabled": false
    },
    ...
  ]
}
```

**Status Fields:**
- `joint`: Joint number (1-6)
- `available`: Whether the servo is available/connected
- `isMoving`: Whether the servo is currently moving
- `angleDegrees`: Current angle in degrees (-90 to +90)
- `position`: Raw servo step position (1024-3072, center=2048)
- `stepPosition`: Same as `position` (legacy field name)
- `speed`: Current speed setting in steps/second (0-3400)
- `load`: Current load percentage (0-100)
- `voltage`: Current voltage in volts
- `temperature`: Current temperature in degrees Celsius
- `torqueEnabled`: Whether torque is enabled (servo is active)

### 3. Move Joint

Moves a specific joint to a target angle.

**Request:**
```json
{
  "command": "moveJoint",
  "joint": 1,
  "angle": 45.0,
  "speed": 1500
}
```

**Parameters:**
- `joint` (required): Joint number (1-6)
- `angle` (required): Target angle in degrees (-90 to +90)
- `speed` (optional): Movement speed in steps/second (0-3400). Default: 1500

**Response:**
```json
{
  "type": "success",
  "message": "Servo 1 moving to 45° at 1500 step/s"
}
```

### 4. Stop Joint

Stops a specific joint immediately.

**Request:**
```json
{
  "command": "stopJoint",
  "joint": 1
}
```

**Parameters:**
- `joint` (required): Joint number (1-6)

**Response:**
```json
{
  "type": "success",
  "message": "Servo 1 stopped"
}
```

### 5. Stop All Joints

Stops all joints immediately.

**Request:**
```json
{
  "command": "stopAllJoints"
}
```

**Response:**
```json
{
  "type": "success",
  "message": "All servos stopped"
}
```

### 6. Set Servo Angle

Sets a servo to a specific angle (alias for `moveJoint` without speed parameter).

**Request:**
```json
{
  "command": "setServoAngle",
  "joint": 1,
  "angle": 90.0
}
```

**Parameters:**
- `joint` (required): Joint number (1-6)
- `angle` (required): Target angle in degrees (-90 to +90)

**Response:**
```json
{
  "type": "success",
  "message": "Servo 1 set to 90°"
}
```

### 7. Set Speed

Sets the speed for a specific joint (affects future movements).

**Request:**
```json
{
  "command": "setSpeed",
  "joint": 1,
  "speed": 2000
}
```

**Parameters:**
- `joint` (required): Joint number (1-6)
- `speed` (required): Speed in steps/second (0-3400)

**Response:**
```json
{
  "type": "success",
  "message": "Servo 1 speed set to 2000 step/s"
}
```

### 8. Set Speed All

Sets the speed for all joints.

**Request:**
```json
{
  "command": "setSpeedAll",
  "speed": 2000
}
```

**Parameters:**
- `speed` (required): Speed in steps/second (0-3400)

**Response:**
```json
{
  "type": "success",
  "message": "All servos speed set to 2000 step/s"
}
```

### 9. Set Torque All

Enables or disables torque for all joints.

**Request:**
```json
{
  "command": "setTorqueAll",
  "enabled": true
}
```

**Parameters:**
- `enabled` (required): `true` to enable torque, `false` to disable

**Response:**
```json
{
  "type": "success",
  "message": "All servos torque enabled"
}
```

### 10. Set Acceleration

Sets the acceleration for a specific joint.

**Request:**
```json
{
  "command": "setAcceleration",
  "joint": 1,
  "acceleration": 50
}
```

**Parameters:**
- `joint` (required): Joint number (1-6)
- `acceleration` (required): Acceleration value (typically 0-255)

**Response:**
```json
{
  "type": "success",
  "message": "Servo 1 acceleration set to 50"
}
```

### 11. End Tool Commands (ESP32 Node, ID 64)

The server also supports an optional ST3215-compatible end-tool node on bus ID `64`.

If the end tool is not connected or not initialized, these commands return:

```json
{
  "type": "error",
  "message": "End tool controller is not initialized"
}
```

#### 11.1 `toolPing`

Checks if the end tool responds on the bus.

**Request:**
```json
{
  "command": "toolPing"
}
```

**Response:**
```json
{
  "type": "toolPing",
  "ok": true
}
```

#### 11.2 `toolGetIdentity`

Reads end-tool protocol and firmware identity bytes.

**Request:**
```json
{
  "command": "toolGetIdentity"
}
```

**Response:**
```json
{
  "type": "toolIdentity",
  "id": 64,
  "protocolVersion": 1,
  "firmwareMajor": 1,
  "firmwareMinor": 0,
  "toolTypeId": 7
}
```

#### 11.3 `toolGetStatus`

Reads status flags, last error code, and low 16 bits of uptime.

**Request:**
```json
{
  "command": "toolGetStatus"
}
```

**Response:**
```json
{
  "type": "toolStatus",
  "statusFlags": 0,
  "lastErrorCode": 0,
  "uptimeSecLow16": 1234
}
```

#### 11.4 `toolSetPwm`

Sets PWM duty for both FET channels and enable states.

**Request:**
```json
{
  "command": "toolSetPwm",
  "pwm1Duty": 128,
  "pwm2Duty": 64,
  "enable1": true,
  "enable2": true
}
```

**Parameters:**
- `pwm1Duty` (optional): PWM1 duty `0-255` (default `0`)
- `pwm2Duty` (optional): PWM2 duty `0-255` (default `0`)
- `enable1` (optional): enable PWM1 (`true` by default)
- `enable2` (optional): enable PWM2 (`true` by default)

**Response:**
```json
{
  "type": "success",
  "message": "Tool PWM outputs updated"
}
```

#### 11.5 `toolGetPwmState`

Reads current PWM register state.

**Request:**
```json
{
  "command": "toolGetPwmState"
}
```

**Response:**
```json
{
  "type": "toolPwmState",
  "pwm1Duty": 128,
  "pwm2Duty": 64,
  "pwmControl": 3
}
```

#### 11.6 `toolReadCurrents`

Reads PWM1, PWM2, and servo shunt current channels (milliamps, 16-bit little-endian each).

**Request:**
```json
{
  "command": "toolReadCurrents"
}
```

**Response:**
```json
{
  "type": "toolCurrents",
  "pwm1CurrentRaw": 120,
  "pwm2CurrentRaw": 95,
  "servoCurrentRaw": 42
}
```

#### 11.7 `toolReadAdc`

Reads ADC0 and ADC1 (raw counts and millivolts).

**Request:**
```json
{
  "command": "toolReadAdc"
}
```

**Response:**
```json
{
  "type": "toolAdc",
  "adc0Raw": 2345,
  "adc1Raw": 1988,
  "adc0mV": 1890,
  "adc1mV": 1602
}
```

#### 11.8 `toolSetServoEnabled`

Enables or disables the end-tool hobby servo output.

**Request:**
```json
{
  "command": "toolSetServoEnabled",
  "enabled": true
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool hobby servo enabled"
}
```

#### 11.9 `toolSetServoPosition`

Sets hobby servo position as an 8-bit value.

**Request:**
```json
{
  "command": "toolSetServoPosition",
  "position": 200
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool hobby servo position set to 200"
}
```

#### 11.10 `toolSetServoAngle`

Sets hobby servo angle in degrees.

**Request:**
```json
{
  "command": "toolSetServoAngle",
  "angle": 90
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool hobby servo angle set to 90"
}
```

#### 11.11 `toolGetServoState`

Reads current hobby servo state values from the end tool.

**Request:**
```json
{
  "command": "toolGetServoState"
}
```

**Response:**
```json
{
  "type": "toolServoState",
  "currentPosition8bit": 200,
  "currentAngle": 90
}
```

#### 11.12 `toolSetWatchdog`

Sets tool watchdog timeout in milliseconds (`0` disables watchdog).

**Request:**
```json
{
  "command": "toolSetWatchdog",
  "timeoutMs": 500
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool watchdog timeout set to 500 ms"
}
```

#### 11.13 `toolClearFaults`

Sends command to clear latched tool faults.

**Request:**
```json
{
  "command": "toolClearFaults"
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool faults clear command sent"
}
```

#### 11.14 `toolReset`

Sends soft reset command to the end tool.

**Request:**
```json
{
  "command": "toolReset"
}
```

**Response:**
```json
{
  "type": "success",
  "message": "Tool reset command sent"
}
```

### 12. Rescan Servo Motors

The server can rescan and reinitialize the configured ST3215 servo IDs at runtime.
This is useful if a servo becomes unresponsive after startup and later recovers.

#### 12.1 `rescanServos`

Pings each configured servo ID and performs recovery:

- keeps working controllers
- recreates missing/unresponsive controllers
- re-enables torque on responsive controllers
- marks still-unavailable joints as unavailable

**Request:**
```json
{
  "command": "rescanServos"
}
```

**Response:**
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
    },
    {
      "joint": 3,
      "servoId": 3,
      "available": false,
      "action": "missing",
      "error": "Servo 3 (ID: 3) did not respond to ping"
    }
  ]
}
```

**Action field meanings:**
- `kept_existing`: existing controller responded and was kept
- `created`: previously missing controller was successfully created
- `recreated`: previously unresponsive controller was replaced and recovered
- `missing`: no controller present and scan failed
- `lost`: previous controller existed but recovery failed

## Example Code

### Python Example

```python
import asyncio
import websockets
import json

async def control_robot_arm():
    # Connect to the robot arm server
    uri = "ws://192.168.1.100:8080"
    
    async with websockets.connect(uri) as websocket:
        # Wait for connection message
        response = await websocket.recv()
        print(f"Connected: {response}")
        
        # Get joint configurations
        await websocket.send(json.dumps({"command": "getJointConfigs"}))
        config_response = await websocket.recv()
        print(f"Config: {config_response}")
        
        # Move joint 1 to 45 degrees at speed 1500
        await websocket.send(json.dumps({
            "command": "moveJoint",
            "joint": 1,
            "angle": 45.0,
            "speed": 1500
        }))
        response = await websocket.recv()
        print(f"Move response: {response}")
        
        # Get status
        await websocket.send(json.dumps({"command": "getStatus"}))
        status_response = await websocket.recv()
        print(f"Status: {status_response}")
        
        # Stop all joints
        await websocket.send(json.dumps({"command": "stopAllJoints"}))
        response = await websocket.recv()
        print(f"Stop response: {response}")

# Run the example
asyncio.run(control_robot_arm())
```

**Installation:**
```bash
pip install websockets
```

### C++ Example (using websocketpp)

```cpp
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include <nlohmann/json.hpp>

typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
using json = nlohmann::json;

int main() {
    client c;
    
    // Initialize ASIO
    c.init_asio();
    
    // Set up message handler
    c.set_message_handler([](websocketpp::connection_hdl hdl, client::message_ptr msg) {
        std::cout << "Received: " << msg->get_payload() << std::endl;
    });
    
    // Connect to server
    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection("ws://192.168.1.100:8080", ec);
    
    if (ec) {
        std::cout << "Connection error: " << ec.message() << std::endl;
        return 1;
    }
    
    c.connect(con);
    
    // Start ASIO io_service run loop
    std::thread t([&c]() { c.run(); });
    
    // Wait a bit for connection
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Send commands
    json move_cmd = {
        {"command", "moveJoint"},
        {"joint", 1},
        {"angle", 45.0},
        {"speed", 1500}
    };
    con->send(move_cmd.dump());
    
    // Keep running
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    c.stop();
    t.join();
    
    return 0;
}
```

**Installation:**
```bash
# Install websocketpp and nlohmann/json libraries
# On Ubuntu/Debian:
sudo apt-get install libwebsocketpp-dev nlohmann-json3-dev
```

### MATLAB Example

```matlab
% Connect to WebSocket server
% Note: MATLAB doesn't have built-in WebSocket support
% You'll need to use a third-party library or Java WebSocket client

% Example using Java WebSocket (requires Java WebSocket API)
import java.net.URI;
import javax.websocket.*;

% Create WebSocket client
uri = URI('ws://192.168.1.100:8080');
% ... (WebSocket client implementation)

% Alternative: Use Python from MATLAB
% system('python control_robot.py');

% Or use HTTP requests if you add an HTTP endpoint to the server
```

**Note:** MATLAB doesn't have native WebSocket support. Consider:
1. Using a third-party MATLAB WebSocket library
2. Calling Python scripts from MATLAB using `system()` or `py` interface
3. Adding an HTTP REST API endpoint to the server

### LabVIEW Example

LabVIEW doesn't have native WebSocket support. Options:

1. **Use HTTP REST API** (if added to server):
   - Use LabVIEW's HTTP client VIs
   - Send POST requests with JSON payloads

2. **Use .NET WebSocket Client**:
   - Use LabVIEW's .NET interface to call .NET WebSocket libraries
   - Requires .NET Framework 4.5+

3. **Use Python Node**:
   - Use LabVIEW's Python Node to call Python WebSocket scripts
   - Requires Python installed on the system

**Example using HTTP (if REST API is added):**
```labview
% LabVIEW Block Diagram pseudocode:
% 1. Build JSON string: {"command":"moveJoint","joint":1,"angle":45.0,"speed":1500}
% 2. Use "HTTP Client POST" VI
% 3. URL: http://192.168.1.100:8080/api/command
% 4. Body: JSON string
% 5. Parse response JSON
```

## Error Handling

All commands may return error responses. Always check the `type` field:

```python
response = json.loads(await websocket.recv())
if response["type"] == "error":
    print(f"Error: {response['message']}")
    # Handle error appropriately
elif response["type"] == "success":
    print(f"Success: {response['message']}")
```

## Common Errors

- **"Invalid joint number"**: Joint number is out of range (must be 1-6)
- **"Servo X is not available"**: The specified servo is not connected or not responding
- **"Unknown command"**: The command name is not recognized
- **Connection errors**: Server is not running, wrong IP address, or network issues

## Notes

1. **Command Queue**: The server processes commands sequentially to prevent conflicts. Multiple clients can connect simultaneously, but commands are queued.

2. **Angle Range**: Joint angles are in degrees, range -90° to +90°:
   - -90° = 1024 steps (minimum position)
   - 0° = 2048 steps (center position)
   - +90° = 3072 steps (maximum position)

3. **Speed Range**: Speed is in steps/second, range 0-3400:
   - 0 = stopped
   - 1500 = default/medium speed
   - 3400 = maximum speed

4. **Status Updates**: The `getStatus` command reads current values from servos. For real-time monitoring, poll `getStatus` periodically (e.g., every 100-500ms).

5. **Connection Management**: The server sends a `connected` message when a client first connects. Always wait for this message before sending commands.

6. **Thread Safety**: The server handles multiple clients safely by queuing all commands. You can connect multiple clients simultaneously without conflicts.

## Troubleshooting

1. **Cannot connect**: 
   - Verify server is running: `ps aux | grep node`
   - Check IP address: `hostname -I` on Raspberry Pi
   - Check firewall: `sudo ufw status`
   - Test connection: `telnet <ip> 8080` or `nc -zv <ip> 8080`

2. **Commands not working**:
   - Check server logs for errors
   - Verify servos are connected and powered
   - Check serial port: `ls -l /dev/serial/by-id/`
   - Verify servo IDs match configuration

3. **Slow response**:
   - Commands are queued - only one processes at a time
   - Check network latency
   - Reduce status polling frequency

## Additional Resources

- Server source code: `server.js`
- Servo control module: `robotArmST3215.js`
- ST3215 servo documentation: Refer to Waveshare ST3215 datasheet






