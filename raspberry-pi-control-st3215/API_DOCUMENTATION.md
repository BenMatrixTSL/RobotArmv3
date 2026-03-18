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






