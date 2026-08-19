# Robot Arm C++ Client

C++ client library for talking to the **raspberry-pi-control-st3215** WebSocket server.
Mirrors the Python client API exactly.

## Requirements

- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)
- CMake 3.14+ **or** a manual compile command
- [nlohmann/json](https://github.com/nlohmann/json/releases) single header (`json.hpp`) — already included in `nlohmann/`
- Robot arm server running on the Pi (`node server.js`, default port **8080**)
- PC and Pi on the same network

## Build

### Windows — CMake (recommended)

Use the Visual Studio developer command prompt, or run the batch below from any terminal after adjusting the VS path:

```bat
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

The executable is written to `build\Release\example_basic.exe`.

### Windows — single compile command

```bat
cl /std:c++17 example_basic.cpp robot_arm.cpp ws_client.cpp /Fe:example_basic.exe
```

### Linux / macOS

```bash
g++ -std=c++17 example_basic.cpp robot_arm.cpp ws_client.cpp -lpthread -o example_basic
```

## Quick start

```bash
# Default IP (192.168.1.112)
example_basic.exe

# Override the Pi IP at the command line
example_basic.exe 192.168.1.50
```

## Using the library in your own project

Add `ws_client.h`, `ws_client.cpp`, `robot_arm.h`, `robot_arm.cpp`, and the `nlohmann/` folder to your project, then:

```cpp
#include "robot_arm.h"

RobotArmClient arm("192.168.1.112", 8080);
arm.connect();

// Read joint angles
auto status = arm.getStatus();
for (auto& joint : status.value("joints", json::array())) {
    std::cout << "Joint " << joint.value("joint", 0)
              << ": "     << joint.value("angleDegrees", 0.0) << " deg\n";
}

// Move a joint
arm.moveJoint(1, 45.0);          // joint 1, 45 degrees, default speed
arm.moveJoint(2, -30.0, 800);    // joint 2, -30 degrees, slower speed

// Forward kinematics — get XYZ from joint angles
auto fk = arm.kinematicsForward({0.0, 45.0, -30.0, 0.0, 0.0, 0.0});
auto& pos = fk["position"];
std::cout << "XYZ: " << pos.value("x", 0.0) << ", "
                     << pos.value("y", 0.0) << ", "
                     << pos.value("z", 0.0) << " mm\n";

// Inverse kinematics — move to an XYZ target
auto ik = arm.kinematicsInverse(150.0, 0.0, 200.0);  // x, y, z in mm
if (ik.contains("jointAngles")) {
    auto angles = ik["jointAngles"].get<std::vector<double>>();
    for (size_t i = 0; i < angles.size(); ++i)
        arm.moveJoint((int)(i + 1), angles[i]);
}

arm.disconnect();
```

## API reference

All methods throw `RobotArmError` on failure (timeout, server error, or not connected).
Timeouts are in milliseconds and can be overridden on every call.

### Connection

| Method | Description |
|--------|-------------|
| `connect(timeoutMs=10000)` | Open WebSocket to the Pi and start the receive thread |
| `disconnect()` | Close the connection and unblock any pending requests |

### Joint commands

| Method | Server command | Description |
|--------|----------------|-------------|
| `getJointConfigs()` | `getJointConfigs` | Servos discovered on the bus |
| `getStatus()` | `getStatus` | Angles, load, voltage for all joints |
| `moveJoint(joint, angle, speed=1500)` | `moveJoint` | Move one joint to an angle (degrees) |
| `stopJoint(joint)` | `stopJoint` | Stop one joint immediately |
| `stopAllJoints()` | `stopAllJoints` | Emergency stop all joints |
| `setSpeed(joint, speed)` | `setSpeed` | Set default speed for one joint |
| `setSpeedAll(speed)` | `setSpeedAll` | Set default speed for all joints |
| `setTorqueAll(enabled)` | `setTorqueAll` | Enable / disable torque on all joints |
| `rescanServos()` | `rescanServos` | Re-scan the serial bus for servos |

### End-tool commands

| Method | Server command | Description |
|--------|----------------|-------------|
| `toolPing()` | `toolPing` | Check the end tool (ID 64) is responding |
| `toolReadCurrents()` | `toolReadCurrents` | PWM1, PWM2, servo currents (mA) |
| `toolReadAdc()` | `toolReadAdc` | ADC0 / ADC1 raw counts and millivolts |
| `toolSetPwm(pwm1, pwm2, en1, en2)` | `toolSetPwm` | Set end-tool PWM outputs (0–255) |
| `toolSetServoEnabled(enabled)` | `toolSetServoEnabled` | Enable / disable hobby servo |
| `toolSetServoAngle(angle)` | `toolSetServoAngle` | Set hobby servo angle (0–180°) |
| `toolSetServoPosition(position)` | `toolSetServoPosition` | Set hobby servo 8-bit position (0–255) |
| `toolGetServoState()` | `toolGetServoState` | Read current servo angle and position |

### Kinematics

IK/FK calls require a URDF to be loaded on the server first.

| Method | Server command | Description |
|--------|----------------|-------------|
| `kinematicsLoadUrdf(xml)` | `kinematicsLoadURDF` | Load URDF text into the server solver |
| `kinematicsForward(jointAngles)` | `kinematicsForwardKinematics` | Joint angles → XYZ position (mm) |
| `kinematicsInverse(x, y, z, initialAngles*)` | `kinematicsInverseKinematics` | XYZ target → joint angles |

### Low-level

```cpp
// Send any command directly, with arbitrary JSON parameters
json response = arm.request("someCommand", {{"param1", 42}, {"param2", true}}, /*timeoutMs=*/5000);
```

## Files

| File | Purpose |
|------|---------|
| `ws_client.h` / `ws_client.cpp` | Minimal RFC 6455 WebSocket client (TCP + frame codec, no external WS library) |
| `robot_arm.h` / `robot_arm.cpp` | Robot arm API — request/response matching, all command wrappers |
| `example_basic.cpp` | Example program with commented-out snippets for joint moves, IK, and FK |
| `nlohmann/json.hpp` | JSON parsing (header-only, v3.11.3) |
| `CMakeLists.txt` | CMake build definition |

## Protocol notes

- Transport: JSON text frames over WebSocket at `ws://<pi-ip>:8080`
- Each request includes a `requestId` integer; the server echoes it on the reply
- Errors come back as `{"type": "error", "message": "..."}`
- The receive thread runs in the background; all public methods are safe to call from any thread

## Troubleshooting

### `Unknown command: toolReadCurrents` / `toolReadAdc`

The Pi is running an older `server.js` that does not support end-tool commands yet.

**Fix:** update and restart the server on the Pi:

```bash
cd raspberry-pi-control-st3215
git pull
npm install
node server.js
```

### Connection failed / timeout

- Confirm `node server.js` is running on the Pi (`ps aux | grep node`)
- Check the IP address (`hostname -I` on the Pi)
- Make sure the PC and Pi are on the same network
- Default port is 8080 — check the Pi's firewall if applicable

### IK / FK returns an error

The server-side kinematics solver needs a URDF loaded before it can run.
Call `arm.kinematicsLoadUrdf(xml)` with the robot's URDF content at startup.

For the full server command list see `../raspberry-pi-control-st3215/API_DOCUMENTATION.md`.
