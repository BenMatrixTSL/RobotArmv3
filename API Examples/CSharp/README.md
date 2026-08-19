# Robot Arm C# Client

C# client for the **raspberry-pi-control-st3215** WebSocket server.
Uses only built-in .NET libraries — no NuGet packages required.

## Requirements

- [.NET 8 SDK](https://dotnet.microsoft.com/download) (free, includes the `dotnet` CLI)
- Robot arm server running on the Pi: `node server.js` (default port **8080**)
- PC and Pi on the same network

## Build and run

```bash
cd "API Examples/CSharp"

dotnet run                        # default IP (192.168.1.112)
dotnet run -- 192.168.1.50        # override the Pi IP
```

To produce a standalone `.exe`:

```bash
dotnet publish -c Release -r win-x64 --self-contained
# output: bin\Release\net8.0\win-x64\publish\RobotArmExample.exe
```

## Using the client in your own project

Copy `RobotArmClient.cs` into your project (no other files needed) then:

```csharp
await using var arm = new RobotArmClient("192.168.1.112");
await arm.ConnectAsync();

// Read joint angles
var status = await arm.GetStatusAsync();
foreach (var joint in status.GetProperty("joints").EnumerateArray())
{
    Console.WriteLine($"Joint {joint.GetProperty("joint").GetInt32()}: " +
                      $"{joint.GetProperty("angleDegrees").GetDouble():F2} deg");
}

// Move a joint
await arm.MoveJointAsync(joint: 1, angle: 45.0);
await arm.MoveJointAsync(joint: 2, angle: -30.0, speed: 800);

// Forward kinematics -- get XYZ from joint angles
var fk = await arm.KinematicsForwardAsync([0.0, 45.0, -30.0, 0.0, 0.0, 0.0]);
var pos = fk.GetProperty("position");
Console.WriteLine($"XYZ: {pos.GetProperty("x").GetDouble():F1}, " +
                        $"{pos.GetProperty("y").GetDouble():F1}, " +
                        $"{pos.GetProperty("z").GetDouble():F1} mm");

// Inverse kinematics -- move to an XYZ position (mm)
var ik = await arm.KinematicsInverseAsync(x: 150.0, y: 0.0, z: 200.0);
if (ik.TryGetProperty("jointAngles", out var solved))
{
    var angles = solved.EnumerateArray().Select(a => a.GetDouble()).ToList();
    for (int i = 0; i < angles.Count; i++)
        await arm.MoveJointAsync(i + 1, angles[i]);
}
```

## API reference

All methods are `async` and throw `RobotArmError` on failure (timeout, server error, or not connected).
Timeouts are in milliseconds and can be overridden on every call.

### Connection

| Method | Description |
|--------|-------------|
| `ConnectAsync(ct?)` | Open WebSocket to the Pi and start the receive task |
| `DisposeAsync()` | Close the connection (`await using` handles this automatically) |

### Joint commands

| Method | Server command | Description |
|--------|----------------|-------------|
| `GetJointConfigsAsync()` | `getJointConfigs` | Servos discovered on the bus |
| `GetStatusAsync()` | `getStatus` | Angles, load, voltage for all joints |
| `MoveJointAsync(joint, angle, speed=1500)` | `moveJoint` | Move one joint to an angle (degrees) |
| `StopJointAsync(joint)` | `stopJoint` | Stop one joint immediately |
| `StopAllJointsAsync()` | `stopAllJoints` | Emergency stop all joints |
| `SetSpeedAsync(joint, speed)` | `setSpeed` | Set default speed for one joint |
| `SetSpeedAllAsync(speed)` | `setSpeedAll` | Set default speed for all joints |
| `SetTorqueAllAsync(enabled)` | `setTorqueAll` | Enable / disable torque on all joints |
| `RescanServosAsync()` | `rescanServos` | Re-scan the serial bus for servos |

### End-tool commands

| Method | Server command | Description |
|--------|----------------|-------------|
| `ToolPingAsync()` | `toolPing` | Check the end tool (ID 64) is responding |
| `ToolReadCurrentsAsync()` | `toolReadCurrents` | PWM1, PWM2, servo currents (mA) |
| `ToolReadAdcAsync()` | `toolReadAdc` | ADC0 / ADC1 raw counts and millivolts |
| `ToolSetPwmAsync(pwm1, pwm2, en1, en2)` | `toolSetPwm` | Set end-tool PWM outputs (0-255) |
| `ToolSetServoEnabledAsync(enabled)` | `toolSetServoEnabled` | Enable / disable hobby servo |
| `ToolSetServoAngleAsync(angle)` | `toolSetServoAngle` | Set hobby servo angle (0-180 deg) |
| `ToolSetServoPositionAsync(position)` | `toolSetServoPosition` | Set hobby servo 8-bit position (0-255) |
| `ToolGetServoStateAsync()` | `toolGetServoState` | Read current servo angle and position |

### Kinematics

IK/FK calls require a URDF loaded on the server first.

| Method | Server command | Description |
|--------|----------------|-------------|
| `KinematicsLoadUrdfAsync(xml)` | `kinematicsLoadURDF` | Load URDF text into the server solver |
| `KinematicsForwardAsync(jointAngles)` | `kinematicsForwardKinematics` | Joint angles -> XYZ (mm) |
| `KinematicsInverseAsync(x, y, z, initialAngles?)` | `kinematicsInverseKinematics` | XYZ target -> joint angles |

### Low-level

```csharp
// Send any command directly with arbitrary parameters
var response = await arm.RequestAsync("someCommand",
    new JsonObject { ["param1"] = 42, ["param2"] = true },
    timeoutMs: 5000);
```

## Files

| File | Purpose |
|------|---------|
| `RobotArmClient.cs` | WebSocket client, request/response matching, all command wrappers |
| `ExampleBasic.cs` | Example program with commented-out IK, FK, joint move snippets |
| `RobotArmExample.csproj` | .NET 8 project file |

## Protocol notes

- Transport: JSON text frames over WebSocket at `ws://<pi-ip>:8080`
- Each request includes a `requestId` integer; the server echoes it on the reply
- Errors come back as `{"type": "error", "message": "..."}`
- The receive loop runs as a background `Task`; all public methods are thread-safe

## Troubleshooting

### `Unknown command: toolReadCurrents` / `toolReadAdc`

The Pi is running an older `server.js` without end-tool support.

```bash
cd raspberry-pi-control-st3215
git pull && npm install && node server.js
```

### Connection refused / timeout

- Confirm `node server.js` is running on the Pi
- Check the IP with `hostname -I` on the Pi
- Make sure port 8080 is not blocked by a firewall

### IK / FK returns an error

Call `KinematicsLoadUrdfAsync()` with the robot's URDF content before making IK/FK calls.

For the full server command list see `../raspberry-pi-control-st3215/API_DOCUMENTATION.md`.
