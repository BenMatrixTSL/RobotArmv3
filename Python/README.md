# Robot Arm Python Client

Small Python library for talking to the **raspberry-pi-control-st3215** WebSocket server over the network.

## Requirements

- Python 3.8 or newer
- Robot arm server running on the Pi (`node server.js`, default port **8080**)
- PC and Pi on the same network

## Install

```bash
cd Firmware/Python
pip install -r requirements.txt
```

## Quick example

Edit `PI_HOST` in `example_basic.py`, then run:

```bash
python example_basic.py
```

## Use in your own script

```python
from robot_arm import RobotArmClient, RobotArmError

arm = RobotArmClient("192.168.1.100", 8080)
arm.connect()

status = arm.get_status()
arm.move_joint(1, 45.0, speed=1500)

currents = arm.tool_read_currents()
print(currents["servoCurrentRaw"])

arm.disconnect()
```

## Main methods

| Method | Server command | Description |
|--------|----------------|-------------|
| `connect()` | — | Open WebSocket to the Pi |
| `disconnect()` | — | Close connection |
| `request(command, **params)` | any | Low-level call with `requestId` matching |
| `get_joint_configs()` | `getJointConfigs` | Servos discovered on bus |
| `get_status()` | `getStatus` | Angles, load, voltage, etc. |
| `move_joint(joint, angle, speed=1500)` | `moveJoint` | Move one joint |
| `stop_joint(joint)` | `stopJoint` | Stop one joint |
| `stop_all_joints()` | `stopAllJoints` | Stop all joints |
| `tool_read_currents()` | `toolReadCurrents` | PWM + servo current (mA) |
| `tool_read_adc()` | `toolReadAdc` | ADC0 / ADC1 raw and mV |
| `tool_set_pwm(...)` | `toolSetPwm` | Set end-tool PWM outputs |
| `kinematics_load_urdf(xml)` | `kinematicsLoadURDF` | Load URDF on server |
| `kinematics_forward(angles)` | `kinematicsForwardKinematics` | Forward kinematics |
| `kinematics_inverse(x, y, z)` | `kinematicsInverseKinematics` | Inverse kinematics |

For the full command list, see `../raspberry-pi-control-st3215/API_DOCUMENTATION.md`.

## Protocol notes

- Messages are JSON over WebSocket: `ws://<pi-ip>:8080`
- Requests should include a `requestId`; the server echoes it on the reply
- Errors come back as `{"type": "error", "message": "..."}`

## Troubleshooting

### `Unknown command: toolReadCurrents` or `toolReadAdc`

Your PC has the latest Python client, but the **Raspberry Pi is still running an older `server.js`** that does not define those commands yet.

**Fix:** update the server on the Pi, then restart it.

1. Copy this folder to the Pi (overwrite the old one):
   - `Firmware/raspberry-pi-control-st3215/`
   - Needs at least: `server.js`, `robotArmST3215.js`, `package.json`
2. On the Pi:
   ```bash
   cd raspberry-pi-control-st3215
   npm install
   node server.js
   ```
3. Run `example_basic.py` again.

If the Pi uses git and systemd, you can also pull from your repo and restart the service (same as the Electron “update server” feature).

### Connection refused (`WinError 10061`)

Nothing is listening on port 8080. Check:

- `node server.js` is running on the Pi
- IP address is correct (`hostname -I` on the Pi)
- PC and Pi are on the same network
