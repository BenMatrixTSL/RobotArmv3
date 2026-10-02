# Using the ST3215 server standalone

This guide is for anyone driving the arm from their own software (Python, C#, C++, a
browser page, a PLC gateway) without the Electron app. It explains which parts of the
repository matter, how to run the server, which commands to use for joint and Cartesian
control, how to read the arm's state, and the rules the server enforces or expects you
to follow.

The server is the only process that talks to the servo bus. Everything else, including
the Electron app, is a WebSocket client of it. Your client is a peer of the app, not a
replacement for the server.

For the full command-by-command reference see `SERVER_API_REFERENCE.md`. For the raw
servo protocol see `ST3215_PROTOCOL_REFERENCE.md`. For multi-client semantics see
`MULTI_CLIENT.md`.

---

## 1. Which parts of the repository you need

| Path | What it is | Needed standalone? |
|---|---|---|
| `raspberry-pi-control-st3215/server.js` | WebSocket server on port 8080. Routes commands, caches status, owns the control session, runs kinematics. | Yes, runs on the Pi |
| `raspberry-pi-control-st3215/servoWorker.js` | Child process that owns the serial port. Polls status, queues bus writes, applies retries and backoff, torque watchdog heartbeat. | Yes, started by server.js |
| `raspberry-pi-control-st3215/robotArmST3215.js` | ST3215 bus protocol (frames, checksums, retries). | Yes, used by the worker |
| `raspberry-pi-control-st3215/kinematicsService.js` | Server-side FK/IK, orientation refinement, tool spin, linear path planner. | Yes, used by server.js |
| `raspberry-pi-control-st3215/kinematics.urdf` | Arm geometry, joint axes and limits, end-tool TCP offsets. Loaded at start. | Yes |
| `raspberry-pi-control-st3215/install-service.sh`, `st3215-server.service` | systemd install. | For a permanent install |
| `raspberry-pi-control-st3215/ledController.js`, `led_driver.py` | Optional WS2812 status LED. Harmless if no LED is fitted. | No |
| `raspberry-pi-control-st3215/servo-tuner.js`, `test-st3215.js`, `ik_test.js`, `reprogramServo2.js` | Maintenance scripts. They open the serial port themselves, so the service must be stopped first. | No |
| `API Examples/Python`, `API Examples/CSharp`, `API Examples/Cpp` | Reference clients. The Python package `robot_arm` (`client.py`) wraps connect, status, moves, tool and kinematics calls. | Recommended starting point |
| `End Tool API ESP32/` | Firmware and register map for the ESP32 end tool (bus ID 64). | Only if you change the tool firmware |
| `electron-app/` | The pendant UI. Not needed, but `robotArmClient.js` is a worked example of a robust client (request correlation, reconnect, motion-complete wait, retry on bus faults). | No |

Not needed on the Pi for a standalone install: `electron-app/`, `documentation/`, `BTEC Specs/`, `ST3215 Datasheets/`, `Archive/`.

---

## 2. Running the server

Hardware assumptions: six ST3215 servos with IDs 1 to 6 daisy-chained on `/dev/serial0`
at 1 Mbps, optional ESP32 end tool answering as ID 64.

```bash
cd raspberry-pi-control-st3215
npm install
node server.js                              # foreground, port 8080
SERIAL_PORT=/dev/ttyUSB0 node server.js     # different serial device
```

Permanent install (auto-start, restart on crash, logs under `/var/log/robot-arm-st3215/`):

```bash
sudo bash install-service.sh /opt/RobotArm/raspberry-pi-control-st3215
sudo systemctl status st3215-server.service
sudo journalctl -u st3215-server.service -f
```

Environment variables (set in the unit file or the shell):

| Variable | Default | Meaning |
|---|---|---|
| `SERIAL_PORT` | `/dev/serial0` | Serial device |
| `STATUS_POLL_INTERVAL_MS` | `20` | Bus poll period. Also the status push period to every client. |
| `ROBOT_ARM_DEBUG_LOG` | unset | Path for the debug log file (the service sets it) |
| `VERBOSE_LOG` | unset | `1` logs every poll and move |
| `BUS_DIAGNOSTICS_LOG_INTERVAL_MS` | `0` | Periodic bus health line (`BUS diag: ...`) when > 0 |

Log files when installed as a service: `/var/log/robot-arm-st3215/server.log` (bus and
move log, UTC timestamps) and `server-debug.log`.

**Only one process may open the serial port.** Stop the service before running any of
the maintenance scripts, and start it again afterwards:

```bash
sudo systemctl stop st3215-server.service
node servo-tuner.js --joint=2
sudo systemctl start st3215-server.service
```

---

## 3. Wire protocol in one page

- Connect to `ws://<pi>:8080`. Messages are JSON objects in both directions.
- A request is `{"command": "<name>", ...params, "requestId": <any>}`. The reply carries
  the same `requestId`. Without one you still get a reply, but you cannot tell it apart
  from unsolicited pushes, so always send one.
- Errors are `{"type": "error", "message": "...", "requestId": ...}`. A control problem
  also sets `"controlRequired": true`. Treat an unknown command as an old server.
- On connect the server sends `{"type":"connected", "pushesStatus":true,
  "statusIntervalMs":20, "serverBuildId":...}`, then a `jointConfigs`, a `controlStatus`
  and an `endTool` message.

Unsolicited pushes you must be prepared to receive at any time:

| `type` | When | Notes |
|---|---|---|
| `status` | every bus tick, about 50 times a second | `joints[]` plus `cacheAgeMs`. This is the feedback channel. Do not also poll `getStatus` in a loop. |
| `controlStatus` | on connect and whenever control changes hands, locks, unlocks or times out | Check `youHaveControl`. |
| `jointConfigs` | on connect and when the set of responding servos changes | |
| `endTool` | on connect and when the tool changes (probed every ~15 s) | `toolTypeId`, `tool.lengthMm`, `present` |
| `linearPathComplete` | when an `executeLinearMove` you started finishes | |
| `servoThermalFault` | a joint tripped on temperature and is in a 30 s cool-down | |
| `servoWorkerCrashed` | the bus process died and is restarting | Expect a few seconds without status. |
| `shutdown` | server stopping | |

Rate: at the default 20 ms poll a client receives roughly 50 status messages per
second. Parse them cheaply or raise `STATUS_POLL_INTERVAL_MS`.

---

## 4. Session rules

1. **Reads never need control.** `getStatus`, `getJointConfigs`, `getEndTool`, all
   `kinematics*` commands and the tool telemetry reads (`toolGetServoState`,
   `toolReadCurrents`, `toolReadAdc`, `toolGetStatus`, `toolGetPwmState`,
   `toolGetIdentity`) work for any client at any time.
2. **Anything that writes to the bus needs the control session**: `moveJoint`,
   `stopJoint`, `stopAll`, `setSpeed`, `setSpeedAll`, `setAcceleration`,
   `setTorqueAll`, `rescanServos`, every `tool*` write, every EEPROM command and
   `executeLinearMove`. Only one client holds it.
3. Take it explicitly with `{"command":"takeControl","label":"my-cell-controller"}` and
   check `youHaveControl` in the reply. If nobody holds control the first bus write also
   auto-grants it, but do not rely on that.
4. Control is released when your socket closes, when you send `releaseControl`, or
   automatically after **5 minutes with no move command** (`moveJoint`, `setServo`,
   `setAcceleration` reset the timer; other writes do not). A long-running controller
   that moves rarely should re-take control before each batch of moves rather than
   assume it still has it.
5. `takeControl` with `"force": true` takes control from another client unless that
   client locked it. `lockControl` with the lock password (`MatrixRA123`, local-network
   grade) holds control for `durationMs` (default 15 min, max 60 min) and suspends the
   idle timeout. Use it for unattended runs so the pendant cannot interrupt you.
6. `moveJoint`, `stopJoint`, `stopAll` bypass the write queue and go to the bus
   immediately. Everything else is queued behind the status poll. A move is
   acknowledged when the servo has accepted the command, **not** when it has arrived.
7. One `executeLinearMove` at a time. Send `abortLinearPath` (and `stopAll` if you want
   the arm to hold) to cancel.
8. Never open the serial port yourself while the service runs, and never run two
   servers against one bus.

---

## 5. Reading the arm

### Joint status

Each entry in `status.joints[]` (also returned by `getStatus`):

| Field | Meaning |
|---|---|
| `joint` | 1 to 6 |
| `available` | a servo answered on this joint's ID at discovery |
| `angleDegrees` | current angle, degrees, 0 at the servo's calibrated centre |
| `position` | raw servo steps, 0 to 4095 (2048 is 0°) |
| `isMoving` | servo reports motion in progress |
| `speed`, `load`, `voltage`, `temperature` | servo telemetry (raw speed, load %, volts, °C) |
| `torqueEnabled` | false after `stopJoint`/`stopAll`, a watchdog trip or an overload trip |
| `readStale` | this joint's last poll failed; the values are the last good ones |
| `pollError` | reason for the stale read, when stale |
| `lastGoodAt` | ms timestamp of the last good read |

`cacheAgeMs` is the age of the oldest joint reading. Under normal conditions it is
20 to 40 ms. Hundreds of ms means a joint is timing out; seconds means the bus is in
trouble.

The worker filters out single implausible samples (a position that would imply more
than 400°/s), so a corrupted frame does not reach you. A genuine jump, such as a joint
pushed by hand with torque off, shows up after three consistent samples.

### Waiting for a move to finish

The reply to `moveJoint` arrives before the joint has moved. To know a move is done:

1. Send all the joint moves for the step and wait for each reply (the bus drain).
2. Only then start watching `status` pushes.
3. Treat the move as complete when **no available joint reports `isMoving` for three
   consecutive pushes** (about 60 ms). Put a timeout on it (the app uses 30 s).

Status pushes that arrive while the moves are still being written to the bus can show
`isMoving:false` for joints that have not started yet, which is why step 2 matters.

### Other reads

- `getJointConfigs`: which joints have a responding servo.
- `getEndTool`: which tool is fitted (`toolTypeId`), its TCP length and whether the
  URDF knows it.
- `getServerDiagnostics`: bus tick timing, write queue depth, write timeout counters,
  connected client count, uptime, build id. Useful for a health check.
- `kinematicsGetInfo`: joint count, per-joint limits and max reach from the loaded URDF.

---

## 6. Joint-space control

`moveJoint` is the primitive everything else is built on.

```json
{"command":"moveJoint","joint":2,"angle":-30.0,"speed":455,"requestId":12}
```

- `angle` is in degrees. The server clamps only to ±180°. **It does not enforce the
  mechanical joint limits**; see section 8.
- `speed` is in servo steps per second. 1° is 11.37 steps, so 40°/s is 455 and 100°/s
  is 1137. The default when omitted is 1500 (about 132°/s). The hard maximum is 3400.
- Acceleration is a separate per-servo register in units of 100 steps/s², set with
  `setAcceleration` (0 to 254). The server does not set it; the servos keep whatever
  was last written (the pendant writes 30 on connect). With a low value such as 5 a
  servo needs over 100° of travel to reach 100°/s, so `speed` appears to have no
  effect on short moves. If speeds look wrong, check acceleration first.
- To make several joints **arrive together**, scale each joint's speed by its travel:
  `speed_i = base_speed * |delta_i| / max_delta`, with a floor of about 50 steps/s.
  To make the **tool tip** move at a given mm/s, compute the straight-line distance of
  the step, divide by the speed to get a duration, then give each joint
  `|delta_i| / duration`, capped at about 120°/s. Both patterns are in
  `electron-app/app.js` (`computeCoordinatedSpeeds`, `computeTipSpeeds`).
- `stopJoint` and `stopAll` disable torque. The arm will sag under gravity. To hold
  position without moving, do not send a stop; the servo holds its last target. The
  next `moveJoint` re-enables torque automatically.
- `setTorqueAll` with `enabled:false` releases all joints for hand positioning; read
  the angles from `status` while the arm is limp to teach positions.
- The worker re-sends a move up to four times, 30 ms apart, if a servo fails to
  acknowledge. If you still get `"Failed to move joint N: Write timeout"`, wait about
  400 ms and send the same move again; a re-sent move is idempotent. The app retries
  twice this way. Joint 2 has been seen to go silent for several hundred ms under load.
- Three consecutive poll failures put a joint into a 2 s backoff during which its
  status is stale. Moves to it still go out.

---

## 7. Kinematics and Cartesian control

### Conventions

- Units are millimetres and degrees. Angles in `jointAngles` arrays are in joint
  order 1 to 6.
- The world frame is right-handed with the origin at the base: **+X forward, +Y on the
  anticlockwise side when viewed from above, +Z up**. Joint 1 positive turns the arm
  clockwise from above, which puts the tool at negative Y.
- The tool centre point (TCP) is the tip of whatever end tool the server has detected
  (`endTool.tool.lengthMm`). With no tool present it is the mount face. The same XYZ
  resolves to different joint angles for different tools, so re-solve after a tool
  change rather than reusing stored angles.
- Tool orientation is given as the direction the tool points, plus an optional spin:
  `{"x":0,"y":0,"z":-1,"rotation":90}` is straight down with the jaws turned 90°.
  `rotation` 0 is a fixed world-aligned reference, so as the base swings the server
  counter-rotates joint 6 to keep the tool's heading constant in the room. Positive
  rotation turns the tool anticlockwise viewed from above. Joint 6 is limited to
  ±90°, so rotations beyond that are clamped.

### Commands

| Command | Params | Result |
|---|---|---|
| `kinematicsForwardKinematics` | `jointAngles:number[]` | `result.position {x,y,z}` and `result.rotation` (4×4 matrix; the tool points along the negated third column) |
| `kinematicsForwardKinematicsBatch` | `jointAnglesList:number[][]` | `positions[]` |
| `kinematicsInverseKinematics` | `targetPose:{x,y,z, orientation?:{x,y,z,rotation?}}`, `initialAngles?:number[]` | `result`: joint angles array, or `null` if unreachable (8 mm tolerance) |
| `kinematicsRefineOrientationWithAccuracy` | `targetPose:{x,y,z}`, `baseAngles`, `desiredOrientation:{x,y,z,rotation?}`, `referenceAngles?` | `result:{angles, positionErrorMm, orientationErrorDeg, spinErrorDeg, achievedPosition}` |
| `kinematicsApplyToolSpin` | `angles`, `orientation:{x,y,z,rotation}` | `result:{angles, spinErrorDeg}` with only joint 6 changed |
| `kinematicsGetInfo` | none | limits and reach |
| `executeLinearMove` | `startAngles`, `targetPose`, `desiredOrientation?`, `stepMm?` (2), `speedMmPerSec?` (50) | `linearPathStarted` now, `linearPathComplete` later |

### Recommended point-to-point sequence

1. Read the current angles from the latest `status` push.
2. `kinematicsInverseKinematics` with `targetPose` including `orientation`, seeded with
   the current angles. Seeding with the current pose keeps the solver in the arm's
   present configuration and avoids elbow flips.
3. `kinematicsRefineOrientationWithAccuracy` with the same target, the IK result as
   `baseAngles`, the same orientation, and the current angles as `referenceAngles`.
   Check `positionErrorMm` and `orientationErrorDeg` against your tolerance.
4. Send one `moveJoint` per joint with coordinated speeds (section 6).
5. Wait for motion complete (section 5).

Costs: IK takes 0.2 to 0.4 s on the Pi. Refinement returns in a few ms when the IK
result is already within 1.5 mm and 3°, otherwise it runs a grid search that takes
about 2 s **and blocks the server's event loop**, during which no status reaches any
client. Do not call it at high rate; for a jog-style stream of small moves use IK alone.

Reachability: a 4° pointing error at full reach near the table is a known limit of the
solver (position is prioritised over orientation). If `orientationErrorDeg` matters,
try a slightly less extended target.

### Straight-line moves

`executeLinearMove` interpolates the tip along a straight line on the server with a
fixed step and speed, driving all joints together. It needs the control session, runs
one path at a time, and reports `linearPathComplete` on your connection when done. It
produces many small bus writes, so it is the move most exposed to a noisy bus; keep
`stepMm` at 2 or above.

### Joint limits (from `kinematics.urdf`)

| Joint | Limits |
|---|---|
| 1 base yaw | ±180° |
| 2 shoulder pitch | −90° to +40° |
| 3 elbow pitch | ±90° |
| 4 wrist roll | ±90° |
| 5 wrist pitch | −5° to +90° |
| 6 tool roll | ±90° |

The kinematics commands respect these. `moveJoint` does not. Clamp in your client.

---

## 8. Rules that are not enforced for you

- **Do not change the joint zero positions.** The arm is supplied pre-zeroed: each
  servo's zero-point calibration (EEPROM register 31, "Position correction", also
  reachable through the servo's own centre-calibration command) is set so that 0° on
  every joint is the physical home pose the URDF describes, with the arm upright and
  the tool pointing down. The kinematics solver has no other reference. Shifting a
  zero, re-centring a servo, or fitting a replacement servo without restoring the same
  physical zero makes every forward and inverse kinematics result wrong by that
  offset, so XYZ moves land in the wrong place while joint-angle moves still look
  fine. Treat the zero offsets as part of the mechanical build, and if a servo has
  to be replaced, zero it with the joint physically at the home pose before using
  any Cartesian command.
- **Joint limits** on `moveJoint` (above). Exceeding them can drive a link into the base
  or the table.
- **Dead zones and collision avoidance.** The pendant plans "up, over, down" paths
  around user-defined boxes; the server has no notion of them. Plan your own
  approach heights.
- **Load.** The arm is a classroom-grade 6-axis with hobby-class servos. The shoulder
  (joint 2) at full reach and low height is the most stressed pose and the one that
  produces bus timeouts and overload trips. Keep payloads light and speeds moderate
  there.
- **Overload protection** lives in each servo's EEPROM. If a joint goes limp under load
  and recovers slowly, it has tripped to its protection torque. Those registers are
  readable with `readServoEeprom` and writable with `writeServoEeprom`, but changing
  them is a commissioning decision, not something a client should do at runtime.
- **Torque watchdog.** The servos self-disable about 1 s after the last bus write. The
  worker broadcasts a heartbeat every 700 ms, so you do not need to keep writing, but
  if the worker crashes the arm will go limp within a second. Design your cell so that
  is safe.
- **EEPROM writes** (`writeServoEeprom`, `writeServoEepromRaw`) are persistent and the
  raw variant needs the lock password. Treat them as configuration tooling, not control.

---

## 9. End tool (bus ID 64)

The end tool is an ESP32 that answers on the servo bus as ID 64 using the ST3215 packet
format, but exposes a register map of its own (`End Tool API ESP32/REGISTER_MAP.md`).
It carries a hobby-servo output (the gripper), two PWM FET outputs with current sensing
(pump and valve on the pneumatic tool) and two ADC inputs. The current firmware
implements registers 0 to 56 only; see "Not implemented" below.

### Tool types the server knows

The tool reports a type ID in register 3. The server reads it every 15 s (and
immediately after `refreshEndTool`), pushes an `endTool` message when it changes, and
switches the kinematics TCP to the matching `<end_tool>` entry in `kinematics.urdf`.

| `toolTypeId` | Label | TCP offset from mount face | `controls` | Notes |
|---|---|---|---|---|
| 0 | Unassigned | 0 mm (TCP is the mount face) | none | Also used when no tool answers |
| 1 | Pneumatic vacuum and valve | 119.0 mm down, 2.0 mm forward (suction cup face) | `pump`, `solenoid` | Pump on PWM1, valve on PWM2 |
| 2 | Servo motor (gripper) | 120.2 mm down (midpoint of the closed fingers) | `servo` | Hobby servo on the servo output |
| 3 | Pen | 155 mm down (pen tip, retracted) | none | Marked `provisional`: length is a placeholder |

An ID with no URDF entry leaves the TCP at the mount face and `endTool.known` false.
Because the TCP moves with the tool, **re-solve XYZ targets after a tool change**; joint
angles stored for one tool put a different tool somewhere else.

The `endTool` push carries `present`, `toolTypeId`, `known`, `tool:{id,label,lengthMm,
controls,provisional}`, `tools` (every URDF entry) and `lastError`.

### Commands and accepted values

All tool writes need the control session. All tool reads work without it. Every value
is clamped server-side to the range shown; out-of-range numbers are not rejected.

| Command | Parameters | Range and effect |
|---|---|---|
| `toolSetServoEnabledAndAngle` | `angle` | 0 to 180 degrees, clamped. Writes enable = 1 and the angle in one packet, so it works from the boot state. This is the gripper command: 0 is closed, 180 is fully open (the firmware maps it to the 8-bit position as `angle × 255 / 180`). The pendant and programs use 0 and 180. |
| `toolSetServoAngle` | `angle` | 0 to 180, clamped. Angle only; the servo must already be enabled or nothing moves. |
| `toolSetServoPosition` | `position` | 0 to 255, clamped. Raw position mapped linearly onto the pulse range (default 1000 to 2000 µs). |
| `toolSetServoEnabled` | `enabled` | boolean. **Omitting it means true.** `false` removes drive from the servo, so a loaded gripper may relax. |
| `toolGetServoState` | none | `{enabled, currentPosition8bit, currentAngle}`: the last *applied* command, not a measured position. |
| `toolSetPwm` | `pwm1Duty`, `pwm2Duty`, `enable1`, `enable2` | Duties 0 to 255, clamped; a missing duty is 0. Enables are booleans and **a missing enable means true**, so always send all four fields. Both channels are written together. The pendant drives the pump as PWM1 at duty 80 and the valve as PWM2 at duty 255. |
| `toolGetPwmState` | none | `{pwm1Duty, pwm2Duty, pwmControl}` where `pwmControl` bit 0 is PWM1 enable and bit 1 is PWM2 enable. |
| `toolReadCurrents` | none | `{pwm1CurrentRaw, pwm2CurrentRaw, servoCurrentRaw}`; despite the names these are milliamps on the current firmware. |
| `toolReadAdc` | none | `{adc0Raw, adc1Raw, adc0mV, adc1mV}`. |
| `toolGetIdentity` | none | `{protocolVersion, firmwareMajor, firmwareMinor, toolTypeId}`. `firmwareMinor` 2 or higher means the hobby-servo registers exist. |
| `toolPing` | none | `{ok}`. |

### Not implemented on the current firmware

The server still exposes four commands written against an earlier register spec.
The firmware only services registers 0 to 56, so:

- **`toolSetWatchdog` must not be used.** It writes a 16-bit timeout to registers 6
  and 7, which on this firmware are *PWM2 duty* and the *PWM control flags*. A
  "timeout" of, say, 2000 ms would set PWM2 to duty 208 and switch PWM outputs on.
- `toolGetStatus` reads registers 66 and 68, which do not exist; expect an error or
  meaningless values. The status-flag bits listed in the firmware document (overcurrent,
  watchdog, forced-off) are not set by this firmware.
- `toolClearFaults` and `toolReset` write registers 65 and 64, which do not exist, so
  they do nothing.

### Behaviour and rules

- The hobby servo is **disabled at boot** until something writes enable = 1. Use
  `toolSetServoEnabledAndAngle` for the first command after power-up.
- There is **no watchdog on the tool**. Pump, valve and servo stay exactly as last
  commanded if your client crashes or disconnects. Switch outputs off
  (`toolSetPwm` with both duties 0 and both enables false) and park the gripper before
  you exit, and treat that as part of your error handling.
- A reply confirms the command was accepted, not that the gripper has moved. Allow
  roughly 500 ms for the gripper to travel before the next motion; the pendant's
  programs wait that long after every open or close.
- The tool answers more slowly than the servos. The server gives each tool write a
  1 s timeout and up to five attempts 150 ms apart, so a tool command can take most of
  a second to fail. Do not issue tool commands faster than that budget allows.
- Both PWM outputs are written together by `toolSetPwm`. To change one channel,
  resend the other channel's current values with it.
- The servo pulse range (registers 51 to 54, default 1000 to 2000 µs) and the tool
  type ID (register 3) have no server command; they are configuration set with raw
  register writes on the tool and should be left alone in normal use.

---

## 10. Minimal session

```json
→ {"command":"takeControl","label":"cell-pc","requestId":1}
← {"type":"controlStatus","youHaveControl":true,...,"requestId":1}

→ {"command":"kinematicsInverseKinematics","targetPose":{"x":223,"y":0,"z":30,"orientation":{"x":0,"y":0,"z":-1,"rotation":0}},"initialAngles":[0,-26.6,0.5,0,25.4,0],"requestId":2}
← {"type":"kinematicsInverseResult","result":[-0.5,-33.9,-0.8,-0.2,30.6,0],"requestId":2}

→ {"command":"kinematicsRefineOrientationWithAccuracy","targetPose":{"x":223,"y":0,"z":30},"baseAngles":[-0.5,-33.9,-0.8,-0.2,30.6,0],"desiredOrientation":{"x":0,"y":0,"z":-1,"rotation":0},"referenceAngles":[0,-26.6,0.5,0,25.4,0],"requestId":3}
← {"type":"kinematicsRefineOrientationResult","result":{"angles":[...],"positionErrorMm":0.3,"orientationErrorDeg":0.5,"spinErrorDeg":0.0},"requestId":3}

→ {"command":"moveJoint","joint":1,"angle":-0.5,"speed":60,"requestId":4}
→ {"command":"moveJoint","joint":2,"angle":-33.9,"speed":455,"requestId":5}
   ... joints 3 to 6 ...
← {"type":"success","message":"Servo 2 moving to -33.9° at 455 step/s","requestId":5}
   ... then watch "status" pushes until no joint reports isMoving for 3 pushes ...

→ {"command":"toolSetServoEnabledAndAngle","angle":0,"requestId":10}
← {"type":"success",...,"requestId":10}

→ {"command":"releaseControl","requestId":11}
```

---

## 11. Reference clients and scripts

- `API Examples/Python/robot_arm/client.py`: `connect`, `get_status`, `get_joint_configs`,
  `move_joint`, `stop_joint`, `stop_all_joints`, `set_speed`, `set_torque_all`,
  `rescan_servos`, `tool_*`, `kinematics_load_urdf`, `kinematics_forward`,
  `kinematics_inverse`. `example_basic.py` and `example_end_tool_servo.py` show the flow.
  The wrapper predates the control session; send `takeControl` through its generic
  `request` method before moving.
- `API Examples/CSharp` and `API Examples/Cpp`: the same surface in .NET and C++.
- `electron-app/robotArmClient.js`: the most complete client. Worth reading for
  `waitForMotionComplete`, the retry policy in `moveJoint`, and `executeLinearMove`.
- `test-st3215.js`: bus smoke test for one servo (service stopped).
- `ik_test.js`: runs the server-side solver on sample targets without hardware.
- `servo-tuner.js`: PID sweep that writes to servo EEPROM (service stopped, arm near
  home, stand clear).

---

## 12. Troubleshooting quick table

| Symptom | Likely cause | What to do |
|---|---|---|
| Every write returns `controlRequired` | another client holds control, or yours idled out | `getControlStatus`, then `takeControl` (with `force` or the lock password) |
| `Write timeout` on one joint, others fine | that servo missed the acknowledgement, usually under load | wait 400 ms and resend; check the cable and connectors at that joint if it recurs |
| `cacheAgeMs` climbs into seconds, joints show `readStale` | bus or power problem, or a maintenance script holding the port | check `journalctl -u st3215-server`, confirm nothing else has `/dev/serial0` |
| Arm goes limp for a moment and slowly recovers | overload protection tripped on a servo | reduce load or speed at that pose; review protection registers with `readServoEeprom` |
| Speed setting makes no visible difference | servo acceleration register is low | `setAcceleration` to about 30 on each joint |
| `Unknown command` | server older than this guide | `git pull` on the Pi and restart the service |
| `servoWorkerCrashed` push | serial process died | wait for the automatic restart; status resumes within a few seconds |
