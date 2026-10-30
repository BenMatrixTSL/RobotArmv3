# Multiple Electron / kiosk clients

Several UIs can connect to one Pi server (port 8080). The server centralizes bus access so clients do not fight over the serial port.

## Server-owned (buffered / cached)

| Data / action | How it works |
|---------------|--------------|
| **Joint angles & torque** | Polled every **100 ms** on the server (`STATUS_POLL_INTERVAL_MS`). Cached in `jointStatusCache`. |
| **getStatus** | Returns cache only — **no bus read** per client request. |
| **Status push** | After each poll, server **broadcasts** `type: status` to **all** connected WebSockets. |
| **getJointConfigs** | Returns `cachedJointConfigs` (rebuilt on init / rescan). |
| **Speed / acceleration** | Server skips bus write if value unchanged. |
| **setTorqueAll** | Skipped if torque state already matches. |
| **moveJoint queue** | Older queued moves for the **same joint** are dropped (keeps latest pendant target). |
| **rescanServos** | Rate-limited (10 s minimum between rescans). |
| **Kinematics** | Runs on the Pi in memory — no servo bus. |

## Control lock (one writer at a time)

Commands that **move** the arm or change torque/speed go through the control lock:

- `moveJoint`, `stopJoint`, `stopAllJoints`, `setTorqueAll`, `setSpeed`, `setAcceleration`, `rescanServos`, end-tool writes, etc.

- If **no client** has control yet, the **first** command (or `takeControl`) **auto-grants** control to that client.
- If **another** client already has control, move commands are rejected.

Read-only commands always work: `getStatus`, `getJointConfigs`, kinematics, etc.

| Command | Purpose |
|---------|---------|
| `takeControl` | Claim control (`force: true` takes it from another app). |
| `releaseControl` | Give up control so another app can move the arm. |
| `lockControl` | Take control (if free) and **lock** it — see below. |
| `unlockControl` | Clear the lock (holder: no password needed; anyone else: needs the lock password). |
| `verifyPassword` | Check the lock password without taking, locking or changing anything. Replies `{ type: "verifyPasswordResult", ok, message }`. Used by UIs to reveal write controls only for the real password. |
| `getControlStatus` | See who has control, and whether it's locked. |

When a client disconnects, control is released automatically (including a locked session).

An unlocked session is also released after 5 minutes with no control-gated command from its
holder. Any command that needs the session renews the timer - moves, stops, speed and torque,
end-tool commands, calibration register reads and writes, trainee trims and linear paths. Cached
reads such as `getStatus` or `getControlStatus` do not.

### Control lock

`lockControl` takes control if nobody has it, then marks the session **locked** for a
duration (`durationMs`, default 15 min, capped at 60 min). While locked:

- Other clients' `takeControl` — even with `force: true` — is rejected unless they supply
  the correct `password` (the control password - see below).
- The 5-minute idle-control timeout is suspended, so the lock survives periods with no
  movement commands.
- The lock ends when the holder calls `releaseControl`/`unlockControl`, when its duration
  elapses, or when the holder disconnects — whichever comes first.

`lockControl` and `unlockControl` both require the control password (the holder can
`unlockControl` without one — it's their own lock).

### Control password

The same password locks control and allows servo EEPROM writes (`writeServoEeprom`,
`writeServoEepromRaw` - the Calibration tab). It is **not in the repo**: it lives on the Pi
in `/etc/robot-arm-st3215/control-password`, readable only by root and the service user,
so students who can read the source cannot find it. Override the path with
`ROBOT_ARM_CONTROL_PASSWORD_FILE` if needed.

- Set or change it on the Pi: `sudo bash set-control-password.sh` (install-service.sh
  asks for one on first install). The server reads the file on every check, so a change
  takes effect without a restart.
- If the file is missing, every password check fails - lock and EEPROM writes are
  disabled rather than left open, and `verifyPassword` says no password is set up.

### Electron app

- Normal desktop app: connecting does **not** take control. The user presses
  **Take control** on the Connection tab (`takeControl('electron')`). If the link drops
  and comes back, the app re-takes control only if it held control before the drop.
- **Kiosk** (`?kiosk=1`): never takes control (read-only display).

### Multiple kiosks

Safe: they only receive pushed status. They should not call `takeControl`.

### Desktop + kiosk

1. Open kiosk displays first (read-only).
2. Open desktop app, connect, then press Take control to move the arm. Other desktop
   apps can connect at the same time and watch; refreshing one of them no longer
   steals control.

## Client polling

With server push enabled, the Electron app:

- Uses pushed status for the UI.
- Sends `getStatus` only as a **fallback** (every 2+ s if push stops).

You can still change **Settings → Update interval**; minimum fallback is 2 s when push is active.

## Tuning

```bash
# Poll interval (ms) — in st3215-server.service or environment
STATUS_POLL_INTERVAL_MS=100
```

## Deploy

```bash
cd /opt/RobotArm
git pull origin main
sudo ./raspberry-pi-control-st3215/install-service.sh /opt/RobotArm/raspberry-pi-control-st3215
```

Restart kiosk browsers so they load updated `app.js`.
