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
| `getControlStatus` | See who has control. |

When a client disconnects, control is released automatically.

### Electron app

- Normal desktop app: calls **`takeControl('electron')`** on connect.
- **Kiosk** (`?kiosk=1`): does **not** take control (read-only display).

### Multiple kiosks

Safe: they only receive pushed status. They should not call `takeControl`.

### Desktop + kiosk

1. Open kiosk displays first (read-only).
2. Open desktop app and connect — it takes control and can move the arm.

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
