# Chromium kiosk mode on the Raspberry Pi

Run the robot arm **web UI** in Chromium fullscreen on the Pi touchscreen. This is much lighter than running the full Electron app on the Pi.

The ST3215 server (`raspberry-pi-control-st3215`) must still run separately — install that first with `install-service.sh`.

## What it does

1. Starts a small **Python HTTP server** on `127.0.0.1:3080` (serves `electron-app/` files).
2. Opens **Chromium in kiosk mode** to `index.html?kiosk=1`.
3. The page **auto-connects** to `127.0.0.1:8080` (local WebSocket server).

### Two displays (HDMI-1 and HDMI-2)

By default the installer sets `ROBOT_ARM_KIOSK_SCREENS=all`. When **two monitors** are connected, the script:

- Reads their position and size with `xrandr`
- Starts **one separate Chromium instance per screen** (each with its own profile folder)
- Places each window on the correct monitor (`--window-position` / `--window-size`)

Both show the same UI. **Important:** each browser opens its own WebSocket to the server, so both can send commands. Use one touchscreen for control, or set `ROBOT_ARM_KIOSK_SCREENS=1` for a single fullscreen window only.

```bash
# Install with dual-screen kiosk (default)
sudo ./install-kiosk-service.sh

# Single display only (one fullscreen window)
sudo ROBOT_ARM_KIOSK_SCREENS=1 ./install-kiosk-service.sh /opt/RobotArm/electron-app

# Exactly two windows (first two outputs from xrandr)
sudo ROBOT_ARM_KIOSK_SCREENS=2 ./install-kiosk-service.sh /opt/RobotArm/electron-app

# Test dual screen manually
ROBOT_ARM_KIOSK_SCREENS=all ./start-kiosk.sh
```

Check monitor layout:

```bash
xrandr --query
```

## Requirements

- Raspberry Pi OS **with desktop** (not Lite-only, unless you add a display server yourself).
- **Auto-login** to the desktop is strongly recommended (`raspi-config` or Raspberry Pi Configuration → Desktop → Auto Login).
- **Chromium** installed: `sudo apt install -y chromium`
- **ST3215 server** installed and running (`st3215-server.service`).

## Install (on the Pi)

```bash
cd /opt/RobotArm/electron-app
# or: cd ~/RobotArmv3/electron-app

chmod +x install-kiosk-service.sh start-kiosk.sh uninstall-kiosk-service.sh
sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app
```

Re-run after `git pull` (same as the server installer):

```bash
sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app
```

## Useful commands

```bash
sudo systemctl status robot-arm-kiosk.service
sudo systemctl restart robot-arm-kiosk.service
sudo systemctl stop robot-arm-kiosk.service
```

Test without systemd (logged into the desktop):

```bash
./start-kiosk.sh
```

## Uninstall

```bash
sudo ./uninstall-kiosk-service.sh
```

## Troubleshooting

| Problem | What to try |
|--------|-------------|
| Black screen / no browser | Enable desktop auto-login; reboot; check `sudo journalctl -u robot-arm-kiosk.service -e` |
| UI loads but “Disconnected” | `sudo systemctl status st3215-server.service` — server must be running on port 8080 |
| `DISPLAY` errors | You are not booting to the desktop, or no user is logged in |
| Chromium missing | `sudo apt install -y chromium` |

## Differences from Electron on a PC

- No separate “Update from git” buttons in the UI (use SSH/`git pull` on the Pi).
- 3D “popup window” is not available; inline 3D view still works.
- Git/settings features that need Electron APIs are disabled; pendant control and WebSocket commands work normally.
