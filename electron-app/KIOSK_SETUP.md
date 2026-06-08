# Chromium kiosk mode on the Raspberry Pi

Run the robot arm **web UI** in Chromium fullscreen on the Pi touchscreen. This is much lighter than running the full Electron app on the Pi.

The ST3215 server (`raspberry-pi-control-st3215`) must still run separately — install that first with `install-service.sh`.

## What it does

1. Starts a small **Python HTTP server** on `127.0.0.1:3080` (serves `electron-app/` files).
2. Opens **Chromium in kiosk mode** to `index.html?kiosk=1`.
3. The page **auto-connects** to `127.0.0.1:8080` (local WebSocket server) and opens the **Pendant Control** tab.

At boot, a **desktop autostart** entry runs the kiosk script after you log into the graphical desktop (auto-login recommended).

### Two displays (HDMI-1 and HDMI-2)

By default the installer sets `ROBOT_ARM_KIOSK_SCREENS=1` (one fullscreen window — most reliable at boot). For **two monitors**, set `ROBOT_ARM_KIOSK_SCREENS=all` when installing. When two monitors are connected, the script:

- Reads their position and size with `xrandr`
- Starts **one separate Chromium instance per screen** (each with its own profile folder)
- Places each window on the correct monitor (`--window-position` / `--window-size`)

Both show the same UI. **Important:** each browser opens its own WebSocket to the server, so both can send commands. Use one touchscreen for control, or set `ROBOT_ARM_KIOSK_SCREENS=1` for a single fullscreen window only.

```bash
# Install with single-screen kiosk (default)
sudo ./install-kiosk-service.sh

# Single display only (one fullscreen window)
sudo ROBOT_ARM_KIOSK_SCREENS=1 ./install-kiosk-service.sh /opt/RobotArm/electron-app

# Exactly two windows (first two outputs from xrandr)
sudo ROBOT_ARM_KIOSK_SCREENS=2 ./install-kiosk-service.sh /opt/RobotArm/electron-app

# All connected monitors
sudo ROBOT_ARM_KIOSK_SCREENS=all ./install-kiosk-service.sh /opt/RobotArm/electron-app

# Test dual screen manually
ROBOT_ARM_KIOSK_SCREENS=all ./start-kiosk.sh
```

Check monitor layout:

```bash
xrandr --query
```

## HTTP on port 80 (optional)

To serve the same UI on port 80 (e.g. for phones/tablets on the LAN), use `start-web-server.sh` or `install-web-server-service.sh` — see `README.md`. That is separate from the kiosk (port 3080).

## Requirements

- Raspberry Pi OS **with desktop** (not Lite-only, unless you add a display server yourself).
- **Auto-login** to the desktop is strongly recommended (`raspi-config` → System Options → Boot / Auto Login → Desktop).
- **Kiosk must be installed for the auto-login user.** If auto-login is `mxuser` but you run `sudo` as `mxadmin`, the installer now detects that and installs for `mxuser` automatically.
- **Chromium** installed: `sudo apt install -y chromium`
- **ST3215 server** installed and running (`st3215-server.service`).

## Install (on the Pi)

```bash
cd /opt/RobotArm/electron-app
# or: cd ~/RobotArmv3/electron-app

chmod +x install-kiosk-service.sh start-kiosk.sh uninstall-kiosk-service.sh

# If you get "command not found" or "bad interpreter", fix line endings first:
sed -i 's/\r$//' install-kiosk-service.sh start-kiosk.sh uninstall-kiosk-service.sh

sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app
# Or bypass the shebang entirely:
# sudo bash install-kiosk-service.sh /opt/RobotArm/electron-app

sudo reboot
```

Re-run after `git pull` (same as the server installer):

```bash
sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app
sudo reboot
```

## Easy diagnostics (start here if kiosk does not appear)

While logged into the Pi desktop, run:

```bash
cd /opt/RobotArm/electron-app
bash check-kiosk.sh
```

This prints **OK / FAIL / WARN** for each check and shows the last lines of the kiosk log.

## Useful commands

The kiosk starts from **desktop autostart** and **labwc autostart** (Pi OS Wayland) after graphical login.

Test manually while logged into the desktop (as `mxadmin`):

```bash
/opt/RobotArm/electron-app/start-kiosk.sh
```

Read the log if the browser does not appear:

```bash
tail -50 ~/.robot-arm-kiosk/kiosk.log
```

Check autostart is installed:

```bash
ls -la ~/.config/autostart/robot-arm-kiosk.desktop
cat ~/.config/labwc/autostart
```

### Do NOT use the old system service

```bash
# WRONG — this was the old broken approach:
sudo systemctl status robot-arm-kiosk.service
```

A system-wide service cannot access your Wayland/X11 session and causes:

```text
Invalid MIT-MAGIC-COOKIE-1 key
Missing X server or $DISPLAY
```

The installer removes that service and installs desktop autostart instead.

## Uninstall

```bash
sudo ./uninstall-kiosk-service.sh
```

## Troubleshooting

| Problem | What to try |
|--------|-------------|
| No browser at all after reboot | Enable desktop **auto-login** for your user, then reboot |
| Black screen / no browser | `tail -50 ~/.robot-arm-kiosk/kiosk.log` — look for display or Chromium errors |
| UI loads but “Disconnected” | `sudo systemctl status st3215-server.service` — server must be running on port 8080 |
| `DISPLAY` errors in log | You are not booting to the desktop — enable **auto-login** and reboot |
| `MIT-MAGIC-COOKIE-1` / `Missing X server` | Old **system** service still enabled — run `sudo ./install-kiosk-service.sh` again and reboot |
| Browser opens but not fullscreen | Check URL includes `?kiosk=1`; reinstall autostart with `install-kiosk-service.sh` |
| Chromium missing | `sudo apt install -y chromium` |
| Port 3080 already in use | Reboot, or `fuser -k 3080/tcp` then run `start-kiosk.sh` again |
| `command not found` (file exists) | `chmod +x install-kiosk-service.sh` then `sed -i 's/\r$//' install-kiosk-service.sh` — or run `sudo bash install-kiosk-service.sh /opt/RobotArm/electron-app` |
| `bad interpreter` / `/bin/bash^M` | Windows line endings — run `sed -i 's/\r$//' *.sh` in `electron-app`, then try again |

## Differences from Electron on a PC

- No separate “Update from git” buttons in the UI (use SSH/`git pull` on the Pi).
- 3D “popup window” is not available; inline 3D view still works.
- Git/settings features that need Electron APIs are disabled; pendant control and WebSocket commands work normally.
