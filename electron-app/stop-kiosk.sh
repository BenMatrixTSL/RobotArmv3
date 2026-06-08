#!/bin/bash
#
# Stop the robot arm kiosk (Chromium + start-kiosk.sh).
#
# Run as mxadmin with sudo (kiosk runs as mxuser at boot):
#   sudo bash /opt/RobotArm/electron-app/stop-kiosk.sh mxuser
#
# Or run as the desktop user:
#   bash /opt/RobotArm/electron-app/stop-kiosk.sh

KIOSK_USER="${1:-}"

if [ -z "$KIOSK_USER" ]; then
    if [ -n "$SUDO_USER" ]; then
        KIOSK_USER="$SUDO_USER"
    else
        KIOSK_USER="$USER"
    fi
fi

# Auto-login user runs the kiosk at boot — default mxuser if lightdm says so
if [ -z "$1" ] && [ -f /etc/lightdm/lightdm.conf ]; then
    AUTOLOGIN_USER="$(grep -E '^autologin-user=' /etc/lightdm/lightdm.conf 2>/dev/null | cut -d= -f2 | tr -d ' ')"
    if [ -n "$AUTOLOGIN_USER" ]; then
        KIOSK_USER="$AUTOLOGIN_USER"
    fi
fi

if ! id "$KIOSK_USER" &>/dev/null; then
    echo "Error: user '$KIOSK_USER' does not exist."
    exit 1
fi

echo "Stopping kiosk for user: $KIOSK_USER"

run_as_kiosk_user() {
    if [ "$(id -un)" = "$KIOSK_USER" ]; then
        "$@"
    else
        sudo -u "$KIOSK_USER" "$@"
    fi
}

# Stop launcher script
run_as_kiosk_user pkill -f "start-kiosk.sh" 2>/dev/null || true
sleep 1
run_as_kiosk_user pkill -9 -f "start-kiosk.sh" 2>/dev/null || true

# Stop Chromium using the kiosk profile folder
run_as_kiosk_user pkill -f "robot-arm-kiosk" 2>/dev/null || true
run_as_kiosk_user pkill -f "index.html?kiosk=1" 2>/dev/null || true
sleep 1
run_as_kiosk_user pkill -9 -f "robot-arm-kiosk" 2>/dev/null || true
run_as_kiosk_user pkill chromium 2>/dev/null || true
run_as_kiosk_user pkill -9 chromium 2>/dev/null || true

# Remove lock file so a fresh start works
rm -f "/home/${KIOSK_USER}/.robot-arm-kiosk/kiosk.lock" 2>/dev/null || true

echo "Done. Remaining processes (if any):"
pgrep -af "start-kiosk|chromium" -u "$KIOSK_USER" 2>/dev/null || echo "  (none)"
