#!/bin/bash
#
# Install Chromium kiosk for the robot arm UI.
#
# Uses desktop AUTOSTART (most reliable on Pi OS Wayland).
# Do NOT use a system-wide systemd service — that causes MIT-MAGIC-COOKIE errors.
#
# Prerequisites:
#   - Raspberry Pi OS with desktop + auto-login for your user
#   - Chromium: sudo apt install -y chromium
#   - ST3215 server installed (raspberry-pi-control-st3215/install-service.sh)
#
# Usage:
#   cd /opt/RobotArm/electron-app
#   chmod +x install-kiosk-service.sh start-kiosk.sh
#   sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app
#
# Optional: install for a specific desktop user (must match auto-login):
#   sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app mxuser

set -e

SERVICE_NAME="robot-arm-kiosk.service"
AUTOSTART_NAME="robot-arm-kiosk.desktop"
KIOSK_PORT="80"
KIOSK_SCREENS="${ROBOT_ARM_KIOSK_SCREENS:-1}"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

KIOSK_DESKTOP_USER="$2"

echo "=========================================="
echo "Robot Arm UI — install Chromium kiosk"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run this script with sudo"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/index.html" ] || [ ! -f "$INSTALL_DIR/start-kiosk.sh" ]; then
    echo "Error: missing files in $INSTALL_DIR"
    exit 1
fi

AUTOLOGIN_USER=""
if [ -f /etc/lightdm/lightdm.conf ]; then
    AUTOLOGIN_USER="$(grep -E '^autologin-user=' /etc/lightdm/lightdm.conf 2>/dev/null | cut -d= -f2 | tr -d ' ')"
fi

if [ -n "$KIOSK_DESKTOP_USER" ]; then
    SERVICE_USER="$KIOSK_DESKTOP_USER"
elif [ -n "$AUTOLOGIN_USER" ]; then
    SERVICE_USER="$AUTOLOGIN_USER"
elif [ -n "$SUDO_USER" ]; then
    SERVICE_USER="$SUDO_USER"
else
    SERVICE_USER="pi"
fi

if ! id "$SERVICE_USER" &>/dev/null; then
    echo "Error: user '$SERVICE_USER' does not exist."
    exit 1
fi

if [ -n "$SUDO_USER" ] && [ -n "$AUTOLOGIN_USER" ] && [ "$SUDO_USER" != "$AUTOLOGIN_USER" ] && [ -z "$KIOSK_DESKTOP_USER" ]; then
    echo "NOTE: You ran sudo as '$SUDO_USER' but desktop auto-login is '$AUTOLOGIN_USER'."
    echo "      Installing kiosk for '$AUTOLOGIN_USER' (the user who gets the desktop at boot)."
    echo ""
fi

SERVICE_UID="$(id -u "$SERVICE_USER")"
SERVICE_HOME="$(eval echo "~$SERVICE_USER")"
AUTOSTART_DIR="$SERVICE_HOME/.config/autostart"
USER_UNIT_DIR="$SERVICE_HOME/.config/systemd/user"

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed."
    exit 1
fi

if ! command -v chromium-browser >/dev/null && ! command -v chromium >/dev/null; then
    echo "Chromium is not installed. Installing..."
    apt-get update
    apt-get install -y chromium || apt-get install -y chromium-browser
fi

run_as_user() {
    sudo -u "$SERVICE_USER" \
        XDG_RUNTIME_DIR="/run/user/$SERVICE_UID" \
        DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$SERVICE_UID/bus" \
        "$@"
}

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER"
echo "Kiosk port:      $KIOSK_PORT"
echo "Kiosk screens:   $KIOSK_SCREENS"
echo ""

echo "Step 1: Repo owned by git user (for git pull), scripts readable by $SERVICE_USER"
if [ -n "$SUDO_USER" ]; then
    GIT_OWNER="$SUDO_USER"
else
    GIT_OWNER="$SERVICE_USER"
fi
REPO_DIR="$INSTALL_DIR"
while [ "$REPO_DIR" != "/" ]; do
    if [ -d "$REPO_DIR/.git" ]; then
        chown -R "$GIT_OWNER:$GIT_OWNER" "$REPO_DIR"
        sudo -u "$GIT_OWNER" git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true
        echo "  Git repo owner: $GIT_OWNER ($REPO_DIR)"
        break
    fi
    REPO_DIR="$(dirname "$REPO_DIR")"
done
chmod -R a+rX "$INSTALL_DIR"
chmod +x "$INSTALL_DIR/start-kiosk.sh" "$INSTALL_DIR/check-kiosk.sh" "$INSTALL_DIR/stop-kiosk.sh" 2>/dev/null || true
sudo -u "$SERVICE_USER" git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true
if [ -f "$INSTALL_DIR/check-kiosk.sh" ]; then
    sed -i 's/\r$//' "$INSTALL_DIR/check-kiosk.sh" 2>/dev/null || true
fi
sed -i 's/\r$//' "$INSTALL_DIR/start-kiosk.sh" "$INSTALL_DIR/install-kiosk-service.sh" 2>/dev/null || true
echo "  Done."
echo ""

echo "Step 2: Remove old system-wide kiosk service (causes MIT-MAGIC-COOKIE errors)"
if [ -f "/etc/systemd/system/$SERVICE_NAME" ]; then
    echo "  WARNING: found old system service — removing it."
fi
systemctl stop "$SERVICE_NAME" 2>/dev/null || true
systemctl disable "$SERVICE_NAME" 2>/dev/null || true
rm -f "/etc/systemd/system/$SERVICE_NAME"
systemctl daemon-reload
echo "  Done."
echo ""

echo "Step 3: Remove user systemd kiosk (avoid duplicate browsers)"
if [ -f "$USER_UNIT_DIR/$SERVICE_NAME" ]; then
    run_as_user systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
    run_as_user systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true
    rm -f "$USER_UNIT_DIR/$SERVICE_NAME"
    run_as_user systemctl --user daemon-reload 2>/dev/null || true
    echo "  Removed $USER_UNIT_DIR/$SERVICE_NAME"
else
    echo "  No user systemd unit found."
fi
echo ""

echo "Step 4: Install desktop autostart (starts after graphical login)"
mkdir -p "$AUTOSTART_DIR"
TEMP_DESKTOP="/tmp/$AUTOSTART_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    "$INSTALL_DIR/robot-arm-kiosk.desktop" > "$TEMP_DESKTOP"
cp "$TEMP_DESKTOP" "$AUTOSTART_DIR/$AUTOSTART_NAME"
rm -f "$TEMP_DESKTOP"
chown -R "$SERVICE_USER:$SERVICE_USER" "$SERVICE_HOME/.config"
echo "  Installed $AUTOSTART_DIR/$AUTOSTART_NAME"
echo ""

echo "Step 5: Install labwc autostart (Pi OS Bookworm+ Wayland — often required)"
LABWC_AUTOSTART="$SERVICE_HOME/.config/labwc/autostart"
mkdir -p "$(dirname "$LABWC_AUTOSTART")"
if [ -f "$LABWC_AUTOSTART" ]; then
    grep -v "start-kiosk.sh" "$LABWC_AUTOSTART" > "${LABWC_AUTOSTART}.tmp" 2>/dev/null || true
    mv "${LABWC_AUTOSTART}.tmp" "$LABWC_AUTOSTART"
else
    touch "$LABWC_AUTOSTART"
fi
# start-kiosk.sh runs until Chromium closes — do not use lwrespawn (causes duplicate launches)
echo "$INSTALL_DIR/start-kiosk.sh &" >> "$LABWC_AUTOSTART"
chmod +x "$LABWC_AUTOSTART"
chown "$SERVICE_USER:$SERVICE_USER" "$LABWC_AUTOSTART"
echo "  Installed $LABWC_AUTOSTART"
echo ""

echo "Step 6: Write kiosk environment file"
KIOSK_ENV="$SERVICE_HOME/.config/robot-arm-kiosk.env"
cat > "$KIOSK_ENV" <<EOF
ROBOT_ARM_KIOSK_PORT=$KIOSK_PORT
ROBOT_ARM_KIOSK_SCREENS=$KIOSK_SCREENS
EOF
chown "$SERVICE_USER:$SERVICE_USER" "$KIOSK_ENV"
echo "  Installed $KIOSK_ENV"
echo ""

echo "=========================================="
echo "Installation complete"
echo "=========================================="
echo ""
echo "IMPORTANT:"
echo "  1. Enable desktop AUTO-LOGIN for $SERVICE_USER"
echo "     (raspi-config -> Boot -> Desktop Autologin)"
echo "  2. Reboot: sudo reboot"
echo ""
echo "After reboot, Chromium should open fullscreen automatically."
echo ""
echo "Test while logged into the desktop (as $SERVICE_USER):"
echo "  $INSTALL_DIR/start-kiosk.sh"
echo ""
echo "If kiosk does not appear, run diagnostics (as $SERVICE_USER):"
echo "  bash $INSTALL_DIR/check-kiosk.sh"
echo ""
echo "Or read the log:"
echo "  tail -50 $SERVICE_HOME/.robot-arm-kiosk/kiosk.log"
echo ""
echo "Do NOT use: sudo systemctl status robot-arm-kiosk"
echo "That checks the old broken system service."
echo ""
