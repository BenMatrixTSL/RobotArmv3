#!/bin/bash
#
# Install Chromium kiosk mode for the robot arm UI (starts after desktop login).
# Serves index.html locally and opens Chromium fullscreen — lighter than Electron.
#
# Prerequisites on the Pi:
#   - Raspberry Pi OS with desktop (not Lite-only)
#   - Auto-login to desktop recommended (Raspberry Pi Configuration → Desktop)
#   - Chromium: sudo apt install -y chromium
#   - ST3215 server already installed (install-service.sh in raspberry-pi-control-st3215)
#
# Usage:
#   cd electron-app
#   chmod +x install-kiosk-service.sh start-kiosk.sh uninstall-kiosk-service.sh
#   sudo ./install-kiosk-service.sh
#
# Optional path (e.g. /opt/RobotArm/electron-app):
#   sudo ./install-kiosk-service.sh /opt/RobotArm/electron-app

set -e

SERVICE_NAME="robot-arm-kiosk.service"
KIOSK_PORT="3080"
# 1 = single screen (default) | 2 | all — see KIOSK_SETUP.md
KIOSK_SCREENS="${ROBOT_ARM_KIOSK_SCREENS:-1}"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

echo "=========================================="
echo "Robot Arm UI — install Chromium kiosk"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run this script with sudo"
    echo "  sudo ./install-kiosk-service.sh"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/index.html" ]; then
    echo "Error: index.html not found in: $INSTALL_DIR"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/start-kiosk.sh" ]; then
    echo "Error: start-kiosk.sh not found in: $INSTALL_DIR"
    exit 1
fi

if [ -n "$SUDO_USER" ]; then
    SERVICE_USER="$SUDO_USER"
else
    SERVICE_USER="pi"
fi

if ! id "$SERVICE_USER" &>/dev/null; then
    echo "Error: user '$SERVICE_USER' does not exist."
    exit 1
fi

SERVICE_UID="$(id -u "$SERVICE_USER")"

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed."
    exit 1
fi

if ! command -v chromium-browser >/dev/null && ! command -v chromium >/dev/null; then
    echo "Chromium is not installed. Installing..."
    apt-get update
    apt-get install -y chromium || apt-get install -y chromium-browser
fi

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER"
echo "Kiosk HTTP port: $KIOSK_PORT (local only)"
echo "Kiosk screens:   $KIOSK_SCREENS (all = one browser per HDMI output)"
echo "UI URL:          http://127.0.0.1:$KIOSK_PORT/index.html?kiosk=1"
echo ""
echo "Note: The Pi must boot to the desktop (user logged in) so DISPLAY=:0 works."
echo "      Enable auto-login in Raspberry Pi Configuration if the browser does not start."
echo ""

echo "Step 1: Fix folder ownership"
REPO_DIR="$INSTALL_DIR"
while [ "$REPO_DIR" != "/" ]; do
    if [ -d "$REPO_DIR/.git" ]; then
        chown -R "$SERVICE_USER:$SERVICE_USER" "$REPO_DIR"
        sudo -u "$SERVICE_USER" git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true
        echo "  Git repo owned by $SERVICE_USER: $REPO_DIR"
        break
    fi
    REPO_DIR="$(dirname "$REPO_DIR")"
done
chown -R "$SERVICE_USER:$SERVICE_USER" "$INSTALL_DIR"
chmod +x "$INSTALL_DIR/start-kiosk.sh"
echo "  Done."
echo ""

echo "Step 2: Install systemd service"
TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|SERVICE_UID|$SERVICE_UID|g" \
    -e "s|KIOSK_PORT|$KIOSK_PORT|g" \
    -e "s|KIOSK_SCREENS|$KIOSK_SCREENS|g" \
    "$INSTALL_DIR/robot-arm-kiosk.service" > "$TEMP_SERVICE"
cp "$TEMP_SERVICE" "/etc/systemd/system/$SERVICE_NAME"
rm -f "$TEMP_SERVICE"
echo "  Copied to /etc/systemd/system/$SERVICE_NAME"
echo ""

echo "Step 3: Enable and start service"
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"
echo "  Done."
echo ""

echo "=========================================="
echo "Kiosk installation complete"
echo "=========================================="
echo ""
echo "Useful commands:"
echo "  sudo systemctl status $SERVICE_NAME"
echo "  sudo systemctl restart $SERVICE_NAME"
echo "  sudo systemctl stop $SERVICE_NAME"
echo ""
echo "Test manually (while logged into the desktop):"
echo "  cd $INSTALL_DIR"
echo "  ./start-kiosk.sh"
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
