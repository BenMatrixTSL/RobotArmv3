#!/bin/bash
#
# Install the kiosk watchdog: a systemd timer that runs every 30s, detects a
# hung or crashed kiosk Chromium (see kiosk-watchdog.sh for how), and
# restarts it automatically. Install the kiosk itself first
# (install-kiosk-service.sh).
#
# Usage:
#   cd /opt/RobotArm/electron-app
#   chmod +x install-kiosk-watchdog.sh kiosk-watchdog.sh
#   sudo ./install-kiosk-watchdog.sh /opt/RobotArm/electron-app
#
# Optional: install for a specific desktop user (must match the kiosk user):
#   sudo ./install-kiosk-watchdog.sh /opt/RobotArm/electron-app mxuser

set -e

SERVICE_NAME="robot-arm-kiosk-watchdog.service"
TIMER_NAME="robot-arm-kiosk-watchdog.timer"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

KIOSK_DESKTOP_USER="$2"

echo "=========================================="
echo "Robot Arm UI — install kiosk watchdog"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run this script with sudo"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/kiosk-watchdog.sh" ] || [ ! -f "$INSTALL_DIR/start-kiosk.sh" ]; then
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

echo "Install folder:  $INSTALL_DIR"
echo "Kiosk user:      $SERVICE_USER"
echo ""

sed -i 's/\r$//' "$INSTALL_DIR/kiosk-watchdog.sh" "$INSTALL_DIR/install-kiosk-watchdog.sh" 2>/dev/null || true
chmod +x "$INSTALL_DIR/kiosk-watchdog.sh"

echo "Step 1: Write $SERVICE_NAME"
sed -e "s|KIOSK_SCRIPT_DIR|$INSTALL_DIR|g" \
    -e "s|KIOSK_USER|$SERVICE_USER|g" \
    "$INSTALL_DIR/robot-arm-kiosk-watchdog.service" > "/etc/systemd/system/$SERVICE_NAME"
echo "  Installed /etc/systemd/system/$SERVICE_NAME"
echo ""

echo "Step 2: Write $TIMER_NAME"
cp "$INSTALL_DIR/robot-arm-kiosk-watchdog.timer" "/etc/systemd/system/$TIMER_NAME"
echo "  Installed /etc/systemd/system/$TIMER_NAME"
echo ""

echo "Step 3: Enable and start the timer"
systemctl daemon-reload
systemctl enable --now "$TIMER_NAME"
echo "  Done."
echo ""

echo "=========================================="
echo "Installation complete"
echo "=========================================="
echo ""
echo "Check timer status:"
echo "  systemctl status $TIMER_NAME"
echo ""
echo "Check last watchdog run:"
echo "  systemctl status $SERVICE_NAME"
echo "  journalctl -t robot-arm-kiosk-watchdog -n 50"
echo ""
echo "Uninstall:"
echo "  sudo systemctl disable --now $TIMER_NAME"
echo "  sudo rm /etc/systemd/system/$SERVICE_NAME /etc/systemd/system/$TIMER_NAME"
echo "  sudo systemctl daemon-reload"
echo ""
