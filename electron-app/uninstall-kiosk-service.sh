#!/bin/bash
#
# Remove the Chromium kiosk autostart and any old systemd services.
#
# Usage:
#   sudo ./uninstall-kiosk-service.sh

set -e

SERVICE_NAME="robot-arm-kiosk.service"
AUTOSTART_NAME="robot-arm-kiosk.desktop"

if [ "$EUID" -ne 0 ]; then
    echo "Error: run with sudo"
    exit 1
fi

if [ -n "$SUDO_USER" ]; then
    SERVICE_USER="$SUDO_USER"
else
    SERVICE_USER="pi"
fi

SERVICE_UID="$(id -u "$SERVICE_USER")"
SERVICE_HOME="$(eval echo "~$SERVICE_USER")"
USER_UNIT="$SERVICE_HOME/.config/systemd/user/$SERVICE_NAME"
AUTOSTART_FILE="$SERVICE_HOME/.config/autostart/$AUTOSTART_NAME"
KIOSK_ENV="$SERVICE_HOME/.config/robot-arm-kiosk.env"

echo "Removing system-wide service (if present) ..."
systemctl stop "$SERVICE_NAME" 2>/dev/null || true
systemctl disable "$SERVICE_NAME" 2>/dev/null || true
rm -f "/etc/systemd/system/$SERVICE_NAME"
systemctl daemon-reload

echo "Removing user systemd service (if present) ..."
if [ -f "$USER_UNIT" ]; then
    sudo -u "$SERVICE_USER" \
        XDG_RUNTIME_DIR="/run/user/$SERVICE_UID" \
        DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$SERVICE_UID/bus" \
        systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
    sudo -u "$SERVICE_USER" \
        XDG_RUNTIME_DIR="/run/user/$SERVICE_UID" \
        DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$SERVICE_UID/bus" \
        systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true
    rm -f "$USER_UNIT"
    sudo -u "$SERVICE_USER" \
        XDG_RUNTIME_DIR="/run/user/$SERVICE_UID" \
        DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$SERVICE_UID/bus" \
        systemctl --user daemon-reload 2>/dev/null || true
    echo "Removed $USER_UNIT"
fi

echo "Removing desktop autostart (if present) ..."
if [ -f "$AUTOSTART_FILE" ]; then
    rm -f "$AUTOSTART_FILE"
    echo "Removed $AUTOSTART_FILE"
fi

if [ -f "$KIOSK_ENV" ]; then
    rm -f "$KIOSK_ENV"
    echo "Removed $KIOSK_ENV"
fi

LABWC_AUTOSTART="$SERVICE_HOME/.config/labwc/autostart"
if [ -f "$LABWC_AUTOSTART" ]; then
    grep -v "start-kiosk.sh" "$LABWC_AUTOSTART" > "${LABWC_AUTOSTART}.tmp" 2>/dev/null || true
    mv "${LABWC_AUTOSTART}.tmp" "$LABWC_AUTOSTART"
    echo "Removed kiosk line from $LABWC_AUTOSTART"
fi

echo "Done."
