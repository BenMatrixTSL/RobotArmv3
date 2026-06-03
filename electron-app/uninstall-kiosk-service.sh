#!/bin/bash
#
# Remove the Chromium kiosk service (user and old system-wide).
#
# Usage:
#   sudo ./uninstall-kiosk-service.sh

set -e

SERVICE_NAME="robot-arm-kiosk.service"

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

echo "Removing system-wide service (if present) ..."
systemctl stop "$SERVICE_NAME" 2>/dev/null || true
systemctl disable "$SERVICE_NAME" 2>/dev/null || true
rm -f "/etc/systemd/system/$SERVICE_NAME"
systemctl daemon-reload

echo "Removing user service for $SERVICE_USER ..."
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

echo "Done."
