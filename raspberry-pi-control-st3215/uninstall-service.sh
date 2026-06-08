#!/bin/bash
#
# Remove the ST3215 systemd service (does not delete your project files).
#
# Usage:
#   sudo ./uninstall-service.sh

set -e

SERVICE_NAME="st3215-server.service"

if [ "$EUID" -ne 0 ]; then
    echo "Error: run with sudo"
    exit 1
fi

echo "Stopping and disabling $SERVICE_NAME ..."
systemctl stop "$SERVICE_NAME" 2>/dev/null || true
systemctl disable "$SERVICE_NAME" 2>/dev/null || true

if [ -f "/etc/systemd/system/$SERVICE_NAME" ]; then
    rm -f "/etc/systemd/system/$SERVICE_NAME"
    echo "Removed /etc/systemd/system/$SERVICE_NAME"
fi

systemctl daemon-reload
echo "Done. Log files in /var/log/robot-arm-st3215/ were left in place."
