#!/bin/bash
#
# Install a systemd service that serves electron-app on port 80 (Python http.server).
#
# Usage:
#   cd electron-app
#   chmod +x install-web-server-service.sh start-web-server.sh
#   sudo ./install-web-server-service.sh
#
# Optional path:
#   sudo ./install-web-server-service.sh /opt/RobotArm/electron-app

set -e

SERVICE_NAME="robot-arm-web-server.service"
WEB_PORT="80"
WEB_BIND="0.0.0.0"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

echo "=========================================="
echo "Robot Arm UI — install web server (port 80)"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run this script with sudo"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/index.html" ]; then
    echo "Error: index.html not found in: $INSTALL_DIR"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/start-web-server.sh" ]; then
    echo "Error: start-web-server.sh not found in: $INSTALL_DIR"
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

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed."
    exit 1
fi

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER"
echo "Listen:          http://${WEB_BIND}:${WEB_PORT}/"
echo ""
echo "(Re-running this script is OK after git pull.)"
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
chmod +x "$INSTALL_DIR/start-web-server.sh"
echo "  Done."
echo ""

echo "Step 2: Install systemd service"
TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|WEB_PORT|$WEB_PORT|g" \
    -e "s|WEB_BIND|$WEB_BIND|g" \
    "$INSTALL_DIR/robot-arm-web-server.service" > "$TEMP_SERVICE"
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
echo "Web server installation complete"
echo "=========================================="
echo ""
echo "Open from another device: http://<pi-ip>/index.html"
echo ""
echo "Useful commands:"
echo "  sudo systemctl status $SERVICE_NAME"
echo "  sudo systemctl restart $SERVICE_NAME"
echo "  sudo systemctl stop $SERVICE_NAME"
echo ""
echo "Run once by hand (foreground):"
echo "  sudo $INSTALL_DIR/start-web-server.sh"
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
