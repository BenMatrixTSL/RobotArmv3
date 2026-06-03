#!/bin/bash
#
# Install the ST3215 WebSocket server as a systemd service.
# - Starts automatically on boot
# - Restarts automatically if the process stops (Restart=always)
# - Writes debug logs under /var/log/robot-arm-st3215/
#
# Usage (on the Raspberry Pi):
#   cd raspberry-pi-control-st3215
#   chmod +x install-service.sh
#   sudo ./install-service.sh
#
# Optional: install from a different folder
#   sudo ./install-service.sh /opt/RobotArm/raspberry-pi-control-st3215
#
# Safe to re-run after "git pull" — fixes /opt permissions, runs npm install,
# refreshes the systemd unit, and restarts the server. Do not run "npm install"
# by hand in /opt unless you use sudo; use this script instead.

set -e

SERVICE_NAME="st3215-server.service"
LOG_DIR="/var/log/robot-arm-st3215"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

echo "=========================================="
echo "Robot Arm ST3215 server — install service"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run this script with sudo"
    echo "  sudo ./install-service.sh"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/server.js" ]; then
    echo "Error: server.js not found in: $INSTALL_DIR"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/st3215-server.service" ]; then
    echo "Error: st3215-server.service not found in: $INSTALL_DIR"
    exit 1
fi

# User that runs the service (the person who used sudo, or pi)
if [ -n "$SUDO_USER" ]; then
    SERVICE_USER="$SUDO_USER"
else
    SERVICE_USER="pi"
fi

if ! id "$SERVICE_USER" &>/dev/null; then
    echo "Error: user '$SERVICE_USER' does not exist."
    exit 1
fi

NODE_PATH="$(command -v node || true)"
if [ -z "$NODE_PATH" ]; then
    echo "Error: node is not installed. Install Node.js first, for example:"
    echo "  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -"
    echo "  sudo apt-get install -y nodejs"
    exit 1
fi

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER"
echo "Node:            $NODE_PATH"
echo "Log folder:      $LOG_DIR"
echo ""
echo "(Re-running this script is OK — use it to upgrade after git pull.)"
echo ""

# Serial port group
if getent group dialout >/dev/null; then
    usermod -aG dialout "$SERVICE_USER" 2>/dev/null || true
    echo "  User $SERVICE_USER added to dialout group (serial port access)."
fi

echo "Step 1: Fix folder ownership (needed if files are root-owned under /opt)"
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
echo "  $INSTALL_DIR owned by $SERVICE_USER"
echo ""

echo "Step 2: npm install"
cd "$INSTALL_DIR"
sudo -u "$SERVICE_USER" npm install
echo "  Done."
echo ""

echo "Step 3: Create log directory"

mkdir -p "$LOG_DIR"
chown "$SERVICE_USER:$SERVICE_USER" "$LOG_DIR"
chmod 755 "$LOG_DIR"
touch "$LOG_DIR/server.log" "$LOG_DIR/server-debug.log"
chown "$SERVICE_USER:$SERVICE_USER" "$LOG_DIR/server.log" "$LOG_DIR/server-debug.log"
echo "  Done."
echo ""

echo "Step 4: Install systemd service"
TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|NODE_PATH|$NODE_PATH|g" \
    -e "s|LOG_DIR|$LOG_DIR|g" \
    "$INSTALL_DIR/st3215-server.service" > "$TEMP_SERVICE"
cp "$TEMP_SERVICE" "/etc/systemd/system/$SERVICE_NAME"
rm -f "$TEMP_SERVICE"
echo "  Copied to /etc/systemd/system/$SERVICE_NAME"
echo ""

echo "Step 5: Enable and start service"
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"
echo "  Done."
echo ""

echo "=========================================="
echo "Installation complete"
echo "=========================================="
echo ""
echo "The server will:"
echo "  - Start automatically when the Pi boots"
echo "  - Restart automatically if it stops (systemd Restart=always)"
echo ""
echo "Useful commands:"
echo "  sudo systemctl status $SERVICE_NAME"
echo "  sudo systemctl restart $SERVICE_NAME"
echo "  sudo systemctl stop $SERVICE_NAME"
echo ""
echo "Log files:"
echo "  $LOG_DIR/server-debug.log   (startup, shutdown, errors)"
echo "  $LOG_DIR/server.log         (normal console output)"
echo ""
echo "Follow logs live:"
echo "  tail -f $LOG_DIR/server-debug.log"
echo "  sudo journalctl -u $SERVICE_NAME -f"
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
