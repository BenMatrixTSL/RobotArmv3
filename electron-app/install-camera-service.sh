#!/bin/bash
#
# Install USB camera MJPEG stream as a systemd service (port 8082).
#
# Prerequisites on the Pi:
#   sudo apt install -y ffmpeg
#   USB camera plugged in (/dev/video0)
#
# Usage:
#   sudo ./install-camera-service.sh /opt/RobotArm/electron-app

set -e

SERVICE_NAME="robot-arm-camera.service"
CAMERA_PORT="8082"
CAMERA_DEVICE="/dev/video0"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

echo "=========================================="
echo "Robot Arm — install USB camera stream"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run with sudo"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/camera-stream.py" ] || [ ! -f "$INSTALL_DIR/start-camera-stream.sh" ]; then
    echo "Error: missing camera files in $INSTALL_DIR"
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

if ! command -v ffmpeg >/dev/null; then
    echo "Installing ffmpeg..."
    apt-get update
    apt-get install -y ffmpeg
fi

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER"
echo "Camera port:     $CAMERA_PORT"
echo "Camera device:   $CAMERA_DEVICE"
echo ""

chmod +x "$INSTALL_DIR/start-camera-stream.sh"
chmod +x "$INSTALL_DIR/camera-stream.py" 2>/dev/null || true

TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|CAMERA_PORT|$CAMERA_PORT|g" \
    -e "s|CAMERA_DEVICE|$CAMERA_DEVICE|g" \
    "$INSTALL_DIR/robot-arm-camera.service" > "$TEMP_SERVICE"

cp "$TEMP_SERVICE" "/etc/systemd/system/$SERVICE_NAME"
rm -f "$TEMP_SERVICE"

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

echo ""
echo "=========================================="
echo "Installation complete"
echo "=========================================="
echo ""
echo "Status:  sudo systemctl status $SERVICE_NAME"
echo "Logs:    sudo journalctl -u $SERVICE_NAME -n 30 --no-pager"
echo "Test:    curl -I http://127.0.0.1:${CAMERA_PORT}/stream"
echo ""
echo "Open the Camera tab in the robot arm app."
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
