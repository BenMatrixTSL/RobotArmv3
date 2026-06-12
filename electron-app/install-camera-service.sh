#!/bin/bash
#
# Install USB camera MJPEG stream as a systemd service (port 8082).
#
# Prerequisites on the Pi:
#   sudo apt install -y ffmpeg v4l-utils
#   USB camera plugged in
#
# Usage:
#   sudo ./install-camera-service.sh /opt/RobotArm/electron-app
#   sudo ROBOT_ARM_CAMERA_DEVICE=/dev/video1 ./install-camera-service.sh /opt/RobotArm/electron-app

set -e

SERVICE_NAME="robot-arm-camera.service"
CAMERA_PORT="8082"
INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "$1" ]; then
    INSTALL_DIR="$(cd "$1" && pwd)"
fi

find_capture_device() {
  local dev

  if [ -n "$ROBOT_ARM_CAMERA_DEVICE" ]; then
    echo "$ROBOT_ARM_CAMERA_DEVICE"
    return 0
  fi

  if command -v v4l2-ctl >/dev/null 2>&1; then
    for dev in /dev/video*; do
      if [ -e "$dev" ] && v4l2-ctl -d "$dev" --all 2>/dev/null | grep -q "Video Capture"; then
        echo "$dev"
        return 0
      fi
    done
  fi

  # Many USB cameras use video0 for metadata and video1 for capture
  if [ -e /dev/video1 ]; then
    echo "/dev/video1"
    return 0
  fi

  echo "/dev/video0"
}

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

if ! command -v v4l2-ctl >/dev/null; then
    echo "Installing v4l-utils (for camera device detection)..."
    apt-get update
    apt-get install -y v4l-utils
fi

CAMERA_DEVICE="$(find_capture_device)"

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER (group: video)"
echo "Camera port:     $CAMERA_PORT"
echo "Camera device:   $CAMERA_DEVICE"
echo ""

# Service user must be in video group to open /dev/video*
usermod -aG video "$SERVICE_USER" 2>/dev/null || true

chmod +x "$INSTALL_DIR/start-camera-stream.sh"
chmod +x "$INSTALL_DIR/camera-stream.py" 2>/dev/null || true

TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|__CAMERA_PORT__|$CAMERA_PORT|g" \
    -e "s|__CAMERA_DEVICE__|$CAMERA_DEVICE|g" \
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
echo "List cameras:  v4l2-ctl --list-devices"
echo "Status:        sudo systemctl status $SERVICE_NAME"
echo "Logs:          sudo journalctl -u robot-arm-camera.service -n 30 --no-pager"
echo "Test snapshot: curl -s -o /tmp/cam-test.jpg -w 'HTTP %{http_code}\\n' --max-time 15 http://127.0.0.1:${CAMERA_PORT}/snapshot"
echo "Test stream:   curl -s -o /dev/null -w 'HTTP %{http_code}\\n' --max-time 5 http://127.0.0.1:${CAMERA_PORT}/stream"
echo ""
echo "If no picture, try another device:"
echo "  sudo ROBOT_ARM_CAMERA_DEVICE=/dev/video1 ./install-camera-service.sh $INSTALL_DIR"
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
