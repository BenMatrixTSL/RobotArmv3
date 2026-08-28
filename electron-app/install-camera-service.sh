#!/bin/bash
#
# Install USB camera MJPEG stream as a systemd service (port 8082).
#
# Prerequisites on the Pi:
#   sudo apt install -y python3-opencv ffmpeg v4l-utils
#   USB camera plugged in
#
# Usage:
#   sudo ./install-camera-service.sh /opt/RobotArm/electron-app
#   sudo ROBOT_ARM_CAMERA_DEVICE=/dev/video1 ./install-camera-service.sh /opt/RobotArm/electron-app
#   sudo ROBOT_ARM_CAMERA_EXPOSURE=80 ROBOT_ARM_CAMERA_BRIGHTNESS=30 ROBOT_ARM_CAMERA_GAMMA=400 \
#     ./install-camera-service.sh /opt/RobotArm/electron-app
#
# Tune exposure/brightness/gamma live first (values are site/lighting-specific):
#   v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1,exposure_time_absolute=80
#   v4l2-ctl -d /dev/video0 --set-ctrl=brightness=30,gamma=400
# then re-run this script with the working values so they persist across restarts/reboots.

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

  # Only nodes that list pixel formats are real capture devices.
  # /dev/video1 is often metadata-only ("Inappropriate ioctl for device").
  if command -v v4l2-ctl >/dev/null 2>&1; then
    for dev in /dev/video*; do
      if [ -e "$dev" ] && v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | grep -qE '^[[:space:]]*\[[0-9]+\]:'; then
        echo "$dev"
        return 0
      fi
    done
  fi

  echo "/dev/video0"
}

# Windows checkouts sometimes save CRLF line endings; Linux cannot run those scripts.
fix_windows_line_endings() {
  echo "Fixing script line endings (CRLF -> LF)..."
  for f in "$INSTALL_DIR"/*.sh "$INSTALL_DIR"/*.py; do
    if [ -f "$f" ]; then
      sed -i 's/\r$//' "$f"
    fi
  done
}

echo "=========================================="
echo "Robot Arm — install USB camera stream"
echo "=========================================="
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: run with sudo"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/start-camera-stream.sh" ]; then
    echo "Error: missing camera files in $INSTALL_DIR"
    exit 1
fi

if [ ! -f "$INSTALL_DIR/camera-stream.py" ] && [ ! -f "$INSTALL_DIR/camera-vision.py" ]; then
    echo "Error: missing camera-stream.py or camera-vision.py in $INSTALL_DIR"
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

if ! python3 -c "import cv2; import cv2.aruco" 2>/dev/null; then
    echo "Installing python3-opencv (for ArUco and coloured block detection)..."
    apt-get update
    apt-get install -y python3-opencv
fi

CAMERA_DEVICE="$(find_capture_device)"
# Manual exposure/brightness/gamma (V4L2 units) — site-specific, tune per Pi
# (see the "Tune ... live first" note above), then pass the working values
# here so they persist. All unset by default (auto-exposure, no correction).
CAMERA_EXPOSURE="${ROBOT_ARM_CAMERA_EXPOSURE:-}"
CAMERA_BRIGHTNESS="${ROBOT_ARM_CAMERA_BRIGHTNESS:-}"
CAMERA_GAMMA="${ROBOT_ARM_CAMERA_GAMMA:-}"

echo "Install folder:  $INSTALL_DIR"
echo "Service user:    $SERVICE_USER (group: video)"
echo "Camera port:     $CAMERA_PORT"
echo "Camera device:   $CAMERA_DEVICE"
echo "Manual exposure: ${CAMERA_EXPOSURE:-<unset, auto-exposure>}"
echo "Brightness:      ${CAMERA_BRIGHTNESS:-<unset>}"
echo "Gamma:           ${CAMERA_GAMMA:-<unset>}"
echo ""

# Service user must be in video group to open /dev/video*
usermod -aG video "$SERVICE_USER" 2>/dev/null || true

fix_windows_line_endings

chmod +x "$INSTALL_DIR/start-camera-stream.sh"
chmod +x "$INSTALL_DIR/camera-stream.py" 2>/dev/null || true
chmod +x "$INSTALL_DIR/camera-vision.py" 2>/dev/null || true

TEMP_SERVICE="/tmp/$SERVICE_NAME"
sed -e "s|INSTALL_DIR|$INSTALL_DIR|g" \
    -e "s|SERVICE_USER|$SERVICE_USER|g" \
    -e "s|__CAMERA_PORT__|$CAMERA_PORT|g" \
    -e "s|__CAMERA_DEVICE__|$CAMERA_DEVICE|g" \
    -e "s|__CAMERA_EXPOSURE__|$CAMERA_EXPOSURE|g" \
    -e "s|__CAMERA_BRIGHTNESS__|$CAMERA_BRIGHTNESS|g" \
    -e "s|__CAMERA_GAMMA__|$CAMERA_GAMMA|g" \
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
echo "Test vision:   curl -s http://127.0.0.1:${CAMERA_PORT}/vision"
echo "Test stream:   curl -s -o /dev/null -w 'HTTP %{http_code}\\n' --max-time 5 http://127.0.0.1:${CAMERA_PORT}/stream"
echo ""
echo "List capture devices:"
echo "  for d in /dev/video*; do echo \"=== \$d ===\"; v4l2-ctl -d \"\$d\" --list-formats-ext 2>/dev/null | head -5; done"
echo "Reinstall after fresh git clone:"
echo "  sudo bash $INSTALL_DIR/install-camera-service.sh $INSTALL_DIR"
echo "  sudo bash $INSTALL_DIR/install-web-server-service.sh $INSTALL_DIR"
echo "Diagnostics:"
echo "  bash $INSTALL_DIR/check-camera.sh"
echo "Reinstall with a specific device:"
echo "  sudo ROBOT_ARM_CAMERA_DEVICE=/dev/video0 ./install-camera-service.sh $INSTALL_DIR"
echo "Reinstall with exposure/brightness/gamma tuned for this Pi's lighting:"
echo "  sudo ROBOT_ARM_CAMERA_EXPOSURE=80 ROBOT_ARM_CAMERA_BRIGHTNESS=30 ROBOT_ARM_CAMERA_GAMMA=400 \\"
echo "    ./install-camera-service.sh $INSTALL_DIR"
echo ""

systemctl --no-pager status "$SERVICE_NAME" || true
