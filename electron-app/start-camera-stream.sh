#!/bin/bash
#
# Start the USB camera service for the Camera tab in the web UI.
#
# Uses OpenCV vision (ArUco + coloured blocks) when python3-opencv is installed.
# Falls back to ffmpeg-only streaming otherwise.
#
#   ./start-camera-stream.sh
#   ROBOT_ARM_CAMERA_DEVICE=/dev/video0 ./start-camera-stream.sh
#
# View in browser: http://<pi-ip>:8082/snapshot

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_CAMERA_PORT:-8082}"
DEVICE="${ROBOT_ARM_CAMERA_DEVICE:-/dev/video0}"

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed." >&2
    exit 1
fi

if [ ! -e "$DEVICE" ]; then
    echo "Error: camera device not found: $DEVICE" >&2
    echo "Try: ls -l /dev/video*" >&2
    exit 1
fi

export ROBOT_ARM_CAMERA_PORT="$PORT"
export ROBOT_ARM_CAMERA_DEVICE="$DEVICE"

cd "$SCRIPT_DIR"

if [ -f "$SCRIPT_DIR/camera-vision.py" ] && python3 -c "import cv2; import cv2.aruco" 2>/dev/null; then
    echo "Camera vision: http://0.0.0.0:${PORT}/snapshot"
    echo "Vision JSON:   http://0.0.0.0:${PORT}/vision"
    echo "Device: $DEVICE"
    echo "Press Ctrl+C to stop"
    exec python3 "$SCRIPT_DIR/camera-vision.py"
fi

if ! command -v ffmpeg >/dev/null; then
    echo "Error: ffmpeg is not installed and OpenCV is not available." >&2
    echo "Install on the Pi: sudo apt install -y python3-opencv ffmpeg" >&2
    exit 1
fi

echo "Camera stream (ffmpeg): http://0.0.0.0:${PORT}/stream"
echo "Device: $DEVICE"
echo "Press Ctrl+C to stop"
exec python3 "$SCRIPT_DIR/camera-stream.py"
