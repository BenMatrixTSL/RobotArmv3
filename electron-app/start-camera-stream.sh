#!/bin/bash
#
# Start the USB camera MJPEG stream for the Camera tab in the web UI.
#
#   ./start-camera-stream.sh
#   ROBOT_ARM_CAMERA_DEVICE=/dev/video0 ./start-camera-stream.sh
#
# View in browser: http://<pi-ip>:8082/stream

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_CAMERA_PORT:-8082}"
DEVICE="${ROBOT_ARM_CAMERA_DEVICE:-/dev/video0}"

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed." >&2
    exit 1
fi

if ! command -v ffmpeg >/dev/null; then
    echo "Error: ffmpeg is not installed." >&2
    echo "Install on the Pi: sudo apt install -y ffmpeg" >&2
    exit 1
fi

if [ ! -e "$DEVICE" ]; then
    echo "Error: camera device not found: $DEVICE" >&2
    echo "Try: ls -l /dev/video*" >&2
    exit 1
fi

export ROBOT_ARM_CAMERA_PORT="$PORT"
export ROBOT_ARM_CAMERA_DEVICE="$DEVICE"

echo "Camera stream: http://0.0.0.0:${PORT}/stream"
echo "Device: $DEVICE"
echo "Press Ctrl+C to stop"

cd "$SCRIPT_DIR"
exec python3 "$SCRIPT_DIR/camera-stream.py"
