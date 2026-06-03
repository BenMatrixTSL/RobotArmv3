#!/bin/bash
#
# Serve the robot arm web UI (electron-app folder) with Python's built-in HTTP server.
#
#   sudo ./start-web-server.sh              # port 80 (needs root on Linux)
#   ./start-web-server.sh                   # uses ROBOT_ARM_WEB_PORT if set (e.g. 8081)
#   ROBOT_ARM_WEB_PORT=8081 ./start-web-server.sh
#
# Then open in a browser: http://<pi-ip>/index.html
# Kiosk on the Pi still uses port 3080 locally; this is for port 80 / network access.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_WEB_PORT:-80}"
BIND="${ROBOT_ARM_WEB_BIND:-0.0.0.0}"

if [ "$PORT" -lt 1024 ] && [ "$EUID" -ne 0 ]; then
    echo "Error: port $PORT requires root. Run:"
    echo "  sudo ./start-web-server.sh"
    exit 1
fi

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed."
    exit 1
fi

echo "Web server: folder $SCRIPT_DIR"
echo "Web server: http://${BIND}:${PORT}/"
echo "Web server: press Ctrl+C to stop"
echo ""

cd "$SCRIPT_DIR"
exec python3 -m http.server "$PORT" --bind "$BIND" --directory "$SCRIPT_DIR"
