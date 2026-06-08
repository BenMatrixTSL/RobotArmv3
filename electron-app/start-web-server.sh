#!/bin/bash
#
# Serve the robot arm web UI (electron-app folder) with Python's built-in HTTP server.
#
#   sudo ./start-web-server.sh              # port 80 by hand (needs root unless using the systemd service)
#   ./start-web-server.sh                   # high port, e.g. ROBOT_ARM_WEB_PORT=8081
#   ROBOT_ARM_WEB_PORT=8081 ./start-web-server.sh
#
# Then open in a browser: http://<pi-ip>/index.html
# Kiosk on the Pi uses this port 80 server when installed (see install-web-server-service.sh).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_WEB_PORT:-80}"
BIND="${ROBOT_ARM_WEB_BIND:-0.0.0.0}"

if ! command -v python3 >/dev/null; then
    echo "Error: python3 is not installed." >&2
    exit 1
fi

echo "Web server: folder $SCRIPT_DIR"
echo "Web server: http://${BIND}:${PORT}/"
echo "Web server: press Ctrl+C to stop"

cd "$SCRIPT_DIR"

# Use cd for the served files; older Python on the Pi may not support --directory
if python3 -m http.server --help 2>&1 | grep -q -- '--bind'; then
    exec python3 -m http.server "$PORT" --bind "$BIND"
else
    exec python3 -m http.server "$PORT"
fi
