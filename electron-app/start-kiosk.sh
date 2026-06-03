#!/bin/bash
#
# Serve the robot arm UI over HTTP and open Chromium kiosk mode.
#
#   ./start-kiosk.sh                    # one window (primary / fullscreen)
#   ROBOT_ARM_KIOSK_SCREENS=all ./start-kiosk.sh   # one window per connected monitor
#   ROBOT_ARM_KIOSK_SCREENS=2 ./start-kiosk.sh    # first two monitors only
#
# Needs a desktop session (DISPLAY) and the ST3215 server on port 8080.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_KIOSK_PORT:-3080}"
KIOSK_URL="http://127.0.0.1:${PORT}/index.html?kiosk=1"
# 1 = single fullscreen window | 2 = first two monitors | all = every connected monitor
KIOSK_SCREENS="${ROBOT_ARM_KIOSK_SCREENS:-all}"

find_chromium() {
    if command -v chromium-browser >/dev/null 2>&1; then
        echo "chromium-browser"
        return 0
    fi
    if command -v chromium >/dev/null 2>&1; then
        echo "chromium"
        return 0
    fi
    return 1
}

CHROMIUM="$(find_chromium || true)"
if [ -z "$CHROMIUM" ]; then
    echo "Error: Chromium not found. Install it on the Pi:"
    echo "  sudo apt update"
    echo "  sudo apt install -y chromium"
    exit 1
fi

if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Each monitor: "WIDTH HEIGHT X Y" (from xrandr, e.g. 1920x1080+0+0)
MONITOR_LIST=()

load_monitors_from_xrandr() {
    MONITOR_LIST=()

    if ! command -v xrandr >/dev/null 2>&1; then
        echo "Kiosk: xrandr not found — using single fullscreen window."
        return 1
    fi

    while IFS= read -r line; do
        case "$line" in
            *" connected"*)
                if [[ "$line" =~ ([0-9]+)x([0-9]+)\+([0-9]+)\+([0-9]+) ]]; then
                    MONITOR_LIST+=("${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]}")
                fi
                ;;
        esac
    done < <(xrandr --query 2>/dev/null || true)

    if [ "${#MONITOR_LIST[@]}" -eq 0 ]; then
        echo "Kiosk: no monitor geometry from xrandr — using single fullscreen window."
        return 1
    fi

    return 0
}

pick_monitors_to_use() {
    MONITORS_TO_USE=()

    if [ "$KIOSK_SCREENS" = "1" ]; then
        return 1
    fi

    if ! load_monitors_from_xrandr; then
        return 1
    fi

    local count="${#MONITOR_LIST[@]}"
    local limit="$count"

    if [ "$KIOSK_SCREENS" = "2" ]; then
        limit=2
    fi

    if [ "$count" -lt 2 ]; then
        echo "Kiosk: only one monitor detected — using single fullscreen window."
        return 1
    fi

    local i=0
    while [ "$i" -lt "$count" ] && [ "$i" -lt "$limit" ]; do
        MONITORS_TO_USE+=("${MONITOR_LIST[$i]}")
        i=$((i + 1))
    done

    echo "Kiosk: launching ${#MONITORS_TO_USE[@]} Chromium window(s) (screens=$KIOSK_SCREENS)."
    return 0
}

launch_chromium_on_monitor() {
    local index="$1"
    local width="$2"
    local height="$3"
    local x="$4"
    local y="$5"
    local profile_dir="$SCRIPT_DIR/.kiosk-profile-$index"

    mkdir -p "$profile_dir"

    echo "Kiosk: display $index at ${x},${y} size ${width}x${height}"

    "$CHROMIUM" \
        --user-data-dir="$profile_dir" \
        --window-position="${x},${y}" \
        --window-size="${width},${height}" \
        --kiosk \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --start-fullscreen \
        "$KIOSK_URL" &
}

echo "Kiosk: serving $SCRIPT_DIR on port $PORT"
echo "Kiosk: URL $KIOSK_URL"

cd "$SCRIPT_DIR"
python3 -m http.server "$PORT" --bind 127.0.0.1 &
HTTP_PID=$!

CHROMIUM_PIDS=()

cleanup() {
    local pid
    for pid in "${CHROMIUM_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    kill "$HTTP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2

if pick_monitors_to_use; then
    index=0
    for entry in "${MONITORS_TO_USE[@]}"; do
        read -r width height x y <<< "$entry"
        launch_chromium_on_monitor "$index" "$width" "$height" "$x" "$y"
        CHROMIUM_PIDS+=("$!")
        index=$((index + 1))
    done

    wait "${CHROMIUM_PIDS[@]}"
else
    echo "Kiosk: single fullscreen window"
    exec "$CHROMIUM" \
        --user-data-dir="$SCRIPT_DIR/.kiosk-profile-0" \
        --kiosk \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --start-fullscreen \
        "$KIOSK_URL"
fi
