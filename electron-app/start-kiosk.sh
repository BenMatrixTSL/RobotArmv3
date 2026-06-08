#!/bin/bash
#
# Serve the robot arm UI over HTTP and open Chromium kiosk mode.
# Must run inside a graphical desktop session (auto-login recommended).
#
#   ./start-kiosk.sh
#   ROBOT_ARM_KIOSK_SCREENS=all ./start-kiosk.sh
#
# Log file: ~/.robot-arm-kiosk/kiosk.log

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$HOME/.config/robot-arm-kiosk.env" ]; then
    # shellcheck disable=SC1091
    source "$HOME/.config/robot-arm-kiosk.env"
fi

PORT="${ROBOT_ARM_KIOSK_PORT:-80}"
KIOSK_URL="http://127.0.0.1:${PORT}/index.html?kiosk=1"
KIOSK_SCREENS="${ROBOT_ARM_KIOSK_SCREENS:-1}"
PROFILE_BASE="${HOME}/.robot-arm-kiosk"
LOG_FILE="${PROFILE_BASE}/kiosk.log"

CHROMIUM_FLAGS=()
CHROMIUM_USE_WAYLAND=0
HTTP_PID=""

mkdir -p "$PROFILE_BASE"
exec >>"$LOG_FILE" 2>&1
echo ""
echo "========== Kiosk start $(date) =========="

find_chromium() {
    if command -v chromium >/dev/null 2>&1; then
        echo "chromium"
        return 0
    fi
    if command -v chromium-browser >/dev/null 2>&1; then
        echo "chromium-browser"
        return 0
    fi
    return 1
}

find_wayland_socket() {
    local sock

    if [ -z "$XDG_RUNTIME_DIR" ] || [ ! -d "$XDG_RUNTIME_DIR" ]; then
        return 1
    fi

    for sock in "$XDG_RUNTIME_DIR"/wayland-*; do
        if [ -S "$sock" ]; then
            basename "$sock"
            return 0
        fi
    done

    return 1
}

find_xauthority_file() {
    local f

    if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
        echo "$XAUTHORITY"
        return 0
    fi
    if [ -f "$HOME/.Xauthority" ]; then
        echo "$HOME/.Xauthority"
        return 0
    fi
    if [ -n "$XDG_RUNTIME_DIR" ]; then
        for f in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.* \
                 "$XDG_RUNTIME_DIR"/.xwaylandauth* \
                 "$XDG_RUNTIME_DIR"/gdm/Xauthority; do
            if [ -f "$f" ]; then
                echo "$f"
                return 0
            fi
        done
    fi
    return 1
}

build_chromium_flags() {
    CHROMIUM_FLAGS=(
        --disable-dev-shm-usage
        --no-first-run
        --no-default-browser-check
        --touch-events=enabled
    )
    if [ "$CHROMIUM_USE_WAYLAND" = "1" ]; then
        CHROMIUM_FLAGS+=(--ozone-platform=wayland --enable-features=UseOzonePlatform)
    else
        CHROMIUM_FLAGS+=(--disable-gpu)
    fi
}

prepare_chromium_display() {
    local uid
    local auth
    local wayland_name

    uid="$(id -u)"

    if [ -z "$XDG_RUNTIME_DIR" ] && [ -d "/run/user/$uid" ]; then
        export XDG_RUNTIME_DIR="/run/user/$uid"
    fi

    CHROMIUM_USE_WAYLAND=0
    wayland_name="$(find_wayland_socket || true)"

    if [ -n "$wayland_name" ]; then
        export WAYLAND_DISPLAY="$wayland_name"
        CHROMIUM_USE_WAYLAND=1
        echo "Kiosk: using Wayland ($WAYLAND_DISPLAY)"
    else
        export DISPLAY="${DISPLAY:-:0}"
        auth="$(find_xauthority_file || true)"
        if [ -n "$auth" ]; then
            export XAUTHORITY="$auth"
        fi
        echo "Kiosk: using X11 (DISPLAY=$DISPLAY, XAUTHORITY=${XAUTHORITY:-<none>})"
    fi

    build_chromium_flags
}

display_is_ready() {
    local wayland_name

    wayland_name="$(find_wayland_socket || true)"
    if [ -n "$wayland_name" ]; then
        return 0
    fi
    if command -v xdpyinfo >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
            return 0
        fi
    fi
    return 1
}

wait_for_display() {
    local attempt=0
    local max_attempts=60

    while [ "$attempt" -lt "$max_attempts" ]; do
        prepare_chromium_display
        if display_is_ready; then
            echo "Kiosk: display ready after $((attempt * 2)) seconds"
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 2
    done

    echo "Kiosk: display still not ready after $((max_attempts * 2)) seconds" >&2
    return 1
}

free_port_if_busy() {
    local pid

    if command -v fuser >/dev/null 2>&1; then
        fuser -k "${PORT}/tcp" 2>/dev/null || true
        sleep 1
        return 0
    fi

    if command -v ss >/dev/null 2>&1; then
        pid="$(ss -ltnp "sport = :$PORT" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)"
        if [ -n "$pid" ]; then
            echo "Kiosk: stopping old process on port $PORT (pid $pid)"
            kill "$pid" 2>/dev/null || true
            sleep 1
        fi
    fi
}

http_page_is_ready() {
    local url="http://127.0.0.1:${PORT}/index.html"

    if command -v curl >/dev/null 2>&1; then
        curl -sf --max-time 3 "$url" 2>/dev/null | grep -q "Robot Arm Control"
        return $?
    fi

    python3 - "$url" <<'PY'
import sys
import urllib.request

url = sys.argv[1]
try:
    with urllib.request.urlopen(url, timeout=3) as response:
        body = response.read().decode("utf-8", errors="replace")
        if response.status == 200 and "Robot Arm Control" in body:
            sys.exit(0)
except Exception:
    pass
sys.exit(1)
PY
}

wait_for_http_server() {
    local attempt=0
    local max_attempts=45

    echo "Kiosk: waiting for HTTP server on port $PORT ..."

    while [ "$attempt" -lt "$max_attempts" ]; do
        if http_page_is_ready; then
            echo "Kiosk: HTTP server ready (index.html loads OK)"
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 1
    done

    echo "Kiosk: HTTP server did not become ready in ${max_attempts}s" >&2
    return 1
}

start_http_server() {
    cd "$SCRIPT_DIR"

    # Port 80 is usually served by robot-arm-web-server.service — reuse it, do not kill it.
    if http_page_is_ready; then
        echo "Kiosk: using existing HTTP server on port $PORT"
        HTTP_PID=""
        return 0
    fi

    if [ "$PORT" -lt 1024 ]; then
        echo "Kiosk: waiting for existing service on port $PORT (needs root to start here) ..."
        if wait_for_http_server; then
            HTTP_PID=""
            return 0
        fi
        echo "Kiosk: error — nothing is serving port $PORT. Install: sudo ./install-web-server-service.sh" >&2
        return 1
    fi

    free_port_if_busy

    if python3 -m http.server --help 2>&1 | grep -q -- '--bind'; then
        python3 -m http.server "$PORT" --bind 127.0.0.1 &
    else
        python3 -m http.server "$PORT" &
    fi
    HTTP_PID=$!
    echo "Kiosk: HTTP server pid $HTTP_PID on port $PORT"

    if ! wait_for_http_server; then
        echo "Kiosk: warning — starting Chromium anyway (page may reload until ready)" >&2
    fi
}

stop_http_server() {
    if [ -n "$HTTP_PID" ]; then
        kill "$HTTP_PID" 2>/dev/null || true
        HTTP_PID=""
    fi
}

cleanup() {
    stop_http_server
}
trap cleanup EXIT INT TERM

MONITOR_LIST=()

load_monitors_from_xrandr() {
    MONITOR_LIST=()

    if ! command -v xrandr >/dev/null 2>&1; then
        return 1
    fi

    local old_display="$DISPLAY"
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
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

    export DISPLAY="$old_display"

    if [ "${#MONITOR_LIST[@]}" -eq 0 ]; then
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
        return 1
    fi

    local i=0
    while [ "$i" -lt "$count" ] && [ "$i" -lt "$limit" ]; do
        MONITORS_TO_USE+=("${MONITOR_LIST[$i]}")
        i=$((i + 1))
    done

    echo "Kiosk: ${#MONITORS_TO_USE[@]} Chromium window(s)."
    return 0
}

launch_chromium() {
    local profile_dir="$1"
    shift
    local extra_args=("$@")
    local saved_display="$DISPLAY"

    prepare_chromium_display

    if [ "$CHROMIUM_USE_WAYLAND" = "1" ]; then
        unset DISPLAY
    fi

    mkdir -p "$profile_dir"

    echo "Kiosk: launching Chromium -> $KIOSK_URL"

    "$CHROMIUM" \
        --user-data-dir="$profile_dir" \
        "${CHROMIUM_FLAGS[@]}" \
        "${extra_args[@]}" \
        --kiosk \
        --app="$KIOSK_URL" \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --start-fullscreen \
        "$KIOSK_URL"

    local exit_code=$?
    export DISPLAY="$saved_display"
    return "$exit_code"
}

CHROMIUM="$(find_chromium || true)"
if [ -z "$CHROMIUM" ]; then
    echo "Error: Chromium not found. Install: sudo apt install -y chromium" >&2
    exit 1
fi

echo "Kiosk: serving $SCRIPT_DIR"
echo "Kiosk: URL $KIOSK_URL"
echo "Kiosk: log $LOG_FILE"

if ! wait_for_display; then
    echo "Kiosk: enable desktop auto-login for $USER, reboot, then check $LOG_FILE" >&2
    exit 1
fi

start_http_server

if pick_monitors_to_use; then
    while true; do
        CHROMIUM_PIDS=()
        index=0
        for entry in "${MONITORS_TO_USE[@]}"; do
            read -r width height x y <<< "$entry"
            profile_dir="${PROFILE_BASE}/profile-${index}"
            echo "Kiosk: display $index at ${x},${y} ${width}x${height}"
            launch_chromium "$profile_dir" \
                --window-position="${x},${y}" \
                --window-size="${width},${height}" &
            CHROMIUM_PIDS+=("$!")
            index=$((index + 1))
        done

        wait "${CHROMIUM_PIDS[@]}"
        exit_code=$?
        echo "Kiosk: Chromium exited (code $exit_code), restarting in 5 seconds..." >&2
        sleep 5
        wait_for_display || sleep 10
    done
fi

echo "Kiosk: single fullscreen window"
while true; do
    launch_chromium "${PROFILE_BASE}/profile-0"
    exit_code=$?
    echo "Kiosk: Chromium exited (code $exit_code), restarting in 5 seconds..." >&2
    sleep 5
    wait_for_display || sleep 10
done
