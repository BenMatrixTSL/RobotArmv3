#!/bin/bash
#
# Serve the robot arm UI over HTTP and open Chromium kiosk mode.
# Run from a graphical login session (desktop auto-login), not as a system service.
#
#   ./start-kiosk.sh
#   ROBOT_ARM_KIOSK_SCREENS=all ./start-kiosk.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ROBOT_ARM_KIOSK_PORT:-3080}"
KIOSK_URL="http://127.0.0.1:${PORT}/index.html?kiosk=1"
KIOSK_SCREENS="${ROBOT_ARM_KIOSK_SCREENS:-1}"

CHROMIUM_FLAGS=()
CHROMIUM_USE_WAYLAND=0

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

find_xauthority_file() {
    if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
        echo "$XAUTHORITY"
        return 0
    fi
    if [ -f "$HOME/.Xauthority" ]; then
        echo "$HOME/.Xauthority"
        return 0
    fi
    if [ -n "$XDG_RUNTIME_DIR" ]; then
        local f
        for f in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
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
        --disable-gpu
        --no-first-run
        --no-default-browser-check
    )
    if [ "$CHROMIUM_USE_WAYLAND" = "1" ]; then
        CHROMIUM_FLAGS+=(--ozone-platform=wayland)
    fi
}

# Call again right before starting Chromium (session may be ready now).
prepare_chromium_display() {
    local uid
    local auth
    uid="$(id -u)"

    if [ -z "$XDG_RUNTIME_DIR" ] && [ -d "/run/user/$uid" ]; then
        export XDG_RUNTIME_DIR="/run/user/$uid"
    fi

    CHROMIUM_USE_WAYLAND=0

    if [ -n "$XDG_RUNTIME_DIR" ] && [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
        export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
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

wait_for_display() {
    local attempt=0
    local max_attempts=30

    while [ "$attempt" -lt "$max_attempts" ]; do
        if [ -n "$XDG_RUNTIME_DIR" ] && [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
            return 0
        fi
        if command -v xdpyinfo >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then
            if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
                return 0
            fi
        fi
        attempt=$((attempt + 1))
        sleep 2
    done

    echo "Kiosk: warning — display not ready yet." >&2
    return 1
}

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

    "$CHROMIUM" \
        --user-data-dir="$profile_dir" \
        "${CHROMIUM_FLAGS[@]}" \
        "${extra_args[@]}" \
        --kiosk \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --start-fullscreen \
        "$KIOSK_URL"

    export DISPLAY="$saved_display"
}

CHROMIUM="$(find_chromium || true)"
if [ -z "$CHROMIUM" ]; then
    echo "Error: Chromium not found. Install: sudo apt install -y chromium" >&2
    exit 1
fi

prepare_chromium_display

if ! wait_for_display; then
    echo "Kiosk: enable desktop auto-login for $USER and reboot, then try again." >&2
    exit 1
fi

PROFILE_BASE="${HOME}/.robot-arm-kiosk"

echo "Kiosk: serving $SCRIPT_DIR on port $PORT"
echo "Kiosk: URL $KIOSK_URL"

cd "$SCRIPT_DIR"

if python3 -m http.server --help 2>&1 | grep -q -- '--bind'; then
    python3 -m http.server "$PORT" --bind 127.0.0.1 &
else
    python3 -m http.server "$PORT" &
fi
HTTP_PID=$!

cleanup() {
    kill "$HTTP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2

if pick_monitors_to_use; then
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
    echo "Kiosk: Chromium exited (code $exit_code)" >&2
    exit "$exit_code"
fi

echo "Kiosk: single fullscreen window"
launch_chromium "${PROFILE_BASE}/profile-0"
