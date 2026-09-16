#!/bin/bash
#
# Detect a hung or crashed kiosk Chromium and restart it automatically.
#
# start-kiosk.sh's own monitor loop only checks whether *any* Chromium-related
# process exists (zygote, GPU, utility, ...). When a renderer/GPU process
# crashes, sibling processes can stay alive and the browser window is left
# showing a frozen frame forever — the monitor loop never notices, so nothing
# restarts it. This watchdog adds two extra checks the loop can't do itself:
#
#   1. Fully dead: no kiosk Chromium AND no start-kiosk.sh running at all.
#   2. Hung/crashed: a defunct ("<defunct>") Chromium child process persists
#      for more than DEFUNCT_THRESHOLD_SECS. A zombie normally gets reaped
#      within a second; one that lingers is the signature of the browser
#      process failing to recover from a child crash (see kiosk.log entries
#      like "Couldn't initialize main thread" / "ptrace: No such process").
#
# Run periodically via robot-arm-kiosk-watchdog.timer (systemd, root).
#
# Usage: kiosk-watchdog.sh <kiosk-user> <path-to-start-kiosk.sh>

set -u

KIOSK_USER="${1:?usage: kiosk-watchdog.sh <kiosk-user> <path-to-start-kiosk.sh>}"
KIOSK_SCRIPT="${2:?usage: kiosk-watchdog.sh <kiosk-user> <path-to-start-kiosk.sh>}"

STATE_DIR="/run/robot-arm-kiosk-watchdog"
DEFUNCT_STATE_FILE="${STATE_DIR}/defunct-since"
DEFUNCT_THRESHOLD_SECS=45
LOG_TAG="robot-arm-kiosk-watchdog"

mkdir -p "$STATE_DIR"

log() {
    logger -t "$LOG_TAG" "$1" 2>/dev/null || true
    echo "$1"
}

restart_kiosk() {
    log "Restarting kiosk: killing Chromium for $KIOSK_USER"
    pkill -u "$KIOSK_USER" -f chromium 2>/dev/null || true
    sleep 2
    pkill -9 -u "$KIOSK_USER" -f chromium 2>/dev/null || true
    rm -f "$DEFUNCT_STATE_FILE"

    if pgrep -u "$KIOSK_USER" -f "$(basename "$KIOSK_SCRIPT")" >/dev/null 2>&1; then
        log "start-kiosk.sh still running — it will relaunch Chromium on its own"
        return
    fi

    log "Launching $KIOSK_SCRIPT as $KIOSK_USER"
    runuser -l "$KIOSK_USER" -c "nohup '$KIOSK_SCRIPT' >/dev/null 2>&1 &"
}

# 1. Fully dead: nothing kiosk-related running at all.
if ! pgrep -u "$KIOSK_USER" -f chromium >/dev/null 2>&1 \
    && ! pgrep -u "$KIOSK_USER" -f "$(basename "$KIOSK_SCRIPT")" >/dev/null 2>&1; then
    log "No kiosk processes found for $KIOSK_USER — relaunching"
    restart_kiosk
    exit 0
fi

# 2. Hung/crashed: a defunct Chromium child persisting past the threshold.
defunct_pid="$(ps -u "$KIOSK_USER" -o pid=,stat=,comm= 2>/dev/null \
    | awk '$2 ~ /Z/ && $3 ~ /chromium/ {print $1; exit}')"

if [ -n "$defunct_pid" ]; then
    now="$(date +%s)"
    if [ -f "$DEFUNCT_STATE_FILE" ]; then
        read -r stored_pid stored_time < "$DEFUNCT_STATE_FILE"
        if [ "$stored_pid" = "$defunct_pid" ]; then
            age=$(( now - stored_time ))
            if [ "$age" -ge "$DEFUNCT_THRESHOLD_SECS" ]; then
                log "Chromium child PID $defunct_pid defunct for ${age}s — treating as crashed"
                restart_kiosk
            fi
            exit 0
        fi
    fi
    echo "$defunct_pid $now" > "$DEFUNCT_STATE_FILE"
    exit 0
fi

rm -f "$DEFUNCT_STATE_FILE"
exit 0
