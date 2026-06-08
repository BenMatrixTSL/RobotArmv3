#!/bin/bash
#
# Easy kiosk diagnostics — run on the Pi while logged into the desktop.
#
#   cd /opt/RobotArm/electron-app
#   bash check-kiosk.sh
#
# Or from anywhere:
#   bash /opt/RobotArm/electron-app/check-kiosk.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSTART_DESKTOP="$HOME/.config/autostart/robot-arm-kiosk.desktop"
LABWC_AUTOSTART="$HOME/.config/labwc/autostart"
KIOSK_LOG="$HOME/.robot-arm-kiosk/kiosk.log"
KIOSK_ENV="$HOME/.config/robot-arm-kiosk.env"
PORT="${ROBOT_ARM_KIOSK_PORT:-80}"

PASS_COUNT=0
FAIL_COUNT=0

ok() {
    echo "  OK   $1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

bad() {
    echo "  FAIL $1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

warn() {
    echo "  WARN $1"
}

section() {
    echo ""
    echo "=== $1 ==="
}

echo "Robot Arm Kiosk — diagnostics"
echo "Time:  $(date)"
echo "User:  $USER"
echo "Home:  $HOME"
echo "Folder: $SCRIPT_DIR"

section "1. Kiosk script files"

if [ -f "$SCRIPT_DIR/start-kiosk.sh" ]; then
    ok "start-kiosk.sh exists"
else
    bad "start-kiosk.sh missing in $SCRIPT_DIR"
fi

if [ -f "$SCRIPT_DIR/install-kiosk-service.sh" ]; then
    ok "install-kiosk-service.sh exists"
else
    bad "install-kiosk-service.sh missing"
fi

if [ -x "$SCRIPT_DIR/start-kiosk.sh" ]; then
    ok "start-kiosk.sh is executable"
else
    bad "start-kiosk.sh is NOT executable — run: chmod +x $SCRIPT_DIR/start-kiosk.sh"
fi

if command -v file >/dev/null 2>&1 && [ -f "$SCRIPT_DIR/start-kiosk.sh" ]; then
    FILE_INFO="$(file "$SCRIPT_DIR/start-kiosk.sh")"
    if echo "$FILE_INFO" | grep -qi "CRLF"; then
        bad "start-kiosk.sh has Windows line endings (CRLF) — run: sed -i 's/\r$//' $SCRIPT_DIR/*.sh"
    else
        ok "start-kiosk.sh line endings look fine"
    fi
fi

section "2. Boot autostart (how kiosk should start after reboot)"

AUTOLOGIN_USER=""
if [ -f /etc/lightdm/lightdm.conf ]; then
    AUTOLOGIN_USER="$(grep -E '^autologin-user=' /etc/lightdm/lightdm.conf 2>/dev/null | cut -d= -f2 | tr -d ' ')"
fi

if [ -n "$AUTOLOGIN_USER" ] && [ "$AUTOLOGIN_USER" != "$USER" ]; then
    bad "Auto-login user is '$AUTOLOGIN_USER' but you are '$USER'"
    echo "         Kiosk autostart must be in /home/$AUTOLOGIN_USER/.config/ — not only in your home folder."
    AUTOLOGIN_DESKTOP="/home/$AUTOLOGIN_USER/.config/autostart/robot-arm-kiosk.desktop"
    if [ -f "$AUTOLOGIN_DESKTOP" ]; then
        ok "Autostart exists for auto-login user $AUTOLOGIN_USER"
    else
        bad "No autostart for auto-login user $AUTOLOGIN_USER"
        echo "         Fix: sudo bash $SCRIPT_DIR/install-kiosk-service.sh $SCRIPT_DIR $AUTOLOGIN_USER"
    fi
fi

if [ -f "$AUTOSTART_DESKTOP" ]; then
    ok "Desktop autostart file exists: $AUTOSTART_DESKTOP"
    echo "       Exec line:"
    grep "^Exec=" "$AUTOSTART_DESKTOP" | sed 's/^/         /'
else
    if [ -z "$AUTOLOGIN_USER" ] || [ "$AUTOLOGIN_USER" = "$USER" ]; then
        bad "Desktop autostart missing — run: sudo bash $SCRIPT_DIR/install-kiosk-service.sh $SCRIPT_DIR"
    else
        warn "No autostart in your home ($USER) — see auto-login user check above"
    fi
fi

check_labwc_autostart() {
    local user_home="$1"
    local label="$2"
    local file="$user_home/.config/labwc/autostart"

    if [ -f "$file" ] && grep -q "start-kiosk.sh" "$file" 2>/dev/null; then
        ok "labwc autostart for $label includes start-kiosk.sh"
        grep "start-kiosk.sh" "$file" | sed 's/^/         /'
        return 0
    fi
    return 1
}

LABWC_OK=0
if check_labwc_autostart "$HOME" "$USER"; then
    LABWC_OK=1
elif [ -n "$AUTOLOGIN_USER" ] && [ "$AUTOLOGIN_USER" != "$USER" ]; then
    if check_labwc_autostart "/home/$AUTOLOGIN_USER" "$AUTOLOGIN_USER"; then
        LABWC_OK=1
    fi
fi

if [ "$LABWC_OK" -eq 0 ]; then
    warn "No labwc autostart with start-kiosk.sh (often required on Pi OS Bookworm+ Wayland)"
    echo "         Re-run: sudo bash $SCRIPT_DIR/install-kiosk-service.sh $SCRIPT_DIR"
fi

if [ -f "/etc/systemd/system/robot-arm-kiosk.service" ]; then
    bad "Old system-wide service still installed (causes MIT-MAGIC-COOKIE errors)"
    echo "         Run: sudo bash $SCRIPT_DIR/install-kiosk-service.sh $SCRIPT_DIR"
else
    ok "No old broken system-wide kiosk service"
fi

section "3. Desktop session (must be logged in)"

if [ -n "$XDG_RUNTIME_DIR" ] && [ -d "$XDG_RUNTIME_DIR" ]; then
    ok "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
else
    bad "XDG_RUNTIME_DIR not set — are you logged into the graphical desktop?"
fi

WAYLAND_FOUND=0
if [ -n "$XDG_RUNTIME_DIR" ]; then
    for sock in "$XDG_RUNTIME_DIR"/wayland-*; do
        if [ -S "$sock" ]; then
            ok "Wayland socket found: $(basename "$sock")"
            WAYLAND_FOUND=1
            break
        fi
    done
fi

if [ "$WAYLAND_FOUND" -eq 0 ]; then
    if [ -n "$DISPLAY" ] && command -v xdpyinfo >/dev/null 2>&1; then
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
            ok "X11 display works (DISPLAY=$DISPLAY)"
        else
            bad "DISPLAY=$DISPLAY but xdpyinfo failed"
        fi
    elif [ -n "$DISPLAY" ]; then
        warn "DISPLAY=$DISPLAY (xdpyinfo not installed to verify)"
    elif [ -n "$SSH_CONNECTION" ] || [ -n "$SSH_CLIENT" ]; then
        warn "No display here — you are on SSH. Open a terminal ON the Pi screen to test the kiosk."
    else
        bad "No Wayland socket and no DISPLAY — open a terminal on the Pi desktop (not SSH)"
    fi
fi

section "4. Auto-login (needed for kiosk at boot)"

if [ -n "$AUTOLOGIN_USER" ]; then
    ok "Desktop auto-login user: $AUTOLOGIN_USER"
else
    warn "Could not detect auto-login — enable it in raspi-config (Boot -> Desktop Autologin)"
fi

section "5. Required programs"

if command -v chromium >/dev/null 2>&1; then
    ok "chromium found: $(command -v chromium)"
elif command -v chromium-browser >/dev/null 2>&1; then
    ok "chromium-browser found: $(command -v chromium-browser)"
else
    bad "Chromium not installed — run: sudo apt install -y chromium"
fi

if command -v python3 >/dev/null 2>&1; then
    ok "python3 found"
else
    bad "python3 not installed"
fi

section "6. Robot arm server"

if systemctl is-active --quiet st3215-server.service 2>/dev/null; then
    ok "st3215-server.service is running"
else
    warn "st3215-server.service is not running (UI may show Disconnected)"
    echo "         Check: sudo systemctl status st3215-server.service"
fi

section "7. Kiosk HTTP port ($PORT)"

if command -v ss >/dev/null 2>&1; then
    if ss -ltn "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"; then
        ok "Something is listening on port $PORT (kiosk HTTP server may already be running)"
    else
        warn "Nothing listening on port $PORT right now (normal if kiosk has not started yet)"
    fi
fi

if command -v curl >/dev/null 2>&1; then
    if curl -s -o /dev/null -w "%{http_code}" "http://127.0.0.1:$PORT/index.html" 2>/dev/null | grep -q "200"; then
        ok "http://127.0.0.1:$PORT/index.html responds"
    else
        warn "http://127.0.0.1:$PORT/index.html not responding yet"
    fi
fi

section "8. Kiosk log (most useful for errors)"

if [ -f "$KIOSK_LOG" ]; then
    ok "Log file exists: $KIOSK_LOG"
    echo ""
    echo "--- Last 30 lines of kiosk log ---"
    tail -30 "$KIOSK_LOG"
    echo "--- end of log ---"
else
    warn "No log file yet — kiosk script has never run successfully"
    echo "         Log will appear at: $KIOSK_LOG"
    echo "         Try: $SCRIPT_DIR/start-kiosk.sh"
fi

if [ -f "$KIOSK_ENV" ]; then
    ok "Kiosk env file: $KIOSK_ENV"
    cat "$KIOSK_ENV" | sed 's/^/         /'
fi

section "9. Running processes"

if pgrep -af "start-kiosk.sh" >/dev/null 2>&1; then
    ok "start-kiosk.sh is running"
    pgrep -af "start-kiosk.sh" | sed 's/^/         /'
else
    warn "start-kiosk.sh is not running right now"
fi

if pgrep -af "chromium" >/dev/null 2>&1; then
    ok "Chromium process found"
else
    warn "No Chromium process running"
fi

section "Summary"

echo "  Passed: $PASS_COUNT"
echo "  Failed: $FAIL_COUNT"
echo ""

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo "Fix the FAIL items above, then try:"
    echo "  sudo bash $SCRIPT_DIR/install-kiosk-service.sh $SCRIPT_DIR"
    echo "  sudo reboot"
    echo ""
fi

echo "Manual test (should open fullscreen browser now):"
echo "  $SCRIPT_DIR/start-kiosk.sh"
echo ""
echo "If manual test works but boot does not:"
echo "  - Enable desktop auto-login for your user"
echo "  - Re-run the installer (adds labwc autostart on Pi OS Wayland)"
echo "  - Reboot"
echo ""
