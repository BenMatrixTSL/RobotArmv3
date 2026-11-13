#!/bin/bash
# Keeps the Matrix boot picture on screen from the moment the desktop session
# starts until the kiosk window covers it, so the Pi desktop (wallpaper,
# Wastebasket, taskbar) is never seen between the boot splash and the app.
#
# Started from the kiosk user's labwc autostart, before start-kiosk.sh
# (install-kiosk-service.sh writes that). It:
#   1. fills every screen with the boot-splash colour straight away
#   2. renders the boot-splash picture at the screen size (kiosk-boot-splash.py,
#      cached) and shows it on every screen with swaybg
#   3. gives the desktop (pcmanfm) the same picture and no icons on every screen,
#      because the desktop draws over swaybg once it starts
#   4. removes the taskbar
# The picture and the Plymouth boot theme share one logo and one layout.

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="${HOME}/.robot-arm-kiosk"
IMG="${OUT_DIR}/desktop-splash.png"
SIZE_FILE="${OUT_DIR}/desktop-splash.size"
BG_COLOUR='#0d1320'
PCMANFM_DIR="${HOME}/.config/pcmanfm/default"

mkdir -p "$OUT_DIR" "$PCMANFM_DIR"
exec >>"${OUT_DIR}/boot-splash.log" 2>&1
echo "==== $(date) boot splash ===="

# 1. Colour first: covers the gap before anything else draws
swaybg -c "$BG_COLOUR" &
COLOUR_PID=$!

# 4. Taskbar: the system autostart starts it (through lwrespawn, which would
#    restart it). Keep removing it for the first 20 s of the session.
(
    for _ in $(seq 1 100); do
        pkill -f 'lwrespawn.*wf-panel-pi' 2>/dev/null
        pkill -x wf-panel-pi 2>/dev/null
        sleep 0.2
    done
) &

# 3. Desktop settings, written before the picture exists so a desktop starting
#    now already has no icons and the right colour. Each screen has its own
#    file, named after the output; the common names are written too so a
#    screen moved to another port is already covered on the next boot.
write_desktop_config() {
    local output="$1"
    cat > "${PCMANFM_DIR}/desktop-items-${output}.conf" <<EOF
[*]
wallpaper_mode=crop
wallpaper_common=1
wallpaper=${IMG}
desktop_bg=${BG_COLOUR}
desktop_fg=#ffffff
desktop_shadow=${BG_COLOUR}
show_wm_menu=0
show_documents=0
show_home=0
show_trash=0
show_mounts=0
EOF
}

# Screen names and size from the compositor (it can take a moment to answer)
OUTPUTS=""
SIZE=""
for _ in $(seq 1 30); do
    RANDR="$(wlr-randr 2>/dev/null)"
    OUTPUTS="$(printf '%s\n' "$RANDR" | awk '/^[^ ]/ {print $1}')"
    SIZE="$(printf '%s\n' "$RANDR" | awk '/current/ {print $1; exit}')"
    [ -n "$OUTPUTS" ] && [ -n "$SIZE" ] && break
    sleep 0.1
done
SIZE="${SIZE:-1024x600}"
echo "screens: $(echo $OUTPUTS) size: $SIZE"

for output in $OUTPUTS HDMI-A-1 HDMI-A-2 DSI-1 DSI-2; do
    write_desktop_config "$output"
done

# 2. The picture, rendered once per screen size
LOGO=/usr/share/plymouth/themes/matrixtsl/logo.png
[ -f "$LOGO" ] || LOGO="${APP_DIR}/boot-logo.png"
if [ ! -s "$IMG" ] || [ "$(cat "$SIZE_FILE" 2>/dev/null)" != "$SIZE" ]; then
    if python3 "${APP_DIR}/kiosk-boot-splash.py" "$LOGO" "${SIZE%x*}" "${SIZE#*x}" "$IMG"; then
        echo "$SIZE" > "$SIZE_FILE"
        echo "rendered $IMG"
    else
        echo "could not render the picture; keeping the plain colour"
    fi
fi

if [ -s "$IMG" ]; then
    swaybg -o '*' -i "$IMG" -m fill -c "$BG_COLOUR" &
    sleep 0.5
    kill "$COLOUR_PID" 2>/dev/null
fi

# A desktop that started before the settings were written reloads them
pcmanfm --reconfigure 2>/dev/null || true

wait
