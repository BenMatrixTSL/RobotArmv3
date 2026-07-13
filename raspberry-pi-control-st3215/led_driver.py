#!/usr/bin/env python3
"""
WS2812B LED driver for the robot arm server.
Reads newline-delimited JSON commands from stdin, controls LEDs, replies on stdout.

Protocol:
  Input:  {"cmd": "fill",  "r": 0, "g": 170, "b": 0}
          {"cmd": "comet", "r": 255, "g": 112, "b": 0, "tail": 14, "speed": 1.4}
          {"cmd": "off"}
          {"cmd": "quit"}
  Output: {"ok": true} or {"ok": false, "error": "..."}
"""
import sys, json, signal, threading, time, math

LED_COUNT = 30
LED_PIN   = 18

try:
    import rpi_ws281x
    strip = rpi_ws281x.PixelStrip(LED_COUNT, LED_PIN)
    strip.begin()
    available = True
except Exception as e:
    sys.stderr.write(f"[led_driver] Init failed: {e}\n")
    sys.stderr.flush()
    available = False

# ── Animation thread ──────────────────────────────────────────────────────────

_anim_stop  = threading.Event()
_anim_thread = None

def _stop_anim():
    global _anim_thread
    _anim_stop.set()
    if _anim_thread and _anim_thread.is_alive():
        _anim_thread.join(timeout=0.5)
    _anim_thread = None
    _anim_stop.clear()

def _comet_loop(r, g, b, tail, secs_per_lap):
    """Bright head travels around the ring with a power-curve fading tail."""
    FRAME_MS  = 0.025          # 40 fps
    step      = LED_COUNT / (secs_per_lap / FRAME_MS)
    pos       = 0.0

    while not _anim_stop.is_set():
        for i in range(LED_COUNT):
            dist = (pos - i) % LED_COUNT  # distance behind head (0 = head)
            if dist < tail:
                t = 1.0 - (dist / tail)
                brightness = t ** 1.8     # power curve: sharp at head, long fade
                strip.setPixelColor(i, rpi_ws281x.Color(
                    int(r * brightness),
                    int(g * brightness),
                    int(b * brightness)))
            else:
                strip.setPixelColor(i, rpi_ws281x.Color(0, 0, 0))
        strip.show()
        pos = (pos + step) % LED_COUNT
        time.sleep(FRAME_MS)

# ── Helpers ───────────────────────────────────────────────────────────────────

def fill(r, g, b):
    if not available:
        return
    _stop_anim()
    c = rpi_ws281x.Color(r, g, b)
    for i in range(LED_COUNT):
        strip.setPixelColor(i, c)
    strip.show()

def start_comet(r, g, b, tail, speed):
    global _anim_thread
    if not available:
        return
    _stop_anim()
    _anim_thread = threading.Thread(
        target=_comet_loop, args=(r, g, b, tail, speed), daemon=True)
    _anim_thread.start()

def off():
    fill(0, 0, 0)

def reply(ok, error=None):
    obj = {"ok": ok}
    if error:
        obj["error"] = error
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()

def shutdown(sig=None, frame=None):
    _stop_anim()
    off()
    sys.exit(0)

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT,  shutdown)

reply(True)  # ready signal

# ── Command loop ──────────────────────────────────────────────────────────────

for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        msg = json.loads(line)
        cmd = msg.get("cmd", "")
        if cmd == "fill":
            fill(int(msg.get("r", 0)), int(msg.get("g", 0)), int(msg.get("b", 0)))
            reply(True)
        elif cmd == "comet":
            start_comet(
                r     = int(msg.get("r",     255)),
                g     = int(msg.get("g",     112)),
                b     = int(msg.get("b",       0)),
                tail  = int(msg.get("tail",   14)),
                speed = float(msg.get("speed", 1.4)),
            )
            reply(True)
        elif cmd == "off":
            off()
            reply(True)
        elif cmd == "quit":
            off()
            reply(True)
            break
        else:
            reply(False, f"Unknown cmd: {cmd}")
    except Exception as e:
        reply(False, str(e))

_stop_anim()
off()
