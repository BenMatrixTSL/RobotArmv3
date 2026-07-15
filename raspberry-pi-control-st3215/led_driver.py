#!/usr/bin/env python3
"""
WS2812B LED driver for the robot arm server.
Reads newline-delimited JSON commands from stdin, controls LEDs, replies on stdout.

Commands:
  {"cmd": "fill",       "r":0, "g":170, "b":0}
  {"cmd": "comet",      "r":255, "g":208, "b":0, "tail":14, "speed":1.4, "heads":1}
  {"cmd": "pulse",      "r":170, "g":170, "b":170, "lo":0.04, "hi":0.7, "period":2.5}
  {"cmd": "flash",      "r":255, "g":102, "b":0, "on_ms":150, "period_ms":500}
  {"cmd": "off"}
  {"cmd": "quit"}
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

_anim_stop   = threading.Event()
_anim_thread = None

def _stop_anim():
    global _anim_thread
    _anim_stop.set()
    if _anim_thread and _anim_thread.is_alive():
        _anim_thread.join(timeout=0.5)
    _anim_thread = None
    _anim_stop.clear()

def _color(r, g, b):
    return rpi_ws281x.Color(int(r), int(g), int(b))

def _show(pixels):
    for i, (r, g, b) in enumerate(pixels):
        strip.setPixelColor(i, _color(r, g, b))
    strip.show()

def _comet_loop(r, g, b, tail, secs_per_lap, heads):
    FRAME = 0.025
    step  = LED_COUNT / (secs_per_lap / FRAME)
    pos   = 0.0
    offsets = [i * LED_COUNT / heads for i in range(heads)]
    while not _anim_stop.is_set():
        pixels = [[0, 0, 0]] * LED_COUNT
        for offset in offsets:
            head = (pos + offset) % LED_COUNT
            for i in range(LED_COUNT):
                dist = (head - i) % LED_COUNT
                if dist < tail:
                    t = (1.0 - dist / tail) ** 1.8
                    pixels[i] = [
                        max(pixels[i][0], int(r * t)),
                        max(pixels[i][1], int(g * t)),
                        max(pixels[i][2], int(b * t)),
                    ]
        _show(pixels)
        pos = (pos + step) % LED_COUNT
        time.sleep(FRAME)

def _pulse_loop(r, g, b, lo, hi, period):
    FRAME = 0.025
    t = 0.0
    while not _anim_stop.is_set():
        br = lo + (hi - lo) * (0.5 + 0.5 * math.sin(t * 2 * math.pi / period))
        c  = [int(r * br), int(g * br), int(b * br)]
        _show([c] * LED_COUNT)
        t += FRAME
        time.sleep(FRAME)

def _flash_loop(r, g, b, on_ms, period_ms):
    on_s  = on_ms  / 1000.0
    per_s = period_ms / 1000.0
    while not _anim_stop.is_set():
        _show([[int(r), int(g), int(b)]] * LED_COUNT)
        time.sleep(on_s)
        if _anim_stop.is_set(): break
        _show([[0, 0, 0]] * LED_COUNT)
        time.sleep(per_s - on_s)

def _start(target, *args):
    global _anim_thread
    _stop_anim()
    _anim_thread = threading.Thread(target=target, args=args, daemon=True)
    _anim_thread.start()

# ── Helpers ───────────────────────────────────────────────────────────────────

def fill(r, g, b):
    if not available: return
    _stop_anim()
    _show([[r, g, b]] * LED_COUNT)

def off():
    fill(0, 0, 0)

def reply(ok, error=None):
    obj = {"ok": ok}
    if error: obj["error"] = error
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
    if not line: continue
    try:
        msg = json.loads(line)
        cmd = msg.get("cmd", "")
        r, g, b = int(msg.get("r",0)), int(msg.get("g",0)), int(msg.get("b",0))

        if cmd == "fill":
            fill(r, g, b)
            reply(True)

        elif cmd == "comet":
            if available:
                _start(_comet_loop, r, g, b,
                       float(msg.get("tail",  14)),
                       float(msg.get("speed", 1.4)),
                       int(  msg.get("heads", 1)))
            reply(True)

        elif cmd == "pulse":
            if available:
                _start(_pulse_loop, r, g, b,
                       float(msg.get("lo",     0.06)),
                       float(msg.get("hi",     0.75)),
                       float(msg.get("period", 2.5)))
            reply(True)

        elif cmd == "flash":
            if available:
                _start(_flash_loop, r, g, b,
                       float(msg.get("on_ms",     150)),
                       float(msg.get("period_ms", 500)))
            reply(True)

        elif cmd == "off":
            off(); reply(True)

        elif cmd == "quit":
            off(); reply(True); break

        else:
            reply(False, f"Unknown cmd: {cmd}")

    except Exception as e:
        reply(False, str(e))

_stop_anim()
off()
