#!/usr/bin/env python3
"""
WS2812B LED driver for the robot arm server.
Reads newline-delimited JSON commands from stdin, controls LEDs, replies on stdout.

Protocol:
  Input:  {"cmd": "fill", "r": 0, "g": 170, "b": 0}
          {"cmd": "off"}
          {"cmd": "quit"}
  Output: {"ok": true} or {"ok": false, "error": "..."}
"""
import sys, json, signal

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

def fill(r, g, b):
    if not available:
        return
    c = rpi_ws281x.Color(r, g, b)
    for i in range(LED_COUNT):
        strip.setPixelColor(i, c)
    strip.show()

def off():
    fill(0, 0, 0)

def reply(ok, error=None):
    obj = {"ok": ok}
    if error:
        obj["error"] = error
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()

def shutdown(sig=None, frame=None):
    off()
    sys.exit(0)

signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)

reply(True)  # ready signal

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

off()
