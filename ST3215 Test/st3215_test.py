#!/usr/bin/env python3
"""
ST3215 servo scanner and full health test.

Uses the official Feetech SC/ST servo SDK (`pip install feetech-servo-sdk`,
imported as `scservo_sdk`) for all bus traffic, so packet framing, checksums
and timeouts are the vendor's, not ours.

Purpose: screen a batch of ST3215 servos and flag the ones that are
electrically or mechanically damaged. Every test reports its own comms
failure count, and a summary line per motor is appended to reports/summary.csv
so motors can be compared against each other.

Typical use:
    python st3215_test.py                    # scan COM16, test everything found
    python st3215_test.py --id 1             # test one known ID
    python st3215_test.py --scan-only        # just list what is on the bus
    python st3215_test.py --no-motion        # comms/electrical only, no movement
    python st3215_test.py --manual           # adds hand-turn encoder sweep test

The motor MUST be free to move (no linkage, nothing to crash into) unless
--no-motion is given.
"""

import argparse
import csv
import json
import os
import statistics
import sys
import time
from datetime import datetime

try:
    from scservo_sdk import (
        PortHandler, PacketHandler, SCS_TOHOST, SCS_TOSCS,
        COMM_SUCCESS, COMM_RX_TIMEOUT, COMM_RX_CORRUPT, COMM_RX_FAIL,
        COMM_TX_FAIL, COMM_TX_ERROR,
    )
except ImportError:
    sys.exit("Missing dependency. Run:  pip install feetech-servo-sdk pyserial")

# --------------------------------------------------------------------------
# ST3215 / STS control table (byte addresses)
# --------------------------------------------------------------------------
EEPROM = {
    "fw_major": (0, 1), "fw_minor": (1, 1),
    "model_major": (3, 1), "model_minor": (4, 1),
    "id": (5, 1), "baud_index": (6, 1), "return_delay": (7, 1),
    "status_return_level": (8, 1),
    "min_angle_limit": (9, 2), "max_angle_limit": (11, 2),
    "max_temperature": (13, 1), "max_voltage": (14, 1), "min_voltage": (15, 1),
    "max_torque": (16, 2), "phase": (18, 1), "unloading_condition": (19, 1),
    "led_alarm_condition": (20, 1),
    "kp": (21, 1), "kd": (22, 1), "ki": (23, 1),
    "min_startup_force": (24, 2), "cw_deadband": (26, 1), "ccw_deadband": (27, 1),
    "protection_current": (28, 2), "angular_resolution": (30, 1),
    "position_offset": (31, 2), "operating_mode": (33, 1),
    "protective_torque": (34, 1), "protection_time": (35, 1),
    "overload_torque": (36, 1), "speed_loop_p": (37, 1),
    "overcurrent_time": (38, 1), "speed_loop_i": (39, 1),
}

ADDR_OPERATING_MODE = 33         # EEPROM: 0 = position, 1 = constant speed
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_LOCK = 55                   # 0 = EEPROM writable, 1 = locked
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_STATUS = 65
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69

CURRENT_UNIT_MA = 6.5      # ST3215 present-current LSB
ACC_UNIT = 100             # acceleration register LSB, counts/s^2
COUNTS_PER_REV = 4096
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV

ERROR_BITS = [
    (0x01, "voltage"),
    (0x02, "angle/encoder sensor"),
    (0x04, "overheat"),
    (0x08, "overcurrent"),
    (0x10, "angle limit"),
    (0x20, "overload"),
]

# --------------------------------------------------------------------------
# Pass / warn thresholds. These are screening heuristics for a 12 V ST3215
# running unloaded on the bench - tighten them once you have measured a few
# known-good motors from your own batch.
# --------------------------------------------------------------------------
class Limits:
    comm_fail_rate_warn = 0.005      # 0.5 % of transactions
    comm_fail_rate_fail = 0.02       # 2 %
    voltage_min = 6.0
    voltage_max = 14.5
    temp_max_start = 45              # degC before the test starts
    temp_rise_warn = 20              # degC rise across the whole test
    encoder_jitter_warn = 3          # counts peak-to-peak, torque off, still
    encoder_jitter_fail = 10
    idle_hold_current_warn_ma = 250  # holding position, no load
    idle_hold_current_fail_ma = 600
    move_current_warn_ma = 700       # mean current while sweeping unloaded
    move_current_fail_ma = 1400
    wheel_current_warn_ma = 300      # mean current spinning free in wheel mode
    wheel_current_fail_ma = 800
    settle_error_warn = 12           # counts of steady-state error
    settle_error_fail = 40
    settle_time_warn_s = 1.0
    settle_time_fail_s = 2.5
    hysteresis_warn = 15             # counts, forward vs reverse sweep
    hysteresis_fail = 45
    speed_ratio_warn = 0.65          # achieved / commanded speed
    speed_ratio_fail = 0.35

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"
RANK = {PASS: 0, SKIP: 1, WARN: 2, FAIL: 3}


# --------------------------------------------------------------------------
# Bus wrapper: every transaction is counted and classified
# --------------------------------------------------------------------------
class CommCounters:
    def __init__(self):
        self.attempts = 0
        self.ok = 0
        self.timeout = 0
        self.corrupt = 0
        self.rx_fail = 0
        self.tx_fail = 0
        self.other = 0
        self.hw_error_frames = 0     # replies whose ERROR byte was non-zero
        self.retried_ops = 0         # logical ops that needed more than one attempt
        self.failed_ops = 0          # logical ops that failed after all retries
        self.latency_ms = []

    def snapshot(self):
        return {k: (list(v) if isinstance(v, list) else v)
                for k, v in self.__dict__.items()}

    @staticmethod
    def delta(before, after):
        d = {}
        for k, v in after.items():
            if k == "latency_ms":
                d[k] = v[len(before[k]):]
            else:
                d[k] = v - before[k]
        return d

    @staticmethod
    def summarise(d):
        att = d["attempts"]
        bad = d["timeout"] + d["corrupt"] + d["rx_fail"] + d["tx_fail"] + d["other"]
        rate = (bad / att) if att else 0.0
        lat = d["latency_ms"]
        line = (f"comms: {att} transactions, {bad} failed ({rate * 100:.2f}%) "
                f"[timeout {d['timeout']}, corrupt {d['corrupt']}, rx_fail {d['rx_fail']}, "
                f"tx_fail {d['tx_fail']}, other {d['other']}], "
                f"hw-error replies {d['hw_error_frames']}, "
                f"retried ops {d['retried_ops']}, failed ops {d['failed_ops']}")
        if lat:
            line += (f", latency avg {statistics.mean(lat):.2f} ms / "
                     f"max {max(lat):.2f} ms")
        return line, rate, bad


class Bus:
    def __init__(self, port_name, baudrate, retries=2, verbose=False):
        self.port = PortHandler(port_name)
        self.pk = PacketHandler(0)          # STS/SMS little-endian protocol
        self.retries = retries
        self.verbose = verbose
        self.c = CommCounters()
        self.last_error_byte = 0
        if not self.port.openPort():
            raise SystemExit(f"Could not open {port_name}")
        if not self.port.setBaudRate(baudrate):
            raise SystemExit(f"Could not set baud rate {baudrate}")

    def close(self):
        try:
            self.port.closePort()
        except Exception:
            pass

    def _classify(self, result, error):
        if result == COMM_SUCCESS:
            self.c.ok += 1
            if error:
                self.c.hw_error_frames += 1
                self.last_error_byte |= error
            return True
        if result == COMM_RX_TIMEOUT:
            self.c.timeout += 1
        elif result == COMM_RX_CORRUPT:
            self.c.corrupt += 1
        elif result == COMM_RX_FAIL:
            self.c.rx_fail += 1
        elif result in (COMM_TX_FAIL, COMM_TX_ERROR):
            self.c.tx_fail += 1
        else:
            self.c.other += 1
        return False

    def _run(self, fn, retries=None):
        """fn() -> (value, result, error). Returns value, or None if it failed."""
        tries = self.retries if retries is None else retries
        for attempt in range(tries + 1):
            t0 = time.perf_counter()
            value, result, error = fn()
            dt = (time.perf_counter() - t0) * 1000.0
            self.c.attempts += 1
            self.c.latency_ms.append(dt)
            if self._classify(result, error):
                if attempt:
                    self.c.retried_ops += 1
                return value
            if self.verbose:
                print(f"      ! {self.pk.getTxRxResult(result).strip()} "
                      f"(attempt {attempt + 1}/{tries + 1})")
            # a late reply left in the buffer would shift every following
            # packet, turning one hiccup into a cascade of false failures
            self.port.clearPort()
        self.c.failed_ops += 1
        return None

    # --- primitives -------------------------------------------------------
    def ping(self, sid, retries=None):
        return self._run(lambda: self.pk.ping(self.port, sid), retries)

    def read1(self, sid, addr):
        return self._run(lambda: self.pk.read1ByteTxRx(self.port, sid, addr))

    def read2(self, sid, addr):
        return self._run(lambda: self.pk.read2ByteTxRx(self.port, sid, addr))

    def read_block(self, sid, addr, length):
        return self._run(lambda: self.pk.readTxRx(self.port, sid, addr, length))

    def write1(self, sid, addr, value):
        def op():
            r, e = self.pk.write1ByteTxRx(self.port, sid, addr, value & 0xFF)
            return True, r, e
        return self._run(op) is not None

    def write2(self, sid, addr, value):
        def op():
            r, e = self.pk.write2ByteTxRx(self.port, sid, addr, value & 0xFFFF)
            return True, r, e
        return self._run(op) is not None

    # --- typed helpers ----------------------------------------------------
    def position(self, sid):
        return self.read2(sid, ADDR_PRESENT_POSITION)

    def voltage(self, sid):
        v = self.read1(sid, ADDR_PRESENT_VOLTAGE)
        return None if v is None else v / 10.0

    def temperature(self, sid):
        return self.read1(sid, ADDR_PRESENT_TEMPERATURE)

    def status_byte(self, sid):
        return self.read1(sid, ADDR_STATUS)

    def telemetry(self, sid):
        """One 31-byte read covering 40..70 - fewer transactions, less time skew."""
        blk = self.read_block(sid, ADDR_TORQUE_ENABLE, 31)
        if blk is None or len(blk) < 31:
            return None

        def w(a):
            i = a - ADDR_TORQUE_ENABLE
            return blk[i] | (blk[i + 1] << 8)

        return {
            "torque_enable": blk[0],
            "goal_position": w(ADDR_GOAL_POSITION),
            "position": w(ADDR_PRESENT_POSITION),
            "speed": SCS_TOHOST(w(ADDR_PRESENT_SPEED), 15),
            "load_pct": SCS_TOHOST(w(ADDR_PRESENT_LOAD), 10) / 10.0,
            "voltage": blk[ADDR_PRESENT_VOLTAGE - ADDR_TORQUE_ENABLE] / 10.0,
            "temperature": blk[ADDR_PRESENT_TEMPERATURE - ADDR_TORQUE_ENABLE],
            "status": blk[ADDR_STATUS - ADDR_TORQUE_ENABLE],
            "moving": bool(blk[ADDR_MOVING - ADDR_TORQUE_ENABLE]),
            "current_ma": SCS_TOHOST(w(ADDR_PRESENT_CURRENT), 15) * CURRENT_UNIT_MA,
        }


def decode_status(byte):
    if byte is None:
        return ["unreadable"]
    return [name for bit, name in ERROR_BITS if byte & bit]


def signed15(v):
    return SCS_TOHOST(v, 15)


# --------------------------------------------------------------------------
# Test harness
# --------------------------------------------------------------------------
class Tester:
    def __init__(self, bus, sid, args):
        self.bus = bus
        self.sid = sid
        self.args = args
        self.results = []
        self.saved = {}
        self.config = {}
        self.range = (0, COUNTS_PER_REV - 1)
        self.start_temp = None

    # -- plumbing ----------------------------------------------------------
    def record(self, name, status, metrics, notes, comm_delta):
        line, rate, _bad = CommCounters.summarise(comm_delta)
        # comms quality can escalate a test's verdict on its own
        if comm_delta["attempts"] >= 20 and rate >= Limits.comm_fail_rate_fail:
            status = FAIL
            notes = notes + [f"comms failure rate {rate * 100:.2f}% on this test"]
        elif comm_delta["attempts"] >= 20 and rate >= Limits.comm_fail_rate_warn:
            if RANK[status] < RANK[WARN]:
                status = WARN
            notes = notes + [f"comms failure rate {rate * 100:.2f}% on this test"]
        entry = {
            "test": name, "status": status, "metrics": metrics, "notes": notes,
            "comms": {k: v for k, v in comm_delta.items() if k != "latency_ms"},
            "comms_fail_rate": round(rate, 5),
        }
        lat = comm_delta["latency_ms"]
        if lat:
            entry["comms"]["latency_ms_avg"] = round(statistics.mean(lat), 3)
            entry["comms"]["latency_ms_max"] = round(max(lat), 3)
        self.results.append(entry)
        print(f"  [{status:4}] {name}")
        for k, v in metrics.items():
            if k == "detail":
                continue
            print(f"         {k}: {v}")
        for n in notes:
            print(f"         - {n}")
        print(f"         {line}")
        return entry

    def run(self, name, fn):
        before = self.bus.c.snapshot()
        print(f"\n--- {name} ---")
        try:
            status, metrics, notes = fn()
        except KeyboardInterrupt:
            raise
        except Exception as exc:                      # a crash is a result too
            status, metrics, notes = FAIL, {}, [f"exception: {exc!r}"]
        after = self.bus.c.snapshot()
        return self.record(name, status, metrics, notes,
                           CommCounters.delta(before, after))

    # -- motion helpers ----------------------------------------------------
    def torque(self, on):
        return self.bus.write1(self.sid, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def goto(self, pos, speed=None, acc=None):
        if acc is not None:
            self.bus.write1(self.sid, ADDR_ACC, acc)
        if speed is not None:
            self.bus.write2(self.sid, ADDR_GOAL_SPEED, speed)
        return self.bus.write2(self.sid, ADDR_GOAL_POSITION, int(pos))

    def wait_settled(self, target, tol=10, timeout=3.0, poll=0.01):
        """Returns (settled, seconds_to_settle, telemetry_samples)."""
        t0 = time.perf_counter()
        samples = []
        settled_at = None
        while True:
            t = self.bus.telemetry(self.sid)
            now = time.perf_counter() - t0
            if t:
                t["t"] = now
                samples.append(t)
                if settled_at is None and abs(t["position"] - target) <= tol \
                        and not t["moving"]:
                    settled_at = now
            if settled_at is not None and now - settled_at > 0.25:
                return True, settled_at, samples
            if now > timeout:
                return False, None, samples
            time.sleep(poll)

    def safe_range(self):
        """Test span, clamped to the servo's own angle limits where they exist."""
        lo, hi = 0, COUNTS_PER_REV - 1
        min_lim = self.bus.read2(self.sid, 9)
        max_lim = self.bus.read2(self.sid, 11)
        if min_lim is not None and max_lim is not None and max_lim > min_lim:
            lo, hi = min_lim, max_lim
        centre = self.args.centre if self.args.centre is not None else (lo + hi) // 2
        half = self.args.span // 2
        return max(lo + 20, centre - half), min(hi - 20, centre + half)

    # ----------------------------------------------------------------------
    # Tests
    # ----------------------------------------------------------------------
    def t_identity(self):
        cfg = {}
        missing = []
        for name, (addr, size) in EEPROM.items():
            v = (self.bus.read1(self.sid, addr) if size == 1
                 else self.bus.read2(self.sid, addr))
            if v is None:
                missing.append(name)
            cfg[name] = v
        if cfg.get("position_offset") is not None:
            cfg["position_offset"] = signed15(cfg["position_offset"])
        self.config = cfg
        notes = []
        status = PASS
        if missing:
            status = FAIL
            notes.append("could not read: " + ", ".join(missing))
        if cfg.get("operating_mode") not in (0, None):
            notes.append(f"operating mode is {cfg['operating_mode']} "
                         "(0 = position); the motion tests assume position mode")
            if status == PASS:
                status = WARN
        metrics = {
            "firmware": f"{cfg.get('fw_major')}.{cfg.get('fw_minor')}",
            "model": f"{cfg.get('model_major')}.{cfg.get('model_minor')}",
            "id": cfg.get("id"),
            "angle limits": f"{cfg.get('min_angle_limit')} .. {cfg.get('max_angle_limit')}",
            "PID (P/I/D)": f"{cfg.get('kp')}/{cfg.get('ki')}/{cfg.get('kd')}",
            "deadband CW/CCW": f"{cfg.get('cw_deadband')}/{cfg.get('ccw_deadband')}",
            "position offset": cfg.get("position_offset"),
            "protection current": cfg.get("protection_current"),
            "limits (temp / voltage)": (f"{cfg.get('max_temperature')} degC, "
                                        f"{cfg.get('min_voltage')}-{cfg.get('max_voltage')} "
                                        "(0.1 V units)"),
            "operating mode": cfg.get("operating_mode"),
        }
        return status, metrics, notes

    def t_comms(self):
        n = self.args.comm_samples
        ping_fail = read_fail = 0
        latencies = []
        for i in range(n):
            t0 = time.perf_counter()
            if self.bus.ping(self.sid, retries=0) is None:
                ping_fail += 1
            latencies.append((time.perf_counter() - t0) * 1000)
            if self.bus.read_block(self.sid, ADDR_TORQUE_ENABLE, 31) is None:
                read_fail += 1
            if i % max(1, n // 20) == 0:
                print(".", end="", flush=True)
        print()
        total = n * 2
        bad = ping_fail + read_fail
        rate = bad / total
        status = PASS
        notes = []
        if rate >= Limits.comm_fail_rate_fail:
            status = FAIL
            notes.append("unreliable bus: check wiring, connector crimps, ground, "
                         "and the servo's own UART/MCU")
        elif rate >= Limits.comm_fail_rate_warn:
            status = WARN
            notes.append("intermittent comms - suspect the cable or connector first")
        return status, {
            "ping failures": f"{ping_fail}/{n}",
            "31-byte read failures": f"{read_fail}/{n}",
            "overall failure rate": f"{rate * 100:.2f}%",
            "ping latency avg/max": f"{statistics.mean(latencies):.2f} / "
                                    f"{max(latencies):.2f} ms",
        }, notes

    def t_power_idle(self):
        self.torque(False)
        time.sleep(0.2)
        v, tmp, cur, ld, st = [], [], [], [], 0
        for _ in range(20):
            t = self.bus.telemetry(self.sid)
            if t:
                v.append(t["voltage"])
                tmp.append(t["temperature"])
                cur.append(t["current_ma"])
                ld.append(abs(t["load_pct"]))
                st |= t["status"]
            time.sleep(0.02)
        if not v:
            return FAIL, {}, ["no telemetry returned"]
        self.start_temp = statistics.median(tmp)
        flags = decode_status(st)
        status = PASS
        notes = []
        vm = statistics.median(v)
        peak_i = max(abs(c) for c in cur)
        if not (Limits.voltage_min <= vm <= Limits.voltage_max):
            status = FAIL
            notes.append(f"supply voltage {vm:.1f} V outside "
                         f"{Limits.voltage_min}-{Limits.voltage_max} V")
        if max(v) - min(v) > 0.5:
            if status == PASS:
                status = WARN
            notes.append(f"supply sagging or noisy at idle: {min(v):.1f}-{max(v):.1f} V")
        if self.start_temp >= Limits.temp_max_start:
            if status == PASS:
                status = WARN
            notes.append(f"starts at {self.start_temp:.0f} degC - let it cool, or "
                         "suspect a stalled/shorted winding")
        if peak_i > 80:
            if status == PASS:
                status = WARN
            notes.append(f"draws {peak_i:.0f} mA with torque off - unexpected for a "
                         "de-energised motor")
        if flags:
            status = FAIL
            notes.append("servo error flags set at idle: " + ", ".join(flags))
        return status, {
            "voltage": f"{vm:.1f} V (min {min(v):.1f}, max {max(v):.1f})",
            "temperature": f"{self.start_temp:.0f} degC",
            "current (torque off)": f"{statistics.mean(cur):.0f} mA "
                                    f"(peak {peak_i:.0f})",
            "load": f"{statistics.mean(ld):.1f} %",
            "status flags": ", ".join(flags) or "none",
        }, notes

    def t_encoder_static(self):
        self.torque(False)
        time.sleep(0.2)
        pos = []
        for _ in range(self.args.encoder_samples):
            p = self.bus.position(self.sid)
            if p is not None:
                pos.append(p)
            time.sleep(0.005)
        if len(pos) < 10:
            return FAIL, {"samples": len(pos)}, ["not enough position reads"]
        p2p = max(pos) - min(pos)
        sd = statistics.pstdev(pos)
        jumps = [abs(b - a) for a, b in zip(pos, pos[1:])]
        big = sum(1 for j in jumps if j > 20)
        status = PASS
        notes = []
        if p2p >= Limits.encoder_jitter_fail or big:
            status = FAIL
            notes.append("magnetic encoder unstable while stationary - damaged "
                         "sensor, cracked or shifted magnet, or a loose output shaft")
        elif p2p >= Limits.encoder_jitter_warn:
            status = WARN
            notes.append("more encoder noise than a healthy unit shows at rest")
        median_pos = statistics.median(pos)
        return status, {
            "samples": len(pos),
            "position": f"{median_pos:.0f} counts "
                        f"({(median_pos - 2048) * DEG_PER_COUNT:+.1f} deg from centre)",
            "peak-to-peak": f"{p2p} counts",
            "std dev": f"{sd:.2f} counts",
            "jumps > 20 counts": big,
        }, notes

    def t_manual_sweep(self):
        """Hand-turn test: catches dead spots, dropouts and a slipping magnet."""
        self.torque(False)
        time.sleep(0.2)
        print("  Torque is off. Slowly turn the output horn through its full "
              "travel, both ways.")
        print(f"  Recording for {self.args.manual_seconds} s ...")
        t_end = time.perf_counter() + self.args.manual_seconds
        pos, gaps, fails = [], [], 0
        last = None
        while time.perf_counter() < t_end:
            p = self.bus.position(self.sid)
            if p is None:
                fails += 1
            else:
                pos.append(p)
                if last is not None:
                    d = abs(p - last)
                    d = min(d, COUNTS_PER_REV - d)     # wrap-safe
                    if d > 200:
                        gaps.append(d)
                last = p
            time.sleep(0.005)
        if len(pos) < 50:
            return FAIL, {"samples": len(pos)}, ["too few reads to judge"]
        span = max(pos) - min(pos)
        status = PASS
        notes = []
        if gaps:
            status = FAIL
            notes.append(f"{len(gaps)} encoder discontinuities (max {max(gaps)} counts) "
                         "- dead zone or dropout in the magnetic sensor")
        if span < 200:
            if status == PASS:
                status = WARN
            notes.append("barely moved - either it was not turned, or the shaft is seized")
        return status, {
            "samples": len(pos),
            "range turned": f"{span} counts ({span * DEG_PER_COUNT:.0f} deg)",
            "min / max": f"{min(pos)} / {max(pos)}",
            "read failures": fails,
        }, notes

    def t_torque_hold(self):
        lo, hi = self.range
        mid = (lo + hi) // 2
        self.torque(True)
        self.goto(mid, speed=self.args.speed, acc=self.args.acc)
        settled, t_s, _ = self.wait_settled(mid, timeout=4.0)
        time.sleep(0.3)
        cur, tmp, ld, st = [], [], [], 0
        t_end = time.perf_counter() + self.args.hold_seconds
        while time.perf_counter() < t_end:
            t = self.bus.telemetry(self.sid)
            if t:
                cur.append(abs(t["current_ma"]))
                tmp.append(t["temperature"])
                ld.append(abs(t["load_pct"]))
                st |= t["status"]
            time.sleep(0.02)
        if not cur:
            return FAIL, {}, ["no telemetry while holding"]
        mean_i, max_i = statistics.mean(cur), max(cur)
        flags = decode_status(st)
        status = PASS
        notes = []
        if not settled:
            status = FAIL
            notes.append("did not reach the hold position - jammed gearbox, stripped "
                         "gear, or a dead driver stage")
        if mean_i >= Limits.idle_hold_current_fail_ma:
            status = FAIL
            notes.append(f"{mean_i:.0f} mA to hold an unloaded shaft - shorted winding, "
                         "dragging gearbox, or the control loop is oscillating")
        elif mean_i >= Limits.idle_hold_current_warn_ma:
            if status == PASS:
                status = WARN
            notes.append("higher holding current than a healthy unloaded motor")
        if flags:
            status = FAIL
            notes.append("error flags while holding: " + ", ".join(flags))
        return status, {
            "settled": ("yes" if settled else "NO")
                       + (f" in {t_s:.2f} s" if settled else ""),
            "holding current mean/max": f"{mean_i:.0f} / {max_i:.0f} mA",
            "load": f"{statistics.mean(ld):.1f} %",
            "temperature": f"{statistics.mean(tmp):.0f} degC",
            "status flags": ", ".join(flags) or "none",
        }, notes

    def t_step_response(self):
        lo, hi = self.range
        mid = (lo + hi) // 2
        self.torque(True)
        rows = []
        worst = PASS
        notes = []
        for delta in self.args.steps:
            for sign in (+1, -1):
                target = max(lo, min(hi, mid + sign * delta))
                self.goto(mid, speed=self.args.speed, acc=self.args.acc)
                self.wait_settled(mid, timeout=4.0)
                time.sleep(0.15)
                start = self.bus.position(self.sid)
                start = mid if start is None else start
                self.goto(target)
                settled, t_settle, samples = self.wait_settled(target, tol=8, timeout=4.0)
                if not samples:
                    rows.append({"delta": sign * delta, "settled": False})
                    worst = FAIL
                    notes.append(f"step {sign * delta:+d}: no telemetry during the move")
                    continue
                final = statistics.median([s["position"] for s in samples[-5:]])
                err = int(final - target)
                travel = [s["position"] for s in samples]
                overshoot = (max(0, max(travel) - target) if sign > 0
                             else max(0, target - min(travel)))
                peak_i = max(abs(s["current_ma"]) for s in samples)
                moved = int(abs(final - start))
                rows.append({
                    "delta": sign * delta,
                    "settled": bool(settled),
                    "settle_s": round(t_settle, 3) if t_settle else None,
                    "error_counts": err,
                    "overshoot_counts": int(overshoot),
                    "peak_current_ma": round(peak_i),
                    "moved_counts": moved,
                })
                if moved < abs(delta) * 0.5:
                    worst = FAIL
                    notes.append(f"step {sign * delta:+d}: only moved {moved} counts "
                                 "- slipping or stripped gear train")
                elif not settled:
                    worst = FAIL
                    notes.append(f"step {sign * delta:+d}: never settled")
                else:
                    if abs(err) >= Limits.settle_error_fail:
                        worst = FAIL
                        notes.append(f"step {sign * delta:+d}: {err:+d} counts "
                                     "steady-state error")
                    elif abs(err) >= Limits.settle_error_warn and RANK[worst] < RANK[WARN]:
                        worst = WARN
                    if t_settle and t_settle >= Limits.settle_time_fail_s:
                        worst = FAIL
                        notes.append(f"step {sign * delta:+d}: took {t_settle:.2f} s "
                                     "to settle")
                    elif t_settle and t_settle >= Limits.settle_time_warn_s \
                            and RANK[worst] < RANK[WARN]:
                        worst = WARN
                if peak_i >= Limits.move_current_fail_ma:
                    worst = FAIL
                    notes.append(f"step {sign * delta:+d}: peak {peak_i:.0f} mA - "
                                 "excessive drive current for an unloaded move")
        good = [r for r in rows if r.get("settled")]
        metrics = {
            "steps run": len(rows),
            "steps settled": f"{len(good)}/{len(rows)}",
        }
        if good:
            metrics["settle time avg/max"] = (
                f"{statistics.mean([r['settle_s'] for r in good]):.2f} / "
                f"{max(r['settle_s'] for r in good):.2f} s")
            metrics["|error| avg/max"] = (
                f"{statistics.mean([abs(r['error_counts']) for r in good]):.1f} / "
                f"{max(abs(r['error_counts']) for r in good)} counts")
            metrics["overshoot max"] = f"{max(r['overshoot_counts'] for r in good)} counts"
            metrics["peak current max"] = f"{max(r['peak_current_ma'] for r in good)} mA"
        metrics["detail"] = rows
        return worst, metrics, notes

    def t_sweep(self):
        """Slow bidirectional sweep: friction, dead spots, backlash, tight spots."""
        lo, hi = self.range
        n = max(3, self.args.sweep_points)
        targets = [int(round(lo + (hi - lo) * i / (n - 1))) for i in range(n)]
        self.torque(True)
        self.goto(targets[0], speed=self.args.sweep_speed, acc=self.args.acc)
        self.wait_settled(targets[0], timeout=5.0)
        time.sleep(0.3)

        def pass_over(seq):
            out = []
            for tgt in seq:
                self.goto(tgt)
                settled, _t, samples = self.wait_settled(tgt, tol=8, timeout=3.0)
                pos = self.bus.position(self.sid)
                cur = [abs(s["current_ma"]) for s in samples] or [0]
                out.append({
                    "target": tgt,
                    "reached": pos,
                    "error": None if pos is None else pos - tgt,
                    "settled": settled,
                    "peak_current_ma": round(max(cur)),
                    "mean_current_ma": round(statistics.mean(cur)),
                    "load_pct": (round(max(abs(s["load_pct"]) for s in samples), 1)
                                 if samples else None),
                })
                print(".", end="", flush=True)
            return out

        fwd = pass_over(targets)
        rev = pass_over(list(reversed(targets)))
        print()
        rev_by_target = {r["target"]: r for r in rev}
        hyst = [abs(f["error"] - rev_by_target[f["target"]]["error"])
                for f in fwd
                if f["error"] is not None
                and rev_by_target[f["target"]]["error"] is not None]
        allpts = fwd + rev
        errs = [abs(p["error"]) for p in allpts if p["error"] is not None]
        cur_mean = [p["mean_current_ma"] for p in allpts]
        cur_peak = [p["peak_current_ma"] for p in allpts]
        stuck = [p for p in allpts if not p["settled"]]
        status = PASS
        notes = []
        if stuck:
            status = FAIL
            notes.append(f"{len(stuck)} of {len(allpts)} sweep points never settled "
                         f"(first at target {stuck[0]['target']}) - dead spot, notchy "
                         "gearbox, or a bad encoder region")
        if errs and max(errs) >= Limits.settle_error_fail:
            status = FAIL
            notes.append(f"worst positioning error {max(errs)} counts "
                         f"({max(errs) * DEG_PER_COUNT:.1f} deg)")
        h = statistics.mean(hyst) if hyst else 0.0
        if h >= Limits.hysteresis_fail:
            status = FAIL
            notes.append(f"{h:.0f} counts of hysteresis ({h * DEG_PER_COUNT:.1f} deg) "
                         "- worn or damaged gear train")
        elif h >= Limits.hysteresis_warn and RANK[status] < RANK[WARN]:
            status = WARN
            notes.append("noticeable backlash between sweep directions")
        mi = statistics.mean(cur_mean) if cur_mean else 0.0
        if mi >= Limits.move_current_fail_ma:
            status = FAIL
            notes.append(f"{mi:.0f} mA average to move unloaded - binding gearbox or "
                         "failing motor")
        elif mi >= Limits.move_current_warn_ma and RANK[status] < RANK[WARN]:
            status = WARN
            notes.append("high running current for an unloaded motor")
        # one point drawing far more than the rest means a localised tight spot
        if len(cur_peak) > 4:
            med = statistics.median(cur_peak)
            spikes = [p for p in allpts
                      if med > 0 and p["peak_current_ma"] > max(3 * med, 400)]
            if spikes:
                status = FAIL
                notes.append(f"{len(spikes)} current spike(s) at specific angles "
                             f"(e.g. target {spikes[0]['target']} drew "
                             f"{spikes[0]['peak_current_ma']} mA vs {med:.0f} mA "
                             "typical) - localised mechanical tight spot")
        return status, {
            "range swept": f"{lo} .. {hi} counts ({(hi - lo) * DEG_PER_COUNT:.0f} deg)",
            "points (each direction)": n,
            "points not settled": len(stuck),
            "|error| avg/max": (f"{statistics.mean(errs):.1f} / {max(errs)} counts"
                                if errs else "n/a"),
            "hysteresis (backlash)": f"{h:.1f} counts ({h * DEG_PER_COUNT:.2f} deg)",
            "running current avg/peak": f"{mi:.0f} / {max(cur_peak) if cur_peak else 0} mA",
            "detail": allpts,
        }, notes

    def t_speed(self):
        """Judged on the cruise (plateau) speed, not the average over the move:
        a short move is dominated by the acceleration ramp, so its average can
        never reach the commanded speed even on a perfect motor."""
        lo, hi = self.range
        span = hi - lo
        if span < 400:
            return SKIP, {}, ["test span too small for a speed measurement"]
        # a speed only plateaus if the ramp up and back down fits inside the
        # span; above that the move is triangular and its speed says more about
        # the acceleration setting than about the motor. Those are left to the
        # constant-speed test, which has no distance to run out of.
        accel = max(1, self.args.acc) * ACC_UNIT
        testable_max = (accel * span) ** 0.5 * 0.6
        testable = [s for s in self.args.speeds if s <= testable_max]
        too_fast = [s for s in self.args.speeds if s > testable_max]
        if not testable:
            return SKIP, {}, [f"no commanded speed below {testable_max:.0f} counts/s "
                              "fits in this span - see the constant-speed test"]
        self.torque(True)
        rows = []
        status = PASS
        notes = []
        if too_fast:
            notes.append(f"{', '.join(str(s) for s in too_fast)} counts/s cannot "
                         f"plateau within {span} counts at acc {self.args.acc}; "
                         "top speed is measured by the constant-speed test instead")
        for cmd in testable:
            self.goto(lo, speed=self.args.speed, acc=self.args.acc)
            self.wait_settled(lo, timeout=6.0)
            time.sleep(0.3)
            self.bus.write2(self.sid, ADDR_GOAL_SPEED, cmd)
            readback = self.bus.read2(self.sid, ADDR_GOAL_SPEED)
            self.goto(hi)
            timeout = max(6.0, span / max(cmd, 1) * 3.0)
            settled, t_s, samples = self.wait_settled(hi, tol=10, timeout=timeout)
            if not settled or not samples or not t_s:
                rows.append({"commanded": cmd, "goal_speed_readback": readback,
                             "cruise": None})
                status = FAIL
                notes.append(f"did not complete the move at speed {cmd}")
                continue
            speeds = [abs(s["speed"]) for s in samples]
            peak = max(speeds)
            # cruise = median of the samples above half peak, i.e. ramps excluded
            cruising = [s for s in speeds if s > peak * 0.5] or [peak]
            cruise = statistics.median(cruising)
            ratio = cruise / cmd if cmd else 0
            row = {
                "commanded": cmd,
                "goal_speed_readback": readback,
                "cruise": round(cruise),
                "peak_reported": peak,
                "average_over_move": round(span / t_s),
                "ratio": round(ratio, 2),
                "peak_current_ma": round(max(abs(s["current_ma"]) for s in samples)),
            }
            rows.append(row)
            if readback != cmd:
                if RANK[status] < RANK[WARN]:
                    status = WARN
                notes.append(f"goal speed read back as {readback} after writing {cmd}")
            if ratio < Limits.speed_ratio_fail:
                status = FAIL
                notes.append(f"only cruised at {cruise:.0f} of {cmd} counts/s - weak "
                             "motor, low supply, or heavy drag")
            elif ratio < Limits.speed_ratio_warn and RANK[status] < RANK[WARN]:
                status = WARN
                notes.append(f"slow at commanded speed {cmd} "
                             f"(cruised {cruise:.0f} counts/s)")
        metrics = {"detail": rows}
        for r in rows:
            metrics[f"speed {r['commanded']}"] = (
                f"cruised {r['cruise']} counts/s (ratio {r.get('ratio')}), "
                f"peak {r.get('peak_reported')}, avg over move "
                f"{r.get('average_over_move')}, {r.get('peak_current_ma')} mA"
                if r["cruise"] is not None else "move did not complete")
        return status, metrics, notes

    def set_mode(self, mode):
        """Operating mode lives in EEPROM, so unlock, write, verify, re-lock.
        The commit takes a few ms, and a read issued too early comes back
        stale, so the verification is retried before it is believed."""
        self.bus.write1(self.sid, ADDR_LOCK, 0)
        time.sleep(0.05)
        self.bus.write1(self.sid, ADDR_OPERATING_MODE, mode)
        ok = False
        for _ in range(5):
            time.sleep(0.05)
            self.bus.port.clearPort()
            if self.bus.read1(self.sid, ADDR_OPERATING_MODE) == mode:
                ok = True
                break
        self.bus.write1(self.sid, ADDR_LOCK, 1)
        time.sleep(0.05)
        return ok

    def spin(self, speed_signed, revolutions, timeout):
        """Run in wheel mode until the shaft has turned `revolutions`.
        Returns (samples, travel_counts, elapsed). Position is unwrapped as we
        go, so this measures real mechanical travel across the 0/4095 seam."""
        target = revolutions * COUNTS_PER_REV
        self.bus.write2(self.sid, ADDR_GOAL_SPEED,
                        SCS_TOSCS(int(speed_signed), 15))
        t0 = time.perf_counter()
        samples = []
        travel = 0
        last_pos = None
        jumps = []
        while True:
            t = self.bus.telemetry(self.sid)
            now = time.perf_counter() - t0
            if t:
                pos = t["position"]
                if last_pos is not None:
                    d = pos - last_pos
                    if d > COUNTS_PER_REV // 2:
                        d -= COUNTS_PER_REV
                    elif d < -COUNTS_PER_REV // 2:
                        d += COUNTS_PER_REV
                    travel += d
                    if abs(d) > 400:            # far beyond one poll of travel
                        jumps.append((pos, d))
                last_pos = pos
                t["t"] = now
                t["travel"] = travel
                samples.append(t)
                if abs(travel) >= target:
                    break
            if now > timeout:
                break
            time.sleep(0.01)
        elapsed = time.perf_counter() - t0
        self.bus.write2(self.sid, ADDR_GOAL_SPEED, 0)
        time.sleep(0.4)
        return samples, travel, elapsed, jumps

    def t_continuous(self):
        """Constant-speed (wheel) mode: multiple full revolutions in each
        direction. The position-mode tests only ever see the span they are
        given, so this is the only test that exercises the whole 360 deg of
        the gearbox and encoder, and both directions of it."""
        original_mode = self.saved.get("mode")
        self.bus.write1(self.sid, ADDR_TORQUE_ENABLE, 0)
        if not self.set_mode(1):
            return FAIL, {}, ["servo refused to enter constant-speed mode "
                              "(EEPROM register 33) - it may be write-locked"]
        try:
            self.torque(True)
            cmd = self.args.wheel_speed
            turns = self.args.turns
            timeout = (turns * COUNTS_PER_REV / max(cmd, 1)) * 2.5 + 5.0
            per_dir = {}
            status = PASS
            notes = []
            for label, sign in (("forward", +1), ("reverse", -1)):
                print(f"  {label}: {turns} revolutions at {cmd} counts/s ...",
                      flush=True)
                samples, travel, elapsed, jumps = self.spin(sign * cmd, turns, timeout)
                revs = abs(travel) / COUNTS_PER_REV
                if not samples or revs < 0.2:
                    status = FAIL
                    notes.append(f"{label}: did not turn (only {revs:.2f} rev) "
                                 "- stalled, jammed, or no drive")
                    per_dir[label] = {"revolutions": round(revs, 2)}
                    continue
                moving = [s for s in samples if s["t"] > 0.3]
                speeds = [abs(s["speed"]) for s in moving] or [0]
                currents = [abs(s["current_ma"]) for s in moving] or [0]
                mean_speed = abs(travel) / elapsed if elapsed else 0
                stalls = [s for s in moving if abs(s["speed"]) < cmd * 0.3]
                flags = 0
                for s in samples:
                    flags |= s["status"]
                # bin the whole mechanical circle: one bad tooth or a tight
                # spot shows up as a current peak or a speed dip in its sector
                nb = 24
                bins = [[] for _ in range(nb)]
                cbins = [[] for _ in range(nb)]
                for s in moving:
                    i = min(nb - 1, s["position"] * nb // COUNTS_PER_REV)
                    bins[i].append(abs(s["speed"]))
                    cbins[i].append(abs(s["current_ma"]))
                sector_speed = [statistics.median(b) if b else None for b in bins]
                sector_current = [statistics.median(b) if b else None for b in cbins]
                covered = sum(1 for b in bins if b)
                med_speed = statistics.median([v for v in sector_speed if v is not None]) \
                    if covered else 0
                med_cur = statistics.median([v for v in sector_current if v is not None]) \
                    if covered else 0
                slow_sectors = [(i, v) for i, v in enumerate(sector_speed)
                                if v is not None and med_speed > 0 and v < med_speed * 0.7]
                hot_sectors = [(i, v) for i, v in enumerate(sector_current)
                               if v is not None and v > max(3 * med_cur, 400)]
                per_dir[label] = {
                    "revolutions": round(revs, 2),
                    "elapsed_s": round(elapsed, 2),
                    "commanded_speed": cmd,
                    "mean_speed": round(mean_speed),
                    "speed_ratio": round(mean_speed / cmd, 2) if cmd else None,
                    "speed_min/median/max": [round(min(speeds)), round(statistics.median(speeds)),
                                             round(max(speeds))],
                    "current_mean_ma": round(statistics.mean(currents)),
                    "current_peak_ma": round(max(currents)),
                    "sectors_covered": f"{covered}/{nb}",
                    "sector_speed": [None if v is None else round(v) for v in sector_speed],
                    "sector_current_ma": [None if v is None else round(v)
                                          for v in sector_current],
                    "stall_samples": len(stalls),
                    "encoder_jumps": [{"position": p, "delta": d} for p, d in jumps],
                    "error_flags": decode_status(flags),
                }
                if revs < turns * 0.9:
                    status = FAIL
                    notes.append(f"{label}: managed only {revs:.2f} of {turns} "
                                 "revolutions before timing out - it is binding "
                                 "or stalling somewhere in the rotation")
                if jumps:
                    status = FAIL
                    notes.append(f"{label}: {len(jumps)} encoder discontinuities "
                                 f"(worst {max(abs(d) for _p, d in jumps)} counts near "
                                 f"position {jumps[0][0]}) - dropout or dead zone")
                if len(stalls) > len(moving) * 0.05:
                    status = FAIL
                    notes.append(f"{label}: speed collapsed on {len(stalls)} of "
                                 f"{len(moving)} samples - it catches somewhere in "
                                 "the turn")
                if slow_sectors:
                    status = FAIL
                    i, v = slow_sectors[0]
                    notes.append(f"{label}: {len(slow_sectors)} of {nb} sectors run "
                                 f"slow (e.g. {i * 360 // nb}-{(i + 1) * 360 // nb} deg "
                                 f"at {v:.0f} vs {med_speed:.0f} counts/s typical) - "
                                 "localised drag in the gearbox")
                if hot_sectors:
                    status = FAIL
                    i, v = hot_sectors[0]
                    notes.append(f"{label}: current spikes in {len(hot_sectors)} "
                                 f"sector(s) (e.g. {i * 360 // nb}-{(i + 1) * 360 // nb} "
                                 f"deg at {v:.0f} mA vs {med_cur:.0f} mA typical) - "
                                 "tight spot or a damaged tooth at that angle")
                if statistics.mean(currents) >= Limits.wheel_current_fail_ma:
                    status = FAIL
                    notes.append(f"{label}: {statistics.mean(currents):.0f} mA average "
                                 "to spin unloaded - binding gearbox or failing motor")
                elif statistics.mean(currents) >= Limits.wheel_current_warn_ma \
                        and RANK[status] < RANK[WARN]:
                    status = WARN
                    notes.append(f"{label}: high running current for a free-spinning "
                                 "motor")
                if decode_status(flags):
                    status = FAIL
                    notes.append(f"{label}: error flags during the run: "
                                 + ", ".join(decode_status(flags)))
            # top speed, measured where there is no distance to run out of
            top_cmd = self.args.max_wheel_speed
            print(f"  top speed: {max(1.0, turns)} revolution(s) at {top_cmd} "
                  "counts/s ...", flush=True)
            samples, travel, elapsed, jumps = self.spin(
                top_cmd, max(1.0, turns),
                (max(1.0, turns) * COUNTS_PER_REV / max(top_cmd, 1)) * 3.0 + 5.0)
            cruising = [abs(s["speed"]) for s in samples if s["t"] > 0.5]
            if cruising:
                cruise = statistics.median(cruising)
                ratio = cruise / top_cmd if top_cmd else 0
                per_dir["top_speed"] = {
                    "commanded": top_cmd,
                    "cruise": round(cruise),
                    "peak_reported": round(max(cruising)),
                    "mean_speed": round(abs(travel) / elapsed) if elapsed else 0,
                    "ratio": round(ratio, 2),
                    "current_mean_ma": round(statistics.mean(
                        [abs(s["current_ma"]) for s in samples if s["t"] > 0.5])),
                    "revolutions": round(abs(travel) / COUNTS_PER_REV, 2),
                }
                if ratio < Limits.speed_ratio_fail:
                    status = FAIL
                    notes.append(f"top speed: only {cruise:.0f} of {top_cmd} counts/s "
                                 "- weak motor, low supply, or heavy drag")
                elif ratio < Limits.speed_ratio_warn and RANK[status] < RANK[WARN]:
                    status = WARN
                    notes.append(f"top speed: cruised {cruise:.0f} of {top_cmd} "
                                 "counts/s")
            else:
                status = FAIL
                notes.append("top speed: no motion at all")
            # a motor that is fine one way and heavy the other has a directional
            # fault: a chipped tooth face, or a bearing loaded to one side
            f, r = per_dir.get("forward", {}), per_dir.get("reverse", {})
            if f.get("current_mean_ma") and r.get("current_mean_ma"):
                hi_c = max(f["current_mean_ma"], r["current_mean_ma"])
                lo_c = min(f["current_mean_ma"], r["current_mean_ma"])
                if hi_c > max(2.5 * lo_c, 120):
                    status = FAIL
                    notes.append(f"direction asymmetry: {f['current_mean_ma']} mA "
                                 f"forward vs {r['current_mean_ma']} mA reverse")
            metrics = {
                "forward": (f"{f.get('revolutions')} rev, "
                            f"{f.get('mean_speed')} counts/s (cmd {self.args.wheel_speed}), "
                            f"{f.get('current_mean_ma')} mA avg / "
                            f"{f.get('current_peak_ma')} mA peak"),
                "reverse": (f"{r.get('revolutions')} rev, "
                            f"{r.get('mean_speed')} counts/s (cmd {self.args.wheel_speed}), "
                            f"{r.get('current_mean_ma')} mA avg / "
                            f"{r.get('current_peak_ma')} mA peak"),
                "full circle covered": f"{f.get('sectors_covered')} forward, "
                                       f"{r.get('sectors_covered')} reverse (24 sectors)",
                "top speed": (f"cruised {per_dir['top_speed']['cruise']} of "
                              f"{per_dir['top_speed']['commanded']} counts/s "
                              f"(ratio {per_dir['top_speed']['ratio']}), "
                              f"{per_dir['top_speed']['current_mean_ma']} mA"
                              if "top_speed" in per_dir else "not measured"),
                "detail": per_dir,
            }
            return status, metrics, notes
        finally:
            self.bus.write2(self.sid, ADDR_GOAL_SPEED, 0)
            self.bus.write1(self.sid, ADDR_TORQUE_ENABLE, 0)
            restore_to = 0 if original_mode is None else original_mode
            if not self.set_mode(restore_to):
                print(f"  !! could not restore operating mode to {restore_to} - "
                      "check register 33 before using this servo")

    def t_thermal(self):
        t = self.bus.temperature(self.sid)
        v = self.bus.voltage(self.sid)
        st = self.bus.status_byte(self.sid)
        flags = decode_status(st)
        status = PASS
        notes = []
        rise = None
        if t is not None and self.start_temp is not None:
            rise = t - self.start_temp
            if rise >= Limits.temp_rise_warn:
                status = WARN
                notes.append(f"{rise:.0f} degC rise over the test - more heat than an "
                             "unloaded motor should make")
        if flags:
            status = FAIL
            notes.append("error flags at end of test: " + ", ".join(flags))
        latched = decode_status(self.bus.last_error_byte)
        if latched:
            status = FAIL
            notes.append("flags seen at any point during the run: " + ", ".join(latched))
        return status, {
            "temperature start/end": (f"{self.start_temp:.0f} / {t} degC"
                                      if (t is not None and self.start_temp is not None)
                                      else str(t)),
            "rise": f"{rise:.0f} degC" if rise is not None else "n/a",
            "voltage at end": f"{v:.1f} V" if v is not None else "n/a",
            "status flags": ", ".join(flags) or "none",
        }, notes

    # ----------------------------------------------------------------------
    def save_state(self):
        for key, addr, size in (("torque", ADDR_TORQUE_ENABLE, 1),
                                ("acc", ADDR_ACC, 1),
                                ("goal_speed", ADDR_GOAL_SPEED, 2),
                                ("goal_position", ADDR_GOAL_POSITION, 2),
                                ("mode", ADDR_OPERATING_MODE, 1),
                                ("lock", ADDR_LOCK, 1)):
            self.saved[key] = (self.bus.read1(self.sid, addr) if size == 1
                               else self.bus.read2(self.sid, addr))

    def restore_state(self):
        # mode first: everything else means something different in wheel mode
        mode = self.saved.get("mode")
        if mode is not None:
            current = self.bus.read1(self.sid, ADDR_OPERATING_MODE)
            if current != mode:
                self.bus.write2(self.sid, ADDR_GOAL_SPEED, 0)
                self.bus.write1(self.sid, ADDR_TORQUE_ENABLE, 0)
                if not self.set_mode(mode):
                    print(f"  !! operating mode is {current}, not the original "
                          f"{mode} - check register 33 before using this servo")
        if self.saved.get("acc") is not None:
            self.bus.write1(self.sid, ADDR_ACC, self.saved["acc"])
        if self.saved.get("goal_speed") is not None:
            self.bus.write2(self.sid, ADDR_GOAL_SPEED, self.saved["goal_speed"])
        if self.args.park is not None:
            self.bus.write2(self.sid, ADDR_GOAL_POSITION, self.args.park)
            time.sleep(0.8)
        self.bus.write1(self.sid, ADDR_TORQUE_ENABLE,
                        0 if not self.args.keep_torque else 1)

    def execute(self):
        print(f"\n{'=' * 72}\nTesting servo ID {self.sid}\n{'=' * 72}")
        self.run("Identity and EEPROM configuration", self.t_identity)
        self.run("Communications reliability", self.t_comms)
        self.run("Power and idle electrical check", self.t_power_idle)
        self.run("Encoder stability (torque off)", self.t_encoder_static)
        if self.args.manual:
            self.run("Manual hand-turn encoder sweep", self.t_manual_sweep)
        if self.args.no_motion:
            print("\n(motion tests skipped: --no-motion)")
        else:
            self.save_state()
            self.range = self.safe_range()
            print(f"\nMotion tests will use {self.range[0]}..{self.range[1]} counts "
                  f"({(self.range[1] - self.range[0]) * DEG_PER_COUNT:.0f} deg of travel)")
            try:
                self.run("Torque enable and position hold", self.t_torque_hold)
                self.run("Step response", self.t_step_response)
                self.run("Bidirectional sweep (friction, dead spots, backlash)",
                         self.t_sweep)
                self.run("Speed capability", self.t_speed)
                if not self.args.no_wheel:
                    self.run("Constant-speed mode (full revolutions, both directions)",
                             self.t_continuous)
            finally:
                self.restore_state()
        self.run("Thermal and latched error flags", self.t_thermal)
        return self.summary()

    def summary(self):
        overall = PASS
        for r in self.results:
            if RANK[r["status"]] > RANK[overall]:
                overall = r["status"]
        totals = self.bus.c.snapshot()
        bad = (totals["timeout"] + totals["corrupt"] + totals["rx_fail"]
               + totals["tx_fail"] + totals["other"])
        rate = bad / totals["attempts"] if totals["attempts"] else 0
        return {
            "servo_id": self.sid,
            "port": self.args.port,
            "baud": self.args.baud,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "overall": overall,
            "config": self.config,
            "tests": self.results,
            "comms_total": {
                "transactions": totals["attempts"],
                "failed": bad,
                "failure_rate_pct": round(rate * 100, 3),
                "timeouts": totals["timeout"],
                "corrupt_packets": totals["corrupt"],
                "rx_fail": totals["rx_fail"],
                "tx_fail": totals["tx_fail"],
                "other": totals["other"],
                "hw_error_replies": totals["hw_error_frames"],
                "retried_ops": totals["retried_ops"],
                "failed_ops": totals["failed_ops"],
                "latency_ms_avg": (round(statistics.mean(totals["latency_ms"]), 3)
                                   if totals["latency_ms"] else None),
                "latency_ms_max": (round(max(totals["latency_ms"]), 3)
                                   if totals["latency_ms"] else None),
            },
            "latched_error_flags": decode_status(self.bus.last_error_byte),
        }


# --------------------------------------------------------------------------
def scan(bus, first, last):
    print(f"Scanning IDs {first}..{last} on {bus.port.getPortName()} "
          f"at {bus.port.getBaudRate()} baud")
    found = []
    for sid in range(first, last + 1):
        model = bus.ping(sid, retries=0)
        if model is not None:
            found.append((sid, model))
            print(f"\n  found ID {sid} (model {model})")
        elif sid % 8 == 0:
            print(".", end="", flush=True)
    print()
    return found


def print_report(report):
    print(f"\n{'=' * 72}")
    print(f"SUMMARY - servo ID {report['servo_id']}: {report['overall']}")
    print(f"{'=' * 72}")
    for r in report["tests"]:
        c = r["comms"]
        bad = c["timeout"] + c["corrupt"] + c["rx_fail"] + c["tx_fail"] + c["other"]
        print(f"  {r['status']:4}  {r['test']:<50} {bad}/{c['attempts']} comms failures")
        for n in r["notes"]:
            print(f"          -> {n}")
    t = report["comms_total"]
    print(f"\n  Bus totals: {t['transactions']} transactions, {t['failed']} failures "
          f"({t['failure_rate_pct']}%)")
    print(f"    timeouts {t['timeouts']}, corrupt {t['corrupt_packets']}, "
          f"rx_fail {t['rx_fail']}, tx_fail {t['tx_fail']}, other {t['other']}")
    print(f"    replies carrying a hardware error flag: {t['hw_error_replies']}")
    print(f"    operations needing a retry: {t['retried_ops']}, "
          f"operations that failed outright: {t['failed_ops']}")
    if t["latency_ms_avg"] is not None:
        print(f"    latency avg {t['latency_ms_avg']} ms, max {t['latency_ms_max']} ms")
    if report["latched_error_flags"]:
        print(f"    latched servo error flags: {', '.join(report['latched_error_flags'])}")
    print()


def write_reports(report, outdir):
    os.makedirs(outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(outdir, f"ST3215_id{report['servo_id']}_{stamp}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    # one row per motor so a batch can be compared side by side
    csv_path = os.path.join(outdir, "summary.csv")
    t = report["comms_total"]
    row = {
        "timestamp": stamp,
        "servo_id": report["servo_id"],
        "overall": report["overall"],
        "firmware": f"{report['config'].get('fw_major')}.{report['config'].get('fw_minor')}",
        "transactions": t["transactions"],
        "comm_failures": t["failed"],
        "comm_failure_pct": t["failure_rate_pct"],
        "hw_error_replies": t["hw_error_replies"],
        "latched_flags": "|".join(report["latched_error_flags"]),
    }
    for r in report["tests"]:
        row[r["test"]] = r["status"]
    # the set of tests can change between runs (e.g. --no-wheel), so merge the
    # columns and rewrite rather than appending a row that silently loses fields
    existing = []
    fields = []
    if os.path.exists(csv_path):
        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            fields = list(reader.fieldnames or [])
            # a row written by an older version may carry values past the
            # header; csv parks those under a None key, which cannot be written
            existing = [{k: v for k, v in r.items() if k is not None}
                        for r in reader]
    for k in row:
        if k not in fields:
            fields.append(k)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, restval="")
        w.writeheader()
        for old in existing:
            w.writerow(old)
        w.writerow(row)
    return path, csv_path


def main():
    p = argparse.ArgumentParser(description="Scan and health-test ST3215 servos")
    p.add_argument("--port", default="COM16")
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--id", type=int, default=None,
                   help="test this ID only, skipping the bus scan")
    p.add_argument("--scan-first", type=int, default=0)
    p.add_argument("--scan-last", type=int, default=20,
                   help="use 252 for a full sweep of the address space")
    p.add_argument("--scan-only", action="store_true")
    p.add_argument("--no-motion", action="store_true",
                   help="comms/electrical checks only - the motor will not move")
    p.add_argument("--manual", action="store_true",
                   help="add the hand-turn encoder sweep (needs an operator)")
    p.add_argument("--manual-seconds", type=float, default=15.0)
    p.add_argument("--retries", type=int, default=2,
                   help="retries per transaction before an operation is failed")
    p.add_argument("--comm-samples", type=int, default=200,
                   help="ping+read pairs in the reliability test")
    p.add_argument("--encoder-samples", type=int, default=200)
    p.add_argument("--hold-seconds", type=float, default=3.0)
    p.add_argument("--centre", type=int, default=None,
                   help="centre of the motion test span, in counts (default: the "
                        "midpoint of the servo's angle limits)")
    p.add_argument("--span", type=int, default=2000,
                   help="total motion test span in counts (4096 = one turn)")
    p.add_argument("--steps", type=int, nargs="+", default=[50, 200, 800])
    p.add_argument("--sweep-points", type=int, default=17)
    p.add_argument("--speed", type=int, default=2000, help="counts/s for general moves")
    p.add_argument("--sweep-speed", type=int, default=1200)
    p.add_argument("--speeds", type=int, nargs="+", default=[500, 1500, 3000])
    p.add_argument("--acc", type=int, default=50)
    p.add_argument("--turns", type=float, default=2.0,
                   help="revolutions per direction in the constant-speed test")
    p.add_argument("--wheel-speed", type=int, default=1500,
                   help="counts/s for the constant-speed test (4096 = one turn)")
    p.add_argument("--max-wheel-speed", type=int, default=3000,
                   help="counts/s for the top-speed pass of the constant-speed test")
    p.add_argument("--no-wheel", action="store_true",
                   help="skip the constant-speed test, so the operating mode "
                        "register is never written")
    p.add_argument("--park", type=int, default=None,
                   help="drive to this position before finishing")
    p.add_argument("--keep-torque", action="store_true",
                   help="leave torque enabled when the test finishes")
    p.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "reports"))
    p.add_argument("--verbose", action="store_true",
                   help="print every failed transaction")
    p.add_argument("--yes", action="store_true", help="skip the safety prompt")
    args = p.parse_args()

    bus = Bus(args.port, args.baud, retries=args.retries, verbose=args.verbose)
    try:
        if args.id is not None:
            model = bus.ping(args.id)
            if model is None:
                print(f"No response from ID {args.id} on {args.port}. "
                      "Check power, wiring and baud rate.")
                return 2
            targets = [(args.id, model)]
            print(f"Found ID {args.id} (model {model})")
        else:
            targets = scan(bus, args.scan_first, args.scan_last)
            if not targets:
                print("No servos responded. Check the port, the 1 Mbps baud rate, "
                      "power, and the data line.")
                return 2
            print(f"Found {len(targets)} servo(s): "
                  + ", ".join(str(s) for s, _ in targets))

        if args.scan_only:
            return 0

        if not args.no_motion and not args.yes:
            extra = ("" if args.no_wheel else
                     f" and then spin continuously through {args.turns} full "
                     "revolutions each way")
            ans = input(f"\nThe motor WILL move through its test span{extra}. "
                        "Is it clear and free to rotate? [y/N] ").strip().lower()
            if ans != "y":
                print("Aborted. Use --no-motion for a static test.")
                return 1

        exit_code = 0
        for sid, _model in targets:
            bus.c = CommCounters()          # per-motor counters
            bus.last_error_byte = 0
            tester = Tester(bus, sid, args)
            try:
                report = tester.execute()
            except KeyboardInterrupt:
                print("\nInterrupted - stopping and disabling torque.")
                bus.write2(sid, ADDR_GOAL_SPEED, 0)
                bus.write1(sid, ADDR_TORQUE_ENABLE, 0)
                raise
            print_report(report)
            jpath, cpath = write_reports(report, args.outdir)
            print(f"  report: {jpath}\n  batch summary: {cpath}\n")
            if report["overall"] == FAIL:
                exit_code = 3
            elif report["overall"] == WARN and exit_code == 0:
                exit_code = 4
        return exit_code
    finally:
        bus.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
