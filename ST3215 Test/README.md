# ST3215 servo health test

Screens ST3215 servos on COM16 (1 Mbps) and flags units that are electrically or
mechanically damaged. Built on the official Feetech SC/ST SDK
(`feetech-servo-sdk`, imported as `scservo_sdk`), so all framing, checksums and
timeouts come from the vendor library rather than hand-rolled packet code.

## Install

```
pip install -r requirements.txt
```

## Run

```
python st3215_test.py                 # scan IDs 0-20 on COM16, full test of each
python st3215_test.py --id 1          # skip the scan, test ID 1
python st3215_test.py --scan-only     # just list what answers on the bus
python st3215_test.py --no-motion     # comms + electrical only, motor stays still
python st3215_test.py --manual        # adds the hand-turn encoder sweep
python st3215_test.py --scan-last 252 # full address-space scan
```

The motor **will move**, including several continuous full revolutions, unless
`--no-motion` is given; the script asks for confirmation first (`--yes` skips the
prompt). Torque is left disabled when the test finishes, and the SRAM settings it
touched (acceleration, goal speed) are restored.

The one EEPROM write is the operating-mode register (33), which has to be set to
1 for the constant-speed test and is put back to 0 afterwards — unlocked via
register 55, written, verified by read-back, then re-locked. Restoring it also
happens if the run is interrupted or a test throws. If you would rather the
EEPROM was never touched at all, use `--no-wheel`, at the cost of losing the only
test that covers a full turn. Expect the EEPROM commit to cost roughly one
timeout (about 34 ms) per run; it is retried automatically and counted honestly
in that test's comms figures.

Exit codes: `0` pass, `4` warn, `3` fail, `2` nothing found, `1` aborted.

## What each test looks for

| Test | Detects |
|---|---|
| Identity / EEPROM | wrong firmware or model, corrupted config, non-position operating mode, odd PID or deadband, position offset that has drifted |
| Communications reliability | 200 ping + 200 block-read pairs; timeouts, corrupt frames, latency outliers — bad cable, crimp, ground, or a failing servo UART |
| Power and idle electrical | supply voltage and sag, starting temperature, current with torque off, latched error flags |
| Encoder stability (torque off) | magnetic encoder noise and jumps while stationary — damaged sensor, cracked or shifted magnet, loose output shaft |
| Manual hand-turn sweep (`--manual`) | dead zones and dropouts across the full mechanical travel, seized shaft |
| Torque enable and hold | won't reach position (jammed or stripped gearbox, dead driver), holding current far above a healthy unloaded motor, error flags |
| Step response | ±50/200/800 count steps both ways: settle time, steady-state error, overshoot, peak current, and "commanded 800 counts, moved 100" gear slip |
| Bidirectional sweep | positioning error across the range, points that never settle (dead spots), backlash/hysteresis between directions, running current, and current spikes at specific angles = a localised tight spot |
| Speed capability | cruise (plateau) speed vs commanded, with the goal-speed register read back afterwards — weak motor, low supply, heavy drag. Judged on cruise speed rather than the average over the move, which is acceleration-limited. Any commanded speed whose ramp will not fit inside the test span is skipped with a note rather than reported as a bad result; it is covered by the constant-speed test instead |
| Constant-speed mode | top speed, measured over continuous rotation where there is no distance to run out of (the reference motor holds a full 3000 counts/s at 123 mA), plus 2 continuous revolutions in **each** direction, so the whole 360° of gearbox and encoder is exercised, not just the position-test span. Splits the circle into 24 sectors of 15° and compares speed and current sector by sector: a tight spot, a damaged tooth or a dead encoder zone shows up as a current peak or a speed dip at one specific angle. Also checks encoder continuity across the 0/4095 wrap, stalls, and forward-vs-reverse current asymmetry (a chipped tooth face or a side-loaded bearing is usually worse one way) |
| Thermal and latched flags | temperature rise across the run, plus every error flag seen at any point |

## Reports

Every test prints its own comms counters (transactions, timeouts, corrupt
packets, retried operations, failed operations, latency). A per-test and bus-wide
summary is printed at the end, plus:

- `reports/ST3215_id<N>_<timestamp>.json` — everything, including per-step and
  per-sweep-point raw data
- `reports/summary.csv` — one row per motor tested, appended, so a batch can be
  compared side by side

A test is escalated to WARN at a 0.5 % comms failure rate and to FAIL at 2 %,
independently of its own measurements.

## Calibrating the thresholds

The pass/warn/fail limits live in the `Limits` class at the top of
`st3215_test.py` and are screening heuristics for a 12 V ST3215 running
**unloaded** on the bench. Run two or three motors you trust first, look at
`summary.csv` and the JSON detail, then tighten the limits to match your own
batch. The ones most worth adjusting:

- `idle_hold_current_warn_ma` / `_fail_ma` — holding current, the clearest single
  indicator of a dragging gearbox or a shorted winding
- `move_current_warn_ma` / `_fail_ma` — running current while sweeping
- `wheel_current_warn_ma` / `_fail_ma` — current spinning free in constant-speed
  mode; on the reference motor here it sits at 44-47 mA, and the per-sector
  figures in the JSON are the best evidence of a localised mechanical fault
- `hysteresis_warn` / `_fail` — backlash, in counts (4096 counts = 360 deg)
- `encoder_jitter_warn` / `_fail` — a good unit reads 0-1 counts peak-to-peak

## Useful options

| Option | Purpose |
|---|---|
| `--span 2000` | total motion test span in counts (4096 = one turn) |
| `--centre 2048` | centre the motion span somewhere other than the midpoint of the servo's angle limits |
| `--steps 50 200 800` | step sizes for the step-response test |
| `--sweep-points 17` | resolution of the bidirectional sweep |
| `--speeds 500 1500 3000` | commanded speeds for the speed test |
| `--turns 2` | revolutions per direction in the constant-speed test |
| `--wheel-speed 1500` | counts/s for the constant-speed test (4096 = one turn) |
| `--max-wheel-speed 3000` | counts/s for its top-speed pass |
| `--no-wheel` | skip the constant-speed test, leaving EEPROM entirely untouched |
| `--retries 2` | retries per transaction before an operation counts as failed |
| `--verbose` | print every failed transaction as it happens |
| `--park 2048` | drive to a known position before finishing |

## Notes

- The bus runs at 1 Mbps; only one motor should be connected when screening, so
  a failure is unambiguously that motor's.
- Present current is read from register 69 and scaled at 6.5 mA/LSB. If your
  firmware scales it differently the absolute mA figures shift, but the
  comparison between motors still holds.
- Test one motor at a time and keep `summary.csv` — the fastest way to spot a bad
  unit is the column that differs from the rest of the batch.
