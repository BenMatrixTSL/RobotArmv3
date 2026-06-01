# Hobby Servo Firmware — Build Notes

**Master source:** `RobotArmv3_End_Tool_API.fcfx` (Flowcode).  
Rebuilding in Flowcode **overwrites** `RobotArmv3_End_Tool_API.c` and `.h`.

## PWM / pin layout (ESP32 end tool)

| Function | Flowcode component | CAL PWM ref | Hardware ch. | GPIO |
|----------|-------------------|-------------|--------------|------|
| Hobby servo | `HobbyServoPwm` + `Servo_Controller1` | **1** (after rebuild) | 1 | **IO18** @ **50 Hz** |
| Register PWM1 | `PWM1` | **2** | 10 | **IO22** |
| Register PWM2 | `PWM2` | **3** | 11 | **IO23** |

### Critical Flowcode bug (ESP32)

`Servo_Controller1` **always calls** `FC_CAL_PWM_Enable_1` / `SetDutyFloat_1` in generated C, even when Flowcode assigns the servo pin to **REF3** (`MX_PWM_PIN_3 = 18`).

If REF1 is mapped to **A22** (tool `PWM1` component), the bus commands succeed but **no PWM appears on the servo wire**.

**Fix in this repo (two parts):**

1. **`HobbyServoPwm` component** (first in component list) — owns CAL PWM **ref 1** on **A18 @ 50 Hz**, so regenerated `Servo_Controller` `*_1` calls hit the servo pin.
2. **Until you recompile from `.fcfx`**, the patched `.c` / `.h` in git use **`FC_CAL_PWM_*_3`** for all `Servo_Controller1` ESP paths (matching `MX_PWM_REF3`).

## Component order in `.fcfx`

1. `HobbyServoPwm` — pin A18, channel 1, 50 Hz  
2. `Servo_Controller1` — channel 0, pin A18  
3. `PWM1` — channel 10, pin A22  
4. `PWM2` — channel 11, pin A23  

`ApplyPendingHobbyServo` calls `HobbyServoPwm.SetFrequency(50)` before `EnableServo` / `SetPosition`.

## Build steps

1. Open `RobotArmv3_End_Tool_API.fcfx` in Flowcode 11.
2. Confirm `HobbyServoPwm` exists and is **above** `PWM1` / `PWM2` in the project.
3. Build / compile for **ESP32**.
4. Open generated `RobotArmv3_End_Tool_API.h` and confirm:
   - **REF1:** `MX_PWM_PIN_1` = **18**, `MX_PWM_FREQ_1` = **50**
   - **REF2:** `MX_PWM_PIN_2` = **22**, `MX_PWM_CHANNEL_2` = **10**
   - **REF3:** `MX_PWM_PIN_3` = **23**, `MX_PWM_CHANNEL_3` = **11**
5. Flash the ESP32.
6. On the Pi: restart `node server.js`, run the Python example.

## After recompile — if servo still silent

Search generated `RobotArmv3_End_Tool_API.c` for `FCD_0dd21_Servo_Controller1__EnableServo` and check it uses `FC_CAL_PWM_Enable_1` (OK if REF1 is pin 18) or wrongly still targets pin 22.

If Flowcode regenerates `*_1` on the wrong pin again, keep the `*_3` patches in `.c` and the REF3 prototype block in `.h` before `includes.c`.

## Quick test

```bash
cd Firmware/Python
python example_end_tool_servo.py
```

Probe **GPIO 18** for ~50 Hz PWM after enable + set angle.

## Register quick reference

| Address | Use |
|--------:|-----|
| 48 | Enable (`1` = on) |
| 49 | Position 0–255 |
| 50 | Angle 0–180° |

See `REGISTER_MAP.md` for full details.
