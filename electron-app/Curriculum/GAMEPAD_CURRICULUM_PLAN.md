# Gamepad Control page: curriculum plan (not started)

Ideas for using the Gamepad Control page (`electron-app/gamepadPage.js`) to close gaps listed in
`btec-gap-analysis.html`. Nothing here has been built or written yet.

Unit 4068 LO3 names "joysticks" under online programming, but the curriculum mentions a joystick only
once, in passing, in Module 4 (`module-04.html`, teach pendant section).

## What the page does today

- Left stick moves X/Y, right stick up/down moves Z, right stick left/right rotates the tool (joint 6).
- Velocity control at about 33 Hz using local IK, sent as `moveJointsBatch`.
- Shows "Reach limit" and "Tool rotation limit" notes when motion is blocked.
- With Tool Down/Up orientation locked, the spin setting follows J6 (`syncSpinToJ6`).

## Ideas, highest assessment value first

| # | Idea | Gaps it addresses | Work needed |
|---|---|---|---|
| 1 | **Three-position enabling switch.** The arm moves only while an analogue trigger is held in its middle band. Releasing it or squeezing hard stops motion, as on an industrial pendant's enabling device. | Dead man's handle / enabling device: Unit 56 A1, Robotic Ops U1, gap-analysis priority 08 | App change. The trigger currently used by the gripper moves to buttons. |
| 2 | **Operating-mode selector.** Manual reduced speed (hard cap, e.g. 250 mm/s as in T1) vs manual full speed, with a note of what each mode allows and requires (enabling switch always on in manual). | Operating modes (auto / manual reduced / teach): 4068 LO2, 4015 LO4, Robotic Ops U1 | App change plus written section |
| 3 | **Jog-frame selector.** World, Tool and Joint jogging. In Tool frame the sticks move along the tool's own axes. Joint mode puts one joint per stick axis. Wrist joints labelled roll / pitch / roll. | Reference frames as a practical skill: Unit 56 B2, Robotic Ops U2 (user frames are a listed gap). Wrist yaw/pitch/roll vocabulary: Unit 56 B2 gap | App change plus written section |
| 4 | **"Rotate about the tip" mode plus a four-point TCP calibration activity.** Reorient the tool while the tip stays on a fixed pointer, then use four approaches to the same point to work out the TCP offset. | TCP calibration, the gap analysis's "highest-value addition": Unit 56 C2, Robotic Ops U2, priority 06 | App change plus activity |
| 5 | **Singularity warning while driving.** Show when the wrist nears a singularity (J5 near 0°) or the arm nears full reach. | Singularities: Unit 56 D2. Also supports the objectives document's claim that Module 7 covers singularities, which the gap analysis says is currently unsupported | Small app change plus written section |
| 6 | **Record a point from a controller button**, then play the points back (store into `positionsManager` slots). | Online programming with joysticks, teach-and-playback: 4068 LO3 | Small app change |
| 7 | **Written content only:** teleoperation in hazardous environments (nuclear, bomb disposal, subsea), and comparing pendant, tablet and gamepad as operator interfaces. | 4068 LO1 applications; evaluating interface effectiveness (Module 4) | Writing only |

## Where the written material would go

- **Module 4 (Basic Software & HMI):** new section on the gamepad as an operator interface, the
  enabling switch, operating modes, and an interface-comparison worksheet (ideas 1, 2, 6, 7).
- **Module 7 (FK & Coordinate Systems):** jog frames, wrist roll/pitch/roll, singularities, TCP
  (ideas 3, 4, 5).
- **Module 14 (Safety Standards):** enabling devices and operating modes, linking to ISO 10218
  (ideas 1, 2).
- **Module 15 (Maintenance & Calibration):** four-point TCP calibration activity (idea 4).

## Suggested first batch

Ideas 1, 2 and 3, plus a new Module 4 section with a worksheet. Together they close the most items
named in more than one spec.

## When doing the work

- Update the status of each closed item in `btec-gap-analysis.html` and the module mapping in
  `btec-objectives.html`.
- Both of those files had uncommitted local edits when this plan was written; check `git status`
  before committing so those edits aren't swept in by accident.
- Test the enabling switch on the real arm with a range of controllers: analogue trigger ranges and
  rest values differ between Xbox, PlayStation and generic pads.
