/**
 * Gamepad Control page.
 *
 * Drives the tool tip in world XYZ from a game controller (USB or Bluetooth,
 * read with the browser Gamepad API) or from the on-screen controller:
 *
 *   Left stick   X (right = X+) and Y (up = Y+)
 *   Right stick  Z (up = Z+)
 *   L1           gripper fully open  + pneumatics (pump and valve) off
 *   R1           gripper fully closed + pneumatics on
 *   L2 / R2      open / close the gripper smoothly - the harder the press, the faster
 *
 * When a controller is in use the on-screen controller mirrors it.
 *
 * Motion is velocity control. A ~33 Hz loop turns the stick position into a
 * smoothed tool velocity, steps a commanded XYZ target, solves IK locally
 * (warm-started from the previous solution, so it converges in a few
 * iterations) and sends every joint's goal in one moveJointsBatch message.
 * Each goal is projected a little ahead along the joint's current velocity
 * and the joint is told to move at that velocity (plus a catch-up term if it
 * has fallen behind), so a servo never reaches its goal and pauses while the
 * stick is held - motion is continuous, not stop-start. When the tip stops,
 * one last goal with no projection settles every joint exactly on the target.
 * The Pi keeps only the newest pending batch, so a slow bus can lag a little
 * but never builds a backlog.
 *
 * Uses globals from app.js: robotArmClient, robotKinematics, lastGoodJointStatus,
 * currentToolOrientation, deadZones, pointInAnyZone, jogCommandedPose,
 * lastEndToolState, setVacuum, showAppMessage, stopAllJoints, pendantSetOrientation.
 */

(function () {
    'use strict';

    // ---- Tuning ---------------------------------------------------------
    const MOTION_PERIOD_MS = 30;        // control loop period (~33 Hz)
    const STICK_DEADZONE = 0.12;        // ignore stick drift below this
    const STICK_EXPO = 0.6;             // 0 = linear, 1 = fully cubic (finer control near centre)
    const ACCEL_MM_S2 = 500;            // how fast the tip speeds up
    const DECEL_MM_S2 = 1500;           // how fast it stops (quicker than speeding up)
    const LEAD_S = 0.08;                // project each goal this far ahead so servos never arrive and pause
    const MAX_LEAD_DEG = 4;             // cap on that projection per joint
    const SPEED_MARGIN = 1.05;          // joint speed vs. its commanded velocity
    const HORIZON_S = 0.11;             // a joint is sped up to reach its goal in this time if it is behind
    const MIN_JOINT_DEG_S = 3;
    const MAX_JOINT_STEPS_S = 3400;     // ST3215 limit
    const MAX_JOINT_JUMP_DEG = 8;       // bigger jump in one tick = IK switched branch, reject it
    const MAX_TIP_ERROR_MM = 2;         // IK answers further off than this count as unreachable
    const ALIGN_DEG_S = 40;             // joint speed when first aligning the tool to a locked orientation
    const ALIGN_MIN_DEG = 1.5;          // smaller misalignment than this is ignored
    const MIN_SEND_DELTA_DEG = 0.02;
    const RESYNC_AFTER_IDLE_MS = 250;   // re-read the real arm pose when movement restarts after this
    const GRIP_RATE_DEG_S = 150;        // L2/R2 full press
    const GRIP_SEND_PERIOD_MS = 50;
    const GRIP_OPEN = (typeof GRIPPER_OPEN_ANGLE === 'number') ? GRIPPER_OPEN_ANGLE : 180;
    const GRIP_CLOSED = (typeof GRIPPER_CLOSED_ANGLE === 'number') ? GRIPPER_CLOSED_ANGLE : 0;
    const TRIGGER_THRESHOLD = 0.04;
    const SPEED_STORAGE_KEY = 'robotArmGamepadSpeed';
    const PAD_STORAGE_KEY = 'robotArmGamepadId';

    // Standard Gamepad mapping indices
    const BTN_L1 = 4, BTN_R1 = 5, BTN_L2 = 6, BTN_R2 = 7;
    const AXIS_LX = 0, AXIS_LY = 1, AXIS_RX = 2, AXIS_RY = 3;

    // ---- State ----------------------------------------------------------
    let active = false;
    let rafId = null;
    let lastFrameAt = 0;
    let lastMotionAt = 0;
    let lastMovingAt = 0;
    let maxSpeed = 50;                  // mm/s at full stick

    let selectedPadIndex = null;        // Gamepad API index in use
    let preferredPadId = null;          // remembered across sessions

    const screenInput = { lx: 0, ly: 0, rx: 0, ry: 0, l1: false, r1: false, l2: 0, r2: 0 };
    const screenActive = { left: false, right: false, l2: false, r2: false };
    let prevButtons = { l1: false, r1: false };

    let velocity = { x: 0, y: 0, z: 0 };
    let pose = null;                    // commanded tip position {x,y,z}
    let angles = null;                  // commanded joint angles (deg)
    let limitNote = '';
    // Direction (+1/-1) each axis last hit a limit in. That direction is not
    // retried until the stick on that axis is released or reversed, so a
    // failing (slow) IK solve is not repeated every tick against a wall.
    let blocked = { x: 0, y: 0, z: 0 };
    let alignUntil = 0;

    let streaming = false;              // goals with projection have been sent; a settle goal is owed
    let initialized = false;

    let gripAngle = null;               // last commanded gripper angle
    let gripSentAngle = null;
    let gripLastSendAt = 0;
    let pneumaticsOn = false;

    let ikTimeAvgMs = 0;
    let els = {};

    // ---- Helpers --------------------------------------------------------
    function $(id) { return document.getElementById(id); }

    function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

    function connected() { return typeof robotArmClient !== 'undefined' && robotArmClient && robotArmClient.isConnected; }
    function hasControl() { return connected() && !!robotArmClient.hasArmControl; }
    function kinematicsReady() {
        return typeof robotKinematics !== 'undefined' && robotKinematics && robotKinematics.isConfigured && robotKinematics.isConfigured();
    }

    function degPerSecToSteps(dps) {
        if (typeof degreesPerSecondToStepsPerSecond === 'function') return degreesPerSecondToStepsPerSecond(dps);
        return dps * 4096 / 360;
    }

    /** Same rules as app.js applyEndToolControlVisibility(). */
    function endToolAllows(control) {
        const s = (typeof lastEndToolState !== 'undefined') ? lastEndToolState : null;
        if (!s) return true;
        if (!s.present) return false;
        if (!s.known || !s.tool || !s.tool.controls) return true;
        return s.tool.controls.indexOf(control) >= 0;
    }

    function actualJointAngles() {
        if (typeof lastGoodJointStatus === 'undefined' || !Array.isArray(lastGoodJointStatus) || !lastGoodJointStatus.length) return null;
        const a = lastGoodJointStatus.map(j => (j && typeof j.angleDegrees === 'number' && isFinite(j.angleDegrees)) ? j.angleDegrees : NaN);
        return a.some(v => !isFinite(v)) ? null : a;
    }

    /** Radial dead zone + expo so small stick movements give fine control. */
    function shapeStick2(x, y) {
        const m = Math.hypot(x, y);
        if (m < STICK_DEADZONE) return { x: 0, y: 0 };
        const n = Math.min(1, (m - STICK_DEADZONE) / (1 - STICK_DEADZONE));
        const curved = (1 - STICK_EXPO) * n + STICK_EXPO * n * n * n;
        return { x: x / m * curved, y: y / m * curved };
    }
    function shapeAxis(v) {
        return shapeStick2(v, 0).x;
    }

    function approach(current, target, dt) {
        // Speeding up (same sign, bigger) uses ACCEL; slowing or reversing uses DECEL.
        const rate = (Math.abs(target) > Math.abs(current) && Math.sign(target) === Math.sign(current || target)) ? ACCEL_MM_S2 : DECEL_MM_S2;
        const step = rate * dt;
        if (Math.abs(target - current) <= step) return target;
        return current + Math.sign(target - current) * step;
    }

    // ---- Gamepad input --------------------------------------------------
    function listPads() {
        const pads = (navigator.getGamepads ? navigator.getGamepads() : []) || [];
        return Array.from(pads).filter(p => p && p.connected);
    }

    function currentPad() {
        if (selectedPadIndex === null) return null;
        const pads = navigator.getGamepads ? navigator.getGamepads() : [];
        const p = pads && pads[selectedPadIndex];
        return (p && p.connected) ? p : null;
    }

    function readPad(pad) {
        const ax = (i) => (pad.axes && typeof pad.axes[i] === 'number') ? pad.axes[i] : 0;
        const btn = (i) => pad.buttons && pad.buttons[i];
        const pressed = (i) => { const b = btn(i); return !!(b && (b.pressed || b.value > 0.5)); };
        const value = (i) => { const b = btn(i); return b ? (typeof b.value === 'number' ? b.value : (b.pressed ? 1 : 0)) : 0; };
        return {
            lx: ax(AXIS_LX), ly: ax(AXIS_LY), rx: ax(AXIS_RX), ry: ax(AXIS_RY),
            l1: pressed(BTN_L1), r1: pressed(BTN_R1),
            l2: value(BTN_L2), r2: value(BTN_R2)
        };
    }

    /** Merges controller and on-screen input; whatever the user is touching on screen wins. */
    function readInput() {
        const pad = currentPad();
        const p = pad ? readPad(pad) : { lx: 0, ly: 0, rx: 0, ry: 0, l1: false, r1: false, l2: 0, r2: 0 };
        return {
            lx: screenActive.left ? screenInput.lx : p.lx,
            ly: screenActive.left ? screenInput.ly : p.ly,
            rx: screenActive.right ? screenInput.rx : p.rx,
            ry: screenActive.right ? screenInput.ry : p.ry,
            l1: p.l1 || screenInput.l1,
            r1: p.r1 || screenInput.r1,
            l2: screenActive.l2 ? screenInput.l2 : p.l2,
            r2: screenActive.r2 ? screenInput.r2 : p.r2
        };
    }

    function selectPad(index) {
        selectedPadIndex = index;
        updateSourceLabel();
        const pad = currentPad();
        if (pad) {
            preferredPadId = pad.id;
            try { localStorage.setItem(PAD_STORAGE_KEY, pad.id); } catch (e) { /* storage unavailable */ }
        }
        renderPadList();
    }

    function autoSelectPad() {
        if (currentPad()) return;
        const pads = listPads();
        if (!pads.length) { selectedPadIndex = null; return; }
        const preferred = pads.find(p => p.id === preferredPadId);
        selectPad((preferred || pads[0]).index);
    }

    function padLabel(pad) {
        // Chrome ids look like "Xbox Wireless Controller (STANDARD GAMEPAD Vendor: 045e Product: 0b13)"
        const name = pad.id.replace(/\s*\(.*\)\s*$/, '').trim() || pad.id;
        return name;
    }

    function renderPadList() {
        if (!els.padList) return;
        const pads = listPads();
        els.padList.innerHTML = '';
        if (!pads.length) {
            const li = document.createElement('div');
            li.className = 'gp-pad-empty';
            li.textContent = 'No controller detected. Plug one in by USB or pair it by Bluetooth, then press any button on it.';
            els.padList.appendChild(li);
        }
        pads.forEach(pad => {
            const row = document.createElement('button');
            row.type = 'button';
            row.className = 'gp-pad-row' + (pad.index === selectedPadIndex ? ' selected' : '');
            const name = document.createElement('span');
            name.className = 'gp-pad-name';
            name.textContent = padLabel(pad);
            const tag = document.createElement('span');
            tag.className = 'gp-pad-tag';
            tag.textContent = pad.index === selectedPadIndex ? 'In use' : 'Use';
            row.appendChild(name);
            row.appendChild(tag);
            if (pad.mapping !== 'standard') {
                const warn = document.createElement('span');
                warn.className = 'gp-pad-warn';
                warn.textContent = 'Non-standard layout - buttons may not match';
                row.appendChild(warn);
            }
            row.addEventListener('click', () => selectPad(pad.index));
            els.padList.appendChild(row);
        });
        updateSourceLabel();
    }

    function updateSourceLabel() {
        if (!els || !els.source) return;
        const pad = currentPad();
        els.source.textContent = pad ? padLabel(pad) : 'On-screen controller';
        els.source.classList.toggle('gp-chip-ok', !!pad);
    }

    function scanForPads() {
        renderPadList();
        if (els.scanHint) {
            els.scanHint.hidden = false;
            els.scanHint.textContent = listPads().length
                ? 'Controllers found. Press a button on another controller to add it.'
                : 'Searching... press any button on your controller. Browsers only show a controller after one of its buttons is pressed.';
        }
    }

    // ---- Bluetooth pairing ---------------------------------------------
    function uiIsOnPi() {
        const addr = (typeof robotArmClient !== 'undefined' && robotArmClient && robotArmClient.piAddress) || '';
        return /^(localhost|127\.0\.0\.1|::1)$/i.test(addr) || window.location.search.indexOf('kiosk=1') >= 0;
    }

    function renderBluetoothPanel() {
        if (!els.btPi || !els.btLocal) return;
        const onPi = uiIsOnPi();
        els.btPi.hidden = !onPi;
        els.btLocal.hidden = onPi;
        if (els.btOpenSettings) {
            els.btOpenSettings.hidden = !(window.electronAPI && typeof window.electronAPI.openBluetoothSettings === 'function');
        }
    }

    async function bluetoothScan() {
        if (!connected()) { showAppMessage('Connect to the robot arm first'); return; }
        els.btScanBtn.disabled = true;
        els.btStatus.textContent = 'Scanning for 8 seconds... put the controller in pairing mode now.';
        els.btList.innerHTML = '';
        try {
            const res = await robotArmClient.bluetoothScan(8000);
            renderBluetoothDevices(res.devices || []);
            els.btStatus.textContent = (res.devices || []).length ? 'Pick your controller and press Pair.' : 'Nothing found. Check the controller is in pairing mode and scan again.';
        } catch (e) {
            els.btStatus.textContent = 'Scan failed: ' + e.message;
        } finally {
            els.btScanBtn.disabled = false;
        }
    }

    function renderBluetoothDevices(devices) {
        els.btList.innerHTML = '';
        devices.forEach(d => {
            const row = document.createElement('div');
            row.className = 'gp-bt-row' + (d.gamepad ? ' gp-bt-gamepad' : '');
            const name = document.createElement('span');
            name.className = 'gp-bt-name';
            name.textContent = d.name + (d.gamepad ? '  🎮' : '');
            const state = document.createElement('span');
            state.className = 'gp-bt-state';
            state.textContent = d.connected ? 'Connected' : (d.paired ? 'Paired' : '');
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'btn ' + (d.connected ? 'btn-secondary' : 'btn-primary') + ' gp-bt-btn';
            btn.textContent = d.connected ? 'Forget' : (d.paired ? 'Connect' : 'Pair');
            btn.addEventListener('click', async () => {
                btn.disabled = true;
                try {
                    if (d.connected) {
                        els.btStatus.textContent = 'Forgetting ' + d.name + '...';
                        await robotArmClient.bluetoothRemove(d.mac);
                        els.btStatus.textContent = d.name + ' removed.';
                        row.remove();
                    } else {
                        els.btStatus.textContent = 'Pairing with ' + d.name + '... keep it in pairing mode.';
                        await robotArmClient.bluetoothPair(d.mac);
                        els.btStatus.textContent = d.name + ' connected. Press any button on it to start.';
                        state.textContent = 'Connected';
                        btn.textContent = 'Forget';
                        btn.className = 'btn btn-secondary gp-bt-btn';
                        d.connected = true;
                    }
                } catch (e) {
                    els.btStatus.textContent = e.message;
                } finally {
                    btn.disabled = false;
                }
            });
            row.appendChild(name);
            row.appendChild(state);
            row.appendChild(btn);
            els.btList.appendChild(row);
        });
    }

    // ---- Motion ---------------------------------------------------------
    /** Starts a movement session from where the arm really is. */
    function syncFromArm(now) {
        const a = actualJointAngles();
        if (!a || !kinematicsReady()) return false;
        blocked = { x: 0, y: 0, z: 0 };
        try {
            const fk = robotKinematics.forwardKinematics(a);
            if (!fk || !fk.position) return false;
            pose = { x: fk.position.x, y: fk.position.y, z: fk.position.z };
            angles = a.slice();
        } catch (e) {
            console.warn('Gamepad: forward kinematics failed:', e);
            return false;
        }

        // With Tool Down / Up locked, the arm may not be pointing that way yet.
        // Turn the tool first with one gentle move, then start streaming;
        // otherwise the first step would be a big jump and be refused.
        const orient = (typeof currentToolOrientation !== 'undefined') ? currentToolOrientation : null;
        if (orient) {
            let aligned = null;
            try { aligned = robotKinematics.inverseKinematics({ x: pose.x, y: pose.y, z: pose.z, orientation: orient }, angles); } catch (e) { aligned = null; }
            if (!aligned) {
                pose = null;
                limitNote = 'The tool cannot point that way here - choose Free or move the arm';
                return false;
            }
            const jump = Math.max(...aligned.map((v, i) => Math.abs(v - angles[i])));
            if (jump > ALIGN_MIN_DEG) {
                const seconds = jump / ALIGN_DEG_S;
                const moves = aligned.map((v, i) => ({
                    joint: i + 1,
                    angle: Math.round(v * 100) / 100,
                    speed: Math.min(MAX_JOINT_STEPS_S, Math.max(30, Math.round(degPerSecToSteps(Math.abs(v - angles[i]) / seconds))))
                }));
                robotArmClient.moveJointsBatch(moves);
                alignUntil = now + seconds * 1000 + 250;
                lastMovingAt = alignUntil;
                limitNote = 'Aligning tool...';
            }
            angles = aligned;
            try {
                const p = robotKinematics.forwardKinematics(aligned).position;
                pose = { x: p.x, y: p.y, z: p.z };
            } catch (e) { /* keep pose */ }
        }
        return true;
    }

    function solve(target) {
        if (typeof deadZones !== 'undefined' && typeof pointInAnyZone === 'function' && Array.isArray(deadZones) && deadZones.length && pointInAnyZone(target, deadZones)) {
            return { ok: false, reason: 'Dead zone' };
        }
        const t0 = performance.now();
        let result = null;
        try {
            const orient = (typeof currentToolOrientation !== 'undefined') ? currentToolOrientation : null;
            result = robotKinematics.inverseKinematics(orient ? { x: target.x, y: target.y, z: target.z, orientation: orient } : target, angles);
        } catch (e) {
            result = null;
        }
        ikTimeAvgMs = ikTimeAvgMs * 0.9 + (performance.now() - t0) * 0.1;
        if (!result) return { ok: false, reason: 'Reach limit' };

        // A large jump means IK flipped to another arm configuration - not smooth, refuse it.
        for (let i = 0; i < result.length; i++) {
            if (Math.abs(result[i] - angles[i]) > MAX_JOINT_JUMP_DEG) return { ok: false, reason: 'Reach limit' };
        }
        // Make sure the solution actually lands on the target.
        try {
            const fk = robotKinematics.forwardKinematics(result);
            if (Math.hypot(fk.position.x - target.x, fk.position.y - target.y, fk.position.z - target.z) > MAX_TIP_ERROR_MM) {
                return { ok: false, reason: 'Reach limit' };
            }
        } catch (e) { /* keep result */ }
        return { ok: true, angles: result };
    }

    /**
     * Moves the target by delta. If the full move is not possible, tries each
     * axis on its own so the tip slides along a limit instead of sticking.
     */
    function stepPose(delta) {
        const full = { x: pose.x + delta.x, y: pose.y + delta.y, z: pose.z + delta.z };
        let r = solve(full);
        if (r.ok) { pose = full; limitNote = ''; return r.angles; }

        let reason = r.reason;
        let moved = null;
        ['x', 'y', 'z'].forEach(axis => {
            if (!delta[axis]) return;
            const t = { x: pose.x, y: pose.y, z: pose.z };
            t[axis] += delta[axis];
            const ra = solve(t);
            if (ra.ok) {
                pose = t;
                angles = ra.angles;
                moved = ra.angles;
            } else {
                velocity[axis] = 0;
                blocked[axis] = Math.sign(delta[axis]);
                reason = ra.reason;
            }
        });
        limitNote = reason;
        return moved;
    }

    function jointLimits(i) {
        const j = robotKinematics.joints && robotKinematics.joints[i];
        const lim = j && j.limits;
        return {
            lo: (lim && typeof lim.lowerDegrees === 'number') ? lim.lowerDegrees : -Infinity,
            hi: (lim && typeof lim.upperDegrees === 'number') ? lim.upperDegrees : Infinity
        };
    }

    /**
     * Sends the new commanded angles. `angles` still holds the previous
     * commanded angles, so (target - angles) / dt is each joint's velocity.
     */
    function sendJoints(target, dt) {
        const actual = actualJointAngles();
        const moves = [];
        let maxDelta = 0;
        for (let i = 0; i < target.length; i++) {
            const vel = (target[i] - angles[i]) / dt;                       // deg/s
            maxDelta = Math.max(maxDelta, Math.abs(target[i] - angles[i]));
            const lead = clamp(vel * LEAD_S, -MAX_LEAD_DEG, MAX_LEAD_DEG);
            const lim = jointLimits(i);
            const goal = clamp(target[i] + lead, lim.lo, lim.hi);
            // At least the commanded velocity, and fast enough to close the gap
            // from where the joint really is within HORIZON_S. Sizing from the
            // remaining distance keeps a steady gap while moving (so the joint
            // never arrives and pauses) and gives a smooth glide when slowing.
            const toGo = actual ? Math.abs(goal - actual[i]) : Math.abs(goal - angles[i]);
            const degPerSec = Math.max(MIN_JOINT_DEG_S, Math.abs(vel) * SPEED_MARGIN, toGo / HORIZON_S);
            moves.push({ joint: i + 1, angle: Math.round(goal * 100) / 100, speed: Math.min(MAX_JOINT_STEPS_S, Math.round(degPerSecToSteps(degPerSec))) });
        }
        angles = target.slice();
        if (maxDelta < MIN_SEND_DELTA_DEG && streaming) return;
        robotArmClient.moveJointsBatch(moves);
        streaming = true;
    }

    /** Tip has stopped: put every joint exactly on the commanded angles. */
    function settleJoints() {
        streaming = false;
        if (!angles || !hasControl()) return;
        const actual = actualJointAngles();
        const moves = angles.map((a, i) => {
            const toGo = actual ? Math.abs(a - actual[i]) : 0;
            const degPerSec = Math.max(MIN_JOINT_DEG_S, toGo / HORIZON_S);
            return { joint: i + 1, angle: Math.round(a * 100) / 100, speed: Math.min(MAX_JOINT_STEPS_S, Math.round(degPerSecToSteps(degPerSec))) };
        });
        robotArmClient.moveJointsBatch(moves);
    }

    function motionTick(input, now) {
        const dt = clamp((now - lastMotionAt) / 1000, 0.005, 0.1);
        lastMotionAt = now;

        const canMove = hasControl() && kinematicsReady();
        const left = shapeStick2(input.lx, input.ly);
        const want = canMove ? {
            x: left.x * maxSpeed,
            y: -left.y * maxSpeed,          // stick up is negative
            z: -shapeAxis(input.ry) * maxSpeed
        } : { x: 0, y: 0, z: 0 };

        // Hold back directions that are against a limit; release them as soon
        // as that stick axis lets go or reverses.
        ['x', 'y', 'z'].forEach(axis => {
            if (!blocked[axis]) return;
            if (Math.sign(want[axis]) === blocked[axis]) want[axis] = 0;
            else blocked[axis] = 0;
        });

        const wantsMotion = want.x || want.y || want.z;
        const isMoving = velocity.x || velocity.y || velocity.z;

        if (!canMove) {
            velocity = { x: 0, y: 0, z: 0 };
            streaming = false;
            pose = null;
            return;
        }

        // Fresh start: re-read the arm, it may have been moved from elsewhere.
        if (now < alignUntil) return;   // tool still turning to the locked orientation
        if (limitNote === 'Aligning tool...') limitNote = '';

        if (wantsMotion && !isMoving && (now - lastMovingAt > RESYNC_AFTER_IDLE_MS || !pose)) {
            limitNote = '';
            if (!syncFromArm(now)) {
                if (!limitNote) limitNote = 'Waiting for arm position';
                return;
            }
            if (now < alignUntil) return;
        }
        if (!pose || !angles) return;

        velocity.x = approach(velocity.x, want.x, dt);
        velocity.y = approach(velocity.y, want.y, dt);
        velocity.z = approach(velocity.z, want.z, dt);

        if (!velocity.x && !velocity.y && !velocity.z) {
            if (streaming) settleJoints();
            if (!wantsMotion && !blocked.x && !blocked.y && !blocked.z) limitNote = '';
            return;
        }
        lastMovingAt = now;
        if (typeof jogCommandedPose !== 'undefined') jogCommandedPose = null; // pendant jog re-syncs after us

        // stepPose may advance `angles` while trying single axes (to seed the
        // next solve); restore it so sendJoints sizes speeds from the last goal sent.
        const before = angles.slice();
        const target = stepPose({ x: velocity.x * dt, y: velocity.y * dt, z: velocity.z * dt });
        angles = before;
        if (target) sendJoints(target, dt);
        else if (streaming) settleJoints();   // every direction blocked: stop exactly where we are
    }

    // ---- Gripper and pneumatics ----------------------------------------
    function sendGrip(angle, force) {
        if (!hasControl() || !endToolAllows('servo')) return;
        const a = Math.round(clamp(angle, Math.min(GRIP_OPEN, GRIP_CLOSED), Math.max(GRIP_OPEN, GRIP_CLOSED)));
        const now = performance.now();
        if (!force && (a === gripSentAngle || now - gripLastSendAt < GRIP_SEND_PERIOD_MS)) return;
        gripSentAngle = a;
        gripLastSendAt = now;
        robotArmClient.sendCommand('toolSetServoEnabledAndAngle', { angle: a });
    }

    function setPneumatics(on) {
        if (!hasControl()) return;
        if (!endToolAllows('pump') && !endToolAllows('solenoid')) return;
        pneumaticsOn = on;
        if (typeof setVacuum === 'function') setVacuum(on);
    }

    function toolTick(input, dt) {
        // L1: release - open fully, pneumatics off. R1: grip - close fully, pneumatics on.
        if (input.l1 && !prevButtons.l1) {
            gripAngle = GRIP_OPEN;
            sendGrip(gripAngle, true);
            setPneumatics(false);
        }
        if (input.r1 && !prevButtons.r1) {
            gripAngle = GRIP_CLOSED;
            sendGrip(gripAngle, true);
            setPneumatics(true);
        }
        prevButtons = { l1: input.l1, r1: input.r1 };

        // L2 opens, R2 closes, rate follows how hard they are pressed.
        const l2 = input.l2 > TRIGGER_THRESHOLD ? input.l2 : 0;
        const r2 = input.r2 > TRIGGER_THRESHOLD ? input.r2 : 0;
        if (l2 || r2) {
            if (gripAngle === null) gripAngle = (GRIP_OPEN + GRIP_CLOSED) / 2;
            const dir = Math.sign(GRIP_OPEN - GRIP_CLOSED);  // +1 when open is the larger angle
            gripAngle += (l2 - r2) * dir * GRIP_RATE_DEG_S * dt;
            gripAngle = clamp(gripAngle, Math.min(GRIP_OPEN, GRIP_CLOSED), Math.max(GRIP_OPEN, GRIP_CLOSED));
            sendGrip(gripAngle, false);
        } else if (gripAngle !== null && Math.round(gripAngle) !== gripSentAngle) {
            sendGrip(gripAngle, true);     // make sure the final position is sent
        }
    }

    function readGripperState() {
        if (!connected() || !endToolAllows('servo')) return;
        robotArmClient.sendRequest('toolGetServoState', {}, 2000)
            .then(resp => {
                if (resp && typeof resp.currentAngle === 'number' && gripAngle === null) {
                    gripAngle = resp.currentAngle;
                    gripSentAngle = Math.round(resp.currentAngle);
                }
            })
            .catch(() => { /* older end tool - start from the middle on first use */ });
    }

    // ---- On-screen controller ------------------------------------------
    function bindStick(el, side) {
        const thumb = el.querySelector('.gp-stick-thumb');
        let pointerId = null;

        const update = (e) => {
            const r = el.getBoundingClientRect();
            const radius = r.width / 2 - thumb.offsetWidth / 4;
            let dx = (e.clientX - (r.left + r.width / 2)) / radius;
            let dy = (e.clientY - (r.top + r.height / 2)) / radius;
            const m = Math.hypot(dx, dy);
            if (m > 1) { dx /= m; dy /= m; }
            if (side === 'left') {
                screenInput.lx = dx; screenInput.ly = dy;
            } else {
                // Right stick only drives Z: keep it vertical like a slider
                screenInput.rx = 0; screenInput.ry = dy;
            }
        };
        el.addEventListener('pointerdown', (e) => {
            if (pointerId !== null) return;
            pointerId = e.pointerId;
            try { el.setPointerCapture(e.pointerId); } catch (err) { /* synthetic or finished pointer */ }
            screenActive[side] = true;
            el.classList.add('dragging');
            update(e);
            e.preventDefault();
        });
        el.addEventListener('pointermove', (e) => { if (e.pointerId === pointerId) update(e); });
        const release = (e) => {
            if (e.pointerId !== pointerId) return;
            pointerId = null;
            screenActive[side] = false;
            el.classList.remove('dragging');
            if (side === 'left') { screenInput.lx = 0; screenInput.ly = 0; } else { screenInput.rx = 0; screenInput.ry = 0; }
        };
        el.addEventListener('pointerup', release);
        el.addEventListener('pointercancel', release);
        el.addEventListener('lostpointercapture', release);
    }

    function bindTrigger(el, key) {
        let pointerId = null;
        const update = (e) => {
            const r = el.getBoundingClientRect();
            // Drag down to press harder (like pulling a trigger)
            screenInput[key] = clamp((e.clientY - r.top) / r.height, 0, 1);
        };
        el.addEventListener('pointerdown', (e) => {
            if (pointerId !== null) return;
            pointerId = e.pointerId;
            try { el.setPointerCapture(e.pointerId); } catch (err) { /* synthetic or finished pointer */ }
            screenActive[key] = true;
            update(e);
            e.preventDefault();
        });
        el.addEventListener('pointermove', (e) => { if (e.pointerId === pointerId) update(e); });
        const release = (e) => {
            if (e.pointerId !== pointerId) return;
            pointerId = null;
            screenActive[key] = false;
            screenInput[key] = 0;
        };
        el.addEventListener('pointerup', release);
        el.addEventListener('pointercancel', release);
        el.addEventListener('lostpointercapture', release);
    }

    function bindBumper(el, key) {
        const down = (e) => { screenInput[key] = true; try { el.setPointerCapture(e.pointerId); } catch (err) { /* synthetic or finished pointer */ } e.preventDefault(); };
        const up = () => { screenInput[key] = false; };
        el.addEventListener('pointerdown', down);
        el.addEventListener('pointerup', up);
        el.addEventListener('pointercancel', up);
        el.addEventListener('lostpointercapture', up);
    }

    // Last values drawn, so the DOM is only touched when something changed.
    const drawn = {};
    function setIfChanged(key, value, apply) {
        if (drawn[key] === value) return;
        drawn[key] = value;
        apply(value);
    }

    function renderController(input) {
        const travel = (els.leftStick.clientWidth / 2) - (els.leftThumb.offsetWidth / 2);
        const lt = `translate(${(input.lx * travel).toFixed(1)}px, ${(input.ly * travel).toFixed(1)}px)`;
        const rt = `translate(${(input.rx * travel).toFixed(1)}px, ${(input.ry * travel).toFixed(1)}px)`;
        setIfChanged('lt', lt, v => { els.leftThumb.style.transform = v; });
        setIfChanged('rt', rt, v => { els.rightThumb.style.transform = v; });
        setIfChanged('l1', input.l1, v => els.l1.classList.toggle('pressed', v));
        setIfChanged('r1', input.r1, v => els.r1.classList.toggle('pressed', v));
        setIfChanged('l2', input.l2.toFixed(2), v => { els.l2Fill.style.transform = `scaleY(${v})`; els.l2.classList.toggle('pressed', input.l2 > TRIGGER_THRESHOLD); });
        setIfChanged('r2', input.r2.toFixed(2), v => { els.r2Fill.style.transform = `scaleY(${v})`; els.r2.classList.toggle('pressed', input.r2 > TRIGGER_THRESHOLD); });
    }

    function renderStatus() {
        const p = pose || (() => {
            const a = actualJointAngles();
            if (!a || !kinematicsReady()) return null;
            try { return robotKinematics.forwardKinematics(a).position; } catch (e) { return null; }
        })();
        setIfChanged('x', p ? p.x.toFixed(1) : '-', v => { els.x.textContent = v; });
        setIfChanged('y', p ? p.y.toFixed(1) : '-', v => { els.y.textContent = v; });
        setIfChanged('z', p ? p.z.toFixed(1) : '-', v => { els.z.textContent = v; });

        const speed = Math.hypot(velocity.x, velocity.y, velocity.z);
        setIfChanged('spd', speed.toFixed(0), v => { els.liveSpeed.textContent = v; });

        let blocker = '';
        if (!connected()) blocker = 'Not connected - use the Connection tab';
        else if (!robotArmClient.hasArmControl) blocker = 'Read-only - press Take control on the Connection tab';
        else if (!kinematicsReady()) blocker = 'Kinematics not loaded';
        setIfChanged('blocker', blocker, v => { els.blocker.textContent = v; els.blocker.hidden = !v; });
        setIfChanged('limit', limitNote, v => { els.limit.textContent = v ? '⚠ ' + v : ''; els.limit.hidden = !v; });

        const orient = (typeof currentToolOrientation !== 'undefined' && currentToolOrientation)
            ? (currentToolOrientation.z < -0.5 ? 'down' : (currentToolOrientation.z > 0.5 ? 'up' : 'free')) : 'free';
        setIfChanged('orient', orient, v => {
            els.orientBtns.forEach(b => b.classList.toggle('active', b.getAttribute('data-orient') === v));
        });
        setIfChanged('pneu', pneumaticsOn, v => { els.pneu.textContent = v ? 'On' : 'Off'; els.pneu.classList.toggle('gp-chip-ok', v); });
        setIfChanged('grip', gripAngle === null ? '-' : Math.round(gripAngle) + '°', v => { els.grip.textContent = v; });
    }

    // ---- Loop -----------------------------------------------------------
    function frame(now) {
        if (!active) return;
        rafId = requestAnimationFrame(frame);

        autoSelectPad();
        const input = readInput();
        const dt = clamp((now - (lastFrameAt || now)) / 1000, 0, 0.1);
        lastFrameAt = now;

        if (now - lastMotionAt >= MOTION_PERIOD_MS - 2) {
            motionTick(input, now);
        }
        if (hasControl()) toolTick(input, dt);

        renderController(input);
        renderStatus();
    }

    function stopMotion() {
        if (streaming && connected()) settleJoints();
        streaming = false;
        velocity = { x: 0, y: 0, z: 0 };
        blocked = { x: 0, y: 0, z: 0 };
        alignUntil = 0;
        pose = null;
        angles = null;
        limitNote = '';
        Object.keys(screenActive).forEach(k => { screenActive[k] = false; });
        Object.assign(screenInput, { lx: 0, ly: 0, rx: 0, ry: 0, l1: false, r1: false, l2: 0, r2: 0 });
    }

    // ---- Public API -----------------------------------------------------
    function init() {
        if (initialized) return;
        if (!document.getElementById('gpLeftStick')) return;
        initialized = true;
        els = {
            leftStick: $('gpLeftStick'), rightStick: $('gpRightStick'),
            leftThumb: document.querySelector('#gpLeftStick .gp-stick-thumb'),
            rightThumb: document.querySelector('#gpRightStick .gp-stick-thumb'),
            l1: $('gpL1'), r1: $('gpR1'), l2: $('gpL2'), r2: $('gpR2'),
            l2Fill: document.querySelector('#gpL2 .gp-trigger-fill'),
            r2Fill: document.querySelector('#gpR2 .gp-trigger-fill'),
            x: $('gpX'), y: $('gpY'), z: $('gpZ'),
            liveSpeed: $('gpLiveSpeed'),
            speed: $('gpMaxSpeed'), speedLabel: $('gpMaxSpeedLabel'),
            blocker: $('gpBlocker'), limit: $('gpLimit'),
            source: $('gpSource'), padList: $('gpPadList'), scanHint: $('gpScanHint'),
            pneu: $('gpPneumatics'), grip: $('gpGrip'),
            orientBtns: Array.from(document.querySelectorAll('#gamepad-tab [data-orient]')),
            btPi: $('gpBtPi'), btLocal: $('gpBtLocal'), btScanBtn: $('gpBtScan'),
            btStatus: $('gpBtStatus'), btList: $('gpBtList'), btOpenSettings: $('gpBtOpenSettings')
        };
        if (!els.leftStick) return;

        bindStick(els.leftStick, 'left');
        bindStick(els.rightStick, 'right');
        bindTrigger(els.l2, 'l2');
        bindTrigger(els.r2, 'r2');
        bindBumper(els.l1, 'l1');
        bindBumper(els.r1, 'r1');

        try {
            const s = parseFloat(localStorage.getItem(SPEED_STORAGE_KEY));
            if (isFinite(s) && s > 0) maxSpeed = s;
            preferredPadId = localStorage.getItem(PAD_STORAGE_KEY);
        } catch (e) { /* storage unavailable */ }
        els.speed.value = maxSpeed;
        els.speedLabel.textContent = maxSpeed;
        els.speed.addEventListener('input', () => {
            maxSpeed = parseFloat(els.speed.value) || 50;
            els.speedLabel.textContent = maxSpeed;
            try { localStorage.setItem(SPEED_STORAGE_KEY, String(maxSpeed)); } catch (e) { /* storage unavailable */ }
        });

        els.orientBtns.forEach(b => b.addEventListener('click', () => {
            if (typeof pendantSetOrientation === 'function') pendantSetOrientation(b.getAttribute('data-orient'));
            pose = null;   // re-solve from the arm with the new orientation
        }));

        $('gpScan').addEventListener('click', scanForPads);
        $('gpEstop').addEventListener('click', () => { stopMotion(); if (typeof stopAllJoints === 'function') stopAllJoints(); });
        if (els.btScanBtn) els.btScanBtn.addEventListener('click', bluetoothScan);
        if (els.btOpenSettings) els.btOpenSettings.addEventListener('click', () => window.electronAPI.openBluetoothSettings());

        window.addEventListener('gamepadconnected', (e) => {
            if (selectedPadIndex === null || e.gamepad.id === preferredPadId) selectPad(e.gamepad.index);
            else renderPadList();
            if (els.scanHint) els.scanHint.hidden = true;
            if (active) showAppMessage('Controller connected: ' + padLabel(e.gamepad));
        });
        window.addEventListener('gamepaddisconnected', (e) => {
            if (e.gamepad.index === selectedPadIndex) {
                selectedPadIndex = null;
                if (active) showAppMessage('Controller disconnected - arm stopped');
            }
            renderPadList();
        });
        // Leaving the window must never leave the arm moving
        document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotion(); });
        window.addEventListener('blur', () => { if (active) stopMotion(); });
    }

    function activate() {
        init();
        if (!initialized || active) return;
        active = true;
        stopMotion();
        lastFrameAt = 0;
        lastMotionAt = performance.now();
        renderPadList();
        renderBluetoothPanel();
        readGripperState();
        rafId = requestAnimationFrame(frame);
    }

    function deactivate() {
        if (!active) return;
        active = false;
        if (rafId) cancelAnimationFrame(rafId);
        rafId = null;
        stopMotion();
    }

    window.gamepadPage = {
        init, activate, deactivate,
        // For diagnostics in DevTools
        stats: () => ({ ikTimeAvgMs: +ikTimeAvgMs.toFixed(2), velocity: { ...velocity }, pose, selectedPadIndex })
    };

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
