'use strict';
/**
 * WS2812B LED status indicator for the robot arm server.
 * Spawns led_driver.py (Python, rpi_ws281x) and communicates via stdin/stdout JSON.
 *
 * States (highest priority first):
 *   error      → solid red    (worker crashed, fatal fault)
 *   torque_off → solid blue   (motors online but torque disabled)
 *   moving     → orange pulse (any joint is moving)
 *   connected  → solid orange (a client holds the control session)
 *   online     → solid green  (motors ready, no active session)
 *   off        → all LEDs off (startup / shutdown)
 */

const { spawn } = require('child_process');
const path       = require('path');

// 0-255 RGB values for each state
const COLORS = {
    off:       { r: 0,   g: 0,   b: 0   },
    red:       { r: 255, g: 0,   b: 0   },
    green:     { r: 0,   g: 170, b: 0   },
    blue:      { r: 0,   g: 0,   b: 200 },
    orange:    { r: 255, g: 112, b: 0   },
};

let pyProc      = null;
let available   = false;
let currentState = 'off';
let animTimer   = null;
let animPhase   = 0;

function startDriver() {
    const script = path.join(__dirname, 'led_driver.py');
    pyProc = spawn('python3', [script], { stdio: ['pipe', 'pipe', 'pipe'] });

    pyProc.stdout.once('data', () => {
        // First line is the ready signal from the driver
        available = true;
        console.log('[LED] Python driver ready');
        // Re-apply whatever state was set before the driver was ready
        applyState(currentState);
    });

    pyProc.stderr.on('data', (d) => {
        console.error('[LED] driver stderr:', d.toString().trim());
    });

    pyProc.on('exit', (code) => {
        available = false;
        if (code !== 0 && code !== null) {
            console.error('[LED] driver exited with code', code, '— LEDs disabled');
        }
    });
}

function send(obj) {
    if (!available || !pyProc || pyProc.killed) return;
    try {
        pyProc.stdin.write(JSON.stringify(obj) + '\n');
    } catch (e) { /* ignore write errors if process died */ }
}

function scaleColor(color, brightness) {
    return {
        r: Math.round(color.r * brightness),
        g: Math.round(color.g * brightness),
        b: Math.round(color.b * brightness),
    };
}

function fill(color) {
    send({ cmd: 'fill', r: color.r, g: color.g, b: color.b });
}

function stopAnim() {
    if (animTimer) { clearInterval(animTimer); animTimer = null; }
    animPhase = 0;
}

function startPulse(color) {
    stopAnim();
    animTimer = setInterval(() => {
        animPhase += 0.06;
        const brightness = 0.25 + 0.75 * (0.5 + 0.5 * Math.sin(animPhase));
        fill(scaleColor(color, brightness));
    }, 30);
}

function applyState(state) {
    stopAnim();
    switch (state) {
        case 'error':      fill(COLORS.red);                   break;
        case 'torque_off': fill(COLORS.blue);                  break;
        case 'moving':     startPulse(COLORS.orange);          break;
        case 'connected':  fill(COLORS.orange);                break;
        case 'online':     fill(COLORS.green);                 break;
        default:           send({ cmd: 'off' });               break;
    }
}

function setState(state) {
    if (state === currentState) return;
    currentState = state;
    if (available) applyState(state);
}

function shutdown() {
    stopAnim();
    send({ cmd: 'quit' });
    if (pyProc) { try { pyProc.stdin.end(); } catch (e) { /* ignore */ } }
}

startDriver();

module.exports = { setState, shutdown };
