'use strict';
/**
 * WS2812B LED status indicator for the robot arm server.
 * Spawns led_driver.py and communicates via stdin/stdout JSON lines.
 *
 * States (highest priority first):
 *   error        → solid red          (fatal / unrecoverable)
 *   recovering   → pulsing red        (worker crashed, restart in progress)
 *   thermal      → flashing orange    (servo thermal fault, auto-clears after 30 s)
 *   booting      → slow white pulse   (startup, worker not yet ready)
 *   torque_off   → solid grey/white   (motors online but disengaged)
 *   linear_path  → dual yellow comet  (executeLinearMove in progress)
 *   moving       → yellow comet       (any joint moving)
 *   connected    → solid cyan         (client holds the control session)
 *   partial      → solid purple       (some servos not detected)
 *   online       → solid green        (all servos ready, no active session)
 *   off          → all LEDs off       (startup / shutdown)
 */

const { spawn } = require('child_process');
const path       = require('path');

let pyProc       = null;
let available    = false;
let currentState = 'off';

function startDriver() {
    const script = path.join(__dirname, 'led_driver.py');
    pyProc = spawn('python3', [script], { stdio: ['pipe', 'pipe', 'pipe'] });

    pyProc.stdout.once('data', () => {
        available = true;
        console.log('[LED] Python driver ready');
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
    try { pyProc.stdin.write(JSON.stringify(obj) + '\n'); } catch (e) { /* ignore */ }
}

function applyState(state) {
    switch (state) {
        case 'error':
            send({ cmd: 'fill', r: 255, g: 0, b: 0 });
            break;
        case 'recovering':
            send({ cmd: 'pulse', r: 255, g: 0, b: 0, lo: 0.04, hi: 0.7, period: 1.2 });
            break;
        case 'thermal':
            send({ cmd: 'flash', r: 255, g: 100, b: 0, on_ms: 150, period_ms: 500 });
            break;
        case 'booting':
            send({ cmd: 'pulse', r: 160, g: 160, b: 160, lo: 0.04, hi: 0.6, period: 2.5 });
            break;
        case 'torque_off':
            send({ cmd: 'fill', r: 130, g: 130, b: 130 });
            break;
        case 'linear_path':
            send({ cmd: 'comet', r: 255, g: 208, b: 0, tail: 10, speed: 1.2, heads: 2 });
            break;
        case 'moving':
            send({ cmd: 'comet', r: 255, g: 208, b: 0, tail: 14, speed: 1.4, heads: 1 });
            break;
        case 'connected':
            send({ cmd: 'fill', r: 0, g: 160, b: 255 });
            break;
        case 'partial':
            send({ cmd: 'fill', r: 140, g: 0, b: 220 });
            break;
        case 'online':
            send({ cmd: 'fill', r: 0, g: 200, b: 60 });
            break;
        default:
            send({ cmd: 'off' });
            break;
    }
}

function setState(state) {
    if (state === currentState) return;
    currentState = state;
    if (available) applyState(state);
}

function shutdown() {
    send({ cmd: 'quit' });
    if (pyProc) { try { pyProc.stdin.end(); } catch (e) { /* ignore */ } }
}

startDriver();

module.exports = { setState, shutdown };
