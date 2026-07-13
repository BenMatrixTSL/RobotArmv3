'use strict';
/**
 * WS2812B LED status indicator for the robot arm server.
 * Connects to GPIO pin 18 (configurable via LED_GPIO env var).
 *
 * States (highest priority first):
 *   error      → solid red    (worker crashed, fatal fault)
 *   torque_off → solid blue   (motors online but torque disabled)
 *   moving     → orange pulse (any joint is moving)
 *   connected  → solid orange (a client holds the control session)
 *   online     → solid green  (motors ready, no active session)
 *   off        → all LEDs off (startup / shutdown)
 */

const LED_COUNT = parseInt(process.env.LED_COUNT || '30', 10);
const LED_GPIO  = parseInt(process.env.LED_GPIO  || '18', 10);

// 0x00RRGGBB — the rpi-ws281x-native library converts to WS2812B GRB internally.
const C_OFF    = 0x000000;
const C_RED    = 0xFF0000;
const C_GREEN  = 0x00AA00;
const C_BLUE   = 0x0000CC;
const C_ORANGE = 0xFF7000;

let ws281x     = null;
let pixelData  = null;
let available  = false;

try {
    ws281x    = require('rpi-ws281x-native');
    pixelData = new Uint32Array(LED_COUNT);
    ws281x.init(LED_COUNT, { gpioPin: LED_GPIO });
    available = true;
    console.log(`[LED] Initialized ${LED_COUNT} LEDs on GPIO${LED_GPIO}`);
} catch (e) {
    console.log('[LED] rpi-ws281x-native not available — LED output disabled (' + e.message + ')');
}

let currentState   = 'off';
let animTimer      = null;
let animPhase      = 0;

function scaleColor(color, brightness) {
    const r = Math.round(((color >> 16) & 0xFF) * brightness);
    const g = Math.round(((color >>  8) & 0xFF) * brightness);
    const b = Math.round(( color        & 0xFF) * brightness);
    return (r << 16) | (g << 8) | b;
}

function fill(color) {
    if (!available) return;
    for (let i = 0; i < LED_COUNT; i++) pixelData[i] = color;
    ws281x.render(pixelData);
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

function setState(state) {
    if (state === currentState) return;
    currentState = state;
    stopAnim();

    switch (state) {
        case 'error':      fill(C_RED);             break;
        case 'torque_off': fill(C_BLUE);            break;
        case 'moving':     startPulse(C_ORANGE);    break;
        case 'connected':  fill(C_ORANGE);          break;
        case 'online':     fill(C_GREEN);           break;
        default:           fill(C_OFF);             break;
    }
}

function shutdown() {
    stopAnim();
    if (available) {
        fill(C_OFF);
        try { ws281x.reset(); } catch (e) { /* ignore */ }
    }
}

module.exports = { setState, shutdown };
