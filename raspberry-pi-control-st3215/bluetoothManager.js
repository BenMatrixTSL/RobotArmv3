/**
 * Bluetooth game controller pairing for the Pi (BlueZ via bluetoothctl).
 *
 * Used by the Gamepad Control page when the UI runs on the Pi itself
 * (the touchscreen kiosk): a controller must pair with the machine whose
 * browser reads it, so server.js only allows these calls from local clients.
 *
 * Each operation runs its own interactive bluetoothctl session, feeds it
 * commands on stdin and watches stdout for the result lines. The server runs
 * as root, so no D-Bus policy changes are needed.
 */

'use strict';

const { spawn } = require('child_process');

const MAC_RE = /^[0-9A-F]{2}(:[0-9A-F]{2}){5}$/i;
const ANSI_RE = /\x1b\[[0-9;?]*[A-Za-z]|\x01|\x02|\r/g;

let busy = false;

/**
 * Runs one bluetoothctl session.
 * script(api) gets { send(line), waitFor(regex, ms), wait(ms), output() }
 * and returns the result. The session is always closed afterwards.
 */
function runSession(script, overallTimeoutMs) {
    if (busy) return Promise.reject(new Error('Bluetooth is busy with another scan or pairing - try again in a moment'));
    busy = true;

    return new Promise((resolve, reject) => {
        let proc;
        try {
            proc = spawn('bluetoothctl', [], { stdio: ['pipe', 'pipe', 'pipe'] });
        } catch (e) {
            busy = false;
            reject(new Error('bluetoothctl is not available on this system'));
            return;
        }

        let text = '';
        let waiters = [];
        let finished = false;

        const onData = (chunk) => {
            text += chunk.toString('utf8').replace(ANSI_RE, '');
            waiters = waiters.filter((w) => {
                const m = text.slice(w.from).match(w.re);
                if (m) { clearTimeout(w.timer); w.resolve(m); return false; }
                return true;
            });
        };
        proc.stdout.on('data', onData);
        proc.stderr.on('data', onData);

        const finish = (err, value) => {
            if (finished) return;
            finished = true;
            clearTimeout(overallTimer);
            waiters.forEach((w) => { clearTimeout(w.timer); w.resolve(null); });
            waiters = [];
            try { proc.stdin.write('scan off\nquit\n'); } catch (e) { /* closed */ }
            setTimeout(() => { try { proc.kill(); } catch (e) { /* gone */ } }, 500);
            busy = false;
            if (err) reject(err); else resolve(value);
        };

        proc.on('error', () => finish(new Error('bluetoothctl is not available on this system')));
        proc.on('exit', () => finish(new Error('bluetoothctl exited unexpectedly')));

        const overallTimer = setTimeout(() => finish(new Error('Bluetooth operation timed out')), overallTimeoutMs);

        const api = {
            send(line) { if (!finished) proc.stdin.write(line + '\n'); },
            // Resolves with the match, or null on timeout. Only output that
            // arrives after this call is searched.
            waitFor(re, ms) {
                return new Promise((res) => {
                    const w = { re, from: text.length, resolve: res, timer: null };
                    w.timer = setTimeout(() => {
                        waiters = waiters.filter((x) => x !== w);
                        res(null);
                    }, ms);
                    waiters.push(w);
                });
            },
            wait(ms) { return new Promise((res) => setTimeout(res, ms)); },
            output() { return text; }
        };

        Promise.resolve()
            .then(() => script(api))
            .then((value) => finish(null, value), (err) => finish(err));
    });
}

/** Parses "Device AA:BB:.. Name" lines into a Map of mac -> name. */
function parseDeviceLines(text) {
    const devices = new Map();
    const re = /Device ([0-9A-F]{2}(?::[0-9A-F]{2}){5}) ?([^\n]*)/gi;
    let m;
    while ((m = re.exec(text))) {
        const mac = m[1].toUpperCase();
        let name = (m[2] || '').trim();
        // Skip property-change lines such as "Device AA:.. RSSI: -60"
        if (/^(RSSI|TxPower|ManufacturerData|ServiceData|UUIDs|Connected|Paired|Bonded|Trusted|ServicesResolved|Class|Icon|Appearance|Alias|Modalias|WakeAllowed|LegacyPairing|AdvertisingFlags|Blocked|CablePairing|Sets)\b/i.test(name)) {
            if (!devices.has(mac)) devices.set(mac, '');
            continue;
        }
        if (name.startsWith('Name: ')) name = name.slice(6);
        if (!devices.has(mac) || (name && !devices.get(mac))) devices.set(mac, name);
    }
    return devices;
}

/** Reads the "info" block for one device. */
async function readInfo(api, mac) {
    api.send('info ' + mac);
    // The info block starts "Device <mac> (public|random)"; async "[CHG] Device
    // <mac> RSSI: .." lines don't, so they can't be mistaken for it. Connected
    // is listed before WakeAllowed/LegacyPairing/UUIDs, so stopping there is enough.
    const m = await api.waitFor(new RegExp('Device ' + mac + ' \\((?:public|random)\\)[\\s\\S]*?(?:WakeAllowed|LegacyPairing|CablePairing|UUID|Modalias|\\[[^\\]]*\\][#>])', 'i'), 1500);
    const block = m ? m[0] : '';
    const prop = (key) => {
        const r = block.match(new RegExp('\\n\\s*' + key + ': ([^\\n]*)'));
        return r ? r[1].trim() : null;
    };
    return {
        name: prop('Name') || prop('Alias') || '',
        icon: prop('Icon') || '',
        paired: prop('Paired') === 'yes',
        trusted: prop('Trusted') === 'yes',
        connected: prop('Connected') === 'yes'
    };
}

function looksLikeGamepad(name, icon) {
    if (icon === 'input-gaming') return true;
    return /controller|gamepad|joy-?con|dualshock|dualsense|wireless controller|xbox|8bitdo|pro controller|joystick/i.test(name || '');
}

/**
 * Scans for nearby devices and returns them with their state, game
 * controllers first. Controllers must be in pairing mode to show up.
 */
function scan(durationMs) {
    const ms = Math.max(3000, Math.min(Number(durationMs) || 8000, 20000));
    return runSession(async (api) => {
        api.send('power on');
        await api.wait(300);
        api.send('scan on');
        await api.wait(ms);
        api.send('scan off');
        await api.wait(200);
        api.send('devices');
        await api.wait(600);

        const devices = parseDeviceLines(api.output());
        const result = [];
        for (const [mac, seenName] of devices) {
            const info = await readInfo(api, mac);
            const name = info.name || seenName || '';
            // Unnamed devices that are not controllers are just noise
            // (phones' random addresses, beacons, ...).
            if (!name && info.icon !== 'input-gaming') continue;
            if (name.replace(/-/g, ':').toUpperCase() === mac) continue;
            result.push({
                mac,
                name: name || mac,
                icon: info.icon,
                paired: info.paired,
                trusted: info.trusted,
                connected: info.connected,
                gamepad: looksLikeGamepad(name, info.icon)
            });
        }
        result.sort((a, b) => (b.gamepad - a.gamepad) || (b.connected - a.connected) || a.name.localeCompare(b.name));
        return result;
    }, ms + 30000);
}

/** Pairs, trusts and connects a device. Trusting lets it reconnect by itself later. */
function pair(mac) {
    if (typeof mac !== 'string' || !MAC_RE.test(mac)) {
        return Promise.reject(new Error('Invalid Bluetooth address'));
    }
    mac = mac.toUpperCase();
    return runSession(async (api) => {
        api.send('power on');
        api.send('agent NoInputNoOutput');
        api.send('default-agent');
        await api.wait(300);

        // The controller has to be visible to pair, so scan until it shows up.
        api.send('scan on');
        await api.waitFor(new RegExp(mac, 'i'), 8000);

        const info = await readInfo(api, mac);
        if (!info.paired) {
            api.send('pair ' + mac);
            const r = await api.waitFor(/Pairing successful|Failed to pair[^\n]*|AlreadyExists|not available/i, 25000);
            if (!r) throw new Error('Pairing timed out - put the controller in pairing mode and try again');
            if (/Failed|not available/i.test(r[0])) {
                throw new Error(r[0].includes('not available')
                    ? 'Controller not found - put it in pairing mode and scan again'
                    : r[0].trim());
            }
        }
        api.send('scan off');
        api.send('trust ' + mac);
        await api.waitFor(/trust succeeded|Trusted: yes/i, 3000);

        api.send('connect ' + mac);
        const c = await api.waitFor(/Connection successful|Failed to connect[^\n]*|Connected: yes/i, 15000);
        if (!c || /Failed/i.test(c[0])) {
            throw new Error('Paired, but could not connect - press the controller\'s home button to wake it');
        }
        return { mac, paired: true, connected: true };
    }, 60000);
}

/** Disconnects and forgets a device. */
function remove(mac) {
    if (typeof mac !== 'string' || !MAC_RE.test(mac)) {
        return Promise.reject(new Error('Invalid Bluetooth address'));
    }
    mac = mac.toUpperCase();
    return runSession(async (api) => {
        api.send('remove ' + mac);
        await api.waitFor(/Device has been removed|not available|Failed/i, 5000);
        return { mac, removed: true };
    }, 10000);
}

module.exports = { scan, pair, remove };
