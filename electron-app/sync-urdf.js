#!/usr/bin/env node
/**
 * URDF Sync
 *
 * kinematics.urdf is the robot description, but it exists in three places:
 *
 *   1. electron-app/kinematics.urdf                  <- the source of truth
 *   2. raspberry-pi-control-st3215/kinematics.urdf   <- the Pi's copy
 *   3. DEFAULT_URDF in electron-app/app.js           <- fallback when the file
 *                                                       cannot be read
 *
 * Copies 2 and 3 used to be maintained by hand and drifted apart, which meant
 * the fallback described a different arm to the one actually loaded. This
 * script regenerates them from copy 1.
 *
 * Usage:
 *   node sync-urdf.js           regenerate the copies
 *   node sync-urdf.js --check   report drift and exit 1 (for CI or a hook)
 */

const fs = require('fs');
const path = require('path');

const SOURCE = path.join(__dirname, 'kinematics.urdf');
const APP_JS = path.join(__dirname, 'app.js');
const PI_URDF = path.join(__dirname, '..', 'raspberry-pi-control-st3215', 'kinematics.urdf');

const checkOnly = process.argv.includes('--check');

/**
 * Escapes URDF text for a JavaScript template literal.
 * @param {string} text - Raw URDF
 * @returns {string} Text safe between backticks
 */
function escapeForTemplateLiteral(text) {
    return text
        .replace(/\\/g, '\\\\')
        .replace(/`/g, '\\`')
        .replace(/\$\{/g, '\\${');
}

/**
 * Compares two texts ignoring line endings and trailing whitespace.
 * @param {string} a
 * @param {string} b
 * @returns {boolean} True when they match
 */
function sameContent(a, b) {
    const normalise = (t) => t.replace(/\r\n/g, '\n').replace(/[ \t]+$/gm, '').trim();
    return normalise(a) === normalise(b);
}

function main() {
    if (!fs.existsSync(SOURCE)) {
        console.error('sync-urdf: source not found: ' + SOURCE);
        process.exit(1);
    }

    const urdf = fs.readFileSync(SOURCE, 'utf8');
    const problems = [];
    const written = [];

    // ---- 1. the Pi's copy ----
    if (fs.existsSync(PI_URDF)) {
        const piText = fs.readFileSync(PI_URDF, 'utf8');
        if (!sameContent(piText, urdf)) {
            if (checkOnly) {
                problems.push('raspberry-pi-control-st3215/kinematics.urdf differs from electron-app/kinematics.urdf');
            } else {
                fs.writeFileSync(PI_URDF, urdf);
                written.push('raspberry-pi-control-st3215/kinematics.urdf');
            }
        }
    } else {
        console.warn('sync-urdf: no Pi copy at ' + PI_URDF + ' — skipped');
    }

    // ---- 2. the fallback embedded in app.js ----
    const appSource = fs.readFileSync(APP_JS, 'utf8');
    const pattern = /(const DEFAULT_URDF\s*=\s*`)([\s\S]*?)(`;)/;
    const match = appSource.match(pattern);

    if (!match) {
        console.error('sync-urdf: could not find "const DEFAULT_URDF = `...`;" in app.js');
        process.exit(1);
    }

    const embedded = match[2];
    if (!sameContent(embedded, urdf)) {
        if (checkOnly) {
            problems.push('DEFAULT_URDF in app.js differs from kinematics.urdf');
        } else {
            const replacement = match[1] + escapeForTemplateLiteral(urdf.trim()) + match[3];
            fs.writeFileSync(APP_JS, appSource.replace(pattern, () => replacement));
            written.push('DEFAULT_URDF in app.js');
        }
    }

    // ---- report ----
    if (checkOnly) {
        if (problems.length) {
            console.error('sync-urdf: out of sync with kinematics.urdf');
            problems.forEach((p) => console.error('  - ' + p));
            console.error('Run "npm run sync-urdf" to regenerate.');
            process.exit(1);
        }
        console.log('sync-urdf: all copies match kinematics.urdf');
        return;
    }

    if (written.length) {
        console.log('sync-urdf: regenerated from kinematics.urdf');
        written.forEach((w) => console.log('  - ' + w));
    } else {
        console.log('sync-urdf: already in sync, nothing to do');
    }
}

main();
