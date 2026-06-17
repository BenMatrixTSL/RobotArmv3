/**
 * reprogramServo2.js
 *
 * Re-programs the servo currently responding as ID=1 (joint 2's servo, which
 * was accidentally assigned the same ID as joint 1) to use ID=2.
 *
 * BEFORE RUNNING:
 *   1. Stop the st3215-server: sudo systemctl stop st3215-server.service
 *   2. Physically disconnect joint 1's servo DATA cable from the bus so that
 *      only joint 2's servo is connected. (Power does not need to be removed.)
 *   3. Run: node reprogramServo2.js
 *   4. Reconnect joint 1's servo.
 *   5. Restart the server: sudo systemctl start st3215-server.service
 *
 * The ST3215 ID register is at EEPROM address 0x05.
 * The EEPROM lock register is at address 0x37 (0=unlocked, 1=locked).
 * After writing, the servo needs a power cycle to apply the new ID.
 */

'use strict';

const { ServoController } = require('./robotArmST3215.js');
const { SerialPort }       = require('serialport');

const SERIAL_PORT  = process.env.SERIAL_PORT || '/dev/serial0';
const BAUD_RATE    = 1000000;

const EEPROM_LOCK_ADDR = 0x37;  // 0 = unlocked, 1 = locked
const ID_ADDR          = 0x05;  // servo ID register (EEPROM)

async function main() {
    console.log('Opening serial port:', SERIAL_PORT);
    const port = new SerialPort({
        path: SERIAL_PORT,
        baudRate: BAUD_RATE,
        dataBits: 8,
        parity: 'none',
        stopBits: 1,
        autoOpen: false
    });

    await new Promise((res, rej) => port.open(e => e ? rej(e) : res()));
    console.log('Port open.');

    // Create a controller talking to ID=1 (the current ID of joint 2's servo)
    const servo = new ServoController(1, port, 1, BAUD_RATE);
    await servo.open();

    port.on('data', d => servo.handleIncomingData(d));

    await new Promise(r => setTimeout(r, 600));

    // Step 1: ping to confirm a servo is there
    console.log('Pinging ID=1...');
    const alive = await servo.ping();
    if (!alive) {
        console.error('No servo responded to ID=1. Make sure:');
        console.error('  - Joint 2 servo is powered and connected to the bus');
        console.error('  - Joint 1 servo data cable is DISCONNECTED');
        port.close();
        process.exit(1);
    }
    console.log('Servo found at ID=1.');

    // Step 2: read current position so user can confirm it is the right servo
    try {
        const posData = await servo.readData(0x38, 2);
        const pos     = posData[0] | (posData[1] << 8);
        const angle   = ((pos - 2048) / 11.375).toFixed(1);
        console.log('Current position: steps=' + pos + '  angle=' + angle + ' deg');
        console.log('Expected: this should match joint 2\'s angle shown in the UI (~40 deg).');
    } catch (e) {
        console.warn('Could not read position (non-fatal):', e.message);
    }

    // Step 3: unlock EEPROM
    console.log('Unlocking EEPROM (writing 0 to register 0x37)...');
    await servo.writeData(EEPROM_LOCK_ADDR, [0]);
    await new Promise(r => setTimeout(r, 100));

    // Step 4: write new ID=2 to the ID register
    console.log('Writing new ID=2 to register 0x05...');
    await servo.writeData(ID_ADDR, [2]);
    await new Promise(r => setTimeout(r, 200));

    console.log('');
    console.log('Done. The servo has been re-programmed to ID=2.');
    console.log('');
    console.log('Next steps:');
    console.log('  1. Power-cycle the joint 2 servo (or power-cycle the whole arm)');
    console.log('     so the new ID takes effect.');
    console.log('  2. Reconnect joint 1\'s servo data cable.');
    console.log('  3. Restart the server:');
    console.log('       sudo systemctl start st3215-server.service');

    await servo.close();
    port.close();
    process.exit(0);
}

main().catch(e => {
    console.error('Fatal error:', e.message);
    process.exit(1);
});
