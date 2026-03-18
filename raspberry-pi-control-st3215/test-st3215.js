/**
 * Simple test script for ST3215 servo control
 * 
 * This script tests basic communication with a single ST3215 servo.
 * 
 * Usage:
 *   node test-st3215.js
 */

const RobotArm = require('./robotArmST3215');

// Configuration - adjust these for your setup
const SERIAL_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0158625-if00';  // Change to your serial port
const SERVO_ID = 5;                   // ST3215 servo ID (1-6)
const TEST_SERVO_NUMBER = 5;          // For logging purposes

async function testServo() {
    console.log('ST3215 Servo Test');
    console.log('==================');
    console.log(`Serial Port: ${SERIAL_PORT}`);
    console.log(`Servo ID: ${SERVO_ID}`);
    console.log();

    const servo = new RobotArm.ServoController(TEST_SERVO_NUMBER, SERIAL_PORT, SERVO_ID);

    try {
        // Open connection
        console.log('Opening serial port...');
        await servo.open();
        console.log('✓ Serial port opened');
        console.log();

        // Ping servo
        console.log('Pinging servo...');
        const pingResult = await servo.ping();
        if (pingResult) {
            console.log('✓ Servo responded to ping');
        } else {
            console.log('✗ Servo did not respond to ping');
            await servo.close();
            return;
        }
        console.log();

        // Enable torque
        console.log('Enabling torque...');
        await servo.startServo();
        console.log('✓ Torque enabled');
        console.log();

        // Simple continuous status read loop for debugging servo 5
        console.log('Starting continuous status read for servo 5 (Ctrl+C to stop)...');
        for (let i = 0; i < 200; i++) {
            try {
                const status = await servo.readStatus();
                console.log(
                    `Read ${i}: position=${status.position}, ` +
                    `angle=${status.angleDegrees.toFixed(2)}°, ` +
                    `voltage=${status.voltage}V, temp=${status.temperature}°C`
                );
            } catch (err) {
                console.error('Error during readStatus:', err.message);
            }
            // 200ms between reads so we roughly match the server polling rate
            await new Promise(resolve => setTimeout(resolve, 200));
        }

        // After loop, we fall through to "Disabling torque..." and close as normal
        console.log('Disabling torque...');
        await servo.stopServo();
        console.log('✓ Torque disabled');
        console.log();

        // Close connection
        console.log('Closing serial port...');
        await servo.close();
        console.log('✓ Serial port closed');
        console.log();

        console.log('Test completed successfully!');

    } catch (error) {
        console.error('Error during test:', error);
        console.error(error.stack);
        
        // Try to close connection
        try {
            await servo.close();
        } catch (closeError) {
            console.error('Error closing connection:', closeError);
        }
        
        process.exit(1);
    }
}

// Run the test
testServo();

