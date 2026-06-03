/**
 * Test Script for Robot Arm I2C Communication
 * 
 * This script demonstrates how to use the robotArmI2C module to communicate
 * with PIC controllers. Run this on your Raspberry Pi to test the connection.
 * 
 * Usage:
 *   node test-i2c.js
 */

const RobotArm = require('./robotArmI2C');

// This function demonstrates all the basic operations
async function testJointController() {
    console.log('=== Robot Arm I2C Test ===\n');
    
    // Create a joint controller for joint 1
    // Using software I2C on /dev/i2c-3 (GPIO 17=SDA, GPIO 27=SCL)
    // I2C address 0x22 is the default PIC controller address
    const joint1 = new RobotArm.JointController(1, '/dev/i2c-3', 0x22);
    
    try {
        // Step 1: Open the I2C bus connection
        console.log('Step 1: Opening I2C bus...');
        await joint1.open();
        console.log('✓ I2C bus opened successfully\n');
        
        // Step 2: Read the current status
        console.log('Step 2: Reading current status...');
        const status = await joint1.readStatus();
        console.log('Current Status:');
        console.log(`  - Is Moving: ${status.isMoving}`);
        console.log(`  - Stall Detected: ${status.stallDetected}`);
        console.log(`  - Step Position: ${status.stepPosition}`);
        console.log(`  - Current Angle: ${status.angleDegrees.toFixed(2)}°\n`);
        
        // Step 3: Read the settings
        console.log('Step 3: Reading settings...');
        const settings = await joint1.readSettings();
        console.log('Settings:');
        console.log(`  - Sensor Address: 0x${settings.sensorAddress.toString(16)}`);
        console.log(`  - Sensor Offset: ${settings.sensorOffsetDegrees.toFixed(2)}°`);
        console.log('  - Kinematics Parameters:');
        console.log(`    d: ${settings.kinematics.d.toFixed(2)} mm`);
        console.log(`    a: ${settings.kinematics.a.toFixed(2)} mm`);
        console.log(`    theta: ${settings.kinematics.theta.toFixed(2)}°`);
        console.log(`    alpha: ${settings.kinematics.alpha.toFixed(2)}°\n`);
        
        // Step 4: Read ADC values
        console.log('Step 4: Reading ADC values...');
        const adc = await joint1.readADC();
        console.log('ADC Values:');
        console.log(`  - Raw ADC: ${adc.rawADC}`);
        console.log(`  - Voltage: ${adc.voltageVolts.toFixed(3)} V (${adc.voltageMillivolts} mV)\n`);
        
        // Step 5: Test motor movement (commented out by default - uncomment to test)
        // WARNING: This will move the motor! Make sure it's safe to do so.
        /*
        console.log('Step 5: Testing motor movement...');
        console.log('Moving to 45 degrees...');
        await joint1.moveToAngle(45.0);
        
        // Wait for movement to complete (check status every 500ms)
        let isMoving = true;
        while (isMoving) {
            await new Promise(resolve => setTimeout(resolve, 500));
            const status = await joint1.readStatus();
            isMoving = status.isMoving;
            console.log(`  Current angle: ${status.angleDegrees.toFixed(2)}°, Moving: ${isMoving}`);
        }
        
        console.log('Movement complete!\n');
        */
        
        // Step 6: Test servo control (commented out by default)
        /*
        console.log('Step 6: Testing servo control...');
        console.log('Setting servo to 90 degrees...');
        await joint1.setServoAngle(90);
        await new Promise(resolve => setTimeout(resolve, 1000));
        console.log('Servo command sent.\n');
        */
        
        console.log('=== Test Complete ===');
        
    } catch (error) {
        console.error('\n❌ Error during test:', error.message);
        console.error('\nTroubleshooting:');
        console.error('1. Make sure the Raspberry Pi I2C is enabled:');
        console.error('   sudo raspi-config -> Interface Options -> I2C -> Enable');
        console.error('2. Make sure your PIC controller is connected and powered');
        console.error('3. Check that the I2C address is correct (default: 0x22)');
        console.error('4. You may need to run with sudo: sudo node test-i2c.js');
    } finally {
        // Always close the I2C bus when done
        console.log('\nClosing I2C bus...');
        await joint1.close();
        console.log('✓ Test finished');
    }
}

// Run the test
testJointController();


