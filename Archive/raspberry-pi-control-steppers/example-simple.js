/**
 * Simple Example: Basic Robot Arm Control
 * 
 * This is a very simple example showing the most common operations.
 * Copy this and modify it for your needs.
 */

const RobotArm = require('./robotArmI2C');

async function simpleExample() {
    console.log('Simple Robot Arm Control Example\n');
    
    // Create a controller for joint 1
    // Using software I2C on /dev/i2c-3 (GPIO 17=SDA, GPIO 27=SCL)
    const joint = new RobotArm.JointController(1, '/dev/i2c-3', 0x22);
    
    try {
        // Open the I2C connection
        console.log('Connecting to joint controller...');
        await joint.open();
        console.log('Connected!\n');
        
        // Read and display current status
        console.log('Reading current status...');
        const status = await joint.readStatus();
        console.log(`Current angle: ${status.angleDegrees.toFixed(2)}°`);
        console.log(`Motor is moving: ${status.isMoving}\n`);
        
        // Move the motor to 90 degrees
        console.log('Moving to 90 degrees...');
        await joint.moveToAngle(90.0);
        
        // Wait a bit for movement to start
        await new Promise(resolve => setTimeout(resolve, 500));
        
        // Check status while moving
        let checkCount = 0;
        while (status.isMoving && checkCount < 20) {
            const currentStatus = await joint.readStatus();
            console.log(`  Angle: ${currentStatus.angleDegrees.toFixed(2)}°, Moving: ${currentStatus.isMoving}`);
            
            // Update status for next check
            status.isMoving = currentStatus.isMoving;
            status.angleDegrees = currentStatus.angleDegrees;
            
            // Wait 500ms before checking again
            await new Promise(resolve => setTimeout(resolve, 500));
            checkCount++;
        }
        
        if (status.isMoving) {
            console.log('\nMovement taking longer than expected. Stopping...');
            await joint.stopMotion();
        } else {
            console.log('\nMovement complete!');
        }
        
    } catch (error) {
        console.error('Error:', error.message);
        console.error('\nMake sure:');
        console.error('1. Raspberry Pi I2C is enabled');
        console.error('2. PIC controller is connected and powered');
        console.error('3. Run with sudo if needed: sudo node example-simple.js');
    } finally {
        // Always close the connection when done
        await joint.close();
        console.log('\nConnection closed.');
    }
}

// Run the example
simpleExample();


