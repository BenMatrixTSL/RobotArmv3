# Robot Arm Raspberry Pi Control System

This is a simple Node.js module for controlling robot arm joints via I2C communication with PIC16F18326 microcontrollers.

## What This Does

- Communicates with PIC controllers over I2C
- Controls stepper motors (move to angle, stop)
- Controls servo motors
- Reads sensor data (angle, position, ADC values)
- Reads and writes settings (sensor calibration, kinematics parameters)

## Prerequisites

### Hardware
- Raspberry Pi 5 (or Raspberry Pi 4)
- PIC16F18326 controllers with I2C firmware
- I2C bus connections (SDA, SCL, GND, +5V)

### Software
- Node.js installed on Raspberry Pi (version 14 or newer recommended)
- I2C enabled on Raspberry Pi

## Setup Instructions

### Step 1: Enable I2C on Raspberry Pi

1. Open the Raspberry Pi configuration tool:
   ```bash
   sudo raspi-config
   ```

2. Navigate to: **Interface Options** → **I2C** → **Enable**

3. Reboot the Raspberry Pi:
   ```bash
   sudo reboot
   ```

### Step 2: Install Node.js

If Node.js is not already installed:

```bash
# Update package list
sudo apt update

# Install Node.js (this installs version 18.x)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version
npm --version
```

### Step 3: Install Project Dependencies

1. Navigate to this directory:
   ```bash
   cd raspberry-pi-control
   ```

2. Install the required npm packages:
   ```bash
   npm install
   ```

This will install:
- `i2c-bus` - Library for I2C communication on Raspberry Pi

### Step 4: Verify I2C Connection

1. Check if I2C devices are detected:
   ```bash
   sudo i2cdetect -y 1
   ```

   You should see your PIC controllers listed (default address: 0x22)

2. If you see devices, you're ready to test!

## Usage

### Basic Example

```javascript
const RobotArm = require('./robotArmI2C');

async function example() {
    // Create a controller for joint 1
    // Using software I2C on /dev/i2c-3 (GPIO 17=SDA, GPIO 27=SCL)
    const joint1 = new RobotArm.JointController(1, '/dev/i2c-3', 0x22);
    
    // Open the I2C connection
    await joint1.open();
    
    // Move motor to 90 degrees
    await joint1.moveToAngle(90.0);
    
    // Read current status
    const status = await joint1.readStatus();
    console.log(`Current angle: ${status.angleDegrees}°`);
    
    // Close the connection
    await joint1.close();
}

example();
```

### Run the Test Script

To test your setup, run the test script:

```bash
# You may need sudo for I2C access
sudo node test-i2c.js
```

The test script will:
- Open the I2C connection
- Read current status
- Read settings
- Read ADC values
- (Motor movement tests are commented out by default)

## Available Functions

### Motor Control

```javascript
// Move to a specific angle (in degrees)
await joint1.moveToAngle(90.5);

// Stop motor immediately
await joint1.stopMotion();
```

### Servo Control

```javascript
// Set servo angle (0-180 degrees)
await joint1.setServoAngle(90);

// Set servo as relay (on/off)
await joint1.setRelayState(true);  // ON
await joint1.setRelayState(false); // OFF
```

### Reading Status

```javascript
// Read current status (returns object with all info)
const status = await joint1.readStatus();
console.log(status.isMoving);        // true/false
console.log(status.stallDetected);   // true/false
console.log(status.stepPosition);    // current step count
console.log(status.angleDegrees);    // current angle in degrees
```

### Reading Settings

```javascript
// Read settings and kinematics parameters
const settings = await joint1.readSettings();
console.log(settings.sensorAddress);
console.log(settings.sensorOffsetDegrees);
console.log(settings.kinematics.d);     // link offset in mm
console.log(settings.kinematics.a);     // link length in mm
console.log(settings.kinematics.theta); // joint angle in degrees
console.log(settings.kinematics.alpha); // link twist in degrees
```

### Setting Calibration

```javascript
// Set sensor offset
await joint1.setSensorOffset(1.5); // 1.5 degrees offset

// Set sensor I2C address
await joint1.setSensorAddress(0x36);
```

### Setting Kinematics Parameters

```javascript
// Set kinematics parameters (Denavit-Hartenberg)
await joint1.setKinematicsParameter('d', 50.5, 'mm');        // link offset
await joint1.setKinematicsParameter('a', 100.0, 'mm');       // link length
await joint1.setKinematicsParameter('theta', 45.0, 'degrees'); // joint angle
await joint1.setKinematicsParameter('alpha', 90.0, 'degrees'); // link twist
```

### Reading ADC

```javascript
// Read analog input values
const adc = await joint1.readADC();
console.log(adc.rawADC);           // raw ADC value (0-1023)
console.log(adc.voltageVolts);     // voltage in volts
console.log(adc.voltageMillivolts); // voltage in millivolts
```

## Multiple Joints

To control multiple joints, create multiple controllers:

```javascript
// All joints share the same I2C bus but have different addresses
const joint1 = new RobotArm.JointController(1, '/dev/i2c-3', 0x22);
const joint2 = new RobotArm.JointController(2, '/dev/i2c-3', 0x23);
const joint3 = new RobotArm.JointController(3, '/dev/i2c-3', 0x24);

await joint1.open();
await joint2.open();
await joint3.open();

// Control them independently
await joint1.moveToAngle(90);
await joint2.moveToAngle(45);
await joint3.moveToAngle(180);

// Close when done
await joint1.close();
await joint2.close();
await joint3.close();
```

**Note:** Make sure each PIC controller has a different I2C address. You can change the address in the PIC firmware if needed.

## Troubleshooting

### "Permission denied" error

I2C access requires root privileges. Run with sudo:
```bash
sudo node test-i2c.js
```

Or add your user to the i2c group:
```bash
sudo usermod -a -G i2c $USER
# Then log out and log back in
```

### "Cannot find module 'i2c-bus'"

Make sure you installed dependencies:
```bash
npm install
```

### No I2C devices detected

1. Check I2C is enabled:
   ```bash
   sudo raspi-config
   # Interface Options -> I2C -> Enable
   ```

2. Check physical connections:
   - SDA connected to GPIO 2 (Pin 3)
   - SCL connected to GPIO 3 (Pin 5)
   - GND connected to ground
   - Power connected

3. Check if I2C bus exists:
   ```bash
   ls -l /dev/i2c-*
   ```

### "Failed to open I2C bus"

1. Make sure you're using the correct bus number:
   - This project uses software I2C on `/dev/i2c-3` (GPIO 17=SDA, GPIO 27=SCL)
   - Check available buses with `ls /dev/i2c-*`

2. Check if bus exists:
   ```bash
   ls -l /dev/i2c-3
   ```

3. Verify your PIC is detected:
   ```bash
   sudo i2cdetect -y 3
   ```

## Next Steps

Once this is working:

1. **Phase 2:** Create a module to coordinate multiple joints
2. **Phase 3:** Add G-code processing
3. **Phase 4:** Create Electron.js desktop app for pendant control
4. **Phase 5:** Add path planning and kinematics calculations

## File Structure

```
raspberry-pi-control/
├── robotArmI2C.js    # Main I2C communication module
├── test-i2c.js       # Test script
├── package.json      # Node.js dependencies
└── README.md         # This file
```

## License

MIT


