# Kinematics Integration Guide

This document explains the kinematics functionality that has been added to the Electron app.

## What Was Added

### 1. Kinematics Module (`kinematics.js`)

A complete forward and inverse kinematics module using Denavit-Hartenberg (DH) parameters:

- **Forward Kinematics**: Calculates end effector position from joint angles
- **Inverse Kinematics**: Calculates joint angles from end effector position
- **DH Parameter Support**: Uses standard Denavit-Hartenberg transformation matrices

### 2. Joint Configuration Reading

The app can now:
- Read kinematics parameters from PIC controllers
- Store configurations locally
- Display and edit parameters in the UI
- Save parameters back to PIC controllers

### 3. G-Code Integration

G-code processing now uses inverse kinematics:
- Converts X, Y, Z coordinates to joint angles
- Moves joints to calculated positions
- Works with standard G-code commands (G0, G1)

### 4. UI Enhancements

Added to the Settings tab:
- **Kinematics Configuration Panel**: View and edit DH parameters for each joint
- **Kinematics Testing Panel**: Test forward and inverse kinematics calculations

## How to Use

### Step 1: Load Joint Configurations

1. Connect to your Raspberry Pi
2. Go to the **Settings** tab
3. Click **"Load from Robot"**
4. The app will read DH parameters from all PIC controllers
5. Parameters will be displayed in cards for each joint

### Step 2: Configure Kinematics

1. Review the loaded parameters (d, a, theta, alpha)
2. Edit values if needed directly in the input fields
3. Click **"Save to Robot"** to save changes to a specific joint
4. Click **"Apply to Kinematics"** to use the configurations

### Step 3: Test Kinematics

**Forward Kinematics Test:**
- Enter joint angles (e.g., "0, 90, 45, 0")
- Click **Calculate**
- See the calculated end effector position (X, Y, Z)

**Inverse Kinematics Test:**
- Enter target position (X, Y, Z in mm)
- Click **Calculate**
- See the calculated joint angles

### Step 4: Use with G-Code

1. Load joint configurations (Step 1)
2. Load a G-code file
3. Start execution
4. The app will automatically:
   - Convert X, Y, Z coordinates to joint angles
   - Move joints to calculated positions

## Understanding DH Parameters

Each joint has 4 Denavit-Hartenberg parameters:

- **d (link offset)**: Distance along the previous z-axis (in mm)
- **a (link length)**: Distance along the x-axis (in mm)
- **θ (joint angle)**: Rotation about z-axis (in degrees)
- **α (link twist)**: Rotation about x-axis (in degrees)

These parameters define the geometry of your robot arm and are used to calculate forward and inverse kinematics.

## Current Limitations

### Inverse Kinematics

The current inverse kinematics implementation is simplified:
- Works for 2D planar arms (2 joints)
- For more complex configurations, you'll need to implement custom inverse kinematics based on your robot's geometry

### G-Code Processing

- Currently handles basic G0/G1 movement commands
- Does not handle tool offsets or coordinate system transformations
- Does not track current position (uses target position only)

## Extending the Kinematics

### For Your Specific Robot Arm

1. **Understand Your Geometry**: Measure or calculate the DH parameters for your robot
2. **Set Parameters**: Use the UI to set parameters for each joint
3. **Test Forward Kinematics**: Verify positions match your measurements
4. **Implement Custom Inverse Kinematics**: If the simple implementation doesn't work, implement a solver for your specific geometry

### Example: Custom Inverse Kinematics

If you need to implement custom inverse kinematics for your robot, modify the `inverseKinematics()` function in `kinematics.js`:

```javascript
inverseKinematics(targetPose) {
    // Your custom implementation here
    // This depends on your robot's specific geometry
    
    // Example for a specific configuration:
    // Calculate joint angles based on your robot's geometry
    // Return array of joint angles in degrees
}
```

## API Reference

### RobotKinematics Class

```javascript
// Set joint configurations
robotKinematics.setJointConfigurations([
    { d: 50, a: 100, theta: 0, alpha: 90 },
    { d: 0, a: 150, theta: 0, alpha: 0 }
]);

// Forward kinematics
const pose = robotKinematics.forwardKinematics([0, 90, 45, 0]);
// Returns: { position: { x, y, z }, rotation: [...], transformation: [...] }

// Inverse kinematics
const angles = robotKinematics.inverseKinematics({ x: 200, y: 100, z: 50 });
// Returns: [angle1, angle2, ...] or null if unreachable
```

### RobotArmClient

```javascript
// Request joint configurations
robotArmClient.requestJointConfigs();

// Set kinematics parameter
robotArmClient.setKinematics(jointNumber, 'd', 50.5, 'mm');

// Get stored configurations
const configs = robotArmClient.getJointConfigs();
```

## Troubleshooting

### "Kinematics not configured" error

- Load joint configurations from the robot first
- Make sure all joints are available and configured

### Inverse kinematics returns null

- Target position may be out of reach
- Check your DH parameters are correct
- The simplified implementation only works for 2D planar arms

### G-code doesn't move joints

- Check that kinematics is configured
- Verify G-code has X, Y, Z coordinates
- Check connection to Raspberry Pi

### Parameters won't save

- Verify connection to Raspberry Pi
- Check I2C communication is working
- Look at server console for errors

## Next Steps

1. **Calibrate Your Robot**: Measure actual DH parameters and update in the UI
2. **Test Forward Kinematics**: Move joints manually and verify calculated positions
3. **Implement Custom IK**: If needed, implement inverse kinematics for your specific geometry
4. **Enhance G-Code**: Add support for more G-code features (tool offsets, coordinate systems, etc.)

## Files Modified

- `electron-app/kinematics.js` - New kinematics module
- `electron-app/robotArmClient.js` - Added configuration reading
- `electron-app/app.js` - Added UI functions and G-code integration
- `electron-app/index.html` - Added UI panels
- `electron-app/styles.css` - Added styling
- `raspberry-pi-control/server.js` - Added configuration reading support

## Summary

You now have a complete kinematics system that:
- ✅ Reads joint configurations from PIC controllers
- ✅ Calculates forward kinematics (joint angles → position)
- ✅ Calculates inverse kinematics (position → joint angles)
- ✅ Integrates with G-code processing
- ✅ Provides UI for viewing and editing parameters

The system is ready to use, and you can extend it based on your specific robot arm geometry!


