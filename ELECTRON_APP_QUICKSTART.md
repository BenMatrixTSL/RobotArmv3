# Electron App Quick Start Guide

Complete setup guide for the robot arm control system.

## System Overview

```
┌─────────────────────────────┐
│   Electron Desktop App      │
│   (Windows/Mac/Linux)       │
│   - Pendant Control         │
│   - G-Code Processing       │
└────────────┬────────────────┘
             │ WebSocket
             │ (port 8080)
             ▼
┌─────────────────────────────┐
│   Raspberry Pi 5            │
│   WebSocket Server          │
│   (server.js)               │
└────────────┬────────────────┘
             │ I2C Bus
             │
    ┌────────┴────────┬───────────┐
    │                 │           │
┌───▼───┐      ┌──────▼───┐  ┌───▼───┐
│ PIC 1 │      │ PIC 2    │  │ PIC N │
│Joint 1│      │Joint 2   │  │Joint N│
└───────┘      └──────────┘  └───────┘
```

## Step-by-Step Setup

### Part 1: Raspberry Pi Setup

1. **Enable I2C on Raspberry Pi:**
   ```bash
   sudo raspi-config
   # Interface Options → I2C → Enable
   sudo reboot
   ```

2. **Copy files to Raspberry Pi:**
   - Copy the `raspberry-pi-control` folder to your Raspberry Pi
   - Or clone/transfer files via network

3. **Install Node.js (if needed):**
   ```bash
   curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
   sudo apt install -y nodejs
   ```

4. **Install dependencies:**
   ```bash
   cd raspberry-pi-control
   npm install
   ```

5. **Configure I2C addresses:**
   - Edit `server.js`
   - Update the `addresses` array to match your PIC controllers:
     ```javascript
     const addresses = [0x22, 0x23, 0x24, 0x25];
     ```

6. **Test I2C connection:**
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see your PIC controllers listed.

7. **Start the server:**
   ```bash
   sudo npm run server
   ```
   
   Or:
   ```bash
   sudo node server.js
   ```

8. **Note the Raspberry Pi IP address:**
   ```bash
   hostname -I
   ```
   You'll need this for the Electron app (e.g., `192.168.1.100`)

### Part 2: Electron App Setup

1. **On your computer (Windows/Mac/Linux):**

2. **Install Node.js** (if not already installed):
   - Download from: https://nodejs.org/
   - Install version 14 or newer

3. **Copy the Electron app:**
   - Copy the `electron-app` folder to your computer

4. **Install dependencies:**
   ```bash
   cd electron-app
   npm install
   ```

5. **Start the Electron app:**
   ```bash
   npm start
   ```

### Part 3: Connect Everything

1. **In the Electron app:**
   - Enter your Raspberry Pi's IP address (from step 1.8)
   - Enter port `8080` (or your configured port)
   - Click **Connect**

2. **Verify connection:**
   - Connection status should turn green
   - Status should say "Connected"

3. **Test pendant control:**
   - Go to **Pendant Control** tab
   - Enter an angle (e.g., `90`)
   - Click **Move** for a joint
   - Watch the status update in real-time

## File Structure

```
Robot Arm 3/
├── Firmware/
│   ├── StepperControl.X/          # PIC firmware (existing)
│   ├── raspberry-pi-control/      # Raspberry Pi code
│   │   ├── robotArmI2C.js         # I2C communication module
│   │   ├── server.js              # WebSocket server
│   │   ├── test-i2c.js            # I2C test script
│   │   └── package.json
│   └── electron-app/              # Desktop app
│       ├── main.js                # Electron main process
│       ├── index.html             # UI layout
│       ├── app.js                 # Main application logic
│       ├── robotArmClient.js      # WebSocket client
│       ├── gcodeProcessor.js      # G-code parser
│       └── package.json
```

## Quick Reference

### Raspberry Pi Commands

```bash
# Start server
sudo npm run server

# Test I2C
sudo i2cdetect -y 1

# Find IP address
hostname -I
```

### Electron App Commands

```bash
# Start app
npm start

# Install dependencies (first time)
npm install
```

### Default Configuration

- **Server Port:** 8080
- **I2C Bus:** /dev/i2c-1
- **Joint Count:** 4 (configurable)
- **I2C Addresses:** 0x22, 0x23, 0x24, 0x25 (configurable)

## Troubleshooting

### Electron app can't connect

- ✅ Check Raspberry Pi server is running
- ✅ Check IP address is correct
- ✅ Check both devices on same network
- ✅ Check firewall (port 8080)
- ✅ Try pinging Raspberry Pi: `ping 192.168.1.100`

### Raspberry Pi server won't start

- ✅ Check I2C is enabled
- ✅ Check I2C addresses are correct
- ✅ Run with `sudo` (I2C needs root)
- ✅ Check for error messages in console

### Joints not moving

- ✅ Check PIC controllers are powered
- ✅ Check I2C connections
- ✅ Verify I2C addresses with `i2cdetect`
- ✅ Check server console for errors

### Status not updating

- ✅ Check WebSocket connection is active
- ✅ Check server is receiving status requests
- ✅ Check I2C communication is working
- ✅ Increase update interval if needed

## Next Steps

Once everything is working:

1. **Test all joints** individually
2. **Test pendant control** with multiple joints
3. **Load a G-code file** and test execution
4. **Adjust settings** as needed
5. **Add more features** (inverse kinematics, path planning, etc.)

## Getting Help

1. Check the README files in each folder
2. Check server console for error messages
3. Check Electron app console (DevTools)
4. Test I2C directly: `sudo node raspberry-pi-control/test-i2c.js`

## Summary

✅ **Raspberry Pi:** Run `sudo npm run server`  
✅ **Electron App:** Run `npm start`  
✅ **Connect:** Enter Pi IP address and click Connect  
✅ **Control:** Use pendant or G-code interface  

That's it! You're ready to control your robot arm! 🎉


