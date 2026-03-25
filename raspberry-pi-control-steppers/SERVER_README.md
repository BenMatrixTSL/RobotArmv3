# Robot Arm WebSocket Server

This server runs on your Raspberry Pi and connects the Electron desktop app to the I2C controllers.

## What It Does

- Listens for connections from the Electron desktop app
- Receives commands (move joint, stop, etc.)
- Translates commands to I2C communication with PIC controllers
- Sends status updates back to the Electron app

## Quick Start

1. **Make sure I2C is working:**
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see your PIC controllers listed.

2. **Install dependencies (if not already done):**
   ```bash
   npm install
   ```

3. **Configure I2C addresses:**
   - Open `server.js`
   - Edit the `addresses` array to match your PIC controller addresses:
     ```javascript
     const addresses = [0x22, 0x23, 0x24, 0x25]; // Change these!
     ```

4. **Start the server:**
   ```bash
   npm run server
   ```
   
   Or:
   ```bash
   node server.js
   ```

5. **The server will:**
   - Initialize all joint controllers
   - Start listening on port 8080
   - Wait for the Electron app to connect

## Configuration

### Change Port Number

Edit `server.js` and change:
```javascript
const PORT = 8080; // Change this to any port you want
```

### Change Number of Joints

Edit `server.js` and change:
```javascript
const JOINT_COUNT = 4; // Change to match your robot arm
```

### Change I2C Addresses

Edit `server.js` and change:
```javascript
const addresses = [0x22, 0x23, 0x24, 0x25]; // Your PIC addresses
```

## Troubleshooting

### "Permission denied" errors

Run with sudo:
```bash
sudo node server.js
```

### "Cannot find module 'ws'"

Install dependencies:
```bash
npm install
```

### "Failed to initialize joint"

- Check I2C addresses are correct
- Make sure PIC controllers are powered and connected
- Verify I2C addresses with: `sudo i2cdetect -y 1`

### "Address already in use"

- Another process is using port 8080
- Change the port number in `server.js`
- Or stop the other process

## Testing

1. Start the server
2. In the Electron app, connect to your Raspberry Pi's IP address
3. Try moving a joint
4. Check the server console for messages

## Stopping the Server

Press `Ctrl+C` in the terminal. The server will:
- Close all I2C connections
- Shut down gracefully

## Next Steps

Once the server is running, you can:
1. Connect from the Electron app
2. Control the robot arm
3. Monitor status updates

The server handles all communication automatically!


