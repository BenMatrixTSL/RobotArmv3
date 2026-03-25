# Raspberry Pi 5 Setup Guide

## Important: Architecture Overview

The system has **two parts** that run on **different computers**:

```
┌─────────────────────────────────────┐
│   Your Desktop Computer             │
│   (Windows/Mac/Linux)               │
│   ✅ Electron App runs here         │
│   - Nice GUI interface              │
│   - Pendant control                 │
│   - G-code processing               │
└──────────────┬──────────────────────┘
               │ Network Connection
               │ (WebSocket over WiFi/Ethernet)
               ▼
┌─────────────────────────────────────┐
│   Raspberry Pi 5                    │
│   ✅ Server runs here               │
│   - WebSocket server (server.js)    │
│   - I2C communication with PICs     │
│   - No GUI needed                   │
└──────────────┬──────────────────────┘
               │ I2C Bus (wires)
               ▼
         PIC Controllers
```

**You do NOT need to run the Electron app on the Raspberry Pi!**

The Raspberry Pi only needs to run the **server** (a simple Node.js script). The Electron app with the nice GUI runs on your desktop computer and connects to the Pi over the network.

---

## Option 1: Normal Setup (Recommended)

### Raspberry Pi 5: Run the Server Only

The Raspberry Pi runs a simple Node.js server - no GUI needed!

#### Step 1: Enable I2C

```bash
sudo raspi-config
```

Navigate to: **Interface Options** → **I2C** → **Enable**

Then reboot:
```bash
sudo reboot
```

#### Step 1.5: Set Up Network Discovery (Optional but Recommended)

This makes it easier for the Electron app to find your Raspberry Pi automatically.

**Option A: Set a Static Hostname (Recommended)**

```bash
# Set a friendly hostname
sudo hostnamectl set-hostname robot-arm

# Or use the default 'raspberrypi' (already set)
```

**Option B: Install and Configure mDNS (for .local hostname)**

```bash
# Install avahi-daemon (usually already installed)
sudo apt update
sudo apt install -y avahi-daemon

# The service should start automatically
# Your Pi will be accessible as 'raspberrypi.local' or 'robot-arm.local'
```

**Verify mDNS is working:**
```bash
# Check if avahi-daemon is running
sudo systemctl status avahi-daemon

# If not running, start it:
sudo systemctl start avahi-daemon
sudo systemctl enable avahi-daemon
```

**Note:** On Windows, you may need to install "Bonjour Print Services" from Apple for .local hostnames to work. On Mac and Linux, mDNS works out of the box.

#### Step 2: Install Node.js

```bash
# Update package list
sudo apt update

# Install Node.js (version 18.x)
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version
npm --version
```

You should see version numbers printed (e.g., `v18.x.x` and `9.x.x`).

#### Step 3: Transfer Files to Raspberry Pi

You need to copy the `raspberry-pi-control` folder to your Raspberry Pi. You can:

**Option A: Use SCP (from your computer)**
```bash
# From your computer, copy the folder
scp -r raspberry-pi-control pi@YOUR_PI_IP:/home/pi/
```

**Option B: Use USB drive**
- Copy `raspberry-pi-control` folder to USB drive
- Plug into Raspberry Pi
- Copy to `/home/pi/`

**Option C: Use Git (if you have a repository)**
```bash
# On Raspberry Pi
git clone YOUR_REPO_URL
```

**Option D: Use SFTP client** (like FileZilla)
- Connect to Raspberry Pi via SFTP
- Upload the folder

#### Step 4: Install Dependencies

On the Raspberry Pi:
```bash
cd ~/raspberry-pi-control
npm install
```

This installs:
- `i2c-bus` - For I2C communication
- `ws` - For WebSocket server

#### Step 5: Configure I2C Addresses

Edit `server.js` to match your PIC controller addresses:

```bash
nano server.js
```

Find this line (around line 34):
```javascript
const addresses = [0x22, 0x23, 0x24, 0x25];
```

Change to match your PIC controller I2C addresses. If you're not sure, you can detect them later.

#### Step 6: Test I2C Connection

```bash
# Detect I2C devices
sudo i2cdetect -y 1
```

You should see your PIC controllers listed (e.g., `22`, `23`, `24`, `25`).

If you don't see them:
- Check I2C is enabled: `sudo raspi-config`
- Check physical connections
- Make sure PIC controllers are powered

#### Step 7: Start the Server

```bash
sudo npm run server
```

Or:
```bash
sudo node server.js
```

**Note:** You need `sudo` because I2C access requires root privileges.

You should see:
```
Initializing joint controllers...
Joint 1 initialized at address 0x22
Joint 2 initialized at address 0x23
...
Starting WebSocket server on port 8080...
Server listening on port 8080
Waiting for clients to connect...
```

#### Step 8: Get Raspberry Pi IP Address

In a new terminal (or keep the server running and open SSH in another window):

```bash
hostname -I
```

You'll see something like: `192.168.1.100`

**Write this down!** You'll need it for the Electron app.

#### Step 9: Keep Server Running

The server needs to keep running. You have a few options:

**Option A: Run in terminal** (good for testing)
- Just keep the terminal open
- Server runs until you press Ctrl+C

**Option B: Run in background**
```bash
sudo nohup node server.js > server.log 2>&1 &
```

**Option C: Create a systemd service** (best for production)
- See "Running as a Service" section below

---

### Your Desktop Computer: Run the Electron App

The Electron app runs on your desktop computer (not on the Pi).

#### Step 1: Install Node.js on Your Computer

Download and install from: https://nodejs.org/
- Get version 18 or newer (LTS recommended)

#### Step 2: Copy Electron App to Your Computer

Copy the `electron-app` folder to your computer (Windows/Mac/Linux).

#### Step 3: Install Dependencies

```bash
cd electron-app
npm install
```

This installs Electron and WebSocket client libraries.

#### Step 4: Start the Electron App

```bash
npm start
```

The app window will open!

#### Step 5: Connect to Raspberry Pi

1. Enter your Raspberry Pi's IP address (from Step 8 above)
2. Enter port `8080`
3. Click **Connect**

You should see the connection status turn green!

---

## Option 2: Run Electron App ON Raspberry Pi (Not Recommended)

If you really want to run the Electron app on the Raspberry Pi itself (not recommended due to performance, but possible):

### Prerequisites

The Raspberry Pi needs a display, keyboard, and mouse, or you need to set up remote desktop.

### Setup Steps

1. **Install Node.js** (same as Option 1, Step 2)

2. **Install X Server** (if using headless):
   ```bash
   sudo apt install xserver-xorg xinit
   ```

3. **Copy Electron app** to Raspberry Pi:
   ```bash
   cd ~
   # Copy electron-app folder here
   ```

4. **Install Electron app dependencies**:
   ```bash
   cd ~/electron-app
   npm install
   ```
   
   **Warning:** This will take a long time and use a lot of space!

5. **Start both server and app**:
   
   Terminal 1 (Server):
   ```bash
   cd ~/raspberry-pi-control
   sudo node server.js
   ```
   
   Terminal 2 (Electron app):
   ```bash
   cd ~/electron-app
   npm start
   ```

6. **Connect**:
   - Use `127.0.0.1` or `localhost` as the IP address
   - Port `8080`

**Why not recommended:**
- ❌ Electron apps are heavy and slow on Raspberry Pi
- ❌ Poor performance and responsiveness
- ❌ Takes up lots of disk space
- ❌ Overkill - the server is all you need on the Pi

---

## Running Server as a Service (Recommended for Production)

This makes the server start automatically on boot and run in the background.

### Create Systemd Service

Create a service file:
```bash
sudo nano /etc/systemd/system/robot-arm-server.service
```

Add this content (adjust paths as needed):
```ini
[Unit]
Description=Robot Arm WebSocket Server
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/pi/raspberry-pi-control
ExecStart=/usr/bin/node /home/pi/raspberry-pi-control/server.js
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Important:** Adjust paths if your files are in a different location!

### Enable and Start Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable service (starts on boot)
sudo systemctl enable robot-arm-server.service

# Start service now
sudo systemctl start robot-arm-server.service

# Check status
sudo systemctl status robot-arm-server.service
```

### Useful Commands

```bash
# Stop service
sudo systemctl stop robot-arm-server.service

# Start service
sudo systemctl start robot-arm-server.service

# Restart service
sudo systemctl restart robot-arm-server.service

# View logs
sudo journalctl -u robot-arm-server.service -f
```

---

## Quick Setup Checklist

### Raspberry Pi 5 Setup:
- [ ] Enable I2C in `raspi-config`
- [ ] Install Node.js (version 18+)
- [ ] Copy `raspberry-pi-control` folder to Pi
- [ ] Run `npm install` in that folder
- [ ] Configure I2C addresses in `server.js`
- [ ] Test I2C: `sudo i2cdetect -y 1`
- [ ] Start server: `sudo npm run server`
- [ ] Note IP address: `hostname -I`

### Desktop Computer Setup:
- [ ] Install Node.js (version 18+)
- [ ] Copy `electron-app` folder to computer
- [ ] Run `npm install` in that folder
- [ ] Start app: `npm start`
- [ ] Connect using Pi's IP address

---

## Troubleshooting

### Server won't start on Raspberry Pi

**Error: "Permission denied"**
```bash
# Run with sudo
sudo node server.js
```

**Error: "Cannot find module 'i2c-bus'"
```bash
# Install dependencies
cd ~/raspberry-pi-control
npm install
```

**Error: "Cannot find module 'ws'"
```bash
# Install dependencies
npm install
```

**Error: "EACCES: permission denied, open '/dev/i2c-1'"
```bash
# Add user to i2c group (optional, or just use sudo)
sudo usermod -a -G i2c $USER
# Then log out and back in
```

### Can't connect from Electron app

**Check server is running:**
```bash
# On Raspberry Pi
sudo systemctl status robot-arm-server.service
# Or check if process is running
ps aux | grep server.js
```

**Check firewall:**
```bash
# Allow port 8080
sudo ufw allow 8080
# Or disable firewall temporarily for testing
sudo ufw disable
```

**Check network:**
```bash
# On Raspberry Pi - check it's on network
hostname -I

# From desktop - ping the Pi
ping 192.168.1.100
```

**Check server logs:**
```bash
# If running as service
sudo journalctl -u robot-arm-server.service -f

# If running manually
# Check the terminal output
```

---

## Summary

✅ **Raspberry Pi runs:** Node.js server (`server.js`)  
✅ **Desktop runs:** Electron app (`npm start`)  
✅ **They connect:** Over network via WebSocket  

**The Electron app does NOT need to run on the Raspberry Pi!**

Just run the server on the Pi and connect from your desktop computer. This gives you the best performance and a much better user experience.

---

## Need Help?

1. Check the server console for error messages
2. Verify I2C devices: `sudo i2cdetect -y 1`
3. Test I2C directly: `sudo node raspberry-pi-control/test-i2c.js`
4. Check network connectivity: `ping YOUR_PI_IP`
5. Verify firewall settings on Raspberry Pi


