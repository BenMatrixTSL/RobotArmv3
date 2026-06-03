# Quick Start Guide

Get up and running in 5 minutes!

## Step 1: Enable I2C on Raspberry Pi

```bash
sudo raspi-config
```

Then: **Interface Options** → **I2C** → **Enable** → **OK** → **Finish**

Reboot:
```bash
sudo reboot
```

## Step 2: Install Node.js (if not already installed)

```bash
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs
```

## Step 3: Install Dependencies

In this directory:
```bash
npm install
```

## Step 4: Test Connection

```bash
# Check if your PIC controller is detected (should show address 0x22)
# Uses software I2C on bus 3 (GPIO 17=SDA pin 11, GPIO 27=SCL pin 13)
sudo i2cdetect -y 3

# Run the test script
sudo node test-i2c.js
```

## Step 5: Try the Simple Example

```bash
sudo node example-simple.js
```

## That's It!

If the test works, you're ready to control your robot arm joints!

### Next Steps:
- Read `README.md` for detailed documentation
- Modify `example-simple.js` to control your robot
- See all available functions in `robotArmI2C.js`

## Common Issues

**"Permission denied"** → Run with `sudo`

**"Cannot find module"** → Run `npm install`

**"No I2C devices"** → Check connections and I2C enable

**"Failed to open bus"** → Make sure bus exists: `ls /dev/i2c-*`


