# Unique Hostname Script for Raspberry Pi

This script automatically sets a unique hostname on the first boot of your Raspberry Pi. The hostname format is: **FIXEDSTRING-PARTIALMACADDRESS**

## Example
- Fixed String: `RASPBERRYPI`
- MAC Address: `B8:27:EB:A1:B2:C3`
- Result Hostname: `raspberrypi-a1b2c3`

## Files Included

1. **set-hostname.sh** - The main script that sets the hostname
2. **set-hostname.service** - Systemd service file to run the script on boot
3. **install.sh** - Easy installation script (recommended)
4. **README.md** - This file with instructions

## Quick Installation (Recommended)

### Easy Method - Use the Install Script

1. Copy all files to your Raspberry Pi
2. Make the install script executable:
   ```bash
   chmod +x install.sh
   ```
3. Run the installation:
   ```bash
   sudo ./install.sh
   ```
4. Reboot:
   ```bash
   sudo reboot
   ```

That's it! The script will run automatically on first boot.

## Manual Installation

If you prefer to install manually:

### Step 1: Copy Files to Raspberry Pi

Copy all files to your Raspberry Pi. You can use SCP, USB drive, or copy-paste the contents.

### Step 2: Make Script Executable

On your Raspberry Pi, open a terminal and run:

```bash
sudo chmod +x set-hostname.sh
```

### Step 3: Copy Script to System Directory

```bash
sudo cp set-hostname.sh /usr/local/bin/
```

### Step 4: Copy Service File

```bash
sudo cp set-hostname.service /etc/systemd/system/
```

### Step 5: Enable the Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable set-hostname.service
```

### Step 6: Reboot

```bash
sudo reboot
```

After reboot, the script will run automatically and set your unique hostname.

## Customization

You can customize the script by editing `/usr/local/bin/set-hostname.sh`:

- **FIXED_STRING**: Change `"RASPBERRYPI"` to your desired prefix
- **MAC_LENGTH**: Change `6` to use more or fewer characters from the MAC address
- **PREFIX_FILE**: External file that contains the fixed prefix string (script checks `/boot/firmware/hostname-prefix.txt` first, then `/boot/hostname-prefix.txt`)

**Important**: The script automatically validates that the hostname doesn't exceed 63 characters (the maximum allowed by Linux). If your custom settings create a hostname that's too long, the script will show an error message with suggestions on how to fix it.

### Using one image for many device types

You can keep the same script on all devices and control the hostname prefix using the external file:

1. Create `/boot/firmware/hostname-prefix.txt` (recommended on newer Raspberry Pi OS)
   - Or use `/boot/hostname-prefix.txt` on older setups
2. Put only the prefix text in it (for example: `MATRIX_CLASSROOM`)
3. Reboot

If the file is missing or empty, the script now **does nothing**:
- It does not change hostname
- It does not create `/etc/hostname-set-flag`

This lets you prepare a generic image and enable hostname setup only when that file is added.

### Prefix sanitization and truncation

When the prefix is read from the external file, the script now:

1. Converts it to lowercase
2. Replaces invalid hostname characters with `-`
3. Collapses repeated `-`
4. Removes leading/trailing `-`
5. Truncates the prefix if needed so `<prefix>-<macpart>` stays within 63 characters

If the prefix becomes empty after sanitization (or after truncation cleanup), the script skips hostname setup and does not set the completion flag.

## How It Works

1. The script runs on first boot (after network is ready)
2. It finds the MAC address from your network interface (eth0 or wlan0)
3. It extracts the last 6 characters (or your specified length)
4. It combines the fixed string with the MAC portion
5. It sets the hostname using `hostnamectl`
6. It creates a flag file so it only runs once

## Verification

After reboot, check your hostname:

```bash
hostname
```

Or:

```bash
hostnamectl
```

## Troubleshooting

### Script doesn't run
- Check if the service is enabled: `sudo systemctl status set-hostname.service`
- Check logs: `sudo journalctl -u set-hostname.service`

### MAC address not found
- The script waits 10 seconds and retries if MAC address isn't found immediately
- Make sure your network interface is connected

### Hostname not changed
- Make sure you rebooted after installation
- Check if the flag file exists: `ls /etc/hostname-set-flag`
- If flag file exists, delete it to run again: `sudo rm /etc/hostname-set-flag`

## Manual Testing

To test the script without rebooting:

```bash
sudo /usr/local/bin/set-hostname.sh
```

Then check the hostname:

```bash
hostname
```

Note: You may need to restart your terminal or SSH session to see the new hostname.

