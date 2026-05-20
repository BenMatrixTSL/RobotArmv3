#!/bin/bash

# Installation script for Unique Hostname Script
# This script makes installation easier by doing all the setup steps automatically

echo "=========================================="
echo "Installing Unique Hostname Script"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: Please run this script with sudo"
    echo "Usage: sudo ./install.sh"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if set-hostname.sh exists
if [ ! -f "$SCRIPT_DIR/set-hostname.sh" ]; then
    echo "Error: set-hostname.sh not found in $SCRIPT_DIR"
    exit 1
fi

# Check if set-hostname.service exists
if [ ! -f "$SCRIPT_DIR/set-hostname.service" ]; then
    echo "Error: set-hostname.service not found in $SCRIPT_DIR"
    exit 1
fi

echo "Step 1: Copying script to /usr/local/bin/"
cp "$SCRIPT_DIR/set-hostname.sh" /usr/local/bin/
chmod +x /usr/local/bin/set-hostname.sh
echo "  ✓ Script copied and made executable"

echo ""
echo "Step 2: Copying service file to /etc/systemd/system/"
cp "$SCRIPT_DIR/set-hostname.service" /etc/systemd/system/
echo "  ✓ Service file copied"

echo ""
echo "Step 3: Reloading systemd daemon"
systemctl daemon-reload
echo "  ✓ Systemd reloaded"

echo ""
echo "Step 4: Enabling service"
systemctl enable set-hostname.service
echo "  ✓ Service enabled"

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "The script will run automatically on the next boot."
echo ""
echo "To test it now (optional), run:"
echo "  sudo /usr/local/bin/set-hostname.sh"
echo ""
echo "To see the new hostname after it runs:"
echo "  hostname"
echo ""
echo "To reboot and activate:"
echo "  sudo reboot"
echo ""

