#!/bin/bash

# Script to set a unique hostname on first boot of Raspberry Pi
# Hostname format: FIXEDSTRING-PARTIALMACADDRESS
# Example: RASPBERRYPI-A1B2C3

# Configuration
# Default fixed string (used only if PREFIX_FILE is not configured)
FIXED_STRING="MATRIX_ROBOTARM"
MAC_LENGTH=6  # Number of characters from MAC address to use (last 6 characters)

# Optional external file that provides the fixed prefix string.
# If this file is missing or empty, the script will NOT change hostname
# and will NOT create the completion flag.
# Example file contents: MATRIX_CLASSROOM
# The script checks /boot/firmware first (newer Raspberry Pi OS),
# then /boot (older layouts).
PREFIX_FILE_PRIMARY="/boot/firmware/hostname-prefix.txt"
PREFIX_FILE_FALLBACK="/boot/hostname-prefix.txt"

# Maximum hostname length (per RFC 1123)
MAX_HOSTNAME_LENGTH=63

# File to track if script has already run
FLAG_FILE="/etc/hostname-set-flag"

# Check if script has already run
if [ -f "$FLAG_FILE" ]; then
    echo "Hostname already set. Skipping."
    exit 0
fi

# Read fixed string from external file.
# If both paths are missing or empty, skip without setting hostname/flag.
PREFIX_FILE=""
if [ -f "$PREFIX_FILE_PRIMARY" ]; then
    PREFIX_FILE="$PREFIX_FILE_PRIMARY"
elif [ -f "$PREFIX_FILE_FALLBACK" ]; then
    PREFIX_FILE="$PREFIX_FILE_FALLBACK"
else
    echo "Prefix file not found in either location:"
    echo "  $PREFIX_FILE_PRIMARY"
    echo "  $PREFIX_FILE_FALLBACK"
    echo "Skipping hostname setup."
    exit 0
fi

FIXED_FROM_FILE=$(tr -d '\r\n\t ' < "$PREFIX_FILE" 2>/dev/null)
if [ -z "$FIXED_FROM_FILE" ]; then
    echo "Prefix file is empty ($PREFIX_FILE). Skipping hostname setup."
    exit 0
fi

FIXED_STRING="$FIXED_FROM_FILE"

echo "Setting unique hostname on first boot..."

# Function to get MAC address from a network interface
get_mac_address() {
    # Try to get MAC address from eth0 (Ethernet) first
    if [ -d /sys/class/net/eth0 ]; then
        MAC=$(cat /sys/class/net/eth0/address 2>/dev/null)
        if [ ! -z "$MAC" ]; then
            echo "$MAC"
            return 0
        fi
    fi
    
    # If eth0 doesn't work, try wlan0 (WiFi)
    if [ -d /sys/class/net/wlan0 ]; then
        MAC=$(cat /sys/class/net/wlan0/address 2>/dev/null)
        if [ ! -z "$MAC" ]; then
            echo "$MAC"
            return 0
        fi
    fi
    
    # If neither works, try to find any network interface
    for interface in /sys/class/net/*; do
        if [ -d "$interface" ] && [ "$(basename $interface)" != "lo" ]; then
            MAC=$(cat "$interface/address" 2>/dev/null)
            if [ ! -z "$MAC" ]; then
                echo "$MAC"
                return 0
            fi
        fi
    done
    
    return 1
}

# Get MAC address
MAC_ADDRESS=$(get_mac_address)

if [ -z "$MAC_ADDRESS" ]; then
    echo "Error: Could not find MAC address. Waiting 10 seconds and retrying..."
    sleep 10
    MAC_ADDRESS=$(get_mac_address)
    
    if [ -z "$MAC_ADDRESS" ]; then
        echo "Error: Still could not find MAC address. Exiting."
        exit 1
    fi
fi

echo "Found MAC address: $MAC_ADDRESS"

# Remove colons from MAC address and convert to uppercase
MAC_CLEAN=$(echo "$MAC_ADDRESS" | tr -d ':' | tr '[:lower:]' '[:upper:]')

# Extract the last N characters from MAC address
MAC_LENGTH_ACTUAL=${#MAC_CLEAN}
if [ $MAC_LENGTH -gt $MAC_LENGTH_ACTUAL ]; then
    MAC_LENGTH=$MAC_LENGTH_ACTUAL
fi

MAC_PARTIAL=${MAC_CLEAN: -$MAC_LENGTH}

# Sanitize fixed string so it is safe for Linux hostnames:
# - lowercase only
# - keep only a-z, 0-9, and hyphen
# - collapse repeated hyphens
# - remove leading/trailing hyphens
SANITIZED_FIXED_STRING=$(echo "$FIXED_STRING" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9-]/-/g' | sed 's/--*/-/g' | sed 's/^-*//; s/-*$//')

if [ -z "$SANITIZED_FIXED_STRING" ]; then
    echo "Prefix became empty after sanitization. Skipping hostname setup."
    exit 0
fi

# Truncate prefix if required so final hostname stays within 63 chars.
# Final format: <prefix>-<macpartial>
MAC_PARTIAL_LENGTH=${#MAC_PARTIAL}
MAX_PREFIX_LENGTH=$((MAX_HOSTNAME_LENGTH - 1 - MAC_PARTIAL_LENGTH))
if [ $MAX_PREFIX_LENGTH -le 0 ]; then
    echo "Error: Invalid configuration. MAC portion leaves no room for prefix."
    exit 1
fi

if [ ${#SANITIZED_FIXED_STRING} -gt $MAX_PREFIX_LENGTH ]; then
    SANITIZED_FIXED_STRING=${SANITIZED_FIXED_STRING:0:$MAX_PREFIX_LENGTH}
    SANITIZED_FIXED_STRING=$(echo "$SANITIZED_FIXED_STRING" | sed 's/-*$//')
fi

if [ -z "$SANITIZED_FIXED_STRING" ]; then
    echo "Prefix became empty after truncation. Skipping hostname setup."
    exit 0
fi

# Use sanitized (and possibly truncated) prefix
FIXED_STRING="$SANITIZED_FIXED_STRING"

# Create the hostname
NEW_HOSTNAME="${FIXED_STRING}-${MAC_PARTIAL}"

# Convert to lowercase (hostnames should be lowercase)
NEW_HOSTNAME=$(echo "$NEW_HOSTNAME" | tr '[:upper:]' '[:lower:]')

# Check hostname length (maximum 63 characters per RFC 1123)
HOSTNAME_LENGTH=${#NEW_HOSTNAME}
if [ $HOSTNAME_LENGTH -gt $MAX_HOSTNAME_LENGTH ]; then
    echo "Error: Hostname is too long!"
    echo "  Generated hostname: $NEW_HOSTNAME"
    echo "  Length: $HOSTNAME_LENGTH characters"
    echo "  Maximum allowed: $MAX_HOSTNAME_LENGTH characters"
    echo ""
    echo "To fix this, edit the script and reduce:"
    echo "  - FIXED_STRING length (currently: ${#FIXED_STRING} characters)"
    echo "  - MAC_LENGTH value (currently: $MAC_LENGTH characters)"
    echo ""
    echo "Example: If FIXED_STRING is too long, shorten it."
    echo "Example: If MAC_LENGTH is too high, reduce it."
    exit 1
fi

echo "Setting hostname to: $NEW_HOSTNAME (length: $HOSTNAME_LENGTH characters)"

# Set hostname using hostnamectl (modern method)
hostnamectl set-hostname "$NEW_HOSTNAME"

# Also update /etc/hostname file (for compatibility)
echo "$NEW_HOSTNAME" > /etc/hostname

# Update /etc/hosts file to include the new hostname
# Remove old hostname entries and add new one
sed -i "/127.0.1.1/d" /etc/hosts
echo "127.0.1.1	$NEW_HOSTNAME" >> /etc/hosts

# Create flag file to prevent running again
touch "$FLAG_FILE"

echo "Hostname set successfully to: $NEW_HOSTNAME"
echo "Please reboot for changes to take full effect."

