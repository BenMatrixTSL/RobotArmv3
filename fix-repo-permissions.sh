#!/bin/bash
#
# Fix git "dubious ownership" and "Permission denied" on /opt/RobotArm after sudo git pull.
# Run once on the Pi:
#
#   chmod +x fix-repo-permissions.sh
#   sudo ./fix-repo-permissions.sh
#
# Optional: sudo ./fix-repo-permissions.sh /opt/RobotArm

set -e

REPO_DIR="${1:-/opt/RobotArm}"

if [ "$EUID" -ne 0 ]; then
    echo "Error: run with sudo"
    echo "  sudo ./fix-repo-permissions.sh"
    exit 1
fi

if [ -n "$SUDO_USER" ]; then
    REPO_USER="$SUDO_USER"
else
    REPO_USER="pi"
fi

if [ ! -d "$REPO_DIR" ]; then
    echo "Error: folder not found: $REPO_DIR"
    exit 1
fi

if ! id "$REPO_USER" &>/dev/null; then
    echo "Error: user '$REPO_USER' does not exist."
    exit 1
fi

echo "Fixing ownership of $REPO_DIR for user $REPO_USER ..."
chown -R "$REPO_USER:$REPO_USER" "$REPO_DIR"

echo "Adding safe.directory for git ..."
sudo -u "$REPO_USER" git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true

echo ""
echo "Done. From now on, update the repo as $REPO_USER (do not use sudo git pull):"
echo "  cd $REPO_DIR"
echo "  git pull origin main"
