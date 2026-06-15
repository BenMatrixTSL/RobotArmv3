#!/bin/bash
#
# Quick camera diagnostics on the Raspberry Pi.
#
#   bash check-camera.sh

set -e

echo "=========================================="
echo "Robot Arm — camera diagnostics"
echo "=========================================="
echo ""

echo "1. Camera service status"
systemctl is-active robot-arm-camera.service 2>/dev/null || echo "  not running"
systemctl --no-pager status robot-arm-camera.service 2>/dev/null | head -15 || true
echo ""

echo "2. Web server status"
systemctl is-active robot-arm-web-server.service 2>/dev/null || echo "  not running"
echo ""

echo "3. Video devices"
if command -v v4l2-ctl >/dev/null; then
  v4l2-ctl --list-devices 2>/dev/null || true
else
  ls -l /dev/video* 2>/dev/null || true
fi
echo ""

echo "4. Snapshot on port 8082 (camera service)"
curl -s -o /tmp/robot-arm-cam-test.jpg -w "  HTTP %{http_code}, size %{size_download} bytes\n" \
  --max-time 15 http://127.0.0.1:8082/snapshot || echo "  FAILED"
if [ -f /tmp/robot-arm-cam-test.jpg ]; then
  file /tmp/robot-arm-cam-test.jpg 2>/dev/null || true
fi
echo ""

echo "5. Snapshot via port 80 proxy (web app path)"
curl -s -o /dev/null -w "  HTTP %{http_code}\n" --max-time 15 http://127.0.0.1/camera/snapshot || echo "  FAILED"
echo ""

echo "6. Vision JSON"
curl -s --max-time 5 http://127.0.0.1:8082/vision 2>/dev/null | head -c 400 || echo "  FAILED"
echo ""
echo ""

echo "7. Recent camera logs"
journalctl -u robot-arm-camera.service -n 15 --no-pager 2>/dev/null || true
echo ""

echo "=========================================="
echo "If step 4 shows 'Exec format error' in logs, run:"
echo "  sudo sed -i 's/\\r$//' /opt/RobotArm/electron-app/*.sh"
echo "If step 4 fails: sudo systemctl restart robot-arm-camera.service"
echo "If step 5 fails: sudo bash electron-app/install-web-server-service.sh /opt/RobotArm/electron-app"
echo "Device fix:       sudo ROBOT_ARM_CAMERA_DEVICE=/dev/video0 bash electron-app/install-camera-service.sh /opt/RobotArm/electron-app"
echo "=========================================="
