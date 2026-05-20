#!/usr/bin/env python3
"""
Example: control the hobby servo on the end effector (ESP32 end tool, bus ID 64).

Before running:
  1. Install dependencies:  pip install -r requirements.txt
  2. Start the server on the Pi:  node server.js   (in raspberry-pi-control-st3215)
  3. Set PI_HOST below to your Pi's IP address
  4. Make sure the end tool is wired and powered; keep fingers clear of the gripper

This script:
  - Connects to the Pi
  - Pings the end tool
  - Enables the hobby servo output
  - Moves through a few angles (0, 90, 180, back to 90)
  - Reads back the reported angle
"""

import time

from robot_arm import RobotArmClient, RobotArmError

# Change this to your Raspberry Pi address
PI_HOST = "192.168.1.97"
PI_PORT = 8080

# Angles to visit, in degrees (0 to 180 for a typical hobby servo)
DEMO_ANGLES = [0, 90, 180, 90]

# Seconds to wait after each move so the servo can reach the angle
WAIT_SECONDS = 1.0


def main():
    arm = RobotArmClient(PI_HOST, PI_PORT)

    try:
        print("Connecting to ws://{0}:{1} ...".format(PI_HOST, PI_PORT))
        arm.connect()
        print("Connected.\n")

        # Step 1: Check the end tool is on the bus
        print("Pinging end tool (ID 64)...")
        ping_result = arm.tool_ping()
        if ping_result.get("type") == "toolPing" and ping_result.get("ok"):
            print("  End tool responded.\n")
        else:
            print("  End tool did not respond.")
            print("  Check power, serial wiring, and that server.js sees the tool at startup.")
            return

        # Step 2: Try to read current servo state (optional — may fail on older ESP32 firmware)
        print("Current end-effector servo state:")
        try:
            state = arm.tool_get_servo_state()
            print("  Enabled: {0}".format(state.get("enabled", "n/a")))
            print("  Angle: {0} degrees".format(state.get("currentAngle")))
            print("  Position (8-bit): {0}\n".format(state.get("currentPosition8bit")))
        except RobotArmError as read_error:
            print("  (could not read — {0})".format(read_error))
            print("  Continuing anyway; move commands may still work.\n")

        # Step 3: Enable the hobby servo output on the ESP32
        print("Enabling end-effector hobby servo...")
        arm.tool_set_servo_enabled(True)
        print("  Enabled.\n")

        # Step 4: Move through demo angles
        print("Moving end-effector servo through demo angles:")
        for angle in DEMO_ANGLES:
            print("  -> {0} degrees".format(angle))
            arm.tool_set_servo_angle(angle)
            time.sleep(WAIT_SECONDS)

        # Step 5: Try to read state again
        print("\nFinal end-effector servo state:")
        try:
            state = arm.tool_get_servo_state()
            print("  Enabled: {0}".format(state.get("enabled", "n/a")))
            print("  Angle: {0} degrees".format(state.get("currentAngle")))
            print("  Position (8-bit): {0}".format(state.get("currentPosition8bit")))
        except RobotArmError as read_error:
            print("  (could not read — {0})".format(read_error))

        print("\nDone.")

    except RobotArmError as error:
        print("Robot arm error:", error)
        print()
        print("If you see 'Unknown command: toolSetServoAngle', update server.js on the Pi")
        print("and restart:  node server.js")
    except Exception as error:
        print("Unexpected error:", error)
    finally:
        arm.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
