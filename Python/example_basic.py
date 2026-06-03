#!/usr/bin/env python3
"""
Basic example: connect to the Raspberry Pi robot arm server and read status.

Before running:
  1. Install dependencies:  pip install -r requirements.txt
  2. Start the server on the Pi:  node server.js   (in raspberry-pi-control-st3215)
  3. Set PI_HOST below to your Pi's IP address (or hostname)
"""

from robot_arm import RobotArmClient, RobotArmError

# Change this to your Raspberry Pi address
PI_HOST = "192.168.1.112"
PI_PORT = 8080


def print_tool_command_help(error):
    """Print a short hint when the Pi server is missing end-tool commands."""
    message = str(error)
    print("  (skipped — {0})".format(message))
    if "Unknown command" in message and "toolRead" in message:
        print()
        print("  >>> The Pi is running an OLD copy of server.js.")
        print("  >>> Copy the latest raspberry-pi-control-st3215 folder to the Pi,")
        print("  >>> then restart:  node server.js")
        print("  >>> (or use the Electron app: update server from git, then restart)")


def main():
    arm = RobotArmClient(PI_HOST, PI_PORT)

    try:
        print("Connecting to ws://{0}:{1} ...".format(PI_HOST, PI_PORT))
        arm.connect()
        print("Connected.\n")

        # How many servos are on the bus?
        configs = arm.get_joint_configs()
        print(
            "Servos found: {0} of {1}".format(
                configs.get("count", 0),
                configs.get("total", 0),
            )
        )

        # Read each joint angle
        status = arm.get_status()
        joints = status.get("joints", [])
        print("\nJoint angles:")
        for joint_info in joints:
            joint_num = joint_info.get("joint", "?")
            angle = joint_info.get("angleDegrees", 0)
            available = joint_info.get("available", False)
            if available:
                print("  Joint {0}: {1:.1f} degrees".format(joint_num, angle))
            else:
                print("  Joint {0}: (not available)".format(joint_num))

        # Try reading end-tool sensors (optional — fails if tool is not connected)
        print("\nEnd tool currents (mA):")
        try:
            currents = arm.tool_read_currents()
            print("  PWM1:  {0}".format(currents.get("pwm1CurrentRaw")))
            print("  PWM2:  {0}".format(currents.get("pwm2CurrentRaw")))
            print("  Servo: {0}".format(currents.get("servoCurrentRaw")))
        except RobotArmError as tool_error:
            print_tool_command_help(tool_error)

        print("\nEnd tool ADC:")
        try:
            adc = arm.tool_read_adc()
            print("  ADC0 raw: {0}  ({1} mV)".format(adc.get("adc0Raw"), adc.get("adc0mV")))
            print("  ADC1 raw: {0}  ({1} mV)".format(adc.get("adc1Raw"), adc.get("adc1mV")))
        except RobotArmError as tool_error:
            print_tool_command_help(tool_error)

        # Uncomment to move joint 1 (use with care — arm must have clearance):
        # print("\nMoving joint 1 to 0 degrees...")
        # arm.move_joint(1, 0.0)
        # print("Move command sent.")

    except RobotArmError as error:
        print("Robot arm error:", error)
    except Exception as error:
        print("Unexpected error:", error)
    finally:
        arm.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()
