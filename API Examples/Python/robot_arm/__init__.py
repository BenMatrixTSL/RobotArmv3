"""
Python client for the Robot Arm WebSocket server (raspberry-pi-control-st3215).
"""

from .client import RobotArmClient, RobotArmError

__all__ = ["RobotArmClient", "RobotArmError"]
