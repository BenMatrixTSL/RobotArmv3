"""
Robot Arm WebSocket client.

Talks to the Node.js server on the Raspberry Pi (default port 8080).
Protocol matches the Electron app: JSON messages with optional requestId.
"""

import json
import threading
from typing import Any, Dict, List, Optional

import websocket


class RobotArmError(Exception):
    """Raised when the server returns an error or the request times out."""


class RobotArmClient:
    """
    Simple client for the robot arm WebSocket API.

    Example:
        arm = RobotArmClient("192.168.1.100", 8080)
        arm.connect()
        status = arm.get_status()
        arm.move_joint(1, 45.0)
        arm.disconnect()
    """

    def __init__(self, host: str, port: int = 8080):
        self.host = host
        self.port = port
        self.ws = None
        self.connected = False
        self._running = False
        self._receive_thread = None
        self._next_request_id = 1
        self._pending = {}
        self._lock = threading.Lock()

    def connect(self, timeout: float = 10.0) -> None:
        """
        Open a WebSocket connection to the Pi server.

        Args:
            timeout: Seconds to wait while connecting.
        """
        if self.connected:
            return

        url = "ws://{0}:{1}".format(self.host, self.port)
        self.ws = websocket.create_connection(url, timeout=timeout)
        self._running = True
        self.connected = True

        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            name="RobotArmReceive",
            daemon=True,
        )
        self._receive_thread.start()

    def disconnect(self) -> None:
        """Close the WebSocket connection."""
        self._running = False
        self.connected = False

        with self._lock:
            for request_id, pending in list(self._pending.items()):
                pending["error"] = RobotArmError("Connection closed")
                pending["event"].set()
            self._pending.clear()

        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
            self.ws = None

    def request(
        self,
        command: str,
        timeout: float = 5.0,
        **params: Any,
    ) -> Dict[str, Any]:
        """
        Send a command and wait for the matching response (by requestId).

        Args:
            command: Server command name (e.g. "getStatus", "moveJoint").
            timeout: Seconds to wait for a reply.
            **params: Extra JSON fields (joint, angle, speed, etc.).

        Returns:
            Parsed JSON response from the server.

        Raises:
            RobotArmError: Not connected, timeout, or server error response.
        """
        if not self.connected or self.ws is None:
            raise RobotArmError("Not connected to robot arm server")

        request_id = self._next_request_id
        self._next_request_id += 1

        message = {"command": command, "requestId": request_id}
        message.update(params)

        event = threading.Event()
        pending = {"event": event, "result": None, "error": None}

        with self._lock:
            self._pending[request_id] = pending

        try:
            self.ws.send(json.dumps(message))
        except Exception as send_error:
            with self._lock:
                self._pending.pop(request_id, None)
            raise RobotArmError("Failed to send command: {0}".format(command)) from send_error

        if not event.wait(timeout):
            with self._lock:
                self._pending.pop(request_id, None)
            raise RobotArmError("Request timed out: {0}".format(command))

        if pending["error"] is not None:
            raise pending["error"]

        result = pending["result"]
        if result is None:
            raise RobotArmError("No response received for: {0}".format(command))

        if result.get("type") == "error":
            raise RobotArmError(result.get("message", "Unknown server error"))

        return result

    def _receive_loop(self) -> None:
        """Background thread: read WebSocket messages and match requestId."""
        while self._running and self.ws is not None:
            try:
                raw = self.ws.recv()
            except Exception:
                if self._running:
                    self._fail_all_pending(RobotArmError("Connection lost"))
                break

            if not raw:
                continue

            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            self._handle_message(data)

    def _handle_message(self, data: Dict[str, Any]) -> None:
        request_id = data.get("requestId")
        if request_id is not None:
            with self._lock:
                pending = self._pending.pop(request_id, None)
            if pending is not None:
                pending["result"] = data
                pending["event"].set()
                return

        # Unsolicited messages (e.g. welcome "connected") are ignored here.
        # You can extend this later with callbacks if needed.

    def _fail_all_pending(self, error: RobotArmError) -> None:
        with self._lock:
            for pending in self._pending.values():
                pending["error"] = error
                pending["event"].set()
            self._pending.clear()

    # ----- Joint commands -----

    def get_joint_configs(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Return how many servos were discovered on the bus."""
        return self.request("getJointConfigs", timeout=timeout)

    def get_status(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Return current status for all joints."""
        return self.request("getStatus", timeout=timeout)

    def move_joint(
        self,
        joint: int,
        angle: float,
        speed: int = 1500,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Move one joint to an angle in degrees."""
        return self.request(
            "moveJoint",
            timeout=timeout,
            joint=joint,
            angle=angle,
            speed=speed,
        )

    def stop_joint(self, joint: int, timeout: float = 5.0) -> Dict[str, Any]:
        """Stop one joint immediately."""
        return self.request("stopJoint", timeout=timeout, joint=joint)

    def stop_all_joints(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Stop every joint immediately."""
        return self.request("stopAllJoints", timeout=timeout)

    def set_speed(self, joint: int, speed: int, timeout: float = 5.0) -> Dict[str, Any]:
        """Set default move speed for one joint (steps per second)."""
        return self.request("setSpeed", timeout=timeout, joint=joint, speed=speed)

    def set_speed_all(self, speed: int, timeout: float = 5.0) -> Dict[str, Any]:
        """Set default move speed for all joints."""
        return self.request("setSpeedAll", timeout=timeout, speed=speed)

    def set_torque_all(self, enabled: bool, timeout: float = 5.0) -> Dict[str, Any]:
        """Enable or disable torque on all joints."""
        return self.request("setTorqueAll", timeout=timeout, enabled=enabled)

    def rescan_servos(self, timeout: float = 10.0) -> Dict[str, Any]:
        """Scan the serial bus again for servos."""
        return self.request("rescanServos", timeout=timeout)

    # ----- End tool commands -----

    def tool_ping(self, timeout: float = 5.0) -> Dict[str, Any]:
        """Check if the end tool (ID 64) responds."""
        return self.request("toolPing", timeout=timeout)

    def tool_read_currents(self, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Read PWM1, PWM2, and servo shunt currents (milliamps).

        Response fields: pwm1CurrentRaw, pwm2CurrentRaw, servoCurrentRaw
        """
        return self.request("toolReadCurrents", timeout=timeout)

    def tool_read_adc(self, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Read ADC0 and ADC1 raw counts and millivolts.

        Response fields: adc0Raw, adc1Raw, adc0mV, adc1mV
        """
        return self.request("toolReadAdc", timeout=timeout)

    def tool_set_pwm(
        self,
        pwm1_duty: int,
        pwm2_duty: int,
        enable1: bool = True,
        enable2: bool = True,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Set end-tool PWM outputs (duty 0-255)."""
        return self.request(
            "toolSetPwm",
            timeout=timeout,
            pwm1Duty=pwm1_duty,
            pwm2Duty=pwm2_duty,
            enable1=enable1,
            enable2=enable2,
        )

    def tool_set_servo_enabled(
        self,
        enabled: bool,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Enable or disable the hobby servo on the end effector (ESP32 tool node)."""
        return self.request(
            "toolSetServoEnabled",
            timeout=timeout,
            enabled=enabled,
        )

    def tool_set_servo_angle(
        self,
        angle: float,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Set end-effector hobby servo angle in degrees (0 to 180).

        The ESP32 end tool drives a standard hobby servo output.
        """
        return self.request(
            "toolSetServoAngle",
            timeout=timeout,
            angle=angle,
        )

    def tool_set_servo_position(
        self,
        position: int,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Set end-effector hobby servo position as 8-bit value (0 to 255).

        0 is one end of travel, 255 is the other (pulse width mapped by firmware).
        """
        return self.request(
            "toolSetServoPosition",
            timeout=timeout,
            position=position,
        )

    def tool_get_servo_state(self, timeout: float = 5.0) -> Dict[str, Any]:
        """
        Read current hobby servo state from the end tool.

        Response fields: currentPosition8bit, currentAngle
        """
        return self.request("toolGetServoState", timeout=timeout)

    # ----- Kinematics (optional, if URDF loaded on server) -----

    def kinematics_load_urdf(self, urdf_xml: str, timeout: float = 8.0) -> Dict[str, Any]:
        """Load URDF text into the server-side kinematics solver."""
        return self.request("kinematicsLoadURDF", timeout=timeout, urdfXml=urdf_xml)

    def kinematics_forward(
        self,
        joint_angles: List[float],
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """Run forward kinematics for the given joint angles (degrees)."""
        return self.request(
            "kinematicsForwardKinematics",
            timeout=timeout,
            jointAngles=joint_angles,
        )

    def kinematics_inverse(
        self,
        x: float,
        y: float,
        z: float,
        joint_angles_guess: Optional[List[float]] = None,
        timeout: float = 10.0,
    ) -> Dict[str, Any]:
        """Run inverse kinematics toward a target XYZ position (mm)."""
        params = {"targetPose": {"x": x, "y": y, "z": z}}
        if joint_angles_guess is not None:
            params["initialAngles"] = joint_angles_guess
        return self.request("kinematicsInverseKinematics", timeout=timeout, **params)
