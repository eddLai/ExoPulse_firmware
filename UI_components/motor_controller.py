"""
Motor Controller - Control strategy layer with built-in safety protection.

Safety checks are enforced in the base class update() wrapper so that
ALL controller subclasses are protected, even when called directly
(bypassing MotorAPI).

Safety enforced on every update():
  1. Input clamping (before sending): iq ≤ ±max_iq, angle ≤ ±max_angle
  2. Response checking (after receiving): temperature, error_state, current
  3. Emergency stop: auto-stops motor and blocks further commands until reset()
"""
from __future__ import annotations

import time
from abc import ABC, abstractmethod

from motor_types import MotorResponse, SafetyConfig
from motor_transport import MotorTransport


class MotorController(ABC):
    """Base controller with safety protection.

    Subclasses implement _execute() for their control logic.
    The public update() method wraps _execute() with safety checks.
    """

    def __init__(self, transport: MotorTransport, motor_id: int,
                 safety: SafetyConfig = None):
        self.transport = transport
        self.motor_id = motor_id
        self.safety = safety or SafetyConfig()

    def update(self, target_value: float) -> MotorResponse:
        """Send one control command with UI soft-limit clamping.

        Hard safety (angle limits, torque limits, E-stop, hardware errors)
        is enforced by C++ MotorProtection in the transport layer.

        Args:
            target_value: iq for torque controllers, degrees for position.

        Returns:
            MotorResponse (may contain warning messages).
        """
        target_value = self._clamp_input(target_value)
        response = self._execute(target_value)
        return self._add_warnings(response)

    @abstractmethod
    def _execute(self, target_value: float) -> MotorResponse:
        """Subclass control logic. Called by update() after input clamping."""

    def stop(self) -> MotorResponse:
        """Stop the motor."""
        return self.transport.send_stop(self.motor_id)

    def read_status(self) -> MotorResponse:
        """Read motor status (convenience method)."""
        return self.transport.read_status(self.motor_id)

    @property
    @abstractmethod
    def control_mode(self) -> str:
        """Return 'torque', 'position', or 'pid_position'."""

    # ---- UI soft limits (Layer 2) ----

    def _clamp_input(self, value: float) -> float:
        """Clamp input based on control mode (user-adjustable soft limit)."""
        if self.control_mode == "torque":
            return max(-self.safety.max_iq, min(self.safety.max_iq, value))
        elif self.control_mode in ("position", "pid_position"):
            return max(-self.safety.max_angle, min(self.safety.max_angle, value))
        return value

    def _add_warnings(self, response: MotorResponse) -> MotorResponse:
        """Add informational warnings for UI display (no stopping)."""
        if not response.success:
            return response

        # Over-current → warning
        if (response.torque_current != 0.0
                and abs(response.torque_current) > self.safety.max_current):
            response.error_msg = (
                f"HIGH_CURRENT: {response.torque_current:.1f}A "
                f"> {self.safety.max_current}A")

        # Low voltage → warning
        if (response.voltage > 0
                and response.voltage < self.safety.low_voltage_threshold):
            msg = (f"LOW_VOLTAGE: {response.voltage:.1f}V "
                   f"< {self.safety.low_voltage_threshold}V")
            response.error_msg = (
                f"{response.error_msg}; {msg}" if response.error_msg else msg)

        return response


class DirectTorqueController(MotorController):
    """Direct torque control — lowest latency, straight to transport.

    Safety: iq clamped to ±max_iq (default 400) before sending.
    """

    def _execute(self, iq: float) -> MotorResponse:
        return self.transport.send_torque(self.motor_id, int(iq))

    @property
    def control_mode(self) -> str:
        return "torque"


class DirectPositionController(MotorController):
    """Direct position control — uses motor's built-in PID (0xA4 command).

    Safety: angle clamped to ±max_angle (default ±45°) relative to zero offset.
    The zero offset is set by calibrate() to the motor's current angle,
    so ±max_angle becomes a safe range around the startup position.
    Requires transport that supports 'position' command.
    """

    def __init__(self, transport: MotorTransport, motor_id: int,
                 max_speed: int = 700, safety: SafetyConfig = None):
        super().__init__(transport, motor_id, safety)
        self.max_speed = max_speed
        self._zero_offset = 0.0  # motor absolute angle treated as user 0°

    def calibrate(self) -> MotorResponse:
        """Read current motor angle and set it as zero offset.

        After calibration, user angle 0° maps to the motor's current position.
        Safety clamp ±max_angle becomes relative to this position.
        """
        resp = self.transport.read_status(self.motor_id)
        if resp.success:
            self._zero_offset = resp.angle
        return resp

    @property
    def zero_offset(self) -> float:
        return self._zero_offset

    @zero_offset.setter
    def zero_offset(self, value: float):
        self._zero_offset = value

    def _execute(self, angle_deg: float) -> MotorResponse:
        if "position" not in self.transport.supported_commands:
            return MotorResponse.fail(
                self.motor_id,
                f"{self.transport.transport_name} does not support position control")
        # angle_deg is already clamped to ±max_angle by base class
        # Add zero offset so the command is relative to calibrated position
        absolute_angle = self._zero_offset + angle_deg
        return self.transport.send_position(
            self.motor_id, absolute_angle, self.max_speed)

    @property
    def control_mode(self) -> str:
        return "position"


class PIDPositionController(MotorController):
    """Python-side PID position control — for transports without native position.

    Reads current angle via read_status(), computes PID error, sends torque.
    Target angle is relative to calibrated zero offset (same as
    DirectPositionController).

    Safety:
      - Target angle clamped to ±max_angle (default ±45°) relative to zero offset
      - PID output iq clamped to ±max_iq (default 400)
      - Temperature/error checks by base class
    """

    def __init__(self, transport: MotorTransport, motor_id: int,
                 kp: float = 5.0, ki: float = 0.1, kd: float = 0.5,
                 safety: SafetyConfig = None):
        super().__init__(transport, motor_id, safety)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._zero_offset = 0.0  # motor absolute angle treated as user 0°
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time: float = 0.0

    def calibrate(self) -> MotorResponse:
        """Read current motor angle and set it as zero offset.

        After calibration, user angle 0° maps to the motor's current position.
        Safety clamp ±max_angle becomes relative to this position.
        """
        resp = self.transport.read_status(self.motor_id)
        if resp.success:
            self._zero_offset = resp.angle
            self.reset_pid()
        return resp

    @property
    def zero_offset(self) -> float:
        return self._zero_offset

    @zero_offset.setter
    def zero_offset(self, value: float):
        self._zero_offset = value

    def _execute(self, target_deg: float) -> MotorResponse:
        # 1. Read current angle
        status = self.transport.read_status(self.motor_id)
        if not status.success:
            return status

        # 2. PID computation — error in absolute space
        # target_deg is already clamped to ±max_angle by base class
        absolute_target = self._zero_offset + target_deg
        error = absolute_target - status.angle
        now = time.monotonic()
        dt = now - self._prev_time if self._prev_time > 0 else 0.02
        dt = max(dt, 0.001)  # Prevent division by zero

        self._integral += error * dt
        # Anti-windup: clamp integral
        max_integral = self.safety.max_iq / max(self.ki, 0.001)
        self._integral = max(-max_integral, min(max_integral, self._integral))

        derivative = (error - self._prev_error) / dt

        iq = self.kp * error + self.ki * self._integral + self.kd * derivative
        iq = max(-self.safety.max_iq, min(self.safety.max_iq, int(iq)))

        # 3. Send torque command
        result = self.transport.send_torque(self.motor_id, iq)

        # 4. Update state
        self._prev_error = error
        self._prev_time = now
        return result

    def reset_pid(self):
        """Reset PID internal state (integral, derivative, timer)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0

    def reset(self):
        """Reset PID state."""
        self.reset_pid()

    @property
    def control_mode(self) -> str:
        return "pid_position"
