"""
Motor API - Unified entry point for motor control.

Manages multiple motors, each with its own controller and shared transport.
Provides high-level operations and controller/transport hot-swapping.

Usage:
    from motor_api import MotorAPI
    from transports.can_direct import CANDirectTransport
    from motor_controller import DirectTorqueController

    transport = CANDirectTransport()
    transport.connect()

    api = MotorAPI(transport)
    api.set_controller(1, DirectTorqueController)
    api.set_controller(2, DirectTorqueController)

    response = api.set_torque(1, 200)
    print(response.angle, response.temperature)

    api.stop_all()
"""
from __future__ import annotations

from typing import Dict, Optional, Type

from motor_types import MotorResponse, SafetyConfig
from motor_transport import MotorTransport
from motor_controller import (
    MotorController,
    DirectTorqueController,
    DirectPositionController,
    PIDPositionController,
)


class MotorAPI:
    """Unified motor control interface managing multiple motors."""

    def __init__(self, transport: MotorTransport,
                 safety: SafetyConfig = None):
        """
        Args:
            transport: Communication backend (CAN/Serial/WiFi).
            safety: Safety config applied to all controllers.
        """
        self._transport = transport
        self._safety = safety or SafetyConfig()
        self._controllers: Dict[int, MotorController] = {}

    # ---- Transport management ----

    @property
    def transport(self) -> MotorTransport:
        return self._transport

    def set_transport(self, transport: MotorTransport):
        """Hot-swap transport. Re-creates all controllers with new transport."""
        self._transport = transport
        for motor_id, ctrl in list(self._controllers.items()):
            ctrl_class = type(ctrl)
            kwargs = {}
            if isinstance(ctrl, DirectPositionController):
                kwargs['max_speed'] = ctrl.max_speed
            elif isinstance(ctrl, PIDPositionController):
                kwargs['kp'] = ctrl.kp
                kwargs['ki'] = ctrl.ki
                kwargs['kd'] = ctrl.kd
            self._controllers[motor_id] = ctrl_class(
                transport, motor_id, safety=self._safety, **kwargs)

    @property
    def connected(self) -> bool:
        return self._transport.is_connected

    # ---- Controller management ----

    def set_controller(self, motor_id: int,
                       controller_class: Type[MotorController],
                       **kwargs):
        """Set the controller type for a motor.

        Args:
            motor_id: Motor ID (1 or 2).
            controller_class: One of DirectTorqueController,
                DirectPositionController, PIDPositionController.
            **kwargs: Extra args passed to controller constructor
                (e.g. max_speed, kp, ki, kd).
        """
        self._controllers[motor_id] = controller_class(
            self._transport, motor_id, safety=self._safety, **kwargs)

    def get_controller(self, motor_id: int) -> Optional[MotorController]:
        """Get the controller for a motor (or None)."""
        return self._controllers.get(motor_id)

    # ---- Motor operations ----

    def set_torque(self, motor_id: int, iq: int) -> MotorResponse:
        """Send torque command via the motor's controller.

        If the motor has a torque controller, calls update(iq).
        Otherwise sends directly via transport (still clamped by safety).
        """
        ctrl = self._controllers.get(motor_id)
        if ctrl and ctrl.control_mode == "torque":
            return ctrl.update(float(iq))
        # Fallback: direct transport call with safety clamping
        iq = max(-self._safety.max_iq, min(self._safety.max_iq, iq))
        return self._transport.send_torque(motor_id, iq)

    def set_position(self, motor_id: int, angle_deg: float,
                     max_speed: int = 700) -> MotorResponse:
        """Send position command via the motor's controller.

        Works with DirectPositionController or PIDPositionController.
        """
        ctrl = self._controllers.get(motor_id)
        if ctrl and ctrl.control_mode in ("position", "pid_position"):
            return ctrl.update(angle_deg)
        # Fallback: direct transport if it supports position
        if "position" in self._transport.supported_commands:
            angle_deg = max(-self._safety.max_angle,
                            min(self._safety.max_angle, angle_deg))
            return self._transport.send_position(motor_id, angle_deg, max_speed)
        return MotorResponse.fail(
            motor_id, "No position controller configured and transport "
                      "does not support position")

    def read_status(self, motor_id: int) -> MotorResponse:
        """Read motor status."""
        return self._transport.read_status(motor_id)

    def stop(self, motor_id: int) -> MotorResponse:
        """Stop a single motor."""
        ctrl = self._controllers.get(motor_id)
        if ctrl:
            return ctrl.stop()
        return self._transport.send_stop(motor_id)

    def stop_all(self):
        """Stop all known motors."""
        for motor_id in list(self._controllers.keys()):
            self.stop(motor_id)
        # Also stop via transport in case there are unmanaged motors
        self._transport.send_stop(0)

    def reset(self, motor_id: int):
        """Reset safety stop for a motor's controller."""
        ctrl = self._controllers.get(motor_id)
        if ctrl:
            ctrl.reset()

    def reset_all(self):
        """Reset safety stop for all controllers."""
        for ctrl in self._controllers.values():
            ctrl.reset()

    # ---- Info ----

    def get_info(self) -> dict:
        """Return current configuration summary."""
        return {
            'transport': self._transport.transport_name,
            'connected': self._transport.is_connected,
            'supported_commands': list(self._transport.supported_commands),
            'motors': {
                mid: {
                    'controller': type(ctrl).__name__,
                    'control_mode': ctrl.control_mode,
                    'safety_stopped': ctrl._stopped,
                }
                for mid, ctrl in self._controllers.items()
            },
            'safety': {
                'max_iq': self._safety.max_iq,
                'max_angle': self._safety.max_angle,
                'max_temperature': self._safety.max_temperature,
            },
        }
