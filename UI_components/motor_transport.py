"""
Motor Transport - Abstract base class for motor communication backends.

Each transport implements the raw CAN protocol commands over a specific
physical link. The controller and API layers build on top of this.

Transports must be thread-safe if used from multiple threads.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Set, Tuple

from motor_types import MotorResponse


class MotorTransport(ABC):
    """Abstract motor communication interface.

    Implementations: CANDirectTransport, SerialTransport, WiFiTransport.
    """

    @abstractmethod
    def connect(self, **kwargs) -> Tuple[bool, str]:
        """Connect to the motor bus.

        Returns:
            (success, message) tuple.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect and release resources."""

    @abstractmethod
    def send_torque(self, motor_id: int, iq: int) -> MotorResponse:
        """Send torque command (0xA1).

        Args:
            motor_id: Motor ID (1 or 2).
            iq: Torque current in iq units. Caller should clamp beforehand.

        Returns:
            MotorResponse with motor feedback from the command response.
        """

    @abstractmethod
    def send_position(self, motor_id: int, angle_deg: float,
                      max_speed: int = 700) -> MotorResponse:
        """Send position command (0xA4 multi-turn).

        Args:
            motor_id: Motor ID (1 or 2).
            angle_deg: Target angle in degrees (supports multi-turn).
            max_speed: Maximum speed in dps.

        Returns:
            MotorResponse with motor feedback.

        Raises:
            NotImplementedError if the transport does not support position.
        """

    @abstractmethod
    def read_status(self, motor_id: int) -> MotorResponse:
        """Read motor status (0x9A + 0x9C + 0x92).

        Returns:
            MotorResponse with all status fields populated.
        """

    @abstractmethod
    def send_stop(self, motor_id: int) -> MotorResponse:
        """Stop a motor (0x81).

        Args:
            motor_id: Motor ID (1 or 2), or 0 for all motors.
        """

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the transport is currently connected."""

    @property
    @abstractmethod
    def transport_name(self) -> str:
        """Human-readable transport name (e.g. 'CAN Direct', 'Serial')."""

    @property
    @abstractmethod
    def supported_commands(self) -> Set[str]:
        """Set of supported command types.

        Possible values: {"torque", "position", "status", "stop"}
        """
