"""
Sensor Transport - Abstract base class for sensor communication backends.

Parallel to motor_transport.py but simpler: sensors are read-only,
no commands to send.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

from sensor_types import SensorReading


class SensorTransport(ABC):
    """Abstract sensor communication interface.

    Implementations: IMUI2CTransport, IMUSerialTransport, etc.
    """

    @abstractmethod
    def connect(self, **kwargs) -> Tuple[bool, str]:
        """Connect to the sensor.

        Returns:
            (success, message) tuple.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect and release resources."""

    @abstractmethod
    def read(self) -> SensorReading:
        """Read one sample from the sensor.

        Returns:
            SensorReading with the appropriate typed data.
        """

    @abstractmethod
    def read_batch(self, n_samples: int) -> List[SensorReading]:
        """Read multiple samples.

        Args:
            n_samples: Number of samples to collect.

        Returns:
            List of SensorReading.
        """

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the sensor is currently connected."""

    @property
    @abstractmethod
    def sensor_type(self) -> str:
        """Sensor type identifier (e.g. 'imu', 'emg')."""

    @property
    @abstractmethod
    def transport_name(self) -> str:
        """Human-readable transport name (e.g. 'IMU I2C', 'IMU Serial')."""

    @property
    @abstractmethod
    def sample_rate(self) -> float:
        """Expected samples per second."""
