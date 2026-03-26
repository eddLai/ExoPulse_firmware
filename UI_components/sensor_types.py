"""
Sensor Types - Unified data types for sensor system.

Mirrors the motor_types.py pattern: all sensor transports return
these types for consistent data exchange.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class IMUData:
    """IMU sensor reading (MPU6050 / compatible 6-axis IMU).

    All IMU transports (I2C direct, ESP32 serial/UDP) return this type.
    """
    success: bool
    timestamp: float = 0.0                              # seconds (monotonic)
    accel: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (ax, ay, az) in g
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # (gx, gy, gz) in deg/s
    quaternion: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # (w, x, y, z)
    euler: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) in degrees
    temperature: float = 0.0                             # °C
    error_msg: Optional[str] = None

    @staticmethod
    def fail(msg: str) -> IMUData:
        """Convenience constructor for failure responses."""
        return IMUData(success=False, error_msg=msg)


@dataclass
class SensorReading:
    """Generic sensor reading container.

    Wraps typed sensor data with a type tag for dispatching.
    Future sensor types (EMG, force plate, etc.) can be added here.
    """
    sensor_type: str                    # "imu" (future: "emg", etc.)
    imu: Optional[IMUData] = None
