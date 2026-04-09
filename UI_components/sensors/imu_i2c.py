"""
IMU I2C Transport - Direct I2C reading from MPU6050 via exo_imu (pybind11).

Uses the pybind11 exo_imu module to directly hold a C++ MPU6050 object.
Runs on Jetson Orin with MPU6050 connected to I2C bus 7.
"""
from __future__ import annotations

import os
import sys
import time
from typing import List, Tuple

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from sensor_types import IMUData, SensorReading
from sensor_transport import SensorTransport

# Import pybind11 IMU module
_IMU_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "sensing", "imu")
if _IMU_DIR not in sys.path:
    sys.path.insert(0, _IMU_DIR)

try:
    import exo_imu
    _EXO_IMU_AVAILABLE = True
except ImportError as e:
    _EXO_IMU_AVAILABLE = False
    print(f"[IMUI2C] exo_imu not available: {e}")
    print(f"[IMUI2C] Build it: cd {_IMU_DIR} && make")


class IMUI2CTransport(SensorTransport):
    """IMU sensor via direct I2C on Jetson Orin.

    Uses exo_imu pybind11 module (C++ MPU6050 RAII object directly).
    """

    def __init__(self, i2c_device: str = "/dev/i2c-7",
                 address: int = 0x68,
                 sample_rate_hz: int = 200):
        self._i2c_device = i2c_device
        self._address = address
        self._sample_rate_hz = sample_rate_hz
        self._imu = None  # exo_imu.MPU6050 instance
        self._connected = False

    def connect(self, **kwargs) -> Tuple[bool, str]:
        if not _EXO_IMU_AVAILABLE:
            return False, "exo_imu module not available. Run: cd sensing/imu && make"

        try:
            self._imu = exo_imu.MPU6050(self._i2c_device, self._address)
        except RuntimeError as e:
            return False, f"Failed to create MPU6050: {e}"

        if self._imu.init_config(self._sample_rate_hz, 500, 0.1) < 0:
            self._imu = None
            return False, "MPU6050 init failed (check I2C wiring)"

        self._connected = True
        return True, (f"MPU6050 on {self._i2c_device}:0x{self._address:02X} "
                       f"@ {self._sample_rate_hz}Hz")

    def disconnect(self) -> None:
        self._imu = None  # pybind11 destructor calls mpu6050_destroy
        self._connected = False

    def read(self) -> SensorReading:
        if not self._connected or not self._imu:
            return SensorReading(
                sensor_type="imu",
                imu=IMUData.fail("IMU not connected"))

        ret, reading = self._imu.read()
        if ret == 0:
            roll, pitch, yaw = exo_imu.quat_to_euler(reading.quat)
            return SensorReading(
                sensor_type="imu",
                imu=IMUData(
                    success=True,
                    timestamp=reading.timestamp,
                    accel=tuple(reading.accel),
                    gyro=tuple(reading.gyro),
                    quaternion=tuple(reading.quat),
                    euler=(roll, pitch, yaw),
                    temperature=reading.temperature,
                ))

        return SensorReading(
            sensor_type="imu",
            imu=IMUData.fail("I2C read failed"))

    def read_batch(self, n_samples: int) -> List[SensorReading]:
        results = []
        interval = 1.0 / self._sample_rate_hz
        for _ in range(n_samples):
            results.append(self.read())
            time.sleep(interval)
        return results

    def reset_quaternion(self):
        """Reset the Madgwick filter quaternion to identity [1,0,0,0]."""
        if self._imu:
            self._imu.reset_quaternion()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def sensor_type(self) -> str:
        return "imu"

    @property
    def transport_name(self) -> str:
        return f"IMU I2C ({self._i2c_device}:0x{self._address:02X})"

    @property
    def sample_rate(self) -> float:
        return float(self._sample_rate_hz)
