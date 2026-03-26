"""
IMU I2C Transport - Direct I2C reading from MPU6050 via libexo_imu.so.

Wraps the C library (sensing/imu/mpu6050.h) through ctypes.
Runs on Jetson Orin with MPU6050 connected to I2C bus 7.
"""
from __future__ import annotations

import ctypes
import math
import os
import sys
import time
from typing import List, Tuple

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from sensor_types import IMUData, SensorReading
from sensor_transport import SensorTransport


# ctypes mirror of imu_data_t
class _IMUDataCTypes(ctypes.Structure):
    _fields_ = [
        ("accel", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("quat", ctypes.c_float * 4),
        ("temperature", ctypes.c_float),
        ("timestamp", ctypes.c_double),
    ]


class _MPU6050ConfigCTypes(ctypes.Structure):
    _fields_ = [
        ("sample_rate_hz", ctypes.c_uint16),
        ("calibration_samples", ctypes.c_uint16),
        ("madgwick_beta", ctypes.c_float),
    ]


# Library search paths
_LIB_SEARCH_PATHS = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "sensing", "imu", "libexo_imu.so"),
    "/home/ntk/ExoPulse/external/firmware_layer/sensing/imu/libexo_imu.so",
]


def _data_to_imu(data: _IMUDataCTypes) -> IMUData:
    """Convert ctypes imu_data_t to IMUData."""
    q = (data.quat[0], data.quat[1], data.quat[2], data.quat[3])

    # Quaternion to Euler (roll, pitch, yaw in degrees)
    w, x, y, z = q
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return IMUData(
        success=True,
        timestamp=data.timestamp,
        accel=(data.accel[0], data.accel[1], data.accel[2]),
        gyro=(data.gyro[0], data.gyro[1], data.gyro[2]),
        quaternion=q,
        euler=(roll, pitch, yaw),
        temperature=data.temperature,
    )


class IMUI2CTransport(SensorTransport):
    """IMU sensor via direct I2C on Jetson Orin.

    Uses libexo_imu.so (compiled from sensing/imu/mpu6050.cpp).
    """

    def __init__(self, i2c_device: str = "/dev/i2c-7",
                 address: int = 0x68,
                 sample_rate_hz: int = 200):
        self._i2c_device = i2c_device
        self._address = address
        self._sample_rate_hz = sample_rate_hz
        self._lib = None
        self._dev = None
        self._connected = False

    def connect(self, **kwargs) -> Tuple[bool, str]:
        # Find library
        lib_path = None
        for path in _LIB_SEARCH_PATHS:
            if os.path.exists(path):
                lib_path = path
                break
        if not lib_path:
            return False, "libexo_imu.so not found. Run: cd sensing/imu && make"

        try:
            self._lib = ctypes.CDLL(lib_path)
        except OSError as e:
            return False, f"Failed to load library: {e}"

        # Setup ctypes signatures
        self._lib.mpu6050_create.restype = ctypes.c_void_p
        self._lib.mpu6050_create.argtypes = [ctypes.c_char_p, ctypes.c_uint8]
        self._lib.mpu6050_init.restype = ctypes.c_int
        self._lib.mpu6050_init.argtypes = [ctypes.c_void_p]
        self._lib.mpu6050_init_config.restype = ctypes.c_int
        self._lib.mpu6050_init_config.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_MPU6050ConfigCTypes)]
        self._lib.mpu6050_read.restype = ctypes.c_int
        self._lib.mpu6050_read.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_IMUDataCTypes)]
        self._lib.mpu6050_destroy.argtypes = [ctypes.c_void_p]
        self._lib.mpu6050_reset_quaternion.restype = ctypes.c_int
        self._lib.mpu6050_reset_quaternion.argtypes = [ctypes.c_void_p]

        # Create and initialize
        device_bytes = self._i2c_device.encode('utf-8')
        self._dev = self._lib.mpu6050_create(device_bytes, self._address)
        if not self._dev:
            return False, f"Failed to create MPU6050 on {self._i2c_device}"

        # Configure
        cfg = _MPU6050ConfigCTypes()
        cfg.sample_rate_hz = self._sample_rate_hz
        cfg.calibration_samples = 500
        cfg.madgwick_beta = 0.1

        if self._lib.mpu6050_init_config(self._dev, ctypes.byref(cfg)) < 0:
            self._lib.mpu6050_destroy(self._dev)
            self._dev = None
            return False, "MPU6050 init failed (check I2C wiring)"

        self._connected = True
        return True, (f"MPU6050 on {self._i2c_device}:0x{self._address:02X} "
                       f"@ {self._sample_rate_hz}Hz")

    def disconnect(self) -> None:
        if self._dev and self._lib:
            self._lib.mpu6050_destroy(self._dev)
            self._dev = None
        self._connected = False

    def read(self) -> SensorReading:
        if not self._connected or not self._dev:
            return SensorReading(
                sensor_type="imu",
                imu=IMUData.fail("IMU not connected"))

        data = _IMUDataCTypes()
        if self._lib.mpu6050_read(self._dev, ctypes.byref(data)) == 0:
            return SensorReading(
                sensor_type="imu",
                imu=_data_to_imu(data))

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
        if self._dev and self._lib:
            self._lib.mpu6050_reset_quaternion(self._dev)

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
