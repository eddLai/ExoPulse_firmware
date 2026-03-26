"""
IMU Serial Transport - Receives IMU data from ESP32 via UDP.

The ESP32 firmware streams IMU data (roll, pitch, yaw) over UDP.
This transport listens on a UDP port and parses incoming packets.

Note: This provides Euler angles only (no raw accel/gyro/quaternion)
since the ESP32 firmware only transmits fused orientation.
"""
from __future__ import annotations

import os
import re
import socket
import sys
import threading
import time
from typing import List, Tuple

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from sensor_types import IMUData, SensorReading
from sensor_transport import SensorTransport


# Expected UDP packet format from ESP32:
# "roll pitch yaw" or "X ... roll pitch yaw" (varies by firmware version)
_FLOAT_RE = re.compile(r'(-?\d+\.?\d*)')


class IMUSerialTransport(SensorTransport):
    """IMU sensor via ESP32 UDP stream.

    The ESP32 sends IMU attitude data over UDP (default port 5000).
    Provides Euler angles; raw accel/gyro/quaternion are not available
    through this transport.
    """

    def __init__(self, udp_port: int = 5000, timeout: float = 0.5):
        self._udp_port = udp_port
        self._timeout = timeout
        self._sock: socket.socket = None
        self._connected = False
        self._latest: IMUData = IMUData(success=False, error_msg="No data yet")
        self._lock = threading.Lock()
        self._listen_thread: threading.Thread = None
        self._stop_evt = threading.Event()

    def connect(self, **kwargs) -> Tuple[bool, str]:
        port = kwargs.get('port', self._udp_port)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', port))
            sock.settimeout(self._timeout)
            self._sock = sock
            self._udp_port = port
            self._connected = True

            # Start background listener
            self._stop_evt.clear()
            self._listen_thread = threading.Thread(
                target=self._listener_loop, daemon=True)
            self._listen_thread.start()

            return True, f"Listening on UDP port {port}"
        except OSError as e:
            return False, f"UDP bind failed: {e}"

    def disconnect(self) -> None:
        self._stop_evt.set()
        if self._listen_thread:
            self._listen_thread.join(timeout=2.0)
            self._listen_thread = None
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._connected = False

    def _listener_loop(self):
        """Background thread: receive and parse UDP packets."""
        while not self._stop_evt.is_set():
            try:
                data, _ = self._sock.recvfrom(1024)
                line = data.decode('utf-8', errors='ignore').strip()
                imu = self._parse_imu_line(line)
                if imu:
                    with self._lock:
                        self._latest = imu
            except socket.timeout:
                continue
            except OSError:
                break

    def _parse_imu_line(self, line: str) -> IMUData:
        """Parse IMU data from ESP32 UDP packet.

        Tries multiple formats:
        1. "roll pitch yaw" (3 floats)
        2. "X ... roll pitch yaw" (last 3 floats are RPY)
        """
        floats = _FLOAT_RE.findall(line)
        if len(floats) >= 3:
            try:
                # Take last 3 values as roll, pitch, yaw
                roll = float(floats[-3])
                pitch = float(floats[-2])
                yaw = float(floats[-1])
                return IMUData(
                    success=True,
                    timestamp=time.monotonic(),
                    euler=(roll, pitch, yaw),
                )
            except (ValueError, IndexError):
                pass
        return None

    def read(self) -> SensorReading:
        with self._lock:
            return SensorReading(sensor_type="imu", imu=self._latest)

    def read_batch(self, n_samples: int) -> List[SensorReading]:
        """Collect n_samples at approximately the streaming rate."""
        results = []
        for _ in range(n_samples):
            results.append(self.read())
            time.sleep(0.05)  # ~20Hz sampling of the latest value
        return results

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def sensor_type(self) -> str:
        return "imu"

    @property
    def transport_name(self) -> str:
        return f"IMU Serial (UDP:{self._udp_port})"

    @property
    def sample_rate(self) -> float:
        return 20.0  # ESP32 typically streams at ~10-20 Hz
