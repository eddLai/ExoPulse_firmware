"""
WiFi Transport - TCP → ESP32 → CAN → LK-TECH motors.

Same command protocol as SerialTransport but over TCP socket.
ESP32 must be connected to WiFi and running TCP server.
"""
from __future__ import annotations

import os
import socket
import sys
import threading
import time
from typing import Set, Tuple

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from motor_types import MotorResponse
from motor_transport import MotorTransport

# Reuse status parser from serial transport
from transports.serial_esp32 import _parse_status_line


class WiFiTransport(MotorTransport):
    """Motor communication via TCP to ESP32 WiFi.

    The ESP32 runs a TCP server that accepts the same command format
    as the serial interface (T, P, M, STOP commands).
    """

    def __init__(self, ip: str = "10.154.48.200", port: int = 8080,
                 timeout: float = 2.0):
        self._ip = ip
        self._port = port
        self._timeout = timeout
        self._sock: socket.socket = None
        self._connected = False
        self._lock = threading.Lock()

    def connect(self, **kwargs) -> Tuple[bool, str]:
        ip = kwargs.get('ip', self._ip)
        port = kwargs.get('port', self._port)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout)
            sock.connect((ip, port))
            self._sock = sock
            self._ip = ip
            self._port = port
            self._connected = True

            # Send MODE_WIFI to switch ESP32 output to TCP
            self._send_raw(b"MODE_WIFI\n")
            time.sleep(0.1)

            return True, f"Connected to {ip}:{port}"
        except (socket.error, OSError) as e:
            return False, f"TCP connection failed: {e}"

    def disconnect(self) -> None:
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
        self._connected = False

    def _send_raw(self, data: bytes) -> bool:
        """Send raw bytes to TCP socket."""
        with self._lock:
            if not self._sock:
                return False
            try:
                self._sock.sendall(data)
                return True
            except (socket.error, OSError):
                self._connected = False
                return False

    def _send_and_read(self, cmd: str, timeout: float = 0.3) -> list:
        """Send command and read response lines."""
        with self._lock:
            if not self._sock:
                return []
            try:
                self._sock.sendall((cmd.strip() + '\n').encode('utf-8'))
            except (socket.error, OSError):
                self._connected = False
                return []

            # Read response
            lines = []
            self._sock.settimeout(timeout)
            buf = b""
            end = time.time() + timeout
            while time.time() < end:
                try:
                    chunk = self._sock.recv(1024)
                    if not chunk:
                        break
                    buf += chunk
                except socket.timeout:
                    break
                except (socket.error, OSError):
                    self._connected = False
                    break

            for line in buf.decode('utf-8', errors='ignore').splitlines():
                line = line.strip()
                if line:
                    lines.append(line)
            return lines

    def _find_status_response(self, lines: list, motor_id: int) -> MotorResponse:
        """Find and parse motor status from response lines."""
        for line in lines:
            resp = _parse_status_line(line, motor_id)
            if resp and resp.motor_id == motor_id:
                return resp
        return MotorResponse(success=True, motor_id=motor_id)

    def send_torque(self, motor_id: int, iq: int) -> MotorResponse:
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")
        lines = self._send_and_read(f"T{motor_id}:{iq}")
        return self._find_status_response(lines, motor_id)

    def send_position(self, motor_id: int, angle_deg: float,
                      max_speed: int = 700) -> MotorResponse:
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")
        cmd = f"P{motor_id}:{angle_deg:.2f}:{max_speed}"
        lines = self._send_and_read(cmd)
        return self._find_status_response(lines, motor_id)

    def read_status(self, motor_id: int) -> MotorResponse:
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")
        # Read whatever the ESP32 is streaming
        with self._lock:
            if not self._sock:
                return MotorResponse.fail(motor_id, "Socket not open")
            self._sock.settimeout(0.2)
            buf = b""
            try:
                while True:
                    chunk = self._sock.recv(1024)
                    if not chunk:
                        break
                    buf += chunk
            except socket.timeout:
                pass
            except (socket.error, OSError):
                self._connected = False
                return MotorResponse.fail(motor_id, "Socket error")

        lines = buf.decode('utf-8', errors='ignore').splitlines()
        for line in reversed(lines):
            resp = _parse_status_line(line.strip(), motor_id)
            if resp and resp.motor_id == motor_id:
                return resp
        return MotorResponse.fail(motor_id, "No status received")

    def send_stop(self, motor_id: int) -> MotorResponse:
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")
        self._send_and_read("STOP", timeout=0.2)
        return MotorResponse(success=True, motor_id=motor_id)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._sock is not None

    @property
    def transport_name(self) -> str:
        return f"WiFi TCP ({self._ip}:{self._port})"

    @property
    def supported_commands(self) -> Set[str]:
        return {"torque", "position", "status", "stop"}
