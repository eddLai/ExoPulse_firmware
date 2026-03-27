"""
Serial Transport - UART → ESP32 → CAN → LK-TECH motors.

Sends text commands to ESP32 firmware which translates them to CAN messages.
Supports all command types (torque, position, status, stop).

Command protocol:
  T{id}:{iq}             → Torque (0xA1)
  P{id}:{angle}:{speed}  → Position multi-turn (0xA4)
  M{id}:{angle}          → Position single-turn (0xA6, legacy)
  STOP                   → Stop all motors (0x80)
"""
from __future__ import annotations

import re
import sys
import os
import threading
import time
from typing import Optional, Set, Tuple

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from motor_types import MotorResponse
from motor_transport import MotorTransport

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def find_esp32_port() -> Optional[str]:
    """Auto-detect ESP32 serial port."""
    if not SERIAL_AVAILABLE:
        return None
    for port in serial.tools.list_ports.comports():
        if 'USB' in port.device or 'ACM' in port.device:
            return port.device
        if 'CP210' in port.description or 'CH340' in port.description:
            return port.device
    return None


# Regex for parsing motor status lines from ESP32
# Format: [timestamp] SEQ:xx M:x T:xx V:xx I:xx S:xx ACC:xx E:xxx A:xx ERR:xxx
_STATUS_RE = re.compile(
    r'M:(\d+)\s+T:(-?\d+)\s+V:(-?[\d.]+)\s+'
    r'I:(-?[\d.]+)\s+S:(-?[\d.]+)\s+'
    r'ACC:(-?[\d.]+)\s+E:(\d+)\s+'
    r'A:(-?[\d.]+)\s+ERR:(\d+)')


def _parse_status_line(line: str, motor_id: int) -> Optional[MotorResponse]:
    """Parse an ESP32 motor status line into MotorResponse."""
    m = _STATUS_RE.search(line)
    if not m:
        return None
    return MotorResponse(
        success=True,
        motor_id=int(m.group(1)),
        temperature=float(m.group(2)),
        voltage=float(m.group(3)),
        torque_current=float(m.group(4)),
        speed=float(m.group(5)),
        acceleration=float(m.group(6)),
        encoder=int(m.group(7)),
        angle=float(m.group(8)),
        error_state=int(m.group(9)),
    )


class SerialTransport(MotorTransport):
    """Motor communication via UART to ESP32.

    The ESP32 acts as a CAN bridge, translating serial commands to CAN frames.
    """

    def __init__(self, port: str = None, baudrate: int = 115200, timeout: float = 1.0):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser = None
        self._connected = False
        self._lock = threading.Lock()

    def connect(self, **kwargs) -> Tuple[bool, str]:
        if not SERIAL_AVAILABLE:
            return False, "pyserial not installed"

        port = kwargs.get('port', self._port)
        if port is None:
            port = find_esp32_port()
            if port is None:
                return False, "No ESP32 serial port found"

        try:
            self._ser = serial.Serial(port, self._baudrate, timeout=self._timeout)
            time.sleep(2)  # Wait for ESP32 reset
            self._ser.reset_input_buffer()
            self._port = port
            self._connected = True
            return True, f"Connected to {port} @ {self._baudrate}"
        except serial.SerialException as e:
            return False, f"Serial connection failed: {e}"

    def disconnect(self) -> None:
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
        self._connected = False

    def _send_and_read(self, cmd: str, timeout: float = 0.3) -> list:
        """Send command and collect response lines."""
        with self._lock:
            if not self._ser or not self._ser.is_open:
                return []
            try:
                self._ser.reset_input_buffer()
                self._ser.write((cmd.strip() + '\n').encode('utf-8'))
                self._ser.flush()
            except serial.SerialException:
                return []

            lines = []
            end = time.time() + timeout
            while time.time() < end:
                if self._ser.in_waiting > 0:
                    try:
                        line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                    except serial.SerialException:
                        break
                else:
                    time.sleep(0.005)
            return lines

    def _find_status_response(self, lines: list, motor_id: int) -> MotorResponse:
        """Find and parse motor status from response lines."""
        for line in lines:
            resp = _parse_status_line(line, motor_id)
            if resp and resp.motor_id == motor_id:
                return resp
        # No parseable status — return success without data
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
        # ESP32 continuously outputs status; just read the latest
        with self._lock:
            if not self._ser or not self._ser.is_open:
                return MotorResponse.fail(motor_id, "Serial not open")
            lines = []
            end = time.time() + 0.2
            while time.time() < end:
                if self._ser.in_waiting > 0:
                    try:
                        line = self._ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                    except serial.SerialException:
                        break
                else:
                    time.sleep(0.005)

        # Find status for this motor_id
        for line in reversed(lines):
            resp = _parse_status_line(line, motor_id)
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
        return self._connected and self._ser is not None and self._ser.is_open

    @property
    def transport_name(self) -> str:
        return f"Serial ({self._port or 'auto'})"

    @property
    def supported_commands(self) -> Set[str]:
        return {"torque", "position", "status", "stop"}
