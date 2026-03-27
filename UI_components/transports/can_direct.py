"""
CAN Direct Transport - MCP2515 SPI → libexo_motor.so → LK-TECH motors.

Wraps the existing CANDirectBackend from motor_control.py into the
MotorTransport interface with unified MotorResponse output.

Thread-safe: all CAN operations protected by RLock.
"""
from __future__ import annotations

import ctypes
import os
import sys
import threading
from typing import Set, Tuple

# Ensure parent directory is in path for imports
_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

from motor_types import MotorResponse
from motor_transport import MotorTransport


class _MotorStatusCTypes(ctypes.Structure):
    """ctypes mirror of motor_status_t from lktech_motor.h"""
    _fields_ = [
        ("temperature", ctypes.c_int8),
        ("torque_current", ctypes.c_int16),
        ("speed", ctypes.c_int16),
        ("acceleration", ctypes.c_int32),
        ("angle", ctypes.c_int64),
        ("encoder", ctypes.c_uint32),
        ("voltage", ctypes.c_uint16),
        ("error_state", ctypes.c_uint8),
    ]


# Library search paths
_LIB_SEARCH_PATHS = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "control", "motor", "libexo_motor.so"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "control", "motor", "libexo_motor.so"),
    "/home/ntk/ExoPulse/external/firmware_layer/control/motor/libexo_motor.so",
]


def _status_to_response(motor_id: int, status: _MotorStatusCTypes) -> MotorResponse:
    """Convert ctypes MotorStatus to unified MotorResponse."""
    return MotorResponse(
        success=True,
        motor_id=motor_id,
        temperature=float(status.temperature),
        torque_current=status.torque_current * (33.0 / 2048.0),  # iq → Amps
        speed=float(status.speed),
        acceleration=float(status.acceleration),
        angle=status.angle * 0.01,  # 0.01°/LSB → degrees
        encoder=status.encoder,
        voltage=status.voltage * 0.1,  # 0.1V/LSB → Volts
        error_state=status.error_state,
    )


class CANDirectTransport(MotorTransport):
    """Motor communication via MCP2515 SPI (Jetson Orin direct CAN).

    Uses libexo_motor.so compiled from control/motor/.
    Currently supports: torque, status, stop.
    Position (0xA4) will be added when the C library is extended.
    """

    def __init__(self, spi_device: bytes = b"/dev/spidev0.0",
                 baud_rate: int = 1000000):
        self._spi_device = spi_device
        self._baud_rate = baud_rate
        self._lib = None
        self._can = None
        self._motors = {}  # {motor_id: handle}
        self._connected = False
        self._can_lock = threading.RLock()

    def connect(self, motor_ids=(0x141, 0x142), **kwargs) -> Tuple[bool, str]:
        """Initialize CAN bus and connect to motors."""
        # Find library
        lib_path = None
        for path in _LIB_SEARCH_PATHS:
            if os.path.exists(path):
                lib_path = path
                break
        if not lib_path:
            return False, "libexo_motor.so not found. Run: cd control/motor && make"

        try:
            self._lib = ctypes.CDLL(lib_path)
        except OSError as e:
            return False, f"Failed to load library: {e}"

        # Setup ctypes signatures
        self._lib.mcp2515_create.restype = ctypes.c_void_p
        self._lib.mcp2515_create.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        self._lib.mcp2515_init.restype = ctypes.c_int
        self._lib.mcp2515_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self._lib.mcp2515_destroy.argtypes = [ctypes.c_void_p]

        self._lib.motor_create.restype = ctypes.c_void_p
        self._lib.motor_create.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
        self._lib.motor_init.restype = ctypes.c_int
        self._lib.motor_init.argtypes = [ctypes.c_void_p]
        self._lib.motor_set_torque.restype = ctypes.c_int
        self._lib.motor_set_torque.argtypes = [
            ctypes.c_void_p, ctypes.c_int16, ctypes.POINTER(_MotorStatusCTypes)]
        self._lib.motor_read_status.restype = ctypes.c_int
        self._lib.motor_read_status.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_MotorStatusCTypes)]
        self._lib.motor_stop.restype = ctypes.c_int
        self._lib.motor_stop.argtypes = [ctypes.c_void_p]
        self._lib.motor_shutdown.restype = ctypes.c_int
        self._lib.motor_shutdown.argtypes = [ctypes.c_void_p]
        self._lib.motor_destroy.argtypes = [ctypes.c_void_p]

        # Initialize CAN bus
        self._can = self._lib.mcp2515_create(self._spi_device, 1000000)
        if not self._can:
            return False, "Failed to create MCP2515 (check SPI device)"

        if self._lib.mcp2515_init(self._can, self._baud_rate) < 0:
            self._lib.mcp2515_destroy(self._can)
            self._can = None
            return False, "Failed to init MCP2515 (check SPI wiring)"

        # Initialize motors
        results = []
        for mid in motor_ids:
            handle = self._lib.motor_create(self._can, mid)
            if not handle:
                results.append(f"Motor 0x{mid:03X}: create failed")
                continue
            if self._lib.motor_init(handle) < 0:
                self._lib.motor_destroy(handle)
                results.append(f"Motor 0x{mid:03X}: no CAN response")
                continue
            self._motors[mid - 0x140] = handle  # 0x141→1, 0x142→2
            results.append(f"Motor 0x{mid:03X}: OK")

        if not self._motors:
            self._lib.mcp2515_destroy(self._can)
            self._can = None
            return False, "No motors responded. " + "; ".join(results)

        self._connected = True
        return True, "; ".join(results)

    def disconnect(self) -> None:
        with self._can_lock:
            for handle in self._motors.values():
                self._lib.motor_stop(handle)
                self._lib.motor_destroy(handle)
            self._motors.clear()
            if self._can:
                self._lib.mcp2515_destroy(self._can)
                self._can = None
        self._connected = False

    def send_torque(self, motor_id: int, iq: int) -> MotorResponse:
        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")
        status = _MotorStatusCTypes()
        with self._can_lock:
            if self._lib.motor_set_torque(handle, int(iq),
                                          ctypes.byref(status)) == 0:
                return _status_to_response(motor_id, status)
        return MotorResponse.fail(motor_id, "CAN torque command failed")

    def send_position(self, motor_id: int, angle_deg: float,
                      max_speed: int = 700) -> MotorResponse:
        # TODO: implement when C library adds motor_set_position (0xA4)
        return MotorResponse.fail(
            motor_id,
            "CAN Direct position control not yet implemented "
            "(requires C library extension for 0xA4)")

    def read_status(self, motor_id: int) -> MotorResponse:
        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")
        status = _MotorStatusCTypes()
        with self._can_lock:
            if self._lib.motor_read_status(handle, ctypes.byref(status)) == 0:
                return _status_to_response(motor_id, status)
        return MotorResponse.fail(motor_id, "CAN status read failed")

    def send_stop(self, motor_id: int) -> MotorResponse:
        if motor_id == 0:
            # Stop all
            with self._can_lock:
                for mid, handle in self._motors.items():
                    self._lib.motor_stop(handle)
            return MotorResponse(success=True, motor_id=0)

        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")
        with self._can_lock:
            if self._lib.motor_stop(handle) == 0:
                return MotorResponse(success=True, motor_id=motor_id)
        return MotorResponse.fail(motor_id, "CAN stop command failed")

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def transport_name(self) -> str:
        return "CAN Direct (MCP2515 SPI)"

    @property
    def supported_commands(self) -> Set[str]:
        return {"torque", "status", "stop"}
