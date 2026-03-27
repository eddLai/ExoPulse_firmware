"""
CAN Direct Transport - MCP2515 SPI -> libexo_motor.so -> LK-TECH motors.

Wraps the existing CANDirectBackend from motor_control.py into the
MotorTransport interface with unified MotorResponse output.

Architecture:
  - Command register: holds latest pending command per motor (overwrite semantics)
  - Worker thread: single CAN executor, processes commands at up to 30Hz
  - Priority: motor control commands > status reads
  - Stop commands bypass the queue for immediate execution
"""
from __future__ import annotations

import ctypes
import os
import queue
import sys
import threading
import time
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
        torque_current=status.torque_current * (33.0 / 2048.0),  # iq -> Amps
        speed=float(status.speed),
        acceleration=float(status.acceleration),
        angle=status.angle * 0.01,  # 0.01 deg/LSB -> degrees
        encoder=status.encoder,
        voltage=status.voltage * 0.1,  # 0.1V/LSB -> Volts
        error_state=status.error_state,
    )


# Priority levels (lower = higher priority)
_PRIO_CONTROL = 0
_PRIO_STATUS = 1


class CANDirectTransport(MotorTransport):
    """Motor communication via MCP2515 SPI (Jetson Orin direct CAN).

    Uses libexo_motor.so compiled from control/motor/.
    Supports: torque (0xA1), position (0xA4), status, stop.

    All CAN operations are serialized through a single worker thread.
    Motor commands (position/torque) have priority over status reads.
    Control commands are rate-limited to CONTROL_RATE_HZ per motor.
    """

    CONTROL_RATE_HZ = 30

    def __init__(self, spi_device: bytes = b"/dev/spidev0.0",
                 baud_rate: int = 1000000):
        self._spi_device = spi_device
        self._baud_rate = baud_rate
        self._lib = None
        self._can = None
        self._motors = {}  # {motor_id: handle}
        self._connected = False
        self._can_lock = threading.RLock()

        # Command register: {(priority, motor_id): (func, event, result_holder)}
        # Using dict with overwrite semantics - latest command wins per motor+priority
        self._reg_lock = threading.Lock()
        self._control_reg = {}   # {motor_id: (func, event, result_holder)}
        self._status_reg = {}    # {motor_id: (func, event, result_holder)}
        self._cmd_available = threading.Event()

        # Worker thread
        self._worker_running = False
        self._worker_thread = None

        # Rate limiting per motor
        self._min_interval = 1.0 / self.CONTROL_RATE_HZ
        self._last_cmd_time = {}  # {motor_id: monotonic timestamp}

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
        self._lib.motor_set_position.restype = ctypes.c_int
        self._lib.motor_set_position.argtypes = [
            ctypes.c_void_p, ctypes.c_int32, ctypes.c_uint16,
            ctypes.POINTER(_MotorStatusCTypes)]
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
            self._motors[mid - 0x140] = handle  # 0x141->1, 0x142->2
            results.append(f"Motor 0x{mid:03X}: OK")

        if not self._motors:
            self._lib.mcp2515_destroy(self._can)
            self._can = None
            return False, "No motors responded. " + "; ".join(results)

        self._connected = True
        self._start_worker()
        return True, "; ".join(results)

    def disconnect(self) -> None:
        self._stop_worker()
        with self._can_lock:
            for handle in self._motors.values():
                self._lib.motor_stop(handle)
                self._lib.motor_destroy(handle)
            self._motors.clear()
            if self._can:
                self._lib.mcp2515_destroy(self._can)
                self._can = None
        self._connected = False

    # ---- Worker thread ----

    def _start_worker(self):
        self._worker_running = True
        self._worker_thread = threading.Thread(
            target=self._worker_loop, daemon=True, name="CAN-worker")
        self._worker_thread.start()

    def _stop_worker(self):
        self._worker_running = False
        self._cmd_available.set()  # wake up worker to exit
        if self._worker_thread:
            self._worker_thread.join(timeout=2.0)
            self._worker_thread = None
        # Cancel any pending commands
        with self._reg_lock:
            for _, (__, event, result_holder) in self._control_reg.items():
                result_holder[0] = MotorResponse.fail(0, "Transport disconnected")
                event.set()
            for _, (__, event, result_holder) in self._status_reg.items():
                result_holder[0] = MotorResponse.fail(0, "Transport disconnected")
                event.set()
            self._control_reg.clear()
            self._status_reg.clear()

    def _worker_loop(self):
        while self._worker_running:
            # Wait for commands or periodic wakeup
            self._cmd_available.wait(timeout=self._min_interval)
            self._cmd_available.clear()

            if not self._worker_running:
                break

            # Phase 1: process all pending control commands (high priority)
            self._process_control_commands()

            # Phase 2: process pending status reads (low priority)
            self._process_status_commands()

    def _process_control_commands(self):
        """Execute pending motor control commands with rate limiting."""
        with self._reg_lock:
            pending = dict(self._control_reg)
            self._control_reg.clear()

        for motor_id, (func, event, result_holder) in pending.items():
            if not self._worker_running:
                result_holder[0] = MotorResponse.fail(motor_id, "Shutting down")
                event.set()
                continue

            # Rate limiting: wait if too soon since last command for this motor
            now = time.monotonic()
            last = self._last_cmd_time.get(motor_id, 0)
            wait_time = self._min_interval - (now - last)
            if wait_time > 0:
                time.sleep(wait_time)

            with self._can_lock:
                result_holder[0] = func()
            self._last_cmd_time[motor_id] = time.monotonic()
            event.set()

    def _process_status_commands(self):
        """Execute pending status reads. Abort early if new control commands arrive."""
        with self._reg_lock:
            pending = dict(self._status_reg)
            self._status_reg.clear()

        for motor_id, (func, event, result_holder) in pending.items():
            if not self._worker_running:
                result_holder[0] = MotorResponse.fail(motor_id, "Shutting down")
                event.set()
                continue

            # If new control commands arrived, skip remaining status reads
            if self._control_reg:
                result_holder[0] = MotorResponse.fail(motor_id, "Preempted by command")
                event.set()
                continue

            with self._can_lock:
                result_holder[0] = func()
            event.set()

    # ---- Command submission ----

    def _submit_control(self, motor_id: int, func) -> MotorResponse:
        """Submit a motor control command (high priority, register semantics)."""
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")

        event = threading.Event()
        result_holder = [None]

        with self._reg_lock:
            # Overwrite any existing command for this motor (latest wins)
            old = self._control_reg.get(motor_id)
            if old:
                # Cancel the previous pending command
                old[1].set()  # event
                old[2][0] = MotorResponse.fail(motor_id, "Superseded by newer command")
            self._control_reg[motor_id] = (func, event, result_holder)

        self._cmd_available.set()

        if event.wait(timeout=1.0):
            return result_holder[0]
        return MotorResponse.fail(motor_id, "Command timeout")

    def _submit_status(self, motor_id: int, func) -> MotorResponse:
        """Submit a status read (low priority, register semantics)."""
        if not self._connected:
            return MotorResponse.fail(motor_id, "Not connected")

        event = threading.Event()
        result_holder = [None]

        with self._reg_lock:
            old = self._status_reg.get(motor_id)
            if old:
                old[1].set()
                old[2][0] = MotorResponse.fail(motor_id, "Superseded by newer read")
            self._status_reg[motor_id] = (func, event, result_holder)

        self._cmd_available.set()

        if event.wait(timeout=1.0):
            return result_holder[0]
        return MotorResponse.fail(motor_id, "Status read timeout")

    # ---- Public API ----

    def send_torque(self, motor_id: int, iq: int) -> MotorResponse:
        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            status = _MotorStatusCTypes()
            if self._lib.motor_set_torque(handle, int(iq),
                                          ctypes.byref(status)) == 0:
                return _status_to_response(motor_id, status)
            return MotorResponse.fail(motor_id, "CAN torque command failed")

        return self._submit_control(motor_id, execute)

    def send_position(self, motor_id: int, angle_deg: float,
                      max_speed: int = 700) -> MotorResponse:
        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            status = _MotorStatusCTypes()
            angle_001deg = int(angle_deg * 100)  # deg -> 0.01 deg/LSB
            if self._lib.motor_set_position(handle, angle_001deg, int(max_speed),
                                            ctypes.byref(status)) == 0:
                return _status_to_response(motor_id, status)
            return MotorResponse.fail(motor_id, "CAN position command failed")

        return self._submit_control(motor_id, execute)

    def read_status(self, motor_id: int) -> MotorResponse:
        handle = self._motors.get(motor_id)
        if not handle:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            status = _MotorStatusCTypes()
            if self._lib.motor_read_status(handle, ctypes.byref(status)) == 0:
                return _status_to_response(motor_id, status)
            return MotorResponse.fail(motor_id, "CAN status read failed")

        return self._submit_status(motor_id, execute)

    def send_stop(self, motor_id: int) -> MotorResponse:
        """Stop command bypasses the queue for immediate execution."""
        if motor_id == 0:
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
        return {"torque", "position", "status", "stop"}
