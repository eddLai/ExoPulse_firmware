"""
CAN Direct Transport - MCP2515 SPI -> exo_motor (pybind11) -> LK-TECH motors.

Uses the pybind11 exo_motor module to directly hold C++ LktechMotor objects.

Architecture:
  - Command register: holds latest pending command per motor (overwrite semantics)
  - Worker thread: single CAN executor, processes commands at up to 30Hz
  - Priority: motor control commands > status reads
  - Stop commands bypass the queue for immediate execution
"""
from __future__ import annotations

import os
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

# Import pybind11 motor module
_MOTOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "..", "control", "motor")
if _MOTOR_DIR not in sys.path:
    sys.path.insert(0, _MOTOR_DIR)

try:
    import exo_motor
    _EXO_MOTOR_AVAILABLE = True
except ImportError as e:
    _EXO_MOTOR_AVAILABLE = False
    print(f"[CANDirect] exo_motor not available: {e}")
    print(f"[CANDirect] Build it: cd {_MOTOR_DIR} && make")


def _status_to_response(motor_id: int, status) -> MotorResponse:
    """Convert exo_motor.MotorStatus or ProtectedStatus to unified MotorResponse."""
    # ProtectedStatus wraps raw MotorStatus in .raw attribute
    raw = status.raw if hasattr(status, 'raw') else status
    return MotorResponse(
        success=True,
        motor_id=motor_id,
        temperature=float(raw.temperature),
        torque_current=raw.torque_current * (33.0 / 2048.0),  # iq -> Amps
        speed=float(raw.speed),
        acceleration=float(raw.acceleration),
        angle=raw.angle * 0.01,  # 0.01 deg/LSB -> degrees
        encoder=raw.encoder,
        voltage=raw.voltage * 0.1,  # 0.1V/LSB -> Volts
        error_state=raw.error_state,
    )


# Priority levels (lower = higher priority)
_PRIO_CONTROL = 0
_PRIO_STATUS = 1


class CANDirectTransport(MotorTransport):
    """Motor communication via MCP2515 SPI (Jetson Orin direct CAN).

    Uses exo_motor pybind11 module (C++ LktechMotor objects directly).
    Supports: torque (0xA1), position (0xA4), status, stop.

    All CAN operations are serialized through a single worker thread.
    Motor commands (position/torque) have priority over status reads.
    Control commands are rate-limited to CONTROL_RATE_HZ per motor.
    """

    CONTROL_RATE_HZ = 30

    def __init__(self, spi_device: str = "/dev/spidev0.0",
                 baud_rate: int = 1000000):
        # Handle legacy bytes argument
        if isinstance(spi_device, bytes):
            spi_device = spi_device.decode()
        self._spi_device = spi_device
        self._baud_rate = baud_rate
        self._can = None          # exo_motor.Mcp2515Can instance
        self._motors = {}         # {motor_id: exo_motor.LktechMotor} (raw, for position)
        self._protections = {}    # {motor_id: exo_motor.MotorProtection} (for torque)
        self._connected = False
        self._can_lock = threading.RLock()

        # Command register: {motor_id: (func, event, result_holder)}
        self._reg_lock = threading.Lock()
        self._control_reg = {}
        self._status_reg = {}
        self._cmd_available = threading.Event()

        # Worker thread
        self._worker_running = False
        self._worker_thread = None

        # Rate limiting per motor
        self._min_interval = 1.0 / self.CONTROL_RATE_HZ
        self._last_cmd_time = {}  # {motor_id: monotonic timestamp}

    def connect(self, motor_ids=(0x141, 0x142), **kwargs) -> Tuple[bool, str]:
        """Initialize CAN bus and connect to motors."""
        if not _EXO_MOTOR_AVAILABLE:
            return False, "exo_motor module not available. Run: cd control/motor && make"

        # Create CAN bus
        try:
            self._can = exo_motor.Mcp2515Can(self._spi_device)
        except RuntimeError as e:
            return False, f"Failed to create MCP2515: {e}"

        if self._can.init(self._baud_rate) < 0:
            self._can = None
            return False, "Failed to init MCP2515 (check SPI wiring)"

        # Initialize motors with C++ MotorProtection wrapper
        results = []
        for mid in motor_ids:
            name = f"motor_{mid:03X}"
            try:
                motor = exo_motor.LktechMotor(self._can, mid, name)
                if motor.init() < 0:
                    results.append(f"Motor 0x{mid:03X}: no CAN response")
                    continue
                # Wrap with C++ MotorProtection (angle ±45°, torque 150Nm, CLAMP mode)
                config = exo_motor.ProtectConfig()
                prot = exo_motor.MotorProtection(motor, config)
                ret = prot.init()
                if ret != exo_motor.MOTOR_OK:
                    results.append(f"Motor 0x{mid:03X}: protection init failed ({ret})")
                    continue
                key = mid - 0x140  # 0x141->1, 0x142->2
                self._motors[key] = motor       # raw motor for position commands
                self._protections[key] = prot   # protected wrapper for torque
                results.append(f"Motor 0x{mid:03X}: OK (protected)")
            except Exception as e:
                results.append(f"Motor 0x{mid:03X}: {e}")

        if not self._protections:
            self._can = None
            return False, "No motors responded. " + "; ".join(results)

        self._connected = True
        self._start_worker()
        return True, "; ".join(results)

    def disconnect(self) -> None:
        self._stop_worker()
        with self._can_lock:
            for prot in self._protections.values():
                try:
                    prot.stop()
                except Exception:
                    pass
            self._protections.clear()
            self._motors.clear()
            self._can = None  # pybind11 destructor handles cleanup
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
                old[1].set()
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
        prot = self._protections.get(motor_id)
        if not prot:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            ret, status = prot.set_torque_raw(int(iq))
            if ret == exo_motor.MOTOR_OK:
                return _status_to_response(motor_id, status)
            err_msg = exo_motor.MotorProtection.strerror(ret)
            return MotorResponse.fail(motor_id, f"Torque rejected: {err_msg}")

        return self._submit_control(motor_id, execute)

    def send_position(self, motor_id: int, angle_deg: float,
                      max_speed: int = 700) -> MotorResponse:
        motor = self._motors.get(motor_id)  # raw motor for position commands
        if not motor:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            angle_001deg = int(angle_deg * 100)  # deg -> 0.01 deg/LSB
            ret, status = motor.set_position(angle_001deg, int(max_speed))
            if ret == 0:
                return _status_to_response(motor_id, status)
            return MotorResponse.fail(motor_id, "CAN position command failed")

        return self._submit_control(motor_id, execute)

    def read_status(self, motor_id: int) -> MotorResponse:
        prot = self._protections.get(motor_id)
        if not prot:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")

        def execute():
            ret, status = prot.read_status()
            if ret == exo_motor.MOTOR_OK:
                return _status_to_response(motor_id, status)
            return MotorResponse.fail(motor_id, "CAN status read failed")

        return self._submit_status(motor_id, execute)

    def send_stop(self, motor_id: int) -> MotorResponse:
        """Stop command bypasses the queue for immediate execution."""
        if motor_id == 0:
            with self._can_lock:
                for prot in self._protections.values():
                    prot.stop()
            return MotorResponse(success=True, motor_id=0)

        prot = self._protections.get(motor_id)
        if not prot:
            return MotorResponse.fail(motor_id, f"Motor {motor_id} not connected")
        with self._can_lock:
            if prot.stop() == exo_motor.MOTOR_OK:
                return MotorResponse(success=True, motor_id=motor_id)
        return MotorResponse.fail(motor_id, "CAN stop command failed")

    def get_protection(self, motor_id: int):
        """Return the C++ MotorProtection wrapper for a motor (or None)."""
        return self._protections.get(motor_id)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def transport_name(self) -> str:
        return "CAN Direct (MCP2515 SPI)"

    @property
    def supported_commands(self) -> Set[str]:
        return {"torque", "position", "status", "stop"}
