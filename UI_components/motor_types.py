"""
Motor Types - Unified data types for motor control system.

All layers (transport, controller, API) use these types for consistent
data exchange regardless of communication backend.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MotorResponse:
    """Unified response from any motor operation.

    All transports and controllers return this type, making it safe to
    swap backends without changing upstream code.
    """
    success: bool
    motor_id: int
    temperature: float = 0.0        # °C
    torque_current: float = 0.0     # A (converted from raw iq)
    speed: float = 0.0              # deg/s
    acceleration: float = 0.0       # deg/s²
    angle: float = 0.0              # degrees, multi-turn
    encoder: int = 0                # raw 18-bit (0~262143)
    voltage: float = 0.0            # V
    error_state: int = 0            # bit flags (bit0: low voltage, bit3: over-temp)
    error_msg: Optional[str] = None

    @staticmethod
    def fail(motor_id: int, msg: str) -> MotorResponse:
        """Convenience constructor for failure responses."""
        return MotorResponse(success=False, motor_id=motor_id, error_msg=msg)


@dataclass
class SafetyConfig:
    """Motor safety parameters enforced at the controller layer.

    These limits apply regardless of how the controller is called,
    preventing unsafe commands even when bypassing MotorAPI.
    """
    max_iq: int = 400                   # Torque limit (iq units, 400 ≈ 6.4A)
    max_angle: float = 45.0             # Position limit ±45°
    max_temperature: float = 80.0       # °C — auto-stop above this
    max_current: float = 10.0           # A — warning above this
    low_voltage_threshold: float = 20.0  # V — warning below this
    error_auto_stop: bool = True        # Auto-stop on non-zero error_state
