#!/usr/bin/env python3
"""
Real-time AI controller for ExoPulse exoskeleton.

Loads a depRL RL policy and runs a real-time control loop:
  sensor read → RL inference → smoothing → motor torque command

Usage (headless):
    python realtime_controller.py --config config.yaml

Calls depRL modules from /home/ntk/Documents/depRL/ directly.
"""

import argparse
import enum
import logging
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

# Ensure depRL is importable
_DEPRL_ROOT = Path(__file__).resolve().parents[3]  # -> Documents/
_DEPRL_PATH = _DEPRL_ROOT / "depRL"
if str(_DEPRL_PATH) not in sys.path:
    sys.path.insert(0, str(_DEPRL_PATH))

from deprl.env_wrappers.hardware_bridge import HardwareBridge
from deprl.env_wrappers.scone_wrapper import RealtimeSmoothingFilter, ButterworthFilter

logger = logging.getLogger("ai_agent")


# ─── Safety Pipeline ────────────────────────────────────────────

class MotorProtection:
    """Stage 1: Protect the motor hardware.

    Applied in order:
        1. iq_limit clamp   — hard current ceiling
        2. Smoothing filter  — EMA or Butterworth low-pass
        3. Torque ramp       — linear 0→100 % over startup period
    """

    def __init__(self, iq_limit: int, exo_indices: list[int],
                 smoothing_filter, torque_ramp_seconds: float,
                 external_scale: float):
        self._iq_limit = iq_limit
        self._exo_indices = exo_indices
        self._filter = smoothing_filter
        self._ramp_seconds = torque_ramp_seconds
        self._external_scale = external_scale

    def reset(self):
        if self._filter:
            self._filter.reset()

    def apply(self, action: np.ndarray, elapsed: float) -> np.ndarray:
        """Run all motor-protection stages on *action* (mutates a copy)."""
        action = action.copy()

        # 1) iq_limit clamp (in action-space units, before external_scale)
        if self._external_scale > 0:
            max_action = self._iq_limit / self._external_scale
            for idx in self._exo_indices:
                action[idx] = np.clip(action[idx], -max_action, max_action)

        # 2) Smoothing filter
        if self._filter:
            action = self._filter.filter(action, self._exo_indices)

        # 3) Torque ramp
        if elapsed < self._ramp_seconds:
            ramp = elapsed / self._ramp_seconds
            for idx in self._exo_indices:
                action[idx] *= ramp

        return action


class HumanProtection:
    """Stage 2: Protect the human wearer (placeholder — pass-through).

    Each method is a defined extension point. Implement the body when
    the corresponding sensor / algorithm is ready.
    """

    def __init__(self, config=None):
        self._config = config or {}

    def apply(self, action: np.ndarray, exo_indices: list[int]) -> np.ndarray:
        """Run all human-protection checks. Currently pass-through."""
        action = self.torque_rate_limit(action, exo_indices)
        action = self.biomechanical_limit(action, exo_indices)
        action = self.contact_force_check(action, exo_indices)
        return action

    # -- Extension points (implement when ready) --

    def torque_rate_limit(self, action: np.ndarray,
                          exo_indices: list[int]) -> np.ndarray:
        """Limit torque change rate (Nm/s) to protect joints."""
        # TODO: implement dτ/dt limiting
        return action

    def biomechanical_limit(self, action: np.ndarray,
                            exo_indices: list[int]) -> np.ndarray:
        """Constrain torque based on joint angle / velocity envelope."""
        # TODO: implement joint-angle-dependent torque ceiling
        return action

    def contact_force_check(self, action: np.ndarray,
                            exo_indices: list[int]) -> np.ndarray:
        """Reduce torque if contact-force sensor reports excessive load."""
        # TODO: implement contact-force sensor feedback
        return action


class MotorRateLimiter:
    """Accumulate actions between CAN sends; emit time-weighted average.

    The control loop runs at ``control_freq_hz`` (e.g. 50 Hz) but the
    motor should only receive CAN commands at ``max_motor_freq_hz``
    (e.g. 10 Hz).  Between sends, every inference result is buffered
    with its timestamp.  When the send interval elapses the buffer is
    collapsed into a single action via time-weighted average (newer
    samples weigh more).

    Weight formula:  w_i = (t_i - t_first) + epsilon
    Result:          sum(w_i * a_i) / sum(w_i)
    """

    _EPSILON = 1e-6

    def __init__(self, max_motor_freq_hz: float, action_dim: int):
        self._min_interval = 1.0 / max_motor_freq_hz
        self._action_dim = action_dim
        self._buffer: list[tuple[float, np.ndarray]] = []  # (timestamp, action)
        self._last_send_time: float = 0.0
        self._last_sent_action: np.ndarray = np.zeros(action_dim, dtype=np.float32)

    def reset(self):
        self._buffer.clear()
        self._last_send_time = 0.0
        self._last_sent_action = np.zeros(self._action_dim, dtype=np.float32)

    def accumulate(self, action: np.ndarray, now: float):
        """Add an inference result to the buffer."""
        self._buffer.append((now, action.copy()))

    def maybe_send(self, now: float):
        """Return the weighted-average action if the send interval has
        elapsed, otherwise return ``None``.
        """
        if not self._buffer:
            return None

        if self._last_send_time > 0 and (now - self._last_send_time) < self._min_interval:
            return None

        # Compute time-weighted average
        t_first = self._buffer[0][0]
        weights = np.array(
            [t - t_first + self._EPSILON for t, _ in self._buffer],
            dtype=np.float64,
        )
        actions = np.stack([a for _, a in self._buffer]).astype(np.float64)
        result = (weights[:, None] * actions).sum(axis=0) / weights.sum()

        self._last_sent_action = result.astype(np.float32)
        self._last_send_time = now
        self._buffer.clear()
        return self._last_sent_action

    @property
    def last_sent_action(self) -> np.ndarray:
        return self._last_sent_action


class TorqueSafetyPipeline:
    """Two-stage safety pipeline: MotorProtection → HumanProtection,
    followed by rate-limited motor output.
    """

    def __init__(self, motor_protection: MotorProtection,
                 human_protection: HumanProtection,
                 rate_limiter: MotorRateLimiter,
                 exo_indices: list[int]):
        self.motor = motor_protection
        self.human = human_protection
        self.rate_limiter = rate_limiter
        self._exo_indices = exo_indices

    def reset(self):
        self.motor.reset()
        self.rate_limiter.reset()

    def apply(self, action: np.ndarray, elapsed: float) -> np.ndarray:
        """Run Stage 1 + Stage 2 on *action*."""
        action = self.motor.apply(action, elapsed)
        action = self.human.apply(action, self._exo_indices)
        return action

    def accumulate_and_maybe_send(self, action: np.ndarray,
                                   now: float):
        """Buffer the processed action; return weighted-average if it is
        time to send, otherwise ``None``.
        """
        self.rate_limiter.accumulate(action, now)
        return self.rate_limiter.maybe_send(now)


# ─── Configuration ───────────────────────────────────────────────

@dataclass
class RealtimeControllerConfig:
    checkpoint_path: str = ""
    control_freq_hz: float = 50.0
    external_scale: float = 45.0
    iq_limit: int = 800
    obs_dim: int = 125
    action_dim: int = 20
    enable_imu: bool = True
    enable_emg: bool = False
    enable_motor: bool = True
    filter_type: str = "ema"          # "ema" or "butterworth"
    ema_alpha: float = 0.3
    butterworth_cutoff: float = 5.0
    noisy: bool = False
    watchdog_timeout_ms: int = 200
    max_consecutive_errors: int = 5
    torque_ramp_seconds: float = 2.0
    max_motor_freq_hz: float = 10.0

    @classmethod
    def from_yaml(cls, path: str) -> "RealtimeControllerConfig":
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})


# ─── Controller State ────────────────────────────────────────────

class ControllerState(enum.Enum):
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    ERROR = "ERROR"
    STOPPED = "STOPPED"


# ─── Telemetry ───────────────────────────────────────────────────

@dataclass
class Telemetry:
    timestamp: float = 0.0
    loop_hz: float = 0.0
    inference_ms: float = 0.0
    obs: np.ndarray = field(default_factory=lambda: np.zeros(125))
    action: np.ndarray = field(default_factory=lambda: np.zeros(20))
    torque_right: float = 0.0
    torque_left: float = 0.0
    imu_ok: bool = False
    motor_ok: bool = False
    emg_ok: bool = False
    error_count: int = 0


# ─── Main Controller ─────────────────────────────────────────────

class RealtimeController:
    """Real-time RL control loop for ExoPulse exoskeleton."""

    def __init__(self, config: RealtimeControllerConfig):
        self.config = config
        self._state = ControllerState.IDLE
        self._agent = None
        self._bridge = None
        self._safety: TorqueSafetyPipeline | None = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._telemetry = Telemetry()
        self._telemetry_history = deque(maxlen=500)

        # Callbacks (for GUI integration)
        self.on_state_changed = None
        self.on_error = None

    # ── Properties ──

    @property
    def state(self) -> ControllerState:
        return self._state

    @property
    def is_running(self) -> bool:
        return self._state == ControllerState.RUNNING

    # ── Lifecycle ──

    def initialize(self) -> bool:
        """Load depRL agent and initialize hardware bridge."""
        self._set_state(ControllerState.INITIALIZING)
        cfg = self.config

        if not cfg.checkpoint_path:
            logger.error("checkpoint_path is empty")
            self._set_state(ControllerState.ERROR)
            return False

        # 1) Load agent
        try:
            logger.info("Loading agent from %s ...", cfg.checkpoint_path)
            from control.ai_agent.dummy_env import DummyExoEnv
            from deprl.utils.load_utils import load as deprl_load

            dummy = DummyExoEnv(obs_dim=cfg.obs_dim, action_dim=cfg.action_dim)
            self._agent = deprl_load(cfg.checkpoint_path, dummy)
            logger.info("Agent loaded successfully")
        except Exception as e:
            logger.error("Failed to load agent: %s", e, exc_info=True)
            self._set_state(ControllerState.ERROR)
            return False

        # 2) Initialize hardware bridge
        try:
            logger.info("Initializing hardware bridge ...")
            self._bridge = HardwareBridge({
                "enable_imu": cfg.enable_imu,
                "enable_motor": cfg.enable_motor,
                "enable_emg": cfg.enable_emg,
            })
            if not self._bridge.init():
                logger.error("HardwareBridge.init() returned False")
                self._set_state(ControllerState.ERROR)
                return False
            logger.info("Hardware bridge initialized")
        except Exception as e:
            logger.error("Failed to init hardware bridge: %s", e, exc_info=True)
            self._set_state(ControllerState.ERROR)
            return False

        # 3) Setup safety pipeline
        exo_indices = [18, 19]  # action indices for hip R/L

        if cfg.filter_type == "butterworth":
            smoothing = ButterworthFilter(
                cutoff_freq=cfg.butterworth_cutoff,
                sampling_freq=cfg.control_freq_hz,
            )
        else:
            smoothing = RealtimeSmoothingFilter(alpha=cfg.ema_alpha)

        motor_prot = MotorProtection(
            iq_limit=cfg.iq_limit,
            exo_indices=exo_indices,
            smoothing_filter=smoothing,
            torque_ramp_seconds=cfg.torque_ramp_seconds,
            external_scale=cfg.external_scale,
        )
        human_prot = HumanProtection()
        rate_limiter = MotorRateLimiter(
            max_motor_freq_hz=cfg.max_motor_freq_hz,
            action_dim=cfg.action_dim,
        )
        self._safety = TorqueSafetyPipeline(
            motor_protection=motor_prot,
            human_protection=human_prot,
            rate_limiter=rate_limiter,
            exo_indices=exo_indices,
        )
        logger.info("Safety pipeline: filter=%s, ramp=%.1fs, motor_freq<=%.0fHz",
                     cfg.filter_type, cfg.torque_ramp_seconds, cfg.max_motor_freq_hz)

        self._set_state(ControllerState.IDLE)
        logger.info("Initialization complete")
        return True

    def start(self):
        """Start the control loop in a background thread."""
        if self._state not in (ControllerState.IDLE, ControllerState.STOPPED):
            logger.warning("Cannot start from state %s", self._state)
            return

        if self._agent is None or self._bridge is None:
            logger.error("Must call initialize() before start()")
            return

        self._stop_event.clear()
        if self._safety:
            self._safety.reset()

        self._thread = threading.Thread(
            target=self._control_loop, name="ai_control_loop", daemon=True
        )
        self._thread.start()
        self._set_state(ControllerState.RUNNING)
        logger.info("Control loop started at %.0f Hz", self.config.control_freq_hz)

    def stop(self):
        """Graceful shutdown: ramp down torque, then stop motors."""
        if not self.is_running:
            return

        logger.info("Stopping control loop ...")
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

        # Send zero torque
        self._send_zero_torque()
        self._set_state(ControllerState.STOPPED)
        logger.info("Control loop stopped")

    def emergency_stop(self):
        """Immediately cut all motor torque."""
        logger.warning("EMERGENCY STOP triggered")
        self._stop_event.set()
        self._send_zero_torque()
        self._set_state(ControllerState.STOPPED)

    def shutdown(self):
        """Release all resources."""
        self.stop()
        if self._bridge:
            self._bridge.close()
            self._bridge = None
        self._agent = None

    def get_telemetry(self) -> dict:
        """Thread-safe telemetry snapshot for GUI."""
        with self._lock:
            t = self._telemetry
            return {
                "timestamp": t.timestamp,
                "state": self._state.value,
                "loop_hz": round(t.loop_hz, 1),
                "inference_ms": round(t.inference_ms, 2),
                "torque_right": round(t.torque_right, 1),
                "torque_left": round(t.torque_left, 1),
                "imu_ok": t.imu_ok,
                "motor_ok": t.motor_ok,
                "emg_ok": t.emg_ok,
                "error_count": t.error_count,
                "obs": t.obs.copy(),
                "action": t.action.copy(),
            }

    # ── Control Loop (runs in thread) ──

    def _control_loop(self):
        cfg = self.config
        period = 1.0 / cfg.control_freq_hz
        consecutive_errors = 0
        step_count = 0
        start_time = time.monotonic()
        last_tick = start_time

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            try:
                # 1) Construct observation
                obs = np.zeros(cfg.obs_dim, dtype=np.float32)

                # 2) Inject real sensor data
                self._bridge.inject(obs)

                # 3) Run RL inference
                t0 = time.monotonic()
                if cfg.noisy:
                    action = self._agent.noisy_test_step(obs, steps=int(1e6))
                else:
                    action = self._agent.test_step(obs, steps=int(1e6))
                action = np.asarray(action, dtype=np.float32).flatten()
                inference_ms = (time.monotonic() - t0) * 1000.0

                # 4) Safety pipeline (clamp → filter → ramp → human protection)
                elapsed = time.monotonic() - start_time
                action = self._safety.apply(action, elapsed)

                # 5) Accumulate into rate limiter; send if interval reached
                now = time.monotonic()
                motor_action = self._safety.accumulate_and_maybe_send(action, now)
                if motor_action is not None:
                    self._bridge.apply_action(
                        motor_action,
                        external_scale=cfg.external_scale,
                        iq_limit=cfg.iq_limit,
                    )

                # 6) Compute actual loop rate
                dt = now - last_tick
                loop_hz = 1.0 / dt if dt > 0 else 0.0
                last_tick = now

                # 7) Compute torque values for telemetry (use last sent action)
                sent = self._safety.rate_limiter.last_sent_action
                torque_r = float(np.clip(sent[18] * cfg.external_scale, -cfg.iq_limit, cfg.iq_limit))
                torque_l = float(np.clip(sent[19] * cfg.external_scale, -cfg.iq_limit, cfg.iq_limit))

                # 8) Update telemetry
                with self._lock:
                    self._telemetry = Telemetry(
                        timestamp=now,
                        loop_hz=loop_hz,
                        inference_ms=inference_ms,
                        obs=obs,
                        action=action,
                        torque_right=torque_r,
                        torque_left=torque_l,
                        imu_ok=cfg.enable_imu,
                        motor_ok=cfg.enable_motor,
                        emg_ok=cfg.enable_emg,
                        error_count=consecutive_errors,
                    )

                consecutive_errors = 0
                step_count += 1

            except Exception as e:
                consecutive_errors += 1
                logger.error("Control loop error (%d/%d): %s",
                             consecutive_errors, cfg.max_consecutive_errors, e)

                if consecutive_errors >= cfg.max_consecutive_errors:
                    logger.critical("Max consecutive errors reached — emergency stop")
                    self._send_zero_torque()
                    self._set_state(ControllerState.ERROR)
                    if self.on_error:
                        self.on_error(str(e))
                    return

            # Sleep for remainder of period
            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ── Helpers ──

    def _send_zero_torque(self):
        """Send zero torque to all motors."""
        if self._bridge and self._bridge._initialized:
            try:
                zero_action = np.zeros(self.config.action_dim, dtype=np.float32)
                self._bridge.apply_action(zero_action, external_scale=1.0, iq_limit=0)
            except Exception as e:
                logger.error("Failed to send zero torque: %s", e)

    def _set_state(self, new_state: ControllerState):
        old = self._state
        self._state = new_state
        if old != new_state:
            logger.info("State: %s -> %s", old.value, new_state.value)
            if self.on_state_changed:
                self.on_state_changed(new_state)


# ─── CLI Entry Point ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="ExoPulse AI Real-time Controller")
    parser.add_argument("--config", type=str,
                        default=str(Path(__file__).parent / "config.yaml"),
                        help="Path to config YAML file")
    parser.add_argument("--checkpoint", type=str, default=None,
                        help="Override checkpoint path from config")
    parser.add_argument("--freq", type=float, default=None,
                        help="Override control frequency (Hz)")
    parser.add_argument("--iq-limit", type=int, default=None,
                        help="Override torque limit")
    parser.add_argument("--dry-run", action="store_true",
                        help="Load agent only, do not start control loop")
    args = parser.parse_args()

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # Load config
    cfg = RealtimeControllerConfig.from_yaml(args.config)
    if args.checkpoint:
        cfg.checkpoint_path = args.checkpoint
    if args.freq:
        cfg.control_freq_hz = args.freq
    if args.iq_limit:
        cfg.iq_limit = args.iq_limit

    logger.info("Config: checkpoint=%s, freq=%.0fHz, iq_limit=%d, filter=%s",
                cfg.checkpoint_path, cfg.control_freq_hz, cfg.iq_limit, cfg.filter_type)

    # Create controller
    ctrl = RealtimeController(cfg)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        logger.info("Caught signal %d, stopping ...", sig)
        ctrl.emergency_stop()
        ctrl.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize
    if not ctrl.initialize():
        logger.error("Initialization failed, exiting")
        sys.exit(1)

    if args.dry_run:
        logger.info("Dry run complete — agent loaded successfully")
        ctrl.shutdown()
        return

    # Start control loop
    ctrl.start()
    logger.info("Control loop running. Press Ctrl+C to stop.")

    # Main thread: print telemetry every second
    try:
        while ctrl.is_running:
            time.sleep(1.0)
            t = ctrl.get_telemetry()
            logger.info(
                "Hz=%.1f | infer=%.1fms | torque R=%.0f L=%.0f | errors=%d",
                t["loop_hz"], t["inference_ms"],
                t["torque_right"], t["torque_left"],
                t["error_count"],
            )
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.emergency_stop()
        ctrl.shutdown()


if __name__ == "__main__":
    main()
