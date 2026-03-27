"""
Hardware Bridge: Real sensor data -> depRL observation vector

Bridges firmware_layer API (SensorAPI, MotorAPI) into the depRL observation
vector, replacing SCONE simulation values at the external input indices.

Uses the unified firmware_layer transports (IMUI2CTransport, CANDirectTransport)
instead of raw ctypes calls, ensuring consistent init, unit conversion, and
thread safety with the cloud version.

Usage:
    bridge = HardwareBridge(config)
    bridge.init()
    ...
    # In scone_wrapper.py step(), before obs splitting:
    bridge.inject(observation)  # modifies observation in-place
    ...
    bridge.close()

Observation index mapping (from external_input_config files):
    EMG:   [54, 58, 63, 67]     -> thigh muscle activations
    IMU:   [72:76] quat, [76:79] gyro  -> trunk orientation/angular velocity
    Motor: [121:125]            -> hip exo pos/vel (L/R)
"""

import math
import numpy as np
import os
import sys
import logging

logger = logging.getLogger(__name__)

# Resolve paths relative to this file (firmware_layer/UI_components/)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SENSORS_DIR = os.path.join(_THIS_DIR, "sensors")
_TRANSPORTS_DIR = os.path.join(_THIS_DIR, "transports")

# Legacy env var for EMG module
_FIRMWARE_BASE = os.environ.get(
    "EXOPULSE_FIRMWARE_PATH",
    os.path.expanduser("~/Documents/ExoPulse_firmware"),
)

DEG2RAD = math.pi / 180.0


def _ensure_import_paths():
    """Ensure sibling directories are importable."""
    for p in (_THIS_DIR, _SENSORS_DIR, _TRANSPORTS_DIR):
        if p not in sys.path:
            sys.path.insert(0, p)


# ============== Hardware Bridge ==============

class HardwareBridge:
    """Reads real hardware and injects data into the depRL observation vector.

    Delegates to firmware_layer's SensorAPI/MotorAPI for proper init sequences,
    unit conversions, and thread-safe CAN access.
    """

    def __init__(self, config=None):
        """
        config dict keys (all optional, with defaults):
            enable_imu: bool (True)
            enable_motor: bool (True)
            enable_emg: bool (False)  -- requires BLE, disabled by default
            i2c_device: str ("/dev/i2c-7")
            i2c_address: int (0x68)
            imu_sample_rate: int (200) -- Hz
            spi_device: str ("/dev/spidev0.0")
            can_baud: int (1_000_000)
            motor_ids: list[int] ([0x141, 0x142])
            imu_gyro_unit: str ("rad/s")  -- SCONE uses rad/s
            axis_remap: np.ndarray (3x3) -- rotation matrix for IMU->SCONE frame
        """
        cfg = config or {}
        self.enable_imu = cfg.get("enable_imu", True)
        self.enable_motor = cfg.get("enable_motor", True)
        self.enable_emg = cfg.get("enable_emg", False)

        # IMU config
        self._i2c_device = cfg.get("i2c_device", "/dev/i2c-7")
        self._i2c_address = cfg.get("i2c_address", 0x68)
        self._imu_sample_rate = cfg.get("imu_sample_rate", 200)
        self._imu_gyro_unit = cfg.get("imu_gyro_unit", "rad/s")
        self._axis_remap = cfg.get("axis_remap", None)  # 3x3 rotation matrix

        # Motor config
        self._spi_device = cfg.get("spi_device", "/dev/spidev0.0")
        self._can_baud = cfg.get("can_baud", 1_000_000)
        self._motor_ids = cfg.get("motor_ids", [0x141, 0x142])

        # firmware_layer handles (set in init)
        self._sensor_api = None   # SensorAPI instance
        self._motor_api = None    # MotorAPI instance
        self._imu_transport = None
        self._motor_transport = None
        self._emg = None

        self._initialized = False

    # ---- Lifecycle ----

    def init(self):
        """Initialize all enabled hardware modules. Call once before inject()."""
        if self._initialized:
            logger.warning("[HardwareBridge] Already initialized")
            return True

        _ensure_import_paths()

        success = True

        if self.enable_imu:
            if not self._init_imu():
                logger.error("[HardwareBridge] IMU init failed")
                success = False

        if self.enable_motor:
            if not self._init_motor():
                logger.error("[HardwareBridge] Motor init failed")
                success = False

        if self.enable_emg:
            if not self._init_emg():
                logger.error("[HardwareBridge] EMG init failed")
                success = False

        self._initialized = success
        if success:
            logger.info("[HardwareBridge] Initialized (IMU=%s, Motor=%s, EMG=%s)",
                        self.enable_imu, self.enable_motor, self.enable_emg)
        return success

    def close(self):
        """Release all hardware resources."""
        if self.enable_motor and self._motor_api:
            try:
                self._motor_api.stop_all()
            except Exception:
                pass
            if self._motor_transport:
                try:
                    self._motor_transport.disconnect()
                except Exception:
                    pass

        if self.enable_imu and self._sensor_api:
            try:
                self._sensor_api.shutdown()
            except Exception:
                pass

        if self.enable_emg and self._emg:
            try:
                self._emg.close()
            except Exception:
                pass

        self._initialized = False
        logger.info("[HardwareBridge] Closed")

    # ---- Core: inject real data into observation ----

    def inject(self, observation):
        """Inject real hardware data into the observation vector (in-place).

        Args:
            observation: np.ndarray of shape (125,) or larger.
                         Modified in-place at the external input indices.
        """
        if not self._initialized:
            return

        if self.enable_imu:
            self._inject_imu(observation)

        if self.enable_motor:
            self._inject_motor(observation)

        if self.enable_emg:
            self._inject_emg(observation)

    def read_all(self):
        """Read all enabled sensors and return as dict (for debugging).

        Returns:
            dict with keys: imu_quat, imu_gyro, imu_accel, motor_pos, motor_vel, emg
        """
        result = {}

        if self.enable_imu and self._sensor_api:
            imu_data = self._sensor_api.get_imu("imu_trunk")
            if imu_data and imu_data.success:
                quat = np.array(imu_data.quaternion)
                gyro = np.array(imu_data.gyro)
                if self._imu_gyro_unit == "rad/s":
                    gyro = gyro * DEG2RAD
                if self._axis_remap is not None:
                    gyro = self._axis_remap @ gyro
                result["imu_quat"] = quat
                result["imu_gyro"] = gyro
                result["imu_accel"] = np.array(imu_data.accel)

        if self.enable_motor and self._motor_api:
            positions = np.zeros(2)
            velocities = np.zeros(2)
            # motor_id 1 = 0x141 = right, motor_id 2 = 0x142 = left
            for i, motor_id in enumerate([1, 2]):
                resp = self._motor_api.read_status(motor_id)
                if resp.success:
                    positions[i] = resp.angle      # already in degrees
                    velocities[i] = resp.speed     # already in deg/s
            result["motor_pos"] = positions
            result["motor_vel"] = velocities

        if self.enable_emg and self._emg:
            result["emg"] = self._emg.read()

        return result

    # ---- Private: sensor injection ----

    def _inject_imu(self, obs):
        """Read IMU via SensorAPI and write into obs[72:79]."""
        imu_data = self._sensor_api.get_imu("imu_trunk")
        if not imu_data or not imu_data.success:
            return

        # Quaternion [qw, qx, qy, qz] -> obs[72:76]
        quat = np.array(imu_data.quaternion, dtype=np.float32)
        obs[72:76] = quat

        # Gyroscope -> obs[76:79]
        gyro = np.array(imu_data.gyro, dtype=np.float32)

        # Unit conversion: firmware_layer outputs deg/s, SCONE uses rad/s
        if self._imu_gyro_unit == "rad/s":
            gyro *= DEG2RAD

        # Axis remap: align MPU6050 frame to SCONE trunk frame
        if self._axis_remap is not None:
            gyro = (self._axis_remap @ gyro).astype(np.float32)

        obs[76:79] = gyro

    def _inject_motor(self, obs):
        """Read motor status via MotorAPI and write into obs[121:125].

        SCONE mapping (from external_input_config_e6b_4emg_imu.txt):
            obs[121] = hip_exo_l_pos
            obs[122] = hip_exo_r_pos
            obs[123] = hip_exo_l_vel
            obs[124] = hip_exo_r_vel

        Hardware mapping (via CANDirectTransport):
            motor_id 1 (0x141) = right hip
            motor_id 2 (0x142) = left hip
        """
        # Right hip: motor_id 1 (0x141)
        resp_r = self._motor_api.read_status(1)
        if resp_r.success:
            obs[122] = resp_r.angle   # already in degrees (0.01 deg/LSB converted)
            obs[124] = resp_r.speed   # already in deg/s

        # Left hip: motor_id 2 (0x142)
        resp_l = self._motor_api.read_status(2)
        if resp_l.success:
            obs[121] = resp_l.angle
            obs[123] = resp_l.speed

    def _inject_emg(self, obs):
        """Read BLE EMG and write into obs[54, 58, 63, 67].

        SCONE mapping:
            obs[54] = hamstrings_r  -> ABE_T2
            obs[58] = rect_fem_r   -> ABE_T4
            obs[63] = hamstrings_l  -> ABB_T2
            obs[67] = rect_fem_l   -> ABB_T4
        """
        if self._emg is None:
            return

        emg_data = self._emg.read()  # np.ndarray(4,): [ABE_T2, ABE_T4, ABB_T2, ABB_T4]
        obs[54] = emg_data[0]  # hamstrings_r
        obs[58] = emg_data[1]  # rect_fem_r
        obs[63] = emg_data[2]  # hamstrings_l
        obs[67] = emg_data[3]  # rect_fem_l

    # ---- Private: initialization ----

    def _init_imu(self):
        """Initialize IMU via firmware_layer's IMUI2CTransport + SensorAPI."""
        try:
            from sensor_api import SensorAPI
            from imu_i2c import IMUI2CTransport

            self._imu_transport = IMUI2CTransport(
                i2c_device=self._i2c_device,
                address=self._i2c_address,
                sample_rate_hz=self._imu_sample_rate,
            )
            ok, msg = self._imu_transport.connect()
            if not ok:
                logger.error("[HardwareBridge] IMU connect failed: %s", msg)
                return False

            self._sensor_api = SensorAPI()
            self._sensor_api.add_sensor("imu_trunk", self._imu_transport)

            logger.info("[HardwareBridge] IMU initialized via SensorAPI (%s)", msg)
            return True
        except ImportError as e:
            logger.error("[HardwareBridge] Cannot import firmware_layer sensor modules: %s", e)
            return False
        except Exception as e:
            logger.error("[HardwareBridge] IMU init error: %s", e)
            return False

    def _init_motor(self):
        """Initialize motors via firmware_layer's CANDirectTransport + MotorAPI."""
        try:
            from motor_api import MotorAPI
            from motor_types import SafetyConfig
            from can_direct import CANDirectTransport
            from motor_controller import DirectTorqueController

            self._motor_transport = CANDirectTransport(
                spi_device=self._spi_device.encode()
                    if isinstance(self._spi_device, str) else self._spi_device,
                baud_rate=self._can_baud,
            )
            ok, msg = self._motor_transport.connect(motor_ids=tuple(self._motor_ids))
            if not ok:
                logger.error("[HardwareBridge] Motor connect failed: %s", msg)
                return False

            safety = SafetyConfig(max_iq=800)
            self._motor_api = MotorAPI(self._motor_transport, safety=safety)

            # Setup torque controllers for both motors
            # motor_id 1 = 0x141 (right hip), motor_id 2 = 0x142 (left hip)
            for motor_id in [1, 2]:
                self._motor_api.set_controller(motor_id, DirectTorqueController)

            logger.info("[HardwareBridge] Motors initialized via MotorAPI (%s)", msg)
            return True
        except ImportError as e:
            logger.error("[HardwareBridge] Cannot import firmware_layer motor modules: %s", e)
            return False
        except Exception as e:
            logger.error("[HardwareBridge] Motor init error: %s", e)
            return False

    def _init_emg(self):
        """Initialize BLE EMG module."""
        try:
            emg_path = os.path.join(_FIRMWARE_BASE, "sensing", "emg")
            if emg_path not in sys.path:
                sys.path.insert(0, emg_path)
            from ble_emg_module import BLEEMGModule

            self._emg = BLEEMGModule()
            ok = self._emg.init()
            if not ok:
                logger.error("[HardwareBridge] BLE EMG init failed (no devices found)")
                self._emg = None
                return False
            logger.info("[HardwareBridge] EMG initialized (4 channels)")
            return True
        except ImportError as e:
            logger.error("[HardwareBridge] Cannot import BLEEMGModule: %s", e)
            return False

    # ---- Action output (motor torque) ----

    def apply_action(self, action, external_scale=45.0, iq_limit=800):
        """Send RL action to motors via MotorAPI.

        Args:
            action: full action vector from agent
            external_scale: multiplier (default 45.0 from scone_wrapper)
            iq_limit: torque current safety clamp (default 800 ~ 12.8A)

        Mapping:
            action[18] -> motor_id 1 (0x141, right hip)
            action[19] -> motor_id 2 (0x142, left hip)
        """
        if not self.enable_motor or not self._motor_api:
            return

        for action_idx, motor_id in zip([18, 19], [1, 2]):
            if action_idx >= len(action):
                continue

            iq_raw = action[action_idx] * external_scale
            iq = int(max(-iq_limit, min(iq_limit, iq_raw)))
            self._motor_api.set_torque(motor_id, iq)
