# ExoPulse Firmware

ESP32 firmware for ExoPulse exoskeleton motor control using **MGv2 (LK-TECH) motor drivers** via CAN bus.

## Overview

This project provides real-time motor control and monitoring for dual-motor exoskeleton systems. It features:

- **Dual motor control** via CAN bus (CAN IDs: 0x141, 0x142)
- **Real-time data acquisition**: temperature, current, voltage, speed, acceleration, multi-turn angle
- **Software calibration** for zero-point setting (no ROM write, unlimited calibrations)
- **FreeRTOS multi-tasking** for efficient ESP32 dual-core utilization
- **Modular UI components** (low-level control + high-level monitoring)
- **WiFi and UART communication** modes

## Hardware

- **MCU**: ESP32 DevKit v1 (dual-core, 240 MHz)
- **Motor Drivers**: MGv2 (LK-TECH) with CAN bus interface
- **CAN Transceiver**: MCP2515 + TJA1050
- **Communication**: 1Mbps CAN bus
- **EMG Sensor**: ADS1256 24-bit ADC (optional)

### Motor Features (MGv2)
- Multi-turn angle tracking (±360° * N turns)
- Acceleration data via Command 0x33
- Temperature and current monitoring
- Software zero-position calibration

## Quick Start

### 1. Hardware Setup

Connect CAN transceiver to ESP32:
```
ESP32 GPIO5  → MCP2515 CS
ESP32 GPIO23 → MCP2515 SI/MOSI
ESP32 GPIO19 → MCP2515 SO/MISO
ESP32 GPIO18 → MCP2515 SCK
ESP32 GPIO21 → MCP2515 INT
```

See [MGv2/README.md](MGv2/README.md) for detailed hardware documentation.

### 2. Build and Upload Firmware

```bash
# Install PlatformIO
pip install platformio

# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### 3. Run User Interface

**Unified GUI Launcher (Recommended)**
```bash
# Launch the unified control panel with sidebar navigation
python3 gui.py
```

The unified GUI provides:
- 🎨 **Smooth sidebar navigation** - Easy switching between all components
- 🚀 **Integrated launcher** - Start/stop components with one click
- 📊 **Real-time process monitoring** - View component output logs
- 📁 **Component organization** - Clear separation of low-level and high-level tools

**Direct Component Launch**
```bash
# Low-level: Basic motor control
python3 UI_components/motor_control.py

# High-level: Advanced monitoring
python3 UI_components/motor_monitor.py
```

See [UI_components/README.md](UI_components/README.md) for detailed UI documentation.

## Project Structure

```
ExoPulse_firmware/
├── gui.py                          # Unified GUI launcher with sidebar (entry point)
├── src/ -> MGv2/src/               # Symlink to firmware source
├── MGv2/                           # MGv2 motor driver firmware
│   ├── src/
│   │   ├── main.cpp                # ESP32 firmware (MGv2 motors)
│   │   ├── main.h                  # Header file
│   │   ├── main_wifi.cpp           # WiFi mode firmware
│   │   └── main_wifi_motor.cpp     # WiFi motor control
│   ├── test/                       # Test programs
│   ├── README.md                   # MGv2 documentation
│   └── 20230220145958f_datasheet_protocol.pdf
├── UI_components/                  # User interface components
│   ├── motor_control.py            # [Low-level] Basic motor control GUI
│   ├── motor_monitor.py            # [High-level] Advanced monitoring GUI
│   ├── dual_motor_plotter.py       # [High-level] Real-time dual motor plotter
│   ├── wifi_monitor.py             # [High-level] WiFi-based monitor
│   ├── wifi_dual_motor_plotter.py  # [High-level] WiFi dual motor plotter
│   ├── can_plotter.py              # [High-level] CAN bus data plotter
│   ├── emg_plotter.py              # [High-level] EMG signal plotter
│   ├── serial_reader.py            # [Low-level] Simple serial reader
│   └── README.md                   # UI components documentation
├── EMG/                            # EMG signal acquisition module
│   ├── src/ADS.cpp                 # ADS1256 ADC driver
│   ├── lib/ADS1256/                # ADS1256 library
│   └── README.md                   # EMG module documentation
├── RMD_motor_legacy/               # Legacy RMD motor code (archived)
│   └── README.md                   # Legacy documentation
├── docs/                           # Additional documentation
│   └── MGv2/                       # MGv2-specific docs
├── include/                        # Shared header files
│   └── CAN_commands.h              # CAN protocol definitions
├── QUICK_START_WIFI.md             # WiFi mode quick start
├── WIFI_MOTOR_GUIDE.md             # WiFi motor control guide
└── platformio.ini                  # PlatformIO configuration
```

## User Interface Components

The project provides modular UI components organized by complexity:

### Low-Level Components (Basic Control)
- **motor_control.py** - PySide6 GUI for basic motor control
- **serial_reader.py** - Command-line serial data reader

### High-Level Components (Advanced Monitoring)
- **motor_monitor.py** - Comprehensive monitoring with configurable plots
- **dual_motor_plotter.py** - Real-time dual motor visualization
- **wifi_monitor.py** - Remote monitoring via WiFi
- **emg_plotter.py** - EMG signal visualization

See [UI_components/README.md](UI_components/README.md) for component selection guide.

## Communication Modes

### UART Mode (Default)
- Direct serial communication via USB
- Serial ports: ttyUSB devices only
- Baud rate: 115200

### WiFi Mode
- TCP/UDP socket communication
- Remote monitoring and control
- See [QUICK_START_WIFI.md](QUICK_START_WIFI.md)

## Serial Commands

The firmware supports the following serial commands:

### Motor Control
- `CAL1` / `CAL_M1` - Calibrate Motor 1 zero position (software offset)
- `CAL2` / `CAL_M2` - Calibrate Motor 2 zero position
- `CLEAR_CAL` - Clear all calibration offsets

### System Commands
- `HELP` - Display available commands
- `STATUS` - Show system status
- `DETAILED` - Enable detailed debug output

## Software Calibration

The firmware implements **software angle calibration** (Command: CAL1/CAL2/CLEAR_CAL):

**Advantages**:
- ✅ No ROM write (unlimited calibrations)
- ✅ Instant calibration (no reboot required)
- ✅ Easily reversible with CLEAR_CAL
- ✅ Perfect for testing and development

**Note**: Calibration offsets reset on ESP32 reboot (by design for safety).

## Dependencies

### Firmware
- PlatformIO
- ESP32 Arduino framework
- mcp_can library

### UI Components
```bash
pip3 install PySide6 matplotlib pyserial numpy
```

See [requirements.txt](requirements.txt) for complete list.

## Documentation

- **[UI_components/README.md](UI_components/README.md)** - User interface documentation
- **[MGv2/README.md](MGv2/README.md)** - MGv2 motor firmware documentation
- **[EMG/README.md](EMG/README.md)** - EMG signal acquisition documentation
- **[QUICK_START_WIFI.md](QUICK_START_WIFI.md)** - WiFi mode quick start
- **[WIFI_MOTOR_GUIDE.md](WIFI_MOTOR_GUIDE.md)** - WiFi motor control guide
- **[RMD_motor_legacy/README.md](RMD_motor_legacy/README.md)** - Legacy RMD documentation

## Troubleshooting

### Serial Port Issues
If you encounter "port already in use" errors:
1. Close all other programs using the port
2. The GUI includes automatic port cleanup on startup
3. Check with: `lsof /dev/ttyUSB0`

### Connection Lost
The monitoring GUIs include auto-reconnect (max 5 attempts):
- Connection status shown with color indicators (🟢🟡🔴)
- Automatically reconnects if cable is moved or disconnected

### Build Errors
Check PlatformIO configuration in [platformio.ini](platformio.ini).

## Development History

- **November 2025**: Project reorganization
  - Created modular UI_components structure
  - Separated low-level and high-level components
  - Added WiFi communication mode
- **Earlier**: Transitioned from RMD to MGv2 (LK-TECH) motor drivers
  - Added software calibration feature
  - Implemented robust serial port error handling
  - Enhanced GUI with auto-reconnect
- **Previous**: RMD motor development (see `RMD_motor_legacy/`)

## Modular HAL (`sensing/` + `control/`)

Modular hardware abstraction layer for Jetson Orin, designed for future FPGA migration.
Each module uses a unified `create/init/read(or write)/destroy` pattern with C-linkage for Python ctypes compatibility.

### Architecture

```
ExoPulse_firmware/
├── sensing/                    # Sensor modules
│   ├── imu/                    # MPU6050 6-axis IMU (I2C)
│   │   ├── mpu6050.h/.cpp     # libexo_imu.so
│   │   └── test_mpu6050.cpp
│   ├── emg/                    # BLE EMG 4-channel (BLE)
│   │   ├── emg_interface.py    # Abstract base class
│   │   ├── ble_emg_module.py   # BLE implementation
│   │   └── test_emg.py
│   └── README.md               # Sensing API reference
├── control/                    # Actuator modules
│   ├── motor/                  # LK-TECH motor via MCP2515 CAN (SPI)
│   │   ├── mcp2515.h/.cpp     # CAN controller driver
│   │   ├── lktech_motor.h/.cpp # Motor protocol
│   │   ├── Makefile            # libexo_motor.so
│   │   └── test_motor.cpp
│   └── README.md               # Control API reference
└── module_test/                # Original standalone tests (reference)
```

### Hardware Topology

```
Jetson Orin I2C (bus 7) ──> MPU6050 (6-axis IMU)
Jetson Orin SPI ──> MCP2515 ──> CAN Bus ──> MGv2 Motor x2 (0x141, 0x142)
Jetson Orin BLE ──> EMG2ch_B x2 (4-channel EMG)
```

### Quick Build

```bash
make -C sensing/imu      # -> libexo_imu.so + test_mpu6050
make -C control/motor     # -> libexo_motor.so + test_motor
# EMG is pure Python: python3 sensing/emg/test_emg.py
```

### depRL Integration API

This section documents the API contract for the `depRL` reinforcement learning framework.
depRL code is **not modified** — this serves as a handoff specification.

#### Sensor API Summary

| Module | Language | Init | Read | Output |
|--------|----------|------|------|--------|
| IMU | C (.so) | `mpu6050_create` + `mpu6050_init` | `mpu6050_read` | `imu_data_t`: accel[3], gyro[3], temp, timestamp |
| EMG | Python | `BLEEMGModule().init()` | `.read()` | `np.ndarray(4,)`: raw ADC [ABE_T2, ABE_T4, ABB_T2, ABB_T4] |
| Motor | C (.so) | `motor_create` + `motor_init` | `motor_read_status` | `motor_status_t`: temp, iq, speed, accel, angle, encoder |

#### depRL Observation Vector Mapping

Based on `external_input_config_e6b_4emg_imu.txt` (15-dim external input):

| Indices | Source | Description |
|---------|--------|-------------|
| 54, 58, 63, 67 | EMG `read()` | Thigh EMG: hamstrings_r, rect_fem_r, hamstrings_l, rect_fem_l |
| 72, 73, 74, 75 | IMU `accel/gyro` | Trunk IMU orientation quaternion (qw, qx, qy, qz) |
| 76, 77, 78 | IMU `gyro` | Trunk IMU angular velocity (x, y, z) |
| 121, 123 | Motor `read_status` | hip_exo_l_pos, hip_exo_l_vel |
| 122, 124 | Motor `read_status` | hip_exo_r_pos, hip_exo_r_vel |

#### depRL Action Mapping

Based on `scone_wrapper.py` (external_scale=45.0):

| Action Index | Target | Mapping |
|-------------|--------|---------|
| action[18] | Motor 0x141 (right hip) | `iq = clamp(action * 45.0, -800, 800)` then `motor_set_torque(motor1, iq)` |
| action[19] | Motor 0x142 (left hip) | `iq = clamp(action * 45.0, -800, 800)` then `motor_set_torque(motor2, iq)` |

#### Python ctypes Integration Example

```python
import ctypes
import numpy as np
from sensing.emg.ble_emg_module import BLEEMGModule

# --- Load shared libraries ---
imu_lib = ctypes.CDLL("sensing/imu/libexo_imu.so")
motor_lib = ctypes.CDLL("control/motor/libexo_motor.so")

# --- C structures ---
class ImuData(ctypes.Structure):
    _fields_ = [
        ("accel", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("temperature", ctypes.c_float),
        ("timestamp", ctypes.c_double),
    ]

class MotorStatus(ctypes.Structure):
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

# --- Setup function signatures ---
# IMU
imu_lib.mpu6050_create.restype = ctypes.c_void_p
imu_lib.mpu6050_create.argtypes = [ctypes.c_char_p, ctypes.c_uint8]
imu_lib.mpu6050_init.restype = ctypes.c_int
imu_lib.mpu6050_init.argtypes = [ctypes.c_void_p]
imu_lib.mpu6050_read.restype = ctypes.c_int
imu_lib.mpu6050_read.argtypes = [ctypes.c_void_p, ctypes.POINTER(ImuData)]
imu_lib.mpu6050_destroy.argtypes = [ctypes.c_void_p]

# MCP2515
motor_lib.mcp2515_create.restype = ctypes.c_void_p
motor_lib.mcp2515_create.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
motor_lib.mcp2515_init.restype = ctypes.c_int
motor_lib.mcp2515_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
motor_lib.mcp2515_destroy.argtypes = [ctypes.c_void_p]

# Motor
motor_lib.motor_create.restype = ctypes.c_void_p
motor_lib.motor_create.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
motor_lib.motor_init.restype = ctypes.c_int
motor_lib.motor_init.argtypes = [ctypes.c_void_p]
motor_lib.motor_set_torque.restype = ctypes.c_int
motor_lib.motor_set_torque.argtypes = [ctypes.c_void_p, ctypes.c_int16, ctypes.POINTER(MotorStatus)]
motor_lib.motor_read_status.restype = ctypes.c_int
motor_lib.motor_read_status.argtypes = [ctypes.c_void_p, ctypes.POINTER(MotorStatus)]
motor_lib.motor_stop.restype = ctypes.c_int
motor_lib.motor_stop.argtypes = [ctypes.c_void_p]
motor_lib.motor_destroy.argtypes = [ctypes.c_void_p]

# --- Initialize all hardware ---
imu = imu_lib.mpu6050_create(b"/dev/i2c-7", 0x68)
imu_lib.mpu6050_init(imu)

can = motor_lib.mcp2515_create(b"/dev/spidev0.0", 1000000)
motor_lib.mcp2515_init(can, 1000000)

motor1 = motor_lib.motor_create(can, 0x141)
motor2 = motor_lib.motor_create(can, 0x142)
motor_lib.motor_init(motor1)
motor_lib.motor_init(motor2)

emg = BLEEMGModule()
emg.init()

# --- Read loop ---
imu_data = ImuData()
motor_status = MotorStatus()

imu_lib.mpu6050_read(imu, ctypes.byref(imu_data))
emg_data = emg.read()  # np.ndarray(4,)
motor_lib.motor_read_status(motor1, ctypes.byref(motor_status))

# --- Send torque ---
EXTERNAL_SCALE = 45.0
IQ_LIMIT = 800

def apply_action(action_18, action_19):
    iq1 = int(max(-IQ_LIMIT, min(IQ_LIMIT, action_18 * EXTERNAL_SCALE)))
    iq2 = int(max(-IQ_LIMIT, min(IQ_LIMIT, action_19 * EXTERNAL_SCALE)))
    motor_lib.motor_set_torque(motor1, iq1, None)
    motor_lib.motor_set_torque(motor2, iq2, None)

# --- Cleanup ---
motor_lib.motor_stop(motor1)
motor_lib.motor_stop(motor2)
motor_lib.motor_destroy(motor1)
motor_lib.motor_destroy(motor2)
motor_lib.mcp2515_destroy(can)
imu_lib.mpu6050_destroy(imu)
emg.close()
```

See [sensing/README.md](sensing/README.md) and [control/README.md](control/README.md) for detailed API documentation.

---

## Related Modules

- **[MGv2/](MGv2/)** - MGv2 motor control firmware and documentation (ESP32, original)
- **[EMG/](EMG/)** - EMG signal acquisition using ADS1256 ADC (24-bit, 8-channel)
- **[RMD_motor_legacy/](RMD_motor_legacy/)** - Archived RMD motor control code

## License

This project is part of the ExoPulse exoskeleton research platform.

## References

- MGv2 Motor Datasheet: [MGv2/20230220145958f_datasheet_protocol.pdf](MGv2/20230220145958f_datasheet_protocol.pdf)
- CAN Protocol: [include/CAN_commands.h](include/CAN_commands.h)
