# ExoPulse Firmware

ESP32 firmware for ExoPulse exoskeleton motor control using **MGv2 (LK-TECH) motor drivers** via CAN bus.

## Overview

This project provides real-time motor control and monitoring for dual-motor exoskeleton systems. It features:

- **Dual motor control** via CAN bus (CAN IDs: 0x141, 0x142)
- **Real-time data acquisition**: temperature, current, voltage, speed, acceleration, multi-turn angle
- **Software calibration** for zero-point setting (no ROM write, unlimited calibrations)
- **FreeRTOS multi-tasking** for efficient ESP32 dual-core utilization
- **Live monitoring GUI** with matplotlib visualization

## Hardware

- **MCU**: ESP32 DevKit v1 (dual-core, 240 MHz)
- **Motor Drivers**: MGv2 (LK-TECH) with CAN bus interface
- **CAN Transceiver**: MCP2515 + TJA1050
- **Communication**: 1Mbps CAN bus

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

See [QUICK_START_UBUNTU.md](QUICK_START_UBUNTU.md) for detailed setup.

### 2. Build and Upload

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

### 3. Run GUI Monitor

```bash
# Install dependencies
pip install pyserial matplotlib

# Run enhanced dual-motor monitor
python3 src/monitor_dual_motor_enhanced.py
```

The GUI provides:
- Real-time plots for both motors (5 parameters each)
- Software calibration buttons (Cal Motor 1/2, Cal Both, Clear Cal)
- Connection status indicator (🟢🟡🔴)
- Auto-reconnect on cable disconnection

## Project Structure

```
ExoPulse_firmware/
├── src/
│   ├── main.cpp                         # ESP32 firmware (MGv2 motors)
│   ├── main.h                           # Header file
│   └── monitor_dual_motor_enhanced.py   # Real-time GUI monitor
├── test/
│   ├── hardware_validation/             # Hardware test programs
│   └── ...
├── docs/                                # Additional documentation
├── examples/                            # Example usage
├── EMG/                                 # EMG signal acquisition (ADS1256 ADC)
│   ├── src/ADS.cpp                      # ADC driver
│   ├── lib/ADS1256/                     # ADC library
│   └── ads1256_plotter.py               # EMG visualization
├── RMD_motor_legacy/                    # Legacy RMD motor code (archived)
│   ├── src/                             # Old RMD control files
│   └── README.md                        # Legacy documentation
├── HARDWARE_TEST_GUIDE.md               # Complete testing procedures
├── QUICK_START_UBUNTU.md                # Fast setup guide for Ubuntu
├── DEPLOYMENT_CHECKLIST.md              # Pre-deployment verification
├── BUILD_REPORT.md                      # Build configuration details
├── test_calibration.py                  # Software calibration tests
├── test_cal_m1.py                       # Motor 1 calibration test
├── test_cal_full.py                     # Full calibration sequence
├── read_serial.py                       # Simple serial reader
├── send_set_zero.py                     # Set zero position (0x19)
└── platformio.ini                       # PlatformIO configuration

```

## Documentation

- **[HARDWARE_TEST_GUIDE.md](HARDWARE_TEST_GUIDE.md)** - Comprehensive hardware validation procedures
- **[QUICK_START_UBUNTU.md](QUICK_START_UBUNTU.md)** - Fast Ubuntu setup guide
- **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)** - Pre-deployment verification checklist
- **[BUILD_REPORT.md](BUILD_REPORT.md)** - Build system configuration and troubleshooting
- **[docs/](docs/)** - Additional technical documentation

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

For permanent zero-point setting, use `send_set_zero.py` (Command 0x19, writes to motor ROM).

## Testing

```bash
# Test software calibration (Motor 1)
python3 test_cal_m1.py

# Test full calibration sequence (CAL1, CAL2, CLEAR_CAL)
python3 test_cal_full.py

# Read serial output
python3 read_serial.py

# Set permanent zero position (writes to motor ROM, use carefully!)
python3 send_set_zero.py
```

## Troubleshooting

### Serial Port Issues
If you encounter "port already in use" errors:
1. Close all other programs using the port
2. The GUI includes automatic port cleanup on startup
3. Check with: `lsof /dev/ttyUSB0`

### Connection Lost
The GUI includes auto-reconnect (max 5 attempts):
- Connection status shown with color indicators (🟢🟡🔴)
- Automatically reconnects if cable is moved or disconnected

### Build Errors
See [BUILD_REPORT.md](BUILD_REPORT.md) for common build issues and solutions.

## Development History

- **November 2025**: Transitioned from RMD to MGv2 (LK-TECH) motor drivers
  - Added software calibration feature
  - Implemented robust serial port error handling
  - Enhanced GUI with auto-reconnect
- **Previous**: RMD motor development (see `RMD_motor_legacy/`)

## Related Modules

- **[EMG/](EMG/)** - EMG signal acquisition using ADS1256 ADC (24-bit, 8-channel)
- **[RMD_motor_legacy/](RMD_motor_legacy/)** - Archived RMD motor control code

## License

This project is part of the ExoPulse exoskeleton research platform.

## References

- MGv2 Motor Datasheet: `20230220145958f_datasheet_protocol.pdf`
- Hardware Test Results: [HARDWARE_TEST_GUIDE.md](HARDWARE_TEST_GUIDE.md)
- Project Analysis: [PROJECT_ANALYSIS_REPORT.md](PROJECT_ANALYSIS_REPORT.md)
