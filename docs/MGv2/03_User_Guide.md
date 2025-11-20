# ExoPulse Firmware - User Guide

**Version:** 1.0.0
**Date:** November 2025
**Hardware:** ESP32 + MGv2 (LK-TECH) Motor Drivers

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Requirements](#system-requirements)
3. [Quick Start](#quick-start)
4. [Installation Guide](#installation-guide)
5. [Building and Uploading Firmware](#building-and-uploading-firmware)
6. [Using the GUI Monitor](#using-the-gui-monitor)
7. [Serial Console Commands](#serial-console-commands)
8. [Software Calibration](#software-calibration)
9. [Common Workflows](#common-workflows)
10. [Troubleshooting](#troubleshooting)
11. [FAQ](#faq)
12. [Safety Guidelines](#safety-guidelines)

---

## Introduction

The ExoPulse Firmware provides real-time motor control and monitoring for dual-motor exoskeleton systems using MGv2 (LK-TECH) motor drivers. This guide will help you set up, configure, and operate the system.

### Key Features

- **Dual Motor Control**: Simultaneous control of two motors via CAN bus (IDs: 0x141, 0x142)
- **Real-time Monitoring**: Live visualization of temperature, current, voltage, speed, acceleration, and multi-turn angle
- **Software Calibration**: Zero-point setting without ROM write (unlimited calibrations)
- **FreeRTOS Multi-tasking**: Efficient dual-core ESP32 utilization
- **Live GUI Monitor**: matplotlib-based visualization with auto-reconnect
- **Safety Features**: Watchdog timer, emergency stop, connection monitoring

---

## System Requirements

### Hardware
- **Microcontroller**: ESP32 DevKit v1 (dual-core, 240 MHz)
- **Motor Drivers**: MGv2 (LK-TECH) with CAN bus interface
- **CAN Transceiver**: MCP2515 SPI-to-CAN module + TJA1050
- **Power Supply**: Adequate for motors and ESP32 (typically 12-24V for motors, 5V for ESP32)
- **Cables**: USB cable for ESP32, CAN bus wiring with 120Ω terminators

### Software
- **Operating System**: Ubuntu 20.04+ (or any Linux distribution)
- **Python**: 3.8 or higher
- **PlatformIO**: Latest version
- **Python Packages**: pyserial, matplotlib

### Wiring Connections
```
ESP32 → MCP2515 CAN Transceiver
--------------------------------
GPIO5  → CS (Chip Select)
GPIO23 → SI/MOSI
GPIO19 → SO/MISO
GPIO18 → SCK
GPIO21 → INT (Interrupt)
3.3V   → VCC
GND    → GND

MCP2515 → TJA1050 → CAN Bus
----------------------------
CANH → CAN_H (to motor drivers)
CANL → CAN_L (to motor drivers)
120Ω resistor between CANH and CANL at both ends
```

---

## Quick Start

### For First-Time Users (Ubuntu)

**Total Time: ~10 minutes**

#### Step 1: One-Time Setup (5 minutes)

```bash
# 1. Install PlatformIO
sudo apt update
sudo apt install -y python3 python3-pip
pip3 install --user platformio
export PATH="$HOME/.local/bin:$PATH"

# 2. Configure serial permissions
sudo usermod -a -G dialout $USER
# ⚠️ LOG OUT and LOG IN after this command

# 3. Navigate to project
cd /path/to/ExoPulse_firmware
```

#### Step 2: Find Your ESP32 Port

```bash
# Unplug ESP32, then run:
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Plug in ESP32, then run again:
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# New device that appears is your port (usually /dev/ttyUSB0)
```

#### Step 3: Upload Firmware (3 minutes)

```bash
# Build firmware
pio run --environment esp32doit-devkit-v1

# Upload to ESP32
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected Output:**
```
[ESP32] MGv2 Motor Controller Started
[CAN] MCP2515 initialized successfully
[CAN] CAN bus speed: 1000 KBPS
[SYSTEM] FreeRTOS tasks started
[MOTOR] Polling Motor 1 (0x141) and Motor 2 (0x142)
```

#### Step 4: Run GUI Monitor (2 minutes)

```bash
# Install dependencies
pip3 install pyserial matplotlib

# Run enhanced dual-motor monitor
python3 test/hardware_validation/monitor_dual_motor_enhanced.py
```

**Expected Result:** GUI window opens with real-time plots for both motors.

---

## Installation Guide

### Detailed Ubuntu Setup

#### 1. Install System Dependencies

```bash
# Update package lists
sudo apt update

# Install Python 3 and pip
sudo apt install -y python3 python3-pip python3-venv

# Install build tools
sudo apt install -y build-essential git
```

#### 2. Install PlatformIO

```bash
# Install via pip (recommended)
pip3 install --user platformio

# Add to PATH (add this to ~/.bashrc for persistence)
export PATH="$HOME/.local/bin:$PATH"

# Verify installation
pio --version
```

#### 3. Configure Serial Port Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify group membership (after logging out and back in)
groups | grep dialout

# Alternative: Temporary permissions (not recommended for production)
sudo chmod 666 /dev/ttyUSB0
```

#### 4. Clone or Download Project

```bash
# If using git
git clone https://github.com/your-repo/ExoPulse_firmware.git
cd ExoPulse_firmware

# If using downloaded archive
unzip ExoPulse_firmware.zip
cd ExoPulse_firmware
```

#### 5. Install Python Dependencies

```bash
# Create virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate

# Install required packages
pip install pyserial matplotlib numpy
```

---

## Building and Uploading Firmware

### Build Process

#### Option 1: Using PlatformIO CLI

```bash
# Navigate to project directory
cd /path/to/ExoPulse_firmware

# Clean previous builds (optional)
pio run --target clean

# Build for ESP32 DevKit v1
pio run --environment esp32doit-devkit-v1

# Build output location:
# .pio/build/esp32doit-devkit-v1/firmware.bin
```

#### Option 2: Build and Upload in One Step

```bash
# Build and upload
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

### Upload Process

#### Standard Upload

```bash
# Upload firmware
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

#### Troubleshooting Upload Issues

**If upload fails:**

1. **Hold BOOT button**: Press and hold the BOOT button on ESP32 during upload
2. **Check port**: Verify correct port with `ls /dev/ttyUSB*`
3. **Check permissions**: Ensure you're in `dialout` group
4. **Try different USB port**: Some USB ports have better connectivity

```bash
# Manual upload with esptool (if PlatformIO fails)
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 \
  write_flash -z 0x1000 .pio/build/esp32doit-devkit-v1/firmware.bin
```

### Monitoring Serial Output

```bash
# Using PlatformIO
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Press Ctrl+C to exit

# Using screen (alternative)
screen /dev/ttyUSB0 115200

# Press Ctrl+A then K to exit
```

---

## Using the GUI Monitor

### Starting the Monitor

```bash
# Navigate to project directory
cd /path/to/ExoPulse_firmware

# Run the enhanced dual-motor monitor
python3 test/hardware_validation/monitor_dual_motor_enhanced.py

# Or specify port explicitly
python3 test/hardware_validation/monitor_dual_motor_enhanced.py /dev/ttyUSB0
```

### GUI Features

#### Connection Status Indicator
- **🟢 Green**: Connected and receiving data
- **🟡 Yellow**: Attempting to reconnect
- **🔴 Red**: Disconnected

#### Real-time Plots (5 parameters per motor)
1. **Temperature (°C)**: Motor driver temperature
2. **Current (A)**: Phase current consumption
3. **Voltage (V)**: Bus voltage
4. **Speed (deg/s)**: Rotational velocity
5. **Multi-turn Angle (deg)**: Absolute position with turn counting

#### Calibration Buttons
- **Cal Motor 1**: Set current Motor 1 position as zero (software offset)
- **Cal Motor 2**: Set current Motor 2 position as zero
- **Cal Both**: Calibrate both motors simultaneously
- **Clear Cal**: Remove all calibration offsets

### Using Calibration

**Scenario**: You want to set the current arm position as the zero reference.

**Steps**:
1. Move the arm to desired zero position
2. Click **"Cal Motor 1"** (or **"Cal Motor 2"** for second motor)
3. Observe angle reading reset to ~0°
4. Future angle readings are relative to this position

**To remove calibration**:
- Click **"Clear Cal"** to return to absolute encoder readings
- Or reboot ESP32 (calibration is not persistent)

### Auto-Reconnect Feature

The GUI automatically handles connection issues:
- Detects cable disconnection
- Attempts reconnection (up to 5 tries)
- Displays connection status
- Resumes data collection when reconnected

**Note**: If you need to unplug and replug the USB cable, the GUI will automatically attempt to reconnect.

---

## Serial Console Commands

### Command List

Send commands via serial terminal (`pio device monitor`) or GUI.

#### Motor Status and Information

```
HELP                    # Display all available commands
STATUS                  # Show system status and motor states
STATUS <id>            # Query specific motor (e.g., STATUS 1)
```

#### Motor Control

```
STOP                    # Emergency stop all motors
IQ <id> <value>        # Set torque current in A (e.g., IQ 1 10.0)
SPEED <id> <value>     # Set speed in deg/s (e.g., SPEED 1 50.0)
POS <id> <value>       # Set position in degrees (e.g., POS 1 180.0)
```

#### Calibration Commands

```
CAL1 / CAL_M1          # Calibrate Motor 1 zero position (software)
CAL2 / CAL_M2          # Calibrate Motor 2 zero position (software)
CLEAR_CAL              # Clear all calibration offsets
```

#### Debugging

```
DEBUG <id>             # Dump raw CAN frames for motor (e.g., DEBUG 1)
DETAILED               # Enable detailed debug output
```

### Command Examples

#### Example 1: Check System Status

```
# Send command
STATUS

# Expected response
[SYSTEM] CAN Status: OK
[SYSTEM] Motor 1 (0x141): Temperature=35.2°C, Current=1.2A, Voltage=24.1V
[SYSTEM] Motor 2 (0x142): Temperature=36.5°C, Current=0.8A, Voltage=24.0V
```

#### Example 2: Apply Torque

```
# Apply 5A torque current to Motor 1
IQ 1 5.0

# Response
[MOTOR 1] Torque set to 5.0 A
[MOTOR 1] Current torque: 5.02 A
```

#### Example 3: Position Control

```
# Move Motor 1 to 90 degrees
POS 1 90.0

# Response
[MOTOR 1] Target position: 90.0 deg
[MOTOR 1] Current position: 0.0 deg
[MOTOR 1] Moving to target...
[MOTOR 1] Position reached: 89.8 deg
```

#### Example 4: Emergency Stop

```
# Stop all motors immediately
STOP

# Response
[SYSTEM] Emergency stop triggered
[MOTOR 1] Current: 0.0 A
[MOTOR 2] Current: 0.0 A
```

---

## Software Calibration

### Overview

Software calibration allows you to set a zero-point reference without writing to motor ROM.

**Advantages**:
- ✅ **Unlimited calibrations**: No ROM wear from repeated writes
- ✅ **Instant**: No motor reboot required
- ✅ **Reversible**: Easily cleared with CLEAR_CAL
- ✅ **Safe for testing**: Perfect for development and experimentation

**Limitations**:
- ⚠️ **Non-persistent**: Calibration resets on ESP32 reboot
- ⚠️ **Software-based**: Offset stored in ESP32 RAM only

### Calibration Workflow

#### Via Serial Console

```bash
# 1. Connect to serial monitor
pio device monitor --baud 115200 --port /dev/ttyUSB0

# 2. Move motor to desired zero position

# 3. Send calibration command
CAL1        # For Motor 1
# or
CAL2        # For Motor 2

# 4. Verify calibration
STATUS 1

# Expected: Angle reading shows ~0.0 degrees

# 5. Clear calibration (optional)
CLEAR_CAL
```

#### Via GUI Monitor

1. Run `python3 test/hardware_validation/monitor_dual_motor_enhanced.py`
2. Position motor at desired zero point
3. Click **"Cal Motor 1"** or **"Cal Motor 2"**
4. Observe angle plot reset to 0°
5. Click **"Clear Cal"** to remove offset

### Permanent Zero-Point Setting

For permanent zero-point (writes to motor ROM):

```bash
# WARNING: Limited ROM write cycles (~10,000)
# Use this only when final zero-point is determined

python3 send_set_zero.py
```

**This sends Command 0x19 to write encoder zero position to motor ROM.**

⚠️ **Use sparingly**: ROM has limited write cycles. Software calibration is preferred for testing.

---

## Common Workflows

### Workflow 1: First-Time System Verification

**Objective**: Verify hardware is correctly connected and functional.

```bash
# Step 1: Upload firmware
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Step 2: Monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Step 3: Verify CAN initialization
# Look for: [CAN] MCP2515 initialized successfully

# Step 4: Test commands
HELP
STATUS
```

**Expected**: Console responds to commands, motor data is displayed.

### Workflow 2: Real-Time Monitoring During Operation

**Objective**: Monitor motor parameters during movement.

```bash
# Step 1: Start GUI monitor
python3 test/hardware_validation/monitor_dual_motor_enhanced.py

# Step 2: Observe baseline readings (motors at rest)
# Temperature: ~25-40°C
# Current: ~0 A
# Voltage: ~24V (or your supply voltage)
# Speed: 0 deg/s
# Angle: Current encoder reading

# Step 3: Apply torque or move motors
# Via serial console or programmatically

# Step 4: Observe real-time changes in plots
# Temperature increases during operation
# Current shows load
# Angle tracks position
```

### Workflow 3: Calibration and Testing

**Objective**: Set zero-point and validate angle tracking.

```bash
# Step 1: Start GUI monitor
python3 test/hardware_validation/monitor_dual_motor_enhanced.py

# Step 2: Move motor to desired zero position

# Step 3: Click "Cal Motor 1"

# Step 4: Manually rotate motor
# Observe angle plot showing relative position from zero

# Step 5: Return motor to calibrated position
# Angle should read ~0.0 degrees

# Step 6: Test angle accuracy
# Rotate motor 90 degrees clockwise
# Angle should read ~90.0 degrees

# Step 7: Clear calibration (if testing)
# Click "Clear Cal"
# Angle returns to absolute encoder reading
```

### Workflow 4: Development and Debugging

**Objective**: Test new control algorithms or debug issues.

```bash
# Step 1: Enable detailed debug output
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Send command:
DETAILED

# Step 2: Dump CAN frames for specific motor
DEBUG 1

# Expected output:
# [DEBUG M1] RX ID=0x141 Len=8 Data: A1 02 00 00 B4 00 00 00

# Step 3: Analyze frame data
# Compare with MGv2 protocol documentation

# Step 4: Test specific CAN commands
# Modify src/main.cpp to send custom commands
# Rebuild and upload

# Step 5: Monitor results in GUI
python3 test/hardware_validation/monitor_dual_motor_enhanced.py
```

### Workflow 5: Production Deployment

**Objective**: Deploy firmware to production hardware.

See [DEPLOYMENT_CHECKLIST.md](../MGv2/DEPLOYMENT_CHECKLIST.md) for complete procedure.

**Quick summary**:
1. Run all hardware validation tests
2. Build clean production firmware
3. Backup current firmware (if updating existing system)
4. Upload firmware to target ESP32
5. Verify all functionality
6. Monitor for 15+ minutes to ensure stability
7. Test emergency stop and safety features

---

## Troubleshooting

### Serial Port Issues

#### Problem: "Permission denied" on /dev/ttyUSB0

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in (or reboot)

# Verify group membership
groups | grep dialout

# If immediate access needed (temporary fix)
sudo chmod 666 /dev/ttyUSB0
```

#### Problem: "Port already in use"

**Solution**:
```bash
# Find process using the port
lsof /dev/ttyUSB0

# Kill the process
kill -9 <PID>

# Or close all programs accessing the port:
# - Arduino IDE Serial Monitor
# - PlatformIO Device Monitor
# - screen sessions
# - Previous Python scripts

# Verify port is free
lsof /dev/ttyUSB0
# Should return nothing if free
```

#### Problem: ESP32 not detected (no /dev/ttyUSB*)

**Solution**:
```bash
# Check USB connection
lsusb | grep -i cp210x  # or CH340, FTDI depending on chip

# Install USB drivers (if missing)
sudo apt install -y brltty
sudo systemctl disable brltty

# Check kernel messages
dmesg | tail -20

# Try different USB port
# Try different USB cable
```

### Connection Issues

#### Problem: GUI shows "Connection Lost" (🔴)

**Solutions**:
1. **Cable disconnected**: Replug USB cable, GUI will auto-reconnect
2. **ESP32 rebooted**: Wait for ESP32 to restart, GUI will reconnect
3. **Wrong port**: Close GUI, check port with `ls /dev/ttyUSB*`, restart GUI with correct port

#### Problem: GUI auto-reconnect fails

**Solution**:
```bash
# Check if port exists
ls -l /dev/ttyUSB0

# Check ESP32 is responding
pio device monitor --baud 115200 --port /dev/ttyUSB0

# If monitor works but GUI doesn't:
# Close GUI, kill any port-locking processes, restart GUI

# Manual port cleanup
fuser -k /dev/ttyUSB0
```

### CAN Bus Issues

#### Problem: [CAN] MCP2515 initialization failed

**Solutions**:
1. **Check wiring**: Verify all 7 connections (VCC, GND, CS, MOSI, MISO, SCK, INT)
2. **Check power**: Measure 3.3V on MCP2515 VCC pin
3. **Check SPI pins**: Confirm GPIO numbers match platformio.ini configuration
4. **Try different CS pin**: Modify code if GPIO5 has conflicts
5. **Check crystal frequency**: MCP2515 should have 8 MHz crystal (or update code for 16 MHz)

```bash
# Enable detailed CAN debug
# Modify src/main.cpp to add verbose CAN logs
# Rebuild and upload
```

#### Problem: No motor data received

**Solutions**:
1. **Check CAN termination**: 120Ω resistors on both ends of CAN bus
2. **Check CAN wiring**: CANH and CANL not swapped
3. **Check motor power**: Motors must be powered on
4. **Check motor IDs**: Ensure motors are configured as 0x141 and 0x142
5. **Check CAN speed**: Both ESP32 and motors must use same speed (1000 KBPS)

```bash
# Test with CAN loopback
# This isolates MCP2515 from motors

cp examples/test_loopback.h src/main.cpp
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Expected: All loopback frames succeed
```

### Build Issues

#### Problem: Compilation errors

**Solutions**:
```bash
# Clean build directory
pio run --target clean

# Update PlatformIO
pip3 install --upgrade platformio

# Update libraries
pio lib update

# Check platformio.ini for correct configuration
# Verify build_flags and lib_deps are correct
```

#### Problem: Multiple definition errors

**Solution**:
```bash
# Ensure only one main.cpp is being compiled
# Check platformio.ini for:
build_src_filter = +<main.cpp> +<*.h>

# Remove any other .cpp files from src/ directory
# Or move them to examples/ or test/ directories
```

### Motor Control Issues

#### Problem: Motor doesn't respond to commands

**Solutions**:
1. **Check motor power**: Ensure motor driver is powered
2. **Check CAN connection**: Verify data is being received (use DEBUG command)
3. **Check motor ID**: Confirm motor ID matches command (STATUS 1 for motor ID 1)
4. **Check enable signal**: Some motor drivers require enable pin
5. **Check command validity**: Ensure command values are within motor limits

```bash
# Test with minimal torque
IQ 1 1.0

# Check motor response
STATUS 1

# Enable debug to see CAN frames
DEBUG 1
```

#### Problem: Incorrect angle readings

**Solutions**:
1. **Check encoder direction**: May need to invert in code
2. **Verify multi-turn tracking**: Use Command 0x92 (Read Multi-turn Angle)
3. **Check calibration**: Use software calibration if offset is needed
4. **Verify CAN data parsing**: Use DEBUG to see raw data

---

## FAQ

### General Questions

**Q: What motor drivers are supported?**
A: This firmware is designed for MGv2 (LK-TECH) motor drivers with CAN bus interface.

**Q: Can I use different motor IDs?**
A: Yes, modify the `MOTOR1_ID` and `MOTOR2_ID` definitions in `src/main.cpp`.

**Q: What CAN speed should I use?**
A: Default is 1000 KBPS. Both ESP32 and motor drivers must match.

**Q: Can I control more than 2 motors?**
A: Yes, but requires code modifications. Add additional motor tasks in FreeRTOS configuration.

### Calibration Questions

**Q: What's the difference between software calibration and 0x19 command?**
A:
- **Software calibration (CAL1/CAL2)**: Offset stored in ESP32 RAM, unlimited uses, non-persistent
- **0x19 command (send_set_zero.py)**: Writes to motor ROM, limited write cycles, persistent

**Q: Does software calibration survive reboot?**
A: No, calibration offsets reset when ESP32 reboots. This is by design for safety.

**Q: How accurate is software calibration?**
A: Accuracy is limited by encoder resolution (typically 0.1° or better for MGv2 motors).

**Q: Can I save calibration offsets to ESP32 flash?**
A: Not currently implemented, but can be added using ESP32 NVS (Non-Volatile Storage) library.

### Monitoring Questions

**Q: Why is the GUI showing old data?**
A: Check connection status. If red (🔴), cable may be disconnected or ESP32 rebooted.

**Q: Can I log data to a file?**
A: Not built-in, but you can modify `monitor_dual_motor_enhanced.py` to add CSV logging.

**Q: What's the update rate?**
A: GUI updates at 10 Hz. Actual CAN polling rate is configurable in firmware (default: 50 Hz per motor).

**Q: Can I monitor remotely over WiFi?**
A: Not currently implemented. ESP32 only sends data via serial USB.

### Safety Questions

**Q: What happens if CAN connection is lost?**
A: Watchdog timer triggers and stops motors after 2 seconds of no communication.

**Q: How do I emergency stop?**
A: Send `STOP` command via serial, or power off motors/ESP32.

**Q: Are there torque/speed limits?**
A: Software limits can be configured in `src/main.cpp`. Hardware limits are defined by motor driver specifications.

**Q: What safety features are included?**
A:
- Watchdog timer (2s timeout)
- Emergency stop command
- Connection status monitoring
- Temperature monitoring
- Current limiting (configurable)

### Technical Questions

**Q: What ESP32 cores are used?**
A: Core 0 runs FreeRTOS system tasks and WiFi. Core 1 runs motor control tasks.

**Q: Can I use ESP32-S3 or ESP32-C3?**
A: Yes, but requires testing. Configuration exists in `platformio.ini` for ESP32-C3.

**Q: What's the maximum CAN bus length?**
A: Depends on speed. At 1 Mbps: ~40m. At 500 kbps: ~100m. Use proper termination.

**Q: Can I use a different CAN transceiver?**
A: Yes, any SPI-to-CAN controller compatible with MCP2515 library should work.

---

## Safety Guidelines

### Before Operating

- ✅ Verify all wiring connections are secure
- ✅ Ensure CAN termination resistors are installed (120Ω at both ends)
- ✅ Check motor mounting is secure
- ✅ Clear workspace of obstacles
- ✅ Have emergency stop accessible
- ✅ Verify power supply is adequate and stable
- ✅ Test emergency stop functionality before full operation

### During Operation

- ✅ Monitor temperature readings (typical: 25-60°C, max: 80°C)
- ✅ Monitor current consumption
- ✅ Stay clear of moving parts
- ✅ Have emergency stop button ready
- ✅ Watch for unusual sounds or vibrations
- ✅ Stop immediately if error messages appear

### Emergency Procedures

**If motor misbehaves**:
1. Send `STOP` command immediately
2. If unresponsive, power off motor driver
3. If still moving, power off ESP32
4. Do not resume until issue is identified

**If overheating detected** (temperature > 80°C):
1. Send `STOP` command
2. Allow motors to cool for 15+ minutes
3. Check for mechanical binding or excessive load
4. Reduce torque/speed settings
5. Improve cooling (add heatsinks or fans)

**If connection lost**:
1. Motors will stop automatically (watchdog timeout: 2s)
2. Check USB cable connection
3. Check ESP32 power
4. Restart system if needed

### Maintenance

- 🔧 Inspect wiring connections weekly
- 🔧 Check CAN bus termination resistors monthly
- 🔧 Verify encoder calibration monthly
- 🔧 Update firmware when new versions available
- 🔧 Keep logs of any unusual behavior
- 🔧 Test emergency stop monthly

### DO NOT

- ❌ Operate motors without proper mounting
- ❌ Exceed motor torque/speed specifications
- ❌ Operate with damaged wiring
- ❌ Disable watchdog or safety features
- ❌ Leave system unattended during operation
- ❌ Ignore warning messages or high temperatures

---

## Additional Resources

### Documentation
- **[Technical Documentation](01_Technical_Documentation.md)**: System architecture, CAN protocol, API reference
- **[Hardware Guide](02_Hardware_Guide.md)**: Hardware setup, testing procedures, troubleshooting
- **[README.md](../../MGv2/README.md)**: Project overview and quick reference
- **[HARDWARE_TEST_GUIDE.md](../../MGv2/HARDWARE_TEST_GUIDE.md)**: Comprehensive testing procedures

### Example Code
- **`examples/test_led.h`**: LED test for ESP32 verification
- **`examples/test_loopback.h`**: MCP2515 CAN loopback test
- **`examples/test_can_sender.cpp`**: CAN bus sender test
- **`examples/test_can_receiver.h`**: CAN bus receiver test

### Test Scripts
- **`test_calibration.py`**: Software calibration testing
- **`test_cal_m1.py`**: Motor 1 calibration test
- **`test_cal_full.py`**: Full calibration sequence test
- **`read_serial.py`**: Simple serial output reader
- **`send_set_zero.py`**: Permanent zero-point setting (0x19 command)

### Support
- **Issues**: Report bugs at project repository
- **Community**: Contact development team for assistance

---

**Document Version**: 1.0.0
**Last Updated**: November 2025
**Firmware Version**: MGv2 v1.0.0

*ExoPulse Firmware - Empowering Exoskeleton Research*
