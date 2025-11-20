# Motor Status Monitoring Tools

**Real-time LK-TECH Motor Monitoring over CAN Bus via ESP32**

---

## Overview

This guide covers two Python-based real-time monitoring tools for LK-TECH motor status:

1. **Terminal Monitor** (`monitor_motor.py`) - Lightweight, SSH-friendly, text-based interface
2. **GUI Monitor** (`monitor_motor_gui.py`) - Graphical interface with live plots and historical data

Both tools read motor status data from the ESP32 via serial port and display it in real-time.

---

## Table of Contents

- [Installation](#installation)
- [Terminal Monitor](#terminal-monitor)
- [GUI Monitor](#gui-monitor)
- [Comparison](#comparison)
- [Motor Parameters](#motor-parameters)
- [Troubleshooting](#troubleshooting)

---

## Installation

### Requirements

**For Terminal Monitor:**
```bash
pip install pyserial
```

**For GUI Monitor:**
```bash
pip install pyserial matplotlib
```

### Verify Installation

```bash
# Check pyserial
python3 -c "import serial; print(serial.__version__)"

# Check matplotlib (for GUI only)
python3 -c "import matplotlib; print('OK')"
```

---

## Terminal Monitor

**Lightweight real-time terminal-based monitoring**

### Features

- **Real-time Display**: Auto-refreshing terminal interface with color-coded status
- **Statistics**: Min/Max/Average values for last 20 readings
- **Color Coding**:
  - 🟢 Green = Normal (temp < 50°C)
  - 🟡 Yellow = Warning (temp 50-70°C, voltage < 10V)
  - 🔴 Red = Critical (temp > 70°C, errors present)
- **Error Detection**: Automatically detects LOW_VOLTAGE and OVER_TEMP flags
- **Historical Data**: Tracks last 20 readings for trend analysis
- **SSH-Friendly**: Works over SSH without X forwarding
- **Low Resource Usage**: Minimal CPU and memory footprint

### Usage

#### Basic Usage (default /dev/ttyUSB0 @ 115200 baud)
```bash
python3 test/hardware_validation/monitor_motor.py
```

#### Custom Serial Port
```bash
python3 test/hardware_validation/monitor_motor.py /dev/ttyUSB1
```

#### Custom Port and Baudrate
```bash
python3 test/hardware_validation/monitor_motor.py /dev/ttyUSB0 115200
```

#### Make it Executable
```bash
chmod +x test/hardware_validation/monitor_motor.py
./test/hardware_validation/monitor_motor.py
```

### Display Format

```
============================================================
  LK-TECH Motor Status Monitor - 14:23:45
============================================================

  Temperature:         27 °C
  Voltage:            0.8 V
  Torque Current:   -0.03 A
  Speed:                0 dps
  Encoder:           4832 (29.5%)
  Angle:           265.43° (  0.74 turns)
  Error State:       0x0

------------------------------------------------------------
  Statistics (last 20 readings):

  Temp:  Min= 27°C  Max= 28°C  Avg= 27.5°C
  Current: Min=-0.05A  Max= 0.05A  Avg=-0.03A
  Speed:   Min=   0dps Max=   0dps Avg=    0.0dps

============================================================
  Press Ctrl+C to exit
============================================================
```

### When to Use Terminal Monitor

✅ **Best for:**
- Remote monitoring over SSH
- Low-resource environments
- Quick status checks
- Continuous monitoring without GUI overhead
- Headless servers or embedded systems

❌ **Not ideal for:**
- Detailed trend analysis
- Visual pattern recognition
- Long-term data visualization
- Multi-parameter comparison

---

## GUI Monitor

**Graphical interface with real-time plotting and historical trends**

### Features

- **6 Live Plots** - Real-time graphs for all motor parameters:
  - Temperature (°C)
  - Voltage (V)
  - Torque Current (A)
  - Speed (dps)
  - Encoder Position
  - Multi-turn Angle (°)

- **Color-coded Status Display**:
  - 🟢 Green = Normal operation
  - 🟡 Yellow = Warning (temp 50-70°C)
  - 🔴 Red = Critical (temp > 70°C or errors)

- **Auto-scaling Graphs** - Automatically adjusts Y-axis ranges
- **Historical Data** - Shows last 100 data points (configurable)
- **Dark Theme** - Easy on the eyes for extended monitoring
- **Zoom/Pan** - Use matplotlib toolbar to analyze specific time ranges
- **Save Plots** - Export graphs as images for documentation

### Usage

#### Basic Usage (default /dev/ttyUSB0 @ 115200 baud)
```bash
python3 test/hardware_validation/monitor_motor_gui.py
```

#### Custom Serial Port
```bash
python3 test/hardware_validation/monitor_motor_gui.py /dev/ttyUSB1
```

#### Custom Port and Baudrate
```bash
python3 test/hardware_validation/monitor_motor_gui.py /dev/ttyUSB0 115200
```

#### Custom Max Data Points (default 100)
```bash
python3 test/hardware_validation/monitor_motor_gui.py /dev/ttyUSB0 115200 200
```

#### Make it Executable
```bash
chmod +x test/hardware_validation/monitor_motor_gui.py
./test/hardware_validation/monitor_motor_gui.py
```

### GUI Layout

```
┌─────────────────────────────────────────────────────────┐
│  LK-TECH Motor Monitor - Real-time                     │
├──────────────────────┬──────────────────────────────────┤
│  Temperature (°C)    │  Voltage (V)                     │
│  [Live Graph]        │  [Live Graph]                    │
├──────────────────────┼──────────────────────────────────┤
│  Torque Current (A)  │  Speed (dps)                     │
│  [Live Graph]        │  [Live Graph]                    │
├──────────────────────┼──────────────────────────────────┤
│  Encoder Position    │  Multi-turn Angle (°)            │
│  [Live Graph]        │  [Live Graph]                    │
├──────────────────────┴──────────────────────────────────┤
│  Current Status                                         │
│  Temperature: 30°C | Voltage: 0.8V | Current: 0.03A     │
│  Speed: 0dps | Encoder: 35047 | Angle: 1925.22°         │
│  Error State: 0x0                                       │
└─────────────────────────────────────────────────────────┘
```

### Controls

- **Close Window** - Stop monitoring
- **Zoom** - Use matplotlib toolbar to zoom into specific time ranges
- **Pan** - Use matplotlib toolbar to pan through data
- **Save** - Use matplotlib toolbar to save plots as images

### When to Use GUI Monitor

✅ **Best for:**
- Visual trend analysis
- Long-term monitoring sessions
- Pattern recognition across multiple parameters
- Documentation (save plots as images)
- Local workstation with GUI
- Debugging motor behavior issues

❌ **Not ideal for:**
- Remote SSH sessions without X forwarding
- Low-resource systems
- Quick status checks
- Headless environments

### Performance Tips

1. **Reduce max_points** for slower systems:
   ```bash
   python3 monitor_motor_gui.py /dev/ttyUSB0 115200 50
   ```

2. **Close other applications** to free up resources

3. **Use full screen** for better visibility

---

## Comparison

### Terminal vs GUI - Feature Matrix

| Feature | Terminal Monitor | GUI Monitor |
|---------|-----------------|-------------|
| **Visual Plots** | ❌ | ✅ |
| **Historical Trends** | Limited (20 readings) | ✅ Full history (100+ points) |
| **Multi-parameter View** | Sequential | ✅ Simultaneous |
| **Zoom/Pan** | ❌ | ✅ |
| **Save Graphs** | ❌ | ✅ |
| **Color Coding** | ✅ Limited | ✅ Full |
| **SSH Compatible** | ✅ | ❌ (requires X forwarding) |
| **Resource Usage** | ✅ Very low | Moderate |
| **Startup Time** | ✅ Instant | ~2-3 seconds |
| **Statistics** | ✅ Min/Max/Avg | ❌ |
| **Works on Headless** | ✅ | ❌ |

### Use Case Recommendations

| Scenario | Recommended Tool |
|----------|-----------------|
| Remote monitoring over SSH | **Terminal Monitor** |
| Local debugging with visual analysis | **GUI Monitor** |
| Extended testing sessions | **GUI Monitor** |
| Quick status checks | **Terminal Monitor** |
| Low-resource systems | **Terminal Monitor** |
| Pattern recognition across parameters | **GUI Monitor** |
| Documentation (screenshots) | **GUI Monitor** |
| Continuous background monitoring | **Terminal Monitor** |
| Temperature trend analysis | **GUI Monitor** |
| Current load analysis | **GUI Monitor** |

---

## Motor Parameters

### Parameter Reference

| Parameter | Range | Description | Normal Range |
|-----------|-------|-------------|--------------|
| **Temperature** | °C | Motor internal temperature | < 50°C normal, < 70°C safe |
| **Voltage** | V | Motor bus voltage | 24-48V when powered |
| **Torque Current** | -33A ~ +33A | Motor torque current (iq) | Depends on load |
| **Speed** | dps | Motor speed in degrees per second | Depends on command |
| **Encoder** | 0 ~ 16383 | 14-bit absolute encoder position | Full range |
| **Angle** | degrees | Multi-turn cumulative angle | 0.01° resolution |
| **Error State** | 0x0 ~ 0xFF | Error flags bitmap | 0x0 = no errors |

### Error Flags

| Flag | Hex | Description | Action Required |
|------|-----|-------------|-----------------|
| **LOW_VOLTAGE** | `0x01` | Motor voltage below safe threshold | Check power supply |
| **OVER_TEMP** | `0x08` | Motor temperature too high | Stop motor, allow cooling |

### Color Coding Guide

**Temperature:**
- 🟢 Green: < 50°C (Normal operation)
- 🟡 Yellow: 50-70°C (Warning, monitor closely)
- 🔴 Red: > 70°C (Critical, stop motor)

**Voltage:**
- 🟢 Green: > 10V (Normal)
- 🟡 Yellow: < 10V (Low voltage warning)

**Errors:**
- 🟢 Green: 0x0 (No errors)
- 🔴 Red: Any error flag set

---

## Troubleshooting

### Common Issues

#### "Device or resource busy"

The serial port is already in use. Close any other programs using the port:

```bash
# Check what's using the port
lsof /dev/ttyUSB0

# Kill screen sessions
pkill screen

# Or use fuser to identify and kill
fuser -k /dev/ttyUSB0
```

#### "Permission denied"

Add your user to the dialout group:

```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### No data displayed

1. Verify ESP32 is running the motor read test program
2. Check serial port connection: `ls -l /dev/ttyUSB*`
3. Verify baudrate matches (default 115200)
4. Check motor is powered and CAN bus is connected
5. Try running with `sudo` (temporary test only)

#### GUI doesn't show up

1. Check if X11 is running: `echo $DISPLAY`
2. If using SSH, connect with X forwarding: `ssh -X user@host`
3. Or use VNC/remote desktop
4. Try `export DISPLAY=:0` if running locally

#### Plots are frozen

1. Check if ESP32 is sending data
2. Verify serial connection: `ls -l /dev/ttyUSB*`
3. Check motor is powered and CAN bus connected
4. Restart the monitor program

#### "ModuleNotFoundError: No module named 'matplotlib'"

Install matplotlib:
```bash
pip install matplotlib
# or
pip3 install matplotlib
# or
sudo apt install python3-matplotlib
```

#### "ModuleNotFoundError: No module named 'serial'"

Install pyserial:
```bash
pip install pyserial
# or
pip3 install pyserial
```

#### Data appears garbled or incorrect

1. Verify baudrate matches ESP32 (default 115200)
2. Check serial cable quality
3. Try different USB port
4. Restart ESP32 and monitor program

---

## Use Cases

### 1. Motor Testing
Monitor motor behavior during testing phases to ensure stable operation.

**Recommended:** GUI Monitor for visual feedback

### 2. Debug CAN Communication
Visualize communication quality by observing data continuity.

**Recommended:** GUI Monitor to see gaps in communication

### 3. Temperature Monitoring
Track motor temperature during extended operation to prevent overheating.

**Recommended:** GUI Monitor for temperature trends over time

### 4. Current Analysis
Observe torque current patterns during load changes.

**Recommended:** GUI Monitor for current pattern visualization

### 5. Position Tracking
Monitor encoder and angle data for positioning accuracy.

**Recommended:** Either, depending on access method

### 6. Error Detection
Quickly identify voltage drops or error conditions.

**Recommended:** Terminal Monitor for quick alerts

### 7. Remote Monitoring
Monitor motor status over SSH from remote location.

**Recommended:** Terminal Monitor (SSH-friendly)

### 8. Extended Testing
Long-term monitoring sessions with data logging.

**Recommended:** GUI Monitor for historical analysis

---

## Exit

**Terminal Monitor:** Press `Ctrl+C` to gracefully stop monitoring and close the serial port.

**GUI Monitor:** Close the GUI window or press `Ctrl+C` in the terminal to stop monitoring.

---

## Advantages over Basic Tools

### vs `screen` or `cat`

1. **Parsed Data**: Extracts and formats motor parameters automatically
2. **Color Coding**: Visual indicators for status (green/yellow/red)
3. **Statistics**: Automatically calculates min/max/avg
4. **Clean Display**: Auto-clearing screen with formatted output
5. **Real-time**: Updates as data arrives
6. **Error Highlighting**: Automatically detects and highlights errors

### vs `pio device monitor`

1. **Data Parsing**: Interprets motor status packets
2. **Visual Formatting**: Structured display with labels
3. **Statistics**: Tracks trends and patterns
4. **Error Detection**: Automatic error flag recognition
5. **Historical Analysis**: Maintains recent history (GUI version)

---

## Additional Resources

**Related Documentation:**
- `docs/HARDWARE_TESTING.md` - Hardware setup and testing procedures
- `test/hardware_validation/CAN_BUS_TEST_RESULTS_FINAL.md` - CAN bus validation
- `README.md` - Main firmware documentation

**Test Programs:**
- `test/hardware_validation/test_dual_motor_rtos.cpp` - Dual motor test
- `test/hardware_validation/test_motor_read_rtos.cpp` - Motor read test

---

**ExoPulse Firmware - Motor Monitoring Tools**
*November 2025*
