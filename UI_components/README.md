# UI Components

This directory contains all user interface components for the ExoPulse firmware, organized by complexity level.

## Low-Level Components (Basic Control)

### Motor Control
- **motor_control.py** - Basic motor control GUI
  - Motor swing control
  - Demo mode
  - WiFi/UART communication selection
  - Motor type selection (CAN/Lower Chip)
  - IMU 3D visualization (optional)
  - Calibration controls
  - Serial port: ttyUSB devices only

### Utilities
- **serial_reader.py** - Simple serial port data reader
  - Lightweight command-line tool
  - Real-time serial data display

## High-Level Components (Advanced Monitoring)

### Comprehensive Monitoring
- **motor_monitor.py** - Advanced motor monitoring GUI with configurable display
  - All features from motor_control.py
  - Real-time data visualization (5 parameters per motor)
  - Configurable sidebar with display options
  - Temperature, Current, Speed, Acceleration, Angle plots
  - Dual motor support with synchronized plots
  - Auto-reconnect on serial disconnect

### Data Visualization
- **dual_motor_plotter.py** - Dual motor real-time plotter
  - 5-row GridSpec layout (Temperature, Current, Speed, Acceleration, Angle)
  - Multi-turn angle tracking
  - Software calibration (CAL1, CAL2, CLEAR_CAL)
  - Auto-calibration on startup
  - Matplotlib-based visualization

- **can_plotter.py** - CAN bus data plotter
  - Real-time CAN bus data visualization
  - Motor status monitoring

- **emg_plotter.py** - EMG signal plotter
  - ADS1256 ADC data visualization
  - Real-time muscle activity monitoring

### WiFi Monitoring
- **wifi_monitor.py** - WiFi-based motor monitor
  - TCP/UDP socket communication
  - Remote motor monitoring over WiFi

- **wifi_dual_motor_plotter.py** - WiFi dual motor plotter
  - Combines WiFi communication with dual motor plotting
  - Network-based real-time visualization

## Usage

### Basic Motor Control (Quick Start)
```bash
python3 UI_components/motor_control.py
```

### Advanced Monitoring with Full Visualization
```bash
python3 UI_components/motor_monitor.py
```

### Standalone Plotters
```bash
# Dual motor via serial
python3 UI_components/dual_motor_plotter.py /dev/ttyUSB0

# Dual motor via WiFi
python3 UI_components/wifi_dual_motor_plotter.py

# EMG signals
python3 UI_components/emg_plotter.py

# CAN bus data
python3 UI_components/can_plotter.py
```

## Requirements

- Python 3.7+
- PySide6 (for motor_control.py and motor_monitor.py)
- matplotlib (for all plotters)
- pyserial (for serial communication)
- numpy (for data processing)

Install dependencies:
```bash
pip3 install PySide6 matplotlib pyserial numpy
```

## Component Selection Guide

| Use Case | Recommended Component | Level |
|----------|----------------------|-------|
| Basic motor control | motor_control.py | Low |
| Full monitoring & visualization | motor_monitor.py | High |
| Real-time dual motor plots | dual_motor_plotter.py | High |
| WiFi remote monitoring | wifi_monitor.py | High |
| EMG signal analysis | emg_plotter.py | High |
| CAN bus debugging | can_plotter.py | High |

## Architecture

**Low-Level Components**: Direct control interfaces with minimal dependencies
- Focus: Control and basic status
- UI Framework: PySide6
- Target Users: End users, operators

**High-Level Components**: Advanced monitoring with data visualization
- Focus: Analysis, debugging, development
- UI Framework: Matplotlib + PySide6
- Target Users: Developers, researchers, testers
