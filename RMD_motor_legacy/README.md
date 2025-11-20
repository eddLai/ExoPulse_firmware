# RMD Motor Legacy Files

This folder contains legacy code and test files for **RMD motor drivers**.

## Background

The original ExoPulse firmware was developed for RMD motor drivers. The project has since transitioned to **MGv2 (LK-TECH) motor drivers**, which offer improved features and better integration with the CAN bus system.

## Contents

### Main RMD Control Files
- `ESP_RMD_Driver_loopback_test.ino` - RMD driver loopback test
- `src/RMD_driver_control.ino` - Main RMD driver control code
- `src/RMD_motor_control_debug.ino` - RMD motor debugging code
- `src/RMD_motor_control_CAN_KD240.py` - Python control via KD240

### UDP Control Implementations
- `src/ESP32_UDP_control/` - ESP32 UDP control code for RMD motors
- `src/KD240_UDP_control/` - KD240 UDP control code for exoskeleton

## Current Project Status

**Active Development**: The main project now uses MGv2 (LK-TECH) motor drivers.

**Motor Comparison**:
- **RMD Motors**: Previous generation, basic CAN protocol
- **MGv2 Motors**: Current generation, advanced features including:
  - Multi-turn angle tracking
  - Acceleration data (Command 0x33)
  - Software calibration support
  - Better temperature and current reporting

## Migration Notes

If you need to work with RMD motors:
1. The code in this folder is archived but functional
2. Refer to the commit history for the last known working state
3. The CAN protocol differs from MGv2 - do not mix implementations

## References

For current MGv2 motor implementation, see:
- `/src/main.cpp` - Current firmware for MGv2 motors
- `/docs/` - Hardware validation and testing guides
- `HARDWARE_TEST_GUIDE.md` - Complete testing procedures

---

**Archived**: November 2025
**Reason**: Transition to MGv2 (LK-TECH) motor drivers
