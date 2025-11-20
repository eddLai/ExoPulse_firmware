# EMG Signal Acquisition Module

This folder contains code and resources for EMG (Electromyography) signal acquisition using the ADS1256 ADC (Analog-to-Digital Converter).

## Hardware

**ADC Chip**: ADS1256
- 24-bit resolution
- 8 differential input channels
- High-precision analog signal acquisition
- SPI communication interface

## Contents

### Source Files
- `src/ADS.cpp` - Main ADS1256 driver implementation for ESP32
- `lib/ADS1256/` - ADS1256 library files
  - `ADS1256.cpp` - Library implementation
  - `ADS1256.h` - Library header

### Tools
- `ads1256_plotter.py` - Real-time plotting tool for EMG signal visualization

### Documentation
- `img/ADS1256_Arduino_ESP32.jpg` - Hardware connection diagram
- `img/ADS1256_Arduino_ESP32_Devmodule.jpg` - Development module wiring

## Purpose

The EMG signal acquisition system is used to capture muscle electrical activity for:
- Gait analysis
- Muscle fatigue detection
- Exoskeleton control signal input
- Rehabilitation monitoring

## Integration with ExoPulse

The EMG module provides:
1. **Real-time muscle activity data** for adaptive control
2. **User intent detection** for exoskeleton assistance
3. **Fatigue monitoring** to adjust assistance levels

## Usage

Refer to `src/ADS.cpp` for ESP32 integration examples.

For data visualization, run:
```bash
python3 ads1256_plotter.py
```

---

**Note**: This module is separate from the motor control system (MGv2) but can be integrated for advanced control algorithms.
