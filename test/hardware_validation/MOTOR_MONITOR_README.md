# Motor Status Monitor - Python Real-time Monitor

A Python-based real-time monitoring tool for LK-TECH motor status over CAN bus via ESP32.

## Features

- **Real-time Display**: Auto-refreshing terminal interface with color-coded status
- **Statistics**: Min/Max/Average values for last 20 readings
- **Color Coding**:
  - 🟢 Green = Normal (temp < 50°C)
  - 🟡 Yellow = Warning (temp 50-70°C, voltage < 10V)
  - 🔴 Red = Critical (temp > 70°C, errors present)
- **Error Detection**: Automatically detects LOW_VOLTAGE and OVER_TEMP flags
- **Historical Data**: Tracks last 20 readings for trend analysis

## Requirements

```bash
pip install pyserial
```

Or if already installed:
```bash
python3 -c "import serial; print(serial.__version__)"
```

## Usage

### Basic Usage (default /dev/ttyUSB0 @ 115200 baud)
```bash
python3 monitor_motor.py
```

### Custom Serial Port
```bash
python3 monitor_motor.py /dev/ttyUSB1
```

### Custom Port and Baudrate
```bash
python3 monitor_motor.py /dev/ttyUSB0 115200
```

### Make it Executable
```bash
chmod +x monitor_motor.py
./monitor_motor.py
```

## Display Format

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

## Motor Parameters Explained

| Parameter | Range | Description |
|-----------|-------|-------------|
| Temperature | °C | Motor internal temperature (safe < 70°C) |
| Voltage | V | Motor bus voltage (typical 24-48V when powered) |
| Torque Current | -33A ~ +33A | Motor torque current (iq) |
| Speed | dps | Motor speed in degrees per second |
| Encoder | 0 ~ 16383 | 14-bit absolute encoder position |
| Angle | degrees | Multi-turn cumulative angle (0.01° resolution) |
| Error State | 0x0 | Error flags (0x0 = no errors) |

## Error Flags

- `0x01` - **LOW_VOLTAGE**: Motor voltage below safe threshold
- `0x08` - **OVER_TEMP**: Motor temperature too high

## Troubleshooting

### "Device or resource busy"
The serial port is already in use. Close any other programs using the port:
```bash
# Check what's using the port
lsof /dev/ttyUSB0

# Kill screen sessions
pkill screen

# Or use fuser to identify and kill
fuser -k /dev/ttyUSB0
```

### "Permission denied"
Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### No data displayed
1. Verify ESP32 is running the motor read test program
2. Check serial port connection: `ls -l /dev/ttyUSB*`
3. Verify baudrate matches (default 115200)
4. Check motor is powered and CAN bus is connected

## Exit

Press `Ctrl+C` to gracefully stop monitoring and close the serial port.

## Advantages over `screen` or `cat`

1. **Parsed Data**: Extracts and formats motor parameters
2. **Color Coding**: Visual indicators for status (green/yellow/red)
3. **Statistics**: Automatically calculates min/max/avg
4. **Clean Display**: Auto-clearing screen with formatted output
5. **Real-time**: Updates as data arrives
6. **Error Highlighting**: Automatically detects and highlights errors
