# Motor Status Monitor - GUI Version with Real-time Plotting

A graphical real-time monitoring tool for LK-TECH motor status with live plots.

## Features

- **6 Live Plots** - Real-time graphs for all motor parameters
  - Temperature (°C)
  - Voltage (V)
  - Torque Current (A)
  - Speed (dps)
  - Encoder Position
  - Multi-turn Angle (°)

- **Color-coded Status Display**
  - 🟢 Green = Normal operation
  - 🟡 Yellow = Warning (temp 50-70°C)
  - 🔴 Red = Critical (temp > 70°C or errors)

- **Auto-scaling Graphs** - Automatically adjusts Y-axis ranges
- **Historical Data** - Shows last 100 data points (configurable)
- **Dark Theme** - Easy on the eyes for extended monitoring

## Requirements

```bash
pip install pyserial matplotlib
```

Or check if already installed:
```bash
python3 -c "import serial, matplotlib; print('OK')"
```

## Usage

### Basic Usage (default /dev/ttyUSB0 @ 115200 baud)
```bash
python3 monitor_motor_gui.py
```

### Custom Serial Port
```bash
python3 monitor_motor_gui.py /dev/ttyUSB1
```

### Custom Port and Baudrate
```bash
python3 monitor_motor_gui.py /dev/ttyUSB0 115200
```

### Custom Max Data Points (default 100)
```bash
python3 monitor_motor_gui.py /dev/ttyUSB0 115200 200
```

### Make it Executable
```bash
chmod +x monitor_motor_gui.py
./monitor_motor_gui.py
```

## GUI Layout

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

## Controls

- **Close Window** - Stop monitoring
- **Zoom** - Use matplotlib toolbar to zoom into specific time ranges
- **Pan** - Use matplotlib toolbar to pan through data
- **Save** - Use matplotlib toolbar to save plots as images

## Advantages over Terminal Monitor

| Feature | Terminal | GUI |
|---------|----------|-----|
| Visual Plots | ❌ | ✅ |
| Historical Trends | Limited | ✅ Full history |
| Multi-parameter View | Sequential | ✅ Simultaneous |
| Zoom/Pan | ❌ | ✅ |
| Save Graphs | ❌ | ✅ |
| Color Coding | Limited | ✅ Full |

## Troubleshooting

### "Device or resource busy"
Close other programs using the port:
```bash
fuser -k /dev/ttyUSB0
pkill screen
```

### "Permission denied"
Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### GUI doesn't show up
1. Check if X11 is running: `echo $DISPLAY`
2. If using SSH, connect with X forwarding: `ssh -X user@host`
3. Or use VNC/remote desktop

### Plots are frozen
1. Check if ESP32 is sending data
2. Verify serial connection
3. Check motor is powered and CAN bus connected

### "ModuleNotFoundError: No module named 'matplotlib'"
Install matplotlib:
```bash
pip install matplotlib
# or
pip3 install matplotlib
```

## Performance Tips

1. **Reduce max_points** for slower systems:
   ```bash
   python3 monitor_motor_gui.py /dev/ttyUSB0 115200 50
   ```

2. **Close other applications** to free up resources

3. **Use full screen** for better visibility

## Use Cases

### 1. Motor Testing
Monitor motor behavior during testing phases to ensure stable operation.

### 2. Debug CAN Communication
Visualize communication quality by observing data continuity.

### 3. Temperature Monitoring
Track motor temperature during extended operation to prevent overheating.

### 4. Current Analysis
Observe torque current patterns during load changes.

### 5. Position Tracking
Monitor encoder and angle data for positioning accuracy.

### 6. Error Detection
Quickly identify voltage drops or error conditions.

## Exit

Close the GUI window or press `Ctrl+C` in the terminal to stop monitoring.

## Comparison: Terminal vs GUI

### Terminal Monitor (`monitor_motor.py`)
- ✅ Lightweight
- ✅ Works over SSH without X forwarding
- ✅ Lower resource usage
- ❌ No visual plots
- ❌ Limited history view

### GUI Monitor (`monitor_motor_gui.py`)
- ✅ Visual plots for all parameters
- ✅ Historical trend analysis
- ✅ Zoom/pan capabilities
- ✅ Save plots as images
- ❌ Requires GUI environment
- ❌ Higher resource usage

Choose based on your needs:
- **Remote monitoring**: Use terminal version
- **Local analysis**: Use GUI version
- **Extended testing**: Use GUI version for trend analysis
