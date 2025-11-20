# ExoPulse Unified GUI Guide

## Overview

The ExoPulse Unified GUI (`gui.py`) provides a comprehensive control panel with sidebar navigation for all ExoPulse UI components.

## Interface Layout

```
┌─────────────────────────────────────────────────────────────┐
│                    ExoPulse GUI                             │
├──────────────┬──────────────────────────────────────────────┤
│              │                                              │
│  ExoPulse    │         Component Launcher                   │
│     GUI      │                                              │
│              │  Motor Control                               │
├──────────────┤  Basic motor control interface with          │
│              │  WiFi/UART support                           │
│ Low-Level    │                                              │
│ Components   │  Script: motor_control.py                    │
│              │                                              │
│ ⚙️  Motor    │  ┌──────────────────────────────────┐        │
│    Control   │  │   🚀 Launch Component            │        │
│              │  └──────────────────────────────────┘        │
│ ⚙️  Serial   │  ┌──────────────────────────────────┐        │
│    Reader    │  │   ⏹ Stop Component               │        │
│              │  └──────────────────────────────────┘        │
├──────────────┤                                              │
│              │  Status: Not running                         │
│ High-Level   │                                              │
│ Components   │  ┌─ Process Output ─────────────────┐        │
│              │  │                                   │        │
│ 📊 Motor     │  │ [Terminal output displayed here]  │        │
│    Monitor   │  │                                   │        │
│              │  └───────────────────────────────────┘        │
│ 📊 Dual Motor│                                              │
│    Plotter   │                                              │
│              │                                              │
│ 📊 WiFi      │                                              │
│    Monitor   │                                              │
│              │                                              │
│ 📊 WiFi Dual │                                              │
│    Motor Plot│                                              │
│              │                                              │
│ 📊 CAN       │                                              │
│    Plotter   │                                              │
│              │                                              │
│ 📊 EMG       │                                              │
│    Plotter   │                                              │
├──────────────┤                                              │
│   v1.0.0     │                                              │
└──────────────┴──────────────────────────────────────────────┘
```

## Features

### Sidebar Navigation
- **Organized by complexity**: Low-level and High-level components clearly separated
- **Visual indicators**: Icons (⚙️ for low-level, 📊 for high-level)
- **Active state**: Selected component highlighted in blue
- **Hover effects**: Smooth visual feedback on mouse hover

### Component Launcher
Each component provides:
- **Component name and description**
- **Script path reference**
- **Launch button** - Start the component as a subprocess
- **Stop button** - Terminate the running component
- **Status indicator** - Shows current state (Not running, Running, Stopped)
- **Process output log** - Real-time terminal output with color coding
  - Green text for standard output
  - Red text for error messages
  - Monospace font for readability

### Real-time Monitoring
- View component output in real-time
- Auto-scroll to latest output
- Color-coded messages (stdout: green, stderr: red)
- Process exit code display

## Components

### Low-Level Components

#### 1. Motor Control
- **Purpose**: Basic motor control interface
- **Features**: WiFi/UART communication, CAN/Lower Chip modes
- **Use case**: General motor control and testing

#### 2. Serial Reader
- **Purpose**: Simple serial port data reader
- **Features**: Lightweight, command-line style
- **Use case**: Quick serial data inspection

### High-Level Components

#### 3. Motor Monitor
- **Purpose**: Advanced monitoring with configurable plots
- **Features**: Sidebar controls, real-time visualization
- **Use case**: Comprehensive motor analysis

#### 4. Dual Motor Plotter
- **Purpose**: Real-time dual motor data visualization
- **Features**: 5-parameter plots (temp, current, speed, accel, angle)
- **Use case**: Detailed motor performance monitoring

#### 5. WiFi Monitor
- **Purpose**: WiFi-based remote monitoring
- **Features**: TCP/UDP communication
- **Use case**: Remote motor control over network

#### 6. WiFi Dual Motor Plotter
- **Purpose**: WiFi dual motor data plotter
- **Features**: Network-based real-time visualization
- **Use case**: Remote dual motor monitoring

#### 7. CAN Plotter
- **Purpose**: CAN bus data visualization
- **Features**: Real-time CAN message plotting
- **Use case**: CAN bus debugging and analysis

#### 8. EMG Plotter
- **Purpose**: EMG signal visualization
- **Features**: ADS1256 ADC data plotting
- **Use case**: Muscle activity monitoring

## Quick Start

### Launch the Unified GUI
```bash
python3 gui.py
```

### Using the Interface
1. Click on any component in the sidebar
2. Read the component description
3. Click "🚀 Launch Component" to start
4. Monitor the process output in the log viewer
5. Click "⏹ Stop Component" to terminate

### Switching Components
- Simply click another component in the sidebar
- Previous component continues running in background
- You can launch multiple components simultaneously

## Requirements

```bash
pip3 install PySide6
```

Additional requirements depend on which components you launch:
- `matplotlib` - For plotters
- `pyserial` - For serial communication
- `numpy` - For data processing

## Tips

1. **Multiple Components**: You can run multiple components at the same time
2. **Process Management**: Each component runs as an independent subprocess
3. **Log Monitoring**: Scroll through process output to debug issues
4. **Stop Before Restart**: Stop a component before relaunching it

## Troubleshooting

### Component Won't Launch
- Check that the script exists in `UI_components/`
- Verify all dependencies are installed
- Check the process output log for error messages

### Port Already in Use
- Stop the previous instance using the Stop button
- Check for other programs using the port
- Use `lsof /dev/ttyUSB0` to find processes using serial port

### GUI Won't Start
- Ensure PySide6 is installed: `pip3 install PySide6`
- Check Python version: Requires Python 3.7+
- Verify Qt platform plugin is available

## Architecture

The unified GUI uses:
- **PySide6** for the Qt6-based interface
- **QProcess** for subprocess management
- **QStackedWidget** for content switching
- **QSplitter** for resizable sidebar
- **Signal/Slot** mechanism for event handling

Each launched component runs as an independent Python subprocess, allowing:
- Process isolation
- Independent crashes don't affect main GUI
- Real-time output capture
- Clean process termination

---

**Version**: 1.0.0
**Platform**: Linux, macOS, Windows (with Qt support)
**License**: Part of ExoPulse project
