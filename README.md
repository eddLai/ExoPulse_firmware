# ExoPulse Firmware

ESP32-based CAN bus control system for exoskeleton motor control using MCP2515.

## Features

- ✅ MCP2515 CAN controller support (via dedalqq/esp32-mcp2515)
- ✅ Serial console with command interface
- ✅ Torque/Speed/Position control modes
- ✅ Real-time motor state feedback
- ✅ Multi-joint support (up to 4 motors)

## Hardware Requirements

- ESP32 DevKit V1
- MCP2515 CAN module (8MHz crystal)
- CAN-compatible motor controller

## Wiring

| ESP32 Pin | MCP2515 Pin |
|-----------|-------------|
| GPIO 18   | SCK         |
| GPIO 19   | MISO        |
| GPIO 23   | MOSI        |
| GPIO 5    | CS          |
| 3.3V      | VCC         |
| GND       | GND         |

## Build & Upload

```bash
platformio run --target upload
platformio device monitor --baud 230400
```

## Serial Commands

```
TORQ <id> <Nm>      - Set torque in Newton-meters
IQ <id> <iq>        - Set torque current (raw Iq)
SPEED <id> <dps>    - Set speed in degrees/second
POS <id> <deg>      - Set position in degrees
ZERO <id|ALL>       - Zero encoder at current position
STATUS <id>         - Query motor state
STOP                - Emergency stop
HELP                - Show command list
```

## Example Usage

```
IQ 1 50          # Set motor 1 to Iq=50
SPEED 1 10.5     # Set motor 1 to 10.5 deg/s
POS 1 90.0       # Move motor 1 to 90 degrees
ZERO 1           # Zero motor 1 encoder
STATUS 1         # Query motor 1 state
```

## License

MIT
