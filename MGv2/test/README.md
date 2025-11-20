# ExoPulse Test Programs

This directory contains various test programs for hardware verification and development.

## Test Programs

### Hardware Tests (No Motor Required)

#### `test_led.h`
- **Purpose**: Verify ESP32 board is working
- **Hardware**: ESP32 only
- **Usage**: Tests LED functionality with various blink patterns
- **How to use**:
  1. Rename to `main.cpp` in `src/`
  2. Build and upload
  3. Observe LED patterns

#### `test_loopback.h`
- **Purpose**: Verify MCP2515 CAN controller without external CAN device
- **Hardware**: ESP32 + MCP2515
- **Usage**: Uses MCP2515 loopback mode for self-test
- **How to use**:
  1. Rename to `main.cpp` in `src/`
  2. Build and upload
  3. Monitor serial output for pass/fail results

### CAN Communication Tests (Requires 2 Boards)

#### `test_can_sender.cpp`
- **Purpose**: Send test CAN messages
- **Hardware**: ESP32 + MCP2515 + CAN termination
- **Usage**: Continuously sends CAN frames with counter
- **How to use**:
  1. Copy to `src/main.cpp`
  2. Build and upload to board #1
  3. Use with test_can_receiver on board #2

#### `test_can_receiver.h`
- **Purpose**: Receive and verify CAN messages
- **Hardware**: ESP32 + MCP2515 + CAN termination
- **Usage**: Receives CAN frames and checks for missing packets
- **How to use**:
  1. Rename to `main.cpp` in `src/`
  2. Build and upload to board #2
  3. Use with test_can_sender on board #1
  4. Connect CAN_H and CAN_L between boards
  5. **Important**: Add 120Ω termination resistors on both ends

### Legacy Test Code

#### `test0.txt` and `test1.txt`
- Earlier versions of motor control test code
- Reference for protocol implementation
- Not meant to be compiled directly

## General Setup

### SPI Wiring (All Tests)
```
MCP2515 → ESP32
CS      → GPIO 5
SCK     → GPIO 18
MISO    → GPIO 19
MOSI    → GPIO 23
VCC     → 3.3V or 5V
GND     → GND
```

### CAN Bus Wiring (Communication Tests)
```
Board 1 MCP2515    Board 2 MCP2515
CAN_H ------------ CAN_H
CAN_L ------------ CAN_L
GND   ------------ GND

120Ω termination resistor on BOTH ends
```

## Testing Workflow

1. **Start with LED test**: Verify board works
2. **Run loopback test**: Verify MCP2515 connection
3. **Run sender/receiver**: Verify CAN communication
4. **Deploy main application**: Use actual motor control

## Serial Monitor Settings

- Baud rate: 115200 (115200 for most tests, check individual files)
- Line ending: Newline (LF)

## Notes

- All test programs use standard PlatformIO build process
- Only one `.cpp` file should exist in `src/` at a time
- The main application is in `src/main.cpp` (production code)
- Crystal frequency: 8MHz (primary) or 16MHz (fallback)
