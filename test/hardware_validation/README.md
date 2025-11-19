# Hardware Validation Test Results

This directory contains the hardware validation test results and programs for the ExoPulse CAN bus communication system.

## Test Status: ✅ PASSED (100% Success)

**Date:** November 19, 2025
**Result:** All hardware tests passed with 100% success rate

## Files

### Test Programs
- `test_can_sender_fixed.cpp` - CAN bus sender test program (fixed version)
- `test_can_receiver_fixed.cpp` - CAN bus receiver test program (fixed version)

### Documentation
- `CAN_BUS_TEST_RESULTS_FINAL.md` - Complete test results and analysis
- `CAN_BUS_WIRING_CHECKLIST.md` - Hardware wiring guide and checklist
- `HARDWARE_TEST_RESULTS.txt` - Initial hardware test results

## Key Findings

### Critical Configuration
- **MCP2515 Power:** Requires **5V** (not 3.3V!)
- **CAN Baud Rate:** 500 KBPS
- **Crystal:** 8 MHz
- **Termination:** 120Ω resistors required on both ends

### Test Results
- Sender: 40/40 messages sent successfully (100%)
- Receiver: 40/40 messages received successfully (100%)
- Zero errors, zero missed messages

## How to Run Tests

### Option 1: Manual Testing with Two Boards

1. Upload sender program to Board 1:
   ```bash
   cp test/hardware_validation/test_can_sender_fixed.cpp src/main.cpp
   pio run -t upload --upload-port /dev/ttyUSB0
   ```

2. Upload receiver program to Board 2:
   ```bash
   cp test/hardware_validation/test_can_receiver_fixed.cpp src/main.cpp
   pio run -t upload --upload-port /dev/ttyUSB1
   ```

3. Monitor both serial ports:
   ```bash
   # Terminal 1:
   screen /dev/ttyUSB0 115200

   # Terminal 2:
   screen /dev/ttyUSB1 115200
   ```

### Option 2: Automated Testing Script

See `CAN_BUS_TEST_RESULTS_FINAL.md` for Python scripts to automate testing.

## Hardware Requirements

- 2x ESP32 boards
- 2x MCP2515 CAN modules
- Proper CAN bus wiring (CAN_H, CAN_L, GND)
- 2x 120Ω termination resistors
- 5V power for MCP2515 modules

## References

For complete documentation, see:
- [CAN_BUS_TEST_RESULTS_FINAL.md](CAN_BUS_TEST_RESULTS_FINAL.md) - Full test report
- [CAN_BUS_WIRING_CHECKLIST.md](CAN_BUS_WIRING_CHECKLIST.md) - Wiring guide
