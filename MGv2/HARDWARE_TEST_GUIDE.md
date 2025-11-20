# ExoPulse Firmware - Hardware Testing Guide
## Ubuntu Native Environment Setup & Testing Procedures

**Target Environment:** Native Ubuntu (not WSL)
**Project Path:** `/mnt/d/ExoPulse_firmware` (adjust as needed)
**Date:** November 19, 2025

---

## Quick Start Checklist

- [ ] Ubuntu native environment ready
- [ ] ESP32 board connected via USB
- [ ] PlatformIO installed
- [ ] Serial permissions configured
- [ ] Project directory accessible
- [ ] Test equipment ready

---

## Part 1: Environment Setup (One-Time)

### Step 1.1: Install PlatformIO

```bash
# Install Python and pip
sudo apt update
sudo apt install -y python3 python3-pip python3-venv

# Install PlatformIO
pip3 install --user platformio

# Add to PATH (add to ~/.bashrc for persistence)
export PATH="$HOME/.local/bin:$PATH"

# Verify installation
pio --version
```

Expected output:
```
PlatformIO Core, version 6.1.x
```

---

### Step 1.2: Configure Serial Permissions

```bash
# Add user to dialout group (required for serial port access)
sudo usermod -a -G dialout $USER

# IMPORTANT: Log out and log back in for group change to take effect
# Or use: newgrp dialout

# Verify group membership
groups | grep dialout
```

Expected: Your username should show `dialout` group membership.

---

### Step 1.3: Detect ESP32 Board

```bash
# List USB devices
lsusb

# List serial ports (before plugging ESP32)
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Plug in ESP32 board

# List serial ports again (new device should appear)
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Check detailed device info
dmesg | tail -20
```

Expected:
- New device appears (typically `/dev/ttyUSB0` or `/dev/ttyACM0`)
- dmesg shows: "USB Serial converter now attached to ttyUSB0" or similar
- Common ESP32 chips: CP2102, CH340, FTDI

**Note your port:** `______________________` (e.g., /dev/ttyUSB0)

---

### Step 1.4: Navigate to Project

```bash
# Navigate to project directory
cd /mnt/d/ExoPulse_firmware

# Verify project files
ls -la src/main.cpp include/ExoBus.h examples/

# Set PATH for this session
export PATH="$HOME/.local/bin:$PATH"
```

---

## Part 2: Hardware Test Sequence

### Test 1: LED Blink Test
**Objective:** Verify ESP32 board is functional
**Hardware Required:** ESP32 board only (no MCP2515 needed)
**Duration:** ~2 minutes

#### Step 2.1: Prepare Test Code

```bash
# Backup production code (if not already done)
cp src/main.cpp src/main.cpp.backup

# Copy LED test to main
cp examples/test_led.h src/main.cpp

# Verify copy
head -20 src/main.cpp
```

Expected: Should see LED test code with comments about LED testing.

---

#### Step 2.2: Build and Upload

```bash
# Clean previous builds
pio run --target clean

# Build for your board (choose one):
pio run --environment esp32doit-devkit-v1
# OR
# pio run --environment esp32-s3-n16r8
# OR
# pio run --environment firebeetle32

# Upload to board (replace ttyUSB0 with your port)
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

**Build Success Indicators:**
```
✓ Building in release mode
✓ Compiling .pio/build/.../src/main.cpp.o
✓ Linking .pio/build/.../firmware.elf
✓ Building .pio/build/.../firmware.bin
[SUCCESS] Took X.XX seconds
```

**Upload Success Indicators:**
```
Writing at 0x00010000... (100 %)
Hash of data verified.
Leaving...
Hard resetting via RTS pin...
[SUCCESS] Took X.XX seconds
```

---

#### Step 2.3: Observe LED Behavior

**Expected Behavior:**
- LED patterns change every 5 seconds
- Pattern 1: All LEDs blink together (ON/OFF)
- Pattern 2: Running light (left to right)
- Pattern 3: Running light (right to left)
- Pattern 4: Alternate pattern (even/odd LEDs)
- Pattern 5: Random blink

**Serial Output (Optional - to monitor):**
```bash
pio device monitor --baud 115200 --port /dev/ttyUSB0
# Press Ctrl+C to exit
```

Expected serial output:
```
========================================
   ESP32 LED Test Program
========================================

Test 1: Blink all LEDs together
All LEDs: ON
All LEDs: OFF
...

Test 2: Running light (left to right)
LED GPIO 2 ON
LED GPIO 4 ON
...
```

**✅ TEST 1 RESULT:** PASS / FAIL ________

**Notes:** _________________________________

---

### Test 2: MCP2515 Loopback Test
**Objective:** Verify MCP2515 CAN controller without external CAN bus
**Hardware Required:** ESP32 + MCP2515 module (correctly wired)
**Duration:** ~5 minutes

#### Step 2.4: Verify Wiring

**CRITICAL: Verify these connections before proceeding**

```
MCP2515 Module → ESP32 Board
---------------------------------
VCC     → 3.3V (or 5V depending on module)
GND     → GND
SCK     → GPIO 18
MISO    → GPIO 19
MOSI    → GPIO 23
CS      → GPIO 5
INT     → (not used, leave disconnected)
```

**Photo/Visual Check:**
- [ ] All 6 connections verified
- [ ] No loose wires
- [ ] Correct voltage (3.3V or 5V per module specs)
- [ ] MCP2515 power LED on (if present)

---

#### Step 2.5: Prepare Loopback Test

```bash
# Copy loopback test to main
cp examples/test_loopback.h src/main.cpp

# Verify copy
head -30 src/main.cpp | grep -i loopback
```

Expected: Should see "Loopback Test" in comments.

---

#### Step 2.6: Build and Upload

```bash
# Build
pio run --environment esp32doit-devkit-v1

# Upload (adjust port as needed)
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

---

#### Step 2.7: Monitor Loopback Test

```bash
# Monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected Output (SUCCESS):**
```
=== MCP2515 Loopback Test ===
Initializing MCP2515...
[OK] MCP2515 initialized
Setting LOOPBACK mode...
[OK] Loopback mode set

Starting self-test...

[TX] Sent frame #0 ID=0x141 -> [RX] ID=0x141 len=8 data=9C 0 0 0 0 0 0 0  ✓ PASS
[TX] Sent frame #1 ID=0x141 -> [RX] ID=0x141 len=8 data=9C 0 0 0 1 0 0 0  ✓ PASS
[TX] Sent frame #2 ID=0x141 -> [RX] ID=0x141 len=8 data=9C 0 0 0 2 0 0 0  ✓ PASS
...
[TX] Sent frame #9 ID=0x141 -> [RX] ID=0x141 len=8 data=9C 0 0 0 9 0 0 0  ✓ PASS

--- Stats: Total=10, Success=10, Fail=0 ---
```

**Possible Errors and Solutions:**

**Error: "MCP2515 initialization FAILED"**
```
Causes:
1. Wiring incorrect (check SPI connections)
2. Wrong CS pin (should be GPIO 5)
3. MCP2515 not powered
4. MCP2515 module defective

Solutions:
- Re-check all 6 wire connections
- Verify 3.3V or 5V on VCC pin with multimeter
- Try swapping MCP2515 module
```

**Error: "No response" after TX**
```
Causes:
1. Crystal frequency mismatch (code tries 8MHz then 16MHz)
2. MCP2515 not in loopback mode

Solutions:
- Check serial output for which frequency was used
- Power cycle the board
- Re-flash firmware
```

**✅ TEST 2 RESULT:** PASS / FAIL ________

**Success Rate:** ______ / 10 frames (should be 10/10)

**Notes:** _________________________________

---

### Test 3: Production Firmware Test
**Objective:** Verify production firmware and serial console
**Hardware Required:** ESP32 + MCP2515 (motors optional)
**Duration:** ~10 minutes

#### Step 2.8: Restore Production Code

```bash
# Restore original production code
cp src/main.cpp.backup src/main.cpp

# OR if backup doesn't exist:
git checkout src/main.cpp

# Verify it's production code
head -20 src/main.cpp | grep -i "ExoBus\|SerialConsole"
```

Expected: Should see ExoBus and SerialConsole includes.

---

#### Step 2.9: Build and Upload Production Firmware

```bash
# Build production firmware
pio run --environment esp32doit-devkit-v1

# Upload
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

---

#### Step 2.10: Test Serial Console

```bash
# Monitor serial console
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected Boot Sequence:**
```
# Exo Serial Control ready @115200
========================================
# ExoBus Serial Console
========================================
# Commands:
#   TORQ <id> <Nm>      - Set torque (Nm)
#   IQ <id> <iq>        - Set torque (Iq raw)
#   SPEED <id> <dps>    - Set speed (deg/s)
#   POS <id> <deg>      - Set position (deg)
#   ZERO <id|ALL>       - Zero encoder
#   STATUS <id>         - Query motor state
#   DEBUG [<id>]        - Dump recent CAN frames
#   STOP                - Emergency stop
#   HELP                - Show this help
# ========================================

[INFO] Using coryjfowler MCP_CAN driver
[OK] ExoBus initialized
```

**If MCP2515 initialization fails:**
```
[ERROR] ExoBus initialization FAILED
Possible causes:
  1. Check MCP2515 module is powered
  2. Verify SPI wiring
  3. Check CAN_H and CAN_L connections
  4. Verify 120Ω termination resistor
```

**Note:** Without motors connected, initialization may still succeed, but STATUS commands will show no valid state. This is expected.

---

#### Step 2.11: Test Commands

**In the serial monitor, type each command and press Enter:**

**Test 1: HELP Command**
```
> HELP
```
Expected:
```
[RX] HELP
# Commands:
#   TORQ <id> <Nm>      - Set torque (Nm)
#   ...
OK
```

**Test 2: DEBUG Command (View CAN frames)**
```
> DEBUG
```
Expected:
```
[RX] DEBUG
[DEBUG] dumpRecentFrames() - start
[DEBUG] dumpRecentFrames() - end
OK
```

**Test 3: STATUS Command (Query motor - will fail without motor)**
```
> STATUS 1
```
Expected (no motor connected):
```
[RX] STATUS 1
ERR: no valid state
```

**Test 4: STOP Command**
```
> STOP
```
Expected:
```
[RX] STOP
[TX] STOP
OK
```

**Test 5: Invalid Command (Error Handling)**
```
> INVALID
```
Expected:
```
[RX] INVALID
[WARN] Unknown command: INVALID
ERR: unknown command
# Commands:
...
```

**✅ TEST 3 RESULT:** PASS / FAIL ________

**Commands Tested:**
- [ ] HELP - Working
- [ ] DEBUG - Working
- [ ] STATUS - Working (shows no motor as expected)
- [ ] STOP - Working
- [ ] Error handling - Working

**Notes:** _________________________________

---

## Part 3: Optional Motor Testing

**⚠️ WARNING: Only proceed if you have:**
- [ ] Actual motor hardware connected
- [ ] Motor controller powered
- [ ] CAN bus properly wired with 120Ω termination
- [ ] Emergency stop accessible
- [ ] Safety precautions in place

### Motor Test Sequence (If Hardware Available)

```bash
# In serial monitor:

1. ZERO 1              # Zero encoder on motor 1
   Expected: OK, motor position reset

2. IQ 1 5              # Apply minimal torque (5 Iq units)
   Expected: OK, motor should respond slightly

3. STATUS 1            # Check motor state
   Expected: Display angle, speed, current, temperature

4. IQ 1 0              # Release torque
   Expected: OK, motor relaxes

5. SPEED 1 5.0         # Slow rotation (5 deg/s)
   Expected: OK, motor rotates slowly

6. STOP                # Emergency stop
   Expected: OK, motor stops immediately

7. POS 1 10.0          # Move to 10 degrees
   Expected: OK, motor moves to position

8. STATUS 1            # Verify position
   Expected: angle ≈ 10.0 degrees

9. STOP                # Final stop
   Expected: OK
```

**✅ MOTOR TEST RESULT:** PASS / FAIL / NOT TESTED ________

**Notes:** _________________________________

---

## Part 4: Results Documentation

### Test Summary

| Test | Status | Notes |
|------|--------|-------|
| LED Test | ☐ PASS ☐ FAIL | |
| Loopback Test | ☐ PASS ☐ FAIL | |
| Serial Console | ☐ PASS ☐ FAIL | |
| Motor Control | ☐ PASS ☐ FAIL ☐ N/A | |

### Hardware Configuration

**ESP32 Board:**
- Model: _________________
- USB Port: _________________
- Environment Used: _________________

**MCP2515 Module:**
- Model: _________________
- Crystal: ☐ 8MHz ☐ 16MHz
- Power: ☐ 3.3V ☐ 5V

**Motor (if tested):**
- Model: _________________
- CAN ID: _________________
- Status: _________________

---

## Part 5: Troubleshooting

### Common Issues

#### Issue: "Permission denied" when accessing serial port
```bash
# Solution: Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### Issue: Port not found or changes
```bash
# Find current port
ls /dev/ttyUSB* /dev/ttyACM*

# Or monitor kernel messages
dmesg -w
# Then plug/unplug device
```

#### Issue: Upload fails with "Failed to connect"
```bash
# Solutions:
1. Hold BOOT button on ESP32 during upload
2. Try different USB cable
3. Check USB port works with: lsusb
4. Power cycle the board
5. Try lower upload speed in platformio.ini
```

#### Issue: MCP2515 initialization fails
```bash
# Check list:
1. Verify all 6 SPI connections
2. Check VCC voltage (should be 3.3V or 5V)
3. Verify CS pin is GPIO 5
4. Try different MCP2515 module
5. Check crystal frequency (8MHz vs 16MHz)
```

#### Issue: No serial output
```bash
# Solutions:
1. Verify correct baud rate: 115200
2. Press RESET button on ESP32
3. Try different terminal program
4. Check USB cable supports data (not charge-only)
```

---

## Part 6: Completion

### When All Tests Pass

```bash
# Save test results
cat > HARDWARE_TEST_RESULTS.txt << EOF
Hardware Testing Complete
Date: $(date)
Tester: [Your Name]

LED Test: PASS
Loopback Test: PASS
Serial Console: PASS
Motor Test: [PASS/N/A]

Board: [Model]
Port: [Device]
Notes: [Any observations]
EOF

# Commit results to git
git add HARDWARE_TEST_RESULTS.txt
git commit -m "Hardware validation complete - all tests passed

Tested on native Ubuntu with ESP32 + MCP2515.
All tier 1 and tier 3 tests successful.

See HARDWARE_TEST_RESULTS.txt for details."
```

### Ready for Merge

If all tests passed:
```bash
# Switch to master
git checkout master

# Merge bugfix branch
git merge bugfix/main-source-and-testing

# Push to remote (if applicable)
git push origin master
```

---

## Quick Reference Card

**Essential Commands:**
```bash
# Setup
export PATH="$HOME/.local/bin:$PATH"
cd /mnt/d/ExoPulse_firmware

# Build & Upload
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Monitor
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Switch Test Code
cp examples/test_led.h src/main.cpp        # LED test
cp examples/test_loopback.h src/main.cpp   # Loopback test
git checkout src/main.cpp                   # Production code

# Serial Commands (in monitor)
HELP
STATUS 1
DEBUG
STOP
```

**Port Detection:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -20
```

**Permissions:**
```bash
sudo usermod -a -G dialout $USER
# Then log out/in
```

---

## Contact & Support

**Documentation:**
- BUILD_REPORT.md - Technical details
- DEPLOYMENT_CHECKLIST.md - Full procedures
- HANDOFF.md - Project summary

**Common Serial Terminals:**
```bash
# PlatformIO (recommended)
pio device monitor --baud 115200

# Screen
screen /dev/ttyUSB0 115200

# Minicom
minicom -D /dev/ttyUSB0 -b 115200

# CuteICOM (GUI)
sudo apt install cutecom
cutecom
```

---

**END OF HARDWARE TEST GUIDE**

*Print this guide or keep it open during testing*
*ExoPulse Firmware Project - November 2025*
