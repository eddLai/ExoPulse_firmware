# ExoPulse Firmware - Deployment Checklist

**Version:** 1.0.0-bugfix
**Date:** November 19, 2025
**Branch:** bugfix/main-source-and-testing → master

---

## Pre-Deployment Verification

### ✅ Code Quality
- [x] All critical bugs fixed
- [x] Code compiles without errors (3/3 environments)
- [x] Static analysis passed (0 HIGH, 0 MEDIUM)
- [x] Code organization meets standards
- [x] Documentation up-to-date

### ✅ Build Verification
```
Environment          Status    Size (text/data/bss)      RAM    Flash
-------------------  --------  -----------------------  -----  -------
esp32doit-devkit-v1  SUCCESS   217949/76128/5873        6.9%   22.4%
esp32-s3-n16r8       SUCCESS   221141/71332/247701      6.2%   4.5%
firebeetle32         SUCCESS   217949/76128/5873        6.9%   22.4%
```

### ✅ Repository Status
- [x] Working tree clean
- [x] All changes committed
- [x] Commit message descriptive
- [x] Branch up-to-date

---

## Hardware Testing Requirements

### Test Tier 1: Basic Validation (REQUIRED)

#### 1.1 LED Test
**Objective:** Verify ESP32 board operation
**Hardware:** ESP32 only
**Procedure:**
```bash
# Backup current main.cpp
cp src/main.cpp src/main.cpp.backup

# Load LED test
cp examples/test_led.h src/main.cpp

# Upload and monitor
export PATH="/root/.local/bin:$PATH"
pio run -t upload --environment esp32doit-devkit-v1
pio device monitor --baud 115200
```
**Expected:** LED patterns cycle every 5 seconds
**Pass Criteria:** LEDs respond correctly

---

#### 1.2 MCP2515 Loopback Test
**Objective:** Verify MCP2515 CAN controller without external devices
**Hardware:** ESP32 + MCP2515
**Procedure:**
```bash
# Load loopback test
cp examples/test_loopback.h src/main.cpp

# Upload and monitor
pio run -t upload --environment esp32doit-devkit-v1
pio device monitor --baud 115200
```
**Expected Output:**
```
[TX] Sent frame #0 ID=0x141 -> [RX] ID=0x141 len=8 data=... ✓ PASS
[TX] Sent frame #1 ID=0x141 -> [RX] ID=0x141 len=8 data=... ✓ PASS
...
--- Stats: Total=10, Success=10, Fail=0 ---
```
**Pass Criteria:** 100% success rate on loopback frames

---

### Test Tier 2: CAN Communication (RECOMMENDED)

#### 2.1 CAN Bus Sender/Receiver Test
**Objective:** Verify CAN bus communication
**Hardware:** 2x ESP32 + 2x MCP2515 + CAN wiring + 120Ω terminators

**Setup:**
```
Board 1 (Sender):
  - Copy examples/test_can_sender.cpp to src/main.cpp
  - Upload to first board

Board 2 (Receiver):
  - Copy examples/test_can_receiver.h to src/main.cpp
  - Upload to second board

Wiring:
  MCP2515_1 CAN_H ─────────── CAN_H MCP2515_2
  MCP2515_1 CAN_L ─────────── CAN_L MCP2515_2
  MCP2515_1 GND   ─────────── GND   MCP2515_2

  120Ω resistor between CAN_H and CAN_L on BOTH ends
```

**Expected:**
- Sender shows continuous TX success
- Receiver displays incoming messages with correct counter
- Zero missed messages

**Pass Criteria:** Clean CAN communication, no errors

---

### Test Tier 3: Production Application (REQUIRED)

#### 3.1 Serial Console Test
**Objective:** Verify production firmware functionality
**Hardware:** ESP32 + MCP2515 (motors optional)

**Procedure:**
```bash
# Restore production code
cp src/main.cpp.backup src/main.cpp
# OR it's already there as default

# Upload
pio run -t upload --environment esp32doit-devkit-v1
pio device monitor --baud 115200
```

**Test Commands:**
```
1. HELP           # Should display command list
2. STATUS 1       # Query motor 1 (may show no valid state if no motor)
3. DEBUG 1        # Dump CAN frames for motor 1
4. IQ 1 10        # Set motor 1 torque (safe low value)
5. STOP           # Emergency stop
```

**Pass Criteria:**
- All commands execute without errors
- Console responds correctly
- Watchdog does not trigger unexpectedly

---

#### 3.2 Motor Control Test (PRODUCTION HARDWARE)
**Objective:** Full system integration test
**Hardware:** ESP32 + MCP2515 + Motor controller + Motor

**Safety Checklist:**
- [ ] Motor securely mounted
- [ ] Emergency stop accessible
- [ ] No obstacles in movement path
- [ ] CAN termination resistors installed
- [ ] Power supply adequate
- [ ] Personnel clear of danger zone

**Test Sequence:**
```
1. Connect and power on
2. HELP                  # Verify console
3. STATUS 1              # Check initial state
4. ZERO 1                # Zero encoder
5. IQ 1 5                # Apply minimal torque
6. STATUS 1              # Verify torque applied
7. IQ 1 0                # Release torque
8. SPEED 1 5.0           # Slow speed test (5 deg/s)
9. STATUS 1              # Monitor speed
10. STOP                 # Emergency stop
11. POS 1 10.0           # Small position move (10 degrees)
12. STATUS 1             # Verify position
13. STOP                 # Final stop
```

**Pass Criteria:**
- Motor responds to all commands
- Feedback data is accurate
- Emergency stop works immediately
- Watchdog triggers on communication loss
- No unexpected behavior

---

## Deployment Steps

### 1. Merge to Master
```bash
# Switch to master
git checkout master

# Merge bugfix branch
git merge bugfix/main-source-and-testing

# Verify merge
git log --oneline -5
```

### 2. Tag Release
```bash
# Create annotated tag
git tag -a v1.0.0-stable -m "Production release: Critical bug fixes

- Fixed missing main.cpp entry point
- Reorganized test programs
- Standardized configurations
- Verified all build targets
- Passed static analysis

Ready for production deployment."

# Push tag
git push origin v1.0.0-stable
```

### 3. Production Build
```bash
# Clean build for production
export PATH="/root/.local/bin:$PATH"
pio run --target clean
pio run --environment esp32doit-devkit-v1

# Verify binary
ls -lh .pio/build/esp32doit-devkit-v1/firmware.bin
```

### 4. Backup Current Firmware (if updating existing system)
```bash
# Create backup directory
mkdir -p backups/$(date +%Y%m%d)

# Backup current firmware (if retrievable)
# Document current configuration
# Save serial logs
```

### 5. Upload to Production Hardware
```bash
# Upload firmware
pio run -t upload --environment esp32doit-devkit-v1

# Monitor boot sequence
pio device monitor --baud 115200
```

### 6. Post-Deployment Verification
```bash
# Run production test suite
# Monitor for 15 minutes minimum
# Verify all commands respond
# Test emergency stop
# Validate motor control
# Check watchdog operation
```

---

## Rollback Procedure

If issues are encountered:

```bash
# 1. Stop system immediately
# Send STOP command or power off

# 2. Revert to previous firmware
git checkout v0.9.x-previous  # or appropriate version
pio run -t upload --environment esp32doit-devkit-v1

# 3. Document issue
# Create bug report with:
# - Exact failure description
# - Serial logs
# - Test conditions
# - Hardware configuration

# 4. Return to testing phase
# Do not proceed until issue resolved
```

---

## Sign-Off Checklist

### Development Team
- [ ] All unit tests passed
- [ ] Code review completed
- [ ] Documentation reviewed
- [ ] Build verification passed
- [ ] Static analysis passed

### QA/Testing Team
- [ ] LED test passed
- [ ] Loopback test passed
- [ ] CAN communication test passed
- [ ] Serial console test passed
- [ ] Motor control test passed
- [ ] Safety systems verified

### Deployment Authorization
- [ ] All tests completed successfully
- [ ] Hardware configuration documented
- [ ] Rollback procedure prepared
- [ ] Stakeholders informed
- [ ] Deployment window scheduled

**Authorized By:** ___________________ **Date:** ___________

**Deployed By:** ___________________ **Date:** ___________

---

## Production Environment

**Default Configuration:**
- Board: ESP32 DevKit V1
- Baud Rate: 115200
- CAN Speed: 1000 KBPS (fallback: 500 KBPS)
- MCP2515 Crystal: 8 MHz (fallback: 16 MHz)
- Watchdog Timeout: 2000 ms
- Motor IDs: 1, 2 (configurable)

**Safety Limits:**
- Max Torque: ±30 Nm
- Max Speed: ±50 deg/s
- Watchdog: Enabled after first command

---

## Support Contacts

**Issues:** https://github.com/anthropics/claude-code/issues
**Documentation:** README.md, BUILD_REPORT.md, examples/README.md

---

*Deployment checklist for ExoPulse Firmware v1.0.0*
*Generated: November 2025*
