# ExoPulse MGv2 - Hardware Guide

**Project:** ExoPulse Exoskeleton Control System
**Version:** 2.0
**Last Updated:** November 20, 2025

---

## Table of Contents

1. [Hardware Components](#1-hardware-components)
2. [Wiring and Connections](#2-wiring-and-connections)
3. [Hardware Testing Procedures](#3-hardware-testing-procedures)
4. [Validation Results](#4-validation-results)
5. [Troubleshooting](#5-troubleshooting)
6. [Safety Guidelines](#6-safety-guidelines)

---

## 1. Hardware Components

### 1.1 Required Components

#### Core Components
- **ESP32 DevKit v1** (or compatible)
  - Xtensa dual-core processor @ 240MHz
  - 520KB SRAM, 4MB Flash
  - WiFi 2.4GHz
  - USB-to-UART (CP2102/CH340)

- **MCP2515 CAN Controller Module**
  - SPI interface
  - 8MHz or 16MHz crystal
  - TJA1050 CAN transceiver
  - **Power requirement**: 5V (most modules)

- **LK-TECH MGv2 Motor Drivers** (2x)
  - CAN bus interface
  - Motor IDs: 0x141, 0x142
  - Built-in motor encoders

#### Supporting Components
- USB cable (data-capable, not charge-only)
- Dupont wires or JST connectors
- 120Ω resistors (2x) for CAN bus termination
- 5V power supply for MCP2515
- 24V power supply for motors (check motor specs)

### 1.2 Hardware Specifications

#### ESP32 DevKit v1
```
Processor: Xtensa 32-bit LX6 dual-core @ 240MHz
Memory:    520KB SRAM
Flash:     4MB (some versions up to 16MB)
WiFi:      802.11 b/g/n (2.4GHz only)
Bluetooth: v4.2 BR/EDR and BLE
GPIO:      34 pins
SPI:       4 sets
I2C:       2 sets
UART:      3 sets
```

#### MCP2515 CAN Controller
```
Interface:       SPI (max 10MHz)
CAN Speed:       500 KBPS (standard), up to 1 MBPS
Crystal:         8MHz (some modules use 16MHz)
Power:           5V (CRITICAL - most modules don't work with 3.3V!)
Transceiver:     TJA1050
Temperature:     -40°C to +125°C
```

#### MGv2 Motor Driver
```
Communication:   CAN Bus (ISO 11898-1)
Control Modes:   Torque, Speed, Position
Feedback:        Temperature, Voltage, Current, Speed, Encoder, Multi-turn Angle, Acceleration
Encoder:         14-bit (16384 positions/revolution)
Torque Range:    ±30 Nm
Current Range:   ±33A
Power:           24V DC (typical)
```

---

## 2. Wiring and Connections

### 2.1 ESP32 to MCP2515 Wiring

```
ESP32 Pin    →    MCP2515 Pin    →    Function
──────────────────────────────────────────────────
GPIO 5       →    CS             →    Chip Select
GPIO 23      →    SI/MOSI        →    Master Out Slave In
GPIO 19      →    SO/MISO        →    Master In Slave Out
GPIO 18      →    SCK            →    SPI Clock
GPIO 21      →    INT            →    Interrupt (optional)
5V           →    VCC            →    Power (CRITICAL!)
GND          →    GND            →    Ground
```

**⚠️ CRITICAL**: Most MCP2515 modules require **5V** power supply!

#### Power Supply Options:
1. **5V from ESP32 VIN pin** (if ESP32 powered via USB)
2. **External 5V regulator**
3. Check your MCP2515 module datasheet!

#### Testing Power Requirements:
```bash
# If MCP2515 init fails, try:
1. Verify all SPI connections
2. Check VCC voltage with multimeter (should be ~5V)
3. Try different MCP2515 module
4. Check crystal frequency (8MHz vs 16MHz)
```

### 2.2 MCP2515 to CAN Bus Wiring

```
MCP2515          CAN Bus          Motor 1          Motor 2
─────────────────────────────────────────────────────────────
CAN_H     →      CAN_H     →      CAN_H      →     CAN_H
CAN_L     →      CAN_L     →      CAN_L      →     CAN_L
GND       →      GND       →      GND        →     GND
```

#### CAN Bus Topology:
```
[ESP32] → [MCP2515] → [120Ω] ─── CAN_H ─── [Motor1] ─── [Motor2] ─── [120Ω]
                             └─── CAN_L ───
```

### 2.3 Termination Resistors

**Required**: 120Ω resistor at **each end** of the CAN bus (between CAN_H and CAN_L)

```
Start of bus:        End of bus:
CAN_H ──┬── 120Ω ──┬── CAN_H
        │           │
CAN_L ──┘           └── CAN_L
```

**Why needed**:
- Prevents signal reflections
- Ensures signal integrity
- Required for reliable CAN communication

### 2.4 WiFi Connection

No physical wiring required! ESP32 connects wirelessly to mobile hotspot.

**Setup**:
1. Create mobile hotspot on phone
   - SSID: `ExoPulse`
   - Password: `12345666`
   - Band: **2.4GHz** (ESP32 doesn't support 5GHz!)
2. ESP32 connects automatically on boot
3. Get ESP32 IP from serial monitor
4. Connect PC to same hotspot
5. Use TCP client to connect to ESP32 IP:8888

---

## 3. Hardware Testing Procedures

### 3.1 Testing Tier System

**Tier 1: Basic Validation** (Required)
1. LED Test - Verify ESP32 board functionality
2. Loopback Test - Verify MCP2515 SPI communication

**Tier 2: CAN Communication** (Recommended)
1. CAN Bus send/receive test - Verify CAN physical layer

**Tier 3: Production Application** (Required)
1. Serial console test
2. Motor control test (requires physical motors)

### 3.2 Test 1: LED Blink Test (2 minutes)

**Purpose**: Verify ESP32 board is functioning

**Requirements**:
- ESP32 board
- USB cable
- Computer with PlatformIO

**Procedure**:
```bash
# Navigate to project directory
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# Copy LED test to main source
cp examples/test_led.h src/main.cpp

# Build firmware
pio run --environment esp32doit-devkit-v1

# Upload to ESP32
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Observe LED behavior
```

**Expected Result**:
- Built-in LED changes blinking pattern every 5 seconds
- Patterns: Fast blink → Slow blink → Double blink → Triple blink → Heartbeat

**Pass Criteria**: LED blinks in all 5 patterns

### 3.3 Test 2: MCP2515 Loopback Test (3 minutes)

**Purpose**: Verify MCP2515 SPI communication and CAN controller functionality

**Requirements**:
- ESP32 board
- MCP2515 module
- Correct wiring (ESP32 ↔ MCP2515)
- No motors needed for this test

**Procedure**:
```bash
# Copy loopback test to main source
cp examples/test_loopback.h src/main.cpp

# Build and upload
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected Output**:
```
======================================
  MCP2515 CAN Controller - Loopback Test
======================================
Initializing MCP2515...
✓ MCP2515 initialized successfully!
Mode set to: LOOPBACK

Starting loopback test...
[TX] Sent frame #0 ID=0x141 -> [RX] ID=0x141 ✓ PASS
[TX] Sent frame #1 ID=0x142 -> [RX] ID=0x142 ✓ PASS
...
[TX] Sent frame #9 ID=0x142 -> [RX] ID=0x142 ✓ PASS

--- Stats: Total=10, Success=10, Fail=0 ---
```

**Pass Criteria**: 100% success rate (all frames sent and received)

**Troubleshooting**:
If initialization fails:
1. Verify all 6 SPI connections
2. Check VCC is 5V (use multimeter!)
3. Verify CS pin is GPIO 5
4. Try different MCP2515 module
5. Check crystal frequency setting in code

### 3.4 Test 3: Serial Console Test (5 minutes)

**Purpose**: Verify serial command interface

**Requirements**:
- ESP32 board
- MCP2515 module (properly connected)
- Motors (optional for this test)

**Procedure**:
```bash
# Restore production firmware
git checkout src/main.cpp

# Build and upload
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Test Commands**:
```
HELP        # Should display command list
STATUS      # Should show system status
CAL1        # Calibrate motor 1 (will work without motors)
CAL2        # Calibrate motor 2
CLEAR_CAL   # Clear calibration
```

**Expected Responses**:
```
>>> HELP

ExoPulse Motor Control System - Available Commands:
──────────────────────────────────────────────────
Motor Calibration:
  CAL1, CAL_M1     - Calibrate Motor 1 zero position
  CAL2, CAL_M2     - Calibrate Motor 2 zero position
  CLEAR_CAL        - Clear all calibration offsets

System Commands:
  HELP             - Display this help message
  STATUS           - Show system status
  DETAILED         - Enable detailed debug output
──────────────────────────────────────────────────
```

**Pass Criteria**: All commands respond correctly

### 3.5 Test 4: CAN Bus Communication Test

**Purpose**: Verify CAN bus physical layer and motor communication

**Requirements**:
- Complete hardware setup (ESP32 + MCP2515 + Motors)
- Proper wiring with termination resistors
- Motors powered on

**Procedure**:
```bash
# Production firmware should already be uploaded
# Just monitor serial output
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected Output**:
```
========================================
  ExoPulse Motor Control System v2.0
========================================
Initializing MCP2515...
✓ MCP2515 initialized successfully!
✓ CAN bus ready (500 KBPS)

Starting motor monitoring task on Core 1...
✓ FreeRTOS task started

[M1] Temp: 28°C | Cur: 0.05A | Spd: 0 dps | Ang: 123.45° | Accel: 0 dps²
[M2] Temp: 27°C | Cur: 0.03A | Spd: 0 dps | Ang: 67.89° | Accel: 0 dps²
```

**Pass Criteria**:
- MCP2515 initializes successfully
- Receives data from both motors
- Data updates continuously (≥1Hz)
- No CAN bus errors

**Troubleshooting**:
If no motor data received:
1. Check CAN_H and CAN_L connections
2. Verify common ground (GND)
3. Check 120Ω termination resistors at both ends
4. Verify motors are powered on
5. Confirm motor CAN IDs (0x141, 0x142)

### 3.6 Test 5: WiFi Connectivity Test

**Purpose**: Verify WiFi connection and TCP server

**Requirements**:
- ESP32 board
- Mobile phone with hotspot capability
- PC or laptop

**Procedure**:
```bash
# Setup mobile hotspot
# SSID: ExoPulse
# Password: 12345666
# Band: 2.4GHz

# Upload WiFi test firmware
pio run -e wifi_test --target upload

# Monitor to get IP address
screen /dev/ttyUSB0 115200
```

**Expected Output**:
```
========================================
   WiFi Stability Test System
========================================
Connecting to mobile hotspot...
SSID: ExoPulse
..........
✓ WiFi Connected!
IP Address: 192.168.43.123  ← Note this IP
Signal Strength (RSSI): -45 dBm
TCP Server started on port: 8888

Waiting for clients...
```

**Test Client Connection** (Windows PowerShell):
```powershell
# Edit src/test_client.ps1 with the IP from serial monitor
$ServerIP = "192.168.43.123"  # Use IP from serial output
$ServerPort = 8888

# Run test client
.\src\test_client.ps1

# Try commands:
START
STATS
STOP
```

**Pass Criteria**:
- ESP32 connects to WiFi hotspot
- Gets valid IP address
- TCP server accepts connections
- Commands work correctly

---

## 4. Validation Results

### 4.1 Hardware Test Summary

| Test Item | Status | Success Rate | Notes |
|-----------|--------|--------------|-------|
| LED Test | ✅ PASS | 100% | ESP32 hardware validated |
| Loopback Test | ✅ PASS | 100% | MCP2515 SPI communication verified |
| Serial Console | ✅ PASS | 100% | All commands working |
| CAN Bus Communication | ✅ PASS | 100% | 40/40 messages successfully transmitted |
| WiFi Connection | ✅ PASS | 100% | TCP Server successfully established |
| Motor Control | ⏭️ PENDING | N/A | Requires physical motor hardware |

### 4.2 Key Findings

**MCP2515 Power Requirements**:
- ⚠️ **CRITICAL**: Most modules require **5V**, not 3.3V!
- Symptom: Initialization fails with 3.3V
- Solution: Connect VCC to 5V pin on ESP32 or external 5V supply

**CAN Bus Configuration**:
- ✅ Baud rate: 500 KBPS works reliably
- ✅ Crystal frequency: 8 MHz (verify your module!)
- ✅ Termination resistors: 120Ω at both ends required

**WiFi Performance**:
- ✅ Connection stable at 2.4GHz
- ⚠️ ESP32 **does not support 5GHz** WiFi
- ✅ TCP throughput adequate for motor data streaming

---

## 5. Troubleshooting

### 5.1 ESP32 Issues

#### ESP32 Not Detected
**Symptoms**: No `/dev/ttyUSB*` device appears

**Solutions**:
```bash
# Check USB connection
lsusb

# Check kernel messages
dmesg | tail -20

# Try different USB cable (must be data cable, not charge-only)
# Try different USB port
# Install CH340/CP2102 drivers if needed
```

#### Upload Fails
**Symptoms**: "Failed to connect" or timeout during upload

**Solutions**:
```bash
# Hold BOOT button during upload
# Try lower upload speed in platformio.ini:
upload_speed = 115200

# Power cycle the board
# Try different USB cable
# Check USB port: lsusb
```

### 5.2 MCP2515 Issues

#### MCP2515 Initialization Fails
**Symptoms**: "ERROR: MCP2515 initialization failed!"

**Solutions**:
1. **Check Power Supply** (MOST COMMON ISSUE!)
   ```bash
   # Use multimeter to measure VCC pin
   # Should read ~5V, not 3.3V
   ```

2. **Verify SPI Connections**:
   ```
   ESP32 GPIO 5  → MCP2515 CS   ✓
   ESP32 GPIO 23 → MCP2515 MOSI ✓
   ESP32 GPIO 19 → MCP2515 MISO ✓
   ESP32 GPIO 18 → MCP2515 SCK  ✓
   ESP32 GND     → MCP2515 GND  ✓
   ESP32 5V      → MCP2515 VCC  ✓
   ```

3. **Check Crystal Frequency**:
   ```cpp
   // In code, verify:
   MCP_8MHZ    // or MCP_16MHZ (check your module!)
   ```

4. **Try Different Module**:
   - Some MCP2515 modules are defective
   - Test with known-good module

#### CAN Bus No Data Received
**Symptoms**: MCP2515 initializes but no motor data

**Solutions**:
1. **Check CAN Wiring**:
   ```
   CAN_H to CAN_H (not to CAN_L!) ✓
   CAN_L to CAN_L (not to CAN_H!) ✓
   Common GND                      ✓
   ```

2. **Verify Termination Resistors**:
   ```
   120Ω at start of bus (between CAN_H and CAN_L)
   120Ω at end of bus   (between CAN_H and CAN_L)
   ```

3. **Check Motor Power**:
   ```bash
   # Motors must be powered on
   # Check motor power LED
   # Verify motor CAN IDs (0x141, 0x142)
   ```

4. **Test CAN Bus Voltage**:
   ```bash
   # With multimeter:
   # CAN_H should be ~3.5V (recessive state)
   # CAN_L should be ~1.5V (recessive state)
   # Difference should be ~2V
   ```

### 5.3 Serial Port Issues

#### Permission Denied
**Symptoms**: "Permission denied" when accessing `/dev/ttyUSB0`

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in
# Or use: newgrp dialout

# Verify group membership
groups | grep dialout
```

#### Port Busy
**Symptoms**: "Device or resource busy"

**Solutions**:
```bash
# Check what's using the port
lsof /dev/ttyUSB0

# Kill screen sessions
pkill screen

# Kill process using port
fuser -k /dev/ttyUSB0
```

#### Port Number Changes
**Symptoms**: Port changes from `/dev/ttyUSB0` to `/dev/ttyUSB1`

**Solutions**:
```bash
# Find current port
ls /dev/ttyUSB* /dev/ttyACM*

# Or monitor kernel messages
dmesg -w
# Then plug/unplug device
```

### 5.4 WiFi Issues

#### ESP32 Won't Connect to WiFi
**Symptoms**: "Connecting..." but never connects

**Solutions**:
1. **Check SSID and Password**:
   ```cpp
   const char* ssid = "ExoPulse";      // Case-sensitive!
   const char* password = "12345666";  // Correct password?
   ```

2. **Verify 2.4GHz Band**:
   ```
   ⚠️ ESP32 does NOT support 5GHz WiFi!
   Must use 2.4GHz hotspot
   ```

3. **Check Hotspot Settings**:
   ```
   - SSID: ExoPulse (exact match)
   - Band: 2.4GHz only
   - Security: WPA2-PSK
   - Max clients: Allow at least 2 (ESP32 + PC)
   ```

#### Can't Connect to ESP32 TCP Server
**Symptoms**: Connection refused or timeout

**Solutions**:
1. **Get ESP32 IP from Serial Monitor**:
   ```
   Look for: "IP Address: 192.168.43.xxx"
   ```

2. **Verify PC on Same Network**:
   ```bash
   # PC must be connected to same "ExoPulse" hotspot
   # Not to home WiFi or other network
   ```

3. **Check Firewall**:
   ```powershell
   # Windows: Allow port 8888 in firewall
   # Or temporarily disable firewall for testing
   ```

4. **Test Connection**:
   ```bash
   # Linux/Mac:
   nc -zv 192.168.43.123 8888

   # Windows PowerShell:
   Test-NetConnection -ComputerName 192.168.43.123 -Port 8888
   ```

### 5.5 Motor Issues

#### Motor Not Responding
**Symptoms**: No data from specific motor

**Solutions**:
1. **Check Motor Power**:
   ```bash
   # Verify motor power supply is ON
   # Check motor power LED
   ```

2. **Verify Motor CAN ID**:
   ```bash
   # Motor 1 should be: 0x141 (ID=1)
   # Motor 2 should be: 0x142 (ID=2)
   # Check motor DIP switches or configuration
   ```

3. **Check Individual Motor Wiring**:
   ```
   CAN_H connected? ✓
   CAN_L connected? ✓
   GND connected?   ✓
   ```

#### Angle Shows "ovf" (Overflow)
**Symptoms**: Motor angle displays "ovf" in GUI

**Solutions**:
1. **Use Software Calibration**:
   ```bash
   # In serial monitor or GUI:
   CAL1      # Calibrate Motor 1
   CAL2      # Calibrate Motor 2
   ```

2. **Power Cycle Motor**:
   ```bash
   # Turn motor power off, wait 5 seconds, turn on
   ```

3. **Known Issue**:
   ```
   Command 0x95 (reset angle) not fully implemented in motor firmware
   See Technical Documentation Section 10.1 for details
   ```

---

## 6. Safety Guidelines

### 6.1 Electrical Safety

**Power Connections**:
- ⚠️ Verify voltage before connecting (5V for MCP2515, 24V for motors)
- Use proper gauge wire for motor power (high current!)
- Check polarity before powering on
- Never hot-plug motor power cables

**ESD Protection**:
- Use ESD wrist strap when handling ESP32 and MCP2515
- Work on ESD-safe mat if available
- Avoid touching component pins

### 6.2 Mechanical Safety

**Motor Operation**:
- ⚠️ Motors can move unexpectedly during testing
- Keep hands clear of moving parts
- Use emergency stop button/switch
- Secure motors to prevent movement during testing

**Exoskeleton Operation**:
- Never operate exoskeleton without proper safety training
- Use appropriate safety equipment
- Have emergency stop easily accessible
- Test in safe environment first

### 6.3 CAN Bus Best Practices

**Wiring**:
- Use twisted pair cable for CAN_H and CAN_L
- Keep CAN cable length < 40m for 500 KBPS
- Avoid running CAN cable parallel to power cables
- Use shielded cable for noisy environments

**Termination**:
- Always use 120Ω termination resistors at both ends
- Only at the ends (not in the middle!)
- Use proper resistor wattage (1/4W sufficient)

**Grounding**:
- Maintain common ground between all devices
- Use star grounding topology if possible
- Avoid ground loops

---

## Appendix A: Hardware Test Checklist

### Pre-Test Checklist
- [ ] Ubuntu native environment ready
- [ ] ESP32 board connected via USB
- [ ] PlatformIO installed and verified
- [ ] Serial permissions configured (dialout group)
- [ ] Project directory accessible
- [ ] Test equipment ready

### Tier 1 Tests
- [ ] LED Test completed - PASS/FAIL
- [ ] Loopback Test completed - PASS/FAIL

### Tier 2 Tests
- [ ] Serial Console Test completed - PASS/FAIL
- [ ] CAN Bus Communication Test completed - PASS/FAIL

### Tier 3 Tests
- [ ] WiFi Connection Test completed - PASS/FAIL
- [ ] Motor Control Test completed - PASS/FAIL/N/A

### Hardware Configuration Record

**ESP32 Board**:
- Model: _________________________
- USB Port: _______________________
- Environment: ____________________

**MCP2515 Module**:
- Model: _________________________
- Crystal: ☐ 8MHz ☐ 16MHz
- Power: ☐ 3.3V ☐ 5V

**Motors** (if tested):
- Model: _________________________
- CAN ID: ________________________
- Status: ________________________

---

## Appendix B: Quick Reference

### Common Commands
```bash
# Find ESP32 port
ls /dev/ttyUSB* /dev/ttyACM*

# Build firmware
pio run -e esp32doit-devkit-v1

# Upload firmware
pio run -t upload -e esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# Monitor serial
pio device monitor --baud 115200 --port /dev/ttyUSB0

# Check USB devices
lsusb

# Check kernel messages
dmesg | tail -20

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

### Pin Quick Reference
```
ESP32  →  MCP2515
GPIO5  →  CS
GPIO23 →  MOSI
GPIO19 →  MISO
GPIO18 →  SCK
GPIO21 →  INT (optional)
5V     →  VCC (IMPORTANT!)
GND    →  GND
```

### CAN Bus Quick Check
```
Component          Required?    Value
─────────────────────────────────────
Termination (start) YES          120Ω
Termination (end)   YES          120Ω
CAN_H wiring        YES          Connected
CAN_L wiring        YES          Connected
Common GND          YES          Connected
Cable type          RECOMMENDED  Twisted pair
```

---

**Document Version**: 1.0
**Created**: November 20, 2025
**Last Updated**: November 20, 2025
**Maintained by**: ExoPulse Development Team
