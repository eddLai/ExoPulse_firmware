# ExoPulse MGv2 - Technical Documentation

**Project:** ExoPulse Exoskeleton Control System
**Version:** 2.0
**Last Updated:** November 20, 2025

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Software Architecture](#3-software-architecture)
4. [CAN Bus Protocol](#4-can-bus-protocol)
5. [API Reference](#5-api-reference)
6. [FreeRTOS Task Management](#6-freertos-task-management)
7. [WiFi Communication](#7-wifi-communication)

---

## 1. System Overview

### 1.1 Project Description

ExoPulse is an ESP32-based exoskeleton control system integrating:
- **Wireless Communication**: WiFi STA mode
- **Motor Control**: CAN Bus interface to LK-TECH MGv2 motors
- **Real-time Monitoring**: Python GUI and web interface
- **Multi-core Processing**: FreeRTOS for efficient dual-core utilization

### 1.2 Technology Stack

| Component | Technology |
|-----------|------------|
| **Microcontroller** | ESP32 (Xtensa dual-core, 240MHz) |
| **Operating System** | FreeRTOS |
| **Communication** | CAN Bus (ISO 11898-1) + WiFi (2.4GHz) |
| **CAN Controller** | MCP2515 + TJA1050 transceiver |
| **Development** | PlatformIO + Arduino Framework |
| **Languages** | C++ (firmware) + Python (monitoring) |

### 1.3 Key Features

- Dual motor control via CAN bus (IDs: 0x141, 0x142)
- Real-time data acquisition: temperature, current, voltage, speed, acceleration, multi-turn angle
- Software calibration for zero-point setting (no ROM write, unlimited calibrations)
- FreeRTOS multi-tasking for efficient ESP32 dual-core utilization
- Live monitoring GUI with matplotlib visualization

---

## 2. Hardware Architecture

### 2.1 Core Hardware Components

#### ESP32 DevKit v1
- **Processor**: Xtensa 32-bit LX6 dual-core @ 240MHz
- **Memory**: 520KB SRAM, 4MB Flash
- **WiFi**: 802.11 b/g/n (2.4GHz)
- **Bluetooth**: v4.2 BR/EDR and BLE
- **GPIO**: 34 pins
- **USB**: CP2102/CH340 UART-to-USB

#### MCP2515 CAN Controller
- **Interface**: SPI (max 10MHz)
- **CAN Rate**: 500 KBPS (standard)
- **Crystal**: 8MHz (backup: 16MHz)
- **Power**: **5V** (most modules require 5V, not 3.3V!)
- **Transceiver**: TJA1050

#### MGv2 (LK-TECH) Motor Driver
- **Communication**: CAN Bus
- **Motor IDs**: 0x141 (Motor 1), 0x142 (Motor 2)
- **Features**:
  - Multi-turn angle tracking
  - Acceleration data (Command 0x33)
  - Temperature and current monitoring
  - Software zero-point calibration

### 2.2 Pin Connections

#### ESP32 ↔ MCP2515 (SPI Connection)
```
ESP32 GPIO  →  MCP2515
─────────────────────────
GPIO 5      →  CS (Chip Select)
GPIO 23     →  SI/MOSI (Master Out Slave In)
GPIO 19     →  SO/MISO (Master In Slave Out)
GPIO 18     →  SCK (Clock)
GPIO 21     →  INT (Interrupt, optional)
3.3V or 5V  →  VCC (check module requirements!)
GND         →  GND
```

**⚠️ Important**: Most MCP2515 modules require **5V** power, not 3.3V!

#### MCP2515 ↔ CAN Bus (Dual Motor System)
```
MCP2515  →  CAN Bus  →  Motor 1 (0x141)
                     →  Motor 2 (0x142)

Wiring:
CAN_H ─────────────── CAN_H (Motor 1, Motor 2)
CAN_L ─────────────── CAN_L (Motor 1, Motor 2)
GND ───────────────── GND (common ground)

Termination Resistors:
- 120Ω resistor at each end of the bus (between CAN_H and CAN_L)
```

---

## 3. Software Architecture

### 3.1 Project Structure

```
ExoPulse_firmware/
├── MGv2/                          # Motor control system (MGv2 motors)
│   ├── src/
│   │   ├── main.cpp               # Dual motor control main program
│   │   ├── main.h                 # Header file
│   │   └── monitor_dual_motor_enhanced.py  # GUI monitoring tool
│   ├── include/
│   │   ├── ExoBus.h               # CAN communication core
│   │   └── SerialConsole.h        # Serial command interface
│   ├── test/hardware_validation/  # Hardware validation tests
│   └── docs/                      # Technical documentation
├── src/
│   ├── WiFi_STA_Test.h            # WiFi test firmware
│   └── main_wifi_test.cpp         # WiFi test entry point
└── platformio.ini                 # Build configuration
```

### 3.2 Core Classes

#### ExoBus Class (`include/ExoBus.h`)

The `ExoBus` class provides a high-level abstraction for CAN bus communication with LK-TECH motors.

**Key Methods**:
```cpp
class ExoBus {
public:
    // Motor Control
    void setTorque(uint8_t motorId, float torqueNm);
    void setSpeed(uint8_t motorId, float speedDps);
    void setPosition(uint8_t motorId, float angleDeg);

    // Status Reading
    void readMotorStatus1(uint8_t motorId);  // Temperature, voltage, error
    void readMotorStatus2(uint8_t motorId);  // Current, speed, encoder
    void readMultiTurnAngle(uint8_t motorId);
    void readAcceleration(uint8_t motorId);

    // System Control
    void enableMotor(uint8_t motorId);
    void disableMotor(uint8_t motorId);
    void resetAngle(uint8_t motorId);
    void setZeroPosition(uint8_t motorId);
};
```

#### SerialConsole Class (`include/SerialConsole.h`)

Handles serial command input and processing.

**Supported Commands**:
- `CAL1` / `CAL_M1` - Calibrate Motor 1 zero position
- `CAL2` / `CAL_M2` - Calibrate Motor 2 zero position
- `CLEAR_CAL` - Clear all calibration offsets
- `HELP` - Display available commands
- `STATUS` - Show system status
- `DETAILED` - Enable detailed debug output

---

## 4. CAN Bus Protocol

### 4.1 Frame Format

All CAN frames follow this structure:
```
CAN ID = 0x140 + Motor_ID
Data Length = 8 bytes
Data[0] = Command Code
Data[1-7] = Parameters
```

### 4.2 Motor Control Commands

#### 0xA1 - Torque Control
```cpp
// Send Format
Data[0] = 0xA1           // Command code
Data[1-3] = 0x00         // Reserved
Data[4-5] = iq (int16)   // Torque current (-2048~2048 → -33A~33A)
Data[6-7] = 0x00         // Reserved

// Conversion Formula
int16_t iq = (int16_t)round(torqueNm * 20.0);
float actualCurrent = (float)iq_raw * 33.0f / 2048.0f;
```

#### 0xA2 - Speed Control
```cpp
// Send Format
Data[0] = 0xA2               // Command code
Data[1-3] = 0x00             // Reserved
Data[4-7] = speed (int32)    // Speed (unit: 0.01 dps)

// Conversion Formula
int32_t speed_raw = (int32_t)(speedDps * 100.0f);
float speedDps = (float)speed_raw / 100.0f;
```

#### 0xA3 - Position Control
```cpp
// Send Format
Data[0] = 0xA3                  // Command code
Data[1-3] = 0x00                // Reserved
Data[4-7] = position (int32)    // Position (unit: 0.01 degrees)

// Conversion Formula
int32_t position_raw = (int32_t)(angleDeg * 100.0f);
float angleDeg = (float)position_raw / 100.0f;
```

### 4.3 Status Query Commands

#### 0x9C - Read Motor Status 2 (Most Used)
```cpp
// Response Format
Data[0] = 0x9C                  // Command code echo
Data[1] = temperature (int8)    // Temperature (°C)
Data[2-3] = iq (int16)          // Torque current
Data[4-5] = speed (int16)       // Speed (dps)
Data[6-7] = encoder (uint16)    // Encoder position (0~16383)

// Parsing
int8_t temperature = (int8_t)buf[1];
int16_t iq_raw = (int16_t)(buf[2] | (buf[3] << 8));
int16_t speed_raw = (int16_t)(buf[4] | (buf[5] << 8));
uint16_t encoder = (uint16_t)(buf[6] | (buf[7] << 8));
```

#### 0x9A - Read Motor Status 1 (With Error Flags)
```cpp
// Response Format
Data[0] = 0x9A                  // Command code echo
Data[1] = temperature (int8)    // Temperature (°C)
Data[2] = 0x00                  // Reserved
Data[3-4] = voltage (uint16)    // Voltage (0.1V/LSB)
Data[5-6] = 0x00                // Reserved
Data[7] = errorState (uint8)    // Error flags

// Error Flags
if (errorState & 0x01) → Low voltage (LOW_VOLTAGE)
if (errorState & 0x08) → Over temperature (OVER_TEMP)
```

#### 0x92 - Read Multi-Turn Angle
```cpp
// Response Format
Data[0] = 0x92                    // Command code echo
Data[1-6] = motorAngle (int64)    // Angle (0.01°/LSB, 48-bit)
Data[7] = 0x00                    // Reserved

// Parsing
int64_t motorAngle = (int64_t)buf[1]
                   | ((int64_t)buf[2] << 8)
                   | ((int64_t)buf[3] << 16)
                   | ((int64_t)buf[4] << 24)
                   | ((int64_t)buf[5] << 32)
                   | ((int64_t)buf[6] << 40);
float angleDeg = (float)motorAngle * 0.01f;
```

#### 0x33 - Read Acceleration
```cpp
// Response Format
Data[0] = 0x33                        // Command code echo
Data[1-4] = acceleration (int32)      // Acceleration (1 dps²/LSB)
Data[5-7] = 0x00                      // Reserved

// Parsing
int32_t accel_raw = (int32_t)(buf[1]
                             | (buf[2] << 8)
                             | (buf[3] << 16)
                             | (buf[4] << 24));
float accel_dps2 = (float)accel_raw;  // dps²
```

---

## 5. API Reference

### 5.1 Motor Control Functions

#### setTorque()
```cpp
void ExoBus::setTorque(uint8_t motorId, float torqueNm)
```
Sets motor torque in Newton-meters.

**Parameters**:
- `motorId`: Motor ID (1 or 2)
- `torqueNm`: Torque in Nm (range: ±30 Nm)

**Example**:
```cpp
exoBus.setTorque(1, 5.0);  // Set motor 1 to 5 Nm
```

#### setSpeed()
```cpp
void ExoBus::setSpeed(uint8_t motorId, float speedDps)
```
Sets motor speed in degrees per second.

**Parameters**:
- `motorId`: Motor ID (1 or 2)
- `speedDps`: Speed in degrees/second

**Example**:
```cpp
exoBus.setSpeed(1, 180.0);  // Set motor 1 to 180°/s
```

#### setPosition()
```cpp
void ExoBus::setPosition(uint8_t motorId, float angleDeg)
```
Sets target motor position in degrees.

**Parameters**:
- `motorId`: Motor ID (1 or 2)
- `angleDeg`: Target angle in degrees

**Example**:
```cpp
exoBus.setPosition(1, 90.0);  // Move motor 1 to 90°
```

### 5.2 Status Reading Functions

#### readMotorStatus2()
```cpp
void ExoBus::readMotorStatus2(uint8_t motorId)
```
Reads motor status including temperature, current, speed, and encoder position.

**Returns** (via callback or global variables):
- Temperature (°C)
- Torque current (A)
- Speed (dps)
- Encoder position (0-16383)

#### readMultiTurnAngle()
```cpp
void ExoBus::readMultiTurnAngle(uint8_t motorId)
```
Reads multi-turn angle (tracks rotations beyond ±360°).

**Returns**:
- Angle in degrees (can exceed ±360°)

#### readAcceleration()
```cpp
void ExoBus::readAcceleration(uint8_t motorId)
```
Reads motor angular acceleration.

**Returns**:
- Acceleration in dps² (degrees per second squared)

---

## 6. FreeRTOS Task Management

### 6.1 Dual-Core Architecture

The system utilizes ESP32's dual-core architecture:

**Core 0 (Arduino Loop)**:
- Serial communication
- Command processing
- Watchdog feeding

**Core 1 (FreeRTOS Task)**:
- Motor status reading (50ms cycle)
- CAN Bus communication
- Data parsing and display

### 6.2 Task Implementation

```cpp
void motorReadTask(void *param) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50ms = 20Hz

    for (;;) {
        // Read Motor 1 status
        exoBus.readMotorStatus2(1);
        exoBus.readMultiTurnAngle(1);
        exoBus.readAcceleration(1);

        // Read Motor 2 status
        exoBus.readMotorStatus2(2);
        exoBus.readMultiTurnAngle(2);
        exoBus.readAcceleration(2);

        // Parse received data
        exoBus.parseCANData();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

### 6.3 Task Synchronization

- **Mutex Protection**: CAN bus access protected by `canMutex`
- **Task Priority**: Motor read task runs at priority 1
- **Core Assignment**: Task pinned to Core 1 for consistent timing

---

## 7. WiFi Communication

### 7.1 WiFi Configuration

| Parameter | Value |
|-----------|-------|
| **SSID** | ExoPulse |
| **Password** | 12345666 |
| **Band** | 2.4GHz (ESP32 doesn't support 5GHz) |
| **IP Assignment** | DHCP (typically 192.168.43.xxx) |
| **TCP Port** | 8888 |
| **Encryption** | WPA2-PSK |

### 7.2 WiFi Commands

After connection, available commands:

| Command | Function |
|---------|----------|
| `START` | Start continuous transmission test |
| `STOP` | Stop test and display statistics |
| `STATS` | Show real-time statistics |
| `PING` | Test connection latency |
| `EXIT` | Exit client program |

### 7.3 WiFi Implementation

```cpp
// WiFi connection
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}

// Create TCP server
WiFiServer server(8888);
server.begin();

// Handle client connections
WiFiClient client = server.available();
if (client) {
    // Process commands
    String command = client.readStringUntil('\n');
    // Execute and respond
}
```

---

## 8. Software Calibration

### 8.1 Calibration System

The firmware implements **software angle calibration** (commands: CAL1/CAL2/CLEAR_CAL):

**Advantages**:
- ✅ No ROM write (unlimited calibrations)
- ✅ Instant calibration (no reboot required)
- ✅ Easily reversible with CLEAR_CAL
- ✅ Perfect for testing and development

**Note**: Calibration offsets reset on ESP32 reboot (by design for safety).

### 8.2 Calibration Implementation

```cpp
// Calibration offset storage
float motor1_angle_offset = 0.0f;
float motor2_angle_offset = 0.0f;

// Calibrate Motor 1
void calibrateMotor1() {
    motor1_angle_offset = -motor1_latest_angle;
    Serial.println("Motor 1 calibrated to zero!");
}

// Apply calibration
float getCalibratedAngle(uint8_t motorId, float rawAngle) {
    if (motorId == 1) {
        return rawAngle + motor1_angle_offset;
    } else if (motorId == 2) {
        return rawAngle + motor2_angle_offset;
    }
    return rawAngle;
}
```

### 8.3 Permanent Zero Setting

For permanent zero-point setting, use `send_set_zero.py` (Command 0x19):
- Writes to motor ROM
- Survives power cycles
- Use carefully (limited ROM write cycles)

---

## 9. Build Configuration

### 9.1 PlatformIO Environments

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
build_src_filter = +<main.cpp> +<*.h>
build_flags =
  -std=gnu++17
lib_deps=
  https://github.com/coryjfowler/MCP_CAN_lib
```

### 9.2 Memory Usage

| Environment | RAM Usage | Flash Usage |
|-------------|-----------|-------------|
| esp32doit-devkit-v1 | 6.6% (21.6KB) | 22.1% (289KB) |
| esp32-s3-n16r8 | 6.2% | 4.5% |
| firebeetle32 | 6.9% | 22.4% |

---

## 10. Known Issues and Limitations

### 10.1 Motor Angle Reset (0x95) Not Implemented

**Severity**: High
**Status**: Confirmed - Motor firmware limitation
**Impact**: LK-TECH M series motors

**Issue Summary**:
CAN bus command 0x95 (clear motor angle) is documented in the datasheet but **not fully implemented** in motor firmware. Motor sends ACK but doesn't actually reset multi-turn angle counter.

**Evidence**:
```
>>> Sent: RESET_M2
>>> [CMD] Resetting Motor 2 angle to zero...
>>> [OK] Motor 2 angle reset successful!

Motor 2 angle BEFORE reset: 630.05°
Motor 2 angle AFTER reset:  630.06° (unchanged)
```

**Workaround**:
- Use software calibration (CAL1/CAL2 commands)
- Hardware reset (power cycle motor)
- Use single-turn encoder (0x94 command, limited to 0-360°)

**Long-term Solutions**:
1. Contact manufacturer (LK-TECH) for firmware update
2. Consider alternative motors with proper angle reset support

---

## Appendix A: Command Reference Table

### Motor Control Commands

| Code | Command | Parameters | Response |
|------|---------|------------|----------|
| 0xA1 | Torque Control | iq (int16) | None |
| 0xA2 | Speed Control | speed (int32) | None |
| 0xA3 | Position Control | position (int32) | None |
| 0x80 | Enable Motor | None | ACK |
| 0x81 | Disable Motor | None | ACK |
| 0x19 | Set Zero Position | None | ACK |
| 0x95 | Reset Angle | None | ACK (but doesn't work!) |

### Status Query Commands

| Code | Command | Response Data |
|------|---------|--------------|
| 0x9A | Read Status 1 | Temp, Voltage, Error |
| 0x9C | Read Status 2 | Temp, Current, Speed, Encoder |
| 0x92 | Read Multi-Turn Angle | 48-bit angle |
| 0x33 | Read Acceleration | 32-bit acceleration |
| 0x94 | Read Single-Turn Angle | 16-bit angle (0-360°) |

---

**Document Version**: 1.0
**Created**: November 20, 2025
**Last Updated**: November 20, 2025
**Maintained by**: ExoPulse Development Team
