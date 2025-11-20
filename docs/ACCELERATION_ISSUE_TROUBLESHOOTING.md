# ExoPulse Motor Acceleration Data Issue - Troubleshooting Handout

**Date:** 2025-11-20
**Firmware Version:** MGv2
**Motor Model:** LK-TECH M Series
**Status:** ⚠️ ONGOING INVESTIGATION

---

## 🔴 Problem Summary

During sine wave motor testing, the following anomalies were observed:

### 1. **Acceleration Data Reading Zero**
- **Symptom:** Both Motor 1 and Motor 2 acceleration values constantly show `0 dps/s`
- **Expected:** Should display varying INT32 acceleration values during sine wave motion
- **Impact:** Cannot monitor motor acceleration dynamics

### 2. **Angle Data Showing Abnormal Values**
- **Symptom:**
  - Motor 1: Angle displays `~3e12` (3 trillion degrees)
  - Motor 2: Angle displays `~2.8e12` (2.8 trillion degrees)
  - Graph shows square wave pattern instead of smooth sine wave
- **Expected:** Reasonable angle values in degrees (typically ±360° range or reasonable multi-turn values)
- **Impact:** Position feedback completely unreliable

### 3. **Current Data Shows Correct Patterns**
- **Observation:** Current readings for both motors display expected sine wave patterns
- **Conclusion:** CAN communication is working, but specific data fields are being parsed incorrectly

---

## 🔍 Root Cause Analysis

### Initial Issue: INT16 Truncation
**Problem:** Original code truncated INT32 acceleration to INT16
```cpp
// ❌ WRONG: Data loss
int32_t accel = (int32_t)(rxData[4] | ... | (rxData[7] << 24));
status.acceleration = (int16_t)(accel);  // Truncation!
```

**Fix Applied:**
```cpp
// ✅ CORRECT: Preserve full INT32
status.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) |
                                (rxData[6] << 16) | (rxData[7] << 24));
```

### Byte Position Investigation

**CAN Protocol Format (LK-TECH):**
```
CAN Response Structure:
Byte:  [0]      [1]  [2]  [3]  [4]  [5]  [6]  [7]
       CMD      ←──────── Data Payload ────────→
```

**Acceleration Command (0x33):**
- **Attempted Fix 1:** Used bytes `[1-4]` → ❌ FAILED (acceleration = 0)
- **Reverted to Original:** Using bytes `[4-7]` → ⚠️ Still shows 0

**Current Implementation:**
```cpp
// Read acceleration from bytes [4-7]
status.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) |
                                (rxData[6] << 16) | (rxData[7] << 24));
```

---

## 📊 Data Format Reference

### Motor Status Structure
```cpp
struct MotorStatus {
    uint8_t motorID;         // Motor ID (1 or 2)
    int8_t temperature;      // °C
    uint16_t voltage;        // 0.1V/LSB
    uint8_t errorState;      // Error flags
    int16_t torqueCurrent;   // iq: -2048~2048 → -33A~33A ✅ WORKING
    int16_t speed;           // dps (degrees per second) ✅ WORKING
    int32_t acceleration;    // dps/s (INT32) ❌ READS 0
    uint16_t encoder;        // 0~16383 (14-bit)
    int64_t motorAngle;      // 0.01°/LSB (multi-turn) ❌ ABNORMAL VALUES
    uint32_t timestamp;      // millis() when read
};
```

### Working Data Fields
- ✅ Temperature (INT8)
- ✅ Torque Current (INT16) - Shows correct sine wave
- ✅ Speed (INT16)
- ✅ Encoder (UINT16)
- ✅ Voltage (UINT16)

### Problematic Data Fields
- ❌ Acceleration (INT32) - Always 0
- ❌ Motor Angle (INT64) - Shows 1e12 magnitude values

---

## 🔧 Troubleshooting Steps Attempted

### Step 1: Fixed INT32 Truncation ✅
- **Action:** Changed `acceleration` from INT16 to INT32 in struct
- **Action:** Removed truncation in parsing function
- **Result:** Compile successful, but acceleration still reads 0

### Step 2: Byte Position Experiment ❌
- **Action:** Tried reading acceleration from bytes `[1-4]` instead of `[4-7]`
- **Result:** Made problem worse - both acceleration and angle became abnormal

### Step 3: Reverted to Original Byte Positions ⚠️
- **Action:** Restored acceleration reading from bytes `[4-7]`
- **Result:** Angle values still abnormal (1e12 magnitude)
- **Result:** Acceleration still reads 0

---

## 📋 Next Investigation Steps

### Priority 1: Verify CAN Response Format
- [ ] Use CAN sniffer/logic analyzer to capture raw motor responses
- [ ] Verify actual byte positions for acceleration data (0x33 command)
- [ ] Compare with LK-TECH official protocol documentation

### Priority 2: Check Multi-turn Angle Parsing
- [ ] Review `readMultiTurnAngle()` function
- [ ] Verify INT64 assembly from 6 bytes (bytes [1-6])
- [ ] Check for sign extension issues
- [ ] Validate 0.01°/LSB scaling factor

### Priority 3: Debug Output
- [ ] Add Serial.print() to show raw rxData bytes
- [ ] Print hex dump of CAN responses for each command
- [ ] Compare with known-good motor response data

### Priority 4: Motor Firmware Version
- [ ] Check if motor firmware supports 0x33 command properly
- [ ] Verify motor firmware version compatibility
- [ ] Test with single motor first

---

## 🗂️ Related Files

### Firmware Files
- `MGv2/include/motor_protocol.h` - Data structures and command definitions
- `MGv2/include/motor_operations.h` - CAN parsing functions
- `MGv2/src/main.cpp` - Main firmware logic
- `MGv2/include/output_manager.h` - Data formatting and output

### UI Files
- `UI_components/wifi_monitor.py` - Python monitoring GUI
- `UI_components/motor_control.py` - Motor control interface

### Documentation
- `docs/ACCELERATION_ISSUE_TROUBLESHOOTING.md` - This file
- `MGv2/include/motor_protocol.h` - Protocol documentation (lines 4-15)

---

## 🔬 Debug Commands

### View Raw CAN Data
Add to `motor_operations.h` in `readAcceleration()`:
```cpp
Serial.print("ACC Raw: ");
for(int i=0; i<8; i++) {
    Serial.printf("%02X ", rxData[i]);
}
Serial.println();
```

### View Raw Angle Data
Add to `motor_operations.h` in `readMultiTurnAngle()`:
```cpp
Serial.print("ANG Raw: ");
for(int i=0; i<8; i++) {
    Serial.printf("%02X ", rxData[i]);
}
Serial.println();
```

---

## 📞 Support Resources

- **LK-TECH Documentation:** Check official CAN protocol specification
- **Motor Model:** MS/MF/MG/MH series
- **CAN Baud Rate:** 1 Mbps
- **Motor IDs:** ID=1 (0x141), ID=2 (0x142)

---

## 📝 Change Log

| Date | Action | Result |
|------|--------|--------|
| 2025-11-20 | Changed acceleration from INT16 to INT32 | ✅ Compiled |
| 2025-11-20 | Tried bytes [1-4] for acceleration | ❌ Worse |
| 2025-11-20 | Reverted to bytes [4-7] | ⚠️ Still 0 |
| 2025-11-20 | Uploaded fixed firmware | ⚠️ Issues persist |

---

## ⚠️ Known Limitations

1. **No Logic Analyzer Available:** Cannot verify raw CAN traffic
2. **Motor Documentation:** Official byte layout not fully confirmed
3. **Firmware Version Unknown:** Motor controller firmware version not verified
4. **Single Test Setup:** Only tested with one specific motor pair

---

**Document Status:** Living document - will be updated as investigation progresses
