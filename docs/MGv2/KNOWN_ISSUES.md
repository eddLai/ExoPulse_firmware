# Known Issues - ExoPulse Motor Control

## Motor Angle Reset Command (0x95) Not Implemented in Motor Firmware

**Date Discovered:** 2025-11-20
**Severity:** High
**Status:** Confirmed - Motor firmware limitation
**Affects:** LK-TECH M Series Motors

### Summary

The CAN bus command 0x95 (Clear Motor Angle) is documented in the motor datasheet but is **not fully implemented** in the motor's internal firmware. While motors acknowledge the command, they do not actually reset their multi-turn angle counter to zero.

### Technical Details

**Command:** 0x95 - Clear Motor Angle Command
**Expected Behavior:** Clears multi-turn and single-turn angle data, sets current position as zero
**Actual Behavior:** Motor sends ACK response but angle value remains unchanged

**Datasheet Quote (Chinese):**
```
(11) 清除电机角度命令（1 帧）暂未实现
该命令清除电机的多圈和单圈角度数据，并将当前位置设为电机的零点，断电后失效
```

**Translation:** "(11) Clear Motor Angle Command (1 frame) - NOT YET IMPLEMENTED"

### Evidence

Test results from `test_reset_verbose.py` show:

```
>>> Sent: RESET_M2
>>> [CMD] Resetting Motor 2 angle to zero...
>>> [OK] Motor 2 angle reset successful!

Motor 2 angle BEFORE reset: 630.05°
Motor 2 angle AFTER reset:  630.06° (unchanged)
```

**Findings:**
- ✅ ESP32 firmware correctly sends 0x95 command via CAN bus
- ✅ Motor responds with ACK (acknowledges command)
- ❌ Motor angle value does not change

### Impact

1. **Motor 1 Angle Overflow Issue:**
   - Motor 1 shows angle as "ovf" (overflow)
   - Cannot be reset via software command 0x95
   - Affects angle and acceleration display in GUI (shows as 0)

2. **Workarounds Implemented:**
   - GUI converts "ovf" to 0.0 to prevent crashes
   - Thread-safe serial communication added
   - Auto-reset attempt on firmware/GUI startup (gracefully fails)
   - Regex parser handles both numeric angles and "ovf" string

3. **Data Still Available:**
   - Temperature: ✅ Working
   - Voltage: ✅ Working
   - Current: ✅ Working
   - Speed: ✅ Working
   - Acceleration: ✅ Working (reads 0 when motor stationary)
   - Encoder: ✅ Working
   - Multi-turn Angle: ❌ Motor 1 shows overflow, Motor 2 works

### Attempted Solutions

1. **Software reset via 0x95** - Failed (motor firmware limitation)
2. **Auto-reset on MCU startup** - Implemented but ineffective
3. **Auto-reset on GUI launch** - Implemented but ineffective
4. **Multiple reset attempts** - No improvement

### Recommendations

#### Short-term:
1. **Accept limitation** - Use Motor 2 angle data, ignore Motor 1 angle
2. **Hardware reset** - Power cycle Motor 1 to attempt clearing overflow
3. **Use single-turn encoder** - Read command 0x94 instead of 0x92 (within 0-360°)

#### Long-term:
1. **Contact manufacturer** (LK-TECH) to request:
   - Motor firmware update implementing 0x95
   - Alternative reset method if available
   - Explanation of overflow condition
2. **Consider alternative motors** with proper angle reset support

### Files Modified

**Firmware:**
- `test/hardware_validation/test_dual_motor_rtos.cpp` - Added auto-reset on startup, overflow detection

**GUI:**
- `test/hardware_validation/monitor_dual_motor_enhanced.py` - Thread-safe serial, "ovf" handling, auto-reset
- `test/hardware_validation/monitor_dual_motor_optimized.py` - "ovf" handling in regex

**Test Scripts:**
- `test/hardware_validation/test_reset_proof.py` - Proves reset command doesn't work
- `test/hardware_validation/test_reset_verbose.py` - Shows motor ACKs but doesn't reset
- `test/hardware_validation/debug_serial.py` - Debugging tool for serial output

### Related Code

**Overflow detection** (`test_dual_motor_rtos.cpp:327-333`):
```cpp
// Check for overflow: valid range is roughly ±2^53 (float precision limit)
if (status.motorAngle > 9007199254740992LL || status.motorAngle < -9007199254740992LL) {
    Serial.print("ovf");
} else {
    float angleDeg = (float)status.motorAngle * 0.01;
    Serial.print(angleDeg, 2);
}
```

**GUI "ovf" handling** (`monitor_dual_motor_enhanced.py:67-78`):
```python
match = re.match(r'\[(\d+)\] M:(\d+) ... A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
if match:
    angle_str = match.group(9)
    angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
```

### References

- Motor Datasheet: `20230220145958f_datasheet_protocol.pdf` page 8
- CAN Protocol: Command 0x92 (Read Multi-turn Angle), 0x95 (Clear Angle)
- Test Results: See `test_reset_proof.py` and `test_reset_verbose.py` output

---

**Last Updated:** 2025-11-20
**Tested By:** Development Team
**Motor Model:** LK-TECH M Series (CAN bus version)
