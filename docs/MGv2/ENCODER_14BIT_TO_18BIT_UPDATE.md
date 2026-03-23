# Encoder 14-bit → 18-bit 更新計畫

## 背景

根據 LK-TECH MG 系列馬達協議文件，MG 系列馬達使用 **18-bit 編碼器**，範圍為 `0 ~ 262143`。

目前程式碼中多處使用 **14-bit (0~16383)** 作為編碼器範圍，需要修正為 18-bit。

---

## 需要修改的檔案與位置

### 1. include/motor_protocol.h - Line 63
```cpp
// 現在: uint16_t encoder;        // 0~16383 (14-bit)
// 改成: uint16_t encoder;        // 0~262143 (18-bit)
```

### 2. include/motor_control.h - Line 20
```cpp
// 現在: uint16_t encoder;        // Encoder value (0~16383)
// 改成: uint16_t encoder;        // Encoder value (0~262143)
```

### 3. include/calibration.h - Line 123, 127, 128-129

#### Line 123
```cpp
// 現在: // encoderOffset: uint16_t, range 0~16383 (14-bit encoder)
// 改成: // encoderOffset: uint16_t, range 0~262143 (18-bit encoder)
```

#### Line 127
```cpp
// 現在: // Clamp to valid 14-bit range
// 改成: // Clamp to valid 18-bit range
```

#### Line 128-129
```cpp
// 現在: if (encoderOffset > 16383) { encoderOffset = 16383; }
// 改成: if (encoderOffset > 262143) { encoderOffset = 262143; }
```

### 4. include/serial_commands.h - Line 114, 164, 165

#### Line 114
```cpp
// 現在: Serial.println(" (0~16383)");
// 改成: Serial.println(" (0~262143)");
```

#### Line 164
```cpp
// 現在: Serial.println("SET_OFFSET_M1:<val>   - Write encoder offset to Motor 1 ROM (0x91, 0~16383)");
// 改成: Serial.println("SET_OFFSET_M1:<val>   - Write encoder offset to Motor 1 ROM (0x91, 0~262143)");
```

#### Line 165
```cpp
// 現在: Serial.println("SET_OFFSET_M2:<val>   - Write encoder offset to Motor 2 ROM (0x91, 0~16383)");
// 改成: Serial.println("SET_OFFSET_M2:<val>   - Write encoder offset to Motor 2 ROM (0x91, 0~262143)");
```

### 5. test/hardware_validation/test_dual_motor_rtos.cpp - Line 57, 379

#### Line 57
```cpp
// 現在: uint16_t encoder;        // 0~16383 (14-bit)
// 改成: uint16_t encoder;        // 0~262143 (18-bit)
```

#### Line 379
```cpp
// 現在: Serial.println(" (0~16383)");
// 改成: Serial.println(" (0~262143)");
```

---

## 修改摘要

| 檔案 | 行號 | 類型 | 修改內容 |
|------|------|------|----------|
| motor_protocol.h | 63 | 註解 | 14-bit → 18-bit |
| motor_control.h | 20 | 註解 | 0~16383 → 0~262143 |
| calibration.h | 123 | 註解 | 14-bit → 18-bit |
| calibration.h | 127 | 註解 | 14-bit → 18-bit |
| calibration.h | 128-129 | 常數 | 16383 → 262143 |
| serial_commands.h | 114 | 字串 | 0~16383 → 0~262143 |
| serial_commands.h | 164 | 字串 | 0~16383 → 0~262143 |
| serial_commands.h | 165 | 字串 | 0~16383 → 0~262143 |
| test_dual_motor_rtos.cpp | 57 | 註解 | 14-bit → 18-bit |
| test_dual_motor_rtos.cpp | 379 | 字串 | 0~16383 → 0~262143 |

**總計: 5 個檔案, 10 處修改**
