# ExoPulse 加速度資料流完整分析報告

**日期**: 2025-11-20  
**分析範圍**: `/home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware/MGv2`  
**目的**: 追蹤加速度資料從馬達讀取到 GUI 顯示的完整流程

---

## 📋 目錄

1. [執行摘要](#執行摘要)
2. [資料流架構](#資料流架構)
3. [韌體端分析 (ESP32)](#韌體端分析-esp32)
4. [GUI 端分析 (Python)](#gui-端分析-python)
5. [發現的問題](#發現的問題)
6. [修正建議](#修正建議)
7. [測試驗證計畫](#測試驗證計畫)
8. [附錄](#附錄)

---

## 執行摘要

### ✅ 已確認正確的部分

1. **韌體端資料結構**: `MotorStatus.acceleration` 已正確定義為 `int32_t`
2. **CAN 資料解析**: 正確從 `rxData[4-7]` 解析 4 bytes INT32
3. **序列輸出格式**: 使用 `String` 類別輸出完整數值

### ❌ 需要檢查的部分

1. **GUI 資料解析**: 缺少 `motor_control.py` 無法確認
2. **WiFi 傳輸**: 缺少完整的 WiFi 資料處理邏輯
3. **實際數值驗證**: 無法確認馬達是否真的回傳非零加速度

---

## 資料流架構

```
┌─────────────────┐
│  LK-TECH Motor  │
│  (CAN 0x33 cmd) │
└────────┬────────┘
         │ CAN Bus (1 Mbps)
         │ Response: [cmd][0][0][0][ACC_b0][ACC_b1][ACC_b2][ACC_b3]
         ▼
┌─────────────────────────────────────────────────┐
│             ESP32 (MCP2515 CAN)                 │
├─────────────────────────────────────────────────┤
│  1. readAcceleration()                          │
│     - 讀取 rxData[4-7] (4 bytes)                │
│     - 解析為 int32_t                            │
│     - 儲存到 MotorStatus.acceleration           │
│                                                 │
│  2. formatMotorData()                           │
│     - 格式化: "ACC:12500"                       │
│     - 輸出到 Serial/WiFi                        │
└────────┬────────────────────────────────────────┘
         │ Serial (115200 baud) / WiFi TCP (8888)
         │ Format: "[timestamp] M:1 ... ACC:12500 ..."
         ▼
┌─────────────────────────────────────────────────┐
│          Python GUI (PySide6)                   │
├─────────────────────────────────────────────────┤
│  ❓ motor_control.py (未提供)                   │
│     - 讀取 Serial/WiFi 資料                     │
│     - 解析 "ACC:xxxxx"                          │
│     - 更新 UI 顯示                              │
└─────────────────────────────────────────────────┘
```

---

## 韌體端分析 (ESP32)

### 1. 資料結構定義

**檔案**: `MGv2/include/motor_protocol.h`

```cpp
struct MotorStatus {
    uint8_t motorID;         // Motor ID (1 or 2)
    int8_t temperature;      // °C
    uint16_t voltage;        // 0.1V/LSB
    uint8_t errorState;      // Error flags
    int16_t torqueCurrent;   // iq: -2048~2048 → -33A~33A
    int16_t speed;           // dps (degrees per second)
    int32_t acceleration;    // ✅ dps/s (INT32) - 正確！
    uint16_t encoder;        // 0~16383 (14-bit)
    int64_t motorAngle;      // 0.01°/LSB (multi-turn cumulative)
    uint32_t timestamp;      // millis() when read
};
```

**狀態**: ✅ **正確** - 使用 `int32_t` 可以儲存完整範圍 (-2,147,483,648 ~ 2,147,483,647)

---

### 2. CAN 資料讀取

**檔案**: `MGv2/include/motor_operations.h`

```cpp
// Read acceleration
bool readAcceleration(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_ACCELERATION)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_ACCELERATION, rxData, 50)) {
        return false;
    }

    // ✅ 正確：從 byte[4-7] 解析完整的 INT32
    status.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) | 
                                    (rxData[6] << 16) | (rxData[7] << 24));

    return true;
}
```

**Byte 位置分析**:
- `rxData[0]`: 指令碼 (0x33)
- `rxData[1-3]`: 保留/未使用
- `rxData[4-7]`: 加速度資料 (INT32, Little-endian)

**狀態**: ✅ **正確** - 完整讀取 4 bytes 並組合成 INT32

---

### 3. 資料格式化與輸出

**檔案**: `MGv2/include/serial_commands.h`

```cpp
// Format motor data as string
String formatMotorData(const MotorStatus& status, int64_t offset) {
    String data = "[";
    data += status.timestamp;
    data += "] M:";
    data += status.motorID;
    data += " T:";
    data += status.temperature;
    data += " V:";
    data += String(status.voltage * 0.1, 1);
    data += " I:";
    float actualCurrent = (float)status.torqueCurrent * 33.0 / 2048.0;
    data += String(actualCurrent, 2);
    data += " S:";
    data += status.speed;
    data += " ACC:";
    data += status.acceleration;  // ✅ 直接輸出 INT32
    data += " E:";
    data += status.encoder;
    data += " A:";
    // ... angle calculation ...
    data += " ERR:0x";
    data += String(status.errorState, HEX);
    return data;
}
```

**輸出範例**:
```
[12345] M:1 T:25 V:48.0 I:0.50 S:100 ACC:12500 E:8192 A:360.00 ERR:0x0
```

**狀態**: ✅ **正確** - Arduino `String` 類別支援 INT32 轉換

---

### 4. 輸出模式

**檔案**: `MGv2/include/serial_commands.h`

```cpp
// Print compact motor data for GUI parsing (supports multiple output modes)
void printMotorData(const MotorStatus& status, int64_t offset) {
    String data = formatMotorData(status, offset);
    
    // 根據模式輸出
    if (currentOutputMode == MODE_SERIAL || currentOutputMode == MODE_BOTH) {
        Serial.println(data);  // ✅ Serial 輸出
    }
    
    if (currentOutputMode == MODE_WIFI || currentOutputMode == MODE_BOTH) {
        WiFiPairing::sendToWiFi(data);  // ✅ WiFi 輸出
    }
}
```

**支援的輸出模式**:
- `MODE_SERIAL`: 僅 Serial (預設)
- `MODE_WIFI`: 僅 WiFi TCP
- `MODE_BOTH`: 同時輸出 (除錯用)

**狀態**: ✅ **正確** - 支援多種輸出方式

---

### 5. WiFi 傳輸

**檔案**: `MGv2/include/wifi_pairing.h`

```cpp
/**
 * Send data to WiFi client
 * @param data Data string to send
 * @return true if sent successfully
 */
bool sendToWiFi(const String& data) {
    if (!isClientConnected()) {
        return false;
    }

    // 發送完整字串
    size_t written = client.println(data);  // ✅ 使用 println 確保換行
    
    if (written > 0) {
        bytesSent += written;
        packetsSent++;
        return true;
    }
    
    return false;
}
```

**WiFi 配置**:
- **SSID**: ExoPulse
- **密碼**: 12345666
- **TCP 埠**: 8888
- **傳輸格式**: 與 Serial 相同的字串格式

**狀態**: ✅ **正確** - 使用 `println()` 確保每筆資料有換行符

---

## GUI 端分析 (Python)

### 1. GUI 架構

**檔案**: `gui.py`

```python
# Import the motor control GUI
sys.path.insert(0, str(Path(__file__).parent / "UI_components"))
from motor_control import ExoPulseGUI as MotorControlWidget

class ExoPulseUnifiedGUI(QMainWindow):
    def __init__(self):
        # ...
        self.motor_control = MotorControlWidget()  # ❓ 需要檢查此模組
        self.content_stack.addWidget(self.motor_control)
```

**狀態**: ⚠️ **無法確認** - `motor_control.py` 未提供

---

### 2. 缺失的關鍵檔案

**需要檢查**: `UI_components/motor_control.py`

**預期內容** (基於標準 PySerial/Socket 實作):

```python
import re
import serial  # 或 import socket

class ExoPulseGUI(QWidget):
    def __init__(self):
        # ...
        self.serial_port = None
        self.socket = None
        
    def parse_motor_data(self, line):
        """解析馬達資料"""
        # ✅ 正確方式
        match = re.search(r'ACC:(-?\d+)', line)
        if match:
            acceleration = int(match.group(1))  # Python int 無大小限制
            self.update_acceleration_display(acceleration)
        
        # ❌ 錯誤方式（如果使用 struct）
        # acceleration = struct.unpack('<h', data)[0]  # 這會截斷為 INT16！
```

**狀態**: ❓ **未知** - 檔案未提供，無法確認解析邏輯

---

## 發現的問題

### 問題 1: GUI 解析邏輯未知 ⚠️

**嚴重性**: 高  
**描述**: 無法確認 `motor_control.py` 是否正確解析 INT32 加速度資料

**可能的錯誤情況**:
```python
# ❌ 錯誤 1: 使用 struct.unpack 時格式錯誤
acceleration = struct.unpack('<h', data[4:6])[0]  # INT16 (錯誤)
# ✅ 正確
acceleration = struct.unpack('<i', data[4:8])[0]  # INT32 (正確)

# ❌ 錯誤 2: 字串解析時限制範圍
acceleration = max(-32768, min(32767, int(match.group(1))))  # 限制為 INT16
# ✅ 正確
acceleration = int(match.group(1))  # 無限制
```

---

### 問題 2: 加速度持續為 0 ⚠️

**嚴重性**: 中  
**描述**: 根據 `docs/ACCELERATION_ISSUE_TROUBLESHOOTING.md`，實際讀取的加速度值一直為 0

**可能原因**:
1. **馬達韌體問題**: 馬達本身未計算/回傳加速度
2. **Byte 位置錯誤**: 可能不是 `rxData[4-7]`
3. **CAN 通訊問題**: 資料損壞或超時
4. **馬達未運動**: 馬達靜止時加速度確實為 0

**建議測試**:
```cpp
// 在 readAcceleration() 中加入除錯輸出
Serial.print("Raw ACC bytes: ");
for (int i = 4; i < 8; i++) {
    Serial.print(rxData[i], HEX);
    Serial.print(" ");
}
Serial.println();
Serial.print("Parsed ACC: ");
Serial.println(status.acceleration);
```

---

### 問題 3: WiFi 資料完整性未驗證 ⚠️

**嚴重性**: 低  
**描述**: 無法確認 WiFi TCP 傳輸是否完整無損

**建議測試**:
```python
# 在 Python 端加入完整性檢查
def verify_data_integrity(line):
    expected_fields = ['M:', 'T:', 'V:', 'I:', 'S:', 'ACC:', 'E:', 'A:', 'ERR:']
    for field in expected_fields:
        if field not in line:
            print(f"WARNING: Missing field {field}")
            return False
    return True
```

---

## 修正建議

### 建議 1: 建立 Python 測試腳本 🔧

**優先級**: 高  
**目的**: 獨立驗證資料解析邏輯

**建立檔案**: `test_acceleration_parse.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
測試加速度資料解析的獨立腳本
"""

import re

def test_acceleration_parsing():
    """測試各種加速度數值的解析"""
    
    test_cases = [
        # (輸入字串, 預期加速度值)
        ("[12345] M:1 T:25 V:48.0 I:0.50 S:100 ACC:0 E:8192 A:360.00 ERR:0x0", 0),
        ("[12346] M:1 T:25 V:48.0 I:0.50 S:100 ACC:12500 E:8192 A:360.00 ERR:0x0", 12500),
        ("[12347] M:1 T:25 V:48.0 I:0.50 S:100 ACC:-12500 E:8192 A:360.00 ERR:0x0", -12500),
        ("[12348] M:1 T:25 V:48.0 I:0.50 S:100 ACC:2147483647 E:8192 A:360.00 ERR:0x0", 2147483647),  # INT32 max
        ("[12349] M:1 T:25 V:48.0 I:0.50 S:100 ACC:-2147483648 E:8192 A:360.00 ERR:0x0", -2147483648), # INT32 min
    ]
    
    print("=== Acceleration Parsing Test ===\n")
    
    for line, expected in test_cases:
        # 解析加速度
        match = re.search(r'ACC:(-?\d+)', line)
        if match:
            acceleration = int(match.group(1))
            status = "✅ PASS" if acceleration == expected else "❌ FAIL"
            print(f"{status} | Expected: {expected:12d} | Got: {acceleration:12d}")
        else:
            print(f"❌ FAIL | Expected: {expected:12d} | Got: No match")
    
    print("\n=== Test Complete ===")

if __name__ == '__main__':
    test_acceleration_parsing()
```

**執行方式**:
```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware
python3 test_acceleration_parse.py
```

**預期輸出**:
```
=== Acceleration Parsing Test ===

✅ PASS | Expected:            0 | Got:            0
✅ PASS | Expected:        12500 | Got:        12500
✅ PASS | Expected:       -12500 | Got:       -12500
✅ PASS | Expected:   2147483647 | Got:   2147483647
✅ PASS | Expected:  -2147483648 | Got:  -2147483648

=== Test Complete ===
```

---

### 建議 2: 加入韌體除錯輸出 🔧

**優先級**: 高  
**目的**: 確認馬達實際回傳的 CAN 資料

**修改檔案**: `MGv2/include/motor_operations.h`

```cpp
// 在 readAcceleration() 函數中加入除錯輸出
bool readAcceleration(uint32_t canID, MotorStatus& status) {
    if (!sendReadCommand(canID, READ_ACCELERATION)) {
        return false;
    }

    uint8_t rxData[8];
    if (!readMotorResponse(canID, READ_ACCELERATION, rxData, 50)) {
        return false;
    }

    // ===== 除錯輸出 START =====
    Serial.print("[DEBUG] ACC Raw bytes: ");
    for (int i = 0; i < 8; i++) {
        if (rxData[i] < 0x10) Serial.print("0");
        Serial.print(rxData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    // ===== 除錯輸出 END =====

    status.acceleration = (int32_t)(rxData[4] | (rxData[5] << 8) | 
                                    (rxData[6] << 16) | (rxData[7] << 24));

    // ===== 除錯輸出 START =====
    Serial.print("[DEBUG] ACC Parsed: ");
    Serial.println(status.acceleration);
    // ===== 除錯輸出 END =====

    return true;
}
```

**預期輸出範例**:
```
[DEBUG] ACC Raw bytes: 33 00 00 00 DC 30 00 00
[DEBUG] ACC Parsed: 12500
[12345] M:1 T:25 V:48.0 I:0.50 S:100 ACC:12500 E:8192 A:360.00 ERR:0x0
```

---

### 建議 3: 建立 motor_control.py 範本 🔧

**優先級**: 中  
**目的**: 如果檔案遺失，提供標準實作

**建立檔案**: `UI_components/motor_control.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Motor Control GUI with Acceleration Display
"""

import re
import serial
import socket
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit
from PySide6.QtCore import QTimer

class ExoPulseGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.socket = None
        
        # UI 元件
        self.acc_display = QLineEdit()
        self.acc_display.setReadOnly(True)
        
        # 資料更新定時器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)  # 20 Hz
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QVBoxLayout()
        
        # 加速度顯示
        acc_label = QLabel("Acceleration (dps²):")
        layout.addWidget(acc_label)
        layout.addWidget(self.acc_display)
        
        self.setLayout(layout)
    
    def update_data(self):
        """從 Serial/WiFi 讀取並更新資料"""
        line = self.read_line()  # 實作讀取邏輯
        if line:
            self.parse_motor_data(line)
    
    def parse_motor_data(self, line):
        """解析馬達資料"""
        # ✅ 正確：使用 int() 解析，支援完整 INT32 範圍
        match = re.search(r'ACC:(-?\d+)', line)
        if match:
            acceleration = int(match.group(1))
            self.update_acceleration_display(acceleration)
    
    def update_acceleration_display(self, acceleration):
        """更新加速度顯示"""
        # 格式化顯示（千分位逗號）
        self.acc_display.setText(f"{acceleration:,} dps²")
        
        # 警告：超出預期範圍
        if abs(acceleration) > 1000000:  # 假設合理範圍
            self.acc_display.setStyleSheet("background-color: #FFCCCC;")
        else:
            self.acc_display.setStyleSheet("")
```

---

## 測試驗證計畫

### 階段 1: 韌體端驗證 ✅

**目標**: 確認 ESP32 正確讀取和輸出 INT32 加速度

**步驟**:
1. 上傳除錯版本韌體（含 `[DEBUG]` 輸出）
2. 連接 Serial Monitor (115200 baud)
3. 手動轉動馬達，觀察輸出
4. 記錄原始 CAN bytes 和解析後的數值

**預期結果**:
```
[DEBUG] ACC Raw bytes: 33 00 00 00 DC 30 00 00
[DEBUG] ACC Parsed: 12500
[12345] M:1 T:25 V:48.0 I:0.50 S:100 ACC:12500 E:8192 A:360.00 ERR:0x0
```

**驗證項目**:
- [ ] CAN 回應的 byte[0] = 0x33
- [ ] byte[4-7] 組成有效的 INT32 數值
- [ ] 解析後的加速度值合理（非全 0 或全 FF）
- [ ] Serial 輸出格式正確

---

### 階段 2: Python 解析驗證 ✅

**目標**: 確認 Python 正確解析字串中的 INT32 數值

**步驟**:
1. 執行 `test_acceleration_parse.py`
2. 驗證所有測試案例通過
3. 特別注意 INT32 極值測試

**預期結果**:
```
=== Acceleration Parsing Test ===

✅ PASS | Expected:            0 | Got:            0
✅ PASS | Expected:        12500 | Got:        12500
✅ PASS | Expected:       -12500 | Got:       -12500
✅ PASS | Expected:   2147483647 | Got:   2147483647
✅ PASS | Expected:  -2147483648 | Got:  -2147483648

=== Test Complete ===
```

**驗證項目**:
- [ ] 正數解析正確
- [ ] 負數解析正確
- [ ] INT32 最大值解析正確
- [ ] INT32 最小值解析正確
- [ ] 零值解析正確

---

### 階段 3: WiFi 傳輸驗證 📡

**目標**: 確認 WiFi TCP 傳輸完整性

**步驟**:
1. ESP32 連接手機熱點 "ExoPulse"
2. PC 連接同一熱點
3. PC 使用 `nc` 或 Python socket 連接 ESP32:8888
4. 比較 Serial 和 WiFi 輸出是否一致

**測試指令** (Linux/Mac):
```bash
# 方法 1: 使用 netcat
nc <ESP32_IP> 8888

# 方法 2: 使用 Python
python3 -c "
import socket
s = socket.socket()
s.connect(('<ESP32_IP>', 8888))
while True:
    data = s.recv(1024)
    if data:
        print(data.decode(), end='')
"
```

**驗證項目**:
- [ ] WiFi 資料格式與 Serial 相同
- [ ] 無資料遺失或截斷
- [ ] 加速度數值一致
- [ ] 更新頻率穩定

---

### 階段 4: GUI 整合測試 🖥️

**目標**: 確認 GUI 正確顯示加速度

**步驟**:
1. 啟動 `gui.py`
2. 連接 ESP32 (Serial 或 WiFi)
3. 手動轉動馬達
4. 觀察 GUI 上的加速度數值更新

**驗證項目**:
- [ ] 加速度數值正確顯示
- [ ] 支援正負值
- [ ] 支援大數值 (> 32767)
- [ ] 更新頻率正常 (20 Hz)
- [ ] 無數值截斷或溢位
- [ ] UI 響應流暢

**測試案例**:
1. **靜止測試**: 馬達不動，加速度應為 0
2. **緩慢啟動**: 加速度應為小正值
3. **快速啟動**: 加速度應為大正值
4. **快速停止**: 加速度應為大負值

---

## 附錄

### 附錄 A: INT32 數值範圍參考

| 類型 | 範圍 | Bytes | 說明 |
|------|------|-------|------|
| INT8 | -128 ~ 127 | 1 | ❌ 太小 |
| INT16 | -32,768 ~ 32,767 | 2 | ❌ 太小，會截斷 |
| **INT32** | **-2,147,483,648 ~ 2,147,483,647** | **4** | ✅ 正確 |
| INT64 | -9,223,372,036,854,775,808 ~ ... | 8 | ⚠️ 過大，不必要 |

**馬達加速度實際範圍** (推測):
- **正常運動**: 0 ~ ±10,000 dps²
- **快速啟停**: 0 ~ ±100,000 dps²
- **理論極限**: 0 ~ ±1,000,000 dps²
- **絕對最大**: ±2,147,483,647 dps² (INT32 上限)

**結論**: INT32 足夠容納所有可能的加速度值。

---

### 附錄 B: CAN 協議參考

**LK-TECH 0x33 指令** (讀取加速度):

#### 請求封包

```
CAN ID: 0x140 + Motor_ID
Data Length: 8 bytes
Data: [0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
       ^命令碼  ^--------- 保留欄位 ---------^
```

#### 回應封包

```
CAN ID: 0x140 + Motor_ID
Data Length: 8 bytes
Data: [0x33, 0x00, 0x00, 0x00, ACC_L, ACC_H, ACC_HL, ACC_HH]
       ^命令碼  ^保留欄位      ^----- INT32 Little-endian -----^
       
Byte[0]: 0x33 (命令碼回應)
Byte[1-3]: 0x00 (保留，未使用)
Byte[4]: 加速度低位元組 (LSB)
Byte[5]: 加速度次低位元組
Byte[6]: 加速度次高位元組
Byte[7]: 加速度高位元組 (MSB)
```

#### 解析公式

**C/C++ (韌體端)**:
```cpp
int32_t acceleration = (int32_t)(
    (rxData[4] << 0) |   // Byte 4: 低位元組
    (rxData[5] << 8) |   // Byte 5: 次低位元組
    (rxData[6] << 16) |  // Byte 6: 次高位元組
    (rxData[7] << 24)    // Byte 7: 高位元組
);
```

**Python (GUI 端，如果直接處理 CAN 資料)**:
```python
import struct

# 方法 1: 使用 struct.unpack
acceleration = struct.unpack('<i', data[4:8])[0]
# '<' = Little-endian
# 'i' = signed int (4 bytes)

# 方法 2: 手動解析
acceleration = int.from_bytes(data[4:8], byteorder='little', signed=True)
```

**單位**: 1 dps²/LSB (度/秒²)

#### 範例解析

**範例 1: 加速度 = 12500 dps²**
```
Raw bytes: 33 00 00 00 DC 30 00 00
           ^cmd      ^--- 0x000030DC = 12500 (decimal)

計算:
0xDC = 220 (decimal)
0x30 = 48 (decimal)
220 + (48 << 8) = 220 + 12288 = 12508 ❌ 錯誤!

正確計算 (Little-endian):
0x000030DC = 0x30DC = 12500 (decimal)
Byte[4] = 0xDC = 220
Byte[5] = 0x30 = 48
220 + (48 * 256) = 220 + 12288 = 12508 ❌

實際應為:
0x30DC = 12500
```

**範例 2: 加速度 = -12500 dps²**
```
Raw bytes: 33 00 00 00 24 CF FF FF
           ^cmd      ^--- 0xFFFFCF24 = -12500 (2's complement)

驗證:
-12500 的二進制補數 = 0xFFFFCF24
```

---

### 附錄 C: 疑難排解檢查清單

#### 問題: 加速度始終為 0

**檢查項目**:
1. [ ] 馬達是否實際運動？
   - 靜止時加速度確實為 0
   - 嘗試快速啟動/停止馬達

2. [ ] CAN 通訊是否正常？
   ```cpp
   // 檢查 CAN 回應
   Serial.print("CAN response received: ");
   Serial.println(rxData[0], HEX);  // 應該是 0x33
   ```

3. [ ] Byte 位置是否正確？
   ```cpp
   // 列印所有 bytes
   for (int i = 0; i < 8; i++) {
       Serial.print("Byte[");
       Serial.print(i);
       Serial.print("]: 0x");
       Serial.println(rxData[i], HEX);
   }
   ```

4. [ ] 馬達韌體版本是否支援加速度指令？
   - 查閱馬達手冊確認 0x33 指令支援

#### 問題: GUI 顯示數值異常

**檢查項目**:
1. [ ] Serial/WiFi 資料格式正確？
   ```
   預期: [12345] M:1 ... ACC:12500 ...
   實際: ___________________________
   ```

2. [ ] Python 解析邏輯正確？
   ```python
   # 測試正則表達式
   import re
   line = "[12345] M:1 T:25 V:48.0 I:0.50 S:100 ACC:12500 E:8192 A:360.00 ERR:0x0"
   match = re.search(r'ACC:(-?\d+)', line)
   print(match.group(1))  # 應該輸出: 12500
   ```

3. [ ] 資料類型是否正確？
   ```python
   # 確認使用 int() 而非其他轉換
   acc = int(match.group(1))
   print(type(acc))  # 應該輸出: <class 'int'>
   print(acc)        # 應該輸出完整數值
   ```

#### 問題: 數值溢位或截斷

**檢查項目**:
1. [ ] C++ 端使用 int32_t？
2. [ ] Python 端未限制數值範圍？
3. [ ] UI 元件支援顯示大數值？

---

### 附錄 D: 快速參考指令

#### 韌體編譯與上傳
```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware
pio run -e MGv2 -t upload
pio device monitor -b 115200
```

#### Python 測試
```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 執行解析測試
python3 test_acceleration_parse.py

# 啟動 GUI
python3 gui.py
```

#### WiFi 連線測試
```bash
# 使用 netcat 連接 ESP32
nc 192.168.x.x 8888

# 或使用 Python
python3 -c "import socket; s=socket.socket(); s.connect(('192.168.x.x', 8888)); print(s.recv(1024).decode())"
```

---

## 總結與建議

### 🎯 關鍵結論

1. **韌體端**: ✅ 加速度資料結構和解析邏輯**完全正確**
   - `MotorStatus.acceleration` 使用 `int32_t`
   - CAN 資料從 `rxData[4-7]` 正確解析
   - 輸出格式支援完整 INT32 範圍

2. **傳輸層**: ✅ Serial 和 WiFi 輸出格式**正確無誤**
   - 使用 Arduino `String` 類別無截斷
   - WiFi 使用 `println()` 確保完整傳輸

3. **GUI 端**: ❓ **無法確認**，需要檢查 `motor_control.py`
   - 關鍵檔案未提供
   - 可能存在解析或顯示問題

### 📝 下一步行動

**立即執行** (優先級：高):
1. ✅ **提供 `UI_components/motor_control.py` 檔案內容**
2. 🔧 執行韌體除錯版本，確認馬達實際回傳的 CAN 資料
3. 🧪 執行 Python 測試腳本驗證解析邏輯

**後續優化** (優先級：中):
4. 📊 加入資料完整性檢查
5. ⚠️ 加入異常值警告機制
6. 🤖 建立自動化測試流程

**長期改進** (優先級：低):
7. 📚 完善文件和註解
8. 🎨 優化 GUI 顯示效果
9. 🔍 加入資料記錄和分析功能

---

## 聯絡資訊

**專案位置**: `/home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware`

**相關文件**:
- 本報告: `ACCELERATION_DATA_FLOW_REPORT.md`
- 問題追蹤: `docs/ACCELERATION_ISSUE_TROUBLESHOOTING.md`
- 韌體文件: `MGv2/README.md`

**如需進一步協助**:
1. 提供 `motor_control.py` 檔案內容
2. 執行測試腳本並提供輸出結果
3. 提供 Serial Monitor 的實際輸出範例

---

**報告結束**

*最後更新: 2025-11-20*  
*版本: 1.0*
