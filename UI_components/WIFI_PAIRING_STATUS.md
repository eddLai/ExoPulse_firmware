# WiFi 配對狀態顯示功能

## 功能概述

在 `wifi_monitor.py` 中添加了 WiFi 配對狀態欄，用於顯示 ESP32 和 PC 的 WiFi 連接狀態，並在兩者連接到同一 WiFi 網絡時提示可以安全斷開 USB 連接。

## 使用流程

### 1. 有線連接 ESP32

```bash
# 通過 USB 連接 ESP32 並監控串口輸出
cd ExoPulse_firmware/MGv2
pio device monitor -p /dev/ttyACM0
```

或使用 Python 直接讀取：

```bash
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
while True:
    line = ser.readline().decode('utf-8', errors='ignore')
    if line:
        print(line.strip())
"
```

### 2. 確認 ESP32 WiFi 配置

從串口輸出中查找以下信息：

```
[WiFi] Connecting to: ExoPulse
[WiFi] Connected!
  IP Address: 10.154.48.200
  MAC Address: XX:XX:XX:XX:XX:XX
  Signal Strength (RSSI): -45 dBm
[WiFi] TCP Server started on port 8888
```

**重要信息：**
- **SSID**: WiFi 網絡名稱（例如：ExoPulse）
- **IP 地址**: ESP32 的 IP（例如：10.154.48.200）
- **端口**: TCP 服務器端口（默認：8888）

### 3. PC 連接到相同 WiFi

確保 PC 連接到與 ESP32 相同的 WiFi 網絡：

```bash
# 檢查當前 PC WiFi 連接
iwconfig

# 或使用
nmcli device wifi list
```

**輸出示例：**
```
wlp3s0    IEEE 802.11  ESSID:"ExoPulse"
          Mode:Managed  Frequency:2.437 GHz
          Signal level=-42 dBm
```

### 4. 運行 WiFi Monitor

使用從串口獲取的 IP 地址運行監控程序：

```bash
cd ExoPulse_firmware/UI_components
python3 wifi_monitor.py <ESP32_IP> 8888
```

**示例：**
```bash
python3 wifi_monitor.py 10.154.48.200 8888
```

### 5. 觀察配對狀態

GUI 會顯示三個狀態欄：

#### 狀態 1: 連接狀態
```
Status: Connected | WiFi: MCU=-45dBm | PC=-42dBm
```

#### 狀態 2: WiFi 配對狀態（重點）

| 圖標 | 顏色 | 訊息 | 說明 |
|------|------|------|------|
| ✓ | <span style="color:lime">**綠色**</span> | **WiFi Matched - Safe to disconnect USB cable** | ESP32 和 PC 在同一 WiFi，可以安全拔掉 USB |
| ⚠ | <span style="color:yellow">**黃色**</span> | **WiFi Mismatch - ESP32: xxx \| PC: yyy** | ESP32 和 PC 在不同 WiFi，需要調整 |
| ⊗ | <span style="color:orange">**橙色**</span> | **Waiting for ESP32 WiFi connection...** | ESP32 尚未連接到 WiFi |

#### 狀態 3: WiFi 詳細信息
```
ESP32: SSID=ExoPulse | IP=10.154.48.200
PC: SSID=ExoPulse | IP=10.154.48.123
```

### 6. 安全斷開 USB

當看到 **綠色 ✓** 圖標和 **"WiFi Matched - Safe to disconnect USB cable"** 訊息時：

1. **確認配對成功**：ESP32 SSID 和 PC SSID 一致
2. **確認通訊正常**：Motor 1 和 Motor 2 數據正常更新
3. **安全拔掉 USB**：數據將通過 WiFi 繼續傳輸

---

## 技術實現

### 新增的功能

#### 1. WiFi 狀態追蹤

```python
# WiFi pairing status tracking
self.esp32_wifi_ssid = "Unknown"
self.esp32_wifi_ip = "Unknown"
self.esp32_connected_to_wifi = False
self.pc_wifi_ssid = "Unknown"
self.pc_wifi_ip = "Unknown"
self.wifi_pairing_matched = False
```

#### 2. PC WiFi 信息獲取

```python
def get_pc_wifi_details(self):
    """Get PC WiFi connection details (SSID and IP)"""
    # 從 iwconfig 獲取 SSID
    # 從 hostname -I 獲取 IP 地址
```

#### 3. ESP32 WiFi 信息解析

從 TCP 數據流中解析 ESP32 的 WiFi 信息：

```python
# Check for ESP32 WiFi connection info
if 'WiFi Connected' in line or 'Connected to' in line:
    self.esp32_connected_to_wifi = True
if 'SSID:' in line:
    match = re.search(r'SSID:\s*(\S+)', line)
    if match:
        self.esp32_wifi_ssid = match.group(1)
if 'IP Address:' in line or 'IP:' in line:
    match = re.search(r'IP(?:\s+Address)?:\s*([\d.]+)', line)
    if match:
        self.esp32_wifi_ip = match.group(1)
```

#### 4. 配對狀態判斷

```python
# Check if WiFi pairing matched
if self.esp32_wifi_ssid != "Unknown" and self.pc_wifi_ssid != "Unknown":
    self.wifi_pairing_matched = (self.esp32_wifi_ssid == self.pc_wifi_ssid)
```

#### 5. GUI 狀態欄顯示

```python
# WiFi pairing status
if self.wifi_pairing_matched:
    pairing_color = 'lime'
    pairing_icon = '✓'
    pairing_msg = 'WiFi Matched - Safe to disconnect USB cable'
elif self.esp32_connected_to_wifi and self.esp32_wifi_ssid != "Unknown":
    pairing_color = 'yellow'
    pairing_icon = '⚠'
    pairing_msg = f'WiFi Mismatch - ESP32: {self.esp32_wifi_ssid} | PC: {self.pc_wifi_ssid}'
else:
    pairing_color = 'orange'
    pairing_icon = '⊗'
    pairing_msg = 'Waiting for ESP32 WiFi connection...'

self.pairing_text.set_text(f'{pairing_icon} {pairing_msg}',
                           color=pairing_color, fontweight='bold')
```

---

## 測試方法

### 方法 1: 使用真實硬件

```bash
# Terminal 1: 監控 ESP32 串口
cd ExoPulse_firmware/MGv2
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
while True:
    print(ser.readline().decode('utf-8', errors='ignore').strip())
"

# Terminal 2: 運行 WiFi Monitor
cd ExoPulse_firmware/UI_components
python3 wifi_monitor.py 10.154.48.200 8888
```

### 方法 2: 使用模擬服務器（無需硬件）

```bash
# Terminal 1: 啟動模擬 ESP32 服務器
cd ExoPulse/depRL/deploy_IoT
python3 test_wifi_monitor_standalone.py

# Terminal 2: 運行 WiFi Monitor
cd ExoPulse_firmware/UI_components
python3 wifi_monitor.py 127.0.0.1 8888
```

### 測試腳本

運行測試腳本查看完整的使用指南：

```bash
cd ExoPulse/depRL/deploy_IoT
python3 test_wifi_pairing_status.py
```

---

## 故障排除

### 問題 1: 無法連接到 ESP32

**症狀：**
```
⚚ Connecting to 10.154.48.200:8888...
✗ Connection failed: timed out
```

**解決方案：**

1. **檢查 ESP32 電源**
   ```bash
   # 確認 ESP32 已上電
   ls -la /dev/ttyACM*
   ```

2. **檢查 ESP32 WiFi 連接**
   ```bash
   # 監控串口確認 WiFi 已連接
   python3 -c "import serial; ser=serial.Serial('/dev/ttyACM0', 115200);
   [print(ser.readline()) for _ in range(20)]"
   ```

3. **驗證 IP 地址**
   ```bash
   # 從串口輸出中找到正確的 IP
   # 使用 ping 測試連通性
   ping -c 3 10.154.48.200
   ```

4. **檢查網絡連接**
   ```bash
   # 確認 PC 和 ESP32 在同一網絡
   iwconfig
   ip addr show
   ```

### 問題 2: WiFi 配對不匹配

**症狀：**
```
⚠ WiFi Mismatch - ESP32: ExoPulse | PC: OtherNetwork
```

**解決方案：**

```bash
# 1. 斷開當前 WiFi 連接
nmcli device disconnect wlp3s0

# 2. 連接到 ESP32 的 WiFi（例如：ExoPulse）
nmcli device wifi connect ExoPulse password exopulse123

# 3. 確認連接成功
iwconfig | grep ESSID
```

### 問題 3: ESP32 未連接到 WiFi

**症狀：**
```
⊗ Waiting for ESP32 WiFi connection...
```

**解決方案：**

1. **檢查 WiFi 配置**

   編輯 `ExoPulse_firmware/MGv2/include/wifi_sta_config.h`：

   ```cpp
   namespace WiFiSTAConfig {
       constexpr const char* SSID = "ExoPulse";        // 確認 SSID 正確
       constexpr const char* PASSWORD = "exopulse123"; // 確認密碼正確
       constexpr uint16_t TCP_PORT = 8888;
   }
   ```

2. **重新編譯並上傳固件**
   ```bash
   cd ExoPulse_firmware/MGv2
   pio run -e firebeetle32 --target upload
   ```

3. **檢查 WiFi 熱點**
   - 確認手機熱點或路由器已開啟
   - 確認 WiFi 名稱和密碼匹配
   - 確認 WiFi 在 2.4GHz 頻段（ESP32 不支持 5GHz）

---

## 網絡診斷工具

### 快速連接測試

```bash
python3 test_esp32_connection.py 10.154.48.200 8888
```

**輸出示例：**
```
============================================================
ExoPulse ESP32 Connection Tester
============================================================

[TEST] Checking WiFi interface...
✓ Interface wlp3s0 connected to: ExoPulse
  Signal strength: -42 dBm (Excellent)

[TEST] Pinging 10.154.48.200...
✓ Host 10.154.48.200 is reachable via ping

[TEST] Testing TCP connection to 10.154.48.200:8888...
✓ TCP connection successful (took 15.3ms)
✓ Received data from server:
  > === ExoPulse Dual Motor Monitor ===

============================================================
SUMMARY:
  Ping test: ✓ PASS
  TCP test:  ✓ PASS

✓ ESP32 is ready! You can run:
  cd ExoPulse_firmware/UI_components
  python3 wifi_monitor.py 10.154.48.200 8888
============================================================
```

### 網絡掃描

```bash
# 掃描局域網中的 ESP32 設備
python3 test_esp32_connection.py
```

### 手動 WiFi 檢查

```bash
# PC WiFi 狀態
iwconfig
nmcli device wifi list

# PC IP 地址
hostname -I
ip addr show

# 網絡連通性
ping -c 3 10.154.48.200

# 端口檢查
nc -zv 10.154.48.200 8888
```

---

## 相關文件

| 文件 | 說明 |
|------|------|
| [wifi_monitor.py](wifi_monitor.py) | WiFi 監控 GUI（已添加配對狀態） |
| [test_esp32_connection.py](../../test_esp32_connection.py) | ESP32 連接診斷工具 |
| [test_wifi_monitor_standalone.py](../../test_wifi_monitor_standalone.py) | 模擬 ESP32 服務器 |
| [test_wifi_pairing_status.py](../../test_wifi_pairing_status.py) | WiFi 配對測試指南 |
| [TEST_WIFI_MONITOR.md](../../TEST_WIFI_MONITOR.md) | 完整測試文檔 |

---

## ESP32 固件配置

### WiFi STA 配置

文件：`ExoPulse_firmware/MGv2/include/wifi_sta_config.h`

```cpp
namespace WiFiSTAConfig {
    // WiFi credentials
    constexpr const char* SSID = "ExoPulse";
    constexpr const char* PASSWORD = "exopulse123";

    // TCP Server configuration
    constexpr uint16_t TCP_PORT = 8888;
    constexpr int MAX_CLIENTS = 1;

    // Connection timeouts
    constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;
}
```

### 數據格式

ESP32 發送的馬達數據格式：

```
[timestamp] M:motor_id T:temp V:voltage I:current S:speed ACC:accel E:encoder A:angle ERR:error
```

**示例：**
```
[592655] M:1 T:38 V:0.8 I:-0.03 S:0 ACC:0 E:1344 A:-0.07 ERR:0x0
[592672] M:2 T:36 V:0.8 I:-0.06 S:5 ACC:0 E:54790 A:9.10 ERR:0x0
```

### 編譯和上傳

```bash
# 編譯
cd ExoPulse_firmware/MGv2
pio run -e firebeetle32

# 上傳
pio run -e firebeetle32 --target upload

# 監控
pio device monitor -p /dev/ttyACM0
```

---

## 開發者注意事項

### Python 依賴

```bash
pip install matplotlib numpy pyserial
```

### 代碼修改位置

如需自定義配對邏輯，修改以下部分：

1. **WiFi 信息獲取**：`get_pc_wifi_details()` 方法
2. **配對判斷邏輯**：`read_data()` 方法中的配對檢查
3. **GUI 顯示**：`update_plot()` 方法中的狀態欄更新

### 擴展功能建議

- [ ] 添加自動 WiFi 切換功能
- [ ] 支持多個 ESP32 設備同時監控
- [ ] 添加 WiFi 信號強度警告
- [ ] 記錄 WiFi 連接歷史
- [ ] 支持 WiFi 配置熱重載

---

## 總結

此功能實現了：

✅ **自動檢測** - 自動獲取 ESP32 和 PC 的 WiFi 信息
✅ **實時配對** - 實時比對兩者的 WiFi SSID
✅ **視覺提示** - 清晰的顏色和圖標指示配對狀態
✅ **安全斷開** - 明確提示何時可以安全拔掉 USB
✅ **詳細信息** - 顯示完整的 WiFi 連接詳情

**工作流程：**
```
USB 連接 → 檢查 ESP32 WiFi → PC 連接同一 WiFi → 配對成功 → 安全斷開 USB → WiFi 通訊
```

---

## 聯繫與支持

如遇問題，請提供以下信息：

1. ESP32 串口輸出
2. PC WiFi 狀態（`iwconfig` 輸出）
3. 連接測試結果（`test_esp32_connection.py` 輸出）
4. WiFi Monitor 錯誤信息
