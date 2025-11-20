# ExoPulse WiFi 馬達監控系統 - 使用指南

**版本：** 1.0
**日期：** 2025年11月20日
**系統：** ESP32 + CAN Bus + WiFi網路傳輸

---

## 📋 目錄

1. [系統概述](#系統概述)
2. [硬體需求](#硬體需求)
3. [快速開始](#快速開始)
4. [詳細設定步驟](#詳細設定步驟)
5. [Python監控GUI使用](#python監控gui使用)
6. [WiFi命令參考](#wifi命令參考)
7. [疑難排解](#疑難排解)

---

## 系統概述

### 特色功能

✅ **無線監控** - 透過WiFi網路即時監控雙馬達狀態
✅ **高更新率** - 10Hz (100ms) 資料更新頻率
✅ **遠端校正** - 透過網路遠端進行馬達零點校正
✅ **雙馬達支援** - 同時監控兩個LK-TECH馬達
✅ **即時圖表** - Python GUI提供5種參數即時繪圖
✅ **自動重連** - 網路中斷時自動嘗試重新連接
✅ **FreeRTOS** - 多工架構確保穩定運作

### 資料參數

每個馬達提供以下即時資料：
- **溫度** (°C)
- **電壓** (V)
- **電流** (A)
- **速度** (dps)
- **加速度** (dps²)
- **編碼器位置** (14-bit)
- **多圈角度** (°)
- **錯誤狀態** (Error flags)

### 架構圖

```
┌─────────────────┐
│   ESP32         │
│   (WiFi+CAN)    │
│                 │
│ ├─ CAN Bus ────┼──→ Motor 1 (0x141)
│ │              │
│ ├─ CAN Bus ────┼──→ Motor 2 (0x142)
│ │              │
│ └─ WiFi ───────┼──→ PC/筆電 (Python GUI)
│                 │      ↑
└─────────────────┘      │
         ↑               │
         │               │
    行動熱點 "ExoPulse"  │
         └───────────────┘
```

---

## 硬體需求

### 必要硬體

| 項目 | 規格 | 備註 |
|------|------|------|
| **ESP32 開發板** | ESP32-WROOM-32 | DevKit v1 或相容板 |
| **MCP2515 CAN 模組** | SPI介面，8MHz晶振 | **需要5V電源！** |
| **LK-TECH 馬達** | M系列（MGv2） | 支援CAN Bus通訊 |
| **CAN 終端電阻** | 120Ω × 2 | 匯流排兩端各一個 |
| **行動熱點** | 2.4GHz WiFi | SSID: ExoPulse |
| **電源供應** | 5V for MCP2515 | 大部分模組需要5V |

### 選用硬體

- USB Serial線（用於韌體上傳與除錯）
- LED指示燈（GPIO 2，心跳顯示）

---

## 快速開始

### 步驟 1：建立WiFi熱點

在手機上建立行動熱點：
- **SSID (名稱)**：`ExoPulse`
- **密碼**：`12345666`
- **頻段**：2.4GHz（**必須！ESP32不支援5GHz**）

### 步驟 2：編譯並上傳韌體

```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 編譯WiFi馬達控制韌體
pio run -e wifi_motor

# 上傳到ESP32
pio run -e wifi_motor --target upload

# 監看序列埠以取得IP位址
pio device monitor --baud 115200
```

### 步驟 3：記錄ESP32的IP位址

從序列埠輸出中找到IP位址：

```
========================================
   ExoPulse WiFi Motor Monitor
========================================

Initializing CAN Bus... OK (1 Mbps)
Connecting to WiFi: ExoPulse
..........
✓ WiFi Connected!
IP Address: 192.168.43.123  <--- 記下這個IP
Signal (RSSI): -45 dBm
TCP Server on port: 8888
Connect to: 192.168.43.123:8888
========================================
```

### 步驟 4：執行Python監控GUI

確保PC也連接到同一個WiFi熱點 `ExoPulse`，然後：

```bash
# 方法1：使用預設IP (需先修改程式碼中的預設值)
python3 src/monitor_wifi.py

# 方法2：指定IP位址（建議）
python3 src/monitor_wifi.py 192.168.43.123 8888

# 安裝必要套件（首次使用）
pip3 install pyserial matplotlib numpy
```

---

## 詳細設定步驟

### ESP32 硬體接線

#### MCP2515 ↔ ESP32 (SPI連接)

```
MCP2515    →    ESP32
─────────────────────────
VCC        →    5V (重要！)
GND        →    GND
CS         →    GPIO 5
SI/MOSI    →    GPIO 23
SO/MISO    →    GPIO 19
SCK        →    GPIO 18
INT        →    (未使用)
```

#### MCP2515 ↔ CAN Bus (馬達連接)

```
MCP2515    →    CAN Bus
─────────────────────────
CAN_H      →    Motor 1 & 2 CAN_H
CAN_L      →    Motor 1 & 2 CAN_L
GND        →    共地

終端電阻：CAN_H 與 CAN_L 之間各接一個 120Ω (匯流排兩端)
```

### WiFi 網路設定

#### 行動熱點設定（手機）

1. 開啟手機設定 → 個人熱點
2. 設定熱點名稱：`ExoPulse`
3. 設定密碼：`12345666`
4. 確認頻段為 **2.4GHz**（重要！）
5. 開啟熱點

#### PC連接到熱點

1. PC WiFi設定 → 搜尋可用網路
2. 連接到 `ExoPulse`
3. 輸入密碼：`12345666`
4. 確認已連接

### 韌體編譯與上傳

#### 使用PlatformIO CLI

```bash
# 前往專案目錄
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 檢查ESP32連接
ls /dev/ttyUSB*  # 或 /dev/ttyACM*

# 編譯（檢查語法）
pio run -e wifi_motor

# 編譯並上傳
pio run -e wifi_motor --target upload

# 監看序列埠輸出
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

#### 預期編譯結果

```
...
RAM:   [==        ]  22.4% (used 73408 bytes from 327680 bytes)
Flash: [====      ]  35.2% (used 461248 bytes from 1310720 bytes)
========================= [SUCCESS] Took X.XX seconds =========================
```

#### 預期序列埠輸出

```
========================================
   ExoPulse WiFi Motor Monitor
========================================

Initializing CAN Bus... OK (1 Mbps)
Connecting to WiFi: ExoPulse
..........
✓ WiFi Connected!
IP Address: 192.168.43.123
Signal (RSSI): -45 dBm
TCP Server on port: 8888
Connect to: 192.168.43.123:8888

System ready!
========================================

[1234] M:1 T:28 V:0.8 I:0.02 S:0 ACC:0 E:4832 A:265.43 ERR:0x00
[1345] M:2 T:29 V:0.8 I:-0.01 S:0 ACC:0 E:8945 A:489.22 ERR:0x00
...
```

---

## Python監控GUI使用

### 啟動GUI

```bash
# 基本用法（需先修改程式碼中的預設IP）
python3 src/monitor_wifi.py

# 指定ESP32 IP位址（建議）
python3 src/monitor_wifi.py 192.168.43.123

# 完整參數
python3 src/monitor_wifi.py <ESP32_IP> <Port>
# 範例：
python3 src/monitor_wifi.py 192.168.43.123 8888
```

### GUI介面說明

GUI視窗分為三個區域：

#### 1. 圖表區（雙欄配置）

**左欄 - Motor 1：**
- 溫度 (°C)
- 電流 (A)
- 速度 (dps)
- 加速度 (dps²)
- 角度 (°)

**右欄 - Motor 2：**
- 溫度 (°C)
- 電流 (A)
- 速度 (dps)
- 加速度 (dps²)
- 角度 (°)

#### 2. 狀態顯示區

```
狀態: Connected
M1: T=28°C | I=0.02A | S=0dps | A=265.43°
M2: T=29°C | I=-0.01A | S=0dps | A=489.22°
```

#### 3. 控制按鈕區

| 按鈕 | 功能 |
|------|------|
| **校正 M1** | 將Motor 1當前位置設為零點 |
| **校正 M2** | 將Motor 2當前位置設為零點 |
| **校正兩者** | 同時校正兩個馬達 |
| **清除校正** | 清除所有校正偏移值 |
| **狀態** | 查詢系統狀態 |

### GUI操作提示

- **縮放**：使用matplotlib工具列的縮放工具
- **平移**：使用matplotlib工具列的平移工具
- **儲存圖表**：使用matplotlib工具列的儲存按鈕
- **關閉**：直接關閉視窗即可

### 連線狀態指示

- 🟢 **綠色**：`Connected` - 正常連線
- 🟡 **黃色**：`Reconnecting...` - 重新連線中
- 🔴 **紅色**：`Disconnected` / `Error` - 連線失敗

---

## WiFi命令參考

### 可用命令列表

透過網路連接後，可以發送以下命令：

| 命令 | 功能 | 範例回應 |
|------|------|----------|
| `CAL1` 或 `CAL_M1` | 校正Motor 1 | `[OK] Motor 1 calibrated` |
| `CAL2` 或 `CAL_M2` | 校正Motor 2 | `[OK] Motor 2 calibrated` |
| `CAL_BOTH` | 校正兩個馬達 | `[OK] Both motors calibrated` |
| `CLEAR_CAL` | 清除校正 | `[OK] Calibration cleared` |
| `STATUS` | 查詢系統狀態 | `[STATUS] WiFi:OK Client:OK M1:OK M2:OK` |
| `HELP` | 顯示命令列表 | 命令列表 |

### 使用Telnet測試

可以使用telnet直接連接測試：

```bash
telnet 192.168.43.123 8888

# 連接後輸入命令
CAL1
STATUS
HELP
```

### 資料格式

ESP32持續輸出的資料格式：

```
[時間戳] M:馬達ID T:溫度 V:電壓 I:電流 S:速度 ACC:加速度 E:編碼器 A:角度 ERR:錯誤
```

範例：
```
[1234] M:1 T:28 V:0.8 I:0.02 S:0 ACC:0 E:4832 A:265.43 ERR:0x00
[1345] M:2 T:29 V:0.8 I:-0.01 S:0 ACC:0 E:8945 A:489.22 ERR:0x00
```

欄位說明：
- `[1234]`: 時間戳 (milliseconds)
- `M:1`: 馬達ID
- `T:28`: 溫度 28°C
- `V:0.8`: 電壓 0.8V
- `I:0.02`: 電流 0.02A
- `S:0`: 速度 0 dps
- `ACC:0`: 加速度 0 dps²
- `E:4832`: 編碼器位置
- `A:265.43`: 角度 265.43°
- `ERR:0x00`: 錯誤狀態（0x00=正常）

---

## 疑難排解

### WiFi連線問題

#### ESP32無法連接WiFi

**症狀：** 序列埠顯示 `✗ WiFi Failed!`

**解決方案：**
1. 確認熱點SSID完全一致：`ExoPulse`（區分大小寫）
2. 確認密碼正確：`12345666`
3. **確認熱點頻段為2.4GHz**（ESP32不支援5GHz）
4. 檢查手機熱點是否已開啟
5. 嘗試重新啟動ESP32

#### PC無法連接到ESP32

**症狀：** Python程式顯示 `Connection failed`

**解決方案：**
1. 確認PC已連接到 `ExoPulse` WiFi
2. 確認IP位址正確（從序列埠輸出取得）
3. 檢查防火牆設定（可能封鎖8888 port）
4. 嘗試ping ESP32：`ping 192.168.43.123`
5. 使用telnet測試：`telnet 192.168.43.123 8888`

#### 連線中斷

**症狀：** GUI顯示 `Reconnecting...`

**原因：**
- WiFi訊號不穩定
- ESP32重新啟動
- 網路擁塞

**解決方案：**
- GUI會自動嘗試重連（最多5次）
- 檢查WiFi訊號強度（RSSI值）
- 將ESP32與路由器（手機）距離拉近
- 重新啟動Python GUI

### CAN Bus問題

#### MCP2515初始化失敗

**症狀：** 序列埠顯示 `CAN Bus... FAILED!`

**解決方案：**
1. 檢查所有SPI接線（6條線）
2. **確認MCP2515使用5V電源**（重要！）
3. 檢查CS引腳為GPIO 5
4. 嘗試更換MCP2515模組
5. 檢查晶振頻率（8MHz vs 16MHz）

#### 無馬達資料

**症狀：** GUI圖表無資料顯示

**解決方案：**
1. 確認馬達電源已開啟
2. 檢查CAN_H與CAN_L接線
3. 確認有120Ω終端電阻（兩端）
4. 檢查馬達CAN ID設定（應為1和2）
5. 查看序列埠是否有錯誤訊息

### Python GUI問題

#### GUI視窗不顯示

**症狀：** 執行程式後無視窗

**解決方案：**
```bash
# 檢查X11
echo $DISPLAY

# 如使用SSH，啟用X forwarding
ssh -X user@host

# 或使用VNC/遠端桌面

# 本機執行嘗試
export DISPLAY=:0
```

#### 圖表凍結

**症狀：** 圖表不更新

**解決方案：**
1. 檢查ESP32是否正在傳送資料（序列埠）
2. 檢查網路連線
3. 重新啟動Python GUI
4. 檢查馬達電源與CAN bus

#### ModuleNotFoundError

**症狀：** 缺少Python套件

**解決方案：**
```bash
# 安裝matplotlib
pip3 install matplotlib

# 安裝pyserial
pip3 install pyserial

# 安裝numpy
pip3 install numpy

# 或一次安裝全部
pip3 install matplotlib pyserial numpy
```

### 效能問題

#### 資料更新太慢

**原因：** WiFi訊號不佳或網路擁塞

**解決方案：**
1. 改善WiFi訊號強度
2. 減少GUI最大資料點數（修改程式碼）
3. 降低ESP32更新頻率（修改韌體）

#### 記憶體不足

**症狀：** ESP32重新啟動或當機

**解決方案：**
1. 減少佇列大小（修改韌體）
2. 降低更新頻率
3. 檢查是否有記憶體洩漏

---

## 進階配置

### 修改WiFi設定

編輯 `src/main_wifi.cpp`：

```cpp
// 修改這些參數
const char* WIFI_SSID = "你的WiFi名稱";
const char* WIFI_PASSWORD = "你的密碼";
const uint16_t TCP_PORT = 8888;  // 修改port
```

### 修改更新頻率

編輯 `src/main_wifi.cpp`：

```cpp
// 預設100ms (10Hz)
constexpr uint32_t UPDATE_INTERVAL_MS = 100;

// 改為50ms (20Hz) - 更高頻率
constexpr uint32_t UPDATE_INTERVAL_MS = 50;

// 改為200ms (5Hz) - 較低頻率
constexpr uint32_t UPDATE_INTERVAL_MS = 200;
```

### 修改馬達ID

編輯 `src/main_wifi.cpp`：

```cpp
// 預設馬達ID為1和2
constexpr uint8_t MOTOR_ID_1 = 1;
constexpr uint8_t MOTOR_ID_2 = 2;

// 可改為其他ID (1-127)
constexpr uint8_t MOTOR_ID_1 = 3;
constexpr uint8_t MOTOR_ID_2 = 4;
```

---

## 檔案結構

```
ExoPulse_firmware/
├── src/
│   ├── main_wifi.cpp              # WiFi馬達控制韌體
│   └── monitor_wifi.py            # Python WiFi監控GUI
├── platformio.ini                 # 包含 wifi_motor 環境
├── WIFI_MOTOR_GUIDE.md            # 本文檔
└── COMPREHENSIVE_DOCUMENTATION.md # 完整系統文檔
```

---

## 技術規格

### 網路規格
- **協定**：TCP/IP
- **WiFi**：802.11 b/g/n (2.4GHz)
- **TCP Port**：8888
- **資料速率**：~10KB/s (10Hz更新)

### CAN Bus規格
- **波特率**：1 Mbps (fallback: 500 Kbps)
- **晶振**：8 MHz
- **馬達協定**：LK-TECH M系列

### 效能指標
- **更新頻率**：10 Hz (100ms interval)
- **延遲**：< 50ms (WiFi條件良好時)
- **最大資料點**：100點 (可調整)

---

## 參考資源

### 相關文檔
- [COMPREHENSIVE_DOCUMENTATION.md](COMPREHENSIVE_DOCUMENTATION.md) - 完整系統文檔
- [WIFI_TEST_INSTRUCTIONS.md](WIFI_TEST_INSTRUCTIONS.md) - WiFi測試指南
- [MGv2/README.md](MGv2/README.md) - MGv2馬達系統文檔

### 外部資源
- [ESP32 Arduino參考](https://docs.espressif.com/projects/arduino-esp32/)
- [PlatformIO文檔](https://docs.platformio.org/)
- [Matplotlib文檔](https://matplotlib.org/stable/contents.html)

---

## 版本歷史

**v1.0 (2025-11-20)**
- 初版發布
- 支援WiFi網路傳輸
- 雙馬達即時監控
- Python GUI介面
- 遠端校正功能

---

**文檔版本：** 1.0
**最後更新：** 2025年11月20日
**維護者：** ExoPulse開發團隊

---

**ExoPulse WiFi 馬達監控系統**
*從Serial到WiFi的完整升級方案*
