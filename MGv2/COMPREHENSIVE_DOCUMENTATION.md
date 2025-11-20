# ExoPulse 外骨骼系統 - 完整技術文檔

**專案名稱：** ExoPulse Exoskeleton Control System
**最後更新：** 2025年11月20日
**版本：** 2.0 (WiFi + Motor Control)

---

## 目錄

1. [系統概述](#1-系統概述)
2. [硬體架構](#2-硬體架構)
3. [WiFi 無線連線系統](#3-wifi-無線連線系統)
4. [馬達控制系統 (MGv2)](#4-馬達控制系統-mgv2)
5. [快速開始指南](#5-快速開始指南)
6. [硬體測試程序](#6-硬體測試程序)
7. [部署檢查清單](#7-部署檢查清單)
8. [已知問題與解決方案](#8-已知問題與解決方案)
9. [監控工具](#9-監控工具)
10. [疑難排解](#10-疑難排解)

---

## 1. 系統概述

### 1.1 專案簡介

ExoPulse 是一個基於 ESP32 的外骨骼控制系統，整合了：
- **無線通訊**：WiFi STA 模式連接行動熱點
- **馬達控制**：透過 CAN Bus 控制 LK-TECH MGv2 馬達
- **即時監控**：Python GUI 與 Web 介面
- **多核心處理**：FreeRTOS 實現高效多工

### 1.2 技術棧

| 組件 | 技術 |
|------|------|
| **微控制器** | ESP32 (Xtensa dual-core, 240MHz) |
| **作業系統** | FreeRTOS |
| **通訊協定** | CAN Bus (ISO 11898-1) + WiFi (2.4GHz) |
| **CAN 控制器** | MCP2515 + TJA1050 收發器 |
| **開發環境** | PlatformIO + Arduino Framework |
| **程式語言** | C++ (韌體) + Python (監控工具) |

### 1.3 專案結構

```
ExoPulse/depRL/deploy_IoT/
├── ExoPulse_firmware/
│   ├── MGv2/                      # 馬達控制系統 (MGv2 motors)
│   │   ├── src/
│   │   │   ├── main.cpp           # 雙馬達控制主程式
│   │   │   ├── main.h             # 標頭檔
│   │   │   └── monitor_dual_motor_enhanced.py  # GUI監控工具
│   │   ├── include/
│   │   │   ├── ExoBus.h           # CAN通訊核心
│   │   │   └── SerialConsole.h    # 序列埠命令介面
│   │   ├── test/hardware_validation/  # 硬體驗證測試
│   │   ├── docs/                  # 技術文檔
│   │   ├── examples/              # 測試範例
│   │   ├── README.md
│   │   ├── BUILD_REPORT.md
│   │   ├── HARDWARE_TEST_GUIDE.md
│   │   └── DEPLOYMENT_CHECKLIST.md
│   │
│   ├── src/
│   │   ├── WiFi_STA_Test.h        # WiFi測試韌體
│   │   ├── main_wifi_test.cpp     # WiFi測試入口
│   │   └── test_client.ps1        # Windows測試客戶端
│   │
│   ├── platformio.ini             # 建置配置
│   ├── WIFI_TEST_INSTRUCTIONS.md  # WiFi測試指南
│   └── read_serial.py             # 序列埠讀取工具
│
└── COMPREHENSIVE_DOCUMENTATION.md  # 本文檔
```

---

## 2. 硬體架構

### 2.1 核心硬體

#### ESP32 DevKit v1
- **處理器**：Xtensa 32-bit LX6 雙核心 @ 240MHz
- **記憶體**：520KB SRAM
- **Flash**：4MB (部分版本可達 16MB)
- **WiFi**：802.11 b/g/n (2.4GHz)
- **藍牙**：Bluetooth v4.2 BR/EDR 和 BLE
- **GPIO**：34個
- **USB**：CP2102 / CH340 UART-to-USB

#### MCP2515 CAN 控制器
- **介面**：SPI (最高 10MHz)
- **CAN 速率**：500 KBPS (標準)
- **晶振**：8MHz (備用：16MHz)
- **電源**：**5V** (重要！大部分模組需要 5V，非 3.3V)
- **收發器**：TJA1050

#### MGv2 (LK-TECH) 馬達驅動器
- **通訊**：CAN Bus
- **馬達 ID**：0x141 (Motor 1), 0x142 (Motor 2)
- **特性**：
  - 多圈角度追蹤
  - 加速度資料 (Command 0x33)
  - 溫度與電流監控
  - 軟體零點校正

### 2.2 接線圖

#### ESP32 ↔ MCP2515 (SPI 連接)
```
ESP32 GPIO  →  MCP2515
─────────────────────────
GPIO 5      →  CS (晶片選擇)
GPIO 23     →  SI/MOSI (主出從入)
GPIO 19     →  SO/MISO (主入從出)
GPIO 18     →  SCK (時鐘)
GPIO 21     →  INT (中斷，選用)
3.3V        →  VCC (部分模組)
5V          →  VCC (大部分模組需要！)
GND         →  GND
```

#### MCP2515 ↔ CAN Bus (雙馬達系統)
```
MCP2515_1  →  CAN Bus  →  Motor 1 (0x141)
          →           →  Motor 2 (0x142)

接線：
CAN_H ─────────────── CAN_H (Motor 1, Motor 2)
CAN_L ─────────────── CAN_L (Motor 1, Motor 2)
GND ───────────────── GND (共地)

終端電阻：
- CAN Bus 兩端各需一個 120Ω 電阻 (CAN_H 與 CAN_L 之間)
```

#### WiFi 連線 (無線)
```
ESP32 WiFi → 行動熱點 "ExoPulse" (2.4GHz)
         → PC/手機 (同一 WiFi 網路)
```

### 2.3 硬體驗證結果

| 測試項目 | 狀態 | 成功率 | 備註 |
|---------|------|-------|------|
| LED 測試 | ✅ PASS | 100% | ESP32 硬體驗證 |
| Loopback 測試 | ✅ PASS | 100% | MCP2515 SPI 通訊驗證 |
| 序列埠控制台 | ✅ PASS | 100% | 所有命令正常運作 |
| CAN Bus 通訊 | ✅ PASS | 100% | 40/40 訊息成功傳輸 |
| WiFi 連線 | ✅ PASS | 100% | TCP Server 成功建立 |
| 馬達控制 | ⏭️ PENDING | N/A | 需要實體馬達硬體 |

**關鍵發現：**
- ⚠️ **MCP2515 電源需求**：大部分模組需要 **5V**，非 3.3V！
- ✅ **CAN 波特率**：500 KBPS
- ✅ **晶振頻率**：8 MHz
- ✅ **終端電阻**：兩端各 120Ω

---

## 3. WiFi 無線連線系統

### 3.1 WiFi 測試系統概述

WiFi 測試系統允許 ESP32 連接到行動熱點，建立 TCP Server，提供：
- 即時傳輸統計（封包數、速率、延遲、RSSI）
- 遠端命令控制（START、STOP、STATS、PING）
- 無線馬達狀態監控（未來擴展）

### 3.2 WiFi 配置

| 參數 | 值 |
|------|---|
| **SSID** | ExoPulse |
| **密碼** | 12345666 |
| **頻段** | 2.4GHz (ESP32 不支援 5GHz) |
| **IP 分配** | DHCP (通常 192.168.43.xxx) |
| **TCP Port** | 8888 |
| **加密** | WPA2-PSK |

### 3.3 WiFi 韌體檔案

- **src/WiFi_STA_Test.h** - WiFi 測試主程式
- **src/main_wifi_test.cpp** - WiFi 測試入口點
- **src/test_client.ps1** - Windows PowerShell 測試客戶端
- **WIFI_TEST_INSTRUCTIONS.md** - 完整測試指南

### 3.4 WiFi 測試指令

連線後可用的命令：

| 指令 | 功能 |
|-----|------|
| `START` | 開始連續傳輸測試（測試穩定性） |
| `STOP` | 停止測試並顯示統計資料 |
| `STATS` | 顯示即時統計資料 |
| `PING` | 測試連線延遲 |
| `EXIT` | 結束客戶端程式 |

### 3.5 建立與測試 WiFi 連線

#### 步驟 1：建立行動熱點
```
手機設定：
- 熱點名稱 (SSID): ExoPulse
- 密碼: 12345666
- 頻段: 2.4GHz (必須！)
```

#### 步驟 2：編譯並上傳 WiFi 測試韌體
```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 編譯 WiFi 測試環境
pio run -e wifi_test

# 上傳到 ESP32
pio run -e wifi_test --target upload

# 監看序列埠輸出以取得 IP 位址
screen /dev/ttyUSB0 115200
```

#### 步驟 3：從序列埠讀取 ESP32 的 IP 位址
```
========================================
   WiFi Stability Test System
========================================

Connecting to mobile hotspot...
SSID: ExoPulse
..........
✓ WiFi Connected!
IP Address: 192.168.43.123  <--- 記下這個 IP
Signal Strength (RSSI): -45 dBm
TCP Server started on port: 8888
```

#### 步驟 4：更新測試客戶端 IP
編輯 `src/test_client.ps1`:
```powershell
$ServerIP = "192.168.43.123"  # 改成從序列埠看到的 IP
$ServerPort = 8888
```

#### 步驟 5：執行測試客戶端 (Windows PowerShell)
```powershell
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware/src
.\test_client.ps1
```

預期輸出：
```
Connected!

========================================
   WiFi Stability Test System
========================================
Available commands:
  START - Start continuous transmission test
  STOP  - Stop test
  STATS - Show statistics
  PING  - Test connection latency
========================================
```

### 3.6 WiFi 環境配置

在 `platformio.ini` 中已新增：
```ini
[env:wifi_test]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
build_flags =
  -std=gnu++17
build_src_filter =
  +<main_wifi_test.cpp>
  -<main.cpp>
```

### 3.7 WiFi 疑難排解

#### ESP32 無法連接 WiFi
1. 確認熱點 SSID 完全一致：`ExoPulse`（區分大小寫）
2. 確認密碼正確：`12345666`
3. **確認頻段為 2.4GHz**（ESP32 不支援 5GHz）
4. 檢查手機熱點是否已開啟

#### PC 無法連接到 ESP32
1. 確認 PC 和 ESP32 在同一個 WiFi 網路（都連到 `ExoPulse`）
2. 從序列埠確認 ESP32 的實際 IP 位址
3. 檢查 Windows 防火牆設定

#### 恢復原本的馬達控制程式
```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware
pio run -e esp32doit-devkit-v1 --target upload
```

---

## 4. 馬達控制系統 (MGv2)

### 4.1 MGv2 系統概述

MGv2 系統使用 LK-TECH M 系列馬達，提供：
- 雙馬達同步控制（Motor 1: 0x141, Motor 2: 0x142）
- 即時狀態讀取（溫度、電流、速度、角度、加速度）
- 軟體校正功能（無需寫入 ROM）
- FreeRTOS 多工處理（50ms 讀取週期）

### 4.2 CAN Bus 命令協定

所有 CAN 幀格式：
```
CAN ID = 0x140 + Motor_ID
Data Length = 8 bytes
Data[0] = Command Code
Data[1-7] = Parameters
```

#### 4.2.1 馬達控制命令

**0xA1 - 扭矩電流控制**
```cpp
// 發送格式
Data[0] = 0xA1           // 命令碼
Data[1-3] = 0x00         // 保留
Data[4-5] = iq (int16)   // 扭矩電流 (-2048~2048 → -33A~33A)
Data[6-7] = 0x00         // 保留

// 換算公式
int16_t iq = (int16_t)round(torqueNm * 20.0);  // kNmToIq_ = 20.0
float actualCurrent = (float)iq_raw * 33.0f / 2048.0f;
```

**0xA2 - 速度控制**
```cpp
// 發送格式
Data[0] = 0xA2               // 命令碼
Data[1-3] = 0x00             // 保留
Data[4-7] = speed (int32)    // 速度（單位：0.01 dps）

// 換算公式
int32_t speed_raw = (int32_t)(speedDps * 100.0f);
float speedDps = (float)speed_raw / 100.0f;
```

**0xA3 - 位置控制**
```cpp
// 發送格式
Data[0] = 0xA3                  // 命令碼
Data[1-3] = 0x00                // 保留
Data[4-7] = position (int32)    // 位置（單位：0.01 度）

// 換算公式
int32_t position_raw = (int32_t)(angleDeg * 100.0f);
float angleDeg = (float)position_raw / 100.0f;
```

#### 4.2.2 狀態查詢命令

**0x9C - 讀取馬達狀態2（最常用）**
```cpp
// 回應格式
Data[0] = 0x9C                  // 命令碼回傳
Data[1] = temperature (int8)    // 溫度（°C）
Data[2-3] = iq (int16)          // 扭矩電流
Data[4-5] = speed (int16)       // 速度（dps）
Data[6-7] = encoder (uint16)    // 編碼器位置（0~16383）

// 解析
int8_t temperature = (int8_t)buf[1];
int16_t iq_raw = (int16_t)(buf[2] | (buf[3] << 8));
int16_t speed_raw = (int16_t)(buf[4] | (buf[5] << 8));
uint16_t encoder = (uint16_t)(buf[6] | (buf[7] << 8));
```

**0x9A - 讀取馬達狀態1（含錯誤標誌）**
```cpp
// 回應格式
Data[0] = 0x9A                  // 命令碼回傳
Data[1] = temperature (int8)    // 溫度（°C）
Data[2] = 0x00                  // 保留
Data[3-4] = voltage (uint16)    // 電壓（0.1V/LSB）
Data[5-6] = 0x00                // 保留
Data[7] = errorState (uint8)    // 錯誤標誌

// 錯誤標誌
if (errorState & 0x01) → 低電壓 (LOW_VOLTAGE)
if (errorState & 0x08) → 過溫 (OVER_TEMP)
```

**0x92 - 讀取多圈角度**
```cpp
// 回應格式
Data[0] = 0x92                    // 命令碼回傳
Data[1-6] = motorAngle (int64)    // 角度（0.01°/LSB，48-bit）
Data[7] = 0x00                    // 保留

// 解析
int64_t motorAngle = (int64_t)buf[1]
                   | ((int64_t)buf[2] << 8)
                   | ((int64_t)buf[3] << 16)
                   | ((int64_t)buf[4] << 24)
                   | ((int64_t)buf[5] << 32)
                   | ((int64_t)buf[6] << 40);
float angleDeg = (float)motorAngle * 0.01f;
```

**0x33 - 讀取加速度**
```cpp
// 回應格式
Data[0] = 0x33                        // 命令碼回傳
Data[1-4] = acceleration (int32)      // 加速度（1 dps²/LSB）
Data[5-7] = 0x00                      // 保留

// 解析
int32_t accel_raw = (int32_t)(buf[1]
                             | (buf[2] << 8)
                             | (buf[3] << 16)
                             | (buf[4] << 24));
float accel_dps2 = (float)accel_raw;  // dps²
```

### 4.3 序列埠命令介面

透過 115200 baud 序列埠連線，支援的命令：

#### 馬達校正命令
```
CAL1 / CAL_M1     - 校正馬達 1 零點位置（軟體偏移）
CAL2 / CAL_M2     - 校正馬達 2 零點位置
CLEAR_CAL         - 清除所有校正偏移
```

#### 系統命令
```
HELP              - 顯示可用命令
STATUS            - 顯示系統狀態
DETAILED          - 啟用詳細除錯輸出
```

### 4.4 軟體校正功能

**優點：**
- ✅ 不寫入 ROM（無限次校正）
- ✅ 即時校正（無需重開機）
- ✅ 可輕易使用 CLEAR_CAL 還原
- ✅ 適合測試與開發

**注意：** 校正偏移值在 ESP32 重開機後會重置（安全設計）

**永久零點設定：** 使用 `send_set_zero.py`（Command 0x19，寫入馬達 ROM，謹慎使用！）

### 4.5 FreeRTOS 多工架構

系統使用雙核心架構：

**Core 0（Arduino Loop）：**
- 序列埠通訊
- 命令處理
- 看門狗 (Watchdog) 餵狗

**Core 1（FreeRTOS Task）：**
- 馬達狀態讀取（50ms 週期）
- CAN Bus 通訊
- 資料解析與顯示

---

## 5. 快速開始指南

### 5.1 一次性設定（Ubuntu）

```bash
# 1. 安裝 PlatformIO
sudo apt update
sudo apt install -y python3 python3-pip
pip3 install --user platformio
export PATH="$HOME/.local/bin:$PATH"

# 2. 設定序列埠權限
sudo usermod -a -G dialout $USER
# ⚠️ 登出後重新登入

# 3. 前往專案目錄
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware
```

### 5.2 尋找 ESP32 連接埠

```bash
# 拔除 ESP32
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# 插入 ESP32
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# 新出現的裝置就是你的連接埠（通常是 /dev/ttyUSB0）
```

### 5.3 測試 1：LED 測試（2 分鐘）

```bash
cp examples/test_led.h src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

**預期結果：** LED 每 5 秒變換不同閃爍模式

### 5.4 測試 2：Loopback 測試（3 分鐘）

**需要：** ESP32 + MCP2515（接線正確）

```bash
cp examples/test_loopback.h src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**預期輸出：**
```
[TX] Sent frame #0 ID=0x141 -> [RX] ID=0x141 ✓ PASS
...
--- Stats: Total=10, Success=10, Fail=0 ---
```

### 5.5 測試 3：生產韌體測試（5 分鐘）

```bash
git checkout src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**測試命令：**
```
HELP
STATUS 1
DEBUG
STOP
```

### 5.6 執行 GUI 監控工具

```bash
# 安裝相依套件
pip install pyserial matplotlib

# 執行增強版雙馬達監控
python3 MGv2/src/monitor_dual_motor_enhanced.py
```

GUI 提供：
- 即時圖表（兩個馬達各 5 個參數）
- 軟體校正按鈕（Cal Motor 1/2、Cal Both、Clear Cal）
- 連線狀態指示（🟢🟡🔴）
- 自動重連（線材鬆脫時）

---

## 6. 硬體測試程序

### 6.1 測試分級

**Tier 1：基本驗證（必須）**
1. LED 測試 - 驗證 ESP32 板功能
2. Loopback 測試 - 驗證 MCP2515 SPI 通訊

**Tier 2：CAN 通訊（建議）**
1. CAN Bus 發送/接收測試 - 驗證 CAN Bus 實體層

**Tier 3：生產應用（必須）**
1. 序列埠控制台測試
2. 馬達控制測試（需要實體馬達）

### 6.2 測試檢查表

| 測試 | 狀態 | 備註 |
|------|-----|------|
| LED 測試 | ☐ PASS ☐ FAIL | |
| Loopback 測試 | ☐ PASS ☐ FAIL | |
| 序列埠控制台 | ☐ PASS ☐ FAIL | |
| 馬達控制 | ☐ PASS ☐ FAIL ☐ N/A | |

### 6.3 硬體配置記錄

**ESP32 板：**
- 型號：_________________
- USB 連接埠：_________________
- 環境：_________________

**MCP2515 模組：**
- 型號：_________________
- 晶振：☐ 8MHz ☐ 16MHz
- 電源：☐ 3.3V ☐ 5V

**馬達（如有測試）：**
- 型號：_________________
- CAN ID：_________________
- 狀態：_________________

---

## 7. 部署檢查清單

### 7.1 程式碼品質
- [x] 所有重大 bug 已修復
- [x] 程式碼編譯無錯誤（3/3 環境）
- [x] 靜態分析通過（0 HIGH, 0 MEDIUM）
- [x] 程式碼組織符合標準
- [x] 文件已更新

### 7.2 建置驗證

| 環境 | 狀態 | RAM 使用率 | Flash 使用率 |
|------|------|-----------|-------------|
| esp32doit-devkit-v1 | ✅ SUCCESS | 6.9% | 22.4% |
| esp32-s3-n16r8 | ✅ SUCCESS | 6.2% | 4.5% |
| firebeetle32 | ✅ SUCCESS | 6.9% | 22.4% |
| wifi_test | ✅ SUCCESS | - | - |

### 7.3 部署步驟

#### 1. 合併到 Master
```bash
git checkout master
git merge feature/esp32-wifi-connectivity
git log --oneline -5
```

#### 2. 標記發布版本
```bash
git tag -a v2.0.0-wifi-motor -m "WiFi + Motor Control Release"
git push origin v2.0.0-wifi-motor
```

#### 3. 生產建置
```bash
export PATH="$HOME/.local/bin:$PATH"
pio run --target clean
pio run --environment esp32doit-devkit-v1
```

#### 4. 上傳到生產硬體
```bash
pio run -t upload --environment esp32doit-devkit-v1
pio device monitor --baud 115200
```

#### 5. 部署後驗證
- 監控 15 分鐘以上
- 驗證所有命令回應
- 測試緊急停止
- 確認看門狗運作
- 驗證馬達控制

### 7.4 回滾程序

如遇問題：
```bash
# 1. 立即停止系統
# 發送 STOP 命令或斷電

# 2. 還原到先前韌體
git checkout v1.0.x-previous
pio run -t upload --environment esp32doit-devkit-v1

# 3. 記錄問題並建立 bug 報告

# 4. 返回測試階段，問題解決前不要繼續
```

---

## 8. 已知問題與解決方案

### 8.1 馬達角度重置命令 (0x95) 未實作

**發現日期：** 2025-11-20
**嚴重性：** 高
**狀態：** 已確認 - 馬達韌體限制
**影響：** LK-TECH M 系列馬達

#### 問題摘要
CAN bus 命令 0x95（清除馬達角度）在馬達資料表中有記載，但馬達內部韌體**未完全實作**。馬達會回應 ACK，但實際上不會重置多圈角度計數器。

#### 技術細節
- **命令：** 0x95 - 清除馬達角度命令
- **預期行為：** 清除多圈和單圈角度資料，設定當前位置為零
- **實際行為：** 馬達發送 ACK 回應，但角度值保持不變

#### 證據
測試結果：
```
>>> Sent: RESET_M2
>>> [CMD] Resetting Motor 2 angle to zero...
>>> [OK] Motor 2 angle reset successful!

Motor 2 angle BEFORE reset: 630.05°
Motor 2 angle AFTER reset:  630.06° (unchanged)
```

#### 影響
1. **Motor 1 角度溢位問題：**
   - Motor 1 顯示角度為 "ovf"（overflow）
   - 無法透過軟體命令 0x95 重置
   - 影響 GUI 中的角度和加速度顯示（顯示為 0）

2. **已實作的應變措施：**
   - GUI 將 "ovf" 轉換為 0.0 以防止崩潰
   - 新增執行緒安全的序列埠通訊
   - 韌體/GUI 啟動時嘗試自動重置（會優雅地失敗）
   - Regex 解析器處理數值角度和 "ovf" 字串

3. **仍可用的資料：**
   - 溫度：✅ 正常運作
   - 電壓：✅ 正常運作
   - 電流：✅ 正常運作
   - 速度：✅ 正常運作
   - 加速度：✅ 正常運作（馬達靜止時讀取為 0）
   - 編碼器：✅ 正常運作
   - 多圈角度：❌ Motor 1 顯示溢位，Motor 2 正常

#### 建議解決方案

**短期：**
1. 接受限制 - 使用 Motor 2 的角度資料，忽略 Motor 1 角度
2. 硬體重置 - 對 Motor 1 斷電重啟以嘗試清除溢位
3. 使用單圈編碼器 - 讀取命令 0x94 而非 0x92（限制在 0-360°）

**長期：**
1. 聯絡製造商（LK-TECH）請求：
   - 馬達韌體更新以實作 0x95
   - 替代的重置方法（如果有）
   - 溢位狀況的說明
2. 考慮使用具有適當角度重置支援的替代馬達

#### 相關檔案
- `test/hardware_validation/test_dual_motor_rtos.cpp` - 新增啟動自動重置、溢位檢測
- `test/hardware_validation/monitor_dual_motor_enhanced.py` - 執行緒安全序列埠、"ovf" 處理
- `test/hardware_validation/test_reset_proof.py` - 證明重置命令無效
- `test/hardware_validation/test_reset_verbose.py` - 顯示馬達 ACK 但不重置

### 8.2 MCP2515 電源需求

**問題：** 大部分 MCP2515 模組需要 5V 電源，而非 3.3V
**症狀：** 使用 3.3V 時初始化失敗
**解決：** 連接到 5V 引腳

### 8.3 序列埠權限問題

**問題：** "Permission denied" 存取 /dev/ttyUSB0
**解決：**
```bash
sudo usermod -a -G dialout $USER
# 登出後重新登入
```

---

## 9. 監控工具

### 9.1 終端監控器 (monitor_motor.py)

**特性：**
- 輕量即時終端介面
- 統計資料（最近 20 筆讀取的 Min/Max/Avg）
- 顏色編碼狀態
- SSH 友善
- 低資源使用

**使用方式：**
```bash
python3 test/hardware_validation/monitor_motor.py [port] [baudrate]

# 預設
python3 test/hardware_validation/monitor_motor.py

# 自訂連接埠
python3 test/hardware_validation/monitor_motor.py /dev/ttyUSB1 115200
```

**顯示格式：**
```
============================================================
  LK-TECH Motor Status Monitor - 14:23:45
============================================================

  Temperature:         27 °C
  Voltage:            0.8 V
  Torque Current:   -0.03 A
  Speed:                0 dps
  Encoder:           4832 (29.5%)
  Angle:           265.43° (  0.74 turns)
  Error State:       0x0

------------------------------------------------------------
  Statistics (last 20 readings):

  Temp:  Min= 27°C  Max= 28°C  Avg= 27.5°C
  Current: Min=-0.05A  Max= 0.05A  Avg=-0.03A
  Speed:   Min=   0dps Max=   0dps Avg=    0.0dps
============================================================
```

### 9.2 GUI 監控器 (monitor_motor_gui.py)

**特性：**
- 6 個即時圖表（所有馬達參數）
- 顏色編碼狀態顯示
- 自動縮放圖表
- 歷史資料（最近 100 個資料點，可配置）
- 深色主題
- 縮放/平移/儲存圖表

**使用方式：**
```bash
python3 test/hardware_validation/monitor_motor_gui.py [port] [baudrate] [max_points]

# 預設
python3 test/hardware_validation/monitor_motor_gui.py

# 自訂最大資料點數
python3 test/hardware_validation/monitor_motor_gui.py /dev/ttyUSB0 115200 200
```

### 9.3 增強版雙馬達監控器 (monitor_dual_motor_enhanced.py)

**特性：**
- 同時監控兩個馬達
- 軟體校正按鈕
- 連線狀態指示
- 自動重連
- 執行緒安全序列埠通訊
- "ovf" 溢位處理

**使用方式：**
```bash
python3 MGv2/src/monitor_dual_motor_enhanced.py
```

### 9.4 工具比較

| 特性 | 終端監控器 | GUI 監控器 | 增強版雙馬達 |
|-----|-----------|-----------|------------|
| 視覺化圖表 | ❌ | ✅ | ✅ |
| 歷史趨勢 | 有限 (20 筆) | ✅ (100+ 點) | ✅ |
| 多參數同時顯示 | ❌ | ✅ | ✅ |
| 縮放/平移 | ❌ | ✅ | ✅ |
| SSH 相容 | ✅ | ❌ | ❌ |
| 資源使用 | ✅ 極低 | 中等 | 中等 |
| 軟體校正 | ❌ | ❌ | ✅ |
| 雙馬達支援 | ❌ | ❌ | ✅ |

---

## 10. 疑難排解

### 10.1 WiFi 相關

#### ESP32 無法連接 WiFi
```
檢查：
1. SSID 完全一致：ExoPulse（區分大小寫）
2. 密碼正確：12345666
3. 頻段為 2.4GHz（ESP32 不支援 5GHz）
4. 行動熱點已開啟
```

#### PC 無法連接到 ESP32
```
檢查：
1. PC 和 ESP32 在同一 WiFi 網路
2. 從序列埠確認 ESP32 的 IP 位址
3. Windows 防火牆設定
```

### 10.2 CAN Bus 相關

#### MCP2515 初始化失敗
```bash
# 檢查清單：
1. 驗證所有 6 條 SPI 連線
2. 檢查 VCC 電壓（應為 3.3V 或 5V）
3. ⚠️ 重要：大部分 MCP2515 模組需要 5V，而非 3.3V！
4. 驗證 CS 引腳為 GPIO 5
5. 嘗試不同的 MCP2515 模組
6. 檢查晶振頻率（8MHz vs 16MHz）
```

#### CAN Bus 通訊無回應
```
檢查：
1. CAN_H 到 CAN_H 連接正確
2. CAN_L 到 CAN_L 連接正確
3. 共地（GND）
4. 兩端各有 120Ω 終端電阻（CAN_H 與 CAN_L 之間）
5. 馬達電源已開啟
6. 馬達 CAN ID 設定正確
```

### 10.3 序列埠相關

#### "Permission denied" 錯誤
```bash
sudo usermod -a -G dialout $USER
# 登出後重新登入
```

#### 連接埠找不到或改變
```bash
# 尋找當前連接埠
ls /dev/ttyUSB* /dev/ttyACM*

# 或監控核心訊息
dmesg -w
# 然後插拔裝置
```

#### 上傳失敗 "Failed to connect"
```bash
# 解決方案：
1. 上傳時按住 ESP32 的 BOOT 按鈕
2. 嘗試不同的 USB 線
3. 檢查 USB 連接埠運作：lsusb
4. 對板子斷電重啟
5. 在 platformio.ini 中嘗試較低的上傳速度
```

#### 無序列埠輸出
```bash
# 解決方案：
1. 驗證正確的鮑率：115200
2. 按 ESP32 的 RESET 按鈕
3. 嘗試不同的終端程式
4. 檢查 USB 線支援資料傳輸（非僅充電線）
```

### 10.4 GUI 監控器相關

#### GUI 視窗不顯示
```bash
# 檢查 X11 是否執行
echo $DISPLAY

# 如使用 SSH，連接時啟用 X forwarding
ssh -X user@host

# 或使用 VNC/遠端桌面

# 如在本機執行，嘗試
export DISPLAY=:0
```

#### 圖表凍結
```
檢查：
1. ESP32 是否正在傳送資料
2. 驗證序列埠連接：ls -l /dev/ttyUSB*
3. 檢查馬達電源和 CAN bus 連接
4. 重啟監控程式
```

#### "Device or resource busy"
```bash
# 序列埠已被使用，關閉其他程式

# 檢查誰在使用連接埠
lsof /dev/ttyUSB0

# 終止 screen 會話
pkill screen

# 或使用 fuser 識別並終止
fuser -k /dev/ttyUSB0
```

### 10.5 建置相關

#### "No module named 'matplotlib'"
```bash
pip install matplotlib
# 或
pip3 install matplotlib
# 或
sudo apt install python3-matplotlib
```

#### "No module named 'serial'"
```bash
pip install pyserial
# 或
pip3 install pyserial
```

---

## 附錄 A：快速參考命令

### 基本命令
```bash
# 設定
export PATH="$HOME/.local/bin:$PATH"
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 建置與上傳
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0

# 監控
pio device monitor --baud 115200 --port /dev/ttyUSB0

# 切換測試程式
cp examples/test_led.h src/main.cpp        # LED 測試
cp examples/test_loopback.h src/main.cpp   # Loopback 測試
git checkout src/main.cpp                   # 生產程式碼

# WiFi 測試
pio run -e wifi_test --target upload
screen /dev/ttyUSB0 115200

# 序列埠命令（在監控器中）
HELP
STATUS 1
DEBUG
STOP
CAL1    # 校正馬達 1
CAL2    # 校正馬達 2
CLEAR_CAL  # 清除校正
```

### 連接埠偵測
```bash
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -20
```

### 權限
```bash
sudo usermod -a -G dialout $USER
# 然後登出/登入
```

### Git 分支
```bash
# WiFi 開發分支
git checkout feature/esp32-wifi-connectivity

# 馬達控制主分支
git checkout master

# CAN bus 硬體驗證分支
git checkout feature/can-bus-hardware-validation
```

---

## 附錄 B：檔案索引

### WiFi 系統
- `src/WiFi_STA_Test.h` - WiFi 測試主程式
- `src/main_wifi_test.cpp` - WiFi 測試入口
- `src/test_client.ps1` - Windows 測試客戶端
- `WIFI_TEST_INSTRUCTIONS.md` - WiFi 測試完整指南

### MGv2 馬達系統
- `MGv2/src/main.cpp` - 雙馬達控制主程式
- `MGv2/include/ExoBus.h` - CAN 通訊核心類別
- `MGv2/include/SerialConsole.h` - 序列埠命令介面
- `MGv2/src/monitor_dual_motor_enhanced.py` - 增強版 GUI 監控
- `MGv2/README.md` - MGv2 系統文檔

### 測試與驗證
- `examples/test_led.h` - LED 硬體驗證
- `examples/test_loopback.h` - MCP2515 自我測試
- `examples/test_can_sender.cpp` - CAN 發送器測試
- `examples/test_can_receiver.h` - CAN 接收器測試
- `test/hardware_validation/` - 硬體驗證測試程式

### 文檔
- `MGv2/BUILD_REPORT.md` - 建置驗證報告
- `MGv2/HANDOFF.md` - 專案交接文件
- `MGv2/HARDWARE_TEST_GUIDE.md` - 硬體測試指南
- `MGv2/DEPLOYMENT_CHECKLIST.md` - 部署檢查清單
- `MGv2/PROJECT_ANALYSIS_REPORT.md` - 程式碼分析報告
- `MGv2/QUICK_START_UBUNTU.md` - Ubuntu 快速開始
- `docs/MOTOR_MONITORING.md` - 馬達監控工具
- `docs/HARDWARE_TESTING.md` - 硬體測試程序
- `docs/KNOWN_ISSUES.md` - 已知問題

### 配置
- `platformio.ini` - PlatformIO 建置配置

---

## 附錄 C：技術規格

### ESP32 規格
- 處理器：Xtensa 32-bit LX6 雙核心 @ 240MHz
- SRAM：520KB
- Flash：4MB (可擴展至 16MB)
- WiFi：802.11 b/g/n (2.4GHz)
- 藍牙：v4.2 BR/EDR 和 BLE
- GPIO：34 個
- SPI：4 組
- I2C：2 組
- UART：3 組

### CAN Bus 規格
- 協定：ISO 11898-1
- 波特率：500 KBPS (標準)，1 MBPS (選用)
- 拓撲：線性匯流排
- 最大節點：127 個
- 終端電阻：120Ω（兩端）

### MGv2 馬達規格
- 通訊：CAN Bus
- 控制模式：扭矩、速度、位置
- 回饋：溫度、電壓、電流、速度、編碼器、多圈角度、加速度
- 編碼器解析度：14-bit (16384 位置/圈)
- 扭矩範圍：±30 Nm
- 電流範圍：±33A

### WiFi 規格
- 頻段：2.4 GHz
- 模式：STA (Station)
- 加密：WPA2-PSK
- IP 分配：DHCP
- TCP Server Port：8888

---

## 附錄 D：聯絡與支援

### 文檔
- 本綜合文檔：`COMPREHENSIVE_DOCUMENTATION.md`
- WiFi 測試：`WIFI_TEST_INSTRUCTIONS.md`
- MGv2 系統：`MGv2/README.md`
- 硬體測試：`MGv2/HARDWARE_TEST_GUIDE.md`
- 建置報告：`MGv2/BUILD_REPORT.md`

### 資源
- PlatformIO 官方文件：https://docs.platformio.org/
- ESP32 技術參考：https://docs.espressif.com/
- Arduino Framework：https://www.arduino.cc/reference/
- FreeRTOS：https://www.freertos.org/

### 問題回報
- GitHub Issues（如適用）
- 查閱 `docs/KNOWN_ISSUES.md` 瞭解已知問題

---

**文檔版本：** 2.0
**建立日期：** 2025年11月20日
**最後更新：** 2025年11月20日
**維護者：** ExoPulse 開發團隊

---

**ExoPulse 外骨骼控制系統 - 完整技術文檔**
*整合 WiFi 無線通訊與 MGv2 馬達控制*
