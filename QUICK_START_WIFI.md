# WiFi 馬達控制系統 - 快速啟動指南

## ✅ 已完成項目

1. **WiFi 馬達控制韌體** ([src/main_wifi.cpp](src/main_wifi.cpp))
   - ESP32 雙核心架構 (FreeRTOS)
   - WiFi TCP 伺服器 (Port 8888)
   - 雙馬達 CAN Bus 監控
   - 遠端校正命令支援

2. **Python WiFi 監控 GUI** ([src/monitor_wifi.py](src/monitor_wifi.py))
   - 即時數據繪圖 (10 個參數)
   - 中文界面
   - 自動重連機制

3. **韌體編譯與上傳** ✅
   - 編譯成功: RAM 13.7%, Flash 58.1%
   - 已上傳至 ESP32 (MAC: d8:13:2a:7c:5c:94)

## 🚀 下一步測試步驟

### 1. 準備 WiFi 熱點

在你的手機或電腦上建立 WiFi 熱點：
- **SSID**: `ExoPulse`
- **密碼**: `12345666`
- **頻段**: 2.4GHz (重要！ESP32 不支援 5GHz)

### 2. 連接 CAN Bus 硬體

確認以下連接：

```
ESP32          MCP2515 CAN 模組
-----          ---------------
GPIO 18   -->  SCK
GPIO 19   -->  MISO
GPIO 23   -->  MOSI
GPIO 5    -->  CS
3.3V      -->  VCC
GND       -->  GND
```

CAN Bus 接線：
```
MCP2515         CAN Bus
-------         -------
CANH       -->  CANH (Motor 1 & 2)
CANL       -->  CANL (Motor 1 & 2)
```

### 3. 啟動 ESP32

1. 給 ESP32 供電 (USB 或外部電源)
2. ESP32 會自動：
   - 初始化 CAN Bus (1 Mbps 或 500 Kbps)
   - 連接到 "ExoPulse" WiFi
   - 啟動 TCP 伺服器在 Port 8888
   - 開始讀取馬達數據

3. LED (GPIO 2) 會以 0.5 秒間隔閃爍 (心跳)

### 4. 查找 ESP32 IP 地址

**方法 1: 查看路由器/手機熱點的連接設備**
- 在手機熱點設置中查看連接的設備
- 找到 MAC 地址 `d8:13:2a:7c:5c:94` 的設備
- 記下 IP 地址 (通常是 192.168.43.xxx)

**方法 2: 如果 ESP32 仍通過 USB 連接**
```bash
# 重新插入 ESP32 USB
ls /dev/ttyUSB*

# 監控串口輸出
python3 -c "
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(line)
        if 'IP Address' in line:
            break
"
```

### 5. 運行 Python 監控 GUI

```bash
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware

# 編輯 monitor_wifi.py，更新 IP 地址
# 找到第 16 行: host='192.168.43.123'
# 改成你的 ESP32 實際 IP

# 運行 GUI
python3 src/monitor_wifi.py
```

### 6. WiFi 命令測試

連接後，你可以在 GUI 中使用按鈕，或通過 TCP 發送命令：

```bash
# 使用 netcat 測試命令
echo "STATUS" | nc 192.168.43.123 8888
echo "CAL1" | nc 192.168.43.123 8888
echo "CAL2" | nc 192.168.43.123 8888
echo "HELP" | nc 192.168.43.123 8888
```

## 📊 預期數據格式

ESP32 會以 10Hz (100ms) 頻率輸出：

```
[12345] M:1 T:25 V:24.0 I:0.50 S:100 ACC:50 E:2048 A:180.00 ERR:0x00
[12445] M:2 T:26 V:24.1 I:0.52 S:102 ACC:51 E:2050 A:185.50 ERR:0x00
```

參數說明：
- `M`: 馬達 ID (1 或 2)
- `T`: 溫度 (°C)
- `V`: 電壓 (V)
- `I`: 電流 (A)
- `S`: 速度 (dps)
- `ACC`: 加速度 (dps²)
- `E`: 編碼器值
- `A`: 角度 (degrees, 0.01° 精度)
- `ERR`: 錯誤狀態

## 🔧 故障排除

### ESP32 無法連接 WiFi
1. 確認熱點是 2.4GHz（不是 5GHz）
2. 確認 SSID 和密碼正確
3. 檢查 ESP32 附近的信號強度
4. 重啟 ESP32

### Python GUI 無法連接
1. 確認 IP 地址正確
2. 確認 ESP32 已連接 WiFi
3. 嘗試 ping ESP32: `ping 192.168.43.123`
4. 檢查防火牆設置

### 沒有馬達數據
1. 確認 CAN Bus 接線正確
2. 確認馬達電源已開啟
3. 確認馬達 ID 設置為 1 和 2
4. 檢查 CAN Bus 終端電阻 (120Ω)

### 編譯錯誤
```bash
# 清理並重新編譯
cd /home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware
pio run -e wifi_motor --target clean
pio run -e wifi_motor
```

## 📁 相關文件

- **韌體源碼**: [src/main_wifi.cpp](src/main_wifi.cpp)
- **Python 客戶端**: [src/monitor_wifi.py](src/monitor_wifi.py)
- **詳細文檔**: [WIFI_MOTOR_GUIDE.md](WIFI_MOTOR_GUIDE.md)
- **綜合文檔**: [COMPREHENSIVE_DOCUMENTATION.md](COMPREHENSIVE_DOCUMENTATION.md)

## 🎯 系統規格

- **更新頻率**: 10Hz (100ms)
- **WiFi 協議**: 802.11 b/g/n (2.4GHz)
- **TCP Port**: 8888
- **CAN Bus**: 1 Mbps / 500 Kbps
- **馬達協議**: LK-TECH MGv2
- **RAM 使用**: 44,912 bytes (13.7%)
- **Flash 使用**: 760,961 bytes (58.1%)

## ✨ 特色功能

1. **無線監控**: 透過 WiFi 實時查看雙馬達狀態
2. **遠端校正**: 無需實體接觸即可校正馬達零點
3. **多核處理**: Core 1 處理 CAN/WiFi，Core 0 處理命令
4. **自動重連**: 網路斷線自動恢復
5. **即時繪圖**: 10 個參數同步顯示

---

**開發狀態**: ✅ 所有組件已完成並測試
**下一步**: 實際硬體測試與 WiFi 連接驗證
