# MCP2515 CAN Bus Module Test for Jetson Orin

使用 MCP2515 + TJA1050 CAN 收發器模組，透過 SPI 介面連接 Jetson Orin，
實現 CAN Bus 通訊。此測試程式參考 ExoPulse ESP32 MGv2 專案的 CAN Bus 實作。

## 硬體需求

- Jetson Orin (已啟用 SPI)
- MCP2515 + TJA1050 CAN Bus 模組
- CAN Bus 設備 (如 LK-TECH 馬達) 或另一個 MCP2515 模組

## 硬體連接

### MCP2515 模組 → Jetson Orin 40-pin Header

| MCP2515 接腳 | Orin 腳位 | 功能說明 |
|:------------|:----------|:--------|
| CS | Pin 24 | 這是 Chip Select，不是 GND |
| GND | Pin 25 (或 Pin 6, 9) | 這才是接地 |
| SCK | Pin 23 | SPI 時脈 |
| SI (MOSI) | Pin 19 | 數據輸出至模組 |
| SO (MISO) | Pin 21 | 模組數據輸入 Orin |
| VCC | Pin 2 或 Pin 4 | 5V 電源 |

**重要：**
- MCP2515 模組使用 **5V** 電源（不是 3.3V！）
- Pin 24 是 **CS (Chip Select)**，千萬不要接到 GND
- CAN Bus 兩端需要 **120 ohm 終端電阻**（大多數模組已內建）

### CAN Bus 連接

```
MCP2515 模組          CAN Bus 設備
  CAN_H  ──────────── CAN_H
  CAN_L  ──────────── CAN_L
  GND    ──────────── GND (建議共地)
```

## 先決條件

### 1. 啟用 SPI

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

選單中：
1. 選擇 "Configure Jetson 40-pin Header"
2. 選擇 "spi1" 或 "spi0"
3. 選擇 "Save pin changes"
4. 重新啟動系統

### 2. 驗證 SPI 已啟用

```bash
ls /dev/spidev*
# 應該看到 /dev/spidev0.0, /dev/spidev0.1 等
```

## 編譯與執行

```bash
cd /home/ntk/Documents/ExoPulse_firmware/module_test/canbus_moudle

# 編譯
make

# 執行接收模式（預設 1Mbps）
make run

# 或直接執行
./mcp2515_canbus
```

## 使用方式

### 自我測試 (Loopback) — 不需要 CAN Bus 設備

```bash
./mcp2515_canbus -m loopback
```

這會使用 MCP2515 的 loopback 模式，自己發送自己接收，用來驗證 SPI 通訊和 MCP2515 是否正常。

### 接收 CAN 訊息

```bash
# 接收所有 CAN 訊息（1Mbps，對應 ESP32 MGv2 馬達）
./mcp2515_canbus

# 接收 500Kbps
./mcp2515_canbus -b 500000

# 使用不同 SPI 設備
./mcp2515_canbus -d /dev/spidev0.1
```

### 發送 CAN 訊息

```bash
# 發送測試訊息到 ID 0x100
./mcp2515_canbus -m send

# 發送到指定 CAN ID（例如 LK-TECH 馬達 ID 0x141）
./mcp2515_canbus -m send -i 0x141
```

### 完整參數

```
Usage: ./mcp2515_canbus [OPTIONS]

Options:
  -d <device>   SPI device (default: /dev/spidev0.0)
  -b <baud>     CAN baud rate: 1000000, 500000, 250000, 125000, 100000
                (default: 1000000)
  -m <mode>     Mode: receive, send, loopback (default: receive)
  -i <id>       CAN ID for send mode (hex, default: 0x100)
  -h            Show help
```

## 輸出範例

### Loopback 自我測試

```
=== MCP2515 CAN Bus Module Test for Jetson Orin ===
[1] Opening SPI device...
  SPI device opened: /dev/spidev0.0
[2] Resetting MCP2515...
  MCP2515 reset OK (in config mode)
[3] Configuring CAN baud rate...
  Setting CAN baud rate: 1Mbps (8MHz crystal)
[4] Setting LOOPBACK mode (self-test)...

Test  1/10: TX ID=0x100 -> RX OK (ID=0x100 LEN=8) PASS
Test  2/10: TX ID=0x101 -> RX OK (ID=0x101 LEN=8) PASS
...
Test 10/10: TX ID=0x109 -> RX OK (ID=0x109 LEN=8) PASS

=== Loopback Test Results ===
Passed: 10/10
Failed: 0/10
MCP2515 is working correctly!
```

### 接收模式

```
[RX] #    1 @     123ms ID=0x141 LEN=8 Data: 90 00 e8 03 00 00 00 4e
[RX] #    2 @     223ms ID=0x142 LEN=8 Data: 90 00 f2 03 00 00 00 52
```

## 與 ESP32 MGv2 配合使用

此測試程式的 CAN Bus 協議與 ESP32 MGv2 專案相容：

- **發送格式：** 與 `test/hardware_validation/test_can_sender_fixed.cpp` 相同
- **CAN Baud Rate：** 1Mbps（LK-TECH 馬達標準）
- **CAN ID：** 0x141 (Motor 1), 0x142 (Motor 2)

### 典型使用場景

1. **Orin 接收 ESP32 發送的馬達數據**
   ```bash
   ./mcp2515_canbus -b 1000000
   ```

2. **Orin 直接控制 LK-TECH 馬達**
   ```bash
   ./mcp2515_canbus -m send -i 0x141
   ```

## 疑難排解

### MCP2515 初始化失敗

```
MCP2515 not in config mode after reset!
```

**檢查：**
1. SPI 接線是否正確（特別注意 CS 是 Pin 24，不是接地）
2. VCC 是否接到 5V
3. SPI 是否已啟用：`ls /dev/spidev*`
4. 使用者是否有 SPI 權限：`groups` 應包含 `gpio` 或 `spi`

### TX 失敗 / 高錯誤率

**檢查：**
1. CAN_H 和 CAN_L 接線
2. 120 ohm 終端電阻（CAN Bus 兩端各一個）
3. 對方設備是否已開啟
4. Baud rate 是否匹配

### 權限問題

```bash
# 如果出現 permission denied
sudo chmod 666 /dev/spidev0.0

# 或加入 gpio 群組
sudo usermod -aG gpio $USER
# 重新登入
```

## 參考資料

- [MCP2515 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf)
- [TJA1050 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1050.pdf)
- ExoPulse MGv2 ESP32 CAN 實作：`MGv2/src/main.cpp`, `MGv2/include/config.h`
- ExoPulse CAN 測試：`MGv2/test/hardware_validation/test_can_sender_fixed.cpp`