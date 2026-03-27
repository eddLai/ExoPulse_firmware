

# IMU Sensor Readers for Jetson Orin

此目錄包含多種 IMU 感測器的測試程式，支援 I2C 和 SPI 通訊協定。

## 支援的感測器

### ✅ MPU6050 (GY-521) - I2C - 已測試成功
- 6 軸 IMU：3 軸加速度計 + 3 軸陀螺儀
- I2C 介面
- 程式：`mpu6050_reader.cpp`

### 🔧 ADXL345 (GY-291) - SPI - 待測試
- 3 軸加速度計
- SPI 介面
- 程式：`adxl345_reader.cpp`

---

## MPU6050 (GY-521) 使用說明

### 硬體連接

將 MPU6050 (GY-521) 連接到 Jetson Orin 40-pin header：

| MPU6050 腳位 | Jetson Orin Pin | 功能 | 說明 |
|-------------|----------------|------|------|
| VCC | Pin 1 | 3.3V | 電源 |
| GND | Pin 6 | GND | 接地 |
| SCL | Pin 5 | I2C_SCL (I2C Bus 7) | 時鐘訊號 |
| SDA | Pin 3 | I2C_SDA (I2C Bus 7) | 資料訊號 |
| AD0 | GND | - | I2C 地址選擇（接 GND = 0x68） |

**重要：** Jetson Orin 40-pin header 的 Pin 3/5 對應到 **I2C Bus 7** (`/dev/i2c-7`)，不是 i2c-0 或 i2c-1！

**注意：** 請使用 3.3V 電源，不要使用 5V！

### 編譯與執行

```bash
cd /home/ntk/Documents/ExoPulse_firmware/sensor_test/imu_sensor

# 編譯
g++ -Wall -std=c++11 -o mpu6050_reader mpu6050_reader.cpp

# 執行（無需 sudo，使用者已在 i2c 群組）
./mpu6050_reader
```

### 輸出範例

```
=== MPU6050 (GY-521) IMU Reader for Jetson Orin ===
...
=== Scanning I2C Bus 7 ===
  Found device at 0x68 (WHO_AM_I: 0x68)

Trying /dev/i2c-7 at address 0x68...
WHO_AM_I: 0x68
MPU6050 detected!
✓ MPU6050 initialized successfully!

=== Reading IMU Data (Ctrl+C to stop) ===
Accel: X= 0.326g Y=-0.802g Z= 0.524g  |  Gyro: X= 34.595°/s Y= -3.282°/s Z=  0.023°/s
```

### 感測器規格

- **加速度計範圍：** ±2g（可調整至 ±4g, ±8g, ±16g）
- **陀螺儀範圍：** ±250°/s（可調整至 ±500°/s, ±1000°/s, ±2000°/s）
- **更新頻率：** 10 Hz
- **I2C 地址：** 0x68（AD0 = GND）或 0x69（AD0 = VCC）

---

## ADXL345 (GY-291) 使用說明

### 硬體連接

將 ADXL345 (GY-291) 連接到 Jetson Orin 40-pin header：

| ADXL345 腳位 | Jetson Orin Pin | 功能 | 說明 |
|-------------|----------------|------|------|
| CS | Pin 24 | SPI0_CS0* | 晶片選擇 |
| SCL (時鐘) | Pin 23 | SPI0_SCK | SPI 時鐘 |
| SDA (MOSI) | Pin 19 | SPI0_MOSI | 主機輸出 |
| SDO (MISO) | Pin 21 | SPI0_MISO | 主機輸入 |
| VCC | Pin 1 或 17 | 3.3V | 電源 |
| GND | Pin 6, 9, 14, 20, 25, 30, 34, 39 | GND | 接地 |

**注意：**
- 請使用 3.3V 電源，不要使用 5V！
- GY-291 模組預設為 I2C 模式，需要特殊設定才能切換到 SPI 模式

### 先決條件

#### 1. 啟用 SPI

在 Jetson Orin 上啟用 SPI 介面：

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

在選單中：
1. 選擇 "Configure Jetson 40-pin Header"
2. 選擇 "spi1" 或 "spi0"
3. 選擇 "Save pin changes"
4. 重新啟動系統

#### 2. 驗證 SPI 已啟用

```bash
ls /dev/spidev*
```

應該會看到 `/dev/spidev0.0`、`/dev/spidev0.1` 等設備。

### 編譯與執行

```bash
cd /home/ntk/Documents/ExoPulse_firmware/sensor_test/imu_sensor

# 編譯
make

# 執行
./adxl345_reader
```

### 輸出範例

```
=== ADXL345 IMU Reader for Jetson Orin ===
SPI Pin Connections:
  CS   -> Pin 24 (SPI0_CS0)
  SCL  -> Pin 23 (SPI0_SCK)
  SDA  -> Pin 19 (SPI0_MOSI)
  SDO  -> Pin 21 (SPI0_MISO)
===========================================

ADXL345 detected! Device ID: 0xe5

Reading accelerometer data (Ctrl+C to stop)...

X:   0.012 g  Y:  -0.004 g  Z:   0.996 g
```

---

## 通用工具

### I2C 掃描

掃描所有 I2C 總線以尋找連接的設備：

```bash
# 掃描特定總線（例如 bus 7）
i2cdetect -y -r 7

# 掃描所有常用總線
for i in 0 1 2 7 8; do echo "=== I2C Bus $i ==="; i2cdetect -y -r $i; done
```

### SPI 測試

使用提供的 SPI 掃描工具測試所有 SPI 設備：

```bash
g++ -Wall -std=c++11 -o spi_scan spi_scan.cpp
./spi_scan
```

## 疑難排解

### MPU6050 問題

#### 錯誤：找不到 MPU6050

```
MPU6050 not found. Expected 0x68 or 0x72, got 0x0
```

**檢查：**
1. 確認 VCC 接到 3.3V（Pin 1）
2. 確認 GND 接好（Pin 6）
3. 確認 SDA 接到 Pin 3，SCL 接到 Pin 5
4. 使用三用電表檢查模組的 VCC 是否有 3.3V
5. 掃描 I2C 總線：`i2cdetect -y -r 7`

#### 重要提醒

- Jetson Orin 40-pin header 的 I2C 對應到 **Bus 7**，不是 Bus 0 或 Bus 1
- 如果使用者在 `i2c` 群組中，不需要 sudo 權限

### ADXL345 問題

#### 錯誤：無法開啟 SPI 設備

```
Failed to open SPI device: /dev/spidev0.0
```

**原因：** SPI 控制器已啟用，但 `spidev` 內核驅動模組未加載，導致 `/dev/spidev*` 設備節點不存在。

**解決方法：**

```bash
# 加載 spidev 模組
sudo modprobe spidev

# 或使用 insmod（如果 modprobe 失敗）
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/spi/spidev.ko

# 驗證設備已建立
ls -la /dev/spidev*
```

**詳細 Debug 步驟：** 參見 `../canbus_moudle/README.md` 的「Failed to open SPI device」章節

**永久解決（開機自動加載）：**

```bash
echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
```

#### 錯誤：找不到 ADXL345

```
ADXL345 not found. Expected 0xe5, got 0x0
```

**檢查順序：**

1. **先確認 SPI 設備已建立**
   ```bash
   ls -la /dev/spidev*
   # 如果沒有，參見上方「無法開啟 SPI 設備」的解決方法
   ```

2. **驗證硬體接線**
   - 確認所有接線連接正確（特別是 CS/MOSI/MISO/SCK）
   - 確認 ADXL345 有接 3.3V 電源（不要接 5V！）
   - 檢查 GND 連接
   - 使用三用電表確認電源電壓

3. **檢查 GY-291 模組模式**
   - GY-291 模組預設為 **I2C 模式**
   - 需要特殊設定或焊接才能切換到 SPI 模式
   - 如果是預設 I2C 模式的模組，改用 I2C 接線或更換支持 SPI 的模組

4. **測試 SPI 通訊**
   ```bash
   # 執行 spi_scan 工具測試
   ./spi_scan
   ```

## 參考資料

### MPU6050
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [GY-521 Library](https://github.com/RobTillaart/GY521)

### ADXL345
- [SparkFun ADXL345 Arduino Library](https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library)
- [ADXL345 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf)

### Jetson Orin
- [Jetson Orin Nano GPIO Pinout](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/)
- [Jetson GPIO Library](https://github.com/NVIDIA/jetson-gpio)
