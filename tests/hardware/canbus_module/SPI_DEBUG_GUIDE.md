# Jetson Orin SPI 配置調試指南

## 概述：發生了什麼？

### 症狀
執行 CAN bus 測試時出現錯誤：
```
Failed to open SPI device: /dev/spidev0.0
```

### 根本原因
**SPI 硬體已啟用，但驅動軟體尚未加載**

---

## 問題分析：分層理解

### 第 1 層：硬體層 ✅
- **Jetson Orin 的 SPI 控制器** 已通電並可用
- **MCP2515 模組** 已正確連接到 40-pin header
- **NVIDIA 硬體配置** 沒有問題

**驗證：**
```bash
ls /sys/devices/platform/3210000.spi/
# 結果：存在（硬體控制器可用）
```

---

### 第 2 層：設備樹層 ✅
- **執行 `jetson-io.py`** 後，SPI 被啟用到內核設備樹
- **設備樹編譯** 成功，SPI 設備被註冊到系統
- **spi0.0 和 spi0.1** 出現在 `/sys/devices` 中

**驗證：**
```bash
ls /sys/devices/platform/3210000.spi/spi_master/spi0/
# 結果：
# spi0.0    spi0.1    statistics    uevent    ...

cat /sys/devices/platform/3210000.spi/spi_master/spi0/spi0.0/modalias
# 結果：spi:tegra-spidev （表示需要 spidev 驅動）
```

---

### 第 3 層：驅動層 ❌（問題所在）

**問題：** `spidev` 內核驅動模組未加載

- 模組存在於系統中：`/lib/modules/5.10.192-tegra/kernel/drivers/spi/spidev.ko`
- 模組支持的設備：`spi:spidev`, `spi:tegra-spidev` 等
- **但模組未被自動加載**（可能是內核配置或啟動腳本未包含）

**驗證：**
```bash
lsmod | grep spidev
# 結果：（空白 = 未加載）

ls -la /dev/spidev*
# 結果：cannot access '/dev/spidev*': No such file or directory
```

**為什麼會這樣？**

- 設備樹知道「有一個 SPI 設備需要 tegra-spidev 驅動」
- 但內核沒有自動加載這個驅動（可能配置未啟用 CONFIG_SPI_SPIDEV）
- 沒有驅動 = 沒有 `/dev/spidev*` 設備節點
- 沒有設備節點 = 應用程式無法開啟 SPI

---

### 第 4 層：應用層 ❌（最終失敗）

```cpp
spi_fd = open(spi_device, O_RDWR);  // /dev/spidev0.0
if (spi_fd < 0) {
    std::cerr << "Failed to open SPI device" << std::endl;
    return false;
}
```

**結果：**
- `open()` 系統呼叫失敗
- CAN bus 初始化無法進行
- 程式退出

---

## 解決方案

### 立即解決（臨時）

```bash
# 加載 spidev 驅動模組
sudo modprobe spidev

# 驗證
ls -la /dev/spidev*
# 應該看到：
# crw-rw----+ 1 root gpio 153, 0 Mar 26 12:34 /dev/spidev0.0
# crw-rw----+ 1 root gpio 153, 1 Mar 26 12:34 /dev/spidev0.1
```

完成後可以執行 CAN bus 測試：
```bash
./mcp2515_canbus -m loopback
```

---

### 永久解決（重啟後自動加載）

**方法 A：編輯 `/etc/modules`**
```bash
echo "spidev" | sudo tee -a /etc/modules
```

**方法 B：建立新配置檔案（推薦）**
```bash
echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
```

**驗證配置**
```bash
cat /etc/modules-load.d/spidev.conf
# 應該看到：spidev
```

重啟系統後，SPI 設備將自動初始化：
```bash
sudo reboot
```

---

## 詳解：永久解決指令

### 🔍 指令分解：`echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf`

這個指令由四個部分組成，讓我逐一解釋：

#### 第 1 部分：`echo "spidev"`
**作用：** 輸出文本 "spidev"

```bash
$ echo "spidev"
# 結果：spidev（輸出到螢幕）
```

#### 第 2 部分：`|` （管道符號）
**作用：** 將前一個命令的輸出傳送給下一個命令作為輸入

```
echo "spidev"  →  輸出：spidev
                      ↓
                    管道 |
                      ↓
              tee 命令接收輸入
```

#### 第 3 部分：`sudo`
**作用：** 用管理員（root）權限執行後面的命令

- 因為 `/etc/modules-load.d/` 是系統目錄，需要管理員權限
- 不加 `sudo` 會得到 `Permission denied` 錯誤

#### 第 4 部分：`tee /etc/modules-load.d/spidev.conf`
**作用：** 將輸入內容同時寫入檔案和顯示到螢幕

```
輸入：spidev
  ↓
 tee 命令
  ├─→ 寫入檔案：/etc/modules-load.d/spidev.conf
  └─→ 顯示在螢幕
```

---

### 📊 完整流程圖

```
┌─────────────────┐
│  echo "spidev"  │  輸出：spidev
└────────┬────────┘
         │
         │ (管道 |)
         ↓
┌──────────────────────────────────────────────┐
│ sudo tee /etc/modules-load.d/spidev.conf     │
│ (用 root 權限寫入檔案 + 顯示在螢幕)           │
└────────┬─────────────────────────────────────┘
         │
    ┌────┴──────────┐
    ↓               ↓
[建立/更新檔案]  [螢幕顯示：spidev]
```

---

### 🎯 具體例子

#### 執行前
```bash
$ cat /etc/modules-load.d/spidev.conf
# 檔案不存在或為空
```

#### 執行命令
```bash
$ echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
spidev
# ↑ 螢幕上看到 spidev（tee 命令顯示的）
```

#### 執行後
```bash
$ cat /etc/modules-load.d/spidev.conf
spidev
# ↑ 檔案已建立，內容是 "spidev"
```

---

### ❓ 為什麼要用 `tee` 而不是其他方法？

#### ❌ 為什麼不能用簡單的重定向？
```bash
echo "spidev" > /etc/modules-load.d/spidev.conf
# 錯誤：Permission denied
# 原因：只有 echo 有 sudo 權限，但 > 操作沒有
```

#### ❌ 為什麼不能用這樣？
```bash
sudo echo "spidev" > /etc/modules-load.d/spidev.conf
# 錯誤：Permission denied
# 原因：sudo 只作用於 echo，不作用於 > 重定向
```

#### ✅ 為什麼 `sudo tee` 能work？
```bash
echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
# 正確！原因：整個 tee 命令都用 sudo 執行
# 所以既能寫檔案，也能顯示輸出
```

---

### 📌 不執行這個指令會怎樣？

#### 情況對比

| 情況 | 每次開機需要做的事 | 操作頻率 |
|------|------------------|--------|
| **沒執行指令** | 手動執行 `sudo modprobe spidev` | ⏱️ **每次開機都要做** |
| **執行了指令** | 什麼都不用做 | ⏱️ **只需執行一次** |

#### 時間成本

```
沒執行指令：
  開機 → Jetson 啟動 → 手動執行 modprobe → 等待 5 秒 → 才能執行 CAN 測試

  每次開機都要重複這個過程！

執行了指令：
  開機 → Jetson 自動加載 spidev → /dev/spidev* 自動出現 → 直接執行 CAN 測試

  完全自動化，無需任何手動操作！
```

---

### ✨ 指令執行後會發生什麼？

1. **立即發生：**
   ```bash
   $ echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
   spidev  # ← 螢幕顯示

   # 檔案已建立
   $ cat /etc/modules-load.d/spidev.conf
   spidev
   ```

2. **下次開機時（自動）：**
   ```
   Jetson 啟動
     ↓
   內核掃描 /etc/modules-load.d/*.conf
     ↓
   發現 spidev.conf 中的 "spidev"
     ↓
   自動加載 spidev 驅動
     ↓
   /dev/spidev0.0, /dev/spidev0.1 自動出現
     ↓
   應用程式無需任何修改，直接可以使用 SPI
   ```

---

### 📋 其他等價的寫法

#### 寫法 1（最推薦）
```bash
echo "spidev" | sudo tee /etc/modules-load.d/spidev.conf
```

#### 寫法 2（使用 heredoc 多行）
```bash
sudo tee /etc/modules-load.d/spidev.conf << EOF
spidev
EOF
```

#### 寫法 3（直接編輯，需要互動）
```bash
sudo nano /etc/modules-load.d/spidev.conf
# 手動輸入 spidev
# 按 Ctrl+X → Y → Enter 保存退出
```

#### 寫法 4（使用 bash -c）
```bash
sudo bash -c 'echo "spidev" > /etc/modules-load.d/spidev.conf'
```

---

### 🔍 驗證配置是否成功

#### 驗證檔案已建立
```bash
cat /etc/modules-load.d/spidev.conf
# 應該看到：spidev
```

#### 驗證檔案權限
```bash
ls -la /etc/modules-load.d/spidev.conf
# 應該看到：-rw-r--r-- 1 root root ... spidev.conf
```

#### 重啟後驗證驅動已加載
```bash
sudo reboot
# 重啟後執行：

lsmod | grep spidev
# 應該看到：spidev

ls -la /dev/spidev*
# 應該看到：/dev/spidev0.0, /dev/spidev0.1
```

---

## 完整 Debug 檢查清單

```bash
# 1️⃣ 確認 SPI 控制器已啟用
echo "=== Check SPI Controller ==="
ls /sys/devices/platform/3210000.spi/
# 期望：目錄存在

# 2️⃣ 確認設備樹中的 SPI 設備
echo "=== Check Device Tree Devices ==="
ls /sys/devices/platform/3210000.spi/spi_master/spi0/
# 期望：spi0.0, spi0.1 等

# 3️⃣ 確認設備 modalias
echo "=== Check Device Modalias ==="
cat /sys/devices/platform/3210000.spi/spi_master/spi0/spi0.0/modalias
# 期望：spi:tegra-spidev

# 4️⃣ 檢查 spidev 模組是否已加載
echo "=== Check Loaded Modules ==="
lsmod | grep spidev
# 期望：spidev（如果為空，需要加載）

# 5️⃣ 檢查 spidev 模組是否存在
echo "=== Check Module File ==="
modinfo spidev | head -5
# 期望：輸出模組信息

# 6️⃣ 檢查使用者群組
echo "=== Check User Groups ==="
groups
# 期望：包含 gpio

# 7️⃣ 如果上面都 OK，現在加載模組
echo "=== Load spidev Module ==="
sudo modprobe spidev

# 8️⃣ 驗證設備節點已建立
echo "=== Check Device Nodes ==="
ls -la /dev/spidev*
# 期望：/dev/spidev0.0, /dev/spidev0.1 (crw-rw---- root gpio)

# 9️⃣ 測試設備訪問權限
echo "=== Test Device Access ==="
cat /dev/spidev0.0 &>/dev/null && echo "✓ Can read spidev0.0" || echo "✗ Permission denied"
```

---

## 內核配置驗證

Jetson Orin 當前內核配置已包含 spidev：

```bash
# 驗證內核中 SPI_SPIDEV 已啟用
grep "CONFIG_SPI_SPIDEV" /boot/config-$(uname -r)
# 期望：CONFIG_SPI_SPIDEV=m （m 表示可加載模組）
```

---

## 為什麼不自動加載？

可能的原因：

1. **模組配置未包含在啟動腳本**
   - Jetson 自定義啟動流程
   - `/etc/modules-load.d/` 中沒有 spidev 條目

2. **內核啟動參數**
   - 某些內核啟動參數禁用了自動加載

3. **設計決策**
   - NVIDIA 可能預留給用戶手動加載
   - 避免不使用 SPI 的系統額外加載驅動

---

## 分層架構圖

```
應用層 (Application Layer)
  ↓ open("/dev/spidev0.0")
設備節點層 (Device Node Layer)  ← ❌ 失敗：/dev/spidev* 不存在
  ↓
驅動層 (Driver Layer)  ← ❌ spidev 未加載
  ↓
設備樹層 (Device Tree Layer)  ✅ spi0.0, spi0.1 已註冊
  ↓
硬體層 (Hardware Layer)  ✅ SPI 控制器已啟用
```

**解決方法：** 在驅動層加載 spidev 模組
```bash
sudo modprobe spidev
```

---

## 總結：三句話說完

1. **發生了什麼：** SPI 硬體和設備樹都已啟用，但內核的 spidev 驅動未加載
2. **為什麼發生：** Jetson 在啟動時沒有自動加載 spidev 模組
3. **怎麼解決：** 執行 `sudo modprobe spidev` 加載驅動，或編輯 `/etc/modules-load.d/spidev.conf` 實現開機自動加載

---

## 快速參考表

| 問題 | 命令 | 說明 |
|------|------|------|
| 檢查 SPI 硬體 | `ls /sys/devices/platform/3210000.spi/` | 確認硬體存在 |
| 檢查設備樹 | `ls /sys/devices/platform/3210000.spi/spi_master/spi0/` | 確認設備已註冊 |
| 檢查驅動加載 | `lsmod \| grep spidev` | 確認驅動已加載 |
| 檢查設備節點 | `ls /dev/spidev*` | 確認設備節點存在 |
| 加載驅動 | `sudo modprobe spidev` | 立即加載驅動 |
| 永久加載 | `echo "spidev" \| sudo tee /etc/modules-load.d/spidev.conf` | 開機自動加載 |

---

## 相關資源

- **MCP2515 README：** `README.md` - 完整的 CAN bus 使用指南
- **IMU Sensor README：** `../imu_sensor/README.md` - ADXL345 SPI 配置
- **Jetson 官方文檔：** [Jetson GPIO](https://github.com/NVIDIA/jetson-gpio)
- **內核驅動源碼：** `/lib/modules/$(uname -r)/kernel/drivers/spi/spidev.ko`

