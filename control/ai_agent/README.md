# ExoPulse AI Agent — Orin 邊緣推論控制器

在 Jetson Orin 上運行外骨骼即時控制迴圈，不需要 SCONE 模擬器。

> **第一次使用？** 請先看「[Checkpoint 轉換](#checkpoint-轉換)」將 .pt 權重轉成 .npz，之後才能在不需要 torch 的環境下運行。
>
> **每次開機後？** 需要先啟用 SPI：`sudo modprobe spidev`，詳見「[硬體連接](#硬體連接)」。

## 架構

```
MPU6050 IMU (I2C) → Expert 3 神經網路 (numpy) → MCP2515 CAN → LK-TECH 馬達
```

## 依賴

只需要 **numpy** 和 **pyyaml**（系統 Python 內建），不需要 conda 環境。

| 套件 | 用途 | 必要？ |
|------|------|--------|
| numpy | 神經網路推論、數據處理 | 必要 |
| pyyaml | 讀取 config.yaml | 必要 |
| scipy | Butterworth 濾波器 | 選用（預設用 EMA 不需要） |
| torch | 一次性 .pt → .npz 轉換 | 只需要一次 |

## 快速開始

```bash
cd /home/ntk/Documents/depRL/firmware_layer/control/ai_agent
python realtime_controller.py --config config.yaml
```

## Checkpoint 轉換

神經網路權重需要從 PyTorch 格式 (.pt) 轉成 numpy 格式 (.npz)。**只需要做一次**。

### 方法一：用轉換腳本

```bash
conda activate forward_sim
python convert_checkpoint.py /path/to/checkpoints/expert_3/step_2300000.pt
```

### 方法二：程式自動轉換

在有 torch 的環境下運行 `realtime_controller.py`，程式會自動偵測 `.npz` 不存在並轉換：

```bash
conda activate forward_sim
python realtime_controller.py --config config.yaml
# 第一次會自動轉換，之後不需要 forward_sim
```

轉換後的檔案結構：
```
checkpoints/expert_3/
├── step_2300000.pt    ← 原始檔（需要 torch 讀取）
└── step_2300000.npz   ← 轉換後（numpy 直接讀取）
```

## 硬體連接

| 感測器 | 介面 | Orin 腳位 |
|--------|------|-----------|
| MPU6050 IMU | I2C bus 7 | Pin 3 (SDA), Pin 5 (SCL) |
| MCP2515 CAN | SPI0 | Pin 19 (MOSI), Pin 21 (MISO), Pin 23 (SCK), Pin 24 (CS) |
| LK-TECH 馬達 | CAN Bus | CANH/CANL（經 TJA1050 收發器） |

啟動前需確認 SPI 已啟用：
```bash
sudo modprobe spidev
ls /dev/spidev0.0  # 應該要存在
```

## config.yaml 設定

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `checkpoint_path` | — | checkpoint 資料夾路徑 |
| `control_freq_hz` | 50 | 控制迴圈頻率 (Hz) |
| `external_scale` | 45.0 | 動作 [-1,1] → 扭矩 (Nm) 的縮放 |
| `iq_limit` | 800 | 電流安全上限 (~12.8A) |
| `filter_type` | "ema" | 濾波器類型："ema" 或 "butterworth" |
| `ema_alpha` | 0.3 | EMA 平滑係數（越小越平滑） |
| `butterworth_cutoff` | 5.0 | Butterworth 截止頻率 (Hz) |
| `torque_ramp_seconds` | 2.0 | 啟動時扭矩從 0 線性增加到 100% 的時間 |
| `max_motor_freq_hz` | 10.0 | CAN 指令發送頻率上限 |

## 安全機制

1. **扭矩斜坡** — 啟動後 2 秒內扭矩從 0 線性增加
2. **平滑濾波** — EMA 或 Butterworth 防止扭矩突變
3. **電流限制** — iq 限制在 ±800（~12.8A）
4. **速率限制** — CAN 指令最高 10Hz，中間用時間加權平均
5. **Ctrl+C** — 立即停止所有馬達

## 神經網路

Expert 3（scheme_b_imu）：純 numpy 前向傳播

```
輸入 (11D):  IMU 四元數(4) + 角速度(3) + 馬達位置(2) + 馬達速度(2)
網路:        11 → 128 → 128 → 64 → 2（ReLU + Tanh）
輸出 (2D):   左右髖關節扭矩 [-1, +1]，乘以 external_scale 得到 Nm
```

## 檔案結構

```
control/ai_agent/
├── realtime_controller.py   ← 主程式（控制迴圈 + 神經網路推論）
├── config.yaml              ← 設定檔
├── convert_checkpoint.py    ← .pt → .npz 一次性轉換工具
└── README.md                ← 本文件
```
