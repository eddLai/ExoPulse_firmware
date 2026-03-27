# ExoPulse 控制系統資料流全圖

## 總覽架構

```
┌─────────────────────────────────────────────────────────────────┐
│                   AI Agent (depRL)                              │
│  (realtime_controller.py)                                       │
│  - RL policy inference                                          │
│  - Safety pipeline (clamp → filter → ramp)                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  HardwareBridge      │
                    │  (depRL)             │
                    │ - inject(obs)        │
                    │ - apply_action(a)    │
                    └─┬─────────────────┬──┘
                      │                 │
        ┌─────────────┴──┐         ┌────┴─────────┐
        ▼                ▼         ▼               ▼
    IMU Data        EMG Data    Motor Status   Zero Torque
    (I2C)          (BLE)       (CAN)           Command
     (125D         (EMG         (Motor         (Safety)
      obs          features)    feedback)
      slice)
```

## 層級詳解

### 第 1 層：硬體感測器 (Hardware Sensors)

#### 1.1 IMU (MPU6050) via I2C
- **位置：** `/dev/i2c-7` @ 0x68
- **資料：** 加速度計 (3軸) + 陀螺儀 (3軸) + 四元數姿態 + 溫度
- **連接：** Jetson Orin 40-pin header (Pin 3=SDA, Pin 5=SCL)
- **驅動層：** `sensing/imu/mpu6050.h/cpp`
  - 核心功能：
    - 陀螺儀零偏自動校準 (開機時 500 筆樣本)
    - Madgwick AHRS 濾波 (四元數姿態估計)
    - 可配置採樣率 (4~1000 Hz)
    - 時間戳 (CLOCK_MONOTONIC)
  - 主要 API：
    - `mpu6050_create()` / `mpu6050_init_config()` / `mpu6050_read()`
    - `mpu6050_get_quaternion()` / `mpu6050_reset_quaternion()`
    - `quat_to_euler()` (四元數轉歐拉角)
- **應用層：**
  - 測試：`module_test/imu_sensor/mpu6050_reader.cpp` (簡易讀取)
  - 實時 UI：`sensor_test/imu_sensor/imu_realtime_ui.py` (Python GUI)
  - 集成：`depRL/env_wrappers/hardware_bridge.py` → 注入到 125D obs 向量

#### 1.2 EMG (uMyo BLE)
- **位置：** BLE 無線連接
- **資料：** EMG 信號向量
- **驅動：** `sensing/emg/ble_emg_module.py`
- **可選：** config 中 `enable_emg: false` 時不使用

#### 1.3 Motor (LK-TECH MGv2) via CAN
- **位置：** MCP2515 SPI @ `/dev/spidev0.0` (SPI0)
- **CAN ID：** 0x141 (右髖), 0x142 (左髖)
- **資料返回：** 溫度、扭力電流、速度、編碼器角度
- **驅動：** `control/motor/lktech_motor.h/cpp`
  - 基礎層：`mcp2515.h/cpp`
  - 應用層：`lktech_motor.h/cpp`

---

### 第 2 層：觀測向量構建 (Observation Construction)

**檔案：** `depRL/env_wrappers/hardware_bridge.py` → `HardwareBridge.inject(obs)`

```c
obs[0..2]     ← IMU accel (x, y, z)
obs[3..5]     ← IMU gyro (x, y, z)
obs[6..N]     ← EMG features (if enable_emg)
obs[...]      ← Motor status (if enable_motor)
              ← ... 共 125D
```

**資料來源流：**
```
IMU.readAccelGyro()
EMG.read_power_spectrum()
Motor.read_status()
    ↓
HardwareBridge.inject()
    ↓
125D observation vector (obs)
```

---

### 第 3 層：RL 推理 (Inference)

**檔案：** `control/ai_agent/realtime_controller.py` → `RealtimeController._control_loop()`

**步驟：**
```
obs (125D)
    ↓
agent.test_step(obs)  ← 或 noisy_test_step()
    ↓
action (20D)
    ├─ action[0..17]  = 人體肌肉動作
    └─ action[18..19] = 外骨骼動作 (右髖, 左髖)
```

**計時：**
- 控制迴圈頻率：`control_freq_hz` (預設 50 Hz)
- 推理時間：記錄在 telemetry (`inference_ms`)

---

### 第 4 層：安全管線 (Safety Pipeline)

**檔案：** `control/ai_agent/realtime_controller.py` → `TorqueSafetyPipeline`

**三級保護（順序執行）：**

#### 4.1 馬達保護 (MotorProtection)

1. **扭力限制 (IQ Clamp)**
   ```python
   # action → [-max_action, +max_action]
   max_action = iq_limit / external_scale
   # iq_limit = 800 (A·100)，external_scale = 45
   # max_action ≈ 17.8
   ```

2. **平滑濾波器**
   - EMA (指數移動平均)：`alpha = 0.3` (預設)
   - 或 Butterworth 低通：`cutoff_freq = 5 Hz`

3. **扭力爬升 (Torque Ramp)**
   ```python
   # 開機時從 0 → 100% 線性升高
   ramp = elapsed / torque_ramp_seconds
   action[exo_idx] *= ramp
   # torque_ramp_seconds = 2.0 (預設)
   ```

#### 4.2 人體保護 (HumanProtection) — 留空
- `torque_rate_limit()` — TODO：限制 dτ/dt
- `biomechanical_limit()` — TODO：關節角度依存上限
- `contact_force_check()` — TODO：接觸力回饋

---

### 第 5 層：速率限制 (Rate Limiting)

**檔案：** `MotorRateLimiter` 類

**目的：** 控制迴圈 (50 Hz) 與 CAN 送速 (10 Hz) 的頻率匹配

```
多個推理結果 (每 20ms)
    ↓ (時間加權平均)
單一 CAN 命令 (每 100ms)
```

**計算：**
```python
weights[i] = t[i] - t[0] + epsilon
result = sum(weights[i] * action[i]) / sum(weights[i])
```

---

### 第 6 層：CAN 馬達命令 (Motor Command)

**檔案：** `HardwareBridge.apply_action()` → `motor_set_torque()`

**步驟：**
1. 將 action[18..19] 轉換為電流命令 (iq)：
   ```python
   iq_right = action[18] * external_scale
   iq_left = action[19] * external_scale
   # 最終再次限制在 [-iq_limit, +iq_limit]
   ```

2. 發送 CAN 幀（0xA1 扭力閉迴路命令）
   ```c
   TX: [0xA1] [00] [00] [00] [iq_L] [iq_H] [00] [00]
   RX: [0xA1] [temp] [iq] [iq] [speed_L] [speed_H] [enc_L] [enc_H]
   ```

3. 馬達執行並返回狀態 (temperature, torque_current, speed, encoder)

---

## 調用鏈路總結

```
GUI / realtime_controller.py (CLI)
    ↓
RealtimeController.initialize()
    ├─ Load depRL agent
    ├─ Init HardwareBridge
    │   ├─ I2C.init("/dev/i2c-7", 0x68)  [MPU6050]
    │   ├─ BLE.init()                      [EMG, optional]
    │   └─ CAN.init("/dev/spidev0.0")      [MCP2515]
    │       ├─ mcp2515_create()
    │       ├─ mcp2515_init(1000000 baud)
    │       ├─ motor_create(0x141)
    │       └─ motor_create(0x142)
    └─ Setup TorqueSafetyPipeline
        └─ MotorProtection + HumanProtection + MotorRateLimiter

RealtimeController.start()
    └─ spawn _control_loop() thread @ 50 Hz
        ├─ HardwareBridge.inject(obs)
        │   ├─ IMU.read() → obs[0..5]
        │   ├─ EMG.read() → obs[6..X]
        │   └─ Motor.read_status() → obs[X..124]
        ├─ agent.test_step(obs) → action[20]
        ├─ TorqueSafetyPipeline.apply(action) → safe_action
        ├─ MotorRateLimiter.accumulate(safe_action)
        ├─ every 100ms: motor_action = MotorRateLimiter.maybe_send()
        │   └─ HardwareBridge.apply_action(motor_action)
        │       ├─ motor_set_torque(motor[0], iq_right)  @ 0x141
        │       └─ motor_set_torque(motor[1], iq_left)   @ 0x142
        └─ update telemetry (Hz, inference_ms, torques, errors)
```

---

## 重要 Pin 腳連接

### I2C (IMU) — `/dev/i2c-7`
| MPU6050 | Jetson 40-pin |
|---------|---------------|
| SDA | Pin 3 |
| SCL | Pin 5 |
| VCC | Pin 1 (3.3V) |
| GND | Pin 6 |

### SPI (CAN) — `/dev/spidev0.0`
| MCP2515 | Jetson 40-pin |
|---------|---------------|
| CS | Pin 24 |
| SCK | Pin 23 |
| MOSI | Pin 19 |
| MISO | Pin 21 |
| VCC | Pin 2/4 (5V) |
| GND | Pin 25 |

### CAN Bus
| Device | CAN ID |
|--------|--------|
| Motor 1 (Right Hip) | 0x141 |
| Motor 2 (Left Hip) | 0x142 |

---

## 配置檔案

**主配置：** `control/ai_agent/config.yaml`

```yaml
checkpoint_path: "/path/to/policy.pkl"
control_freq_hz: 50.0          # 控制迴圈
external_scale: 45.0            # action → iq 縮放
iq_limit: 800                   # 電流上限 (×100mA)
enable_imu: true
enable_emg: false
enable_motor: true
filter_type: "ema"              # "ema" 或 "butterworth"
ema_alpha: 0.3
butterworth_cutoff: 5.0
torque_ramp_seconds: 2.0
max_motor_freq_hz: 10.0         # CAN 送速
```

---

## 常見問題排查

### 觀測向量為零？
- ✓ 檢查 `HardwareBridge.inject()` 中的感測器初始化
- ✓ 確認 IMU 校準已完成（陀螺儀零偏）
- ✓ 驗證 CAN 馬達通訊

### 扭力命令未送出？
- ✓ 檢查 `MotorRateLimiter` 的時間邏輯
- ✓ 確認 `max_motor_freq_hz` 設定
- ✓ 驗證 SPI 設備存在 (`/dev/spidev0.0`)

### 馬達未回應？
- ✓ 檢查 CAN 線連接
- ✓ 驗證電源 (MCP2515 需要 5V)
- ✓ 使用 `module_test/canbus_moudle/mcp2515_canbus -m loopback` 測試

---

## 驅動層架構對比

### IMU 層級 (感測器 → 驅動 → 應用)
```
Hardware (I2C)
    ↓
[sensing/imu/mpu6050.h/cpp]  ← 驅動層（校準、Madgwick、配置）
    ↓
[depRL/env_wrappers/hardware_bridge.py]  ← 應用層（注入 obs）
    ↓
RealtimeController (125D obs)
```

### CAN Motor 層級 (感測器 → 驅動 → 應用)
```
Hardware (CAN via SPI)
    ↓
[control/motor/mcp2515.h/cpp]  ← 底層驅動（SPI、CAN 幀）
    ↓
[control/motor/lktech_motor.h/cpp]  ← 中層驅動（馬達協議、扭力/狀態）
    ↓
[HardwareBridge.apply_action()]  ← 應用層（ iq 命令）
    ↓
RealtimeController (馬達反饋)
```

## 代碼導引

| 需求 | 檔案位置 |
|------|---------|
| **IMU** |
| IMU 驅動層 | `sensing/imu/mpu6050.h/cpp` |
| IMU 校準配置 | `mpu6050_init_config(cfg)` (sample_rate, calibration_samples, madgwick_beta) |
| IMU 重置姿態 | `mpu6050_reset_quaternion()` |
| IMU 即時監控 | `sensor_test/imu_sensor/imu_realtime_ui.py` |
| **CAN Motor** |
| CAN 底層驅動 | `control/motor/mcp2515.h/cpp` |
| 馬達中層驅動 | `control/motor/lktech_motor.h/cpp` |
| **Control** |
| 修改安全參數 | `control/ai_agent/config.yaml` |
| 加入新感測器 | `depRL/env_wrappers/hardware_bridge.py` (inject) |
| 修改控制頻率 | `config.yaml` 或 CLI `--freq` |
| 調整扭力限制 | `config.yaml` (`iq_limit`, `external_scale`) |
| 新增人體保護 | `control/ai_agent/realtime_controller.py` (HumanProtection) |
