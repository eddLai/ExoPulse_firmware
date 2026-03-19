# Sensing Modules API Reference

## IMU Module (`imu/`)

### Hardware
- **Sensor:** MPU6050 (GY-521) 6-axis IMU
- **Interface:** I2C (Jetson Orin bus 7)
- **Address:** 0x68 (AD0=GND) or 0x69 (AD0=VCC)
- **Scale:** Accel +/-2g (16384 LSB/g), Gyro +/-250 deg/s (131 LSB/(deg/s))
- **Sample Rate:** Configurable 4~1000 Hz (default 200 Hz)
- **Attitude:** Madgwick AHRS quaternion from accel + gyro fusion

### C API — Sensor (`mpu6050.h`)

```c
#include "mpu6050.h"

// Data structure
typedef struct {
    float accel[3];    // [ax, ay, az] in g
    float gyro[3];     // [gx, gy, gz] in deg/s
    float quat[4];     // [qw, qx, qy, qz] orientation quaternion
    float temperature; // Celsius
    double timestamp;  // seconds (CLOCK_MONOTONIC)
} imu_data_t;

// Configuration (optional)
typedef struct {
    uint16_t sample_rate_hz;      // ODR: 4~1000 Hz (default 200)
    uint16_t calibration_samples; // gyro bias samples (default 500)
    float    madgwick_beta;       // filter gain (default 0.1)
} mpu6050_config_t;

// Lifecycle
mpu6050_t* mpu6050_create(const char* i2c_device, uint8_t address);
int         mpu6050_init(mpu6050_t* dev);                              // default config
int         mpu6050_init_config(mpu6050_t* dev, const mpu6050_config_t* cfg);
int         mpu6050_read(mpu6050_t* dev, imu_data_t* data);           // 0=OK
void        mpu6050_destroy(mpu6050_t* dev);
void        mpu6050_default_config(mpu6050_config_t* cfg);
```

### C API — Quaternion (`mpu6050.h`)

```c
// Get current quaternion without reading sensor
int  mpu6050_get_quaternion(mpu6050_t* dev, float quat_out[4]);

// Reset quaternion to identity [1, 0, 0, 0]
int  mpu6050_reset_quaternion(mpu6050_t* dev);

// Convert quaternion to Euler angles (degrees)
void quat_to_euler(const float q[4], float* roll, float* pitch, float* yaw);
```

### Build

```bash
cd sensing/imu
make          # -> libexo_imu.so + test_mpu6050
./test_mpu6050                        # default: /dev/i2c-7, 0x68, 200 Hz
./test_mpu6050 /dev/i2c-7 0x68 100   # custom rate
```

### Python (ctypes)

```python
import ctypes

lib = ctypes.CDLL("./sensing/imu/libexo_imu.so")

class ImuData(ctypes.Structure):
    _fields_ = [
        ("accel", ctypes.c_float * 3),
        ("gyro", ctypes.c_float * 3),
        ("quat", ctypes.c_float * 4),
        ("temperature", ctypes.c_float),
        ("timestamp", ctypes.c_double),
    ]

class MpuConfig(ctypes.Structure):
    _fields_ = [
        ("sample_rate_hz", ctypes.c_uint16),
        ("calibration_samples", ctypes.c_uint16),
        ("madgwick_beta", ctypes.c_float),
    ]

lib.mpu6050_create.restype = ctypes.c_void_p
lib.mpu6050_create.argtypes = [ctypes.c_char_p, ctypes.c_uint8]
lib.mpu6050_init.restype = ctypes.c_int
lib.mpu6050_init.argtypes = [ctypes.c_void_p]
lib.mpu6050_init_config.restype = ctypes.c_int
lib.mpu6050_init_config.argtypes = [ctypes.c_void_p, ctypes.POINTER(MpuConfig)]
lib.mpu6050_read.restype = ctypes.c_int
lib.mpu6050_read.argtypes = [ctypes.c_void_p, ctypes.POINTER(ImuData)]
lib.mpu6050_destroy.argtypes = [ctypes.c_void_p]
lib.mpu6050_get_quaternion.restype = ctypes.c_int
lib.mpu6050_get_quaternion.argtypes = [ctypes.c_void_p, ctypes.c_float * 4]
lib.mpu6050_reset_quaternion.restype = ctypes.c_int
lib.mpu6050_reset_quaternion.argtypes = [ctypes.c_void_p]

dev = lib.mpu6050_create(b"/dev/i2c-7", 0x68)
lib.mpu6050_init(dev)

data = ImuData()
lib.mpu6050_read(dev, ctypes.byref(data))
print(f"accel: {list(data.accel)}")
print(f"gyro:  {list(data.gyro)}")
print(f"quat:  {list(data.quat)}")  # [qw, qx, qy, qz]

lib.mpu6050_destroy(dev)
```

---

## EMG Module (`emg/`)

### Hardware
- **Sensor:** EMG2ch_B x2 (ITRI EMG, 2 channels each)
- **Interface:** BLE (Bluetooth Low Energy)
- **Devices:**
  - Device 1 (ABE): packet header "ABE"
  - Device 2 (ABB): packet header "ABB"
- **Channels:** 4 total (Device1_T2, Device1_T4, Device2_T2, Device2_T4)
- **Output:** Raw ADC values or filtered envelope (via `emg_filter.py`)

### Python API (`ble_emg_module.py`)

```python
from ble_emg_module import BLEEMGModule
import numpy as np

emg = BLEEMGModule(scan_timeout=10.0, connect_timeout=20.0)

emg.init()          # -> bool (True if at least 1 device connected)
data = emg.read()   # -> np.ndarray(4,) [ABE_T2, ABE_T4, ABB_T2, ABB_T4]
batch = emg.read_batch(n_samples=7)  # -> np.ndarray(4, 7) last N samples
stats = emg.get_stats()              # -> dict with packet loss info
emg.close()
```

### Abstract Interface (`emg_interface.py`)

```python
from emg_interface import EMGInterface

class EMGInterface(ABC):
    def init(self) -> bool: ...
    def read(self) -> np.ndarray: ...  # shape (n_channels,)
    def close(self): ...
```

### Signal Processing (`emg_filter.py`)

Three-stage real-time pipeline:

| Stage | Operation | Parameters |
|-------|-----------|------------|
| 1 | High-pass filter | 30 Hz, 2nd-order Butterworth |
| 2 | Full-wave rectification | `abs()` |
| 3 | Low-pass filter | 6 Hz, 4th-order Butterworth |

```python
from emg_filter import EMGFilterPipeline, FilteredEMGModule
from ble_emg_module import BLEEMGModule

# Option A: Wrap the BLE module (same API, filtered output)
raw_emg = BLEEMGModule()
emg = FilteredEMGModule(raw_emg, fs=500.0)
emg.init()
envelope = emg.read()            # -> np.ndarray(4,) filtered
raw = emg.read_raw()             # -> np.ndarray(4,) bypass filter
emg.close()

# Option B: Standalone pipeline (process any numpy array)
pipeline = EMGFilterPipeline(n_channels=4, fs=500.0)
filtered = pipeline.process_batch(raw_batch)  # (4, N) -> (4, N)
pipeline.reset()                              # reset filter state
```

Filter parameters can be customized:

```python
EMGFilterPipeline(n_channels=4, fs=500.0,
                  hp_cutoff=30.0, hp_order=2,
                  lp_cutoff=6.0,  lp_order=4)
```

### Dependencies

```bash
pip install bleak numpy scipy
```

### Test

```bash
cd sensing/emg
python3 test_emg.py              # BLE hardware test
python3 test_emg_filter.py       # filter test (no hardware needed)
```

### Packet Format (ITRI Protocol)
- BLE notification: hex string, 4-char header + 7 groups x 24 hex chars
- Header: 3-char device ID + 1-char sequence number (0-F)
- Each group: T2 value at offset [6:12], T4 value at offset [18:24]
- Sequence number wraps 0->F for packet loss detection
