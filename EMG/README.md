# EMG Signal Acquisition Module

## Overview

This folder contains code and resources for EMG (Electromyography) signal acquisition using two primary methods:

1. **uMyo Wireless Sensor** - Wearable EMG/IMU sensor using nRF52 radio (primary method)
2. **ADS1256 ADC** - High-precision wired EMG acquisition (alternative method)

The uMyo system is the recommended approach for the ExoPulse exoskeleton due to its wireless capability, integrated IMU, and real-time muscle activity detection.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ExoPulse EMG Subsystem                               │
└─────────────────────────────────────────────────────────────────────────────┘

                              ┌─────────────────────┐
                              │   uMyo Sensor       │
                              │   (nRF52810)        │
                              │                     │
                              │ ┌─────────────────┐ │
                              │ │ ADC (1kHz)      │ │  ← EMG Electrodes
                              │ │ 8-bin FFT       │ │
                              │ │ Muscle Level    │ │
                              │ ├─────────────────┤ │
                              │ │ IMU (LSM6DS3)   │ │  ← Orientation
                              │ │ Mag (QMC7983)   │ │
                              │ │ Quaternion      │ │
                              │ └─────────────────┘ │
                              └──────────┬──────────┘
                                         │
           ┌─────────────────────────────┼─────────────────────────────┐
           │                             │                             │
           ▼                             ▼                             ▼
    ┌──────────────┐            ┌──────────────┐            ┌──────────────┐
    │  BLE Mode    │            │ Fast32 Mode  │            │ Fast64 Mode  │
    │  (BLUE LED)  │            │ (MAGENTA)    │            │ (GREEN LED)  │
    │              │            │              │            │              │
    │ Channels:    │            │ Channel: 83  │            │ Channel: 21  │
    │ 37, 38, 39   │            │ 32-byte PKT  │            │ 64-byte PKT  │
    │ ~100 Hz      │            │ ~1000 Hz     │            │ ~500 Hz      │
    └──────┬───────┘            └──────┬───────┘            └──────┬───────┘
           │                           │                           │
           ▼                           ▼                           ▼
    ┌──────────────┐            ┌──────────────┐            ┌──────────────┐
    │   ESP32      │            │  nRF52       │            │ nRF52840 DK  │
    │ BLE Scanner  │            │ rf_init()    │            │ star_init()  │
    │              │            │              │            │              │
    │ uMyo_BLE lib │            │ Single       │            │ Multi-sensor │
    │ ble_scanner/ │            │ sensor only  │            │ TDMA support │
    └──────┬───────┘            └──────┬───────┘            └──────┬───────┘
           │                           │                           │
           └───────────────────────────┼───────────────────────────┘
                                       │
                                       ▼
                              ┌─────────────────────┐
                              │   Host PC / GUI     │
                              │                     │
                              │ - plot_umyo.py      │
                              │ - umyo_gui.py       │
                              │ - motor_control.py  │
                              └─────────────────────┘
```

---

## Folder Structure

```
EMG/
├── README.md                    # This file
│
├── nRF_DK/                      # nRF52840 DK Fast64 Receiver
│   └── receiver/
│       ├── main.c               # Star protocol central (receiver)
│       ├── Makefile             # Build configuration
│       ├── nrf52840_dk.ld       # Linker script
│       ├── flash.jlink          # J-Link flash script
│       ├── plot_umyo.py         # Real-time ADC plotter
│       ├── umyo_diagnostic.py   # Diagnostic tool
│       └── umyo_gui.py          # PySide6 GUI
│
├── uMyo/                        # uMyo Sensor Firmware
│   ├── uMyo_fw_v3_1/            # Production firmware (nRF52810/52832)
│   ├── uMyo_fw_v3_1_April_2025_RGB_update/  # RGB LED variant
│   ├── uMyo_fw_v2_1/            # Legacy version
│   ├── nrf52840_dongle_receiver/  # USB dongle receiver
│   ├── urf_lib/                 # Shared radio library (submodule)
│   │   ├── urf_radio.c/h        # Nordic radio driver
│   │   ├── urf_timer.c/h        # Timing functions
│   │   ├── urf_star_protocol.c/h  # TDMA star protocol
│   │   ├── urf_ble_peripheral.c/h # BLE advertising
│   │   └── nrf_usdk52/          # Nordic SDK headers
│   └── README.md
│
├── uMyo_BLE/                    # Arduino BLE Library (ESP32)
│   ├── src/
│   │   ├── uMyo_BLE.cpp/h       # Main BLE receiver class
│   │   └── quat_math.cpp/h      # Quaternion mathematics
│   ├── examples/
│   │   ├── umyo_basic/          # Basic usage example
│   │   ├── umyo_IMU/            # IMU data example
│   │   └── umyo_LED_strip/      # Visual feedback example
│   └── README.md
│
├── lib/ADS1256/                 # ADS1256 ADC Library (alternative)
│   ├── ADS1256.cpp/h
│   └── examples/
│
├── src/
│   └── ADS.cpp                  # ADS1256 ESP32 driver
│
├── img/                         # Hardware diagrams
│   └── ADS1256_Arduino_ESP32.jpg
│
└── ads1256_plotter.py           # ADS1256 visualization tool
```

---

## uMyo Radio Protocols

The uMyo sensor supports three transmission modes, selected via button press:

| Mode | LED | Protocol | Channel | Packet Size | Data Rate | Best For |
|------|-----|----------|---------|-------------|-----------|----------|
| **BLE** | BLUE (3 pulses) | BLE Advertisement | 37/38/39 | 31 bytes | ~100 Hz | ESP32, smartphones |
| **Fast32** | MAGENTA (3 pulses) | Enhanced ShockBurst | 83 | 32 bytes | ~1000 Hz | Single sensor |
| **Fast64** | GREEN (3 pulses) | Star Protocol (TDMA) | 21 | 64 bytes | ~500 Hz | Multi-sensor |

### Mode Selection

Press the uMyo button to cycle through modes:
```
BLE (BLUE) → Fast32 (MAGENTA) → Fast64 (GREEN) → BLE ...
```

---

## Fast64 Packet Structure (Recommended)

The Fast64 mode provides the most comprehensive data in a 62-byte payload:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Packet ID | Incrementing counter |
| 1 | 1 | Length | Payload length (62) |
| 2-5 | 4 | Unit ID | Unique sensor identifier |
| 6 | 1 | Param count | Number of parameters |
| 7 | 1 | Param ID | Parameter identifier |
| 8 | 1 | Battery | Battery level (0-255) |
| 9 | 1 | Version | Firmware version |
| 10 | 1 | ADC ID | ADC sample counter |
| **11-26** | **16** | **ADC[8]** | **8 consecutive 16-bit samples (1kHz)** |
| 27-34 | 8 | Spectrum[4] | 4-bin FFT (16-bit each) |
| 35-42 | 8 | Quaternion[4] | w, x, y, z (16-bit each) |
| 43-48 | 6 | Accel[3] | X, Y, Z acceleration |
| 49-54 | 6 | YPR[3] | Yaw, Pitch, Roll |
| 55-60 | 6 | Mag[3] | Magnetometer X, Y, Z |

**Important**: The ADC field contains **8 consecutive time samples** at 1kHz (8ms of data per packet), NOT 8 different channels. The uMyo has a single ADC channel (P0.28/AIN4).

---

## Receiver Options

### Option 1: nRF52840 DK (Fast64 Mode) - Recommended

Best for high-performance, multi-sensor applications.

**Setup**:
```bash
cd EMG/nRF_DK/receiver
make clean && make
make flash
```

**Monitor**:
```bash
python3 plot_umyo.py -p /dev/ttyACM0
# Or GUI:
python3 umyo_gui.py
```

**Output Format**:
```
RX: ID=0x12345678 LEN=62 BAT=180 ADC_ID=42 ADC:1234,1235,1236,1237,1238,1239,1240,1241 SP:100,200,300,400 Q:1000,200,300,400 ACC:100,-200,9800 YPR:450,100,-50
```

### Option 2: ESP32 BLE Scanner (BLE Mode)

Best for simple integration with existing ESP32 projects.

**Setup**:
```bash
cd /media/eddlai/DATA/ExoPulse_firmware/ble_scanner
pio run -t upload
```

**Code** (simplified):
```cpp
#include <uMyo_BLE.h>

uMyo_BLE uMyo;

void setup() {
    Serial.begin(115200);
    uMyo.begin();
}

void loop() {
    uMyo.run();
    if (uMyo.getDeviceCount() > 0) {
        int muscle = uMyo.getMuscleLevel(0);
        float pitch = uMyo.getPitch(0);
        Serial.printf("Muscle: %d, Pitch: %.1f\n", muscle, pitch);
    }
    delay(10);
}
```

### Option 3: ESP32 Direct BLE (Custom Implementation)

See `ble_scanner/src/main.cpp` for a minimal BLE advertisement parser:

```cpp
class MyCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
        uint8_t* payload = dev.getPayload();
        // Search for "uMyo" string in payload
        // Parse manufacturer data (type 0xFF)
        // Extract ADC samples, battery, etc.
    }
};
```

---

## Data Flow: ESP32 to PC

```
┌─────────────┐     BLE      ┌─────────────┐     UART/WiFi   ┌─────────────┐
│   uMyo      │ ──────────► │   ESP32     │ ──────────────► │    PC       │
│   Sensor    │  Advertisement│ BLE Scanner │   Serial/TCP    │   Python    │
└─────────────┘              └─────────────┘                  └─────────────┘

ESP32 receives BLE advertisement packets from uMyo
↓
Parses manufacturer data to extract:
  - ADC samples (muscle signal)
  - Battery level
  - Spectrum (4-bin FFT)
  - Quaternion (orientation)
↓
Forwards data via Serial or WiFi to PC
↓
Python GUI displays real-time plots and controls motors
```

---

## Data Flow: nRF52840 DK to PC

```
┌─────────────┐   Fast64     ┌─────────────┐      UART       ┌─────────────┐
│   uMyo      │ ──────────► │ nRF52840 DK │ ──────────────► │    PC       │
│   Sensor    │  Star Protocol│  Central    │  J-Link CDC     │   Python    │
└─────────────┘              └─────────────┘                  └─────────────┘

nRF52840 DK runs as star protocol central (receiver)
↓
Receives 64-byte packets from uMyo nodes
↓
Parses packet fields:
  - Unit ID (unique sensor identifier)
  - ADC[8] (8 consecutive samples at 1kHz)
  - Spectrum[4], Quaternion[4], Accel[3], YPR[3], Mag[3]
↓
Outputs parsed data via UART (115200 baud)
↓
Python tools (plot_umyo.py, umyo_gui.py) visualize data
```

---

## Integration with ExoPulse Motor Control

The EMG signals can be used to control the MGv2 motors:

```python
# Conceptual integration in motor_control.py
from umyo_interface import UMyoReceiver
from motor_interface import MotorController

umyo = UMyoReceiver(port='/dev/ttyACM0')
motors = MotorController(port='/dev/ttyUSB0')

while True:
    emg_data = umyo.read()

    # Map muscle activity to motor torque
    if emg_data.muscle_level > threshold:
        torque = map_emg_to_torque(emg_data.muscle_level)
        motors.set_torque(torque)
```

---

## Hardware Components

### uMyo Sensor (nRF52810/52832)

- **MCU**: nRF52810 (BLE SoC)
- **ADC**: Single channel, 1kHz sampling, 12-bit
- **IMU**: LSM6DS3 (accelerometer + gyroscope)
- **Magnetometer**: QMC7983
- **Radio**: 3 modes (BLE, Fast32, Fast64)
- **Power**: Li-Po battery, ~180mAh
- **Electrodes**: Dry contact (replaceable with gel electrodes)

### nRF52840 DK (PCA10056)

- **MCU**: nRF52840 (BLE 5.0 SoC)
- **Debug**: J-Link OB (on-board)
- **Serial**: J-Link CDC (115200 baud)
- **LEDs**: 4 user LEDs (P0.13-P0.16)

### ADS1256 (Alternative)

- **Resolution**: 24-bit
- **Channels**: 8 differential inputs
- **Sample Rate**: Up to 30kSPS
- **Interface**: SPI

---

## Key Libraries

| Library | Location | Purpose |
|---------|----------|---------|
| `urf_radio` | `uMyo/urf_lib/` | Nordic radio driver |
| `urf_star_protocol` | `uMyo/urf_lib/` | TDMA star protocol |
| `urf_timer` | `uMyo/urf_lib/` | Timing (millis, micros) |
| `urf_ble_peripheral` | `uMyo/urf_lib/` | BLE advertising |
| `uMyo_BLE` | `uMyo_BLE/src/` | ESP32 Arduino library |
| `ADS1256` | `lib/ADS1256/` | 24-bit ADC driver |

---

## Building the Code

### nRF52840 DK Receiver

```bash
cd EMG/nRF_DK/receiver

# Install toolchain
sudo apt install gcc-arm-none-eabi

# Build
make clean && make

# Flash
make flash
```

### uMyo Firmware

```bash
cd EMG/uMyo/uMyo_fw_v3_1

# Initialize submodule
git submodule update --init --recursive

# Build
make clean && make

# Flash via OpenOCD + ST-LINK
make flash
```

### ESP32 BLE Scanner

```bash
cd /media/eddlai/DATA/ExoPulse_firmware/ble_scanner

# Build and upload
pio run -t upload

# Monitor
pio device monitor
```

---

## Troubleshooting

### No packets received (nRF52840 DK)

1. Verify uMyo is in Fast64 mode (GREEN LED, 3 pulses)
2. Check correct serial port (`/dev/ttyACM0` or `/dev/ttyACM1`)
3. Kill other processes: `fuser -k /dev/ttyACM0`
4. Verify J-Link connection: `lsusb | grep -i segger`

### No BLE devices found (ESP32)

1. Verify uMyo is in BLE mode (BLUE LED, 3 pulses)
2. Check ESP32 is scanning: Serial should show "Scanning for uMyo..."
3. Ensure uMyo is powered and within range (~10m)

### Build errors

1. Install ARM toolchain: `sudo apt install gcc-arm-none-eabi`
2. Initialize submodules: `git submodule update --init --recursive`
3. Use `-DNRF52832_XXAA` flag (SDK compatibility)

---

## Related Documentation

- [nRF52840_DK_uMyo_Receiver.md](../docs/nRF52840_DK_uMyo_Receiver.md) - Detailed receiver setup
- [uMyo_Radio_Protocols.md](../docs/uMyo_Radio_Protocols.md) - Protocol specifications
- [BLE_Advertisement_Explained.md](../docs/BLE_Advertisement_Explained.md) - BLE packet structure
- [BLE_Scanning_ESP32_vs_PC.md](../docs/BLE_Scanning_ESP32_vs_PC.md) - BLE receiver comparison

---

## Credits

- **uMyo Hardware/Firmware**: [Ultimate Robotics](https://github.com/ultimaterobotics)
- **urf_lib**: Nordic radio and BLE library by Ultimate Robotics
- **ExoPulse Integration**: Custom integration for exoskeleton control
