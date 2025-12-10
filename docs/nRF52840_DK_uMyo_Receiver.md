# nRF52840 DK uMyo Fast64 Receiver

## Overview

This document describes the nRF52840 DK-based receiver for collecting EMG data from uMyo sensors using the Fast64 (star protocol) radio mode.

---

## Framework Design

### System Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        ExoPulse EMG/Motor Control System                      │
└──────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────────────┐
  │                              SENSOR LAYER                                    │
  ├─────────────────────────────────────────────────────────────────────────────┤
  │                                                                              │
  │   ┌─────────────────────────────┐        ┌─────────────────────────────┐    │
  │   │      uMyo Sensor #1          │        │      uMyo Sensor #N          │    │
  │   │      (nRF52810)              │  ...   │      (nRF52810)              │    │
  │   │                              │        │                              │    │
  │   │  ┌───────────────────────┐  │        │  ┌───────────────────────┐  │    │
  │   │  │ ADC (AIN4, 1kHz)      │  │        │  │ ADC (AIN4, 1kHz)      │  │    │
  │   │  │ → DMA → 8-bin FFT    │  │        │  │ → DMA → 8-bin FFT    │  │    │
  │   │  │ → Muscle level calc  │  │        │  │ → Muscle level calc  │  │    │
  │   │  ├───────────────────────┤  │        │  ├───────────────────────┤  │    │
  │   │  │ IMU (LSM6DS3)         │  │        │  │ IMU (LSM6DS3)         │  │    │
  │   │  │ → Quaternion fusion  │  │        │  │ → Quaternion fusion  │  │    │
  │   │  │ → Yaw/Pitch/Roll     │  │        │  │ → Yaw/Pitch/Roll     │  │    │
  │   │  ├───────────────────────┤  │        │  ├───────────────────────┤  │    │
  │   │  │ Radio (urf_lib)       │  │        │  │ Radio (urf_lib)       │  │    │
  │   │  │ → Fast64 star node   │  │        │  │ → Fast64 star node   │  │    │
  │   │  └───────────────────────┘  │        │  └───────────────────────┘  │    │
  │   └──────────────┬──────────────┘        └──────────────┬──────────────┘    │
  │                  │                                      │                    │
  └──────────────────┼──────────────────────────────────────┼────────────────────┘
                     │         2.4GHz Fast64 (Ch 21)        │
                     │         TDMA Star Protocol           │
                     └──────────────────┬───────────────────┘
                                        │
  ┌─────────────────────────────────────┼───────────────────────────────────────┐
  │                          RECEIVER LAYER                  │                   │
  ├─────────────────────────────────────┼───────────────────────────────────────┤
  │                                     ▼                                        │
  │                     ┌─────────────────────────────┐                          │
  │                     │    nRF52840 DK (PCA10056)    │                          │
  │                     │    Star Protocol Central     │                          │
  │                     │                              │                          │
  │                     │  ┌────────────────────────┐ │                          │
  │                     │  │ urf_star_protocol      │ │                          │
  │                     │  │ → Channel 21           │ │                          │
  │                     │  │ → 1000 kbps            │ │                          │
  │                     │  │ → 2000 μs phase        │ │                          │
  │                     │  │ → Central mode         │ │                          │
  │                     │  ├────────────────────────┤ │                          │
  │                     │  │ Packet Parser          │ │                          │
  │                     │  │ → Unit ID extraction   │ │                          │
  │                     │  │ → ADC[8] samples       │ │                          │
  │                     │  │ → Spectrum[4]          │ │                          │
  │                     │  │ → Quaternion[4]        │ │                          │
  │                     │  │ → IMU data             │ │                          │
  │                     │  ├────────────────────────┤ │                          │
  │                     │  │ UART Output            │ │                          │
  │                     │  │ → 115200 baud          │ │                          │
  │                     │  │ → J-Link CDC           │ │                          │
  │                     │  └────────────────────────┘ │                          │
  │                     └──────────────┬───────────────┘                          │
  │                                    │                                          │
  └────────────────────────────────────┼──────────────────────────────────────────┘
                                       │ USB (J-Link CDC)
                                       │ /dev/ttyACM0
  ┌────────────────────────────────────┼──────────────────────────────────────────┐
  │                        HOST/GUI LAYER               │                         │
  ├────────────────────────────────────┼──────────────────────────────────────────┤
  │                                    ▼                                          │
  │  ┌─────────────────────────────────────────────────────────────────────────┐ │
  │  │                         Python Applications                              │ │
  │  │                                                                          │ │
  │  │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐   │ │
  │  │  │ plot_umyo.py     │  │ umyo_gui.py      │  │ umyo_diagnostic.py   │   │ │
  │  │  │                  │  │                  │  │                      │   │ │
  │  │  │ Real-time ADC    │  │ PySide6 GUI      │  │ Command-line         │   │ │
  │  │  │ waveform plot    │  │ - Serial config  │  │ diagnostic tool      │   │ │
  │  │  │ matplotlib       │  │ - ADC plot       │  │ - Mode info          │   │ │
  │  │  │ animation        │  │ - Spectrum plot  │  │ - Packet stats       │   │ │
  │  │  │                  │  │ - Raw data log   │  │ - Monitoring         │   │ │
  │  │  └──────────────────┘  └──────────────────┘  └──────────────────────┘   │ │
  │  └─────────────────────────────────────────────────────────────────────────┘ │
  │                                    │                                          │
  │                                    ▼                                          │
  │  ┌─────────────────────────────────────────────────────────────────────────┐ │
  │  │                      Motor Control Integration                           │ │
  │  │                                                                          │ │
  │  │  EMG Data ──► Signal Processing ──► Control Algorithm ──► Motor Command │ │
  │  │                                                                          │ │
  │  │  muscle_level > threshold  ──►  torque = f(muscle_level)  ──►  CAN TX   │ │
  │  └─────────────────────────────────────────────────────────────────────────┘ │
  └───────────────────────────────────────────────────────────────────────────────┘
```

### Software Component Layers

```
┌─────────────────────────────────────────────────────────────────┐
│                     APPLICATION LAYER                            │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Python GUI Applications                                    │ │
│  │  - umyo_gui.py (PySide6 real-time visualization)           │ │
│  │  - plot_umyo.py (matplotlib ADC plotter)                   │ │
│  │  - motor_control.py (integrated control GUI)               │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     INTERFACE LAYER                              │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Serial Protocol                                            │ │
│  │  - Format: "RX: ID=0x%08X LEN=%d BAT=%d ADC_ID=%d ADC:..."  │ │
│  │  - Baud: 115200                                             │ │
│  │  - Port: /dev/ttyACM0 (J-Link CDC)                          │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     FIRMWARE LAYER (nRF52840 DK)                 │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  main.c - Star Protocol Central                             │ │
│  │  - star_init(21, 1000, 2000, 1)  // Central mode           │ │
│  │  - star_loop_step()              // Protocol processing    │ │
│  │  - star_has_packet()             // Check for data         │ │
│  │  - star_get_packet()             // Retrieve packet        │ │
│  │  - uart_send()                   // Output to host         │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  urf_lib - Radio Abstraction                                │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │ │
│  │  │ urf_star_    │  │ urf_radio    │  │ urf_timer        │  │ │
│  │  │ protocol     │  │              │  │                  │  │ │
│  │  │              │  │ rf_init()    │  │ millis()         │  │ │
│  │  │ star_init()  │  │ rf_listen()  │  │ micros()         │  │ │
│  │  │ star_send()  │  │ rf_send()    │  │ delay_ms()       │  │ │
│  │  │ star_loop()  │  │ rf_get_pkt() │  │ delay_mcs()      │  │ │
│  │  └──────────────┘  └──────────────┘  └──────────────────┘  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│                              ▼                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  nrf_usdk52 - Nordic SDK (Hardware Abstraction)             │ │
│  │  - RADIO peripheral                                         │ │
│  │  - TIMER peripheral                                         │ │
│  │  - UART peripheral                                          │ │
│  │  - GPIO peripheral                                          │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     HARDWARE LAYER                               │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  nRF52840 SoC                                               │ │
│  │  - 64MHz ARM Cortex-M4F                                     │ │
│  │  - 2.4GHz Radio (BLE 5.0 / proprietary)                     │ │
│  │  - 1MB Flash, 256KB RAM                                     │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### uMyo Firmware Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    uMyo Sensor Firmware (nRF52810)               │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│  main.c - Main Application Loop                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                     Mode State Machine                       ││
│  │                                                              ││
│  │    ┌──────────┐      ┌──────────┐      ┌──────────┐        ││
│  │    │  BLE     │ BTN  │ Fast32   │ BTN  │ Fast64   │        ││
│  │    │  Mode    │─────►│  Mode    │─────►│  Mode    │────┐   ││
│  │    │  (BLUE)  │      │ (MAGENTA)│      │ (GREEN)  │    │   ││
│  │    └──────────┘      └──────────┘      └──────────┘    │   ││
│  │         ▲                                              │   ││
│  │         └──────────────────────────────────────────────┘   ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    Data Acquisition Pipeline                 ││
│  │                                                              ││
│  │   ADC (AIN4)                                                ││
│  │      │                                                       ││
│  │      ▼                                                       ││
│  │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     ││
│  │   │ adc_read.c  │───►│ fft_opt.c   │───►│ push_adc_   │     ││
│  │   │             │    │             │    │ data()      │     ││
│  │   │ DMA-based   │    │ 8-bin FFT   │    │             │     ││
│  │   │ 1kHz sample │    │ Spectrum    │    │ Ring buffer │     ││
│  │   │ Continuous  │    │ Analysis    │    │ 8 samples   │     ││
│  │   └─────────────┘    └─────────────┘    └─────────────┘     ││
│  │                                                              ││
│  │   IMU (LSM6DS3)                                             ││
│  │      │                                                       ││
│  │      ▼                                                       ││
│  │   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     ││
│  │   │ lsm6ds3.c   │───►│ quat_math.c │───►│ Yaw/Pitch/  │     ││
│  │   │             │    │             │    │ Roll calc   │     ││
│  │   │ Accel/Gyro  │    │ Quaternion  │    │             │     ││
│  │   │ Integration │    │ Fusion      │    │ Orientation │     ││
│  │   └─────────────┘    └─────────────┘    └─────────────┘     ││
│  │                                                              ││
│  │   Magnetometer (QMC7983)                                    ││
│  │      │                                                       ││
│  │      ▼                                                       ││
│  │   ┌─────────────┐                                           ││
│  │   │ qmc7983.c   │───► Heading correction                    ││
│  │   └─────────────┘                                           ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    Packet Preparation                        ││
│  │                                                              ││
│  │   Mode-specific functions:                                  ││
│  │   ┌─────────────────────┐                                   ││
│  │   │ prepare_and_send_   │ ──► BLE Advertisement (31 bytes)  ││
│  │   │ BLE()               │     Channels 37/38/39             ││
│  │   ├─────────────────────┤                                   ││
│  │   │ prepare_data_       │ ──► Fast32 packet (32 bytes)      ││
│  │   │ packet32()          │     Channel 83                    ││
│  │   ├─────────────────────┤                                   ││
│  │   │ prepare_data_       │ ──► Fast64 packet (64 bytes)      ││
│  │   │ packet()            │     Channel 21, TDMA              ││
│  │   └─────────────────────┘                                   ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## Project Location

```
/media/eddlai/DATA/ExoPulse_firmware/EMG/nRF_DK/receiver/
```

## Hardware Setup

- **Receiver**: nRF52840 DK (PCA10056)
- **Transmitter**: uMyo EMG sensor (must be in Fast64/GREEN mode)
- **Test Setup**: Function generator connected to uMyo for sine wave testing

### nRF52840 DK Connections

- **J-Link USB port**: For programming and UART output (use this port, NOT the nRF USB)
- **Serial output**: 115200 baud via J-Link CDC (`/dev/ttyACM0` or `/dev/ttyACM1`)
- **LEDs**:
  - LED1 (P0.13): Blinks on packet receive
  - LED2 (P0.14): Toggles every 5 seconds (heartbeat)
  - LED4 (P0.16): On when receiver is running

## uMyo Radio Modes

uMyo cycles through 3 modes with button press:

| Mode | LED Color | Protocol | Receiver Required |
|------|-----------|----------|-------------------|
| **Fast64** | GREEN (3 pulses) | Star protocol, Ch 21 | nRF52 with star_protocol |
| Fast32 | MAGENTA (3 pulses) | Simple RF, Ch 83 | nRF52 with rf_init |
| BLE | BLUE (3 pulses) | BLE Advertisement | ESP32, phone, or BLE dongle |

**For this receiver, uMyo must be in Fast64 mode (GREEN LED).**

## Building and Flashing

```bash
cd /media/eddlai/DATA/ExoPulse_firmware/EMG/nRF_DK/receiver

# Build
make clean && make

# Flash via J-Link
make flash
# Or manually:
JLinkExe -device NRF52840_XXAA -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink
```

## Firmware Details

### Source Files

- `main.c` - Main receiver application
- `Makefile` - Build configuration (uses NRF52832 defines for SDK compatibility)
- `nrf52840_dk.ld` - Linker script
- `flash.jlink` - J-Link commander script

### Dependencies

Uses the urf_lib from the uMyo project:
- `urf_radio.c/h` - Nordic radio driver
- `urf_timer.c/h` - Timer/millis functions
- `urf_star_protocol.c/h` - Star protocol implementation

### Protocol Configuration

```c
// Star protocol as CENTRAL (receiver)
star_init(21, 1000, 2000, 1);  // Channel 21, 1000kbps, 2000us phase, is_central=1
```

### Serial Output Format

```
=== nRF52840 DK Star Protocol Receiver ===
Fast64 mode - Channel 21, 1000kbps, 2000us phase
Central ID: 0xABCD1234
Listening for uMyo sensors...

RX: ID=0x12345678 LEN=62 BAT=180 ADC_ID=42 ADC:1234,1235,1236,1237,1238,1239,1240,1241 SP:100,200,300,400 Q:1000,200,300,400 ACC:100,-200,9800 YPR:450,100,-50
--- Status: 1234 packets, 60s uptime ---
```

### Packet Structure (Fast64, 62 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Packet ID |
| 1 | 1 | Length |
| 2-5 | 4 | Unit ID |
| 6 | 1 | Param count |
| 7 | 1 | Param ID |
| 8 | 1 | Battery level (0-255) |
| 9 | 1 | Version ID |
| 10 | 1 | ADC data ID |
| 11-26 | 16 | ADC samples (8 x 16-bit)* |
| 27-34 | 8 | Spectrum (4 x 16-bit) |
| 35-42 | 8 | Quaternion (4 x 16-bit) |
| 43-48 | 6 | Accelerometer (3 x 16-bit) |
| 49-54 | 6 | Yaw/Pitch/Roll (3 x 16-bit) |
| 55-60 | 6 | Magnetometer (3 x 16-bit) |

*\*ADC samples: The uMyo has **ONE ADC channel** (P0.28/AIN4) sampled at 1kHz. The 8 values are 8 **consecutive time samples** (8ms of data), NOT 8 different channels. Each packet contains samples at T+0ms, T+1ms, ... T+7ms. The plotter stitches these together into a continuous waveform.*

## Python Tools

### Diagnostic Tool

```bash
python3 umyo_diagnostic.py -p /dev/ttyACM0
python3 umyo_diagnostic.py -m  # Continuous monitoring
python3 umyo_diagnostic.py -i  # Show uMyo mode info
```

### Real-time Plotter

```bash
python3 plot_umyo.py -p /dev/ttyACM0
```

### PySide6 GUI

```bash
python3 umyo_gui.py
```

Features:
- Serial port selection and connection
- Connection status with packet rate
- Real-time ADC and spectrum plots
- Raw data log

## Troubleshooting

### Identifying USB Devices and Serial Ports

When both the nRF52840 DK and uMyo programmer are connected:

| Device | USB ID | Serial Ports | Purpose |
|--------|--------|--------------|---------|
| **nRF52840 DK** | `1366:1061` SEGGER J-Link | `/dev/ttyACM0`, `/dev/ttyACM1` | Receiver + serial output |
| **uMyo (ST-LINK/V2)** | `0483:3748` STMicroelectronics | None (SWD only) | Programming uMyo |

To identify ports:
```bash
# Check USB devices
lsusb | grep -E "SEGGER|STMicro"

# Verify serial port belongs to J-Link
udevadm info --query=all --name=/dev/ttyACM0 | grep ID_MODEL
```

### No packets received

1. **Check uMyo mode**: Must be Fast64 (GREEN LED, 3 pulses)
2. **Check serial port**: Use `/dev/ttyACM0` or `/dev/ttyACM1`
3. **Kill other processes**: `fuser -k /dev/ttyACM0`
4. **Check J-Link connection**: `lsusb | grep -i segger`

### J-Link not detected

1. Use the **J-Link USB port** (not nRF USB)
2. Check power switch position on DK
3. Unplug and replug USB cable
4. Kill hanging processes: `pkill -f JLinkExe`

### Build errors

The SDK only has nRF52832 headers. Use `-DNRF52832_XXAA` define - the radio/timer peripherals are compatible with nRF52840.

## Related Documentation

- [uMyo Radio Protocols](uMyo_Radio_Protocols.md) - Detailed protocol specs
- [BLE Advertisement Explained](BLE_Advertisement_Explained.md) - BLE mode details
- [BLE Scanning ESP32 vs PC](BLE_Scanning_ESP32_vs_PC.md) - BLE receiver options

## Next Steps / TODO

- [ ] Add USB CDC support for nRF52840's native USB (instead of J-Link UART)
- [ ] Implement bidirectional communication (send commands to uMyo)
- [ ] Add data logging to file
- [ ] Integrate with ExoPulse main GUI
- [ ] Support multiple uMyo sensors simultaneously

---

## ExoPulse Project Overview (For Context)

This receiver is part of the larger **ExoPulse** exoskeleton control system.

### Project Structure

```
/media/eddlai/DATA/ExoPulse_firmware/
├── MGv2/                    # Main motor control (LK-TECH motors via CAN)
│   └── src/main.cpp         # ESP32 FreeRTOS firmware
├── EMG/                     # EMG signal acquisition
│   ├── nRF_DK/receiver/     # ← THIS PROJECT (nRF52840 DK receiver)
│   ├── uMyo/                # uMyo sensor firmware (nRF52832)
│   │   └── urf_lib/         # Radio libraries (urf_radio, urf_star_protocol)
│   ├── uMyo_BLE/            # uMyo BLE interface library
│   └── src/ADS.cpp          # ADS1256 24-bit ADC driver
├── UI_components/           # Python GUI tools
│   ├── motor_control.py     # Main control GUI (PySide6)
│   ├── motor_monitor.py     # Advanced monitoring
│   ├── dual_motor_plotter.py
│   └── emg_plotter.py
├── ble_scanner/             # ESP32 BLE scanner for uMyo
├── docs/                    # Documentation
├── gui.py                   # Unified GUI launcher
└── platformio.ini           # Build config for ESP32 projects
```

### System Architecture

```
┌─────────────────┐     ┌─────────────────┐
│  uMyo Sensor    │     │  MGv2 Motors    │
│  (EMG + IMU)    │     │  (CAN Bus)      │
└────────┬────────┘     └────────┬────────┘
         │ Fast64/BLE            │ CAN
         │                       │
┌────────▼────────┐     ┌────────▼────────┐
│ nRF52840 DK     │     │ ESP32 DevKit    │
│ (This Receiver) │     │ + MCP2515       │
└────────┬────────┘     └────────┬────────┘
         │ UART (J-Link CDC)     │ UART/WiFi
         │                       │
         └───────────┬───────────┘
                     │
              ┌──────▼──────┐
              │  PC / GUI   │
              │ (Python)    │
              └─────────────┘
```

### Key Libraries Used

| Library | Location | Purpose |
|---------|----------|---------|
| `urf_radio` | `EMG/uMyo/urf_lib/` | Nordic radio driver |
| `urf_star_protocol` | `EMG/uMyo/urf_lib/` | Star protocol TDMA |
| `urf_timer` | `EMG/uMyo/urf_lib/` | millis()/micros() timing |
| `nrf_usdk52` | `EMG/uMyo/urf_lib/` | Nordic SDK headers |

### Related Components

| Component | Location | Purpose |
|-----------|----------|---------|
| uMyo Firmware | `EMG/uMyo/uMyo_fw_v3_1/` | Sensor firmware (transmitter) |
| BLE Scanner | `ble_scanner/src/main.cpp` | ESP32 BLE mode receiver |
| Motor Control | `MGv2/src/main.cpp` | Motor control firmware |
| Main GUI | `UI_components/motor_control.py` | Control interface |

### Current Session Notes

**What we accomplished:**
1. Set up J-Link tools (installed JLink V8.92)
2. Set up nrfutil for DFU programming
3. Created bare-metal Fast64 star protocol receiver
4. Built and flashed to nRF52840 DK
5. Created Python tools: diagnostic, plotter, PySide6 GUI
6. Fixed UART by switching to `urf_uart` library
7. Created minimal Fast64 uMyo firmware (no LED/IMU/magnetometer)
8. **Successfully tested wireless communication!**

**Working Configuration (2025-12-10):**
- **DK Receiver**: nRF52840 DK running Fast64 star protocol (central mode)
- **uMyo Transmitter**: nRF52810 with minimal Fast64 firmware
- **Protocol**: Channel 21, 1000kbps, 2000us phase
- **Serial Output**: 115200 baud on `/dev/ttyACM0` (J-Link CDC)

**Example Output:**
```
RX: ID=0x823A5872 LEN=62 BAT=200 ADC_ID=0 ADC:17408,18944,19200,15872,17664,19968,19712 SP:13568,0,0,0 Q:127,-256,0,0 ACC:0,0,0 YPR:0,0,0
```

**uMyo Firmware Notes:**
- Location: `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/`
- Uses `MINIMAL_FAST64` define to skip LED/IMU/magnetometer init
- Fixed issues: FPU flags (`-mfloat-abi=soft`), UICR write removed
- Flashing via OpenOCD + ST-LINK/V2

**Hardware tested:**
- nRF52840 DK with J-Link OB (S/N: 1050285157)
- uMyo sensor (nRF52810) with ST-LINK/V2 programmer
