# ExoPulse EMG System - Complete Architecture Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Signal Chain (uMyo Firmware)](#signal-chain-umyo-firmware)
3. [Component Architecture](#component-architecture)
4. [Communication Protocols](#communication-protocols)
5. [System Flow Diagram](#system-flow-diagram)
6. [Key Files Reference](#key-files-reference)

---

## System Overview

The ExoPulse EMG subsystem is a wireless multi-sensor acquisition platform for exoskeleton control consisting of:

### Hardware Components
1. **uMyo Wearable Sensors** (nRF52810/nRF52832)
   - EMG electrode acquisition (1kHz, 14-bit)
   - LSM6DS3 6-axis IMU (accelerometer + gyroscope)
   - QMC7983 3-axis magnetometer
   - RGB LED feedback
   - Battery monitoring

2. **nRF52840 DK Receiver** (Star Protocol Hub)
   - Central TDMA coordinator
   - Multi-sensor aggregation
   - UART/USB bridge to PC (921600 baud)
   - LED status indicators

3. **PC Analysis Tools** (Python)
   - Real-time waveform visualization (matplotlib)
   - GUI dashboard (PySide6)
   - Diagnostic utilities

### Operating Modes
- **BLE Mode** (100 Hz): Universal mobile compatibility, 31-byte advertisements
- **Fast32 Mode** (1000 Hz): Single sensor, Enhanced ShockBurst, 32-byte packets
- **Fast64 Mode** (500 Hz/sensor): Multi-sensor TDMA, 62-byte packets, star protocol

---

## Signal Chain (uMyo Firmware)

### Full Signal Processing Pipeline

```
EMG Electrode Input (±5mV - ±3.6V)
          ↓
    [Hardware ADC]
          ↓
[128x Oversampling @ 1116 Hz]
          ↓
   [14-bit Resolution]
          ↓
    [IRQ Handler]
          ↓
  [Mains Frequency Detection]
          ↓
   [Notch Filter (50/60Hz)]
          ↓
   [DC Removal Filter]
          ↓
  [Circular Buffer Storage]
          ↓
   [FFT Spectral Analysis]
          ↓
 [Adaptive Gain Control]
          ↓
   [Packet Assembly]
          ↓
[Fast64 Wireless TX] → nRF52840 DK Receiver
```

### Stage 1: Hardware ADC
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/adc_read.c`

**Configuration:**
- **Input Pin:** P0.28 (AIN4) - single-ended EMG input
- **Resolution:** 14-bit (0-16383 counts)
- **Reference:** Internal 0.6V
- **Gain:** 1/6 → Input range: ±3.6V
- **Oversampling:** 128x hardware oversampling
- **Sample Rate:** ~1116 Hz (after oversampling)
- **Acquisition Time:** 5μs per sample
- **Burst Mode:** Enabled (reduces latency)

**ADC Formula:**
```
Digital_Output = (Input_Voltage / 0.6V) × (1/6) × 16384
For ±5mV input: Output ≈ 6826 ± 137 counts
For ±3.6V max: Output spans 0-16383
```

**IRQ-Driven Acquisition:**
- ADC fills buffer with 8 samples at 1116 Hz
- `SAADC_IRQHandler()` fires on `EVENTS_END`
- Copies 8 samples from `adc_buf[]` to `res_buf[]`
- Sets `has_new_data = 1` flag
- Automatically restarts next conversion
- Every 1000 conversions: switches to battery voltage measurement on AIN5

### Stage 2: Data Retrieval
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` → `push_adc_data()` (line 525)

**Function:** `adc_get_data(emg_raw, emg_sp_data)`
- Returns 8 raw ADC samples as `int16_t emg_raw[8]`
- Clears `has_new_data` flag
- Called at ~1116 Hz / 8 = **139.5 Hz packet rate**

### Stage 3: Mains Frequency Detection
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` (lines 535-553)

**Purpose:** Adaptively detect 50Hz vs 60Hz power line interference

**Algorithm:** Sine/Cosine Decomposition
```c
// Generate reference tones at 50Hz and 60Hz
t50 += t50_coeff;  // 0.044802867 × 2π → 50 Hz
t60 += t60_coeff;  // 0.053763441 × 2π → 60 Hz

// Correlate with input signal using exponential moving average
noise50_s = 0.99×noise50_s + 0.01×sin(t50)×emg_raw[x]
noise50_c = 0.99×noise50_c + 0.01×cos(t50)×emg_raw[x]
noise60_s = 0.99×noise60_s + 0.01×sin(t60)×emg_raw[x]
noise60_c = 0.99×noise60_c + 0.01×cos(t60)×emg_raw[x]

// Compare power at each frequency
power50 = noise50_s² + noise50_c²
power60 = noise60_s² + noise60_c²
mains_is_50Hz = (power50 > power60)
```

**Characteristics:**
- Time constant: ~100 samples (exponential decay 0.99)
- Continuously adapts to environment
- Used to select appropriate notch filter

### Stage 4: Notch Filter (50Hz/60Hz Rejection)
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` (lines 559-562)

**Method:** Tap-Delay Summation (Comb Filter)

**Algorithm:**
```c
// Store raw sample in circular buffer
emg_raw_buffer[adc_buf_pos] = emg_raw[x];

// Select delay based on detected mains frequency
fpos1 = adc_buf_pos - 11;  // 50Hz: 11 samples back
if (!mains_is_50Hz)
    fpos1 = adc_buf_pos - 9;  // 60Hz: 9 samples back

// Apply tap-delay filter
adc_buffer[adc_buf_pos] = emg_raw_buffer[adc_buf_pos] + emg_raw_buffer[fpos1];
```

**Theory:**
- **50Hz filter:** Delay = 11 samples @ 1116 Hz = 9.86 ms ≈ λ/2 @ 50.6 Hz
- **60Hz filter:** Delay = 9 samples @ 1116 Hz = 8.06 ms ≈ λ/2 @ 62 Hz
- **Effect:** Adds signal to its half-period-delayed version → cancels periodic noise
- **Notch depth:** ~20-30 dB at mains frequency
- **Side lobes:** Present at harmonics of notch frequency

### Stage 5: DC Removal / Baseline Tracking
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` (lines 557-558)

**Method:** Exponential Moving Average (Low-Pass Filter)

**Algorithm:**
```c
avg_emg_long = 0.99×avg_emg_long + 0.01×emg_raw[x]
```

**Characteristics:**
- **Time constant:** τ = 1/0.01 = 100 samples ≈ 90 ms
- **Cutoff frequency:** fc ≈ 11 Hz
- **Purpose:** Tracks slow DC drift and baseline offset
- **Usage:** Later subtracted as DC offset correction in adaptive gain stage

### Stage 6: FFT Spectral Analysis
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` (lines 569-576), `fft_opt.c`

**Method:** 8-point Real FFT (Radix-8 Butterfly)

**Algorithm:**
```c
// Prepare 8 filtered samples
bufX[0..7] = adc_buffer[adc_buf_pos-7 .. adc_buf_pos]

// Perform 8-point Real FFT
fft8_real(bufX, bufYr, bufYi);

// Calculate magnitude spectrum
for (x = 0; x < 4; x++) {
    cur_spectr[x] = sqrt(bufYr[x]² + bufYi[x]²);
}
```

**Frequency Bins (@ 1116 Hz sample rate):**
- **Bin 0 (DC):** 0 Hz
- **Bin 1:** 139.5 Hz (fundamental muscle frequency band)
- **Bin 2:** 279 Hz (higher EMG harmonics)
- **Bin 3:** 418.5 Hz (high-frequency muscle activity)
- **Nyquist:** 558 Hz

**Usage:**
- `cur_spectr[2]` + `cur_spectr[3]` → muscle activity level
- Drives LED color/brightness
- Transmitted in data packet for remote monitoring

### Stage 7: Adaptive Gain Control
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` (lines 600-618)

**Method:** Proportional-Integral (PI) Controller with Derivative Term

**Purpose:** Automatically adjust DC offset to keep signal centered around 8000 counts

**Algorithm:**
```c
// Calculate DC deviation from target (8000)
d_corr = avg_emg_long - 8000;

// Derivative term (rate of change)
d_corr_speed = d_corr - prev_d_corr;

// Integral term (accumulated error)
i_corr += d_corr;
i_corr *= 0.999;  // Prevent windup

// Update correction value
avg_corr += (d_corr/1000) + (d_corr_speed × 0.05);

// Clamp to 10-bit range
if (avg_corr > 1022) avg_corr = 1022;
if (avg_corr < 2) avg_corr = 2;
```

**Output:** `d_val_out` (0-1022) → used for LED brightness PWM control

**Characteristics:**
- **Proportional gain:** 0.001
- **Derivative gain:** 0.05
- **Integral decay:** 0.999 (prevents windup)
- **Update rate:** 100 Hz (every 10ms)
- **Convergence time:** ~1-2 seconds

### Stage 8: Packet Assembly
**File:** `EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/main.c` → `prepare_data_packet()` (lines 70-186)

**Packet Structure (62 bytes total):**

| Byte Range | Field | Size | Description |
|------------|-------|------|-------------|
| 0 | packet_id | 1 | Rolling packet counter (0-128) |
| 1 | length | 1 | Total packet length |
| 2-5 | unit_id | 4 | Device unique ID (from FICR) |
| 6 | data_type | 1 | 80 + send_cnt (88 for 8 samples) |
| 7 | param_send_id | 1 | Parameter type (0=battery) |
| 8 | battery_level | 1 | 0-255 (maps to 2.0V-4.5V) |
| 9 | version_id | 1 | Firmware version (101) |
| 10 | reserved | 1 | Padding |
| 11 | adc_data_id | 1 | ADC packet counter (0-255) |
| **12-27** | **ADC samples** | **16** | **8 × 16-bit filtered samples** |
| 28-35 | spectrum | 8 | 4 × 16-bit FFT bins |
| 36-43 | quaternion | 8 | qw, qx, qy, qz (IMU orientation) |
| 44-49 | accelerometer | 6 | ax, ay, az (raw accel) |
| 50-55 | yaw/pitch/roll | 6 | Euler angles |
| 56-61 | magnetometer | 6 | mx, my, mz |

**Note:** ADC field contains **time-sequential samples**, NOT 8 channels. Single 1kHz channel captured over 8ms.

### Stage 9: Wireless Transmission
**Files:** `EMG/uMyo/urf_lib/urf_star_protocol.c`, `urf_radio.c`

**Protocol:** Fast64 Star Protocol (Nordic proprietary)
- **Radio Config:** Channel 21, 1000 kbps, 2000μs phase
- **TX Power:** Configurable (typically 0 dBm)
- **Packet Rate:** ~125 packets/second (8ms per packet)
- **Range:** ~10-30 meters line-of-sight

---

## Component Architecture

### 1. uMyo Firmware Components

#### Core Files (`EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/`)

**main.c** - Main Entry Point
- `prepare_data_packet()` - Assembles 62-byte Fast64 packets
- `prepare_data_packet32()` - Encodes 32-byte Fast32 Enhanced ShockBurst format
- `prepare_and_send_BLE()` - BLE Advertisement mode
- `prepare_minimal_data_packet()` - Minimal config for boards without IMU
- `push_adc_data()` - Main signal processing pipeline
  - Reads 8 ADC samples
  - Performs 50/60Hz mains notch filtering (adaptive)
  - Applies FFT via `fft8_real()` for spectrum
  - Calculates muscle activity level
  - Updates device state (calibration, mode)

**adc_read.c/h** - ADC Management
- `adc_init()` - Configure SAADC
  - Input: P0.28 (AIN4) for EMG
  - 14-bit resolution, 128x oversampling → 1116 Hz
  - Pulldown resistor on negative input
  - 1/6 gain for ±3.6V input range
- `SAADC_IRQHandler()` - Interrupt handler for continuous sampling
- `adc_read_battery()` - Periodic battery monitoring (every ~1s)
- Dual buffer management: raw ADC + processed data

**fft_opt.c/h** - Fast Fourier Transform
- `fft8_real()` - Real FFT on 8 samples → 4-bin spectrum
- Pre-computed sine/cosine lookup table (65 entries)
- Optimized radix-8 butterfly for low CPU load

**leds.c/h** - RGB LED Feedback
- `leds_init()` - Configure RGB LED pins
- `start_leds_pwm()` - PWM sequence control
  - LED mode indication (Blue=BLE, Magenta=Fast32, Green=Fast64)
  - Real-time muscle activity visualization
- Pulse sequences for mode switching feedback

**lsm6ds3.c/h** - 6-Axis IMU Driver
- Register definitions for LSM6DS3 control
- Fetch accelerometer (3-axis), gyroscope (3-axis) @ 100+ Hz
- Functions: `lsm_get_quat_packed()`, `lsm_get_acc()`, `lsm_get_W()`, `lsm_get_angles()`

**qmc7983.c/h** - 3-Axis Magnetometer Driver
- Magnetometer calibration (min/max tracking)
- `qmc_process_calibration()` - Calibration state machine
- `qmc_get_mag()` - Returns calibrated magnetic field vectors
- Used for orientation estimation in quaternion calculations

**quat_math.c/h** - Quaternion Mathematics
- Data structures: `sQ` (quaternion), `sV` (vector)
- Operations: `q_mult()`, `rotate_v()`, `q_from_vectors()`, `q_renorm()`
- Fast trigonometric approximations: `sin_f()`, `cos_f()`, `atan2_f()`

**fast_math.c/h** - Optimized Math Functions
- Approximated sin/cos functions for embedded systems
- Polynomial approximation (Bhaskara variant)
- Small-angle optimizations
- Used for notch filter tone generation

**persistent_storage.c/h** - Non-Volatile Configuration
- Stores calibration: magnetometer min/max values
- Device state: radio mode, zero quaternion offset
- `sDevice_state` structure for flash persistence

**spim_functions.c/h** - SPI Master / I2C Bridge
- Bit-banging I2C over nRF52 GPIO
- Communicates with LSM6DS3 and QMC7983

### 2. URF_LIB - Unified Radio Framework (`EMG/uMyo/urf_lib/`)

**urf_radio.c/h** - Low-Level Radio Control
- `rf_init(channel, speed, crc_len)` - Configure Nordic radio
  - Channel selection (0-100, 2.4GHz band)
  - Speed: 1000 kbps (1 Mbps) typical
  - CRC: 3-byte polynomial
- `rf_init_ext()` - Extended config with whitening
- `rf_mode_rx_only()` / `rf_mode_tx_only()` - Mode switching
- `rf_send()` - Transmit raw packet (byte 0=anything, byte 1=length)
- `rf_listen()` - Enable RX mode
- `rf_has_new_packet()` - Poll for incoming data
- `rf_get_packet()` - Retrieve received data
- `rf_autorespond_on()` - Hardware auto-reply for acknowledgments

**urf_star_protocol.c/h** - TDMA Multi-Sensor Coordination
**CRITICAL: Enables multi-uMyo operation via time-division multiplexing**

Functions:
- `star_init(channel, speed, phase_length_mcs, is_central)` - Initialize as hub or node
- `star_set_id(id)` - Assign unique ID
- `star_queue_send(pack, length)` - Queue packet for transmission
- `star_send_to_node(node_id, payload_8b)` - Central hub sends to specific node
- `star_has_packet()` / `star_get_packet()` - Receive interface
- `star_loop_step()` - **Must be called every ~1ms to maintain TDMA timing**

Internal Variables:
- `role_central` - Hub (1) or sensor node (0)
- `cycle_step_mcs` - Phase length in microseconds (2000us = 2ms typical)
- `active_nodes` / `node_list[64]` - Track connected sensors
- `node_active[]` - Last activity timestamp per sensor
- `active_timeout` - 30s timeout to remove unresponsive nodes

**TDMA Cycle:**
```
|--Slot1(Node1)--|--Slot2(Node2)--|--Slot3(Node3)--|--Beacon(Central)|
  2000us           2000us            2000us            2000us
```

**urf_timer.c/h** - Millisecond Timing
- `time_start()` - Initialize 32-bit timer
- `millis()` - Current time in milliseconds
- `delay_ms()` - Blocking delay
- Synchronizes ADC sampling and packet transmission

**urf_uart.c/h** - Serial Communication
- `uart_init(pin_TX, pin_RX, speed)` - Initialize UART
- `uart_send(buf, length)` - Transmit bytes
- `uart_send_ready()` - Check if ready for next packet (prevents overflow)
- `uart_send_remains()` - Bytes still in buffer
- `uprintf()` - Formatted printf for debugging
- TX buffer: 512 bytes, sends in 64-byte chunks

**urf_ble_peripheral.c/h** - BLE Advertisement Engine
- `ble_init_radio()` - Set up BLE radio
- `ble_send_advertisement_packet()` - Broadcast on channels 37/38/39
- PDU helpers:
  - `ble_add_field_to_pdu()` - Add AD structure
  - `ble_prepare_adv_pdu()` - Build complete advertisement
  - Constants: `PDU_FLAGS`, `PDU_SHORT_NAME`, `PDU_MANUFACTURER_SPEC`

### 3. nRF52840 DK Receiver (`EMG/nRF_DK/receiver/`)

**main.c** - Central Hub Receiver

**Main Loop Architecture:**
```c
while(1) {
    star_loop_step();          // Process TDMA coordination

    if(star_has_packet()) {
        star_get_packet(pack);  // Retrieve received data

        // Parse packet fields:
        // Bytes 2-5:  Unit ID
        // Byte 11:    ADC data ID
        // Bytes 12-27: 8 × 16-bit ADC samples
        // Bytes 28-35: 4 × 16-bit FFT spectrum
        // Bytes 36-43: Quaternion (w,x,y,z)
        // Bytes 44-49: Accelerometer (x,y,z)
        // Bytes 50-55: Yaw/Pitch/Roll angles
        // Bytes 56-61: Magnetometer (x,y,z)

        // Format: "RX: ID=0xXXXX ADC_ID=N ADC:v0,v1,... SP:s0,s1,..."
        uart_send(formatted_line);
    }

    // LED feedback
    if(packet_received) led_blink(LED1);

    // Status heartbeat every 5s
}
```

**Hardware Configuration:**
- **LEDs (nRF52840 DK):**
  - LED1 (P0.13): Packet received indicator
  - LED2 (P0.14): Heartbeat toggle (5s)
  - LED3 (P0.15): Unused
  - LED4 (P0.16): Operational status (always on)
- **Serial:** UART on P0.06 (TX) / P0.08 (RX), 921600 baud
- **Protocol:** ASCII text lines ending with \r\n

**Helper Functions:**
- `led_init()`, `led_on(pin)`, `led_off(pin)`, `led_toggle(pin)`
- `uart_puts()` - Send null-terminated string
- `int_to_str()`, `hex_to_str()` - Number formatting without sprintf
- `fast_clock_start()` - Enable HF clock for radio

### 4. Python Analysis Tools (`EMG/nRF_DK/receiver/`)

#### plot_umyo.py - Real-Time EMG Waveform Plotter

**Purpose:** Live visualization of single-channel EMG data + spectrum

**Key Features:**
- Reads UART from nRF52840 DK (default: /dev/ttyACM0, 921600 baud)
- Parses "RX:" lines to extract:
  - 8 ADC samples (16-bit signed integers)
  - 4 FFT spectrum bins
- Displays:
  - Top plot: 2000-sample waveform (2 seconds @ 1kHz)
  - Bottom plot: 4 spectrum channels over 500 packets
- Auto-scales Y-axis based on recent data range
- Info: packet count, sample count, samples per packet

**Usage:**
```bash
python3 plot_umyo.py -p /dev/ttyACM0 -b 921600
```

#### umyo_diagnostic.py - Connection Testing Tool

**Purpose:** Verify receiver functionality

**Tests:**
- Lists available serial ports
- Connects to receiver and monitors for 10 seconds
- Counts RX packets, status messages, unknown lines
- Identifies baud rate/connection issues
- Diagnostic output for troubleshooting

#### umyo_gui.py - PySide6 GUI Dashboard

**Purpose:** Comprehensive receiver monitoring and configuration

**Architecture:**
- `SerialWorker` - Background thread for non-blocking serial I/O
- Signal-slot model (Qt):
  - `data_received` - Raw line from serial
  - `connection_changed` - Port open/close events
  - `packet_received` - Parsed packet dict
- `_parse_packet()` - Extract ID, ADC samples, battery, etc.

**GUI Components:**
- Port selection dropdown
- Connect/disconnect buttons
- Real-time data display
- Packet statistics
- Battery voltage monitoring

### 5. uMyo_BLE Library (`EMG/uMyo_BLE/`)

**Arduino-compatible library for ESP32 BLE scanning**

**Data Structure:**
```cpp
struct uMyo_data {
    uint32_t id;                    // Device identifier
    int batt_mv;                    // Battery in mV
    uint8_t last_data_id;           // Sequence counter
    int16_t cur_spectrum[4];        // FFT bins
    uint16_t device_avg_muscle_level; // Smoothed activity
    uint32_t last_data_time;        // Timestamp
    sQ Qsg;                         // Quaternion
};
```

**Main Class `uMyo_BLE_`:**
- `begin()` - Start BLE scanning
- `run()` - Process advertisements (call in main loop)
- `getDeviceCount()` - Number of discovered uMyo devices
- `getBattery(idx)` - Battery voltage for device N
- `getID(idx)` - Get device ID
- `getMuscleLevel(idx)` - Instantaneous activity
- `getAverageMuscleLevel(idx)` - Low-pass filtered activity
- `getSpectrum(idx, float*)` - Get 4-bin FFT
- `getPitch/getRoll/getYaw(idx)` - Orientation angles
- `getDeviceByID(id)` - Lookup by device ID

**Supports:** Up to 12 simultaneous uMyo sensors

---

## Communication Protocols

### Fast64 Packet Format (62 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | Packet ID | Incrementing counter 0-128 |
| 1 | 1 | Length | Always 62 |
| 2-5 | 4 | Unit ID | Unique sensor chip ID |
| 6 | 1 | Param Count | Number of parameters (8) |
| 7 | 1 | Param ID | Parameter type (0=battery) |
| 8 | 1 | Battery | Level 0-255 (0-5V) |
| 9 | 1 | Version | Firmware version (101) |
| 10 | 1 | Reserved | Padding |
| 11 | 1 | ADC ID | Sample counter |
| **12-27** | **16** | **ADC[8]** | **8 consecutive 16-bit samples @ 1kHz** |
| 28-35 | 8 | Spectrum[4] | 4-bin FFT magnitude (16-bit each) |
| 36-43 | 8 | Quaternion[4] | w, x, y, z quaternion (16-bit each) |
| 44-49 | 6 | Accel[3] | X, Y, Z acceleration (16-bit each) |
| 50-55 | 6 | YPR[3] | Yaw, Pitch, Roll angles (16-bit each) |
| 56-61 | 6 | Mag[3] | Magnetometer X, Y, Z (16-bit each) |

**Important:** ADC field contains **sequential time samples**, NOT 8 channels.

### Serial Output Format (UART 921600 baud)
```
RX: ID=0x12345678 ADC_ID=42 ADC:1234,1235,1236,1237,1238,1239,1240,1241 SP:100,200,300,400
```

### Radio Modes

| Mode | LED Color | Protocol | Channel | Packet | Rate | Use Case |
|------|-----------|----------|---------|--------|------|----------|
| **BLE** | BLUE (3 pulses) | BLE Advertisement | 37/38/39 | 31 bytes | ~100 Hz | Mobile apps, universal |
| **Fast32** | MAGENTA (3 pulses) | Enhanced ShockBurst | 83 | 32 bytes | ~1000 Hz | Single sensor |
| **Fast64** | GREEN (3 pulses) | Star Protocol (TDMA) | 21 | 62 bytes | ~500 Hz/sensor | Multi-sensor |

Mode switching: Button press on uMyo (see `dev_state` management in main.c)

---

## System Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                  ExoPulse EMG System Architecture                 │
└──────────────────────────────────────────────────────────────────┘

            ┌─────────────────────────────────────┐
            │      uMyo Wearable Sensor (x N)     │
            │   nRF52810 / nRF52832               │
            │  ┌─────────────────────────────────┐│
            │  │ EMG ADC (1kHz, 14-bit)          ││
            │  │ LSM6DS3 IMU (Accel + Gyro)      ││
            │  │ QMC7983 Magnetometer            ││
            │  │ FFT Analysis (4-bin)            ││
            │  │ Quaternion Math                 ││
            │  │ RGB LED (Visual Feedback)       ││
            │  └─────────────────────────────────┘│
            │  [Persistent Storage: Calibration] │
            └────────────┬─────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
    ┌─────────┐    ┌─────────┐    ┌─────────┐
    │  BLE    │    │ Fast32  │    │ Fast64  │
    │ (100Hz) │    │(1000Hz) │    │(500Hz)  │
    └────┬────┘    └────┬────┘    └────┬────┘
         │              │              │
         └──────────────┼──────────────┘
                        │
         ┌──────────────▼──────────────────┐
         │   nRF52840 DK Star Hub         │
         │  (Central TDMA Coordinator)    │
         │  ┌────────────────────────────┐│
         │  │ Radio Receiver             ││
         │  │ Packet Parser              ││
         │  │ UART Formatter (921600)    ││
         │  │ LED Status Indicators      ││
         │  └────────────────────────────┘│
         └────────────┬────────────────────┘
                      │
             ┌────────▼────────┐
             │   USB/Serial    │
             │  /dev/ttyACM0   │
             └────────┬────────┘
                      │
         ┌────────────┼────────────┐
         │            │            │
         ▼            ▼            ▼
    ┌──────────┐ ┌──────────┐ ┌──────────┐
    │plot_umyo │ │umyo_gui  │ │diagnostic│
    │  .py     │ │  .py     │ │  .py     │
    │(Matplotlib)│(PySide6) │ │(Testing) │
    └──────────┘ └──────────┘ └──────────┘
         │            │            │
         └────────────┼────────────┘
                      │
             ┌────────▼──────────┐
             │ Motor Control PC  │
             │ Visualization GUI │
             │ Data Analysis     │
             └───────────────────┘
```

### Data Flow Example (Single Packet)

```
TIME: 0ms
uMyo: ADC sample #0 (1234) → adc_buf[0]
      LSM6DS3 read (Accel=100,200,300)
      QMC7983 read (Mag=50,60,70)

TIME: 1-7ms
uMyo: ADC samples #1-7 → adc_buf[1-7]

TIME: 8ms
uMyo: push_adc_data() triggered
      ├─ Apply mains filter
      ├─ FFT: cur_spectr[0-3]
      ├─ avg_muscle_level calculation
      └─ adc_data_id++

TIME: 8.5ms
uMyo: prepare_data_packet()
      ├─ data_packet[0] = packet_id++
      ├─ data_packet[2-5] = device_id
      ├─ data_packet[12-27] = 8 ADC samples
      ├─ data_packet[28-35] = spectrum
      └─ ...IMU data...

TIME: 8.7ms
uMyo: star_queue_send(data_packet, 62)
      └─ Radio TX begins

TIME: 9.0ms
nRF52840: Radio RX complete (62 bytes, CRC valid)

TIME: 9.1ms
nRF52840: Main loop
      ├─ star_has_packet() = true
      ├─ star_get_packet(pack)
      ├─ LED1 blink (20ms)
      ├─ Parse packet fields
      ├─ Format output string
      └─ uart_send() @ 921600 baud (~0.5ms)

TIME: 9.6ms
PC: pyserial reads "RX: ID=0x12345678 ADC_ID=N ADC:1234,...\r\n"
    ├─ parse_line() extracts samples
    ├─ Append to waveform buffer (2000 samples)
    └─ Update plot (20 Hz refresh)

TIME: 10.0ms
uMyo: Next ADC sample cycle begins...
```

---

## Key Files Reference

### uMyo Firmware

| File | Location | Purpose |
|------|----------|---------|
| main.c | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | Main entry point, signal processing |
| adc_read.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | ADC configuration, IRQ handler |
| fft_opt.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | 8-point Real FFT |
| fast_math.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | Optimized sin/cos |
| leds.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | RGB LED PWM control |
| lsm6ds3.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | 6-axis IMU driver |
| qmc7983.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | Magnetometer driver |
| quat_math.c/h | EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update/ | Quaternion operations |

### URF Library

| File | Location | Purpose |
|------|----------|---------|
| urf_radio.c/h | EMG/uMyo/urf_lib/ | Nordic radio hardware layer |
| urf_star_protocol.c/h | EMG/uMyo/urf_lib/ | TDMA multi-sensor coordination |
| urf_timer.c/h | EMG/uMyo/urf_lib/ | Millisecond timing |
| urf_uart.c/h | EMG/uMyo/urf_lib/ | Serial communication |
| urf_ble_peripheral.c/h | EMG/uMyo/urf_lib/ | BLE advertisement engine |

### nRF52840 DK Receiver

| File | Location | Purpose |
|------|----------|---------|
| main.c | EMG/nRF_DK/receiver/ | Central hub receiver, UART bridge |
| plot_umyo.py | EMG/nRF_DK/receiver/ | Real-time EMG waveform plotter |
| umyo_gui.py | EMG/nRF_DK/receiver/ | PySide6 GUI dashboard |
| umyo_diagnostic.py | EMG/nRF_DK/receiver/ | Connection testing tool |

### ESP32 Integration

| File | Location | Purpose |
|------|----------|---------|
| uMyo_BLE.h/cpp | EMG/uMyo_BLE/src/ | Arduino BLE scanner library |

---

## Performance Metrics

### Signal Processing (per 8-sample burst @ 139.5 Hz)

| Stage | Processing Time | Notes |
|-------|----------------|-------|
| Hardware ADC | 640μs | 5μs × 128 oversample |
| IRQ Handler | <10μs | DMA transfer |
| Mains Detect | ~200μs | Sin/cos computation |
| Notch Filter | <5μs | Simple addition |
| DC Removal | <2μs | Single multiply-add |
| FFT | ~500μs | Radix-8 butterfly |
| Gain Control | ~50μs | PI controller |
| Packet Prep | ~100μs | Memory copies |
| Radio TX | ~500μs | Radio hardware |

**Total:** ~1.5ms per packet
**Available Time:** 7.2ms (@ 139.5 Hz)
**CPU Utilization:** ~20%

### Radio Performance

| Mode | Packet Rate | Latency | Range | Power |
|------|------------|---------|-------|-------|
| BLE | 100 Hz | ~10ms | 30m | Low |
| Fast32 | 1000 Hz | ~1ms | 20m | Medium |
| Fast64 | 500 Hz/sensor | ~2ms | 10-30m | Medium |

### Multi-Sensor Capacity (Fast64 TDMA)

| Sensors | Update Rate/Sensor | Total Throughput | Slot Time |
|---------|-------------------|------------------|-----------|
| 1 | 500 Hz | 500 packets/s | 2ms |
| 2 | 250 Hz | 500 packets/s | 4ms |
| 4 | 125 Hz | 500 packets/s | 8ms |
| 8 | 62.5 Hz | 500 packets/s | 16ms |

---

## Configuration Summary

### Currently Active: MINIMAL_FAST64 Mode

**Enabled:**
- ✅ Hardware ADC with 128x oversampling
- ✅ 14-bit resolution @ 1116 Hz
- ✅ Fast64 wireless transmission
- ✅ Basic packet assembly with 8 ADC samples

**Bypassed:**
- ❌ Mains frequency detection
- ❌ Notch filter (50/60Hz rejection)
- ❌ DC removal filter
- ❌ FFT spectral analysis
- ❌ Adaptive gain control
- ❌ IMU/magnetometer data

**Minimal Signal Chain:**
```
Hardware ADC → IRQ Handler → Packet Assembly → Fast64 TX
```

### Full Firmware Configuration

To enable all signal processing stages, remove `#define MINIMAL_FAST64` from main.c and ensure IMU hardware is present.

---

## Troubleshooting

### Common Issues

1. **No packets received at receiver:**
   - Check uMyo LED color (Green = Fast64 mode)
   - Verify receiver is in Fast64 central mode
   - Check radio channel (21 for Fast64)
   - Ensure both devices powered on

2. **Serial output corrupted:**
   - Verify baud rate (921600)
   - Check UART buffer not overflowing (use `uart_send_ready()`)
   - Reduce packet output rate if needed

3. **ADC values saturated (4095 or 0):**
   - Check electrode connection
   - Verify input voltage within ±3.6V range
   - Confirm ADC gain setting (1/6)

4. **FFT spectrum all zeros:**
   - Ensure `MINIMAL_FAST64` not defined
   - Check `fft8_real()` being called
   - Verify ADC data has variation

5. **IMU data not updating:**
   - Check I2C communication (LSM6DS3 address)
   - Verify IMU initialization succeeded
   - Confirm IMU interrupt wired correctly

---

## Conclusion

The ExoPulse EMG system is a sophisticated wireless multi-sensor platform featuring:

1. **Distributed Architecture:** Individual nRF52 wearables collect EMG and IMU data
2. **Three Transmission Modes:** BLE for mobile, Fast32 for speed, Fast64 for multi-sensor
3. **Real-Time Signal Processing:** 1kHz ADC with adaptive filtering and FFT analysis
4. **Robust Star Protocol:** TDMA coordination enables collision-free operation
5. **Flexible Receiver:** nRF52840 DK hub with UART/USB for PC connectivity
6. **Rich Visualization:** Python tools for real-time waveforms and monitoring

The modular `urf_lib` framework abstracts radio complexity, while the main firmware implements domain-specific processing for exoskeleton control feedback.

---

**Document Version:** 1.0
**Last Updated:** 2025-12-10
**Firmware Version:** v3.1 April 2025 RGB Update
