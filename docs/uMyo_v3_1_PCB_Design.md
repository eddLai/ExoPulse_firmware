# uMyo v3.1 PCB Design

## Overview

This document describes the uMyo v3.1 PCB hardware design, a wireless wearable EMG sensor with integrated IMU. The PCB design files are located in `EMG/uMyo_v3_1_pcb/` as a git submodule from [Ultimate Robotics](https://github.com/ultimaterobotics/uMyo_v3_1_pcb).

---

## PCB Location

```
EMG/uMyo_v3_1_pcb/
├── LICENSE                     # CERN-OHL-P-2.0 (permissive hardware license)
├── README.md
└── uMyo_v3_1/
    ├── uMyo_v3_1.kicad_sch     # KiCad 6 schematic
    ├── uMyo_v3_1.kicad_pcb     # KiCad 6 PCB layout
    ├── uMyo_v3_1.kicad_pro     # KiCad 6 project file
    ├── uMyo_v3_1.pdf           # Schematic PDF export
    └── uMyo_v3-Edge_Cuts.svg   # Board outline
```

---

## Hardware Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           uMyo v3.1 PCB                                      │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                         POWER MANAGEMENT                                 ││
│  │                                                                          ││
│  │   Li-Po Battery (3.7V)                                                  ││
│  │        │                                                                 ││
│  │        ├──► Charge Controller ──► Battery                               ││
│  │        │    (USB charging)                                              ││
│  │        │                                                                 ││
│  │        ▼                                                                 ││
│  │   ┌──────────────┐                                                      ││
│  │   │ XC6206P332MR │  LDO Regulator                                       ││
│  │   │   (SOT-23)   │  3.3V @ 200mA                                        ││
│  │   └──────┬───────┘                                                      ││
│  │          │ 3.3V                                                         ││
│  │          ▼                                                               ││
│  └──────────┼──────────────────────────────────────────────────────────────┘│
│             │                                                                │
│  ┌──────────┼──────────────────────────────────────────────────────────────┐│
│  │          │              MAIN MCU                                         ││
│  │          ▼                                                               ││
│  │   ┌───────────────────────────────────────────┐                         ││
│  │   │         nRF52810-QFAA (QFN-48)            │                         ││
│  │   │                                           │                         ││
│  │   │  ┌─────────────┐  ┌─────────────────────┐ │                         ││
│  │   │  │ ARM Cortex  │  │ 2.4GHz Radio        │ │                         ││
│  │   │  │   M4        │  │ - BLE 5.0           │ │                         ││
│  │   │  │ 64MHz       │  │ - Proprietary modes │ │                         ││
│  │   │  └─────────────┘  │ - Fast32/Fast64     │ │                         ││
│  │   │                   └─────────────────────┘ │                         ││
│  │   │  ┌─────────────┐  ┌─────────────────────┐ │                         ││
│  │   │  │ 12-bit ADC  │  │ GPIO                │ │                         ││
│  │   │  │ AIN4 (EMG)  │  │ - I2C (IMU/Mag)     │ │                         ││
│  │   │  │ 1kHz sample │  │ - SWD (programming) │ │                         ││
│  │   │  └─────────────┘  │ - Button            │ │                         ││
│  │   │                   │ - RGB LED           │ │                         ││
│  │   │  ┌─────────────┐  └─────────────────────┘ │                         ││
│  │   │  │ 192KB Flash │                          │                         ││
│  │   │  │  24KB RAM   │                          │                         ││
│  │   │  └─────────────┘                          │                         ││
│  │   └───────────────────────────────────────────┘                         ││
│  │          │                                                               ││
│  └──────────┼──────────────────────────────────────────────────────────────┘│
│             │                                                                │
│  ┌──────────┼──────────────────────────────────────────────────────────────┐│
│  │          │              SENSORS                                          ││
│  │          │                                                               ││
│  │   ┌──────┴──────┐         ┌────────────────┐      ┌────────────────┐    ││
│  │   │             │   I2C   │                │ I2C  │                │    ││
│  │   │  EMG Input  │◄───────►│   LSM6DS3      │◄────►│   QMC5883L     │    ││
│  │   │   (AIN4)    │         │   (LGA-14)     │      │   (LGA-16)     │    ││
│  │   │             │         │                │      │                │    ││
│  │   │ Dry contact │         │ 6-DOF IMU:     │      │ 3-axis         │    ││
│  │   │ electrodes  │         │ - Accelerometer│      │ Magnetometer   │    ││
│  │   │             │         │ - Gyroscope    │      │                │    ││
│  │   └─────────────┘         └────────────────┘      └────────────────┘    ││
│  │                                                                          ││
│  └──────────────────────────────────────────────────────────────────────────┘│
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────────┐│
│  │                         USER INTERFACE                                   ││
│  │                                                                          ││
│  │   ┌────────────────┐      ┌────────────────┐      ┌────────────────┐    ││
│  │   │  RGB LED       │      │  Push Button   │      │  SWD Header    │    ││
│  │   │  (0603 BGRA)   │      │  (Mode select) │      │  (Programming) │    ││
│  │   │                │      │                │      │                │    ││
│  │   │ - BLUE: BLE    │      │ Short: Cycle   │      │ - SWDIO        │    ││
│  │   │ - MAGENTA:Fast32│     │   radio modes  │      │ - SWCLK        │    ││
│  │   │ - GREEN: Fast64│      │ Long: Calib    │      │ - GND          │    ││
│  │   └────────────────┘      └────────────────┘      │ - VCC          │    ││
│  │                                                   └────────────────┘    ││
│  └──────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Key Components

| Reference | Component | Package | Description |
|-----------|-----------|---------|-------------|
| **U2** | nRF52810-QFAA | QFN-48 (6x6mm) | Main MCU with BLE radio |
| **U3** | XC6206P332MR | SOT-23 | 3.3V LDO regulator (200mA) |
| **U6** | QMC5883L | LGA-16 (3x3mm) | 3-axis magnetometer |
| **U7** | LSM6DS3 | LGA-14 (3x2.5mm) | 6-DOF IMU (accel + gyro) |
| **D1** | LED_BGRA | 0603 RGB | Mode indicator LED |
| **Y1** | Crystal | - | 32MHz main crystal |
| **Y2** | Crystal | - | 32.768kHz RTC crystal |

---

## Pin Assignments

### nRF52810 GPIO Mapping

| Pin | Function | Description |
|-----|----------|-------------|
| P0.28 | AIN4 | EMG analog input (1kHz sampling) |
| P0.xx | SDA | I2C data (LSM6DS3, QMC5883L) |
| P0.xx | SCL | I2C clock |
| P0.xx | LED_R | RGB LED Red cathode |
| P0.xx | LED_G | RGB LED Green cathode |
| P0.xx | LED_B | RGB LED Blue cathode |
| P0.xx | BUTTON | Mode select button |
| SWDIO | SWD | Programming/debug |
| SWCLK | SWD | Programming/debug |

### EMG Input Circuit

```
                    ┌─────────────────────┐
  Electrode 1 ──────┤                     │
                    │   Differential      │
                    │   Amplifier         ├──────► P0.28 (AIN4)
                    │   + High-pass       │
  Electrode 2 ──────┤   + Low-pass        │
                    └─────────────────────┘
  Reference   ───────────────────────────────────► GND
```

The EMG input is filtered and amplified before reaching the ADC:
- High-pass filter: Removes DC offset and low-frequency drift
- Low-pass filter: Anti-aliasing for 1kHz sampling
- Gain stage: Amplifies weak EMG signals (~1mV) to ADC range

---

## I2C Bus Configuration

Both IMU and magnetometer share the I2C bus:

```
nRF52810 (Master)
    │
    ├──── SDA ────┬──── LSM6DS3 (0x6A/0x6B)
    │             │
    │             └──── QMC5883L (0x0D)
    │
    └──── SCL ────┬──── LSM6DS3
                  │
                  └──── QMC5883L
```

| Device | I2C Address | Description |
|--------|-------------|-------------|
| LSM6DS3 | 0x6A or 0x6B | Accelerometer + Gyroscope |
| QMC5883L | 0x0D | Magnetometer |

---

## Power Budget

| Component | Typical Current | Peak Current |
|-----------|-----------------|--------------|
| nRF52810 (active) | 3-5 mA | 15 mA (TX) |
| LSM6DS3 | 0.9 mA | 1.25 mA |
| QMC5883L | 0.3 mA | 2.6 mA |
| RGB LED (one color) | ~5 mA | 20 mA |
| **Total** | ~10 mA | ~40 mA |

With a typical 180mAh Li-Po battery:
- **Estimated runtime**: 10-15 hours continuous operation
- **Standby**: Several days (with sleep mode)

---

## Board Dimensions

- **Form factor**: Wearable armband
- **Approximate size**: ~30mm x 20mm
- **Layers**: 2-layer PCB
- **Electrode spacing**: Designed for ~20mm inter-electrode distance

---

## Antenna Design

The nRF52810 uses a PCB trace antenna for 2.4GHz operation:

```
┌──────────────────────────────────┐
│                                  │
│   ┌─────────────────────────┐   │
│   │    nRF52810             │   │
│   │                         │   │
│   │                    ANT ─┼───┼──► Meander/Inverted-F antenna
│   │                         │   │    (PCB trace)
│   └─────────────────────────┘   │
│                                  │
│   Keep-out zone (no copper)      │
│                                  │
└──────────────────────────────────┘
```

**Design considerations**:
- Keep-out zone around antenna for proper radiation
- Ground plane on bottom layer
- Matching network for 50Ω impedance

---

## Programming Interface

### SWD Pinout

```
┌───────────┐
│  1   2    │  1: VCC (3.3V)
│  ●   ●    │  2: SWDIO
│  3   4    │  3: SWCLK
│  ●   ●    │  4: GND
└───────────┘
```

### Supported Programmers

| Programmer | Interface | Notes |
|------------|-----------|-------|
| ST-LINK/V2 | SWD | Low-cost, tested with uMyo |
| J-Link | SWD | Professional, fastest |
| CMSIS-DAP | SWD | Open-source alternative |
| Black Magic Probe | SWD | GDB-native debugging |

### OpenOCD Flash Command

```bash
openocd -f interface/stlink.cfg \
        -f target/nrf52.cfg \
        -c "program firmware.hex verify reset exit"
```

---

## Design Files

### KiCad 6 Project

The design was created in KiCad 6 and includes:

| File | Description |
|------|-------------|
| `uMyo_v3_1.kicad_sch` | Schematic (hierarchical) |
| `uMyo_v3_1.kicad_pcb` | PCB layout |
| `uMyo_v3_1.kicad_pro` | Project settings |
| `uMyo_v3_1.pdf` | Schematic PDF export |
| `sym-lib-table` | Symbol library references |
| `fp-info-cache` | Footprint cache |

### Opening the Design

```bash
# Install KiCad 6+
sudo apt install kicad

# Open project
kicad EMG/uMyo_v3_1_pcb/uMyo_v3_1/uMyo_v3_1.kicad_pro
```

---

## Manufacturing Notes

### PCB Specifications

| Parameter | Value |
|-----------|-------|
| Layers | 2 |
| Minimum trace width | 0.15mm (6 mil) |
| Minimum spacing | 0.15mm (6 mil) |
| Via size | 0.3mm drill, 0.6mm pad |
| Board thickness | 1.6mm |
| Surface finish | HASL or ENIG |
| Solder mask | Green (or any color) |

### BOM (Bill of Materials)

Key components for ordering:

| Qty | Part Number | Description | Package |
|-----|-------------|-------------|---------|
| 1 | nRF52810-QFAA | BLE SoC | QFN-48 |
| 1 | LSM6DS3TR | IMU | LGA-14 |
| 1 | QMC5883L | Magnetometer | LGA-16 |
| 1 | XC6206P332MR | 3.3V LDO | SOT-23 |
| 1 | RGB LED | Common anode | 0603 |
| 1 | 32MHz crystal | Main clock | - |
| 1 | 32.768kHz crystal | RTC | - |
| - | Capacitors/resistors | Various | 0402/0603 |

---

## License

The PCB design is licensed under **CERN-OHL-P-2.0** (CERN Open Hardware License - Permissive):

- Free to use, modify, and distribute
- Commercial use permitted
- No copyleft requirements
- Attribution required (acknowledge derived from uMyo)

---

## Related Documentation

- [EMG/README.md](../EMG/README.md) - EMG module overview
- [nRF52840_DK_uMyo_Receiver.md](nRF52840_DK_uMyo_Receiver.md) - Receiver setup
- [uMyo_Radio_Protocols.md](uMyo_Radio_Protocols.md) - Protocol specifications
- [uMyo Firmware](https://github.com/ultimaterobotics/uMyo) - Firmware source

---

## JLCPCB PCBA Production Log

This section documents the experience of ordering PCB assembly (PCBA) through JLCPCB, including issues encountered and solutions found. This serves as a reference for future production runs.

### Production Overview

| Item | Details |
|------|---------|
| **Date** | September - October 2025 |
| **Status** | In Production |
| **Manufacturer** | JLCPCB |
| **EDA Tool** | KiCad 6 |
| **Service** | SMT Assembly (PCBA) |

---

### 1. File Preparation Issues

#### Missing Drill Layer

- **Issue (9/20):** After uploading Gerber files, the order was rejected with the message "Missing Drill Layer".
- **Root Cause:** The `.drl` drill file was missing from the ZIP archive. In KiCad, drill files are generated separately from Gerber plots, and it was accidentally omitted during packaging.
- **Attempted Fix:** Contacted customer service (Sunny) to submit the missing file.
- **Result:** JLCPCB's system **does not allow direct file replacement** on existing orders.
- **Solution:** Had to cancel the original order for a refund, repackage the correct Gerber ZIP including the `.drl` file, and place a new order.

#### Lesson Learned

```
KiCad Export Checklist:
1. Plot → Generate Gerber files
2. Plot → Generate Drill Files (separate step!)
3. Verify ZIP contents before upload:
   - *.gtl (Top Copper)
   - *.gbl (Bottom Copper)
   - *.gts (Top Solder Mask)
   - *.gbs (Bottom Solder Mask)
   - *.gto (Top Silkscreen)
   - *.gbo (Bottom Silkscreen)
   - *.gm1 (Board Outline)
   - *.drl (Drill File) ← Don't forget this!
```

---

### 2. BOM and Component Sourcing

This was the most time-consuming part of the communication process (contact: Fiona).

#### Component Substitutions

To save cost and lead time, several passive components (U2, J1, J2, etc.) were confirmed to use JLCPCB's in-stock alternatives (Basic Parts).

#### Critical Shortage: QMC5883L Magnetometer

- **Issue:** The magnetometer U6 (QMC5883L) showed zero stock at JLCPCB.
- **Vendor Suggestion:** Use "Global Sourcing" (worldwide procurement), but this process is complex and extends lead time significantly.
- **Decision:** Since this is a first-revision prototype validation, waiting for this single IC wasn't worth the delay.
  - Reduced assembly quantity to **10 boards** (small batch for initial testing)
  - Selected **Option A: Do Not Populate** for U6
  - Plan to hand-solder the magnetometer later or include it in the next revision

#### D1 RGB LED Shortage

- **Issue:** RGB LED (D1) also out of stock
- **Decision:** Selected "Do Not Populate" - getting the main board functional is the priority

#### Component Decision Matrix

| Reference | Component | Stock Status | Decision |
|-----------|-----------|--------------|----------|
| U2 | nRF52810 | Available (substitute) | Use alternative |
| U6 | QMC5883L | Out of stock | Do Not Populate |
| D1 | RGB LED | Out of stock | Do Not Populate |
| J1, J2 | Connectors | Available (substitute) | Use alternative |

---

### 3. DFM Engineering Review

After entering the engineering review phase, JLCPCB engineers (contact: Doris) identified several layout issues. Credit to their thorough review process.

#### Footprint Mismatch Issues

- **Issue:** Physical footprints for D1, J4, J5 did not match the PCB pad designs.
- **Resolution:**
  - D1: Already marked as "Do Not Populate" due to stock issues - skipped
  - J4, J5: Adjusted or confirmed based on engineer recommendations

#### RF Component Polarity Verification

- **Issue (9/27):** Question regarding chip antenna (AE1) placement orientation. This is critical for RF performance.
- **Verification Process:**
  1. Reopened KiCad project
  2. Cross-referenced component datasheet
  3. Confirmed layout orientation
  4. Verified **Pad 1 is the Feed pin**
  5. Current placement direction is correct
- **Status:** After confirmation reply, vendor responded "Will proceed with production"

#### Engineering Questions Timeline

| Date | Issue | Resolution |
|------|-------|------------|
| 9/20 | Missing drill layer | Reorder with complete files |
| 9/22 | Component substitutions | Approved alternatives |
| 9/25 | QMC5883L shortage | Do Not Populate |
| 9/27 | Antenna polarity check | Confirmed correct |
| 9/28 | All EQ resolved | Production started |

---

### 4. Key Takeaways

#### Gerber File Checklist

KiCad's export workflow is convenient, but missing the drill layer is an easy mistake that wastes 1-2 days of review time. Always verify ZIP contents before upload.

#### Prototype Strategy for Component Shortages

During prototyping phase, don't insist on having every component populated in one shot. When facing "Global Sourcing" delays that slow down the schedule, choose "Do Not Populate" decisively. Getting boards in hand to verify core functionality is what matters.

#### Communication Efficiency

JLCPCB's Engineering Questions (EQ) emails require quick responses. Typically, after confirming a response, they continue the process the next day. Don't let emails sit.

---

### 5. Next Steps

- [ ] Receive boards in Taiwan
- [ ] Functional testing (especially ESP32/nRF communication)
- [ ] Hand-solder unpopulated sensors (QMC5883L, RGB LED)
- [ ] Document test results
- [ ] Plan revision 2 if needed

---

### Cost Breakdown (Reference)

| Item | Quantity | Notes |
|------|----------|-------|
| PCB Fabrication | 10 pcs | 2-layer, standard specs |
| SMT Assembly | 10 pcs | Partial BOM (sensors DNP) |
| Shipping | 1 | DHL to Taiwan |

*Note: Actual costs vary based on component availability and shipping destination.*

---

## Credits

- **Hardware Design**: [Ultimate Robotics](https://github.com/ultimaterobotics)
- **Repository**: https://github.com/ultimaterobotics/uMyo_v3_1_pcb
