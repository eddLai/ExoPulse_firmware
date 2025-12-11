# uMyo Radio Transmission Protocols

The uMyo firmware supports three different radio transmission modes, each with different characteristics for data rate, range, and receiver requirements.

## Overview

| Protocol | Data Rate | Receiver | Range | Power | Use Case |
|----------|-----------|----------|-------|-------|----------|
| BLE Advertisement | ~10-100 Hz | Any BLE device (phone, ESP32, PC) | ~10-30m | Low | General use, mobile apps |
| Fast32 | ~1000 Hz | nRF52 with star_protocol | ~10-20m | Medium | Real-time EMG |
| Fast64 | ~1000 Hz | nRF52 with star_protocol | ~10-20m | Medium | Real-time EMG + IMU |

---

## 1. BLE Advertisement Mode (`radio_mode_ble`)

### Description
Standard Bluetooth Low Energy non-connectable advertisements. The uMyo broadcasts data packets that any BLE scanner can receive.

### Technical Details
- **Protocol**: BLE 4.0+ ADV_NONCONN_IND
- **Channels**: 37, 38, 39 (advertising channels)
- **Packet Size**: Up to 31 bytes payload
- **Interval**: Configurable (typically 10-100ms)
- **Data Rate**: 10-100 packets/second

### Packet Structure
```
[Preamble][Access Addr][PDU Header][MAC 6B][AD Fields][CRC]

AD Fields:
- Flags (0x01): 1 byte
- Short Name (0x08): "uMyo" or "uMyo v2"
- Manufacturer Data (0xFF): Counter + Battery + ADC data
```

### Advantages
- ✅ Universal compatibility (phones, tablets, PCs, ESP32)
- ✅ No pairing required
- ✅ Multiple receivers can listen simultaneously
- ✅ Low power consumption
- ✅ Good range (~30m line of sight)

### Disadvantages
- ❌ Limited bandwidth (~100 packets/sec max)
- ❌ No acknowledgment (packets can be lost)
- ❌ Higher latency due to channel hopping

### Receiver Options
- ESP32 with BLE scan
- Smartphone with BLE app
- PC with BLE dongle
- Raspberry Pi with BLE

### Code Reference
```c
// In main.c
void switch_to_ble()
{
    ble_init_radio();
    // Uses ble_send_advertisement_packet()
}
```

---

## 2. Fast32 Mode (`radio_mode_fast32`)

### Description
Nordic proprietary 2.4GHz radio protocol with 32-byte packets. Uses direct radio communication without BLE overhead for maximum speed.

### Technical Details
- **Protocol**: Nordic Enhanced ShockBurst compatible
- **Channel**: 83 (2.483 GHz)
- **Packet Size**: 32 bytes
- **Data Rate**: 1 Mbps RF, ~1000 packets/second application
- **Addressing**: 5-byte address

### Packet Structure
```
[Preamble 1B][Address 5B][Payload 32B][CRC 2B]

Payload (32 bytes):
- Packet ID: 1 byte
- Length: 1 byte
- Unit ID: 4 bytes
- ADC Samples: 8 x 16-bit = 16 bytes
- Spectrum Data: 4 x 16-bit = 8 bytes
- Checksum: 2 bytes
```

### Advantages
- ✅ High data rate (~1000 Hz sampling)
- ✅ Low latency (<2ms)
- ✅ Simple protocol, low overhead
- ✅ Good for single sensor streaming

### Disadvantages
- ❌ Requires nRF52 receiver (not compatible with BLE devices)
- ❌ Single channel - no frequency hopping
- ❌ No collision avoidance for multiple sensors
- ❌ Shorter range than BLE in noisy environments

### Receiver Requirements
- nRF52 series microcontroller
- Running `rf_init_ext()` with matching parameters
- USB-serial connection to PC

### Code Reference
```c
// In main.c
void switch_to_fr32()
{
    rf_init_ext(83, 1000, 0, 0, 0, 0, 32);
    // Channel 83, 1000 kbps, 32-byte packets
}
```

---

## 3. Fast64 Mode (`radio_mode_fast64`) - Star Protocol

### Description
Custom time-division multiple access (TDMA) protocol supporting multiple sensors. Uses 64-byte packets with synchronized timing for coordinated multi-sensor operation.

### Technical Details
- **Protocol**: Star Protocol (custom TDMA)
- **Channel**: 21 (2.421 GHz)
- **Packet Size**: 64 bytes
- **Phase Length**: 2000 μs per slot
- **Data Rate**: 1 Mbps RF, ~500 packets/second per sensor
- **Topology**: Star (central hub + multiple nodes)

### Star Protocol Architecture
```
        [Central Hub]
        /    |    \
       /     |     \
   [Node1] [Node2] [Node3]

Time Division:
|--Slot1--|--Slot2--|--Slot3--|--Beacon--|
   Node1     Node2     Node3     Central
```

### Packet Structure
```
[Preamble][Address][Payload 64B][CRC]

Payload (64 bytes):
- Packet ID: 1 byte
- Data Length: 1 byte
- Unit ID: 4 bytes
- Parameter ID: 1 byte
- Battery Level: 1 byte
- ADC Data ID: 1 byte
- ADC Samples: 8 x 16-bit = 16 bytes
- Spectrum: 4 x 16-bit = 8 bytes
- Quaternion: 4 x 16-bit = 8 bytes
- Accelerometer: 3 x 16-bit = 6 bytes
- Angles (Yaw/Pitch/Roll): 3 x 16-bit = 6 bytes
- Magnetometer: 3 x 16-bit = 6 bytes
- Padding/Reserved: remaining bytes
```

### Advantages
- ✅ Supports multiple synchronized sensors
- ✅ High data rate with full IMU data
- ✅ Deterministic timing (TDMA)
- ✅ Central can send commands to nodes
- ✅ Synchronized timestamps across sensors

### Disadvantages
- ❌ Requires nRF52 central hub
- ❌ More complex setup
- ❌ Fixed time slots limit flexibility
- ❌ Not compatible with standard BLE

### Receiver Requirements
- nRF52 series as central hub
- Running `star_init()` with `is_central=1`
- Must call `star_loop_step()` regularly

### Code Reference
```c
// In main.c
void switch_to_fr64()
{
    star_init(21, 1000, 2000, 0);
    // Channel 21, 1000 kbps, 2000μs phase, node mode (0)
}

// In main loop
if(radio_mode == radio_mode_fast64)
    star_loop_step();
```

### Star Protocol API
```c
// urf_star_protocol.h
void star_init(int channel, int speed, int phase_length_mcs, int is_central);
void star_set_id(uint32_t id);
void star_queue_send(uint8_t *pack, int length);
void star_send_to_node(uint32_t node_id, uint8_t *payload_8b);
int star_has_packet();
int star_get_packet(uint8_t *pack, int max_length);
void star_loop_step();
uint32_t star_get_synced_time();
```

---

## Mode Switching

The uMyo cycles through modes with button presses:

```
Fast64 → Fast32 → BLE → Fast64 → ...
```

### Button Actions
- **Short press** (<1 second): Switch radio mode
- **Long press** (>1 second): Other functions (calibration, etc.)

### LED Indicators
| Mode | LED Color |
|------|-----------|
| BLE | Blue (3 pulses) |
| Fast32 | Magenta (3 pulses) |
| Fast64 | Green (3 pulses) |

---

## Choosing the Right Mode

### Use BLE when:
- You need smartphone/tablet compatibility
- You're using ESP32 or standard BLE receivers
- You only need ~100 Hz data rate
- You want easy setup with no custom receiver

### Use Fast32 when:
- You need 1kHz+ sampling for single sensor
- You have an nRF52 receiver available
- Latency is critical (<2ms)
- You don't need multi-sensor sync

### Use Fast64 when:
- You have multiple uMyo sensors
- You need synchronized data across sensors
- You need full EMG + IMU data at high rate
- You have an nRF52 central hub

---

## Receiver Implementation Examples

### BLE Receiver (ESP32)
```cpp
// ESP32 Arduino
#include <BLEDevice.h>
#include <BLEScan.h>

class MyCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
        // Parse uMyo advertisement data
        uint8_t* payload = dev.getPayload();
        // Extract ADC samples from manufacturer data
    }
};
```

### Fast32/Fast64 Receiver (nRF52)
```c
// nRF52 receiver
#include "urf_radio.h"
#include "urf_star_protocol.h"

// For Fast32:
rf_init_ext(83, 1000, 0, 0, 0, 0, 32);

// For Fast64:
star_init(21, 1000, 2000, 1);  // is_central=1

while(1) {
    star_loop_step();
    if(star_has_packet()) {
        uint8_t data[64];
        int len = star_get_packet(data, 64);
        // Process received data
    }
}
```

---

## Data Rate Comparison

| Mode | Packets/sec | Samples/packet | Effective Sample Rate |
|------|-------------|----------------|----------------------|
| BLE (standard) | 10 | 1 | 10 Hz |
| BLE (fast) | 100 | 10 | 1000 Hz |
| Fast32 | 500 | 8 | 4000 Hz |
| Fast64 | 500 | 8 | 4000 Hz |

*Note: BLE with batched samples can achieve 1kHz effective rate by sending 10 samples per packet at 100 packets/second.*
