# BLE Advertisement Explained

## What is BLE Advertisement?

BLE (Bluetooth Low Energy) advertisement is a **one-way broadcast mechanism** where a device periodically transmits small packets of data to any nearby receiver. It's like a radio station broadcasting - anyone with a receiver can listen, but there's no two-way communication.

```
┌─────────────┐                    ┌─────────────┐
│   uMyo      │  ~~~~ RF ~~~~>     │   ESP32     │
│  (Beacon)   │   Advertisement    │  (Scanner)  │
│             │   every 100ms      │             │
└─────────────┘                    └─────────────┘

      No connection required!
      Just broadcast and listen.
```

## Why Use Advertisement?

| Feature | Advertisement Mode | Connection Mode |
|---------|-------------------|-----------------|
| Power consumption | Very low | Higher |
| Data direction | One-way (broadcast) | Two-way |
| Multiple receivers | Yes (unlimited) | No (one at a time) |
| Setup required | None | Pairing/bonding |
| Data size | Small (31 bytes max) | Large |
| Latency | Fixed interval | Variable |

**uMyo uses advertisement because:**
- Multiple ESP32s can receive data simultaneously
- No pairing required - just power on and it works
- Lower power consumption for battery-operated device
- Simpler firmware implementation

## BLE Advertising Channels

BLE uses 3 dedicated advertising channels to avoid interference:

```
2.4 GHz Band
├── Channel 37 (2402 MHz) ◄── Advertising
├── Channel 0-10 (Data)
├── Channel 38 (2426 MHz) ◄── Advertising
├── Channel 11-36 (Data)
└── Channel 39 (2480 MHz) ◄── Advertising
```

The uMyo cycles through all 3 advertising channels:
```c
static int adv_ch = 37;
// ...
adv_ch++;
if(adv_ch > 39) adv_ch = 37;  // Cycle: 37 → 38 → 39 → 37...
```

## Advertisement Packet Structure

A BLE advertisement packet has this structure:

```
┌──────────────────────────────────────────────────────────────────┐
│                    BLE Advertisement Packet                       │
├──────────────┬───────────────────────────────────────────────────┤
│   Preamble   │  1 byte (0xAA pattern for receiver sync)          │
├──────────────┼───────────────────────────────────────────────────┤
│ Access Addr  │  4 bytes (0x8E89BED6 for all advertisements)      │
├──────────────┼───────────────────────────────────────────────────┤
│   PDU        │  2-39 bytes (the actual data - see below)         │
├──────────────┼───────────────────────────────────────────────────┤
│    CRC       │  3 bytes (error checking)                         │
└──────────────┴───────────────────────────────────────────────────┘
```

## PDU (Protocol Data Unit) Structure

The PDU is the "payload" of the advertisement:

```
┌─────────────────────────────────────────────────────────────────┐
│                         PDU Structure                            │
├─────────────┬───────────────────────────────────────────────────┤
│   Header    │  2 bytes                                          │
│             │  ├── PDU Type (4 bits): Type of advertisement     │
│             │  ├── TxAdd (1 bit): 0=Public, 1=Random address    │
│             │  ├── RxAdd (1 bit): Target address type           │
│             │  └── Length (6 bits): Payload length              │
├─────────────┼───────────────────────────────────────────────────┤
│   Payload   │  6-37 bytes                                       │
│             │  ├── AdvA (6 bytes): Advertiser's MAC address     │
│             │  └── AdvData (0-31 bytes): Advertisement data     │
└─────────────┴───────────────────────────────────────────────────┘
```

## Advertisement Types

| PDU Type | Name | Description |
|----------|------|-------------|
| 0x00 | ADV_IND | Connectable, scannable (most common) |
| 0x01 | ADV_DIRECT_IND | Directed to specific device |
| 0x02 | **ADV_NONCONN_IND** | Non-connectable, non-scannable **(uMyo uses this)** |
| 0x03 | SCAN_REQ | Scanner requesting more data |
| 0x04 | SCAN_RSP | Response to scan request |
| 0x06 | ADV_SCAN_IND | Scannable but not connectable |

**uMyo uses ADV_NONCONN_IND** because it only broadcasts data and doesn't need connections.

## Advertisement Data (AdvData) Format

The AdvData contains multiple "AD Structures", each with:

```
┌─────────┬─────────┬──────────────────┐
│ Length  │  Type   │      Data        │
│ 1 byte  │ 1 byte  │  (Length-1) bytes│
└─────────┴─────────┴──────────────────┘
```

Common AD Types:

| Type | Name | Description |
|------|------|-------------|
| 0x01 | Flags | Device capabilities |
| 0x08 | Short Name | Shortened device name |
| 0x09 | Complete Name | Full device name |
| 0xFF | Manufacturer Data | Custom vendor data |

## uMyo Advertisement Data Breakdown

Here's what uMyo broadcasts:

```
uMyo AdvData (31 bytes max):
┌────────────────────────────────────────────────────────────────┐
│ MAC Address (6 bytes)                                          │
│   D4:xx:xx:xx:xx:D7  (derived from device ID)                 │
├────────────────────────────────────────────────────────────────┤
│ AD Structure 1: Flags                                          │
│   [02] [01] [00]                                               │
│    │    │    └── Flags value (0x00)                           │
│    │    └────── AD Type: Flags                                │
│    └─────────── Length: 2                                      │
├────────────────────────────────────────────────────────────────┤
│ AD Structure 2: Short Name                                     │
│   [08] [08] [75 4D 79 6F 20 76 32]                            │
│    │    │    └── "uMyo v2" in ASCII                           │
│    │    └────── AD Type: Short Name                           │
│    └─────────── Length: 8                                      │
├────────────────────────────────────────────────────────────────┤
│ AD Structure 3: Manufacturer Specific Data                     │
│   [XX] [FF] [counter] [battery] [spectrum...] [quaternion...]  │
│    │    │    └── Custom uMyo sensor data                      │
│    │    └────── AD Type: Manufacturer Data                    │
│    └─────────── Length: varies                                 │
└────────────────────────────────────────────────────────────────┘
```

## uMyo Code That Builds the Advertisement

From `main.c`:

```c
int prepare_and_send_BLE()
{
    // 1. Set MAC address (6 bytes)
    uint32_t unit_id = NRF_FICR->DEVICEID[1];
    ble_mac[0] = 0xD4;
    ble_mac[1] = (unit_id>>24)&0xFF;
    ble_mac[2] = (unit_id>>16)&0xFF;
    ble_mac[3] = (unit_id>>8)&0xFF;
    ble_mac[4] = (unit_id)&0xFF;
    ble_mac[5] = 0xD7;

    // 2. Add MAC to PDU data
    for(int x = 0; x < 6; x++)
        pdu_data[pp++] = ble_mac[x];

    // 3. Add Flags (AD Type 0x01)
    sBLE_flags flags;
    flags.value = 0;
    data[0] = flags.value;
    pp = ble_add_field_to_pdu(pdu_data, pp, data, 1, PDU_FLAGS);

    // 4. Add Short Name "uMyo v2" (AD Type 0x08)
    int nlen = sprintf((char*)data, "uMyo v2");
    pp = ble_add_field_to_pdu(pdu_data, pp, data, nlen, PDU_SHORT_NAME);

    // 5. Add Manufacturer Data (AD Type 0xFF)
    data[dpos++] = counter++;     // Packet counter
    data[dpos++] = 100;           // Battery level
    pp = ble_add_field_to_pdu(pdu_data, pp, data, dpos, PDU_MANUFACTURER_SPEC);

    // 6. Build PDU with header
    int len = ble_prepare_adv_pdu(pdu, pp, pdu_data, BLE_ADV_NONCONN_IND_TYPE, 0, 1);

    // 7. Transmit on advertising channel
    ble_send_advertisement_packet(len, pdu, adv_ch);
}
```

## Advertisement Timing

```
Time ──────────────────────────────────────────────────────────►

     ┌──┐         ┌──┐         ┌──┐         ┌──┐
     │37│         │38│         │39│         │37│
     └──┘         └──┘         └──┘         └──┘
     ▲            ▲            ▲            ▲
     │            │            │            │
   t=0ms       t=100ms      t=200ms      t=300ms

   uMyo advertises every 100ms, cycling through channels
```

The uMyo main loop:
```c
if(ms - last_ble_send > 100) {  // Every 100ms
    prepare_and_send_BLE();
    last_ble_send = ms;
}
```

## How ESP32 Receives the Advertisement

```
1. ESP32 BLE scanner listens on channels 37, 38, 39
2. When packet received, callback is triggered
3. Callback extracts:
   - MAC address
   - Device name (if present)
   - RSSI (signal strength)
   - Raw payload data
4. Scanner searches payload for "uMyo" string
5. If found, device is identified as uMyo
```

## Summary

| Term | Meaning |
|------|---------|
| Advertisement | Periodic broadcast of small data packets |
| PDU | The actual data packet structure |
| AD Structure | Individual data fields within the advertisement |
| Channel | Radio frequency used (37, 38, or 39 for ads) |
| RSSI | Received Signal Strength Indicator (dBm) |
| Non-connectable | Device broadcasts only, no two-way connection |

The uMyo acts as a **BLE beacon** - it continuously broadcasts its sensor data (EMG, battery, IMU) so any nearby receiver (like the ESP32) can pick it up without establishing a connection.
