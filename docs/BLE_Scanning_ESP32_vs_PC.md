# Why ESP32 Can Scan uMyo but PC Cannot

## Summary

The ESP32 successfully detects the uMyo nRF52810 BLE device while PC Bluetooth (using `bluetoothctl`) does not show it. This document explains the technical reasons behind this behavior.

## Test Results

| Scanner | uMyo Detected? | Details |
|---------|---------------|---------|
| ESP32 BLE Scanner | Yes | `d7:72:58:3a:82:d4 RSSI: -47 [uMyo in payload]` |
| PC bluetoothctl | No | Device not listed in scan results |

## Root Causes

### 1. Advertisement Type: Non-Connectable

The uMyo firmware broadcasts using `BLE_ADV_NONCONN_IND_TYPE` (non-connectable, non-scannable advertisement):

```c
// From main.c prepare_and_send_BLE()
ble_set_connection_mode(0);  // Non-connectable
int len = ble_prepare_adv_pdu(pdu, pp, pdu_data, BLE_ADV_NONCONN_IND_TYPE, 0, 1);
```

**Impact:** Many PC Bluetooth stacks and scanning tools filter out non-connectable devices by default, as they are primarily designed for connecting to peripherals (keyboards, mice, headphones). The ESP32's raw BLE scanner does not apply this filter.

### 2. Device Name Location in PDU

The uMyo device name "uMyo v2" is embedded in the advertisement payload using `PDU_SHORT_NAME`:

```c
int nlen = sprintf((char*)data, "uMyo v2");
pp = ble_add_field_to_pdu(pdu_data, pp, data, nlen, PDU_SHORT_NAME);
```

**How scanning differs:**

| Scanner | Name Detection Method |
|---------|----------------------|
| ESP32 | Scans raw payload bytes, finds "uMyo" at any position |
| PC | Expects name in specific AD Type fields, may not parse all payload |

The ESP32 scanner explicitly searches for "uMyo" in the raw payload:
```cpp
for (int i = 0; i < (int)payloadLen - 4; i++) {
    if (payload[i] == 'u' && payload[i+1] == 'M' &&
        payload[i+2] == 'y' && payload[i+3] == 'o') {
        Serial.print(" [uMyo in payload]");
        break;
    }
}
```

### 3. Scan Response Not Sent

For non-connectable advertisements, the device does not respond to scan requests. PC Bluetooth often relies on scan responses to retrieve the complete device name:

```
PC Scan Request  →  [No Response from uMyo]
                    (because non-connectable mode)
```

The ESP32 passive scanner doesn't require scan responses - it just reads the advertisement packet directly.

### 4. BlueZ Stack Filtering (Linux)

On Linux, the BlueZ Bluetooth stack (used by `bluetoothctl`) applies several filters:

1. **Duplicate filtering**: May ignore repeated advertisements from the same device
2. **RSSI threshold**: May filter weak signals (though uMyo signal is strong at -47 dBm)
3. **Advertisement type filtering**: May prioritize connectable devices
4. **Cache behavior**: May not immediately show new devices

### 5. Random vs Public Address

The uMyo uses a random static address (indicated by `tx_addr = 1` in the PDU):

```c
int len = ble_prepare_adv_pdu(pdu, pp, pdu_data, BLE_ADV_NONCONN_IND_TYPE, 0, 1);
//                                                                            ^ tx_addr=1 (random)
```

Some PC scanning tools default to showing only devices with public addresses.

## BLE Advertisement Packet Structure

```
uMyo Advertisement PDU:
┌─────────────────────────────────────────────────────────────┐
│ Header (1 byte)                                             │
│   - PDU Type: ADV_NONCONN_IND (0x02)                       │
│   - TxAdd: 1 (Random Address)                               │
├─────────────────────────────────────────────────────────────┤
│ Length (1 byte)                                             │
├─────────────────────────────────────────────────────────────┤
│ Payload:                                                    │
│   - MAC Address (6 bytes): D4:xx:xx:xx:xx:D7               │
│   - Flags AD (3 bytes): len=2, type=0x01, value=0x00       │
│   - Short Name AD: len=8, type=0x08, "uMyo v2"             │
│   - Manufacturer Data AD: counter, battery, etc.            │
└─────────────────────────────────────────────────────────────┘
```

## Solutions

### Option 1: Use ESP32 for Scanning (Recommended)

The ESP32 BLE scanner provides:
- Raw payload access
- No filtering of non-connectable devices
- Custom parsing for uMyo-specific data
- Reliable detection

### Option 2: Modify uMyo Firmware for PC Compatibility

To make uMyo visible to PC Bluetooth:

```c
// Change from non-connectable to scannable
ble_set_connection_mode(1);  // Enable connectable mode
int len = ble_prepare_adv_pdu(pdu, pp, pdu_data, BLE_ADV_IND_TYPE, 0, 1);
//                                                ^^^^^^^^^^^^^^^^
//                                                Use ADV_IND instead of ADV_NONCONN_IND
```

**Trade-off:** This increases power consumption and changes the device's BLE behavior.

### Option 3: Use hcitool with Raw Mode (Linux)

```bash
sudo hcitool lescan --duplicates --passive
```

Or use `btmgmt` for more control:
```bash
sudo btmgmt find -l
```

### Option 4: Use nRF Connect App (Mobile)

The nRF Connect app (Android/iOS) shows all BLE advertisements including non-connectable devices, similar to the ESP32 scanner.

## Conclusion

The ESP32 detects the uMyo device because it performs raw BLE scanning without the filtering applied by typical PC Bluetooth stacks. The uMyo's non-connectable advertisement type and payload-embedded name are fully visible to the ESP32's low-level BLE API but may be filtered or not parsed correctly by PC Bluetooth tools.

For production use with uMyo devices, an ESP32-based scanner or the official uMyo_BLE library is recommended.
