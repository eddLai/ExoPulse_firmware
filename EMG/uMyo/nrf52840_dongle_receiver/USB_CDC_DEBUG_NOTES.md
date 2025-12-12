# nRF52840 Dongle USB CDC Debug Notes

## Summary

We successfully ported the uMyo receiver from nRF52840 DK (UART output) to nRF52840 Dongle (USB CDC output). The radio communication works, but USB CDC enumeration on the host is not yet functional.

## Key Differences: nRF52840 DK vs Dongle

### Hardware Differences

| Feature | nRF52840 DK | nRF52840 Dongle |
|---------|-------------|-----------------|
| Serial Output | UART via J-Link CDC | Native USB CDC |
| Programming | J-Link SWD | USB DFU Bootloader |
| Flash Address | 0x00000000 | 0x00001000 (after MBR) |
| RAM Start | 0x20000000 | 0x20000008 (MBR reserves 8 bytes) |
| LED Pin | P0.13 | P0.06 |
| USB Power | External or USB | USB only |

### Software Differences

| Aspect | DK | Dongle |
|--------|-----|--------|
| Serial Driver | Simple UART (hardware handles protocol) | USB CDC (software handles USB protocol) |
| Complexity | ~50 lines | ~400 lines |
| Requires | Just configure baud rate | Full USB stack: descriptors, endpoints, enumeration |
| Errata | None for UART | Multiple USB errata (171, 187) |

## Problems Encountered and Solutions

### 1. Firmware Crash After DFU Flash (SOLVED)

**Symptom:** Device disappeared from USB immediately after flashing, no LED activity.

**Root Cause:** Linker script had RAM starting at `0x20000000`, but the MBR (Master Boot Record) reserves the first 8 bytes for interrupt forwarding.

**Solution:** Changed linker script:
```
RAM (rwx) : ORIGIN = 0x20000008, LENGTH = 0x3FFF8
```

### 2. USB Regulator Not Ready (SOLVED)

**Symptom:** Firmware hung waiting for `POWER_USBREGSTATUS` bit 1 (regulator ready).

**Root Cause:** On nRF52840, the USB regulator status bits may not be reliable. Need to use the USBREG peripheral events instead.

**Solution:** Added USBREG peripheral definitions and use `USBREG_EVENTS_USBPWRRDY` or timeout:
```c
#define USBREG_BASE 0x40037000UL
#define USBREG_EVENTS_USBDETECTED (*(volatile uint32_t *)(USBREG_BASE + 0x100))
#define USBREG_EVENTS_USBPWRRDY   (*(volatile uint32_t *)(USBREG_BASE + 0x108))
```

### 3. USB Peripheral Won't Enable (SOLVED)

**Symptom:** `USBD_ENABLE = 1` had no effect, `USBD_EVENTCAUSE.READY` never set.

**Root Cause:** nRF52840 has multiple USB errata that require magic register writes:
- **Errata 171:** USB might not power up after enable
- **Errata 187:** USB cannot be enabled after soft reset (DFU counts as soft reset!)

**Solution:** Apply errata workarounds before and after enabling USBD:
```c
// Before USBD_ENABLE:
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006EC14 = 0x000000C0;  // Errata 171
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006ED14 = 0x00000003;  // Errata 187
*(volatile uint32_t *)0x4006EC00 = 0x00009375;

USBD_ENABLE = 1;
// Wait for EVENTCAUSE.READY...

// After USBD enabled:
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006EC14 = 0x00000000;
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
*(volatile uint32_t *)0x4006ED14 = 0x00000000;
*(volatile uint32_t *)0x4006EC00 = 0x00009375;
```

### 4. USB Pull-up Enabled But No Enumeration (CURRENT ISSUE)

**Symptom:**
- Minimal test shows all 7 blink stages complete (USBD enabled, pull-up set)
- LED blinks continuously indicating main loop running
- Host PC shows NO USB device at all in `lsusb`
- No `/dev/ttyACM*` device created

**Current Status:**
- USB peripheral initialization completes successfully (verified via LED blinks)
- Pull-up is enabled (`USBD_USBPULLUP = 1`)
- But host doesn't see the device

**Possible Causes Being Investigated:**
1. USB descriptors may have issues
2. EP0 setup packet handling may not be working
3. USB reset event handling may be missing
4. Something in the `usb_cdc_process()` function may be blocking enumeration

## Why This Took So Long

### 1. No Debug Access
- DK has J-Link for printf debugging and breakpoints
- Dongle only has LED blinks for debugging
- Each test requires: modify code → build → enter bootloader → flash → observe LED

### 2. Multiple Layered Issues
We had to solve problems in this order:
1. RAM address (crash)
2. USB regulator detection
3. Errata workarounds for USB enable
4. Now: actual USB protocol handling

### 3. Undocumented Errata Workarounds
- The errata workarounds use magic register addresses (`0x4006EC00`, `0x4006ED14`) not in the datasheet
- Had to search Nordic forums and SDK source code to find these
- Errata 187 is critical because DFU flash counts as "soft reset"

### 4. USB CDC Complexity
UART is hardware-based - configure and go. USB CDC requires:
- Device descriptor
- Configuration descriptor
- CDC functional descriptors
- String descriptors
- Endpoint configuration
- Setup packet handling (GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION)
- CDC class requests (SET_LINE_CODING, SET_CONTROL_LINE_STATE)
- Proper event handling in main loop

## Working Test Results

The minimal USB test (`usb_test_only.c`) successfully:
- Starts HFCLK (1 blink)
- Detects VBUS (2 blinks)
- Applies errata workarounds (3 blinks)
- Enables USBD (4 blinks)
- Gets EVENTCAUSE.READY (5 blinks)
- Cleans up errata (6 blinks)
- Enables pull-up (7 blinks)
- Runs main loop (continuous fast blinks)

This proves the USB hardware is working. The issue is in the protocol handling layer.

## Files Modified

- `nrf52840_dongle.ld` - RAM start address fix
- `usb_cdc.c` - Added USBREG peripheral, errata workarounds
- `usb_cdc.h` - Added `usb_cdc_write_ready()` function
- `main.c` - Adapted from DK UART output format
- `Makefile` - Build configuration

## Next Steps

1. Add LED blinks inside `usb_cdc_process()` to see if USB reset/setup events are being received
2. Verify EP0 buffer addresses are in valid RAM region
3. Check if USB descriptors need alignment attributes
4. Consider using Nordic SDK USB driver instead of bare-metal implementation

## References

- [Nordic Errata 187](https://infocenter.nordicsemi.com/topic/errata_nRF52840_Rev2/ERR/nRF52840/Rev2/latest/anomaly_840_187.html)
- [Nordic Errata 171](https://infocenter.nordicsemi.com/topic/errata_nRF52840_EngD/ERR/nRF52840/EngineeringD/latest/anomaly_840_171.html)
- [nrfx USBD driver source](https://github.com/NordicSemiconductor/nrfx/blob/master/drivers/src/nrfx_usbd.c)
