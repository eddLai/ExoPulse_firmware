# CAN Bus Wiring Checklist - ESP32 + MCP2515
## Two-Board Communication Setup

**Date:** 2025-11-19
**Issue:** CAN communication failing with TXErr=128
**Diagnosis:** Physical CAN bus connection required

---

## Current Status

✅ **Board 1 (Sender):** MCP2515 initialized successfully
✅ **Board 2 (Receiver):** MCP2515 initialized successfully
❌ **CAN Communication:** FAILED - No messages transmitted/received
❌ **Error:** Sender TXErr=128 (bus-off condition)

**Root Cause:** CAN_H and CAN_L physical bus connection missing or faulty

---

## Complete Wiring Required

### Board 1 - ESP32 to MCP2515 (SPI Connection)
```
ESP32 Pin          →  MCP2515 Pin       Status
=====================================  ========
GPIO 5  (CS)       →  CS               ✅ OK
GPIO 18 (SCK)      →  SCK              ✅ OK
GPIO 19 (MISO)     →  MISO (SO)        ✅ OK
GPIO 23 (MOSI)     →  MOSI (SI)        ✅ OK
3.3V or 5V         →  VCC              ✅ OK (verified in loopback test)
GND                →  GND              ✅ OK
GPIO X (optional)  →  INT              ⚠️  NOT USED (polling mode)
```

### Board 2 - ESP32 to MCP2515 (SPI Connection)
```
ESP32 Pin          →  MCP2515 Pin       Status
=====================================  ========
GPIO 5  (CS)       →  CS               ✅ OK
GPIO 18 (SCK)      →  SCK              ✅ OK
GPIO 19 (MISO)     →  MISO (SO)        ✅ OK
GPIO 23 (MOSI)     →  MOSI (SI)        ✅ OK
3.3V or 5V         →  VCC              ✅ OK (verified in loopback test)
GND                →  GND              ✅ OK
GPIO X (optional)  →  INT              ⚠️  NOT USED (polling mode)
```

---

## ⚠️ CRITICAL: CAN Bus Physical Connection (MISSING!)

### CAN Bus Twisted Pair Connection
```
Board 1 MCP2515              Board 2 MCP2515
===============================================
    CAN_H  ─────[wire]─────  CAN_H
    CAN_L  ─────[wire]─────  CAN_L
    GND    ─────[wire]─────  GND (common ground)
```

**Status:** ❌ MISSING - This is why communication fails!

### CAN Bus Termination (REQUIRED!)
```
Board 1:
    CAN_H ─┬─ [120Ω resistor] ─┬─ CAN_L
           │                    │
        [to Board 2]        [to Board 2]

Board 2:
    CAN_H ─┬─ [120Ω resistor] ─┬─ CAN_L
           │                    │
        [to Board 1]        [to Board 1]
```

**Status:** ❌ UNKNOWN - Must have 120Ω on BOTH ends!

---

## Why Communication is Failing

### Error Analysis
**Sender Error Code:** `rc=6` or `rc=7`
**TX Error Count:** `TXErr=128` (maximum, bus-off state)
**RX Error Count:** `RXErr=0`

### What This Means
1. **Sender transmits** a CAN frame successfully to its MCP2515 chip (via SPI) ✅
2. **MCP2515 tries to send** the frame on CAN_H/CAN_L lines
3. **No other node ACKs** the message (because bus not connected) ❌
4. **Sender retries** multiple times
5. **Error counter increases** with each failed transmission
6. **At 128 errors**, sender enters "bus-off" state and stops trying

### CAN Protocol Requirement
**Every CAN message MUST be acknowledged by at least one other node on the bus.**

If no ACK is received → Error counter increases
If error counter reaches 128 → Bus-off state (communication stops)

This is exactly what we're seeing!

---

## Required Actions (In Order)

### Step 1: Verify MCP2515 Module Pinout
Many MCP2515 modules have different pinouts. Common variations:

**Type A (Most Common):**
```
VCC  GND  CS  SO  SI  SCK  INT  [CAN_L]  [CAN_H]
```

**Type B:**
```
VCC  CS  SI  SO  SCK  INT  [CANH]  [CANL]  GND
```

**CHECK YOUR MODULES:** Identify which pins are CAN_H (CANH) and CAN_L (CANL)

### Step 2: Physical CAN Bus Connection
```
Materials needed:
- 2x wires (twisted pair recommended, or 2 separate wires)
- 2x 120Ω resistors (1/4W or similar)
- 1x wire for common ground

Connections:
1. Wire Board 1 CAN_H to Board 2 CAN_H
2. Wire Board 1 CAN_L to Board 2 CAN_L
3. Wire Board 1 GND to Board 2 GND (common ground)
4. Install 120Ω resistor between CAN_H and CAN_L on Board 1
5. Install 120Ω resistor between CAN_H and CAN_L on Board 2
```

### Step 3: Verification Checks

**Visual Inspection:**
- [ ] CAN_H to CAN_H wire connected
- [ ] CAN_L to CAN_L wire connected
- [ ] GND to GND wire connected
- [ ] 120Ω resistor on Board 1 (between CAN_H and CAN_L)
- [ ] 120Ω resistor on Board 2 (between CAN_H and CAN_L)
- [ ] No loose connections
- [ ] No swapped wires (CAN_H not connected to CAN_L)

**Multimeter Checks:**
- [ ] Continuity between Board 1 CAN_H and Board 2 CAN_H
- [ ] Continuity between Board 1 CAN_L and Board 2 CAN_L
- [ ] Resistance between CAN_H and CAN_L on Board 1 ≈ 120Ω
- [ ] Resistance between CAN_H and CAN_L on Board 2 ≈ 120Ω
- [ ] Total resistance CAN_H to CAN_L across both boards ≈ 60Ω (two 120Ω in parallel)

### Step 4: Re-test After Wiring
Once physical connections are made:
1. Power cycle both boards
2. Run the test again
3. Expected result: Messages transmit successfully!

---

## Expected Success Output

**When properly wired, you should see:**

**Sender Board:**
```
[TX] #0 ID=0x100 Data: AA 55 00 00 00 00 00 00 ✓ OK
[TX] #1 ID=0x100 Data: AA 55 00 00 00 01 00 00 ✓ OK
[TX] #2 ID=0x100 Data: AA 55 00 00 00 02 00 00 ✓ OK
...
--- Statistics ---
Total: 10 | Success: 10 | Failed: 0 | Rate: 100.0%
```

**Receiver Board:**
```
[RX] #0 ID=0x100 Len=8 Data: AA 55 00 00 00 00 00 00 | Counter=0
[RX] #1 ID=0x100 Len=8 Data: AA 55 00 00 00 01 00 00 | Counter=1
[RX] #2 ID=0x100 Len=8 Data: AA 55 00 00 00 02 00 00 | Counter=2
...
--- Statistics ---
Received: 10 messages
```

---

## Troubleshooting After Wiring

### If still failing after connecting CAN_H/CAN_L:

**1. Verify Termination Resistors**
- Must have 120Ω on BOTH ends of the bus
- Without terminators: signal reflections cause errors
- Check: Measure resistance between CAN_H and CAN_L should be ~60Ω total

**2. Check for Swapped Wires**
- CAN_H must connect to CAN_H (not CAN_L)
- CAN_L must connect to CAN_L (not CAN_H)
- Swap the wires if communication still fails

**3. Verify Common Ground**
- Both ESP32 boards must share common ground
- CAN requires common ground reference for signal levels

**4. Check Baud Rate Match**
- Both boards using 500KBPS @ 8MHz ✅ (already verified in code)

**5. Cable Length**
- For 500KBPS: Maximum cable length ~100 meters
- For testing: Keep cables short (<1 meter)

---

## Why Loopback Test Worked But CAN Bus Doesn't

**Loopback Test (PASSED ✅):**
- MCP2515 in LOOPBACK mode
- Messages sent internally within the chip
- No external CAN bus required
- No termination resistors needed
- Tests: SPI communication only

**CAN Bus Test (FAILED ❌):**
- MCP2515 in NORMAL mode
- Messages must be sent on physical CAN_H/CAN_L wires
- Requires second node on bus to ACK messages
- Requires termination resistors
- Tests: Full CAN protocol stack

**This is why loopback passed but CAN communication fails!**

---

## Common MCP2515 Module Types

### Module with Screw Terminals (Most Common)
```
Pinout:
[VCC] [GND] [CS] [SO] [SI] [SCK] [INT]   [L] [H]
                                          CAN CAN
                                          _L  _H
```

### Module with Pin Headers
```
Pinout may vary:
VCC  CS  MOSI MISO SCK  INT  CANL CANH GND
or
GND CANH CANL INT  SCK  MISO MOSI CS  VCC
```

**ACTION REQUIRED:** Physically identify CAN_H and CAN_L pins on your modules!

---

## Next Steps

1. **Identify CAN_H and CAN_L pins** on both MCP2515 modules
2. **Connect the physical CAN bus** (CAN_H to CAN_H, CAN_L to CAN_L, GND to GND)
3. **Install 120Ω termination resistors** on both ends
4. **Re-run the test** - communication should work!
5. **Report results** so we can document the complete working setup

---

## Reference

- MCP2515 Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
- CAN Bus Basics: Requires differential signaling (CAN_H/CAN_L) and termination
- Baud Rate: 500 KBPS (configured in both sender and receiver)
- Crystal: 8 MHz (auto-detected in initialization)

---

**This checklist will be updated once physical wiring is completed and tested.**
