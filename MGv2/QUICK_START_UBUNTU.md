# ExoPulse Firmware - Ubuntu Quick Start

**For Native Ubuntu Environment**

---

## One-Time Setup (5 minutes)

```bash
# 1. Install PlatformIO
sudo apt update
sudo apt install -y python3 python3-pip
pip3 install --user platformio
export PATH="$HOME/.local/bin:$PATH"

# 2. Configure serial permissions
sudo usermod -a -G dialout $USER
# ⚠️ LOG OUT and LOG IN after this command

# 3. Navigate to project
cd /mnt/d/ExoPulse_firmware
```

---

## Find Your ESP32 Port

```bash
# Unplug ESP32, then run:
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Plug in ESP32, then run again:
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# New device that appears is your port (usually /dev/ttyUSB0)
# Note it here: _______________
```

---

## Test 1: LED Test (2 minutes)

```bash
cp examples/test_led.h src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
```

**Expected:** LED blinks in different patterns every 5 seconds

---

## Test 2: Loopback Test (3 minutes)

**⚠️ Requires MCP2515 wired to ESP32:**
- VCC → 3.3V, GND → GND
- SCK → GPIO18, MISO → GPIO19, MOSI → GPIO23, CS → GPIO5

```bash
cp examples/test_loopback.h src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Expected:**
```
[TX] Sent frame #0 ID=0x141 -> [RX] ID=0x141 ✓ PASS
...
--- Stats: Total=10, Success=10, Fail=0 ---
```

Press Ctrl+C to exit monitor.

---

## Test 3: Production Firmware (5 minutes)

```bash
git checkout src/main.cpp
pio run --environment esp32doit-devkit-v1
pio run -t upload --environment esp32doit-devkit-v1 --upload-port /dev/ttyUSB0
pio device monitor --baud 115200 --port /dev/ttyUSB0
```

**Try these commands:**
```
HELP
STATUS 1
DEBUG
STOP
```

**Expected:** Console responds to all commands.

Press Ctrl+C to exit monitor.

---

## If All Tests Pass

```bash
# Document results
echo "Hardware tests passed on $(date)" > HARDWARE_TEST_RESULTS.txt

# Merge to master
git checkout master
git merge bugfix/main-source-and-testing
```

---

## Troubleshooting

**Permission denied on /dev/ttyUSB0:**
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in
```

**Upload fails:**
```bash
# Hold BOOT button on ESP32 during upload
# Or try: sudo chmod 666 /dev/ttyUSB0
```

**MCP2515 init fails:**
```bash
# Verify all 6 wiring connections
# Check VCC has 3.3V power
# Ensure CS is on GPIO 5
```

---

## Full Documentation

- **HARDWARE_TEST_GUIDE.md** - Complete procedures
- **BUILD_REPORT.md** - Technical details
- **DEPLOYMENT_CHECKLIST.md** - Production deployment

---

**Total Time:** ~10 minutes for all tests

*ExoPulse Firmware - November 2025*
