# TP-Link UB500 Bluetooth Fix on Jetson (Kernel 5.10.192-tegra)

## Problem

The TP-Link UB500 USB Bluetooth adapter (RTL8761B, USB ID `2357:0604`) is not natively supported on kernel 5.10.x. The stock `btusb` kernel module does not include this device ID, so:

- The adapter is not recognized as a Realtek Bluetooth device
- RTL8761B firmware is not loaded
- BLE connections (e.g. for EMG sensor data) fail or are unstable

## Root Cause

The `btusb.c` driver in kernel 5.10 lacks the USB device entry for TP-Link UB500 (`0x2357:0x0604`). Without this entry mapped to `BTUSB_REALTEK`, the Realtek-specific initialization (including firmware loading) is skipped.

## Fix

### 1. Patch btusb.c

Download `btusb.c` and required headers from the v5.10 kernel source:

```bash
mkdir ~/ub500_fix && cd ~/ub500_fix
wget https://raw.githubusercontent.com/torvalds/linux/v5.10/drivers/bluetooth/btusb.c
wget https://raw.githubusercontent.com/torvalds/linux/v5.10/drivers/bluetooth/btintel.h
wget https://raw.githubusercontent.com/torvalds/linux/v5.10/drivers/bluetooth/btbcm.h
wget https://raw.githubusercontent.com/torvalds/linux/v5.10/drivers/bluetooth/btrtl.h
```

Add the following entry in `btusb.c` before the "Silicon Wave based devices" section:

```c
/* Tp-Link UB500 */
{ USB_DEVICE(0x2357, 0x0604), .driver_info = BTUSB_REALTEK },
```

### 2. Compile

Create `Makefile`:

```makefile
obj-m += btusb.o
KDIR := /usr/src/linux-headers-5.10.192-tegra-ubuntu20.04_aarch64/kernel-5.10
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
```

Build:

```bash
make clean && make
```

### 3. Install module and firmware

```bash
# Backup original
sudo cp /lib/modules/$(uname -r)/kernel/drivers/bluetooth/btusb.ko \
       /lib/modules/$(uname -r)/kernel/drivers/bluetooth/btusb.ko.bak

# Install patched module
sudo cp btusb.ko /lib/modules/$(uname -r)/kernel/drivers/bluetooth/btusb.ko

# Download RTL8761B firmware
sudo curl -s https://raw.githubusercontent.com/Realtek-OpenSource/android_hardware_realtek/rtk1395/bt/rtkbt/Firmware/BT/rtl8761b_fw \
     -o /lib/firmware/rtl_bt/rtl8761b_fw.bin

# Reload module
sudo modprobe -r btusb
sudo modprobe -v btusb
```

### 4. Verify

```bash
hciconfig -a        # Should show UP RUNNING, firmware revision updated
bluetoothctl show    # Should show Powered: yes
bluetoothctl devices # Should discover nearby devices
```

After fix, `dmesg` should show:
```
Bluetooth: hci0: RTL: loading rtl_bt/rtl8761b_fw.bin
Bluetooth: hci0: RTL: fw version 0x097bec43
```

## Reference

- https://github.com/tedboudros/tplink-ub500-linux-patch-guide
