# UGREEN BLE 6.0 Dongle Driver Fix

## Problem

The UGREEN Bluetooth 6.0 USB dongle (ID `33fa:0012`) is not recognized by the Linux kernel on Jetson Orin (kernel 5.10).

When plugged in, `lsusb` shows:
```
Bus 001 Device 007: ID 33fa:0012
```

But the device has no name and is not bound to the `btusb` driver, so `hciconfig` does not show it.

## Cause

The device ID `33fa:0012` is not in the kernel's `btusb` driver device table. The kernel doesn't know this USB device is a Bluetooth adapter.

## Solution

Manually add the device ID to the `btusb` driver:

```bash
# Load btusb module
sudo modprobe btusb

# Add UGREEN BLE 6.0 device ID
echo "33fa 0012" | sudo tee /sys/bus/usb/drivers/btusb/new_id

# Verify - should show hci0 or hci1
hciconfig -a
```

## Persistent Fix (Survives Reboot)

Create a udev rule:

```bash
sudo nano /etc/udev/rules.d/99-ugreen-ble6.rules
```

Add this line:
```
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="33fa", ATTR{idProduct}=="0012", RUN+="/bin/sh -c 'echo 33fa 0012 > /sys/bus/usb/drivers/btusb/new_id'"
```

Then reload udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Device Info

| Property | Value |
|----------|-------|
| Vendor ID | 33fa |
| Product ID | 0012 |
| Brand | UGREEN |
| Type | Bluetooth 6.0 USB Adapter |
| Driver | btusb |

## References

- https://linux-hardware.org/?id=usb:33fa-0001
- https://aamnah.com/notes/sysadmin/troubleshoot-ugreen-bluetooth-5.4-ubuntu-linux/