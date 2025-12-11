#!/usr/bin/env python3
"""
uMyo Connection Diagnostic Tool

Tests connection to uMyo in all 3 radio modes:
- BLE Advertisement (blue LED, 3 pulses)
- Fast32 (magenta LED, 3 pulses)
- Fast64 / Star Protocol (green LED, 3 pulses)

This tool helps diagnose connectivity issues with the nRF52840 DK receiver.
"""

import serial as pyserial
import serial.tools.list_ports
import sys
import time
import argparse
import re

def list_ports():
    """List available serial ports."""
    print("\n=== Available Serial Ports ===")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  No serial ports found!")
        return []
    for p in ports:
        print(f"  {p.device}: {p.description}")
    return ports

def test_connection(port, baud=115200, duration=10):
    """Test serial connection and analyze incoming data."""
    print(f"\n=== Testing {port} at {baud} baud ===")

    try:
        ser = pyserial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"ERROR: Cannot open {port}: {e}")
        return False

    print(f"Connected! Listening for {duration} seconds...")
    print("-" * 60)

    start_time = time.time()
    packet_count = 0
    byte_count = 0
    lines = []
    raw_bytes = bytearray()

    # Data pattern detection
    rx_packets = 0
    status_msgs = 0
    unknown_lines = 0

    try:
        while time.time() - start_time < duration:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
                raw_bytes.extend(data)

                # Try to decode as text
                try:
                    text = data.decode('utf-8', errors='ignore')
                    for line in text.split('\n'):
                        line = line.strip()
                        if line:
                            lines.append(line)
                            if line.startswith('RX:'):
                                rx_packets += 1
                                if rx_packets <= 3:
                                    print(f"  RX: {line[:80]}...")
                            elif '---' in line or 'Status' in line:
                                status_msgs += 1
                                print(f"  STATUS: {line}")
                            elif 'uMyo' in line or 'Central' in line or 'Listening' in line:
                                print(f"  INFO: {line}")
                            else:
                                unknown_lines += 1
                                if unknown_lines <= 5:
                                    print(f"  DATA: {line[:60]}")
                except:
                    pass

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    ser.close()
    elapsed = time.time() - start_time

    print("-" * 60)
    print(f"\n=== Summary ===")
    print(f"Duration: {elapsed:.1f} seconds")
    print(f"Total bytes received: {byte_count}")
    print(f"Data rate: {byte_count/elapsed:.1f} bytes/sec")
    print(f"Total lines: {len(lines)}")
    print(f"  - RX packets: {rx_packets}")
    print(f"  - Status messages: {status_msgs}")
    print(f"  - Unknown lines: {unknown_lines}")

    if rx_packets > 0:
        print(f"\n✓ RECEIVING uMyo DATA! ({rx_packets} packets)")
        print("  uMyo is likely in Fast64 mode (green LED)")
        return True
    elif byte_count > 0:
        print(f"\n? RECEIVING SOME DATA but no RX packets detected")
        print("  Raw bytes sample:", raw_bytes[:50].hex() if raw_bytes else "none")

        # Check if it's raw binary data
        if raw_bytes:
            # Look for patterns
            printable = sum(1 for b in raw_bytes if 32 <= b < 127)
            print(f"  Printable chars: {printable}/{len(raw_bytes)} ({100*printable/len(raw_bytes):.0f}%)")

            if printable < len(raw_bytes) * 0.5:
                print("  Looks like binary data - might be different protocol")
        return False
    else:
        print("\n✗ NO DATA RECEIVED")
        print("  Possible issues:")
        print("  1. uMyo not powered on or in range")
        print("  2. uMyo in wrong mode (try pressing button to cycle modes)")
        print("  3. nRF52840 DK firmware not running correctly")
        print("  4. Wrong serial port selected")
        return False

def show_umyo_modes():
    """Display uMyo radio mode information."""
    print("""
=== uMyo Radio Modes ===

Press the uMyo button briefly to cycle through modes:

┌─────────────┬────────────────┬────────────────────────────────────┐
│ Mode        │ LED Color      │ Description                        │
├─────────────┼────────────────┼────────────────────────────────────┤
│ Fast64      │ GREEN (3 pulses)│ Star protocol, needs nRF52 receiver│
│ Fast32      │ MAGENTA (3 pulses)│ Simple RF, needs nRF52 receiver │
│ BLE         │ BLUE (3 pulses)│ Bluetooth, works with ESP32/phone  │
└─────────────┴────────────────┴────────────────────────────────────┘

For the nRF52840 DK receiver, uMyo should be in:
  → FAST64 mode (GREEN LED with 3 pulses)

The receiver listens on channel 21 with star_protocol.
""")

def continuous_monitor(port, baud=115200):
    """Continuously monitor and display data."""
    print(f"\n=== Continuous Monitor: {port} ===")
    print("Press Ctrl+C to stop\n")

    try:
        ser = pyserial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR: Cannot open {port}: {e}")
        return

    packet_count = 0
    last_status_time = time.time()

    try:
        while True:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith('RX:'):
                            packet_count += 1
                            # Parse and display key data
                            adc_match = re.search(r'ADC:([-\d,]+)', line)
                            if adc_match:
                                adc_vals = adc_match.group(1)
                                print(f"[{packet_count:5d}] ADC: {adc_vals}")
                            else:
                                print(f"[{packet_count:5d}] {line[:70]}")
                        elif '---' in line:
                            print(f"\n{line}\n")
                        else:
                            print(line)
                except:
                    pass

            # Print status every 5 seconds if no packets
            if packet_count == 0 and time.time() - last_status_time > 5:
                print("... waiting for uMyo packets (is uMyo in Fast64/GREEN mode?) ...")
                last_status_time = time.time()

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total packets: {packet_count}")

    ser.close()

def main():
    parser = argparse.ArgumentParser(description='uMyo Connection Diagnostic Tool')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('-t', '--time', type=int, default=10, help='Test duration in seconds')
    parser.add_argument('-m', '--monitor', action='store_true', help='Continuous monitoring mode')
    parser.add_argument('-l', '--list', action='store_true', help='List serial ports')
    parser.add_argument('-i', '--info', action='store_true', help='Show uMyo mode info')
    args = parser.parse_args()

    print("=" * 60)
    print("        uMyo Connection Diagnostic Tool")
    print("=" * 60)

    if args.info:
        show_umyo_modes()
        return

    if args.list:
        list_ports()
        return

    # Show mode info first
    show_umyo_modes()

    # List available ports
    list_ports()

    if args.monitor:
        continuous_monitor(args.port, args.baud)
    else:
        # Run connection test
        test_connection(args.port, args.baud, args.time)

        print("\n=== Next Steps ===")
        print("1. Make sure uMyo is in Fast64 mode (GREEN LED, 3 pulses)")
        print("2. Run: python3 umyo_diagnostic.py -m  (for continuous monitoring)")
        print("3. Run: python3 plot_umyo.py -p /dev/ttyACM0  (for plotting)")

if __name__ == '__main__':
    main()
