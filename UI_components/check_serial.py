#!/usr/bin/env python3
"""
Simple Serial Monitor - Check ESP32 Status
"""

import serial
import time
import sys

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

try:
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.5)

    print("Clearing buffers...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("\nListening for 5 seconds...\n")
    print("=" * 60)

    start = time.time()
    line_count = 0

    while (time.time() - start) < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
                line_count += 1
        time.sleep(0.01)

    print("=" * 60)
    print(f"\nReceived {line_count} lines in 5 seconds")

    if line_count == 0:
        print("\n⚠ No data received from ESP32")
        print("   ESP32 may be in bootloader mode")
        print("   Please press RESET button on ESP32 and try again")

    ser.close()

except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
