#!/usr/bin/env python3
"""
Test script to reset Motor 1 angle and monitor raw data
"""
import serial
import time

port = '/dev/ttyUSB0'
baudrate = 115200

try:
    print(f"Opening {port}...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(0.5)

    print("Clearing buffer...")
    ser.reset_input_buffer()

    print("\n=== Sending RESET_M1 command ===")
    ser.write(b"RESET_M1\n")
    ser.flush()
    print("Command sent!\n")

    print("=== Monitoring output for 10 seconds ===")
    start_time = time.time()

    while time.time() - start_time < 10:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:
                # Highlight Motor 1 data
                if 'M:1' in line:
                    print(f"\033[1;36m{line}\033[0m")  # Cyan for Motor 1
                elif 'Motor 1' in line or 'RESET' in line or 'CMD' in line:
                    print(f"\033[1;33m{line}\033[0m")  # Yellow for messages
                else:
                    print(line)
        time.sleep(0.01)

    print("\n=== Test complete ===")
    ser.close()

except Exception as e:
    print(f"Error: {e}")