#!/usr/bin/env python3
"""
Test CAL_M1 command to calibrate Motor 1
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

    print("\n=== Monitoring Motor 1 BEFORE calibration ===")
    start_time = time.time()
    while time.time() - start_time < 2:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line and 'M:1' in line:
                print(f"\033[1;36m{line}\033[0m")
        time.sleep(0.01)

    print("\n=== Sending CAL1 command ===")
    ser.write(b"CAL1\n")
    ser.flush()

    # Wait for calibration response
    time.sleep(1)
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').rstrip()
        if line:
            if 'CMD' in line or 'OK' in line or 'calibrate' in line.lower() or 'INFO' in line:
                print(f"\033[1;32m{line}\033[0m")
            else:
                print(line)

    print("\n=== Monitoring Motor 1 AFTER calibration ===")
    print("*** Motor 1 angle should now be ~0° ***\n")
    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line and 'M:1' in line:
                print(f"\033[1;36m{line}\033[0m")
        time.sleep(0.01)

    print("\n=== Test complete ===")
    ser.close()

except Exception as e:
    print(f"Error: {e}")
