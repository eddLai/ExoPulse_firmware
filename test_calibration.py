#!/usr/bin/env python3
"""
Test software calibration feature
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

    print("\n=== Showing HELP ===")
    ser.write(b"HELP\n")
    ser.flush()
    time.sleep(0.5)

    # Read help output
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').rstrip()
        if line:
            print(line)

    print("\n=== Monitoring current motor data for 3 seconds ===")
    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line and 'M:' in line:
                if 'M:1' in line:
                    print(f"\033[1;36m{line}\033[0m")  # Cyan for Motor 1
                elif 'M:2' in line:
                    print(f"\033[1;33m{line}\033[0m")  # Yellow for Motor 2
        time.sleep(0.01)

    print("\n=== Calibrating ALL motors (CAL_ALL) ===")
    ser.write(b"CAL_ALL\n")
    ser.flush()

    # Wait for calibration response
    time.sleep(1.5)
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').rstrip()
        if line:
            if 'CMD' in line or 'OK' in line or 'calibrate' in line.lower():
                print(f"\033[1;32m{line}\033[0m")  # Green for calibration messages
            else:
                print(line)

    print("\n=== Monitoring calibrated data for 5 seconds ===")
    print("*** Angles should now be close to 0° ***\n")
    start_time = time.time()
    while time.time() - start_time < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:
                if 'M:1' in line:
                    print(f"\033[1;36m{line}\033[0m")  # Cyan for Motor 1
                elif 'M:2' in line:
                    print(f"\033[1;33m{line}\033[0m")  # Yellow for Motor 2
                elif 'Motor' in line or '---' in line:
                    print(f"\033[1;37m{line}\033[0m")  # White for detailed output
        time.sleep(0.01)

    print("\n=== Test complete ===")
    ser.close()

except Exception as e:
    print(f"Error: {e}")
