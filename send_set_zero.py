#!/usr/bin/env python3
"""
Send SET_ZERO command to motors
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

    print("\n=== Sending SET_ZERO_M1 command (0x19) ===")
    ser.write(b"SET_ZERO_M1\n")
    ser.flush()
    print("Command sent!\n")

    print("=== Monitoring output for 5 seconds ===")
    start_time = time.time()

    while time.time() - start_time < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line:
                # Highlight important messages
                if 'CMD' in line or 'OK' in line or 'ERROR' in line or 'WARNING' in line:
                    print(f"\033[1;33m{line}\033[0m")  # Yellow
                elif 'M:1' in line:
                    print(f"\033[1;36m{line}\033[0m")  # Cyan
                else:
                    print(line)
        time.sleep(0.01)

    print("\n=== Command sent successfully ===")
    print("\033[1;31m*** IMPORTANT: Reboot MCU for changes to take effect! ***\033[0m")
    ser.close()

except Exception as e:
    print(f"Error: {e}")
