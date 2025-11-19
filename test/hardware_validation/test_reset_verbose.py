#!/usr/bin/env python3
"""
Verbose test to see ALL serial output during reset
"""
import serial
import time

port = '/dev/ttyUSB0'
baudrate = 115200

print("Connecting...")
ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(0.5)

print("\n" + "=" * 70)
print("Sending RESET_M2 command and showing ALL responses...")
print("=" * 70 + "\n")

ser.write(b"RESET_M2\n")
ser.flush()
print(">>> Sent: RESET_M2\n")

# Read all output for 3 seconds
start = time.time()
while time.time() - start < 3:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            # Highlight command responses
            if 'CMD' in line or 'Reset' in line or 'OK' in line or 'ERROR' in line:
                print(f">>> {line}")
            else:
                print(f"    {line}")

ser.close()
print("\nDone!")
