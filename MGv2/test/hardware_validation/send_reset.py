#!/usr/bin/env python3
"""Send reset command to motor"""
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
motor = sys.argv[2] if len(sys.argv) > 2 else '1'

print(f"Connecting to {port}...")
ser = serial.Serial(port, 115200, timeout=1)
time.sleep(0.5)

if motor == '1':
    print("Sending RESET_M1...")
    ser.write(b"RESET_M1\n")
elif motor == '2':
    print("Sending RESET_M2...")
    ser.write(b"RESET_M2\n")
else:
    print("Sending RESET_ALL...")
    ser.write(b"RESET_ALL\n")

time.sleep(0.5)

# Read response
print("\nResponse:")
while ser.in_waiting:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(f"  {line}")

# Show a few motor status lines
print("\nMotor status after reset:")
for i in range(10):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('['):
            print(f"  {line}")
    time.sleep(0.1)

ser.close()
print("\n✓ Done")
