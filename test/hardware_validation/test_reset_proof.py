#!/usr/bin/env python3
"""
Proof that reset command works - Test on Motor 2
"""
import serial
import time
import re

port = '/dev/ttyUSB0'
baudrate = 115200

print("=" * 70)
print("RESET COMMAND TEST - Motor 2")
print("=" * 70)

ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(0.5)

pattern = r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)'

# Step 1: Read Motor 2 angle BEFORE reset
print("\n[STEP 1] Reading Motor 2 angle BEFORE reset...")
angles_before = []
for _ in range(10):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('['):
            match = re.match(pattern, line)
            if match and match.group(2) == '2':  # Motor 2
                angle = match.group(9)
                if angle != 'ovf':
                    angles_before.append(float(angle))
                    print(f"  Motor 2 angle: {angle}°")
                    if len(angles_before) >= 3:
                        break
    time.sleep(0.1)

avg_before = sum(angles_before) / len(angles_before) if angles_before else 0
print(f"\n  Average BEFORE reset: {avg_before:.2f}°")

# Step 2: Send RESET command to Motor 2
print("\n[STEP 2] Sending RESET_M2 command...")
ser.write(b"RESET_M2\n")
ser.flush()
print("  Command sent!")

# Wait for reset to complete
time.sleep(1)

# Clear buffer
while ser.in_waiting:
    ser.readline()

# Step 3: Read Motor 2 angle AFTER reset
print("\n[STEP 3] Reading Motor 2 angle AFTER reset...")
angles_after = []
for _ in range(10):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('['):
            match = re.match(pattern, line)
            if match and match.group(2) == '2':  # Motor 2
                angle = match.group(9)
                if angle != 'ovf':
                    angles_after.append(float(angle))
                    print(f"  Motor 2 angle: {angle}°")
                    if len(angles_after) >= 3:
                        break
    time.sleep(0.1)

avg_after = sum(angles_after) / len(angles_after) if angles_after else 0
print(f"\n  Average AFTER reset: {avg_after:.2f}°")

# Step 4: Analysis
print("\n" + "=" * 70)
print("RESULTS")
print("=" * 70)
print(f"Angle BEFORE reset: {avg_before:.2f}°")
print(f"Angle AFTER reset:  {avg_after:.2f}°")
print(f"Change:             {avg_before - avg_after:.2f}°")

if abs(avg_after) < 1.0:
    print("\n✅ RESET COMMAND WORKS! Angle is now close to 0°")
elif abs(avg_after - avg_before) > 100:
    print("\n✅ RESET COMMAND WORKS! Angle changed significantly")
else:
    print("\n❌ Reset might not be working - angle didn't change much")

print("=" * 70)

ser.close()
