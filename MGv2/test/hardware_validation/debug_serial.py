#!/usr/bin/env python3
"""
Debug script to see raw serial output from ESP32
"""
import serial
import sys
import re

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

print(f"Connecting to {port} at {baudrate} baud...")

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print("✓ Connected! Reading data...\n")

    pattern = r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)'

    line_count = 0
    match_count = 0

    while line_count < 20:  # Show first 20 lines
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                line_count += 1
                print(f"[{line_count}] RAW: {line}")

                # Try to parse
                if line.startswith('['):
                    match = re.match(pattern, line)
                    if match:
                        match_count += 1
                        print(f"     ✓ PARSED: M{match.group(2)} T={match.group(3)}°C V={match.group(4)}V "
                              f"I={match.group(5)}A S={match.group(6)}dps ACC={match.group(7)}dps² "
                              f"A={match.group(9)}°")
                    else:
                        print(f"     ✗ PARSE FAILED - regex mismatch")
                print()

    print(f"\n=== Summary ===")
    print(f"Total lines: {line_count}")
    print(f"Parsed successfully: {match_count}")
    print(f"Parse rate: {match_count}/{line_count} ({100*match_count/line_count if line_count > 0 else 0:.1f}%)")

    ser.close()

except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)
