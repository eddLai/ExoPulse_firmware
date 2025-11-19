#!/usr/bin/env python3
"""Test regex parsing for motor data"""
import re

# Test line based on firmware format
test_lines = [
    "[1234] M:1 T:25 V:12.0 I:0.05 S:0 ACC:0 E:1234 A:0.00 ERR:0x0",
    "[1235] M:2 T:26 V:12.1 I:-0.03 S:5 ACC:10 E:1235 A:1.50 ERR:0x0",
    "[1236] M:1 T:25 V:12.0 I:0.05 S:-10 ACC:-5 E:1236 A:-2.50 ERR:0x00",
]

pattern = r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+) ERR:(0x[\w]+)'

print("Testing regex pattern:")
print(f"Pattern: {pattern}\n")

for line in test_lines:
    print(f"Input:  {line}")
    match = re.match(pattern, line)
    if match:
        print(f"✓ MATCH - Groups: {match.groups()}")
        print(f"  motor_id={match.group(2)}, temp={match.group(3)}, speed={match.group(6)}")
        print(f"  acceleration={match.group(7)}, angle={match.group(9)}")
    else:
        print("✗ NO MATCH")
    print()
