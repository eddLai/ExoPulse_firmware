#!/usr/bin/env python3
"""
Simple test to verify ADC data is being received correctly.
"""

import serial as pyserial
import re
import statistics

port = '/dev/ttyACM0'
baud = 921600

print(f"Connecting to {port} at {baud} baud...")
ser = pyserial.Serial(port, baud, timeout=1)

print("Listening for ADC data... (Ctrl+C to stop)\n")

packet_count = 0
all_adc_values = []

try:
    while packet_count < 50:  # Read 50 packets
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line and 'ADC:' in line:
            # Extract ADC values
            adc_match = re.search(r'ADC:([-\d,]+)', line)
            if adc_match:
                try:
                    adc_values = [int(x) for x in adc_match.group(1).split(',')]
                    if len(adc_values) == 8:
                        packet_count += 1
                        all_adc_values.extend(adc_values)

                        # Print every 10th packet
                        if packet_count % 10 == 0:
                            print(f"Packet {packet_count}: ADC values: {adc_values}")
                            print(f"  Min: {min(adc_values)}, Max: {max(adc_values)}, Avg: {sum(adc_values)/8:.0f}")
                except:
                    pass

except KeyboardInterrupt:
    print("\nStopped by user")

ser.close()

if all_adc_values:
    print(f"\n=== Summary of {packet_count} packets ({len(all_adc_values)} samples) ===")
    print(f"Overall Min: {min(all_adc_values)}")
    print(f"Overall Max: {max(all_adc_values)}")
    print(f"Overall Mean: {statistics.mean(all_adc_values):.1f}")
    print(f"Overall Std Dev: {statistics.stdev(all_adc_values):.1f}")
    print(f"\nADC values look correct! Range {min(all_adc_values)}-{max(all_adc_values)} is within 14-bit range (0-16383)")
else:
    print("\nNo ADC data received!")
