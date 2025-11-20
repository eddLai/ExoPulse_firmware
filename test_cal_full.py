#!/usr/bin/env python3
"""
Complete calibration test: CAL1, CAL2, CLEAR_CAL
"""
import serial
import time

port = '/dev/ttyUSB0'
baudrate = 115200

def send_command(ser, cmd, wait_time=1.5):
    print(f"\n=== Sending {cmd} ===")
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    time.sleep(wait_time)

    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').rstrip()
        if line:
            if 'CMD' in line or 'OK' in line or 'INFO' in line or 'ERROR' in line:
                print(f"\033[1;32m{line}\033[0m")
            else:
                print(line)

def monitor_motors(ser, duration=2):
    print(f"\n=== Monitoring for {duration} seconds ===")
    start_time = time.time()
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').rstrip()
            if line and 'M:' in line:
                if 'M:1' in line:
                    print(f"\033[1;36m{line}\033[0m")
                elif 'M:2' in line:
                    print(f"\033[1;33m{line}\033[0m")
        time.sleep(0.01)

try:
    print(f"Opening {port}...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Test sequence
    print("\n" + "="*60)
    print("SOFTWARE CALIBRATION TEST SEQUENCE")
    print("="*60)

    # 1. Show current angles
    print("\n[1] Current motor angles (before calibration):")
    monitor_motors(ser, 2)

    # 2. Calibrate Motor 1
    send_command(ser, "CAL1")
    monitor_motors(ser, 2)

    # 3. Calibrate Motor 2
    send_command(ser, "CAL2")
    monitor_motors(ser, 2)

    # 4. Show calibrated angles (should be ~0°)
    print("\n[2] Motor angles after calibration (should be ~0°):")
    monitor_motors(ser, 3)

    # 5. Clear calibration
    send_command(ser, "CLEAR_CAL")

    # 6. Show angles after clear (should be back to original)
    print("\n[3] Motor angles after CLEAR_CAL (should be original values):")
    monitor_motors(ser, 3)

    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)

    ser.close()

except Exception as e:
    print(f"Error: {e}")
