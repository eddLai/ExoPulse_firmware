#!/usr/bin/env python3
"""
Test script to compare Serial vs WiFi motor data
Helps identify if angle difference is from firmware or PC parsing
"""

import serial
import socket
import time
import re
import threading

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TCP_IP = '10.16.241.20'  # ESP32 IP
TCP_PORT = 8888

def parse_motor_data(line):
    """Parse motor data line and return dict"""
    # Format: [timestamp] SEQ:xxx M:x T:xx V:x.x I:x.xx S:x ACC:x E:xxxxx A:xxx.xx ERR:0xx
    match = re.search(
        r'\[(\d+)\] SEQ:(\d+) M:(\d) T:(\d+) V:([\d.]+) I:([-\d.]+) S:([-\d]+) ACC:([-\d]+) E:(\d+) A:([-\d.]+) ERR:(0x[0-9A-Fa-f]+)',
        line
    )
    if match:
        return {
            'timestamp': int(match.group(1)),
            'seq': int(match.group(2)),
            'motor': int(match.group(3)),
            'temp': int(match.group(4)),
            'voltage': float(match.group(5)),
            'current': float(match.group(6)),
            'speed': int(match.group(7)),
            'acc': int(match.group(8)),
            'encoder': int(match.group(9)),
            'angle': float(match.group(10)),
            'error': match.group(11),
            'raw': line
        }
    return None

def read_serial_data(duration=3):
    """Read motor data from Serial port"""
    print(f"\n[Serial] Reading from {SERIAL_PORT}...")
    data = {'M1': [], 'M2': []}
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        start = time.time()
        while time.time() - start < duration:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                parsed = parse_motor_data(line)
                if parsed:
                    key = f"M{parsed['motor']}"
                    data[key].append(parsed)
        
        ser.close()
    except Exception as e:
        print(f"[Serial] Error: {e}")
    
    return data

def read_wifi_data(duration=3):
    """Read motor data from WiFi TCP"""
    print(f"\n[WiFi] Reading from {TCP_IP}:{TCP_PORT}...")
    data = {'M1': [], 'M2': []}
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((TCP_IP, TCP_PORT))
        sock.settimeout(1)
        
        buffer = ""
        start = time.time()
        while time.time() - start < duration:
            try:
                chunk = sock.recv(1024).decode(errors='ignore')
                if chunk:
                    buffer += chunk
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        # Remove [TCP] prefix if present
                        if line.startswith('[TCP]'):
                            line = line[5:].strip()
                        parsed = parse_motor_data(line)
                        if parsed:
                            key = f"M{parsed['motor']}"
                            data[key].append(parsed)
            except socket.timeout:
                continue
        
        sock.close()
    except Exception as e:
        print(f"[WiFi] Error: {e}")
    
    return data

def compare_data(serial_data, wifi_data):
    """Compare Serial vs WiFi data"""
    print("\n" + "="*70)
    print("COMPARISON RESULTS")
    print("="*70)
    
    for motor in ['M1', 'M2']:
        print(f"\n--- {motor} ---")
        
        s_data = serial_data.get(motor, [])
        w_data = wifi_data.get(motor, [])
        
        print(f"  Serial samples: {len(s_data)}")
        print(f"  WiFi samples:   {len(w_data)}")
        
        if s_data:
            s_last = s_data[-1]
            print(f"\n  [Serial] Last reading:")
            print(f"    Encoder: {s_last['encoder']}")
            print(f"    Angle:   {s_last['angle']}°")
            print(f"    Raw:     {s_last['raw'][:80]}...")
        
        if w_data:
            w_last = w_data[-1]
            print(f"\n  [WiFi] Last reading:")
            print(f"    Encoder: {w_last['encoder']}")
            print(f"    Angle:   {w_last['angle']}°")
            print(f"    Raw:     {w_last['raw'][:80]}...")
        
        # Compare if both have data
        if s_data and w_data:
            angle_diff = abs(s_data[-1]['angle'] - w_data[-1]['angle'])
            encoder_diff = abs(s_data[-1]['encoder'] - w_data[-1]['encoder'])
            
            print(f"\n  DIFFERENCE:")
            print(f"    Angle diff:   {angle_diff:.2f}°")
            print(f"    Encoder diff: {encoder_diff}")
            
            if angle_diff > 1.0:
                print(f"    >>> SIGNIFICANT ANGLE DIFFERENCE DETECTED!")
                if encoder_diff < 100:
                    print(f"    >>> Encoder similar but angle different - likely PARSING issue on PC")
                else:
                    print(f"    >>> Encoder also different - likely FIRMWARE issue")

if __name__ == "__main__":
    print("="*70)
    print("Serial vs WiFi Motor Data Comparison Test")
    print("="*70)
    
    # First, read from Serial
    serial_data = read_serial_data(duration=2)
    
    # Then, read from WiFi
    wifi_data = read_wifi_data(duration=2)
    
    # Compare results
    compare_data(serial_data, wifi_data)
    
    print("\n" + "="*70)
    print("Test complete!")
    print("="*70)
