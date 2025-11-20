#!/usr/bin/env python3
"""
Real-time Motor Status Monitor
Reads and displays LK-TECH motor status from ESP32 serial output
"""

import serial
import re
import sys
import time
from datetime import datetime
from collections import deque

class MotorMonitor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.data_history = {
            'temperature': deque(maxlen=20),
            'voltage': deque(maxlen=20),
            'current': deque(maxlen=20),
            'speed': deque(maxlen=20),
            'encoder': deque(maxlen=20),
            'angle': deque(maxlen=20),
        }

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            print("=" * 60)
            time.sleep(2)  # Wait for ESP32 to stabilize
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False

    def parse_motor_status(self, lines):
        """Parse motor status from serial lines"""
        status = {}

        for line in lines:
            line = line.strip()

            # Temperature: 27 °C
            if 'Temperature:' in line:
                match = re.search(r'Temperature:\s+(-?\d+)\s+°C', line)
                if match:
                    status['temperature'] = int(match.group(1))

            # Voltage: 0.8 V
            elif 'Voltage:' in line:
                match = re.search(r'Voltage:\s+(\d+\.?\d*)\s+V', line)
                if match:
                    status['voltage'] = float(match.group(1))

            # Torque Current: -0.03 A (raw=-2)
            elif 'Torque Current:' in line:
                match = re.search(r'Torque Current:\s+(-?\d+\.?\d*)\s+A', line)
                if match:
                    status['current'] = float(match.group(1))

            # Speed: 0 dps
            elif 'Speed:' in line:
                match = re.search(r'Speed:\s+(-?\d+)\s+dps', line)
                if match:
                    status['speed'] = int(match.group(1))

            # Encoder: 4832 (0~16383)
            elif 'Encoder:' in line:
                match = re.search(r'Encoder:\s+(\d+)', line)
                if match:
                    status['encoder'] = int(match.group(1))

            # Multi-turn Angle: 265.43 ° (0.74 turns)
            elif 'Multi-turn Angle:' in line:
                match = re.search(r'Multi-turn Angle:(-?\d+\.?\d*)\s+°', line)
                if match:
                    status['angle'] = float(match.group(1))

            # Error State: 0x0
            elif 'Error State:' in line:
                match = re.search(r'Error State:\s+(0x[0-9A-Fa-f]+)', line)
                if match:
                    status['error'] = match.group(1)
                    # Check for error flags
                    if '[LOW_VOLTAGE]' in line:
                        status['error_flags'] = status.get('error_flags', []) + ['LOW_VOLTAGE']
                    if '[OVER_TEMP]' in line:
                        status['error_flags'] = status.get('error_flags', []) + ['OVER_TEMP']

        return status if status else None

    def display_status(self, status):
        """Display motor status with color coding"""
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Clear screen (move cursor to top)
        print("\033[H\033[J", end="")

        print("=" * 60)
        print(f"  LK-TECH Motor Status Monitor - {timestamp}")
        print("=" * 60)
        print()

        # Temperature
        temp = status.get('temperature', 'N/A')
        temp_color = '\033[92m' if isinstance(temp, int) and temp < 50 else '\033[93m'
        if isinstance(temp, int) and temp > 70:
            temp_color = '\033[91m'
        print(f"  Temperature:    {temp_color}{temp:>6}{'\033[0m' if temp != 'N/A' else ''} °C")

        # Voltage
        voltage = status.get('voltage', 'N/A')
        voltage_color = '\033[93m' if isinstance(voltage, (int, float)) and voltage < 10 else '\033[92m'
        voltage_str = f"{voltage:>6.1f}" if isinstance(voltage, (int, float)) else str(voltage)
        print(f"  Voltage:        {voltage_color}{voltage_str}{'\033[0m' if voltage != 'N/A' else ''} V")

        # Torque Current
        current = status.get('current', 'N/A')
        current_str = f"{current:>6.2f}" if isinstance(current, (int, float)) else current
        print(f"  Torque Current: {current_str} A")

        # Speed
        speed = status.get('speed', 'N/A')
        speed_str = f"{speed:>6}" if isinstance(speed, int) else speed
        print(f"  Speed:          {speed_str} dps")

        # Encoder
        encoder = status.get('encoder', 'N/A')
        encoder_str = f"{encoder:>6}" if isinstance(encoder, int) else encoder
        encoder_pct = f" ({encoder/163.83:.1f}%)" if isinstance(encoder, int) else ""
        print(f"  Encoder:        {encoder_str}{encoder_pct}")

        # Multi-turn Angle
        angle = status.get('angle', 'N/A')
        if isinstance(angle, (int, float)):
            turns = angle / 360.0
            print(f"  Angle:          {angle:>6.2f}° ({turns:>6.2f} turns)")
        else:
            print(f"  Angle:          {angle}")

        # Error State
        error = status.get('error', 'N/A')
        error_flags = status.get('error_flags', [])
        error_color = '\033[92m' if error == '0x0' else '\033[91m'
        error_text = f"{error_color}{error}\033[0m"
        if error_flags:
            error_text += f" {'\033[91m' + ', '.join(error_flags) + '\033[0m'}"
        print(f"  Error State:    {error_text}")

        print()
        print("-" * 60)

        # Statistics (min/max/avg from history)
        if len(self.data_history['temperature']) > 1:
            print("  Statistics (last 20 readings):")
            print()

            temps = [t for t in self.data_history['temperature'] if t is not None]
            if temps:
                print(f"  Temp:  Min={min(temps):>3}°C  Max={max(temps):>3}°C  Avg={sum(temps)/len(temps):>5.1f}°C")

            currents = [c for c in self.data_history['current'] if c is not None]
            if currents:
                print(f"  Current: Min={min(currents):>6.2f}A  Max={max(currents):>6.2f}A  Avg={sum(currents)/len(currents):>6.2f}A")

            speeds = [s for s in self.data_history['speed'] if s is not None]
            if speeds and any(s != 0 for s in speeds):
                print(f"  Speed:   Min={min(speeds):>5}dps Max={max(speeds):>5}dps Avg={sum(speeds)/len(speeds):>7.1f}dps")

        print()
        print("=" * 60)
        print("  Press Ctrl+C to exit")
        print("=" * 60)

    def monitor(self):
        """Main monitoring loop"""
        if not self.connect():
            return

        print("Starting monitor... (reading motor status)")
        print()

        buffer = []

        try:
            while True:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        buffer.append(line)

                        # Check if we've completed a status block
                        if '--------------------' in line:
                            status = self.parse_motor_status(buffer)

                            if status:
                                # Update history
                                self.data_history['temperature'].append(status.get('temperature'))
                                self.data_history['voltage'].append(status.get('voltage'))
                                self.data_history['current'].append(status.get('current'))
                                self.data_history['speed'].append(status.get('speed'))
                                self.data_history['encoder'].append(status.get('encoder'))
                                self.data_history['angle'].append(status.get('angle'))

                                # Display
                                self.display_status(status)

                            buffer = []

                time.sleep(0.01)  # Small delay to prevent CPU spinning

        except KeyboardInterrupt:
            print("\n\n✓ Monitor stopped by user")
        except Exception as e:
            print(f"\n\n✗ Error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print(f"✓ Disconnected from {self.port}")

def main():
    """Main entry point"""
    port = '/dev/ttyUSB0'
    baudrate = 115200

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])

    print("=" * 60)
    print("  LK-TECH Motor Real-time Monitor")
    print("=" * 60)
    print(f"  Port: {port}")
    print(f"  Baudrate: {baudrate}")
    print("=" * 60)
    print()

    monitor = MotorMonitor(port, baudrate)
    monitor.monitor()

if __name__ == '__main__':
    main()
