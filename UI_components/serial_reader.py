#!/usr/bin/env python3
"""
Simple serial port reader for ESP32
Reads data from /dev/ttyUSB0 at 115200 baud
"""
import serial
import sys
import time

def main():
    port = '/dev/ttyUSB0'
    baudrate = 115200
    reconnect_delay = 2  # seconds

    while True:
        try:
            print(f"Opening {port} at {baudrate} baud...")
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected! Press Ctrl+C to stop.\n")
            print("="*60)

            # Clear any stale data
            time.sleep(0.1)
            ser.reset_input_buffer()

            while True:
                try:
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='replace').rstrip()
                            if line:
                                print(line)
                                sys.stdout.flush()
                        except UnicodeDecodeError:
                            pass
                    time.sleep(0.01)
                except (OSError, serial.SerialException) as e:
                    # I/O error or serial exception (device disconnected)
                    print(f"\n[!] Connection lost: {e}")
                    print(f"[!] Reconnecting in {reconnect_delay} seconds...")
                    break

        except serial.SerialException as e:
            print(f"Error: {e}")
            print(f"\n[!] Cannot open port. Retrying in {reconnect_delay} seconds...")
            print("    (Press Ctrl+C to stop)")
            try:
                time.sleep(reconnect_delay)
                continue
            except KeyboardInterrupt:
                print("\n\nStopped by user")
                return 0
        except KeyboardInterrupt:
            print("\n\nStopped by user")
            return 0
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("[✓] Serial port closed")

        # Wait before reconnecting
        try:
            time.sleep(reconnect_delay)
        except KeyboardInterrupt:
            print("\n\nStopped by user")
            return 0

if __name__ == "__main__":
    sys.exit(main())