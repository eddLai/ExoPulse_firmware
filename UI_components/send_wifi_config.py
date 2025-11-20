#!/usr/bin/env python3
"""
Send WiFi Configuration Command to ESP32
Captures and displays the IP address response
"""

import serial
import time
import sys
import re

# Serial port configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 30  # seconds to wait for IP address

def send_wifi_config(ssid, password):
    """
    Send WIFI_CONFIG command and capture IP address
    """
    try:
        # Open serial connection
        print(f"Opening serial port {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(0.5)  # Allow port to stabilize

        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(f"\nSending command: WIFI_CONFIG {ssid} {password}")
        command = f"WIFI_CONFIG {ssid} {password}\n"
        ser.write(command.encode())
        ser.flush()

        print("\nWaiting for firmware response...\n")
        print("=" * 60)

        # Read responses
        start_time = time.time()
        ip_address = None

        while (time.time() - start_time) < TIMEOUT:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    # Print all firmware output
                    print(line)

                    # Search for IP address
                    if "IP Address:" in line or "IP:" in line:
                        # Extract IP address using regex
                        ip_match = re.search(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', line)
                        if ip_match:
                            ip_address = ip_match.group(1)

                    # Check for success/failure messages
                    if "[OK] WiFi configured successfully!" in line:
                        print("\n" + "=" * 60)
                        print("✓ WiFi configuration successful!")
                    elif "[ERROR] WiFi connection failed" in line:
                        print("\n" + "=" * 60)
                        print("✗ WiFi configuration failed!")
                        break
            else:
                time.sleep(0.1)

        print("=" * 60)

        # Close serial connection
        ser.close()

        # Report IP address
        if ip_address:
            print(f"\n✓ ESP32 IP Address: {ip_address}")
            print(f"\nTo connect via WiFi, run:")
            print(f"  python3 UI_components/wifi_monitor.py {ip_address} 8888")
            return ip_address
        else:
            print("\n✗ Could not find IP address in firmware response")
            print("   Please check Serial Monitor for details")
            return None

    except serial.SerialException as e:
        print(f"✗ Serial port error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if ESP32 is connected")
        print("  2. Press RESET button on ESP32")
        print("  3. Check if another program is using the port")
        return None
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return None
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return None

if __name__ == "__main__":
    print("=" * 60)
    print("ESP32 WiFi Configuration Tool")
    print("=" * 60)

    # Configuration
    SSID = "ExoPulse"
    PASSWORD = "12345666"

    print(f"\nConfiguration:")
    print(f"  SSID: {SSID}")
    print(f"  Password: {PASSWORD}")
    print(f"  Serial Port: {SERIAL_PORT}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print()

    # Send command
    ip = send_wifi_config(SSID, PASSWORD)

    # Exit with appropriate code
    sys.exit(0 if ip else 1)
