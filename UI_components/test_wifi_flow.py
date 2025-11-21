#!/usr/bin/env python3
"""
Independent test script for WiFi configuration flow
Tests the complete flow from serial config to WiFi data reception
"""

import serial
import socket
import time
import re
import sys

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
WIFI_SSID = 'ExoPulse'
WIFI_PASSWORD = '12345666'
ESP32_PORT = 8888

def print_step(step_num, description):
    """Print a formatted step header"""
    print(f"\n{'='*60}")
    print(f"STEP {step_num}: {description}")
    print(f"{'='*60}")

def test_serial_config():
    """Test serial WiFi configuration"""
    print_step(1, "Serial WiFi Configuration")

    try:
        # Open serial port
        print(f"Opening serial port {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(0.5)
        print("✓ Serial port opened")

        # Clear buffer
        ser.reset_input_buffer()
        print("✓ Cleared input buffer")

        # Send WiFi config command
        cmd = f"WIFI_CONFIG {WIFI_SSID} {WIFI_PASSWORD}\n"
        print(f"Sending: WIFI_CONFIG {WIFI_SSID} ****")
        ser.write(cmd.encode())
        ser.flush()
        print("✓ Command sent")

        # Wait for response
        print("\nWaiting for ESP32 response (max 20s)...")
        start_time = time.time()
        ip_address = None

        while time.time() - start_time < 20:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    # Filter and display WiFi-related messages
                    if any(keyword in line for keyword in ['WiFi', 'wifi', 'WIFI', 'IP', 'CMD', 'Connecting', 'Connected', 'Error', 'Failed']):
                        print(f"  {line}")

                    # Check for IP address
                    if "IP" in line and any(char.isdigit() for char in line):
                        match = re.search(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', line)
                        if match:
                            ip_address = match.group(1)
                            print(f"\n✓ Found IP: {ip_address}")
                            break

        ser.close()
        print("✓ Serial port closed")

        if ip_address:
            print(f"\n✅ WiFi configuration successful! IP: {ip_address}")
            return ip_address
        else:
            print("\n❌ Failed to get IP address")
            return None

    except Exception as e:
        print(f"\n❌ Error during serial configuration: {e}")
        return None

def test_tcp_connection(ip_address):
    """Test TCP connection to ESP32"""
    print_step(2, "TCP Connection Test")

    try:
        # Connect to ESP32
        print(f"Connecting to {ip_address}:{ESP32_PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ip_address, ESP32_PORT))
        print("✓ TCP connection established")

        return sock

    except socket.timeout:
        print("❌ Connection timeout")
        return None
    except ConnectionRefusedError:
        print("❌ Connection refused - TCP server not running")
        return None
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return None

def test_wifi_status(sock):
    """Test WIFI_STATUS command"""
    print_step(3, "WiFi Status Check")

    try:
        # Send WIFI_STATUS command
        print("Sending WIFI_STATUS command...")
        sock.send(b"WIFI_STATUS\n")
        time.sleep(0.5)

        # Receive response
        print("Receiving status...")
        sock.settimeout(3)
        response = b""
        start_time = time.time()

        while time.time() - start_time < 3:
            try:
                chunk = sock.recv(1024)
                if chunk:
                    response += chunk
                else:
                    break
            except socket.timeout:
                break

        if response:
            print("\n--- WiFi Status Response ---")
            lines = response.decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                line = line.strip()
                if line and any(keyword in line for keyword in ['WiFi', 'Status', 'SSID', 'IP', 'MAC', 'RSSI', 'TCP', 'Client', 'Connection', '=====']):
                    print(f"  {line}")
            print("----------------------------")
            print("\n✅ WiFi status check successful")
            return True
        else:
            print("\n❌ No response received")
            return False

    except Exception as e:
        print(f"\n❌ Error during status check: {e}")
        return False

def test_mode_wifi(sock):
    """Test MODE_WIFI command"""
    print_step(4, "MODE_WIFI Command")

    try:
        # Send MODE_WIFI command
        print("Sending MODE_WIFI command...")
        sock.send(b"MODE_WIFI\n")
        time.sleep(0.3)
        print("✓ MODE_WIFI command sent")

        # Try to receive response
        sock.settimeout(1)
        try:
            response = sock.recv(1024).decode('utf-8', errors='ignore')
            if response:
                print(f"Response: {response.strip()}")
        except socket.timeout:
            print("(No immediate response - this is normal)")

        print("\n✅ MODE_WIFI command completed")
        return True

    except Exception as e:
        print(f"\n❌ Error sending MODE_WIFI: {e}")
        return False

def test_data_reception(sock, duration=10):
    """Test receiving motor data via WiFi"""
    print_step(5, f"Data Reception Test ({duration}s)")

    try:
        print(f"Listening for motor data for {duration} seconds...")
        print("(You should see motor data packets if ESP32 is sending)\n")

        sock.settimeout(0.5)
        start_time = time.time()
        packet_count = 0
        motor1_count = 0
        motor2_count = 0

        while time.time() - start_time < duration:
            try:
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                if data:
                    lines = data.split('\n')
                    for line in lines:
                        line = line.strip()
                        if line:
                            packet_count += 1

                            # Count motor packets
                            if 'M:1' in line:
                                motor1_count += 1
                            elif 'M:2' in line:
                                motor2_count += 1

                            # Print first few packets
                            if packet_count <= 10:
                                print(f"  [{packet_count}] {line[:100]}...")
                            elif packet_count == 11:
                                print(f"  ... (continuing to receive data) ...")

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")
                break

        print(f"\n--- Reception Statistics ---")
        print(f"Total packets: {packet_count}")
        print(f"Motor 1 packets: {motor1_count}")
        print(f"Motor 2 packets: {motor2_count}")
        print(f"Duration: {duration}s")
        if packet_count > 0:
            print(f"Average rate: {packet_count/duration:.1f} packets/s")
        print(f"---------------------------")

        if packet_count > 0:
            print(f"\n✅ Data reception successful! Received {packet_count} packets")
            return True
        else:
            print(f"\n❌ No data received in {duration} seconds")
            print("\nPossible issues:")
            print("  1. ESP32 is not in MODE_WIFI output mode")
            print("  2. Motors are not running")
            print("  3. Data is being sent via UART instead of WiFi")
            return False

    except Exception as e:
        print(f"\n❌ Error during data reception: {e}")
        return False

def main():
    """Run complete WiFi flow test"""
    print("\n" + "="*60)
    print("ESP32 WiFi Configuration Flow Test")
    print("="*60)
    print(f"Serial Port: {SERIAL_PORT}")
    print(f"WiFi SSID: {WIFI_SSID}")
    print(f"WiFi Password: {'*' * len(WIFI_PASSWORD)}")
    print("="*60)

    # Step 1: Serial configuration
    ip_address = test_serial_config()
    if not ip_address:
        print("\n❌ TEST FAILED: Could not configure WiFi")
        sys.exit(1)

    time.sleep(1)

    # Step 2: TCP connection
    sock = test_tcp_connection(ip_address)
    if not sock:
        print("\n❌ TEST FAILED: Could not connect via TCP")
        sys.exit(1)

    time.sleep(0.5)

    # Step 3: WiFi status check
    if not test_wifi_status(sock):
        print("\n⚠ WARNING: WiFi status check failed, but continuing...")

    time.sleep(0.5)

    # Step 4: MODE_WIFI command
    if not test_mode_wifi(sock):
        print("\n❌ TEST FAILED: Could not send MODE_WIFI command")
        sock.close()
        sys.exit(1)

    time.sleep(0.5)

    # Step 5: Data reception test
    success = test_data_reception(sock, duration=10)

    # Cleanup
    print("\nClosing connection...")
    sock.close()
    print("✓ Connection closed")

    # Final result
    print("\n" + "="*60)
    if success:
        print("✅ ALL TESTS PASSED!")
        print("="*60)
        print("\nThe WiFi flow is working correctly.")
        print("Motor data should now be visible in the main GUI.")
    else:
        print("❌ TEST FAILED!")
        print("="*60)
        print("\nThe WiFi flow has issues.")
        print("Please check the error messages above.")
    print("="*60 + "\n")

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
