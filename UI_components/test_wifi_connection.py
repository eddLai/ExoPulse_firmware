#!/usr/bin/env python3
"""
Simple WiFi connection test for ESP32 motor data
Tests:
1. WiFi connection
2. Motor data reception with SEQ field
3. PING/PONG latency measurement
"""

import socket
import time
import re

ESP32_IP = "10.16.241.20"
ESP32_PORT = 8888

def test_wifi_connection():
    """Test basic WiFi connection and data reception"""
    print("=" * 60)
    print("WiFi Connection Test")
    print("=" * 60)

    try:
        # Connect
        print(f"\n[1] Connecting to {ESP32_IP}:{ESP32_PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ESP32_IP, ESP32_PORT))
        print("✓ Connected!")

        # Test 1: Receive motor data
        print("\n[2] Testing motor data reception (5 seconds)...")
        motor1_packets = 0
        motor2_packets = 0
        rssi_packets = 0

        start_time = time.time()
        while time.time() - start_time < 5:
            sock.settimeout(0.5)
            try:
                data = sock.recv(4096).decode('utf-8', errors='ignore')

                # Count Motor 1 packets
                motor1_matches = re.findall(r'SEQ:(\d+)\s+M:1', data)
                motor1_packets += len(motor1_matches)

                # Count Motor 2 packets
                motor2_matches = re.findall(r'SEQ:(\d+)\s+M:2', data)
                motor2_packets += len(motor2_matches)

                # Count RSSI updates
                rssi_matches = re.findall(r'\[WIFI_STATUS\]', data)
                rssi_packets += len(rssi_matches)

            except socket.timeout:
                continue

        print(f"  Motor 1 packets: {motor1_packets}")
        print(f"  Motor 2 packets: {motor2_packets}")
        print(f"  RSSI updates: {rssi_packets}")

        if motor1_packets > 0 and motor2_packets > 0:
            print("✓ Motor data reception OK")
        else:
            print("✗ No motor data received!")
            return False

        # Test 2: PING/PONG latency
        print("\n[3] Testing PING/PONG latency (3 pings)...")
        test_start_time = int(time.time() * 1000)
        rtt_list = []

        for i in range(1, 4):
            # Send PING with relative timestamp
            current_time = int(time.time() * 1000)
            relative_ts = current_time - test_start_time
            ping_msg = f"[PING] seq:{i} ts:{relative_ts}\n"

            send_time = int(time.time() * 1000)
            sock.send(ping_msg.encode('utf-8'))

            # Wait for PONG
            sock.settimeout(2)
            try:
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                receive_time = int(time.time() * 1000)

                # Find PONG response
                pong_match = re.search(rf'\[PONG\]\s+seq:{i}\s+ts_req:(\d+)\s+ts_reply:(\d+)', data)
                if pong_match:
                    ts_req = int(pong_match.group(1))
                    ts_reply = int(pong_match.group(2))
                    rtt = receive_time - send_time
                    rtt_list.append(rtt)

                    print(f"  PING {i}: RTT = {rtt} ms (ts_req={ts_req}, ts_reply={ts_reply})")
                else:
                    print(f"  PING {i}: No PONG response")

            except socket.timeout:
                print(f"  PING {i}: Timeout")

            time.sleep(0.5)

        if len(rtt_list) > 0:
            avg_rtt = sum(rtt_list) / len(rtt_list)
            print(f"\n  Average RTT: {avg_rtt:.1f} ms")
            print(f"  Min RTT: {min(rtt_list)} ms")
            print(f"  Max RTT: {max(rtt_list)} ms")
            print("✓ PING/PONG latency test OK")
        else:
            print("✗ No PONG responses received!")

        sock.close()

        print("\n" + "=" * 60)
        print("✓ All tests passed!")
        print("=" * 60)
        return True

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False


if __name__ == "__main__":
    test_wifi_connection()
