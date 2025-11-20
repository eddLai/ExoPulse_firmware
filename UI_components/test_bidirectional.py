#!/usr/bin/env python3
"""
Simple bidirectional communication test for ESP32
Tests the new [TEST] command for easy verification
"""

import socket
import time

ESP32_IP = "10.16.241.20"
ESP32_PORT = 8888

def test_bidirectional():
    """Test bidirectional communication with TEST command"""
    print("=" * 60)
    print("Bidirectional Communication Test")
    print("=" * 60)

    try:
        # Connect
        print(f"\n[1] Connecting to {ESP32_IP}:{ESP32_PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((ESP32_IP, ESP32_PORT))
        print("✓ Connected!")

        # Send 5 TEST commands
        print("\n[2] Sending TEST commands...")
        success_count = 0

        for i in range(1, 6):
            test_msg = f"[TEST] seq:{i}\n"

            print(f"\n  Sending: {test_msg.strip()}")
            send_time = time.time()
            sock.send(test_msg.encode('utf-8'))

            # Wait for TEST_REPLY
            sock.settimeout(2)
            try:
                data = sock.recv(4096).decode('utf-8', errors='ignore')
                receive_time = time.time()
                rtt = (receive_time - send_time) * 1000  # ms

                # Look for TEST_REPLY
                for line in data.split('\n'):
                    if '[TEST_REPLY]' in line and f'seq:{i}' in line:
                        print(f"  Received: {line}")
                        print(f"  RTT: {rtt:.1f} ms")
                        success_count += 1
                        break
                else:
                    print(f"  ✗ No TEST_REPLY for seq:{i}")

            except socket.timeout:
                print(f"  ✗ Timeout waiting for seq:{i}")

            time.sleep(0.5)

        # Summary
        print(f"\n" + "=" * 60)
        print(f"Results: {success_count}/5 tests passed")

        if success_count == 5:
            print("✓ Bidirectional communication working perfectly!")
        elif success_count > 0:
            print("⚠ Partial success - some packets lost")
        else:
            print("✗ Bidirectional communication failed")

        print("=" * 60)

        sock.close()
        return success_count == 5

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return False


if __name__ == "__main__":
    test_bidirectional()
