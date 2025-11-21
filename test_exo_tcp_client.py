#!/usr/bin/env python3
"""
Test script for ExoDataBroadcaster TCP connection

Usage:
    1. Start experiment_menu.py via ./launch.sh
    2. Wait for "📊 Realtime data panel: ON"
    3. Run this script: python test_exo_tcp_client.py
    4. Should see exo data streaming every 100ms (throttled display every 1s)
"""

import socket
import json
import time
import math

HOST = "127.0.0.1"
PORT = 9998

def main():
    print("=" * 60)
    print("ExoDataBroadcaster TCP Client Test")
    print("=" * 60)
    print(f"Connecting to {HOST}:{PORT}...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((HOST, PORT))
        print(f"✓ Connected to {HOST}:{PORT}")
        print("-" * 60)
        print("Waiting for data... (Ctrl+C to stop)")
        print("-" * 60)

        sock.settimeout(2.0)
        buffer = ""
        last_print_time = 0
        msg_count = 0

        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    print("\n⚠ Server closed connection")
                    break

                buffer += data.decode('utf-8')

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            exo_data = json.loads(line)
                            msg_count += 1

                            # Throttle display to every 1 second
                            current_time = time.time()
                            if current_time - last_print_time >= 1.0:
                                last_print_time = current_time

                                la = exo_data.get('la', 0)
                                ra = exo_data.get('ra', 0)
                                lt = exo_data.get('lt')
                                rt = exo_data.get('rt')
                                ts = exo_data.get('ts', 0)

                                la_deg = math.degrees(la)
                                ra_deg = math.degrees(ra)

                                lt_str = f"{lt:.2f}" if lt is not None else "N/A"
                                rt_str = f"{rt:.2f}" if rt is not None else "N/A"

                                # Check if values are non-zero
                                status = "✓" if (la != 0 or ra != 0) else "⚠️ ZERO"
                                print(f"[{msg_count:6d}] {status} L:{la_deg:7.2f}° R:{ra_deg:7.2f}° | "
                                      f"Torque L:{lt_str:>6}Nm R:{rt_str:>6}Nm | ts:{ts:.3f}")

                        except json.JSONDecodeError as e:
                            print(f"⚠ JSON parse error: {e}")

            except socket.timeout:
                print("⚠ Timeout - no data received for 2 seconds")
                continue

    except ConnectionRefusedError:
        print(f"✗ Connection refused - is experiment_menu.py running?")
        print(f"  Make sure ExoDataBroadcaster is started (port {PORT})")
    except socket.timeout:
        print(f"✗ Connection timeout - server not responding")
    except KeyboardInterrupt:
        print(f"\n\nStopped by user. Total messages received: {msg_count}")
    except Exception as e:
        print(f"✗ Error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass
        print("\nConnection closed.")

if __name__ == "__main__":
    main()
