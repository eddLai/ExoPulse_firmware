#!/usr/bin/env python3
"""
EMG Module Test
Standalone test: BLE connect -> continuous print of 4 channel raw values

Usage: python3 test_emg.py
"""

import time
import signal
import sys

from ble_emg_module import BLEEMGModule

running = True


def signal_handler(sig, frame):
    global running
    running = False


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("=" * 50)
    print("EMG Module Test")
    print("Channels: [ABE_T2, ABE_T4, ABB_T2, ABB_T4]")
    print("=" * 50)

    emg = BLEEMGModule(scan_timeout=10.0, connect_timeout=20.0)

    print("\nInitializing (scanning + connecting)...")
    if not emg.init():
        print("Failed to initialize EMG module")
        sys.exit(1)

    print("\nReading EMG data (Ctrl+C to stop)...\n")

    count = 0
    try:
        while running:
            data = emg.read()
            count += 1

            if count % 10 == 0:  # Print every 10th read (~1s at 10Hz)
                stats = emg.get_stats()
                print(f"EMG [{count:6d}]: "
                      f"ABE_T2={data[0]:8.0f}  ABE_T4={data[1]:8.0f}  "
                      f"ABB_T2={data[2]:8.0f}  ABB_T4={data[3]:8.0f}  | "
                      f"Loss: ABE={stats['ABE']['loss_rate']:.1f}% "
                      f"ABB={stats['ABB']['loss_rate']:.1f}%")

            time.sleep(0.1)  # 10 Hz read rate

    except KeyboardInterrupt:
        pass

    print("\n\nFinal statistics:")
    stats = emg.get_stats()
    for key, s in stats.items():
        print(f"  {key}: received={s['received']}, lost={s['lost']}, "
              f"loss_rate={s['loss_rate']:.2f}%, elapsed={s['elapsed']:.1f}s")

    emg.close()
    print("Done")


if __name__ == "__main__":
    main()
