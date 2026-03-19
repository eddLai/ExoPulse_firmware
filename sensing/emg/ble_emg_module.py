"""
BLE EMG Module
Wraps BLE communication with dual EMG2ch_B devices (4 channels total).

Refactored from module_test/ble_emg/ble_emg_receiver.py
- Removed matplotlib plotting
- Added thread-safe ring buffer + daemon BLE event loop
- read() returns latest 4 channels: [Dev1_T2, Dev1_T4, Dev2_T2, Dev2_T4]

Note: Outputs raw ADC values. Signal processing (DC removal, bandpass,
rectification, envelope detection, normalization) not yet implemented.
"""

import asyncio
import time
import threading
import numpy as np
from collections import deque
from typing import Optional, List, Tuple

from bleak import BleakClient, BleakScanner

from emg_interface import EMGInterface


# ============== Configuration ==============
SAMPLES_PER_PACKET = 7
DEVICE_NAME = "EMG2ch_B"
UUID_NOTIFY = "6b400003-b5a3-f393-e0a9-e50e24dcca9e"

DEVICES = {
    'ABE': {
        'label': 'Device 1 (ABE)',
        'header': 'ABE',
    },
    'ABB': {
        'label': 'Device 2 (ABB)',
        'header': 'ABB',
    },
}

# Ring buffer size (keep last N samples per channel)
RING_BUFFER_SIZE = 1024


# ============== Packet Parsing ==============
def parse_emg_packet(hex_text: str) -> Tuple[List[float], List[float], Optional[int]]:
    """Parse EMG data packet.
    ITRI format: 7 groups x 24 hex chars, extract T2[6:12] and T4[18:24].
    Returns (t2_list, t4_list, seq_num).
    """
    t2_list, t4_list = [], []
    bit_num = 24

    # Sequence number: 4th character (hex digit, 0-F)
    seq_num = int(hex_text[3], 16) if len(hex_text) > 3 else None

    for i in range(1, 8):
        start = 4 + bit_num * (i - 1)
        end = 4 + bit_num * i
        if end > len(hex_text):
            break
        group = hex_text[start:end]
        t2_list.append(float(int(group[6:12], 16)))
        t4_list.append(float(int(group[18:24], 16)))

    return t2_list, t4_list, seq_num


def get_device_by_header(hex_str: str) -> Optional[str]:
    """Identify device by packet header (first 3 chars)."""
    header = hex_str[:3]
    for key, cfg in DEVICES.items():
        if cfg['header'] == header:
            return key
    return None


# ============== BLE EMG Module ==============
class BLEEMGModule(EMGInterface):
    """BLE EMG module for dual EMG2ch_B devices (4 channels).

    Channels:
        [0] Device1 (ABE) T2
        [1] Device1 (ABE) T4
        [2] Device2 (ABB) T2
        [3] Device2 (ABB) T4
    """

    def __init__(self, scan_timeout: float = 10.0, connect_timeout: float = 20.0):
        self._scan_timeout = scan_timeout
        self._connect_timeout = connect_timeout

        # Thread-safe ring buffers: latest samples per device/channel
        self._lock = threading.Lock()
        self._buffers = {
            'ABE': {'t2': deque(maxlen=RING_BUFFER_SIZE),
                    't4': deque(maxlen=RING_BUFFER_SIZE)},
            'ABB': {'t2': deque(maxlen=RING_BUFFER_SIZE),
                    't4': deque(maxlen=RING_BUFFER_SIZE)},
        }

        # Packet statistics
        self._stats = {
            key: {'received': 0, 'expected_seq': None, 'lost': 0, 'start_time': None}
            for key in DEVICES
        }

        # BLE state
        self._clients = {}
        self._loop = None
        self._thread = None
        self._running = False
        self._connected = False

    def init(self) -> bool:
        """Start BLE daemon thread, scan and connect to devices.
        Returns True if at least one device connected.
        """
        self._running = True
        self._thread = threading.Thread(target=self._ble_thread, daemon=True)
        self._thread.start()

        # Wait for connection
        deadline = time.time() + self._scan_timeout + self._connect_timeout + 5
        while not self._connected and self._running and time.time() < deadline:
            time.sleep(0.5)

        return self._connected

    def read(self) -> np.ndarray:
        """Return latest 4 channel values as np.ndarray(4,).
        Order: [ABE_T2, ABE_T4, ABB_T2, ABB_T4]
        Returns zeros if no data available.
        """
        result = np.zeros(4, dtype=np.float64)
        with self._lock:
            if self._buffers['ABE']['t2']:
                result[0] = self._buffers['ABE']['t2'][-1]
            if self._buffers['ABE']['t4']:
                result[1] = self._buffers['ABE']['t4'][-1]
            if self._buffers['ABB']['t2']:
                result[2] = self._buffers['ABB']['t2'][-1]
            if self._buffers['ABB']['t4']:
                result[3] = self._buffers['ABB']['t4'][-1]
        return result

    def read_batch(self, n_samples: int = 7) -> np.ndarray:
        """Return last n_samples for each channel as np.ndarray(4, n_samples).
        Useful for buffered processing. Pads with zeros if not enough data.
        """
        result = np.zeros((4, n_samples), dtype=np.float64)
        with self._lock:
            for i, (dev, ch) in enumerate([
                ('ABE', 't2'), ('ABE', 't4'),
                ('ABB', 't2'), ('ABB', 't4'),
            ]):
                buf = self._buffers[dev][ch]
                n = min(len(buf), n_samples)
                if n > 0:
                    data = list(buf)[-n:]
                    result[i, n_samples - n:] = data
        return result

    def get_stats(self) -> dict:
        """Return packet statistics for each device."""
        with self._lock:
            return {
                key: {
                    'received': s['received'],
                    'lost': s['lost'],
                    'loss_rate': (s['lost'] / max(s['received'] + s['lost'], 1)) * 100,
                    'elapsed': time.time() - s['start_time'] if s['start_time'] else 0,
                }
                for key, s in self._stats.items()
            }

    def close(self):
        """Stop BLE event loop and disconnect."""
        self._running = False
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=5)

    # ---- Internal BLE logic ----

    def _handle_notification(self, data: bytes):
        """Parse BLE notification and store in ring buffer."""
        hex_str = data.hex().upper()
        device_key = get_device_by_header(hex_str)
        if device_key is None:
            return

        try:
            t2, t4, seq_num = parse_emg_packet(hex_str)

            with self._lock:
                stats = self._stats[device_key]

                if stats['start_time'] is None:
                    stats['start_time'] = time.time()

                # Packet loss tracking
                if seq_num is not None:
                    if stats['expected_seq'] is not None:
                        expected = stats['expected_seq']
                        if seq_num != expected:
                            if seq_num > expected:
                                lost = seq_num - expected
                            else:
                                lost = (16 - expected) + seq_num
                            stats['lost'] += lost
                    stats['expected_seq'] = (seq_num + 1) % 16

                stats['received'] += 1

                self._buffers[device_key]['t2'].extend(t2)
                self._buffers[device_key]['t4'].extend(t4)

        except Exception as e:
            print(f"[EMG] Parse error: {e}")

    def _ble_thread(self):
        """Daemon thread running BLE event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ble_main())
        except Exception as e:
            print(f"[EMG] BLE thread error: {e}")
        finally:
            self._loop.close()

    async def _ble_main(self):
        """Scan, connect, and receive notifications."""
        # Scan
        print(f"[EMG] Scanning for '{DEVICE_NAME}' devices ({self._scan_timeout}s)...")
        devices = await BleakScanner.discover(timeout=self._scan_timeout)
        emg_devices = [d for d in devices if d.name == DEVICE_NAME]

        if not emg_devices:
            print(f"[EMG] No devices named '{DEVICE_NAME}' found")
            self._running = False
            return

        print(f"[EMG] Found {len(emg_devices)} device(s)")

        # Connect to up to 2 devices
        device_keys = list(DEVICES.keys())
        for i, device in enumerate(emg_devices[:len(device_keys)]):
            key = device_keys[i]
            label = DEVICES[key]['label']
            print(f"[EMG] Connecting to {label} ({device.address})...")

            try:
                client = BleakClient(device, timeout=self._connect_timeout)
                await client.connect()

                def on_notify(_, data, _self=self):
                    _self._handle_notification(bytes(data))

                await client.start_notify(UUID_NOTIFY, on_notify)
                self._clients[key] = client
                print(f"[EMG] {label} connected")
            except Exception as e:
                print(f"[EMG] {label} connection failed: {e}")

        if not self._clients:
            print("[EMG] Failed to connect to any device")
            self._running = False
            return

        self._connected = True
        names = [DEVICES[k]['label'] for k in self._clients]
        print(f"[EMG] Active: {', '.join(names)}")

        # Keep running
        while self._running:
            await asyncio.sleep(0.1)

        # Disconnect
        for key, client in self._clients.items():
            try:
                if client.is_connected:
                    await client.stop_notify(UUID_NOTIFY)
                    await client.disconnect()
                    print(f"[EMG] {DEVICES[key]['label']} disconnected")
            except Exception:
                pass
