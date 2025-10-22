"""
ESP32 Exoskeleton UDP Streamer (Raw)
====================================
Streams raw hip_exo_r_rotation / hip_exo_l_rotation values
from .sto or .csv directly to ESP32 via UDP — no validation.

Author: Exoskeleton Control System
Date: October 20, 2025
"""

import socket
import time
import json
import pandas as pd
import logging
import sys
import signal
from threading import Event

# -----------------------------
# Logging Setup
# -----------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("UDPStreamer")


# -----------------------------
# UDP Streamer Class
# -----------------------------
class UDPStreamer:
    def __init__(self, esp32_ip: str, esp32_port: int, file_path: str):
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.file_path = file_path
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop_event = Event()
        self.df = None
        self.col_r = None
        self.col_l = None

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        logger.warning("Received stop signal, shutting down...")
        self.stop_event.set()

    # ----------------------------------------------------
    def load_data(self):
        """Load .sto or .csv, detect delimiter and header automatically."""
        try:
            logger.info(f"Loading data from {self.file_path}")

            # Read lines
            with open(self.file_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()

            # Find header index
            header_index = None
            for i, line in enumerate(lines):
                if 'endheader' in line.lower():
                    header_index = i + 1
                    break
            if header_index is None:
                for i, line in enumerate(lines):
                    if any(ch.isdigit() for ch in line):
                        header_index = i - 1
                        break

            # Detect delimiter
            sample_line = lines[header_index].strip()
            if ',' in sample_line:
                sep = ','
            elif '\t' in sample_line:
                sep = '\t'
            else:
                sep = r'\s+'

            # Read numeric data
            df = pd.read_csv(
                self.file_path,
                sep=sep,
                skiprows=header_index,
                engine="python"
            )
            df.columns = df.columns.astype(str).str.strip().str.replace('\ufeff', '')
            self.df = df

            logger.info(f"Detected delimiter: '{sep}'")
            logger.info(f"Detected columns: {list(df.columns)[:10]} ... total {len(df.columns)}")
            logger.info(f"✓ Loaded {len(df)} data rows")

            # Find matching hip columns
            candidates_r = [c for c in df.columns if 'hip_exo_r_rotation' in c]
            candidates_l = [c for c in df.columns if 'hip_exo_l_rotation' in c]
            if not candidates_r or not candidates_l:
                raise KeyError("No hip_exo rotation columns found")

            # Prefer .input if present
            self.col_r = next((c for c in candidates_r if c.endswith('.input')), candidates_r[0])
            self.col_l = next((c for c in candidates_l if c.endswith('.input')), candidates_l[0])

            logger.info(f"Using columns: {self.col_r}, {self.col_l}")
            return True

        except Exception as e:
            logger.error(f"Failed to load data: {e}")
            return False

    # ----------------------------------------------------
    def send_packet(self, right, left):
        """Send one packet (JSON) over UDP."""
        packet = {
            "timestamp": time.time(),
            "right_hip": float(right),
            "left_hip": float(left)
        }
        msg = json.dumps(packet).encode()
        self.sock.sendto(msg, (self.esp32_ip, self.esp32_port))

    # ----------------------------------------------------
    def stream(self, rate_hz: float = 100.0):
        """Stream values directly without any validation."""
        if not self.load_data():
            logger.error("Could not load data file.")
            return

        interval = 1.0 / rate_hz
        logger.info(f"Starting UDP stream to {self.esp32_ip}:{self.esp32_port} at {rate_hz} Hz")
        logger.info("Press Ctrl+C to stop")

        start_time = time.time()
        sent = 0
        try:
            for _, row in self.df.iterrows():
                if self.stop_event.is_set():
                    break
                right = float(row[self.col_r])
                left = float(row[self.col_l])
                self.send_packet(right, left)
                sent += 1
                time.sleep(interval)
            logger.info(f"✓ Stream complete: {sent} packets in {time.time() - start_time:.2f}s")

        except KeyboardInterrupt:
            logger.info("User interrupted streaming.")
        except Exception as e:
            logger.error(f"Error during streaming: {e}")
        finally:
            self.sock.close()
            logger.info("Socket closed.")


# -----------------------------
# Main
# -----------------------------
def main():
    print("=" * 60)
    print("ESP32 UDP Exoskeleton Raw Streamer")
    print("=" * 60)

    ESP32_IP = input("Enter ESP32 IP (default 192.168.0.180): ").strip() or "192.168.0.180"
    ESP32_PORT = int(input("Enter ESP32 UDP Port (default 4210): ").strip() or "4210")
    FILE_PATH = input("Enter STO/CSV file path: ").strip() or "./00001_818.945.sto.csv"

    streamer = UDPStreamer(ESP32_IP, ESP32_PORT, FILE_PATH)
    streamer.stream(rate_hz=50.0)





if __name__ == "__main__":
    main()
