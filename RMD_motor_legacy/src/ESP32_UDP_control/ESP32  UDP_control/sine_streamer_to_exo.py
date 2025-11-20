import socket
import time
import math
import json
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
    def __init__(self, esp32_ip: str, esp32_port: int, rate_hz: float = 50.0):
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.rate_hz = rate_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop_event = Event()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        logger.warning("Received stop signal, shutting down...")
        self.stop_event.set()

    # ----------------------------------------------------
    def send_packet(self, right, left, seq):
        """Send one packet (JSON) over UDP."""
        packet = {
            "seq": seq,
            "timestamp": time.time(),
            "right_hip": float(right),
            "left_hip": float(left)
        }
        msg = json.dumps(packet).encode()
        self.sock.sendto(msg, (self.esp32_ip, self.esp32_port))

    # ----------------------------------------------------
    def stream(self):
        """Stream sine wave motion to ESP32 with 50 Hz frequency."""
        logger.info(f"Starting UDP stream to {self.esp32_ip}:{self.esp32_port} at {self.rate_hz} Hz")
        logger.info("Press Ctrl+C to stop")

        # Start the sine wave
        seq = 0
        start_time = time.time()
        try:
            while not self.stop_event.is_set():
                # Calculate sine wave values for right and left hips
                t = time.time() - start_time
                right_hip = 45.0 * math.sin(2 * math.pi * self.rate_hz * t)  # sine wave between -45° and +45°
                left_hip = 45.0 * math.sin(2 * math.pi * self.rate_hz * t)

                # Send the UDP packet
                self.send_packet(right_hip, left_hip, seq)

                # Increment the sequence number
                seq += 1

                # Sleep to achieve the desired sample rate (1/rate_hz)
                time.sleep(1.0 / self.rate_hz)

            logger.info(f"✓ Stream complete. Sent {seq} packets.")
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
    print("ESP32 UDP Exoskeleton Sine Wave Streamer")
    print("=" * 60)

    ESP32_IP = input("Enter ESP32 IP (default 192.168.0.180): ").strip() or "192.168.0.180"
    ESP32_PORT = int(input("Enter ESP32 UDP Port (default 4210): ").strip() or "4210")
    RATE_HZ = float(input("Enter sine wave frequency (Hz, default 50): ").strip() or "50.0")

    streamer = UDPStreamer(ESP32_IP, ESP32_PORT, RATE_HZ)
    streamer.stream()

if __name__ == "__main__":
    main()
