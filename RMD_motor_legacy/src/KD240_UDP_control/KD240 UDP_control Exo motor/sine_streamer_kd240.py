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
    def __init__(self, kd240_ip: str, kd240_port: int, rate_hz: float = 50.0):
        self.kd240_ip = kd240_ip
        self.kd240_port = kd240_port
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
        self.sock.sendto(msg, (self.kd240_ip, self.kd240_port))

    # ----------------------------------------------------
    def stream(self):
        """Stream sine wave motion to KD240 with the given frequency."""
        logger.info(f"Starting UDP stream to {self.kd240_ip}:{self.kd240_port} at {self.rate_hz} Hz")
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
    print("Host PC to KD240 UDP Streamer")
    print("=" * 60)

    KD240_IP = input("Enter KD240 IP (default 192.168.0.192): ").strip() or "192.168.0.192"
    KD240_PORT = int(input("Enter KD240 UDP Port (default 4210): ").strip() or "4210")
    RATE_HZ = float(input("Enter sine wave frequency (Hz, default 50): ").strip() or "50.0")

    streamer = UDPStreamer(KD240_IP, KD240_PORT, RATE_HZ)
    streamer.stream()

if __name__ == "__main__":
    main()
