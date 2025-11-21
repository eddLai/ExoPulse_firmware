#!/usr/bin/env python3
"""
Torque Serial Control - Python script to control motor torque via ESP32
Communicates through Serial (115200 baud) to send torque commands

Commands:
    T1:<iq>     - Set Motor 1 torque (-800~800, e.g., T1:200)
    T2:<iq>     - Set Motor 2 torque (-800~800, e.g., T2:-200)
    STOP        - Stop all motors (0x80)

Usage:
    python torque_serial_control.py                  # Auto-detect port
    python torque_serial_control.py /dev/ttyUSB0    # Specify port
"""

import serial
import serial.tools.list_ports
import time
import sys
import threading


class TorqueController:
    def __init__(self, port=None, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.read_thread = None

    def find_esp32_port(self):
        """Auto-detect ESP32 serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.device or 'ACM' in port.device:
                print(f"Found potential ESP32 port: {port.device} - {port.description}")
                return port.device
            if 'CP210' in port.description or 'CH340' in port.description:
                print(f"Found ESP32 port: {port.device} - {port.description}")
                return port.device
        return None

    def connect(self):
        """Connect to ESP32"""
        if self.port is None:
            self.port = self.find_esp32_port()
            if self.port is None:
                print("ERROR: No ESP32 port found. Please specify port manually.")
                return False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for ESP32 to reset
            self.ser.reset_input_buffer()
            print(f"Connected to {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from ESP32"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")

    def send_command(self, cmd):
        """Send command to ESP32"""
        if not self.ser or not self.ser.is_open:
            print("ERROR: Not connected")
            return False

        try:
            cmd_bytes = (cmd.strip() + '\n').encode('utf-8')
            self.ser.write(cmd_bytes)
            print(f"TX: {cmd.strip()}")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Send failed: {e}")
            return False

    def read_response(self, timeout=0.5):
        """Read response from ESP32"""
        if not self.ser or not self.ser.is_open:
            return None

        responses = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    responses.append(line)
            else:
                time.sleep(0.01)

        return responses

    def set_torque(self, motor_id, iq_value):
        """
        Set motor torque

        Args:
            motor_id: 1 or 2
            iq_value: Torque value (-800~800)
        """
        if motor_id not in [1, 2]:
            print("ERROR: motor_id must be 1 or 2")
            return False

        # Clamp iq_value to safety limit
        iq_value = max(-800, min(800, int(iq_value)))

        cmd = f"T{motor_id}:{iq_value}"
        self.send_command(cmd)
        responses = self.read_response(timeout=0.3)
        for r in responses:
            print(f"RX: {r}")
        return responses

    def stop_all(self):
        """Stop all motors"""
        self.send_command("STOP")
        responses = self.read_response(timeout=0.5)
        for r in responses:
            print(f"RX: {r}")
        return responses

    def torque_test(self, motor_id, iq_value=200, duration=5.0):
        """
        Execute torque test: forward for duration, then reverse for duration

        Args:
            motor_id: 1, 2, or 0 for both
            iq_value: Torque value (default 200)
            duration: Duration for each direction in seconds (default 5s)
        """
        print(f"\n=== Torque Test ===")
        print(f"  Motor: {'Both' if motor_id == 0 else motor_id}")
        print(f"  iq value: ±{iq_value}")
        print(f"  Duration: {duration}s per direction")
        print(f"  Press Ctrl+C to stop")
        print(f"===================\n")

        # Clear buffer
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()

        motors = [1, 2] if motor_id == 0 else [motor_id]

        try:
            # Phase 1: Forward (positive torque)
            print(f"[FORWARD] iq={iq_value} for {duration}s")
            start_time = time.time()
            last_print = start_time
            while time.time() - start_time < duration:
                for m in motors:
                    cmd = f"T{m}:{iq_value}\n"
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.encode('utf-8'))
                        if self.ser.in_waiting > 0:
                            self.ser.reset_input_buffer()

                # Print progress every 100ms, but send commands every 10ms
                now = time.time()
                if now - last_print >= 0.1:
                    elapsed = now - start_time
                    print(f"\r  t={elapsed:.1f}s / {duration:.1f}s", end='', flush=True)
                    last_print = now
                time.sleep(0.01)  # 10ms interval for torque control

            print()

            # Phase 2: Reverse (negative torque)
            print(f"[REVERSE] iq={-iq_value} for {duration}s")
            start_time = time.time()
            last_print = start_time
            while time.time() - start_time < duration:
                for m in motors:
                    cmd = f"T{m}:{-iq_value}\n"
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.encode('utf-8'))
                        if self.ser.in_waiting > 0:
                            self.ser.reset_input_buffer()

                # Print progress every 100ms, but send commands every 10ms
                now = time.time()
                if now - last_print >= 0.1:
                    elapsed = now - start_time
                    print(f"\r  t={elapsed:.1f}s / {duration:.1f}s", end='', flush=True)
                    last_print = now
                time.sleep(0.01)  # 10ms interval for torque control

            print()

        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")

        print(f"\nTest complete. Stopping motors...")
        self.stop_all()
        return True


def interactive_mode(controller):
    """Interactive command line interface"""
    print("\n=== Torque Control ===")
    print("  T1:<iq>     - Set Motor 1 torque (-800~800)")
    print("  T2:<iq>     - Set Motor 2 torque (-800~800)")
    print("  STOP        - Stop all motors")
    print("  TEST <motor> [iq] [duration]")
    print("              - Torque test (5s forward + 5s reverse)")
    print("              - motor: 0=both, 1=M1, 2=M2")
    print("              - default: iq=200, duration=5s")
    print("  q           - Exit")
    print("======================\n")

    # Clear any pending data
    if controller.ser and controller.ser.is_open:
        controller.ser.reset_input_buffer()

    while True:
        try:
            cmd = input("> ").strip()
            if not cmd:
                continue

            if cmd.lower() in ['q', 'quit', 'exit']:
                break

            # Parse TEST command
            if cmd.upper().startswith('TEST'):
                parts = cmd.split()
                try:
                    motor_id = int(parts[1]) if len(parts) > 1 else 0
                    iq_value = int(parts[2]) if len(parts) > 2 else 200
                    duration = float(parts[3]) if len(parts) > 3 else 5.0

                    controller.torque_test(motor_id, iq_value, duration)
                except (ValueError, IndexError) as e:
                    print("Usage: TEST <motor> [iq] [duration]")
                    print("Example: TEST 0 200 5  → both motors, iq=±200, 5s each direction")
                    print("Example: TEST 1 300 3  → motor 1, iq=±300, 3s each direction")
                continue

            # Clear buffer before sending
            controller.ser.reset_input_buffer()

            controller.send_command(cmd)
            responses = controller.read_response(timeout=0.5)
            for r in responses:
                print(f"  {r}")

        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except EOFError:
            break


def main():
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]

    controller = TorqueController(port=port)

    if not controller.connect():
        sys.exit(1)

    try:
        interactive_mode(controller)

    finally:
        controller.stop_all()
        controller.disconnect()


if __name__ == "__main__":
    main()
