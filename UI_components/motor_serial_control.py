#!/usr/bin/env python3
"""
Motor Serial Control - Python script to control motors via ESP32
Communicates through Serial (115200 baud) to send position commands

Commands (Multi-Turn Absolute Position - 0xA4):
    P1:<angle>         - Move Motor 1 to absolute angle (e.g., P1:30, P1:-45.5)
    P2:<angle>         - Move Motor 2 to absolute angle (e.g., P2:90)
    P1:<angle>:<speed> - Move with speed limit (e.g., P1:30:500)
                         Supports ±359999.99°, default speed=700 dps

Commands (Single-Turn Position - 0xA6, legacy):
    M1:<angle>  - Move Motor 1 to angle (0~359.99°)
    M2:<angle>  - Move Motor 2 to angle

Commands (General):
    STOP        - Stop all motors (0x80)

Usage:
    python motor_serial_control.py                  # Auto-detect port
    python motor_serial_control.py /dev/ttyUSB0    # Specify port
"""

import serial
import serial.tools.list_ports
import time
import sys
import threading
import math


class MotorController:
    def __init__(self, port=None, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.read_thread = None
        self.show_status = False  # Filter out motor status messages by default

    def find_esp32_port(self):
        """Auto-detect ESP32 serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Common ESP32 identifiers
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

    def is_status_line(self, line):
        """Check if line is a motor status update (to be filtered)"""
        # Status lines start with '[' and contain 'SEQ:' and 'M:'
        return line.startswith('[') and 'SEQ:' in line and 'M:' in line

    def read_response(self, timeout=0.5):
        """Read response from ESP32, filtering out status updates"""
        if not self.ser or not self.ser.is_open:
            return None

        responses = []
        end_time = time.time() + timeout
        while time.time() < end_time:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Filter out motor status lines unless show_status is True
                    if not self.is_status_line(line) or self.show_status:
                        responses.append(line)
            else:
                time.sleep(0.01)

        return responses

    def move_motor(self, motor_id, angle):
        """
        Move motor to specified angle (Single-turn, 0xA6 command)

        Args:
            motor_id: 1 or 2
            angle: Target angle in degrees (-359.99 to 359.99)
                   Positive = CW, Negative = CCW
        """
        if motor_id not in [1, 2]:
            print("ERROR: motor_id must be 1 or 2")
            return False

        cmd = f"M{motor_id}:{angle}"
        self.send_command(cmd)
        responses = self.read_response(timeout=0.5)
        for r in responses:
            print(f"RX: {r}")
        return responses

    def move_motor_multi_turn(self, motor_id, angle, max_speed=700):
        """
        Move motor to specified absolute angle (Multi-turn, 0xA4 command)

        Args:
            motor_id: 1 or 2
            angle: Target angle in degrees (supports ±359999.99°)
            max_speed: Maximum speed in dps (default: 700 dps)
        """
        if motor_id not in [1, 2]:
            print("ERROR: motor_id must be 1 or 2")
            return False

        if max_speed != 700:
            cmd = f"P{motor_id}:{angle}:{max_speed}"
        else:
            cmd = f"P{motor_id}:{angle}"
        self.send_command(cmd)
        responses = self.read_response(timeout=0.5)
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

    def sin_motion(self, motor_id, amplitude=45.0, period=2.0, center=90.0, duration=10.0, steps_per_period=50):
        """
        Execute sinusoidal reciprocating motion

        Args:
            motor_id: 1 or 2
            amplitude: Peak amplitude in degrees (e.g., 45 means ±45° from center)
            period: Time for one complete cycle in seconds
            center: Center position in degrees (must be >= amplitude to avoid negative angles)
            duration: Total duration of motion in seconds
            steps_per_period: Number of position updates per period (smoothness)

        Motion: angle = center + amplitude * sin(2*pi*t/period)
        Range: [center - amplitude, center + amplitude]

        Note: 0xA6 command only supports 0~359.99°, negative angles are NOT supported.
              Use center >= amplitude to ensure all angles are positive.
        """
        if motor_id not in [1, 2]:
            print("ERROR: motor_id must be 1 or 2")
            return False

        # Check if motion will produce negative angles
        min_angle = center - amplitude
        max_angle = center + amplitude

        if min_angle < 0:
            print(f"WARNING: min angle ({min_angle:.1f}°) < 0°")
            print(f"  Suggest: center >= amplitude (e.g., center={amplitude})")
            print(f"  Continuing anyway, but motor may not move as expected...")

        if max_angle > 359.99:
            print(f"WARNING: max angle ({max_angle:.1f}°) > 359.99°")
            print(f"  Angles will be clamped to 359.99°")

        print(f"\n=== Sin Motion ===")
        print(f"  Motor: {motor_id}")
        print(f"  Center: {center}°")
        print(f"  Amplitude: ±{amplitude}°")
        print(f"  Range: [{min_angle:.1f}° ~ {max_angle:.1f}°]")
        print(f"  Period: {period}s")
        print(f"  Duration: {duration}s")
        print(f"  Press Ctrl+C to stop")
        print(f"==================\n")

        # Clear buffer
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()

        dt = period / steps_per_period  # Time between updates
        start_time = time.time()

        try:
            while time.time() - start_time < duration:
                t = time.time() - start_time

                # Calculate target angle: center + amplitude * sin(2*pi*t/period)
                angle = center + amplitude * math.sin(2 * math.pi * t / period)

                # Clamp to valid range (0xA6 only supports 0~359.99)
                angle = max(0.0, min(359.99, angle))

                # Send command (don't wait for response to maintain timing)
                cmd = f"M{motor_id}:{angle:.2f}"
                if self.ser and self.ser.is_open:
                    self.ser.write((cmd + '\n').encode('utf-8'))
                    # Clear any responses to prevent buffer overflow
                    if self.ser.in_waiting > 0:
                        self.ser.reset_input_buffer()

                # Print progress
                elapsed = time.time() - start_time
                print(f"\r  t={elapsed:.1f}s  angle={angle:+7.2f}°", end='', flush=True)

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n\nMotion interrupted by user")

        print(f"\n\nMotion complete. Stopping motor...")
        self.stop_all()
        return True

    def sin_motion_multi_turn(self, motor_id, amplitude=30.0, period=2.0, center=0.0, duration=10.0, max_speed=700, steps_per_period=10):
        """
        Execute sinusoidal reciprocating motion using multi-turn position control (0xA4)

        Args:
            motor_id: 1 or 2
            amplitude: Peak amplitude in degrees (e.g., 30 means ±30° from center)
            period: Time for one complete cycle in seconds
            center: Center position in degrees (can be negative)
            duration: Total duration of motion in seconds
            max_speed: Maximum speed limit in dps (default: 700)
            steps_per_period: Number of position updates per period (default: 10)
                              Lower = smoother but slower response
                              Higher = more responsive but may overwhelm motor

        Motion: angle = center + amplitude * sin(2*pi*t/period)
        Range: [center - amplitude, center + amplitude]

        Note: 0xA4 command supports ±359999.99°, so negative angles are OK!
        """
        if motor_id not in [1, 2]:
            print("ERROR: motor_id must be 1 or 2")
            return False

        min_angle = center - amplitude
        max_angle = center + amplitude
        dt = period / steps_per_period  # Time between updates

        print(f"\n=== Sin Motion (Multi-Turn 0xA4) ===")
        print(f"  Motor: {motor_id}")
        print(f"  Center: {center}°")
        print(f"  Amplitude: ±{amplitude}°")
        print(f"  Range: [{min_angle:.1f}° ~ {max_angle:.1f}°]")
        print(f"  Period: {period}s")
        print(f"  Duration: {duration}s")
        print(f"  Max Speed: {max_speed} dps")
        print(f"  Update interval: {dt*1000:.0f}ms ({steps_per_period} steps/period)")
        print(f"  Press Ctrl+C to stop")
        print(f"====================================\n")

        # Clear buffer
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()

        start_time = time.time()
        last_response = ""
        cmd_count = 0

        try:
            while time.time() - start_time < duration:
                t = time.time() - start_time

                # Calculate target angle: center + amplitude * sin(2*pi*t/period)
                angle = center + amplitude * math.sin(2 * math.pi * t / period)

                # Send command using P<motor>:<angle>:<speed> format
                cmd = f"P{motor_id}:{angle:.2f}:{max_speed}"
                if self.ser and self.ser.is_open:
                    self.ser.write((cmd + '\n').encode('utf-8'))
                    cmd_count += 1

                    # Read response to check motor status (non-blocking)
                    time.sleep(0.05)  # Wait 50ms for response
                    if self.ser.in_waiting > 0:
                        try:
                            response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if response and 'S=' in response:
                                # Extract speed from response for monitoring
                                last_response = response
                        except:
                            pass
                        # Clear remaining buffer
                        self.ser.reset_input_buffer()

                # Print progress with motor feedback
                elapsed = time.time() - start_time
                if last_response:
                    # Try to extract speed value for monitoring
                    speed_str = ""
                    if 'S=' in last_response:
                        try:
                            s_idx = last_response.index('S=')
                            end_idx = last_response.find('dps', s_idx)
                            if end_idx > s_idx:
                                speed_str = last_response[s_idx:end_idx+3]
                        except:
                            pass
                    print(f"\r  t={elapsed:.1f}s  target={angle:+7.2f}°  {speed_str}       ", end='', flush=True)
                else:
                    print(f"\r  t={elapsed:.1f}s  target={angle:+7.2f}°", end='', flush=True)

                # Sleep for remaining time (accounting for response read delay)
                remaining_dt = dt - 0.05
                if remaining_dt > 0:
                    time.sleep(remaining_dt)

        except KeyboardInterrupt:
            print("\n\nMotion interrupted by user")

        print(f"\n\nMotion complete ({cmd_count} commands sent). Stopping motor...")
        self.stop_all()
        return True

    def start_monitor(self):
        """Start background thread to monitor motor status"""
        self.running = True
        self.read_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.read_thread.start()

    def _monitor_loop(self):
        """Background loop to consume serial buffer (silent)"""
        while self.running:
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                try:
                    # Just consume the data to keep buffer clear, don't print anything
                    self.ser.readline()
                except:
                    pass
            time.sleep(0.01)


def interactive_mode(controller):
    """Interactive command line interface"""
    print("\n=== Motor Control ===")
    print("--- Multi-Turn Position (0xA4, recommended) ---")
    print("  P1:<angle>         - Motor 1 to absolute angle (e.g., P1:30, P1:-45)")
    print("  P2:<angle>         - Motor 2 to absolute angle")
    print("  P1:<angle>:<speed> - With speed limit (e.g., P1:30:500)")
    print("                       Supports ±359999.99°, default 700 dps")
    print("")
    print("  PSIN <motor> [amp] [period] [center] [duration] [speed]")
    print("              - Sin wave with multi-turn (supports negative angles)")
    print("              - default: amp=30, period=2s, center=0, duration=10s, speed=700")
    print("              - Example: PSIN 1 30 2 0 10 → oscillate between -30° and 30°")
    print("")
    print("--- Single-Turn Position (0xA6, legacy) ---")
    print("  M1:<angle>  - Move Motor 1 (0~359.99°)")
    print("  M2:<angle>  - Move Motor 2")
    print("  SIN <motor> [amp] [period] [center] [duration]")
    print("              - Sin wave (legacy, positive angles only)")
    print("")
    print("--- Other Commands ---")
    print("  STOP        - Stop all motors")
    print("  q           - Exit")
    print("=====================\n")

    # Stop background monitor to avoid consuming responses
    controller.running = False
    if controller.read_thread:
        controller.read_thread.join(timeout=0.5)

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

            # Parse PSIN command (multi-turn sin wave, supports negative angles)
            if cmd.upper().startswith('PSIN'):
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: PSIN <motor> [amplitude] [period] [center] [duration] [speed]")
                    print("Example: PSIN 1 30 2 0 10      → oscillate between -30° and 30°")
                    print("Example: PSIN 1 40 1.5 10 15  → oscillate between -30° and 50°")
                    print("Example: PSIN 1 30 2 0 10 500 → with max speed 500 dps")
                    continue

                try:
                    motor_id = int(parts[1])
                    amplitude = float(parts[2]) if len(parts) > 2 else 30.0
                    period = float(parts[3]) if len(parts) > 3 else 2.0
                    center = float(parts[4]) if len(parts) > 4 else 0.0
                    duration = float(parts[5]) if len(parts) > 5 else 10.0
                    max_speed = int(parts[6]) if len(parts) > 6 else 700

                    controller.sin_motion_multi_turn(motor_id, amplitude, period, center, duration, max_speed)
                except ValueError as e:
                    print(f"Invalid parameter: {e}")
                continue

            # Parse SIN command (legacy, single-turn)
            if cmd.upper().startswith('SIN'):
                parts = cmd.split()
                if len(parts) < 2:
                    print("Usage: SIN <motor> [amplitude] [period] [center] [duration]")
                    print("Example: SIN 1 45 2 90 10  → oscillate between 45° and 135°")
                    print("Example: SIN 1 30 1.5 180 20  → oscillate between 150° and 210°")
                    print("Note: Use PSIN for multi-turn with negative angle support")
                    continue

                try:
                    motor_id = int(parts[1])
                    amplitude = float(parts[2]) if len(parts) > 2 else 45.0
                    period = float(parts[3]) if len(parts) > 3 else 2.0
                    center = float(parts[4]) if len(parts) > 4 else 90.0
                    duration = float(parts[5]) if len(parts) > 5 else 10.0

                    controller.sin_motion(motor_id, amplitude, period, center, duration)
                except ValueError as e:
                    print(f"Invalid parameter: {e}")
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

    controller = MotorController(port=port)

    if not controller.connect():
        sys.exit(1)

    try:
        # Enter interactive mode (no background monitor)
        interactive_mode(controller)

    finally:
        controller.stop_all()
        controller.disconnect()


if __name__ == "__main__":
    main()
