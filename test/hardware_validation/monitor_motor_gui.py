#!/usr/bin/env python3
"""
Real-time Motor Status Monitor - GUI Version with Live Plotting
Displays LK-TECH motor status with live graphs
"""

import serial
import re
import sys
import threading
import time
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

class MotorMonitorGUI:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False

        # Data storage with timestamps
        self.timestamps = deque(maxlen=max_points)
        self.data = {
            'temperature': deque(maxlen=max_points),
            'voltage': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'encoder': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
            'error': deque(maxlen=max_points),
        }

        # Current status
        self.current_status = {
            'temperature': 'N/A',
            'voltage': 'N/A',
            'current': 'N/A',
            'speed': 'N/A',
            'encoder': 'N/A',
            'angle': 'N/A',
            'error': '0x0',
            'error_flags': [],
        }

        self.start_time = time.time()

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            time.sleep(2)  # Wait for ESP32 to stabilize
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False

    def parse_motor_status(self, lines):
        """Parse motor status from serial lines (optimized for fast format)"""
        status = {}

        for line in lines:
            line = line.strip()

            # Fast format: [12345] T:30 V:24.5 I:0.03 S:0 E:4832 A:265.43 ERR:0x0
            if line.startswith('[') and '] T:' in line:
                try:
                    # Parse compact format
                    match = re.search(r'\[(\d+)\] T:(-?\d+) V:(-?\d+\.?\d*) I:(-?\d+\.?\d*) S:(-?\d+) E:(\d+) A:(-?\d+\.?\d*) ERR:(0x[0-9A-Fa-f]+)', line)
                    if match:
                        status['timestamp'] = int(match.group(1))
                        status['temperature'] = int(match.group(2))
                        status['voltage'] = float(match.group(3))
                        status['current'] = float(match.group(4))
                        status['speed'] = int(match.group(5))
                        status['encoder'] = int(match.group(6))
                        status['angle'] = float(match.group(7))
                        status['error'] = int(match.group(8), 16)
                        status['error_flags'] = []
                        return status
                except:
                    pass

            # Fallback: Old detailed format
            # Temperature
            if 'Temperature:' in line:
                match = re.search(r'Temperature:\s+(-?\d+)\s+°C', line)
                if match:
                    status['temperature'] = int(match.group(1))

            # Voltage
            elif 'Voltage:' in line:
                match = re.search(r'Voltage:\s+(\d+\.?\d*)\s+V', line)
                if match:
                    status['voltage'] = float(match.group(1))

            # Torque Current
            elif 'Torque Current:' in line:
                match = re.search(r'Torque Current:\s+(-?\d+\.?\d*)\s+A', line)
                if match:
                    status['current'] = float(match.group(1))

            # Speed
            elif 'Speed:' in line:
                match = re.search(r'Speed:\s+(-?\d+)\s+dps', line)
                if match:
                    status['speed'] = int(match.group(1))

            # Encoder
            elif 'Encoder:' in line:
                match = re.search(r'Encoder:\s+(\d+)', line)
                if match:
                    status['encoder'] = int(match.group(1))

            # Multi-turn Angle
            elif 'Multi-turn Angle:' in line:
                match = re.search(r'Multi-turn Angle:(-?\d+\.?\d*)\s+°', line)
                if match:
                    status['angle'] = float(match.group(1))

            # Error State
            elif 'Error State:' in line:
                match = re.search(r'Error State:\s+(0x[0-9A-Fa-f]+)', line)
                if match:
                    error_hex = match.group(1)
                    status['error'] = int(error_hex, 16)
                    status['error_flags'] = []
                    if '[LOW_VOLTAGE]' in line:
                        status['error_flags'].append('LOW_VOLTAGE')
                    if '[OVER_TEMP]' in line:
                        status['error_flags'].append('OVER_TEMP')

        return status if status else None

    def read_serial_thread(self):
        """Background thread to read serial data (optimized for fast updates)"""
        buffer = []

        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        # Fast format: single line status update
                        if line.startswith('[') and '] T:' in line:
                            status = self.parse_motor_status([line])

                            if status:
                                # Update current status
                                self.current_status.update(status)

                                # Add to data history
                                elapsed = time.time() - self.start_time
                                self.timestamps.append(elapsed)

                                self.data['temperature'].append(status.get('temperature', None))
                                self.data['voltage'].append(status.get('voltage', None))
                                self.data['current'].append(status.get('current', None))
                                self.data['speed'].append(status.get('speed', None))
                                self.data['encoder'].append(status.get('encoder', None))
                                self.data['angle'].append(status.get('angle', None))
                                self.data['error'].append(status.get('error', 0))
                        else:
                            # Old format: multi-line status block
                            buffer.append(line)

                            # Check if we've completed a status block
                            if '--------------------' in line:
                                status = self.parse_motor_status(buffer)

                                if status:
                                    # Update current status
                                    self.current_status.update(status)

                                    # Add to data history
                                    elapsed = time.time() - self.start_time
                                    self.timestamps.append(elapsed)

                                    self.data['temperature'].append(status.get('temperature', None))
                                    self.data['voltage'].append(status.get('voltage', None))
                                    self.data['current'].append(status.get('current', None))
                                    self.data['speed'].append(status.get('speed', None))
                                    self.data['encoder'].append(status.get('encoder', None))
                                    self.data['angle'].append(status.get('angle', None))
                                    self.data['error'].append(status.get('error', 0))

                                buffer = []

                time.sleep(0.001)  # Much shorter sleep for faster response

            except Exception as e:
                print(f"Serial read error: {e}")
                break

    def init_plot(self):
        """Initialize the plot layout"""
        plt.style.use('dark_background')

        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title('LK-TECH Motor Monitor - Real-time')

        # Create grid layout
        gs = gridspec.GridSpec(4, 2, figure=self.fig, hspace=0.4, wspace=0.3)

        # Create subplots
        self.ax_temp = self.fig.add_subplot(gs[0, 0])
        self.ax_voltage = self.fig.add_subplot(gs[0, 1])
        self.ax_current = self.fig.add_subplot(gs[1, 0])
        self.ax_speed = self.fig.add_subplot(gs[1, 1])
        self.ax_encoder = self.fig.add_subplot(gs[2, 0])
        self.ax_angle = self.fig.add_subplot(gs[2, 1])
        self.ax_status = self.fig.add_subplot(gs[3, :])

        # Configure axes
        self.ax_temp.set_title('Temperature (°C)', fontsize=10, fontweight='bold')
        self.ax_temp.set_ylabel('°C')
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_voltage.set_title('Voltage (V)', fontsize=10, fontweight='bold')
        self.ax_voltage.set_ylabel('V')
        self.ax_voltage.grid(True, alpha=0.3)

        self.ax_current.set_title('Torque Current (A)', fontsize=10, fontweight='bold')
        self.ax_current.set_ylabel('A')
        self.ax_current.grid(True, alpha=0.3)

        self.ax_speed.set_title('Speed (dps)', fontsize=10, fontweight='bold')
        self.ax_speed.set_ylabel('dps')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_encoder.set_title('Encoder Position', fontsize=10, fontweight='bold')
        self.ax_encoder.set_ylabel('Position')
        self.ax_encoder.grid(True, alpha=0.3)

        self.ax_angle.set_title('Multi-turn Angle (°)', fontsize=10, fontweight='bold')
        self.ax_angle.set_ylabel('Degrees')
        self.ax_angle.grid(True, alpha=0.3)

        self.ax_status.set_title('Current Status', fontsize=12, fontweight='bold')
        self.ax_status.axis('off')

        # Initialize line objects
        self.line_temp, = self.ax_temp.plot([], [], 'r-', linewidth=2, label='Temperature')
        self.line_voltage, = self.ax_voltage.plot([], [], 'y-', linewidth=2, label='Voltage')
        self.line_current, = self.ax_current.plot([], [], 'c-', linewidth=2, label='Current')
        self.line_speed, = self.ax_speed.plot([], [], 'g-', linewidth=2, label='Speed')
        self.line_encoder, = self.ax_encoder.plot([], [], 'm-', linewidth=2, label='Encoder')
        self.line_angle, = self.ax_angle.plot([], [], 'b-', linewidth=2, label='Angle')

        # Status text
        self.status_text = self.ax_status.text(
            0.5, 0.5, '',
            ha='center', va='center',
            fontsize=12,
            family='monospace',
            transform=self.ax_status.transAxes
        )

        return (self.line_temp, self.line_voltage, self.line_current,
                self.line_speed, self.line_encoder, self.line_angle, self.status_text)

    def update_plot(self, frame):
        """Update plot data"""
        if len(self.timestamps) == 0:
            return (self.line_temp, self.line_voltage, self.line_current,
                    self.line_speed, self.line_encoder, self.line_angle, self.status_text)

        times = list(self.timestamps)

        # Update temperature
        temp_data = [t if t is not None else 0 for t in self.data['temperature']]
        if temp_data:
            self.line_temp.set_data(times, temp_data)
            self.ax_temp.relim()
            self.ax_temp.autoscale_view()

        # Update voltage
        voltage_data = [v if v is not None else 0 for v in self.data['voltage']]
        if voltage_data:
            self.line_voltage.set_data(times, voltage_data)
            self.ax_voltage.relim()
            self.ax_voltage.autoscale_view()

        # Update current
        current_data = [c if c is not None else 0 for c in self.data['current']]
        if current_data:
            self.line_current.set_data(times, current_data)
            self.ax_current.relim()
            self.ax_current.autoscale_view()

        # Update speed
        speed_data = [s if s is not None else 0 for s in self.data['speed']]
        if speed_data:
            self.line_speed.set_data(times, speed_data)
            self.ax_speed.relim()
            self.ax_speed.autoscale_view()

        # Update encoder
        encoder_data = [e if e is not None else 0 for e in self.data['encoder']]
        if encoder_data:
            self.line_encoder.set_data(times, encoder_data)
            self.ax_encoder.relim()
            self.ax_encoder.autoscale_view()

        # Update angle
        angle_data = [a if a is not None else 0 for a in self.data['angle']]
        if angle_data:
            self.line_angle.set_data(times, angle_data)
            self.ax_angle.relim()
            self.ax_angle.autoscale_view()

        # Update status text
        status = self.current_status
        temp = status.get('temperature', 'N/A')
        voltage = status.get('voltage', 'N/A')
        current = status.get('current', 'N/A')
        speed = status.get('speed', 'N/A')
        encoder = status.get('encoder', 'N/A')
        angle = status.get('angle', 'N/A')
        error = status.get('error', 'N/A')
        error_flags = status.get('error_flags', [])

        # Format status text
        temp_str = f"{temp}°C" if isinstance(temp, (int, float)) else temp
        voltage_str = f"{voltage:.1f}V" if isinstance(voltage, (int, float)) else voltage
        current_str = f"{current:.2f}A" if isinstance(current, (int, float)) else current
        speed_str = f"{speed}dps" if isinstance(speed, (int, float)) else speed
        encoder_str = f"{encoder}" if isinstance(encoder, (int, float)) else encoder
        angle_str = f"{angle:.2f}°" if isinstance(angle, (int, float)) else angle

        error_text = ""
        if error_flags:
            error_text = f"  ⚠️ ERRORS: {', '.join(error_flags)}"

        status_str = (
            f"Temperature: {temp_str:>8}  |  Voltage: {voltage_str:>8}  |  Current: {current_str:>8}\n"
            f"Speed: {speed_str:>13}  |  Encoder: {encoder_str:>8}  |  Angle: {angle_str:>12}\n"
            f"Error State: 0x{error:02X} {error_text}"
        )

        self.status_text.set_text(status_str)

        # Color code based on status
        if error != 0 or error_flags:
            self.status_text.set_color('red')
        elif isinstance(temp, (int, float)) and temp > 70:
            self.status_text.set_color('red')
        elif isinstance(temp, (int, float)) and temp > 50:
            self.status_text.set_color('yellow')
        else:
            self.status_text.set_color('lime')

        return (self.line_temp, self.line_voltage, self.line_current,
                self.line_speed, self.line_encoder, self.line_angle, self.status_text)

    def run(self):
        """Start monitoring with GUI"""
        if not self.connect():
            return

        print("Starting GUI monitor...")

        # Start serial reading thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.thread.start()

        # Initialize plot
        self.init_plot()

        # Start animation (update every 50ms for smoother display)
        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,  # 20 FPS (was 100ms = 10 FPS)
            blit=False,
            cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n✓ Monitor stopped by user")
        finally:
            self.running = False
            if self.ser and self.ser.is_open:
                self.ser.close()
                print(f"✓ Disconnected from {self.port}")

def main():
    """Main entry point"""
    port = '/dev/ttyUSB0'
    baudrate = 115200
    max_points = 100  # Show last 100 data points

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    if len(sys.argv) > 3:
        max_points = int(sys.argv[3])

    print("=" * 60)
    print("  LK-TECH Motor Real-time Monitor (GUI)")
    print("=" * 60)
    print(f"  Port: {port}")
    print(f"  Baudrate: {baudrate}")
    print(f"  Max points: {max_points}")
    print("=" * 60)
    print()

    monitor = MotorMonitorGUI(port, baudrate, max_points)
    monitor.run()

if __name__ == '__main__':
    main()
