#!/usr/bin/env python3
"""
Real-time Dual Motor Status Monitor - GUI Version with Live Plotting
Displays both Motor ID=1 and Motor ID=2 side by side
"""

import serial
import re
import sys
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

class DualMotorMonitorGUI:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False

        # Data storage for Motor 1 and Motor 2
        self.timestamps_m1 = deque(maxlen=max_points)
        self.timestamps_m2 = deque(maxlen=max_points)

        self.data_m1 = {
            'temperature': deque(maxlen=max_points),
            'voltage': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'encoder': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
            'error': deque(maxlen=max_points),
        }

        self.data_m2 = {
            'temperature': deque(maxlen=max_points),
            'voltage': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'encoder': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
            'error': deque(maxlen=max_points),
        }

        # Current status
        self.current_status_m1 = {'motor_id': 1}
        self.current_status_m2 = {'motor_id': 2}

        self.start_time = time.time()

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False

    def parse_motor_status(self, line):
        """Parse motor status from serial line"""
        # Fast format: [12345] M:1 T:30 V:24.5 I:0.03 S:0 E:4832 A:265.43 ERR:0x0
        if line.startswith('[') and '] M:' in line:
            try:
                match = re.search(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:(-?\d+\.?\d*) I:(-?\d+\.?\d*) S:(-?\d+) E:(\d+) A:(-?\d+\.?\d*) ERR:(0x[0-9A-Fa-f]+)', line)
                if match:
                    status = {
                        'timestamp': int(match.group(1)),
                        'motor_id': int(match.group(2)),
                        'temperature': int(match.group(3)),
                        'voltage': float(match.group(4)),
                        'current': float(match.group(5)),
                        'speed': int(match.group(6)),
                        'encoder': int(match.group(7)),
                        'angle': float(match.group(8)),
                        'error': int(match.group(9), 16),
                        'error_flags': []
                    }
                    return status
            except:
                pass

        return None

    def read_serial_thread(self):
        """Background thread to read serial data"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        status = self.parse_motor_status(line)

                        if status:
                            motor_id = status.get('motor_id', 1)
                            elapsed = time.time() - self.start_time

                            if motor_id == 1:
                                # Update Motor 1 data
                                self.current_status_m1.update(status)
                                self.timestamps_m1.append(elapsed)
                                self.data_m1['temperature'].append(status.get('temperature', None))
                                self.data_m1['voltage'].append(status.get('voltage', None))
                                self.data_m1['current'].append(status.get('current', None))
                                self.data_m1['speed'].append(status.get('speed', None))
                                self.data_m1['encoder'].append(status.get('encoder', None))
                                self.data_m1['angle'].append(status.get('angle', None))
                                self.data_m1['error'].append(status.get('error', 0))

                            elif motor_id == 2:
                                # Update Motor 2 data
                                self.current_status_m2.update(status)
                                self.timestamps_m2.append(elapsed)
                                self.data_m2['temperature'].append(status.get('temperature', None))
                                self.data_m2['voltage'].append(status.get('voltage', None))
                                self.data_m2['current'].append(status.get('current', None))
                                self.data_m2['speed'].append(status.get('speed', None))
                                self.data_m2['encoder'].append(status.get('encoder', None))
                                self.data_m2['angle'].append(status.get('angle', None))
                                self.data_m2['error'].append(status.get('error', 0))

                time.sleep(0.001)

            except Exception as e:
                print(f"Serial read error: {e}")
                break

    def init_plot(self):
        """Initialize the plot layout"""
        plt.style.use('dark_background')

        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('LK-TECH Dual Motor Monitor')

        # Create grid layout - 3 rows, 2 columns (Motor 1 left, Motor 2 right)
        gs = gridspec.GridSpec(4, 2, figure=self.fig, hspace=0.4, wspace=0.3)

        # Motor 1 plots (left column)
        self.ax_temp_m1 = self.fig.add_subplot(gs[0, 0])
        self.ax_current_m1 = self.fig.add_subplot(gs[1, 0])
        self.ax_speed_m1 = self.fig.add_subplot(gs[2, 0])

        # Motor 2 plots (right column)
        self.ax_temp_m2 = self.fig.add_subplot(gs[0, 1])
        self.ax_current_m2 = self.fig.add_subplot(gs[1, 1])
        self.ax_speed_m2 = self.fig.add_subplot(gs[2, 1])

        # Status display (bottom)
        self.ax_status = self.fig.add_subplot(gs[3, :])

        # Configure Motor 1 axes
        self.ax_temp_m1.set_title('Motor 1 - Temperature (°C)', fontsize=10, fontweight='bold', color='cyan')
        self.ax_temp_m1.set_ylabel('°C')
        self.ax_temp_m1.grid(True, alpha=0.3)

        self.ax_current_m1.set_title('Motor 1 - Current (A)', fontsize=10, fontweight='bold', color='cyan')
        self.ax_current_m1.set_ylabel('A')
        self.ax_current_m1.grid(True, alpha=0.3)

        self.ax_speed_m1.set_title('Motor 1 - Speed (dps)', fontsize=10, fontweight='bold', color='cyan')
        self.ax_speed_m1.set_ylabel('dps')
        self.ax_speed_m1.set_xlabel('Time (s)')
        self.ax_speed_m1.grid(True, alpha=0.3)

        # Configure Motor 2 axes
        self.ax_temp_m2.set_title('Motor 2 - Temperature (°C)', fontsize=10, fontweight='bold', color='orange')
        self.ax_temp_m2.set_ylabel('°C')
        self.ax_temp_m2.grid(True, alpha=0.3)

        self.ax_current_m2.set_title('Motor 2 - Current (A)', fontsize=10, fontweight='bold', color='orange')
        self.ax_current_m2.set_ylabel('A')
        self.ax_current_m2.grid(True, alpha=0.3)

        self.ax_speed_m2.set_title('Motor 2 - Speed (dps)', fontsize=10, fontweight='bold', color='orange')
        self.ax_speed_m2.set_ylabel('dps')
        self.ax_speed_m2.set_xlabel('Time (s)')
        self.ax_speed_m2.grid(True, alpha=0.3)

        # Status display
        self.ax_status.set_title('Current Status', fontsize=12, fontweight='bold')
        self.ax_status.axis('off')

        # Initialize line objects
        self.line_temp_m1, = self.ax_temp_m1.plot([], [], 'c-', linewidth=2, label='Motor 1')
        self.line_current_m1, = self.ax_current_m1.plot([], [], 'c-', linewidth=2, label='Motor 1')
        self.line_speed_m1, = self.ax_speed_m1.plot([], [], 'c-', linewidth=2, label='Motor 1')

        self.line_temp_m2, = self.ax_temp_m2.plot([], [], 'orange', linewidth=2, label='Motor 2')
        self.line_current_m2, = self.ax_current_m2.plot([], [], 'orange', linewidth=2, label='Motor 2')
        self.line_speed_m2, = self.ax_speed_m2.plot([], [], 'orange', linewidth=2, label='Motor 2')

        # Status text
        self.status_text = self.ax_status.text(
            0.5, 0.5, '',
            ha='center', va='center',
            fontsize=11,
            family='monospace',
            transform=self.ax_status.transAxes
        )

    def update_plot(self, frame):
        """Update plot data"""
        # Update Motor 1 plots
        if len(self.timestamps_m1) > 0:
            times_m1 = list(self.timestamps_m1)

            temp_data = [t if t is not None else 0 for t in self.data_m1['temperature']]
            if temp_data:
                self.line_temp_m1.set_data(times_m1, temp_data)
                self.ax_temp_m1.relim()
                self.ax_temp_m1.autoscale_view()

            current_data = [c if c is not None else 0 for c in self.data_m1['current']]
            if current_data:
                self.line_current_m1.set_data(times_m1, current_data)
                self.ax_current_m1.relim()
                self.ax_current_m1.autoscale_view()

            speed_data = [s if s is not None else 0 for s in self.data_m1['speed']]
            if speed_data:
                self.line_speed_m1.set_data(times_m1, speed_data)
                self.ax_speed_m1.relim()
                self.ax_speed_m1.autoscale_view()

        # Update Motor 2 plots
        if len(self.timestamps_m2) > 0:
            times_m2 = list(self.timestamps_m2)

            temp_data = [t if t is not None else 0 for t in self.data_m2['temperature']]
            if temp_data:
                self.line_temp_m2.set_data(times_m2, temp_data)
                self.ax_temp_m2.relim()
                self.ax_temp_m2.autoscale_view()

            current_data = [c if c is not None else 0 for c in self.data_m2['current']]
            if current_data:
                self.line_current_m2.set_data(times_m2, current_data)
                self.ax_current_m2.relim()
                self.ax_current_m2.autoscale_view()

            speed_data = [s if s is not None else 0 for s in self.data_m2['speed']]
            if speed_data:
                self.line_speed_m2.set_data(times_m2, speed_data)
                self.ax_speed_m2.relim()
                self.ax_speed_m2.autoscale_view()

        # Update status text
        s1 = self.current_status_m1
        s2 = self.current_status_m2

        status_str = (
            f"╔══════════════════════════════════ MOTOR 1 ══════════════════════════════════╗  ╔══════════════════════════════════ MOTOR 2 ══════════════════════════════════╗\n"
            f"  Temp: {s1.get('temperature', 'N/A'):>3}°C  Voltage: {s1.get('voltage', 0):.1f}V  "
            f"Current: {s1.get('current', 0):>6.2f}A  Speed: {s1.get('speed', 0):>5}dps  "
            f"Angle: {s1.get('angle', 0):>8.2f}°    "
            f"Temp: {s2.get('temperature', 'N/A'):>3}°C  Voltage: {s2.get('voltage', 0):.1f}V  "
            f"Current: {s2.get('current', 0):>6.2f}A  Speed: {s2.get('speed', 0):>5}dps  "
            f"Angle: {s2.get('angle', 0):>8.2f}°"
        )

        self.status_text.set_text(status_str)
        self.status_text.set_color('lime')

    def run(self):
        """Start monitoring with GUI"""
        if not self.connect():
            return

        print("Starting Dual Motor GUI monitor...")

        # Start serial reading thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.thread.start()

        # Initialize plot
        self.init_plot()

        # Start animation (50ms = 20 FPS)
        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,
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
    max_points = 100

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    if len(sys.argv) > 3:
        max_points = int(sys.argv[3])

    print("=" * 80)
    print("  LK-TECH Dual Motor Real-time Monitor (GUI)")
    print("=" * 80)
    print(f"  Port: {port}")
    print(f"  Baudrate: {baudrate}")
    print(f"  Max points: {max_points}")
    print("=" * 80)
    print()

    monitor = DualMotorMonitorGUI(port, baudrate, max_points)
    monitor.run()

if __name__ == '__main__':
    main()
