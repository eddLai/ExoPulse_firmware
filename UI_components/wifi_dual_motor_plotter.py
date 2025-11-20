#!/usr/bin/env python3
"""
ExoPulse WiFi Dual Motor Monitor
Real-time monitoring via WiFi TCP connection instead of Serial

Features:
- WiFi TCP client connection to ESP32
- Real-time plotting for dual motors
- Remote calibration commands
- Auto-reconnect on connection loss
- Compatible with macOS, Linux, Windows
"""

import os
import platform
import sys
import socket
import threading
import time
import re
from collections import deque
import numpy as np

# macOS compatibility
if platform.system() == 'Darwin':
    try:
        import matplotlib
        matplotlib.use('Qt5Agg')
    except ImportError:
        try:
            import matplotlib
            matplotlib.use('TkAgg')
        except ImportError:
            pass

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

class WiFiDualMotorGUI:
    def __init__(self, host='192.168.43.123', port=8888, max_points=100):
        self.host = host
        self.port = port
        self.max_points = max_points
        self.sock = None
        self.running = False
        self.connection_lock = threading.Lock()
        self.connection_status = "Disconnected"
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5

        # Data storage for Motor 1
        self.data_m1 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        # Data storage for Motor 2
        self.data_m2 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        self.status_m1 = {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0,
                         'speed': 0, 'acceleration': 0, 'angle': 0}
        self.status_m2 = {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0,
                         'speed': 0, 'acceleration': 0, 'angle': 0}

        self.start_time = time.time()
        self.frame_count = 0

    def connect(self):
        """Connect to ESP32 WiFi TCP server"""
        try:
            print(f"⚙ Connecting to {self.host}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)  # Set timeout for recv
            self.connection_status = "Connected"
            self.reconnect_attempts = 0

            # Read welcome message
            try:
                welcome = self.sock.recv(1024).decode('utf-8', errors='ignore')
                print(f"✓ {welcome.strip()}")
            except:
                pass

            print(f"✓ Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            self.connection_status = f"Error: {e}"
            print(f"✗ Connection failed: {e}")
            print(f"   Tip: Check ESP32 is powered on and WiFi is connected")
            print(f"   Check IP address matches ESP32's Serial output")
            return False

    def reconnect(self):
        """Attempt to reconnect to ESP32"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            print(f"✗ Max reconnection attempts ({self.max_reconnect_attempts}) reached. Giving up.")
            return False

        self.reconnect_attempts += 1
        self.connection_status = f"Reconnecting... (attempt {self.reconnect_attempts}/{self.max_reconnect_attempts})"
        print(f"\n⚠ Connection lost! Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")

        # Close old connection
        try:
            if self.sock:
                self.sock.close()
        except:
            pass

        time.sleep(2)

        # Try to reconnect
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            self.connection_status = "Connected (Reconnected)"
            self.reconnect_attempts = 0
            print(f"✓ Reconnected successfully!")
            return True
        except Exception as e:
            self.connection_status = f"Reconnect failed: {e}"
            print(f"✗ Reconnection failed: {e}")
            return False

    def send_command(self, command):
        """Send command to ESP32 via WiFi"""
        try:
            with self.connection_lock:
                if self.sock:
                    self.sock.sendall((command + '\n').encode('utf-8'))
                    print(f"[CMD] Sent: {command}")
                    return True
        except Exception as e:
            print(f"[ERR] Failed to send command: {e}")
            return False

    def parse_line(self, line):
        """Parse motor status line"""
        if not line.startswith('['):
            return None
        try:
            match = re.match(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(9)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'timestamp': int(match.group(1)),
                    'motor_id': int(match.group(2)),
                    'temp': int(match.group(3)),
                    'voltage': float(match.group(4)),
                    'current': float(match.group(5)),
                    'speed': int(match.group(6)),
                    'acceleration': int(match.group(7)),
                    'encoder': int(match.group(8)),
                    'angle': angle_val,
                    'error': match.group(10)
                }
        except Exception as e:
            print(f"Parse error: {e} | Line: {line}")
        return None

    def read_data(self):
        """Background thread to read data from WiFi"""
        buffer = ""
        while self.running:
            try:
                with self.connection_lock:
                    if self.sock:
                        data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                        if not data:
                            raise ConnectionResetError("Connection closed by server")
                        buffer += data

                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # Skip command responses
                    if line.startswith('[CMD]') or line.startswith('[ERR]') or line.startswith('[STATUS]'):
                        print(line)
                        continue

                    # Parse motor data
                    data = self.parse_line(line)
                    if data:
                        elapsed = time.time() - self.start_time
                        motor_id = data['motor_id']

                        if motor_id == 1:
                            self.status_m1.update(data)
                            self.data_m1['time'].append(elapsed)
                            self.data_m1['temp'].append(data['temp'])
                            self.data_m1['current'].append(data['current'])
                            self.data_m1['speed'].append(data['speed'])
                            self.data_m1['acceleration'].append(data['acceleration'])
                            self.data_m1['angle'].append(data['angle'])

                        elif motor_id == 2:
                            self.status_m2.update(data)
                            self.data_m2['time'].append(elapsed)
                            self.data_m2['temp'].append(data['temp'])
                            self.data_m2['current'].append(data['current'])
                            self.data_m2['speed'].append(data['speed'])
                            self.data_m2['acceleration'].append(data['acceleration'])
                            self.data_m2['angle'].append(data['angle'])

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[ERR] Read error: {e}")
                if self.running and not self.reconnect():
                    break
                time.sleep(1)

    def calibrate_motor1(self, event=None):
        """Calibrate Motor 1"""
        self.send_command("CAL1")

    def calibrate_motor2(self, event=None):
        """Calibrate Motor 2"""
        self.send_command("CAL2")

    def calibrate_both(self, event=None):
        """Calibrate both motors"""
        self.send_command("CAL_BOTH")

    def clear_calibration(self, event=None):
        """Clear calibration"""
        self.send_command("CLEAR_CAL")

    def request_status(self, event=None):
        """Request system status"""
        self.send_command("STATUS")

    def update_plot(self, frame):
        """Update matplotlib plot"""
        self.frame_count += 1

        # Update Motor 1 plots
        if len(self.data_m1['time']) > 0:
            t_m1 = list(self.data_m1['time'])

            self.line_m1_temp.set_data(t_m1, list(self.data_m1['temp']))
            self.ax_m1_temp.relim()
            self.ax_m1_temp.autoscale_view()

            self.line_m1_current.set_data(t_m1, list(self.data_m1['current']))
            self.ax_m1_current.relim()
            self.ax_m1_current.autoscale_view()

            self.line_m1_speed.set_data(t_m1, list(self.data_m1['speed']))
            self.ax_m1_speed.relim()
            self.ax_m1_speed.autoscale_view()

            self.line_m1_accel.set_data(t_m1, list(self.data_m1['acceleration']))
            self.ax_m1_accel.relim()
            self.ax_m1_accel.autoscale_view()

            self.line_m1_angle.set_data(t_m1, list(self.data_m1['angle']))
            self.ax_m1_angle.relim()
            self.ax_m1_angle.autoscale_view()

        # Update Motor 2 plots
        if len(self.data_m2['time']) > 0:
            t_m2 = list(self.data_m2['time'])

            self.line_m2_temp.set_data(t_m2, list(self.data_m2['temp']))
            self.ax_m2_temp.relim()
            self.ax_m2_temp.autoscale_view()

            self.line_m2_current.set_data(t_m2, list(self.data_m2['current']))
            self.ax_m2_current.relim()
            self.ax_m2_current.autoscale_view()

            self.line_m2_speed.set_data(t_m2, list(self.data_m2['speed']))
            self.ax_m2_speed.relim()
            self.ax_m2_speed.autoscale_view()

            self.line_m2_accel.set_data(t_m2, list(self.data_m2['acceleration']))
            self.ax_m2_accel.relim()
            self.ax_m2_accel.autoscale_view()

            self.line_m2_angle.set_data(t_m2, list(self.data_m2['angle']))
            self.ax_m2_angle.relim()
            self.ax_m2_angle.autoscale_view()

        # Update status text
        status_color = 'green' if 'Connected' in self.connection_status else 'red'
        self.status_text.set_text(f'Status: {self.connection_status}', color=status_color)

        m1_text = f'M1: T={self.status_m1["temp"]}°C | I={self.status_m1["current"]:.2f}A | S={self.status_m1["speed"]}dps | A={self.status_m1["angle"]:.2f}°'
        m2_text = f'M2: T={self.status_m2["temp"]}°C | I={self.status_m2["current"]:.2f}A | S={self.status_m2["speed"]}dps | A={self.status_m2["angle"]:.2f}°'
        self.data_text.set_text(f'{m1_text}\n{m2_text}')

        return (self.line_m1_temp, self.line_m1_current, self.line_m1_speed,
                self.line_m1_accel, self.line_m1_angle,
                self.line_m2_temp, self.line_m2_current, self.line_m2_speed,
                self.line_m2_accel, self.line_m2_angle,
                self.status_text, self.data_text)

    def run(self):
        """Start GUI"""
        if not self.connect():
            print("Failed to connect. Exiting...")
            return

        self.running = True

        # Start data reading thread
        read_thread = threading.Thread(target=self.read_data, daemon=True)
        read_thread.start()

        # Setup matplotlib figure
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('ExoPulse WiFi Dual Motor Monitor', fontsize=16, fontweight='bold')

        gs = gridspec.GridSpec(6, 2, figure=self.fig, hspace=0.4, wspace=0.3)

        # Motor 1 plots (left column)
        self.ax_m1_temp = self.fig.add_subplot(gs[0, 0])
        self.ax_m1_current = self.fig.add_subplot(gs[1, 0])
        self.ax_m1_speed = self.fig.add_subplot(gs[2, 0])
        self.ax_m1_accel = self.fig.add_subplot(gs[3, 0])
        self.ax_m1_angle = self.fig.add_subplot(gs[4, 0])

        # Motor 2 plots (right column)
        self.ax_m2_temp = self.fig.add_subplot(gs[0, 1])
        self.ax_m2_current = self.fig.add_subplot(gs[1, 1])
        self.ax_m2_speed = self.fig.add_subplot(gs[2, 1])
        self.ax_m2_accel = self.fig.add_subplot(gs[3, 1])
        self.ax_m2_angle = self.fig.add_subplot(gs[4, 1])

        # Initialize lines
        self.line_m1_temp, = self.ax_m1_temp.plot([], [], 'r-', label='Motor 1')
        self.line_m1_current, = self.ax_m1_current.plot([], [], 'b-')
        self.line_m1_speed, = self.ax_m1_speed.plot([], [], 'g-')
        self.line_m1_accel, = self.ax_m1_accel.plot([], [], 'm-')
        self.line_m1_angle, = self.ax_m1_angle.plot([], [], 'c-')

        self.line_m2_temp, = self.ax_m2_temp.plot([], [], 'r-', label='Motor 2')
        self.line_m2_current, = self.ax_m2_current.plot([], [], 'b-')
        self.line_m2_speed, = self.ax_m2_speed.plot([], [], 'g-')
        self.line_m2_accel, = self.ax_m2_accel.plot([], [], 'm-')
        self.line_m2_angle, = self.ax_m2_angle.plot([], [], 'c-')

        # Set labels
        for ax, title in [(self.ax_m1_temp, 'Temperature (°C)'),
                          (self.ax_m1_current, 'Current (A)'),
                          (self.ax_m1_speed, 'Speed (dps)'),
                          (self.ax_m1_accel, 'Acceleration (dps²)'),
                          (self.ax_m1_angle, 'Angle (°)'),
                          (self.ax_m2_temp, 'Temperature (°C)'),
                          (self.ax_m2_current, 'Current (A)'),
                          (self.ax_m2_speed, 'Speed (dps)'),
                          (self.ax_m2_accel, 'Acceleration (dps²)'),
                          (self.ax_m2_angle, 'Angle (°)')]:
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=8)
            ax.grid(True, alpha=0.3)

        # Status and control area
        ax_status = self.fig.add_subplot(gs[5, :])
        ax_status.axis('off')

        self.status_text = ax_status.text(0.02, 0.8, 'Status: Connecting...', fontsize=12, verticalalignment='top')
        self.data_text = ax_status.text(0.02, 0.5, '', fontsize=10, verticalalignment='top', family='monospace')

        # Add buttons
        from matplotlib.widgets import Button
        ax_cal1 = plt.axes([0.1, 0.02, 0.1, 0.03])
        ax_cal2 = plt.axes([0.22, 0.02, 0.1, 0.03])
        ax_both = plt.axes([0.34, 0.02, 0.12, 0.03])
        ax_clear = plt.axes([0.48, 0.02, 0.12, 0.03])
        ax_status_btn = plt.axes([0.62, 0.02, 0.1, 0.03])

        btn_cal1 = Button(ax_cal1, 'Cal M1')
        btn_cal2 = Button(ax_cal2, 'Cal M2')
        btn_both = Button(ax_both, 'Cal Both')
        btn_clear = Button(ax_clear, 'Clear Cal')
        btn_status = Button(ax_status_btn, 'Status')

        btn_cal1.on_clicked(self.calibrate_motor1)
        btn_cal2.on_clicked(self.calibrate_motor2)
        btn_both.on_clicked(self.calibrate_both)
        btn_clear.on_clicked(self.clear_calibration)
        btn_status.on_clicked(self.request_status)

        # Animation
        ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=True)

        plt.show()

        self.running = False
        if self.sock:
            self.sock.close()

if __name__ == '__main__':
    print("ExoPulse WiFi Dual Motor Monitor")
    print("=" * 50)

    # Get ESP32 IP from command line or use default
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.43.123'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8888

    print(f"Target: {host}:{port}")
    print("Make sure ESP32 is connected to WiFi 'ExoPulse'")
    print("=" * 50)

    gui = WiFiDualMotorGUI(host=host, port=port)
    gui.run()
