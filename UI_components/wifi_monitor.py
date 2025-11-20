#!/usr/bin/env python3
"""
ExoPulse WiFi Dual Motor Monitor
Real-time motor monitoring system via TCP network

Features:
- WiFi TCP connection to ESP32
- Real-time dual motor chart display
- Remote calibration commands
- Automatic reconnection
- Cross-platform support (macOS, Linux, Windows)
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
from matplotlib.widgets import Button

class WiFiMotorMonitor:
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

        # Motor 1 data
        self.data_m1 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
            'acceleration': deque(maxlen=max_points),
            'angle': deque(maxlen=max_points),
        }

        # Motor 2 data
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

        # WiFi signal strength data
        self.data_rssi = {
            'time': deque(maxlen=max_points),
            'mcu_rssi': deque(maxlen=max_points),    # MCU to hotspot
            'pc_rssi': deque(maxlen=max_points),      # PC to hotspot
        }
        self.mcu_rssi = 0
        self.pc_rssi = 0

        self.start_time = time.time()

    def get_pc_wifi_rssi(self):
        """Get PC WiFi signal strength (Linux)"""
        try:
            import subprocess
            result = subprocess.run(['iwconfig'], capture_output=True, text=True, timeout=1)
            for line in result.stdout.split('\n'):
                if 'Signal level' in line:
                    match = re.search(r'Signal level[=:]([-\d]+)', line)
                    if match:
                        return int(match.group(1))
        except:
            pass
        return 0

    def connect(self):
        """Connect to ESP32 WiFi TCP server"""
        try:
            print(f"⚚ Connecting to {self.host}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            self.connection_status = "Connected"
            self.reconnect_attempts = 0

            # Get PC WiFi signal strength
            self.pc_rssi = self.get_pc_wifi_rssi()

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
            print(f"   Hint: Check ESP32 power and ensure WiFi is connected")
            print(f"   Verify IP address matches ESP32 serial output")
            return False

    def reconnect(self):
        """Attempt to reconnect"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            print(f"✗ Maximum reconnection attempts reached ({self.max_reconnect_attempts}). Giving up.")
            return False

        self.reconnect_attempts += 1
        self.connection_status = f"Reconnecting... ({self.reconnect_attempts}/{self.max_reconnect_attempts})"
        print(f"\n⚠ Connection lost! Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")

        # Close old connection
        try:
            if self.sock:
                self.sock.close()
        except:
            pass

        time.sleep(2)

        # Attempt reconnection
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            self.connection_status = "Connected (Reconnected)"
            self.reconnect_attempts = 0
            print(f"✓ Reconnection successful!")
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
            print(f"[ERR] Command send failed: {e}")
            return False

    def parse_line(self, line):
        """Parse motor status data line"""
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
        """Background thread: Read data from WiFi"""
        buffer = ""
        while self.running:
            try:
                with self.connection_lock:
                    if self.sock:
                        data = self.sock.recv(4096).decode('utf-8', errors='ignore')
                        if not data:
                            raise ConnectionResetError("Server closed connection")
                        buffer += data

                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # Skip command responses
                    if line.startswith('[OK]') or line.startswith('[ERR]') or line.startswith('[STATUS]'):
                        print(line)
                        continue

                    # Check for RSSI information
                    if 'Signal:' in line or 'RSSI:' in line:
                        match = re.search(r'(Signal|RSSI):\s*([-\d]+)', line)
                        if match:
                            self.mcu_rssi = int(match.group(2))

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

                            # Update RSSI data
                            self.pc_rssi = self.get_pc_wifi_rssi()
                            self.data_rssi['time'].append(elapsed)
                            self.data_rssi['mcu_rssi'].append(self.mcu_rssi)
                            self.data_rssi['pc_rssi'].append(self.pc_rssi)

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

    # Calibration commands
    def calibrate_motor1(self, event=None):
        self.send_command("CAL1")

    def calibrate_motor2(self, event=None):
        self.send_command("CAL2")

    def calibrate_both(self, event=None):
        self.send_command("CAL_BOTH")

    def clear_calibration(self, event=None):
        self.send_command("CLEAR_CAL")

    def request_status(self, event=None):
        self.send_command("STATUS")

    def update_plot(self, frame):
        """Update matplotlib plots"""
        # Update WiFi RSSI plot
        if len(self.data_rssi['time']) > 0:
            t_rssi = list(self.data_rssi['time'])

            self.line_mcu_rssi.set_data(t_rssi, list(self.data_rssi['mcu_rssi']))
            self.line_pc_rssi.set_data(t_rssi, list(self.data_rssi['pc_rssi']))

            self.ax_rssi.relim()
            self.ax_rssi.autoscale_view(scalex=True, scaley=False)  # Only autoscale X axis

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
        rssi_text = f'WiFi: MCU={self.mcu_rssi}dBm | PC={self.pc_rssi}dBm'
        self.status_text.set_text(f'Status: {self.connection_status} | {rssi_text}', color=status_color)

        m1_text = f'M1: T={self.status_m1["temp"]}°C | I={self.status_m1["current"]:.2f}A | S={self.status_m1["speed"]}dps | A={self.status_m1["angle"]:.2f}°'
        m2_text = f'M2: T={self.status_m2["temp"]}°C | I={self.status_m2["current"]:.2f}A | S={self.status_m2["speed"]}dps | A={self.status_m2["angle"]:.2f}°'
        self.data_text.set_text(f'{m1_text}\n{m2_text}')

        return (self.line_mcu_rssi, self.line_pc_rssi,
                self.line_m1_temp, self.line_m1_current, self.line_m1_speed,
                self.line_m1_accel, self.line_m1_angle,
                self.line_m2_temp, self.line_m2_current, self.line_m2_speed,
                self.line_m2_accel, self.line_m2_angle,
                self.status_text, self.data_text)

    def run(self):
        """Start GUI"""
        if not self.connect():
            print("Connection failed. Exiting...")
            return

        self.running = True

        # Start data reading thread
        read_thread = threading.Thread(target=self.read_data, daemon=True)
        read_thread.start()

        # Setup matplotlib plots
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('ExoPulse WiFi Dual Motor Monitor', fontsize=16, fontweight='bold')

        gs = gridspec.GridSpec(7, 2, figure=self.fig, hspace=0.4, wspace=0.3)

        # WiFi signal strength plot (top row spanning both columns)
        self.ax_rssi = self.fig.add_subplot(gs[0, :])

        # Motor 1 plots (left column)
        self.ax_m1_temp = self.fig.add_subplot(gs[1, 0])
        self.ax_m1_current = self.fig.add_subplot(gs[2, 0])
        self.ax_m1_speed = self.fig.add_subplot(gs[3, 0])
        self.ax_m1_accel = self.fig.add_subplot(gs[4, 0])
        self.ax_m1_angle = self.fig.add_subplot(gs[5, 0])

        # Motor 2 plots (right column)
        self.ax_m2_temp = self.fig.add_subplot(gs[1, 1])
        self.ax_m2_current = self.fig.add_subplot(gs[2, 1])
        self.ax_m2_speed = self.fig.add_subplot(gs[3, 1])
        self.ax_m2_accel = self.fig.add_subplot(gs[4, 1])
        self.ax_m2_angle = self.fig.add_subplot(gs[5, 1])

        # Initialize RSSI lines
        self.line_mcu_rssi, = self.ax_rssi.plot([], [], 'b-', label='MCU→Hotspot', linewidth=2)
        self.line_pc_rssi, = self.ax_rssi.plot([], [], 'g-', label='PC→Hotspot', linewidth=2)
        self.ax_rssi.set_title('WiFi Signal Strength (RSSI)', fontsize=11, fontweight='bold')
        self.ax_rssi.set_xlabel('Time (s)', fontsize=8)
        self.ax_rssi.set_ylabel('dBm', fontsize=8)
        self.ax_rssi.set_ylim(-100, -20)
        self.ax_rssi.grid(True, alpha=0.3)
        self.ax_rssi.legend(loc='upper right')

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
        for ax, title in [(self.ax_m1_temp, 'Temperature (C)'),
                          (self.ax_m1_current, 'Current (A)'),
                          (self.ax_m1_speed, 'Speed (dps)'),
                          (self.ax_m1_accel, 'Acceleration (dps^2)'),
                          (self.ax_m1_angle, 'Angle (deg)'),
                          (self.ax_m2_temp, 'Temperature (C)'),
                          (self.ax_m2_current, 'Current (A)'),
                          (self.ax_m2_speed, 'Speed (dps)'),
                          (self.ax_m2_accel, 'Acceleration (dps^2)'),
                          (self.ax_m2_angle, 'Angle (deg)')]:
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=8)
            ax.grid(True, alpha=0.3)

        # Status and control area
        ax_status = self.fig.add_subplot(gs[6, :])
        ax_status.axis('off')

        self.status_text = ax_status.text(0.02, 0.8, 'Status: Connecting...', fontsize=12, verticalalignment='top')
        self.data_text = ax_status.text(0.02, 0.5, '', fontsize=10, verticalalignment='top', family='monospace')

        # Add buttons
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
    print("ExoPulse WiFi Dual Motor Monitoring System")
    print("=" * 50)

    # Get ESP32 IP from command line or use default
    host = sys.argv[1] if len(sys.argv) > 1 else '192.168.43.123'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8888

    print(f"Target: {host}:{port}")
    print("Please ensure ESP32 is connected to WiFi 'ExoPulse'")
    print("=" * 50)

    monitor = WiFiMotorMonitor(host=host, port=port)
    monitor.run()
