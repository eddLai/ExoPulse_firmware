#!/usr/bin/env python3
"""
OPTIMIZED Real-time Dual Motor Monitor - Minimal lag version
Uses blitting and fixed axes for maximum performance
"""

import serial
import re
import sys
import threading
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

class OptimizedDualMotorGUI:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, max_points=50):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points  # Reduced for better performance
        self.ser = None
        self.running = False

        # Use numpy arrays for better performance
        self.data_m1 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
        }

        self.data_m2 = {
            'time': deque(maxlen=max_points),
            'temp': deque(maxlen=max_points),
            'current': deque(maxlen=max_points),
            'speed': deque(maxlen=max_points),
        }

        self.status_m1 = {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'angle': 0}
        self.status_m2 = {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'angle': 0}

        self.start_time = time.time()
        self.frame_count = 0

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connected to {self.port}")
            time.sleep(1)
            return True
        except Exception as e:
            print(f"✗ Failed: {e}")
            return False

    def parse_line(self, line):
        """Fast regex parser"""
        if not line.startswith('['):
            return None
        try:
            match = re.match(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(8)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'motor_id': int(match.group(2)),
                    'temp': int(match.group(3)),
                    'voltage': float(match.group(4)),
                    'current': float(match.group(5)),
                    'speed': int(match.group(6)),
                    'angle': angle_val,
                }
        except:
            pass
        return None

    def read_serial_thread(self):
        """Optimized serial reader - non-blocking"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        status = self.parse_line(line)
                        if status:
                            elapsed = time.time() - self.start_time
                            mid = status['motor_id']

                            if mid == 1:
                                self.status_m1.update(status)
                                self.data_m1['time'].append(elapsed)
                                self.data_m1['temp'].append(status['temp'])
                                self.data_m1['current'].append(status['current'])
                                self.data_m1['speed'].append(status['speed'])
                            elif mid == 2:
                                self.status_m2.update(status)
                                self.data_m2['time'].append(elapsed)
                                self.data_m2['temp'].append(status['temp'])
                                self.data_m2['current'].append(status['current'])
                                self.data_m2['speed'].append(status['speed'])
                else:
                    time.sleep(0.001)
            except:
                break

    def init_plot(self):
        """Initialize with FIXED axes ranges for speed"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title('Dual Motor Monitor [OPTIMIZED]')

        gs = gridspec.GridSpec(3, 2, hspace=0.35, wspace=0.25)

        # Motor 1 (left)
        self.ax1_temp = self.fig.add_subplot(gs[0, 0])
        self.ax1_curr = self.fig.add_subplot(gs[1, 0])
        self.ax1_speed = self.fig.add_subplot(gs[2, 0])

        # Motor 2 (right)
        self.ax2_temp = self.fig.add_subplot(gs[0, 1])
        self.ax2_curr = self.fig.add_subplot(gs[1, 1])
        self.ax2_speed = self.fig.add_subplot(gs[2, 1])

        # Configure M1
        self.ax1_temp.set_title('Motor 1 - Temp', color='cyan', fontweight='bold')
        self.ax1_temp.set_ylim(0, 80)
        self.ax1_temp.set_ylabel('°C')
        self.ax1_temp.grid(True, alpha=0.3)

        self.ax1_curr.set_title('Motor 1 - Current', color='cyan', fontweight='bold')
        self.ax1_curr.set_ylim(-1, 1)
        self.ax1_curr.set_ylabel('A')
        self.ax1_curr.grid(True, alpha=0.3)

        self.ax1_speed.set_title('Motor 1 - Speed', color='cyan', fontweight='bold')
        self.ax1_speed.set_ylim(-0.35, 0.35)  # rad/s: 20 deg/s ≈ 0.35 rad/s
        self.ax1_speed.set_ylabel('rad/s', color='cyan')
        self.ax1_speed.set_xlabel('Time (s)')
        self.ax1_speed.grid(True, alpha=0.3)

        # Add right y-axis for degrees/s
        self.ax1_speed_deg = self.ax1_speed.twinx()
        self.ax1_speed_deg.set_ylim(-20, 20)
        self.ax1_speed_deg.set_ylabel('°/s', color='yellow', fontsize=9)
        self.ax1_speed_deg.tick_params(axis='y', labelcolor='yellow')

        # Configure M2
        self.ax2_temp.set_title('Motor 2 - Temp', color='orange', fontweight='bold')
        self.ax2_temp.set_ylim(0, 80)
        self.ax2_temp.set_ylabel('°C')
        self.ax2_temp.grid(True, alpha=0.3)

        self.ax2_curr.set_title('Motor 2 - Current', color='orange', fontweight='bold')
        self.ax2_curr.set_ylim(-1, 1)
        self.ax2_curr.set_ylabel('A')
        self.ax2_curr.grid(True, alpha=0.3)

        self.ax2_speed.set_title('Motor 2 - Speed', color='orange', fontweight='bold')
        self.ax2_speed.set_ylim(-0.35, 0.35)  # rad/s: 20 deg/s ≈ 0.35 rad/s
        self.ax2_speed.set_ylabel('rad/s', color='orange')
        self.ax2_speed.set_xlabel('Time (s)')
        self.ax2_speed.grid(True, alpha=0.3)

        # Add right y-axis for degrees/s
        self.ax2_speed_deg = self.ax2_speed.twinx()
        self.ax2_speed_deg.set_ylim(-20, 20)
        self.ax2_speed_deg.set_ylabel('°/s', color='yellow', fontsize=9)
        self.ax2_speed_deg.tick_params(axis='y', labelcolor='yellow')

        # Create lines
        self.line1_temp, = self.ax1_temp.plot([], [], 'c-', lw=2)
        self.line1_curr, = self.ax1_curr.plot([], [], 'c-', lw=2)
        self.line1_speed, = self.ax1_speed.plot([], [], 'c-', lw=2)

        self.line2_temp, = self.ax2_temp.plot([], [], 'orange', lw=2)
        self.line2_curr, = self.ax2_curr.plot([], [], 'orange', lw=2)
        self.line2_speed, = self.ax2_speed.plot([], [], 'orange', lw=2)

        # Status text
        self.fig.text(0.5, 0.02, '', ha='center', fontsize=9, family='monospace', color='lime')

        return (self.line1_temp, self.line1_curr, self.line1_speed,
                self.line2_temp, self.line2_curr, self.line2_speed)

    def update(self, frame):
        """Ultra-fast update with blitting"""
        # Motor 1
        if len(self.data_m1['time']) > 1:
            t1 = list(self.data_m1['time'])
            speed1_dps = list(self.data_m1['speed'])
            speed1_rads = [s * 3.14159 / 180.0 for s in speed1_dps]  # Convert deg/s to rad/s

            self.line1_temp.set_data(t1, list(self.data_m1['temp']))
            self.line1_curr.set_data(t1, list(self.data_m1['current']))
            self.line1_speed.set_data(t1, speed1_rads)

            # Only update X-axis limits
            self.ax1_temp.set_xlim(max(0, t1[-1] - 30), t1[-1] + 1)
            self.ax1_curr.set_xlim(max(0, t1[-1] - 30), t1[-1] + 1)
            self.ax1_speed.set_xlim(max(0, t1[-1] - 30), t1[-1] + 1)

        # Motor 2
        if len(self.data_m2['time']) > 1:
            t2 = list(self.data_m2['time'])
            speed2_dps = list(self.data_m2['speed'])
            speed2_rads = [s * 3.14159 / 180.0 for s in speed2_dps]  # Convert deg/s to rad/s

            self.line2_temp.set_data(t2, list(self.data_m2['temp']))
            self.line2_curr.set_data(t2, list(self.data_m2['current']))
            self.line2_speed.set_data(t2, speed2_rads)

            self.ax2_temp.set_xlim(max(0, t2[-1] - 30), t2[-1] + 1)
            self.ax2_curr.set_xlim(max(0, t2[-1] - 30), t2[-1] + 1)
            self.ax2_speed.set_xlim(max(0, t2[-1] - 30), t2[-1] + 1)

        # Update status display every 20 frames
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            s1 = self.status_m1
            s2 = self.status_m2
            # Enhanced status with voltage, speed in both units, and angle
            status = (f"M1: {s1['temp']}°C | {s1['voltage']:.1f}V | {s1['current']:.2f}A | "
                     f"{s1['speed']}°/s ({s1['speed']*0.01745:.2f}rad/s) | {s1['angle']:.1f}°  ||  "
                     f"M2: {s2['temp']}°C | {s2['voltage']:.1f}V | {s2['current']:.2f}A | "
                     f"{s2['speed']}°/s ({s2['speed']*0.01745:.2f}rad/s) | {s2['angle']:.1f}°")
            self.fig.texts[0].set_text(status)

        return (self.line1_temp, self.line1_curr, self.line1_speed,
                self.line2_temp, self.line2_curr, self.line2_speed)

    def run(self):
        if not self.connect():
            return

        print("Starting OPTIMIZED monitor (50 points, fixed axes)...")

        self.running = True
        self.thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.thread.start()

        self.init_plot()

        # CRITICAL: blit=True for performance!
        self.anim = FuncAnimation(
            self.fig, self.update,
            interval=50,      # 20 FPS
            blit=True,        # ENABLED for speed
            cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n✓ Stopped")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("=" * 60)
    print("  OPTIMIZED Dual Motor Monitor")
    print("  - Fixed Y-axes (no autoscale lag)")
    print("  - Blitting enabled (fast redraw)")
    print("  - 50 data points (reduced memory)")
    print("=" * 60)

    monitor = OptimizedDualMotorGUI(port, baudrate, max_points=50)
    monitor.run()

if __name__ == '__main__':
    main()
