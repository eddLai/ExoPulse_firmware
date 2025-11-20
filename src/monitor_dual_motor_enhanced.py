#!/usr/bin/env python3
"""
Enhanced Real-time Dual Motor Monitor with Angle Display
Shows: Temperature, Current, Speed, and ANGLE for each motor
"""

import os
import platform
import sys

# macOS 相容性設置 - 最小化修改
if platform.system() == 'Darwin':
    try:
        import matplotlib
        matplotlib.use('Qt5Agg')  # 使用穩定的 Qt5Agg 後端
    except ImportError:
        try:
            import matplotlib
            matplotlib.use('TkAgg')  # 備用 TkAgg 後端
        except ImportError:
            pass  # 使用預設後端

import serial
import re
import threading
import time
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec

class EnhancedDualMotorGUI:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, max_points=100):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False
        self.serial_lock = threading.Lock()  # Thread-safe serial access

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

        self.status_m1 = {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0}
        self.status_m2 = {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0}

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
        """Parse motor status line with acceleration"""
        if not line.startswith('['):
            return None
        try:
            match = re.match(r'\[(\d+)\] M:(\d+) T:(-?\d+) V:([\d.]+) I:([-\d.]+) S:(-?\d+) ACC:(-?\d+) E:(\d+) A:([-\d.]+|ovf) ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(9)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'motor_id': int(match.group(2)),
                    'temp': int(match.group(3)),
                    'voltage': float(match.group(4)),
                    'current': float(match.group(5)),
                    'speed': int(match.group(6)),
                    'acceleration': int(match.group(7)),
                    'angle': angle_val,
                }
        except:
            pass
        return None

    def read_serial_thread(self):
        """Serial reader thread with thread-safe access"""
        while self.running:
            try:
                with self.serial_lock:
                    if self.ser and self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    else:
                        line = None

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
                            self.data_m1['acceleration'].append(status['acceleration'])
                            self.data_m1['angle'].append(status['angle'])
                        elif mid == 2:
                            self.status_m2.update(status)
                            self.data_m2['time'].append(elapsed)
                            self.data_m2['temp'].append(status['temp'])
                            self.data_m2['current'].append(status['current'])
                            self.data_m2['speed'].append(status['speed'])
                            self.data_m2['acceleration'].append(status['acceleration'])
                            self.data_m2['angle'].append(status['angle'])
                else:
                    time.sleep(0.001)
            except:
                break

    def calibrate_motor(self, motor_id):
        """Calibrate motor zero position (software offset, no ROM write)"""
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                if motor_id == 1:
                    self.ser.write(b"CAL1\n")
                    self.ser.flush()
                    print("✓ Calibrating Motor 1 (software offset)...")
                elif motor_id == 2:
                    self.ser.write(b"CAL2\n")
                    self.ser.flush()
                    print("✓ Calibrating Motor 2 (software offset)...")
                elif motor_id == 0:  # Calibrate both
                    self.ser.write(b"CAL1\n")
                    self.ser.flush()
                    time.sleep(0.2)  # Small delay between commands
                    self.ser.write(b"CAL2\n")
                    self.ser.flush()
                    print("✓ Calibrating BOTH motors (software offset)...")

    def clear_calibration(self):
        """Clear all calibration offsets (restore original angles)"""
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.write(b"CLEAR_CAL\n")
                self.ser.flush()
                print("✓ Calibration cleared - angles restored to original values")

    def init_plot(self):
        """Initialize plots with 5 rows: Temp, Current, Speed, Acceleration, Angle"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.canvas.manager.set_window_title('Enhanced Dual Motor Monitor - Full Data Display')

        # 5 rows x 2 columns grid (add acceleration row)
        gs = gridspec.GridSpec(5, 2, hspace=0.4, wspace=0.3, top=0.94, bottom=0.08)

        # Motor 1 (left column)
        self.ax1_temp = self.fig.add_subplot(gs[0, 0])
        self.ax1_curr = self.fig.add_subplot(gs[1, 0])
        self.ax1_speed = self.fig.add_subplot(gs[2, 0])
        self.ax1_accel = self.fig.add_subplot(gs[3, 0])
        self.ax1_angle = self.fig.add_subplot(gs[4, 0])

        # Motor 2 (right column)
        self.ax2_temp = self.fig.add_subplot(gs[0, 1])
        self.ax2_curr = self.fig.add_subplot(gs[1, 1])
        self.ax2_speed = self.fig.add_subplot(gs[2, 1])
        self.ax2_accel = self.fig.add_subplot(gs[3, 1])
        self.ax2_angle = self.fig.add_subplot(gs[4, 1])

        # ===== Configure Motor 1 Plots =====
        self.ax1_temp.set_title('Motor 1 - Temperature', color='cyan', fontweight='bold')
        self.ax1_temp.set_ylim(0, 80)
        self.ax1_temp.set_ylabel('°C', color='cyan')
        self.ax1_temp.grid(True, alpha=0.3)

        self.ax1_curr.set_title('Motor 1 - Current', color='cyan', fontweight='bold')
        self.ax1_curr.set_ylim(-1, 1)
        self.ax1_curr.set_ylabel('A', color='cyan')
        self.ax1_curr.grid(True, alpha=0.3)

        self.ax1_speed.set_title('Motor 1 - Speed', color='cyan', fontweight='bold')
        self.ax1_speed.set_ylim(-0.35, 0.35)  # rad/s
        self.ax1_speed.set_ylabel('rad/s', color='cyan')
        self.ax1_speed.grid(True, alpha=0.3)
        # Add right y-axis for degrees/s
        self.ax1_speed_deg = self.ax1_speed.twinx()
        self.ax1_speed_deg.set_ylim(-20, 20)
        self.ax1_speed_deg.set_ylabel('°/s', color='yellow', fontsize=9)
        self.ax1_speed_deg.tick_params(axis='y', labelcolor='yellow')

        self.ax1_accel.set_title('Motor 1 - Acceleration', color='cyan', fontweight='bold')
        self.ax1_accel.set_ylabel('dps/s', color='cyan')
        self.ax1_accel.grid(True, alpha=0.3)

        self.ax1_angle.set_title('Motor 1 - Angle (Multi-turn)', color='cyan', fontweight='bold')
        self.ax1_angle.set_ylabel('degrees', color='cyan')
        self.ax1_angle.set_xlabel('Time (s)')
        self.ax1_angle.grid(True, alpha=0.3)

        # ===== Configure Motor 2 Plots =====
        self.ax2_temp.set_title('Motor 2 - Temperature', color='orange', fontweight='bold')
        self.ax2_temp.set_ylim(0, 80)
        self.ax2_temp.set_ylabel('°C', color='orange')
        self.ax2_temp.grid(True, alpha=0.3)

        self.ax2_curr.set_title('Motor 2 - Current', color='orange', fontweight='bold')
        self.ax2_curr.set_ylim(-1, 1)
        self.ax2_curr.set_ylabel('A', color='orange')
        self.ax2_curr.grid(True, alpha=0.3)

        self.ax2_speed.set_title('Motor 2 - Speed', color='orange', fontweight='bold')
        self.ax2_speed.set_ylim(-0.35, 0.35)  # rad/s
        self.ax2_speed.set_ylabel('rad/s', color='orange')
        self.ax2_speed.grid(True, alpha=0.3)
        # Add right y-axis for degrees/s
        self.ax2_speed_deg = self.ax2_speed.twinx()
        self.ax2_speed_deg.set_ylim(-20, 20)
        self.ax2_speed_deg.set_ylabel('°/s', color='yellow', fontsize=9)
        self.ax2_speed_deg.tick_params(axis='y', labelcolor='yellow')

        self.ax2_accel.set_title('Motor 2 - Acceleration', color='orange', fontweight='bold')
        self.ax2_accel.set_ylabel('dps/s', color='orange')
        self.ax2_accel.grid(True, alpha=0.3)

        self.ax2_angle.set_title('Motor 2 - Angle (Multi-turn)', color='orange', fontweight='bold')
        self.ax2_angle.set_ylabel('degrees', color='orange')
        self.ax2_angle.set_xlabel('Time (s)')
        self.ax2_angle.grid(True, alpha=0.3)

        # Create line objects
        self.line1_temp, = self.ax1_temp.plot([], [], 'c-', lw=2)
        self.line1_curr, = self.ax1_curr.plot([], [], 'c-', lw=2)
        self.line1_speed, = self.ax1_speed.plot([], [], 'c-', lw=2)
        self.line1_accel, = self.ax1_accel.plot([], [], 'c-', lw=2)
        self.line1_angle, = self.ax1_angle.plot([], [], 'c-', lw=2)

        self.line2_temp, = self.ax2_temp.plot([], [], 'orange', lw=2)
        self.line2_curr, = self.ax2_curr.plot([], [], 'orange', lw=2)
        self.line2_speed, = self.ax2_speed.plot([], [], 'orange', lw=2)
        self.line2_accel, = self.ax2_accel.plot([], [], 'orange', lw=2)
        self.line2_angle, = self.ax2_angle.plot([], [], 'orange', lw=2)

        # Add calibration buttons at top (4 buttons layout)
        from matplotlib.widgets import Button
        ax_cal_m1 = plt.axes([0.10, 0.96, 0.11, 0.03])
        ax_cal_m2 = plt.axes([0.79, 0.96, 0.11, 0.03])
        ax_cal_both = plt.axes([0.36, 0.96, 0.13, 0.03])
        ax_clear = plt.axes([0.53, 0.96, 0.13, 0.03])

        self.btn_cal_m1 = Button(ax_cal_m1, 'Cal Motor 1', color='cyan', hovercolor='lightblue')
        self.btn_cal_m2 = Button(ax_cal_m2, 'Cal Motor 2', color='orange', hovercolor='lightsalmon')
        self.btn_cal_both = Button(ax_cal_both, 'Cal Both', color='lime', hovercolor='lightgreen')
        self.btn_clear = Button(ax_clear, 'Clear Cal', color='red', hovercolor='salmon')

        self.btn_cal_m1.on_clicked(lambda event: self.calibrate_motor(1))
        self.btn_cal_m2.on_clicked(lambda event: self.calibrate_motor(2))
        self.btn_cal_both.on_clicked(lambda event: self.calibrate_motor(0))
        self.btn_clear.on_clicked(lambda event: self.clear_calibration())

        # Status text at bottom
        self.fig.text(0.5, 0.03, '', ha='center', fontsize=9, family='monospace', color='lime')

        return (self.line1_temp, self.line1_curr, self.line1_speed, self.line1_accel, self.line1_angle,
                self.line2_temp, self.line2_curr, self.line2_speed, self.line2_accel, self.line2_angle)

    def update(self, frame):
        """Update all plots"""
        window_size = 30  # Show last 30 seconds

        # Motor 1
        if len(self.data_m1['time']) > 1:
            t1 = list(self.data_m1['time'])
            speed1_dps = list(self.data_m1['speed'])
            speed1_rads = [s * 0.01745 for s in speed1_dps]  # deg/s to rad/s

            self.line1_temp.set_data(t1, list(self.data_m1['temp']))
            self.line1_curr.set_data(t1, list(self.data_m1['current']))
            self.line1_speed.set_data(t1, speed1_rads)
            self.line1_accel.set_data(t1, list(self.data_m1['acceleration']))
            self.line1_angle.set_data(t1, list(self.data_m1['angle']))

            # Update X-axis limits
            self.ax1_temp.set_xlim(max(0, t1[-1] - window_size), t1[-1] + 1)
            self.ax1_curr.set_xlim(max(0, t1[-1] - window_size), t1[-1] + 1)
            self.ax1_speed.set_xlim(max(0, t1[-1] - window_size), t1[-1] + 1)
            self.ax1_accel.set_xlim(max(0, t1[-1] - window_size), t1[-1] + 1)
            self.ax1_angle.set_xlim(max(0, t1[-1] - window_size), t1[-1] + 1)

            # Auto-scale Y-axis for acceleration
            if len(self.data_m1['acceleration']) > 0:
                accel_data = list(self.data_m1['acceleration'])
                if max(accel_data) - min(accel_data) > 0:
                    accel_min, accel_max = min(accel_data), max(accel_data)
                    margin = (accel_max - accel_min) * 0.2 + 10
                    self.ax1_accel.set_ylim(accel_min - margin, accel_max + margin)
                else:
                    self.ax1_accel.set_ylim(-100, 100)

            # Auto-scale Y-axis for angle (it can have large range)
            if len(self.data_m1['angle']) > 0:
                angle_data = list(self.data_m1['angle'])
                angle_min, angle_max = min(angle_data), max(angle_data)
                margin = (angle_max - angle_min) * 0.1 + 1
                self.ax1_angle.set_ylim(angle_min - margin, angle_max + margin)

        # Motor 2
        if len(self.data_m2['time']) > 1:
            t2 = list(self.data_m2['time'])
            speed2_dps = list(self.data_m2['speed'])
            speed2_rads = [s * 0.01745 for s in speed2_dps]

            self.line2_temp.set_data(t2, list(self.data_m2['temp']))
            self.line2_curr.set_data(t2, list(self.data_m2['current']))
            self.line2_speed.set_data(t2, speed2_rads)
            self.line2_accel.set_data(t2, list(self.data_m2['acceleration']))
            self.line2_angle.set_data(t2, list(self.data_m2['angle']))

            self.ax2_temp.set_xlim(max(0, t2[-1] - window_size), t2[-1] + 1)
            self.ax2_curr.set_xlim(max(0, t2[-1] - window_size), t2[-1] + 1)
            self.ax2_speed.set_xlim(max(0, t2[-1] - window_size), t2[-1] + 1)
            self.ax2_accel.set_xlim(max(0, t2[-1] - window_size), t2[-1] + 1)
            self.ax2_angle.set_xlim(max(0, t2[-1] - window_size), t2[-1] + 1)

            # Auto-scale Y-axis for acceleration
            if len(self.data_m2['acceleration']) > 0:
                accel_data = list(self.data_m2['acceleration'])
                if max(accel_data) - min(accel_data) > 0:
                    accel_min, accel_max = min(accel_data), max(accel_data)
                    margin = (accel_max - accel_min) * 0.2 + 10
                    self.ax2_accel.set_ylim(accel_min - margin, accel_max + margin)
                else:
                    self.ax2_accel.set_ylim(-100, 100)

            # Auto-scale Y-axis for angle
            if len(self.data_m2['angle']) > 0:
                angle_data = list(self.data_m2['angle'])
                angle_min, angle_max = min(angle_data), max(angle_data)
                margin = (angle_max - angle_min) * 0.1 + 1
                self.ax2_angle.set_ylim(angle_min - margin, angle_max + margin)

        # Update status text
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            s1 = self.status_m1
            s2 = self.status_m2
            status = (f"M1: {s1['temp']}°C | {s1['voltage']:.1f}V | {s1['current']:.2f}A | "
                     f"{s1['speed']}°/s | Acc:{s1['acceleration']}dps² | Angle:{s1['angle']:.1f}°  ||  "
                     f"M2: {s2['temp']}°C | {s2['voltage']:.1f}V | {s2['current']:.2f}A | "
                     f"{s2['speed']}°/s | Acc:{s2['acceleration']}dps² | Angle:{s2['angle']:.1f}°")
            self.fig.texts[0].set_text(status)

        return (self.line1_temp, self.line1_curr, self.line1_speed, self.line1_accel, self.line1_angle,
                self.line2_temp, self.line2_curr, self.line2_speed, self.line2_accel, self.line2_angle)

    def run(self):
        if not self.connect():
            return

        print("Starting Enhanced Monitor with Full Data Display...")
        print("Acceleration data is now available via CAN command 0x33!")

        # Auto-calibrate both motors on GUI launch (software offset)
        print("\nAuto-calibrating motor angles (software offset)...")
        time.sleep(0.5)  # Wait for firmware to be ready
        self.calibrate_motor(0)  # Calibrate both motors
        time.sleep(0.5)  # Wait for calibration to complete
        print("Motor angles calibrated to zero!\n")

        print("Click buttons at top:")
        print("  - Cal Motor 1/2: Calibrate individual motor")
        print("  - Cal Both: Calibrate both motors")
        print("  - Clear Cal: Restore original angles (no calibration)")

        self.running = True
        self.thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.thread.start()

        self.init_plot()

        self.anim = FuncAnimation(
            self.fig, self.update,
            interval=50,      # 20 FPS
            blit=True,
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

    print("=" * 70)
    print("  Enhanced Dual Motor Monitor - FULL DATA")
    print("  - 5 plots per motor: Temp, Current, Speed, Acceleration, Angle")
    print("  - Speed shown in rad/s (left) and °/s (right)")
    print("  - Acceleration in dps/s (degrees per second squared)")
    print("  - Multi-turn angle tracking with SOFTWARE CALIBRATION")
    print("  - Calibration: Safe, instant, no ROM write (resets on reboot)")
    print("  - Buttons: Cal M1/M2, Cal Both, Clear Cal")
    print("=" * 70)

    monitor = EnhancedDualMotorGUI(port, baudrate, max_points=100)
    monitor.run()

if __name__ == '__main__':
    main()
