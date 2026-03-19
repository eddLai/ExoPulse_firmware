#!/usr/bin/env python3
"""
MPU6050 IMU Real-time UI for Jetson Orin
Displays accelerometer, gyroscope, temperature, and tilt angles.
Auto-calibrates gyro on startup.

I2C: /dev/i2c-7, address 0x68
"""

import tkinter as tk
from tkinter import ttk
import smbus2
import struct
import time
import math
import threading

# MPU6050 registers
WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
TEMP_OUT_H = 0x41
SIGNAL_PATH_RESET = 0x68
USER_CTRL = 0x6A

I2C_BUS = 7
I2C_ADDR = 0x68

ACCEL_SCALE = 16384.0   # ±2g
GYRO_SCALE = 131.0      # ±250 deg/s


class MPU6050:
    def __init__(self, bus=I2C_BUS, addr=I2C_ADDR):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        self.gx_off = 0.0
        self.gy_off = 0.0
        self.gz_off = 0.0

    def init(self):
        # Device reset
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        # Wake up
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        # Signal path reset
        self.bus.write_byte_data(self.addr, SIGNAL_PATH_RESET, 0x07)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, USER_CTRL, 0x01)
        time.sleep(0.1)
        # Config: ±2g, ±250 deg/s
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        time.sleep(0.05)

        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        print(f"MPU6050 WHO_AM_I: 0x{who:02X}")
        return who in (0x68, 0x72)

    def calibrate(self, samples=200):
        """Calibrate gyro offset by averaging samples while stationary."""
        sx, sy, sz = 0.0, 0.0, 0.0
        for _ in range(samples):
            _, _, _, gx, gy, gz = self.read_raw()
            sx += gx
            sy += gy
            sz += gz
            time.sleep(0.005)
        self.gx_off = sx / samples
        self.gy_off = sy / samples
        self.gz_off = sz / samples
        print(f"Gyro offset: X={self.gx_off:.2f} Y={self.gy_off:.2f} Z={self.gz_off:.2f} deg/s")

    def read_raw(self):
        """Read raw accel + gyro, returns (ax, ay, az, gx, gy, gz) in g and deg/s."""
        data = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = struct.unpack('>h', bytes(data[0:2]))[0] / ACCEL_SCALE
        ay = struct.unpack('>h', bytes(data[2:4]))[0] / ACCEL_SCALE
        az = struct.unpack('>h', bytes(data[4:6]))[0] / ACCEL_SCALE
        gx = struct.unpack('>h', bytes(data[8:10]))[0] / GYRO_SCALE
        gy = struct.unpack('>h', bytes(data[10:12]))[0] / GYRO_SCALE
        gz = struct.unpack('>h', bytes(data[12:14]))[0] / GYRO_SCALE
        return ax, ay, az, gx, gy, gz

    def read_calibrated(self):
        """Read calibrated data (gyro offset subtracted)."""
        ax, ay, az, gx, gy, gz = self.read_raw()
        return ax, ay, az, gx - self.gx_off, gy - self.gy_off, gz - self.gz_off

    def read_temp(self):
        data = self.bus.read_i2c_block_data(self.addr, TEMP_OUT_H, 2)
        raw = struct.unpack('>h', bytes(data))[0]
        return raw / 340.0 + 36.53

    def close(self):
        self.bus.close()


class IMUDashboard:
    def __init__(self, root, imu):
        self.root = root
        self.imu = imu
        self.running = True

        root.title("MPU6050 IMU Dashboard")
        root.configure(bg='#1e1e1e')
        root.geometry("800x700")

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Consolas', 16, 'bold'),
                        foreground='#00ff88', background='#1e1e1e')
        style.configure('Header.TLabel', font=('Consolas', 12, 'bold'),
                        foreground='#ff9900', background='#1e1e1e')
        style.configure('Data.TLabel', font=('Consolas', 14),
                        foreground='#ffffff', background='#1e1e1e')
        style.configure('Value.TLabel', font=('Consolas', 18, 'bold'),
                        foreground='#00ccff', background='#2a2a2a')
        style.configure('Status.TLabel', font=('Consolas', 10),
                        foreground='#888888', background='#1e1e1e')
        style.configure('Dark.TFrame', background='#1e1e1e')
        style.configure('Card.TFrame', background='#2a2a2a')

        # Title
        ttk.Label(root, text="MPU6050 IMU Dashboard", style='Title.TLabel').pack(pady=(10, 5))
        ttk.Label(root, text=f"I2C: /dev/i2c-{I2C_BUS} @ 0x{I2C_ADDR:02X}", style='Status.TLabel').pack()

        main = ttk.Frame(root, style='Dark.TFrame')
        main.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)

        # --- Accelerometer ---
        self._make_section_header(main, "Accelerometer (g)")
        accel_frame = ttk.Frame(main, style='Card.TFrame')
        accel_frame.pack(fill=tk.X, pady=(0, 10), ipady=8)
        self.ax_var = tk.StringVar(value="0.000")
        self.ay_var = tk.StringVar(value="0.000")
        self.az_var = tk.StringVar(value="0.000")
        self._make_xyz_row(accel_frame, self.ax_var, self.ay_var, self.az_var)

        # Accel bars
        self.ax_bar = self._make_bar(main, '#ff4444')
        self.ay_bar = self._make_bar(main, '#44ff44')
        self.az_bar = self._make_bar(main, '#4488ff')

        # --- Gyroscope ---
        self._make_section_header(main, "Gyroscope (deg/s) [calibrated]")
        gyro_frame = ttk.Frame(main, style='Card.TFrame')
        gyro_frame.pack(fill=tk.X, pady=(0, 10), ipady=8)
        self.gx_var = tk.StringVar(value="0.000")
        self.gy_var = tk.StringVar(value="0.000")
        self.gz_var = tk.StringVar(value="0.000")
        self._make_xyz_row(gyro_frame, self.gx_var, self.gy_var, self.gz_var)

        # Gyro bars
        self.gx_bar = self._make_bar(main, '#ff4444')
        self.gy_bar = self._make_bar(main, '#44ff44')
        self.gz_bar = self._make_bar(main, '#4488ff')

        # --- Tilt Angles ---
        self._make_section_header(main, "Tilt Angles (deg)")
        tilt_frame = ttk.Frame(main, style='Card.TFrame')
        tilt_frame.pack(fill=tk.X, pady=(0, 10), ipady=8)
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        for col, (label, var) in enumerate([("Roll", self.roll_var), ("Pitch", self.pitch_var)]):
            ttk.Label(tilt_frame, text=label, style='Data.TLabel').grid(row=0, column=col*2, padx=(20, 5))
            ttk.Label(tilt_frame, textvariable=var, style='Value.TLabel', width=8).grid(row=0, column=col*2+1, padx=(0, 20))
        tilt_frame.columnconfigure(0, weight=1)
        tilt_frame.columnconfigure(1, weight=1)
        tilt_frame.columnconfigure(2, weight=1)
        tilt_frame.columnconfigure(3, weight=1)

        # --- Temperature ---
        self._make_section_header(main, "Temperature")
        temp_frame = ttk.Frame(main, style='Card.TFrame')
        temp_frame.pack(fill=tk.X, pady=(0, 10), ipady=8)
        self.temp_var = tk.StringVar(value="0.0 C")
        ttk.Label(temp_frame, textvariable=self.temp_var, style='Value.TLabel').pack()

        # --- Status bar ---
        self.status_var = tk.StringVar(value="Starting...")
        ttk.Label(root, textvariable=self.status_var, style='Status.TLabel').pack(pady=(0, 5))

        # Recalibrate button
        btn_frame = ttk.Frame(root, style='Dark.TFrame')
        btn_frame.pack(pady=(0, 10))
        tk.Button(btn_frame, text="Re-calibrate Gyro", command=self._recalibrate,
                  bg='#333', fg='#00ff88', font=('Consolas', 11), relief=tk.FLAT, padx=15, pady=5).pack()

        self.sample_count = 0
        self.start_time = time.time()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update()

    def _make_section_header(self, parent, text):
        ttk.Label(parent, text=text, style='Header.TLabel').pack(anchor=tk.W, pady=(8, 2))

    def _make_xyz_row(self, parent, xvar, yvar, zvar):
        for col, (label, var, color) in enumerate([("X", xvar, '#ff4444'), ("Y", yvar, '#44ff44'), ("Z", zvar, '#4488ff')]):
            lbl = ttk.Label(parent, text=label, style='Data.TLabel')
            lbl.configure(foreground=color)
            lbl.grid(row=0, column=col*2, padx=(20, 5))
            ttk.Label(parent, textvariable=var, style='Value.TLabel', width=8).grid(row=0, column=col*2+1, padx=(0, 15))
        for i in range(6):
            parent.columnconfigure(i, weight=1)

    def _make_bar(self, parent, color):
        canvas = tk.Canvas(parent, height=12, bg='#1e1e1e', highlightthickness=0)
        canvas.pack(fill=tk.X, pady=1)
        canvas.bar = canvas.create_rectangle(0, 0, 0, 12, fill=color, outline='')
        canvas.mid = canvas.create_line(0, 0, 0, 12, fill='#555', width=1)
        return canvas

    def _update_bar(self, canvas, value, max_val):
        w = canvas.winfo_width()
        mid = w // 2
        bar_w = int((value / max_val) * mid)
        bar_w = max(-mid, min(mid, bar_w))
        x1 = mid
        x2 = mid + bar_w
        if x1 > x2:
            x1, x2 = x2, x1
        canvas.coords(canvas.bar, x1, 0, x2, 12)
        canvas.coords(canvas.mid, mid, 0, mid, 12)

    def _recalibrate(self):
        self.status_var.set("Calibrating... keep sensor still!")
        self.root.update()
        self.imu.calibrate(200)
        self.status_var.set(f"Recalibrated! Offset: gx={self.imu.gx_off:.1f} gy={self.imu.gy_off:.1f} gz={self.imu.gz_off:.1f}")

    def _update(self):
        if not self.running:
            return
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_calibrated()
            temp = self.imu.read_temp()

            self.ax_var.set(f"{ax:+.3f}")
            self.ay_var.set(f"{ay:+.3f}")
            self.az_var.set(f"{az:+.3f}")
            self.gx_var.set(f"{gx:+.2f}")
            self.gy_var.set(f"{gy:+.2f}")
            self.gz_var.set(f"{gz:+.2f}")

            # Tilt angles from accelerometer
            roll = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))
            pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
            self.roll_var.set(f"{roll:+.1f}")
            self.pitch_var.set(f"{pitch:+.1f}")

            self.temp_var.set(f"{temp:.1f} C")

            # Update bars
            self._update_bar(self.ax_bar, ax, 2.0)
            self._update_bar(self.ay_bar, ay, 2.0)
            self._update_bar(self.az_bar, az, 2.0)
            self._update_bar(self.gx_bar, gx, 250.0)
            self._update_bar(self.gy_bar, gy, 250.0)
            self._update_bar(self.gz_bar, gz, 250.0)

            self.sample_count += 1
            elapsed = time.time() - self.start_time
            hz = self.sample_count / elapsed if elapsed > 0 else 0
            self.status_var.set(f"Samples: {self.sample_count} | {hz:.1f} Hz | Gyro offset: "
                                f"X={self.imu.gx_off:.1f} Y={self.imu.gy_off:.1f} Z={self.imu.gz_off:.1f}")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

        self.root.after(50, self._update)  # ~20 Hz refresh

    def _on_close(self):
        self.running = False
        self.imu.close()
        self.root.destroy()


def main():
    print("Initializing MPU6050...")
    imu = MPU6050()
    if not imu.init():
        print("Failed to initialize MPU6050!")
        return

    print("Calibrating gyro (keep sensor still)...")
    imu.calibrate(200)

    root = tk.Tk()
    app = IMUDashboard(root, imu)
    root.mainloop()


if __name__ == '__main__':
    main()
