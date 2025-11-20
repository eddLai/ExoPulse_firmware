#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Enhanced Motor Controller GUI with Integrated Monitoring

Features:
- Real-time motor data visualization (angle, speed, current, temp, acceleration)
- WiFi/UART communication modes
- CAN / Lower Chip motor control modes
- IMU 3D attitude display (optional)
- Configurable data display with sidebar checkboxes
- Motor calibration controls
- Data recording capability
"""

import socket
import threading
import queue
import time
import sys
import re
from datetime import datetime
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QRadioButton, QComboBox,
    QTextEdit, QGroupBox, QMessageBox, QButtonGroup, QCheckBox,
    QSplitter, QScrollArea
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.gridspec as gridspec
import numpy as np

# Optional pyserial
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

socket.setdefaulttimeout(2)

##############################################################################
# Protocol Configuration
##############################################################################
ESP32_IP = "10.154.48.200"
ESP32_PORT = 8080
UDP_PORT = 5000
HEADER = bytes([0xAA, 0x55])
BUF_SIZE = 1024

##############################################################################
# Protocol Helpers
##############################################################################
def crc8(data: bytes) -> int:
    """Calculate CRC-8 checksum (XOR)"""
    c = 0
    for b in data:
        c ^= b
    return c

def odc_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    """Create ODC protocol packet"""
    body = HEADER + bytes([cmd_id, len(payload)]) + payload
    return body + bytes([crc8(body)])

def make_motor_packet(mode1: str, val1: int, mode2: str, val2: int):
    """Create motor control packet"""
    cmd = f"X {mode1} {val1} {mode2} {val2}\r\n"
    return odc_packet(0x01, cmd.encode()), cmd.strip()

def make_led_packet():
    """Create LED toggle packet"""
    return odc_packet(0x10)

##############################################################################
# IMU 3D Window
##############################################################################
class IMU3DWindow(QWidget):
    """Separate window for 3D IMU attitude visualization"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Attitude (Roll-Pitch-Yaw)")

        layout = QVBoxLayout()

        # Create 3D matplotlib figure
        fig = plt.Figure(figsize=(4, 4), dpi=100)
        self.ax = fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.axis("off")

        # Aircraft body and wings
        self.air_body, = self.ax.plot([], [], [], "k-", lw=2, label="Body")
        self.air_wing, = self.ax.plot([], [], [], "g-", lw=2, label="Wings")

        self.canvas = FigureCanvas(fig)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Default state
        self.imu_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def update_attitude(self, roll: float, pitch: float, yaw: float):
        """Update 3D aircraft visualization"""
        self.imu_state = {"roll": roll, "pitch": pitch, "yaw": yaw}

        r = np.deg2rad(roll)
        p = np.deg2rad(pitch)
        y = np.deg2rad(yaw)

        # Rotation matrices
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        Rm = Rz @ Ry @ Rx

        # Transform points
        body_pts = np.array([[1, 0, 0], [-1, 0, 0]]).T
        wing_pts = np.array([[0, -0.6, 0.6], [0, 0, 0], [0, 0, 0]]).T
        b = (Rm @ body_pts).T
        w = (Rm @ wing_pts).T

        self.air_body.set_data(b[:, 0], b[:, 1])
        self.air_body.set_3d_properties(b[:, 2])
        self.air_wing.set_data(w[:, 0], w[:, 1])
        self.air_wing.set_3d_properties(w[:, 2])

        self.canvas.draw_idle()

##############################################################################
# Main Enhanced GUI
##############################################################################
class EnhancedExoPulseGUI(QMainWindow):
    """Main ExoPulse controller with integrated motor monitoring"""

    HIST_LEN = 300  # Data history length (30s at 10Hz)
    PLOT_HZ = 10    # Plot update frequency

    def __init__(self):
        super().__init__()

        # Runtime attributes
        self.ser = None
        self.tcp_sock = None
        self.recording = False
        self.in_demo = False
        self.log_lines = []

        # Threading control
        self.stop_udp_evt = threading.Event()
        self.stop_uart_evt = threading.Event()
        self.data_queue = queue.Queue()

        # Motor data buffers (for CAN motors - detailed monitoring)
        self.motor_data = {
            1: {
                'time': deque(maxlen=self.HIST_LEN),
                'temp': deque(maxlen=self.HIST_LEN),
                'current': deque(maxlen=self.HIST_LEN),
                'speed': deque(maxlen=self.HIST_LEN),
                'acceleration': deque(maxlen=self.HIST_LEN),
                'angle': deque(maxlen=self.HIST_LEN),
            },
            2: {
                'time': deque(maxlen=self.HIST_LEN),
                'temp': deque(maxlen=self.HIST_LEN),
                'current': deque(maxlen=self.HIST_LEN),
                'speed': deque(maxlen=self.HIST_LEN),
                'acceleration': deque(maxlen=self.HIST_LEN),
                'angle': deque(maxlen=self.HIST_LEN),
            }
        }

        self.motor_status = {
            1: {'motor_id': 1, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0},
            2: {'motor_id': 2, 'temp': 0, 'voltage': 0, 'current': 0, 'speed': 0, 'acceleration': 0, 'angle': 0}
        }

        # General motor hist (for other data)
        self.motor_hist = {
            k: deque(maxlen=self.HIST_LEN) for k in
            ("R_ang", "R_spd", "R_cur", "L_ang", "L_spd", "L_cur")
        }
        self.imu_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # IMU 3D window (initially None, created on demand)
        self.imu_window = None

        # Motor mode (CAN or Lower)
        self.motor_mode = "can"  # Default to CAN

        # Connection status
        self.connection_status = "Disconnected"
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.serial_lock = threading.Lock()

        # Display settings
        self.visible_plots = {
            'temp': True,
            'current': True,
            'speed': True,
            'acceleration': True,
            'angle': True
        }

        self.start_time = time.time()
        self.frame_count = 0

        # Build UI
        self.setWindowTitle("ExoPulse Enhanced Controller")
        self.setup_ui()

        # Start with WiFi mode
        self.current_mode = "wifi"
        self._connect_tcp_async()
        self._start_udp_listener()

        # Start data pump timer
        self.timer = QTimer()
        self.timer.timeout.connect(self._pump_data_queue)
        self.timer.start(int(1000 / self.PLOT_HZ))

    def setup_ui(self):
        """Build the user interface with sidebar"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main splitter (sidebar | main area)
        main_splitter = QSplitter(Qt.Horizontal)

        # LEFT: Sidebar with controls and display options
        sidebar = self._create_sidebar()
        main_splitter.addWidget(sidebar)

        # RIGHT: Main content area
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Log display
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet("font-family: Consolas; font-size: 9pt;")
        right_layout.addWidget(QLabel("System Log:"))
        right_layout.addWidget(self.log_text)

        # Plot area (placeholder)
        self.plot_widget = QWidget()
        self.plot_layout = QVBoxLayout(self.plot_widget)
        right_layout.addWidget(self.plot_widget)

        main_splitter.addWidget(right_widget)

        # Set splitter sizes (sidebar smaller)
        main_splitter.setSizes([300, 900])

        layout = QVBoxLayout(central_widget)
        layout.addWidget(main_splitter)

        # Initialize plots
        self._rebuild_plots()

    def _create_sidebar(self):
        """Create sidebar with all controls"""
        sidebar_widget = QWidget()
        sidebar_widget.setMaximumWidth(320)
        layout = QVBoxLayout(sidebar_widget)

        # Scroll area for sidebar
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)

        # === Motor Control Group ===
        control_group = QGroupBox("Motor Control")
        control_layout = QVBoxLayout()

        # Angle inputs
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Angle A:"))
        self.entry_a = QLineEdit("-30")
        self.entry_a.setFixedWidth(50)
        row1.addWidget(self.entry_a)
        row1.addWidget(QLabel("B:"))
        self.entry_b = QLineEdit("30")
        self.entry_b.setFixedWidth(50)
        row1.addWidget(self.entry_b)
        control_layout.addLayout(row1)

        # Speed
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Speed (deg/s):"))
        self.entry_speed = QLineEdit("25")
        self.entry_speed.setFixedWidth(60)
        row2.addWidget(self.entry_speed)
        row2.addStretch()
        control_layout.addLayout(row2)

        # Foot selection
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Foot:"))
        self.foot_group = QButtonGroup()
        self.radio_left = QRadioButton("L")
        self.radio_right = QRadioButton("R")
        self.radio_both = QRadioButton("Both")
        self.radio_both.setChecked(True)
        for radio in [self.radio_left, self.radio_right, self.radio_both]:
            self.foot_group.addButton(radio)
            row3.addWidget(radio)
        row3.addStretch()
        control_layout.addLayout(row3)

        # Motor type
        row4 = QHBoxLayout()
        row4.addWidget(QLabel("Motor:"))
        self.motor_type_group = QButtonGroup()
        self.radio_can = QRadioButton("CAN")
        self.radio_lower = QRadioButton("Lower")
        self.radio_can.setChecked(True)
        self.motor_type_group.addButton(self.radio_can)
        self.motor_type_group.addButton(self.radio_lower)
        row4.addWidget(self.radio_can)
        row4.addWidget(self.radio_lower)
        self.radio_can.toggled.connect(self._on_motor_type_change)
        row4.addStretch()
        control_layout.addLayout(row4)

        control_group.setLayout(control_layout)
        scroll_layout.addWidget(control_group)

        # === Communication Group ===
        comm_group = QGroupBox("Communication")
        comm_layout = QVBoxLayout()

        # Source selection
        row5 = QHBoxLayout()
        self.mode_group = QButtonGroup()
        self.radio_wifi = QRadioButton("WiFi")
        self.radio_uart = QRadioButton("UART")
        self.radio_wifi.setChecked(True)
        if serial is None:
            self.radio_uart.setEnabled(False)
        self.mode_group.addButton(self.radio_wifi)
        self.mode_group.addButton(self.radio_uart)
        row5.addWidget(self.radio_wifi)
        row5.addWidget(self.radio_uart)
        comm_layout.addLayout(row5)

        # COM port
        if serial and list_ports:
            all_ports = [p.device for p in list_ports.comports()]
            ports = [p for p in all_ports if 'ttyUSB' in p]
            if not ports:
                ports = ["No ttyUSB port"]
        else:
            ports = ["No ttyUSB port"]

        self.combo_com = QComboBox()
        self.combo_com.addItems(ports)
        if "/dev/ttyUSB0" in ports:
            self.combo_com.setCurrentText("/dev/ttyUSB0")
        comm_layout.addWidget(self.combo_com)

        self.radio_wifi.toggled.connect(self._on_mode_change)

        comm_group.setLayout(comm_layout)
        scroll_layout.addWidget(comm_group)

        # === Display Options Group ===
        display_group = QGroupBox("Display Options")
        display_layout = QVBoxLayout()

        display_layout.addWidget(QLabel("Show Data:"))
        self.check_temp = QCheckBox("Temperature")
        self.check_temp.setChecked(True)
        self.check_temp.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_temp)

        self.check_current = QCheckBox("Current")
        self.check_current.setChecked(True)
        self.check_current.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_current)

        self.check_speed = QCheckBox("Speed")
        self.check_speed.setChecked(True)
        self.check_speed.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_speed)

        self.check_accel = QCheckBox("Acceleration")
        self.check_accel.setChecked(True)
        self.check_accel.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_accel)

        self.check_angle = QCheckBox("Angle")
        self.check_angle.setChecked(True)
        self.check_angle.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_angle)

        display_group.setLayout(display_layout)
        scroll_layout.addWidget(display_group)

        # === Action Buttons ===
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout()

        btn_swing = QPushButton("Run Swing")
        btn_swing.clicked.connect(self._start_swing_thread)
        action_layout.addWidget(btn_swing)

        btn_led = QPushButton("Toggle LED")
        btn_led.clicked.connect(self._toggle_led)
        action_layout.addWidget(btn_led)

        self.btn_demo = QPushButton("Start Demo")
        self.btn_demo.clicked.connect(self._toggle_demo_mode)
        action_layout.addWidget(self.btn_demo)

        self.btn_record = QPushButton("Start Record")
        self.btn_record.clicked.connect(self._toggle_record)
        action_layout.addWidget(self.btn_record)

        self.btn_imu = QPushButton("Show IMU")
        self.btn_imu.clicked.connect(self._toggle_imu_window)
        action_layout.addWidget(self.btn_imu)

        action_group.setLayout(action_layout)
        scroll_layout.addWidget(action_group)

        # === Calibration Buttons (for CAN motors) ===
        cal_group = QGroupBox("Calibration (CAN)")
        cal_layout = QVBoxLayout()

        btn_cal_m1 = QPushButton("Cal Motor 1")
        btn_cal_m1.clicked.connect(lambda: self._calibrate_motor(1))
        cal_layout.addWidget(btn_cal_m1)

        btn_cal_m2 = QPushButton("Cal Motor 2")
        btn_cal_m2.clicked.connect(lambda: self._calibrate_motor(2))
        cal_layout.addWidget(btn_cal_m2)

        btn_cal_both = QPushButton("Cal Both")
        btn_cal_both.clicked.connect(lambda: self._calibrate_motor(0))
        cal_layout.addWidget(btn_cal_both)

        btn_clear_cal = QPushButton("Clear Cal")
        btn_clear_cal.clicked.connect(self._clear_calibration)
        cal_layout.addWidget(btn_clear_cal)

        cal_group.setLayout(cal_layout)
        scroll_layout.addWidget(cal_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        return sidebar_widget

    def _on_display_change(self):
        """Handle display option changes"""
        self.visible_plots['temp'] = self.check_temp.isChecked()
        self.visible_plots['current'] = self.check_current.isChecked()
        self.visible_plots['speed'] = self.check_speed.isChecked()
        self.visible_plots['acceleration'] = self.check_accel.isChecked()
        self.visible_plots['angle'] = self.check_angle.isChecked()

        self._rebuild_plots()

    def _rebuild_plots(self):
        """Rebuild plots based on visible options"""
        # Clear existing plot
        for i in reversed(range(self.plot_layout.count())):
            widget = self.plot_layout.itemAt(i).widget()
            if widget:
                widget.setParent(None)

        # Count visible plots
        visible_count = sum(1 for v in self.visible_plots.values() if v)

        if visible_count == 0:
            label = QLabel("No data selected for display")
            label.setAlignment(Qt.AlignCenter)
            self.plot_layout.addWidget(label)
            return

        # Create new figure
        plt.style.use('dark_background')
        self.fig = plt.Figure(figsize=(12, 2.5 * visible_count), dpi=100)
        self.canvas = FigureCanvas(self.fig)

        # Create grid
        gs = gridspec.GridSpec(visible_count, 2, hspace=0.4, wspace=0.3, top=0.95, bottom=0.08)

        self.axes = {}
        self.lines = {}
        row = 0

        plot_configs = [
            ('temp', 'Temperature', '°C', (0, 80), 'cyan', 'orange'),
            ('current', 'Current', 'A', (-1, 1), 'cyan', 'orange'),
            ('speed', 'Speed', 'rad/s', (-0.35, 0.35), 'cyan', 'orange'),
            ('acceleration', 'Acceleration', 'dps²', (-200, 200), 'cyan', 'orange'),
            ('angle', 'Angle', '°', (-180, 180), 'cyan', 'orange'),
        ]

        for key, title, ylabel, ylim, color1, color2 in plot_configs:
            if not self.visible_plots[key]:
                continue

            # Motor 1 (left)
            ax1 = self.fig.add_subplot(gs[row, 0])
            ax1.set_title(f'Motor 1 - {title}', color=color1, fontweight='bold')
            ax1.set_ylabel(ylabel, color=color1)
            ax1.set_ylim(*ylim)
            ax1.grid(True, alpha=0.3)
            line1, = ax1.plot([], [], color=color1, lw=2)

            self.axes[f'm1_{key}'] = ax1
            self.lines[f'm1_{key}'] = line1

            # Motor 2 (right)
            ax2 = self.fig.add_subplot(gs[row, 1])
            ax2.set_title(f'Motor 2 - {title}', color=color2, fontweight='bold')
            ax2.set_ylabel(ylabel, color=color2)
            ax2.set_ylim(*ylim)
            ax2.grid(True, alpha=0.3)
            line2, = ax2.plot([], [], color=color2, lw=2)

            self.axes[f'm2_{key}'] = ax2
            self.lines[f'm2_{key}'] = line2

            row += 1

        # Status text
        self.status_text = self.fig.text(0.5, 0.02, '', ha='center', fontsize=8, family='monospace', color='lime')

        self.plot_layout.addWidget(self.canvas)

        # Start animation
        if hasattr(self, 'anim'):
            self.anim.event_source.stop()

        from matplotlib.animation import FuncAnimation
        self.anim = FuncAnimation(
            self.fig, self._update_plots,
            interval=50,
            blit=False,
            cache_frame_data=False
        )

    def _update_plots(self, frame):
        """Update all visible plots"""
        window_size = 30  # seconds

        for motor_id in [1, 2]:
            data = self.motor_data[motor_id]
            prefix = f'm{motor_id}'

            if len(data['time']) < 2:
                continue

            t = list(data['time'])

            for key in ['temp', 'current', 'speed', 'acceleration', 'angle']:
                if not self.visible_plots[key]:
                    continue

                line_key = f'{prefix}_{key}'
                if line_key in self.lines:
                    values = list(data[key])

                    # Special handling for speed (convert to rad/s)
                    if key == 'speed':
                        values = [v * 0.01745 for v in values]  # deg/s to rad/s

                    self.lines[line_key].set_data(t, values)

                    # Update x-axis
                    ax = self.axes[line_key]
                    ax.set_xlim(max(0, t[-1] - window_size), t[-1] + 1)

                    # Auto-scale y-axis for angle and acceleration
                    if key in ['angle', 'acceleration'] and len(values) > 0:
                        val_min, val_max = min(values), max(values)
                        if val_max - val_min > 0:
                            margin = (val_max - val_min) * 0.2 + 5
                            ax.set_ylim(val_min - margin, val_max + margin)

        # Update status text
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            s1 = self.motor_status[1]
            s2 = self.motor_status[2]

            if "Connected" in self.connection_status:
                conn_indicator = "🟢"
            elif "Reconnecting" in self.connection_status:
                conn_indicator = "🟡"
            else:
                conn_indicator = "🔴"

            status = (f"{conn_indicator} {self.connection_status} | "
                     f"M1: {s1['temp']}°C {s1['current']:.2f}A {s1['speed']}°/s | "
                     f"M2: {s2['temp']}°C {s2['current']:.2f}A {s2['speed']}°/s")
            self.status_text.set_text(status)

        return []

    # ========== Motor Type & Mode Management ==========
    def _on_motor_type_change(self):
        """Handle motor type change"""
        if self.radio_can.isChecked():
            self.motor_mode = "can"
            self._log("🔧 Motor mode: CAN")
        else:
            self.motor_mode = "lower"
            self._log("🔧 Motor mode: Lower Chip")

    # ========== Calibration (CAN motors) ==========
    def _calibrate_motor(self, motor_id):
        """Calibrate motor zero position"""
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    if motor_id == 1:
                        self.ser.write(b"CAL1\n")
                        self.ser.flush()
                        self._log("✓ Calibrating Motor 1...")
                    elif motor_id == 2:
                        self.ser.write(b"CAL2\n")
                        self.ser.flush()
                        self._log("✓ Calibrating Motor 2...")
                    elif motor_id == 0:
                        self.ser.write(b"CAL1\n")
                        self.ser.flush()
                        time.sleep(0.2)
                        self.ser.write(b"CAL2\n")
                        self.ser.flush()
                        self._log("✓ Calibrating BOTH motors...")
                else:
                    self._log("✗ Serial port not connected")
        except Exception as e:
            self._log(f"✗ Calibration failed: {e}")

    def _clear_calibration(self):
        """Clear calibration offsets"""
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    self.ser.write(b"CLEAR_CAL\n")
                    self.ser.flush()
                    self._log("✓ Calibration cleared")
                else:
                    self._log("✗ Serial port not connected")
        except Exception as e:
            self._log(f"✗ Clear failed: {e}")

    # ========== IMU Window ==========
    def _toggle_imu_window(self):
        """Show/hide IMU window"""
        if self.imu_window is None or not self.imu_window.isVisible():
            if self.imu_window is None:
                self.imu_window = IMU3DWindow()
            self.imu_window.show()
            self.btn_imu.setText("Hide IMU")
            self._log("✓ IMU window opened")
        else:
            self.imu_window.hide()
            self.btn_imu.setText("Show IMU")
            self._log("✓ IMU window closed")

    # ========== Network Helpers ==========
    def _connect_tcp_async(self):
        """Asynchronously connect to ESP32"""
        def worker():
            try:
                sock = socket.create_connection((ESP32_IP, ESP32_PORT), timeout=2)
                self.data_queue.put(("tcp_ready", sock))
            except Exception as exc:
                self.data_queue.put(("log", f"⚠ TCP connect failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _tcp_ready(self, sock: socket.socket):
        """Handle TCP connection"""
        self.tcp_sock = sock
        self._log(f"✓ TCP connected to {ESP32_IP}:{ESP32_PORT}")

    def _start_udp_listener(self):
        """Start UDP listener"""
        def listener():
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                udp.bind(("", UDP_PORT))
                udp.settimeout(0.5)
                self.data_queue.put(("log", f"✓ UDP listening on port {UDP_PORT}"))

                while not self.stop_udp_evt.is_set():
                    try:
                        data, addr = udp.recvfrom(BUF_SIZE)
                        line = data.decode(errors="ignore").strip()
                        msg = f"[IMU {addr[0]}] {line}"
                        self.data_queue.put(("data", msg))

                        if self.recording:
                            self.log_lines.append(f"{datetime.now().isoformat()} {msg}")
                    except socket.timeout:
                        continue
            finally:
                udp.close()

        threading.Thread(target=listener, daemon=True).start()

    def _start_uart_listener(self):
        """Start UART listener"""
        if serial is None:
            self._log("⚠ pyserial not installed")
            return

        def listener():
            try:
                port = self.combo_com.currentText()
                if port.startswith("No ttyUSB"):
                    self.data_queue.put(("log", "⚠ No ttyUSB port"))
                    return

                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        self.data_queue.put(("log", "🔌 Closed existing UART"))
                    except Exception as e:
                        self.data_queue.put(("log", f"⚠ Close failed: {e}"))

                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.connection_status = "Connected"
                self.data_queue.put(("log", f"✓ UART on {port}"))

                while not self.stop_uart_evt.is_set():
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line:
                        msg = f"[UART] {line}"
                        self.data_queue.put(("data", msg))

                        if self.recording:
                            self.log_lines.append(f"{datetime.now().isoformat()} {msg}")

                self.ser.close()
                self.ser = None
            except Exception as exc:
                self.connection_status = "Error"
                self.data_queue.put(("log", f"⚠ UART error: {exc}"))

        threading.Thread(target=listener, daemon=True).start()

    # ========== Data Processing ==========
    def _parse_motor_line(self, line):
        """Parse CAN motor data line"""
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

    def _parse_general_line(self, raw: str):
        """Parse general motor data (for Lower Chip mode)"""
        try:
            tokens = raw.split()
            if tokens[0] != "X" or len(tokens) < 10:
                return None

            vals = list(map(float, tokens[1:]))
            R_ang, R_spd, R_cur, L_ang, L_spd, L_cur, roll, pitch, yaw = vals

            self.motor_hist["R_ang"].append(R_ang)
            self.motor_hist["R_spd"].append(R_spd)
            self.motor_hist["R_cur"].append(R_cur * 0.01)
            self.motor_hist["L_ang"].append(L_ang)
            self.motor_hist["L_spd"].append(L_spd)
            self.motor_hist["L_cur"].append(L_cur * 0.01)

            self.imu_state.update(roll=roll, pitch=pitch, yaw=yaw)
            return True
        except:
            return None

    def _pump_data_queue(self):
        """Process queued data"""
        while not self.data_queue.empty():
            msg_type, msg = self.data_queue.get()

            if msg_type == "log":
                self._log(msg)
            elif msg_type == "data":
                self._log(msg)

                # Try to parse as CAN motor data
                if "[" in msg:
                    status = self._parse_motor_line(msg)
                    if status:
                        motor_id = status['motor_id']
                        if motor_id in [1, 2]:
                            elapsed = time.time() - self.start_time
                            self.motor_status[motor_id].update(status)
                            data = self.motor_data[motor_id]
                            data['time'].append(elapsed)
                            data['temp'].append(status['temp'])
                            data['current'].append(status['current'])
                            data['speed'].append(status['speed'])
                            data['acceleration'].append(status['acceleration'])
                            data['angle'].append(status['angle'])

                # Try to parse as general data
                if "] X " in msg:
                    self._parse_general_line(msg.split("]", 1)[1].strip())

            elif msg_type == "tcp_ready":
                self._tcp_ready(msg)

        # Update IMU if visible
        if self.imu_window and self.imu_window.isVisible():
            self.imu_window.update_attitude(
                self.imu_state["roll"],
                self.imu_state["pitch"],
                self.imu_state["yaw"]
            )

    def _log(self, message: str):
        """Add message to log"""
        if self.recording:
            self.log_lines.append(f"{datetime.now().isoformat()} {message}")

        self.log_text.append(message)
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    # ========== Control Actions ==========
    def _send(self, pkt: bytes, desc: str = ""):
        """Send packet"""
        mode_str = f"[{self.motor_mode.upper()}]"

        if self.current_mode == "wifi":
            if not self.tcp_sock:
                self._log("⚠ TCP not connected")
                return
            try:
                self.tcp_sock.sendall(pkt)
                self._log(f"→ {mode_str} [TCP] {desc}")
            except Exception as exc:
                self._log(f"⚠ TCP error: {exc}")
        else:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(pkt)
                    self._log(f"→ {mode_str} [UART] {desc}")
                else:
                    self._log("⚠ UART not open")
            except Exception as exc:
                self._log(f"⚠ UART error: {exc}")

    def _toggle_led(self):
        """Toggle LED"""
        self._send(make_led_packet(), "LED toggle")

    def _send_stop(self):
        """Stop motors"""
        self._send(*make_motor_packet("E", 0, "E", 0))

    def _start_swing_thread(self):
        """Start swing motion"""
        threading.Thread(target=self._run_swing, daemon=True).start()

    def _run_swing(self):
        """Execute swing"""
        def deg2centideg(x):
            return int(round(x * 100))

        try:
            a = float(self.entry_a.text())
            b = float(self.entry_b.text())
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid angles")
            return

        foot = "both"
        if self.radio_right.isChecked():
            foot = "right"
        elif self.radio_left.isChecked():
            foot = "left"

        def get_angles(pos):
            if foot == "right":
                return deg2centideg(pos), 0
            elif foot == "left":
                return 0, deg2centideg(pos)
            else:
                return deg2centideg(pos), -deg2centideg(pos)

        self._send(*make_motor_packet("A", *get_angles(b)))
        self._log(f"→ Swing to {b}°")
        time.sleep(2)

        self._send(*make_motor_packet("A", *get_angles(a)))
        self._log(f"← Swing to {a}°")
        time.sleep(2)

        self._send_stop()
        self._log("✓ Swing complete")

    def _toggle_demo_mode(self):
        """Toggle demo mode"""
        if not self.in_demo:
            self._send(*make_motor_packet("G", 0, "G", 0))
            self._log("🚶 Demo started")
            self.btn_demo.setText("Stop Demo")
            self.in_demo = True
        else:
            self._send_stop()
            self._log("🛑 Demo stopped")
            self.btn_demo.setText("Start Demo")
            self.in_demo = False

    def _toggle_record(self):
        """Toggle recording"""
        if not self.recording:
            self.recording = True
            self.log_lines.clear()
            self.btn_record.setText("Stop Record")
            self._log("★ Recording started")
        else:
            self.recording = False
            fname = datetime.now().strftime("exopulse_log_%Y%m%d_%H%M%S.txt")
            try:
                with open(fname, "w", encoding="utf-8") as f:
                    f.write("\n".join(self.log_lines))
                self._log(f"★ Saved → {fname}")
            except Exception as exc:
                self._log(f"⚠ Save error: {exc}")
            self.btn_record.setText("Start Record")

    def _on_mode_change(self):
        """Handle mode change"""
        if self.radio_wifi.isChecked():
            self.current_mode = "wifi"
            self.stop_uart_evt.set()
            self.stop_udp_evt.clear()
            self._log("🔁 WiFi mode")
            self._connect_tcp_async()
            self._start_udp_listener()
        else:
            self.current_mode = "uart"
            self.stop_udp_evt.set()
            self.stop_uart_evt.clear()
            self._log("🔁 UART mode")
            self._start_uart_listener()

    def closeEvent(self, event):
        """Handle close"""
        reply = QMessageBox.question(
            self, "Quit", "Exit?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.timer.stop()
            self.stop_udp_evt.set()
            self.stop_uart_evt.set()

            if self.tcp_sock:
                try:
                    self._send_stop()
                    self.tcp_sock.close()
                except:
                    pass

            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except:
                    pass

            if self.recording:
                self._toggle_record()

            if self.imu_window:
                self.imu_window.close()

            event.accept()
        else:
            event.ignore()

##############################################################################
# Main Entry Point
##############################################################################
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = EnhancedExoPulseGUI()
    window.show()

    sys.exit(app.exec())
