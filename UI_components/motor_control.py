#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Motor Controller GUI (PySide6)

Features:
- Real-time motor data visualization (angle, speed, current)
- WiFi/UART communication modes
- CAN / Lower Chip motor control modes
- IMU 3D attitude display (optional, toggle with button)
- Non-blocking I/O with threading
- Data recording capability
"""

import socket
import threading
import queue
import time
import sys
from datetime import datetime
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QRadioButton, QComboBox,
    QTextEdit, QGroupBox, QMessageBox, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
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
# Main GUI
##############################################################################
class ExoPulseGUI(QMainWindow):
    """Main ExoPulse motor controller window"""

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

        # Data buffers
        self.motor_hist = {
            k: deque(maxlen=self.HIST_LEN) for k in
            ("R_ang", "R_spd", "R_cur", "L_ang", "L_spd", "L_cur")
        }
        self.imu_state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # IMU 3D window (initially None, created on demand)
        self.imu_window = None

        # Motor mode (CAN or Lower)
        self.motor_mode = "can"  # Default to CAN

        # Build UI
        self.setWindowTitle("ExoPulse Motor Controller")
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
        """Build the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Control panel
        control_group = self._create_control_panel()
        main_layout.addWidget(control_group)

        # Log display
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("font-family: Consolas; font-size: 10pt;")
        main_layout.addWidget(QLabel("System Log:"))
        main_layout.addWidget(self.log_text)

        # Plot area
        self.plot_canvas = self._create_plots()
        main_layout.addWidget(self.plot_canvas)

    def _create_control_panel(self):
        """Create control panel with inputs and buttons"""
        group = QGroupBox("Motor Control")
        layout = QVBoxLayout()

        # Row 1: Angle inputs
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Angle A (deg):"))
        self.entry_a = QLineEdit("-30")
        self.entry_a.setFixedWidth(60)
        row1.addWidget(self.entry_a)

        row1.addWidget(QLabel("Angle B (deg):"))
        self.entry_b = QLineEdit("30")
        self.entry_b.setFixedWidth(60)
        row1.addWidget(self.entry_b)

        row1.addWidget(QLabel("Speed (deg/s):"))
        self.entry_speed = QLineEdit("25")
        self.entry_speed.setFixedWidth(60)
        row1.addWidget(self.entry_speed)
        row1.addStretch()
        layout.addLayout(row1)

        # Row 2: Foot selection
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Foot:"))
        self.foot_group = QButtonGroup()
        self.radio_left = QRadioButton("Left")
        self.radio_right = QRadioButton("Right")
        self.radio_both = QRadioButton("Both")
        self.radio_both.setChecked(True)

        for radio in [self.radio_left, self.radio_right, self.radio_both]:
            self.foot_group.addButton(radio)
            row2.addWidget(radio)
        row2.addStretch()
        layout.addLayout(row2)

        # Row 2.5: Motor Type selection
        row2_5 = QHBoxLayout()
        row2_5.addWidget(QLabel("Motor Type:"))
        self.motor_type_group = QButtonGroup()
        self.radio_can = QRadioButton("CAN")
        self.radio_lower = QRadioButton("Lower Chip")
        self.radio_can.setChecked(True)

        self.motor_type_group.addButton(self.radio_can)
        self.motor_type_group.addButton(self.radio_lower)
        row2_5.addWidget(self.radio_can)
        row2_5.addWidget(self.radio_lower)

        # Connect motor type change
        self.radio_can.toggled.connect(self._on_motor_type_change)

        row2_5.addStretch()
        layout.addLayout(row2_5)

        # Row 3: Communication mode
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Source:"))
        self.mode_group = QButtonGroup()
        self.radio_wifi = QRadioButton("WiFi (UDP)")
        self.radio_uart = QRadioButton("UART")
        self.radio_wifi.setChecked(True)

        if serial is None:
            self.radio_uart.setEnabled(False)

        self.mode_group.addButton(self.radio_wifi)
        self.mode_group.addButton(self.radio_uart)
        row3.addWidget(self.radio_wifi)
        row3.addWidget(self.radio_uart)

        # COM port selector - Filter to show only ttyUSB* ports
        if serial and list_ports:
            all_ports = [p.device for p in list_ports.comports()]
            # Filter only ttyUSB ports
            ports = [p for p in all_ports if 'ttyUSB' in p]
            if not ports:
                ports = ["No ttyUSB port"]
        else:
            ports = ["No ttyUSB port"]

        self.combo_com = QComboBox()
        self.combo_com.addItems(ports)
        if "/dev/ttyUSB0" in ports:
            self.combo_com.setCurrentText("/dev/ttyUSB0")
        row3.addWidget(self.combo_com)

        row3.addStretch()
        layout.addLayout(row3)

        # Row 4: Action buttons
        row4 = QHBoxLayout()

        btn_swing = QPushButton("Run Swing")
        btn_swing.clicked.connect(self._start_swing_thread)
        row4.addWidget(btn_swing)

        btn_led = QPushButton("Toggle LED")
        btn_led.clicked.connect(self._toggle_led)
        row4.addWidget(btn_led)

        self.btn_demo = QPushButton("Start Demo")
        self.btn_demo.clicked.connect(self._toggle_demo_mode)
        row4.addWidget(self.btn_demo)

        self.btn_record = QPushButton("Start Record")
        self.btn_record.clicked.connect(self._toggle_record)
        row4.addWidget(self.btn_record)

        self.btn_imu = QPushButton("Show IMU")
        self.btn_imu.clicked.connect(self._toggle_imu_window)
        row4.addWidget(self.btn_imu)

        row4.addStretch()
        layout.addLayout(row4)

        # Connect mode change
        self.radio_wifi.toggled.connect(self._on_mode_change)

        group.setLayout(layout)
        return group

    def _create_plots(self):
        """Create matplotlib plots for motor data"""
        fig = plt.Figure(figsize=(10, 3.5), dpi=100)

        self.axes = {
            "ang": fig.add_subplot(131),
            "spd": fig.add_subplot(132),
            "cur": fig.add_subplot(133),
        }

        # Configure axes
        for ax, title, ylim in [
            (self.axes["ang"], "Angle (°)", (-120, 120)),
            (self.axes["spd"], "Speed (°/s)", (-120, 120)),
            (self.axes["cur"], "Current (A)", (-3, 3)),
        ]:
            ax.set_title(title)
            ax.set_xlim(0, self.HIST_LEN)
            ax.set_ylim(*ylim)
            ax.grid(True)

        # Create lines
        self.lines = {
            "ang_R": self.axes["ang"].plot([], [], "r-", label="Right")[0],
            "ang_L": self.axes["ang"].plot([], [], "b-", label="Left")[0],
            "spd_R": self.axes["spd"].plot([], [], "r-", label="Right")[0],
            "spd_L": self.axes["spd"].plot([], [], "b-", label="Left")[0],
            "cur_R": self.axes["cur"].plot([], [], "r-", label="Right")[0],
            "cur_L": self.axes["cur"].plot([], [], "b-", label="Left")[0],
        }

        for ax in self.axes.values():
            ax.legend(loc="upper right")

        fig.tight_layout()
        canvas = FigureCanvas(fig)
        return canvas

    # ========== IMU Window Management ==========
    def _toggle_imu_window(self):
        """Show/hide IMU 3D attitude window"""
        if self.imu_window is None or not self.imu_window.isVisible():
            # Create or show window
            if self.imu_window is None:
                self.imu_window = IMU3DWindow()
            self.imu_window.show()
            self.btn_imu.setText("Hide IMU")
            self._log("✓ IMU 3D window opened")
        else:
            # Hide window
            self.imu_window.hide()
            self.btn_imu.setText("Show IMU")
            self._log("✓ IMU 3D window closed")

    # ========== Motor Type Management ==========
    def _on_motor_type_change(self):
        """Handle motor type change (CAN / Lower Chip)"""
        if self.radio_can.isChecked():
            self.motor_mode = "can"
            self._log("🔧 Motor mode: CAN")
        else:
            self.motor_mode = "lower"
            self._log("🔧 Motor mode: Lower Chip")

    # ========== Network Helpers ==========
    def _connect_tcp_async(self):
        """Asynchronously connect to ESP32 via TCP"""
        def worker():
            try:
                sock = socket.create_connection((ESP32_IP, ESP32_PORT), timeout=2)
                self.data_queue.put(("tcp_ready", sock))
            except Exception as exc:
                self.data_queue.put(("log", f"⚠ TCP connect failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _tcp_ready(self, sock: socket.socket):
        """Handle successful TCP connection"""
        self.tcp_sock = sock
        self._log(f"✓ TCP connected to {ESP32_IP}:{ESP32_PORT}")

    def _start_udp_listener(self):
        """Start UDP listener thread"""
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
        """Start UART listener thread"""
        if serial is None:
            self._log("⚠ pyserial not installed; UART disabled")
            return

        def listener():
            try:
                port = self.combo_com.currentText()
                if port.startswith("No ttyUSB"):
                    self.data_queue.put(("log", "⚠ No ttyUSB port available"))
                    return

                # Close existing connection
                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        self.data_queue.put(("log", "🔌 Closed existing UART"))
                    except Exception as e:
                        self.data_queue.put(("log", f"⚠ UART close failed: {e}"))

                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.data_queue.put(("log", f"✓ UART listening on {port}"))

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
                self.data_queue.put(("log", f"⚠ UART error: {exc}"))

        threading.Thread(target=listener, daemon=True).start()

    # ========== Data Processing ==========
    def _parse_line(self, raw: str):
        """
        Parse incoming data line
        Format: 'X 0.0 0 0 -0.4 15 22 -56.4 -82.5 37.8'
        """
        try:
            tokens = raw.split()
            if tokens[0] != "X" or len(tokens) < 10:
                return

            vals = list(map(float, tokens[1:]))
            R_ang, R_spd, R_cur, L_ang, L_spd, L_cur, roll, pitch, yaw = vals

            self.motor_hist["R_ang"].append(R_ang)
            self.motor_hist["R_spd"].append(R_spd)
            self.motor_hist["R_cur"].append(R_cur * 0.01)  # Convert to A
            self.motor_hist["L_ang"].append(L_ang)
            self.motor_hist["L_spd"].append(L_spd)
            self.motor_hist["L_cur"].append(L_cur * 0.01)

            self.imu_state.update(roll=roll, pitch=pitch, yaw=yaw)
        except Exception:
            pass

    def _update_plots(self):
        """Update all plots with latest data"""
        xs = range(len(self.motor_hist["R_ang"]))

        # Update motor plots
        line_map = {
            "ang_R": "R_ang", "ang_L": "L_ang",
            "spd_R": "R_spd", "spd_L": "L_spd",
            "cur_R": "R_cur", "cur_L": "L_cur",
        }

        for line_key, hist_key in line_map.items():
            buf = self.motor_hist[hist_key]
            self.lines[line_key].set_data(xs, list(buf))

        for ax in self.axes.values():
            ax.set_xlim(0, self.HIST_LEN)

        self.plot_canvas.draw_idle()

        # Update IMU window if visible
        if self.imu_window and self.imu_window.isVisible():
            self.imu_window.update_attitude(
                self.imu_state["roll"],
                self.imu_state["pitch"],
                self.imu_state["yaw"]
            )

    def _pump_data_queue(self):
        """Process queued data and update UI"""
        while not self.data_queue.empty():
            msg_type, msg = self.data_queue.get()

            if msg_type == "log":
                self._log(msg)
            elif msg_type == "data":
                self._log(msg)
                # Parse data line
                if "] X " in msg:
                    self._parse_line(msg.split("]", 1)[1].strip())
            elif msg_type == "tcp_ready":
                self._tcp_ready(msg)

        self._update_plots()

    def _log(self, message: str):
        """Add message to log display"""
        if self.recording:
            self.log_lines.append(f"{datetime.now().isoformat()} {message}")

        self.log_text.append(message)
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    # ========== Control Actions ==========
    def _send(self, pkt: bytes, desc: str = ""):
        """Send packet via current communication mode"""
        mode_str = f"[{self.motor_mode.upper()}]"

        if self.current_mode == "wifi":
            if not self.tcp_sock:
                self._log("⚠ TCP not connected")
                return
            try:
                self.tcp_sock.sendall(pkt)
                self._log(f"→ {mode_str} [TCP] {desc}")
            except Exception as exc:
                self._log(f"⚠ TCP send error: {exc}")
        else:  # uart
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(pkt)
                    self._log(f"→ {mode_str} [UART] {desc}")
                else:
                    self._log("⚠ UART port not open")
            except Exception as exc:
                self._log(f"⚠ UART send error: {exc}")

    def _toggle_led(self):
        """Toggle LED on device"""
        self._send(make_led_packet(), "LED toggle (0x10)")

    def _send_stop(self):
        """Send motor stop command"""
        self._send(*make_motor_packet("E", 0, "E", 0))

    def _start_swing_thread(self):
        """Start swing motion in background thread"""
        threading.Thread(target=self._run_swing, daemon=True).start()

    def _run_swing(self):
        """Execute swing motion"""
        def deg2centideg(x):
            return int(round(x * 100))

        try:
            a = float(self.entry_a.text())
            b = float(self.entry_b.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Angles must be numeric values")
            return

        # Determine which foot to move
        if self.radio_right.isChecked():
            foot = "right"
        elif self.radio_left.isChecked():
            foot = "left"
        else:
            foot = "both"

        def get_angles(pos):
            if foot == "right":
                return deg2centideg(pos), 0
            elif foot == "left":
                return 0, deg2centideg(pos)
            else:
                return deg2centideg(pos), -deg2centideg(pos)

        # Swing to position B
        self._send(*make_motor_packet("A", *get_angles(b)))
        self._log(f"→ Swing to {b}°")
        time.sleep(2)

        # Swing to position A
        self._send(*make_motor_packet("A", *get_angles(a)))
        self._log(f"← Swing back to {a}°")
        time.sleep(2)

        # Stop
        self._send_stop()
        self._log("✓ Swing cycle complete")

    def _toggle_demo_mode(self):
        """Toggle demo mode (gait generation)"""
        if not self.in_demo:
            self._send(*make_motor_packet("G", 0, "G", 0))
            self._log("🚶 Demo mode started (gait generation)")
            self.btn_demo.setText("Stop Demo")
            self.in_demo = True
        else:
            self._send_stop()
            self._log("🛑 Demo mode stopped")
            self.btn_demo.setText("Start Demo")
            self.in_demo = False

    def _toggle_record(self):
        """Toggle data recording"""
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
                self._log(f"★ Log saved → {fname}")
            except Exception as exc:
                self._log(f"⚠ Save error: {exc}")
            self.btn_record.setText("Start Record")

    def _on_mode_change(self):
        """Handle WiFi/UART mode switch"""
        if self.radio_wifi.isChecked():
            self.current_mode = "wifi"
            self.stop_uart_evt.set()
            self.stop_udp_evt.clear()
            self._log("🔁 Switched to WiFi mode")
            self._connect_tcp_async()
            self._start_udp_listener()
        else:
            self.current_mode = "uart"
            self.stop_udp_evt.set()
            self.stop_uart_evt.clear()
            self._log("🔁 Switched to UART mode")
            self._start_uart_listener()

    def closeEvent(self, event):
        """Handle window close event"""
        reply = QMessageBox.question(
            self, "Quit", "Are you sure you want to exit?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Stop threads
            self.timer.stop()
            self.stop_udp_evt.set()
            self.stop_uart_evt.set()

            # Close connections
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

            # Save recording if active
            if self.recording:
                self._toggle_record()

            # Close IMU window
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
    app.setStyle("Fusion")  # Modern look

    window = ExoPulseGUI()
    window.show()

    sys.exit(app.exec())
