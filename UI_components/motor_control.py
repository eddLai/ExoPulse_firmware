#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Integrated Motor Controller & Monitor GUI

Features:
- Unified motor control and monitoring interface
- Real-time motor data visualization (angle, speed, current, temp, acceleration)
- WiFi/UART communication modes
- CAN / Lower Chip motor control modes
- IMU 3D attitude display (optional)
- Configurable data display with sidebar checkboxes
- Motor calibration controls (CAN mode)
- Data recording capability
- Default display: Current, Angle, Acceleration
"""

import socket
import threading
import queue
import time
import sys
import re
import subprocess
from datetime import datetime
from collections import deque
from pathlib import Path

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QRadioButton, QComboBox,
    QTextEdit, QGroupBox, QMessageBox, QButtonGroup, QCheckBox,
    QSplitter, QScrollArea, QSpinBox
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
# WiFi Configuration Dialog
##############################################################################
class WiFiConfigDialog(QWidget):
    """WiFi Configuration Dialog integrated into main GUI"""

    config_complete = Signal(str, int)  # Signal: (ip, port)

    def __init__(self, parent=None, serial_port='/dev/ttyACM0'):
        super().__init__(parent)
        self.setWindowFlags(Qt.Window | Qt.WindowStaysOnTopHint)
        self.serial_port = serial_port
        self.ser = None
        self.esp32_ip = None
        self.esp32_port = 8888

        # WiFi status tracking
        self.wifi_connected = False
        self.wifi_ssid = ""
        self.wifi_rssi = ""
        self.wifi_mac = ""
        self.no_serial_port = False  # Track if no serial port is available

        self.setWindowTitle("WiFi Configuration")
        self.setFixedSize(650, 500)
        self.setStyleSheet("""
            QWidget {
                background-color: #2C3E50;
                color: #ECF0F1;
            }
            QGroupBox {
                border: 2px solid #34495E;
                border-radius: 5px;
                margin-top: 10px;
                padding: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLineEdit {
                background-color: #34495E;
                border: 1px solid #566573;
                border-radius: 3px;
                padding: 5px;
                color: #ECF0F1;
            }
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #5DADE2, stop:1 #3498DB);
            }
            QPushButton:disabled {
                background-color: #7F8C8D;
                color: #BDC3C7;
            }
            QLabel {
                color: #ECF0F1;
                padding: 2px;
            }
        """)
        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # Title
        title = QLabel("ESP32 WiFi Manager")
        title.setStyleSheet("font-size: 16pt; font-weight: bold; color: #3498DB; padding: 10px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # === Current Status Group (shown when connected) ===
        self.status_group = QGroupBox("Current WiFi Status")
        status_layout = QVBoxLayout()

        # Connection indicator with color
        indicator_row = QHBoxLayout()
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("font-size: 18pt; color: #2ECC71; padding: 0px;")  # Green dot
        indicator_row.addWidget(self.status_indicator)
        self.status_text = QLabel("Connected")
        self.status_text.setStyleSheet("font-size: 12pt; font-weight: bold; color: #2ECC71; padding: 3px;")
        indicator_row.addWidget(self.status_text)
        indicator_row.addStretch()
        status_layout.addLayout(indicator_row)

        # Status info display
        self.status_ssid_label = QLabel("SSID: --")
        self.status_ssid_label.setStyleSheet("font-size: 11pt; padding: 3px;")
        status_layout.addWidget(self.status_ssid_label)

        self.status_ip_label = QLabel("IP Address: --")
        self.status_ip_label.setStyleSheet("font-size: 11pt; padding: 3px;")
        status_layout.addWidget(self.status_ip_label)

        self.status_mac_label = QLabel("MAC Address: --")
        self.status_mac_label.setStyleSheet("font-size: 10pt; color: #95A5A6; padding: 3px;")
        status_layout.addWidget(self.status_mac_label)

        self.status_rssi_label = QLabel("Signal Strength: --")
        self.status_rssi_label.setStyleSheet("font-size: 10pt; padding: 3px;")
        status_layout.addWidget(self.status_rssi_label)

        self.status_group.setLayout(status_layout)
        layout.addWidget(self.status_group)

        # === WiFi Configuration Group (shown when not connected) ===
        self.config_group = QGroupBox("WiFi Configuration")
        cred_layout = QVBoxLayout()

        ssid_row = QHBoxLayout()
        ssid_label = QLabel("SSID:")
        ssid_label.setFixedWidth(80)
        ssid_row.addWidget(ssid_label)
        self.ssid_input = QLineEdit("ExoPulse")
        ssid_row.addWidget(self.ssid_input)
        cred_layout.addLayout(ssid_row)

        pass_row = QHBoxLayout()
        pass_label = QLabel("Password:")
        pass_label.setFixedWidth(80)
        pass_row.addWidget(pass_label)
        self.password_input = QLineEdit("12345666")
        self.password_input.setEchoMode(QLineEdit.Password)
        pass_row.addWidget(self.password_input)
        cred_layout.addLayout(pass_row)

        # IP address input (for testing without serial)
        ip_row = QHBoxLayout()
        ip_label = QLabel("ESP32 IP:")
        ip_label.setFixedWidth(80)
        ip_row.addWidget(ip_label)
        self.ip_input = QLineEdit("10.16.241.20")
        self.ip_input.setPlaceholderText("e.g. 192.168.1.100")
        self.ip_input.textChanged.connect(self._on_ip_changed)
        ip_row.addWidget(self.ip_input)
        cred_layout.addLayout(ip_row)

        self.config_group.setLayout(cred_layout)
        layout.addWidget(self.config_group)

        # Log display
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #34495E; color: #ECF0F1; font-family: monospace; border: 1px solid #566573; border-radius: 3px;")
        layout.addWidget(self.log_text)

        # === Action Buttons ===
        button_layout = QHBoxLayout()

        # Disconnect button (shown when connected)
        self.disconnect_btn = QPushButton("🔌 Disconnect")
        self.disconnect_btn.setMinimumHeight(40)
        self.disconnect_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E67E22, stop:1 #D35400);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #F39C12, stop:1 #E67E22);
            }
        """)
        self.disconnect_btn.clicked.connect(self._start_disconnect)
        button_layout.addWidget(self.disconnect_btn)

        # Configure button (shown when not connected)
        self.configure_btn = QPushButton("⚙ Configure WiFi")
        self.configure_btn.setMinimumHeight(40)
        self.configure_btn.clicked.connect(self._start_configuration)
        button_layout.addWidget(self.configure_btn)

        # Reconfigure button (shown when connected)
        self.reconfig_btn = QPushButton("🔄 Reconfigure")
        self.reconfig_btn.setMinimumHeight(40)
        self.reconfig_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #F39C12, stop:1 #E67E22);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #F5AB35, stop:1 #F39C12);
            }
        """)
        self.reconfig_btn.clicked.connect(self._start_reconfiguration)
        button_layout.addWidget(self.reconfig_btn)

        # Test button
        self.test_btn = QPushButton("🔍 Test Connection")
        self.test_btn.setMinimumHeight(40)
        self.test_btn.setEnabled(False)
        self.test_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #27AE60, stop:1 #229954);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2ECC71, stop:1 #27AE60);
            }
            QPushButton:disabled {
                background-color: #7F8C8D;
                color: #BDC3C7;
            }
        """)
        self.test_btn.clicked.connect(self._test_connection)
        button_layout.addWidget(self.test_btn)

        # Close button
        self.close_btn = QPushButton("✕ Close")
        self.close_btn.setMinimumHeight(40)
        self.close_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:1 #C0392B);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #EC7063, stop:1 #E74C3C);
            }
        """)
        self.close_btn.clicked.connect(self._on_close)
        button_layout.addWidget(self.close_btn)

        layout.addLayout(button_layout)

        # Initially hide all - will be shown after status check
        self.status_group.hide()
        self.config_group.hide()
        self.disconnect_btn.hide()
        self.configure_btn.hide()
        self.reconfig_btn.hide()

    def showEvent(self, event):
        """Called when dialog is shown - check WiFi status first"""
        super().showEvent(event)
        # Check status on first show
        import threading
        self._log("🔍 Checking current WiFi status...")
        threading.Thread(target=self._check_wifi_status, daemon=True).start()

    def _log(self, message):
        """Add log message (thread-safe)"""
        from PySide6.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(
            self.log_text,
            "append",
            Qt.QueuedConnection,
            Q_ARG(str, message)
        )

    def _check_wifi_status(self):
        """Check current WiFi status via serial"""
        import serial as ser_module
        import time
        import re

        # Check if serial port is valid
        if not self.serial_port or self.serial_port.startswith("No "):
            self._log("ℹ No serial port connected")
            self._log("  Connect ESP32 via USB to check WiFi status")
            self.wifi_connected = False
            self.no_serial_port = True
            self._update_ui_for_status()
            return

        self.no_serial_port = False

        try:
            # Open serial port
            self._log(f"Opening {self.serial_port}...")
            self.ser = ser_module.Serial(self.serial_port, 115200, timeout=2)
            time.sleep(0.5)

            # Clear input buffer
            self.ser.reset_input_buffer()
            self._log("Sending WIFI_STATUS command...")

            # Send WIFI_STATUS command
            self.ser.write(b"WIFI_STATUS\n")
            self.ser.flush()

            # Read response
            start_time = time.time()
            status_connected = False
            ssid = ""
            ip_address = ""
            mac_address = ""
            rssi = ""

            while time.time() - start_time < 3:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        # Check for connection status
                        if "Status: Connected" in line or "WiFi] Status: Connected" in line:
                            status_connected = True
                        elif "Status: Not connected" in line or "WiFi] Status: Not connected" in line:
                            status_connected = False

                        # Parse SSID
                        if "SSID:" in line:
                            match = re.search(r'SSID:\s*(\S+)', line)
                            if match:
                                ssid = match.group(1)

                        # Parse IP address
                        if "IP Address:" in line or "IP:" in line:
                            match = re.search(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', line)
                            if match:
                                ip_address = match.group(1)

                        # Parse MAC address
                        if "MAC Address:" in line:
                            match = re.search(r'MAC Address:\s*([0-9A-Fa-f:]+)', line)
                            if match:
                                mac_address = match.group(1)

                        # Parse RSSI
                        if "RSSI:" in line or "Signal Strength" in line:
                            match = re.search(r'(-?\d+)\s*dBm', line)
                            if match:
                                rssi = match.group(1) + " dBm"

            self.ser.close()

            # Update internal state
            self.wifi_connected = status_connected
            self.wifi_ssid = ssid
            self.esp32_ip = ip_address if ip_address else None
            self.wifi_mac = mac_address
            self.wifi_rssi = rssi

            # Log results
            if status_connected:
                self._log(f"✓ WiFi is connected")
                self._log(f"  SSID: {ssid}")
                self._log(f"  IP: {ip_address}")
                self._log(f"  RSSI: {rssi}")
            else:
                self._log("ℹ WiFi is not connected")

            # Update UI based on status
            self._update_ui_for_status()

        except Exception as e:
            self._log(f"✗ Error checking status: {e}")
            # Assume not connected on error
            self.wifi_connected = False
            self._update_ui_for_status()

    def _update_ui_for_status(self):
        """Update UI based on WiFi connection status (thread-safe)"""
        from PySide6.QtCore import QMetaObject, Q_ARG

        if self.wifi_connected:
            # Show status group, hide config group
            QMetaObject.invokeMethod(self.status_group, "show", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.config_group, "hide", Qt.QueuedConnection)

            # Update status labels
            QMetaObject.invokeMethod(
                self.status_ssid_label, "setText", Qt.QueuedConnection,
                Q_ARG(str, f"SSID: {self.wifi_ssid}")
            )
            QMetaObject.invokeMethod(
                self.status_ip_label, "setText", Qt.QueuedConnection,
                Q_ARG(str, f"IP Address: {self.esp32_ip}")
            )
            QMetaObject.invokeMethod(
                self.status_mac_label, "setText", Qt.QueuedConnection,
                Q_ARG(str, f"MAC Address: {self.wifi_mac}")
            )
            QMetaObject.invokeMethod(
                self.status_rssi_label, "setText", Qt.QueuedConnection,
                Q_ARG(str, f"Signal Strength: {self.wifi_rssi}")
            )

            # Show disconnect and reconfigure buttons, hide configure button
            QMetaObject.invokeMethod(self.disconnect_btn, "show", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.reconfig_btn, "show", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.configure_btn, "hide", Qt.QueuedConnection)

            # Enable test button
            QMetaObject.invokeMethod(
                self.test_btn, "setEnabled", Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

        else:
            # Show config group, hide status group
            QMetaObject.invokeMethod(self.status_group, "hide", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.config_group, "show", Qt.QueuedConnection)

            # Show configure button, hide disconnect and reconfigure buttons
            QMetaObject.invokeMethod(self.configure_btn, "show", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.disconnect_btn, "hide", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self.reconfig_btn, "hide", Qt.QueuedConnection)

            # Disable test button
            QMetaObject.invokeMethod(
                self.test_btn, "setEnabled", Qt.QueuedConnection,
                Q_ARG(bool, False)
            )

            # If no serial port, disable configure button but enable test if we have IP
            if hasattr(self, 'no_serial_port') and self.no_serial_port:
                QMetaObject.invokeMethod(
                    self.configure_btn, "setEnabled", Qt.QueuedConnection,
                    Q_ARG(bool, False)
                )
                # Enable test button if we have a valid IP from input or saved
                import re
                ip_pattern = r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$'
                input_ip = self.ip_input.text().strip() if hasattr(self, 'ip_input') else ""
                has_valid_ip = bool(re.match(ip_pattern, input_ip))
                if has_valid_ip:
                    self.esp32_ip = input_ip
                QMetaObject.invokeMethod(
                    self.test_btn, "setEnabled", Qt.QueuedConnection,
                    Q_ARG(bool, has_valid_ip)
                )
            else:
                QMetaObject.invokeMethod(
                    self.configure_btn, "setEnabled", Qt.QueuedConnection,
                    Q_ARG(bool, True)
                )

    def _on_ip_changed(self, text):
        """Handle IP input change - enable test button if valid IP entered"""
        import re
        # Check if it looks like a valid IP
        ip_pattern = r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$'
        is_valid_ip = bool(re.match(ip_pattern, text.strip()))

        if is_valid_ip:
            self.esp32_ip = text.strip()
            # Enable test button when we have a valid IP
            if hasattr(self, 'no_serial_port') and self.no_serial_port:
                self.test_btn.setEnabled(True)
        else:
            if hasattr(self, 'no_serial_port') and self.no_serial_port:
                self.test_btn.setEnabled(False)

    def _start_disconnect(self):
        """Start WiFi disconnection"""
        import threading
        self.disconnect_btn.setEnabled(False)
        self._log("Disconnecting WiFi...")
        threading.Thread(target=self._disconnect_wifi, daemon=True).start()

    def _disconnect_wifi(self):
        """Disconnect from WiFi (runs in thread)"""
        import serial as ser_module
        import time

        # Check if serial port is valid
        if not self.serial_port or self.serial_port.startswith("No "):
            self._log("✗ No serial port available")
            return

        try:
            # Open serial port
            self._log(f"Opening {self.serial_port}...")
            self.ser = ser_module.Serial(self.serial_port, 115200, timeout=2)
            time.sleep(0.5)

            # Clear input buffer
            self.ser.reset_input_buffer()
            self._log("Sending WIFI_DISCONNECT command...")

            # Send WIFI_DISCONNECT command
            self.ser.write(b"WIFI_DISCONNECT\n")
            self.ser.flush()

            # Read response
            start_time = time.time()
            disconnect_ok = False

            while time.time() - start_time < 3:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        # Log WiFi-related messages
                        if any(keyword in line for keyword in ['WiFi', 'WIFI', 'CMD', 'disconnect', 'Disconnect']):
                            self._log(line)
                            if "disconnected" in line.lower() or "OK" in line:
                                disconnect_ok = True

            self.ser.close()

            if disconnect_ok:
                self._log("✓ WiFi disconnected successfully")
            else:
                self._log("ℹ WiFi disconnect command sent")

            # Update state
            self.wifi_connected = False
            self.wifi_ssid = ""
            self.esp32_ip = None
            self.wifi_mac = ""
            self.wifi_rssi = ""

            # Update UI
            self._update_ui_for_status()

            # Re-enable button
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.disconnect_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

        except Exception as e:
            self._log(f"✗ Error disconnecting: {e}")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.disconnect_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

    def _start_reconfiguration(self):
        """Start reconfiguration (disconnect then configure)"""
        import threading
        self.reconfig_btn.setEnabled(False)
        self._log("Reconfiguring WiFi (will disconnect first)...")
        threading.Thread(target=self._reconfigure_wifi, daemon=True).start()

    def _reconfigure_wifi(self):
        """Reconfigure WiFi by disconnecting then showing config UI"""
        import time

        # First disconnect
        self._disconnect_wifi()
        time.sleep(1)

        # Switch UI to configuration mode
        self.wifi_connected = False
        self._update_ui_for_status()
        self._log("ℹ Ready to configure new WiFi network")

        # Re-enable button
        from PySide6.QtCore import QMetaObject, Q_ARG
        QMetaObject.invokeMethod(
            self.reconfig_btn,
            "setEnabled",
            Qt.QueuedConnection,
            Q_ARG(bool, True)
        )

    def _start_configuration(self):
        """Start WiFi configuration"""
        import threading
        self.configure_btn.setEnabled(False)
        self._log("Starting WiFi configuration...")
        threading.Thread(target=self._configure_wifi, daemon=True).start()

    def _configure_wifi(self):
        """Configure ESP32 WiFi (runs in thread)"""
        import serial as ser_module
        import time

        # Check if serial port is valid
        if not self.serial_port or self.serial_port.startswith("No "):
            self._log("✗ No serial port available")
            self._log("  Connect ESP32 via USB to configure WiFi")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.configure_btn, "setEnabled", Qt.QueuedConnection,
                Q_ARG(bool, True)
            )
            return

        ssid = self.ssid_input.text()
        password = self.password_input.text()

        try:
            # Open serial port
            self._log(f"Opening {self.serial_port}...")
            self.ser = ser_module.Serial(self.serial_port, 115200, timeout=2)
            time.sleep(0.5)

            # Clear input buffer to avoid reading old motor data
            self.ser.reset_input_buffer()
            self._log("Cleared serial buffer")

            # Send WiFi config
            self._log(f"Sending: WIFI_CONFIG {ssid} ****")
            self.ser.write(f"WIFI_CONFIG {ssid} {password}\n".encode())
            self.ser.flush()

            # Read response
            self._log("Waiting for ESP32 response...")
            start_time = time.time()
            ip_address = None

            while time.time() - start_time < 20:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        # Only log WiFi-related messages (filter out motor data)
                        if any(keyword in line for keyword in ['WiFi', 'wifi', 'WIFI', 'IP:', 'CMD', 'Connecting', 'Connected']):
                            self._log(line)

                        # Check for IP (matches both "IP:" and "IP Address:")
                        if "IP" in line and any(char.isdigit() for char in line):
                            import re
                            match = re.search(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', line)
                            if match:
                                ip_address = match.group(1)
                                self._log(f"✓ Found IP: {ip_address}")
                                break

            self.ser.close()

            if ip_address:
                self._log("✓ WiFi configuration successful!")
                self.esp32_ip = ip_address
                self.wifi_ssid = ssid
                self.wifi_connected = True

                # Re-enable configure button
                from PySide6.QtCore import QMetaObject, Q_ARG
                QMetaObject.invokeMethod(
                    self.configure_btn,
                    "setEnabled",
                    Qt.QueuedConnection,
                    Q_ARG(bool, True)
                )

                # Check full WiFi status to get MAC, RSSI, etc.
                self._log("Fetching complete WiFi status...")
                time.sleep(1)
                self._check_wifi_status()

            else:
                self._log("✗ Failed to get IP address")
                QMetaObject.invokeMethod(
                    self.configure_btn,
                    "setEnabled",
                    Qt.QueuedConnection,
                    Q_ARG(bool, True)
                )

        except Exception as e:
            self._log(f"✗ Error: {e}")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.configure_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

    def _on_close(self):
        """Handle close button - emit signal if configured successfully"""
        if self.esp32_ip:
            # WiFi was configured successfully, notify main GUI
            self.config_complete.emit(self.esp32_ip, self.esp32_port)
        self.close()

    def _test_connection(self):
        """Test WiFi connection by sending WIFI_STATUS command"""
        import threading
        self._log("🔍 Testing connection...")
        self.test_btn.setEnabled(False)
        threading.Thread(target=self._test_connection_thread, daemon=True).start()

    def _test_connection_thread(self):
        """Test connection in background thread"""
        import socket
        import time

        try:
            if not self.esp32_ip:
                self._log("✗ No IP address configured")
                return

            # Try to connect via TCP
            self._log(f"Connecting to {self.esp32_ip}:{self.esp32_port}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.esp32_ip, self.esp32_port))
            self._log("✓ TCP connection established")

            # Send WIFI_STATUS command
            self._log("Sending WIFI_STATUS command...")
            sock.send(b"WIFI_STATUS\n")
            time.sleep(0.5)

            # Receive response
            self._log("Receiving status...")
            sock.settimeout(3)
            response = b""
            start_time = time.time()

            while time.time() - start_time < 3:
                try:
                    chunk = sock.recv(1024)
                    if chunk:
                        response += chunk
                    else:
                        break
                except socket.timeout:
                    break

            sock.close()

            if response:
                # Parse and display response
                lines = response.decode('utf-8', errors='ignore').split('\n')
                self._log("─" * 50)
                for line in lines:
                    line = line.strip()
                    if line and not line.startswith('['):
                        # Filter out motor data, only show WiFi status
                        if any(keyword in line for keyword in ['WiFi', 'Status', 'SSID', 'IP', 'MAC', 'RSSI', 'TCP', 'Client', 'Connection', '=====']):
                            self._log(line)
                self._log("─" * 50)
                self._log("✓ Connection test successful!")
                self._log("🔄 Switching to WiFi mode...")

                # Emit signal to switch main GUI to WiFi mode
                self.config_complete.emit(self.esp32_ip, self.esp32_port)

            else:
                self._log("⚠ No response received from ESP32")

            # Re-enable test button
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.test_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

        except socket.timeout:
            self._log("✗ Connection timeout - ESP32 not responding")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.test_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )
        except ConnectionRefusedError:
            self._log("✗ Connection refused - TCP server not running on ESP32")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.test_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )
        except Exception as e:
            self._log(f"✗ Connection test failed: {e}")
            from PySide6.QtCore import QMetaObject, Q_ARG
            QMetaObject.invokeMethod(
                self.test_btn,
                "setEnabled",
                Qt.QueuedConnection,
                Q_ARG(bool, True)
            )

##############################################################################
# Main Enhanced GUI
##############################################################################
class ExoPulseGUI(QMainWindow):
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
        self.emg_process = None
        self.demo_thread = None
        self.demo_stop_event = threading.Event()

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

        # Debug windows
        self.serial_reader_window = None
        self.wifi_monitor_window = None
        self.can_plotter_window = None

        # Motor mode (CAN or Lower)
        self.motor_mode = "can"  # Default to CAN

        # Connection status
        self.connection_status = "Disconnected"
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.serial_lock = threading.Lock()

        # Bidirectional transmission test
        self.test_running = False
        self.test_start_time = 0  # Test start timestamp (ms)
        self.ping_seq = 0
        self.ping_sent_times = {}  # Map seq -> sent timestamp (ms)
        self.latency_history = deque(maxlen=10)  # Last 10 RTT measurements
        self.current_rtt = 0

        # Packet loss tracking (per motor)
        self.last_motor1_seq = None
        self.last_motor2_seq = None
        self.motor1_packets_received = 0
        self.motor2_packets_received = 0
        self.motor1_packets_lost = 0
        self.motor2_packets_lost = 0

        # Display settings - Default: current, angle, acceleration only
        self.visible_plots = {
            'temp': False,
            'current': True,
            'speed': False,
            'acceleration': True,
            'angle': True
        }

        self.start_time = time.time()
        self.frame_count = 0

        # Filter initial unstable data (skip first N packets per motor)
        self.packets_received = {1: 0, 2: 0}  # Track packets per motor
        self.skip_initial_packets = 10  # Skip first 10 packets to filter startup glitches

        # X-axis time window for plots (seconds)
        self.plot_time_window = 10  # Default: 10 seconds

        # Note: 新版 firmware 直接輸出度數，不需要角度展開處理
        # (保留變數以相容舊程式碼，但不再使用)
        # self.prev_raw_angle = {1: None, 2: None}
        # self.angle_turns = {1: 0, 2: 0}

        # Build UI
        self.setWindowTitle("ExoPulse Enhanced Controller")
        self.setup_ui()

        # Start with UART mode
        self.current_mode = "uart"
        self._start_uart_listener()

        # Start data pump timer
        self.timer = QTimer()
        self.timer.timeout.connect(self._pump_data_queue)
        self.timer.start(int(1000 / self.PLOT_HZ))

        # PING timer for bidirectional test (controlled by _toggle_test)
        self.ping_timer = QTimer()
        self.ping_timer.timeout.connect(self._send_ping)

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

        # Log display (initially hidden)
        self.log_label = QLabel("System Log:")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet("font-family: Consolas; font-size: 9pt;")
        self.log_label.setVisible(False)
        self.log_text.setVisible(False)
        right_layout.addWidget(self.log_label)
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

        # Status bar with Debug Tools button
        status_bar = QWidget()
        status_bar.setFixedHeight(28)
        status_bar.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1a252f, stop:1 #2C3E50);
                border-top: 1px solid #34495E;
            }
        """)
        status_layout = QHBoxLayout(status_bar)
        status_layout.setContentsMargins(10, 3, 10, 3)

        # X-axis time window control
        time_window_label = QLabel("Time Window:")
        time_window_label.setStyleSheet("color: #ECF0F1; font-size: 9pt;")
        status_layout.addWidget(time_window_label)

        self.time_window_spinbox = QSpinBox()
        self.time_window_spinbox.setMinimum(5)
        self.time_window_spinbox.setMaximum(60)
        self.time_window_spinbox.setValue(10)
        self.time_window_spinbox.setSuffix(" s")
        self.time_window_spinbox.setFixedWidth(70)
        self.time_window_spinbox.setFixedHeight(22)
        self.time_window_spinbox.setStyleSheet("""
            QSpinBox {
                background-color: #34495E;
                color: #ECF0F1;
                border: 1px solid #566573;
                border-radius: 3px;
                padding: 2px 5px;
                font-size: 9pt;
            }
            QSpinBox::up-button, QSpinBox::down-button {
                background-color: #566573;
                border: none;
                width: 16px;
            }
            QSpinBox::up-button:hover, QSpinBox::down-button:hover {
                background-color: #7F8C8D;
            }
        """)
        self.time_window_spinbox.valueChanged.connect(self._on_time_window_change)
        status_layout.addWidget(self.time_window_spinbox)

        status_layout.addStretch()

        # Debug Tools button
        self.debug_btn = QPushButton("🔧 Debug Tools")
        self.debug_btn.setFixedHeight(22)
        self.debug_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #7F8C8D, stop:1 #566573);
                color: white;
                border: none;
                border-radius: 3px;
                padding: 3px 12px;
                font-size: 9pt;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #95A5A6, stop:1 #7F8C8D);
            }
        """)
        self.debug_btn.clicked.connect(self._show_debug_menu)
        status_layout.addWidget(self.debug_btn)
        
        layout.addWidget(status_bar)

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

        # === Communication Status Bar ===
        comm_group = QGroupBox("Communication")
        comm_layout = QVBoxLayout()

        # Status display (read-only, shows current mode)
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Mode:"))
        self.comm_status_label = QLabel("UART")
        self.comm_status_label.setStyleSheet("""
            QLabel {
                background-color: #2C3E50;
                color: #ECF0F1;
                padding: 5px 10px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        status_row.addWidget(self.comm_status_label)
        status_row.addStretch()
        comm_layout.addLayout(status_row)

        # Connection info display (for WiFi IP, etc.)
        info_row = QHBoxLayout()
        info_row.addWidget(QLabel("Info:"))
        self.comm_info_label = QLabel("--")
        self.comm_info_label.setStyleSheet("""
            QLabel {
                color: #95A5A6;
                font-family: monospace;
                padding: 2px 5px;
            }
        """)
        info_row.addWidget(self.comm_info_label)
        info_row.addStretch()
        comm_layout.addLayout(info_row)

        # Auto-detect COM port for UART with dropdown selector
        if serial and list_ports:
            all_ports = [p.device for p in list_ports.comports()]
            # Look for both ttyUSB and ttyACM devices
            ports = [p for p in all_ports if 'ttyUSB' in p or 'ttyACM' in p]
            if not ports:
                ports = ["No USB/ACM port"]
        else:
            ports = ["No USB/ACM port"]

        # Create port dropdown
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Port:"))
        self.combo_com = QComboBox()
        self.combo_com.addItems(ports)
        if ports and not ports[0].startswith("No "):
            self.combo_com.setCurrentIndex(0)
        port_row.addWidget(self.combo_com)
        comm_layout.addLayout(port_row)

        # Store selected port
        self.detected_uart_port = self.combo_com.currentText()

        # WiFi Configuration button
        self.btn_wifi_config = QPushButton("📡 WiFi Configuration")
        self.btn_wifi_config.setMinimumHeight(35)
        self.btn_wifi_config.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                color: white;
                border: none;
                border-radius: 5px;
                font-weight: bold;
                font-size: 10pt;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #5DADE2, stop:1 #3498DB);
            }
        """)
        self.btn_wifi_config.clicked.connect(self._launch_wifi_config)
        comm_layout.addWidget(self.btn_wifi_config)

        comm_group.setLayout(comm_layout)
        scroll_layout.addWidget(comm_group)

        # === Display Options Group ===
        display_group = QGroupBox("Display Options")
        display_layout = QVBoxLayout()

        display_layout.addWidget(QLabel("Show Data:"))
        self.check_temp = QCheckBox("Temperature")
        self.check_temp.setChecked(False)  # Default: hidden
        self.check_temp.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_temp)

        self.check_current = QCheckBox("Current")
        self.check_current.setChecked(True)  # Default: visible
        self.check_current.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_current)

        self.check_speed = QCheckBox("Speed")
        self.check_speed.setChecked(False)  # Default: hidden
        self.check_speed.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_speed)

        self.check_accel = QCheckBox("Acceleration")
        self.check_accel.setChecked(True)  # Default: visible
        self.check_accel.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_accel)

        self.check_angle = QCheckBox("Angle")
        self.check_angle.setChecked(True)  # Default: visible
        self.check_angle.stateChanged.connect(self._on_display_change)
        display_layout.addWidget(self.check_angle)

        display_layout.addWidget(QLabel(""))  # Separator
        self.check_log = QCheckBox("System Log")
        self.check_log.setChecked(False)  # Default: hidden
        self.check_log.stateChanged.connect(self._on_log_display_change)
        display_layout.addWidget(self.check_log)

        self.btn_imu = QPushButton("Show IMU Window")
        self.btn_imu.clicked.connect(self._toggle_imu_window)
        display_layout.addWidget(self.btn_imu)

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

    def _on_log_display_change(self):
        """Toggle system log visibility"""
        visible = self.check_log.isChecked()
        self.log_label.setVisible(visible)
        self.log_text.setVisible(visible)

    def _on_time_window_change(self, value):
        """Handle time window change"""
        self.plot_time_window = value

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
            ('temp', 'Temperature', '°C', 'cyan', 'orange'),
            ('current', 'Current', 'A', 'cyan', 'orange'),
            ('speed', 'Speed', 'rad/s', 'cyan', 'orange'),
            ('acceleration', 'Acceleration', 'dps²', 'cyan', 'orange'),
            ('angle', 'Angle', '°', 'cyan', 'orange'),
        ]

        for key, title, ylabel, color1, color2 in plot_configs:
            if not self.visible_plots[key]:
                continue

            # Motor 1 (left)
            ax1 = self.fig.add_subplot(gs[row, 0])
            ax1.set_title(f'Motor 1 - {title}', color=color1, fontweight='bold')
            ax1.set_ylabel(ylabel, color=color1)
            ax1.set_autoscale_on(True)  # Enable autoscale
            ax1.grid(True, alpha=0.3)
            line1, = ax1.plot([], [], color=color1, lw=2)

            self.axes[f'm1_{key}'] = ax1
            self.lines[f'm1_{key}'] = line1

            # Motor 2 (right)
            ax2 = self.fig.add_subplot(gs[row, 1])
            ax2.set_title(f'Motor 2 - {title}', color=color2, fontweight='bold')
            ax2.set_ylabel(ylabel, color=color2)
            ax2.set_autoscale_on(True)  # Enable autoscale
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
        # Use dynamic time window from spinbox
        window_size = self.plot_time_window

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

                    # Update x-axis with dynamic time window
                    ax = self.axes[line_key]
                    ax.set_xlim(max(0, t[-1] - window_size), t[-1] + 1)

                    # Auto-scale y-axis for ALL data types based on actual values
                    if len(values) > 0:
                        val_min, val_max = min(values), max(values)
                        val_range = val_max - val_min

                        if val_range > 0.01:
                            # 自動縮放 (適用於所有圖表，包括 angle)
                            margin = val_range * 0.2 + (5 if key == 'acceleration' else 0.1)
                            ax.set_ylim(val_min - margin, val_max + margin)
                        else:
                            # For stable values, show small range around the value
                            center = (val_min + val_max) / 2
                            offset = 1 if key == 'acceleration' else 0.1
                            ax.set_ylim(center - offset, center + offset)

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
    def _auto_calibrate(self):
        """Auto-calibrate both motors after connection"""
        time.sleep(0.5)  # Wait for connection to stabilize
        self._log("⚙️ Auto-calibrating motors...")
        self._calibrate_motor(0)  # Calibrate both motors

    def _calibrate_motor(self, motor_id):
        """Calibrate motor zero position"""
        # 重置包計數器 (新版 firmware 直接處理角度，不需重置角度展開狀態)
        if motor_id == 1:
            self.packets_received[1] = 0
        elif motor_id == 2:
            self.packets_received[2] = 0
        elif motor_id == 0:
            self.packets_received = {1: 0, 2: 0}

        # 嘗試發送校準命令到 ESP32
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    # Serial 模式
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
                elif self.tcp_sock:
                    # WiFi 模式 - 通過 TCP 發送
                    if motor_id == 1:
                        self.tcp_sock.send(b"CAL1\n")
                        self._log("✓ Calibrating Motor 1 via WiFi...")
                    elif motor_id == 2:
                        self.tcp_sock.send(b"CAL2\n")
                        self._log("✓ Calibrating Motor 2 via WiFi...")
                    elif motor_id == 0:
                        self.tcp_sock.send(b"CAL\n")
                        self._log("✓ Calibrating BOTH motors via WiFi...")
                else:
                    self._log("⚠ No connection, local angle reset only")
        except Exception as e:
            self._log(f"⚠ Calibration command failed: {e}")

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

    # ========== Debug Tools Menu ==========
    def _show_debug_menu(self):
        """Show debug tools menu in center"""
        from PySide6.QtWidgets import QDialog, QVBoxLayout, QPushButton, QLabel
        from PySide6.QtCore import Qt
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Debug Tools")
        dialog.setModal(True)
        dialog.setFixedSize(300, 200)
        dialog.setStyleSheet("""
            QDialog {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #2C3E50, stop:1 #34495E);
            }
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
                font-size: 11pt;
                text-align: left;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #5DADE2, stop:1 #3498DB);
            }
            QLabel {
                color: #ECF0F1;
                font-size: 12pt;
                font-weight: bold;
            }
        """)
        
        layout = QVBoxLayout()
        layout.setSpacing(10)
        
        title = QLabel("Select Debug Tool:")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Serial Plotter
        btn_serial_plot = QPushButton("📊  Serial Plotter")
        btn_serial_plot.clicked.connect(lambda: (self._launch_serial_plotter(), dialog.accept()))
        layout.addWidget(btn_serial_plot)

        # WiFi Monitor
        btn_wifi_monitor = QPushButton("📡  WiFi Monitor")
        btn_wifi_monitor.clicked.connect(lambda: (self._launch_wifi_monitor(), dialog.accept()))
        layout.addWidget(btn_wifi_monitor)

        dialog.setLayout(layout)
        dialog.exec()

    def _launch_serial_plotter(self):
        """Launch Serial Plotter with current communication settings"""
        import subprocess
        port = self.detected_uart_port if hasattr(self, 'detected_uart_port') else '/dev/ttyACM0'
        script = Path(__file__).parent / 'serial_plotter.py'
        if script.exists():
            # Launch in new terminal window
            subprocess.Popen(['x-terminal-emulator', '-e', sys.executable, str(script), '--port', port],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            QMessageBox.warning(self, "Error", "serial_plotter.py not found")

    def _launch_wifi_monitor(self):
        """Launch WiFi Monitor window"""
        # Check if we have ESP32 IP
        if not hasattr(self, 'tcp_sock') or self.tcp_sock is None:
            QMessageBox.warning(
                self,
                "WiFi Not Connected",
                "Please configure WiFi first using the 'WiFi Configuration' button."
            )
            return

        # Import WiFiSignalMonitor from auto_wifi_setup
        try:
            from auto_wifi_setup import WiFiSignalMonitor
            global ESP32_IP, ESP32_PORT

            # Create and show WiFi monitor window
            if not hasattr(self, 'wifi_monitor') or self.wifi_monitor is None:
                self.wifi_monitor = WiFiSignalMonitor(esp32_ip=ESP32_IP, esp32_port=ESP32_PORT)

            self.wifi_monitor.show()
            self._log("✓ WiFi Monitor opened")

        except ImportError as e:
            QMessageBox.warning(self, "Error", f"Failed to import WiFi Monitor: {e}")

    def _launch_wifi_config(self):
        """Launch WiFi Configuration as integrated dialog"""
        self._log("📡 Opening WiFi Configuration...")

        # Clean up existing connections based on current mode
        if self.current_mode == "uart":
            # Stop UART to free serial port
            self.stop_uart_evt.set()
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    self._log("🔌 Temporarily closed UART for WiFi configuration")
                except:
                    pass
        elif self.current_mode == "wifi":
            # Stop existing WiFi connections to avoid resource conflicts
            self._log("🔌 Closing existing WiFi connections for reconfiguration...")

            # Stop UDP/TCP listeners
            self.stop_udp_evt.set()

            # Close TCP socket
            if self.tcp_sock:
                try:
                    self.tcp_sock.close()
                    self._log("✓ Closed TCP connection")
                except:
                    pass
                self.tcp_sock = None

            # Give listeners time to stop
            import time
            time.sleep(0.5)

        # Get current port
        port = self.combo_com.currentText() if hasattr(self, 'combo_com') else self.detected_uart_port

        # Create and show dialog
        self.wifi_dialog = WiFiConfigDialog(self, serial_port=port)
        self.wifi_dialog.config_complete.connect(self._on_wifi_config_complete)
        self.wifi_dialog.show()

    def _on_wifi_config_complete(self, ip, port):
        """Handle successful WiFi configuration"""
        self._log(f"✓ WiFi configuration complete: {ip}:{port}")

        # Update ESP32 connection info
        global ESP32_IP, ESP32_PORT
        ESP32_IP = ip
        ESP32_PORT = port

        # Switch to WiFi mode
        self.current_mode = "wifi"
        self.stop_udp_evt.clear()

        # Update UI
        self.comm_status_label.setText("WiFi")
        self.comm_status_label.setStyleSheet("""
            QLabel {
                background-color: #27AE60;
                color: white;
                padding: 5px 10px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        self.comm_info_label.setText(f"{ip}:{port}")

        self._log(f"✓ Switched to WiFi mode: {ip}:{port}")

        # Start WiFi connections
        self._connect_tcp_async()
        self._start_udp_listener()

    # ========== Bidirectional Test Functions ==========
    def _toggle_test(self):
        """Toggle bidirectional transmission test"""
        if not self.test_running:
            # Start test
            self.test_running = True
            if hasattr(self, 'test_btn'):
                self.test_btn.setText("■ Stop Test")
                self.test_btn.setStyleSheet("""
                    QPushButton {
                        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                            stop:0 #E74C3C, stop:1 #C0392B);
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-size: 11pt;
                        font-weight: bold;
                        padding: 8px;
                    }
                    QPushButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                            stop:0 #EC7063, stop:1 #E74C3C);
                    }
                """)

            # Reset test statistics and record start time
            self.test_start_time = int(time.time() * 1000)
            self.ping_seq = 0
            self.ping_sent_times.clear()
            self.latency_history.clear()
            self.last_motor1_seq = None
            self.last_motor2_seq = None
            self.motor1_packets_received = 0
            self.motor2_packets_received = 0
            self.motor1_packets_lost = 0
            self.motor2_packets_lost = 0

            # Start PING timer (send PING every 1 second)
            self.ping_timer.start(1000)

            self._log("[TEST] Bidirectional transmission test started")
        else:
            # Stop test
            self.test_running = False
            if hasattr(self, 'test_btn'):
                self.test_btn.setText("▶ Start Test")
                self.test_btn.setStyleSheet("""
                    QPushButton {
                        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                            stop:0 #27AE60, stop:1 #229954);
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-size: 11pt;
                        font-weight: bold;
                        padding: 8px;
                    }
                    QPushButton:hover {
                        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                            stop:0 #2ECC71, stop:1 #27AE60);
                    }
                """)

            # Stop PING timer
            self.ping_timer.stop()

            self._log("[TEST] Bidirectional transmission test stopped")

    def _send_ping(self):
        """Send PING request to ESP32 for latency measurement"""
        if not self.test_running or self.current_mode != "wifi":
            return

        # Check if TCP socket is connected
        if not self.tcp_sock:
            return

        try:
            self.ping_seq += 1
            current_time_ms = int(time.time() * 1000)

            # Use relative timestamp from test start
            relative_timestamp_ms = current_time_ms - self.test_start_time
            ping_msg = f"[PING] seq:{self.ping_seq} ts:{relative_timestamp_ms}\n"

            # Record absolute sent time for RTT calculation
            self.ping_sent_times[self.ping_seq] = current_time_ms

            # Send via TCP socket
            self.tcp_sock.send(ping_msg.encode('utf-8'))
        except Exception as e:
            self._log(f"Ping send error: {e}")

    def _track_packet_loss(self, motor_id: int, seq: int):
        """
        Unified packet loss tracking for both UART and WiFi modes.
        Reuses seq from _parse_motor_line() to avoid duplicate parsing.
        """
        if motor_id == 1:
            self.motor1_packets_received += 1
            if self.last_motor1_seq is not None:
                expected_seq = (self.last_motor1_seq + 1) % (2**32)
                if seq != expected_seq:
                    if seq > expected_seq:
                        lost = seq - expected_seq
                    else:
                        lost = (2**32 - expected_seq) + seq
                    self.motor1_packets_lost += lost
            self.last_motor1_seq = seq

        elif motor_id == 2:
            self.motor2_packets_received += 1
            if self.last_motor2_seq is not None:
                expected_seq = (self.last_motor2_seq + 1) % (2**32)
                if seq != expected_seq:
                    if seq > expected_seq:
                        lost = seq - expected_seq
                    else:
                        lost = (2**32 - expected_seq) + seq
                    self.motor2_packets_lost += lost
            self.last_motor2_seq = seq

        # Update display
        self._update_packet_loss_display()

    def _update_packet_loss_display(self):
        """Update packet loss statistics display"""
        if not self.test_running:
            return

        # Check if UI elements exist
        if not hasattr(self, 'm1_loss_label'):
            return

        # Motor 1 packet loss
        m1_total = self.motor1_packets_received + self.motor1_packets_lost
        if m1_total > 0:
            m1_loss_rate = (self.motor1_packets_lost / m1_total) * 100
            self.m1_loss_label.setText(f"{m1_loss_rate:.2f}%")
            if hasattr(self, 'm1_packets_label'):
                self.m1_packets_label.setText(f"({self.motor1_packets_lost}/{m1_total})")

            # Color code based on loss rate
            if m1_loss_rate < 1.0:
                color = "#2ECC71"  # Green - Good
            elif m1_loss_rate < 5.0:
                color = "#F39C12"  # Orange - Fair
            else:
                color = "#E74C3C"  # Red - Poor
            self.m1_loss_label.setStyleSheet(f"color: {color}; font-size: 16pt; font-weight: bold;")

        # Motor 2 packet loss
        m2_total = self.motor2_packets_received + self.motor2_packets_lost
        if m2_total > 0:
            m2_loss_rate = (self.motor2_packets_lost / m2_total) * 100
            self.m2_loss_label.setText(f"{m2_loss_rate:.2f}%")
            if hasattr(self, 'm2_packets_label'):
                self.m2_packets_label.setText(f"({self.motor2_packets_lost}/{m2_total})")

            # Color code based on loss rate
            if m2_loss_rate < 1.0:
                color = "#2ECC71"  # Green - Good
            elif m2_loss_rate < 5.0:
                color = "#F39C12"  # Orange - Fair
            else:
                color = "#E74C3C"  # Red - Poor
            self.m2_loss_label.setStyleSheet(f"color: {color}; font-size: 16pt; font-weight: bold;")

    def _update_latency_display(self):
        """Update latency statistics display"""
        if not self.test_running or len(self.latency_history) == 0:
            return

        # Check if UI elements exist
        if not hasattr(self, 'latency_label'):
            return

        # Current RTT
        self.latency_label.setText(f"{self.current_rtt:.1f} ms")

        # Statistics
        avg_rtt = sum(self.latency_history) / len(self.latency_history)
        min_rtt = min(self.latency_history)
        max_rtt = max(self.latency_history)

        if hasattr(self, 'latency_avg_label'):
            self.latency_avg_label.setText(f"Avg: {avg_rtt:.1f} ms")
        if hasattr(self, 'latency_minmax_label'):
            self.latency_minmax_label.setText(f"Min: {min_rtt:.1f} ms  Max: {max_rtt:.1f} ms")

        # Color code based on latency
        if self.current_rtt < 50:
            color = "#2ECC71"  # Green - Excellent
        elif self.current_rtt < 100:
            color = "#F39C12"  # Orange - Fair
        else:
            color = "#E74C3C"  # Red - Poor
        self.latency_label.setStyleSheet(f"color: {color}; font-size: 16pt; font-weight: bold;")

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

        # Send MODE_WIFI command to switch ESP32 to WiFi output
        try:
            self.tcp_sock.send(b"MODE_WIFI\n")
            self._log("→ Sent MODE_WIFI command to ESP32")
            import time
            time.sleep(0.2)
        except Exception as e:
            self._log(f"⚠ Failed to send MODE_WIFI: {e}")

        # Auto-calibrate after connection
        if self.motor_mode == "can":
            threading.Thread(target=self._auto_calibrate, daemon=True).start()
        # Start TCP listener for receiving data
        self._start_tcp_listener()

    def _start_tcp_listener(self):
        """Start TCP listener to receive data from ESP32"""
        def listener():
            import re
            try:
                self.data_queue.put(("log", f"✓ TCP listener started"))

                while not self.stop_udp_evt.is_set() and self.tcp_sock:
                    try:
                        # Set timeout for non-blocking receive
                        self.tcp_sock.settimeout(0.1)
                        data = self.tcp_sock.recv(4096).decode('utf-8', errors='ignore')

                        if data:
                            # Put motor data into queue for normal processing
                            lines = data.split('\n')
                            for line in lines:
                                line = line.strip()
                                if line:
                                    # Put regular data into queue
                                    msg = f"[TCP] {line}"
                                    self.data_queue.put(("data", msg))

                                    # Parse PONG responses for latency measurement
                                    if self.test_running:
                                        pong_match = re.search(r'\[PONG\]\s+seq:(\d+)\s+ts_req:(\d+)\s+ts_reply:(\d+)', line)
                                        if pong_match:
                                            seq = int(pong_match.group(1))

                                            # Calculate RTT using PC-side recorded send time
                                            if seq in self.ping_sent_times:
                                                sent_time_ms = self.ping_sent_times[seq]
                                                current_time_ms = int(time.time() * 1000)
                                                rtt = current_time_ms - sent_time_ms

                                                self.current_rtt = rtt
                                                self.latency_history.append(rtt)

                                                # Update latency display
                                                self._update_latency_display()

                                    # NOTE: Packet loss tracking moved to _pump_data_queue()
                                    # to reuse _parse_motor_line() and support both UART/WiFi modes

                    except socket.timeout:
                        continue
                    except Exception as e:
                        if self.tcp_sock:
                            self.data_queue.put(("log", f"⚠ TCP recv error: {e}"))
                        break

            except Exception as e:
                self.data_queue.put(("log", f"⚠ TCP listener error: {e}"))
            finally:
                self.data_queue.put(("log", "✗ TCP listener stopped"))

        threading.Thread(target=listener, daemon=True).start()

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
                # Get port from combo box if available, otherwise use stored value
                port = self.combo_com.currentText() if hasattr(self, 'combo_com') else self.detected_uart_port
                if port.startswith("No "):
                    self.data_queue.put(("log", "⚠ No USB/ACM port available"))
                    return

                if self.ser and self.ser.is_open:
                    try:
                        self.ser.close()
                        self.data_queue.put(("log", "🔌 Closed existing UART"))
                    except Exception as e:
                        self.data_queue.put(("log", f"⚠ Close failed: {e}"))

                # Kill any processes using the port
                try:
                    import subprocess
                    result = subprocess.run(['lsof', '-t', port],
                                          capture_output=True, text=True, timeout=2)
                    if result.returncode == 0 and result.stdout.strip():
                        pids = result.stdout.strip().split('\n')
                        for pid in pids:
                            subprocess.run(['kill', '-9', pid], timeout=1)
                        self.data_queue.put(("log", f"🔧 Killed {len(pids)} process(es) using {port}"))
                        import time
                        time.sleep(0.2)  # Wait for port to be freed
                except Exception as e:
                    # Silently ignore if lsof not found or other errors
                    pass

                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.connection_status = "Connected"
                self.data_queue.put(("log", f"✓ UART on {port}"))
                # Auto-calibrate after connection
                if self.motor_mode == "can":
                    self.data_queue.put(("auto_calibrate", None))

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
    def _unwrap_angle(self, motor_id, raw_angle):
        """
        處理角度值

        新版 firmware 直接輸出度數 (°)，且已處理多圈追蹤
        raw_angle: 角度值 (單位: 度 °)
        回傳: 角度值 (單位: 度 °)

        注意: 舊版 firmware 輸出 0.1° 單位，需要 * 0.1 轉換
              新版 firmware 直接輸出度數，不需轉換
        """
        # 新版 firmware: 直接輸出度數，已包含多圈追蹤
        # 直接回傳，不需要額外處理
        return raw_angle

    def _parse_motor_line(self, line):
        """Parse CAN motor data line"""
        if not line.startswith('['):
            return None

        # Remove prefix like [UART], [WiFi], [TCP], [IMU] if present
        line = re.sub(r'^\[(UART|WiFi|TCP|IMU)[^\]]*\]\s*', '', line)

        try:
            # Try new format with SEQ field first: [timestamp] SEQ:xxx M:x T:xx ...
            match = re.match(r'\[(\d+)\]\s+SEQ:(\d+)\s+M:(\d+)\s+T:(-?\d+)\s+V:([\d.]+)\s+I:([-\d.]+)\s+S:(-?\d+)\s+ACC:(-?\d+)\s+E:(\d+)\s+A:([-\d.]+|ovf)\s+ERR:(0x[\w]+)', line)
            if match:
                angle_str = match.group(10)
                angle_val = 0.0 if angle_str == 'ovf' else float(angle_str)
                return {
                    'seq': int(match.group(2)),
                    'motor_id': int(match.group(3)),
                    'temp': int(match.group(4)),
                    'voltage': float(match.group(5)),
                    'current': float(match.group(6)),
                    'speed': int(match.group(7)),
                    'acceleration': int(match.group(8)),
                    'angle': angle_val,
                }

            # Try old format without SEQ field: [timestamp] M:x T:xx ...
            match = re.match(r'\[(\d+)\]\s+M:(\d+)\s+T:(-?\d+)\s+V:([\d.]+)\s+I:([-\d.]+)\s+S:(-?\d+)\s+ACC:(-?\d+)\s+E:(\d+)\s+A:([-\d.]+|ovf)\s+ERR:(0x[\w]+)', line)
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
                # Only log non-motor data to reduce clutter
                if not ("M:" in msg and "T:" in msg):
                    self._log(msg)

                # Try to parse as CAN motor data
                if "[" in msg:
                    status = self._parse_motor_line(msg)
                    if status:
                        # Debug: log successful parse
                        if not hasattr(self, '_parse_count'):
                            self._parse_count = 0
                            self._log("✓ Motor data parsing started")
                        motor_id = status['motor_id']
                        if motor_id in [1, 2]:
                            # Unified packet loss tracking (works for both UART and WiFi)
                            if 'seq' in status and self.test_running:
                                self._track_packet_loss(motor_id, status['seq'])

                            # Filter initial unstable packets (skip first N packets per motor)
                            self.packets_received[motor_id] += 1
                            if self.packets_received[motor_id] <= self.skip_initial_packets:
                                continue  # Skip this packet

                            elapsed = time.time() - self.start_time
                            self.motor_status[motor_id].update(status)
                            data = self.motor_data[motor_id]
                            data['time'].append(elapsed)
                            data['temp'].append(status['temp'])
                            data['current'].append(status['current'])
                            data['speed'].append(status['speed'])
                            data['acceleration'].append(status['acceleration'])
                            # 使用角度展開，轉換為度 (°)
                            unwrapped_angle = self._unwrap_angle(motor_id, status['angle'])
                            data['angle'].append(unwrapped_angle)

                # Try to parse as general data
                if "] X " in msg:
                    self._parse_general_line(msg.split("]", 1)[1].strip())

            elif msg_type == "tcp_ready":
                self._tcp_ready(msg)
            elif msg_type == "auto_calibrate":
                self._auto_calibrate()

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
        """Toggle demo mode - launches EMG plotter and simulation control"""
        if not self.in_demo:
            self._start_demo()
        else:
            self._stop_demo()

    def _start_demo(self):
        """Start demo mode with EMG plotter"""
        try:
            # Launch EMG plotter in separate process
            ui_dir = Path(__file__).parent
            emg_script = ui_dir / "emg_plotter.py"
            
            if emg_script.exists():
                # Find available serial port for EMG plotter
                import glob
                available_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
                
                if available_ports:
                    emg_port = available_ports[0]
                    self._log(f"📊 Launching EMG Plotter on {emg_port}")
                    self.emg_process = subprocess.Popen(
                        [sys.executable, str(emg_script), '--port', emg_port],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                else:
                    self._log("⚠ No serial port available for EMG Plotter")
            else:
                self._log(f"⚠ EMG script not found: {emg_script}")
            
            # Prepare for simulation control thread (待實現)
            self.demo_stop_event.clear()
            # TODO: self.demo_thread = threading.Thread(target=self._run_simulation_control, daemon=True)
            # TODO: self.demo_thread.start()
            
            self._log("🚶 Demo started")
            self.btn_demo.setText("Stop Demo")
            self.in_demo = True
            
        except Exception as e:
            self._log(f"⚠ Demo start error: {e}")
            import traceback
            self._log(f"Details: {traceback.format_exc()}")

    def _stop_demo(self):
        """Stop demo mode"""
        try:
            # Stop simulation control thread
            self.demo_stop_event.set()
            if self.demo_thread:
                self.demo_thread.join(timeout=1.0)
                self.demo_thread = None
            
            # Terminate EMG plotter
            if self.emg_process:
                self.emg_process.terminate()
                try:
                    self.emg_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.emg_process.kill()
                self.emg_process = None
                self._log("📊 EMG Plotter closed")
            
            self._send_stop()
            self._log("🛑 Demo stopped")
            self.btn_demo.setText("Start Demo")
            self.in_demo = False
            
        except Exception as e:
            self._log(f"⚠ Demo stop error: {e}")
    
    def _run_simulation_control(self):
        """Simulation control thread - 待你提供 torque/degree 控制邏輯"""
        # TODO: 在這裡實現模擬控制邏輯
        # 當你提供 torque 或 degree 控制指令後，會在這裡實現
        pass

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
# Backward compatibility alias
ExoPulseGUI = ExoPulseGUI

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = ExoPulseGUI()
    window.show()

    sys.exit(app.exec())
