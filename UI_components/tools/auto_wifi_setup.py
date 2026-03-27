#!/usr/bin/env python3
"""
Automatically configure ESP32 WiFi and launch monitor
"""

import serial
import time
import subprocess
import sys
import re
import os
import signal
import socket
import threading
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QProgressBar, QGroupBox, QPushButton, QLineEdit,
    QStackedWidget, QTextEdit
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
DEFAULT_SSID = "ExoPulse"
DEFAULT_PASSWORD = "12345666"
TCP_PORT = 8888

def kill_serial_port_users(port):
    """Kill any processes currently using the serial port"""
    print(f"\n[*] Checking for processes using {port}...")

    try:
        # Use lsof to find processes using the serial port
        result = subprocess.run(
            ['lsof', '-t', port],
            capture_output=True,
            text=True,
            timeout=2
        )

        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            print(f"[*] Found {len(pids)} process(es) using {port}")

            for pid in pids:
                try:
                    pid_int = int(pid)
                    # Get process name
                    proc_name = subprocess.run(
                        ['ps', '-p', pid, '-o', 'comm='],
                        capture_output=True,
                        text=True,
                        timeout=1
                    ).stdout.strip()

                    print(f"[*] Killing process: {proc_name} (PID: {pid})")
                    os.kill(pid_int, signal.SIGTERM)
                    time.sleep(0.5)

                    # Check if process still exists, force kill if needed
                    try:
                        os.kill(pid_int, 0)  # Check if process exists
                        print(f"[*] Process {pid} still running, force killing...")
                        os.kill(pid_int, signal.SIGKILL)
                    except ProcessLookupError:
                        pass  # Process already terminated

                except (ValueError, ProcessLookupError, PermissionError) as e:
                    print(f"[!] Warning: Could not kill PID {pid}: {e}")

            print(f"[✓] Cleared processes using {port}")
            time.sleep(1)  # Wait for port to be fully released
        else:
            print(f"[✓] No processes using {port}")

    except FileNotFoundError:
        print("[!] Warning: 'lsof' command not found, skipping process check")
    except Exception as e:
        print(f"[!] Warning: Error checking port users: {e}")

def setup_wifi(ssid=None, password=None):
    """Configure ESP32 WiFi via Serial and get IP"""
    # Use defaults if not provided
    if ssid is None:
        ssid = DEFAULT_SSID
    if password is None:
        password = DEFAULT_PASSWORD

    print("=" * 60)
    print("ESP32 WiFi Auto-Setup")
    print("=" * 60)

    try:
        # Kill any processes using the serial port
        kill_serial_port_users(SERIAL_PORT)

        # Open serial port
        print(f"\n[1] Opening {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(1)

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send WiFi config command
        print(f"\n[2] Sending: WIFI_CONFIG {ssid} {password}")
        ser.write(f"WIFI_CONFIG {ssid} {password}\n".encode())
        ser.flush()

        # Read response and find IP
        print("\n[3] Waiting for ESP32 response...\n")
        print("-" * 60)

        ip_address = None
        start_time = time.time()

        while (time.time() - start_time) < 30:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)

                    # Extract IP address
                    if "IP Address:" in line or "IP:" in line:
                        ip_match = re.search(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', line)
                        if ip_match:
                            ip_address = ip_match.group(1)

                    # Check for success
                    if "[OK] WiFi configured successfully!" in line:
                        print("-" * 60)
                        break

                    # Check for failure
                    if "[ERROR] WiFi connection failed" in line:
                        print("-" * 60)
                        print("\n✗ WiFi configuration failed!")
                        ser.close()
                        return None

        ser.close()

        if ip_address:
            print(f"\n✓ ESP32 IP Address: {ip_address}")
            print(f"✓ WiFi configured successfully!")
            print(f"   Output mode: UART (default)")
            print(f"   You can switch to WiFi mode using: MODE_WIFI command")
            return ip_address
        else:
            print("\n✗ Could not find IP address")
            return None

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return None


class WiFiConfigSignals(QObject):
    """Signals for WiFi configuration thread"""
    config_complete = Signal(str)  # IP address
    config_failed = Signal(str)  # Error message
    log_message = Signal(str)  # Log message


class WiFiSignalMonitor(QMainWindow):
    """WiFi signal strength monitoring GUI"""

    def __init__(self, esp32_ip=None, esp32_port=TCP_PORT):
        super().__init__()
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.socket = None
        self.connected = False

        # Signal strength data
        self.mcu_rssi = 0
        self.pc_rssi = 0
        self.mcu_ssid = "Unknown"
        self.pc_ssid = "Unknown"

        # Connection quality tracking (ping-like statistics)
        self.mcu_ping_history = deque(maxlen=10)  # Last 10 seconds
        self.pc_ping_history = deque(maxlen=10)   # Last 10 seconds
        self.mcu_last_ping = 0
        self.pc_last_ping = 0

        # Bidirectional transmission test
        self.test_running = False
        self.test_start_time = 0  # Test start timestamp (ms)
        self.ping_seq = 0
        self.ping_sent_times = {}  # Map seq -> sent timestamp (ms relative to test start)
        self.latency_history = deque(maxlen=10)  # Last 10 RTT measurements
        self.current_rtt = 0

        # Packet loss tracking (per motor)
        self.last_motor1_seq = None
        self.last_motor2_seq = None
        self.motor1_packets_received = 0
        self.motor2_packets_received = 0
        self.motor1_packets_lost = 0
        self.motor2_packets_lost = 0

        # WiFi configuration
        self.config_signals = WiFiConfigSignals()
        self.config_signals.config_complete.connect(self._on_config_complete)
        self.config_signals.config_failed.connect(self._on_config_failed)
        self.config_signals.log_message.connect(self._on_log_message)

        self._init_ui()

        # If no IP provided, show config page; otherwise show monitor page
        if self.esp32_ip is None:
            self.stacked_widget.setCurrentIndex(0)  # Config page
        else:
            self.stacked_widget.setCurrentIndex(1)  # Monitor page
            self._init_timers()
            self._connect_to_esp32()

    def _init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ExoPulse WiFi Signal Monitor")
        self.setGeometry(100, 100, 800, 600)

        # Create stacked widget for config and monitor pages
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)

        # Page 0: WiFi Configuration
        config_page = self._create_config_page()
        self.stacked_widget.addWidget(config_page)

        # Page 1: Monitor Page
        monitor_page = self._create_monitor_page()
        self.stacked_widget.addWidget(monitor_page)

    def _create_config_page(self):
        """Create WiFi configuration page"""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(20)

        # Background styling
        page.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #2C3E50, stop:1 #34495E);
            }
        """)

        # Title
        title = QLabel("ExoPulse WiFi Configuration")
        title.setStyleSheet("color: #ECF0F1; font-size: 24pt; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Subtitle
        subtitle = QLabel("Configure ESP32 WiFi connection")
        subtitle.setStyleSheet("color: #BDC3C7; font-size: 12pt;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        layout.addSpacing(30)

        # SSID Input
        ssid_label = QLabel("WiFi SSID:")
        ssid_label.setStyleSheet("color: #ECF0F1; font-size: 12pt; font-weight: bold;")
        layout.addWidget(ssid_label)

        self.ssid_input = QLineEdit()
        self.ssid_input.setPlaceholderText("Enter WiFi network name")
        self.ssid_input.setText(DEFAULT_SSID)
        self.ssid_input.setStyleSheet("""
            QLineEdit {
                background-color: #34495E;
                color: #ECF0F1;
                border: 2px solid #7F8C8D;
                border-radius: 5px;
                padding: 10px;
                font-size: 12pt;
            }
            QLineEdit:focus {
                border: 2px solid #3498DB;
            }
        """)
        layout.addWidget(self.ssid_input)

        layout.addSpacing(15)

        # Password Input
        password_label = QLabel("WiFi Password:")
        password_label.setStyleSheet("color: #ECF0F1; font-size: 12pt; font-weight: bold;")
        layout.addWidget(password_label)

        self.password_input = QLineEdit()
        self.password_input.setPlaceholderText("Enter WiFi password")
        self.password_input.setText(DEFAULT_PASSWORD)
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setStyleSheet("""
            QLineEdit {
                background-color: #34495E;
                color: #ECF0F1;
                border: 2px solid #7F8C8D;
                border-radius: 5px;
                padding: 10px;
                font-size: 12pt;
            }
            QLineEdit:focus {
                border: 2px solid #3498DB;
            }
        """)
        layout.addWidget(self.password_input)

        layout.addSpacing(30)

        # Connect Button
        self.connect_btn = QPushButton("Connect to WiFi")
        self.connect_btn.setMinimumHeight(60)
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #27AE60, stop:1 #229954);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2ECC71, stop:1 #27AE60);
            }
            QPushButton:disabled {
                background-color: rgba(0, 0, 0, 0.2);
                color: #7F8C8D;
            }
        """)
        self.connect_btn.clicked.connect(self._start_wifi_config)
        layout.addWidget(self.connect_btn)

        # Log Output
        log_label = QLabel("Configuration Log:")
        log_label.setStyleSheet("color: #ECF0F1; font-size: 11pt; font-weight: bold;")
        layout.addWidget(log_label)

        self.config_log = QTextEdit()
        self.config_log.setReadOnly(True)
        self.config_log.setMaximumHeight(200)
        self.config_log.setStyleSheet("""
            QTextEdit {
                background-color: #1a252f;
                color: #ECF0F1;
                border: 2px solid #34495E;
                border-radius: 5px;
                padding: 10px;
                font-family: monospace;
                font-size: 10pt;
            }
        """)
        layout.addWidget(self.config_log)

        layout.addStretch()

        return page

    def _create_monitor_page(self):
        """Create monitoring page"""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)

        # === HEADER ===
        header = QLabel("WiFi Signal Strength Monitor")
        header.setAlignment(Qt.AlignCenter)
        header_font = QFont()
        header_font.setPointSize(18)
        header_font.setBold(True)
        header.setFont(header_font)
        header.setStyleSheet("color: #ECF0F1; padding: 10px;")
        layout.addWidget(header)

        # === CONNECTION STATUS ===
        self.status_label = QLabel("Status: Connecting...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #F39C12; font-size: 14pt; font-weight: bold; padding: 10px;")
        layout.addWidget(self.status_label)

        # === MCU SIGNAL STRENGTH ===
        mcu_group = self._create_signal_group(
            "ESP32 MCU Signal",
            "MCU",
            is_mcu=True
        )
        layout.addWidget(mcu_group)

        # === PC SIGNAL STRENGTH ===
        pc_group = self._create_signal_group(
            "PC Signal",
            "PC",
            is_mcu=False
        )
        layout.addWidget(pc_group)

        # === TRANSMISSION TEST METRICS ===
        metrics_group = self._create_metrics_group()
        layout.addWidget(metrics_group)

        # === CONTROL BUTTONS ===
        button_layout = QHBoxLayout()
        button_layout.setSpacing(10)

        # Test button
        self.test_btn = QPushButton("▶ Start Test")
        self.test_btn.setMinimumHeight(40)
        self.test_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #27AE60, stop:1 #229954);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2ECC71, stop:1 #27AE60);
            }
        """)
        self.test_btn.clicked.connect(self._toggle_test)
        button_layout.addWidget(self.test_btn)

        self.reconnect_btn = QPushButton("↻ Reconnect")
        self.reconnect_btn.setMinimumHeight(40)
        self.reconnect_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #5DADE2, stop:1 #3498DB);
            }
        """)
        self.reconnect_btn.clicked.connect(self._connect_to_esp32)
        button_layout.addWidget(self.reconnect_btn)

        self.close_btn = QPushButton("✕ Close")
        self.close_btn.setMinimumHeight(40)
        self.close_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:1 #C0392B);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12pt;
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

        layout.addStretch()

        # Apply theme styling
        page.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #2C3E50, stop:1 #34495E);
            }
            QGroupBox {
                border: 2px solid #34495E;
                border-radius: 10px;
                margin-top: 10px;
                padding: 15px;
                background-color: rgba(0, 0, 0, 0.3);
            }
            QGroupBox::title {
                color: #ECF0F1;
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                font-weight: bold;
            }
        """)

        return page

    def _create_signal_group(self, title, prefix, is_mcu):
        """Create a signal strength display group with radar-style indicator"""
        group = QGroupBox(title)
        layout = QVBoxLayout()
        layout.setSpacing(15)

        # SSID Label
        ssid_label = QLabel(f"Network: {self.mcu_ssid if is_mcu else self.pc_ssid}")
        ssid_label.setStyleSheet("color: #BDC3C7; font-size: 11pt;")
        layout.addWidget(ssid_label)

        if is_mcu:
            self.mcu_ssid_label = ssid_label
        else:
            self.pc_ssid_label = ssid_label

        # Signal Strength Display (Radar-style)
        signal_layout = QHBoxLayout()

        # RSSI value (large, prominent)
        rssi_value_label = QLabel("-- dBm")
        rssi_value_label.setStyleSheet("color: #ECF0F1; font-size: 28pt; font-weight: bold; min-width: 150px;")
        signal_layout.addWidget(rssi_value_label)

        if is_mcu:
            self.mcu_rssi_label = rssi_value_label
        else:
            self.pc_rssi_label = rssi_value_label

        signal_layout.addSpacing(20)

        # Radar-style signal indicator (color blocks)
        indicator_layout = QVBoxLayout()
        indicator_title = QLabel("Signal Level")
        indicator_title.setStyleSheet("color: #7F8C8D; font-size: 9pt;")
        indicator_layout.addWidget(indicator_title)

        # Create 5 level blocks (Green -> Yellow -> Red)
        blocks_layout = QHBoxLayout()
        blocks_layout.setSpacing(5)

        signal_blocks = []
        for i in range(5):
            block = QLabel()
            block.setFixedSize(30, 40)
            block.setStyleSheet("background-color: #34495E; border: 1px solid #2C3E50; border-radius: 3px;")
            blocks_layout.addWidget(block)
            signal_blocks.append(block)

        if is_mcu:
            self.mcu_signal_blocks = signal_blocks
        else:
            self.pc_signal_blocks = signal_blocks

        indicator_layout.addLayout(blocks_layout)
        signal_layout.addLayout(indicator_layout)
        signal_layout.addStretch()
        layout.addLayout(signal_layout)

        group.setLayout(layout)
        return group

    def _create_metrics_group(self):
        """Create connection quality metrics display group"""
        group = QGroupBox("Transmission Quality Metrics")
        layout = QVBoxLayout()
        layout.setSpacing(10)

        # Packet Loss Section
        loss_title = QLabel("Packet Loss Rate")
        loss_title.setStyleSheet("color: #ECF0F1; font-size: 12pt; font-weight: bold;")
        layout.addWidget(loss_title)

        loss_layout = QHBoxLayout()

        # Motor 1
        m1_layout = QVBoxLayout()
        m1_title = QLabel("Motor 1")
        m1_title.setStyleSheet("color: cyan; font-size: 10pt; font-weight: bold;")
        m1_layout.addWidget(m1_title)

        self.m1_loss_label = QLabel("0.00%")
        self.m1_loss_label.setStyleSheet("color: #2ECC71; font-size: 20pt; font-weight: bold;")
        m1_layout.addWidget(self.m1_loss_label)

        self.m1_packets_label = QLabel("(0/0)")
        self.m1_packets_label.setStyleSheet("color: #BDC3C7; font-size: 10pt;")
        m1_layout.addWidget(self.m1_packets_label)

        loss_layout.addLayout(m1_layout)
        loss_layout.addSpacing(20)

        # Motor 2
        m2_layout = QVBoxLayout()
        m2_title = QLabel("Motor 2")
        m2_title.setStyleSheet("color: orange; font-size: 10pt; font-weight: bold;")
        m2_layout.addWidget(m2_title)

        self.m2_loss_label = QLabel("0.00%")
        self.m2_loss_label.setStyleSheet("color: #2ECC71; font-size: 20pt; font-weight: bold;")
        m2_layout.addWidget(self.m2_loss_label)

        self.m2_packets_label = QLabel("(0/0)")
        self.m2_packets_label.setStyleSheet("color: #BDC3C7; font-size: 10pt;")
        m2_layout.addWidget(self.m2_packets_label)

        loss_layout.addLayout(m2_layout)
        loss_layout.addStretch()

        layout.addLayout(loss_layout)

        # Divider
        divider = QLabel()
        divider.setStyleSheet("background-color: #34495E; max-height: 2px;")
        layout.addWidget(divider)

        # Latency Section
        latency_title = QLabel("Network Latency (RTT)")
        latency_title.setStyleSheet("color: #ECF0F1; font-size: 12pt; font-weight: bold;")
        layout.addWidget(latency_title)

        latency_layout = QHBoxLayout()

        # Current RTT
        current_layout = QVBoxLayout()
        current_title = QLabel("Current")
        current_title.setStyleSheet("color: #7F8C8D; font-size: 10pt;")
        current_layout.addWidget(current_title)

        self.latency_label = QLabel("-- ms")
        self.latency_label.setStyleSheet("color: #3498DB; font-size: 18pt; font-weight: bold;")
        current_layout.addWidget(self.latency_label)

        latency_layout.addLayout(current_layout)
        latency_layout.addSpacing(20)

        # Stats
        stats_layout = QVBoxLayout()
        self.latency_avg_label = QLabel("Avg: -- ms")
        self.latency_avg_label.setStyleSheet("color: #BDC3C7; font-size: 10pt;")
        stats_layout.addWidget(self.latency_avg_label)

        self.latency_minmax_label = QLabel("Min: -- ms  Max: -- ms")
        self.latency_minmax_label.setStyleSheet("color: #BDC3C7; font-size: 10pt;")
        stats_layout.addWidget(self.latency_minmax_label)

        latency_layout.addLayout(stats_layout)
        latency_layout.addStretch()

        layout.addLayout(latency_layout)

        group.setLayout(layout)
        return group

    def _init_timers(self):
        """Initialize update timers"""
        # Update PC WiFi RSSI every 1 second
        self.pc_timer = QTimer()
        self.pc_timer.timeout.connect(self._update_pc_rssi)
        self.pc_timer.start(1000)

        # Update MCU data every 100ms (to catch 1-second pings)
        self.mcu_timer = QTimer()
        self.mcu_timer.timeout.connect(self._update_mcu_data)
        self.mcu_timer.start(100)

        # Update connection quality stats every 1 second
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._update_connection_stats)
        self.stats_timer.start(1000)

        # PING timer for latency measurement (started/stopped by test toggle)
        self.ping_timer = QTimer()
        self.ping_timer.timeout.connect(self._send_ping)
        # Timer is NOT started here - controlled by _toggle_test()

    def _connect_to_esp32(self):
        """Connect to ESP32 TCP server"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

        self.status_label.setText("Status: Connecting...")
        self.status_label.setStyleSheet("color: #F39C12; font-size: 14pt; font-weight: bold; padding: 10px;")

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.esp32_ip, self.esp32_port))
            self.connected = True

            self.status_label.setText(f"Status: Connected to {self.esp32_ip}:{self.esp32_port}")
            self.status_label.setStyleSheet("color: #27AE60; font-size: 14pt; font-weight: bold; padding: 10px;")

            print(f"✓ Connected to ESP32: {self.esp32_ip}:{self.esp32_port}")

        except Exception as e:
            self.connected = False
            self.status_label.setText(f"Status: Connection Failed - {e}")
            self.status_label.setStyleSheet("color: #E74C3C; font-size: 14pt; font-weight: bold; padding: 10px;")
            print(f"✗ Connection failed: {e}")

    def _update_mcu_data(self):
        """Update MCU WiFi data from ESP32"""
        if not self.connected or not self.socket:
            return

        try:
            # Check if data is available
            self.socket.settimeout(0.1)
            data = self.socket.recv(4096).decode('utf-8', errors='ignore')

            if data:
                # Parse WiFi status messages
                # Format: [WIFI_STATUS] SSID:ExoPulse RSSI:-48
                wifi_status_match = re.search(r'\[WIFI_STATUS\]\s+SSID:(\S+)\s+RSSI:(-?\d+)', data)
                if wifi_status_match:
                    self.mcu_ssid = wifi_status_match.group(1)
                    self.mcu_rssi = int(wifi_status_match.group(2))
                    self.mcu_last_ping = time.time()

                    # Update UI
                    self.mcu_ssid_label.setText(f"Network: {self.mcu_ssid}")

                # Parse motor data with SEQ field (for packet loss tracking)
                if self.test_running:
                    motor_matches = re.finditer(r'\[(\d+)\]\s+SEQ:(\d+)\s+M:(\d+)', data)
                    for match in motor_matches:
                        seq = int(match.group(2))
                        motor_id = int(match.group(3))

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

                    # Update packet loss display
                    self._update_packet_loss_display()

                    # Parse PONG responses for latency measurement
                    pong_match = re.search(r'\[PONG\]\s+seq:(\d+)\s+ts_req:(\d+)\s+ts_reply:(\d+)', data)
                    if pong_match:
                        seq = int(pong_match.group(1))

                        # Calculate RTT using PC-side recorded send time
                        if seq in self.ping_sent_times:
                            sent_time_ms = self.ping_sent_times[seq]
                            current_time_ms = int(time.time() * 1000)
                            rtt = current_time_ms - sent_time_ms

                            self.current_rtt = rtt
                            self.latency_history.append(rtt)

                            # Clean up old entries (keep only last 10)
                            if len(self.ping_sent_times) > 20:
                                # Remove oldest entries
                                old_seqs = sorted(self.ping_sent_times.keys())[:-10]
                                for old_seq in old_seqs:
                                    del self.ping_sent_times[old_seq]

                            # Update latency display
                            self._update_latency_display()

        except socket.timeout:
            # No data available - this is normal
            pass
        except Exception as e:
            print(f"MCU data read error: {e}")
            self.connected = False
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: #E74C3C; font-size: 14pt; font-weight: bold; padding: 10px;")

    def _update_pc_rssi(self):
        """Update PC WiFi RSSI"""
        try:
            # Get WiFi SSID
            ssid_result = subprocess.run(
                ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
                capture_output=True,
                text=True,
                timeout=5
            )

            if ssid_result.returncode == 0:
                for line in ssid_result.stdout.strip().split('\n'):
                    if line.startswith('yes:'):
                        self.pc_ssid = line.split(':', 1)[1]
                        self.pc_ssid_label.setText(f"Network: {self.pc_ssid}")
                        break

            # Get WiFi RSSI
            rssi_result = subprocess.run(
                ["nmcli", "-f", "IN-USE,SIGNAL", "dev", "wifi"],
                capture_output=True,
                text=True,
                timeout=5
            )

            if rssi_result.returncode == 0:
                for line in rssi_result.stdout.strip().split('\n'):
                    if line.startswith('*'):
                        # Extract signal strength (0-100)
                        signal_match = re.search(r'\s+(\d+)\s*$', line)
                        if signal_match:
                            signal_percent = int(signal_match.group(1))
                            # Convert to approximate dBm (-100 to -30 range)
                            self.pc_rssi = int(-100 + (signal_percent * 0.7))
                            self.pc_last_ping = time.time()

                            # Update RSSI value
                            self.pc_rssi_label.setText(f"{self.pc_rssi} dBm")
                            break

        except Exception as e:
            print(f"PC RSSI read error: {e}")

    def _update_connection_stats(self):
        """Update connection quality statistics (ping-like success rate)"""
        current_time = time.time()

        # Update MCU connection stats
        if self.mcu_last_ping > 0:
            time_since_last = current_time - self.mcu_last_ping
            # Record success (1) if received within last 1.5 seconds, else failure (0)
            self.mcu_ping_history.append(1 if time_since_last < 1.5 else 0)
        else:
            self.mcu_ping_history.append(0)

        # Update PC connection stats
        if self.pc_last_ping > 0:
            time_since_last = current_time - self.pc_last_ping
            self.pc_ping_history.append(1 if time_since_last < 1.5 else 0)
        else:
            self.pc_ping_history.append(0)

        # Update signal displays with current RSSI
        if self.mcu_rssi != 0:
            self._update_quality_display(
                self.mcu_rssi_label,
                self.mcu_signal_blocks,
                self.mcu_rssi
            )

        if self.pc_rssi != 0:
            self._update_quality_display(
                self.pc_rssi_label,
                self.pc_signal_blocks,
                self.pc_rssi
            )

    def _update_signal_blocks(self, blocks, rssi):
        """Update radar-style signal blocks based on RSSI
        RSSI ranges: -30 (excellent) to -100 (no signal)
        """
        # Determine how many blocks to light up (0-5)
        if rssi >= -50:
            num_blocks = 5  # Excellent
        elif rssi >= -60:
            num_blocks = 4  # Good
        elif rssi >= -70:
            num_blocks = 3  # Fair
        elif rssi >= -80:
            num_blocks = 2  # Poor
        elif rssi >= -90:
            num_blocks = 1  # Very poor
        else:
            num_blocks = 0  # No signal

        # Update block colors
        for i, block in enumerate(blocks):
            if i < num_blocks:
                # Light up based on position (green -> yellow -> red)
                if i < 3:
                    color = "#27AE60"  # Green
                elif i < 4:
                    color = "#F39C12"  # Yellow/Orange
                else:
                    color = "#E74C3C"  # Red
                block.setStyleSheet(f"background-color: {color}; border: 1px solid #2C3E50; border-radius: 3px;")
            else:
                # Dim/off
                block.setStyleSheet("background-color: #34495E; border: 1px solid #2C3E50; border-radius: 3px;")

    def _update_quality_display(self, rssi_label, blocks, rssi):
        """Update signal strength display with RSSI value and blocks"""
        # Update RSSI label
        rssi_label.setText(f"{rssi} dBm")

        # Update color blocks
        self._update_signal_blocks(blocks, rssi)

    def _toggle_test(self):
        """Toggle bidirectional transmission test"""
        if not self.test_running:
            # Start test
            self.test_running = True
            self.test_btn.setText("■ Stop Test")
            self.test_btn.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #E74C3C, stop:1 #C0392B);
                    color: white;
                    border: none;
                    border-radius: 5px;
                    font-size: 12pt;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #EC7063, stop:1 #E74C3C);
                }
            """)

            # Reset test statistics and record start time
            self.test_start_time = int(time.time() * 1000)  # Record test start time
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

            print("[TEST] Bidirectional transmission test started")
        else:
            # Stop test
            self.test_running = False
            self.test_btn.setText("▶ Start Test")
            self.test_btn.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #27AE60, stop:1 #229954);
                    color: white;
                    border: none;
                    border-radius: 5px;
                    font-size: 12pt;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #2ECC71, stop:1 #27AE60);
                }
            """)

            # Stop PING timer
            self.ping_timer.stop()

            print("[TEST] Bidirectional transmission test stopped")

    def _send_ping(self):
        """Send PING request to ESP32 for latency measurement"""
        if not self.connected or not self.socket or not self.test_running:
            return

        try:
            self.ping_seq += 1
            current_time_ms = int(time.time() * 1000)

            # Use relative timestamp from test start
            relative_timestamp_ms = current_time_ms - self.test_start_time
            ping_msg = f"[PING] seq:{self.ping_seq} ts:{relative_timestamp_ms}\n"

            # Record absolute sent time for RTT calculation
            self.ping_sent_times[self.ping_seq] = current_time_ms

            self.socket.send(ping_msg.encode('utf-8'))
        except Exception as e:
            print(f"Ping send error: {e}")

    def _update_packet_loss_display(self):
        """Update packet loss statistics display"""
        if not self.test_running:
            return

        # Motor 1 packet loss
        m1_total = self.motor1_packets_received + self.motor1_packets_lost
        if m1_total > 0:
            m1_loss_rate = (self.motor1_packets_lost / m1_total) * 100
            self.m1_loss_label.setText(f"{m1_loss_rate:.2f}%")
            self.m1_packets_label.setText(f"({self.motor1_packets_lost}/{m1_total})")

            # Color code based on loss rate
            if m1_loss_rate < 1.0:
                color = "#2ECC71"  # Green - Good
            elif m1_loss_rate < 5.0:
                color = "#F39C12"  # Orange - Fair
            else:
                color = "#E74C3C"  # Red - Poor
            self.m1_loss_label.setStyleSheet(f"color: {color}; font-size: 20pt; font-weight: bold;")

        # Motor 2 packet loss
        m2_total = self.motor2_packets_received + self.motor2_packets_lost
        if m2_total > 0:
            m2_loss_rate = (self.motor2_packets_lost / m2_total) * 100
            self.m2_loss_label.setText(f"{m2_loss_rate:.2f}%")
            self.m2_packets_label.setText(f"({self.motor2_packets_lost}/{m2_total})")

            # Color code based on loss rate
            if m2_loss_rate < 1.0:
                color = "#2ECC71"  # Green - Good
            elif m2_loss_rate < 5.0:
                color = "#F39C12"  # Orange - Fair
            else:
                color = "#E74C3C"  # Red - Poor
            self.m2_loss_label.setStyleSheet(f"color: {color}; font-size: 20pt; font-weight: bold;")

    def _update_latency_display(self):
        """Update latency statistics display"""
        if not self.test_running or len(self.latency_history) == 0:
            return

        # Current RTT
        self.latency_label.setText(f"{self.current_rtt:.1f} ms")

        # Statistics
        avg_rtt = sum(self.latency_history) / len(self.latency_history)
        min_rtt = min(self.latency_history)
        max_rtt = max(self.latency_history)

        self.latency_avg_label.setText(f"Avg: {avg_rtt:.1f} ms")
        self.latency_minmax_label.setText(f"Min: {min_rtt:.1f} ms  Max: {max_rtt:.1f} ms")

        # Color code based on latency
        if self.current_rtt < 50:
            color = "#2ECC71"  # Green - Excellent
        elif self.current_rtt < 100:
            color = "#F39C12"  # Orange - Fair
        else:
            color = "#E74C3C"  # Red - Poor
        self.latency_label.setStyleSheet(f"color: {color}; font-size: 18pt; font-weight: bold;")

    def _start_wifi_config(self):
        """Start WiFi configuration process"""
        ssid = self.ssid_input.text().strip()
        password = self.password_input.text().strip()

        if not ssid:
            self.config_log.append("<span style='color: #E74C3C;'>Error: SSID cannot be empty</span>")
            return

        if not password:
            self.config_log.append("<span style='color: #E74C3C;'>Error: Password cannot be empty</span>")
            return

        # Disable inputs during configuration
        self.ssid_input.setEnabled(False)
        self.password_input.setEnabled(False)
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("Configuring...")

        self.config_log.clear()
        self.config_log.append("<span style='color: #3498DB;'>Starting WiFi configuration...</span>")

        # Run configuration in separate thread
        config_thread = threading.Thread(
            target=self._run_wifi_config_thread,
            args=(ssid, password),
            daemon=True
        )
        config_thread.start()

    def _run_wifi_config_thread(self, ssid, password):
        """Run WiFi configuration in background thread"""
        try:
            # Run setup_wifi function
            import io
            from contextlib import redirect_stdout

            log_buffer = io.StringIO()

            with redirect_stdout(log_buffer):
                ip_address = setup_wifi(ssid, password)

            # Emit log messages
            log_content = log_buffer.getvalue()
            for line in log_content.split('\n'):
                if line.strip():
                    self.config_signals.log_message.emit(line)

            if ip_address:
                self.config_signals.config_complete.emit(ip_address)
            else:
                self.config_signals.config_failed.emit("Failed to obtain IP address")

        except Exception as e:
            self.config_signals.config_failed.emit(f"Configuration error: {str(e)}")

    def _on_config_complete(self, ip_address):
        """Handle successful WiFi configuration"""
        self.config_log.append(f"<span style='color: #27AE60;'>✓ Configuration complete! IP: {ip_address}</span>")
        self.config_log.append(f"<span style='color: #F39C12;'>ℹ Output mode: UART (default)</span>")
        self.config_log.append(f"<span style='color: #F39C12;'>ℹ You can switch to WiFi mode using: MODE_WIFI command</span>")
        self.config_log.append("<span style='color: #3498DB;'>Switching to monitor view...</span>")

        # Set IP and switch to monitor page
        self.esp32_ip = ip_address
        self.stacked_widget.setCurrentIndex(1)

        # Initialize timers and connect
        self._init_timers()
        self._connect_to_esp32()

    def _on_config_failed(self, error_message):
        """Handle failed WiFi configuration"""
        self.config_log.append(f"<span style='color: #E74C3C;'>✗ Configuration failed: {error_message}</span>")

        # Re-enable inputs
        self.ssid_input.setEnabled(True)
        self.password_input.setEnabled(True)
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Connect to WiFi")

    def _on_log_message(self, message):
        """Handle log messages from configuration thread"""
        # Color code log messages
        if "[ERROR]" in message or "✗" in message:
            colored_msg = f"<span style='color: #E74C3C;'>{message}</span>"
        elif "[OK]" in message or "✓" in message:
            colored_msg = f"<span style='color: #27AE60;'>{message}</span>"
        elif "[*]" in message:
            colored_msg = f"<span style='color: #3498DB;'>{message}</span>"
        else:
            colored_msg = f"<span style='color: #BDC3C7;'>{message}</span>"

        self.config_log.append(colored_msg)
        # Auto-scroll to bottom
        self.config_log.verticalScrollBar().setValue(
            self.config_log.verticalScrollBar().maximum()
        )

    def _on_close(self):
        """Handle close button click - save WiFi state if connected"""
        # If connected to ESP32, save IP to temp file for main GUI to switch to WiFi mode
        if self.connected and self.esp32_ip:
            try:
                temp_file = "/tmp/exopulse_wifi_switch.txt"
                with open(temp_file, "w") as f:
                    f.write(f"{self.esp32_ip}:{self.esp32_port}\n")
                print(f"[✓] Saved WiFi config to {temp_file}")
            except Exception as e:
                print(f"[!] Warning: Failed to save WiFi config: {e}")

        # Close the window
        self.close()

    def closeEvent(self, event):
        """Handle window close event"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        event.accept()


def main():
    """Main entry point"""
    # Parse command line arguments
    ssid = None
    password = None
    launch_gui = True

    # Check for command-line mode (SSID and password provided)
    if len(sys.argv) >= 3:
        ssid = sys.argv[1]
        password = sys.argv[2]
        print(f"Using provided credentials: SSID={ssid}")

        # Configure WiFi
        ip = setup_wifi(ssid, password)

        if ip:
            print("\nWaiting 2 seconds for TCP server to start...")
            time.sleep(2)

            # Launch GUI with IP
            app = QApplication(sys.argv)
            app.setApplicationName("ExoPulse WiFi Monitor")
            app.setOrganizationName("ExoPulse")
            app.setApplicationVersion("1.0.0")

            window = WiFiSignalMonitor(ip, TCP_PORT)
            window.show()

            sys.exit(app.exec())
        else:
            print("\n✗ Setup failed. Please check:")
            print("  1. ESP32 is connected to USB")
            print("  2. Mobile hotspot is enabled")
            print("  3. Serial port is not in use")
            sys.exit(1)
    else:
        # Launch GUI in configuration mode
        print("Starting in GUI configuration mode...")
        print(f"Usage: python3 {sys.argv[0]} [SSID] [PASSWORD]")

        app = QApplication(sys.argv)
        app.setApplicationName("ExoPulse WiFi Monitor")
        app.setOrganizationName("ExoPulse")
        app.setApplicationVersion("1.0.0")

        window = WiFiSignalMonitor(None, TCP_PORT)
        window.show()

        sys.exit(app.exec())


if __name__ == "__main__":
    main()
