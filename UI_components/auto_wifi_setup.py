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

        # === CONTROL BUTTONS ===
        button_layout = QHBoxLayout()
        button_layout.setSpacing(10)

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
        self.close_btn.clicked.connect(self.close)
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
        """Create a signal strength display group"""
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

        # Connection Quality Display
        quality_layout = QHBoxLayout()

        # Success rate (large, prominent)
        success_label = QLabel("0%")
        success_label.setStyleSheet("color: #ECF0F1; font-size: 32pt; font-weight: bold; min-width: 100px;")
        quality_layout.addWidget(success_label)

        if is_mcu:
            self.mcu_quality_label = success_label
        else:
            self.pc_quality_label = success_label

        # Vertical divider
        quality_layout.addSpacing(20)

        # RSSI value (smaller, secondary info)
        rssi_info_layout = QVBoxLayout()
        rssi_title = QLabel("RSSI")
        rssi_title.setStyleSheet("color: #7F8C8D; font-size: 10pt;")
        rssi_info_layout.addWidget(rssi_title)

        rssi_value_label = QLabel("0 dBm")
        rssi_value_label.setStyleSheet("color: #BDC3C7; font-size: 14pt; font-weight: bold;")
        rssi_info_layout.addWidget(rssi_value_label)

        if is_mcu:
            self.mcu_rssi_label = rssi_value_label
        else:
            self.pc_rssi_label = rssi_value_label

        quality_layout.addLayout(rssi_info_layout)
        quality_layout.addStretch()
        layout.addLayout(quality_layout)

        # Progress Bar
        progress = QProgressBar()
        progress.setMinimum(0)
        progress.setMaximum(100)
        progress.setValue(0)
        progress.setTextVisible(True)
        progress.setFormat("%p%")
        progress.setMinimumHeight(30)
        progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #34495E;
                border-radius: 5px;
                background-color: #1a252f;
                text-align: center;
                color: #ECF0F1;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #27AE60, stop:1 #229954);
                border-radius: 3px;
            }
        """)
        layout.addWidget(progress)

        if is_mcu:
            self.mcu_progress = progress
        else:
            self.pc_progress = progress

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
            data = self.socket.recv(1024).decode('utf-8', errors='ignore')

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
                    self.mcu_rssi_label.setText(f"{self.mcu_rssi} dBm")

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

        # Calculate success rates
        if len(self.mcu_ping_history) > 0:
            mcu_success_rate = (sum(self.mcu_ping_history) / len(self.mcu_ping_history)) * 100
            self._update_quality_display(
                self.mcu_quality_label,
                self.mcu_progress,
                mcu_success_rate
            )

        if len(self.pc_ping_history) > 0:
            pc_success_rate = (sum(self.pc_ping_history) / len(self.pc_ping_history)) * 100
            self._update_quality_display(
                self.pc_quality_label,
                self.pc_progress,
                pc_success_rate
            )

    def _update_quality_display(self, quality_label, progress, success_rate):
        """Update connection quality display (ping-like success rate)"""
        # Update success rate percentage
        quality_label.setText(f"{int(success_rate)}%")

        # Update progress bar
        progress.setValue(int(success_rate))

        # Determine quality based on success rate
        if success_rate >= 95:
            quality_color = "#27AE60"  # Excellent - Green
            progress_color = "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #27AE60, stop:1 #229954)"
        elif success_rate >= 80:
            quality_color = "#2ECC71"  # Good - Light Green
            progress_color = "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2ECC71, stop:1 #27AE60)"
        elif success_rate >= 60:
            quality_color = "#F39C12"  # Fair - Orange
            progress_color = "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #F39C12, stop:1 #E67E22)"
        elif success_rate >= 30:
            quality_color = "#E67E22"  # Poor - Dark Orange
            progress_color = "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #E67E22, stop:1 #D35400)"
        else:
            quality_color = "#E74C3C"  # Critical - Red
            progress_color = "qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #E74C3C, stop:1 #C0392B)"

        # Update quality label color
        quality_label.setStyleSheet(f"color: {quality_color}; font-size: 32pt; font-weight: bold; min-width: 100px;")

        # Update progress bar color
        progress.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid #34495E;
                border-radius: 5px;
                background-color: #1a252f;
                text-align: center;
                color: #ECF0F1;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background: {progress_color};
                border-radius: 3px;
            }}
        """)

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
