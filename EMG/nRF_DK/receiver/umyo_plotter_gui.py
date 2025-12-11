#!/usr/bin/env python3
"""
uMyo EMG Plotter GUI - PySide6

Simple real-time EMG plotter with:
- Auto-detect and auto-reconnect for J-Link CDC ports
- Configurable window size
- Device connection status terminal
- Real-time waveform display
"""

import sys
import re
import time
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QTextEdit, QComboBox, QSpinBox,
    QGroupBox, QSplitter, QLineEdit, QCheckBox, QDoubleSpinBox
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QPainter, QPen, QColor

# Default window size
DEFAULT_WIDTH = 1200
DEFAULT_HEIGHT = 700

# ADC to mV conversion factor for uMyo v3.1
# Based on: AD8293G80 (80x gain) + nRF52810 SAADC (14-bit, 0.6V ref, 1/6 gain)
# Formula: ADC_value × (0.6V × 6 / 16384 / 80) × 1000 = ADC_value × 0.002746582 mV
# This gives the actual EMG input voltage in mV before amplification
ADC_TO_MV = 0.002746582  # mV per ADC count


class SerialWorker(QThread):
    """Background thread for serial communication with auto-reconnect."""

    data_received = Signal(str)  # Raw line
    connection_changed = Signal(bool, str)  # connected, port_name
    device_discovered = Signal(str, int)  # device_id, packet_count

    def __init__(self):
        super().__init__()
        self.running = False
        self.ser = None
        self.port = None
        self.baud = 921600
        self.reconnect_delay = 2000  # ms
        self.last_reconnect_attempt = 0

    def find_jlink_port(self):
        """Auto-detect J-Link CDC port."""
        ports = serial.tools.list_ports.comports()
        jlink_ports = [p for p in ports if 'J-Link' in p.description]
        jlink_ports.sort(key=lambda p: p.device)

        if jlink_ports:
            return jlink_ports[0].device

        # Fallback: any ACM port
        acm_ports = [p for p in ports if 'ACM' in p.device]
        acm_ports.sort(key=lambda p: p.device)
        if acm_ports:
            return acm_ports[0].device

        return None

    def try_connect(self):
        """Try to connect to the serial port."""
        if self.ser and self.ser.is_open:
            return True

        port = self.find_jlink_port()
        if not port:
            return False

        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.1)
            self.port = port
            self.connection_changed.emit(True, port)
            return True
        except Exception as e:
            self.connection_changed.emit(False, str(e))
            return False

    def do_disconnect(self):
        """Disconnect from the serial port."""
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
        self.connection_changed.emit(False, "Disconnected")

    def run(self):
        """Main worker loop."""
        self.running = True
        devices = {}

        while self.running:
            # Try to connect if not connected
            if not self.ser or not self.ser.is_open:
                if not self.try_connect():
                    self.msleep(self.reconnect_delay)
                    continue

            # Read data
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)

                        # Parse device ID
                        id_match = re.search(r'ID=0x([0-9A-Fa-f]+)', line)
                        if id_match:
                            dev_id = id_match.group(1)
                            devices[dev_id] = devices.get(dev_id, 0) + 1
                            self.device_discovered.emit(dev_id, devices[dev_id])
                else:
                    self.msleep(1)
            except serial.SerialException as e:
                # Connection lost - will auto-reconnect
                self.connection_changed.emit(False, f"Lost: {e}")
                self.ser = None
                self.msleep(500)
            except Exception as e:
                self.msleep(10)

    def stop(self):
        """Stop the worker."""
        self.running = False
        self.do_disconnect()


class WaveformWidget(QFrame):
    """Custom widget for drawing EMG waveform."""

    SAMPLE_RATE = 1000  # Hz (approximate, ~1kHz from uMyo)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.setStyleSheet("background-color: #1a1a2e; border: 1px solid #16213e;")

        self._window_size = 2000
        self.data = deque(maxlen=self._window_size)
        self.y_min = 0
        self.y_max = 16383
        self.auto_scale = True
        self._display_samples = 1500  # Number of samples to display
        self._show_mv = True  # Show mV instead of raw ADC (default on)
        self._total_samples = 0  # Total samples received for time tracking

    def set_window_size(self, size):
        """Set the time window size (number of samples to keep)."""
        self._window_size = max(100, size)
        old_data = list(self.data)
        self.data = deque(old_data[-self._window_size:], maxlen=self._window_size)
        self.update()

    def set_display_samples(self, samples):
        """Set how many samples to display on screen."""
        self._display_samples = max(100, samples)
        self.update()

    def set_y_range(self, y_min, y_max):
        """Set manual Y-axis range."""
        self.y_min = y_min
        self.y_max = y_max
        self.auto_scale = False
        self.update()

    def set_auto_scale(self, enabled):
        """Enable/disable auto-scaling."""
        self.auto_scale = enabled
        self.update()

    def set_show_mv(self, enabled):
        """Enable/disable mV display mode."""
        self._show_mv = enabled
        self.update()

    def add_samples(self, samples):
        """Add new samples to the waveform."""
        for s in samples:
            self.data.append(s)
        self._total_samples += len(samples)

        # Auto-scale Y axis
        if self.auto_scale and len(self.data) > 10:
            recent = list(self.data)[-500:]
            if recent:
                self.y_min = min(recent) - 100
                self.y_max = max(recent) + 100

        self.update()

    def paintEvent(self, event):
        """Draw the waveform."""
        super().paintEvent(event)

        if len(self.data) < 2:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw grid
        painter.setPen(QPen(QColor(50, 50, 80), 1))
        w, h = self.width(), self.height()
        for i in range(0, w, 100):
            painter.drawLine(i, 0, i, h)
        for i in range(0, h, 50):
            painter.drawLine(0, i, w, i)

        # Draw waveform
        painter.setPen(QPen(QColor(0, 255, 255), 1.5))

        data_list = list(self.data)
        n = len(data_list)

        # Map data to screen coordinates
        display_count = min(n, self._display_samples)
        x_scale = w / display_count if display_count > 0 else 1
        y_range = self.y_max - self.y_min
        if y_range == 0:
            y_range = 1

        # Draw only the last _display_samples samples
        start_idx = max(0, n - self._display_samples)

        prev_x, prev_y = None, None
        for i, val in enumerate(data_list[start_idx:]):
            x = int(i * x_scale)
            y = int(h - (val - self.y_min) / y_range * h)
            y = max(0, min(h - 1, y))

            if prev_x is not None:
                painter.drawLine(prev_x, prev_y, x, y)

            prev_x, prev_y = x, y

        # Draw Y axis labels and unit indicator
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)

        if self._show_mv:
            # Convert ADC values to mV for display
            y_max_mv = self.y_max * ADC_TO_MV
            y_min_mv = self.y_min * ADC_TO_MV
            y_max_uv = y_max_mv * 1000
            y_min_uv = y_min_mv * 1000

            # Draw unit label on Y-axis (rotated text simulation with vertical layout)
            painter.setPen(QPen(QColor(255, 200, 100), 1))  # Orange for unit
            font.setPointSize(10)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(5, h // 2 - 20, "µV")

            # Draw max/min values
            font.setBold(False)
            font.setPointSize(8)
            painter.setFont(font)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawText(5, 15, f"{y_max_uv:.1f} µV")
            painter.drawText(5, h - 5, f"{y_min_uv:.1f} µV")
            # Also show mV
            painter.drawText(5, 30, f"({y_max_mv:.4f} mV)")
        else:
            # Draw unit label
            painter.setPen(QPen(QColor(255, 200, 100), 1))  # Orange for unit
            font.setPointSize(10)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(5, h // 2 - 20, "ADC")

            # Draw max/min values
            font.setBold(False)
            font.setPointSize(8)
            painter.setFont(font)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawText(5, 15, f"{self.y_max:.0f}")
            painter.drawText(5, h - 5, f"{self.y_min:.0f}")

        painter.drawText(w - 100, 15, f"Samples: {len(self.data)}")

        # Draw X-axis time labels with tick marks
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        font.setPointSize(8)
        font.setBold(False)
        painter.setFont(font)

        # Calculate time span being displayed (in ms)
        display_count = min(n, self._display_samples) if n > 0 else self._display_samples
        time_span_ms = display_count  # At 1kHz, 1 sample = 1ms

        # Draw time axis with multiple tick marks
        num_ticks = 5  # Number of tick marks on X-axis
        for i in range(num_ticks + 1):
            x_pos = int(w * i / num_ticks)
            # Time value: left is oldest (-time_span_ms), right is newest (0)
            time_val = -time_span_ms + (time_span_ms * i / num_ticks)

            # Draw tick mark
            painter.drawLine(x_pos, h - 25, x_pos, h - 20)

            # Format time label
            if abs(time_val) >= 1000:
                label = f"{time_val/1000:.1f}s"
            else:
                label = f"{time_val:.0f}ms"

            # Center the label on the tick
            label_width = len(label) * 6  # Approximate width
            label_x = x_pos - label_width // 2
            if i == 0:
                label_x = max(5, label_x)
            elif i == num_ticks:
                label_x = min(w - label_width - 5, label_x)

            painter.drawText(label_x, h - 8, label)


class UMyoPlotterGUI(QMainWindow):
    """Main GUI window for uMyo EMG Plotter."""

    def __init__(self, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
        super().__init__()
        self.setWindowTitle("uMyo EMG Plotter")
        self.resize(width, height)

        # Dark theme
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0f0f1a;
                color: #e0e0e0;
            }
            QGroupBox {
                border: 1px solid #2d2d44;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #2d2d44;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
                padding: 8px 16px;
                color: #e0e0e0;
            }
            QPushButton:hover {
                background-color: #3d3d5c;
            }
            QPushButton:pressed {
                background-color: #1d1d2c;
            }
            QTextEdit {
                background-color: #1a1a2e;
                border: 1px solid #2d2d44;
                color: #00ff00;
                font-family: monospace;
            }
            QComboBox, QSpinBox, QLineEdit {
                background-color: #2d2d44;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
                padding: 4px;
                color: #e0e0e0;
            }
            QLineEdit:focus {
                border: 1px solid #4ecdc4;
            }
            QCheckBox {
                color: #e0e0e0;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border: 2px solid #4ecdc4;
                border-radius: 3px;
                background-color: #1a1a2e;
            }
            QCheckBox::indicator:checked {
                background-color: #4ecdc4;
                border-color: #4ecdc4;
            }
            QCheckBox::indicator:hover {
                border-color: #6ee7df;
            }
        """)

        self._init_ui()
        self._init_serial()

        # Update timer for GUI
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_stats)
        self.update_timer.start(500)

        # Discovered devices
        self.devices = {}
        self.packet_count = 0
        self.sample_count = 0

        # Packet loss tracking (per device)
        self.device_last_adc_id = {}  # device_id -> last ADC_ID
        self.packet_loss_count = 0
        self.total_expected_packets = 0

        # Sample rate calculation
        self._rate_start_time = None
        self._rate_start_samples = 0
        self._current_sample_rate = 0.0

        # Time-series data for statistics plot on close
        self._session_start_time = time.time()
        self._stats_history = []  # [(timestamp, sample_rate, loss_count, total_packets)]
        self._last_stats_record_time = 0

    def _init_ui(self):
        """Initialize the UI."""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Top bar - connection status
        top_bar = QHBoxLayout()

        self.status_label = QLabel("● Disconnected")
        self.status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 14px;")
        top_bar.addWidget(self.status_label)

        top_bar.addStretch()

        self.port_label = QLabel("Port: --")
        self.port_label.setStyleSheet("color: #888;")
        top_bar.addWidget(self.port_label)

        self.reconnect_btn = QPushButton("Reconnect")
        self.reconnect_btn.clicked.connect(self._reconnect)
        top_bar.addWidget(self.reconnect_btn)

        main_layout.addLayout(top_bar)

        # Control panel
        control_panel = QHBoxLayout()

        # Time Window controls (in ms, at ~1kHz sample rate: 1 sample = 1ms)
        time_group = QGroupBox("Time Window")
        time_layout = QHBoxLayout(time_group)

        time_layout.addWidget(QLabel("Display:"))
        self.display_time_spin = QSpinBox()
        self.display_time_spin.setRange(100, 10000)
        self.display_time_spin.setValue(1500)
        self.display_time_spin.setSingleStep(100)
        self.display_time_spin.setSuffix(" ms")
        self.display_time_spin.valueChanged.connect(self._on_display_time_changed)
        time_layout.addWidget(self.display_time_spin)

        time_layout.addWidget(QLabel("Buffer:"))
        self.buffer_time_spin = QSpinBox()
        self.buffer_time_spin.setRange(500, 50000)
        self.buffer_time_spin.setValue(2000)
        self.buffer_time_spin.setSingleStep(500)
        self.buffer_time_spin.setSuffix(" ms")
        self.buffer_time_spin.valueChanged.connect(self._on_buffer_time_changed)
        time_layout.addWidget(self.buffer_time_spin)

        control_panel.addWidget(time_group)

        # Amplitude controls (in mV)
        amp_group = QGroupBox("Amplitude")
        amp_layout = QHBoxLayout(amp_group)

        self.auto_scale_cb = QCheckBox("Auto")
        self.auto_scale_cb.setChecked(True)
        self.auto_scale_cb.stateChanged.connect(self._on_auto_scale_changed)
        amp_layout.addWidget(self.auto_scale_cb)

        self.show_mv_cb = QCheckBox("mV")
        self.show_mv_cb.setChecked(True)  # Default to mV display
        self.show_mv_cb.setToolTip("Show amplitude in mV (actual EMG input before amplification)")
        self.show_mv_cb.stateChanged.connect(self._on_show_mv_changed)
        amp_layout.addWidget(self.show_mv_cb)

        amp_layout.addWidget(QLabel("Min:"))
        self.y_min_spin = QDoubleSpinBox()
        self.y_min_spin.setRange(-100.0, 100.0)  # mV range
        self.y_min_spin.setValue(0.0)
        self.y_min_spin.setSingleStep(0.01)
        self.y_min_spin.setDecimals(3)
        self.y_min_spin.setSuffix(" mV")
        self.y_min_spin.setEnabled(False)
        self.y_min_spin.valueChanged.connect(self._on_y_range_changed)
        amp_layout.addWidget(self.y_min_spin)

        amp_layout.addWidget(QLabel("Max:"))
        self.y_max_spin = QDoubleSpinBox()
        self.y_max_spin.setRange(-100.0, 100.0)  # mV range
        self.y_max_spin.setValue(0.045)  # ~16383 ADC counts in mV
        self.y_max_spin.setSingleStep(0.01)
        self.y_max_spin.setDecimals(3)
        self.y_max_spin.setSuffix(" mV")
        self.y_max_spin.setEnabled(False)
        self.y_max_spin.valueChanged.connect(self._on_y_range_changed)
        amp_layout.addWidget(self.y_max_spin)

        control_panel.addWidget(amp_group)

        # Command input
        cmd_group = QGroupBox("Command")
        cmd_layout = QHBoxLayout(cmd_group)

        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Enter command...")
        self.cmd_input.returnPressed.connect(self._on_send_command)
        cmd_layout.addWidget(self.cmd_input)

        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self._on_send_command)
        cmd_layout.addWidget(self.send_btn)

        control_panel.addWidget(cmd_group)

        main_layout.addLayout(control_panel)

        # Splitter for waveform and terminal
        splitter = QSplitter(Qt.Vertical)

        # Waveform section
        waveform_group = QGroupBox("EMG Waveform (1kHz)")
        waveform_layout = QVBoxLayout(waveform_group)
        self.waveform = WaveformWidget()
        waveform_layout.addWidget(self.waveform)
        splitter.addWidget(waveform_group)

        # Bottom section - devices and terminal
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)
        bottom_layout.setContentsMargins(0, 0, 0, 0)

        # Device list
        device_group = QGroupBox("Connected Devices")
        device_layout = QVBoxLayout(device_group)
        self.device_list = QTextEdit()
        self.device_list.setReadOnly(True)
        self.device_list.setMaximumWidth(300)
        self.device_list.setPlaceholderText("No devices detected...")
        device_layout.addWidget(self.device_list)
        bottom_layout.addWidget(device_group)

        # Terminal (collapsible)
        terminal_group = QGroupBox("Serial Monitor (click to expand)")
        terminal_group.setCheckable(True)
        terminal_group.setChecked(False)  # Collapsed by default
        terminal_layout = QVBoxLayout(terminal_group)

        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMaximumHeight(150)
        self.terminal.setVisible(False)  # Hidden by default
        terminal_layout.addWidget(self.terminal)

        # Terminal controls
        term_controls = QHBoxLayout()
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.terminal.clear)
        self.clear_btn.setVisible(False)
        term_controls.addWidget(self.clear_btn)

        self.autoscroll_btn = QPushButton("Auto-scroll: ON")
        self.autoscroll_btn.setCheckable(True)
        self.autoscroll_btn.setChecked(True)
        self.autoscroll_btn.clicked.connect(self._toggle_autoscroll)
        self.autoscroll_btn.setVisible(False)
        term_controls.addWidget(self.autoscroll_btn)

        term_controls.addStretch()
        terminal_layout.addLayout(term_controls)

        # Connect toggle for collapsible terminal
        terminal_group.toggled.connect(self._toggle_terminal)
        self.terminal_group = terminal_group

        bottom_layout.addWidget(terminal_group)

        splitter.addWidget(bottom_widget)
        splitter.setSizes([400, 200])

        main_layout.addWidget(splitter)

        # Status bar
        status_bar = QHBoxLayout()
        self.stats_label = QLabel("Packets: 0 | Samples: 0 | Devices: 0")
        self.stats_label.setStyleSheet("color: #888;")
        status_bar.addWidget(self.stats_label)
        status_bar.addStretch()
        main_layout.addLayout(status_bar)

    def _init_serial(self):
        """Initialize serial worker."""
        self.serial_worker = SerialWorker()
        self.serial_worker.data_received.connect(self._on_data)
        self.serial_worker.connection_changed.connect(self._on_connection_changed)
        self.serial_worker.device_discovered.connect(self._on_device_discovered)
        self.serial_worker.start()

    def _on_data(self, line):
        """Handle received data line."""
        # Update terminal
        if self.autoscroll_btn.isChecked():
            self.terminal.append(line)
            self.terminal.verticalScrollBar().setValue(
                self.terminal.verticalScrollBar().maximum()
            )

        # Parse ADC data and track packet loss
        # Format: RX: ID=0x40B3A92E ADC_ID=33 ADC:5815,5775,5730,... SP:0,0,0,0
        adc_match = re.search(r'ADC:([-\d,]+)', line)
        if adc_match:
            try:
                samples = [int(x) for x in adc_match.group(1).split(',')]
                self.waveform.add_samples(samples)
                self.sample_count += len(samples)
            except:
                pass

        if line.startswith('RX:'):
            self.packet_count += 1

            # Track packet loss using ADC_ID sequence
            id_match = re.search(r'ID=0x([0-9A-Fa-f]+)', line)
            adc_id_match = re.search(r'ADC_ID=(\d+)', line)

            if id_match and adc_id_match:
                dev_id = id_match.group(1)
                adc_id = int(adc_id_match.group(1))

                if dev_id in self.device_last_adc_id:
                    last_id = self.device_last_adc_id[dev_id]
                    # ADC_ID is uint8_t, wraps at 256
                    expected_id = (last_id + 1) % 256
                    if adc_id != expected_id:
                        # Calculate how many packets were lost
                        if adc_id > last_id:
                            lost = adc_id - last_id - 1
                        else:
                            # Wrapped around
                            lost = (256 - last_id - 1) + adc_id
                        if lost > 0 and lost < 128:  # Sanity check
                            self.packet_loss_count += lost
                    self.total_expected_packets += 1

                self.device_last_adc_id[dev_id] = adc_id

    def _on_connection_changed(self, connected, info):
        """Handle connection state changes."""
        if connected:
            self.status_label.setText(f"● Connected")
            self.status_label.setStyleSheet("color: #4ecdc4; font-weight: bold; font-size: 14px;")
            self.port_label.setText(f"Port: {info}")
            self._log(f"[{self._timestamp()}] Connected to {info}")
        else:
            self.status_label.setText(f"● Disconnected")
            self.status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 14px;")
            self.port_label.setText("Port: --")
            self._log(f"[{self._timestamp()}] {info}")

    def _on_device_discovered(self, device_id, packet_count):
        """Handle device discovery."""
        self.devices[device_id] = {
            'packets': packet_count,
            'last_seen': datetime.now()
        }
        self._update_device_list()

    def _update_device_list(self):
        """Update the device list display."""
        lines = []
        for dev_id, info in self.devices.items():
            elapsed = (datetime.now() - info['last_seen']).total_seconds()
            status = "●" if elapsed < 1 else "○"
            color = "#4ecdc4" if elapsed < 1 else "#888"
            lines.append(f'<span style="color:{color}">{status}</span> 0x{dev_id}')
            lines.append(f'   Packets: {info["packets"]}')
            lines.append("")

        self.device_list.setHtml("<br>".join(lines) if lines else "No devices detected...")

    def _update_stats(self):
        """Update statistics display with sample rate and packet loss."""
        import time

        # Calculate sample rate
        now = time.time()
        if self._rate_start_time is None:
            self._rate_start_time = now
            self._rate_start_samples = self.sample_count
        else:
            elapsed = now - self._rate_start_time
            if elapsed >= 2.0:  # Update rate every 2 seconds
                samples_diff = self.sample_count - self._rate_start_samples
                self._current_sample_rate = samples_diff / elapsed
                self._rate_start_time = now
                self._rate_start_samples = self.sample_count

        # Calculate packet loss percentage
        if self.total_expected_packets > 0:
            loss_pct = (self.packet_loss_count / (self.total_expected_packets + self.packet_loss_count)) * 100
            loss_str = f"Loss: {self.packet_loss_count} ({loss_pct:.1f}%)"
        else:
            loss_str = "Loss: 0 (0.0%)"

        # Format sample rate
        rate_str = f"Rate: {self._current_sample_rate:.0f} Hz"

        self.stats_label.setText(
            f"Packets: {self.packet_count} | "
            f"Samples: {self.sample_count} | "
            f"{rate_str} | "
            f"{loss_str} | "
            f"Devices: {len(self.devices)}"
        )

        # Record stats history every 1 second for plotting on close
        if now - self._last_stats_record_time >= 1.0:
            elapsed_session = now - self._session_start_time
            self._stats_history.append((
                elapsed_session,
                self._current_sample_rate,
                self.packet_loss_count,
                self.packet_count
            ))
            self._last_stats_record_time = now

    def _reconnect(self):
        """Force reconnection."""
        self._log(f"[{self._timestamp()}] Reconnecting...")

    def _toggle_terminal(self, checked):
        """Toggle terminal visibility."""
        self.terminal.setVisible(checked)
        self.clear_btn.setVisible(checked)
        self.autoscroll_btn.setVisible(checked)
        if checked:
            self.terminal_group.setTitle("Serial Monitor")
        else:
            self.terminal_group.setTitle("Serial Monitor (click to expand)")

    def _toggle_autoscroll(self):
        """Toggle auto-scroll."""
        if self.autoscroll_btn.isChecked():
            self.autoscroll_btn.setText("Auto-scroll: ON")
        else:
            self.autoscroll_btn.setText("Auto-scroll: OFF")

    def _on_display_time_changed(self, value_ms):
        """Handle display time change (ms -> samples at 1kHz)."""
        # At ~1kHz sample rate, 1ms = 1 sample
        self.waveform.set_display_samples(value_ms)

    def _on_buffer_time_changed(self, value_ms):
        """Handle buffer time change (ms -> samples at 1kHz)."""
        # At ~1kHz sample rate, 1ms = 1 sample
        self.waveform.set_window_size(value_ms)

    def _on_auto_scale_changed(self, state):
        """Handle auto-scale checkbox change."""
        enabled = state == Qt.Checked or state == 2
        self.waveform.set_auto_scale(enabled)
        self.y_min_spin.setEnabled(not enabled)
        self.y_max_spin.setEnabled(not enabled)
        if not enabled:
            # Apply current spin box values
            self._on_y_range_changed()

    def _on_y_range_changed(self):
        """Handle Y-axis range change (mV -> ADC counts)."""
        if not self.auto_scale_cb.isChecked():
            # Convert mV to ADC counts: ADC = mV / ADC_TO_MV
            y_min_adc = int(self.y_min_spin.value() / ADC_TO_MV)
            y_max_adc = int(self.y_max_spin.value() / ADC_TO_MV)
            self.waveform.set_y_range(y_min_adc, y_max_adc)

    def _on_show_mv_changed(self, state):
        """Handle mV display checkbox change."""
        # state can be Qt.CheckState.Checked (2) or Qt.Checked depending on PySide6 version
        self.waveform.set_show_mv(state == 2 or state == Qt.Checked)

    def _on_send_command(self):
        """Handle command send."""
        cmd = self.cmd_input.text().strip()
        if cmd and self.serial_worker.ser and self.serial_worker.ser.is_open:
            try:
                self.serial_worker.ser.write((cmd + '\r\n').encode())
                self._log(f"[{self._timestamp()}] TX: {cmd}")
                self.cmd_input.clear()
            except Exception as e:
                self._log(f"[{self._timestamp()}] TX Error: {e}")
        elif cmd:
            self._log(f"[{self._timestamp()}] Not connected, cannot send: {cmd}")

    def _log(self, msg):
        """Log message to terminal."""
        self.terminal.append(f'<span style="color:#888">{msg}</span>')

    def _timestamp(self):
        """Get current timestamp string."""
        return datetime.now().strftime("%H:%M:%S")

    def closeEvent(self, event):
        """Handle window close."""
        self.serial_worker.stop()
        self.serial_worker.wait(1000)

        # Show statistics plot if we have data
        if len(self._stats_history) > 5:
            self._show_session_stats_plot()

        event.accept()

    def _show_session_stats_plot(self):
        """Show a matplotlib plot with session statistics."""
        if not self._stats_history:
            return

        # Extract data from history
        timestamps = [h[0] for h in self._stats_history]
        sample_rates = [h[1] for h in self._stats_history]
        loss_counts = [h[2] for h in self._stats_history]
        packet_counts = [h[3] for h in self._stats_history]

        # Calculate packet loss rate (packets lost per second)
        loss_rates = [0]
        for i in range(1, len(loss_counts)):
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                loss_rate = (loss_counts[i] - loss_counts[i-1]) / dt
                loss_rates.append(max(0, loss_rate))
            else:
                loss_rates.append(0)

        # Create figure with 3 subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig.suptitle('uMyo EMG Session Statistics', fontsize=14, fontweight='bold')

        # Plot 1: Sample Rate
        ax1 = axes[0]
        ax1.plot(timestamps, sample_rates, 'c-', linewidth=1.5, label='Sample Rate')
        ax1.axhline(y=1000, color='g', linestyle='--', alpha=0.5, label='Target (1000 Hz)')
        ax1.set_ylabel('Sample Rate (Hz)')
        ax1.set_ylim(0, max(1200, max(sample_rates) * 1.1) if sample_rates else 1200)
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.fill_between(timestamps, sample_rates, alpha=0.3, color='cyan')

        # Calculate and show average
        avg_rate = np.mean(sample_rates) if sample_rates else 0
        ax1.axhline(y=avg_rate, color='orange', linestyle='-', alpha=0.7, label=f'Avg: {avg_rate:.0f} Hz')
        ax1.text(timestamps[-1] * 0.02, avg_rate + 50, f'Avg: {avg_rate:.0f} Hz', color='orange', fontsize=10)

        # Plot 2: Packet Loss Rate
        ax2 = axes[1]
        ax2.plot(timestamps, loss_rates, 'r-', linewidth=1.5, label='Loss Rate')
        ax2.set_ylabel('Packet Loss Rate (/s)')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        ax2.fill_between(timestamps, loss_rates, alpha=0.3, color='red')

        # Highlight loss events
        for i, (t, lr) in enumerate(zip(timestamps, loss_rates)):
            if lr > 5:  # Significant loss
                ax2.axvline(x=t, color='red', alpha=0.3, linewidth=2)

        # Plot 3: Cumulative Packet Loss
        ax3 = axes[2]
        ax3.plot(timestamps, loss_counts, 'm-', linewidth=1.5, label='Cumulative Loss')
        ax3.set_ylabel('Total Packets Lost')
        ax3.set_xlabel('Session Time (seconds)')
        ax3.legend(loc='upper left')
        ax3.grid(True, alpha=0.3)
        ax3.fill_between(timestamps, loss_counts, alpha=0.3, color='magenta')

        # Add packet count on secondary y-axis
        ax3b = ax3.twinx()
        ax3b.plot(timestamps, packet_counts, 'g-', linewidth=1, alpha=0.5, label='Total Packets')
        ax3b.set_ylabel('Total Packets Received', color='green')
        ax3b.tick_params(axis='y', labelcolor='green')

        # Add summary text
        total_time = timestamps[-1] if timestamps else 0
        total_loss = loss_counts[-1] if loss_counts else 0
        total_packets = packet_counts[-1] if packet_counts else 0
        loss_pct = (total_loss / (total_packets + total_loss) * 100) if (total_packets + total_loss) > 0 else 0

        summary_text = (
            f"Session Duration: {total_time:.1f}s\n"
            f"Total Packets: {total_packets}\n"
            f"Total Lost: {total_loss} ({loss_pct:.2f}%)\n"
            f"Avg Sample Rate: {avg_rate:.0f} Hz"
        )
        fig.text(0.02, 0.02, summary_text, fontsize=10, family='monospace',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.15)
        plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='uMyo EMG Plotter GUI')
    parser.add_argument('-W', '--width', type=int, default=DEFAULT_WIDTH, help='Window width')
    parser.add_argument('-H', '--height', type=int, default=DEFAULT_HEIGHT, help='Window height')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = UMyoPlotterGUI(width=args.width, height=args.height)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
