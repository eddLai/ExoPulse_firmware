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
import math
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QTextEdit, QComboBox, QSpinBox,
    QGroupBox, QSplitter, QLineEdit, QCheckBox, QDoubleSpinBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QStackedWidget,
    QProgressBar, QAbstractItemView
)
from PySide6.QtCore import Qt, QTimer, Signal, QThread
from PySide6.QtGui import QFont, QPainter, QPen, QColor, QBrush

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


class SignalStrengthWidget(QFrame):
    """Widget to display signal strength as a circular power indicator."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.setMaximumSize(120, 120)
        self._devices = {}  # device_id -> {rate, color, selected}
        self._max_rate = 100.0  # Expected max packet rate for normalization

    def update_device(self, device_id, rate, color, selected=True):
        """Update a device's signal strength."""
        self._devices[device_id] = {
            'rate': rate,
            'color': color,
            'selected': selected
        }
        self.update()

    def remove_device(self, device_id):
        """Remove a device from the display."""
        if device_id in self._devices:
            del self._devices[device_id]
            self.update()

    def clear(self):
        """Clear all devices."""
        self._devices.clear()
        self.update()

    def paintEvent(self, event):
        """Draw the circular signal strength indicators."""
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        center_x, center_y = w // 2, h // 2
        max_radius = min(w, h) // 2 - 10

        # Draw background circle
        painter.setPen(QPen(QColor(50, 50, 80), 2))
        painter.setBrush(QBrush(QColor(26, 26, 46)))
        painter.drawEllipse(center_x - max_radius, center_y - max_radius,
                           max_radius * 2, max_radius * 2)

        # Draw concentric guide circles
        painter.setPen(QPen(QColor(50, 50, 80), 1, Qt.DashLine))
        for r in [0.33, 0.66]:
            radius = int(max_radius * r)
            painter.drawEllipse(center_x - radius, center_y - radius,
                               radius * 2, radius * 2)

        # Draw devices as pie segments or dots based on signal strength
        num_devices = len(self._devices)
        if num_devices == 0:
            # Draw "No Signal" text
            painter.setPen(QPen(QColor(100, 100, 100), 1))
            font = painter.font()
            font.setPointSize(9)
            painter.setFont(font)
            painter.drawText(self.rect(), Qt.AlignCenter, "No Signal")
            return

        # Calculate angle per device
        angle_per_device = 360 / max(num_devices, 1)

        for idx, (dev_id, dev_info) in enumerate(self._devices.items()):
            rate = dev_info['rate']
            color = dev_info['color']
            selected = dev_info['selected']

            # Normalize rate to 0-1 (with some headroom)
            strength = min(1.0, rate / self._max_rate) if self._max_rate > 0 else 0

            # Calculate radius based on signal strength
            radius = int(max_radius * max(0.1, strength))

            # Calculate angle for this device
            start_angle = int(idx * angle_per_device * 16)  # Qt uses 1/16th degrees
            span_angle = int((angle_per_device - 5) * 16)  # Leave small gap

            # Draw filled arc/pie slice
            if selected:
                painter.setBrush(QBrush(color))
                painter.setPen(QPen(color.darker(150), 2))
            else:
                # Dimmed if not selected
                dimmed = QColor(color)
                dimmed.setAlpha(80)
                painter.setBrush(QBrush(dimmed))
                painter.setPen(QPen(dimmed.darker(150), 1))

            # Draw pie slice from center
            pie_rect = (center_x - radius, center_y - radius, radius * 2, radius * 2)
            painter.drawPie(*pie_rect, start_angle, span_angle)

            # Draw device ID label at edge
            label_angle = (idx * angle_per_device + angle_per_device / 2) * math.pi / 180
            label_x = center_x + int((max_radius + 5) * math.cos(label_angle - math.pi/2))
            label_y = center_y + int((max_radius + 5) * math.sin(label_angle - math.pi/2))

        # Draw center dot
        painter.setBrush(QBrush(QColor(78, 205, 196)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(center_x - 5, center_y - 5, 10, 10)


class DeviceScannerWidget(QWidget):
    """Widget for scanning and selecting uMyo devices before data acquisition."""

    # Signal emitted when user confirms device selection
    # Emits list of selected device IDs
    devices_selected = Signal(list)

    # Device colors for visual consistency
    DEVICE_COLORS = [
        QColor(0, 255, 255),    # Cyan
        QColor(255, 100, 100),  # Red
        QColor(100, 255, 100),  # Green
        QColor(255, 200, 100),  # Orange
        QColor(200, 100, 255),  # Purple
        QColor(255, 255, 100),  # Yellow
        QColor(100, 200, 255),  # Light blue
        QColor(255, 100, 200),  # Pink
    ]

    def __init__(self, serial_worker, parent=None):
        super().__init__(parent)
        self.serial_worker = serial_worker
        self.discovered_devices = {}  # device_id -> {packets, last_seen, rssi, selected}
        self._init_ui()

        # Update timer for device status
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_device_table)
        self.update_timer.start(200)  # 5Hz update

    def _init_ui(self):
        """Initialize the scanner UI."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # Title
        title = QLabel("uMyo Device Scanner")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: #4ecdc4;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Instructions
        instructions = QLabel(
            "Scanning for uMyo EMG sensors...\n"
            "Select the devices you want to use and click 'Start Acquisition'"
        )
        instructions.setStyleSheet("font-size: 12px; color: #888; margin-bottom: 10px;")
        instructions.setAlignment(Qt.AlignCenter)
        layout.addWidget(instructions)

        # Connection status
        status_layout = QHBoxLayout()
        self.connection_status = QLabel("● Disconnected")
        self.connection_status.setStyleSheet("color: #ff6b6b; font-weight: bold;")
        status_layout.addWidget(self.connection_status)

        self.port_label = QLabel("Port: --")
        self.port_label.setStyleSheet("color: #888;")
        status_layout.addWidget(self.port_label)

        status_layout.addStretch()

        self.scan_time_label = QLabel("Scan time: 0s")
        self.scan_time_label.setStyleSheet("color: #888;")
        status_layout.addWidget(self.scan_time_label)

        layout.addLayout(status_layout)

        # Main content area with signal visualization and device table
        content_layout = QHBoxLayout()

        # Signal strength visualization (left side)
        signal_group = QGroupBox("Signal Power")
        signal_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #2d2d44;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
                color: #4ecdc4;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        signal_layout = QVBoxLayout(signal_group)

        self.signal_widget = SignalStrengthWidget()
        self.signal_widget.setStyleSheet("background-color: #1a1a2e; border: 1px solid #2d2d44; border-radius: 60px;")
        signal_layout.addWidget(self.signal_widget, alignment=Qt.AlignCenter)

        # Legend for signal widget
        legend_label = QLabel("Size = Packet Rate\nColor = Device ID")
        legend_label.setStyleSheet("color: #666; font-size: 10px;")
        legend_label.setAlignment(Qt.AlignCenter)
        signal_layout.addWidget(legend_label)

        content_layout.addWidget(signal_group)

        # Device table (right side)
        table_group = QGroupBox("Detected Devices")
        table_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #2d2d44;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
                color: #4ecdc4;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        table_layout = QVBoxLayout(table_group)

        self.device_table = QTableWidget()
        self.device_table.setColumnCount(5)
        self.device_table.setHorizontalHeaderLabels([
            "Select", "Device ID", "Packets", "Rate (Hz)", "Status"
        ])
        self.device_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.device_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.device_table.setColumnWidth(0, 60)
        self.device_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.device_table.setStyleSheet("""
            QTableWidget {
                background-color: #1a1a2e;
                border: 1px solid #2d2d44;
                gridline-color: #2d2d44;
            }
            QTableWidget::item {
                padding: 8px;
                border-bottom: 1px solid #2d2d44;
            }
            QTableWidget::item:selected {
                background-color: #2d4a5c;
            }
            QHeaderView::section {
                background-color: #2d2d44;
                padding: 8px;
                border: none;
                font-weight: bold;
            }
        """)
        table_layout.addWidget(self.device_table)

        content_layout.addWidget(table_group, stretch=1)

        layout.addLayout(content_layout)

        # Device count and selection info
        info_layout = QHBoxLayout()
        self.device_count_label = QLabel("Devices found: 0")
        self.device_count_label.setStyleSheet("color: #888;")
        info_layout.addWidget(self.device_count_label)

        info_layout.addStretch()

        self.selected_count_label = QLabel("Selected: 0")
        self.selected_count_label.setStyleSheet("color: #4ecdc4; font-weight: bold;")
        info_layout.addWidget(self.selected_count_label)

        layout.addLayout(info_layout)

        # Button row
        button_layout = QHBoxLayout()

        self.select_all_btn = QPushButton("Select All")
        self.select_all_btn.clicked.connect(self._select_all)
        self.select_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2d44;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
                padding: 10px 20px;
                color: #e0e0e0;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #3d3d5c; }
        """)
        button_layout.addWidget(self.select_all_btn)

        self.clear_btn = QPushButton("Clear Selection")
        self.clear_btn.clicked.connect(self._clear_selection)
        self.clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2d44;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
                padding: 10px 20px;
                color: #e0e0e0;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #3d3d5c; }
        """)
        button_layout.addWidget(self.clear_btn)

        button_layout.addStretch()

        self.start_btn = QPushButton("Start Acquisition")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self._start_acquisition)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #4ecdc4;
                border: none;
                border-radius: 4px;
                padding: 12px 30px;
                color: #0f0f1a;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #6ee7df; }
            QPushButton:disabled { background-color: #2d2d44; color: #666; }
        """)
        button_layout.addWidget(self.start_btn)

        layout.addLayout(button_layout)

        # Scan start time
        self._scan_start_time = time.time()

    def on_connection_changed(self, connected, info):
        """Handle connection state changes."""
        if connected:
            self.connection_status.setText("● Connected")
            self.connection_status.setStyleSheet("color: #4ecdc4; font-weight: bold;")
            self.port_label.setText(f"Port: {info}")
        else:
            self.connection_status.setText("● Disconnected")
            self.connection_status.setStyleSheet("color: #ff6b6b; font-weight: bold;")
            self.port_label.setText("Port: --")

    def on_device_discovered(self, device_id, packet_count):
        """Handle device discovery from serial worker."""
        now = time.time()
        if device_id not in self.discovered_devices:
            self.discovered_devices[device_id] = {
                'packets': 0,
                'first_seen': now,
                'last_seen': now,
                'selected': True,  # Auto-select new devices
                'rate': 0.0,
                'last_rate_time': now,
                'last_rate_packets': 0
            }
        dev = self.discovered_devices[device_id]
        dev['packets'] = packet_count
        dev['last_seen'] = now

        # Calculate packet rate
        rate_elapsed = now - dev['last_rate_time']
        if rate_elapsed >= 1.0:
            packets_diff = packet_count - dev['last_rate_packets']
            dev['rate'] = packets_diff / rate_elapsed
            dev['last_rate_time'] = now
            dev['last_rate_packets'] = packet_count

    def _update_device_table(self):
        """Update the device table display."""
        now = time.time()

        # Update scan time
        scan_elapsed = int(now - self._scan_start_time)
        self.scan_time_label.setText(f"Scan time: {scan_elapsed}s")

        # Get sorted device list
        device_ids = sorted(self.discovered_devices.keys())

        # Update table rows
        self.device_table.setRowCount(len(device_ids))

        selected_count = 0
        for row, dev_id in enumerate(device_ids):
            dev = self.discovered_devices[dev_id]
            elapsed = now - dev['last_seen']

            # Checkbox column
            checkbox = QCheckBox()
            checkbox.setChecked(dev['selected'])
            checkbox.stateChanged.connect(lambda state, d=dev_id: self._on_device_checkbox_changed(d, state))
            checkbox_widget = QWidget()
            checkbox_layout = QHBoxLayout(checkbox_widget)
            checkbox_layout.addWidget(checkbox)
            checkbox_layout.setAlignment(Qt.AlignCenter)
            checkbox_layout.setContentsMargins(0, 0, 0, 0)
            self.device_table.setCellWidget(row, 0, checkbox_widget)

            if dev['selected']:
                selected_count += 1

            # Device ID with color indicator
            color = self.DEVICE_COLORS[row % len(self.DEVICE_COLORS)]
            id_item = QTableWidgetItem(f"0x{dev_id}")
            id_item.setForeground(QBrush(color))
            id_item.setFlags(id_item.flags() & ~Qt.ItemIsEditable)
            self.device_table.setItem(row, 1, id_item)

            # Packet count
            packets_item = QTableWidgetItem(str(dev['packets']))
            packets_item.setTextAlignment(Qt.AlignCenter)
            packets_item.setFlags(packets_item.flags() & ~Qt.ItemIsEditable)
            self.device_table.setItem(row, 2, packets_item)

            # Packet rate
            rate_item = QTableWidgetItem(f"{dev['rate']:.1f}")
            rate_item.setTextAlignment(Qt.AlignCenter)
            rate_item.setFlags(rate_item.flags() & ~Qt.ItemIsEditable)
            self.device_table.setItem(row, 3, rate_item)

            # Status indicator
            if elapsed < 0.5:
                status_text = "● Active"
                status_color = "#4ecdc4"
            elif elapsed < 2.0:
                status_text = "● Recent"
                status_color = "#ffcc00"
            else:
                status_text = "○ Inactive"
                status_color = "#666"

            status_item = QTableWidgetItem(status_text)
            status_item.setForeground(QBrush(QColor(status_color)))
            status_item.setTextAlignment(Qt.AlignCenter)
            status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
            self.device_table.setItem(row, 4, status_item)

            # Update signal strength widget
            self.signal_widget.update_device(
                dev_id,
                dev['rate'],
                color,
                dev['selected']
            )

        # Update labels
        self.device_count_label.setText(f"Devices found: {len(device_ids)}")
        self.selected_count_label.setText(f"Selected: {selected_count}")

        # Enable start button if at least one device selected
        self.start_btn.setEnabled(selected_count > 0)

    def _on_device_checkbox_changed(self, device_id, state):
        """Handle device selection checkbox change."""
        if device_id in self.discovered_devices:
            self.discovered_devices[device_id]['selected'] = (state == Qt.Checked or state == 2)

    def _select_all(self):
        """Select all discovered devices."""
        for dev in self.discovered_devices.values():
            dev['selected'] = True
        self._update_device_table()

    def _clear_selection(self):
        """Clear all device selections."""
        for dev in self.discovered_devices.values():
            dev['selected'] = False
        self._update_device_table()

    def _start_acquisition(self):
        """Start data acquisition with selected devices."""
        selected_ids = [
            dev_id for dev_id, dev in self.discovered_devices.items()
            if dev['selected']
        ]
        if selected_ids:
            self.devices_selected.emit(selected_ids)

    def get_selected_devices(self):
        """Get list of selected device IDs."""
        return [
            dev_id for dev_id, dev in self.discovered_devices.items()
            if dev['selected']
        ]

    def reset(self):
        """Reset scanner state for new scan."""
        self.discovered_devices.clear()
        self.device_table.setRowCount(0)
        self.signal_widget.clear()
        self._scan_start_time = time.time()


class WaveformWidget(QFrame):
    """Custom widget for drawing EMG waveform."""

    SAMPLE_RATE = 1000  # Hz (approximate, ~1kHz from uMyo)

    # Colors for different devices
    DEVICE_COLORS = [
        QColor(0, 255, 255),    # Cyan
        QColor(255, 100, 100),  # Red
        QColor(100, 255, 100),  # Green
        QColor(255, 200, 100),  # Orange
        QColor(200, 100, 255),  # Purple
        QColor(255, 255, 100),  # Yellow
        QColor(100, 200, 255),  # Light blue
        QColor(255, 100, 200),  # Pink
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.setStyleSheet("background-color: #1a1a2e; border: 1px solid #16213e;")

        self._window_size = 2000
        self.data = deque(maxlen=self._window_size)  # Legacy single-device data
        self._device_data = {}  # device_id -> deque of samples
        self._device_order = []  # Ordered list of device IDs for consistent coloring
        self.y_min = 0
        self.y_max = 16383
        self.auto_scale = True
        self._display_samples = 1500  # Number of samples to display
        self._show_mv = True  # Show mV instead of raw ADC (default on)
        self._total_samples = 0  # Total samples received for time tracking
        self._separate_plots = True  # Default to separate plots per device

    def set_window_size(self, size):
        """Set the time window size (number of samples to keep)."""
        self._window_size = max(100, size)
        old_data = list(self.data)
        self.data = deque(old_data[-self._window_size:], maxlen=self._window_size)
        # Update multi-device data buffers
        for dev_id in self._device_data:
            old_dev_data = list(self._device_data[dev_id])
            self._device_data[dev_id] = deque(old_dev_data[-self._window_size:], maxlen=self._window_size)
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

    def set_separate_plots(self, enabled):
        """Enable/disable separate plots per device."""
        self._separate_plots = enabled
        self.update()

    def add_samples(self, samples, device_id=None):
        """Add new samples to the waveform.

        Args:
            samples: List of ADC sample values
            device_id: Optional device identifier for multi-device support
        """
        # Add to legacy single buffer for backward compatibility
        for s in samples:
            self.data.append(s)
        self._total_samples += len(samples)

        # Add to device-specific buffer if device_id provided
        if device_id is not None:
            if device_id not in self._device_data:
                self._device_data[device_id] = deque(maxlen=self._window_size)
                self._device_order.append(device_id)
            for s in samples:
                self._device_data[device_id].append(s)

        # Auto-scale Y axis based on all device data
        if self.auto_scale:
            all_recent = []
            if self._device_data:
                for dev_data in self._device_data.values():
                    if len(dev_data) > 0:
                        all_recent.extend(list(dev_data)[-500:])
            else:
                all_recent = list(self.data)[-500:]

            if len(all_recent) > 10:
                self.y_min = min(all_recent) - 100
                self.y_max = max(all_recent) + 100

        self.update()

    def paintEvent(self, event):
        """Draw the waveform."""
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # Determine which data sources to plot
        num_devices = len(self._device_order) if self._device_data else 0

        if self._separate_plots and num_devices > 1:
            # Separate plot mode: divide screen into horizontal bands
            self._draw_separate_plots(painter, w, h, num_devices)
        else:
            # Combined mode or single device: overlay all on one plot
            self._draw_combined_plot(painter, w, h)

    def _draw_grid(self, painter, x, y, w, h):
        """Draw grid lines for a plot region."""
        painter.setPen(QPen(QColor(50, 50, 80), 1))
        for i in range(x, x + w, 100):
            painter.drawLine(i, y, i, y + h)
        for i in range(y, y + h, 50):
            painter.drawLine(x, i, x + w, i)

    def _draw_waveform_line(self, painter, data_list, x_offset, y_offset, plot_w, plot_h, y_min, y_max, color):
        """Draw a single waveform line."""
        if len(data_list) < 2:
            return

        painter.setPen(QPen(color, 1.5))
        n = len(data_list)

        display_count = min(n, self._display_samples)
        x_scale = plot_w / display_count if display_count > 0 else 1
        y_range = y_max - y_min
        if y_range == 0:
            y_range = 1

        start_idx = max(0, n - self._display_samples)

        prev_x, prev_y = None, None
        for i, val in enumerate(data_list[start_idx:]):
            x = x_offset + int(i * x_scale)
            y = y_offset + int(plot_h - (val - y_min) / y_range * plot_h)
            y = max(y_offset, min(y_offset + plot_h - 1, y))

            if prev_x is not None:
                painter.drawLine(prev_x, prev_y, x, y)

            prev_x, prev_y = x, y

    def _draw_y_labels(self, painter, x, y, h, y_min, y_max, color=None):
        """Draw Y-axis labels for a plot region."""
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)

        if self._show_mv:
            y_max_mv = y_max * ADC_TO_MV
            y_min_mv = y_min * ADC_TO_MV
            y_max_uv = y_max_mv * 1000
            y_min_uv = y_min_mv * 1000

            painter.setPen(QPen(color if color else QColor(255, 200, 100), 1))
            font.setPointSize(9)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(x + 5, y + h // 2 - 10, "µV")

            font.setBold(False)
            font.setPointSize(7)
            painter.setFont(font)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawText(x + 5, y + 12, f"{y_max_uv:.0f}")
            painter.drawText(x + 5, y + h - 5, f"{y_min_uv:.0f}")
        else:
            painter.setPen(QPen(color if color else QColor(255, 200, 100), 1))
            font.setPointSize(9)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(x + 5, y + h // 2 - 10, "ADC")

            font.setBold(False)
            font.setPointSize(7)
            painter.setFont(font)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawText(x + 5, y + 12, f"{y_max:.0f}")
            painter.drawText(x + 5, y + h - 5, f"{y_min:.0f}")

    def _draw_x_labels(self, painter, w, h, n):
        """Draw X-axis time labels."""
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        font = painter.font()
        font.setPointSize(8)
        font.setBold(False)
        painter.setFont(font)

        display_count = min(n, self._display_samples) if n > 0 else self._display_samples
        time_span_ms = display_count

        num_ticks = 5
        for i in range(num_ticks + 1):
            x_pos = int(w * i / num_ticks)
            time_val = -time_span_ms + (time_span_ms * i / num_ticks)

            painter.drawLine(x_pos, h - 25, x_pos, h - 20)

            if abs(time_val) >= 1000:
                label = f"{time_val/1000:.1f}s"
            else:
                label = f"{time_val:.0f}ms"

            label_width = len(label) * 6
            label_x = x_pos - label_width // 2
            if i == 0:
                label_x = max(5, label_x)
            elif i == num_ticks:
                label_x = min(w - label_width - 5, label_x)

            painter.drawText(label_x, h - 8, label)

    def _draw_combined_plot(self, painter, w, h):
        """Draw all devices overlaid on a single plot."""
        if len(self.data) < 2 and not self._device_data:
            return

        self._draw_grid(painter, 0, 0, w, h)

        # Draw each device with different color, or legacy data if no devices
        if self._device_data:
            for idx, dev_id in enumerate(self._device_order):
                data_list = list(self._device_data[dev_id])
                color = self.DEVICE_COLORS[idx % len(self.DEVICE_COLORS)]
                self._draw_waveform_line(painter, data_list, 0, 0, w, h,
                                         self.y_min, self.y_max, color)
        else:
            data_list = list(self.data)
            self._draw_waveform_line(painter, data_list, 0, 0, w, h,
                                     self.y_min, self.y_max, self.DEVICE_COLORS[0])

        # Draw labels
        self._draw_y_labels(painter, 0, 0, h, self.y_min, self.y_max)

        # Sample count and device legend
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QPen(QColor(150, 150, 150), 1))
        painter.drawText(w - 100, 15, f"Samples: {len(self.data)}")

        # Draw device legend if multiple devices
        if len(self._device_order) > 1:
            legend_y = 15
            for idx, dev_id in enumerate(self._device_order):
                color = self.DEVICE_COLORS[idx % len(self.DEVICE_COLORS)]
                painter.setPen(QPen(color, 2))
                painter.drawLine(w - 200, legend_y, w - 180, legend_y)
                painter.setPen(QPen(QColor(150, 150, 150), 1))
                painter.drawText(w - 175, legend_y + 4, f"0x{dev_id[:8]}")
                legend_y += 15

        self._draw_x_labels(painter, w, h, len(self.data))

    def _draw_separate_plots(self, painter, w, h, num_devices):
        """Draw each device in its own horizontal band."""
        plot_height = h // num_devices
        margin = 5

        for idx, dev_id in enumerate(self._device_order):
            y_offset = idx * plot_height
            plot_h = plot_height - margin

            # Draw separator line between plots
            if idx > 0:
                painter.setPen(QPen(QColor(80, 80, 120), 2))
                painter.drawLine(0, y_offset, w, y_offset)

            # Draw grid
            self._draw_grid(painter, 0, y_offset, w, plot_h)

            # Get device data
            data_list = list(self._device_data[dev_id])

            # Calculate per-device Y range if auto-scaling
            if self.auto_scale and len(data_list) > 10:
                recent = data_list[-500:]
                dev_y_min = min(recent) - 100
                dev_y_max = max(recent) + 100
            else:
                dev_y_min = self.y_min
                dev_y_max = self.y_max

            # Draw waveform
            color = self.DEVICE_COLORS[idx % len(self.DEVICE_COLORS)]
            self._draw_waveform_line(painter, data_list, 0, y_offset, w, plot_h,
                                     dev_y_min, dev_y_max, color)

            # Draw Y labels
            self._draw_y_labels(painter, 0, y_offset, plot_h, dev_y_min, dev_y_max, color)

            # Draw device ID label
            font = painter.font()
            font.setPointSize(9)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QPen(color, 1))
            painter.drawText(w - 150, y_offset + 15, f"0x{dev_id[:8]}")

            # Draw sample count
            font.setBold(False)
            font.setPointSize(8)
            painter.setFont(font)
            painter.setPen(QPen(QColor(150, 150, 150), 1))
            painter.drawText(w - 150, y_offset + 30, f"n={len(data_list)}")

        # Draw X labels at bottom
        self._draw_x_labels(painter, w, h, len(self.data) if self.data else self._display_samples)


class UMyoPlotterGUI(QMainWindow):
    """Main GUI window for uMyo EMG Plotter."""

    # View indices for stacked widget
    VIEW_SCANNER = 0
    VIEW_ACQUISITION = 1

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

        # Initialize serial worker first (shared between views)
        self._init_serial()

        # Initialize UI with stacked views
        self._init_ui()

        # Update timer for GUI (only active during acquisition)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_stats)

        # Discovered devices
        self.devices = {}
        self.packet_count = 0
        self.sample_count = 0

        # Selected devices for acquisition (set by scanner)
        self.selected_device_ids = []

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

        # Current view state
        self._in_acquisition = False

    def _init_ui(self):
        """Initialize the UI with stacked widget for scanner and acquisition views."""
        # Create stacked widget as central widget
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        # Create scanner view
        self.scanner_widget = DeviceScannerWidget(self.serial_worker)
        self.scanner_widget.devices_selected.connect(self._on_devices_selected)
        self.stack.addWidget(self.scanner_widget)

        # Create acquisition view container
        self.acquisition_widget = QWidget()
        self._init_acquisition_ui()
        self.stack.addWidget(self.acquisition_widget)

        # Start with scanner view
        self.stack.setCurrentIndex(self.VIEW_SCANNER)

        # Connect serial signals to scanner initially
        self.serial_worker.connection_changed.connect(self.scanner_widget.on_connection_changed)
        self.serial_worker.device_discovered.connect(self.scanner_widget.on_device_discovered)

    def _init_acquisition_ui(self):
        """Initialize the data acquisition UI."""
        main_layout = QVBoxLayout(self.acquisition_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Top bar - connection status and back button
        top_bar = QHBoxLayout()

        # Back to scanner button
        self.back_btn = QPushButton("← Back to Scanner")
        self.back_btn.clicked.connect(self._back_to_scanner)
        self.back_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2d44;
                border: 1px solid #3d3d5c;
                border-radius: 4px;
                padding: 8px 16px;
                color: #e0e0e0;
            }
            QPushButton:hover { background-color: #3d3d5c; }
        """)
        top_bar.addWidget(self.back_btn)

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

        self.separate_plot_cb = QCheckBox("Separate")
        self.separate_plot_cb.setChecked(True)  # Default to separate plots
        self.separate_plot_cb.setToolTip("Show each device in a separate plot (unchecked = combined overlay)")
        self.separate_plot_cb.stateChanged.connect(self._on_separate_plot_changed)
        amp_layout.addWidget(self.separate_plot_cb)

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

    def _on_devices_selected(self, device_ids):
        """Handle device selection from scanner - switch to acquisition view."""
        self.selected_device_ids = device_ids
        self._in_acquisition = True

        # Reset acquisition state
        self.packet_count = 0
        self.sample_count = 0
        self.packet_loss_count = 0
        self.total_expected_packets = 0
        self.device_last_adc_id.clear()
        self.devices.clear()
        self._rate_start_time = None
        self._rate_start_samples = 0
        self._current_sample_rate = 0.0
        self._session_start_time = time.time()
        self._stats_history.clear()
        self._last_stats_record_time = 0

        # Clear waveform data
        self.waveform.data.clear()
        self.waveform._device_data.clear()
        self.waveform._device_order.clear()
        self.waveform._total_samples = 0

        # Update window title with device count
        self.setWindowTitle(f"uMyo EMG Plotter - {len(device_ids)} device(s)")

        # Switch to acquisition view
        self.stack.setCurrentIndex(self.VIEW_ACQUISITION)

        # Start stats update timer
        self.update_timer.start(500)

    def _back_to_scanner(self):
        """Go back to scanner view from acquisition."""
        self._in_acquisition = False

        # Stop stats timer
        self.update_timer.stop()

        # Show statistics plot if we have data
        if len(self._stats_history) > 5:
            self._show_session_stats_plot()

        # Reset scanner
        self.scanner_widget.reset()

        # Update window title
        self.setWindowTitle("uMyo EMG Plotter")

        # Switch to scanner view
        self.stack.setCurrentIndex(self.VIEW_SCANNER)

    def _on_data(self, line):
        """Handle received data line."""
        # Only process data in acquisition mode
        if not self._in_acquisition:
            return

        # Update terminal (if visible)
        if hasattr(self, 'autoscroll_btn') and self.autoscroll_btn.isChecked():
            self.terminal.append(line)
            self.terminal.verticalScrollBar().setValue(
                self.terminal.verticalScrollBar().maximum()
            )

        # Parse device ID first to filter
        id_match = re.search(r'ID=0x([0-9A-Fa-f]+)', line)
        if not id_match:
            return

        dev_id = id_match.group(1)

        # Filter by selected devices
        if dev_id not in self.selected_device_ids:
            return

        # Parse ADC data and track packet loss
        # Format: RX: ID=0x40B3A92E ADC_ID=33 ADC:5815,5775,5730,... SP:0,0,0,0
        adc_match = re.search(r'ADC:([-\d,]+)', line)
        if adc_match:
            try:
                samples = [int(x) for x in adc_match.group(1).split(',')]
                # Pass device_id for multi-device plotting
                self.waveform.add_samples(samples, device_id=dev_id)
                self.sample_count += len(samples)
            except:
                pass

        if line.startswith('RX:'):
            self.packet_count += 1

            # Track packet loss using ADC_ID sequence
            adc_id_match = re.search(r'ADC_ID=(\d+)', line)

            if adc_id_match:
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
        # Update acquisition view status labels if they exist
        if hasattr(self, 'status_label'):
            if connected:
                self.status_label.setText("● Connected")
                self.status_label.setStyleSheet("color: #4ecdc4; font-weight: bold; font-size: 14px;")
                self.port_label.setText(f"Port: {info}")
                if self._in_acquisition:
                    self._log(f"[{self._timestamp()}] Connected to {info}")
            else:
                self.status_label.setText("● Disconnected")
                self.status_label.setStyleSheet("color: #ff6b6b; font-weight: bold; font-size: 14px;")
                self.port_label.setText("Port: --")
                if self._in_acquisition:
                    self._log(f"[{self._timestamp()}] {info}")

    def _on_device_discovered(self, device_id, packet_count):
        """Handle device discovery."""
        # Only update acquisition view's device list when in acquisition mode
        if not self._in_acquisition:
            return

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

    def _on_separate_plot_changed(self, state):
        """Handle separate plot checkbox change."""
        # state can be Qt.CheckState.Checked (2) or Qt.Checked depending on PySide6 version
        self.waveform.set_separate_plots(state == 2 or state == Qt.Checked)

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
