#!/usr/bin/env python3
"""
ExoDataViewer - Standalone window for displaying TCP exo data from ExoPulse Studio

Shows real-time plots of:
- Left/Right hip angles (degrees)
- Left/Right hip torques (Nm)
"""

import sys
import time
import json
import socket
import threading
from collections import deque
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QSpinBox, QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont

import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class DataSignals(QObject):
    """Signals for thread-safe data updates"""
    data_received = Signal(dict)
    connection_status = Signal(str)


class ExoDataViewer(QWidget):
    """Standalone window for viewing TCP exo data"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Exo Data Viewer - TCP Stream")
        self.setMinimumSize(800, 600)

        # Data storage (time-based, max 1000 points)
        self.max_points = 1000
        self.data = {
            'time': deque(maxlen=self.max_points),
            'la': deque(maxlen=self.max_points),  # Left angle (deg)
            'ra': deque(maxlen=self.max_points),  # Right angle (deg)
            'lt': deque(maxlen=self.max_points),  # Left torque (Nm)
            'rt': deque(maxlen=self.max_points),  # Right torque (Nm)
        }
        self.start_time = time.time()
        self.last_data_time = 0
        self.packet_count = 0

        # TCP connection
        self.tcp_sock = None
        self.running = False
        self.connected = False
        self.recv_thread = None

        # Signals for thread-safe updates
        self.signals = DataSignals()
        self.signals.data_received.connect(self._on_data_received)
        self.signals.connection_status.connect(self._on_connection_status)

        # Time window
        self.time_window = 10  # seconds

        self._init_ui()
        self._init_plots()

        # Update timer for plots (20 Hz)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_plots)

        # Auto-connect after a short delay (allow window to show first)
        QTimer.singleShot(500, self._connect)

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        # Header
        header_layout = QHBoxLayout()

        title = QLabel("TCP Exo Data Stream")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        header_layout.addWidget(title)

        header_layout.addStretch()

        # Connection status
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: #888; font-weight: bold;")
        header_layout.addWidget(self.status_label)

        # Packet counter
        self.packet_label = QLabel("Packets: 0")
        self.packet_label.setStyleSheet("color: #666;")
        header_layout.addWidget(self.packet_label)

        layout.addLayout(header_layout)

        # Controls
        controls_layout = QHBoxLayout()

        controls_layout.addWidget(QLabel("Time Window:"))
        self.time_spinbox = QSpinBox()
        self.time_spinbox.setRange(5, 60)
        self.time_spinbox.setValue(10)
        self.time_spinbox.setSuffix(" s")
        self.time_spinbox.valueChanged.connect(lambda v: setattr(self, 'time_window', v))
        controls_layout.addWidget(self.time_spinbox)

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self._clear_data)
        controls_layout.addWidget(self.clear_btn)

        controls_layout.addStretch()

        # Current values display
        values_frame = QFrame()
        values_frame.setStyleSheet("background-color: #2C3E50; border-radius: 5px; padding: 5px;")
        values_layout = QHBoxLayout(values_frame)
        values_layout.setContentsMargins(10, 5, 10, 5)

        self.la_label = QLabel("L Angle: --")
        self.la_label.setStyleSheet("color: #3498DB; font-weight: bold;")
        values_layout.addWidget(self.la_label)

        self.ra_label = QLabel("R Angle: --")
        self.ra_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        values_layout.addWidget(self.ra_label)

        self.lt_label = QLabel("L Torque: --")
        self.lt_label.setStyleSheet("color: #2ECC71; font-weight: bold;")
        values_layout.addWidget(self.lt_label)

        self.rt_label = QLabel("R Torque: --")
        self.rt_label.setStyleSheet("color: #F39C12; font-weight: bold;")
        values_layout.addWidget(self.rt_label)

        controls_layout.addWidget(values_frame)

        layout.addLayout(controls_layout)

        # Plot area
        self.canvas_widget = QWidget()
        layout.addWidget(self.canvas_widget, stretch=1)

    def _init_plots(self):
        """Initialize matplotlib plots"""
        # Create figure with dark theme
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(10, 6), facecolor='#1a1a2e')
        self.canvas = FigureCanvas(self.fig)

        # Replace placeholder widget
        layout = QVBoxLayout(self.canvas_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # Create 2x2 subplot grid
        self.axes = {}
        self.lines = {}

        # Left Angle
        ax1 = self.fig.add_subplot(2, 2, 1)
        ax1.set_title('Left Hip Angle', color='#3498DB', fontsize=10)
        ax1.set_ylabel('Angle (°)', fontsize=9)
        ax1.set_facecolor('#16213e')
        ax1.grid(True, alpha=0.3)
        self.axes['la'] = ax1
        self.lines['la'], = ax1.plot([], [], color='#3498DB', linewidth=1.5)

        # Right Angle
        ax2 = self.fig.add_subplot(2, 2, 2)
        ax2.set_title('Right Hip Angle', color='#E74C3C', fontsize=10)
        ax2.set_ylabel('Angle (°)', fontsize=9)
        ax2.set_facecolor('#16213e')
        ax2.grid(True, alpha=0.3)
        self.axes['ra'] = ax2
        self.lines['ra'], = ax2.plot([], [], color='#E74C3C', linewidth=1.5)

        # Left Torque
        ax3 = self.fig.add_subplot(2, 2, 3)
        ax3.set_title('Left Hip Torque', color='#2ECC71', fontsize=10)
        ax3.set_xlabel('Time (s)', fontsize=9)
        ax3.set_ylabel('Torque (Nm)', fontsize=9)
        ax3.set_facecolor('#16213e')
        ax3.grid(True, alpha=0.3)
        self.axes['lt'] = ax3
        self.lines['lt'], = ax3.plot([], [], color='#2ECC71', linewidth=1.5)

        # Right Torque
        ax4 = self.fig.add_subplot(2, 2, 4)
        ax4.set_title('Right Hip Torque', color='#F39C12', fontsize=10)
        ax4.set_xlabel('Time (s)', fontsize=9)
        ax4.set_ylabel('Torque (Nm)', fontsize=9)
        ax4.set_facecolor('#16213e')
        ax4.grid(True, alpha=0.3)
        self.axes['rt'] = ax4
        self.lines['rt'], = ax4.plot([], [], color='#F39C12', linewidth=1.5)

        self.fig.tight_layout()

    def _connect(self):
        """Start TCP receiver (auto-reconnect enabled)"""
        self.running = True
        self.start_time = time.time()
        self.packet_count = 0

        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

        self.update_timer.start(50)  # 20 Hz update

    def _disconnect(self):
        """Stop TCP receiver"""
        self.running = False
        self.update_timer.stop()

        if self.tcp_sock:
            try:
                self.tcp_sock.close()
            except:
                pass
            self.tcp_sock = None

        self.connected = False

    def _receive_loop(self):
        """TCP receive loop (runs in background thread)"""
        HOST = "127.0.0.1"
        PORT = 9998
        buffer = ""

        while self.running:
            # Connect if needed
            if self.tcp_sock is None:
                try:
                    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.tcp_sock.settimeout(2.0)
                    self.tcp_sock.connect((HOST, PORT))
                    self.tcp_sock.settimeout(0.5)
                    self.connected = True
                    self.signals.connection_status.emit(f"Connected to {HOST}:{PORT}")
                except Exception as e:
                    self.tcp_sock = None
                    self.connected = False
                    self.signals.connection_status.emit(f"Connection failed: {e}")
                    time.sleep(1)
                    continue

            # Receive data
            try:
                data = self.tcp_sock.recv(4096)
                if not data:
                    self.signals.connection_status.emit("Server closed connection")
                    self.tcp_sock.close()
                    self.tcp_sock = None
                    self.connected = False
                    continue

                buffer += data.decode('utf-8', errors='ignore')

                # Debug: log first received data
                if not hasattr(self, '_first_recv_logged'):
                    self._first_recv_logged = True
                    print(f"[ExoDataViewer] First recv: {len(data)} bytes", flush=True)

                # Process complete lines
                lines_processed = 0
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            exo_data = json.loads(line)
                            lines_processed += 1
                            # Debug: log every 100th packet
                            if lines_processed == 1 or (hasattr(self, '_total_lines') and self._total_lines % 100 == 0):
                                print(f"[ExoDataViewer] 📨 Parsed: la={exo_data.get('la', 0):.4f}, ra={exo_data.get('ra', 0):.4f}", flush=True)
                            if not hasattr(self, '_total_lines'):
                                self._total_lines = 0
                            self._total_lines += 1
                            self.signals.data_received.emit(exo_data)
                        except json.JSONDecodeError as e:
                            print(f"[ExoDataViewer] JSON error: {e}, line: {line[:50]}", flush=True)

            except socket.timeout:
                continue
            except Exception as e:
                self.signals.connection_status.emit(f"Receive error: {e}")
                if self.tcp_sock:
                    try:
                        self.tcp_sock.close()
                    except:
                        pass
                    self.tcp_sock = None
                self.connected = False

    def _on_data_received(self, exo_data: dict):
        """Handle received exo data (called in main thread)"""
        import math

        elapsed = time.time() - self.start_time
        self.last_data_time = time.time()
        self.packet_count += 1

        # Debug: log every 100th packet in main thread
        if self.packet_count == 1 or self.packet_count % 100 == 0:
            print(f"[ExoDataViewer] ✅ _on_data_received #{self.packet_count}, data points: {len(self.data['time'])}", flush=True)

        # Extract and convert data
        la = math.degrees(exo_data.get('la', 0))
        ra = math.degrees(exo_data.get('ra', 0))
        lt = exo_data.get('lt', 0) or 0
        rt = exo_data.get('rt', 0) or 0

        # Store data
        self.data['time'].append(elapsed)
        self.data['la'].append(la)
        self.data['ra'].append(ra)
        self.data['lt'].append(lt)
        self.data['rt'].append(rt)

        # Update value labels
        self.la_label.setText(f"L Angle: {la:6.1f}°")
        self.ra_label.setText(f"R Angle: {ra:6.1f}°")
        self.lt_label.setText(f"L Torque: {lt:5.2f} Nm")
        self.rt_label.setText(f"R Torque: {rt:5.2f} Nm")

        self.packet_label.setText(f"Packets: {self.packet_count}")

    def _on_connection_status(self, status: str):
        """Update connection status label"""
        if "Connected to" in status:
            self.status_label.setStyleSheet("color: #2ECC71; font-weight: bold;")
        elif "failed" in status or "error" in status or "Disconnected" in status:
            self.status_label.setStyleSheet("color: #E74C3C; font-weight: bold;")
        else:
            self.status_label.setStyleSheet("color: #F39C12; font-weight: bold;")

        self.status_label.setText(status)

    def _update_plots(self):
        """Update all plots"""
        if len(self.data['time']) < 2:
            return

        t = list(self.data['time'])
        t_max = t[-1]
        t_min = max(0, t_max - self.time_window)

        for key in ['la', 'ra', 'lt', 'rt']:
            values = list(self.data[key])
            self.lines[key].set_data(t, values)

            ax = self.axes[key]
            ax.set_xlim(t_min, t_max + 0.5)

            # Auto-scale y-axis
            # Get visible data
            visible_vals = [v for i, v in enumerate(values) if t[i] >= t_min]
            if visible_vals:
                v_min, v_max = min(visible_vals), max(visible_vals)
                margin = (v_max - v_min) * 0.1 + 0.1
                ax.set_ylim(v_min - margin, v_max + margin)

        self.canvas.draw_idle()

    def _clear_data(self):
        """Clear all data"""
        for key in self.data:
            self.data[key].clear()
        self.start_time = time.time()
        self.packet_count = 0
        self.packet_label.setText("Packets: 0")

    def closeEvent(self, event):
        """Handle window close"""
        self._disconnect()
        super().closeEvent(event)


# Standalone test
if __name__ == '__main__':
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    viewer = ExoDataViewer()
    viewer.show()
    sys.exit(app.exec())
