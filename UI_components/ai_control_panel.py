#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse AI Control Panel - PySide6 widget for depRL real-time control.

Provides:
- Configuration (checkpoint path, frequency, filter, safety limits)
- Control buttons (Initialize / Start / Stop / Emergency Stop)
- Real-time status indicators and plots
- Log output
"""

import sys
import logging
from pathlib import Path
from collections import deque

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QGroupBox,
    QTextEdit, QFileDialog, QSpinBox, QDoubleSpinBox,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import numpy as np

# Add project root to path for imports
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from control.ai_agent.realtime_controller import (
    RealtimeController,
    RealtimeControllerConfig,
    ControllerState,
)


# ─── Qt Log Handler ──────────────────────────────────────────────

class QtLogHandler(logging.Handler):
    """Routes log messages to a QTextEdit widget."""

    def __init__(self, text_widget: QTextEdit):
        super().__init__()
        self._widget = text_widget

    def emit(self, record):
        msg = self.format(record)
        color = "#ECF0F1"
        if record.levelno >= logging.ERROR:
            color = "#E74C3C"
        elif record.levelno >= logging.WARNING:
            color = "#F39C12"
        elif record.levelno >= logging.INFO:
            color = "#2ECC71"
        self._widget.append(f"<span style='color:{color};'>{msg}</span>")
        sb = self._widget.verticalScrollBar()
        sb.setValue(sb.maximum())


# ─── AI Control Panel Widget ─────────────────────────────────────

class AIControlPanel(QWidget):
    """PySide6 widget for AI-based exoskeleton control via depRL."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self._controller = None
        self._telemetry_history = {
            "torque_r": deque(maxlen=200),
            "torque_l": deque(maxlen=200),
            "loop_hz": deque(maxlen=200),
        }

        self._init_ui()
        self._init_plots()

        # Telemetry poll timer (20 Hz)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_telemetry)
        self._timer.setInterval(50)

    # ── UI Construction ──

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title = QLabel("AI Control (depRL)")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet("color: #5DADE2; margin-bottom: 5px;")
        main_layout.addWidget(title)

        # ── Top row: Config + Controls ──
        top_layout = QHBoxLayout()

        # Config group
        config_group = QGroupBox("Configuration")
        config_layout = QGridLayout()

        # Checkpoint path
        config_layout.addWidget(QLabel("Checkpoint:"), 0, 0)
        self._checkpoint_edit = QLineEdit()
        self._checkpoint_edit.setPlaceholderText("Path to depRL checkpoint directory")
        config_layout.addWidget(self._checkpoint_edit, 0, 1)
        browse_btn = QPushButton("...")
        browse_btn.setMaximumWidth(40)
        browse_btn.clicked.connect(self._browse_checkpoint)
        config_layout.addWidget(browse_btn, 0, 2)

        # Frequency
        config_layout.addWidget(QLabel("Frequency (Hz):"), 1, 0)
        self._freq_spin = QSpinBox()
        self._freq_spin.setRange(10, 200)
        self._freq_spin.setValue(50)
        config_layout.addWidget(self._freq_spin, 1, 1)

        # Filter type
        config_layout.addWidget(QLabel("Filter:"), 2, 0)
        self._filter_combo = QComboBox()
        self._filter_combo.addItems(["ema", "butterworth"])
        config_layout.addWidget(self._filter_combo, 2, 1)

        # IQ limit
        config_layout.addWidget(QLabel("Torque Limit:"), 3, 0)
        self._iq_spin = QSpinBox()
        self._iq_spin.setRange(0, 800)
        self._iq_spin.setValue(800)
        self._iq_spin.setSingleStep(50)
        config_layout.addWidget(self._iq_spin, 3, 1)

        # Ramp time
        config_layout.addWidget(QLabel("Ramp (sec):"), 4, 0)
        self._ramp_spin = QDoubleSpinBox()
        self._ramp_spin.setRange(0.0, 10.0)
        self._ramp_spin.setValue(2.0)
        self._ramp_spin.setSingleStep(0.5)
        config_layout.addWidget(self._ramp_spin, 4, 1)

        config_group.setLayout(config_layout)
        top_layout.addWidget(config_group, stretch=2)

        # Control buttons group
        ctrl_group = QGroupBox("Control")
        ctrl_layout = QVBoxLayout()

        self._init_btn = QPushButton("Initialize")
        self._init_btn.setMinimumHeight(40)
        self._init_btn.setStyleSheet(self._btn_style("#3498DB", "#2980B9"))
        self._init_btn.clicked.connect(self._on_initialize)
        ctrl_layout.addWidget(self._init_btn)

        self._start_btn = QPushButton("Start")
        self._start_btn.setMinimumHeight(40)
        self._start_btn.setEnabled(False)
        self._start_btn.setStyleSheet(self._btn_style("#27AE60", "#229954"))
        self._start_btn.clicked.connect(self._on_start)
        ctrl_layout.addWidget(self._start_btn)

        self._stop_btn = QPushButton("Stop")
        self._stop_btn.setMinimumHeight(40)
        self._stop_btn.setEnabled(False)
        self._stop_btn.setStyleSheet(self._btn_style("#F39C12", "#D68910"))
        self._stop_btn.clicked.connect(self._on_stop)
        ctrl_layout.addWidget(self._stop_btn)

        self._estop_btn = QPushButton("EMERGENCY STOP")
        self._estop_btn.setMinimumHeight(50)
        self._estop_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:1 #C0392B);
                color: white; border: 2px solid #922B21;
                border-radius: 5px; font-size: 14pt; font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #EC7063, stop:1 #E74C3C);
            }
        """)
        self._estop_btn.clicked.connect(self._on_emergency_stop)
        ctrl_layout.addWidget(self._estop_btn)

        ctrl_group.setLayout(ctrl_layout)
        top_layout.addWidget(ctrl_group, stretch=1)

        main_layout.addLayout(top_layout)

        # ── Status bar ──
        status_group = QGroupBox("Status")
        status_layout = QHBoxLayout()

        self._state_label = QLabel("IDLE")
        self._state_label.setStyleSheet("color: #7F8C8D; font-weight: bold; font-size: 12pt;")
        status_layout.addWidget(self._state_label)

        status_layout.addSpacing(20)

        self._hz_label = QLabel("Hz: --")
        self._hz_label.setStyleSheet("color: #ECF0F1;")
        status_layout.addWidget(self._hz_label)

        self._infer_label = QLabel("Inference: -- ms")
        self._infer_label.setStyleSheet("color: #ECF0F1;")
        status_layout.addWidget(self._infer_label)

        self._torque_label = QLabel("Torque R: -- / L: --")
        self._torque_label.setStyleSheet("color: #ECF0F1;")
        status_layout.addWidget(self._torque_label)

        self._sensor_label = QLabel("IMU: -- | Motor: -- | EMG: --")
        self._sensor_label.setStyleSheet("color: #ECF0F1;")
        status_layout.addWidget(self._sensor_label)

        status_layout.addStretch()
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # ── Plots ──
        self._plot_widget = QWidget()
        self._plot_layout = QVBoxLayout(self._plot_widget)
        self._plot_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(self._plot_widget, stretch=3)

        # ── Log ──
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(150)
        self._log_text.setStyleSheet(
            "background-color: #1a252f; color: #ECF0F1; "
            "font-family: monospace; border: 1px solid #34495E;"
        )
        log_layout.addWidget(self._log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # Setup logging to log widget
        handler = QtLogHandler(self._log_text)
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s: %(message)s", "%H:%M:%S"))
        logging.getLogger("ai_agent").addHandler(handler)
        logging.getLogger("ai_agent").setLevel(logging.INFO)

    def _init_plots(self):
        """Create matplotlib figure with torque and Hz plots."""
        self._fig, (self._ax_torque, self._ax_hz) = plt.subplots(
            2, 1, figsize=(8, 4), facecolor="#2C3E50"
        )
        self._fig.subplots_adjust(hspace=0.4, left=0.1, right=0.95, top=0.92, bottom=0.1)

        for ax in (self._ax_torque, self._ax_hz):
            ax.set_facecolor("#1a252f")
            ax.tick_params(colors="#ECF0F1", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#34495E")

        self._ax_torque.set_title("Torque Commands", color="#ECF0F1", fontsize=10)
        self._ax_torque.set_ylabel("iq", color="#ECF0F1", fontsize=9)
        self._line_tr, = self._ax_torque.plot([], [], color="#E74C3C", linewidth=1.5, label="Right")
        self._line_tl, = self._ax_torque.plot([], [], color="#3498DB", linewidth=1.5, label="Left")
        self._ax_torque.legend(loc="upper right", fontsize=8, facecolor="#2C3E50",
                               edgecolor="#34495E", labelcolor="#ECF0F1")

        self._ax_hz.set_title("Loop Frequency", color="#ECF0F1", fontsize=10)
        self._ax_hz.set_ylabel("Hz", color="#ECF0F1", fontsize=9)
        self._line_hz, = self._ax_hz.plot([], [], color="#2ECC71", linewidth=1.5)

        self._canvas = FigureCanvas(self._fig)
        self._plot_layout.addWidget(self._canvas)

    # ── Button Actions ──

    def _on_initialize(self):
        cfg = RealtimeControllerConfig(
            checkpoint_path=self._checkpoint_edit.text().strip(),
            control_freq_hz=self._freq_spin.value(),
            iq_limit=self._iq_spin.value(),
            filter_type=self._filter_combo.currentText(),
            torque_ramp_seconds=self._ramp_spin.value(),
        )

        self._controller = RealtimeController(cfg)
        self._controller.on_state_changed = self._on_state_changed

        self._init_btn.setEnabled(False)
        self._log_text.append("<span style='color:#5DADE2;'>Initializing ...</span>")

        # Run initialization (may take a few seconds to load model)
        success = self._controller.initialize()

        self._init_btn.setEnabled(True)
        if success:
            self._start_btn.setEnabled(True)
            self._update_state_label(ControllerState.IDLE)
        else:
            self._update_state_label(ControllerState.ERROR)

    def _on_start(self):
        if self._controller:
            self._controller.start()
            self._start_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
            self._timer.start()

    def _on_stop(self):
        if self._controller:
            self._controller.stop()
            self._timer.stop()
            self._start_btn.setEnabled(True)
            self._stop_btn.setEnabled(False)

    def _on_emergency_stop(self):
        if self._controller:
            self._controller.emergency_stop()
            self._timer.stop()
            self._start_btn.setEnabled(False)
            self._stop_btn.setEnabled(False)
            self._init_btn.setEnabled(True)
            self._update_state_label(ControllerState.STOPPED)

    def _browse_checkpoint(self):
        path = QFileDialog.getExistingDirectory(self, "Select Checkpoint Directory")
        if path:
            self._checkpoint_edit.setText(path)

    # ── Telemetry Update (called by QTimer at 20 Hz) ──

    def _update_telemetry(self):
        if not self._controller or not self._controller.is_running:
            return

        t = self._controller.get_telemetry()

        # Update labels
        self._hz_label.setText(f"Hz: {t['loop_hz']}")
        self._infer_label.setText(f"Inference: {t['inference_ms']} ms")
        self._torque_label.setText(f"Torque R: {t['torque_right']:.0f} / L: {t['torque_left']:.0f}")

        imu = "OK" if t["imu_ok"] else "--"
        motor = "OK" if t["motor_ok"] else "--"
        emg = "OK" if t["emg_ok"] else "--"
        self._sensor_label.setText(f"IMU: {imu} | Motor: {motor} | EMG: {emg}")

        # Update history
        self._telemetry_history["torque_r"].append(t["torque_right"])
        self._telemetry_history["torque_l"].append(t["torque_left"])
        self._telemetry_history["loop_hz"].append(t["loop_hz"])

        # Update plots (every 5th call ~ 4 Hz plot update)
        if len(self._telemetry_history["torque_r"]) % 5 == 0:
            self._update_plots()

    def _update_plots(self):
        tr = list(self._telemetry_history["torque_r"])
        tl = list(self._telemetry_history["torque_l"])
        hz = list(self._telemetry_history["loop_hz"])
        x = list(range(len(tr)))

        self._line_tr.set_data(x, tr)
        self._line_tl.set_data(x, tl)
        self._ax_torque.set_xlim(0, max(len(x), 1))
        if tr:
            ymin = min(min(tr), min(tl), -10)
            ymax = max(max(tr), max(tl), 10)
            self._ax_torque.set_ylim(ymin - 20, ymax + 20)

        x_hz = list(range(len(hz)))
        self._line_hz.set_data(x_hz, hz)
        self._ax_hz.set_xlim(0, max(len(x_hz), 1))
        if hz:
            self._ax_hz.set_ylim(0, max(max(hz), 10) + 10)

        self._canvas.draw_idle()

    # ── State Management ──

    def _on_state_changed(self, new_state: ControllerState):
        """Callback from controller (may be from background thread)."""
        self._update_state_label(new_state)

    def _update_state_label(self, state: ControllerState):
        colors = {
            ControllerState.IDLE: "#7F8C8D",
            ControllerState.INITIALIZING: "#F39C12",
            ControllerState.RUNNING: "#2ECC71",
            ControllerState.ERROR: "#E74C3C",
            ControllerState.STOPPED: "#95A5A6",
        }
        self._state_label.setText(state.value)
        self._state_label.setStyleSheet(
            f"color: {colors.get(state, '#ECF0F1')}; font-weight: bold; font-size: 12pt;"
        )

    # ── Helpers ──

    @staticmethod
    def _btn_style(c1, c2):
        return f"""
            QPushButton {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 {c1}, stop:1 {c2});
                color: white; border: none; border-radius: 5px;
                font-size: 12pt; font-weight: bold;
            }}
            QPushButton:hover {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 {c2}, stop:1 {c1});
            }}
            QPushButton:disabled {{
                background-color: rgba(0, 0, 0, 0.2); color: #7F8C8D;
            }}
        """

    def closeEvent(self, event):
        """Cleanup on widget close."""
        if self._controller:
            self._controller.shutdown()
        if self._timer.isActive():
            self._timer.stop()
        super().closeEvent(event)
