#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Unified GUI with Integrated Motor Control

Main interface with dynamic sidebar based on motor mode selection.
"""

import sys
from pathlib import Path

# Import the motor control GUI
sys.path.insert(0, str(Path(__file__).parent / "UI_components"))
from motor_control import ExoPulseGUI as MotorControlWidget

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QStackedWidget, QScrollArea,
    QSplitter, QGroupBox, QTextEdit
)
from PySide6.QtCore import Qt, QProcess
from PySide6.QtGui import QFont


class ComponentLauncher(QWidget):
    """Widget for launching external UI components"""

    def __init__(self, component_name, display_name, description, script_path):
        super().__init__()
        self.component_name = component_name
        self.display_name = display_name
        self.description = description
        self.script_path = script_path
        self.process = None

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(20, 20, 20, 20)

        # Title
        title = QLabel(self.display_name)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Description
        desc = QLabel(self.description)
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #95A5A6; margin-bottom: 20px;")
        layout.addWidget(desc)

        # Script path
        path_label = QLabel(f"Script: {self.script_path.name}")
        path_label.setStyleSheet("color: #7F8C8D; font-family: monospace; font-size: 10pt;")
        layout.addWidget(path_label)

        layout.addSpacing(20)

        # Launch button
        self.launch_btn = QPushButton("▶ Launch Component")
        self.launch_btn.setMinimumHeight(50)
        self.launch_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #27AE60, stop:1 #229954);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14pt;
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
        self.launch_btn.clicked.connect(self._launch)
        layout.addWidget(self.launch_btn)

        # Stop button
        self.stop_btn = QPushButton("■ Stop Component")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #E74C3C, stop:1 #C0392B);
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12pt;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #EC7063, stop:1 #E74C3C);
            }
            QPushButton:disabled {
                background-color: rgba(0, 0, 0, 0.2);
                color: #7F8C8D;
            }
        """)
        self.stop_btn.clicked.connect(self._stop)
        layout.addWidget(self.stop_btn)

        layout.addSpacing(20)

        # Status label
        self.status_label = QLabel("Status: Not running")
        self.status_label.setStyleSheet("color: #7F8C8D; font-weight: bold;")
        layout.addWidget(self.status_label)

        # Log output
        log_group = QGroupBox("Process Output")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("background-color: #1a252f; color: #ECF0F1; font-family: monospace; border: 1px solid #34495E;")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        layout.addStretch()
        self.setLayout(layout)

    def _launch(self):
        """Launch the component as a subprocess"""
        if not self.script_path.exists():
            self.log_text.append(f"ERROR: Script not found: {self.script_path}")
            return

        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self._on_stdout)
        self.process.readyReadStandardError.connect(self._on_stderr)
        self.process.finished.connect(self._on_finished)

        self.process.start(sys.executable, [str(self.script_path)])

        self.launch_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status_label.setText("Status: Running")
        self.status_label.setStyleSheet("color: #27AE60; font-weight: bold;")
        self.log_text.append(f"[{self.component_name}] Launching...\n")

    def _stop(self):
        """Stop the running component"""
        if self.process and self.process.state() == QProcess.Running:
            self.process.terminate()
            self.log_text.append(f"\n[{self.component_name}] Terminating...\n")

    def _on_stdout(self):
        """Handle standard output"""
        data = self.process.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        self.log_text.append(data.rstrip())
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _on_stderr(self):
        """Handle standard error"""
        data = self.process.readAllStandardError().data().decode('utf-8', errors='ignore')
        self.log_text.append(f"<span style='color: #d32f2f;'>{data.rstrip()}</span>")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _on_finished(self, exit_code, exit_status):
        """Handle process finished"""
        self.launch_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText(f"Status: Stopped (exit code: {exit_code})")
        self.status_label.setStyleSheet("color: #7F8C8D; font-weight: bold;")
        self.log_text.append(f"\n[{self.component_name}] Process finished with exit code {exit_code}\n")


class SidebarButton(QPushButton):
    """Styled sidebar navigation button"""

    def __init__(self, text, icon_text=""):
        super().__init__(f"{icon_text}  {text}")
        self.setCheckable(True)
        self.setMinimumHeight(50)
        self._update_style()

    def _update_style(self):
        """Update button style based on enabled state"""
        self.setStyleSheet("""
            QPushButton {
                text-align: left;
                padding: 10px 20px;
                border: none;
                background-color: transparent;
                color: #ECF0F1;
                font-size: 11pt;
            }
            QPushButton:hover:enabled {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 rgba(52, 152, 219, 0.3), stop:1 rgba(41, 128, 185, 0.3));
            }
            QPushButton:checked {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                color: white;
                font-weight: bold;
                border-left: 3px solid #ECF0F1;
            }
            QPushButton:disabled {
                color: #7F8C8D;
                background-color: rgba(0, 0, 0, 0.2);
            }
        """)

    def setEnabled(self, enabled):
        """Override setEnabled to update style"""
        super().setEnabled(enabled)
        self._update_style()


class ExoPulseUnifiedGUI(QMainWindow):
    """Main unified GUI with integrated motor control and dynamic sidebar"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ExoPulse Unified Control Panel")
        self.setGeometry(100, 100, 1400, 900)

        self.ui_dir = Path(__file__).parent / "UI_components"

        # Initialize collections
        self.sidebar_buttons = []
        self.content_stack = QStackedWidget()

        # Component configurations (name, display, description, script, requires_can_mode)
        self.components = [
            # Always available - Main control (now with integrated monitoring)
            ("motor_control", "Motor Control & Monitor", "Integrated motor control and monitoring interface", None, False, False),

            # High-level monitoring tools
            ("dual_motor_plotter", "Dual Motor Plotter", "Real-time dual motor visualization (CAN only)", "dual_motor_plotter.py", True, False),
            ("can_plotter", "CAN Plotter", "CAN bus data visualization", "can_plotter.py", True, False),

            # WiFi tools
            ("wifi_monitor", "WiFi Monitor", "WiFi-based remote monitoring", "wifi_monitor.py", False, True),
            ("wifi_dual_motor_plotter", "WiFi Dual Motor Plot", "WiFi dual motor plotter", "wifi_dual_motor_plotter.py", False, True),

            # Independent tools
            ("serial_reader", "Serial Reader", "Simple serial port data reader", "serial_reader.py", False, False),
            ("emg_plotter", "EMG Plotter", "EMG signal visualization", "emg_plotter.py", False, False),
        ]

        self._init_ui()

        # Connect motor control signals
        self._connect_motor_control_signals()

    def _init_ui(self):
        """Initialize the user interface"""

        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Create splitter
        splitter = QSplitter(Qt.Horizontal)

        # === LEFT SIDEBAR ===
        sidebar = self._create_sidebar()
        splitter.addWidget(sidebar)

        # === RIGHT CONTENT AREA ===
        # First page: Motor Control (embedded)
        self.motor_control = MotorControlWidget()
        self.content_stack.addWidget(self.motor_control)

        # Additional pages: Component launchers
        for comp in self.components[1:]:  # Skip motor_control (already added)
            comp_id, display_name, description, script_name = comp[:4]
            script_path = self.ui_dir / script_name
            launcher = ComponentLauncher(comp_id, display_name, description, script_path)
            self.content_stack.addWidget(launcher)

        splitter.addWidget(self.content_stack)

        # Set splitter sizes (sidebar: 250px, content: rest)
        splitter.setSizes([250, 1150])
        splitter.setCollapsible(0, False)

        main_layout.addWidget(splitter)

        # Apply theme
        self._apply_theme()

        # Select first component (motor control) by default
        if self.sidebar_buttons:
            self.sidebar_buttons[0].setChecked(True)
            self.content_stack.setCurrentIndex(0)

    def _create_sidebar(self):
        """Create the sidebar with navigation buttons"""
        sidebar_widget = QWidget()
        sidebar_widget.setMinimumWidth(250)
        sidebar_widget.setMaximumWidth(350)
        sidebar_widget.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #1a252f, stop:1 #2C3E50);
            border-right: 1px solid #34495E;
        """)

        layout = QVBoxLayout(sidebar_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header
        header = QLabel("ExoPulse GUI")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                stop:0 #34495E, stop:1 #2C3E50);
            color: #ECF0F1;
            padding: 20px;
            font-size: 18pt;
            font-weight: bold;
            border-bottom: 2px solid #3498DB;
        """)
        layout.addWidget(header)

        # Scroll area for buttons
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("""
            QScrollArea {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #1a252f, stop:1 #2C3E50);
                border: none;
            }
            QScrollBar:vertical {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3498DB, stop:1 #2980B9);
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #5DADE2, stop:1 #3498DB);
                border-radius: 6px;
            }
        """)

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 10, 0, 10)
        scroll_layout.setSpacing(2)

        # Section: Main Control
        section_label = QLabel("Main Control")
        section_label.setStyleSheet("""
            color: #5DADE2;
            font-weight: bold;
            font-size: 10pt;
            padding: 15px 20px 5px 20px;
        """)
        scroll_layout.addWidget(section_label)

        # Create buttons for all components
        for idx, comp in enumerate(self.components):
            comp_id, display_name, description, script_name, requires_can, is_wifi_tool = comp

            # Determine icon
            if idx == 0:
                icon = "●"  # Main control
            elif requires_can:
                icon = "▣"  # CAN monitoring
            elif is_wifi_tool:
                icon = "◈"  # WiFi tools
            else:
                icon = "◆"  # General tools

            # Add section headers
            if idx == 1:
                section_label = QLabel("CAN Monitoring Tools")
                section_label.setStyleSheet("""
                    color: #5DADE2;
                    font-weight: bold;
                    font-size: 10pt;
                    padding: 15px 20px 5px 20px;
                """)
                scroll_layout.addWidget(section_label)
            elif idx == 3:
                section_label = QLabel("WiFi Tools")
                section_label.setStyleSheet("""
                    color: #5DADE2;
                    font-weight: bold;
                    font-size: 10pt;
                    padding: 15px 20px 5px 20px;
                """)
                scroll_layout.addWidget(section_label)
            elif idx == 5:
                section_label = QLabel("General Tools")
                section_label.setStyleSheet("""
                    color: #5DADE2;
                    font-weight: bold;
                    font-size: 10pt;
                    padding: 15px 20px 5px 20px;
                """)
                scroll_layout.addWidget(section_label)

            # Create button
            btn = SidebarButton(display_name, icon)
            btn.clicked.connect(lambda checked, i=idx: self._on_sidebar_click(i))
            scroll_layout.addWidget(btn)
            self.sidebar_buttons.append(btn)

            # Store metadata
            btn.setProperty("requires_can", requires_can)
            btn.setProperty("is_wifi_tool", is_wifi_tool)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        # Footer
        footer = QLabel("v1.0.0")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("""
            color: #7F8C8D;
            padding: 10px;
            font-size: 9pt;
            background-color: rgba(0, 0, 0, 0.2);
        """)
        layout.addWidget(footer)

        return sidebar_widget

    def _connect_motor_control_signals(self):
        """Connect signals from motor control to update sidebar"""
        # Connect motor type change signal
        self.motor_control.radio_can.toggled.connect(self._on_motor_type_changed)
        self.motor_control.radio_wifi.toggled.connect(self._on_comm_mode_changed)

    def _on_motor_type_changed(self, checked):
        """Handle motor type change to enable/disable sidebar buttons"""
        is_can_mode = checked

        for btn in self.sidebar_buttons:
            requires_can = btn.property("requires_can")
            if requires_can:
                btn.setEnabled(is_can_mode)

        # Log the mode change
        mode_name = "CAN" if is_can_mode else "Lower Chip"
        print(f"Motor mode changed to: {mode_name}")
        print(f"CAN monitoring tools {'enabled' if is_can_mode else 'disabled'}")

    def _on_comm_mode_changed(self, checked):
        """Handle communication mode change"""
        is_wifi_mode = checked

        for btn in self.sidebar_buttons:
            is_wifi_tool = btn.property("is_wifi_tool")
            if is_wifi_tool:
                # WiFi tools more useful in WiFi mode, but still accessible
                pass  # Keep enabled for now

    def _on_sidebar_click(self, index):
        """Handle sidebar button click"""
        # Uncheck all buttons
        for btn in self.sidebar_buttons:
            btn.setChecked(False)

        # Check clicked button
        self.sidebar_buttons[index].setChecked(True)

        # Switch content
        self.content_stack.setCurrentIndex(index)

    def _apply_theme(self):
        """Apply application theme"""
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2C3E50, stop:1 #34495E);
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #34495E;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
                color: #ECF0F1;
                background-color: transparent;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #5DADE2;
            }
        """)


def main():
    """Main entry point"""
    app = QApplication(sys.argv)

    # Set application metadata
    app.setApplicationName("ExoPulse Unified GUI")
    app.setOrganizationName("ExoPulse")
    app.setApplicationVersion("1.0.0")

    # Create and show main window
    window = ExoPulseUnifiedGUI()
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
