#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ExoPulse Unified GUI Launcher

A comprehensive GUI application that integrates all ExoPulse UI components
with a smooth sidebar navigation interface.
"""

import sys
import subprocess
from pathlib import Path

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QStackedWidget, QScrollArea, QTextEdit,
    QSplitter, QGroupBox
)
from PySide6.QtCore import Qt, QSize, QProcess
from PySide6.QtGui import QFont, QColor, QPalette, QIcon


class ComponentLauncher(QWidget):
    """Widget for launching individual UI components"""

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
        desc.setStyleSheet("color: #888; margin-bottom: 20px;")
        layout.addWidget(desc)

        # Script path
        path_label = QLabel(f"Script: {self.script_path.name}")
        path_label.setStyleSheet("color: #666; font-family: monospace; font-size: 10pt;")
        layout.addWidget(path_label)

        layout.addSpacing(20)

        # Launch button
        self.launch_btn = QPushButton("🚀 Launch Component")
        self.launch_btn.setMinimumHeight(50)
        self.launch_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14pt;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.launch_btn.clicked.connect(self._launch)
        layout.addWidget(self.launch_btn)

        # Stop button
        self.stop_btn = QPushButton("⏹ Stop Component")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 12pt;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.stop_btn.clicked.connect(self._stop)
        layout.addWidget(self.stop_btn)

        layout.addSpacing(20)

        # Status label
        self.status_label = QLabel("Status: Not running")
        self.status_label.setStyleSheet("color: #666; font-weight: bold;")
        layout.addWidget(self.status_label)

        # Log output
        log_group = QGroupBox("Process Output")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
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
        self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
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
        self.log_text.append(f"<span style='color: #ff6b6b;'>{data.rstrip()}</span>")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _on_finished(self, exit_code, exit_status):
        """Handle process finished"""
        self.launch_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText(f"Status: Stopped (exit code: {exit_code})")
        self.status_label.setStyleSheet("color: #666; font-weight: bold;")
        self.log_text.append(f"\n[{self.component_name}] Process finished with exit code {exit_code}\n")


class SidebarButton(QPushButton):
    """Styled sidebar navigation button"""

    def __init__(self, text, icon_text=""):
        super().__init__(f"{icon_text}  {text}")
        self.setCheckable(True)
        self.setMinimumHeight(50)
        self.setStyleSheet("""
            QPushButton {
                text-align: left;
                padding: 10px 20px;
                border: none;
                background-color: transparent;
                color: #333;
                font-size: 11pt;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
            QPushButton:checked {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                border-left: 4px solid #1976D2;
            }
        """)


class ExoPulseUnifiedGUI(QMainWindow):
    """Main unified GUI application for ExoPulse"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ExoPulse Unified Control Panel")
        self.setGeometry(100, 100, 1200, 800)

        self.ui_dir = Path(__file__).parent / "UI_components"

        self._init_ui()

    def _init_ui(self):
        """Initialize the user interface"""

        # Initialize collections first
        self.sidebar_buttons = []
        self.content_stack = QStackedWidget()

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
        splitter.addWidget(self.content_stack)

        # Set splitter sizes (sidebar: 250px, content: rest)
        splitter.setSizes([250, 950])
        splitter.setCollapsible(0, False)

        main_layout.addWidget(splitter)

        # Apply dark theme
        self._apply_theme()

        # Select first component by default
        if self.sidebar_buttons:
            self.sidebar_buttons[0].setChecked(True)
            self.content_stack.setCurrentIndex(0)

    def _create_sidebar(self):
        """Create the sidebar with navigation buttons"""
        sidebar_widget = QWidget()
        sidebar_widget.setMinimumWidth(250)
        sidebar_widget.setMaximumWidth(350)
        sidebar_widget.setStyleSheet("background-color: #f5f5f5;")

        layout = QVBoxLayout(sidebar_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header
        header = QLabel("ExoPulse GUI")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("""
            background-color: #2196F3;
            color: white;
            padding: 20px;
            font-size: 18pt;
            font-weight: bold;
        """)
        layout.addWidget(header)

        # Scroll area for buttons
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background-color: #f5f5f5;")

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 10, 0, 10)
        scroll_layout.setSpacing(2)

        # Component definitions
        components = [
            # Low-level
            ("Low-Level Components", None, None, None, True),
            ("motor_control", "Motor Control", "Basic motor control interface with WiFi/UART support", "motor_control.py", False),
            ("serial_reader", "Serial Reader", "Simple serial port data reader utility", "serial_reader.py", False),

            # High-level
            ("High-Level Components", None, None, None, True),
            ("motor_monitor", "Motor Monitor", "Advanced monitoring with configurable real-time plots", "motor_monitor.py", False),
            ("dual_motor_plotter", "Dual Motor Plotter", "Real-time dual motor data visualization", "dual_motor_plotter.py", False),
            ("wifi_monitor", "WiFi Monitor", "WiFi-based remote motor monitoring", "wifi_monitor.py", False),
            ("wifi_dual_motor_plotter", "WiFi Dual Motor Plot", "WiFi dual motor data plotter", "wifi_dual_motor_plotter.py", False),
            ("can_plotter", "CAN Plotter", "CAN bus data visualization tool", "can_plotter.py", False),
            ("emg_plotter", "EMG Plotter", "EMG signal acquisition and visualization", "emg_plotter.py", False),
        ]

        for comp in components:
            if comp[4]:  # Is section header
                section_label = QLabel(comp[0])
                section_label.setStyleSheet("""
                    color: #2196F3;
                    font-weight: bold;
                    font-size: 10pt;
                    padding: 15px 20px 5px 20px;
                    background-color: transparent;
                """)
                scroll_layout.addWidget(section_label)
            else:
                comp_id, display_name, description, script_name = comp[:4]
                script_path = self.ui_dir / script_name

                # Create button
                # Determine icon based on component index
                if len(self.sidebar_buttons) < 2:
                    icon = "⚙️"  # Low-level components
                else:
                    icon = "📊"  # High-level components
                btn = SidebarButton(display_name, icon)
                btn.clicked.connect(lambda checked, idx=len(self.sidebar_buttons): self._on_sidebar_click(idx))
                scroll_layout.addWidget(btn)
                self.sidebar_buttons.append(btn)

                # Create launcher widget
                launcher = ComponentLauncher(comp_id, display_name, description, script_path)
                self.content_stack.addWidget(launcher)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        # Footer
        footer = QLabel("v1.0.0")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("""
            background-color: #e0e0e0;
            color: #666;
            padding: 10px;
            font-size: 9pt;
        """)
        layout.addWidget(footer)

        return sidebar_widget

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
                background-color: white;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #ccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
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
