import json
import os
import time
import logging
import threading
from pathlib import Path
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QPushButton, QLineEdit, QMessageBox,
                             QComboBox, QDoubleSpinBox, QDialog, QSpinBox, QFileDialog, QSlider, QScrollArea, QTabWidget, QCheckBox,
                             QFormLayout, QDialogButtonBox)
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtWidgets import QSplitter, QApplication, QAbstractSpinBox
import numpy as np
import csv

from core.path_planner import PathPlanner
from core.sequence_planner import SequenceError, SequencePlanner
from gui.sequence_builder_widget import SequenceBuilderWidget
from core.path_executor import PathExecutor
from gui.stick_viewer import StickViewer
from gui.analytics_panel import AnalyticsPanel, AlgorithmTuningPanel, LatencyDiagnosticsPanel
from core.kinematics import ReBelKinematics, JOINT_LIMITS_DEG
from core.digital_twin_analytics import (
    AdaptiveBaselineRLS, CUSUMDetector, PayloadEstimator, TelemetryLogger, TelemetryRecord,
)
from core.identification_logger import (
    IdentificationLogger,
    find_existing_identification_run,
)
from core.latency_tracking import ExecutionLatencyTracker
from core.latency_experiment_logger import LatencyExperimentLogger
from hardware.robot_manager import RebelManager
from settings import get_settings, save_settings, FFRLSSettings, CUSUMSettings, PayloadSettings

CONFIG_FILE = "rebel_config.json"

class HomingConfigDialog(QDialog):
    """Popup Dialog to configure and save Homing Parameters."""
    def __init__(self, current_config, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configure Safe Home")
        self.setModal(True)
        self.current_config = current_config
        self.spinboxes = []
        
        layout = QVBoxLayout(self)
        
        # Joint Angles
        joint_group = QGroupBox("Home Position (Joint Angles in Degrees)")
        j_layout = QGridLayout()
        for i in range(6):
            j_layout.addWidget(QLabel(f"Joint {i+1}:"), i, 0)
            sb = QDoubleSpinBox()
            sb.setRange(-360.0, 360.0)
            sb.setValue(self.current_config["home_joints"][i])
            j_layout.addWidget(sb, i, 1)
            self.spinboxes.append(sb)
        joint_group.setLayout(j_layout)
        layout.addWidget(joint_group)
        
        # Speed
        speed_group = QGroupBox("Homing Speed (%)")
        s_layout = QHBoxLayout()
        self.speed_sb = QDoubleSpinBox()
        self.speed_sb.setRange(1.0, 100.0)
        self.speed_sb.setValue(self.current_config["home_speed"])
        s_layout.addWidget(self.speed_sb)
        speed_group.setLayout(s_layout)
        layout.addWidget(speed_group)
        
        # Save Button
        btn_save = QPushButton("Save to Config")
        btn_save.clicked.connect(self.accept)
        layout.addWidget(btn_save)

    def get_new_config(self):
        joints = [sb.value() for sb in self.spinboxes]
        return {
            "home_joints": joints,
            "home_speed": self.speed_sb.value()
        }


class IdentificationSettingsDialog(QDialog):
    """Edit metadata that will be attached to the next experiment run."""

    def __init__(self, values, trajectory_id, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Identification Settings")
        self.setModal(True)
        layout = QFormLayout(self)

        self.experiment_id = QLineEdit(values.get("experiment_id", "ID_NORMAL_001"))
        self.trajectory_id = QLineEdit(values.get("trajectory_id", trajectory_id))
        self.trajectory_id.setPlaceholderText(trajectory_id)
        self.run_id = QLineEdit(values.get("run_id", "RUN_01"))
        self.commanded_speed = QDoubleSpinBox()
        self.commanded_speed.setRange(1.0, 100.0)
        self.commanded_speed.setDecimals(0)
        self.commanded_speed.setSuffix(" %")
        self.commanded_speed.setValue(float(values.get("commanded_speed", 50.0)))
        self.payload_g = QDoubleSpinBox()
        self.payload_g.setRange(0.0, 10000.0)
        self.payload_g.setSuffix(" g")
        self.payload_g.setValue(float(values.get("payload_g", 0.0)))
        self.contact_label = QComboBox()
        self.contact_label.addItems(["NORMAL", "CONTACT", "COLLISION", "UNKNOWN"])
        self.contact_label.setCurrentText(values.get("contact_label", "NORMAL"))
        self.grasp_context = QComboBox()
        self.grasp_context.addItems(["NONE", "FALSE", "TRUE"])
        self.grasp_context.setCurrentText(values.get("grasp_context", "NONE"))
        self.thermal_condition = QComboBox()
        self.thermal_condition.addItems(["COLD", "WARM"])
        self.thermal_condition.setCurrentText(values.get("thermal_condition", "COLD"))

        layout.addRow("Experiment ID", self.experiment_id)
        layout.addRow("Trajectory ID (blank = filename)", self.trajectory_id)
        layout.addRow("Run ID", self.run_id)
        layout.addRow("Commanded speed", self.commanded_speed)
        layout.addRow("Payload", self.payload_g)
        layout.addRow("Contact label", self.contact_label)
        layout.addRow("Grasp context", self.grasp_context)
        layout.addRow("Thermal condition", self.thermal_condition)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_values(self):
        return {
            "experiment_id": self.experiment_id.text().strip() or "ID_NORMAL_001",
            "trajectory_id": self.trajectory_id.text().strip(),
            "run_id": self.run_id.text().strip() or "RUN_01",
            "commanded_speed": float(self.commanded_speed.value()),
            "payload_g": float(self.payload_g.value()),
            "contact_label": self.contact_label.currentText(),
            "grasp_context": self.grasp_context.currentText(),
            "thermal_condition": self.thermal_condition.currentText(),
        }


class TitleCornerGroupBox(QGroupBox):
    """Group box that keeps a widget on its title line, right-aligned."""

    def __init__(self, title: str, parent=None) -> None:
        super().__init__(title, parent)
        self._title_widget = None

    def set_title_widget(self, widget: QWidget) -> None:
        self._title_widget = widget
        widget.setParent(self)
        widget.show()
        self._place_title_widget()

    def _place_title_widget(self) -> None:
        widget = self._title_widget
        if widget is None:
            return
        widget.adjustSize()
        x = self.width() - widget.width() - 10
        y = max(0, (self.fontMetrics().height() - widget.height()) // 2)
        widget.move(max(0, x), y)
        widget.raise_()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._place_title_widget()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.path_planner = PathPlanner()
        self.path_executor = PathExecutor()
        self.loaded_trajectory_path = None
        self.loaded_trajectory_plan = None
        self.sim_path_full = []
        self.sim_tip_trail = []
        self.real_tip_trail = []
        self.realtime_trail_active = False
        self.current_frame = 0
        self.total_frames = 0
        self.is_playing = False
        self.current_sim_angles = [0.0] * 6 
        self.pre_sim_angles = [0.0] * 6

        self.setWindowTitle("Igus ReBeL iRC Clone & Digital Twin")
        self.resize(1300, 850)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.pressed_keys = set()
        QApplication.instance().installEventFilter(self)
        
        self.manager = RebelManager()
        self.kinematics = ReBelKinematics()
        self.execution_latency = ExecutionLatencyTracker()
        self.latency_experiment_logger = LatencyExperimentLogger("latency_experiments.csv")
        self._motion_request_lock = threading.RLock()
        self._pending_motion_request = None
        self._init_digital_twin_analytics()
        self.realtime_data_log = []
        self.log_start_time = time.perf_counter()
        self.is_recording_diagnostics = False
        self.execution_complete = False
        self.execution_saved = False
        self.execution_error = None
        self.execution_state = "IDLE"
        self.execution_elapsed = 0.0
        self.execution_session_active = False
        self.execution_session_start = None
        self.execution_energy_j = 0.0
        self.execution_last_sample_time = None
        self.execution_last_power_w = None
        self.execution_stop_event = threading.Event()
        self.identification_logger = None
        self.identification_settings = {
            "experiment_id": "ID_NORMAL_001",
            "trajectory_id": "",
            "run_id": "RUN_01",
            "commanded_speed": 50.0,
            "payload_g": 0.0,
            "contact_label": "NORMAL",
            "grasp_context": "NONE",
            "thermal_condition": "COLD",
        }
        self._identification_last_timestamp = None
        self._identification_last_q = None
        self._identification_last_qdot = None
        self._identification_lock = threading.RLock()
        self.identification_output_path = None
        self.identification_last_summary = None
        self.move_to_start_thread = None
        self.move_to_start_complete = False
        self.move_to_start_error = None
        self.move_to_start_target = None
        
        # Load Configuration File
        self.config = self.load_config()
        
        self.init_ui()
        self.update_visual_points()

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.system_loop)
        self.ui_timer.start(33) 

    def load_config(self):
        default_config = {
            "home_joints": [0.0, -10.0, 135.0, 0.0, 20.0, 0.0],
            "home_speed": 60.0
        }
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e:
                print(f"Error loading config: {e}. Using defaults.")
        return default_config

    def save_config(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            print("Configuration saved successfully.")
        except Exception as e:
            QMessageBox.warning(self, "Save Error", f"Could not save config file:\n{e}")

    def open_config_dialog(self):
        dialog = HomingConfigDialog(self.config, self)
        if dialog.exec():
            self.config = dialog.get_new_config()
            self.save_config()

    def execute_homing(self):
        self._show_realtime_view()
        joints = self.config["home_joints"]
        speed = self.config["home_speed"]
        self.manager.go_home(joints, speed)

    def _build_stylesheet(self):
        """Dark, consistent theme so the tabbed control panel reads as one unit."""
        return """
            QWidget { background-color: #20242b; color: #e6e6e6; font-size: 11px; }
            QMainWindow { background-color: #20242b; }
            QGroupBox {
                border: 1px solid #3a3f47;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: 600;
                color: #9fd3ff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QTabWidget::pane { border: 1px solid #3a3f47; border-radius: 6px; top: -1px; }
            QTabBar::tab {
                background: #2b3038;
                color: #cfd6dd;
                padding: 6px 14px;
                border: 1px solid #3a3f47;
                border-bottom: none;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
            }
            QTabBar::tab:selected { background: #37414d; color: #ffffff; font-weight: 600; }
            QTabBar::tab:hover { background: #333a43; }
            QPushButton {
                background-color: #37414d;
                border: 1px solid #4a5561;
                border-radius: 4px;
                padding: 5px 8px;
            }
            QPushButton:hover { background-color: #445162; }
            QPushButton:pressed { background-color: #2b323a; }
            QPushButton:disabled { background-color: #2b2f35; color: #6b7178; border-color: #3a3f47; }
            QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
                background-color: #14171b;
                border: 1px solid #3a3f47;
                border-radius: 3px;
                padding: 3px;
                color: #e6e6e6;
            }
            QCheckBox { spacing: 6px; }
            QScrollArea { border: none; }
        """

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)
        self.setCentralWidget(main_widget)
        self.setStyleSheet(self._build_stylesheet())

        # LEFT PANEL: tabbed controls above a persistent live status footer.
        left_sidebar = QWidget()
        left_sidebar_layout = QVBoxLayout(left_sidebar)
        left_sidebar_layout.setContentsMargins(0, 0, 0, 0)
        left_sidebar_layout.setSpacing(6)
        left_sidebar.setMaximumWidth(360)

        left_tabs = QTabWidget()
        left_tabs.setMaximumWidth(360)

        # Connection Group (lives in the ROBOT STATUS panel)
        flow_group = QGroupBox()
        flow_layout = QVBoxLayout()
        flow_layout.setContentsMargins(6, 6, 6, 6)
        flow_layout.setSpacing(4)
        
        self.status_label = QLabel("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red; font-size: 12px;")
        self.lbl_execution = QLabel("Execution: Idle")
        self.lbl_execution.setStyleSheet("font-weight: bold; color: #cfd6dd;")
        self.ip_input = QLineEdit("192.168.3.11")
        
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self.connect_robot)
        self.btn_disconnect = QPushButton("Disconnect"); self.btn_disconnect.clicked.connect(self.disconnect_robot)
        self.btn_home = QPushButton("🏠")
        self.btn_home.setToolTip("Home")
        self.btn_home.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        self.btn_home.clicked.connect(self.execute_homing)
        self.btn_enable = QPushButton("Enable"); self.btn_enable.clicked.connect(self.manager.enable_motors)
        self.btn_reset = QPushButton("Reset"); self.btn_reset.clicked.connect(self.manager.reset_errors)
        self.btn_config_home = QPushButton("⚙️")
        self.btn_config_home.setToolTip("Homing configuration")
        self.btn_config_home.clicked.connect(self.open_config_dialog)

        button_grid = QGridLayout()
        button_grid.setHorizontalSpacing(4)
        button_grid.setVerticalSpacing(4)
        for index, btn in enumerate([self.btn_connect, self.btn_disconnect, self.btn_home,
                                     self.btn_enable, self.btn_reset, self.btn_config_home]):
            button_grid.addWidget(btn, index // 3, index % 3)
        button_grid.setColumnStretch(0, 3)
        button_grid.setColumnStretch(1, 3)
        button_grid.setColumnStretch(2, 1)
        self.btn_home.setMinimumWidth(40)
        self.btn_config_home.setMinimumWidth(40)

        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home]:
            btn.setEnabled(False)

        status_row = QHBoxLayout()
        status_row.addWidget(self.status_label)
        status_row.addWidget(self.lbl_execution)
        flow_layout.addLayout(status_row)
        flow_layout.addWidget(self.ip_input)
        flow_layout.addLayout(button_grid)
        flow_group.setLayout(flow_layout)

        # 2. Jogging Group 
        jog_group = TitleCornerGroupBox("Jog Control (Hold I)")
        jog_layout = QGridLayout()
        jog_layout.setHorizontalSpacing(10) # Minimal space between the 2 columns
        jog_layout.setVerticalSpacing(4)

        # Jog speed in percent of maximum axis speed, used by buttons and keyboard.
        self.btn_jog_toggle = QPushButton("ON")
        self.btn_jog_toggle.setCheckable(True)
        self.btn_jog_toggle.setChecked(True)
        self.btn_jog_toggle.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.btn_jog_toggle.setFixedSize(48, 20)
        self.btn_jog_toggle.setStyleSheet(
            "QPushButton { padding: 0px; font-weight: bold; background-color: #7f2a2a; color: white; }"
            "QPushButton:checked { background-color: #2e7d32; }"
        )
        self.btn_jog_toggle.setToolTip("Enable or disable jogging (buttons and keyboard)")
        self.btn_jog_toggle.toggled.connect(self._on_jog_toggled)
        jog_group.set_title_widget(self.btn_jog_toggle)

        self.spin_jog_speed = QDoubleSpinBox()
        self.spin_jog_speed.setRange(1.0, 100.0)
        self.spin_jog_speed.setValue(15.0)
        self.spin_jog_speed.setDecimals(0)
        self.spin_jog_speed.setSuffix(" %")
        self.spin_jog_speed.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.spin_jog_speed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_jog_speed.setFixedWidth(60)
        self.spin_jog_speed.setToolTip("Jog speed (percent of maximum axis speed)")
        self.btn_jog_speed_down = QPushButton("−")
        self.btn_jog_speed_up = QPushButton("+")
        for step_btn, delta in ((self.btn_jog_speed_down, -5.0), (self.btn_jog_speed_up, 5.0)):
            step_btn.setFixedWidth(28)
            step_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            step_btn.setAutoRepeat(True)
            step_btn.clicked.connect(lambda _=False, d=delta: self.spin_jog_speed.setValue(self.spin_jog_speed.value() + d))
        self.slider_jog_speed = QSlider(Qt.Orientation.Horizontal)
        self.slider_jog_speed.setRange(1, 100)
        self.slider_jog_speed.setValue(15)
        self.slider_jog_speed.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.slider_jog_speed.valueChanged.connect(lambda v: self.spin_jog_speed.setValue(float(v)))
        self.spin_jog_speed.valueChanged.connect(self._on_jog_speed_changed)
        jog_speed_row = QHBoxLayout()
        jog_speed_row.addWidget(QLabel("Speed:"))
        jog_speed_row.addWidget(self.slider_jog_speed, 1)
        jog_speed_row.addWidget(self.btn_jog_speed_down)
        jog_speed_row.addWidget(self.spin_jog_speed)
        jog_speed_row.addWidget(self.btn_jog_speed_up)
        jog_layout.addLayout(jog_speed_row, 3, 0, 1, 2)
        
        joint_configs = [
            ("J1", "A", "D", "A1"), ("J2", "W", "S", "A2"),
            ("J3", "Q", "E", "A3"), ("J4", "R", "F", "A4"),
            ("J5", "T", "G", "A5"), ("J6", "Y", "H", "A6")
        ]
        
        self.jog_buttons = []
        for i, (name, key_neg, key_pos, axis) in enumerate(joint_configs):
            lbl = QLabel(name)
            # Remove extra space in button text to fit tighter
            btn_neg = QPushButton(f"-({key_neg})")
            btn_pos = QPushButton(f"+({key_pos})")
            
            btn_neg.setEnabled(False)
            btn_pos.setEnabled(False)
            btn_neg.pressed.connect(lambda a=axis: self.start_jog_with_realtime(a, -self.spin_jog_speed.value()))
            btn_neg.released.connect(self.stop_jog_with_realtime)
            btn_pos.pressed.connect(lambda a=axis: self.start_jog_with_realtime(a, self.spin_jog_speed.value()))
            btn_pos.released.connect(self.stop_jog_with_realtime)
            
            # Pack Label and Buttons tightly together into an inner horizontal layout
            cell_widget = QWidget()
            cell_layout = QHBoxLayout(cell_widget)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(2) # Super tight spacing internally
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(btn_neg)
            cell_layout.addWidget(btn_pos)
            
            # Place the packed widget into the main 2-column grid
            row = i // 2
            col = i % 2
            jog_layout.addWidget(cell_widget, row, col)
            
            self.jog_buttons.extend([btn_neg, btn_pos])
            
        jog_group.setLayout(jog_layout)

        jog_tab = QWidget()
        jog_layout_container = QVBoxLayout(jog_tab)
        jog_layout_container.setAlignment(Qt.AlignmentFlag.AlignTop)
        jog_layout_container.setContentsMargins(6, 6, 6, 6)
        jog_layout_container.addWidget(jog_group)
        jog_layout_container.addStretch()

        # 3. Joint Angles Group
        sensor_group = QGroupBox()
        sensor_layout = QGridLayout()
        sensor_layout.setHorizontalSpacing(4) # Tighten column spacing
        sensor_layout.setVerticalSpacing(4)   # Tighten row spacing
        
        self.lbl_joints = [QLabel(f"J{i+1}: -- °") for i in range(6)]
        for i, lbl in enumerate(self.lbl_joints):
            lbl.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 2px;")
            row = i // 3
            col = i % 3
            sensor_layout.addWidget(lbl, row, col)
            
        sensor_group.setLayout(sensor_layout)

        position_group = QGroupBox()
        position_layout = QHBoxLayout()
        self.lbl_position = [QLabel(f"{axis}: -- mm") for axis in ("X", "Y", "Z")]
        for label in self.lbl_position:
            label.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
            position_layout.addWidget(label)
        position_group.setLayout(position_layout)

        # 4. Real-time Status & Diagnostics
        diag_group = QGroupBox()
        diag_layout = QGridLayout()
        diag_layout.setHorizontalSpacing(6)
        diag_layout.setVerticalSpacing(6)

        # Supply voltage is no longer displayed; the label is kept for the update code.
        self.lbl_supply_voltage = QLabel("Supply Voltage: 24 V")

        self.lbl_total_current = QLabel("Total: -- mA")
        self.lbl_total_current.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
        diag_layout.addWidget(self.lbl_total_current, 0, 0, 1, 2)

        self.lbl_power = QLabel("Power: -- W")
        self.lbl_power.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
        diag_layout.addWidget(self.lbl_power, 0, 2)
        self.lbl_joint_currents = [QLabel(f"J{i+1}: -- mA") for i in range(6)]
        for i, lbl in enumerate(self.lbl_joint_currents):
            lbl.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
            row = 1 + (i // 3)
            col = i % 3
            diag_layout.addWidget(lbl, row, col)

        diag_group.setLayout(diag_layout)

        status_panel = QGroupBox("ROBOT STATUS")
        status_layout = QVBoxLayout(status_panel)
        status_layout.setContentsMargins(6, 10, 6, 6)
        status_layout.setSpacing(4)
        # Untitled boxes: no frame or heading space, so the blocks sit tightly together.
        for untitled_box in (flow_group, position_group, sensor_group, diag_group):
            untitled_box.setStyleSheet("QGroupBox { border: none; margin-top: 0px; padding-top: 0px; }")
            untitled_box.layout().setContentsMargins(0, 0, 0, 0)
        status_layout.addWidget(flow_group)
        status_layout.addSpacing(6)
        status_layout.addWidget(position_group)
        status_layout.addWidget(sensor_group)
        status_layout.addSpacing(6)
        status_layout.addWidget(diag_group)

        jog_scroll = QScrollArea()
        jog_scroll.setWidgetResizable(True)
        jog_scroll.setWidget(jog_tab)
        jog_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(jog_scroll, "JOG")

        # ---- Pathways tab: geometry, constraints, points, playback ----
        # ---- Trajectory Studio: sequence builder (left) ----
        self.sequence_builder = SequenceBuilderWidget()
        self.sequence_builder.generate_requested.connect(self.generate_sequence)
        self.sequence_builder.export_requested.connect(self.export_cri_path)
        self.sequence_builder.points_changed.connect(self.update_visual_points)
        self.btn_export_path = self.sequence_builder.btn_export
        self.sequence_planner = SequencePlanner(self.kinematics, self.path_planner)
        self.sequence_builder.validator = lambda step: self.sequence_planner.validate_step(
            step, self.current_sim_angles
        )
        self.sequence_plan = None

        # ---- Execution tab: trajectory selection, run settings, and live metrics ----
        execution_tab = QWidget()
        execution_layout = QVBoxLayout(execution_tab)
        execution_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        execution_layout.setContentsMargins(6, 4, 6, 4)
        execution_layout.setSpacing(4)

        trajectory_group = QGroupBox("Trajectory to Execute")
        trajectory_layout = QGridLayout(trajectory_group)
        self.cmb_trajectory_files = QComboBox()
        self.cmb_trajectory_files.setEditable(False)
        self.btn_load_trajectory = QPushButton("Load CSV")
        self.btn_load_trajectory.clicked.connect(self.load_selected_trajectory)
        self.btn_browse_trajectory = QPushButton("Browse...")
        self.btn_browse_trajectory.clicked.connect(self.browse_trajectory_file)
        self.lbl_loaded_trajectory = QLabel("No trajectory loaded")
        self.lbl_loaded_waypoints = QLabel("Waypoints: 0")
        self.lbl_loaded_start = QLabel("Start: --")
        self.lbl_loaded_target = QLabel("Target: --")
        self.lbl_trajectory_status = QLabel("Validation: pending")

        trajectory_layout.addWidget(QLabel("Saved pathway:"), 0, 0)
        trajectory_layout.addWidget(self.cmb_trajectory_files, 0, 1, 1, 2)
        trajectory_layout.addWidget(self.btn_load_trajectory, 1, 1)
        trajectory_layout.addWidget(self.btn_browse_trajectory, 1, 2)
        trajectory_layout.addWidget(QLabel("Loaded:"), 2, 0)
        trajectory_layout.addWidget(self.lbl_loaded_trajectory, 2, 1, 1, 2)
        trajectory_layout.addWidget(self.lbl_loaded_waypoints, 3, 0, 1, 2)
        trajectory_layout.addWidget(self.lbl_trajectory_status, 3, 2)
        trajectory_layout.addWidget(self.lbl_loaded_start, 4, 0, 1, 3)
        trajectory_layout.addWidget(self.lbl_loaded_target, 5, 0, 1, 3)
        trajectory_layout.setVerticalSpacing(4)
        execution_layout.addWidget(trajectory_group)

        run_settings_layout = QHBoxLayout()
        self.spin_robot_velocity = QDoubleSpinBox()
        self.spin_robot_velocity.setRange(1.0, 100.0)
        self.spin_robot_velocity.setValue(50.0)
        self.spin_robot_velocity.setDecimals(0)
        self.spin_robot_velocity.setSuffix(" %")
        self.spin_robot_velocity.setToolTip("CRI velocity used for preparation and task motion")
        run_settings_layout.addWidget(QLabel("Motion speed:"))
        run_settings_layout.addWidget(self.spin_robot_velocity)
        self.btn_execute_path = QPushButton("Run")
        self.btn_execute_path.setStyleSheet("background-color: #c0392b; color: white; padding: 6px; font-weight: bold;")
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self.execute_path)
        run_settings_layout.addWidget(self.btn_execute_path, 1)
        execution_layout.addLayout(run_settings_layout)

        identification_tab = QWidget()
        identification_layout = QVBoxLayout(identification_tab)
        identification_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        identification_layout.setContentsMargins(6, 6, 6, 6)
        identification_layout.setSpacing(8)

        identification_group = QGroupBox("Identification Run")
        identification_group_layout = QVBoxLayout(identification_group)
        identification_info = QLabel(
            "Record robot telemetry for an identification experiment. "
            "Connect to the robot before starting a run."
        )
        identification_info.setWordWrap(True)
        identification_group_layout.addWidget(identification_info)
        self.btn_identification_settings = QPushButton("Identification Settings...")
        self.btn_identification_settings.clicked.connect(self.open_identification_settings)
        identification_group_layout.addWidget(self.btn_identification_settings)

        identification_controls = QHBoxLayout()
        self.btn_start_identification = QPushButton("Start Identification Run")
        self.btn_start_identification.setEnabled(False)
        self.btn_start_identification.clicked.connect(self.start_identification_recording)
        self.btn_stop_identification = QPushButton("Stop & Save Run")
        self.btn_stop_identification.setEnabled(False)
        self.btn_stop_identification.clicked.connect(self.stop_identification_recording)
        identification_controls.addWidget(self.btn_start_identification)
        identification_controls.addWidget(self.btn_stop_identification)
        identification_group_layout.addLayout(identification_controls)
        identification_layout.addWidget(identification_group)

        # Playback controls
        playback_layout = QVBoxLayout()
        slider_layout = QHBoxLayout()
        self.playback_slider = QSlider(Qt.Orientation.Horizontal)
        self.playback_slider.setEnabled(False)
        self.playback_slider.valueChanged.connect(self.on_playback_slider_changed)
        slider_layout.addWidget(self.playback_slider)
        self.lbl_playback_frame = QLabel("Frame: 0 / 0")
        self.lbl_playback_frame.setFixedWidth(100)
        slider_layout.addWidget(self.lbl_playback_frame)
        playback_layout.addLayout(slider_layout)

        controls_layout = QHBoxLayout()
        self.btn_play = QPushButton("▶ Play")
        self.btn_play.setEnabled(False)
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self.pause_playback)
        self.btn_stop = QPushButton("⏹ Reset")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.reset_simulation)
        controls_layout.addWidget(self.btn_play)
        controls_layout.addWidget(self.btn_pause)
        controls_layout.addWidget(self.btn_stop)
        playback_layout.addLayout(controls_layout)

        self.lbl_sim_segment = QLabel("Segment: --")
        self.lbl_sim_segment.setStyleSheet("color: #f1c40f; font-size: 11px;")
        playback_layout.insertWidget(0, self.lbl_sim_segment)

        # Simulation / Real-time Options Row
        opts_layout = QHBoxLayout()
        self.chk_auto_pause_on_connect = QCheckBox("Auto-pause Simulation on Connect")
        self.chk_auto_pause_on_connect.setChecked(True)
        opts_layout.addWidget(self.chk_auto_pause_on_connect)

        self.chk_real_time_trail = QCheckBox("Show Real-time Tip Trail")
        self.chk_real_time_trail.setChecked(True)
        self.chk_real_time_trail.toggled.connect(self.real_view_trail_visibility_changed)
        opts_layout.addWidget(self.chk_real_time_trail)

        # Playback sits under the simulation view on the right of the Trajectory Studio.
        self.studio_playback_widget = QWidget()
        studio_playback_layout = QVBoxLayout(self.studio_playback_widget)
        studio_playback_layout.setContentsMargins(0, 0, 0, 0)
        studio_playback_layout.addLayout(playback_layout)
        studio_playback_layout.addLayout(opts_layout)

        self.lbl_execution_timer = QLabel("00:00.0")
        self.lbl_execution_energy = QLabel("0.000 J (0.000 Wh)")
        for label in (self.lbl_execution_timer, self.lbl_execution_energy):
            label.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 2px 6px;")
        
        # Export diagnostics and Move to Start share one row
        export_diag_layout = QHBoxLayout()
        export_diag_layout.setSpacing(4)
        self.btn_export_voltage_csv = QPushButton("📥 Export CSV")
        self.btn_export_voltage_csv.setStyleSheet("background-color: #8e44ad; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_export_voltage_csv.setToolTip("Export Diagnostics CSV")
        self.btn_export_voltage_csv.clicked.connect(self.export_diagnostics_csv)
        export_diag_layout.addWidget(self.btn_export_voltage_csv)

        self.btn_move_to_start = QPushButton("🎯 Move to Start")
        self.btn_move_to_start.setStyleSheet("background-color: #16a085; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_move_to_start.setEnabled(False)
        self.btn_move_to_start.clicked.connect(self.move_to_start)
        export_diag_layout.addWidget(self.btn_move_to_start)
        execution_layout.addLayout(export_diag_layout)
        execution_layout.addStretch()

        self._populate_trajectory_files()

        execution_scroll = QScrollArea()
        execution_scroll.setWidgetResizable(True)
        execution_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        execution_scroll.setWidget(execution_tab)
        execution_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(execution_scroll, "Execution")

        identification_scroll = QScrollArea()
        identification_scroll.setWidgetResizable(True)
        identification_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        identification_scroll.setWidget(identification_tab)
        identification_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(identification_scroll, "Identification")

        # Purple tab strip and orange content area remain above the green footer.
        left_sidebar_layout.addWidget(status_panel)
        left_sidebar_layout.addWidget(left_tabs, 1)
        self.btn_trajectory_studio = QPushButton("Trajectory Studio...")
        self.btn_trajectory_studio.setToolTip("Plan pathways and preview them in the simulation view")
        self.btn_trajectory_studio.clicked.connect(self.open_trajectory_studio)
        left_sidebar_layout.addWidget(self.btn_trajectory_studio)

        # ==========================================
        # RIGHT PANEL: Real-time view (the simulation view lives in the Trajectory Studio)
        # ==========================================
        self.view_tabs = QTabWidget()
        self.view_tabs.setMinimumWidth(620)

        # Simulation viewer: hosted by the Trajectory Studio window.
        self.sim_view = StickViewer()

        # Real-time View Tab
        real_tab = QWidget()
        real_tab_layout = QVBoxLayout(real_tab)
        real_tab_layout.setContentsMargins(6, 6, 6, 6)
        real_tab_layout.setSpacing(6)
        self.real_view = StickViewer()

        # Keep the live robot view and its analytics visible together, as one
        # vertical workspace rather than forcing the operator to change tabs.
        self.analytics_panel = AnalyticsPanel()
        self.tuning_panel = AlgorithmTuningPanel()
        self.latency_panel = LatencyDiagnosticsPanel()
        self.analytics_panel.btn_reset.clicked.connect(self._reset_digital_twin_analytics)
        self.tuning_panel.load_tuning_values(get_settings())
        self.tuning_panel.btn_apply_tuning.clicked.connect(self._apply_tuning_live)
        self.tuning_panel.btn_save_tuning.clicked.connect(self._save_tuning_to_config)

        analytics_scroll = QScrollArea()
        analytics_scroll.setWidgetResizable(True)
        analytics_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        analytics_scroll.setWidget(self.analytics_panel)

        diagnostics_tabs = QTabWidget()
        diagnostics_tabs.addTab(analytics_scroll, "Analytics")
        diagnostics_tabs.addTab(self.latency_panel, "Latency")
        diagnostics_tabs.addTab(self.tuning_panel, "Algorithm Tuning")
        diagnostics_tabs.setMinimumHeight(250)

        # Execution session readout sits on the tab-bar line, right-aligned.
        session_corner = QWidget()
        session_corner_layout = QHBoxLayout(session_corner)
        session_corner_layout.setContentsMargins(6, 0, 6, 0)
        session_corner_layout.setSpacing(6)
        session_corner_layout.addWidget(QLabel("Elapsed time:"))
        session_corner_layout.addWidget(self.lbl_execution_timer)
        session_corner_layout.addWidget(QLabel("Total energy:"))
        session_corner_layout.addWidget(self.lbl_execution_energy)
        diagnostics_tabs.setCornerWidget(session_corner, Qt.Corner.TopRightCorner)

        realtime_splitter = QSplitter(Qt.Orientation.Vertical)
        self.real_view.setMinimumHeight(300)
        realtime_splitter.addWidget(self.real_view)
        realtime_splitter.addWidget(diagnostics_tabs)
        realtime_splitter.setCollapsible(0, False)
        realtime_splitter.setCollapsible(1, False)
        realtime_splitter.setStretchFactor(0, 3)
        realtime_splitter.setStretchFactor(1, 2)
        realtime_splitter.setSizes([560, 300])
        real_tab_layout.addWidget(realtime_splitter, 1)
        self.view_tabs.addTab(real_tab, "Real-time View")
        self.real_tab = real_tab

        main_layout.addWidget(left_sidebar)
        main_layout.addWidget(self.view_tabs, 1)

        from gui.trajectory_studio_window import TrajectoryStudioWindow
        self.trajectory_studio = TrajectoryStudioWindow(
            self.sequence_builder, self.sim_view, self.studio_playback_widget, self
        )
        self.trajectory_studio.setWindowFlag(Qt.WindowType.Window, True)



    # ==========================================
    # LOGIC FUNCTIONS
    # ==========================================

    def update_visual_points(self):
        start, end = self.sequence_builder.endpoints()
        self.sim_view.set_visual_points(list(start), list(end), self.sequence_builder.chain())

    def _show_realtime_view(self):
        """Focus the live view before any command that moves the robot."""
        self.view_tabs.setCurrentWidget(self.real_tab)

    def _execution_motion_speed(self):
        return float(self.spin_robot_velocity.value())

    def _trajectory_directory(self):
        return self.path_executor.ensure_trajectory_directory("Pathways")

    def _populate_trajectory_files(self):
        directory = self._trajectory_directory()
        files = sorted(directory.glob("*.csv"))
        self.cmb_trajectory_files.clear()
        if not files:
            self.cmb_trajectory_files.addItem("No CSV files found")
            self.cmb_trajectory_files.setEnabled(False)
            self.lbl_loaded_trajectory.setText("No trajectory loaded")
            return

        self.cmb_trajectory_files.setEnabled(True)
        for file_path in files:
            self.cmb_trajectory_files.addItem(file_path.name)

    def _update_loaded_trajectory_summary(self, plan=None):
        if plan is None:
            self.lbl_loaded_trajectory.setText("No trajectory loaded")
            self.lbl_loaded_waypoints.setText("Waypoints: 0")
            self.lbl_loaded_start.setText("Start: --")
            self.lbl_loaded_target.setText("Target: --")
            self.lbl_trajectory_status.setText("Validation: pending")
            return

        start = plan.start_angles
        target = plan.final_target or start
        self.lbl_loaded_trajectory.setText(Path(plan.file_path).name if plan.file_path else "Loaded from memory")
        self.lbl_loaded_waypoints.setText(f"Waypoints: {len(plan.task_waypoints)}")
        self.lbl_loaded_start.setText(f"Start: {', '.join(f'{value:.1f}°' for value in start)}")
        self.lbl_loaded_target.setText(f"Target: {', '.join(f'{value:.1f}°' for value in target)}")
        self.lbl_trajectory_status.setText("Validation: valid")

    def load_selected_trajectory(self):
        directory = self._trajectory_directory()
        selected = self.cmb_trajectory_files.currentText()
        if selected == "No CSV files found":
            QMessageBox.information(self, "No Trajectory Files", "No trajectory CSV files were found in the Pathways folder.")
            return

        file_path = directory / selected
        self.load_trajectory_file(file_path)

    def open_trajectory_studio(self):
        self.trajectory_studio.show()
        self.trajectory_studio.raise_()
        self.trajectory_studio.activateWindow()

    def browse_trajectory_file(self):
        directory = self._trajectory_directory()
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select a trajectory CSV",
            str(directory),
            "CSV trajectory files (*.csv)",
        )
        if not file_path:
            return
        self.load_trajectory_file(file_path)

    def load_trajectory_file(self, file_path):
        try:
            plan = self.path_executor.load_trajectory_csv(file_path)
        except Exception as exc:
            self.loaded_trajectory_plan = None
            self.loaded_trajectory_path = None
            self.path_executor.path_data = []
            self.path_executor.start_angles = []
            self.btn_execute_path.setEnabled(False)
            self.lbl_trajectory_status.setText(f"Validation: invalid ({exc})")
            QMessageBox.warning(self, "Trajectory Load Failed", f"Could not load the selected trajectory:\n{exc}")
            return

        self.loaded_trajectory_path = str(file_path)
        self.loaded_trajectory_plan = plan
        self.path_executor.path_data = list(plan.task_waypoints)
        self.path_executor.start_angles = list(plan.start_angles)
        self._update_loaded_trajectory_summary(plan)

        if self.manager.robot.connected:
            self.btn_execute_path.setEnabled(True)
            self.btn_move_to_start.setEnabled(True)

        self.status_label.setText(f"Status: Connected 🟢 | Loaded: {Path(file_path).name}")

        if self.cmb_trajectory_files.count() and self.cmb_trajectory_files.currentText() != Path(file_path).name:
            index = self.cmb_trajectory_files.findText(Path(file_path).name)
            if index >= 0:
                self.cmb_trajectory_files.setCurrentIndex(index)

    def open_identification_settings(self):
        trajectory_id = Path(self.loaded_trajectory_path).stem if self.loaded_trajectory_path else ""
        values = dict(self.identification_settings)
        values["commanded_speed"] = self._execution_motion_speed()
        dialog = IdentificationSettingsDialog(values, trajectory_id, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.identification_settings = dialog.get_values()
            self.spin_robot_velocity.setValue(self.identification_settings["commanded_speed"])

    def _get_identification_metadata(self):
        metadata = dict(self.identification_settings)
        metadata["commanded_speed"] = self._execution_motion_speed()
        selected_trajectory_id = Path(self.loaded_trajectory_path).stem if self.loaded_trajectory_path else "UNSPECIFIED_TRAJECTORY"
        metadata["trajectory_id"] = metadata.get("trajectory_id") or selected_trajectory_id
        return metadata

    def start_identification_recording(self, silent=False, metadata_override=None):
        if not self.manager.robot.connected:
            if not silent:
                QMessageBox.warning(self, "Not Connected", "Connect to the robot before starting an identification run.")
            return False

        with self._identification_lock:
            if self.identification_logger is not None:
                if not silent:
                    QMessageBox.warning(self, "Run Already Active", "Stop and save the active identification run before starting another.")
                return False

        metadata = dict(metadata_override) if metadata_override is not None else self._get_identification_metadata()
        trajectory_id = str(metadata.get("trajectory_id") or "UNSPECIFIED_TRAJECTORY")
        metadata["trajectory_id"] = trajectory_id
        if not all(str(metadata.get(key, "")).strip() for key in ("experiment_id", "trajectory_id", "run_id")):
            if not silent:
                QMessageBox.warning(self, "Incomplete Metadata", "Experiment ID, trajectory ID, and run ID are required.")
            return False
        if not silent and not self._confirm_identification_condition(metadata):
            return False
        existing_run = find_existing_identification_run(
            "identification_data", str(metadata["run_id"])
        )
        if existing_run is not None:
            if not silent:
                QMessageBox.warning(
                    self,
                    "Run ID Already Used",
                    f"Run ID {metadata['run_id']} already appears in {existing_run.name}. "
                    "Choose a new run ID for this physical execution.",
                )
            return False

        try:
            with self._identification_lock:
                if self.identification_logger is not None:
                    if not silent:
                        QMessageBox.warning(self, "Run Already Active", "Stop and save the active identification run before starting another.")
                    return False
                self.identification_logger = IdentificationLogger(
                    output_dir="identification_data",
                    experiment_id=metadata["experiment_id"],
                    trajectory_id=trajectory_id,
                    run_id=metadata["run_id"],
                    commanded_speed=metadata["commanded_speed"],
                    payload_g=metadata["payload_g"],
                    contact_label=metadata["contact_label"],
                    grasp_context=metadata["grasp_context"],
                    thermal_condition=metadata["thermal_condition"],
                    overwrite=False,
                )
                logger = self.identification_logger
            self.identification_output_path = str(logger.file_path)
            self._identification_frame_stats_start = self._frame_stats_snapshot()
            self.identification_last_summary = None
            self._identification_last_timestamp = None
            self._identification_last_q = None
            self._identification_last_qdot = None
        except Exception as exc:
            if not silent:
                QMessageBox.critical(self, "Identification Logger Error", str(exc))
            return False
        if not silent:
            self.btn_start_identification.setEnabled(False)
            self.btn_stop_identification.setEnabled(True)
            QMessageBox.information(
                self,
                "Identification Recording Started",
                f"Writing raw telemetry to:\n{logger.file_path}",
            )
        return True

    def _frame_stats_snapshot(self):
        try:
            return self.manager.robot.frame_stats()
        except Exception:
            return None

    def _confirm_identification_condition(self, metadata):
        is_normal_no_load = (
            float(metadata.get("payload_g", float("nan"))) == 0.0
            and str(metadata.get("contact_label", "")).upper() == "NORMAL"
            and str(metadata.get("grasp_context", "")).upper() == "NONE"
        )
        if is_normal_no_load:
            return True
        condition = (
            f"payload_g={metadata.get('payload_g')}, "
            f"contact_label={metadata.get('contact_label')}, "
            f"grasp_context={metadata.get('grasp_context')}"
        )
        answer = QMessageBox.question(
            self,
            "Non-Standard Identification Condition",
            "This run is not labeled as NORMAL no-load data. Its metadata will remain "
            f"{condition}. Continue with this explicitly classified condition?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        return answer == QMessageBox.StandardButton.Yes

    def stop_identification_recording(self, silent=False):
        with self._identification_lock:
            logger = self.identification_logger
            if logger is None:
                return None
            self.identification_logger = None
            self._identification_last_timestamp = None
            self._identification_last_q = None
            self._identification_last_qdot = None
        end_stats = self._frame_stats_snapshot()
        start_stats = getattr(self, "_identification_frame_stats_start", None)
        accounting = {"start": start_stats, "end": end_stats}
        summary = logger.close(frame_accounting=accounting)
        self.identification_last_summary = summary
        self._start_post_run_report(logger.file_path)
        if not silent:
            self.btn_start_identification.setEnabled(self.manager.robot.connected)
            self.btn_stop_identification.setEnabled(False)
            status = summary.get("quality_status", "FAIL") if summary else "FAIL"
            QMessageBox.information(
                self,
                "Identification Run Saved",
                f"{status}: {logger.file_path}\n"
                f"Rows: {summary.get('row_count', 0) if summary else 0}; "
                f"writer drops: {summary.get('writer_dropped_sample_count', 0) if summary else 0}\n"
                f"Quality sidecar: {logger.summary_path}",
            )
        return summary

    def _start_post_run_report(self, csv_path):
        # Report failure must never affect the already-saved raw CSV.
        def worker():
            try:
                from core.run_report import generate_run_report

                generate_run_report(csv_path)
            except Exception:
                logging.getLogger(__name__).exception("Post-run report failed")

        threading.Thread(target=worker, name="post-run-report", daemon=True).start()

    def _show_identification_close_summary(self, summary):
        file_path = summary.get("file_path", self.identification_output_path or "")
        quality_status = summary.get("quality_status", "FAIL")
        self.status_label.setToolTip(str(file_path))
        QMessageBox.information(
            self,
            "Identification Run Closed",
            f"{quality_status}: saved identification data to:\n{file_path}",
        )

    def _reset_execution_metrics(self):
        self.execution_session_active = False
        self.execution_session_start = None
        self.execution_elapsed = 0.0
        self.execution_energy_j = 0.0
        self.execution_last_sample_time = None
        self.execution_last_power_w = None
        self.lbl_execution_timer.setText("00:00.0")
        self.lbl_execution_energy.setText("0.000 J (0.000 Wh)")

    def _start_execution_metrics(self):
        now = time.monotonic()
        self.execution_session_active = True
        self.execution_session_start = now
        self.execution_elapsed = 0.0
        self.execution_energy_j = 0.0
        self.execution_last_sample_time = None
        self.execution_last_power_w = None

    def _update_execution_metrics(self, sample_time, power_w):
        if not self.execution_session_active:
            return

        if self.execution_last_sample_time is not None:
            interval = max(0.0, sample_time - self.execution_last_sample_time)
            self.execution_energy_j += (
                (self.execution_last_power_w + power_w) * 0.5 * interval
            )

        self.execution_last_sample_time = sample_time
        self.execution_last_power_w = power_w
        self.execution_elapsed = max(0.0, time.monotonic() - self.execution_session_start)
        self.lbl_execution_timer.setText(self._format_execution_time(self.execution_elapsed))
        self.lbl_execution_energy.setText(
            f"{self.execution_energy_j:.3f} J ({self.execution_energy_j / 3600.0:.3f} Wh)"
        )

    @staticmethod
    def _format_execution_time(seconds):
        minutes, remainder = divmod(max(0.0, seconds), 60.0)
        return f"{int(minutes):02d}:{remainder:04.1f}"

    def real_view_trail_visibility_changed(self, visible):
        self.real_view.set_trail_visibility(visible)

    def start_jog_with_realtime(self, axis, speed):
        self._show_realtime_view()
        joint_index = int(axis[1:]) - 1
        with self._motion_request_lock:
            self._pending_motion_request = {
                "t_gui": time.perf_counter(),
                "joint_index": joint_index,
                "direction": speed,
                "commanded_speed": speed,
                "target_velocity_deg_s": None,
                "kind": "ALIVEJOG",
            }
            self._pending_motion_request["trial_id"] = self.execution_latency.arm_trial(
                t_gui=self._pending_motion_request["t_gui"],
                joint_index=joint_index,
                direction=speed,
                commanded_speed=speed,
                command_type="ALIVEJOG",
            )
        self.manager.start_jog(axis, speed)

    def stop_jog_with_realtime(self):
        with self._motion_request_lock:
            self._pending_motion_request = None
        self.execution_latency.cancel_pending()
        self._flush_completed_latency_trials()
        self.manager.stop_jog()

    def _reset_trails(self):
        self.sim_tip_trail.clear()
        self.real_tip_trail.clear()
        self.realtime_trail_active = False
        self.sim_view.clear_tip_trail()
        self.real_view.clear_tip_trail()

    def _apply_planned_path(self, temp_path):
        """Load a planned joint path into the preview, playback and export state."""
        self.pre_sim_angles = self.current_sim_angles.copy()
        self.sim_path_full = list(temp_path)
        
        # Load spatial targets for CRI export and execution.
        try:
            self.path_executor.load_path(self.sim_path_full)
        except Exception as e:
            print(f"Warning: Could not load path into executor: {e}")
        
        self.sim_tip_trail = []
        for angles in self.sim_path_full:
            tip_xyz = self.kinematics.get_tip_position(np.radians(angles))
            self.sim_tip_trail.append(tuple(tip_xyz))

        self.total_frames = len(self.sim_path_full)
        self.current_frame = 0
        self.is_playing = False
        self.sim_view.clear_tip_trail()

        self.playback_slider.setEnabled(self.total_frames > 0)
        self.playback_slider.setRange(0, max(self.total_frames - 1, 0))
        self.playback_slider.setValue(0)
        self.lbl_playback_frame.setText(f"Frame: {self.current_frame + 1} / {self.total_frames}")
        self.btn_play.setEnabled(self.total_frames > 1)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(self.total_frames > 0)
        self.btn_export_path.setEnabled(self.total_frames > 0)
        self.btn_move_to_start.setEnabled(self.total_frames > 0 and self.manager.robot.connected)
        self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan) and self.manager.robot.connected)

    def generate_sequence(self):
        """Plan every step of the sequence builder into one continuous pathway."""
        steps = self.sequence_builder.steps()
        if not steps:
            QMessageBox.information(self, "Empty Sequence", "Add at least one step to the sequence.")
            return

        self.open_trajectory_studio()
        self.real_tip_trail.clear()
        self.realtime_trail_active = False
        self.real_view.clear_tip_trail()

        results = self.sequence_planner.check_steps(steps, self.current_sim_angles)
        self.sequence_builder.set_step_status(results)
        failures = [text for text in results if text]
        if failures:
            self.sequence_builder.set_summary(f"{len(failures)} of {len(steps)} step(s) cannot move.")
            QMessageBox.critical(
                self, "Sequence Cannot Be Generated",
                "These steps cannot be executed:\n\n" + "\n\n".join(failures)
                + "\n\nFix or remove them (see the ✖ marks in the list).",
            )
            return

        try:
            plan = self.sequence_planner.plan(steps, self.current_sim_angles)
        except SequenceError as error:
            QMessageBox.critical(self, "Sequence Cannot Be Planned", str(error))
            return

        self.sequence_plan = plan
        self.current_sim_angles = list(plan.path[0])
        self._apply_planned_path(plan.path)
        self.update_visual_points()
        bridges = sum(1 for segment in plan.segments if segment.is_bridge)
        self.sequence_builder.set_summary(
            f"{len(steps)} step(s) + {bridges} auto re-orient move(s) = {len(plan.path)} frames."
        )
        self._update_segment_label()

    def _update_segment_label(self):
        segment = self.sequence_plan.segment_at(self.current_frame) if self.sequence_plan else None
        self.lbl_sim_segment.setText(f"Segment: {segment.label}" if segment else "Segment: --")

    def _show_ik_failure(self, stage):
        reason = self.kinematics.last_ik_error or "the requested point is outside the robot workspace"
        QMessageBox.critical(
            self,
            "Path Cannot Be Planned",
            f"The robot cannot reach the {stage}.\n\nReason: {reason}\n\n"
            "Try a closer target, unlock a joint, or turn off fixed orientation.",
        )

    def start_playback(self):
        if self.total_frames == 0:
            return
        self.is_playing = True
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(True)

    def pause_playback(self):
        self.is_playing = False
        self.btn_play.setEnabled(True)
        self.btn_pause.setEnabled(False)

    def reset_simulation(self):
        self.is_playing = False
        self.current_frame = 0
        self.total_frames = 0
        self.sim_path_full = []
        self.sequence_plan = None
        self.lbl_sim_segment.setText("Segment: --")
        self.current_sim_angles = self.pre_sim_angles.copy()

        self.playback_slider.setEnabled(False)
        self.playback_slider.setRange(0, 0)
        self.playback_slider.setValue(0)
        self.lbl_playback_frame.setText("Frame: 0 / 0")
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(False)
        self.btn_stop.setEnabled(False)
        self.btn_export_path.setEnabled(False)
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan) and self.manager.robot.connected)
        self._reset_trails()

        # Immediately restore the previous simulated pose
        rad_angles = np.radians(self.current_sim_angles)
        points = self.kinematics.get_stick_points(rad_angles)
        self.trajectory_studio.set_simulated_pose(self.current_sim_angles, points[-1])
        self.sim_view.update_view(points)

    def on_playback_slider_changed(self, value):
        if self.total_frames == 0:
            return
        self.current_frame = value
        self.lbl_playback_frame.setText(f"Frame: {self.current_frame + 1} / {self.total_frames}")
        self._update_segment_label()

        # Redraw the view immediately when the slider is used, preserving cumulative trail.
        display_angles = self.sim_path_full[self.current_frame]
        rad_angles = np.radians(display_angles)
        points = self.kinematics.get_stick_points(rad_angles)
        trail_points = self.sim_tip_trail[: self.current_frame + 1]
        self.sim_view.update_view(points, tip_trail=trail_points)

    def export_cri_path(self):
        """Export ordered joint targets as CRI MOVE command payloads."""
        if not self.path_executor.path_data:
            QMessageBox.information(self, "No Path", "There is no simulation pathway to export.")
            return

        default_dir = self.path_executor.default_trajectory_directory()
        default_dir.mkdir(parents=True, exist_ok=True)
        default_path = str(default_dir / "trajectory_export.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Trajectory CSV", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            velocity = self._execution_motion_speed()
            self.path_executor.export_trajectory_csv(file_path, velocity_percent=velocity, start_angles=self.path_executor.start_angles or None)
            self._populate_trajectory_files()
            self.load_trajectory_file(file_path)
            QMessageBox.information(self, "Export Successful", 
                                  f"Robot joint angles exported to:\n{file_path}\n\n"
                                  f"Format: Phase,Step,J1-J6 (degrees),Velocity (%)")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save CSV:\n{e}")

    def export_diagnostics_csv(self):
        """Export the collected real-time diagnostics data to a CSV file."""
        if not self.realtime_data_log:
            QMessageBox.information(self, "No Diagnostics Data", "No real-time diagnostics data has been collected yet.")
            return

        default_path = os.path.join(os.getcwd(), "robot_diagnostics.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Diagnostics CSV", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            self._write_diagnostics_csv(file_path)
            QMessageBox.information(self, "Export Successful", f"Diagnostics exported to:\n{file_path}")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save diagnostics CSV:\n{e}")

    def _build_diagnostics_record(self, record_time, joint_angles, joint_currents):
        joint_positions = self.kinematics.get_joint_positions(np.radians(joint_angles))
        record = [
            float(f"{record_time:.3f}"),
            float(f"{24.0:.3f}"),
            float(f"{self.manager.get_supply_power_W():.3f}")
        ]
        record.extend([float(f"{angle:.3f}") for angle in joint_angles])
        for pt in joint_positions:
            record.extend([float(f"{coord:.3f}") for coord in pt])
        record.extend([float(f"{value:.6f}") for value in joint_currents])
        return record

    def _write_diagnostics_csv(self, file_path):
        """Write logged diagnostics to CSV, with per-joint velocity/acceleration derived
        after the fact (finite differences of logged angles vs. time) so different
        path geometry can be reviewed without touching the live control loop."""
        times = np.array([record[0] for record in self.realtime_data_log], dtype=float)
        angles = np.array([record[3:9] for record in self.realtime_data_log], dtype=float)
        if len(times) > 1:
            velocities = np.gradient(angles, times, axis=0)
            accelerations = np.gradient(velocities, times, axis=0)
        else:
            velocities = np.zeros_like(angles)
            accelerations = np.zeros_like(angles)

        with open(file_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            header = ["Time (s)", "Supply Voltage (V)", "Power (W)"]
            header += [f"J{i+1} Angle (deg)" for i in range(6)]
            header += [f"J{j+1} {axis} (mm)" for j in range(6) for axis in ("X", "Y", "Z")]
            header += [f"J{i+1} Current (mA)" for i in range(6)]
            header += [f"J{i+1} Velocity (deg/s)" for i in range(6)]
            header += [f"J{i+1} Accel (deg/s^2)" for i in range(6)]
            writer.writerow(header)
            for idx, record in enumerate(self.realtime_data_log):
                row = list(record)
                row.extend(float(f"{v:.3f}") for v in velocities[idx])
                row.extend(float(f"{a:.3f}") for a in accelerations[idx])
                writer.writerow(row)

    def prompt_save_diagnostics_csv(self):
        default_path = os.path.join(os.getcwd(), "robot_diagnostics.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Diagnostics CSV", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            self._write_diagnostics_csv(file_path)
            QMessageBox.information(self, "Export Successful", f"Diagnostics exported to:\n{file_path}")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save diagnostics CSV:\n{e}")


    def _init_digital_twin_analytics(self):
        """Creates one FF-RLS estimator, CUSUM detector, and payload
        estimator per joint, parameterized from config.yaml (settings.py)."""
        cfg = get_settings()
        self.rls_estimators = [
            AdaptiveBaselineRLS(
                forgetting_factor=cfg.ff_rls.forgetting_factor,
                initial_covariance_scale=cfg.ff_rls.initial_covariance,
            )
            for _ in range(6)
        ]
        self.cusum_detectors = [
            CUSUMDetector(k=cfg.cusum.k, h=cfg.cusum.h) for _ in range(6)
        ]
        self.payload_estimators = [
            PayloadEstimator(
                theta_empty=np.zeros(3),
                mass_sensitivity_gain=cfg.payload.mass_sensitivity_gain,
                filter_alpha=cfg.payload.filter_alpha,
                attach_threshold_g=cfg.payload.attach_threshold_g,
            )
            for _ in range(6)
        ]
        self.telemetry_logger = None
        self._analytics_prev_joints = None
        self._analytics_prev_time = None

    def _reset_digital_twin_analytics(self):
        """Clears estimator state and plotted history (e.g. on reconnect)."""
        self._init_digital_twin_analytics()
        self.analytics_panel.reset_history()

    def _apply_tuning_live(self):
        """Rebuilds the estimators from the panel's spin box values without
        touching config.yaml, so changes can be A/B tested in the lab."""
        values = self.tuning_panel.get_tuning_values()
        new_settings = get_settings()
        new_settings.ff_rls = FFRLSSettings(**{**vars(new_settings.ff_rls), **values["ff_rls"]})
        new_settings.cusum = CUSUMSettings(**values["cusum"])
        new_settings.payload = PayloadSettings(**{**vars(new_settings.payload), **values["payload"]})
        self._reset_digital_twin_analytics()
        self.tuning_panel.lbl_tuning_status.setText("Applied live (not saved to config.yaml).")

    def _save_tuning_to_config(self):
        """Persists the panel's spin box values to config.yaml and applies
        them immediately."""
        values = self.tuning_panel.get_tuning_values()
        updated = get_settings()
        updated.ff_rls = FFRLSSettings(**{**vars(updated.ff_rls), **values["ff_rls"]})
        updated.cusum = CUSUMSettings(**values["cusum"])
        updated.payload = PayloadSettings(**{**vars(updated.payload), **values["payload"]})
        try:
            save_settings(updated)
        except Exception as e:
            QMessageBox.warning(self, "Save Failed", f"Could not save config.yaml:\n{e}")
            return
        self._reset_digital_twin_analytics()
        self.tuning_panel.lbl_tuning_status.setText("Saved to config.yaml and applied live.")

    def _run_digital_twin_analytics(self, real_joints, real_currents):
        """Feeds the latest joint telemetry through FF-RLS -> CUSUM ->
        payload estimation for every joint and refreshes the Analytics tab.

        The local timing in this method measures total analytics processing
        for one complete cycle only. Network RTT and telemetry-observed motion
        response are tracked separately at the CRI socket and STATUS receive
        boundaries.
        """
        processing_start = time.perf_counter()
        analytics_time = processing_start

        if self._analytics_prev_time is not None:
            dt = max(analytics_time - self._analytics_prev_time, 1e-3)
            q_dot_list = [
                (real_joints[i] - self._analytics_prev_joints[i]) / dt for i in range(6)
            ]
        else:
            q_dot_list = [0.0] * 6
        self._analytics_prev_joints = list(real_joints)
        self._analytics_prev_time = analytics_time

        log_requested = self.analytics_panel.chk_log_csv.isChecked()
        if log_requested and self.telemetry_logger is None:
            self.telemetry_logger = TelemetryLogger("digital_twin_analytics_log.csv")
        elif not log_requested and self.telemetry_logger is not None:
            self.telemetry_logger.close()
            self.telemetry_logger = None

        selected = self.analytics_panel.selected_joint
        selected_status = None
        for i in range(6):
            # Static-gravity torque proxy: sin(theta) approximates the
            # joint-angle-dependent gravity load in absence of a full
            # dynamic model.
            gravity_term = float(np.sin(np.radians(real_joints[i])))
            y_hat, residual, theta_hat = self.rls_estimators[i].update(
                y_measured=real_currents[i], q_dot=q_dot_list[i], gravity_term=gravity_term
            )
            collision_detected, cusum_score = self.cusum_detectors[i].update(residual)
            mass_g, payload_status = self.payload_estimators[i].update(theta_hat)

            record_time = analytics_time - self.log_start_time
            self.analytics_panel.add_sample(i, record_time, real_currents[i], y_hat, residual)

            if i == selected:
                selected_status = (cusum_score, collision_detected, mass_g, payload_status)

            if self.telemetry_logger is not None:
                snapshot = self.execution_latency.snapshot
                motion_response = snapshot.response if snapshot else None
                self.telemetry_logger.log(TelemetryRecord(
                    t_gui=analytics_time,
                    t_virtual=analytics_time,
                    t_physical=analytics_time,
                    sync_delay_s=0.0,
                    joint_id=f"J{i+1}",
                    raw_current=real_currents[i],
                    rls_residual=residual,
                    cusum_score=cusum_score,
                    collision_detected=collision_detected,
                    estimated_mass_g=mass_g,
                    payload_status=payload_status,
                    timestamp=analytics_time,
                    cri_rtt_ms=self.manager.robot.network_latency.statistics.latest_ms,
                    estimated_one_way_ms=self.manager.robot.network_latency.estimated_one_way_ms,
                    rtt_jitter_ms=self.manager.robot.network_latency.statistics.peak_consecutive_jitter_ms,
                    rtt_mean_ms=self.manager.robot.network_latency.statistics.mean_ms,
                    rtt_std_ms=self.manager.robot.network_latency.statistics.standard_deviation_ms,
                    rtt_p95_ms=self.manager.robot.network_latency.statistics.p95_ms,
                    rtt_p99_ms=self.manager.robot.network_latency.statistics.p99_ms,
                    command_id=motion_response.command_id if motion_response else None,
                    joint_index=motion_response.joint_index if motion_response else None,
                    gui_to_tx_ms=(
                        snapshot.gui_to_tx_ms if snapshot else None
                    ),
                    gui_to_motion_ms=(
                        motion_response.gui_to_motion_ms if motion_response else None
                    ),
                    tx_to_motion_ms=(
                        motion_response.tx_to_motion_ms if motion_response else None
                    ),
                ))

        processing_end = time.perf_counter()
        analytics_processing_ms = (processing_end - processing_start) * 1000.0
        if selected_status is not None:
            self.analytics_panel.update_status(*selected_status, analytics_processing_ms)
        # Total local analytics processing duration for this GUI analytics cycle.
        # This is not CRI communication latency and not physical robot response latency.
        return analytics_processing_ms

    def connect_robot(self):
        if self.manager.connect(self.ip_input.text().strip()):
            self._reset_trails()
            self._reset_digital_twin_analytics()
            self.status_label.setText("Status: Connected 🟢")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.btn_connect.setEnabled(False)
            self.btn_start_identification.setEnabled(True)
            for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home]:
                btn.setEnabled(True)
            self._update_jog_enabled()
            # Enable robot control buttons if a path is loaded.
            if self.total_frames > 0:
                self.btn_move_to_start.setEnabled(True)
            self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan))
            # Auto-pause simulation if requested
            if self.chk_auto_pause_on_connect.isChecked() and self.is_playing:
                self.pause_playback()
            # Optionally show real-time tip trail
            self.real_view.toggle_trail(self.chk_real_time_trail.isChecked())
            # Register status callback so STATUS updates can be handled cleanly
            self.manager.robot.register_status_callback(self._on_robot_status_update)
            self.manager.robot.register_command_sent_callback(self._on_command_sent)
        else:
            reason = self.manager.last_connection_error or (
                "The CRI controller did not complete the connection sequence."
            )
            QMessageBox.critical(self, "Robot Connection Failed", reason)

    def disconnect_robot(self):
        self.execution_latency.cancel_pending()
        self._flush_completed_latency_trials()
        closed_run = self.stop_identification_recording(silent=True)
        self.manager.disconnect()
        self._reset_trails()
        self._reset_execution_metrics()
        if self.telemetry_logger is not None:
            self.telemetry_logger.close()
            self.telemetry_logger = None
        self.btn_start_identification.setEnabled(False)
        self.btn_stop_identification.setEnabled(False)
        self.status_label.setText("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red;")
        if closed_run is not None:
            self._show_identification_close_summary(closed_run)
        self.btn_connect.setEnabled(True)
        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home]:
            btn.setEnabled(False)
        self.pressed_keys.clear()
        self._update_jog_enabled()
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_path.setEnabled(False)
        self.real_view.toggle_trail(False)
        self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
        for i, label in enumerate(self.lbl_joints):
            label.setText(f"J{i+1}: -- °")
        for label, axis in zip(self.lbl_position, ("X", "Y", "Z")):
            label.setText(f"{axis}: -- mm")
        for i, label in enumerate(self.lbl_joint_currents):
            label.setText(f"J{i+1}: -- mA")
        self.lbl_total_current.setText("Total: -- mA")
        self.lbl_power.setText("Power: -- W")

    def closeEvent(self, event):
        self.trajectory_studio.close()
        self.execution_latency.cancel_pending()
        self._flush_completed_latency_trials()
        self.stop_identification_recording(silent=True)
        self.manager.disconnect()
        if self.telemetry_logger is not None:
            self.telemetry_logger.close()
            self.telemetry_logger = None
        self.latency_experiment_logger.close()
        super().closeEvent(event)

    def _flush_completed_latency_trials(self):
        """Writes each newly completed trial once from the GUI thread."""
        network = self.manager.robot.network_latency
        for trial in self.execution_latency.drain_completed_trials():
            self.latency_experiment_logger.log_trial(
                trial,
                cri_rtt_ms=network.statistics.latest_ms,
                estimated_one_way_ms=network.estimated_one_way_ms,
                rtt_jitter_ms=network.statistics.peak_consecutive_jitter_ms,
            )

    def _on_command_sent(self, command_id, command, t_tx):
        with self._motion_request_lock:
            request = self._pending_motion_request
            if request is None or not self._matches_motion_request(command, request):
                return
            self._pending_motion_request = None
        self.execution_latency.bind_transmission(
            request["trial_id"], command_id, t_tx
        )

    @staticmethod
    def _matches_motion_request(command, request):
        if request["kind"] != "ALIVEJOG" or not command.startswith("ALIVEJOG "):
            return False
        try:
            speeds = [float(value) for value in command.split()[1:]]
            selected_speed = speeds[request["joint_index"]]
        except (IndexError, ValueError):
            return False
        return selected_speed != 0.0 and selected_speed * request["direction"] > 0.0

    def _on_robot_status_update(self, state, t_rx=None):
        # No direct UI updates here because this callback runs in the receive thread.
        # The tracker uses STATUS packet timing, not the slower GUI timer.
        with self.manager.robot.robot_state_lock:
            fields_seen = set(state.status_fields_seen)
            if "POSJOINTCURRENT" in fields_seen:
                positions = [
                    state.joints_current.A1,
                    state.joints_current.A2,
                    state.joints_current.A3,
                    state.joints_current.A4,
                    state.joints_current.A5,
                    state.joints_current.A6,
                ]
            else:
                positions = [float("nan")] * 6
            currents = (
                [float(value) for value in state.current_joints[:6]]
                if "CURRENTJOINTS" in fields_seen and len(state.current_joints) >= 6
                else [float("nan")] * 6
            )
            frame_timing = getattr(self.manager.robot, "last_status_timing", None)
            self.execution_latency.update_telemetry(positions, t_rx)

        with self._identification_lock:
            logger = self.identification_logger
            if logger is None:
                return
            current_ts = float(t_rx if t_rx is not None else time.perf_counter())
            logger.log_sample(
                timestamp=current_ts,
                q=positions,
                qdot=None,
                current=currents,
                frame_timing=frame_timing,
                status_fields=fields_seen,
            )

    def move_to_start(self):
        """Move the robot to the trajectory-defined start pose."""
        target = list(self.path_executor.start_angles) if self.path_executor.start_angles else None
        if not target and self.path_executor.path_data:
            target = list(self.path_executor.path_data[0])
        if not target:
            QMessageBox.warning(self, "No Path", "No trajectory start pose is available. Please load or calculate a path first.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return

        self._show_realtime_view()
        
        try:
            start_angles = [float(a) for a in target]
            velocity = self._execution_motion_speed()
            
            self.btn_move_to_start.setEnabled(False)
            self.status_label.setText(f"Status: Moving to start at {velocity:.0f}%...")
            self.lbl_execution.setText("Execution: Moving to start")
            self.move_to_start_complete = False
            self.move_to_start_error = None
            self.move_to_start_target = start_angles

            # Execution-latency trials are currently restricted to controlled JOG commands.
            # Move-to-start is excluded because its CRI TX correlation is not yet part of
            # the validated latency experiment protocol.

            self.move_to_start_thread = threading.Thread(
                target=self._move_to_start_worker,
                args=(start_angles, velocity),
                daemon=True,
            )
            self.move_to_start_thread.start()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error moving robot to start position:\n{e}")
            self.status_label.setText("Status: Connected 🟢")
            if self.manager.robot.connected and self.total_frames > 0:
                self.btn_move_to_start.setEnabled(True)

    def _move_to_start_worker(self, start_angles, velocity):
        # Block on EXECEND so we only report success once the robot firmware confirms
        # the move actually finished, instead of assuming completion right after the ack.
        try:
            success = self.manager.robot.move_joints(
                A1=start_angles[0], A2=start_angles[1], A3=start_angles[2],
                A4=start_angles[3], A5=start_angles[4], A6=start_angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=velocity,
                wait_move_finished=True,
                move_finished_timeout=60.0,
            )
            if success:
                self.move_to_start_error = None
            else:
                detail = getattr(self.manager.robot, "last_move_error", "")
                self.move_to_start_error = "Robot move-to-start command failed or timed out. Check the robot status." + (f"\n\nRobot reply: {detail}" if detail else "")
        except Exception as e:
            self.move_to_start_error = str(e)
        self.move_to_start_complete = True

    def execute_path(self):
        """Execute a saved trajectory CSV by preparing to the start pose and then running the task path."""
        if not self.path_executor.path_data and not self.loaded_trajectory_plan:
            QMessageBox.warning(self, "No Path", "No trajectory is loaded. Please pick an existing CSV or calculate a path in the planning tab.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return
        if self.execution_state in {"PREPARING", "RECORDING/EXECUTING"}:
            return

        path_data = list(self.path_executor.path_data)
        trajectory_plan = self.loaded_trajectory_plan
        if trajectory_plan is None:
            QMessageBox.warning(self, "No Path", "Load a validated trajectory CSV from the Pathways tab before running.")
            return

        path_data = [list(waypoint) for waypoint in trajectory_plan.task_waypoints]
        start_angles = list(trajectory_plan.start_angles)

        if start_angles is None:
            QMessageBox.warning(self, "No Start Pose", "The selected trajectory does not contain a valid start position.")
            return

        task_path = list(path_data)

        for joint_index, (low, high) in enumerate(JOINT_LIMITS_DEG):
            values = [float(point[joint_index]) for point in [start_angles, *task_path]]
            if min(values) < low or max(values) > high:
                worst = max(values, key=abs)
                QMessageBox.warning(
                    self, "Joint Limit Exceeded",
                    f"This trajectory needs J{joint_index + 1} = {worst:.1f}°, but the robot limit is "
                    f"{low:.0f}° to {high:.0f}°.\n\nRe-generate the pathway in the Trajectory Studio "
                    "(the planner respects this limit) and export it again.",
                )
                return

        if not task_path:
            QMessageBox.warning(self, "No Task Motion", "The selected trajectory does not contain task motion after the start pose.")
            return
        
        try:
            velocity = self._execution_motion_speed()
            num_waypoints = len(task_path)
            
            # Confirm execution
            reply = QMessageBox.question(
                self, "Run Trajectory",
                f"Move to the trajectory start pose, then run {num_waypoints} waypoints at {velocity:.0f}% velocity?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.No:
                return

            self._show_realtime_view()
            self.lbl_execution.setText("Execution: Running")
            self.real_tip_trail.clear()
            self.realtime_trail_active = True
            self.real_view.clear_tip_trail()
            self.real_view.set_trail_visibility(True)
            
            # Disable buttons during execution
            self.btn_move_to_start.setEnabled(False)
            self.btn_execute_path.setEnabled(False)

            self.realtime_data_log = []
            self.is_recording_diagnostics = False
            self.execution_complete = False
            self.execution_saved = False
            self.execution_error = None
            self.execution_elapsed = 0.0
            self.execution_state = "PREPARING"
            self.execution_stop_event.clear()
            self._reset_execution_metrics()

            print(f"Starting trajectory execution with preparation to {start_angles} and {num_waypoints} task waypoints...")
            self.execution_thread = threading.Thread(
                target=self._execute_path_worker,
                args=(list(start_angles), task_path, velocity),
                daemon=True,
            )
            self.execution_thread.start()
            return
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error executing path:\n{e}")
            self.btn_move_to_start.setEnabled(True)
            self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan) and self.manager.robot.connected)

    def _execute_path_worker(self, start_angles, task_path, velocity):
        error_message = None
        self.execution_error = None
        self.execution_complete = False
        self.is_recording_diagnostics = False

        try:
            self.execution_state = "PREPARING"
            if not self.manager.robot.connected:
                raise RuntimeError("Robot disconnected before preparation could start.")
            success = self.manager.robot.move_joints(
                A1=start_angles[0], A2=start_angles[1], A3=start_angles[2],
                A4=start_angles[3], A5=start_angles[4], A6=start_angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=float(velocity),
                wait_move_finished=True,
                move_finished_timeout=60.0,
            )
            if not success:
                detail = getattr(self.manager.robot, "last_move_error", "")
                raise RuntimeError(
                    "Robot failed to reach the trajectory start pose."
                    + (f"\n\nRobot reply: {detail}" if detail else "")
                )

            self.execution_state = "RECORDING/EXECUTING"
            task_start = time.monotonic()
            self.execution_session_start = task_start
            self.execution_session_active = True
            self.execution_elapsed = 0.0
            self.execution_energy_j = 0.0
            self.execution_last_sample_time = None
            self.execution_last_power_w = None

            self.log_start_time = time.perf_counter()
            self.is_recording_diagnostics = True
            success, error_message = self.manager.execute_path(
                task_path,
                float(velocity),
                stop_event=self.execution_stop_event,
            )
            self.execution_elapsed = max(0.0, time.monotonic() - task_start)
            if not success:
                raise RuntimeError(error_message or "Task execution failed.")
        except Exception as exc:
            error_message = str(exc)
            self.execution_error = error_message
        finally:
            self.execution_session_active = False
            self.is_recording_diagnostics = False
            self.execution_complete = True
            self.execution_error = error_message
            self.execution_state = "COMPLETE" if error_message is None else "ERROR"

    def system_loop(self):
        with self._identification_lock:
            recording_active = self.identification_logger is not None
        self.btn_start_identification.setEnabled(
            self.manager.robot.connected and not recording_active
        )
        self.btn_stop_identification.setEnabled(recording_active)
        if not self.manager.robot.connected:
            if recording_active:
                closed_run = self.stop_identification_recording(silent=True)
                if closed_run is not None:
                    self.status_label.setText(
                        f"Status: Disconnected | Identification {closed_run['quality_status']}"
                    )
                    self._show_identification_close_summary(closed_run)

        # Real-time robot view update (if connected)
        if self.manager.robot.connected:
            self.manager.maintain_jog()
            real_joints = self.manager.get_joint_angles_list()
            # Keep last-sim angles in sync so simulation can continue from current robot pose
            self.current_sim_angles = real_joints
            for i, angle in enumerate(real_joints):
                self.lbl_joints[i].setText(f"J{i+1}: {angle:.1f}°")

            if self.move_to_start_complete:
                self.move_to_start_complete = False
                target = self.move_to_start_target or [0.0] * 6
                if self.move_to_start_error:
                    QMessageBox.warning(self, "Move to Start Failed", self.move_to_start_error)
                    self.status_label.setText("Status: Connected 🟢")
                else:
                    QMessageBox.information(self, "Move to Start Complete",
                                          f"Robot reached the start position.\n"
                                          f"J1={target[0]:.1f}°, J2={target[1]:.1f}°, J3={target[2]:.1f}°\n"
                                          f"J4={target[3]:.1f}°, J5={target[4]:.1f}°, J6={target[5]:.1f}°")
                    self.status_label.setText("Status: Connected 🟢")
                if self.manager.robot.connected and self.total_frames > 0:
                    self.btn_move_to_start.setEnabled(True)


            real_currents = self.manager.get_joint_currents_list()
            for i, current_val in enumerate(real_currents):
                current_text = f"{current_val:.3f} mA" if np.isfinite(current_val) else "unavailable"
                self.lbl_joint_currents[i].setText(f"J{i+1}: {current_text}")

            total_current = self.manager.get_total_current()
            total_current_text = f"{total_current:.3f} mA" if np.isfinite(total_current) else "unavailable"
            self.lbl_total_current.setText(f"Total: {total_current_text}")

            self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
            power_w = self.manager.get_supply_power_W()
            if np.isfinite(power_w):
                self.lbl_power.setText(f"Power: {power_w:.2f} W")
                self._update_execution_metrics(time.monotonic(), power_w)
            else:
                self.lbl_power.setText("Power: unavailable")

            if all(np.isfinite(current) for current in real_currents):
                processing_time_ms = self._run_digital_twin_analytics(real_joints, real_currents)
            else:
                processing_time_ms = 0.0
            network = self.manager.robot.network_latency
            now = time.perf_counter()
            network.expire_pending(now)
            self.execution_latency.expire_pending(now)
            self._flush_completed_latency_trials()
            snapshot = self.execution_latency.snapshot
            self.latency_panel.update_values(
                rtt_ms=network.statistics.latest_ms,
                one_way_ms=network.estimated_one_way_ms,
                jitter_ms=network.statistics.peak_consecutive_jitter_ms,
                mean_ms=network.statistics.mean_ms,
                std_ms=network.statistics.std_ms,
                gui_to_motion_ms=(snapshot.gui_to_motion_ms if snapshot else None),
                tx_to_motion_ms=(snapshot.tx_to_motion_ms if snapshot else None),
                gui_to_tx_ms=(snapshot.gui_to_tx_ms if snapshot else None),
                status=snapshot.status if snapshot else None,
            )

            if self.is_recording_diagnostics:
                record_time = time.perf_counter() - self.log_start_time
                self.realtime_data_log.append(
                    self._build_diagnostics_record(record_time, real_joints, real_currents)
                )

            rad_real = np.radians(real_joints)
            real_points = self.kinematics.get_stick_points(rad_real)
            tip_position = real_points[-1]
            for label, axis, coordinate in zip(self.lbl_position, ("X", "Y", "Z"), tip_position):
                label.setText(f"{axis}: {coordinate:.1f} mm")
            if self.realtime_trail_active:
                self.real_tip_trail.append(tuple(real_points[-1]))
            self.real_view.update_view(real_points, tip_trail=self.real_tip_trail)

        if self.execution_state in {"PREPARING", "RECORDING/EXECUTING"}:
            self.lbl_execution.setText(f"Execution: {self.execution_state.title()}")

        if self.execution_complete and not self.execution_saved:
            self.is_recording_diagnostics = False
            self.execution_saved = True
            self.lbl_execution_timer.setText(self._format_execution_time(self.execution_elapsed))
            if self.execution_error:
                self.lbl_execution.setText("Execution: Error")
                QMessageBox.warning(self, "Run Failed", self.execution_error)
            else:
                self.lbl_execution.setText("Execution: Complete")
                QMessageBox.information(
                    self,
                    "Run Complete",
                    f"Trajectory completed in {self.execution_elapsed:.2f} seconds.",
                )
            self.execution_complete = False
            self.execution_error = None
            self.execution_session_active = False
            self.realtime_trail_active = False
            if self.manager.robot.connected:
                self.btn_move_to_start.setEnabled(bool(self.loaded_trajectory_plan))
                self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan))

        # Simulation playback and simulation view (always active)
        if self.is_playing and self.total_frames > 0:
            if self.current_frame < self.total_frames - 1:
                self.current_frame += 1
            else:
                self.is_playing = False
                self.btn_play.setEnabled(True)
                self.btn_pause.setEnabled(False)

        if self.total_frames > 0:
            sim_display_angles = self.sim_path_full[self.current_frame]
        else:
            sim_display_angles = self.current_sim_angles

        if self.total_frames > 0:
            self.playback_slider.blockSignals(True)
            self.playback_slider.setValue(self.current_frame)
            self.playback_slider.blockSignals(False)
            self.lbl_playback_frame.setText(f"Frame: {self.current_frame + 1} / {self.total_frames}")
            self._update_segment_label()

        rad_sim = np.radians(sim_display_angles)
        sim_points = self.kinematics.get_stick_points(rad_sim)
        self.trajectory_studio.set_simulated_pose(sim_display_angles, sim_points[-1])
        trail_points = None
        if self.total_frames > 0:
            trail_points = self.sim_tip_trail[: self.current_frame + 1]
        self.sim_view.update_view(sim_points, tip_trail=trail_points)

    JOG_KEY_MAP = {
        Qt.Key.Key_A: ("A1", -1), Qt.Key.Key_D: ("A1", 1),
        Qt.Key.Key_W: ("A2", -1), Qt.Key.Key_S: ("A2", 1),
        Qt.Key.Key_Q: ("A3", -1), Qt.Key.Key_E: ("A3", 1),
        Qt.Key.Key_R: ("A4", -1), Qt.Key.Key_F: ("A4", 1),
        Qt.Key.Key_T: ("A5", -1), Qt.Key.Key_G: ("A5", 1),
        Qt.Key.Key_Y: ("A6", -1), Qt.Key.Key_H: ("A6", 1),
    }

    def _update_jog_enabled(self):
        enabled = self.manager.robot.connected and self.btn_jog_toggle.isChecked()
        for btn in self.jog_buttons:
            btn.setEnabled(enabled)

    def _on_jog_toggled(self, checked):
        self.btn_jog_toggle.setText("ON" if checked else "OFF")
        if not checked:
            self.pressed_keys.clear()
            if self.manager.robot.connected:
                self.stop_jog_with_realtime()
        self._update_jog_enabled()

    def _on_jog_speed_changed(self, value):
        self.slider_jog_speed.blockSignals(True)
        self.slider_jog_speed.setValue(int(value))
        self.slider_jog_speed.blockSignals(False)
        # Apply a new speed immediately if a keyboard jog is in progress.
        if self.pressed_keys:
            self.evaluate_keyboard_jog()

    def eventFilter(self, obj, event):
        # Application-wide filter so jog keys work regardless of which child widget has focus.
        event_type = event.type()
        if event_type in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            if not self.isActiveWindow():
                return super().eventFilter(obj, event)
            key = event.key()
            is_jog_key = self.btn_jog_toggle.isChecked() and (key == Qt.Key.Key_I or key in self.JOG_KEY_MAP)
            if not is_jog_key:
                return super().eventFilter(obj, event)
            focus = QApplication.focusWidget()
            if isinstance(focus, (QLineEdit, QAbstractSpinBox)) or (
                isinstance(focus, QComboBox) and focus.isEditable()
            ):
                return super().eventFilter(obj, event)
            if event.isAutoRepeat():
                return True
            if event_type == QEvent.Type.KeyPress:
                self.pressed_keys.add(key)
                self.evaluate_keyboard_jog()
            elif key in self.pressed_keys:
                self.pressed_keys.discard(key)
                self.evaluate_keyboard_jog()
            return True
        if event_type == QEvent.Type.WindowDeactivate and obj is self and self.pressed_keys:
            # Never keep jogging if the window loses focus while keys are held.
            self.pressed_keys.clear()
            self.evaluate_keyboard_jog()
        return super().eventFilter(obj, event)

    def evaluate_keyboard_jog(self):
        if not self.manager.robot.connected:
            return

        axis_speeds = {}
        if Qt.Key.Key_I in self.pressed_keys:
            speed = self.spin_jog_speed.value()
            for key in self.pressed_keys:
                if key in self.JOG_KEY_MAP:
                    axis, sign = self.JOG_KEY_MAP[key]
                    axis_speeds[axis] = axis_speeds.get(axis, 0.0) + sign * speed
        # Opposing keys on one axis cancel out.
        active = {axis: value for axis, value in axis_speeds.items() if value != 0.0}

        if not active:
            self.stop_jog_with_realtime()
            return

        for axis in self.manager.current_jog_speeds:
            self.manager.current_jog_speeds[axis] = 0.0
        axes = list(active.items())
        # The first axis goes through the tracked path so latency measurement still arms.
        self.start_jog_with_realtime(axes[0][0], axes[0][1])
        for axis, value in axes[1:]:
            self.manager.start_jog(axis, value)