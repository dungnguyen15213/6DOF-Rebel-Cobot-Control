import json
import os
import time
import threading
from pathlib import Path
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QPushButton, QLineEdit, QMessageBox,
                             QComboBox, QDoubleSpinBox, QDialog, QSpinBox, QFileDialog, QSlider, QScrollArea, QTabWidget, QCheckBox,
                             QFormLayout, QDialogButtonBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QSplitter
import numpy as np
import csv

from core.path_planner import PathPlanner
from core.path_executor import PathExecutor
from gui.stick_viewer import StickViewer
from gui.analytics_panel import AnalyticsPanel, AlgorithmTuningPanel, LatencyDiagnosticsPanel
from core.kinematics import ReBelKinematics
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

        connection_tab = QWidget()
        connection_layout = QVBoxLayout(connection_tab)
        connection_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        connection_layout.setContentsMargins(6, 6, 6, 6)
        connection_layout.setSpacing(10)

        # Connection Group
        flow_group = QGroupBox("Connection")
        flow_layout = QVBoxLayout()
        flow_layout.setSpacing(6)
        
        self.status_label = QLabel("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red; font-size: 14px;")
        self.lbl_execution = QLabel("Execution: Idle")
        self.lbl_execution.setStyleSheet("font-weight: bold; color: #cfd6dd;")
        self.ip_input = QLineEdit("192.168.3.11")
        
        # Connect / Disconnect Row
        conn_layout = QHBoxLayout()
        conn_layout.setSpacing(4)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self.connect_robot)
        self.btn_disconnect = QPushButton("Disconnect"); self.btn_disconnect.clicked.connect(self.disconnect_robot)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_disconnect)

        # Enable / Reset Row
        en_res_layout = QHBoxLayout()
        en_res_layout.setSpacing(4)
        self.btn_enable = QPushButton("Enable"); self.btn_enable.clicked.connect(self.manager.enable_motors)
        self.btn_reset = QPushButton("Reset"); self.btn_reset.clicked.connect(self.manager.reset_errors)
        en_res_layout.addWidget(self.btn_enable)
        en_res_layout.addWidget(self.btn_reset)
        
        # Home / Config Row
        home_layout = QHBoxLayout()
        home_layout.setSpacing(4)
        self.btn_home = QPushButton("Home")
        self.btn_home.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        self.btn_home.clicked.connect(self.execute_homing)
        
        self.btn_config_home = QPushButton("⚙️")
        self.btn_config_home.clicked.connect(self.open_config_dialog)
        
        home_layout.addWidget(self.btn_home, stretch=4)
        home_layout.addWidget(self.btn_config_home, stretch=1)

        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home]:
            btn.setEnabled(False)

        flow_layout.addWidget(self.status_label)
        flow_layout.addWidget(self.lbl_execution)
        flow_layout.addWidget(self.ip_input)
        flow_layout.addLayout(conn_layout)
        flow_layout.addLayout(en_res_layout)
        flow_layout.addLayout(home_layout)
        flow_group.setLayout(flow_layout)
        connection_layout.addWidget(flow_group)
        connection_layout.addStretch()

        # 2. Jogging Group 
        jog_group = QGroupBox("Jog Control (Hold I)")
        jog_layout = QGridLayout()
        jog_layout.setHorizontalSpacing(10) # Minimal space between the 2 columns
        jog_layout.setVerticalSpacing(4)
        
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
            btn_neg.pressed.connect(lambda a=axis: self.start_jog_with_realtime(a, -15.0))
            btn_neg.released.connect(self.stop_jog_with_realtime)
            btn_pos.pressed.connect(lambda a=axis: self.start_jog_with_realtime(a, 15.0))
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
        sensor_group = QGroupBox("Joint Angles")
        sensor_layout = QGridLayout()
        sensor_layout.setHorizontalSpacing(4) # Tighten column spacing
        sensor_layout.setVerticalSpacing(4)   # Tighten row spacing
        
        self.lbl_joints = [QLabel(f"J{i+1}: -- °") for i in range(6)]
        for i, lbl in enumerate(self.lbl_joints):
            lbl.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 2px;")
            row = i // 2
            col = i % 2
            sensor_layout.addWidget(lbl, row, col)
            
        sensor_group.setLayout(sensor_layout)

        position_group = QGroupBox("Current Position")
        position_layout = QHBoxLayout()
        self.lbl_position = [QLabel(f"{axis}: -- mm") for axis in ("X", "Y", "Z")]
        for label in self.lbl_position:
            label.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
            position_layout.addWidget(label)
        position_group.setLayout(position_layout)

        # 4. Real-time Status & Diagnostics
        diag_group = QGroupBox("Diagnostics")
        diag_layout = QGridLayout()
        diag_layout.setHorizontalSpacing(6)
        diag_layout.setVerticalSpacing(6)

        self.lbl_supply_voltage = QLabel("Supply Voltage: 24 V")
        self.lbl_supply_voltage.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
        diag_layout.addWidget(self.lbl_supply_voltage, 0, 0, 1, 2)

        self.lbl_power = QLabel("Power: -- W")
        self.lbl_power.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
        diag_layout.addWidget(self.lbl_power, 1, 0, 1, 2)

        self.lbl_total_current = QLabel("Total Joint Current: -- mA")
        self.lbl_total_current.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
        diag_layout.addWidget(self.lbl_total_current, 2, 0, 1, 2)

        self.lbl_joint_currents = [QLabel(f"J{i+1}: -- mA") for i in range(6)]
        for i, lbl in enumerate(self.lbl_joint_currents):
            lbl.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;")
            row = 3 + (i // 2)
            col = i % 2
            diag_layout.addWidget(lbl, row, col)

        diag_group.setLayout(diag_layout)

        status_panel = QGroupBox("ROBOT STATUS")
        status_layout = QVBoxLayout(status_panel)
        status_layout.setContentsMargins(6, 10, 6, 6)
        status_layout.setSpacing(4)
        status_layout.addWidget(position_group)
        status_layout.addWidget(sensor_group)
        status_layout.addWidget(diag_group)

        connection_scroll = QScrollArea()
        connection_scroll.setWidgetResizable(True)
        connection_scroll.setWidget(connection_tab)
        connection_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(connection_scroll, "Connection")

        jog_scroll = QScrollArea()
        jog_scroll.setWidgetResizable(True)
        jog_scroll.setWidget(jog_tab)
        jog_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(jog_scroll, "JOG")

        # ---- Pathways tab: geometry, constraints, points, playback ----
        traj_tab = QWidget()
        sim_layout = QVBoxLayout(traj_tab)
        sim_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        sim_layout.setContentsMargins(6, 6, 6, 6)
        sim_layout.setSpacing(8)

        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("Path type:") )
        self.combo_path_mode = QComboBox()
        self.combo_path_mode.addItems(["Joint interpolation", "Cartesian straight line"])
        path_layout.addWidget(self.combo_path_mode)
        sim_layout.addLayout(path_layout)

        constraint_group = QGroupBox("Cartesian constraints")
        constraint_layout = QVBoxLayout()
        lock_layout = QHBoxLayout()
        lock_layout.addWidget(QLabel("Lock:"))
        self.lock_joint_checks = []
        for joint_index in range(6):
            check = QCheckBox(f"J{joint_index + 1}")
            self.lock_joint_checks.append(check)
            lock_layout.addWidget(check)
        constraint_layout.addLayout(lock_layout)
        self.chk_keep_orientation = QCheckBox("Keep tool orientation from start")
        constraint_layout.addWidget(self.chk_keep_orientation)
        constraint_group.setLayout(constraint_layout)
        sim_layout.addWidget(constraint_group)

        # Start point
        start_group = QGroupBox("Start (X, Y, Z mm)")
        start_layout = QGridLayout()
        self.start_inputs = []
        labels = ["X:", "Y:", "Z:"]
        default_start = [200.0, 0.0, 400.0]
        for i in range(3):
            start_layout.addWidget(QLabel(labels[i]), i, 0)
            spin_box = QDoubleSpinBox()
            spin_box.setRange(-800.0, 800.0)
            spin_box.setSingleStep(10.0)
            spin_box.setValue(default_start[i])
            spin_box.valueChanged.connect(self.update_visual_points)
            start_layout.addWidget(spin_box, i, 1)
            self.start_inputs.append(spin_box)
        start_group.setLayout(start_layout)

        # Place Start and Target side-by-side to save vertical space
        points_layout = QHBoxLayout()
        points_layout.addWidget(start_group)

        # Target point
        target_group = QGroupBox("Target (X, Y, Z mm)")
        target_layout = QGridLayout()
        self.target_inputs = []
        default_target = [200.0, 200.0, 200.0]
        for i in range(3):
            target_layout.addWidget(QLabel(labels[i]), i, 0)
            spin_box = QDoubleSpinBox()
            spin_box.setRange(-800.0, 800.0)
            spin_box.setSingleStep(10.0)
            spin_box.setValue(default_target[i])
            spin_box.valueChanged.connect(self.update_visual_points)
            target_layout.addWidget(spin_box, i, 1)
            self.target_inputs.append(spin_box)
        target_group.setLayout(target_layout)
        points_layout.addWidget(target_group)

        sim_layout.addLayout(points_layout)

        resolution_layout = QHBoxLayout()
        resolution_layout.addWidget(QLabel("Waypoints:"))
        self.spin_waypoint_count = QSpinBox()
        self.spin_waypoint_count.setRange(2, 500)
        self.spin_waypoint_count.setValue(30)
        resolution_layout.addWidget(self.spin_waypoint_count)
        sim_layout.addLayout(resolution_layout)

        # Plan simulation button
        self.btn_run_sim = QPushButton("Calculate Path")
        self.btn_run_sim.setStyleSheet("background-color: #27ae60; color: white; padding: 6px; font-size: 12px; font-weight: bold;")
        self.btn_run_sim.clicked.connect(self.trigger_simulation)
        sim_layout.addWidget(self.btn_run_sim)

        self.btn_export_path = QPushButton("Export Pathway CSV")
        self.btn_export_path.setEnabled(False)
        self.btn_export_path.clicked.connect(self.export_cri_path)
        sim_layout.addWidget(self.btn_export_path)

        # ---- Execution tab: trajectory selection, run settings, and live metrics ----
        execution_tab = QWidget()
        execution_layout = QVBoxLayout(execution_tab)
        execution_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        execution_layout.setContentsMargins(6, 6, 6, 6)
        execution_layout.setSpacing(8)

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
        trajectory_layout.addWidget(self.lbl_loaded_waypoints, 3, 0, 1, 3)
        trajectory_layout.addWidget(self.lbl_loaded_start, 4, 0, 1, 3)
        trajectory_layout.addWidget(self.lbl_loaded_target, 5, 0, 1, 3)
        trajectory_layout.addWidget(self.lbl_trajectory_status, 6, 0, 1, 3)
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
        self.btn_identification_settings = QPushButton("Identification Settings...")
        self.btn_identification_settings.clicked.connect(self.open_identification_settings)
        run_settings_layout.addWidget(self.btn_identification_settings)
        run_settings_layout.addStretch()
        execution_layout.addLayout(run_settings_layout)

        self.btn_execute_path = QPushButton("Run")
        self.btn_execute_path.setStyleSheet("background-color: #c0392b; color: white; padding: 6px; font-weight: bold;")
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self.execute_path)
        execution_layout.addWidget(self.btn_execute_path)

        identification_controls = QHBoxLayout()
        self.btn_start_identification = QPushButton("Start Identification Run")
        self.btn_start_identification.setEnabled(False)
        self.btn_start_identification.clicked.connect(self.start_identification_recording)
        self.btn_stop_identification = QPushButton("Stop & Save Run")
        self.btn_stop_identification.setEnabled(False)
        self.btn_stop_identification.clicked.connect(self.stop_identification_recording)
        identification_controls.addWidget(self.btn_start_identification)
        identification_controls.addWidget(self.btn_stop_identification)
        execution_layout.addLayout(identification_controls)

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

        sim_layout.addLayout(playback_layout)

        # Simulation / Real-time Options Row
        opts_layout = QHBoxLayout()
        self.chk_auto_pause_on_connect = QCheckBox("Auto-pause Simulation on Connect")
        self.chk_auto_pause_on_connect.setChecked(True)
        opts_layout.addWidget(self.chk_auto_pause_on_connect)

        self.chk_real_time_trail = QCheckBox("Show Real-time Tip Trail")
        self.chk_real_time_trail.setChecked(True)
        self.chk_real_time_trail.toggled.connect(self.real_view_trail_visibility_changed)
        opts_layout.addWidget(self.chk_real_time_trail)

        sim_layout.addLayout(opts_layout)

        # Close out the Path Planning tab
        traj_scroll = QScrollArea()
        traj_scroll.setWidgetResizable(True)
        traj_scroll.setWidget(traj_tab)
        traj_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(traj_scroll, "Pathways")

        session_group = QGroupBox("Execution Session")
        session_layout = QGridLayout()
        session_layout.setHorizontalSpacing(6)
        session_layout.setVerticalSpacing(6)
        self.lbl_execution_timer = QLabel("00:00.0")
        self.lbl_execution_energy = QLabel("0.000 J (0.000 Wh)")
        for label in (self.lbl_execution_timer, self.lbl_execution_energy):
            label.setStyleSheet("font-family: monospace; color: #00FF00; background-color: #111; padding: 4px;")
        session_layout.addWidget(QLabel("Elapsed time:"), 0, 0)
        session_layout.addWidget(self.lbl_execution_timer, 0, 1)
        session_layout.addWidget(QLabel("Total energy:"), 1, 0)
        session_layout.addWidget(self.lbl_execution_energy, 1, 1)
        session_group.setLayout(session_layout)
        execution_layout.addWidget(session_group)
        
        # Export real-time diagnostics data
        export_diag_layout = QHBoxLayout()
        self.btn_export_voltage_csv = QPushButton("📥 Export Diagnostics CSV")
        self.btn_export_voltage_csv.setStyleSheet("background-color: #8e44ad; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_export_voltage_csv.clicked.connect(self.export_diagnostics_csv)
        export_diag_layout.addWidget(self.btn_export_voltage_csv)
        execution_layout.addLayout(export_diag_layout)
        
        # Robot control buttons (Move to Start & Execute)
        robot_control_layout = QHBoxLayout()
        robot_control_layout.setSpacing(2)
        
        # Button: Move to Start Position
        self.btn_move_to_start = QPushButton("🎯 Move to Start")
        self.btn_move_to_start.setStyleSheet("background-color: #16a085; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_move_to_start.setEnabled(False)
        self.btn_move_to_start.clicked.connect(self.move_to_start)
        robot_control_layout.addWidget(self.btn_move_to_start)
        
        execution_layout.addLayout(robot_control_layout)
        execution_layout.addStretch()

        self._populate_trajectory_files()

        execution_scroll = QScrollArea()
        execution_scroll.setWidgetResizable(True)
        execution_scroll.setWidget(execution_tab)
        execution_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(execution_scroll, "Execution")

        # Purple tab strip and orange content area remain above the green footer.
        left_sidebar_layout.addWidget(status_panel)
        left_sidebar_layout.addWidget(left_tabs, 1)

        # ==========================================
        # RIGHT PANEL: Tabbed CAD Views (Simulation + Real-time)
        # ==========================================
        self.view_tabs = QTabWidget()
        self.view_tabs.setMinimumWidth(620)

        # Simulation View Tab
        sim_tab = QWidget()
        sim_tab_layout = QVBoxLayout(sim_tab)
        sim_tab_layout.setContentsMargins(6, 6, 6, 6)
        self.sim_view = StickViewer()
        sim_tab_layout.addWidget(self.sim_view)
        self.view_tabs.addTab(sim_tab, "Simulation View")

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

        main_layout.addWidget(left_sidebar)
        main_layout.addWidget(self.view_tabs, 1)



    # ==========================================
    # LOGIC FUNCTIONS
    # ==========================================

    def update_visual_points(self):
        start_xyz = [box.value() for box in self.start_inputs]
        target_xyz = [box.value() for box in self.target_inputs]
        # Simulation view should show start/target points
        self.sim_view.set_visual_points(start_xyz, target_xyz)
        # Real-time view does not need start/target markers, but keep synced optionally
        self.real_view.set_visual_points(start_xyz, target_xyz)

    def _show_realtime_view(self):
        """Focus the live view before any command that moves the robot."""
        self.view_tabs.setCurrentIndex(1)

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
        summary = logger.close()
        self.identification_last_summary = summary
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

    def trigger_simulation(self):
        self.view_tabs.setCurrentIndex(0)
        self.real_tip_trail.clear()
        self.realtime_trail_active = False
        self.real_view.clear_tip_trail()
        start_xyz = [box.value() for box in self.start_inputs]
        target_xyz = [box.value() for box in self.target_inputs]
        waypoint_count = self.spin_waypoint_count.value()
        
        selected_locks = {
            index: self.current_sim_angles[index]
            for index, check in enumerate(self.lock_joint_checks)
            if check.isChecked()
        }
        start_angles = self.kinematics.inverse_kinematics(
            start_xyz,
            initial_guess_angles=self.current_sim_angles,
            locked_joints=selected_locks,
        )
        if start_angles is None:
            self._show_ik_failure("starting point")
            return

        self.current_sim_angles = start_angles
        
        locked_joints = {index: start_angles[index] for index in selected_locks}
        if self.combo_path_mode.currentText() == "Cartesian straight line":
            target_rotation = None
            if self.chk_keep_orientation.isChecked():
                target_rotation = self.kinematics.get_tip_transform(
                    np.radians(start_angles)
                )[:3, :3]

            def solve_waypoint(point, previous_angles):
                return self.kinematics.inverse_kinematics(
                    point,
                    initial_guess_angles=previous_angles or start_angles,
                    locked_joints=locked_joints,
                    target_rotation=target_rotation,
                )

            try:
                temp_path = self.path_planner.generate_cartesian_line(
                    start_xyz, target_xyz, waypoint_count, solve_waypoint
                )
            except ValueError:
                self._show_ik_failure("Cartesian line")
                return
        else:
            target_angles = self.kinematics.inverse_kinematics(
                target_xyz,
                initial_guess_angles=start_angles,
                locked_joints=locked_joints,
            )
            if target_angles is None:
                self._show_ik_failure("target point")
                return
            for index, angle in locked_joints.items():
                target_angles[index] = angle

            temp_path = self.path_planner.generate_joint_path(
                start_angles, target_angles, waypoint_count
            )

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
        for i, angle in enumerate(self.current_sim_angles):
            self.lbl_joints[i].setText(f"J{i+1}: {angle:.1f}° (SIM)")
        rad_angles = np.radians(self.current_sim_angles)
        points = self.kinematics.get_stick_points(rad_angles)
        self.sim_view.update_view(points)

    def on_playback_slider_changed(self, value):
        if self.total_frames == 0:
            return
        self.current_frame = value
        self.lbl_playback_frame.setText(f"Frame: {self.current_frame + 1} / {self.total_frames}")

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
            for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
                btn.setEnabled(True)
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
        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
            btn.setEnabled(False)
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_path.setEnabled(False)
        self.real_view.toggle_trail(False)
        self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
        self.lbl_total_current.setText("Total Joint Current: -- mA")
        self.lbl_power.setText("Power: -- W")

    def closeEvent(self, event):
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
        if all(np.isfinite(value) for value in positions):
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
            self.move_to_start_error = None if success else "Robot move-to-start command failed or timed out. Check the robot status."
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
        if self.identification_logger is not None:
            QMessageBox.warning(self, "Identification Run Active", "Stop and save the manual identification run before starting trajectory execution.")
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

        if not task_path:
            QMessageBox.warning(self, "No Task Motion", "The selected trajectory does not contain task motion after the start pose.")
            return
        
        try:
            velocity = self._execution_motion_speed()
            metadata = self._get_identification_metadata()
            metadata["trajectory_id"] = metadata.get("trajectory_id") or trajectory_plan.trajectory_id
            if not all(str(metadata.get(key, "")).strip() for key in ("experiment_id", "trajectory_id", "run_id")):
                QMessageBox.warning(self, "Incomplete Metadata", "Experiment ID, trajectory ID, and run ID are required before motion.")
                return
            if not self._confirm_identification_condition(metadata):
                return
            existing_run = find_existing_identification_run(
                "identification_data", str(metadata["run_id"])
            )
            if existing_run is not None:
                QMessageBox.warning(
                    self,
                    "Run ID Already Used",
                    f"Run ID {metadata['run_id']} already appears in {existing_run.name}. "
                    "Choose a new run ID before starting robot motion.",
                )
                return
            num_waypoints = len(task_path)
            
            # Confirm execution
            reply = QMessageBox.question(
                self, "Run Experiment",
                f"Prepare to start pose and execute {num_waypoints} task waypoints at {velocity}% velocity?\n\n"
                "This will move to the CSV-defined start pose before the measured experiment motion.",
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
                args=(list(start_angles), task_path, velocity, metadata),
                daemon=True,
            )
            self.execution_thread.start()
            return
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error executing path:\n{e}")
            self.btn_move_to_start.setEnabled(True)
            self.btn_execute_path.setEnabled(bool(self.loaded_trajectory_plan) and self.manager.robot.connected)

    def _execute_path_worker(self, start_angles, task_path, velocity, metadata):
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
                raise RuntimeError("Robot failed to reach the trajectory start pose.")

            self.start_identification_recording(silent=True, metadata_override=metadata)
            if self.identification_logger is None:
                raise RuntimeError("Identification logging could not be started; task motion was not sent.")

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
            if self.identification_logger is not None:
                self.stop_identification_recording(silent=True)
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
            self.lbl_total_current.setText(f"Total Joint Current: {total_current_text}")

            self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
            power_w = self.manager.get_supply_power_W()
            if np.isfinite(power_w):
                self.lbl_power.setText(
                    f"Power: {power_w:.2f} W (24 V × {total_current:.0f} mA / 1000)"
                )
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
                    f"Task motion completed in {self.execution_elapsed:.2f} seconds.\n\n"
                    f"Identification CSV saved to:\n{self.identification_output_path or 'identification_data'}",
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

        # Only show SIM suffix when robot is not connected
        if not self.manager.robot.connected:
            for i, angle in enumerate(sim_display_angles):
                self.lbl_joints[i].setText(f"J{i+1}: {angle:.1f}° (SIM)")

        if self.total_frames > 0:
            self.playback_slider.blockSignals(True)
            self.playback_slider.setValue(self.current_frame)
            self.playback_slider.blockSignals(False)
            self.lbl_playback_frame.setText(f"Frame: {self.current_frame + 1} / {self.total_frames}")

        rad_sim = np.radians(sim_display_angles)
        sim_points = self.kinematics.get_stick_points(rad_sim)
        trail_points = None
        if self.total_frames > 0:
            trail_points = self.sim_tip_trail[: self.current_frame + 1]
        self.sim_view.update_view(sim_points, tip_trail=trail_points)

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            self.pressed_keys.add(event.key())
            self.evaluate_keyboard_jog()

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat() and event.key() in self.pressed_keys:
            self.pressed_keys.remove(event.key())
            self.evaluate_keyboard_jog()

    def evaluate_keyboard_jog(self):
        if not self.manager.robot.connected: return
        for key in self.manager.current_jog_speeds:
            self.manager.current_jog_speeds[key] = 0.0
            
        if Qt.Key.Key_I not in self.pressed_keys:
            self.stop_jog_with_realtime()
            return
            
        speed = 15