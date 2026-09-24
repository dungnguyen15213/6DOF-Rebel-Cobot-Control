import json
import os
import time
import threading
from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QGroupBox, QLabel, QPushButton, QLineEdit, QMessageBox,
                             QComboBox, QDoubleSpinBox, QDialog, QSpinBox, QFileDialog, QSlider, QScrollArea, QTabWidget, QCheckBox) 
from PyQt6.QtCore import Qt, QTimer
import numpy as np
import csv

from core.path_planner import PathPlanner
from core.path_executor import PathExecutor
from gui.stick_viewer import StickViewer
from gui.analytics_panel import AnalyticsPanel
from core.kinematics import ReBelKinematics
from core.digital_twin_analytics import (
    AdaptiveBaselineRLS, CUSUMDetector, PayloadEstimator, TelemetryLogger, TelemetryRecord,
)
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


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.path_planner = PathPlanner()
        self.path_executor = PathExecutor()
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
        self._init_digital_twin_analytics()
        self.realtime_data_log = []
        self.log_start_time = time.time()
        self.is_recording_diagnostics = False
        self.execution_complete = False
        self.execution_saved = False
        self.execution_error = None
        self.execution_elapsed = 0.0
        self.execution_session_active = False
        self.execution_session_start = None
        self.execution_energy_j = 0.0
        self.execution_last_sample_time = None
        self.execution_last_power_w = None
        self.execution_stop_event = threading.Event()
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
            btn_neg.released.connect(self.manager.stop_jog)
            btn_pos.pressed.connect(lambda a=axis: self.start_jog_with_realtime(a, 15.0))
            btn_pos.released.connect(self.manager.stop_jog)
            
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

        # ---- Path Planning tab: geometry, constraints, points, playback ----
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
        left_tabs.addTab(traj_scroll, "Path Planning")

        # ---- Execution tab: path controls, live session metrics, exports ----
        execution_tab = QWidget()
        execution_layout = QVBoxLayout(execution_tab)
        execution_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        execution_layout.setContentsMargins(6, 6, 6, 6)
        execution_layout.setSpacing(8)

        # Shared motion speed for Move to Start and Execute Path.
        vel_layout = QHBoxLayout()
        vel_layout.setSpacing(3)
        vel_layout.addWidget(QLabel("Motion Speed (%):", ), 0)
        self.spin_robot_velocity = QDoubleSpinBox()
        self.spin_robot_velocity.setRange(1.0, 100.0)
        self.spin_robot_velocity.setValue(50.0)
        self.spin_robot_velocity.setDecimals(0)
        self.spin_robot_velocity.setSuffix(" %")
        self.spin_robot_velocity.setToolTip("CRI speed used by Move to Start and Execute Path")
        self.spin_robot_velocity.setMaximumWidth(60)
        vel_layout.addWidget(self.spin_robot_velocity, 0)
        vel_layout.addStretch()
        execution_layout.addLayout(vel_layout)

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
        
        # Export ordered joint targets for CRI inspection or reuse.
        export_btn_layout = QHBoxLayout()
        self.btn_export_path = QPushButton("📤 Export CRI Path CSV")
        self.btn_export_path.setStyleSheet("background-color: #e74c3c; color: white; padding: 6px; font-weight: bold; font-size: 11px;")
        self.btn_export_path.setEnabled(False)
        self.btn_export_path.setMinimumHeight(40)
        self.btn_export_path.clicked.connect(self.export_cri_path)
        export_btn_layout.addWidget(self.btn_export_path)

        execution_layout.addLayout(export_btn_layout)

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
        
        self.btn_execute_path = QPushButton("▶️ Execute Path")
        self.btn_execute_path.setStyleSheet("background-color: #c0392b; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_execute_path.setEnabled(False)
        self.btn_execute_path.clicked.connect(self.execute_path)
        robot_control_layout.addWidget(self.btn_execute_path)
        
        execution_layout.addLayout(robot_control_layout)
        execution_layout.addStretch()

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
        self.real_view = StickViewer()
        real_tab_layout.addWidget(self.real_view)
        self.view_tabs.addTab(real_tab, "Real-time View")

        # Analytics Tab (FF-RLS baseline, CUSUM collision, payload estimation)
        self.analytics_panel = AnalyticsPanel()
        self.analytics_panel.btn_reset.clicked.connect(self._reset_digital_twin_analytics)
        self.analytics_panel.load_tuning_values(get_settings())
        self.analytics_panel.btn_apply_tuning.clicked.connect(self._apply_tuning_live)
        self.analytics_panel.btn_save_tuning.clicked.connect(self._save_tuning_to_config)
        self.view_tabs.addTab(self.analytics_panel, "Analytics")

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
        self.lbl_execution_timer.setText("00:00.0")
        self.lbl_execution_energy.setText("0.000 J (0.000 Wh)")

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
        self.execution_elapsed = sample_time - self.execution_session_start
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
        self.manager.start_jog(axis, speed)

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
        self.btn_execute_path.setEnabled(self.total_frames > 0 and self.manager.robot.connected)

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
        self.btn_execute_path.setEnabled(False)
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

        default_path = os.path.join(os.getcwd(), "cri_path_commands.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save CRI Path Commands", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            velocity = self._execution_motion_speed()
            self.path_executor.export_to_csv_cri_commands(file_path, velocity_percent=velocity)
            QMessageBox.information(self, "Export Successful", 
                                  f"Robot joint angles exported to:\n{file_path}\n\n"
                                  f"Format: Step, Time (s), J1-J6 (degrees), Velocity (%)")
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
        values = self.analytics_panel.get_tuning_values()
        new_settings = get_settings()
        new_settings.ff_rls = FFRLSSettings(**{**vars(new_settings.ff_rls), **values["ff_rls"]})
        new_settings.cusum = CUSUMSettings(**values["cusum"])
        new_settings.payload = PayloadSettings(**{**vars(new_settings.payload), **values["payload"]})
        self._reset_digital_twin_analytics()
        self.analytics_panel.lbl_tuning_status.setText("Applied live (not saved to config.yaml).")

    def _save_tuning_to_config(self):
        """Persists the panel's spin box values to config.yaml and applies
        them immediately."""
        values = self.analytics_panel.get_tuning_values()
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
        self.analytics_panel.lbl_tuning_status.setText("Saved to config.yaml and applied live.")

    def _run_digital_twin_analytics(self, real_joints, real_currents):
        """Feeds the latest joint telemetry through FF-RLS -> CUSUM ->
        payload estimation for every joint and refreshes the Analytics tab.

        t_gui/t_physical mark the start/end of this poll cycle; since the
        CRI stream does not expose per-packet send timestamps, this is an
        approximation of the GUI<->physical round-trip latency rather than
        a true command-dispatch delay.
        """
        t_gui = time.time()

        if self._analytics_prev_time is not None:
            dt = max(t_gui - self._analytics_prev_time, 1e-3)
            q_dot_list = [
                (real_joints[i] - self._analytics_prev_joints[i]) / dt for i in range(6)
            ]
        else:
            q_dot_list = [0.0] * 6
        self._analytics_prev_joints = list(real_joints)
        self._analytics_prev_time = t_gui

        log_requested = self.analytics_panel.chk_log_csv.isChecked()
        if log_requested and self.telemetry_logger is None:
            self.telemetry_logger = TelemetryLogger("digital_twin_analytics_log.csv")
        elif not log_requested and self.telemetry_logger is not None:
            self.telemetry_logger.close()
            self.telemetry_logger = None

        selected = self.analytics_panel.selected_joint
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

            record_time = t_gui - self.log_start_time
            self.analytics_panel.add_sample(i, record_time, real_currents[i], y_hat, residual)

            if i == selected:
                t_physical = time.time()
                sync_delay_ms = (t_physical - t_gui) * 1000.0
                self.analytics_panel.update_status(
                    cusum_score, collision_detected, mass_g, payload_status, sync_delay_ms
                )

            if self.telemetry_logger is not None:
                t_physical = time.time()
                self.telemetry_logger.log(TelemetryRecord(
                    t_gui=t_gui,
                    t_virtual=t_gui,
                    t_physical=t_physical,
                    sync_delay_s=t_physical - t_gui,
                    joint_id=f"J{i+1}",
                    raw_current=real_currents[i],
                    rls_residual=residual,
                    cusum_score=cusum_score,
                    collision_detected=collision_detected,
                    estimated_mass_g=mass_g,
                    payload_status=payload_status,
                ))

    def connect_robot(self):
        if self.manager.connect(self.ip_input.text().strip()):
            self._reset_trails()
            self._reset_digital_twin_analytics()
            self.status_label.setText("Status: Connected 🟢")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.btn_connect.setEnabled(False)
            for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
                btn.setEnabled(True)
            # Enable robot control buttons if a path is loaded.
            if self.total_frames > 0:
                self.btn_move_to_start.setEnabled(True)
                self.btn_execute_path.setEnabled(True)
            # Auto-pause simulation if requested
            if self.chk_auto_pause_on_connect.isChecked() and self.is_playing:
                self.pause_playback()
            # Optionally show real-time tip trail
            self.real_view.toggle_trail(self.chk_real_time_trail.isChecked())
            # Register status callback so STATUS updates can be handled cleanly
            self.manager.robot.register_status_callback(self._on_robot_status_update)
        else:
            QMessageBox.critical(self, "Error", "Failed to connect to physical robot.")

    def disconnect_robot(self):
        self.manager.disconnect()
        self._reset_trails()
        self._reset_execution_metrics()
        if self.telemetry_logger is not None:
            self.telemetry_logger.close()
            self.telemetry_logger = None
        self.status_label.setText("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red;")
        self.btn_connect.setEnabled(True)
        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
            btn.setEnabled(False)
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_path.setEnabled(False)
        self.real_view.toggle_trail(False)
        self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
        self.lbl_total_current.setText("Total Joint Current: -- mA")
        self.lbl_power.setText("Power: -- W")

    def _on_robot_status_update(self, state):
        # No direct UI updates here because this callback runs in the receive thread.
        # The main UI timer polls the latest robot state and updates labels safely.
        return

    def move_to_start(self):
        """Move the robot to the first target in the planned path."""
        if not self.path_executor.path_data:
            QMessageBox.warning(self, "No Path", "No path loaded. Please calculate a path first.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return

        self._show_realtime_view()
        
        try:
            # Get the first waypoint (start position).
            start_angles = self.path_executor.path_data[0]
            start_angles = [float(a) for a in start_angles]
            velocity = self._execution_motion_speed()
            
            self.btn_move_to_start.setEnabled(False)
            self.status_label.setText(f"Status: Moving to start at {velocity:.0f}%...")
            self.lbl_execution.setText("Execution: Moving to start")
            self.move_to_start_complete = False
            self.move_to_start_error = None
            self.move_to_start_target = start_angles

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
        """Execute the loaded spatial path through the CRI controller."""
        if not self.path_executor.path_data:
            QMessageBox.warning(self, "No Path", "No path loaded. Please calculate a path first.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return
        
        try:
            velocity = self._execution_motion_speed()
            num_waypoints = len(self.path_executor.path_data)
            
            # Confirm execution
            reply = QMessageBox.question(
                self, "Execute Path",
                f"Send {num_waypoints} path waypoints to CRI at {velocity}% velocity?\n\n"
                "Make sure the start position is safe!",
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
            self.execution_stop_event.clear()
            self._start_execution_metrics()

            print(f"Starting CRI path execution with {num_waypoints} waypoints...")
            self.execution_thread = threading.Thread(
                target=self._execute_path_worker,
                args=(self.path_executor.path_data.copy(), velocity),
                daemon=True,
            )
            self.execution_thread.start()
            return
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error executing path:\n{e}")
            self.btn_move_to_start.setEnabled(True)
            self.btn_execute_path.setEnabled(True)

    def _execute_path_worker(self, path_data, velocity):
        self.log_start_time = time.time()
        self.is_recording_diagnostics = True
        self.realtime_data_log.append(
            self._build_diagnostics_record(
                0.0,
                self.manager.get_joint_angles_list(),
                self.manager.get_joint_currents_list(),
            )
        )

        execution_start_time = time.monotonic()
        try:
            success, error_message = self.manager.execute_path(
                path_data,
                velocity,
                stop_event=self.execution_stop_event,
            )
        except Exception as e:
            # Defense in depth: never let an unexpected exception kill this thread silently,
            # since that would leave the GUI's buttons disabled forever.
            error_message = f"Unexpected error during execution: {e}"

        self.execution_elapsed = time.monotonic() - execution_start_time
        self.execution_complete = True
        self.execution_error = error_message

    def system_loop(self):
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
                self.lbl_joint_currents[i].setText(f"J{i+1}: {current_val:.3f} mA")

            total_current = self.manager.get_total_current()
            self.lbl_total_current.setText(f"Total Joint Current: {total_current:.3f} mA")

            self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
            power_w = self.manager.get_supply_power_W()
            self.lbl_power.setText(
                f"Power: {power_w:.2f} W (24 V × {total_current:.0f} mA / 1000)"
            )
            self._update_execution_metrics(time.monotonic(), power_w)

            self._run_digital_twin_analytics(real_joints, real_currents)

            if self.is_recording_diagnostics:
                record_time = time.time() - self.log_start_time
                self.realtime_data_log.append(
                    self._build_diagnostics_record(record_time, real_joints, real_currents)
                )

            if self.execution_complete and not self.execution_saved:
                self.is_recording_diagnostics = False
                self.execution_saved = True
                if self.execution_error:
                    self.lbl_execution.setText("Execution: Error")
                    QMessageBox.warning(self, "Execution Error", self.execution_error)
                else:
                    self.lbl_execution.setText("Execution: Complete")
                    QMessageBox.information(
                        self,
                        "Complete",
                        f"Path execution completed!\nActual execution time: {self.execution_elapsed:.2f} seconds",
                    )
                    self.prompt_save_diagnostics_csv()
                self.execution_complete = False
                self.execution_error = None
                self.execution_session_active = False
                self.realtime_trail_active = False
                if self.manager.robot.connected and self.total_frames > 0:
                    self.btn_move_to_start.setEnabled(True)
                    self.btn_execute_path.setEnabled(True)

            rad_real = np.radians(real_joints)
            real_points = self.kinematics.get_stick_points(rad_real)
            tip_position = real_points[-1]
            for label, axis, coordinate in zip(self.lbl_position, ("X", "Y", "Z"), tip_position):
                label.setText(f"{axis}: {coordinate:.1f} mm")
            if self.realtime_trail_active:
                self.real_tip_trail.append(tuple(real_points[-1]))
            self.real_view.update_view(real_points, tip_trail=self.real_tip_trail)

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
            self.manager.stop_jog()
            return
            
        speed = 15