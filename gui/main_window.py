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

from core.trajectory import TrajectoryPlanner
from core.trajectory_executor import TrajectoryExecutor
from gui.stick_viewer import StickViewer
from core.kinematics import ReBelKinematics
from hardware.robot_manager import RebelManager

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
        
        self.planner = TrajectoryPlanner()
        self.trajectory_executor = TrajectoryExecutor()
        self.sim_path_full = []
        self.sim_tip_trail = []
        self.current_frame = 0
        self.total_frames = 0
        self.is_playing = False
        self.current_sim_angles = [0.0] * 6 
        self.pre_sim_angles = [0.0] * 6
        self.current_trajectory_duration = 3.0
        self.current_algorithm = "Cubic Polynomial"

        self.setWindowTitle("Igus ReBeL iRC Clone & Digital Twin")
        self.resize(1300, 850)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.pressed_keys = set()
        
        self.manager = RebelManager()
        self.kinematics = ReBelKinematics()
        self.realtime_data_log = []
        self.log_start_time = time.time()
        self.is_recording_diagnostics = False
        self.recording_started = False
        self.execution_complete = False
        self.execution_saved = False
        self.execution_error = None
        self.execution_elapsed = 0.0
        
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

        # LEFT PANEL: Tabbed control panel keeps each screen focused instead of
        # one long scrolling column of widgets (Control / Trajectory / Export).
        left_tabs = QTabWidget()
        left_tabs.setMaximumWidth(360)

        control_tab = QWidget()
        left_panel = QVBoxLayout(control_tab)
        left_panel.setAlignment(Qt.AlignmentFlag.AlignTop)
        left_panel.setContentsMargins(6, 6, 6, 6)
        left_panel.setSpacing(10)

        # Connection Group
        flow_group = QGroupBox("Connection")
        flow_layout = QVBoxLayout()
        flow_layout.setSpacing(6)
        
        self.status_label = QLabel("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red; font-size: 14px;")
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
        flow_layout.addWidget(self.ip_input)
        flow_layout.addLayout(conn_layout)
        flow_layout.addLayout(en_res_layout)
        flow_layout.addLayout(home_layout)
        flow_group.setLayout(flow_layout)
        left_panel.addWidget(flow_group)

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
            btn_neg.pressed.connect(lambda a=axis: self.manager.start_jog(a, -15.0))
            btn_neg.released.connect(self.manager.stop_jog)
            btn_pos.pressed.connect(lambda a=axis: self.manager.start_jog(a, 15.0))
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
        left_panel.addWidget(jog_group)

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
        left_panel.addWidget(sensor_group)

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
        left_panel.addWidget(diag_group)
        left_panel.addStretch()

        control_scroll = QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setWidget(control_tab)
        control_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(control_scroll, "Control")

        # ---- Trajectory tab: algorithm, constraints, points, playback ----
        traj_tab = QWidget()
        sim_layout = QVBoxLayout(traj_tab)
        sim_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        sim_layout.setContentsMargins(6, 6, 6, 6)
        sim_layout.setSpacing(8)

        # Algorithm selection
        alg_layout = QHBoxLayout()
        alg_layout.addWidget(QLabel("Algorithm:"))
        self.combo_algorithm = QComboBox()
        self.combo_algorithm.addItems(["Cubic Polynomial (Smooth)", "Linear LERP", "Quintic Polynomial"])
        alg_layout.addWidget(self.combo_algorithm)
        sim_layout.addLayout(alg_layout)

        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("Path:") )
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

        # Duration
        dur_layout = QHBoxLayout()
        dur_layout.addWidget(QLabel("Duration (s):"))
        self.spin_duration = QDoubleSpinBox()
        self.spin_duration.setRange(0.5, 20.0)
        self.spin_duration.setValue(3.0)
        dur_layout.addWidget(self.spin_duration)
        sim_layout.addLayout(dur_layout)

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
        self.chk_real_time_trail.setChecked(False)
        opts_layout.addWidget(self.chk_real_time_trail)

        sim_layout.addLayout(opts_layout)

        # Close out the Trajectory tab
        traj_scroll = QScrollArea()
        traj_scroll.setWidgetResizable(True)
        traj_scroll.setWidget(traj_tab)
        traj_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(traj_scroll, "Trajectory")

        # ---- Export tab: velocity, export actions, robot execution ----
        export_tab = QWidget()
        export_layout = QVBoxLayout(export_tab)
        export_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        export_layout.setContentsMargins(6, 6, 6, 6)
        export_layout.setSpacing(8)

        # Velocity control for robot export
        vel_layout = QHBoxLayout()
        vel_layout.setSpacing(3)
        vel_layout.addWidget(QLabel("Velocity (%):", ), 0)
        self.spin_robot_velocity = QDoubleSpinBox()
        self.spin_robot_velocity.setRange(1.0, 100.0)
        self.spin_robot_velocity.setValue(50.0)
        self.spin_robot_velocity.setMaximumWidth(60)
        vel_layout.addWidget(self.spin_robot_velocity, 0)
        vel_layout.addStretch()
        export_layout.addLayout(vel_layout)
        
        # Export buttons in a 2x2 grid for compact layout
        button_grid = QGridLayout()
        button_grid.setSpacing(2)
        button_grid.setContentsMargins(0, 0, 0, 0)
        
        # Button 1: Export CSV (Cartesian)
        self.btn_export_csv = QPushButton("📊 Cartesian\nCSV")
        self.btn_export_csv.setStyleSheet("background-color: #3498db; color: white; padding: 4px; font-weight: bold; font-size: 10px;")
        self.btn_export_csv.setEnabled(False)
        self.btn_export_csv.setMinimumHeight(40)
        self.btn_export_csv.clicked.connect(self.export_sim_path)
        button_grid.addWidget(self.btn_export_csv, 0, 0)
        
        # Button 2: Export Joint Angles
        self.btn_export_robot_angles = QPushButton("🤖 Joint\nAngles")
        self.btn_export_robot_angles.setStyleSheet("background-color: #e74c3c; color: white; padding: 4px; font-weight: bold; font-size: 10px;")
        self.btn_export_robot_angles.setEnabled(False)
        self.btn_export_robot_angles.setMinimumHeight(40)
        self.btn_export_robot_angles.clicked.connect(self.export_robot_joint_angles)
        button_grid.addWidget(self.btn_export_robot_angles, 0, 1)
        
        # Button 3: Export CRI Commands
        self.btn_export_robot_commands = QPushButton("⚙️ CRI\nCommands")
        self.btn_export_robot_commands.setStyleSheet("background-color: #f39c12; color: white; padding: 4px; font-weight: bold; font-size: 10px;")
        self.btn_export_robot_commands.setEnabled(False)
        self.btn_export_robot_commands.setMinimumHeight(40)
        self.btn_export_robot_commands.clicked.connect(self.export_robot_commands)
        button_grid.addWidget(self.btn_export_robot_commands, 1, 0)
        
        # Button 4: Generate Script
        self.btn_generate_script = QPushButton("Generate Script")
        self.btn_generate_script.setStyleSheet("background-color: #9b59b6; color: white; padding: 4px; font-weight: bold; font-size: 10px;")
        self.btn_generate_script.setEnabled(False)
        self.btn_generate_script.setMinimumHeight(40)
        self.btn_generate_script.clicked.connect(self.generate_execution_script)
        button_grid.addWidget(self.btn_generate_script, 1, 1)
        
        export_layout.addLayout(button_grid)

        # Export real-time diagnostics data
        export_diag_layout = QHBoxLayout()
        self.btn_export_voltage_csv = QPushButton("📥 Export Diagnostics CSV")
        self.btn_export_voltage_csv.setStyleSheet("background-color: #8e44ad; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_export_voltage_csv.clicked.connect(self.export_diagnostics_csv)
        export_diag_layout.addWidget(self.btn_export_voltage_csv)
        export_layout.addLayout(export_diag_layout)
        
        # Robot control buttons (Move to Start & Execute)
        robot_control_layout = QHBoxLayout()
        robot_control_layout.setSpacing(2)
        
        # Button: Move to Start Position
        self.btn_move_to_start = QPushButton("🎯 Move to Start")
        self.btn_move_to_start.setStyleSheet("background-color: #16a085; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_move_to_start.setEnabled(False)
        self.btn_move_to_start.clicked.connect(self.move_to_start)
        robot_control_layout.addWidget(self.btn_move_to_start)
        
        # Button: Execute Trajectory
        self.btn_execute_trajectory = QPushButton("▶️ Execute Trajectory")
        self.btn_execute_trajectory.setStyleSheet("background-color: #c0392b; color: white; padding: 5px; font-weight: bold; font-size: 10px;")
        self.btn_execute_trajectory.setEnabled(False)
        self.btn_execute_trajectory.clicked.connect(self.execute_trajectory)
        robot_control_layout.addWidget(self.btn_execute_trajectory)
        
        export_layout.addLayout(robot_control_layout)
        export_layout.addStretch()

        export_scroll = QScrollArea()
        export_scroll.setWidgetResizable(True)
        export_scroll.setWidget(export_tab)
        export_scroll.setStyleSheet("QScrollArea { border: none; }")
        left_tabs.addTab(export_scroll, "Export")

        # ==========================================
        # RIGHT PANEL: Tabbed CAD Views (Simulation + Real-time)
        # ==========================================
        tab_widget = QTabWidget()
        tab_widget.setMinimumWidth(620)

        # Simulation View Tab
        sim_tab = QWidget()
        sim_tab_layout = QVBoxLayout(sim_tab)
        sim_tab_layout.setContentsMargins(6, 6, 6, 6)
        self.sim_view = StickViewer()
        sim_tab_layout.addWidget(self.sim_view)
        tab_widget.addTab(sim_tab, "Simulation View")

        # Real-time View Tab
        real_tab = QWidget()
        real_tab_layout = QVBoxLayout(real_tab)
        real_tab_layout.setContentsMargins(6, 6, 6, 6)
        self.real_view = StickViewer()
        real_tab_layout.addWidget(self.real_view)
        tab_widget.addTab(real_tab, "Real-time View")

        main_layout.addWidget(left_tabs)
        main_layout.addWidget(tab_widget, 1)



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

    def trigger_simulation(self):
        start_xyz = [box.value() for box in self.start_inputs]
        target_xyz = [box.value() for box in self.target_inputs]
        duration = self.spin_duration.value()
        
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
        
        algo_choice = self.combo_algorithm.currentText()
        self.current_algorithm = algo_choice  # Store for export
        self.current_trajectory_duration = duration  # Store for export
        
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
                temp_path = self.planner.generate_cartesian_line(
                    start_xyz, target_xyz, duration, solve_waypoint
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

            if "Cubic" in algo_choice:
                temp_path = self.planner.generate_cubic_path(start_angles, target_angles, duration)
            elif "Linear" in algo_choice and hasattr(self.planner, "generate_linear_path"):
                temp_path = self.planner.generate_linear_path(start_angles, target_angles, duration)
            elif "Quintic" in algo_choice and hasattr(self.planner, "generate_quintic_path"):
                temp_path = self.planner.generate_quintic_path(start_angles, target_angles, duration)
            else:
                temp_path = self.planner.generate_cubic_path(start_angles, target_angles, duration)

        is_safe, error_msg = self.planner.validate_trajectory(temp_path)
        if not is_safe:
            QMessageBox.warning(self, "Hardware Limits Exceeded", 
                                f"⚠️ VELOCITY LIMIT ERROR:\n\n{error_msg}\n\nPlease increase the duration.")
            return

        self.pre_sim_angles = self.current_sim_angles.copy()
        self.sim_path_full = list(temp_path)
        
        # Load trajectory into executor for robot control export
        try:
            self.trajectory_executor.load_from_simulation(
                self.sim_path_full, 
                duration, 
                algorithm=algo_choice
            )
        except Exception as e:
            print(f"Warning: Could not load trajectory into executor: {e}")
        
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
        self.btn_export_csv.setEnabled(self.total_frames > 0)
        self.btn_export_robot_angles.setEnabled(self.total_frames > 0)
        self.btn_export_robot_commands.setEnabled(self.total_frames > 0)
        self.btn_generate_script.setEnabled(self.total_frames > 0)
        self.btn_move_to_start.setEnabled(self.total_frames > 0 and self.manager.robot.connected)
        self.btn_execute_trajectory.setEnabled(self.total_frames > 0 and self.manager.robot.connected)

    def _show_ik_failure(self, stage):
        reason = self.kinematics.last_ik_error or "the requested point is outside the robot workspace"
        QMessageBox.critical(
            self,
            "Trajectory Cannot Be Planned",
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
        self.btn_export_csv.setEnabled(False)
        self.btn_export_robot_angles.setEnabled(False)
        self.btn_export_robot_commands.setEnabled(False)
        self.btn_generate_script.setEnabled(False)
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_trajectory.setEnabled(False)
        self.sim_view.clear_tip_trail()

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

    def export_sim_path(self):
        """Export the calculated full joint-angle path to a CSV file.

        Default location is current working directory; user can choose elsewhere.
        """
        path_data = getattr(self, 'sim_path_full', None)
        if not path_data:
            QMessageBox.information(self, "No Path", "There is no simulation pathway to export.")
            return

        default_path = os.path.join(os.getcwd(), "trajectory.csv")
        file_path, _ = QFileDialog.getSaveFileName(self, "Save Pathway CSV", default_path, "CSV Files (*.csv)")
        if not file_path:
            return

        try:
            with open(file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Header: Step, then J1x,J1y,J1z, J2x,... J6z, then Tip x,y,z
                header = ["Step"] + [f"J{j+1} {axis}" for j in range(6) for axis in ("X", "Y", "Z")] + ["Tip X", "Tip Y", "Tip Z"]
                writer.writerow(header)

                for idx, angles in enumerate(path_data):
                    # angles are in degrees in sim_path_full
                    rad = np.radians(angles)
                    joints_xyz = self.kinematics.get_joint_positions(rad)  # (6,3)
                    tip_xyz = self.kinematics.get_tip_position(rad)  # (3,)
                    flat = []
                    for j in range(6):
                        x, y, z = joints_xyz[j]
                        flat.extend([f"{float(x):.6f}", f"{float(y):.6f}", f"{float(z):.6f}"])
                    flat.extend([f"{float(tip_xyz[0]):.6f}", f"{float(tip_xyz[1]):.6f}", f"{float(tip_xyz[2]):.6f}"])
                    writer.writerow([idx] + flat)

            QMessageBox.information(self, "Export Successful", f"Pathway exported to:\n{file_path}")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save CSV:\n{e}")

    def export_robot_joint_angles(self):
        """Export trajectory as joint angles with timing information for robot control."""
        if not self.trajectory_executor.trajectory_data:
            QMessageBox.information(self, "No Path", "There is no simulation pathway to export.")
            return

        default_path = os.path.join(os.getcwd(), "trajectory_joint_angles.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Robot Joint Angles", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            velocity = self.spin_robot_velocity.value()
            self.trajectory_executor.export_to_csv_joint_angles(file_path, velocity_percent=velocity)
            QMessageBox.information(self, "Export Successful", 
                                  f"Robot joint angles exported to:\n{file_path}\n\n"
                                  f"Format: Step, Time (s), J1-J6 (degrees), Velocity (%)")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save CSV:\n{e}")

    def export_robot_commands(self):
        """Export trajectory as CRI robot commands ready for execution."""
        if not self.trajectory_executor.trajectory_data:
            QMessageBox.information(self, "No Path", "There is no simulation pathway to export.")
            return

        default_path = os.path.join(os.getcwd(), "trajectory_commands.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Robot Commands", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            velocity = self.spin_robot_velocity.value()
            self.trajectory_executor.export_to_csv_robot_commands(file_path, velocity_percent=velocity)
            QMessageBox.information(self, "Export Successful",
                                  f"Robot commands exported to:\n{file_path}\n\n"
                                  f"Format: Step, Time (s), CRI Command (ready to send to robot)")
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
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                header = ["Time (s)", "Supply Voltage (V)", "Power (W)"]
                header += [f"J{i+1} Angle (deg)" for i in range(6)]
                header += [f"J{j+1} {axis} (mm)" for j in range(6) for axis in ("X", "Y", "Z")]
                header += [f"J{i+1} Current (mA)" for i in range(6)]
                writer.writerow(header)
                for record in self.realtime_data_log:
                    writer.writerow(record)

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

    def prompt_save_diagnostics_csv(self):
        default_path = os.path.join(os.getcwd(), "robot_diagnostics.csv")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Diagnostics CSV", default_path, "CSV Files (*.csv)"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                header = [
                    "Time (s)",
                    "Supply Voltage (V)",
                    "Power (W)"
                ]
                header += [f"J{i+1} Angle (deg)" for i in range(6)]
                header += [f"J{j+1} {axis} (mm)" for j in range(6) for axis in ("X", "Y", "Z")]
                header += [f"J{i+1} Current (mA)" for i in range(6)]
                writer.writerow(header)
                for record in self.realtime_data_log:
                    writer.writerow(record)
            QMessageBox.information(self, "Export Successful", f"Diagnostics exported to:\n{file_path}")
        except Exception as e:
            QMessageBox.warning(self, "Export Failed", f"Could not save diagnostics CSV:\n{e}")

    def generate_execution_script(self):
        """Generate a Python script for executing the trajectory on the robot."""
        if not self.trajectory_executor.trajectory_data:
            QMessageBox.information(self, "No Path", "There is no simulation pathway to export.")
            return

        default_path = os.path.join(os.getcwd(), "execute_trajectory.py")
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Execution Script", default_path, "Python Files (*.py)"
        )
        if not file_path:
            return

        try:
            # Get robot IP from the input field (or use localhost as default)
            robot_ip = self.ip_input.text().strip() if hasattr(self, 'ip_input') else "192.168.0.1"
            velocity = self.spin_robot_velocity.value()
            
            self.trajectory_executor.generate_execution_script(
                file_path,
                robot_ip=robot_ip,
                robot_port=3920,
                velocity_percent=velocity
            )
            
            # Show summary report
            summary = self.trajectory_executor.generate_summary_report()
            QMessageBox.information(self, "Script Generated Successfully",
                                  f"Execution script saved to:\n{file_path}\n{summary}\n\n"
                                  f"To run the script:\n"
                                  f"python {os.path.basename(file_path)} [optional_robot_ip]")
        except Exception as e:
            QMessageBox.warning(self, "Script Generation Failed", f"Could not create script:\n{e}")

    def connect_robot(self):
        if self.manager.connect(self.ip_input.text().strip()):
            self.status_label.setText("Status: Connected 🟢")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.btn_connect.setEnabled(False)
            for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
                btn.setEnabled(True)
            # Enable robot control buttons if trajectory is loaded
            if self.total_frames > 0:
                self.btn_move_to_start.setEnabled(True)
                self.btn_execute_trajectory.setEnabled(True)
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
        self.status_label.setText("Status: Disconnected ❌")
        self.status_label.setStyleSheet("font-weight: bold; color: red;")
        self.btn_connect.setEnabled(True)
        for btn in [self.btn_disconnect, self.btn_reset, self.btn_enable, self.btn_home] + self.jog_buttons:
            btn.setEnabled(False)
        self.btn_move_to_start.setEnabled(False)
        self.btn_execute_trajectory.setEnabled(False)
        self.real_view.toggle_trail(False)
        self.lbl_supply_voltage.setText("Supply Voltage: 24 V")
        self.lbl_total_current.setText("Total Joint Current: -- mA")
        self.lbl_power.setText("Power: -- W")

    def _on_robot_status_update(self, state):
        # No direct UI updates here because this callback runs in the receive thread.
        # The main UI timer polls the latest robot state and updates labels safely.
        return

    def move_to_start(self):
        """Move the robot to the start position of the trajectory."""
        if not self.trajectory_executor.trajectory_data:
            QMessageBox.warning(self, "No Trajectory", "No trajectory loaded. Please calculate a path first.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return
        
        try:
            # Get the first waypoint (start position)
            start_angles, start_time = self.trajectory_executor.trajectory_data[0]
            start_angles = [float(a) for a in start_angles]
            velocity = self.spin_robot_velocity.value()
            
            self.btn_move_to_start.setEnabled(False)
            self.status_label.setText("Status: Sending move-to-start command...")

            success = self.manager.robot.move_joints(
                A1=start_angles[0], A2=start_angles[1], A3=start_angles[2],
                A4=start_angles[3], A5=start_angles[4], A6=start_angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=velocity,
                wait_move_finished=False  # Send command without blocking the GUI
            )

            if success:
                QMessageBox.information(self, "Moving to Start", 
                                      f"Robot move-to-start command sent.\n"
                                      f"J1={start_angles[0]:.1f}°, J2={start_angles[1]:.1f}°, J3={start_angles[2]:.1f}°\n"
                                      f"J4={start_angles[3]:.1f}°, J5={start_angles[4]:.1f}°, J6={start_angles[5]:.1f}°")
                self.status_label.setText("Status: Move to start command sent 🟡")
            else:
                QMessageBox.warning(self, "Failed", "Robot move-to-start command failed. Check the robot status.")
                self.status_label.setText("Status: Connected 🟢")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error moving robot to start position:\n{e}")
            self.status_label.setText("Status: Connected 🟢")
        finally:
            if self.manager.robot.connected and self.total_frames > 0:
                self.btn_move_to_start.setEnabled(True)

    def execute_trajectory(self):
        """Execute the loaded trajectory on the robot."""
        if not self.trajectory_executor.trajectory_data:
            QMessageBox.warning(self, "No Trajectory", "No trajectory loaded. Please calculate a path first.")
            return
        
        if not self.manager.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Robot is not connected. Please connect first.")
            return
        
        try:
            velocity = self.spin_robot_velocity.value()
            num_waypoints = len(self.trajectory_executor.trajectory_data)
            
            # Confirm execution
            reply = QMessageBox.question(
                self, "Execute Trajectory",
                f"Execute trajectory with {num_waypoints} waypoints at {velocity}% velocity?\n\n"
                "Make sure the start position is safe!",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.No:
                return
            
            # Disable buttons during execution
            self.btn_move_to_start.setEnabled(False)
            self.btn_execute_trajectory.setEnabled(False)

            self.realtime_data_log = []
            self.is_recording_diagnostics = False
            self.recording_started = True
            self.execution_complete = False
            self.execution_saved = False
            self.execution_error = None
            self.execution_elapsed = 0.0

            print(f"Starting trajectory execution with {num_waypoints} waypoints...")
            self.execution_thread = threading.Thread(
                target=self._execute_trajectory_worker,
                args=(self.trajectory_executor.trajectory_data.copy(), velocity),
                daemon=True,
            )
            self.execution_thread.start()
            return
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error executing trajectory:\n{e}")
            self.btn_move_to_start.setEnabled(True)
            self.btn_execute_trajectory.setEnabled(True)

    def _execute_trajectory_worker(self, trajectory_data, velocity):
        self.log_start_time = time.time()
        self.is_recording_diagnostics = True
        self.realtime_data_log.append(
            self._build_diagnostics_record(
                0.0,
                self.manager.get_joint_angles_list(),
                self.manager.get_joint_currents_list(),
            )
        )

        execution_start_time = time.time()
        error_message = None
        for idx, (angles, timestamp) in enumerate(trajectory_data):
            if not self.manager.robot.connected:
                error_message = "Robot disconnected during execution!"
                break

            # Pace commands based on trajectory time stamps so the robot receives
            # updates at the intended cadence instead of waiting for each small move.
            elapsed = time.time() - execution_start_time
            sleep_time = timestamp - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            is_last_waypoint = idx == len(trajectory_data) - 1
            success = self.manager.robot.move_joints(
                A1=angles[0], A2=angles[1], A3=angles[2],
                A4=angles[3], A5=angles[4], A6=angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=velocity,
                wait_move_finished=is_last_waypoint,
                move_finished_timeout=60.0 if is_last_waypoint else None,
            )
            if not success:
                error_message = f"Failed to execute waypoint {idx}."
                break

        self.execution_elapsed = time.time() - execution_start_time
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

            if self.is_recording_diagnostics:
                record_time = time.time() - self.log_start_time
                self.realtime_data_log.append(
                    self._build_diagnostics_record(record_time, real_joints, real_currents)
                )

            if self.execution_complete and not self.execution_saved:
                self.is_recording_diagnostics = False
                self.execution_saved = True
                if self.execution_error:
                    QMessageBox.warning(self, "Execution Error", self.execution_error)
                else:
                    QMessageBox.information(
                        self,
                        "Complete",
                        f"Trajectory execution completed!\nActual execution time: {self.execution_elapsed:.2f} seconds",
                    )
                    self.prompt_save_diagnostics_csv()
                self.execution_complete = False
                self.execution_error = None
                if self.manager.robot.connected and self.total_frames > 0:
                    self.btn_move_to_start.setEnabled(True)
                    self.btn_execute_trajectory.setEnabled(True)

            rad_real = np.radians(real_joints)
            real_points = self.kinematics.get_stick_points(rad_real)
            self.real_view.update_view(real_points)

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