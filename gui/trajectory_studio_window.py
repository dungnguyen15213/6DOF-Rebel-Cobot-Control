"""Trajectory Studio window: sequence builder on the left, simulation view on the right."""

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QFrame, QGridLayout, QLabel, QMainWindow, QScrollArea, QSplitter, QVBoxLayout, QWidget

READOUT_STYLE = "font-family: monospace; color: #00FF00; background-color: #111; padding: 3px;"


class TrajectoryStudioWindow(QMainWindow):
    """Separate window with the planner controls on the left and the simulation view on the right.

    The widgets are created and wired by the main window; the studio hosts them and owns the
    simulated-pose readout (Cartesian X/Y/Z and joint angles), which is independent of the
    live robot values shown in the main window.
    """

    def __init__(self, planner_widget: QWidget, simulation_widget: QWidget,
                 playback_widget: QWidget, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Trajectory Studio")
        self.resize(1200, 760)
        planner_widget.setMinimumWidth(460)

        self.lbl_position = [QLabel(f"{axis}: -- mm") for axis in ("X", "Y", "Z")]
        self.lbl_joints = [QLabel(f"J{i + 1}: -- °") for i in range(6)]
        readout = QGridLayout()
        readout.setHorizontalSpacing(4)
        readout.setVerticalSpacing(4)
        for col, label in enumerate(self.lbl_position):
            label.setStyleSheet(READOUT_STYLE)
            readout.addWidget(label, 0, col)
        for i, label in enumerate(self.lbl_joints):
            label.setStyleSheet(READOUT_STYLE)
            readout.addWidget(label, 1 + i // 3, i % 3)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        right_layout.addLayout(readout)
        right_layout.addWidget(simulation_widget, 1)
        right_layout.addWidget(playback_widget)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(planner_widget)
        scroll.setMinimumWidth(480)
        splitter.addWidget(scroll)
        splitter.addWidget(right)
        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([500, 700])
        self.setCentralWidget(splitter)

    def set_simulated_pose(self, joint_angles_deg, tip_xyz_mm) -> None:
        for label, axis, value in zip(self.lbl_position, ("X", "Y", "Z"), tip_xyz_mm):
            label.setText(f"{axis}: {value:.1f} mm")
        for i, angle in enumerate(joint_angles_deg):
            self.lbl_joints[i].setText(f"J{i + 1}: {angle:.1f}°")
