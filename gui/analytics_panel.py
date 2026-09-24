"""Live UI panel for the digital-twin analytics pipeline (FF-RLS baseline,
CUSUM collision detection, payload estimation, and telemetry logging).
"""

from __future__ import annotations

from collections import deque
from typing import Deque

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel,
    QComboBox, QPushButton, QCheckBox, QDoubleSpinBox,
)
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

HISTORY_LEN = 300  # rolling samples kept per joint (~10s at 33ms loop rate)


class AnalyticsPanel(QWidget):
    """Displays per-joint current vs. FF-RLS baseline, CUSUM collision
    status, and sensorless payload estimation results."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Rolling history buffers, one deque per joint per series.
        self._t_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._current_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._baseline_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._residual_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self.selected_joint = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- Joint selector ---
        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel("Joint:"))
        self.joint_combo = QComboBox()
        self.joint_combo.addItems([f"J{i+1}" for i in range(6)])
        self.joint_combo.currentIndexChanged.connect(self._on_joint_changed)
        selector_row.addWidget(self.joint_combo)
        selector_row.addStretch(1)
        self.chk_log_csv = QCheckBox("Log to CSV")
        selector_row.addWidget(self.chk_log_csv)
        self.btn_reset = QPushButton("Reset Analytics")
        selector_row.addWidget(self.btn_reset)
        root.addLayout(selector_row)

        # --- Current vs. FF-RLS baseline graph ---
        self.fig = Figure(facecolor="#ffffff", figsize=(5, 3))
        self.canvas = FigureCanvas(self.fig)
        self.ax_current = self.fig.add_subplot(211)
        self.ax_residual = self.fig.add_subplot(212, sharex=self.ax_current)
        self.fig.subplots_adjust(left=0.09, right=0.98, top=0.95, bottom=0.12, hspace=0.35)

        self.ax_current.set_ylabel("Current (mA)")
        (self.line_current,) = self.ax_current.plot([], [], color="#2980b9", label="Measured")
        (self.line_baseline,) = self.ax_current.plot([], [], color="#e67e22", linestyle="--", label="FF-RLS baseline")
        self.ax_current.legend(loc="upper right", fontsize=8)
        self.ax_current.grid(True, alpha=0.3)

        self.ax_residual.set_ylabel("Residual")
        self.ax_residual.set_xlabel("Time (s)")
        (self.line_residual,) = self.ax_residual.plot([], [], color="#c0392b", label="e(t)")
        self.ax_residual.axhline(0.0, color="#888888", linewidth=0.8)
        self.ax_residual.grid(True, alpha=0.3)

        root.addWidget(self.canvas, 1)

        # --- Status readouts ---
        status_group = QGroupBox("Collision & Payload Diagnostics")
        grid = QGridLayout(status_group)

        self.lbl_cusum_score = QLabel("CUSUM Score: --")
        self.lbl_collision = QLabel("Collision: NONE")
        self.lbl_collision.setStyleSheet("font-weight: bold; color: #2ecc71;")
        self.lbl_payload_mass = QLabel("Estimated Payload: -- g")
        self.lbl_payload_status = QLabel("Payload Status: --")
        self.lbl_sync_delay = QLabel("Sync Delay: -- ms")

        grid.addWidget(self.lbl_cusum_score, 0, 0)
        grid.addWidget(self.lbl_collision, 0, 1)
        grid.addWidget(self.lbl_payload_mass, 1, 0)
        grid.addWidget(self.lbl_payload_status, 1, 1)
        grid.addWidget(self.lbl_sync_delay, 2, 0)

        root.addWidget(status_group)

        # --- Algorithm tuning (bound to config.yaml via settings.py) ---
        tuning_group = QGroupBox("Algorithm Tuning (config.yaml)")
        tuning_grid = QGridLayout(tuning_group)

        def _make_spinbox(minimum, maximum, decimals, step):
            sb = QDoubleSpinBox()
            sb.setRange(minimum, maximum)
            sb.setDecimals(decimals)
            sb.setSingleStep(step)
            return sb

        self.spin_lambda = _make_spinbox(0.80, 1.00, 3, 0.001)
        self.spin_p0 = _make_spinbox(1.0, 1_000_000.0, 1, 10.0)
        self.spin_cusum_k = _make_spinbox(0.0, 1000.0, 3, 0.1)
        self.spin_cusum_h = _make_spinbox(0.0, 10000.0, 2, 0.5)
        self.spin_filter_alpha = _make_spinbox(0.0, 1.0, 3, 0.01)
        self.spin_mass_gain = _make_spinbox(0.0, 1_000_000.0, 1, 10.0)
        self.spin_attach_threshold = _make_spinbox(0.0, 100000.0, 1, 1.0)

        tuning_fields = [
            ("FF-RLS lambda (forgetting factor):", self.spin_lambda),
            ("FF-RLS initial covariance P0:", self.spin_p0),
            ("CUSUM drift k:", self.spin_cusum_k),
            ("CUSUM threshold h:", self.spin_cusum_h),
            ("Payload filter alpha:", self.spin_filter_alpha),
            ("Payload mass sensitivity gain (g/unit):", self.spin_mass_gain),
            ("Payload attach threshold (g):", self.spin_attach_threshold),
        ]
        for row, (label_text, spinbox) in enumerate(tuning_fields):
            tuning_grid.addWidget(QLabel(label_text), row, 0)
            tuning_grid.addWidget(spinbox, row, 1)

        button_row = QHBoxLayout()
        self.btn_apply_tuning = QPushButton("Apply Live")
        self.btn_save_tuning = QPushButton("Save to config.yaml")
        button_row.addWidget(self.btn_apply_tuning)
        button_row.addWidget(self.btn_save_tuning)
        tuning_grid.addLayout(button_row, len(tuning_fields), 0, 1, 2)

        self.lbl_tuning_status = QLabel("")
        tuning_grid.addWidget(self.lbl_tuning_status, len(tuning_fields) + 1, 0, 1, 2)

        root.addWidget(tuning_group)

    def load_tuning_values(self, settings) -> None:
        """Populates the tuning spin boxes from a settings.Settings object."""
        self.spin_lambda.setValue(settings.ff_rls.forgetting_factor)
        self.spin_p0.setValue(settings.ff_rls.initial_covariance)
        self.spin_cusum_k.setValue(settings.cusum.k)
        self.spin_cusum_h.setValue(settings.cusum.h)
        self.spin_filter_alpha.setValue(settings.payload.filter_alpha)
        self.spin_mass_gain.setValue(settings.payload.mass_sensitivity_gain)
        self.spin_attach_threshold.setValue(settings.payload.attach_threshold_g)
        self.lbl_tuning_status.setText("")

    def get_tuning_values(self) -> dict:
        """Reads the current tuning spin box values as a plain dict, shaped
        like config.yaml's ff_rls/cusum/payload sections."""
        return {
            "ff_rls": {
                "forgetting_factor": self.spin_lambda.value(),
                "initial_covariance": self.spin_p0.value(),
            },
            "cusum": {
                "k": self.spin_cusum_k.value(),
                "h": self.spin_cusum_h.value(),
            },
            "payload": {
                "filter_alpha": self.spin_filter_alpha.value(),
                "mass_sensitivity_gain": self.spin_mass_gain.value(),
                "attach_threshold_g": self.spin_attach_threshold.value(),
            },
        }

    def _on_joint_changed(self, index: int) -> None:
        self.selected_joint = index
        self._redraw()

    def reset_history(self) -> None:
        for buf_list in (self._t_hist, self._current_hist, self._baseline_hist, self._residual_hist):
            for buf in buf_list:
                buf.clear()

    def add_sample(
        self,
        joint_index: int,
        t: float,
        current: float,
        baseline: float,
        residual: float,
    ) -> None:
        """Appends one analytics sample for the given joint and redraws
        the graph if that joint is currently selected."""
        self._t_hist[joint_index].append(t)
        self._current_hist[joint_index].append(current)
        self._baseline_hist[joint_index].append(baseline)
        self._residual_hist[joint_index].append(residual)

        if joint_index == self.selected_joint:
            self._redraw()

    def _redraw(self) -> None:
        j = self.selected_joint
        t = list(self._t_hist[j])
        self.line_current.set_data(t, list(self._current_hist[j]))
        self.line_baseline.set_data(t, list(self._baseline_hist[j]))
        self.line_residual.set_data(t, list(self._residual_hist[j]))

        for ax, lines in (
            (self.ax_current, (self.line_current, self.line_baseline)),
            (self.ax_residual, (self.line_residual,)),
        ):
            ax.relim()
            ax.autoscale_view()

        self.canvas.draw_idle()

    def update_status(
        self,
        cusum_score: float,
        collision_detected: bool,
        payload_mass_g: float,
        payload_status: str,
        sync_delay_ms: float,
    ) -> None:
        self.lbl_cusum_score.setText(f"CUSUM Score: {cusum_score:.3f}")
        if collision_detected:
            self.lbl_collision.setText("Collision: DETECTED")
            self.lbl_collision.setStyleSheet("font-weight: bold; color: #e74c3c;")
        else:
            self.lbl_collision.setText("Collision: NONE")
            self.lbl_collision.setStyleSheet("font-weight: bold; color: #2ecc71;")
        self.lbl_payload_mass.setText(f"Estimated Payload: {payload_mass_g:.1f} g")
        self.lbl_payload_status.setText(f"Payload Status: {payload_status}")
        self.lbl_sync_delay.setText(f"Sync Delay: {sync_delay_ms:.1f} ms")
