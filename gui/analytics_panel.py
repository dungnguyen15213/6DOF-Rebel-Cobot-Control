"""Analytics, tuning, and latency panels for the live robot UI."""

from __future__ import annotations

from collections import deque
from typing import Deque

from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

HISTORY_LEN = 120
WINDOW_SECONDS = 4.0


class AnalyticsPanel(QWidget):
    """Shows one selectable short-window analytics graph and diagnostics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._t_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._current_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._baseline_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self._residual_hist: list[Deque[float]] = [deque(maxlen=HISTORY_LEN) for _ in range(6)]
        self.selected_joint = 0
        self.selected_signal = "Current vs Baseline"

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        selector = QHBoxLayout()
        selector.addWidget(QLabel("Joint:"))
        self.joint_combo = QComboBox()
        self.joint_combo.addItems([f"J{i + 1}" for i in range(6)])
        self.joint_combo.currentIndexChanged.connect(self._on_joint_changed)
        selector.addWidget(self.joint_combo)
        selector.addWidget(QLabel("Graph:"))
        self.signal_combo = QComboBox()
        self.signal_combo.addItems(["Current vs Baseline", "Residual"])
        self.signal_combo.currentTextChanged.connect(self._on_signal_changed)
        selector.addWidget(self.signal_combo)
        selector.addStretch(1)
        self.chk_log_csv = QCheckBox("Log to CSV")
        selector.addWidget(self.chk_log_csv)
        self.btn_reset = QPushButton("Reset Analytics")
        selector.addWidget(self.btn_reset)
        root.addLayout(selector)

        self.fig = Figure(facecolor="#ffffff", figsize=(5, 2.0))
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setFixedHeight(190)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.09, right=0.98, top=0.90, bottom=0.22)
        (self.line_current,) = self.ax.plot([], [], color="#2980b9", label="Measured")
        (self.line_baseline,) = self.ax.plot([], [], color="#e67e22", linestyle="--", label="FF-RLS baseline")
        (self.line_residual,) = self.ax.plot([], [], color="#c0392b", label="Residual")
        self.ax.set_xlabel("Time (s)")
        self.ax.grid(True, alpha=0.3)
        self.line_residual.set_visible(False)
        self.ax.legend([self.line_current, self.line_baseline], ["Measured", "FF-RLS baseline"], loc="upper right", fontsize=8)

        status_group = QGroupBox("Collision & Payload Diagnostics")
        status_grid = QGridLayout(status_group)
        self.lbl_cusum_score = QLabel("CUSUM Score: --")
        self.lbl_collision = QLabel("Collision: NONE")
        self.lbl_collision.setStyleSheet("font-weight: bold; color: #2ecc71;")
        self.lbl_payload_mass = QLabel("Estimated Payload: -- g")
        self.lbl_payload_status = QLabel("Payload Status: --")
        self.lbl_processing_time = QLabel("Analytics Processing Time: -- ms")
        status_grid.addWidget(self.lbl_cusum_score, 0, 0)
        status_grid.addWidget(self.lbl_collision, 0, 1)
        status_grid.addWidget(self.lbl_payload_mass, 1, 0)
        status_grid.addWidget(self.lbl_payload_status, 1, 1)
        status_grid.addWidget(self.lbl_processing_time, 2, 0, 1, 2)

        content = QHBoxLayout()
        content.addWidget(self.canvas, 3)
        content.addWidget(status_group, 2)
        root.addLayout(content, 1)

    def _on_joint_changed(self, index: int) -> None:
        self.selected_joint = index
        self._redraw()

    def _on_signal_changed(self, signal: str) -> None:
        self.selected_signal = signal
        self._redraw()

    def reset_history(self) -> None:
        for buffers in (self._t_hist, self._current_hist, self._baseline_hist, self._residual_hist):
            for buffer in buffers:
                buffer.clear()
        self._redraw()

    def add_sample(self, joint_index: int, t: float, current: float, baseline: float, residual: float) -> None:
        self._t_hist[joint_index].append(t)
        self._current_hist[joint_index].append(current)
        self._baseline_hist[joint_index].append(baseline)
        self._residual_hist[joint_index].append(residual)
        if joint_index == self.selected_joint:
            self._redraw()

    def _redraw(self) -> None:
        joint = self.selected_joint
        times = list(self._t_hist[joint])
        showing_residual = self.selected_signal == "Residual"
        current = list(self._current_hist[joint])
        baseline = list(self._baseline_hist[joint])
        residual = list(self._residual_hist[joint])
        self.line_current.set_visible(not showing_residual)
        self.line_baseline.set_visible(not showing_residual)
        self.line_residual.set_visible(showing_residual)
        self.line_current.set_data(times if not showing_residual else [], current if not showing_residual else [])
        self.line_baseline.set_data(times if not showing_residual else [], baseline if not showing_residual else [])
        self.line_residual.set_data(times if showing_residual else [], residual if showing_residual else [])
        self.ax.set_ylabel("Residual" if showing_residual else "Current (mA)")
        self.ax.legend(
            [self.line_residual] if showing_residual else [self.line_current, self.line_baseline],
            ["Residual"] if showing_residual else ["Measured", "FF-RLS baseline"],
            loc="upper right",
            fontsize=8,
        )
        self.ax.relim()
        self.ax.autoscale_view()
        if times:
            self.ax.set_xlim(max(0.0, times[-1] - WINDOW_SECONDS), times[-1] + 0.05)
        self.canvas.draw_idle()

    def update_status(self, cusum_score: float, collision_detected: bool, payload_mass_g: float, payload_status: str, processing_time_ms: float | None) -> None:
        self.lbl_cusum_score.setText(f"CUSUM Score: {cusum_score:.3f}")
        self.lbl_collision.setText("Collision: DETECTED" if collision_detected else "Collision: NONE")
        self.lbl_collision.setStyleSheet("font-weight: bold; color: #e74c3c;" if collision_detected else "font-weight: bold; color: #2ecc71;")
        self.lbl_payload_mass.setText(f"Estimated Payload: {payload_mass_g:.1f} g")
        self.lbl_payload_status.setText(f"Payload Status: {payload_status}")
        value = "--" if processing_time_ms is None else f"{processing_time_ms:.1f} ms"
        self.lbl_processing_time.setText(f"Analytics Processing Time: {value}")


class AlgorithmTuningPanel(QWidget):
    """Dedicated tab for FF-RLS, CUSUM, and payload tuning controls."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        tuning_group = QGroupBox("Algorithm Tuning (config.yaml)")
        tuning_grid = QGridLayout(tuning_group)

        def spinbox(minimum, maximum, decimals, step):
            control = QDoubleSpinBox()
            control.setRange(minimum, maximum)
            control.setDecimals(decimals)
            control.setSingleStep(step)
            return control

        self.spin_lambda = spinbox(0.80, 1.00, 3, 0.001)
        self.spin_p0 = spinbox(1.0, 1_000_000.0, 1, 10.0)
        self.spin_cusum_k = spinbox(0.0, 1000.0, 3, 0.1)
        self.spin_cusum_h = spinbox(0.0, 10000.0, 2, 0.5)
        self.spin_filter_alpha = spinbox(0.0, 1.0, 3, 0.01)
        self.spin_mass_gain = spinbox(0.0, 1_000_000.0, 1, 10.0)
        self.spin_attach_threshold = spinbox(0.0, 100000.0, 1, 1.0)
        fields = [
            ("FF-RLS lambda (forgetting factor):", self.spin_lambda),
            ("FF-RLS initial covariance P0:", self.spin_p0),
            ("CUSUM drift k:", self.spin_cusum_k),
            ("CUSUM threshold h:", self.spin_cusum_h),
            ("Payload filter alpha:", self.spin_filter_alpha),
            ("Payload mass sensitivity gain (g/unit):", self.spin_mass_gain),
            ("Payload attach threshold (g):", self.spin_attach_threshold),
        ]
        for index, (label, control) in enumerate(fields):
            column = index // 4
            row = index % 4
            tuning_grid.addWidget(QLabel(label), row, column * 2)
            tuning_grid.addWidget(control, row, column * 2 + 1)
        tuning_grid.setColumnStretch(1, 1)
        tuning_grid.setColumnStretch(3, 1)
        buttons = QHBoxLayout()
        self.btn_apply_tuning = QPushButton("Apply Live")
        self.btn_save_tuning = QPushButton("Save to config.yaml")
        buttons.addWidget(self.btn_apply_tuning)
        buttons.addWidget(self.btn_save_tuning)
        tuning_grid.addLayout(buttons, 4, 0, 1, 4)
        self.lbl_tuning_status = QLabel("")
        tuning_grid.addWidget(self.lbl_tuning_status, 5, 0, 1, 4)
        layout.addWidget(tuning_group)
        layout.addStretch(1)

    def load_tuning_values(self, settings) -> None:
        self.spin_lambda.setValue(settings.ff_rls.forgetting_factor)
        self.spin_p0.setValue(settings.ff_rls.initial_covariance)
        self.spin_cusum_k.setValue(settings.cusum.k)
        self.spin_cusum_h.setValue(settings.cusum.h)
        self.spin_filter_alpha.setValue(settings.payload.filter_alpha)
        self.spin_mass_gain.setValue(settings.payload.mass_sensitivity_gain)
        self.spin_attach_threshold.setValue(settings.payload.attach_threshold_g)
        self.lbl_tuning_status.setText("")

    def get_tuning_values(self) -> dict:
        return {
            "ff_rls": {"forgetting_factor": self.spin_lambda.value(), "initial_covariance": self.spin_p0.value()},
            "cusum": {"k": self.spin_cusum_k.value(), "h": self.spin_cusum_h.value()},
            "payload": {
                "filter_alpha": self.spin_filter_alpha.value(),
                "mass_sensitivity_gain": self.spin_mass_gain.value(),
                "attach_threshold_g": self.spin_attach_threshold.value(),
            },
        }


class LatencyDiagnosticsPanel(QWidget):
    """Displays network RTT and telemetry-observed motion response metrics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        communication = QGroupBox("Communication Diagnostics")
        communication_grid = QGridLayout(communication)
        self.lbl_cri_rtt = QLabel("CRI ACK RTT: --")
        self.lbl_one_way = QLabel("Estimated One-Way Communication Latency: --")
        self.lbl_rtt_jitter = QLabel("Peak Consecutive RTT Jitter: --")
        self.lbl_rtt_mean = QLabel("RTT Mean: --")
        self.lbl_rtt_std = QLabel("RTT Std: --")
        communication_grid.addWidget(self.lbl_cri_rtt, 0, 0)
        communication_grid.addWidget(self.lbl_one_way, 0, 1)
        communication_grid.addWidget(self.lbl_rtt_jitter, 1, 0)
        communication_grid.addWidget(self.lbl_rtt_mean, 1, 1)
        communication_grid.addWidget(self.lbl_rtt_std, 2, 0)
        layout.addWidget(communication)
        motion = QGroupBox("Motion Diagnostics")
        motion_grid = QGridLayout(motion)
        self.lbl_gui_motion = QLabel("GUI -> Motion Response: --")
        self.lbl_tx_motion = QLabel("TX -> Motion Response: --")
        self.lbl_gui_tx = QLabel("GUI -> TX Delay: --")
        self.lbl_motion_status = QLabel("Status: --")
        motion_grid.addWidget(self.lbl_gui_motion, 0, 0)
        motion_grid.addWidget(self.lbl_tx_motion, 0, 1)
        motion_grid.addWidget(self.lbl_gui_tx, 1, 0)
        motion_grid.addWidget(self.lbl_motion_status, 1, 1)
        layout.addWidget(motion)
        layout.addStretch(1)

    def update_values(self, rtt_ms: float | None, one_way_ms: float | None, jitter_ms: float | None, gui_to_motion_ms: float | None, tx_to_motion_ms: float | None, gui_to_tx_ms: float | None = None, mean_ms: float | None = None, std_ms: float | None = None, status=None) -> None:
        def text(value: float | None) -> str:
            return "--" if value is None else f"{value:.1f} ms"
        self.lbl_cri_rtt.setText(f"CRI ACK RTT: {text(rtt_ms)}")
        self.lbl_one_way.setText(f"Estimated One-Way Communication Latency: {text(one_way_ms)}")
        self.lbl_rtt_jitter.setText(f"Peak Consecutive RTT Jitter: {text(jitter_ms)}")
        self.lbl_rtt_mean.setText(f"RTT Mean: {text(mean_ms)}")
        self.lbl_rtt_std.setText(f"RTT Std: {text(std_ms)}")
        self.lbl_gui_motion.setText(f"GUI -> Motion Response: {text(gui_to_motion_ms)}")
        self.lbl_tx_motion.setText(f"TX -> Motion Response: {text(tx_to_motion_ms)}")
        self.lbl_gui_tx.setText(f"GUI -> TX Delay: {text(gui_to_tx_ms)}")
        self.lbl_motion_status.setText(
            f"Status: {getattr(status, 'value', status) or '--'}"
        )
