"""Event-based logger that writes exactly one CSV row per execution-latency trial."""

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass

from core.latency_tracking import CompletedTrial, MeasurementStatus


@dataclass(frozen=True)
class LatencyExperimentRecord:
    """One immutable row of the latency-experiment event log."""

    trial_id: str
    wall_clock_timestamp: float
    command_token: str
    cri_command_id: str
    command_type: str
    joint_index: int
    direction: float
    requested_speed_percent: float | None

    t_gui_perf: float
    t_tx_perf: float | None
    t_motion_perf: float | None

    gui_to_tx_ms: float | None
    tx_to_motion_ms: float | None
    gui_to_motion_ms: float | None

    cri_rtt_ms: float | None
    estimated_one_way_ms: float | None
    rtt_jitter_ms: float | None

    motion_threshold_deg_s: float | None
    stationary_velocity_std_deg_s: float | None
    confirmation_samples: int

    status: str


class LatencyExperimentLogger:
    """Appends one row per completed `CompletedTrial`; never logs a trial twice."""

    FIELDNAMES = list(LatencyExperimentRecord.__dataclass_fields__.keys())

    def __init__(self, csv_path: str = "latency_experiments.csv", flush_every: int = 1) -> None:
        self.csv_path = csv_path
        self.flush_every = flush_every
        self._rows_since_flush = 0
        self._logged_trial_ids: set[str] = set()

        is_new_file = not os.path.exists(csv_path)
        self._file = open(csv_path, mode="a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES, extrasaction="ignore")
        if is_new_file:
            self._writer.writeheader()
            self._file.flush()

    def log_trial(
        self,
        trial: CompletedTrial,
        cri_rtt_ms: float | None = None,
        estimated_one_way_ms: float | None = None,
        rtt_jitter_ms: float | None = None,
        command_token: str | None = None,
    ) -> bool:
        """Writes one row for `trial`. Returns False if `trial_id` was already logged."""
        if trial.trial_id in self._logged_trial_ids:
            return False
        self._logged_trial_ids.add(trial.trial_id)

        status = trial.status.value if isinstance(trial.status, MeasurementStatus) else str(trial.status)
        record = LatencyExperimentRecord(
            trial_id=trial.trial_id,
            wall_clock_timestamp=time.time(),
            command_token=command_token or trial.trial_id,
            cri_command_id=trial.command_id,
            command_type=trial.command_type,
            joint_index=trial.joint_index,
            direction=trial.direction,
            requested_speed_percent=trial.commanded_speed,
            t_gui_perf=trial.t_gui,
            t_tx_perf=trial.t_tx,
            t_motion_perf=trial.t_motion,
            gui_to_tx_ms=trial.gui_to_tx_ms,
            tx_to_motion_ms=trial.tx_to_motion_ms,
            gui_to_motion_ms=trial.gui_to_motion_ms,
            cri_rtt_ms=cri_rtt_ms,
            estimated_one_way_ms=estimated_one_way_ms,
            rtt_jitter_ms=rtt_jitter_ms,
            motion_threshold_deg_s=trial.motion_threshold_deg_s,
            stationary_velocity_std_deg_s=trial.stationary_velocity_std_deg_s,
            confirmation_samples=trial.confirmation_samples,
            status=status,
        )
        self._writer.writerow(vars(record))
        self._rows_since_flush += 1
        if self._rows_since_flush >= self.flush_every:
            self._file.flush()
            self._rows_since_flush = 0
        return True

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "LatencyExperimentLogger":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
