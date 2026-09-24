"""Digital Twin analytics layer for the 6-DOF igus ReBeL supervisory system.

This module is a pure analytics/consumer layer: it never issues CRI motor
commands. It ingests telemetry (joint currents, velocities, timestamps)
produced by the CRI stream and derives:

1. AdaptiveBaselineRLS  - forgetting-factor RLS estimator of the expected
   "no-collision" motor current baseline per joint.
2. CUSUMDetector        - sequential change detector on the RLS residual,
   used for sensorless collision diagnosis.
3. PayloadEstimator     - converts sustained shifts in the estimated gravity
   parameter into an estimated payload mass.
4. TelemetryLogger      - records synchronization latency and per-cycle
   analytics results to a structured CSV file for experimentation.
"""

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


class AdaptiveBaselineRLS:
    """Forgetting-Factor Recursive Least Squares (FF-RLS) baseline estimator.

    Models the expected motor current as a linear combination of joint
    velocity (viscous friction), direction of motion (Coulomb friction),
    and an estimated gravity/load torque term:

        y_hat(t) = phi(t)^T * theta_hat(t)
        phi(t)   = [q_dot(t), sign(q_dot(t)), gravity_term(t)]^T

    The forgetting factor lambda (< 1) discounts old data so the estimator
    tracks slowly varying dynamics (e.g. temperature-dependent friction).
    """

    def __init__(
        self,
        n_params: int = 3,
        forgetting_factor: float = 0.98,
        initial_covariance_scale: float = 1000.0,
        initial_theta: Optional[np.ndarray] = None,
    ) -> None:
        if not (0.0 < forgetting_factor <= 1.0):
            raise ValueError("forgetting_factor (lambda) must be in (0, 1].")

        self.n_params = n_params
        self.lam = forgetting_factor
        self.P = np.eye(n_params) * initial_covariance_scale
        self.theta_hat = (
            np.array(initial_theta, dtype=float)
            if initial_theta is not None
            else np.zeros(n_params)
        )

    def update(
        self, y_measured: float, q_dot: float, gravity_term: float
    ) -> tuple[float, float, np.ndarray]:
        """Runs one FF-RLS recursion step.

        Args:
            y_measured: Measured motor current y(t) for the joint.
            q_dot: Joint velocity.
            gravity_term: Estimated gravity/kinematic torque contribution.

        Returns:
            (y_hat, residual, theta_hat) for this time step.
        """
        phi = np.array([q_dot, np.sign(q_dot), gravity_term], dtype=float)

        # Prediction using previous parameter estimate.
        y_hat = float(phi @ self.theta_hat)
        residual = y_measured - y_hat

        # Gain vector: K(t) = P(t-1) phi / (lambda + phi^T P(t-1) phi)
        p_phi = self.P @ phi
        denom = self.lam + float(phi @ p_phi)
        K = p_phi / denom

        # Parameter update.
        self.theta_hat = self.theta_hat + K * residual

        # Covariance update: P(t) = (P(t-1) - K phi^T P(t-1)) / lambda
        self.P = (self.P - np.outer(K, p_phi)) / self.lam

        return y_hat, residual, self.theta_hat.copy()


class CUSUMDetector:
    """Two-sided CUSUM sequential change detector for collision diagnosis.

    Accumulates deviations of the RLS residual from an expected mean of
    zero. A sustained positive or negative bias (beyond the allowance `k`)
    that exceeds threshold `h` signals a probable collision/contact event.
    """

    def __init__(self, k: float = 0.5, h: float = 5.0) -> None:
        self.k = k
        self.h = h
        self.s_plus = 0.0
        self.s_minus = 0.0

    def update(self, residual: float) -> tuple[bool, float]:
        """Feeds one residual sample into the CUSUM accumulators.

        Args:
            residual: FF-RLS residual e(t) for the current cycle.

        Returns:
            (collision_detected, cusum_score) where cusum_score is the
            larger of the two accumulators (for logging/plotting).
        """
        self.s_plus = max(0.0, self.s_plus + residual - self.k)
        self.s_minus = max(0.0, self.s_minus - residual - self.k)

        collision_detected = self.s_plus > self.h or self.s_minus > self.h
        cusum_score = max(self.s_plus, self.s_minus)

        # Reset accumulators after a detection to re-arm the detector.
        if collision_detected:
            self.s_plus = 0.0
            self.s_minus = 0.0

        return collision_detected, cusum_score


class PayloadEstimator:
    """Sensorless payload mass estimator from FF-RLS gravity parameter shifts.

    Compares the current gravity-term coefficient in theta_hat against an
    empty-load baseline (theta_empty). A low-pass filter isolates sustained
    shifts from transient noise/spikes before converting to mass.
    """

    GRAVITY_PARAM_INDEX = 2  # index of the gravity term inside theta_hat/phi

    def __init__(
        self,
        theta_empty: np.ndarray,
        mass_sensitivity_gain: float,
        filter_alpha: float = 0.05,
        attach_threshold_g: float = 50.0,
    ) -> None:
        """
        Args:
            theta_empty: Baseline theta_hat parameter vector at 0g load.
            mass_sensitivity_gain: K_mass, converts delta_theta[gravity] to
                grams (grams per unit of gravity-parameter change).
            filter_alpha: Low-pass filter coefficient (0-1) applied to the
                gravity parameter to suppress transient spikes.
            attach_threshold_g: Minimum estimated mass to classify as
                "PAYLOAD_ATTACHED" rather than "EMPTY".
        """
        self.theta_empty = np.array(theta_empty, dtype=float)
        self.k_mass = mass_sensitivity_gain
        self.alpha = filter_alpha
        self.attach_threshold_g = attach_threshold_g
        self._filtered_gravity_param: Optional[float] = None

    def update(self, theta_hat: np.ndarray) -> tuple[float, str]:
        """Estimates payload mass from the latest FF-RLS parameter vector.

        Args:
            theta_hat: Current FF-RLS parameter estimate theta_hat(t).

        Returns:
            (estimated_mass_g, status) where status is "EMPTY" or
            "PAYLOAD_ATTACHED".
        """
        raw_gravity_param = float(theta_hat[self.GRAVITY_PARAM_INDEX])

        # Exponential moving average filters out transient spikes.
        if self._filtered_gravity_param is None:
            self._filtered_gravity_param = raw_gravity_param
        else:
            self._filtered_gravity_param = (
                self.alpha * raw_gravity_param
                + (1.0 - self.alpha) * self._filtered_gravity_param
            )

        delta_theta = (
            self._filtered_gravity_param
            - self.theta_empty[self.GRAVITY_PARAM_INDEX]
        )
        estimated_mass_g = delta_theta * self.k_mass

        status = (
            "PAYLOAD_ATTACHED"
            if estimated_mass_g >= self.attach_threshold_g
            else "EMPTY"
        )
        return estimated_mass_g, status


@dataclass
class TelemetryRecord:
    """One structured row of the telemetry/analytics log."""

    t_gui: float
    t_virtual: float
    t_physical: float
    sync_delay_s: float
    joint_id: str
    raw_current: float
    rls_residual: float
    cusum_score: float
    collision_detected: bool
    estimated_mass_g: float
    payload_status: str
    timestamp: float | None = None
    command_id: str | None = None
    joint_index: int | None = None
    cri_rtt_ms: float | None = None
    estimated_one_way_ms: float | None = None
    rtt_jitter_ms: float | None = None
    gui_to_motion_ms: float | None = None
    tx_to_motion_ms: float | None = None
    rtt_mean_ms: float | None = None
    rtt_std_ms: float | None = None
    rtt_p95_ms: float | None = None
    rtt_p99_ms: float | None = None


class TelemetryLogger:
    """Logs analytics rows and scientifically separated latency metrics."""

    FIELDNAMES = list(TelemetryRecord.__dataclass_fields__.keys())

    def __init__(self, csv_path: str, flush_every: int = 1) -> None:
        self.csv_path = csv_path
        self.flush_every = flush_every
        self._rows_since_flush = 0

        is_new_file = not os.path.exists(csv_path)
        fieldnames = self.FIELDNAMES
        if not is_new_file:
            with open(csv_path, mode="r", newline="", encoding="utf-8") as existing:
                reader = csv.DictReader(existing)
                existing_fieldnames = reader.fieldnames or []
                missing_fields = [
                    field for field in self.FIELDNAMES if field not in existing_fieldnames
                ]
                if missing_fields:
                    rows = list(reader)
                    fieldnames = [*existing_fieldnames, *missing_fields]
                    with open(csv_path, mode="w", newline="", encoding="utf-8") as migrated:
                        writer = csv.DictWriter(migrated, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerows(rows)
                else:
                    fieldnames = existing_fieldnames
        self._file = open(csv_path, mode="a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file, fieldnames=fieldnames, extrasaction="ignore"
        )
        if is_new_file:
            self._writer.writeheader()
            self._file.flush()

    @staticmethod
    def compute_sync_delay(t_gui: float, t_physical: float) -> float:
        """Legacy compatibility helper for old log consumers.

        This is local processing elapsed time, not CRI RTT or physical latency.
        """
        return t_physical - t_gui

    def log(self, record: TelemetryRecord) -> None:
        """Appends one telemetry record to the CSV file."""
        self._writer.writerow(vars(record))
        self._rows_since_flush += 1
        if self._rows_since_flush >= self.flush_every:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


def example_integration_loop() -> None:
    """Illustrates how an incoming CRI telemetry packet flows through the
    analytics pipeline for a single joint. Replace the synthetic packet
    generator with real callbacks from `CRIController` / `RebelManager`.
    """
    rls = AdaptiveBaselineRLS(forgetting_factor=0.98, initial_covariance_scale=1000.0)
    cusum = CUSUMDetector(k=0.5, h=5.0)
    payload_est = PayloadEstimator(
        theta_empty=np.array([0.0, 0.0, 0.0]), mass_sensitivity_gain=200.0
    )

    with TelemetryLogger("robot_diagnostics_analytics.csv") as logger:
        for _ in range(5):
            # --- 1. Incoming CRI telemetry packet (simulated here) ---
            t_gui = time.perf_counter()
            packet = {
                "joint_id": "A3",
                "q_dot": 12.5,          # deg/s
                "gravity_term": 3.2,     # Nm-equivalent estimate from kinematics
                "current": 0.85,         # measured motor current
                "t_virtual": t_gui + 0.002,
                "t_physical": t_gui + 0.015,
            }

            # --- 2. FF-RLS baseline prediction + residual ---
            y_hat, residual, theta_hat = rls.update(
                y_measured=packet["current"],
                q_dot=packet["q_dot"],
                gravity_term=packet["gravity_term"],
            )

            # --- 3. CUSUM collision detection on the residual ---
            collision_detected, cusum_score = cusum.update(residual)

            # --- 4. Sensorless payload estimation from theta_hat ---
            estimated_mass_g, payload_status = payload_est.update(theta_hat)

            # --- 5. Log analytics row; network/motion latency is supplied by
            # the CRI timing trackers in the live application. ---
            record = TelemetryRecord(
                t_gui=t_gui,
                t_virtual=packet["t_virtual"],
                t_physical=packet["t_physical"],
                sync_delay_s=TelemetryLogger.compute_sync_delay(
                    t_gui, packet["t_physical"]
                ),
                joint_id=packet["joint_id"],
                raw_current=packet["current"],
                rls_residual=residual,
                cusum_score=cusum_score,
                collision_detected=collision_detected,
                estimated_mass_g=estimated_mass_g,
                payload_status=payload_status,
            )
            logger.log(record)


if __name__ == "__main__":
    example_integration_loop()
