"""Analytics Engine for the Digital Twin supervisory layer.

Pure analytics/consumer classes: they never issue CRI motor commands. They
consume real-time current/velocity/gravity telemetry and produce the
adaptive baseline, collision flag, and payload estimate used for the
"Adaptive Context-Aware Anomaly Classification and Sensorless Payload
Identification" research pipeline.

All tunable parameters are sourced from config.yaml via settings.py so they
can be re-tuned in the lab without touching this file.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from settings import get_settings


class FF_RLS_Estimator:
    """Forgetting-Factor Recursive Least Squares adaptive baseline estimator.

    Models the expected motor current as a linear combination of the
    regressor vector:

        phi(t)   = [q_dot(t), sign(q_dot(t)), gravity_term(t)]^T
        y_hat(t) = phi(t)^T * theta_hat(t-1)

    Standard RLS-with-forgetting-factor recursion:
        e(t)        = y(t) - y_hat(t)                                (residual)
        K(t)        = P(t-1) phi(t) / (lambda + phi^T(t) P(t-1) phi(t))  (gain)
        theta_hat(t)= theta_hat(t-1) + K(t) e(t)                      (update)
        P(t)        = (P(t-1) - K(t) phi^T(t) P(t-1)) / lambda        (covariance)

    The forgetting factor lambda (<1) discounts old samples so the
    estimator tracks slowly varying dynamics (e.g. thermal drift).
    """

    def __init__(
        self,
        n_params: Optional[int] = None,
        forgetting_factor: Optional[float] = None,
        initial_covariance: Optional[float] = None,
        initial_theta: Optional[np.ndarray] = None,
    ) -> None:
        cfg = get_settings().ff_rls
        self.n_params = n_params if n_params is not None else cfg.n_params
        self.lam = forgetting_factor if forgetting_factor is not None else cfg.forgetting_factor
        if not (0.0 < self.lam <= 1.0):
            raise ValueError("forgetting_factor (lambda) must be in (0, 1].")

        p0 = initial_covariance if initial_covariance is not None else cfg.initial_covariance
        self.P = np.eye(self.n_params) * p0
        self.theta_hat = (
            np.array(initial_theta, dtype=float)
            if initial_theta is not None
            else np.zeros(self.n_params)
        )

    def update(
        self, current: float, velocity: float, gravity_term: float
    ) -> tuple[float, float, np.ndarray]:
        """Runs one FF-RLS recursion step.

        Args:
            current: Measured motor current y(t).
            velocity: Joint velocity q_dot(t).
            gravity_term: Estimated kinematic gravity/load torque term.

        Returns:
            (predicted_current, residual, theta_hat) for this time step.
        """
        phi = np.array([velocity, np.sign(velocity), gravity_term], dtype=float)

        # Prediction using the previous parameter estimate.
        predicted_current = float(phi @ self.theta_hat)
        residual = current - predicted_current

        # Kalman-style gain vector weighted by the forgetting factor.
        p_phi = self.P @ phi
        denom = self.lam + float(phi @ p_phi)
        K = p_phi / denom

        # Parameter (theta_hat) update driven by the residual.
        self.theta_hat = self.theta_hat + K * residual

        # Covariance update/decay; dividing by lambda inflates uncertainty
        # over time so old data is "forgotten".
        self.P = (self.P - np.outer(K, p_phi)) / self.lam

        return predicted_current, residual, self.theta_hat.copy()


class CUSUM_Detector:
    """Two-sided CUSUM sequential change detector for collision diagnosis.

    Accumulates deviations of the RLS residual from an expected mean of
    zero. A sustained positive or negative bias beyond the allowance `k`
    that exceeds threshold `h` sets collision_flag = True.
    """

    def __init__(self, k: Optional[float] = None, h: Optional[float] = None) -> None:
        cfg = get_settings().cusum
        self.k = k if k is not None else cfg.k
        self.h = h if h is not None else cfg.h
        self.s_plus = 0.0
        self.s_minus = 0.0

    def update(self, residual: float) -> tuple[bool, float]:
        """Feeds one residual sample into the CUSUM accumulators.

        S_plus(t)  = max(0, S_plus(t-1)  + e(t) - k)   detects positive drift
        S_minus(t) = max(0, S_minus(t-1) - e(t) - k)   detects negative drift

        Returns:
            (collision_flag, cusum_score) where cusum_score is the larger
            of the two accumulators (for logging/plotting).
        """
        self.s_plus = max(0.0, self.s_plus + residual - self.k)
        self.s_minus = max(0.0, self.s_minus - residual - self.k)

        collision_flag = self.s_plus > self.h or self.s_minus > self.h
        cusum_score = max(self.s_plus, self.s_minus)

        # Re-arm the detector immediately after a detection.
        if collision_flag:
            self.s_plus = 0.0
            self.s_minus = 0.0

        return collision_flag, cusum_score


class Payload_Estimator:
    """Sensorless payload mass estimator from FF-RLS gravity parameter shifts.

    Compares the current gravity-term coefficient in theta_hat against an
    empty-load baseline (theta_empty). An EMA low-pass filter isolates
    sustained step-changes from transient noise/spikes before converting
    the shift into an estimated mass in grams.
    """

    GRAVITY_PARAM_INDEX = 2  # index of the gravity term inside theta_hat/phi

    def __init__(
        self,
        theta_empty: np.ndarray,
        mass_sensitivity_gain: Optional[float] = None,
        filter_alpha: Optional[float] = None,
        attach_threshold_g: Optional[float] = None,
    ) -> None:
        cfg = get_settings().payload
        self.theta_empty = np.array(theta_empty, dtype=float)
        self.k_mass = mass_sensitivity_gain if mass_sensitivity_gain is not None else cfg.mass_sensitivity_gain
        self.alpha = filter_alpha if filter_alpha is not None else cfg.filter_alpha
        self.attach_threshold_g = (
            attach_threshold_g if attach_threshold_g is not None else cfg.attach_threshold_g
        )
        self._filtered_gravity_param: Optional[float] = None

    def update(self, theta_hat: np.ndarray) -> tuple[float, str]:
        """Estimates payload mass from the latest FF-RLS parameter vector.

        Returns:
            (estimated_mass_g, status) where status is "EMPTY" or
            "PAYLOAD_ATTACHED".
        """
        raw_gravity_param = float(theta_hat[self.GRAVITY_PARAM_INDEX])

        # Exponential moving average filters out transient spikes so only
        # sustained step-changes (a physically attached payload) register.
        if self._filtered_gravity_param is None:
            self._filtered_gravity_param = raw_gravity_param
        else:
            self._filtered_gravity_param = (
                self.alpha * raw_gravity_param
                + (1.0 - self.alpha) * self._filtered_gravity_param
            )

        delta_theta = self._filtered_gravity_param - self.theta_empty[self.GRAVITY_PARAM_INDEX]
        estimated_mass_g = delta_theta * self.k_mass

        status = "PAYLOAD_ATTACHED" if estimated_mass_g >= self.attach_threshold_g else "EMPTY"
        return estimated_mass_g, status
