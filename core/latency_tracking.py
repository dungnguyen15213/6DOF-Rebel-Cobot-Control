"""Thread-safe CRI communication and telemetry-observed latency tracking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import statistics
import threading
import time
import uuid
from typing import Deque, Iterable


class LatencyStatistics:
    """Bounded, thread-safe statistics for latency samples in milliseconds."""

    def __init__(self, max_samples: int = 1000) -> None:
        if max_samples < 2:
            raise ValueError("max_samples must be at least 2")
        self._samples: Deque[float] = deque(maxlen=max_samples)
        self._lock = threading.RLock()

    def add(self, value_ms: float) -> None:
        if value_ms < 0.0:
            raise ValueError("latency cannot be negative")
        with self._lock:
            self._samples.append(float(value_ms))

    def _snapshot(self) -> list[float]:
        with self._lock:
            return list(self._samples)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._samples)

    @property
    def latest_ms(self) -> float | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    @property
    def mean_ms(self) -> float | None:
        samples = self._snapshot()
        return statistics.fmean(samples) if samples else None

    @property
    def standard_deviation_ms(self) -> float | None:
        samples = self._snapshot()
        return statistics.stdev(samples) if len(samples) >= 2 else None

    @property
    def std_ms(self) -> float | None:
        return self.standard_deviation_ms

    @property
    def median_ms(self) -> float | None:
        samples = self._snapshot()
        return statistics.median(samples) if samples else None

    def percentile_ms(self, percentile: float) -> float | None:
        if not 0.0 <= percentile <= 100.0:
            raise ValueError("percentile must be between 0 and 100")
        samples = sorted(self._snapshot())
        if not samples:
            return None
        if len(samples) == 1:
            return samples[0]
        rank = (len(samples) - 1) * percentile / 100.0
        lower = int(rank)
        upper = min(lower + 1, len(samples) - 1)
        fraction = rank - lower
        return samples[lower] + (samples[upper] - samples[lower]) * fraction

    @property
    def p95_ms(self) -> float | None:
        return self.percentile_ms(95.0)

    @property
    def p99_ms(self) -> float | None:
        return self.percentile_ms(99.0)

    @property
    def minimum_ms(self) -> float | None:
        samples = self._snapshot()
        return min(samples) if samples else None

    @property
    def min_ms(self) -> float | None:
        return self.minimum_ms

    @property
    def maximum_ms(self) -> float | None:
        samples = self._snapshot()
        return max(samples) if samples else None

    @property
    def max_ms(self) -> float | None:
        return self.maximum_ms

    @property
    def peak_consecutive_jitter_ms(self) -> float | None:
        samples = self._snapshot()
        if len(samples) < 2:
            return None
        return max(abs(current - previous) for previous, current in zip(samples, samples[1:]))

    @property
    def peak_jitter_ms(self) -> float | None:
        return self.peak_consecutive_jitter_ms


class NetworkLatencyTracker:
    """Measures CRI command-to-ACK round-trip time using a monotonic clock."""

    def __init__(self, max_samples: int = 1000, pending_ttl_s: float = 10.0) -> None:
        self.statistics = LatencyStatistics(max_samples)
        self.error_statistics = LatencyStatistics(max_samples)
        self._error_count = 0
        self._max_pending = max_samples
        self.pending_ttl_s = pending_ttl_s
        self._pending: dict[str, float] = {}
        self._lock = threading.RLock()

    def record_sent(self, command_id: str, timestamp: float | None = None) -> None:
        sent_at = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            self._expire_pending_locked(sent_at)
            if len(self._pending) >= self._max_pending:
                oldest_id = next(iter(self._pending))
                del self._pending[oldest_id]
            self._pending[str(command_id)] = sent_at

    def expire_pending(self, timestamp: float | None = None) -> int:
        """Remove unmatched command IDs older than the configured TTL."""
        now = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            return self._expire_pending_locked(now)

    def _expire_pending_locked(self, now: float) -> int:
        stale_ids = [
            command_id
            for command_id, sent_at in self._pending.items()
            if now - sent_at > self.pending_ttl_s
        ]
        for command_id in stale_ids:
            del self._pending[command_id]
        return len(stale_ids)

    def record_received(
        self, command_id: str, timestamp: float | None = None
    ) -> float | None:
        received_at = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            self._expire_pending_locked(received_at)
            sent_at = self._pending.pop(str(command_id), None)
        if sent_at is None:
            return None
        rtt_ms = max(0.0, (received_at - sent_at) * 1000.0)
        self.statistics.add(rtt_ms)
        return rtt_ms

    def record_error(
        self, command_id: str, timestamp: float | None = None
    ) -> float | None:
        """Records a CMDERROR response time separately from ACK RTT statistics.

        CMDERROR responses are excluded from `statistics` so that "CRI ACK RTT"
        only reflects successful command acknowledgments.
        """
        received_at = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            self._expire_pending_locked(received_at)
            sent_at = self._pending.pop(str(command_id), None)
            self._error_count += 1
        if sent_at is None:
            return None
        error_rtt_ms = max(0.0, (received_at - sent_at) * 1000.0)
        self.error_statistics.add(error_rtt_ms)
        return error_rtt_ms

    @property
    def error_count(self) -> int:
        with self._lock:
            return self._error_count

    @property
    def estimated_one_way_ms(self) -> float | None:
        """Return RTT/2, an estimate assuming approximately symmetric delay.

        This is not a directly measured one-way latency because the CRI
        protocol does not provide synchronized clocks between the PC and robot.
        """
        latest = self.statistics.latest_ms
        return latest / 2.0 if latest is not None else None


class MeasurementStatus(str, Enum):
    PENDING = "PENDING"
    DETECTED = "DETECTED"
    TIMEOUT = "TIMEOUT"
    INVALID_ALREADY_MOVING = "INVALID_ALREADY_MOVING"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class MotionResponse:
    """A telemetry-observed motion-onset measurement for one DETECTED trial."""

    command_id: str
    joint_index: int
    gui_to_motion_ms: float
    tx_to_motion_ms: float | None
    gui_to_tx_ms: float | None
    motion_threshold_deg_s: float
    stationary_velocity_std_deg_s: float
    status: MeasurementStatus = MeasurementStatus.DETECTED


@dataclass(frozen=True)
class ExecutionLatencySnapshot:
    """Atomic view of the most recently started or completed trial.

    Status and latency values always belong to the same `trial_id`, so a GUI
    reading this snapshot cannot mix a stale response with a newer status.
    """

    trial_id: str | None
    status: MeasurementStatus | None
    gui_to_tx_ms: float | None
    tx_to_motion_ms: float | None
    gui_to_motion_ms: float | None
    response: MotionResponse | None = None


@dataclass(frozen=True)
class CompletedTrial:
    """Immutable, fully-populated record of one finished trial for event logging."""

    trial_id: str
    command_id: str
    command_type: str
    joint_index: int
    direction: float
    commanded_speed: float | None
    target_velocity_deg_s: float | None
    t_gui: float
    t_tx: float | None
    t_motion: float | None
    gui_to_tx_ms: float | None
    tx_to_motion_ms: float | None
    gui_to_motion_ms: float | None
    motion_threshold_deg_s: float | None
    stationary_velocity_std_deg_s: float | None
    confirmation_samples: int
    status: MeasurementStatus


@dataclass
class _MotionCommand:
    command_id: str | None
    trial_id: str
    command_type: str
    t_gui: float
    t_tx: float | None
    joint_index: int
    direction: float
    commanded_speed: float | None
    target_velocity_deg_s: float | None
    consecutive_motion_samples: int = 0
    candidate_motion_time: float | None = None
    timeout_at: float | None = None


class ExecutionLatencyTracker:
    """Detects physical motion onset from consecutive CRI STATUS positions.

    The result is telemetry-observed motion response latency, not pure actuator
    latency. It includes GUI scheduling, network/controller processing, servo
    response, STATUS sampling, return transmission, and PC parsing delay.

    Every trial is finalized through a single atomic completion path
    (`_complete_trial_locked`) so `_pending`, the latest snapshot, and the
    completed-trial event queue never disagree about a trial's outcome.
    """

    def __init__(
        self,
        max_samples: int = 1000,
        minimum_velocity_deg_s: float = 0.05,
        noise_multiplier: float = 3.0,
        required_consecutive_samples: int = 2,
        stationary_velocity_std_deg_s: float = 0.0,
        motion_timeout_s: float = 3.0,
        stationary_band_deg_s: float = 0.5,
    ) -> None:
        if required_consecutive_samples < 1:
            raise ValueError("required_consecutive_samples must be positive")
        self.minimum_velocity_deg_s = minimum_velocity_deg_s
        self.noise_multiplier = noise_multiplier
        self.required_consecutive_samples = required_consecutive_samples
        self.stationary_velocity_std_deg_s = stationary_velocity_std_deg_s
        self.motion_timeout_s = motion_timeout_s
        self.stationary_band_deg_s = stationary_band_deg_s
        self.responses: Deque[MotionResponse] = deque(maxlen=max_samples)
        self.statistics = LatencyStatistics(max_samples)
        self._pending: dict[str, _MotionCommand] = {}
        self._previous_positions: list[float] | None = None
        self._previous_timestamp: float | None = None
        self._previous_velocities = [0.0] * 6
        self._stationary_velocities: list[Deque[float]] = [
            deque(maxlen=max_samples) for _ in range(6)
        ]
        self._latest_snapshot: ExecutionLatencySnapshot | None = None
        self._completed_trials: list[CompletedTrial] = []
        self._trial_session_id = uuid.uuid4().hex[:8].upper()
        self._trial_sequence = 0
        self._lock = threading.RLock()

    def _next_trial_id_locked(self) -> str:
        self._trial_sequence += 1
        return f"LAT-{self._trial_session_id}-{self._trial_sequence:06d}"

    def start_command(
        self,
        command_id: str,
        t_gui: float | None = None,
        joint_index: int = 0,
        direction: float = 0.0,
        commanded_speed: float | None = None,
        target_velocity_deg_s: float | None = None,
        t_tx: float | None = None,
        timeout_s: float | None = None,
        command_type: str = "ALIVEJOG",
    ) -> str:
        """Arms a new controlled trial and returns its unique `trial_id`."""
        if not 0 <= joint_index < 6:
            raise ValueError("joint_index must be between 0 and 5")
        if direction == 0.0:
            raise ValueError("direction must be positive or negative")
        with self._lock:
            started_at = time.perf_counter() if t_gui is None else t_gui
            trial_id = self._next_trial_id_locked()
            command = _MotionCommand(
                command_id=str(command_id),
                trial_id=trial_id,
                command_type=command_type,
                t_gui=started_at,
                t_tx=t_tx,
                joint_index=joint_index,
                direction=1.0 if direction > 0.0 else -1.0,
                commanded_speed=commanded_speed,
                target_velocity_deg_s=target_velocity_deg_s,
                timeout_at=started_at + (self.motion_timeout_s if timeout_s is None else timeout_s),
            )
            if abs(self._previous_velocities[joint_index]) >= self._threshold(joint_index):
                self._complete_trial_locked(command, MeasurementStatus.INVALID_ALREADY_MOVING, started_at)
                return trial_id
            if self._pending:
                # Defensive: a new trial supersedes any unfinished one instead of
                # leaving it to be silently discarded and possibly misreported.
                stale_command = next(iter(self._pending.values()))
                self._complete_trial_locked(stale_command, MeasurementStatus.CANCELLED, started_at)
            self._pending[command.command_id] = command
            self._latest_snapshot = ExecutionLatencySnapshot(
                trial_id=trial_id,
                status=MeasurementStatus.PENDING,
                gui_to_tx_ms=(t_tx - started_at) * 1000.0 if t_tx is not None else None,
                tx_to_motion_ms=None,
                gui_to_motion_ms=None,
                response=None,
            )
            return trial_id

    def arm_trial(
        self,
        t_gui: float | None = None,
        joint_index: int = 0,
        direction: float = 0.0,
        commanded_speed: float | None = None,
        target_velocity_deg_s: float | None = None,
        timeout_s: float | None = None,
        command_type: str = "ALIVEJOG",
    ) -> str:
        """Arms a GUI-originated trial before its CRI command is transmitted."""
        if not 0 <= joint_index < 6:
            raise ValueError("joint_index must be between 0 and 5")
        if direction == 0.0:
            raise ValueError("direction must be positive or negative")
        with self._lock:
            started_at = time.perf_counter() if t_gui is None else t_gui
            trial_id = self._next_trial_id_locked()
            command = _MotionCommand(
                command_id=None,
                trial_id=trial_id,
                command_type=command_type,
                t_gui=started_at,
                t_tx=None,
                joint_index=joint_index,
                direction=1.0 if direction > 0.0 else -1.0,
                commanded_speed=commanded_speed,
                target_velocity_deg_s=target_velocity_deg_s,
                timeout_at=started_at + (self.motion_timeout_s if timeout_s is None else timeout_s),
            )
            if abs(self._previous_velocities[joint_index]) >= self._threshold(joint_index):
                self._complete_trial_locked(command, MeasurementStatus.INVALID_ALREADY_MOVING, started_at)
                return trial_id
            if self._pending:
                stale_command = next(iter(self._pending.values()))
                self._complete_trial_locked(stale_command, MeasurementStatus.CANCELLED, started_at)
            self._pending[trial_id] = command
            self._latest_snapshot = ExecutionLatencySnapshot(
                trial_id=trial_id,
                status=MeasurementStatus.PENDING,
                gui_to_tx_ms=None,
                tx_to_motion_ms=None,
                gui_to_motion_ms=None,
                response=None,
            )
            return trial_id

    def bind_transmission(
        self, trial_id: str, command_id: str, timestamp: float | None = None
    ) -> bool:
        """Binds the first matching CRI transmission to an armed GUI trial."""
        with self._lock:
            command = self._pending.pop(trial_id, None)
            if command is None or command.command_id is not None:
                return False
            command.command_id = str(command_id)
            command.t_tx = time.perf_counter() if timestamp is None else timestamp
            self._pending[command.command_id] = command
            self._latest_snapshot = ExecutionLatencySnapshot(
                trial_id=command.trial_id,
                status=MeasurementStatus.PENDING,
                gui_to_tx_ms=(command.t_tx - command.t_gui) * 1000.0,
                tx_to_motion_ms=None,
                gui_to_motion_ms=None,
                response=None,
            )
            return True

    def cancel_pending(
        self, status: MeasurementStatus = MeasurementStatus.CANCELLED, timestamp: float | None = None
    ) -> bool:
        """Cancels the active trial, if any. Never overwrites a completed trial."""
        with self._lock:
            if not self._pending:
                return False
            now = time.perf_counter() if timestamp is None else timestamp
            command = next(iter(self._pending.values()))
            self._complete_trial_locked(command, status, now)
            return True

    def expire_pending(self, timestamp: float | None = None) -> bool:
        """Finalizes an unobserved active trial as TIMEOUT once its deadline passes."""
        now = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            for command in list(self._pending.values()):
                if command.timeout_at is not None and now >= command.timeout_at:
                    self._complete_trial_locked(command, MeasurementStatus.TIMEOUT, now)
                    return True
            return False

    def get_stationary_velocity_std(self, joint_index: int) -> float | None:
        with self._lock:
            samples = list(self._stationary_velocities[joint_index])
        return statistics.stdev(samples) if len(samples) >= 2 else None

    def _threshold(self, joint_index: int) -> float:
        noise_std = self.get_stationary_velocity_std(joint_index)
        return max(
            self.minimum_velocity_deg_s,
            self.noise_multiplier * (noise_std if noise_std is not None else self.stationary_velocity_std_deg_s),
        )

    def record_transmission(self, command_id: str, timestamp: float | None = None) -> None:
        with self._lock:
            command = self._pending.get(str(command_id))
            if command is None:
                return
            command.t_tx = time.perf_counter() if timestamp is None else timestamp
            if self._latest_snapshot is not None and self._latest_snapshot.trial_id == command.trial_id:
                self._latest_snapshot = ExecutionLatencySnapshot(
                    trial_id=command.trial_id,
                    status=self._latest_snapshot.status,
                    gui_to_tx_ms=(command.t_tx - command.t_gui) * 1000.0,
                    tx_to_motion_ms=self._latest_snapshot.tx_to_motion_ms,
                    gui_to_motion_ms=self._latest_snapshot.gui_to_motion_ms,
                    response=self._latest_snapshot.response,
                )

    def _complete_trial_locked(
        self,
        command: _MotionCommand,
        status: MeasurementStatus,
        completed_at: float,
        motion_time: float | None = None,
    ) -> tuple[CompletedTrial, MotionResponse | None]:
        """Single atomic finalization path for every trial outcome.

        Finalizes status, builds exactly one `CompletedTrial` event, updates the
        snapshot the GUI reads, and clears the trial from `_pending`.
        """
        gui_to_tx_ms = (command.t_tx - command.t_gui) * 1000.0 if command.t_tx is not None else None
        tx_to_motion_ms: float | None = None
        gui_to_motion_ms: float | None = None
        response: MotionResponse | None = None
        threshold = self._threshold(command.joint_index)
        stationary_std = self.get_stationary_velocity_std(command.joint_index)

        if status == MeasurementStatus.DETECTED and motion_time is not None:
            gui_to_motion_ms = (motion_time - command.t_gui) * 1000.0
            tx_to_motion_ms = (
                (motion_time - command.t_tx) * 1000.0 if command.t_tx is not None else None
            )
            response = MotionResponse(
                command_id=command.command_id,
                joint_index=command.joint_index,
                gui_to_motion_ms=gui_to_motion_ms,
                tx_to_motion_ms=tx_to_motion_ms,
                gui_to_tx_ms=gui_to_tx_ms,
                motion_threshold_deg_s=threshold,
                stationary_velocity_std_deg_s=stationary_std or 0.0,
            )
            self.responses.append(response)
            self.statistics.add(gui_to_motion_ms)

        trial = CompletedTrial(
            trial_id=command.trial_id,
            command_id=command.command_id or "",
            command_type=command.command_type,
            joint_index=command.joint_index,
            direction=command.direction,
            commanded_speed=command.commanded_speed,
            target_velocity_deg_s=command.target_velocity_deg_s,
            t_gui=command.t_gui,
            t_tx=command.t_tx,
            t_motion=motion_time if status == MeasurementStatus.DETECTED else None,
            gui_to_tx_ms=gui_to_tx_ms,
            tx_to_motion_ms=tx_to_motion_ms,
            gui_to_motion_ms=gui_to_motion_ms,
            motion_threshold_deg_s=threshold,
            stationary_velocity_std_deg_s=stationary_std,
            confirmation_samples=command.consecutive_motion_samples,
            status=status,
        )
        self._completed_trials.append(trial)
        self._latest_snapshot = ExecutionLatencySnapshot(
            trial_id=command.trial_id,
            status=status,
            gui_to_tx_ms=gui_to_tx_ms,
            tx_to_motion_ms=tx_to_motion_ms,
            gui_to_motion_ms=gui_to_motion_ms,
            response=response,
        )
        for pending_key, pending_command in list(self._pending.items()):
            if pending_command is command:
                del self._pending[pending_key]
                break
        return trial, response

    def update_telemetry(
        self,
        positions: Iterable[float],
        timestamp: float | None = None,
    ) -> tuple[MotionResponse, ...]:
        now = time.perf_counter() if timestamp is None else timestamp
        current = [float(value) for value in positions][:6]
        if len(current) != 6:
            raise ValueError("positions must contain six joint values")
        with self._lock:
            if self._previous_positions is None or self._previous_timestamp is None:
                self._previous_positions = current
                self._previous_timestamp = now
                return ()
            dt = now - self._previous_timestamp
            if dt <= 0.0:
                return ()
            velocities = [
                (current[index] - self._previous_positions[index]) / dt
                for index in range(6)
            ]
            self._previous_positions = current
            self._previous_timestamp = now
            self._previous_velocities = velocities
            if not self._pending:
                for index, velocity in enumerate(velocities):
                    if abs(velocity) <= self.stationary_band_deg_s:
                        self._stationary_velocities[index].append(velocity)
            detected: list[MotionResponse] = []
            for command_id, command in list(self._pending.items()):
                if command.timeout_at is not None and now >= command.timeout_at:
                    self._complete_trial_locked(command, MeasurementStatus.TIMEOUT, now)
                    continue
                threshold = self._threshold(command.joint_index)
                velocity = velocities[command.joint_index]
                direction_matches = velocity * command.direction > 0.0
                threshold_exceeded = abs(velocity) >= threshold
                if command.target_velocity_deg_s is not None:
                    threshold_exceeded = threshold_exceeded and abs(velocity) >= max(
                        threshold, 0.05 * abs(command.target_velocity_deg_s)
                    )
                if direction_matches and threshold_exceeded:
                    if command.consecutive_motion_samples == 0:
                        command.candidate_motion_time = now
                    command.consecutive_motion_samples += 1
                else:
                    command.consecutive_motion_samples = 0
                    command.candidate_motion_time = None
                if command.consecutive_motion_samples < self.required_consecutive_samples:
                    continue
                _, response = self._complete_trial_locked(
                    command,
                    MeasurementStatus.DETECTED,
                    now,
                    motion_time=command.candidate_motion_time or now,
                )
                if response is not None:
                    detected.append(response)
            return tuple(detected)

    @property
    def snapshot(self) -> ExecutionLatencySnapshot | None:
        """Atomic snapshot of the most recent trial's status and latency values."""
        with self._lock:
            return self._latest_snapshot

    def drain_completed_trials(self) -> list[CompletedTrial]:
        """Returns and clears newly completed trials; each trial is returned once."""
        with self._lock:
            drained = list(self._completed_trials)
            self._completed_trials.clear()
            return drained

    @property
    def latest_response(self) -> MotionResponse | None:
        with self._lock:
            return self._latest_snapshot.response if self._latest_snapshot else None

    @property
    def latest_status(self) -> MeasurementStatus | None:
        with self._lock:
            return self._latest_snapshot.status if self._latest_snapshot else None