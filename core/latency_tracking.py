"""Thread-safe CRI communication and telemetry-observed latency tracking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import statistics
import threading
import time
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
    def maximum_ms(self) -> float | None:
        samples = self._snapshot()
        return max(samples) if samples else None

    @property
    def peak_consecutive_jitter_ms(self) -> float | None:
        samples = self._snapshot()
        if len(samples) < 2:
            return None
        return max(abs(current - previous) for previous, current in zip(samples, samples[1:]))


class NetworkLatencyTracker:
    """Measures CRI command-to-ACK round-trip time using a monotonic clock."""

    def __init__(self, max_samples: int = 1000) -> None:
        self.statistics = LatencyStatistics(max_samples)
        self._max_pending = max_samples
        self._pending: dict[str, float] = {}
        self._lock = threading.RLock()

    def record_sent(self, command_id: str, timestamp: float | None = None) -> None:
        sent_at = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            if len(self._pending) >= self._max_pending:
                oldest_id = next(iter(self._pending))
                del self._pending[oldest_id]
            self._pending[str(command_id)] = sent_at

    def record_received(
        self, command_id: str, timestamp: float | None = None
    ) -> float | None:
        received_at = time.perf_counter() if timestamp is None else timestamp
        with self._lock:
            sent_at = self._pending.pop(str(command_id), None)
        if sent_at is None:
            return None
        rtt_ms = max(0.0, (received_at - sent_at) * 1000.0)
        self.statistics.add(rtt_ms)
        return rtt_ms

    @property
    def estimated_one_way_ms(self) -> float | None:
        """Return RTT/2, an estimate assuming approximately symmetric delay.

        This is not a directly measured one-way latency because the CRI
        protocol does not provide synchronized clocks between the PC and robot.
        """
        latest = self.statistics.latest_ms
        return latest / 2.0 if latest is not None else None


@dataclass(frozen=True)
class MotionResponse:
    """A telemetry-observed motion-onset measurement."""

    command_id: str
    joint_index: int
    gui_to_motion_ms: float
    tx_to_motion_ms: float | None


@dataclass
class _MotionCommand:
    command_id: str
    t_gui: float
    t_tx: float | None
    joint_index: int
    direction: float
    commanded_speed: float | None
    target_velocity_deg_s: float | None
    consecutive_motion_samples: int = 0


class ExecutionLatencyTracker:
    """Detects physical motion onset from consecutive CRI STATUS positions.

    The result is telemetry-observed motion response latency, not pure actuator
    latency. It includes GUI scheduling, network/controller processing, servo
    response, STATUS sampling, return transmission, and PC parsing delay.
    """

    def __init__(
        self,
        max_samples: int = 1000,
        minimum_velocity_deg_s: float = 0.05,
        noise_multiplier: float = 3.0,
        required_consecutive_samples: int = 2,
        stationary_velocity_std_deg_s: float = 0.0,
    ) -> None:
        if required_consecutive_samples < 1:
            raise ValueError("required_consecutive_samples must be positive")
        self.minimum_velocity_deg_s = minimum_velocity_deg_s
        self.noise_multiplier = noise_multiplier
        self.required_consecutive_samples = required_consecutive_samples
        self.stationary_velocity_std_deg_s = stationary_velocity_std_deg_s
        self.responses: Deque[MotionResponse] = deque(maxlen=max_samples)
        self.statistics = LatencyStatistics(max_samples)
        self._pending: dict[str, _MotionCommand] = {}
        self._previous_positions: list[float] | None = None
        self._previous_timestamp: float | None = None
        self._lock = threading.RLock()

    def start_command(
        self,
        command_id: str,
        t_gui: float | None = None,
        joint_index: int = 0,
        direction: float = 0.0,
        commanded_speed: float | None = None,
        target_velocity_deg_s: float | None = None,
        t_tx: float | None = None,
    ) -> None:
        if not 0 <= joint_index < 6:
            raise ValueError("joint_index must be between 0 and 5")
        if direction == 0.0:
            raise ValueError("direction must be positive or negative")
        with self._lock:
            self._pending[str(command_id)] = _MotionCommand(
                command_id=str(command_id),
                t_gui=time.perf_counter() if t_gui is None else t_gui,
                t_tx=t_tx,
                joint_index=joint_index,
                direction=1.0 if direction > 0.0 else -1.0,
                commanded_speed=commanded_speed,
                target_velocity_deg_s=target_velocity_deg_s,
            )

    def record_transmission(self, command_id: str, timestamp: float | None = None) -> None:
        with self._lock:
            command = self._pending.get(str(command_id))
            if command is not None:
                command.t_tx = time.perf_counter() if timestamp is None else timestamp

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
            threshold = max(
                self.minimum_velocity_deg_s,
                self.noise_multiplier * self.stationary_velocity_std_deg_s,
            )
            detected: list[MotionResponse] = []
            for command_id, command in list(self._pending.items()):
                velocity = velocities[command.joint_index]
                direction_matches = velocity * command.direction > 0.0
                threshold_exceeded = abs(velocity) >= threshold
                if command.target_velocity_deg_s is not None:
                    threshold_exceeded = threshold_exceeded and abs(velocity) >= max(
                        threshold, 0.05 * abs(command.target_velocity_deg_s)
                    )
                if direction_matches and threshold_exceeded:
                    command.consecutive_motion_samples += 1
                else:
                    command.consecutive_motion_samples = 0
                if command.consecutive_motion_samples < self.required_consecutive_samples:
                    continue
                response = MotionResponse(
                    command_id=command_id,
                    joint_index=command.joint_index,
                    gui_to_motion_ms=(now - command.t_gui) * 1000.0,
                    tx_to_motion_ms=(now - command.t_tx) * 1000.0
                    if command.t_tx is not None
                    else None,
                )
                self.responses.append(response)
                self.statistics.add(response.gui_to_motion_ms)
                detected.append(response)
                del self._pending[command_id]
            return tuple(detected)

    @property
    def latest_response(self) -> MotionResponse | None:
        with self._lock:
            return self.responses[-1] if self.responses else None