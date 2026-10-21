"""Plans a list of motion steps into one continuous joint-space pathway."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from core.kinematics import JOINT_LIMITS_DEG

MOTION_PTP = "ptp"
MOTION_LINE = "line"
MOTION_PTP_LEVEL = "ptp_level"
MOTION_LINE_LEVEL = "line_level"
MOTION_LOCKED = "locked"

MOTION_LABELS = {
    MOTION_PTP: "Point to point (free)",
    MOTION_LINE: "Straight line",
    MOTION_PTP_LEVEL: "Free move, tool level",
    MOTION_LINE_LEVEL: "Straight line, tool level",
    MOTION_LOCKED: "Locked joints",
}
LEVEL_MOTIONS = {MOTION_PTP_LEVEL, MOTION_LINE_LEVEL}

# Largest joint difference (deg) between consecutive steps that needs no re-orientation bridge.
BRIDGE_TOLERANCE_DEG = 0.5
BRIDGE_DEG_PER_WAYPOINT = 2.0
# A larger joint change between two neighbouring waypoints means an IK branch flip or too few waypoints.
MAX_JOINT_JUMP_DEG = 25.0


class SequenceError(ValueError):
    """Raised when a step cannot be planned."""


@dataclass
class SequenceStep:
    motion: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    waypoints: int = 30
    locked: tuple[int, ...] = ()


@dataclass
class SegmentInfo:
    label: str
    is_bridge: bool
    first_frame: int
    last_frame: int


@dataclass
class SequencePlan:
    path: list = field(default_factory=list)
    segments: list = field(default_factory=list)

    def segment_at(self, frame: int):
        for segment in self.segments:
            if segment.first_frame <= frame <= segment.last_frame:
                return segment
        return None


class SequencePlanner:
    def __init__(self, kinematics, path_planner) -> None:
        self.kinematics = kinematics
        self.path_planner = path_planner

    def plan(self, steps, initial_angles) -> SequencePlan:
        if not steps:
            raise SequenceError("The sequence has no steps.")

        raw_segments = []  # (label, is_bridge, joint path)
        previous_end = None
        for number, step in enumerate(steps, start=1):
            self._validate(number, step)
            seed = list(previous_end if previous_end is not None else initial_angles)
            start_angles = self._solve_start(number, step, seed)

            if previous_end is not None:
                gap = max(abs(a - b) for a, b in zip(previous_end, start_angles))
                if gap > BRIDGE_TOLERANCE_DEG:
                    count = int(min(60, max(2, math.ceil(gap / BRIDGE_DEG_PER_WAYPOINT))))
                    bridge = self.path_planner.generate_joint_path(previous_end, start_angles, count)
                    raw_segments.append((f"Step {number - 1}* (re-orient to step {number})", True, bridge))

            segment = self._plan_checked(number, step, start_angles)
            raw_segments.append((f"Step {number}: {MOTION_LABELS[step.motion]}", False, segment))
            previous_end = segment[-1]

        return self._assemble(raw_segments)

    def check_steps(self, steps, initial_angles):
        """Validate every step; returns one entry per step: None when it can move, else the reason."""
        results = []
        previous_end = None
        for number, step in enumerate(steps, start=1):
            try:
                self._validate(number, step)
                seed = list(previous_end if previous_end is not None else initial_angles)
                start_angles = self._solve_start(number, step, seed)
                segment = self._plan_checked(number, step, start_angles)
            except SequenceError as error:
                results.append(str(error))
                continue
            results.append(None)
            previous_end = segment[-1]
        return results

    def validate_step(self, step, seed_angles):
        """Check one step on its own. Returns None when it can move, else the reason."""
        return self.check_steps([step], seed_angles)[0]

    def _plan_checked(self, number, step, start_angles):
        segment = self._plan_step(number, step, start_angles)
        problem = self.check_path(segment)
        if problem:
            raise SequenceError(f"Step {number}: {problem}")
        return segment

    @staticmethod
    def check_path(path):
        """Return a description of the first joint-limit or joint-jump violation, or None."""
        for frame, angles in enumerate(path):
            for index, (low, high) in enumerate(JOINT_LIMITS_DEG):
                if not low - 1e-6 <= angles[index] <= high + 1e-6:
                    return (f"J{index + 1} reaches {angles[index]:.1f}° at waypoint {frame + 1}, "
                            f"outside the robot limit {low:.0f}° to {high:.0f}°.")
            if frame:
                jump = max(abs(a - b) for a, b in zip(angles, path[frame - 1]))
                if jump > MAX_JOINT_JUMP_DEG:
                    return (f"a joint jumps {jump:.0f}° between waypoints {frame} and {frame + 1}; "
                            "increase the waypoint count or choose a different motion/end point.")
        return None

    @staticmethod
    def _validate(number, step) -> None:
        if step.motion not in MOTION_LABELS:
            raise SequenceError(f"Step {number} has an unknown motion type '{step.motion}'.")
        if step.waypoints < 2:
            raise SequenceError(f"Step {number} needs at least 2 waypoints.")
        if step.motion == MOTION_LOCKED and not step.locked:
            raise SequenceError(f"Step {number} uses locked joints but none are selected.")

    def _ik(self, number, role, point, seed, locks=None, level=False):
        angles = self.kinematics.inverse_kinematics(
            point, initial_guess_angles=seed, locked_joints=locks or {}, level_tool=level
        )
        if angles is None:
            reason = self.kinematics.last_ik_error or "the point is outside the robot workspace"
            raise SequenceError(f"Step {number}: cannot reach the {role} point. {reason}")
        return angles

    def _solve_start(self, number, step, seed):
        locks = {index: seed[index] for index in step.locked} if step.motion == MOTION_LOCKED else {}
        return self._ik(number, "start", step.start, seed, locks, step.motion in LEVEL_MOTIONS)

    def _plan_step(self, number, step, start_angles):
        motion = step.motion
        if motion in (MOTION_LINE, MOTION_LINE_LEVEL):
            level = motion == MOTION_LINE_LEVEL

            def solve(point, previous):
                angles = self.kinematics.inverse_kinematics(
                    point, initial_guess_angles=previous or start_angles, level_tool=level
                )
                if angles is None:
                    reason = self.kinematics.last_ik_error or "the point is outside the robot workspace"
                    raise SequenceError(f"Step {number}: the straight line leaves the workspace. {reason}")
                return angles

            path = self.path_planner.generate_cartesian_line(step.start, step.end, step.waypoints, solve)
            path[0] = list(start_angles)
            return path

        locks = {index: start_angles[index] for index in step.locked} if motion == MOTION_LOCKED else {}
        level = motion == MOTION_PTP_LEVEL
        end_angles = self._ik(number, "end", step.end, start_angles, locks, level)
        for index, value in locks.items():
            end_angles[index] = value
        path = self.path_planner.generate_joint_path(start_angles, end_angles, step.waypoints)
        if level:
            # Follow the free joint-space arc but re-solve each point so the tool plane stays level.
            for i in range(1, len(path) - 1):
                tip = self.kinematics.get_tip_position(np.radians(path[i]))
                path[i] = self._ik(number, "intermediate", tip, path[i], level=True)
        return path

    def _assemble(self, raw_segments) -> SequencePlan:
        plan = SequencePlan()
        for label, is_bridge, joint_path in raw_segments:
            part = joint_path if not plan.path else joint_path[1:]
            first = len(plan.path)
            plan.path.extend(part)
            if part:
                plan.segments.append(SegmentInfo(label, is_bridge, first, len(plan.path) - 1))
        return plan
