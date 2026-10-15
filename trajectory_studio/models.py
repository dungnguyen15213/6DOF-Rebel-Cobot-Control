"""Data model for an editable trajectory recipe.

Conventions (mirrors core.path_executor, which stays the execution authority):
- Joint values are degrees, order J1..J6.
- Phase PREPARATION is the start pose reached by Move-to-Start; TASK rows are
  the measured motion sent by the existing executor.
- Velocity is one controller-command percentage for the whole file. The executor
  uses a single speed, so per-waypoint speeds are not representable.
- Dwell time is intentionally not modelled: the executor has no dwell support.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any

JOINT_NAMES = [f"J{i}" for i in range(1, 7)]
PHASES = ("PREPARATION", "TASK")
CATEGORIES = [
    "Joint Validation",
    "Stationary Baselines",
    "Single-Joint Identification",
    "Multi-Joint Identification",
    "General Robot Pathways",
]
VALIDATION_STATUSES = ("DRAFT", "VALIDATED (static)", "OPERATOR APPROVED")
MOTION_EXECUTABLE = "EXECUTABLE"
MOTION_NONE = "NO_MOTION"


@dataclass
class Waypoint:
    phase: str
    joints: list[float]
    note: str = ""

    def copy(self) -> "Waypoint":
        return Waypoint(self.phase, list(self.joints), self.note)

    def to_dict(self) -> dict[str, Any]:
        return {"phase": self.phase, "joints_deg": list(self.joints), "note": self.note}

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "Waypoint":
        return Waypoint(str(data["phase"]), [float(v) for v in data["joints_deg"]], str(data.get("note", "")))


@dataclass
class Trajectory:
    name: str = "untitled"
    description: str = ""
    category: str = "General Robot Pathways"
    kind: str = "manual"  # manual, imported, single_joint, bidirectional, coordinated, stationary, test_b_template
    version: int = 1
    validation_status: str = "DRAFT"
    waypoints: list[Waypoint] = field(default_factory=list)
    velocity_percent: float | None = None
    # Operator-declared limits only; the repository carries no authoritative limits.
    joint_limits_deg: list[list[float] | None] = field(default_factory=lambda: [None] * 6)
    limits_source: str = ""
    locked_joints: list[bool] = field(default_factory=lambda: [False] * 6)
    motion_mode: str = MOTION_EXECUTABLE
    generation: dict[str, Any] = field(default_factory=dict)
    experiment: dict[str, Any] = field(default_factory=dict)
    approval: dict[str, Any] = field(default_factory=lambda: {"approved": False, "by": "", "date": ""})
    source_csv_sha256: str = ""

    def copy(self) -> "Trajectory":
        return copy.deepcopy(self)

    def snapshot(self) -> list[Waypoint]:
        return [w.copy() for w in self.waypoints]

    def moving_joints(self, tolerance_deg: float = 1e-9) -> list[int]:
        """Zero-based joints whose value changes anywhere in the trajectory."""
        if len(self.waypoints) < 2:
            return []
        out = []
        for j in range(6):
            values = [w.joints[j] for w in self.waypoints]
            if max(values) - min(values) > tolerance_deg:
                out.append(j)
        return out

    def joint_range(self, joint: int) -> tuple[float, float] | None:
        values = [w.joints[joint] for w in self.waypoints if math.isfinite(w.joints[joint])]
        return (min(values), max(values)) if values else None

    # --- edit operations (pure; the GUI wraps them in QUndoCommands) ---
    def insert_waypoint(self, index: int, waypoint: Waypoint) -> None:
        self.waypoints.insert(max(0, min(index, len(self.waypoints))), waypoint)
        self.normalize_phases()

    def delete_waypoints(self, indices: list[int]) -> None:
        for i in sorted(set(indices), reverse=True):
            if 0 <= i < len(self.waypoints):
                del self.waypoints[i]
        self.normalize_phases()

    def duplicate_waypoints(self, indices: list[int]) -> None:
        for i in sorted(set(indices), reverse=True):
            if 0 <= i < len(self.waypoints):
                clone = self.waypoints[i].copy()
                clone.phase = "TASK"
                self.waypoints.insert(i + 1, clone)
        self.normalize_phases()

    def move_waypoint(self, source: int, target: int) -> None:
        if not (0 <= source < len(self.waypoints)):
            raise IndexError(source)
        target = max(0, min(target, len(self.waypoints) - 1))
        self.waypoints.insert(target, self.waypoints.pop(source))
        self.normalize_phases()

    def normalize_phases(self) -> None:
        """Row 0 is the PREPARATION pose, every other row TASK (executor semantics)."""
        for i, w in enumerate(self.waypoints):
            w.phase = "PREPARATION" if i == 0 else "TASK"

    def set_joint(self, index: int, joint: int, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError("joint value must be finite")
        self.waypoints[index].joints[joint] = float(value)
