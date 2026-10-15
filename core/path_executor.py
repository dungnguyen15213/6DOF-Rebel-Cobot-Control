"""Trajectory import/export and execution metadata for the ReBeL robot.

The project historically treated the last generated GUI path as the execution
source of truth. That makes execution dependent on a GUI session and prevents a
trajectory CSV from being used as a standalone execution artifact. The model here
keeps a trajectory as a named file with an explicit preparation start pose and a
separate task trajectory.
"""

from __future__ import annotations

import csv
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np


JOINT_COLUMNS = [f"J{i}" for i in range(1, 7)]
JOINT_COLUMN_ALIASES = {
    "J1": "J1",
    "J2": "J2",
    "J3": "J3",
    "J4": "J4",
    "J5": "J5",
    "J6": "J6",
    "A1": "J1",
    "A2": "J2",
    "A3": "J3",
    "A4": "J4",
    "A5": "J5",
    "A6": "J6",
}
DEFAULT_TRAJECTORY_DIR = Path("Pathways")


@dataclass
class TrajectoryPlan:
    """Validated data extracted from a trajectory CSV file."""

    file_path: str | Path | None = None
    trajectory_id: str = "UNSPECIFIED_TRAJECTORY"
    start_angles: list[float] = field(default_factory=list)
    task_waypoints: list[list[float]] = field(default_factory=list)
    velocity_percent: float = 50.0
    legacy_format: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def path_data(self) -> list[list[float]]:
        return list(self.task_waypoints)

    @property
    def final_target(self) -> list[float] | None:
        if not self.task_waypoints:
            return None
        return list(self.task_waypoints[-1])

    def to_dict(self) -> dict[str, Any]:
        return {
            "file_path": str(self.file_path) if self.file_path is not None else "",
            "trajectory_id": self.trajectory_id,
            "start_angles": list(self.start_angles),
            "task_waypoints": [list(waypoint) for waypoint in self.task_waypoints],
            "velocity_percent": self.velocity_percent,
            "legacy_format": self.legacy_format,
            "metadata": dict(self.metadata),
        }


class PathExecutor:
    """Stores and validates trajectory waypoints, with explicit start-position metadata."""

    def __init__(self):
        self.path_data: list[list[float]] = []
        self.start_angles: list[float] = []
        self.current_trajectory: TrajectoryPlan | None = None
        self.trajectory_directory = self.ensure_trajectory_directory()

    @staticmethod
    def ensure_trajectory_directory(directory: str | Path | None = None) -> Path:
        target = Path(directory) if directory is not None else DEFAULT_TRAJECTORY_DIR
        target.mkdir(parents=True, exist_ok=True)
        return target

    @staticmethod
    def default_trajectory_directory() -> Path:
        return PathExecutor.ensure_trajectory_directory(DEFAULT_TRAJECTORY_DIR)

    @staticmethod
    def _normalize_header(value: Any) -> str:
        return re.sub(r"[^a-zA-Z0-9]+", "_", str(value or "")).strip("_").lower()

    @staticmethod
    def _coerce_float(value: Any) -> float | None:
        if value is None:
            return None
        if isinstance(value, (int, float)):
            return float(value)
        text = str(value).strip()
        if not text or text.lower() in {"nan", "none"}:
            return None
        try:
            return float(text)
        except ValueError:
            return None

    @staticmethod
    def _parse_joint_row(row: dict[str, Any]) -> list[float] | None:
        candidates = []
        for key in list(row.keys()):
            normalized = PathExecutor._normalize_header(key)
            base = normalized.replace("_deg", "").replace("_degrees", "").replace("_angle", "")
            if base in {"j1", "j2", "j3", "j4", "j5", "j6", "a1", "a2", "a3", "a4", "a5", "a6"}:
                alias = base.upper()
                if alias.startswith("A"):
                    number = int(alias[1:])
                    alias = f"J{number}"
                if alias not in JOINT_COLUMN_ALIASES:
                    continue
                if row.get(key) is None:
                    continue
                candidates.append((alias, PathExecutor._coerce_float(row.get(key))))

        by_joint: dict[str, float] = {}
        for joint_name, value in candidates:
            if value is None:
                continue
            by_joint[joint_name] = value

        if not by_joint:
            return None

        ordered = []
        for joint_name in JOINT_COLUMNS:
            if joint_name in by_joint:
                ordered.append(float(by_joint[joint_name]))
            else:
                return None
        return ordered

    @staticmethod
    def _parse_legacy_cri_command(command: str) -> tuple[list[float], float | None]:
        if command is None:
            return [], None
        text = str(command).strip()
        match = re.search(r"CMD\s+Move\s+Joint\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+).*?(?:\s+([-+0-9.eE]+))?\s*$", text)
        if match is None:
            raise ValueError(f"Unsupported legacy CRI command: {command!r}")
        angles = [float(value) for value in match.groups()[:6]]
        velocity = PathExecutor._coerce_float(match.group(7))
        return angles, velocity

    def _set_loaded_path(self, waypoint_list: Iterable[Iterable[float]], start_angles: list[float] | None = None):
        path = [[float(value) for value in waypoint] for waypoint in waypoint_list]
        if not path:
            raise ValueError("Empty path data")
        if any(len(angles) != 6 for angles in path):
            raise ValueError("Each path waypoint must contain six joint angles")
        self.path_data = path
        self.start_angles = list(start_angles) if start_angles is not None else list(path[0])

    def load_path(self, joint_angles_list):
        """Load ordered six-axis joint targets without local timing data."""
        self._set_loaded_path(joint_angles_list)
        self.current_trajectory = TrajectoryPlan(
            file_path=None,
            trajectory_id="UNSPECIFIED_TRAJECTORY",
            start_angles=list(self.start_angles),
            task_waypoints=[list(waypoint) for waypoint in self.path_data],
            velocity_percent=50.0,
            metadata={"source": "in_memory"},
        )
        return self.current_trajectory

    def _parse_phase_trajectory(self, file_path: Path, rows: list[dict[str, Any]]) -> TrajectoryPlan:
        start_angles: list[float] | None = None
        task_waypoints: list[list[float]] = []
        velocity_percent = 50.0

        for row in rows:
            normalized = {self._normalize_header(key): value for key, value in row.items()}
            phase = str(normalized.get("phase", "")).strip().upper()
            if not phase and any(key in normalized for key in ("j1", "j2", "j3", "j4", "j5", "j6", "a1", "a2", "a3", "a4", "a5", "a6")):
                phase = "TASK"

            waypoint = self._parse_joint_row(normalized)
            if waypoint is None:
                continue

            if "velocity" in normalized or "velocity_percent" in normalized:
                velocity_value = self._coerce_float(normalized.get("velocity", normalized.get("velocity_percent")))
                if velocity_value is not None:
                    velocity_percent = velocity_value
            elif "velocity_percent" in normalized:
                velocity_value = self._coerce_float(normalized.get("velocity_percent"))
                if velocity_value is not None:
                    velocity_percent = velocity_value

            if phase == "PREPARATION":
                if start_angles is None:
                    start_angles = waypoint
                continue

            if phase in {"TASK", "EXECUTION"} or phase in {"", "MOVE"}:
                task_waypoints.append(waypoint)

        if start_angles is None and task_waypoints:
            start_angles = task_waypoints[0]
            task_waypoints = task_waypoints[1:]

        if start_angles is None:
            raise ValueError("Trajectory CSV does not define a start position.")

        if not task_waypoints:
            task_waypoints = [start_angles]

        plan = TrajectoryPlan(
            file_path=file_path,
            trajectory_id=file_path.stem,
            start_angles=list(start_angles),
            task_waypoints=[list(point) for point in task_waypoints],
            velocity_percent=float(velocity_percent),
            legacy_format=False,
            metadata={"format": "phase"},
        )
        self._set_loaded_path(plan.task_waypoints, plan.start_angles)
        self.current_trajectory = plan
        return plan

    def _parse_legacy_trajectory(self, file_path: Path, rows: list[dict[str, Any]]) -> TrajectoryPlan:
        parsed_points: list[list[float]] = []
        velocity_percent = 50.0

        for row in rows:
            normalized = {self._normalize_header(key): value for key, value in row.items()}
            if "cri_command" in normalized:
                command = normalized.get("cri_command")
                if command is None:
                    continue
                waypoint, command_velocity = self._parse_legacy_cri_command(str(command))
                parsed_points.append(waypoint)
                if command_velocity is not None:
                    velocity_percent = command_velocity
                continue

            waypoint = self._parse_joint_row(normalized)
            if waypoint is not None:
                parsed_points.append(waypoint)
                for key in ("velocity", "velocity_percent"):
                    if key in normalized:
                        value = self._coerce_float(normalized.get(key))
                        if value is not None:
                            velocity_percent = value

        if not parsed_points:
            raise ValueError("Trajectory CSV does not contain any valid waypoints.")

        plan = TrajectoryPlan(
            file_path=file_path,
            trajectory_id=file_path.stem,
            start_angles=list(parsed_points[0]),
            task_waypoints=[list(point) for point in parsed_points],
            velocity_percent=float(velocity_percent),
            legacy_format=True,
            metadata={"format": "legacy"},
        )
        self._set_loaded_path(plan.task_waypoints, plan.start_angles)
        self.current_trajectory = plan
        return plan

    def load_trajectory_csv(self, filepath: str | Path) -> TrajectoryPlan:
        """Load a trajectory file from disk.

        Supported formats:
        - Standard phase-aware CSV: Phase,Step,J1 (deg),...,Velocity (%)
        - Legacy CSV: Step,CRI Command
        - Legacy joint-only CSV: Step,J1 (deg),...,Velocity (%)
        """
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"Trajectory file does not exist: {path}")

        with path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                raise ValueError(f"Trajectory file is empty or missing a header row: {path}")
            rows = [dict(row) for row in reader]

        fieldnames = [self._normalize_header(header) for header in reader.fieldnames]
        normalized_rows = [{self._normalize_header(key): value for key, value in row.items()} for row in rows]

        if "phase" in fieldnames or any(
            key in fieldnames for key in ("j1", "j2", "j3", "j4", "j5", "j6", "a1", "a2", "a3", "a4", "a5", "a6")
        ) and "phase" in fieldnames:
            return self._parse_phase_trajectory(path, normalized_rows)

        if "cri_command" in fieldnames or "step" in fieldnames and any(
            key in fieldnames for key in ("cri_command", "j1", "j2", "j3", "j4", "j5", "j6", "a1", "a2", "a3", "a4", "a5", "a6")
        ):
            if "cri_command" in fieldnames:
                return self._parse_legacy_trajectory(path, normalized_rows)
            return self._parse_legacy_trajectory(path, normalized_rows)

        raise ValueError(f"Unsupported trajectory CSV schema: {path}")

    def export_to_csv_joint_angles(self, filepath, velocity_percent=50.0):
        """Export ordered joint targets and the controller velocity setting."""
        with open(filepath, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Step", "J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)", "J5 (deg)", "J6 (deg)", "Velocity (%)"])
            for index, angles in enumerate(self.path_data):
                writer.writerow([index, *[f"{angle:.4f}" for angle in angles], velocity_percent])

    def export_trajectory_csv(self, filepath, velocity_percent=50.0, start_angles: Iterable[float] | None = None):
        """Export a phase-aware CSV that separates preparation from the measured task path."""
        if not self.path_data:
            raise ValueError("No path data available to export.")

        base_path = Path(filepath)
        base_path.parent.mkdir(parents=True, exist_ok=True)
        target_start = list(start_angles) if start_angles is not None else list(self.start_angles if self.start_angles else self.path_data[0])
        task_waypoints = list(self.path_data)
        if self.start_angles and self.path_data and list(self.start_angles) == list(self.path_data[0]):
            task_waypoints = self.path_data[1:] if len(self.path_data) > 1 else []

        with base_path.open("w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Phase", "Step", "J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)", "J5 (deg)", "J6 (deg)", "Velocity (%)"])
            writer.writerow(["PREPARATION", 0, *[f"{angle:.4f}" for angle in target_start], float(velocity_percent)])
            for index, waypoint in enumerate(task_waypoints, start=1):
                writer.writerow(["TASK", index, *[f"{angle:.4f}" for angle in waypoint], float(velocity_percent)])

    def export_to_csv_cri_commands(self, filepath, velocity_percent=50.0):
        """Export one CRI ``CMD Move Joint`` payload for each path waypoint."""
        with open(filepath, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Step", "CRI Command"])
            for index, angles in enumerate(self.path_data):
                command = self._format_cri_move(angles, velocity_percent)
                writer.writerow([index, command])

    def export_to_json(self, filepath, velocity_percent=50.0):
        """Export raw CRI joint target payload fields as JSON."""
        waypoints = []
        for index, angles in enumerate(self.path_data):
            waypoints.append({
                "step": index,
                "joint_angles": dict(zip(("A1", "A2", "A3", "A4", "A5", "A6"), angles)),
                "end_effector_axes": {"E1": 0.0, "E2": 0.0, "E3": 0.0},
                "velocity_percent": velocity_percent,
            })
        with open(filepath, "w") as jsonfile:
            json.dump({"waypoints": waypoints}, jsonfile, indent=2)

    def generate_summary_report(self):
        """Generate a compact spatial range summary for the loaded path."""
        if not self.path_data:
            return "No path data loaded."
        angles_array = np.asarray(self.path_data)
        summary = f"\n=== PATH SUMMARY ===\nTotal Waypoints: {len(self.path_data)}\n\nJoint Angle Ranges:\n"
        for joint_index in range(6):
            minimum = np.min(angles_array[:, joint_index])
            maximum = np.max(angles_array[:, joint_index])
            summary += f"  J{joint_index + 1}: [{minimum:.2f}, {maximum:.2f}] (range: {maximum - minimum:.2f})\n"
        return summary

    @staticmethod
    def _format_cri_move(angles, velocity_percent):
        return (
            f"CMD Move Joint {angles[0]:.4f} {angles[1]:.4f} {angles[2]:.4f} "
            f"{angles[3]:.4f} {angles[4]:.4f} {angles[5]:.4f} 0.0 0.0 0.0 {velocity_percent:.1f}"
        )
