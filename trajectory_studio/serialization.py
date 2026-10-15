"""CSV import/export (execution-compatible) and recipe JSON save/load."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any

from trajectory_studio.models import Trajectory, Waypoint

CSV_HEADER = ["Phase", "Step", "J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)", "J5 (deg)", "J6 (deg)", "Velocity (%)"]
RECIPE_SCHEMA_VERSION = 1


class CsvImportError(ValueError):
    pass


def sha256_file(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _norm(header: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(header or "").lower()).strip("_")


def import_csv(path: str | Path) -> tuple[Trajectory, list[str]]:
    """Read an execution CSV without altering values. Returns (trajectory, warnings).

    Phase-aware files keep every row's phase. Anything the executor would
    reinterpret (missing PREPARATION, other phase names, mixed speeds) is
    reported as a warning rather than silently changed.
    """
    path = Path(path)
    if not path.is_file():
        raise CsvImportError(f"file not found: {path}")
    warnings: list[str] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise CsvImportError("file is empty or has no header")
        fields = {_norm(name): name for name in reader.fieldnames}
        joint_keys = []
        for j in range(1, 7):
            key = fields.get(f"j{j}_deg") or fields.get(f"j{j}") or fields.get(f"a{j}")
            if key is None:
                raise CsvImportError(
                    f"column for J{j} not found (expected 'J{j} (deg)'); legacy 'CRI Command' files "
                    "must be loaded through the Pathways tab"
                )
            joint_keys.append(key)
        phase_key = fields.get("phase")
        vel_key = fields.get("velocity") or fields.get("velocity_percent")
        waypoints: list[Waypoint] = []
        velocities: list[float] = []
        for line_no, row in enumerate(reader, start=2):
            try:
                joints = [float(str(row[k]).strip()) for k in joint_keys]
            except (TypeError, ValueError):
                raise CsvImportError(f"line {line_no}: joint values must be numeric") from None
            if not all(math.isfinite(v) for v in joints):
                raise CsvImportError(f"line {line_no}: joint values must be finite")
            raw_phase = str(row.get(phase_key, "") if phase_key else "").strip().upper()
            phase = raw_phase
            if raw_phase in {"", "EXECUTION", "MOVE"}:
                phase = "TASK"
                warnings.append(f"line {line_no}: phase '{raw_phase or '(blank)'}' is read by the executor as TASK")
            if vel_key and str(row.get(vel_key, "")).strip():
                try:
                    velocities.append(float(row[vel_key]))
                except ValueError:
                    raise CsvImportError(f"line {line_no}: velocity must be numeric") from None
            waypoints.append(Waypoint(phase, joints))
    if not waypoints:
        raise CsvImportError("file contains no waypoints")
    if not any(w.phase == "PREPARATION" for w in waypoints):
        waypoints[0].phase = "PREPARATION"
        warnings.append("no PREPARATION row: the executor uses the first row as the start pose; shown as PREPARATION")
    velocity = velocities[-1] if velocities else None
    if len(set(velocities)) > 1:
        warnings.append("rows carry different velocities; the executor applies only the last value "
                        f"({velocity}). Per-waypoint speed is not representable.")
    if velocity is None:
        warnings.append("no velocity column; set a velocity before export")
    traj = Trajectory(name=path.stem, kind="imported", waypoints=waypoints, velocity_percent=velocity,
                      source_csv_sha256=sha256_file(path))
    traj.generation = {"template": "imported", "source_file": path.name}
    return traj, warnings


def export_csv(traj: Trajectory, path: str | Path) -> Path:
    """Write the existing execution-compatible format. Refuses unusable input;
    does not validate safety (see validators)."""
    if traj.velocity_percent is None or not math.isfinite(traj.velocity_percent):
        raise ValueError("velocity_percent is not set")
    if not traj.waypoints:
        raise ValueError("trajectory has no waypoints")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    task_step = 0
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_HEADER)
        for w in traj.waypoints:
            if w.phase == "PREPARATION":
                step = 0
            else:
                task_step += 1
                step = task_step
            writer.writerow([w.phase, step, *[f"{v:.4f}" for v in w.joints], float(traj.velocity_percent)])
    return path


def recipe_to_dict(traj: Trajectory, software_revision: str = "") -> dict[str, Any]:
    return {
        "schema_version": RECIPE_SCHEMA_VERSION,
        "recipe_id": traj.name,
        "recipe_version": traj.version,
        "description": traj.description,
        "category": traj.category,
        "kind": traj.kind,
        "validation_status": traj.validation_status,
        "motion_mode": traj.motion_mode,
        "units": {"joints": "deg", "velocity": "controller-command percent (physical speed not verified)"},
        "velocity_percent": traj.velocity_percent,
        "waypoints": [w.to_dict() for w in traj.waypoints],
        "joint_limits_deg": traj.joint_limits_deg,
        "limits_source": traj.limits_source or "none declared (repository has no authoritative joint limits)",
        "locked_joints": traj.locked_joints,
        "generation": traj.generation,
        "experiment": traj.experiment,
        "approval": traj.approval,
        "exported_csv_sha256": traj.source_csv_sha256,
        "software_revision": software_revision,
        "related_run_ids": traj.experiment.get("related_run_ids", []),
    }


def recipe_from_dict(data: dict[str, Any]) -> Trajectory:
    if data.get("schema_version") != RECIPE_SCHEMA_VERSION:
        raise ValueError(f"unsupported recipe schema_version {data.get('schema_version')!r}")
    traj = Trajectory(
        name=data["recipe_id"], description=data.get("description", ""),
        category=data.get("category", "General Robot Pathways"), kind=data.get("kind", "manual"),
        version=int(data.get("recipe_version", 1)), validation_status=data.get("validation_status", "DRAFT"),
        waypoints=[Waypoint.from_dict(w) for w in data.get("waypoints", [])],
        velocity_percent=data.get("velocity_percent"),
        joint_limits_deg=data.get("joint_limits_deg") or [None] * 6,
        limits_source="" if str(data.get("limits_source", "")).startswith("none declared") else data.get("limits_source", ""),
        locked_joints=data.get("locked_joints") or [False] * 6,
        motion_mode=data.get("motion_mode", "EXECUTABLE"),
        generation=data.get("generation", {}), experiment=data.get("experiment", {}),
        approval=data.get("approval", {"approved": False, "by": "", "date": ""}),
        source_csv_sha256=data.get("exported_csv_sha256", ""),
    )
    return traj


def save_recipe(traj: Trajectory, path: str | Path, software_revision: str = "") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(recipe_to_dict(traj, software_revision), indent=2), encoding="utf-8")
    return path


def load_recipe(path: str | Path) -> Trajectory:
    return recipe_from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def save_new_revision(traj: Trajectory, directory: str | Path, software_revision: str = "") -> Path:
    """Write <name>.v<N>.recipe.json using the next free version; earlier files are never overwritten."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    existing = [int(m.group(1)) for p in directory.glob(f"{traj.name}.v*.recipe.json")
                if (m := re.search(r"\.v(\d+)\.recipe\.json$", p.name))]
    traj.version = max(existing + [traj.version - 1]) + 1
    return save_recipe(traj, directory / f"{traj.name}.v{traj.version}.recipe.json", software_revision)
