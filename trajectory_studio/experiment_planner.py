"""Research experiment planner: campaign manifest, planned-excitation coverage, traceability.

Coverage here is an experiment-design aid computed from planned waypoints. It is
not measured coverage and does not establish statistical identifiability.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from trajectory_studio.models import Trajectory
from trajectory_studio.serialization import recipe_to_dict, sha256_file

PLANNED_ONLY = "PLANNED (not measured)"
ROLES = ("UNASSIGNED", "TRAIN", "VALIDATION", "HOLDOUT")


def default_experiment_metadata(experiment_id: str, trajectory_id: str, planned_runs: int = 2,
                                speed_label: str = "", speed_percent: float | None = None) -> dict[str, Any]:
    return {
        "experiment_id": experiment_id, "trajectory_id": trajectory_id, "planned_runs": planned_runs,
        "run_ids": [f"{experiment_id}_{trajectory_id}_R{n:02d}" for n in range(1, planned_runs + 1)],
        "speed_label": speed_label, "commanded_speed_percent": speed_percent,
        "payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE", "thermal_condition": "COLD",
        "operator_notes": "", "approved_initial_pose": None, "intended_directions": [],
        "role": "UNASSIGNED", "related_run_ids": [],
    }


# Research-design starting plan. No angles, poses or speeds are supplied: the operator completes them.
CAMPAIGN_PLAN = (
    [(f"T{n:02d}", "bidirectional", [n], "low") for n in range(1, 7)]
    + [(f"T{n:02d}", "bidirectional", [n - 6], "medium") for n in range(7, 13)]
    + [("T13", "coordinated", [2, 3], ""), ("T14", "coordinated", [4, 5, 6], ""),
       ("T15", "coordinated", [1, 2, 3, 4, 5, 6], "")]
)


@dataclass
class Campaign:
    experiment_id: str = "ID_NORMAL_001"
    name: str = "identification_campaign"
    recipes: list[Trajectory] = field(default_factory=list)

    @staticmethod
    def default_plan(experiment_id: str = "ID_NORMAL_001", repeats: int = 2) -> "Campaign":
        c = Campaign(experiment_id=experiment_id)
        for tid, kind, joints, speed_label in CAMPAIGN_PLAN:
            if kind == "bidirectional":
                j = joints[0]
                traj = Trajectory(name=f"{tid}_J{j}_BIDIR_{speed_label.upper()}", kind="test_b_template",
                                  category="Single-Joint Identification")
                traj.generation = {"template": "bidirectional", "joint": j, "start_pose": None, "positive_deg": None,
                                   "negative_deg": None, "segments_per_leg": 4, "cycles": 1,
                                   "requires_operator_completion": True}
            else:
                traj = Trajectory(name=f"{tid}_COORD_" + "".join(f"J{j}" for j in joints), kind="coordinated",
                                  category="Multi-Joint Identification")
                traj.generation = {"template": "coordinated", "joints": joints, "start_pose": None,
                                   "requires_operator_completion": True}
            traj.description = "Research-design template; angles/speed must be supplied and approved by the operator."
            traj.experiment = default_experiment_metadata(experiment_id, tid, repeats, speed_label)
            traj.experiment["planned_moving_joints"] = joints
            c.recipes.append(traj)
        return c

    def manifest(self) -> dict[str, Any]:
        return {
            "schema_version": 1, "experiment_id": self.experiment_id, "name": self.name,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "note": "Research-design plan, not a set of safety-certified movements. Planned values only.",
            "recipes": [{
                "trajectory_id": r.experiment.get("trajectory_id"), "recipe_id": r.name, "kind": r.kind,
                "recipe_version": r.version, "planned_runs": r.experiment.get("planned_runs"),
                "run_ids": r.experiment.get("run_ids"), "speed_label": r.experiment.get("speed_label"),
                "commanded_speed_percent": r.experiment.get("commanded_speed_percent"),
                "payload_g": r.experiment.get("payload_g"), "contact_label": r.experiment.get("contact_label"),
                "grasp_context": r.experiment.get("grasp_context"), "thermal_condition": r.experiment.get("thermal_condition"),
                "role": r.experiment.get("role"), "planned_moving_joints": r.experiment.get("planned_moving_joints", [j + 1 for j in r.moving_joints()]),
                "waypoints_defined": len(r.waypoints), "requires_operator_completion":
                    bool(r.generation.get("requires_operator_completion")),
                "operator_approved": bool(r.approval.get("approved")),
            } for r in self.recipes],
        }

    def save(self, directory: str | Path) -> Path:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        for r in self.recipes:
            (directory / f"{r.experiment.get('trajectory_id', r.name)}_{r.name}.recipe.json").write_text(
                json.dumps(recipe_to_dict(r), indent=2), encoding="utf-8")
        path = directory / "campaign_manifest.json"
        path.write_text(json.dumps(self.manifest(), indent=2), encoding="utf-8")
        return path


def coverage(campaign: Campaign) -> dict[str, Any]:
    """Planned excitation per joint from recipes that have waypoints; shells count as planned-only."""
    rows = []
    for j in range(6):
        pos = neg = 0.0
        lo, hi = math.inf, -math.inf
        speeds: set[float] = set()
        labels: set[str] = set()
        combos: set[tuple[int, ...]] = set()
        runs = 0
        planned_shells = 0
        for r in campaign.recipes:
            moving = r.moving_joints()
            planned = r.experiment.get("planned_moving_joints", [])
            if j not in moving:
                if (j + 1) in planned and not r.waypoints:
                    planned_shells += 1
                continue
            base = r.waypoints[0].joints[j]
            values = [w.joints[j] for w in r.waypoints]
            lo, hi = min(lo, min(values)), max(hi, max(values))
            pos, neg = max(pos, max(values) - base), max(neg, base - min(values))
            if r.velocity_percent is not None:
                speeds.add(r.velocity_percent)
            if r.experiment.get("speed_label"):
                labels.add(r.experiment["speed_label"])
            combos.add(tuple(m + 1 for m in moving))
            runs += int(r.experiment.get("planned_runs", 0))
        missing = []
        if hi < lo:
            missing.append("no completed recipe moves this joint" + (f" ({planned_shells} template(s) await operator values)" if planned_shells else ""))
        else:
            if pos <= 0:
                missing.append("no positive excursion")
            if neg <= 0:
                missing.append("no negative excursion")
            if len(speeds) < 2:
                missing.append("fewer than two distinct speed settings")
            if runs < 2:
                missing.append("fewer than two planned independent runs")
        rows.append({
            "joint": f"J{j + 1}", "planned_min_deg": None if hi < lo else lo, "planned_max_deg": None if hi < lo else hi,
            "positive_travel_deg": pos, "negative_travel_deg": neg, "speed_settings_percent": sorted(speeds),
            "speed_labels": sorted(labels), "planned_independent_runs": runs,
            "joint_combinations": sorted(combos), "templates_awaiting_values": planned_shells, "missing": missing,
        })
    return {"basis": PLANNED_ONLY, "caveat": "Design aid only; not proof of identifiability or measured coverage.",
            "joints": rows}


def link_run(recipe_path: str | Path, run_csv: str | Path, links_path: str | Path) -> dict[str, Any]:
    """Append a recipe->run link to a separate JSON file; run CSVs and sidecars are only read."""
    recipe_path, run_csv, links_path = Path(recipe_path), Path(run_csv), Path(links_path)
    recipe = json.loads(recipe_path.read_text(encoding="utf-8"))
    sidecar = Path(str(run_csv) + ".json")
    run_id = ""
    if sidecar.is_file():
        run_id = str(json.loads(sidecar.read_text(encoding="utf-8")).get("metadata", {}).get("run_id", ""))
    link = {
        "recipe_id": recipe["recipe_id"], "recipe_version": recipe["recipe_version"],
        "recipe_sha256": hashlib.sha256(recipe_path.read_bytes()).hexdigest(),
        "exported_csv_sha256": recipe.get("exported_csv_sha256", ""),
        "run_csv": run_csv.name, "run_csv_sha256": sha256_file(run_csv), "run_id": run_id,
        "linked_utc": datetime.now(timezone.utc).isoformat(),
    }
    links = json.loads(links_path.read_text(encoding="utf-8")) if links_path.is_file() else []
    links.append(link)
    links_path.write_text(json.dumps(links, indent=2), encoding="utf-8")
    return link


def compare_planned_recorded(traj: Trajectory, run_csv: str | Path) -> dict[str, Any]:
    """Offline range comparison of planned waypoints against recorded q1..q6 and currents.
    No time alignment is claimed (the planner has no verified time base)."""
    with open(run_csv, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("run CSV has no rows")
    out = {"basis": "range comparison only; no time alignment", "joints": []}
    for j in range(6):
        q = [float(r[f"q{j + 1}"]) for r in rows if r.get(f"q{j + 1}") not in (None, "")]
        cur = [float(r[f"current{j + 1}"]) for r in rows if r.get(f"current{j + 1}") not in (None, "", "nan")]
        planned = [w.joints[j] for w in traj.waypoints]
        out["joints"].append({
            "joint": f"J{j + 1}",
            "planned_min": min(planned) if planned else None, "planned_max": max(planned) if planned else None,
            "recorded_min": min(q) if q else None, "recorded_max": max(q) if q else None,
            "recorded_current_min_mA": min(cur) if cur else None, "recorded_current_max_mA": max(cur) if cur else None,
            "range_discrepancy_deg": (None if not (q and planned) else
                                      (max(q) - min(q)) - (max(planned) - min(planned))),
        })
    return out
