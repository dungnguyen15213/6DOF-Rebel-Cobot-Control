"""Authoring templates. They only build waypoint lists; nothing is sent to a robot.

Generated points are linear joint-space interpolation for authoring, not a
validated smooth motion profile. No joint angle is ever defaulted: every pose,
target and speed must come from the operator.
"""

from __future__ import annotations

import math
from typing import Sequence

from trajectory_studio.models import (
    MOTION_EXECUTABLE, MOTION_NONE, Trajectory, Waypoint,
)


class IncompleteTemplate(ValueError):
    """Raised when operator-supplied parameters are missing."""


def _pose(values: Sequence[float] | None, label: str) -> list[float]:
    if values is None or len(values) != 6 or any(v is None for v in values):
        raise IncompleteTemplate(f"{label}: six explicit joint values (deg) are required")
    out = [float(v) for v in values]
    if not all(math.isfinite(v) for v in out):
        raise IncompleteTemplate(f"{label}: values must be finite")
    return out


def _require(value, label):
    if value is None:
        raise IncompleteTemplate(f"{label} is required")
    return value


def interpolate(a: Sequence[float], b: Sequence[float], segments: int) -> list[list[float]]:
    """Linear points after `a` up to and including `b` (`segments` >= 1)."""
    if segments < 1:
        raise ValueError("segments must be >= 1")
    return [[x + (y - x) * k / segments for x, y in zip(a, b)] for k in range(1, segments + 1)]


def _build(name: str, kind: str, stops: list[list[float]], segments: int, velocity: float | None,
           generation: dict, category: str, description: str = "") -> Trajectory:
    waypoints = [Waypoint("PREPARATION", list(stops[0]))]
    for a, b in zip(stops, stops[1:]):
        for point in interpolate(a, b, segments):
            waypoints.append(Waypoint("TASK", point))
    traj = Trajectory(name=name, description=description, category=category, kind=kind,
                      waypoints=waypoints, velocity_percent=velocity, generation=generation)
    # Make lock state visible: every joint that never moves is "fixed" in this recipe.
    moving = set(traj.moving_joints())
    traj.locked_joints = [j not in moving for j in range(6)]
    return traj


def _with_joint(start: list[float], joint: int, value: float) -> list[float]:
    pose = list(start)
    pose[joint] = float(value)
    return pose


def single_joint_sweep(name: str, joint: int, start_pose: Sequence[float] | None, target_deg: float | None,
                       segments: int = 4, velocity: float | None = None, return_to_start: bool = True) -> Trajectory:
    """Template A. `joint` is zero-based; other joints stay at start_pose."""
    start = _pose(start_pose, "start pose")
    target = float(_require(target_deg, "target angle"))
    stops = [start, _with_joint(start, joint, target)]
    if return_to_start:
        stops.append(list(start))
    gen = {"template": "single_joint_sweep", "joint": joint + 1, "start_pose": start, "target_deg": target,
           "segments_per_leg": segments, "return_to_start": return_to_start, "interpolation": "linear joint-space (authoring)"}
    return _build(name, "single_joint", stops, segments, velocity, gen, "Single-Joint Identification")


def bidirectional_sweep(name: str, joint: int, start_pose: Sequence[float] | None,
                        positive_deg: float | None, negative_deg: float | None, segments: int = 4,
                        velocity: float | None = None, cycles: int = 1) -> Trajectory:
    """Template B: Start -> positive -> Start -> negative -> Start (either side may be None/omitted
    when that direction is not approved)."""
    start = _pose(start_pose, "start pose")
    if positive_deg is None and negative_deg is None:
        raise IncompleteTemplate("at least one of positive/negative target is required")
    if cycles < 1:
        raise ValueError("cycles must be >= 1")
    legs: list[list[float]] = []
    for target in (positive_deg, negative_deg):
        if target is not None:
            legs += [_with_joint(start, joint, target), list(start)]
    stops = [list(start)] + legs * cycles
    gen = {"template": "bidirectional", "joint": joint + 1, "start_pose": start, "positive_deg": positive_deg,
           "negative_deg": negative_deg, "segments_per_leg": segments, "cycles": cycles,
           "interpolation": "linear joint-space (authoring)"}
    return _build(name, "bidirectional", stops, segments, velocity, gen, "Single-Joint Identification")


def coordinated_motion(name: str, start_pose: Sequence[float] | None, positive_targets: dict[int, float],
                       negative_targets: dict[int, float] | None = None, segments: int = 4,
                       velocity: float | None = None, cycles: int = 1) -> Trajectory:
    """Template D: several joints move together; every target is explicit and nothing is inferred."""
    start = _pose(start_pose, "start pose")
    if not positive_targets and not negative_targets:
        raise IncompleteTemplate("at least one joint target is required")
    legs: list[list[float]] = []
    for targets in (positive_targets, negative_targets):
        if targets:
            pose = list(start)
            for joint, value in targets.items():
                pose[joint] = float(value)
            legs += [pose, list(start)]
    stops = [list(start)] + legs * cycles
    gen = {"template": "coordinated", "joints": sorted(j + 1 for j in {*positive_targets, *(negative_targets or {})}),
           "start_pose": start, "positive_targets": {str(j + 1): v for j, v in positive_targets.items()},
           "negative_targets": {str(j + 1): v for j, v in (negative_targets or {}).items()},
           "segments_per_leg": segments, "cycles": cycles, "collision_free": "NOT VERIFIED"}
    return _build(name, "coordinated", stops, segments, velocity, gen, "Multi-Joint Identification")


def stationary_hold(name: str, hold_pose: Sequence[float] | None = None) -> Trajectory:
    """Template F: a no-motion recipe. It has no waypoints and cannot be exported as a CSV,
    so it cannot command movement. Record the baseline with the Identification recorder."""
    traj = Trajectory(name=name, kind="stationary", category="Stationary Baselines", motion_mode=MOTION_NONE,
                      description="No-motion baseline: start identification recording with the robot at rest; "
                                  "do not run a trajectory.")
    traj.generation = {"template": "stationary_hold", "hold_pose": list(hold_pose) if hold_pose else None}
    return traj


def multi_speed_variants(base: Trajectory, speeds: Sequence[float]) -> list[Trajectory]:
    """Template C: copies of an approved trajectory at operator-chosen speeds.
    Speeds are controller-command percentages; one physical run per variant."""
    out = []
    for speed in speeds:
        variant = base.copy()
        variant.name = f"{base.name}_V{int(round(speed)):03d}"
        variant.velocity_percent = float(speed)
        variant.version = 1
        variant.validation_status = "DRAFT"
        variant.approval = {"approved": False, "by": "", "date": ""}
        variant.generation = {**base.generation, "speed_variant_of": base.name,
                              "speed_meaning": "controller-command percent (physical speed not verified)"}
        out.append(variant)
    return out


def test_b_template(joint: int) -> Trajectory:
    """TEST_B_J4_BIDIRECTIONAL / TEST_B_J6_BIDIRECTIONAL: deliberately non-executable until the
    operator supplies a verified start pose, displacements and speed (see complete_template)."""
    traj = Trajectory(
        name=f"TEST_B_J{joint}_BIDIRECTIONAL", kind="test_b_template", category="Joint Validation",
        description=f"Operator must supply start pose, +/- displacement and speed. Moves J{joint} only; "
                    "purpose: observe J{0} current response under controlled motion, not final identification.".format(joint),
    )
    traj.generation = {"template": "bidirectional", "joint": joint, "start_pose": None, "positive_deg": None,
                       "negative_deg": None, "segments_per_leg": 4, "cycles": 1, "requires_operator_completion": True}
    return traj


def complete_template(traj: Trajectory, **overrides) -> Trajectory:
    """Fill a template (generation parameters + operator values) and rebuild the waypoints.
    Raises IncompleteTemplate if anything required is still missing."""
    gen = {**traj.generation, **overrides}
    template = gen.get("template")
    joint = int(gen["joint"]) - 1 if "joint" in gen else None
    velocity = overrides.get("velocity", traj.velocity_percent)
    if template == "bidirectional":
        built = bidirectional_sweep(traj.name, joint, gen.get("start_pose"), gen.get("positive_deg"),
                                    gen.get("negative_deg"), int(gen.get("segments_per_leg", 4)),
                                    _require(velocity, "velocity (%)"), int(gen.get("cycles", 1)))
    elif template == "single_joint_sweep":
        built = single_joint_sweep(traj.name, joint, gen.get("start_pose"), gen.get("target_deg"),
                                   int(gen.get("segments_per_leg", 4)), _require(velocity, "velocity (%)"),
                                   bool(gen.get("return_to_start", True)))
    else:
        raise IncompleteTemplate(f"template {template!r} cannot be regenerated")
    built.category = traj.category
    built.kind = traj.kind if traj.kind == "test_b_template" else built.kind
    built.description = traj.description
    built.experiment = dict(traj.experiment)
    built.joint_limits_deg = [list(x) if x else None for x in traj.joint_limits_deg]
    built.limits_source = traj.limits_source
    built.version = traj.version
    built.generation = {**built.generation, **{k: v for k, v in gen.items() if k == "requires_operator_completion"}}
    built.generation["requires_operator_completion"] = False
    return built
