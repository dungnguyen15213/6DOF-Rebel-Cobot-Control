"""Static validation. Never modifies the trajectory and never contacts the robot."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from trajectory_studio.models import MOTION_NONE, PHASES, Trajectory

ERROR, WARNING, INFO, NOT_VERIFIED = "ERROR", "WARNING", "INFO", "NOT VERIFIED"
VELOCITY_RANGE = (1.0, 100.0)  # the range of the existing Motion-speed control (UI range, not a verified controller limit)
NOT_VERIFIED_TEXT = "NOT VERIFIED - requires operator/hardware validation"


@dataclass
class Finding:
    severity: str
    code: str
    message: str
    waypoint: int | None = None


@dataclass
class ValidationReport:
    findings: list[Finding] = field(default_factory=list)

    def add(self, severity, code, message, waypoint=None):
        self.findings.append(Finding(severity, code, message, waypoint))

    @property
    def errors(self):
        return [f for f in self.findings if f.severity == ERROR]

    @property
    def exportable(self) -> bool:
        return not self.errors

    def has(self, code: str) -> bool:
        return any(f.code == code for f in self.findings)

    def summary(self) -> str:
        counts = {s: sum(1 for f in self.findings if f.severity == s) for s in (ERROR, WARNING, NOT_VERIFIED, INFO)}
        head = "EXPORTABLE (format only)" if self.exportable else "BLOCKED"
        return head + " | " + ", ".join(f"{n} {s}" for s, n in counts.items())

    def to_text(self) -> str:
        return "\n".join([self.summary(), ""] + [
            f"[{f.severity}] {f.code}" + (f" @row {f.waypoint}" if f.waypoint is not None else "") + f": {f.message}"
            for f in self.findings])


def validate(traj: Trajectory) -> ValidationReport:
    r = ValidationReport()
    if traj.motion_mode == MOTION_NONE:
        r.add(ERROR, "NO_MOTION_RECIPE", "Stationary recipe commands no motion and has no CSV to execute. "
              "Record the baseline with the identification recorder while the robot is at rest.")
        return r
    if traj.generation.get("requires_operator_completion"):
        r.add(ERROR, "TEMPLATE_INCOMPLETE", "Template needs operator-supplied start pose, displacements and speed "
              "before it can be exported.")
    if not traj.waypoints:
        r.add(ERROR, "EMPTY", "Trajectory has no waypoints.")
        return r
    preps = [i for i, w in enumerate(traj.waypoints) if w.phase == "PREPARATION"]
    for i, w in enumerate(traj.waypoints):
        if w.phase not in PHASES:
            r.add(ERROR, "BAD_PHASE", f"Phase {w.phase!r} is not PREPARATION or TASK.", i)
        if len(w.joints) != 6:
            r.add(ERROR, "BAD_JOINT_COUNT", f"Expected 6 joint values, found {len(w.joints)}.", i)
        elif not all(isinstance(v, (int, float)) and math.isfinite(v) for v in w.joints):
            r.add(ERROR, "NON_FINITE", "Joint values must be finite numbers.", i)
    if len(preps) != 1:
        r.add(ERROR, "PREPARATION_COUNT", f"Exactly one PREPARATION row is required (found {len(preps)}).")
    elif preps[0] != 0:
        r.add(ERROR, "PREPARATION_ORDER", "PREPARATION must be the first row.", preps[0])
    if len(traj.waypoints) < 2:
        r.add(ERROR, "NO_TASK_ROWS", "No TASK motion after the PREPARATION pose.")
    v = traj.velocity_percent
    if v is None or not math.isfinite(v):
        r.add(ERROR, "VELOCITY_MISSING", "Velocity (%) is not set.")
    elif not VELOCITY_RANGE[0] <= v <= VELOCITY_RANGE[1]:
        r.add(ERROR, "VELOCITY_RANGE", f"Velocity {v} is outside the accepted range {VELOCITY_RANGE} of the Motion-speed control.")
    if r.has("BAD_JOINT_COUNT") or r.has("NON_FINITE"):
        return r

    # Joint limits: only operator-declared limits can be checked.
    declared = [lim for lim in traj.joint_limits_deg if lim]
    if not declared:
        r.add(NOT_VERIFIED, "JOINT_LIMITS", "No joint limits declared; the repository holds no authoritative limits. "
              + NOT_VERIFIED_TEXT)
    else:
        r.add(WARNING, "LIMITS_OPERATOR_DECLARED", "Joint limits are operator-declared"
              + (f" ({traj.limits_source})" if traj.limits_source else "") + ", not verified by this software.")
        for j in range(6):
            lim = traj.joint_limits_deg[j]
            if not lim:
                r.add(NOT_VERIFIED, "JOINT_LIMIT_MISSING", f"J{j + 1} has no declared limit.")
                continue
            lo, hi = lim
            for i, w in enumerate(traj.waypoints):
                if not lo <= w.joints[j] <= hi:
                    r.add(ERROR, "LIMIT_VIOLATION", f"J{j + 1}={w.joints[j]:.4f} outside declared [{lo}, {hi}].", i)
    # Adjacent displacement (information only: no verified per-step limit exists).
    for i in range(1, len(traj.waypoints)):
        a, b = traj.waypoints[i - 1].joints, traj.waypoints[i].joints
        if a == b:
            r.add(WARNING, "DUPLICATE_WAYPOINT", "Identical to the previous waypoint (redundant).", i)
    if len(traj.waypoints) > 1:
        steps = [max(abs(x - y) for x, y in zip(traj.waypoints[i - 1].joints, traj.waypoints[i].joints))
                 for i in range(1, len(traj.waypoints))]
        k = max(range(len(steps)), key=steps.__getitem__)
        r.add(INFO, "MAX_STEP", f"Largest single-joint step between adjacent waypoints: {steps[k]:.3f} deg (rows {k}->{k + 1}).", k + 1)
        r.add(NOT_VERIFIED, "STEP_LIMIT", "No verified maximum step size. " + NOT_VERIFIED_TEXT)
    if not traj.moving_joints():
        r.add(WARNING, "NO_MOVEMENT", "No joint changes value anywhere in the trajectory.")
    # Fixed joints should stay constant across the whole file when locked.
    for j, locked in enumerate(traj.locked_joints):
        if locked and j in traj.moving_joints():
            r.add(ERROR, "LOCKED_JOINT_MOVES", f"J{j + 1} is marked locked but changes value.")
    r.add(NOT_VERIFIED, "FEASIBILITY", "Velocity/acceleration feasibility. " + NOT_VERIFIED_TEXT)
    r.add(NOT_VERIFIED, "COLLISION", "Collision-free motion (no collision model is available). " + NOT_VERIFIED_TEXT)
    r.add(NOT_VERIFIED, "SPEED_MEANING", "Velocity (%) is a controller-command setting; its physical speed is not verified.")
    r.add(INFO, "UNSUPPORTED", "Dwell time and per-waypoint velocity are not supported by the executor and are not exported.")
    if traj.approval.get("approved") is not True:
        r.add(WARNING, "NOT_APPROVED", "Operator approval is not recorded for this recipe version.")
    return r
