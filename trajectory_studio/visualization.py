"""Matplotlib figures for the studio (joint-space only; no 3D model is claimed)."""

from __future__ import annotations

from matplotlib.figure import Figure

from trajectory_studio.models import JOINT_NAMES, Trajectory

COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
NO_TIMING_NOTE = ("No verified time model: positions are plotted against waypoint index. Velocity and acceleration "
                  "need validated controller behaviour or measured execution data.")


def draw_positions(fig: Figure, traj: Trajectory, visible: list[bool] | None = None,
                   selected: list[int] | None = None) -> None:
    fig.clear()
    ax = fig.add_subplot(111)
    visible = visible or [True] * 6
    n = len(traj.waypoints)
    moving = set(traj.moving_joints())
    if n == 0:
        ax.text(0.5, 0.5, "No waypoints", ha="center", va="center", transform=ax.transAxes)
    for j in range(6):
        if not visible[j] or n == 0:
            continue
        ys = [w.joints[j] for w in traj.waypoints]
        style = "-" if j in moving else "--"
        label = f"{JOINT_NAMES[j]}{' (moves)' if j in moving else ' (fixed)'}"
        ax.plot(range(n), ys, style, marker="o", ms=3, lw=1.6 if j in moving else 1.0, color=COLORS[j], label=label)
        for i in selected or []:
            if 0 <= i < n:
                ax.plot([i], [ys[i]], "o", ms=9, mfc="none", mec="black", mew=1.5)
    ax.set_xlabel("Waypoint index (not time)")
    ax.set_ylabel("Joint angle [deg]")
    ax.grid(True, alpha=0.3)
    if n:
        ax.legend(loc="upper right", ncol=3, fontsize=8)
    fig.text(0.01, 0.01, NO_TIMING_NOTE, fontsize=7, color="#555555")
    fig.subplots_adjust(bottom=0.16)


def draw_coverage(fig: Figure, coverage_data: dict) -> None:
    fig.clear()
    ax = fig.add_subplot(111)
    joints = coverage_data["joints"]
    labels = [row["joint"] for row in joints]
    pos = [row["positive_travel_deg"] for row in joints]
    neg = [-row["negative_travel_deg"] for row in joints]
    ax.bar(labels, pos, color="#2ca02c", label="planned positive travel")
    ax.bar(labels, neg, color="#d62728", label="planned negative travel")
    ax.axhline(0, color="black", lw=0.8)
    for x, row in enumerate(joints):
        if row["missing"]:
            ax.text(x, 0, "gap", ha="center", va="bottom", fontsize=8, color="#555555")
    ax.set_ylabel("Travel from start pose [deg]")
    ax.set_title("PLANNED excitation (not measured; not proof of identifiability)", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)
