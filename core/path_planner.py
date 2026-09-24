import numpy as np


class PathPlanner:
    """Generates ordered spatial waypoints for preview and CRI execution."""

    def generate_joint_path(self, start_angles, target_angles, waypoint_count):
        """Return a joint-space line with an inclusive start and target waypoint."""
        if waypoint_count < 2:
            return [list(target_angles)]

        start = np.asarray(start_angles, dtype=float)
        target = np.asarray(target_angles, dtype=float)
        return np.linspace(start, target, waypoint_count).tolist()

    def generate_cartesian_line(self, start_xyz, target_xyz, waypoint_count, solve_waypoint):
        """Generate joint waypoints whose tool tip follows a straight XYZ line.

        ``solve_waypoint`` receives ``(xyz, previous_angles)`` and returns the
        joint solution for that point, or ``None`` when it is unreachable.
        """
        start = np.asarray(start_xyz, dtype=float)
        target = np.asarray(target_xyz, dtype=float)
        waypoint_count = max(2, int(waypoint_count))
        path = []
        previous_angles = None

        for step in range(waypoint_count):
            alpha = step / (waypoint_count - 1)
            point = (1.0 - alpha) * start + alpha * target
            angles = solve_waypoint(point.tolist(), previous_angles)
            if angles is None:
                raise ValueError(f"Cartesian waypoint {step + 1}/{waypoint_count} is unreachable")
            path.append(angles)
            previous_angles = angles

        return path