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

    @staticmethod
    def join_segments(segments):
        """Chain segments into one path; each later segment's first waypoint repeats the previous end."""
        joined = []
        for segment in segments:
            joined.extend(segment if not joined else segment[1:])
        return joined

    @staticmethod
    def clean_path(path, min_step_deg=0.01, simplify_tolerance_deg=0.0):
        """Drop duplicate/negligible waypoints and optionally simplify within a joint-space tolerance.

        The first and last waypoints are always kept, order is preserved, and units stay degrees.
        ``min_step_deg`` removes a waypoint whose largest joint change from the previously kept
        waypoint is below it. ``simplify_tolerance_deg`` > 0 additionally removes intermediate
        waypoints lying within that many degrees of the straight joint-space line between kept
        neighbours (Ramer-Douglas-Peucker); this can round corners, so it is off by default.
        """
        points = [np.asarray(p, dtype=float) for p in path]
        if len(points) < 3:
            return [p.tolist() for p in points]

        kept = [points[0]]
        for point in points[1:-1]:
            if float(np.max(np.abs(point - kept[-1]))) >= float(min_step_deg):
                kept.append(point)
        if float(np.max(np.abs(points[-1] - kept[-1]))) < float(min_step_deg) and len(kept) > 1:
            kept[-1] = points[-1]  # the final target replaces a negligible predecessor
        else:
            kept.append(points[-1])

        if simplify_tolerance_deg > 0.0 and len(kept) > 2:
            kept = PathPlanner._rdp(kept, float(simplify_tolerance_deg))
        return [p.tolist() for p in kept]

    @staticmethod
    def _rdp(points, tolerance):
        start, end = points[0], points[-1]
        direction = end - start
        length_sq = float(np.dot(direction, direction))
        worst_index, worst_distance = 0, -1.0
        for index in range(1, len(points) - 1):
            offset = points[index] - start
            if length_sq == 0.0:
                distance = float(np.max(np.abs(offset)))
            else:
                t = min(1.0, max(0.0, float(np.dot(offset, direction)) / length_sq))
                distance = float(np.max(np.abs(offset - t * direction)))
            if distance > worst_distance:
                worst_index, worst_distance = index, distance
        if worst_distance <= tolerance:
            return [start, end]
        left = PathPlanner._rdp(points[: worst_index + 1], tolerance)
        right = PathPlanner._rdp(points[worst_index:], tolerance)
        return left[:-1] + right