"""CRI path waypoint export utilities."""

import csv
import json
import numpy as np


class PathExecutor:
    """Stores spatial waypoints and exports CRI-compatible path payloads."""

    def __init__(self):
        self.path_data = []

    def load_path(self, joint_angles_list):
        """Load ordered six-axis joint targets without local timing data."""
        if not joint_angles_list:
            raise ValueError("Empty path data")
        if any(len(angles) != 6 for angles in joint_angles_list):
            raise ValueError("Each path waypoint must contain six joint angles")
        self.path_data = [[float(angle) for angle in angles] for angles in joint_angles_list]

    def export_to_csv_joint_angles(self, filepath, velocity_percent=50.0):
        """Export ordered joint targets and the controller velocity setting."""
        with open(filepath, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Step", "J1 (deg)", "J2 (deg)", "J3 (deg)", "J4 (deg)", "J5 (deg)", "J6 (deg)", "Velocity (%)"])
            for index, angles in enumerate(self.path_data):
                writer.writerow([index, *[f"{angle:.4f}" for angle in angles], velocity_percent])

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