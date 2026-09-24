"""Create and export a geometry-only CRI joint path."""

from core.path_executor import PathExecutor
from core.path_planner import PathPlanner


def main():
    planner = PathPlanner()
    executor = PathExecutor()
    path = planner.generate_joint_path(
        start_angles=[0.0, -10.0, 135.0, 0.0, 20.0, 0.0],
        target_angles=[20.0, -20.0, 120.0, 10.0, 25.0, -10.0],
        waypoint_count=20,
    )
    executor.load_path(path)
    executor.export_to_csv_cri_commands("cri_path_commands.csv", velocity_percent=30.0)
    print(executor.generate_summary_report())


if __name__ == "__main__":
    main()