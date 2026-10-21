import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from core.path_executor import PathExecutor
from core.path_planner import PathPlanner
from hardware.robot_manager import RebelManager


class PathWorkflowTests(unittest.TestCase):
    def test_joint_path_contains_requested_spatial_waypoints(self):
        path = PathPlanner().generate_joint_path([0.0] * 6, [10.0] * 6, 3)

        self.assertEqual(path, [[0.0] * 6, [5.0] * 6, [10.0] * 6])

    def test_cri_export_contains_no_timing_column(self):
        executor = PathExecutor()
        executor.load_path([[0.0] * 6])

        with tempfile.TemporaryDirectory() as temp_directory:
            file_path = Path(temp_directory) / "commands.csv"
            executor.export_to_csv_cri_commands(file_path, velocity_percent=35.0)
            output = file_path.read_text()

        self.assertIn("Step,CRI Command", output)
        self.assertIn("CMD Move Joint", output)
        self.assertNotIn("Time", output)

    def test_run_follows_csv_points_in_order_waiting_for_each_move(self):
        manager = RebelManager()
        manager.robot = Mock(connected=True)
        manager.robot.move_joints.return_value = True
        path = [[0.0] * 6, [10.0] * 6, [20.0] * 6]

        success, error = manager.execute_path_point_to_point(path, velocity=42.0)

        self.assertTrue(success, error)
        calls = manager.robot.move_joints.call_args_list
        self.assertEqual([c.kwargs["A1"] for c in calls], [0.0, 10.0, 20.0])
        self.assertTrue(all(c.kwargs["velocity"] == 42.0 for c in calls))
        self.assertTrue(all(c.kwargs["wait_move_finished"] is True for c in calls))

    def test_failed_move_stops_the_run_and_reports_the_waypoint(self):
        manager = RebelManager()
        manager.robot = Mock(connected=True, last_move_error="Move aborted: x")
        manager.robot.move_joints.side_effect = [True, False, True]

        success, error = manager.execute_path_point_to_point([[0.0] * 6, [10.0] * 6, [20.0] * 6], 50.0)

        self.assertFalse(success)
        self.assertIn("Waypoint 2/3", error)
        self.assertEqual(manager.robot.move_joints.call_count, 2)

    def test_stop_event_prevents_further_moves(self):
        import threading
        manager = RebelManager()
        manager.robot = Mock(connected=True)
        stop = threading.Event()

        def move(**kwargs):
            stop.set()
            return True

        manager.robot.move_joints.side_effect = move
        success, error = manager.execute_path_point_to_point([[0.0] * 6, [10.0] * 6], 50.0, stop_event=stop)

        self.assertFalse(success)
        self.assertEqual(manager.robot.move_joints.call_count, 1)
    def test_path_execution_has_no_final_position_tolerance_poll(self):
        self.assertFalse(hasattr(RebelManager, "_wait_for_position"))

    def test_trajectory_parser_supports_phase_csv(self):
        executor = PathExecutor()
        with tempfile.TemporaryDirectory() as temp_directory:
            file_path = Path(temp_directory) / "trajectory.csv"
            file_path.write_text(
                "Phase,Step,J1 (deg),J2 (deg),J3 (deg),J4 (deg),J5 (deg),J6 (deg),Velocity (%)\n"
                "PREPARATION,0,10,20,30,40,50,60,35\n"
                "TASK,1,11,21,31,41,51,61,35\n"
                "TASK,2,12,22,32,42,52,62,35\n",
                encoding="utf-8",
            )
            plan = executor.load_trajectory_csv(file_path)

        self.assertEqual(plan.start_angles, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        self.assertEqual(plan.task_waypoints[0], [11.0, 21.0, 31.0, 41.0, 51.0, 61.0])
        self.assertEqual(plan.task_waypoints[-1], [12.0, 22.0, 32.0, 42.0, 52.0, 62.0])
        self.assertEqual(plan.velocity_percent, 35.0)

    def test_trajectory_parser_supports_legacy_step_cri_format(self):
        executor = PathExecutor()
        with tempfile.TemporaryDirectory() as temp_directory:
            file_path = Path(temp_directory) / "legacy.csv"
            file_path.write_text(
                "Step,CRI Command\n"
                "0,CMD Move Joint 1 2 3 4 5 6 0 0 0 35\n"
                "1,CMD Move Joint 2 3 4 5 6 7 0 0 0 35\n",
                encoding="utf-8",
            )
            plan = executor.load_trajectory_csv(file_path)

        self.assertEqual(plan.start_angles, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(plan.task_waypoints, [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        self.assertEqual(plan.velocity_percent, 35.0)


if __name__ == "__main__":
    unittest.main()