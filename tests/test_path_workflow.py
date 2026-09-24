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

    def test_robot_manager_paces_waypoints_and_waits_for_final_completion(self):
        manager = RebelManager()
        manager.robot = Mock(connected=True)
        manager.robot.move_joints.return_value = True
        path = [[0.0] * 6, [10.0] * 6]

        success, error = manager.execute_path(path, velocity=42.0)

        self.assertTrue(success)
        self.assertIsNone(error)
        self.assertEqual(manager.robot.move_joints.call_count, 2)
        self.assertTrue(all(call.kwargs["velocity"] == 42.0 for call in manager.robot.move_joints.call_args_list))
        self.assertEqual(
            [call.kwargs["wait_move_finished"] for call in manager.robot.move_joints.call_args_list],
            [False, True],
        )
        self.assertTrue(all(call.kwargs["move_finished_timeout"] == 60.0 for call in manager.robot.move_joints.call_args_list))

    def test_path_execution_has_no_final_position_tolerance_poll(self):
        self.assertFalse(hasattr(RebelManager, "_wait_for_position"))


if __name__ == "__main__":
    unittest.main()