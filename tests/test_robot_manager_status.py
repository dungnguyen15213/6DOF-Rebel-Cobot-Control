import unittest
from threading import Lock

from cri_lib.cri_protocol_parser import CRIProtocolParser
from cri_lib.robot_state import RobotState
from hardware.robot_manager import RebelManager


class RebelManagerStatusTest(unittest.TestCase):
    def test_status_values_are_exposed_from_robot_state(self) -> None:
        manager = RebelManager()
        manager.robot.robot_state.current_total = 3.456
        manager.robot.robot_state.current_joints = [
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            0.6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.assertAlmostEqual(manager.get_total_current(), 3.456)
        self.assertEqual(manager.get_joint_currents_list(), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_status_message_tracks_seen_electrical_fields(self) -> None:
        robot_state = RobotState()
        parser = CRIProtocolParser(robot_state, Lock())

        message = "CRISTART 1 STATUS SUPPLY 24000 CURRENTALL 3000 CURRENTJOINTS " + " ".join(["1000"] * 16) + " CRIEND"
        parser.parse_message(message)

        self.assertIn("SUPPLY", robot_state.status_fields_seen)
        self.assertIn("CURRENTALL", robot_state.status_fields_seen)
        self.assertIn("CURRENTJOINTS", robot_state.status_fields_seen)


if __name__ == "__main__":
    unittest.main()
