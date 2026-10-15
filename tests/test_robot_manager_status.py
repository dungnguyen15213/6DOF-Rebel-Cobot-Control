import unittest
import math
from threading import Lock

from cri_lib.cri_protocol_parser import CRIProtocolParser
from cri_lib.cri_controller import CRIController
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
        manager.robot.robot_state.status_fields_seen = {"CURRENTALL", "CURRENTJOINTS"}

        self.assertAlmostEqual(manager.get_total_current(), 3.456)
        self.assertEqual(manager.get_joint_currents_list(), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_unavailable_current_is_nan_not_zero_or_cached_value(self) -> None:
        manager = RebelManager()
        manager.robot.robot_state.current_joints = [100.0] * 16

        self.assertTrue(all(math.isnan(value) for value in manager.get_joint_currents_list()))
        self.assertTrue(math.isnan(manager.get_total_current()))

    def test_status_message_tracks_seen_electrical_fields(self) -> None:
        robot_state = RobotState()
        parser = CRIProtocolParser(robot_state, Lock())

        message = "CRISTART 1 STATUS SUPPLY 24000 CURRENTALL 3000 CURRENTJOINTS " + " ".join(["1000"] * 16) + " CRIEND"
        parser.parse_message(message)

        self.assertIn("SUPPLY", robot_state.status_fields_seen)
        self.assertIn("CURRENTALL", robot_state.status_fields_seen)
        self.assertIn("CURRENTJOINTS", robot_state.status_fields_seen)

    def test_incomplete_status_vectors_are_nan_and_marked_present(self) -> None:
        robot_state = RobotState()
        parser = CRIProtocolParser(robot_state, Lock())
        message = (
            "CRISTART 1 STATUS CURRENTJOINTS 100 200 300 "
            "POSJOINTCURRENT 1 2 3 CRIEND"
        )

        parser.parse_message(message)

        self.assertIn("CURRENTJOINTS", robot_state.status_fields_seen)
        self.assertIn("POSJOINTCURRENT", robot_state.status_fields_seen)
        self.assertEqual(robot_state.current_joints[:3], [100.0, 200.0, 300.0])
        self.assertTrue(all(value != value for value in robot_state.current_joints[3:]))
        self.assertTrue(all(
            value != value
            for value in (
                robot_state.joints_current.A4,
                robot_state.joints_current.A5,
                robot_state.joints_current.A6,
            )
        ))

    def test_status_callback_receives_socket_timestamp_and_is_isolated(self) -> None:
        controller = CRIController()
        callback_times = []

        def callback(_state, received_at):
            callback_times.append(received_at)
            raise RuntimeError("observer failure must not escape receive processing")

        controller.register_status_callback(callback)
        controller._parse_message("CRISTART 1 STATUS CRIEND", t_rx=12.5)

        self.assertEqual(callback_times, [12.5])

    def test_cmderror_response_is_excluded_from_ack_rtt_statistics(self) -> None:
        controller = CRIController()
        controller.network_latency.record_sent("7", timestamp=1.0)

        controller._parse_message("CRISTART 99 CMDERROR 7 SomeFault CRIEND", t_rx=1.05)

        self.assertEqual(controller.network_latency.statistics.count, 0)
        self.assertEqual(controller.network_latency.error_statistics.count, 1)
        self.assertEqual(controller.network_latency.error_count, 1)

    def test_cmdack_response_contributes_to_ack_rtt_statistics(self) -> None:
        controller = CRIController()
        controller.network_latency.record_sent("7", timestamp=1.0)

        controller._parse_message("CRISTART 99 CMDACK 7 CRIEND", t_rx=1.05)

        self.assertEqual(controller.network_latency.statistics.count, 1)
        self.assertEqual(controller.network_latency.error_statistics.count, 0)


if __name__ == "__main__":
    unittest.main()
