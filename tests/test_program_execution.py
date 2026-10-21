import math
import threading
import time
import unittest
from unittest.mock import Mock

from core.path_planner import PathPlanner
from cri_lib.cri_controller import CRIController
from cri_lib.cri_errors import CRIConnectionError
from hardware.robot_manager import RebelManager


class FakeSocket:
    """Records frames sent by the controller and answers them like an iRC."""

    def __init__(self, controller):
        self.controller = controller
        self.sent = []
        self.progerror_for = set()
        self.silent_prefixes = ()
        self.on_start = None
        self.on_stop = None
        self.fail_send = False

    def commands(self):
        return [c for c in self.sent if not c.startswith("ALIVEJOG")]

    def sendall(self, data):
        if self.fail_send:
            raise OSError("broken pipe")
        parts = data.decode().split()
        counter, command = parts[1], " ".join(parts[2:-1])
        self.sent.append(command)
        if command.startswith(self.silent_prefixes) and self.silent_prefixes:
            return
        if command.startswith("PROG "):
            cmd_nr = command.split()[1]
            if int(cmd_nr) in self.progerror_for:
                self.controller._parse_message(f"CRISTART 1 PROGERROR {counter} {cmd_nr} unknown_command CRIEND")
            else:
                self.controller._parse_message(f"CRISTART 1 PROGACK {counter} {cmd_nr} CRIEND")
            return
        self.controller._parse_message(f"CRISTART 1 CMDACK {counter} CRIEND")
        if command == "CMD StartProgram" and self.on_start:
            self.on_start()
        if command == "CMD StopProgram" and self.on_stop:
            self.on_stop()


def make_controller(version="V14-004-1"):
    controller = CRIController()
    controller.connected = True
    controller.sock = FakeSocket(controller)
    controller.robot_state.robot_control_version = version
    return controller


def execend(controller, reason="PLAN", cmd=3):
    controller._parse_message(f"CRISTART 1 EXECEND {cmd} 1 {reason} CRIEND")


PATH = [[0.0, 0, 0, 0, 0, 0], [5.0, 1, 0, 0, 0, 0], [10.0, 2, 0, 0, 0, 0]]


class ProgramWireFormatTests(unittest.TestCase):
    def test_prog_joint_format_has_velocity_acceleration_and_smoothing(self):
        text = CRIController.build_prog_joint_command(2, [1, 2, 3, 4, 5, 6], 30, 40, 25)
        self.assertEqual(
            text,
            "PROG 2 JOINT 1.0000 2.0000 3.0000 4.0000 5.0000 6.0000 EXT 0.0000 0.0000 0.0000 "
            "VEL 30.00 ACC 40.00 SM 25.00",
        )

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            CRIController.build_prog_joint_command(1, [math.nan] * 6, 30, 40, 25)
        with self.assertRaises(ValueError):
            CRIController.build_prog_joint_command(1, [0] * 6, 0, 40, 25)
        with self.assertRaises(ValueError):
            CRIController.build_prog_joint_command(1, [0] * 6, 30, 40, 101)


class RunJointProgramTests(unittest.TestCase):
    def setUp(self):
        self.robot = make_controller()
        self.sock = self.robot.sock
        self.sock.on_start = lambda: execend(self.robot, "PLAN")

    def test_all_points_buffered_before_single_start_and_no_move_joint(self):
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertTrue(outcome.success, outcome.message)
        commands = self.sock.commands()
        self.assertEqual(commands[0], "CMD DeleteProgram")
        self.assertEqual([c.split()[1] for c in commands if c.startswith("PROG ")], ["1", "2", "3"])
        self.assertEqual(commands.count("CMD StartProgram"), 1)
        last_prog = max(i for i, c in enumerate(commands) if c.startswith("PROG "))
        self.assertGreater(commands.index("CMD StartProgram"), last_prog)
        self.assertFalse(any("Move Joint" in c for c in commands))
        self.assertTrue(all("ACC 40.00 SM 25.00" in c for c in commands if c.startswith("PROG ")))

    def test_progerror_aborts_before_start(self):
        self.sock.progerror_for = {2}
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertFalse(outcome.success)
        self.assertEqual(outcome.reason, "REJECTED")
        self.assertIn("2", outcome.message)
        self.assertNotIn("CMD StartProgram", self.sock.commands())

    def test_progack_for_wrong_command_is_not_accepted(self):
        original = self.sock.sendall

        def wrong_ack(data):
            parts = data.decode().split()
            command = " ".join(parts[2:-1])
            self.sock.sent.append(command)
            if command.startswith("PROG "):
                self.robot._parse_message(f"CRISTART 1 PROGACK {parts[1]} 99 CRIEND")
            else:
                self.robot._parse_message(f"CRISTART 1 CMDACK {parts[1]} CRIEND")

        self.sock.sendall = wrong_ack
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertFalse(outcome.success)
        self.assertNotIn("CMD StartProgram", self.sock.commands())

    def test_execend_plan_succeeds_other_reasons_fail(self):
        for reason in ("USER", "PLC", "ERROR"):
            robot = make_controller()
            robot.sock.on_start = lambda r=robot, why=reason: execend(r, why)
            outcome = robot.run_joint_program(PATH, 30, 40, 25)
            self.assertFalse(outcome.success, reason)
            self.assertEqual(outcome.reason, reason)

    def test_execerror_fails(self):
        self.sock.on_start = lambda: self.robot._parse_message(
            "CRISTART 1 EXECERROR 2 1 collision_detected CRIEND")
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertFalse(outcome.success)

    def test_stale_execend_before_start_is_ignored(self):
        execend(self.robot, "PLAN")  # belongs to an earlier move, arrives while idle
        self.sock.on_start = None
        outcome = self.robot.program_execution.wait(0.05, lambda: None)
        self.assertFalse(outcome.success)

    def test_timeout_stops_the_program(self):
        self.sock.on_start = None
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25, timeout=0.2)
        self.assertFalse(outcome.success)
        self.assertEqual(outcome.reason, "TIMEOUT")
        self.assertIn("CMD StopProgram", self.sock.commands())

    def test_stop_event_sends_stop_program(self):
        self.sock.on_start = None
        self.sock.on_stop = lambda: execend(self.robot, "USER")
        stop = threading.Event()
        threading.Timer(0.1, stop.set).start()
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25, stop_event=stop, timeout=5)
        self.assertFalse(outcome.success)
        self.assertEqual(outcome.reason, "USER")
        self.assertEqual(self.sock.commands().count("CMD StopProgram"), 1)

    def test_cancel_before_start_never_starts(self):
        stop = threading.Event()
        stop.set()
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25, stop_event=stop)
        self.assertEqual(outcome.reason, "USER")
        self.assertNotIn("CMD StartProgram", self.sock.commands())

    def test_disconnect_during_run(self):
        self.sock.on_start = lambda: setattr(self.robot, "connected", False)
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25, timeout=5)
        self.assertFalse(outcome.success)
        self.assertEqual(outcome.reason, "DISCONNECTED")

    def test_not_connected(self):
        self.robot.connected = False
        self.assertEqual(self.robot.run_joint_program(PATH, 30, 40, 25).reason, "DISCONNECTED")

    def test_unacknowledged_command_times_out(self):
        self.robot.PROGRAM_ACK_TIMEOUT = 0.05
        self.sock.silent_prefixes = ("PROG ",)
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertFalse(outcome.success)
        self.assertNotIn("CMD StartProgram", self.sock.commands())

    def test_loaded_foreign_program_is_preserved_without_consent(self):
        self.robot.robot_state.main_main_program = "user_program.xml"
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertEqual(outcome.reason, "PROGRAM_LOADED")
        self.assertEqual(self.sock.commands(), [])
        outcome = self.robot.run_joint_program(PATH, 30, 40, 25, replace_loaded_program=True)
        self.assertTrue(outcome.success, outcome.message)

    def test_temporary_program_is_deleted_after_run(self):
        self.robot.run_joint_program(PATH, 30, 40, 25)
        self.assertEqual(self.sock.commands()[-1], "CMD DeleteProgram")


class ManagerExecutePathTests(unittest.TestCase):
    def make_manager(self, version="V14-004-1"):
        manager = RebelManager()
        manager.robot = make_controller(version)
        manager.robot.sock.on_start = lambda: execend(manager.robot, "PLAN")
        return manager

    def test_smooth_execution_uses_one_program_and_no_move_joint(self):
        manager = self.make_manager()
        success, error = manager.execute_path(PATH, 30.0, 40.0, 25.0)
        self.assertTrue(success, error)
        commands = manager.robot.sock.commands()
        self.assertEqual(commands.count("CMD StartProgram"), 1)
        self.assertFalse(any("Move Joint" in c for c in commands))

    def test_default_run_sends_velocity_only(self):
        manager = self.make_manager()
        success, error = manager.execute_path(PATH, 30.0)
        self.assertTrue(success, error)
        progs = [c for c in manager.robot.sock.commands() if c.startswith('PROG ')]
        self.assertEqual(len(progs), 3)
        self.assertTrue(all(' VEL 30.00' in c and ' ACC ' not in c and ' SM ' not in c for c in progs))

    def test_waypoint_validation(self):
        manager = self.make_manager()
        self.assertFalse(manager.execute_path([], 30.0)[0])
        self.assertFalse(manager.execute_path([[0, 0, 0, 0, 0, math.nan]], 30.0)[0])
        self.assertFalse(manager.execute_path([[0, 0, 0, 0, 0]], 30.0)[0])
        ok, error = manager.execute_path([[0, 0, 0, 0, 200, 0]], 30.0)
        self.assertFalse(ok)
        self.assertIn("J5", error)
        self.assertEqual(manager.robot.sock.commands(), [])

    def test_tiny_adjacent_waypoints_are_collapsed_and_order_kept(self):
        manager = self.make_manager()
        path = [[0.0] * 6, [0.001] + [0.0] * 5, [5.0] + [0.0] * 5, [5.002] + [0.0] * 5, [10.0] + [0.0] * 5]
        success, error = manager.execute_path(path, 30.0, 40.0, 25.0)
        self.assertTrue(success, error)
        progs = [c for c in manager.robot.sock.commands() if c.startswith("PROG ")]
        firsts = [float(p.split()[3]) for p in progs]
        self.assertEqual(firsts, [0.0, 5.0, 10.0])

    def test_stop_execution_sends_stop_program(self):
        manager = self.make_manager()
        self.assertTrue(manager.stop_execution())
        self.assertIn("CMD StopProgram", manager.robot.sock.commands())
        self.assertIn("CMD Move Stop", manager.robot.sock.commands())

    def test_stop_when_disconnected(self):
        manager = self.make_manager()
        manager.robot.connected = False
        self.assertFalse(manager.stop_execution())


class FeedbackTests(unittest.TestCase):
    def make_manager(self):
        manager = RebelManager()
        manager.robot = make_controller()
        manager.WAYPOINT_POLL_SEC = 0.0
        manager.WAYPOINT_REACH_TIMEOUT_SEC = 0.05
        return manager

    def set_joints(self, manager, values, fresh=True):
        joints = manager.robot.robot_state.joints_current
        for name, value in zip(("A1", "A2", "A3", "A4", "A5", "A6"), values):
            setattr(joints, name, value)
        manager.robot.last_status_received_at = time.perf_counter() if fresh else time.perf_counter() - 100

    def test_fresh_feedback_near_and_far(self):
        manager = self.make_manager()
        self.set_joints(manager, [0.0] * 6)
        self.assertTrue(manager._joints_near([0.5] * 6, 1.0))
        self.assertFalse(manager._joints_near([5.0] * 6, 1.0))

    def test_stale_and_nonfinite_feedback_is_unavailable(self):
        manager = self.make_manager()
        self.set_joints(manager, [0.0] * 6, fresh=False)
        self.assertIsNone(manager._joints_near([0.0] * 6, 1.0))
        self.set_joints(manager, [math.nan] + [0.0] * 5)
        self.assertIsNone(manager._joints_near([0.0] * 6, 1.0))

    def test_missing_feedback_is_not_success(self):
        manager = self.make_manager()
        manager.robot.last_status_received_at = time.perf_counter() - 100
        error = manager._wait_until_near([0.0] * 6, None)
        self.assertIsNotNone(error)
        self.assertIn("feedback", error)

    def test_cancellation_has_priority(self):
        manager = self.make_manager()
        self.set_joints(manager, [0.0] * 6)
        stop = threading.Event()
        stop.set()
        self.assertEqual(manager._wait_until_near([0.0] * 6, stop), "Execution stopped by user.")


class PathCleaningTests(unittest.TestCase):
    def test_duplicates_removed_endpoints_kept(self):
        path = [[0.0] * 6, [0.0] * 6, [1.0] * 6, [1.0] * 6, [2.0] * 6]
        self.assertEqual(PathPlanner.clean_path(path), [[0.0] * 6, [1.0] * 6, [2.0] * 6])

    def test_simplification_is_opt_in_and_bounded(self):
        line = [[float(i)] + [0.0] * 5 for i in range(11)]
        self.assertEqual(len(PathPlanner.clean_path(line)), 11)
        self.assertEqual(len(PathPlanner.clean_path(line, simplify_tolerance_deg=0.1)), 2)
        bend = line[:6] + [[5.0, 5.0] + [0.0] * 4]
        self.assertGreaterEqual(len(PathPlanner.clean_path(bend, simplify_tolerance_deg=0.1)), 3)


if __name__ == "__main__":
    unittest.main()
