import unittest

import numpy as np

from core.kinematics import ReBelKinematics
from core.path_planner import PathPlanner
from core.sequence_planner import (
    MOTION_LINE,
    MOTION_LINE_LEVEL,
    MOTION_LOCKED,
    MOTION_PTP,
    MOTION_PTP_LEVEL,
    SequenceError,
    SequencePlanner,
    SequenceStep,
)


class SequencePlannerTests(unittest.TestCase):
    def setUp(self):
        self.kinematics = ReBelKinematics()
        self.planner = SequencePlanner(self.kinematics, PathPlanner())

    def tip(self, angles):
        return self.kinematics.get_tip_position(np.radians(angles))

    def test_empty_sequence_is_rejected(self):
        with self.assertRaises(SequenceError):
            self.planner.plan([], [0.0] * 6)

    def test_locked_motion_requires_a_joint(self):
        step = SequenceStep(MOTION_LOCKED, (200, 0, 300), (200, 50, 300), 5, ())
        with self.assertRaises(SequenceError):
            self.planner.plan([step], [0.0] * 6)

    def test_straight_line_reaches_both_points(self):
        step = SequenceStep(MOTION_LINE, (200, 0, 300), (200, 100, 300), 8)
        plan = self.planner.plan([step], [0.0] * 6)
        self.assertEqual(len(plan.path), 8)
        np.testing.assert_allclose(self.tip(plan.path[0]), (200, 0, 300), atol=1.5)
        np.testing.assert_allclose(self.tip(plan.path[-1]), (200, 100, 300), atol=1.5)

    def test_level_steps_keep_tool_plane_horizontal(self):
        steps = [
            SequenceStep(MOTION_LINE_LEVEL, (200, 0, 200), (200, 100, 200), 6),
            SequenceStep(MOTION_PTP_LEVEL, (200, 100, 200), (250, 0, 200), 6),
        ]
        plan = self.planner.plan(steps, [0.0] * 6)
        for angles in plan.path:
            rotation = self.kinematics.get_tip_transform(np.radians(angles))[:3, :3]
            self.assertGreater(-rotation[2, 2], np.cos(np.radians(1.5)))

    def test_reorient_bridge_inserted_between_free_and_level_steps(self):
        steps = [
            SequenceStep(MOTION_LINE, (200, 0, 200), (200, 100, 200), 6),
            SequenceStep(MOTION_LINE_LEVEL, (200, 100, 200), (250, 0, 200), 6),
        ]
        plan = self.planner.plan(steps, [0.0] * 6)
        self.assertEqual([s.is_bridge for s in plan.segments], [False, True, False])
        self.assertEqual(plan.segments[-1].last_frame, len(plan.path) - 1)
        self.assertIs(plan.segment_at(plan.segments[1].first_frame), plan.segments[1])

    def test_continuing_steps_need_no_bridge(self):
        steps = [
            SequenceStep(MOTION_LINE, (200, 0, 300), (200, 100, 300), 6),
            SequenceStep(MOTION_LINE, (200, 100, 300), (200, 100, 250), 6),
        ]
        plan = self.planner.plan(steps, [0.0] * 6)
        self.assertFalse(any(s.is_bridge for s in plan.segments))
        self.assertEqual(len(plan.path), 11)

    def test_locked_joint_stays_fixed(self):
        step = SequenceStep(MOTION_LOCKED, (200, 0, 300), (200, 50, 300), 6, (0,))
        plan = self.planner.plan([step], [10.0, 0, 0, 0, 0, 0])
        self.assertTrue(all(abs(a[0] - plan.path[0][0]) < 1e-6 for a in plan.path))

    def test_check_steps_reports_each_step(self):
        steps = [
            SequenceStep(MOTION_LINE, (200, 0, 300), (200, 100, 300), 6),
            SequenceStep(MOTION_PTP, (200, 100, 300), (2000, 0, 300), 5),
            SequenceStep(MOTION_LOCKED, (200, 100, 300), (200, 0, 300), 5, ()),
        ]
        results = self.planner.check_steps(steps, [0.0] * 6)
        self.assertIsNone(results[0])
        self.assertIn("Step 2", results[1])
        self.assertIn("locked", results[2].lower())

    def test_validate_step_accepts_reachable_step(self):
        step = SequenceStep(MOTION_PTP, (200, 0, 300), (200, 100, 300), 10)
        self.assertIsNone(self.planner.validate_step(step, [0.0] * 6))

    def test_check_path_flags_joint_limit_and_jump(self):
        self.assertIn("J5", self.planner.check_path([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 99, 0]]))
        self.assertIn("jumps", self.planner.check_path([[0] * 6, [60, 0, 0, 0, 0, 0]]))
        self.assertIsNone(self.planner.check_path([[0] * 6, [5, 0, 0, 0, 5, 0]]))

    def test_unreachable_point_names_the_step(self):
        step = SequenceStep(MOTION_PTP, (200, 0, 300), (2000, 0, 300), 5)
        with self.assertRaisesRegex(SequenceError, "Step 1"):
            self.planner.plan([step], [0.0] * 6)


if __name__ == "__main__":
    unittest.main()
