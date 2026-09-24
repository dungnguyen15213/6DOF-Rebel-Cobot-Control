import unittest

from core.latency_tracking import (
    ExecutionLatencyTracker,
    LatencyStatistics,
    MeasurementStatus,
    NetworkLatencyTracker,
)


class LatencyTrackingTest(unittest.TestCase):
    def test_statistics_are_bounded_and_compute_jitter(self) -> None:
        statistics = LatencyStatistics(max_samples=3)
        for value in (1.0, 3.0, 2.0, 10.0):
            statistics.add(value)

        self.assertEqual(statistics.latest_ms, 10.0)
        self.assertEqual(statistics.minimum_ms, 2.0)
        self.assertEqual(statistics.maximum_ms, 10.0)
        self.assertEqual(statistics.peak_consecutive_jitter_ms, 8.0)

    def test_network_tracker_matches_ack_by_command_id(self) -> None:
        tracker = NetworkLatencyTracker()
        tracker.record_sent("42", timestamp=10.0)

        self.assertAlmostEqual(tracker.record_received("42", timestamp=10.012), 12.0)
        self.assertAlmostEqual(tracker.estimated_one_way_ms, 6.0)
        self.assertIsNone(tracker.record_received("unknown", timestamp=11.0))

    def test_execution_tracker_handles_negative_motion_direction(self) -> None:
        tracker = ExecutionLatencyTracker(
            minimum_velocity_deg_s=0.5,
            required_consecutive_samples=2,
        )
        tracker.start_command(
            "jog-1",
            t_gui=1.0,
            t_tx=1.01,
            joint_index=0,
            direction=-1.0,
        )
        tracker.update_telemetry([10.0, 0, 0, 0, 0, 0], timestamp=1.0)
        self.assertEqual(tracker.update_telemetry([9.0, 0, 0, 0, 0, 0], timestamp=1.1), ())
        responses = tracker.update_telemetry([8.0, 0, 0, 0, 0, 0], timestamp=1.2)

        self.assertEqual(len(responses), 1)
        self.assertAlmostEqual(responses[0].gui_to_motion_ms, 100.0)
        self.assertAlmostEqual(responses[0].tx_to_motion_ms, 90.0)
        self.assertAlmostEqual(responses[0].gui_to_tx_ms, 10.0)

    def test_wrong_direction_does_not_trigger_and_timeout_is_reported(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5, motion_timeout_s=0.25)
        tracker.start_command("positive", t_gui=1.0, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([-1, 0, 0, 0, 0, 0], timestamp=1.1)
        tracker.update_telemetry([-2, 0, 0, 0, 0, 0], timestamp=1.2)
        self.assertIsNone(tracker.latest_response)
        tracker.update_telemetry([-2, 0, 0, 0, 0, 0], timestamp=1.3)
        self.assertEqual(tracker.latest_status, MeasurementStatus.TIMEOUT)

    def test_cancelled_trial_is_not_later_detected(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.start_command("jog", t_gui=1.0, joint_index=0, direction=1.0)
        tracker.cancel_pending()
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        self.assertEqual(tracker.update_telemetry([1, 0, 0, 0, 0, 0], timestamp=1.1), ())
        self.assertEqual(tracker.latest_status, MeasurementStatus.CANCELLED)

    def test_network_tracker_discards_stale_pending_commands(self) -> None:
        tracker = NetworkLatencyTracker(pending_ttl_s=0.1)
        tracker.record_sent("stale", timestamp=1.0)
        self.assertEqual(tracker.expire_pending(timestamp=1.2), 1)
        self.assertIsNone(tracker.record_received("stale", timestamp=1.3))


if __name__ == "__main__":
    unittest.main()
