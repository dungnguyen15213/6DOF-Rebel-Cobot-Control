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

    def test_network_tracker_excludes_errors_from_ack_rtt_statistics(self) -> None:
        tracker = NetworkLatencyTracker()
        tracker.record_sent("1", timestamp=1.0)
        tracker.record_sent("2", timestamp=1.0)

        self.assertAlmostEqual(tracker.record_received("1", timestamp=1.01), 10.0)
        self.assertAlmostEqual(tracker.record_error("2", timestamp=1.02), 20.0)

        self.assertEqual(tracker.statistics.count, 1)
        self.assertAlmostEqual(tracker.statistics.latest_ms, 10.0)
        self.assertEqual(tracker.error_statistics.count, 1)
        self.assertEqual(tracker.error_count, 1)

    def test_cancel_after_detection_does_not_overwrite_status(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.start_command("jog", t_gui=1.0, t_tx=1.01, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([1, 0, 0, 0, 0, 0], timestamp=1.1)
        responses = tracker.update_telemetry([2, 0, 0, 0, 0, 0], timestamp=1.2)

        self.assertEqual(len(responses), 1)
        self.assertEqual(tracker.latest_status, MeasurementStatus.DETECTED)

        cancelled = tracker.cancel_pending()

        self.assertFalse(cancelled)
        self.assertEqual(tracker.latest_status, MeasurementStatus.DETECTED)
        self.assertIsNotNone(tracker.latest_response)

    def test_new_trial_clears_stale_response_from_snapshot(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.start_command("jog-1", t_gui=1.0, t_tx=1.01, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([1, 0, 0, 0, 0, 0], timestamp=1.1)
        tracker.update_telemetry([2, 0, 0, 0, 0, 0], timestamp=1.2)
        self.assertEqual(tracker.snapshot.status, MeasurementStatus.DETECTED)

        tracker.start_command("jog-2", t_gui=2.0, joint_index=1, direction=1.0)

        snapshot = tracker.snapshot
        self.assertEqual(snapshot.status, MeasurementStatus.PENDING)
        self.assertIsNone(snapshot.response)
        self.assertIsNone(snapshot.gui_to_motion_ms)
        self.assertIsNone(snapshot.tx_to_motion_ms)

    def test_timeout_preserves_gui_to_tx_but_not_motion_fields(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5, motion_timeout_s=0.25)
        tracker.start_command("jog", t_gui=1.0, t_tx=1.02, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.3)

        snapshot = tracker.snapshot
        self.assertEqual(snapshot.status, MeasurementStatus.TIMEOUT)
        self.assertAlmostEqual(snapshot.gui_to_tx_ms, 20.0)
        self.assertIsNone(snapshot.tx_to_motion_ms)
        self.assertIsNone(snapshot.gui_to_motion_ms)

    def test_gui_armed_trial_binds_only_its_first_matching_transmission(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        trial_id = tracker.arm_trial(
            t_gui=1.0, joint_index=0, direction=1.0, commanded_speed=25.0
        )

        self.assertEqual(tracker.snapshot.trial_id, trial_id)
        self.assertEqual(tracker.snapshot.status, MeasurementStatus.PENDING)
        self.assertIsNone(tracker.snapshot.gui_to_tx_ms)
        self.assertTrue(tracker.bind_transmission(trial_id, "42", timestamp=1.01))
        self.assertFalse(tracker.bind_transmission(trial_id, "43", timestamp=1.02))

        snapshot = tracker.snapshot
        self.assertAlmostEqual(snapshot.gui_to_tx_ms, 10.0)
        self.assertIsNone(snapshot.gui_to_motion_ms)

    def test_cancellation_clears_motion_fields_for_active_trial(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.arm_trial(t_gui=1.0, joint_index=0, direction=1.0)
        tracker.cancel_pending(timestamp=1.1)

        snapshot = tracker.snapshot
        self.assertEqual(snapshot.status, MeasurementStatus.CANCELLED)
        self.assertIsNone(snapshot.response)
        self.assertIsNone(snapshot.tx_to_motion_ms)
        self.assertIsNone(snapshot.gui_to_motion_ms)

    def test_one_completed_trial_drains_exactly_once(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.start_command("jog", t_gui=1.0, t_tx=1.01, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([1, 0, 0, 0, 0, 0], timestamp=1.1)
        tracker.update_telemetry([2, 0, 0, 0, 0, 0], timestamp=1.2)

        first_drain = tracker.drain_completed_trials()
        second_drain = tracker.drain_completed_trials()

        self.assertEqual(len(first_drain), 1)
        self.assertEqual(first_drain[0].status, MeasurementStatus.DETECTED)
        self.assertEqual(second_drain, [])

    def test_trial_ids_are_unique_and_shared_between_snapshot_and_event(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        first_id = tracker.start_command("jog-1", t_gui=1.0, joint_index=0, direction=1.0)
        tracker.cancel_pending()
        second_id = tracker.start_command("jog-2", t_gui=2.0, joint_index=0, direction=1.0)
        tracker.cancel_pending()

        self.assertNotEqual(first_id, second_id)
        trials = tracker.drain_completed_trials()
        self.assertEqual([trial.trial_id for trial in trials], [first_id, second_id])



if __name__ == "__main__":
    unittest.main()
