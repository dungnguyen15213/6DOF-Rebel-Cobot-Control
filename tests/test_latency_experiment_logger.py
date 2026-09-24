import csv
import tempfile
import unittest
from pathlib import Path

from core.latency_experiment_logger import LatencyExperimentLogger
from core.latency_tracking import ExecutionLatencyTracker, MeasurementStatus


class LatencyExperimentLoggerTest(unittest.TestCase):
    def test_one_completed_trial_produces_exactly_one_csv_row(self) -> None:
        tracker = ExecutionLatencyTracker(minimum_velocity_deg_s=0.5)
        tracker.start_command("jog", t_gui=1.0, t_tx=1.01, joint_index=0, direction=1.0)
        tracker.update_telemetry([0, 0, 0, 0, 0, 0], timestamp=1.0)
        tracker.update_telemetry([1, 0, 0, 0, 0, 0], timestamp=1.1)
        tracker.update_telemetry([2, 0, 0, 0, 0, 0], timestamp=1.2)

        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "latency_experiments.csv"
            logger = LatencyExperimentLogger(str(csv_path))
            try:
                trials = tracker.drain_completed_trials()
                self.assertEqual(len(trials), 1)
                logged = logger.log_trial(trials[0])
                self.assertTrue(logged)

                # Polling the drain/log path again must not duplicate the row.
                self.assertEqual(tracker.drain_completed_trials(), [])
                self.assertFalse(logger.log_trial(trials[0]))
            finally:
                logger.close()

            with open(csv_path, newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], MeasurementStatus.DETECTED.value)
        self.assertEqual(rows[0]["trial_id"], trials[0].trial_id)


if __name__ == "__main__":
    unittest.main()
