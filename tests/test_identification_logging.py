import csv
import os
import tempfile
import unittest
from pathlib import Path

from core.identification_logger import IdentificationLogger, analyze_identification_csv


class IdentificationLoggerTest(unittest.TestCase):
    def test_schema_and_metadata_propagation(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(
                output_dir=tmpdir,
                experiment_id="ID_NORMAL_001",
                trajectory_id="JOINT2_SWEEP_MEDIUM",
                run_id="RUN_01",
                commanded_speed=42.0,
                payload_g=0,
                contact_label="NORMAL",
                grasp_context="NONE",
                thermal_condition="COLD",
                overwrite=False,
            )
            logger.log_sample(
                timestamp=1.2345,
                q=[10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
                qdot=[0.5, 0.6, 0.7, 0.8, 0.9, 1.0],
                current=[100.0, 101.0, 102.0, 103.0, 104.0, 105.0],
                qddot=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            )
            logger.close()

            with open(logger.file_path, newline="", encoding="utf-8") as csvfile:
                rows = list(csv.DictReader(csvfile))

            self.assertEqual(len(rows), 1)
            self.assertIn("timestamp", rows[0])
            self.assertIn("q1", rows[0])
            self.assertIn("current1", rows[0])
            self.assertEqual(rows[0]["experiment_id"], "ID_NORMAL_001")
            self.assertEqual(rows[0]["trajectory_id"], "JOINT2_SWEEP_MEDIUM")
            self.assertEqual(rows[0]["run_id"], "RUN_01")
            self.assertEqual(rows[0]["payload_g"], "0")
            self.assertEqual(rows[0]["thermal_condition"], "COLD")
            self.assertEqual(rows[0]["current1"], "100.0")

    def test_prevents_accidental_overwrite(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = Path(tmpdir) / "collision_test.csv"
            output_path.write_text("placeholder\n", encoding="utf-8")

            logger = IdentificationLogger(
                output_dir=tmpdir,
                experiment_id="ID_NORMAL_001",
                trajectory_id="JOINT2_SWEEP_MEDIUM",
                run_id="RUN_01",
                overwrite=False,
                filename="collision_test.csv",
            )
            logger.close()

            self.assertNotEqual(logger.file_path, output_path)
            self.assertTrue(output_path.exists())
            self.assertTrue(logger.file_path.exists())

    def test_quality_checker_reports_dt_and_missing_values(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "quality_check.csv"
            with open(path, "w", newline="", encoding="utf-8") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    "timestamp", "q1", "q2", "q3", "q4", "q5", "q6",
                    "qdot1", "qdot2", "qdot3", "qdot4", "qdot5", "qdot6",
                    "current1", "current2", "current3", "current4", "current5", "current6",
                    "experiment_id", "trajectory_id", "run_id", "commanded_speed",
                    "payload_g", "contact_label", "grasp_context", "thermal_condition",
                ])
                writer.writerow([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, "ID1", "TRJ", "RUN_01", "50.0", "0", "NORMAL", "NONE", "COLD"])
                writer.writerow([1.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, "ID1", "TRJ", "RUN_01", "50.0", "0", "NORMAL", "NONE", "COLD"])
                writer.writerow([2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, "ID1", "TRJ", "RUN_02", "50.0", "0", "NORMAL", "NONE", "WARM"])

            report = analyze_identification_csv(path)

            self.assertEqual(report["total_rows"], 3)
            self.assertGreaterEqual(report["duplicate_timestamps"], 1)
            self.assertIn("q1", report["missing_values"])
            self.assertIn("dt_min", report)
            self.assertIn("dt_mean", report)
            self.assertIn("dt_median", report)
            self.assertIn("experiment_id_values", report)
            self.assertIn("trajectory_id_values", report)
            self.assertIn("run_id_values", report)
            self.assertIn("schema_summary", report)


if __name__ == "__main__":
    unittest.main()
