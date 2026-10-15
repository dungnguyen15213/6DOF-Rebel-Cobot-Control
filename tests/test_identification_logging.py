import csv
import os
import tempfile
import threading
import unittest
from pathlib import Path

from core.identification_logger import (
    IDENTIFICATION_COLUMNS,
    IdentificationLogger,
    analyze_identification_csv,
    find_existing_identification_run,
)


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
            self.assertEqual(rows[0]["sample_index"], "0")
            self.assertEqual(list(rows[0]), IDENTIFICATION_COLUMNS)
            self.assertTrue(logger.summary_path.exists())

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

    def test_finds_run_id_already_used_by_another_csv(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(output_dir=tmpdir, run_id="RUN_09")
            logger.log_sample(1.0, [0.0] * 6, None, [1.0] * 6)
            logger.close()
            self.assertEqual(
                find_existing_identification_run(tmpdir, "RUN_09"), logger.file_path
            )
            self.assertIsNone(find_existing_identification_run(tmpdir, "RUN_10"))

            empty_run = IdentificationLogger(output_dir=tmpdir, run_id="RUN_EMPTY")
            empty_run.close()
            self.assertEqual(
                find_existing_identification_run(tmpdir, "RUN_EMPTY"),
                empty_run.file_path,
            )

    def test_missing_currents_are_nan_and_fail_quality(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(output_dir=tmpdir)
            logger.log_sample(
                timestamp=2.0,
                q=[0.0] * 6,
                qdot=None,
                current=[10.0, 20.0],
            )
            summary = logger.close()

            with open(logger.file_path, newline="", encoding="utf-8") as csvfile:
                row = next(csv.DictReader(csvfile))

            self.assertEqual(row["current1"], "10.0")
            self.assertEqual(row["current2"], "20.0")
            self.assertEqual(row["current3"], "nan")
            self.assertEqual(summary["quality_status"], "FAIL")
            self.assertEqual(summary["quality_report"]["incomplete_current_rows"], 1)

    def test_timestamp_precision_and_nonuniform_derivatives(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(output_dir=tmpdir)
            for timestamp, position in ((12.345678901, 0.0), (12.595678901, 2.0), (13.095678901, 8.0)):
                logger.log_sample(
                    timestamp=timestamp,
                    q=[position] * 6,
                    qdot=None,
                    current=[100.0 + position + joint for joint in range(6)],
                )
            logger.close()

            with open(logger.file_path, newline="", encoding="utf-8") as csvfile:
                rows = list(csv.DictReader(csvfile))

            self.assertEqual(rows[0]["timestamp"], "12.345678901")
            self.assertEqual(rows[0]["qdot1"], "nan")
            self.assertAlmostEqual(float(rows[1]["qdot1"]), 8.0)
            self.assertAlmostEqual(float(rows[2]["qdot1"]), 12.0)
            self.assertEqual(rows[1]["qddot1"], "nan")
            self.assertAlmostEqual(float(rows[2]["qddot1"]), 8.0)

    def test_metadata_is_frozen_for_each_run(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(
                output_dir=tmpdir,
                experiment_id="EXP_A",
                trajectory_id="TRAJ_A",
                run_id="RUN_A",
            )
            logger.log_sample(
                timestamp=1.0,
                q=[0.0] * 6,
                qdot=None,
                current=[1.0] * 6,
                metadata={"run_id": "RUN_B", "contact_label": "COLLISION"},
            )
            logger.close()

            with open(logger.file_path, newline="", encoding="utf-8") as csvfile:
                row = next(csv.DictReader(csvfile))
            self.assertEqual(row["experiment_id"], "EXP_A")
            self.assertEqual(row["trajectory_id"], "TRAJ_A")
            self.assertEqual(row["run_id"], "RUN_A")
            self.assertEqual(row["contact_label"], "NORMAL")

    def test_writer_drains_queue_and_close_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(output_dir=tmpdir, queue_capacity=256)
            for sample_index in range(150):
                logger.log_sample(
                    timestamp=sample_index / 100.0,
                    q=[float(sample_index)] * 6,
                    qdot=None,
                    current=[float(sample_index + joint + 1) for joint in range(6)],
                )
            summary = logger.close()
            self.assertIs(logger.close(), summary)

            with open(logger.file_path, newline="", encoding="utf-8") as csvfile:
                rows = list(csv.DictReader(csvfile))
            self.assertEqual(len(rows), 150)
            self.assertEqual(summary["writer_dropped_sample_count"], 0)

    def test_queue_overflow_is_counted_and_reported(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            logger = IdentificationLogger(output_dir=tmpdir, queue_capacity=1)
            write_entered = threading.Event()
            release_write = threading.Event()
            original_writerow = logger._writer.writerow

            def blocked_writerow(row):
                write_entered.set()
                release_write.wait(timeout=2.0)
                original_writerow(row)

            logger._writer.writerow = blocked_writerow
            logger.log_sample(1.0, [0.0] * 6, None, [1.0] * 6)
            self.assertTrue(write_entered.wait(timeout=2.0))
            logger.log_sample(2.0, [1.0] * 6, None, [2.0] * 6)
            logger.log_sample(3.0, [2.0] * 6, None, [3.0] * 6)
            release_write.set()
            summary = logger.close()

            self.assertEqual(summary["writer_dropped_sample_count"], 1)
            self.assertEqual(summary["quality_status"], "FAIL")

    def test_new_run_resets_derived_signal_history(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            first_run = IdentificationLogger(output_dir=tmpdir, run_id="RUN_01")
            first_run.log_sample(100.0, [10.0] * 6, None, [1.0] * 6)
            first_run.log_sample(101.0, [20.0] * 6, None, [2.0] * 6)
            first_run.close()

            second_run = IdentificationLogger(output_dir=tmpdir, run_id="RUN_02")
            second_run.log_sample(1.0, [50.0] * 6, None, [3.0] * 6)
            second_run.close()

            with open(second_run.file_path, newline="", encoding="utf-8") as csvfile:
                row = next(csv.DictReader(csvfile))
            self.assertEqual(row["timestamp"], "1.0")
            self.assertEqual(row["qdot1"], "nan")
            self.assertEqual(row["qddot1"], "nan")

    def _write_quality_csv(self, path, timestamps, metadata_changes=None, missing_current=False):
        metadata_changes = metadata_changes or {}
        with open(path, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=IDENTIFICATION_COLUMNS)
            writer.writeheader()
            for sample_index, timestamp in enumerate(timestamps):
                row = {
                    "timestamp": timestamp,
                    "sample_index": sample_index,
                    "experiment_id": "EXP_1",
                    "trajectory_id": "TRAJ_1",
                    "run_id": metadata_changes.get(sample_index, "RUN_1"),
                    "commanded_speed": "40",
                    "payload_g": "0",
                    "contact_label": "NORMAL",
                    "grasp_context": "NONE",
                    "thermal_condition": "COLD",
                }
                for joint in range(1, 7):
                    row[f"q{joint}"] = sample_index + joint
                    row[f"qdot{joint}"] = 1.0
                    row[f"qddot{joint}"] = 0.0
                    row[f"current{joint}"] = sample_index + joint + 10
                if missing_current and sample_index == 1:
                    row["current4"] = "nan"
                writer.writerow(row)

    def test_quality_checker_passes_clean_synthetic_run(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "clean.csv"
            self._write_quality_csv(path, [10.0, 10.1, 10.25])
            report = analyze_identification_csv(path)

            self.assertEqual(report["quality_status"], "PASS")
            self.assertEqual(report["total_rows"], 3)
            self.assertAlmostEqual(report["dt_min"], 0.1)
            self.assertAlmostEqual(report["dt_max"], 0.15)
            self.assertIn("P99", report["dt_percentiles"])
            self.assertEqual(report["current_limit_verified"], False)

    def test_quality_checker_fails_missing_current(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "missing_current.csv"
            self._write_quality_csv(path, [1.0, 1.1], missing_current=True)
            report = analyze_identification_csv(path)
            self.assertEqual(report["quality_status"], "FAIL")
            self.assertEqual(report["incomplete_current_rows"], 1)
            self.assertEqual(report["missing_values"]["current4"], 1)

    def test_quality_checker_fails_duplicate_and_nonmonotonic_timestamps(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bad_timing.csv"
            self._write_quality_csv(path, [1.0, 1.0, 0.5])
            report = analyze_identification_csv(path)
            self.assertEqual(report["quality_status"], "FAIL")
            self.assertEqual(report["duplicate_timestamp_values"], 1)
            self.assertEqual(report["dt_nonpositive_count"], 2)
            self.assertEqual(report["dt_negative_count"], 1)
            self.assertFalse(report["timestamp_monotonic"])

    def test_quality_checker_fails_inconsistent_metadata(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "mixed_metadata.csv"
            self._write_quality_csv(path, [1.0, 1.1], metadata_changes={1: "RUN_2"})
            report = analyze_identification_csv(path)
            self.assertEqual(report["quality_status"], "FAIL")
            self.assertTrue(report["metadata_inconsistent"])
            self.assertEqual(report["run_id_values"], ["RUN_1", "RUN_2"])

    def test_quality_checker_fails_malformed_extra_fields(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "malformed.csv"
            self._write_quality_csv(path, [1.0])
            with open(path, "a", newline="", encoding="utf-8") as csvfile:
                csv.writer(csvfile).writerow(["extra"] * (len(IDENTIFICATION_COLUMNS) + 1))

            report = analyze_identification_csv(path)

            self.assertEqual(report["quality_status"], "FAIL")
            self.assertEqual(report["malformed_row_count"], 1)

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
