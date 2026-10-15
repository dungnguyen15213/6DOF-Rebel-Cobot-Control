import csv
import hashlib
import tempfile
import threading
import unittest
from pathlib import Path

import numpy as np

from core.identification_audit import (
    FAIL,
    NOT_VERIFIED,
    PASS,
    WARNING,
    UnsupportedMergeError,
    assess_merge_feasibility,
    audit_run,
    build_processed_dataset,
    model_features,
    require_mergeable,
    whole_run_holdout_splits,
)
from core.identification_logger import IDENTIFICATION_COLUMNS, IdentificationLogger
from tools.audit_identification_run import main as audit_cli


def write_run(path, n=1200, dt=0.03, mutate=None, metadata=None, joints=(0, 1, 2, 3, 4, 5)):
    """Synthetic raw CSV in the logger schema; no robot measurements are implied."""
    meta = {
        "experiment_id": "ID_NORMAL_T", "trajectory_id": "SYN", "run_id": "RUN_T",
        "commanded_speed": "50.0", "payload_g": "0.0", "contact_label": "NORMAL",
        "grasp_context": "NONE", "thermal_condition": "COLD",
    }
    meta.update(metadata or {})
    rng = np.random.default_rng(1)
    t = 1000.0 + dt * np.arange(n)
    rows = []
    for i in range(n):
        row = {"timestamp": repr(float(t[i])), "sample_index": i, **meta}
        for j in range(6):
            angle = 20.0 * np.sin(2 * np.pi * (t[i] - t[0]) / 6.0) if j in joints else 5.0
            row[f"q{j + 1}"] = angle
            row[f"current{j + 1}"] = float(np.round(300 + 80 * np.sin(i / 7 + j) + rng.normal(0, 5)))
            row[f"qdot{j + 1}"] = "nan"
            row[f"qddot{j + 1}"] = "nan"
        rows.append(row)
    if mutate:
        mutate(rows)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=IDENTIFICATION_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return Path(path)


def status_of(report, criterion_id):
    return next(c["status"] for c in report["criteria"] if c["id"] == criterion_id)


class AuditPassWarnFailTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_clean_full_coverage_run_passes(self):
        report = audit_run(write_run(self.dir / "ok.csv"))
        self.assertEqual(report["overall_status"], PASS)
        self.assertEqual(report["conclusions"]["initial_0g_dry_test"], PASS)
        self.assertEqual(report["conclusions"]["full_six_joint_campaign"], PASS)
        self.assertEqual(status_of(report, "I7"), NOT_VERIFIED)
        self.assertIn("campaign_note", report)

    def test_unmoved_joints_limit_coverage_without_failing_dry_test(self):
        report = audit_run(write_run(self.dir / "part.csv", joints=(0, 1)))
        self.assertEqual(report["conclusions"]["initial_0g_dry_test"], PASS)
        self.assertEqual(report["conclusions"]["full_six_joint_campaign"], WARNING)
        self.assertEqual(report["excitation_summary"]["excited_joints"], [1, 2])
        self.assertEqual(report["joints"][2]["excitation"], "NOT EXERCISED")

    def test_short_recording_is_warning_not_pass(self):
        report = audit_run(write_run(self.dir / "short.csv", n=100))
        self.assertEqual(report["conclusions"]["initial_0g_dry_test"], WARNING)

    def test_initialized_zero_currents_fail(self):
        def mutate(rows):
            for row in rows[:5]:
                for j in range(1, 7):
                    row[f"current{j}"] = 0.0

        report = audit_run(write_run(self.dir / "zero.csv", mutate=mutate))
        self.assertEqual(status_of(report, "I5"), FAIL)
        self.assertEqual(report["overall_status"], FAIL)

    def test_missing_current_is_distinct_from_measured_zero(self):
        def mutate(rows):
            rows[10]["current3"] = "nan"
            rows[20]["current2"] = 0.0

        report = audit_run(write_run(self.dir / "nan.csv", mutate=mutate))
        self.assertEqual(status_of(report, "I4"), FAIL)
        self.assertEqual(report["conclusions"]["logger_signal_integrity"], FAIL)
        self.assertNotEqual(status_of(report, "I5"), FAIL)  # an isolated measured zero is not a defect

    def test_decreasing_timestamp_fails(self):
        def mutate(rows):
            rows[50]["timestamp"] = repr(float(rows[40]["timestamp"]))

        report = audit_run(write_run(self.dir / "back.csv", mutate=mutate))
        self.assertEqual(status_of(report, "T1"), FAIL)

    def test_small_duplicate_fraction_warns_and_large_fails(self):
        def few(rows):
            rows[100]["timestamp"] = rows[99]["timestamp"]

        def many(rows):
            for i in range(1, len(rows), 2):
                rows[i]["timestamp"] = rows[i - 1]["timestamp"]

        self.assertEqual(status_of(audit_run(write_run(self.dir / "d1.csv", mutate=few)), "T2"), WARNING)
        large = audit_run(write_run(self.dir / "d2.csv", mutate=many))
        self.assertEqual(status_of(large, "T2"), FAIL)
        self.assertEqual(large["conclusions"]["initial_0g_dry_test"], FAIL)

    def test_metadata_change_and_condition_mismatch_fail(self):
        def drift(rows):
            rows[300]["payload_g"] = "250.0"

        report = audit_run(write_run(self.dir / "meta.csv", mutate=drift))
        self.assertEqual(status_of(report, "I1"), FAIL)
        mismatch = audit_run(write_run(self.dir / "load.csv", metadata={"payload_g": "250.0"}))
        self.assertEqual(status_of(mismatch, "I2"), FAIL)

    def test_flat_current_channel_warns(self):
        def mutate(rows):
            for row in rows:
                row["current4"] = 12.0

        report = audit_run(write_run(self.dir / "flat.csv", mutate=mutate))
        self.assertEqual(status_of(report, "I6"), WARNING)
        self.assertEqual(report["overall_status"], WARNING)

    def test_structural_failures(self):
        header_only = self.dir / "empty.csv"
        header_only.write_text(",".join(IDENTIFICATION_COLUMNS) + "\n", encoding="utf-8")
        self.assertEqual(audit_run(header_only)["overall_status"], FAIL)

        missing = self.dir / "missing.csv"
        missing.write_text("timestamp,q1\n1,2\n", encoding="utf-8")
        self.assertEqual(audit_run(missing)["conclusions"]["basic_csv_readability"], FAIL)

        good = write_run(self.dir / "trunc.csv", n=50)
        good.write_bytes(good.read_bytes().rstrip(b"\r\n"))
        self.assertEqual(status_of(audit_run(good), "R3"), FAIL)

    def test_cli_exit_codes_and_raw_file_unchanged(self):
        good = write_run(self.dir / "good.csv")
        bad = write_run(self.dir / "bad.csv", mutate=lambda rows: rows[3].update(current1="nan"))
        digest = hashlib.sha256(bad.read_bytes()).hexdigest()
        out = self.dir / "reports"
        self.assertEqual(audit_cli([str(good), "--out-dir", str(out / "g"), "--no-plots"]), 0)
        self.assertEqual(audit_cli([str(bad), "--out-dir", str(out / "b"), "--no-plots", "--processed"]), 1)
        self.assertTrue((out / "b" / "audit_report.json").exists())
        self.assertEqual(list((out / "b").glob("*_processed.csv")), [])
        self.assertEqual(hashlib.sha256(bad.read_bytes()).hexdigest(), digest)
        self.assertEqual(audit_cli([str(self.dir / "nope.csv")]), 2)


class ProcessedDatasetTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_si_conversion_provenance_and_raw_preservation(self):
        raw = write_run(self.dir / "raw.csv", n=200)
        digest = hashlib.sha256(raw.read_bytes()).hexdigest()
        out = self.dir / "processed.csv"
        provenance = build_processed_dataset(raw, out)
        self.assertEqual(hashlib.sha256(raw.read_bytes()).hexdigest(), digest)
        self.assertEqual(provenance["source_sha256"], digest)
        with open(raw, newline="") as f1, open(out, newline="") as f2:
            raw_rows, new_rows = list(csv.DictReader(f1)), list(csv.DictReader(f2))
        self.assertEqual(len(raw_rows), len(new_rows))
        self.assertAlmostEqual(float(new_rows[7]["q1_rad"]), np.deg2rad(float(raw_rows[7]["q1"])))
        self.assertAlmostEqual(float(new_rows[7]["current2_A"]), float(raw_rows[7]["current2"]) / 1000.0)
        self.assertEqual(new_rows[7]["run_id"], "RUN_T")
        self.assertEqual(new_rows[7]["source_file"], "raw.csv")
        self.assertTrue(Path(f"{out}.provenance.json").exists())

    def test_derivatives_use_measured_dt_and_leave_boundaries_nan(self):
        raw = write_run(self.dir / "raw.csv", n=400, dt=0.03)
        out = self.dir / "p.csv"
        build_processed_dataset(raw, out)
        with open(out, newline="") as handle:
            rows = list(csv.DictReader(handle))
        omega = 2 * np.pi / 6.0
        t = np.array([float(r["t_s"]) for r in rows])
        expected_qdot = np.deg2rad(20.0) * omega * np.cos(omega * t)
        qdot = np.array([float(r["qdot1_rad_s"]) for r in rows])
        self.assertTrue(np.isnan(qdot[0]) and np.isnan(qdot[-1]))
        self.assertTrue(np.nanmax(np.abs(qdot[2:-2] - expected_qdot[2:-2])) < 0.01)
        self.assertTrue(np.isnan(float(rows[1]["qddot1_rad_s2"])))

    def test_duplicate_timestamps_are_flagged_and_not_differentiated(self):
        def mutate(rows):
            for i in range(1, len(rows), 2):
                rows[i]["timestamp"] = rows[i - 1]["timestamp"]

        raw = write_run(self.dir / "dup.csv", n=100, mutate=mutate)
        out = self.dir / "p.csv"
        build_processed_dataset(raw, out)
        with open(out, newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[0]["flag_duplicate_timestamp"], "1")
        self.assertEqual(rows[0]["flag_use_for_derivatives"], "0")
        self.assertEqual(rows[1]["flag_use_for_derivatives"], "1")
        self.assertEqual(rows[0]["qdot1_rad_s"], "nan")
        self.assertEqual(rows[0]["batch_size"], "2")

    def test_refuses_overwrite_of_raw_and_decreasing_time(self):
        raw = write_run(self.dir / "raw.csv", n=50)
        with self.assertRaises(ValueError):
            build_processed_dataset(raw, raw)
        bad = write_run(self.dir / "bad.csv", n=50, mutate=lambda r: r[10].update(timestamp="1.0"))
        with self.assertRaises(ValueError):
            build_processed_dataset(bad, self.dir / "x.csv")

    def test_model_feature_matrices_and_whole_run_holdout(self):
        raw = write_run(self.dir / "raw.csv", n=200)
        out = self.dir / "p.csv"
        build_processed_dataset(raw, out)
        design_a, target = model_features(out, 1, "A", v_s=0.1)
        design_b, _ = model_features(out, 1, "B", v_s=0.1)
        self.assertEqual(design_a.shape[1], 5)
        self.assertEqual(design_b.shape[1], 6)
        self.assertEqual(design_a.shape[0], target.shape[0])
        self.assertTrue(np.all(design_a[:, 0] == 1.0))
        splits = list(whole_run_holdout_splits(["A", "A", "B", "B", "C"]))
        self.assertEqual(len(splits), 3)
        for train, test in splits:
            self.assertFalse(set(train) & set(test))
        with self.assertRaises(ValueError):
            list(whole_run_holdout_splits(["A", "A"]))


class MergeGuardTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.ident = write_run(self.dir / "ident.csv", n=30)

    def tearDown(self):
        self._tmp.cleanup()

    def test_gui_diagnostics_schema_is_not_mergeable(self):
        diag = self.dir / "robot_diagnostics.csv"
        diag.write_text("Time (s),J1 Angle (deg),J1 Current (mA)\n0.0,1.0,2.0\n0.057,1.0,3.0\n", encoding="utf-8")
        assessment = assess_merge_feasibility(self.ident, diag)
        self.assertFalse(assessment["mergeable"])
        self.assertGreaterEqual(len(assessment["reasons"]), 2)
        self.assertNotEqual(assessment["identification_schema"], assessment["diagnostics_schema"])
        with self.assertRaises(UnsupportedMergeError):
            require_mergeable(self.ident, diag)

    def test_equal_row_counts_or_zero_based_time_do_not_enable_merge(self):
        diag = self.dir / "same_shape.csv"
        diag.write_text("Time (s),value\n" + "\n".join(f"{i * 0.03},{i}" for i in range(30)) + "\n", encoding="utf-8")
        self.assertFalse(assess_merge_feasibility(self.ident, diag)["mergeable"])

    def test_shared_run_identity_and_clock_are_required(self):
        other_run = self.dir / "other.csv"
        other_run.write_text("timestamp,run_id,x\n1000.0,RUN_OTHER,1\n", encoding="utf-8")
        self.assertFalse(assess_merge_feasibility(self.ident, other_run)["mergeable"])
        same = self.dir / "same.csv"
        same.write_text("timestamp,run_id,x\n1000.0,RUN_T,1\n", encoding="utf-8")
        self.assertTrue(assess_merge_feasibility(self.ident, same)["mergeable"])


class RecordingStopTest(unittest.TestCase):
    def test_close_during_active_logging_keeps_accepted_samples_in_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = IdentificationLogger(output_dir=tmp, run_id="RUN_STOP")
            accepted = []

            def producer():
                for i in range(2000):
                    logger.log_sample(float(i) * 0.01, [float(i)] * 6, None, [1.0] * 6)
                    accepted.append(i)

            thread = threading.Thread(target=producer)
            thread.start()
            summary = logger.close()  # simulates stop/disconnect while the receive thread logs
            thread.join()
            logger.log_sample(99.0, [0.0] * 6, None, [1.0] * 6)  # after close: ignored, no error
            with open(logger.file_path, newline="") as handle:
                rows = list(csv.DictReader(handle))
            indices = [int(r["sample_index"]) for r in rows]
            self.assertEqual(indices, list(range(len(rows))))
            self.assertEqual(len(rows), summary["row_count"])
            self.assertEqual(summary["writer_dropped_sample_count"], 0)
            self.assertTrue(logger.file_path.exists() and logger.summary_path.exists())
            self.assertTrue(logger._file.closed)


if __name__ == "__main__":
    unittest.main()
