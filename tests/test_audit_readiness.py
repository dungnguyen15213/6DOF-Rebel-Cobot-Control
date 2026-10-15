"""Synthetic-run tests for audit decisions and readiness (no robot or GUI required)."""
from __future__ import annotations

import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from core.identification_audit import audit_run, build_processed_dataset
from core.identification_logger import IDENTIFICATION_COLUMNS, IdentificationLogger, classify_channel
from core import run_report

N = 60


def make_rows(batched=False, validity="FRESH"):
    rows = []
    for i in range(N):
        ts = (i // 2 if batched else i) * 0.05 + 100.0
        if batched:
            label = "BATCH_EARLIER" if i % 2 == 0 else "BATCH_LAST"
        else:
            label = "UNIQUE"
        row = {c: "" for c in IDENTIFICATION_COLUMNS}
        row.update(
            timestamp=repr(ts), sample_index=i, experiment_id="ID", trajectory_id="T", run_id="RUN_X",
            commanded_speed=10, payload_g=0.0, contact_label="NORMAL", grasp_context="NONE",
            thermal_condition="COLD", connection_epoch=1, frame_counter=i, rx_chunk_id=i // 2 if batched else i,
            frame_in_chunk=i % 2 if batched else 0, frames_in_chunk=2 if batched else 1,
            t_frame_s=repr(ts), timing_class=label, status_frame_counter=1000 + i,
            position_validity=validity, current_validity=validity,
        )
        for j in range(1, 7):
            row[f"q{j}"] = 10.0 * math.sin(0.1 * i + j)
            row[f"qdot{j}"] = ""
            row[f"qddot{j}"] = ""
            row[f"current{j}"] = 300.0 + 50.0 * math.sin(0.2 * i + j)
        rows.append(row)
    return rows


def sidecar(rows=N, received=None, dropped=0, error=None):
    received = rows if received is None else received
    snap = lambda n: {"connection_epoch": 1, "status_frames": n, "discarded_frames": 0,
                      "parse_errors": 0, "status_callback_errors": 0}
    return {"quality_status": "PASS", "writer_dropped_sample_count": dropped, "writer_error": error,
            "frame_accounting": {"start": snap(1000), "end": snap(1000 + received)}}


def write_run(tmp, rows, columns=None, side=None):
    path = Path(tmp) / "run.csv"
    columns = columns or IDENTIFICATION_COLUMNS
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    Path(str(path) + ".json").write_text(json.dumps(side if side is not None else sidecar()), encoding="utf-8")
    return path


class ReadinessTests(unittest.TestCase):
    def verdict(self, rows, columns=None, side=None):
        with tempfile.TemporaryDirectory() as tmp:
            path = write_run(tmp, rows, columns, side)
            out = run_report.generate_run_report(path, Path(tmp) / "out")
            return json.loads((out / "RUN_AUDIT.json").read_text())

    def test_valid_run_is_ready(self):
        report = self.verdict(make_rows())
        self.assertEqual(report["decisions"]["overall_readiness"], "READY FOR CONTROLLED 0 G DRY TEST")
        self.assertEqual(report["decisions"]["current_freshness"]["status"], "PASS")

    def test_missing_position_is_not_ready(self):
        rows = make_rows(); rows[10]["q2"] = ""
        d = self.verdict(rows)["decisions"]
        self.assertEqual(d["raw_positions"]["status"], "FAIL")
        self.assertEqual(d["overall_readiness"], "NOT READY")

    def test_nan_and_inf_current_not_ready(self):
        for bad in ("nan", "inf"):
            rows = make_rows(); rows[5]["current3"] = bad
            d = self.verdict(rows)["decisions"]
            self.assertEqual(d["raw_current"]["status"], "FAIL", bad)
            self.assertEqual(d["overall_readiness"], "NOT READY", bad)

    def test_missing_required_column_not_ready(self):
        cols = [c for c in IDENTIFICATION_COLUMNS if c != "current3"]
        d = self.verdict(make_rows(), columns=cols)["decisions"]
        self.assertEqual(d["overall_readiness"], "NOT READY")

    def test_invalid_metadata_not_ready(self):
        rows = make_rows(); rows[30]["payload_g"] = 500.0
        self.assertEqual(self.verdict(rows)["decisions"]["overall_readiness"], "NOT READY")

    def test_missing_current_field_rows_not_ready(self):
        rows = make_rows()
        for r in rows[20:25]:
            r["current_validity"] = "MISSING"
            for j in range(1, 7):
                r[f"current{j}"] = "nan"
        d = self.verdict(rows)["decisions"]
        self.assertEqual(d["current_freshness"]["status"], "FAIL")
        self.assertEqual(d["overall_readiness"], "NOT READY")

    def test_legacy_run_without_freshness_is_conditional(self):
        cols = [c for c in IDENTIFICATION_COLUMNS if not c.endswith("_validity")]
        d = self.verdict(make_rows(), columns=cols)["decisions"]
        self.assertEqual(d["current_freshness"]["status"], "NOT VERIFIED")
        self.assertEqual(d["overall_readiness"], "CONDITIONALLY READY")

    def test_stationary_joint_is_not_a_failure(self):
        rows = make_rows()
        for r in rows:
            r["q4"] = 0.0
        d = self.verdict(rows)["decisions"]
        self.assertEqual(d["raw_positions"]["status"], "PASS")
        self.assertNotEqual(d["overall_readiness"], "NOT READY")

    def test_constant_current_channel_is_conditional(self):
        rows = make_rows()
        for r in rows:
            r["current6"] = 0.0
        d = self.verdict(rows)["decisions"]
        self.assertEqual(d["raw_current"]["status"], "WARNING")
        self.assertEqual(d["overall_readiness"], "CONDITIONALLY READY")

    def test_genuine_zero_with_fresh_flags_not_failed(self):
        rows = make_rows()
        for r in rows[:3]:
            for j in range(1, 7):
                r[f"current{j}"] = 0.0
        report = self.verdict(rows)
        self.assertNotEqual(report["decisions"]["overall_readiness"], "NOT READY")
        i5 = [c for c in report["audit"]["criteria"] if c["id"] == "I5"][0]
        self.assertEqual(i5["status"], "PASS")

    def test_labelled_duplicate_timestamps_are_warning_only(self):
        d = self.verdict(make_rows(batched=True))["decisions"]
        self.assertEqual(d["receive_timing"]["status"], "WARNING")
        self.assertNotEqual(d["overall_readiness"], "NOT READY")

    def test_inconsistent_batching_labels_not_ready(self):
        rows = make_rows(batched=True)
        rows[1]["timing_class"] = "UNIQUE"
        report = self.verdict(rows)
        self.assertEqual(report["decisions"]["receive_timing"]["status"], "FAIL")
        self.assertEqual(report["decisions"]["overall_readiness"], "NOT READY")

    def test_writer_drop_and_error_not_ready(self):
        for side in (sidecar(dropped=3), sidecar(error="disk full")):
            d = self.verdict(make_rows(), side=side)["decisions"]
            self.assertEqual(d["logger_integrity"]["status"], "FAIL")
            self.assertEqual(d["overall_readiness"], "NOT READY")

    def test_lost_status_frames_not_ready(self):
        d = self.verdict(make_rows(), side=sidecar(received=N + 25))["decisions"]
        self.assertEqual(d["status_frame_accounting"]["status"], "FAIL")
        self.assertEqual(d["overall_readiness"], "NOT READY")

    def test_json_and_markdown_agree(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = make_rows(); rows[3]["current2"] = "nan"
            out = run_report.generate_run_report(write_run(tmp, rows), Path(tmp) / "out")
            data = json.loads((out / "RUN_AUDIT.json").read_text())
            md = (out / "RUN_AUDIT.md").read_text()
            self.assertIn(data["decisions"]["overall_readiness"], md)
            self.assertEqual(data["audit_overall_status"], "FAIL")

    def test_cli_exit_codes(self):
        with tempfile.TemporaryDirectory() as tmp:
            good = write_run(tmp, make_rows())
            self.assertEqual(run_report.main([str(good), "--out-root", str(Path(tmp) / "o")]), 0)
            rows = make_rows(); rows[0]["q1"] = ""
            bad = write_run(tmp, rows)
            self.assertEqual(run_report.main([str(bad), "--out-root", str(Path(tmp) / "o")]), 1)


class ValidityClassificationTests(unittest.TestCase):
    def test_states(self):
        six = [1.0] * 6
        self.assertEqual(classify_channel({"CURRENTJOINTS"}, "CURRENTJOINTS", six), "FRESH")
        self.assertEqual(classify_channel({"CURRENTJOINTS"}, "CURRENTJOINTS", [0.0] * 6), "FRESH")  # genuine zero
        self.assertEqual(classify_channel({"MODE"}, "CURRENTJOINTS", [float("nan")] * 6), "MISSING")
        self.assertEqual(classify_channel({"CURRENTJOINTS"}, "CURRENTJOINTS", [1, 2, float("nan"), 4, 5, 6]), "NON_FINITE")
        self.assertEqual(classify_channel({"CURRENTJOINTS"}, "CURRENTJOINTS", [1, 2, float("inf"), 4, 5, 6]), "NON_FINITE")
        self.assertEqual(classify_channel(None, "CURRENTJOINTS", six), "UNKNOWN")

    def test_logger_writes_validity_and_counts(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = IdentificationLogger(output_dir=tmp, experiment_id="E", trajectory_id="T", run_id="R",
                                          commanded_speed=10, payload_g=0, contact_label="NORMAL",
                                          grasp_context="NONE", thermal_condition="COLD")
            q = [1.0] * 6
            logger.log_sample(1.0, q, None, [0.0] * 6, status_fields={"POSJOINTCURRENT", "CURRENTJOINTS"})
            logger.log_sample(1.1, q, None, [float("nan")] * 6, status_fields={"POSJOINTCURRENT"})
            summary = logger.close()
            with open(logger.file_path, newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r["current_validity"] for r in rows], ["FRESH", "MISSING"])
            self.assertEqual(rows[0]["current1"], "0.0")  # measured zero preserved
            self.assertEqual(rows[1]["current1"].lower(), "nan")
            self.assertEqual(summary["validity_accounting"]["current"]["MISSING"], 1)


class ProcessedDatasetTests(unittest.TestCase):
    def test_short_intervals_never_create_derivative_spikes(self):
        rows = make_rows()
        # force rows 20..24 to be 0.2 ms apart
        for k, i in enumerate(range(20, 25)):
            rows[i]["timestamp"] = repr(110.0 + k * 0.0002)
        for i in range(25, N):
            rows[i]["timestamp"] = repr(110.1 + (i - 25) * 0.05)
        with tempfile.TemporaryDirectory() as tmp:
            raw = write_run(tmp, rows)
            before = raw.read_bytes()
            prov = build_processed_dataset(raw, Path(tmp) / "p" / "proc.csv")
            self.assertEqual(before, raw.read_bytes())
            with open(Path(tmp) / "p" / "proc.csv") as handle:
                out = list(csv.DictReader(handle))
            self.assertEqual(len(out), N)  # original rows preserved
            for i in range(21, 24):
                self.assertEqual(out[i]["qdot1_rad_s"], "nan")
                self.assertEqual(out[i]["qddot1_rad_s2"], "nan")
            self.assertIn("exclusions", prov)
            finite = [abs(float(r["qdot1_rad_s"])) for r in out if r["qdot1_rad_s"] != "nan"]
            self.assertLess(max(finite), 5.0)


if __name__ == "__main__":
    unittest.main()
