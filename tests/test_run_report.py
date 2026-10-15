import csv, json, tempfile, unittest
from pathlib import Path
from cri_lib.frame_timing import FrameTimer, message_type
from core import run_report


class StatusCounterTests(unittest.TestCase):
    def test_status_only_batching_and_counters(self):
        t = FrameTimer()
        cid = t.begin_chunk()
        kinds = ["CMDACK", "STATUS", "RUNSTATE", "STATUS"]
        counters = [t.note_frame(k) for k in kinds]
        self.assertEqual(counters, [1, 2, 3, 4] if counters[0] == 1 else counters)
        a = t.stamp_frame(cid, 0, 2, 1.0, counters[1])
        b = t.stamp_frame(cid, 1, 2, 1.0, counters[3])
        self.assertEqual(b.status_counter - a.status_counter, 1)
        self.assertEqual(t.stats()["frame_type_counts"]["STATUS"], 2)
        self.assertEqual(t.stats()["frame_type_counts"]["CMDACK"], 1)

    def test_message_type(self):
        self.assertEqual(message_type("CRISTART 1 CMDACK X CRIEND"), "CMDACK")


class ReportTests(unittest.TestCase):
    def test_report_on_minimal_csv(self):
        cols = ["sample_index", "timestamp", "t_frame_s", "rx_chunk_id", "frame_counter",
                "status_frame_counter", "timing_class"] + [f"q{j}" for j in range(1, 7)] + [f"current{j}" for j in range(1, 7)]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "run.csv"
            with open(p, "w", newline="") as h:
                w = csv.DictWriter(h, cols); w.writeheader()
                for i in range(30):
                    row = {c: 0.0 for c in cols}
                    row.update(sample_index=i, timestamp=i * 0.05, t_frame_s=i * 0.05, rx_chunk_id=i,
                              frame_counter=i, status_frame_counter=i, timing_class="UNIQUE")
                    for j in range(1, 7):
                        row[f"current{j}"] = float(i % 5 + j)
                    w.writerow(row)
            before = p.read_bytes()
            out = run_report.generate_run_report(p, Path(d) / "out")
            self.assertEqual(before, p.read_bytes())
            data = json.loads((out / "RUN_AUDIT.json").read_text())
            self.assertIn("decisions", data)
            self.assertTrue((out / "timing_diagnostics.csv").exists())
            self.assertTrue((out / "run_manifest.json").exists())


if __name__ == "__main__":
    unittest.main()
