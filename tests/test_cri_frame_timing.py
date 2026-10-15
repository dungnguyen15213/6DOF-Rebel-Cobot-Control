import csv
import tempfile
import unittest
from pathlib import Path

from core.identification_logger import IdentificationLogger
from cri_lib.cri_controller import CRIController
from cri_lib.frame_timing import (
    TIMING_BATCH_EARLIER,
    TIMING_BATCH_LAST,
    TIMING_UNIQUE,
    FrameTimer,
    extract_frames,
)


def status_frame(counter: int, q1: float, current: str | None = "10 20 30 40 50 60") -> bytes:
    joints = " ".join([str(q1)] + ["0"] * 15)
    text = f"CRISTART {counter} STATUS POSJOINTCURRENT {joints}"
    if current is not None:
        text += f" CURRENTJOINTS {current}"
    return (text + " CRIEND\n").encode()


class Recorder:
    def __init__(self, controller: CRIController):
        self.samples = []
        controller.register_status_callback(self._callback)
        self.controller = controller

    def _callback(self, state, received_at):
        timing = self.controller.last_status_timing
        self.samples.append((state.joints_current.A1, received_at, timing, set(state.status_fields_seen)))


def feed(controller: CRIController, buffer: bytearray, data: bytes, t_rx: float) -> None:
    controller._process_received_chunk(buffer, data, t_rx)


class ExtractFramesTest(unittest.TestCase):
    def test_partial_frame_stays_buffered(self):
        buffer = bytearray(status_frame(1, 1.0)[:20])
        self.assertEqual(extract_frames(buffer), ([], 0))
        self.assertEqual(len(buffer), 20)

    def test_malformed_terminator_is_discarded_not_fatal(self):
        buffer = bytearray(b"garbage CRIEND\n" + status_frame(2, 2.0))
        frames, discarded = extract_frames(buffer)
        self.assertEqual(len(frames), 1)
        self.assertEqual(discarded, 1)

    def test_undecodable_bytes_are_counted(self):
        buffer = bytearray(b"CRISTART 1 STATUS \xff\xfe CRIEND\n")
        frames, discarded = extract_frames(buffer)
        self.assertEqual((frames, discarded), ([], 1))


class ControllerTimingTest(unittest.TestCase):
    def setUp(self):
        self.controller = CRIController()
        self.controller.frame_timer.restart_connection()
        self.recorder = Recorder(self.controller)
        self.buffer = bytearray()

    def test_one_frame_per_read_is_unique(self):
        for index in range(3):
            feed(self.controller, self.buffer, status_frame(index, float(index)), 10.0 + index)
        timings = [sample[2] for sample in self.recorder.samples]
        self.assertEqual([t.timing_class for t in timings], [TIMING_UNIQUE] * 3)
        self.assertEqual([t.frame_counter for t in timings], [0, 1, 2])
        self.assertEqual([t.chunk_id for t in timings], [0, 1, 2])
        self.assertEqual([s[1] for s in self.recorder.samples], [10.0, 11.0, 12.0])

    def test_multiple_frames_in_one_read_keep_order_and_are_labelled(self):
        data = b"".join(status_frame(i, float(i)) for i in range(3))
        feed(self.controller, self.buffer, data, 5.0)
        self.assertEqual([s[0] for s in self.recorder.samples], [0.0, 1.0, 2.0])
        self.assertEqual(
            [s[2].timing_class for s in self.recorder.samples],
            [TIMING_BATCH_EARLIER, TIMING_BATCH_EARLIER, TIMING_BATCH_LAST],
        )
        self.assertEqual({s[1] for s in self.recorder.samples}, {5.0})
        self.assertTrue({s[2].chunk_id for s in self.recorder.samples} == {0})
        self.assertTrue(all(s[2].frames_in_chunk == 3 for s in self.recorder.samples))
        parse_times = [s[2].t_frame_s for s in self.recorder.samples]
        self.assertEqual(parse_times, sorted(parse_times))

    def test_partial_frames_across_reads_use_completing_read_time(self):
        frame = status_frame(1, 7.0)
        feed(self.controller, self.buffer, frame[:15], 1.0)
        feed(self.controller, self.buffer, frame[15:40], 2.0)
        self.assertEqual(self.recorder.samples, [])
        feed(self.controller, self.buffer, frame[40:], 3.0)
        self.assertEqual(len(self.recorder.samples), 1)
        value, received_at, timing, _ = self.recorder.samples[0]
        self.assertEqual((value, received_at, timing.timing_class), (7.0, 3.0, TIMING_UNIQUE))

    def test_complete_plus_partial_next_frame(self):
        first, second = status_frame(1, 1.0), status_frame(2, 2.0)
        feed(self.controller, self.buffer, first + second[:10], 1.0)
        feed(self.controller, self.buffer, second[10:], 2.0)
        self.assertEqual([s[0] for s in self.recorder.samples], [1.0, 2.0])
        self.assertEqual([s[2].timing_class for s in self.recorder.samples], [TIMING_UNIQUE] * 2)

    def test_malformed_frame_does_not_break_following_frames(self):
        feed(self.controller, self.buffer, b"noise CRIEND\n" + status_frame(1, 4.0), 1.0)
        self.assertEqual([s[0] for s in self.recorder.samples], [4.0])
        self.assertEqual(self.controller.frame_timer.discarded_frames, 1)

    def test_connection_restart_resets_counters_and_advances_epoch(self):
        feed(self.controller, self.buffer, status_frame(1, 1.0), 1.0)
        first_epoch = self.recorder.samples[-1][2].connection_epoch
        self.controller.frame_timer.restart_connection()
        feed(self.controller, bytearray(), status_frame(1, 2.0), 2.0)
        timing = self.recorder.samples[-1][2]
        self.assertEqual(timing.connection_epoch, first_epoch + 1)
        self.assertEqual((timing.frame_counter, timing.chunk_id), (0, 0))

    def test_current_freshness_is_per_status_message(self):
        feed(self.controller, self.buffer, status_frame(1, 1.0), 1.0)
        feed(self.controller, self.buffer, status_frame(2, 2.0, current=None), 2.0)
        self.assertIn("CURRENTJOINTS", self.recorder.samples[0][3])
        self.assertNotIn("CURRENTJOINTS", self.recorder.samples[1][3])

    def test_no_timestamp_is_invented(self):
        data = b"".join(status_frame(i, float(i)) for i in range(4))
        feed(self.controller, self.buffer, data, 9.5)
        self.assertEqual({s[1] for s in self.recorder.samples}, {9.5})


class LoggerWithTimingTest(unittest.TestCase):
    def _record(self, tmp, reads):
        timer = FrameTimer()
        timer.restart_connection()
        with IdentificationLogger(output_dir=tmp, commanded_speed=50) as logger:
            value = 0.0
            for t_rx, count in reads:
                chunk = timer.begin_chunk()
                for index in range(count):
                    timing = timer.stamp_frame(chunk, index, count, t_rx)
                    logger.log_sample(t_rx, [value] * 6, None, [5.0] * 6, frame_timing=timing)
                    value += 1.0
            path = logger.file_path
        with open(path, newline="", encoding="utf-8") as handle:
            return path, list(csv.DictReader(handle))

    def test_columns_derivatives_and_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, rows = self._record(tmp, [(1.0, 1), (1.1, 3), (1.2, 1)])
            self.assertEqual([r["timing_class"] for r in rows],
                             ["UNIQUE", "BATCH_EARLIER", "BATCH_EARLIER", "BATCH_LAST", "UNIQUE"])
            self.assertEqual([r["frame_counter"] for r in rows], ["0", "1", "2", "3", "4"])
            self.assertEqual([r["frame_in_chunk"] for r in rows], ["0", "0", "1", "2", "0"])
            # Earlier batch frames have no derivative; the last frame differences against the prior usable row.
            self.assertEqual(rows[1]["qdot1"], "nan")
            self.assertEqual(rows[2]["qdot1"], "nan")
            self.assertAlmostEqual(float(rows[3]["qdot1"]), 3.0 / 0.1)
            self.assertAlmostEqual(float(rows[4]["qdot1"]), 1.0 / 0.1)
            # Raw q and currents are untouched.
            self.assertEqual([float(r["q1"]) for r in rows], [0.0, 1.0, 2.0, 3.0, 4.0])
            self.assertEqual({r["current6"] for r in rows}, {"5.0"})

    def test_batching_with_provenance_is_warning_not_fail(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            path, _ = self._record(tmp, [(1.0, 1), (1.1, 2), (1.2, 1), (1.3, 1)])
            report = json.loads(Path(f"{path}.json").read_text(encoding="utf-8"))
            self.assertEqual(report["quality_status"], "WARNING")
            self.assertEqual(report["quality_report"]["batched_row_count"], 2)

    def test_unlabelled_duplicate_timestamps_still_fail(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            with IdentificationLogger(output_dir=tmp, commanded_speed=50) as logger:
                logger.log_sample(1.0, [0.0] * 6, None, [5.0] * 6)
                logger.log_sample(1.0, [1.0] * 6, None, [5.0] * 6)
                path = logger.file_path
            report = json.loads(Path(f"{path}.json").read_text(encoding="utf-8"))
            self.assertEqual(report["quality_status"], "FAIL")


class AuditProvenanceTest(unittest.TestCase):
    def _batched_run(self, tmp, labels_ok=True):
        timer = FrameTimer()
        timer.restart_connection()
        with IdentificationLogger(output_dir=tmp, commanded_speed=50) as logger:
            for i in range(60):
                t_rx = 1.0 + 0.04 * i
                count = 2 if i % 3 == 1 else 1
                chunk = timer.begin_chunk()
                for index in range(count):
                    timing = timer.stamp_frame(chunk, index, count, t_rx)
                    logger.log_sample(t_rx, [float(i)] * 6, None, [5.0 + index] * 6, frame_timing=timing)
            return logger.file_path

    def test_labelled_batching_is_warning(self):
        from core.identification_audit import WARNING, audit_run
        with tempfile.TemporaryDirectory() as tmp:
            report = audit_run(self._batched_run(tmp))
            t2 = next(c for c in report["criteria"] if c["id"] == "T2")
            self.assertEqual(t2["status"], WARNING)
            self.assertEqual(t2["evidence"]["labels_inconsistent_with_timestamps"], 0)

    def test_labels_contradicting_timestamps_fail(self):
        from core.identification_audit import FAIL, audit_run
        with tempfile.TemporaryDirectory() as tmp:
            path = self._batched_run(tmp)
            text = Path(path).read_text(encoding="utf-8").replace("BATCH_EARLIER", "UNIQUE")
            Path(path).write_text(text, encoding="utf-8")
            report = audit_run(path)
            t2 = next(c for c in report["criteria"] if c["id"] == "T2")
            self.assertEqual(t2["status"], FAIL)


if __name__ == "__main__":
    unittest.main()
