from __future__ import annotations

import csv
import json
import math
import statistics
import threading
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Full, Queue
from typing import Any, Iterable

Q_VECTOR_COLUMNS = [f"q{i}" for i in range(1, 7)]
QDOT_VECTOR_COLUMNS = [f"qdot{i}" for i in range(1, 7)]
QDDOT_VECTOR_COLUMNS = [f"qddot{i}" for i in range(1, 7)]
CURRENT_VECTOR_COLUMNS = [f"current{i}" for i in range(1, 7)]
METADATA_COLUMNS = [
    "experiment_id",
    "trajectory_id",
    "run_id",
    "commanded_speed",
    "payload_g",
    "contact_label",
    "grasp_context",
    "thermal_condition",
]

# Frame provenance, appended so earlier column positions are unchanged. "timestamp"
# stays the host receive time of the TCP read; frames from one multi-frame read
# share it, and timing_class marks which rows may be used for time derivatives.
TIMING_COLUMNS = [
    "connection_epoch",
    "frame_counter",
    "rx_chunk_id",
    "frame_in_chunk",
    "frames_in_chunk",
    "t_frame_s",
    "timing_class",
    "status_frame_counter",
]

# Per-row validity of the raw channels, from the STATUS message that produced the row.
# FRESH: the field was present in this very STATUS and all six values are finite
#   (a measured 0 is FRESH). MISSING: field absent from this STATUS (value logged as
#   nan, the cached RobotState value is never reused). NON_FINITE: field present but
#   a value did not parse to a finite number. UNKNOWN: caller gave no field info.
VALIDITY_COLUMNS = ["position_validity", "current_validity"]
VALIDITY_STATES = ("FRESH", "MISSING", "NON_FINITE", "UNKNOWN")


def classify_channel(status_fields: Any, field: str, values: Iterable[float]) -> str:
    if status_fields is None:
        return "UNKNOWN"
    if field not in status_fields:
        return "MISSING"
    return "FRESH" if all(math.isfinite(v) for v in values) else "NON_FINITE"

# Live derivatives need receive intervals at least this long. Shorter intervals
# are host-scheduling artefacts (RUN_02: 0.2 ms between reads), not sampling
# intervals, and would turn quantisation steps into huge velocities. Heuristic,
# not a robot specification; the raw CSV is unaffected.
MIN_DERIVATIVE_DT_S = 0.01

IDENTIFICATION_COLUMNS = [
    "timestamp",
    "sample_index",
    *Q_VECTOR_COLUMNS,
    *QDOT_VECTOR_COLUMNS,
    *CURRENT_VECTOR_COLUMNS,
    *QDDOT_VECTOR_COLUMNS,
    *METADATA_COLUMNS,
    *TIMING_COLUMNS,
    *VALIDITY_COLUMNS,
]

USABLE_TIMING_CLASSES = {"", "UNKNOWN", "UNIQUE", "BATCH_LAST"}


def _timing_fields(frame_timing: Any) -> dict[str, Any]:
    """Flatten optional FrameTiming provenance; legacy callers get UNKNOWN."""
    if frame_timing is None:
        return {"timing_class": "UNKNOWN"}
    return {
        "connection_epoch": frame_timing.connection_epoch,
        "frame_counter": frame_timing.frame_counter,
        "rx_chunk_id": frame_timing.chunk_id,
        "frame_in_chunk": frame_timing.frame_in_chunk,
        "frames_in_chunk": frame_timing.frames_in_chunk,
        "t_frame_s": repr(float(frame_timing.t_frame_s)),
        "timing_class": frame_timing.timing_class,
        "status_frame_counter": frame_timing.status_counter,
    }


def _sanitize_filename_component(value: str | None, fallback: str) -> str:
    text = (value or fallback).strip()
    if not text:
        text = fallback
    sanitized = "".join(ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in text)
    sanitized = sanitized.strip("_") or fallback
    return sanitized


def _coerce_float(value: Any, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_float_list(values: Iterable[Any], length: int = 6) -> list[float]:
    result: list[float] = []
    for value in values:
        if value is None or value == "":
            result.append(float("nan"))
        else:
            try:
                result.append(float(value))
            except (TypeError, ValueError):
                result.append(float("nan"))
    if len(result) < length:
        result.extend([float("nan")] * (length - len(result)))
    return result[:length]


def find_existing_identification_run(
    output_dir: str | Path, run_id: str
) -> Path | None:
    """Return a prior CSV using this run ID, if present."""
    for csv_path in Path(output_dir).glob("identification_*.csv"):
        try:
            with open(csv_path, "r", newline="", encoding="utf-8") as csvfile:
                row = next(csv.DictReader(csvfile), None)
        except (OSError, UnicodeError, csv.Error):
            row = None
        if row is not None and str(row.get("run_id", "")).strip() == str(run_id).strip():
            return csv_path
        sidecar_path = Path(f"{csv_path}.json")
        try:
            with open(sidecar_path, "r", encoding="utf-8") as sidecar_file:
                summary = json.load(sidecar_file)
        except (OSError, UnicodeError, json.JSONDecodeError):
            continue
        if str(summary.get("metadata", {}).get("run_id", "")).strip() == str(run_id).strip():
            return csv_path
    return None


class IdentificationLogger:
    """Queue telemetry quickly and write a traceable identification run in a worker."""

    QUEUE_CAPACITY = 4096
    FLUSH_EVERY_SAMPLES = 128
    FLUSH_INTERVAL_SECONDS = 1.0

    def __init__(
        self,
        output_dir: str | Path = "identification_data",
        experiment_id: str = "ID_NORMAL_001",
        trajectory_id: str = "UNSPECIFIED_TRAJECTORY",
        run_id: str = "RUN_01",
        commanded_speed: float | str | None = None,
        payload_g: float | str | int = 0,
        contact_label: str = "NORMAL",
        grasp_context: str = "NONE",
        thermal_condition: str = "COLD",
        overwrite: bool = False,
        filename: str | None = None,
        queue_capacity: int | None = None,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._metadata = {
            "experiment_id": experiment_id,
            "trajectory_id": trajectory_id,
            "run_id": run_id,
            "commanded_speed": str(commanded_speed if commanded_speed is not None else ""),
            "payload_g": str(payload_g),
            "contact_label": contact_label,
            "grasp_context": grasp_context,
            "thermal_condition": thermal_condition,
        }

        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        if filename is None:
            filename = (
                f"identification_{_sanitize_filename_component(self._metadata['experiment_id'], 'ID')}"
                f"_{_sanitize_filename_component(self._metadata['trajectory_id'], 'TRAJ')}"
                f"_{_sanitize_filename_component(self._metadata['run_id'], 'RUN')}_{timestamp}.csv"
            )

        requested_path = self.output_dir / Path(filename).name
        self.file_path, self._file = self._open_unique_file(requested_path, overwrite)
        self._writer = csv.DictWriter(self._file, fieldnames=IDENTIFICATION_COLUMNS)
        self._writer.writeheader()
        self._file.flush()
        self.summary_path = Path(f"{self.file_path}.json")
        self._queue: Queue[Any] = Queue(maxsize=queue_capacity or self.QUEUE_CAPACITY)
        self._sentinel = object()
        self._state_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._accepting_samples = True
        self._closed = False
        self._summary: dict[str, Any] | None = None
        self._next_sample_index = 0
        self.row_count = 0
        self.dropped_sample_count = 0
        self.samples_offered = 0
        self.position_validity_counts = {state: 0 for state in VALIDITY_STATES}
        self.current_validity_counts = {state: 0 for state in VALIDITY_STATES}
        self.samples_q_complete = 0
        self.samples_current_complete = 0
        self.writer_error: str | None = None
        self._first_timestamp: float | None = None
        self._last_timestamp: float | None = None
        self._wall_start = datetime.now(timezone.utc).isoformat()
        self._worker = threading.Thread(
            target=self._writer_loop,
            name="identification-csv-writer",
            daemon=True,
        )
        self._worker.start()

    @property
    def metadata(self) -> dict[str, str]:
        return dict(self._metadata)

    @staticmethod
    def _open_unique_file(path: Path, overwrite: bool):
        if overwrite:
            return path, open(path, "w", newline="", encoding="utf-8")
        candidate = path
        index = 1
        while True:
            try:
                return candidate, open(candidate, "x", newline="", encoding="utf-8")
            except FileExistsError:
                candidate = path.with_name(f"{path.stem}_{index}{path.suffix}")
                index += 1

    def log_sample(
        self,
        timestamp: float,
        q: Iterable[Any],
        qdot: Iterable[Any],
        current: Iterable[Any],
        qddot: Iterable[Any] | None = None,
        metadata: dict[str, Any] | None = None,
        frame_timing: Any = None,
        status_fields: Any = None,
    ) -> None:
        del qdot, qddot, metadata  # Legacy arguments cannot alter frozen run data.
        timing_fields = _timing_fields(frame_timing)
        sample_timestamp = _coerce_float(timestamp, float("nan"))
        sample_q = tuple(_as_float_list(q, 6))
        sample_current = tuple(_as_float_list(current, 6))
        timing_fields["position_validity"] = classify_channel(status_fields, "POSJOINTCURRENT", sample_q)
        timing_fields["current_validity"] = classify_channel(status_fields, "CURRENTJOINTS", sample_current)
        with self._state_lock:
            if not self._accepting_samples:
                return
            sample_index = self._next_sample_index
            self._next_sample_index += 1
            self.samples_offered += 1
            self.position_validity_counts[timing_fields["position_validity"]] += 1
            self.current_validity_counts[timing_fields["current_validity"]] += 1
            self.samples_q_complete += all(math.isfinite(v) for v in sample_q)
            self.samples_current_complete += all(math.isfinite(v) for v in sample_current)
            try:
                self._queue.put_nowait(
                    (sample_index, sample_timestamp, sample_q, sample_current, timing_fields)
                )
            except Full:
                self.dropped_sample_count += 1

    def _writer_loop(self) -> None:
        previous_timestamp: float | None = None
        previous_q: tuple[float, ...] | None = None
        previous_qdot: list[float] | None = None
        previous_epoch: Any = None
        last_flush = time.monotonic()
        unflushed_rows = 0

        while True:
            timeout = max(0.0, self.FLUSH_INTERVAL_SECONDS - (time.monotonic() - last_flush))
            try:
                sample = self._queue.get(timeout=timeout)
            except Empty:
                sample = None

            if sample is self._sentinel:
                self._queue.task_done()
                break
            if sample is not None:
                sample_index, timestamp, q_values, current_values, timing_fields = sample
                usable = timing_fields["timing_class"] in USABLE_TIMING_CLASSES
                epoch = timing_fields.get("connection_epoch")
                if epoch != previous_epoch:
                    previous_timestamp = previous_q = previous_qdot = None
                    previous_epoch = epoch
                dt = (
                    timestamp - previous_timestamp
                    if previous_timestamp is not None
                    else float("nan")
                )
                # Earlier frames of a multi-frame read have no distinct measured time.
                dt_ok = usable and math.isfinite(dt) and dt >= MIN_DERIVATIVE_DT_S
                if dt_ok and previous_q is not None:
                    qdot_values = [
                        (q_values[index] - previous_q[index]) / dt
                        if math.isfinite(q_values[index]) and math.isfinite(previous_q[index])
                        else float("nan")
                        for index in range(6)
                    ]
                else:
                    qdot_values = [float("nan")] * 6

                if dt_ok and previous_qdot is not None:
                    qddot_values = [
                        (qdot_values[index] - previous_qdot[index]) / dt
                        if math.isfinite(qdot_values[index]) and math.isfinite(previous_qdot[index])
                        else float("nan")
                        for index in range(6)
                    ]
                else:
                    qddot_values = [float("nan")] * 6

                row: dict[str, Any] = {
                    "timestamp": timestamp,
                    "sample_index": sample_index,
                    **dict(zip(Q_VECTOR_COLUMNS, q_values)),
                    **dict(zip(QDOT_VECTOR_COLUMNS, qdot_values)),
                    **dict(zip(CURRENT_VECTOR_COLUMNS, current_values)),
                    **dict(zip(QDDOT_VECTOR_COLUMNS, qddot_values)),
                    **self._metadata,
                    **timing_fields,
                }
                try:
                    self._writer.writerow(row)
                    self.row_count += 1
                    unflushed_rows += 1
                    if self._first_timestamp is None:
                        self._first_timestamp = timestamp
                    self._last_timestamp = timestamp
                except (OSError, csv.Error) as exc:
                    self.writer_error = str(exc)
                    self.dropped_sample_count += 1

                if usable:
                    previous_timestamp = timestamp
                    previous_q = q_values
                    previous_qdot = qdot_values if any(map(math.isfinite, qdot_values)) else None
                self._queue.task_done()

            now = time.monotonic()
            if unflushed_rows >= self.FLUSH_EVERY_SAMPLES or now - last_flush >= self.FLUSH_INTERVAL_SECONDS:
                try:
                    self._file.flush()
                except OSError as exc:
                    self.writer_error = str(exc)
                unflushed_rows = 0
                last_flush = now

        try:
            self._file.flush()
        except OSError as exc:
            self.writer_error = str(exc)
        finally:
            self._file.close()

    def close(self, frame_accounting: dict[str, Any] | None = None) -> dict[str, Any] | None:
        with self._close_lock:
            if self._closed:
                return self._summary
            with self._state_lock:
                self._accepting_samples = False
                self._closed = True
            self._queue.put(self._sentinel)
            self._worker.join()
            end_timestamp = self._last_timestamp
            report = analyze_identification_csv(self.file_path)
            status = _quality_status(report, self.dropped_sample_count, self.writer_error)
            self._summary = {
                "schema_version": 1,
                "file_path": str(self.file_path),
                "summary_path": str(self.summary_path),
                "start_timestamp_utc": self._wall_start,
                "end_timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "first_receive_timestamp_s": self._first_timestamp,
                "last_receive_timestamp_s": end_timestamp,
                "duration_s": (
                    end_timestamp - self._first_timestamp
                    if end_timestamp is not None and self._first_timestamp is not None
                    else 0.0
                ),
                "row_count": self.row_count,
                "metadata": self.metadata,
                "units": report["schema_summary"]["units"],
                "signal_provenance": {
                    "timestamp": "host perf_counter at the end of the TCP recv() that completed the STATUS frame; frames of one read share it (see timing_class); CRI STATUS carries no device timestamp",
                    "timing_columns": "connection_epoch, frame_counter, rx_chunk_id, frame_in_chunk, frames_in_chunk, t_frame_s (parse-order time, not acquisition time), timing_class",
                    "q": "raw CRI STATUS POSJOINTCURRENT",
                    "current": "raw CRI STATUS CURRENTJOINTS",
                    "qdot": "derived from consecutive q samples using measured timestamp delta",
                    "qddot": "candidate derived signal; unavailable until sufficient samples exist",
                },
                "writer_dropped_sample_count": self.dropped_sample_count,
                "logger_accounting": {
                    "samples_offered_to_logger": self.samples_offered,
                    "samples_with_complete_q": self.samples_q_complete,
                    "samples_with_complete_current": self.samples_current_complete,
                    "rows_written": self.row_count,
                    "dropped_by_queue_or_writer": self.dropped_sample_count,
                },
                "frame_accounting": frame_accounting,
                "validity_accounting": {
                    "position": dict(self.position_validity_counts),
                    "current": dict(self.current_validity_counts),
                    "note": "FRESH = field present in the same STATUS and finite (measured 0 is FRESH); MISSING/NON_FINITE rows are logged as nan, never as 0",
                },
                "live_derivative_policy": {
                    "min_dt_s": MIN_DERIVATIVE_DT_S,
                    "rule": "qdot/qddot are NaN (unavailable, not zero) for BATCH_EARLIER rows, dt < min_dt_s, or non-finite inputs; they are derived, never ground truth",
                },
                "writer_error": self.writer_error,
                "quality_status": status,
                "quality_report": report,
            }
            sidecar_index = 1
            while True:
                try:
                    summary_file = open(self.summary_path, "x", encoding="utf-8")
                    break
                except FileExistsError:
                    self.summary_path = self.summary_path.with_name(
                        f"{Path(self.summary_path).stem}_{sidecar_index}{self.summary_path.suffix}"
                    )
                    sidecar_index += 1
            self._summary["summary_path"] = str(self.summary_path)
            with summary_file:
                json.dump(_json_safe(self._summary), summary_file, indent=2, allow_nan=False)
            return self._summary

    def __enter__(self) -> "IdentificationLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _quality_status(report: dict[str, Any], dropped: int, writer_error: str | None) -> str:
    raw_columns = [*Q_VECTOR_COLUMNS, *CURRENT_VECTOR_COLUMNS]
    missing_raw = any(report["missing_values"].get(column, 0) for column in raw_columns)
    if (
        report["total_rows"] == 0
        or report["schema_summary"]["missing_columns"]
        or missing_raw
        or report.get("incomplete_current_rows", 0)
        or report["missing_values"].get("timestamp", 0)
        or report.get("invalid_numeric_values")
        or not report["monotonic"]
        or (
            report.get("usable_timestamp_duplicates", 0)
            if report.get("timing_provenance_present")
            else report["duplicate_timestamps"]
        )
        or report.get("metadata_inconsistent", False)
        or any(report.get("metadata_missing_rows", {}).values())
        or report.get("sample_index_gap_count", 0)
        or report.get("malformed_row_count", 0)
        or dropped
        or writer_error
    ):
        return "FAIL"
    if (
        report.get("large_gap_count", 0)
        or report.get("batched_row_count", 0)
        or report.get("suspicious_all_zero_current_rows", 0)
        or any(
            values["adjacent_repeat_count"]
            for values in report.get("repeated_current_values", {}).values()
        )
        or any(
        report["missing_values"].get(column, 0)
        for column in (*QDOT_VECTOR_COLUMNS, *QDDOT_VECTOR_COLUMNS)
        )
    ):
        return "WARNING"
    return "PASS"


def _safe_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


def analyze_identification_csv(path: str | Path) -> dict[str, Any]:
    """Analyze run integrity without imposing scientific acceptance thresholds."""
    csv_path = Path(path)
    with open(csv_path, "r", newline="", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile, strict=True)
        fieldnames = reader.fieldnames or []
        rows = list(reader)

    required_columns = [
        "timestamp", *Q_VECTOR_COLUMNS, *QDOT_VECTOR_COLUMNS,
        *CURRENT_VECTOR_COLUMNS, *METADATA_COLUMNS,
    ]
    missing_columns = [column for column in required_columns if column not in fieldnames]
    timestamps = [_safe_float(row.get("timestamp")) for row in rows]
    valid_timestamps = [value for value in timestamps if value is not None]
    dt_values = [
        timestamps[index] - timestamps[index - 1]
        for index in range(1, len(timestamps))
        if timestamps[index] is not None and timestamps[index - 1] is not None
    ]
    positive_dt = [value for value in dt_values if value > 0.0]
    timestamp_counts = Counter(valid_timestamps)
    duplicate_values = {value for value, count in timestamp_counts.items() if count > 1}
    timing_classes = [str(row.get("timing_class") or "").strip() for row in rows]
    class_counts = Counter(timing_classes)
    provenance_present = any(value in {"UNIQUE", "BATCH_LAST", "BATCH_EARLIER"} for value in class_counts)
    usable_timestamps = [
        value
        for value, klass in zip(timestamps, timing_classes)
        if value is not None and klass in USABLE_TIMING_CLASSES
    ]
    usable_counts = Counter(usable_timestamps)
    usable_duplicates = sum(count - 1 for count in usable_counts.values() if count > 1)
    usable_dt = [b - a for a, b in zip(usable_timestamps, usable_timestamps[1:])]
    batched_rows = class_counts.get("BATCH_LAST", 0) + class_counts.get("BATCH_EARLIER", 0)
    dt_sorted = sorted(dt_values)

    def percentile(values: list[float], percent: float) -> float | None:
        if not values:
            return None
        index = max(0, min(len(values) - 1, math.ceil(percent / 100.0 * len(values)) - 1))
        return values[index]

    missing_values: dict[str, int] = {}
    non_finite_values: dict[str, int] = {}
    invalid_numeric_values: dict[str, int] = {}
    numeric_columns = {
        "timestamp", *Q_VECTOR_COLUMNS, *QDOT_VECTOR_COLUMNS,
        *CURRENT_VECTOR_COLUMNS, *QDDOT_VECTOR_COLUMNS,
    }
    for column in fieldnames:
        absent = 0
        non_finite = 0
        for row in rows:
            text = row.get(column)
            if text is None or text.strip() == "":
                absent += 1
            elif column in numeric_columns and _safe_float(text) is None:
                absent += 1
                if text.strip().lower() in {"nan", "inf", "+inf", "-inf", "infinity", "+infinity", "-infinity"}:
                    non_finite += 1
                else:
                    invalid_numeric_values[column] = invalid_numeric_values.get(column, 0) + 1
        if absent:
            missing_values[column] = absent
        if non_finite:
            non_finite_values[column] = non_finite

    def ranges(columns: list[str]) -> dict[str, dict[str, float | None]]:
        result = {}
        for column in columns:
            values = [
                value for row in rows
                if (value := _safe_float(row.get(column))) is not None
            ]
            result[column] = {
                "min": min(values) if values else None,
                "max": max(values) if values else None,
            }
        return result

    metadata_values = {
        key: sorted({str(row.get(key, "")).strip() for row in rows})
        for key in METADATA_COLUMNS
    }
    metadata_consistency = {
        key: len({value for value in values if value}) <= 1
        for key, values in metadata_values.items()
    }
    metadata_missing = {
        key: sum(not str(row.get(key, "")).strip() for row in rows)
        for key in METADATA_COLUMNS
    }
    current_columns_present = all(column in fieldnames for column in CURRENT_VECTOR_COLUMNS)
    incomplete_current_rows = sum(
        any(_safe_float(row.get(column)) is None for column in CURRENT_VECTOR_COLUMNS)
        for row in rows
    )
    all_zero_current_rows = sum(
        all(_safe_float(row.get(column)) == 0.0 for column in CURRENT_VECTOR_COLUMNS)
        for row in rows
    )
    repeated_current_values: dict[str, dict[str, int]] = {}
    for column in CURRENT_VECTOR_COLUMNS:
        previous = None
        run_length = 0
        longest_run = 0
        repeated_count = 0
        for row in rows:
            current = _safe_float(row.get(column))
            if current is not None and current == previous:
                run_length += 1
                repeated_count += 1
                longest_run = max(longest_run, run_length + 1)
            else:
                run_length = 0
            previous = current
        repeated_current_values[column] = {
            "adjacent_repeat_count": repeated_count,
            "longest_identical_run": longest_run,
        }

    median_dt = statistics.median(dt_values) if dt_values else None
    gap_limit = median_dt * 5.0 if median_dt is not None and median_dt > 0.0 else None
    large_gaps = [value for value in dt_values if gap_limit is not None and value > gap_limit]
    sample_rates = [1.0 / value for value in positive_dt]
    sample_index_values = [_safe_float(row.get("sample_index")) for row in rows]
    sample_index_gaps = sum(
        current is not None and previous is not None and current - previous > 1
        for previous, current in zip(sample_index_values, sample_index_values[1:])
    )
    malformed_row_count = sum(None in row for row in rows)

    report = {
        "file_path": str(csv_path),
        "total_rows": len(rows),
        "duration_s": valid_timestamps[-1] - valid_timestamps[0] if len(valid_timestamps) > 1 else 0.0,
        "first_timestamp": valid_timestamps[0] if valid_timestamps else None,
        "last_timestamp": valid_timestamps[-1] if valid_timestamps else None,
        "timestamp_monotonic": all(value >= 0.0 for value in dt_values),
        "monotonic": all(value >= 0.0 for value in dt_values),
        "duplicate_timestamp_values": len(duplicate_values),
        "duplicate_timestamps": sum(value in duplicate_values for value in valid_timestamps),
        "dt_nonpositive_count": sum(value <= 0.0 for value in dt_values),
        "timing_provenance_present": provenance_present,
        "timing_class_counts": dict(class_counts),
        "batched_row_count": batched_rows,
        "batched_row_fraction": batched_rows / len(rows) if rows else 0.0,
        "usable_timestamp_duplicates": usable_duplicates,
        "usable_nonincreasing_dt_count": sum(value <= 0.0 for value in usable_dt),
        "dt_negative_count": sum(value < 0.0 for value in dt_values),
        "invalid_dt_values": sum(value <= 0.0 for value in dt_values),
        "dt_min": min(dt_values) if dt_values else None,
        "dt_median": median_dt,
        "dt_mean": statistics.fmean(dt_values) if dt_values else None,
        "dt_max": max(dt_values) if dt_values else None,
        "dt_percentiles": {
            f"P{percent}": percentile(dt_sorted, percent)
            for percent in (1, 5, 95, 99)
        },
        "approx_sample_rate_hz": {
            "min": min(sample_rates) if sample_rates else None,
            "median": statistics.median(sample_rates) if sample_rates else None,
            "mean": statistics.fmean(sample_rates) if sample_rates else None,
            "max": max(sample_rates) if sample_rates else None,
            "interval_count": len(sample_rates),
        },
        "large_gap_count": len(large_gaps),
        "large_gap_threshold_s_heuristic": gap_limit,
        "large_gap_values_s": large_gaps,
        "missing_values": missing_values,
        "non_finite_values": non_finite_values,
        "invalid_numeric_values": invalid_numeric_values,
        "incomplete_current_rows": incomplete_current_rows,
        "current_columns_present": current_columns_present,
        "suspicious_all_zero_current_rows": all_zero_current_rows,
        "repeated_current_values": repeated_current_values,
        "q_ranges": ranges(Q_VECTOR_COLUMNS),
        "velocity_ranges": ranges(QDOT_VECTOR_COLUMNS),
        "current_extrema": ranges(CURRENT_VECTOR_COLUMNS),
        "acceleration_ranges": ranges(QDDOT_VECTOR_COLUMNS),
        "experiment_id_values": metadata_values["experiment_id"],
        "trajectory_id_values": metadata_values["trajectory_id"],
        "run_id_values": metadata_values["run_id"],
        "metadata_values": metadata_values,
        "metadata_consistency": metadata_consistency,
        "metadata_inconsistent": not all(metadata_consistency.values()),
        "metadata_missing_rows": metadata_missing,
        "sample_index_gap_count": sample_index_gaps,
        "malformed_row_count": malformed_row_count,
        "schema_summary": {
            "required_columns": required_columns,
            "optional_columns": QDDOT_VECTOR_COLUMNS + ["sample_index"],
            "missing_columns": missing_columns,
            "units": {
                "timestamp": "seconds from host monotonic clock at CRI STATUS reception",
                "sample_index": "zero-based receive-callback sample sequence",
                "q1..q6": "degrees (raw CRI STATUS POSJOINTCURRENT)",
                "qdot1..q6": "deg/s (derived using measured timestamp delta)",
                "current1..current6": "mA (raw CRI STATUS CURRENTJOINTS)",
                "qddot1..qddot6": "deg/s^2 (candidate derived signal)",
            },
            "notes": "No verified current clipping limit was found; extrema are reported and clipping limit is unknown.",
        },
        "current_limit_verified": False,
        "current_clipping_limit": "unknown; no verified repository/manufacturer limit configured",
    }
    report["quality_status"] = _quality_status(report, 0, None)
    return report


def _print_report(report: dict[str, Any]) -> None:
    print(f"{report['quality_status']}: {report['file_path']}")
    print(f"rows: {report['total_rows']} | duration_s: {report['duration_s']}")
    print(
        "timestamps: "
        f"monotonic={report['timestamp_monotonic']} "
        f"duplicates={report['duplicate_timestamps']} "
        f"dt<=0={report['dt_nonpositive_count']} "
        f"dt_s[min/median/mean/max]={report['dt_min']}/{report['dt_median']}/"
        f"{report['dt_mean']}/{report['dt_max']}"
    )
    print(f"dt_percentiles_s: {report['dt_percentiles']}")
    print(f"approx_sample_rate_hz: {report['approx_sample_rate_hz']}")
    print(f"missing_or_nonfinite_by_column: {report['missing_values']}")
    print(f"non_finite_by_column: {report['non_finite_values']}")
    print(f"invalid_numeric_by_column: {report['invalid_numeric_values']}")
    print(f"incomplete_current_rows: {report['incomplete_current_rows']}")
    print(f"current_columns_present: {report['current_columns_present']}")
    print(f"q_ranges: {report['q_ranges']}")
    print(f"qdot_ranges: {report['velocity_ranges']}")
    print(f"current_ranges_mA: {report['current_extrema']}")
    print(f"qddot_ranges: {report['acceleration_ranges']}")
    print(f"metadata_consistency: {report['metadata_consistency']}")
    print(f"metadata_values: {report['metadata_values']}")
    print(f"all_zero_current_rows: {report['suspicious_all_zero_current_rows']}")
    print(f"repeated_current_values: {report['repeated_current_values']}")
    print(f"large_gaps: {report['large_gap_values_s']} (heuristic threshold: {report['large_gap_threshold_s_heuristic']} s)")
    print(f"sample_index_gap_count: {report['sample_index_gap_count']}")
    print(f"current_clipping_limit: {report['current_clipping_limit']}")
    print(f"schema_summary: {report['schema_summary']}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python -m core.identification_logger <csv_path>")
        raise SystemExit(1)

    report = analyze_identification_csv(sys.argv[1])
    _print_report(report)
