from __future__ import annotations

import csv
import math
import statistics
from datetime import datetime
from pathlib import Path
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

IDENTIFICATION_COLUMNS = [
    "timestamp",
    *Q_VECTOR_COLUMNS,
    *QDOT_VECTOR_COLUMNS,
    *CURRENT_VECTOR_COLUMNS,
    *QDDOT_VECTOR_COLUMNS,
    *METADATA_COLUMNS,
]


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


class IdentificationLogger:
    """Logs raw joint telemetry and metadata for offline system identification."""

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
    ) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.metadata = {
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
                f"identification_{_sanitize_filename_component(self.metadata['experiment_id'], 'ID')}"
                f"_{_sanitize_filename_component(self.metadata['trajectory_id'], 'TRAJ')}"
                f"_{_sanitize_filename_component(self.metadata['run_id'], 'RUN')}_{timestamp}.csv"
            )

        self.file_path = self.output_dir / filename
        if self.file_path.exists() and not overwrite:
            stem = self.file_path.stem
            suffix = self.file_path.suffix
            candidate = self.file_path
            index = 1
            while candidate.exists():
                candidate = self.output_dir / f"{stem}_{index}{suffix}"
                index += 1
            self.file_path = candidate

        self._file = open(self.file_path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=IDENTIFICATION_COLUMNS)
        self._writer.writeheader()
        self._file.flush()

    def log_sample(
        self,
        timestamp: float,
        q: Iterable[Any],
        qdot: Iterable[Any],
        current: Iterable[Any],
        qddot: Iterable[Any] | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        q_values = _as_float_list(q, 6)
        qdot_values = _as_float_list(qdot, 6)
        current_values = _as_float_list(current, 6)
        if qddot is None:
            qddot_values = [0.0] * 6
        else:
            qddot_values = _as_float_list(qddot, 6)

        row: dict[str, Any] = {
            "timestamp": float(timestamp),
            **{col: value for col, value in zip(Q_VECTOR_COLUMNS, q_values)},
            **{col: value for col, value in zip(QDOT_VECTOR_COLUMNS, qdot_values)},
            **{col: value for col, value in zip(CURRENT_VECTOR_COLUMNS, current_values)},
            **{col: value for col, value in zip(QDDOT_VECTOR_COLUMNS, qddot_values)},
        }

        effective_metadata = dict(self.metadata)
        if metadata is not None:
            effective_metadata.update(metadata)
        for key in METADATA_COLUMNS:
            row[key] = effective_metadata.get(key, self.metadata.get(key, ""))

        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> "IdentificationLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def _safe_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


def analyze_identification_csv(path: str | Path) -> dict[str, Any]:
    """Returns a compact quality summary for an identification CSV file."""
    csv_path = Path(path)
    with open(csv_path, "r", newline="", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile)
        fieldnames = reader.fieldnames or []
        rows = list(reader)

    total_rows = len(rows)
    required_columns = [
        "timestamp",
        *Q_VECTOR_COLUMNS,
        *QDOT_VECTOR_COLUMNS,
        *CURRENT_VECTOR_COLUMNS,
    ]
    missing_columns = [column for column in required_columns if column not in fieldnames]

    timestamps = []
    for row in rows:
        value = _safe_float(row.get("timestamp"))
        if value is not None:
            timestamps.append(value)

    duplicate_timestamps = 0
    seen_timestamps: set[float] = set()
    duplicate_values: set[float] = set()
    for row in rows:
        value = _safe_float(row.get("timestamp"))
        if value is None:
            continue
        if value in seen_timestamps:
            duplicate_values.add(value)
        else:
            seen_timestamps.add(value)
    duplicate_timestamps = sum(1 for row in rows if _safe_float(row.get("timestamp")) in duplicate_values)

    invalid_dt_count = 0
    dt_values: list[float] = []
    for idx in range(1, len(timestamps)):
        dt = timestamps[idx] - timestamps[idx - 1]
        if math.isnan(dt) or math.isinf(dt):
            invalid_dt_count += 1
            continue
        if dt < 0.0:
            invalid_dt_count += 1
        dt_values.append(dt)

    dt_min = min(dt_values) if dt_values else None
    dt_max = max(dt_values) if dt_values else None
    dt_mean = statistics.fmean(dt_values) if dt_values else None
    dt_median = statistics.median(dt_values) if dt_values else None
    dt_percentiles = {}
    if dt_values:
        for pct in (1, 5, 95, 99):
            dt_percentiles[str(pct)] = float(sorted(dt_values)[max(0, min(len(dt_values) - 1, math.ceil((pct / 100.0) * len(dt_values)) - 1))])

    missing_values: dict[str, int] = {}
    for column in fieldnames:
        count = 0
        for row in rows:
            value = row.get(column)
            if value is None or value == "" or str(value).lower() == "nan":
                count += 1
        if count:
            missing_values[column] = count

    q_ranges: dict[str, dict[str, float]] = {}
    velocity_ranges: dict[str, dict[str, float]] = {}
    current_extrema: dict[str, dict[str, float]] = {}
    acceleration_ranges: dict[str, dict[str, float]] = {}

    for i in range(1, 7):
        q_key = f"q{i}"
        qdot_key = f"qdot{i}"
        current_key = f"current{i}"
        qddot_key = f"qddot{i}"
        q_values = [_safe_float(row.get(q_key)) for row in rows]
        valid_q = [v for v in q_values if v is not None]
        q_ranges[q_key] = {"min": min(valid_q) if valid_q else float("nan"), "max": max(valid_q) if valid_q else float("nan")}

        qdot_values = [_safe_float(row.get(qdot_key)) for row in rows]
        valid_qdot = [v for v in qdot_values if v is not None]
        velocity_ranges[qdot_key] = {"min": min(valid_qdot) if valid_qdot else float("nan"), "max": max(valid_qdot) if valid_qdot else float("nan")}

        current_values = [_safe_float(row.get(current_key)) for row in rows]
        valid_currents = [v for v in current_values if v is not None]
        current_extrema[current_key] = {"min": min(valid_currents) if valid_currents else float("nan"), "max": max(valid_currents) if valid_currents else float("nan")}

        if qddot_key in fieldnames:
            qddot_values = [_safe_float(row.get(qddot_key)) for row in rows]
            valid_qddot = [v for v in qddot_values if v is not None]
            acceleration_ranges[qddot_key] = {"min": min(valid_qddot) if valid_qddot else float("nan"), "max": max(valid_qddot) if valid_qddot else float("nan")}

    experiment_ids = sorted({row.get("experiment_id", "") for row in rows if row.get("experiment_id")})
    trajectory_ids = sorted({row.get("trajectory_id", "") for row in rows if row.get("trajectory_id")})
    run_ids = sorted({row.get("run_id", "") for row in rows if row.get("run_id")})

    report = {
        "total_rows": total_rows,
        "first_timestamp": timestamps[0] if timestamps else None,
        "last_timestamp": timestamps[-1] if timestamps else None,
        "monotonic": all((timestamps[idx] >= timestamps[idx - 1]) for idx in range(1, len(timestamps))),
        "duplicate_timestamps": duplicate_timestamps,
        "invalid_dt_values": invalid_dt_count,
        "dt_min": dt_min,
        "dt_max": dt_max,
        "dt_mean": dt_mean,
        "dt_median": dt_median,
        "dt_percentiles": dt_percentiles,
        "missing_values": missing_values,
        "q_ranges": q_ranges,
        "velocity_ranges": velocity_ranges,
        "current_extrema": current_extrema,
        "acceleration_ranges": acceleration_ranges,
        "experiment_id_values": experiment_ids,
        "trajectory_id_values": trajectory_ids,
        "run_id_values": run_ids,
        "metadata_values": {
            key: sorted({row.get(key, "") for row in rows if row.get(key)})
            for key in [
                "commanded_speed",
                "payload_g",
                "contact_label",
                "grasp_context",
                "thermal_condition",
            ]
        },
        "schema_summary": {
            "required_columns": required_columns,
            "missing_columns": missing_columns,
            "units": {
                "timestamp": "seconds (monotonic perf_counter / CRI receive timestamp)",
                "q1..q6": "degrees",
                "qdot1..q6": "deg/s (finite-difference derivative of q)",
                "current1..current6": "mA",
                "qddot1..qddot6": "deg/s^2 (candidate derivative, if available)",
            },
            "notes": "No verified current clipping limit was found in the repository; observed extrema are reported without inventing a limit.",
        },
        "current_limit_verified": False,
        "current_extrema_summary": {
            key: {
                "min": values["min"],
                "max": values["max"],
            }
            for key, values in current_extrema.items()
        },
    }
    return report


def _print_report(report: dict[str, Any]) -> None:
    print("Identification CSV quality summary")
    print("=" * 36)
    print(f"total_rows: {report['total_rows']}")
    print(f"start_timestamp: {report['first_timestamp']}")
    print(f"end_timestamp: {report['last_timestamp']}")
    print(f"monotonic: {report['monotonic']}")
    print(f"duplicate_timestamps: {report['duplicate_timestamps']}")
    print(f"invalid_dt_values: {report['invalid_dt_values']}")
    print(f"dt_min: {report['dt_min']}")
    print(f"dt_max: {report['dt_max']}")
    print(f"dt_mean: {report['dt_mean']}")
    print(f"dt_median: {report['dt_median']}")
    print(f"dt_percentiles: {report['dt_percentiles']}")
    print(f"missing_values: {report['missing_values']}")
    print(f"experiment_id_values: {report['experiment_id_values']}")
    print(f"trajectory_id_values: {report['trajectory_id_values']}")
    print(f"run_id_values: {report['run_id_values']}")
    print(f"current_limit_verified: {report['current_limit_verified']}")
    print(f"schema_summary: {report['schema_summary']}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python -m core.identification_logger <csv_path>")
        raise SystemExit(1)

    report = analyze_identification_csv(sys.argv[1])
    _print_report(report)
