"""Offline audit, processing, and merge-guard utilities for identification runs.

Raw identification CSVs are never modified. Units of the raw file are degrees,
deg/s, mA and host-monotonic seconds. Every threshold in AuditConfig is an
engineering heuristic, not a manufacturer or scientific specification, and can
be overridden by the caller.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from core.identification_logger import (
    CURRENT_VECTOR_COLUMNS,
    METADATA_COLUMNS,
    Q_VECTOR_COLUMNS,
    QDDOT_VECTOR_COLUMNS,
    QDOT_VECTOR_COLUMNS,
)

PASS = "PASS"
WARNING = "WARNING"
FAIL = "FAIL"
NOT_VERIFIED = "NOT VERIFIED"

REQUIRED_COLUMNS = [
    "timestamp",
    *Q_VECTOR_COLUMNS,
    *CURRENT_VECTOR_COLUMNS,
    *QDOT_VECTOR_COLUMNS,
    *QDDOT_VECTOR_COLUMNS,
    *METADATA_COLUMNS,
]
_NONFINITE_TEXT = {"nan", "inf", "+inf", "-inf", "infinity", "+infinity", "-infinity"}


@dataclass(frozen=True)
class AuditConfig:
    """Configurable heuristics. None of these values come from a robot specification."""

    expected_payload_g: float = 0.0
    expected_contact_label: str = "NORMAL"
    expected_grasp_context: str = "NONE"
    # A gap is a unique-timestamp interval longer than this multiple of the median interval.
    gap_factor: float = 5.0
    # Duplicate host timestamps (batched socket reads) are warned up to this row
    # fraction and fail above it; their true sample times cannot be recovered.
    duplicate_timestamp_fail_fraction: float = 0.05
    # Speed above which a joint is considered moving. Must exceed q quantization noise.
    motion_threshold_deg_s: float = 1.0
    # Smallest travel treated as an exercised joint, and as a covered direction.
    min_motion_range_deg: float = 1.0
    min_excitation_range_deg: float = 5.0
    min_direction_travel_deg: float = 2.0
    # A channel with one identical value over this share of rows is flagged frozen.
    frozen_run_fraction: float = 0.5
    # Repeated extremum samples at least this many times are reported as a possible plateau.
    plateau_min_samples: int = 5
    # Rows with at least this many channels exactly 0 mA are flagged for review.
    multi_zero_channels: int = 3
    min_dry_duration_s: float = 10.0
    min_full_duration_s: float = 30.0


class UnsupportedMergeError(ValueError):
    """Raised when two CSVs cannot be scientifically combined."""


def _status_rank(status: str) -> int:
    return {PASS: 0, NOT_VERIFIED: 1, WARNING: 2, FAIL: 3}[status]


def _worst(statuses: list[str]) -> str:
    """Aggregate PASS/WARNING/FAIL; NOT VERIFIED items are listed separately, never promoted."""
    judged = [status for status in statuses if status != NOT_VERIFIED]
    return max(judged, key=_status_rank) if judged else PASS


def _criterion(cid: str, name: str, status: str, evidence: Any) -> dict[str, Any]:
    return {"id": cid, "name": name, "status": status, "evidence": evidence}


def _to_float(text: Any) -> tuple[float, str]:
    """Return (value, kind) where kind is ok, missing, nonfinite, or invalid."""
    if text is None or str(text).strip() == "":
        return float("nan"), "missing"
    stripped = str(text).strip()
    try:
        value = float(stripped)
    except ValueError:
        return float("nan"), "invalid"
    if not math.isfinite(value):
        return float("nan"), "nonfinite"
    return value, "ok"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_run(path: str | Path) -> dict[str, Any]:
    """Read a CSV into raw text rows plus structural findings, without modifying it."""
    csv_path = Path(path)
    raw_bytes = csv_path.read_bytes()
    result: dict[str, Any] = {
        "path": csv_path,
        "size_bytes": len(raw_bytes),
        "fieldnames": [],
        "rows": [],
        "csv_error": None,
        "ends_with_newline": raw_bytes.endswith(b"\n") if raw_bytes else False,
    }
    try:
        text = raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        result["csv_error"] = f"not valid UTF-8: {exc}"
        return result
    try:
        reader = csv.DictReader(text.splitlines(keepends=True), strict=True)
        result["fieldnames"] = list(reader.fieldnames or [])
        result["rows"] = list(reader)
    except csv.Error as exc:
        result["csv_error"] = str(exc)
    return result


def _column(rows: list[dict[str, str]], name: str) -> tuple[np.ndarray, dict[str, int]]:
    values = np.full(len(rows), np.nan)
    counts = {"missing": 0, "nonfinite": 0, "invalid": 0}
    for index, row in enumerate(rows):
        value, kind = _to_float(row.get(name))
        values[index] = value
        if kind != "ok":
            counts[kind] += 1
    return values, counts


def _batch_info(timestamps: np.ndarray) -> dict[str, np.ndarray]:
    """Group rows that share an identical host timestamp (one socket read, several STATUS)."""
    size = np.ones(len(timestamps), dtype=int)
    position = np.zeros(len(timestamps), dtype=int)
    is_last = np.ones(len(timestamps), dtype=bool)
    start = 0
    for index in range(1, len(timestamps) + 1):
        if index == len(timestamps) or timestamps[index] != timestamps[start]:
            size[start:index] = index - start
            position[start:index] = np.arange(index - start)
            is_last[start : index - 1] = False
            start = index
    return {"size": size, "position": position, "is_last": is_last}


def _joint_summaries(
    q: np.ndarray, current: np.ndarray, t: np.ndarray, config: AuditConfig
) -> list[dict[str, Any]]:
    """Per-joint coverage, using only the last row of each timestamp batch."""
    batch = _batch_info(t)
    keep = batch["is_last"] & np.isfinite(t)
    tk = t[keep]
    summaries = []
    for joint in range(6):
        qj = q[keep, joint]
        cj = current[:, joint]
        finite_q = qj[np.isfinite(qj)]
        finite_c = cj[np.isfinite(cj)]
        entry: dict[str, Any] = {"joint": joint + 1}
        if finite_q.size == 0 or finite_c.size == 0:
            entry["status"] = FAIL
            entry["note"] = "no finite position/current samples"
            summaries.append(entry)
            continue
        dq = np.diff(qj)
        dt = np.diff(tk)
        valid = np.isfinite(dq) & (dt > 0)
        speed = np.where(valid, dq / np.where(dt > 0, dt, 1.0), np.nan)
        moving = valid & (np.abs(speed) >= config.motion_threshold_deg_s)
        travel_pos = float(dq[valid & (dq > 0)].sum())
        travel_neg = float(-dq[valid & (dq < 0)].sum())
        q_range = float(finite_q.max() - finite_q.min())
        # Aligned current rows for moving vs stationary comparison (interval end sample).
        cj_keep = cj[keep][1:]
        moving_current = cj_keep[moving & np.isfinite(cj_keep)]
        still_current = cj_keep[valid & ~moving & np.isfinite(cj_keep)]
        runs = _longest_identical_run(finite_c)
        max_count = int((finite_c == finite_c.max()).sum())
        min_count = int((finite_c == finite_c.min()).sum())
        bidirectional = (
            travel_pos >= config.min_direction_travel_deg
            and travel_neg >= config.min_direction_travel_deg
        )
        if q_range < config.min_motion_range_deg:
            excitation = "NOT EXERCISED"
        elif q_range >= config.min_excitation_range_deg and bidirectional:
            excitation = "EXCITED"
        else:
            excitation = "LIMITED"
        entry.update(
            {
                "q_min_deg": float(finite_q.min()),
                "q_max_deg": float(finite_q.max()),
                "q_range_deg": q_range,
                "travel_positive_deg": travel_pos,
                "travel_negative_deg": travel_neg,
                "bidirectional": bool(bidirectional),
                "speed_min_deg_s": float(np.nanmin(speed)) if np.isfinite(speed).any() else None,
                "speed_max_deg_s": float(np.nanmax(speed)) if np.isfinite(speed).any() else None,
                "moving_fraction": float(moving.sum() / valid.sum()) if valid.any() else None,
                "current_min_mA": float(finite_c.min()),
                "current_max_mA": float(finite_c.max()),
                "current_mean_mA": float(finite_c.mean()),
                "current_std_mA": float(finite_c.std()),
                "current_unique_values": int(np.unique(finite_c).size),
                "current_moving_mean_mA": float(moving_current.mean()) if moving_current.size else None,
                "current_moving_std_mA": float(moving_current.std()) if moving_current.size else None,
                "current_stationary_mean_mA": float(still_current.mean()) if still_current.size else None,
                "current_stationary_std_mA": float(still_current.std()) if still_current.size else None,
                "longest_identical_current_run": runs,
                "samples_at_current_max": max_count,
                "samples_at_current_min": min_count,
                "excitation": excitation,
            }
        )
        summaries.append(entry)
    return summaries


def _longest_identical_run(values: np.ndarray) -> int:
    if values.size == 0:
        return 0
    longest = run = 1
    for previous, current in zip(values[:-1], values[1:]):
        run = run + 1 if current == previous else 1
        longest = max(longest, run)
    return longest


def audit_run(
    path: str | Path,
    config: AuditConfig | None = None,
    sidecar_path: str | Path | None = None,
) -> dict[str, Any]:
    """Audit one identification CSV and return a JSON-serializable report."""
    cfg = config or AuditConfig()
    csv_path = Path(path)
    run = load_run(csv_path)
    rows: list[dict[str, str]] = run["rows"]
    criteria: list[dict[str, Any]] = []
    readability: list[dict[str, Any]] = []
    integrity: list[dict[str, Any]] = []

    # ---- Basic CSV readability -------------------------------------------------
    if run["csv_error"]:
        readability.append(_criterion("R1", "CSV parses", FAIL, run["csv_error"]))
    elif not rows:
        readability.append(_criterion("R1", "CSV parses and has data rows", FAIL, "header only or empty file"))
    else:
        readability.append(_criterion("R1", "CSV parses and has data rows", PASS, f"{len(rows)} data rows"))
    missing_columns = [name for name in REQUIRED_COLUMNS if name not in run["fieldnames"]]
    readability.append(
        _criterion(
            "R2",
            "Required columns present",
            FAIL if missing_columns else PASS,
            {"missing": missing_columns} if missing_columns else "all required columns present",
        )
    )
    malformed = sum(None in row or None in row.values() for row in rows)
    truncated = bool(rows) and not run["ends_with_newline"]
    readability.append(
        _criterion(
            "R3",
            "No malformed or truncated rows",
            FAIL if malformed or truncated else PASS,
            {"malformed_rows": malformed, "final_line_terminated": run["ends_with_newline"]},
        )
    )
    readability_status = _worst([c["status"] for c in readability])

    report: dict[str, Any] = {
        "schema_version": 1,
        "source_file": str(csv_path),
        "source_sha256": _sha256(csv_path),
        "source_size_bytes": run["size_bytes"],
        "config": asdict(cfg),
        "units": {
            "timestamp": "s, host monotonic clock at CRI socket receive (not robot sample time)",
            "q": "deg", "qdot": "deg/s", "qddot": "deg/s^2", "current": "mA",
        },
        "criteria": criteria,
    }
    critical_ready = (
        not run["csv_error"]
        and rows
        and all(name in run["fieldnames"] for name in ("timestamp", *Q_VECTOR_COLUMNS, *CURRENT_VECTOR_COLUMNS))
    )
    if not critical_ready:
        criteria.extend(readability)
        report.update(
            conclusions={
                "basic_csv_readability": readability_status,
                "logger_signal_integrity": NOT_VERIFIED,
                "initial_0g_dry_test": FAIL,
                "full_six_joint_campaign": FAIL,
            },
            overall_status=FAIL,
            joints=[],
            data_use="unusable",
        )
        return report

    t, t_counts = _column(rows, "timestamp")
    q = np.column_stack([_column(rows, c)[0] for c in Q_VECTOR_COLUMNS])
    current = np.column_stack([_column(rows, c)[0] for c in CURRENT_VECTOR_COLUMNS])
    raw_issue_counts = {
        name: _column(rows, name)[1]
        for name in (*Q_VECTOR_COLUMNS, *CURRENT_VECTOR_COLUMNS)
    }

    # ---- Metadata ---------------------------------------------------------------
    metadata_values = {
        key: sorted({str(row.get(key, "")).strip() for row in rows}) for key in METADATA_COLUMNS
    }
    inconsistent = {key: values for key, values in metadata_values.items() if len(values) > 1}
    empty_meta = [key for key, values in metadata_values.items() if "" in values]
    integrity.append(
        _criterion(
            "I1", "Metadata constant for the whole run",
            FAIL if inconsistent or empty_meta else PASS,
            {"inconsistent": inconsistent, "empty_fields": empty_meta, "values": metadata_values},
        )
    )
    expectation_problems = {}
    try:
        payload = float(metadata_values["payload_g"][0])
    except (ValueError, IndexError):
        payload = float("nan")
    if not math.isclose(payload, cfg.expected_payload_g, abs_tol=1e-9):
        expectation_problems["payload_g"] = metadata_values["payload_g"]
    if [v.upper() for v in metadata_values["contact_label"]] != [cfg.expected_contact_label]:
        expectation_problems["contact_label"] = metadata_values["contact_label"]
    if [v.upper() for v in metadata_values["grasp_context"]] != [cfg.expected_grasp_context]:
        expectation_problems["grasp_context"] = metadata_values["grasp_context"]
    integrity.append(
        _criterion(
            "I2", "Experiment conditions match configured expectation",
            FAIL if expectation_problems else PASS,
            {
                "expected": {
                    "payload_g": cfg.expected_payload_g,
                    "contact_label": cfg.expected_contact_label,
                    "grasp_context": cfg.expected_grasp_context,
                },
                "mismatches": expectation_problems,
            },
        )
    )
    thermal = metadata_values["thermal_condition"]
    integrity.append(
        _criterion(
            "I3", "Thermal condition declared",
            NOT_VERIFIED if thermal in ([""], ["UNKNOWN"]) else WARNING if len(thermal) > 1 else PASS,
            {"reported": thermal, "note": "operator-declared label; no temperature signal is verified by this audit"},
        )
    )

    # ---- Primary signal integrity ----------------------------------------------
    bad_primary = {
        name: counts for name, counts in raw_issue_counts.items() if any(counts.values())
    }
    integrity.append(
        _criterion(
            "I4", "q and current are finite for every row",
            FAIL if bad_primary or any(t_counts.values()) else PASS,
            {"bad_columns": bad_primary, "timestamp_issues": t_counts},
        )
    )
    all_zero = np.all(current == 0.0, axis=1)
    leading_zero = 0
    for flag in all_zero:
        if not flag:
            break
        leading_zero += 1
    multi_zero = np.flatnonzero((current == 0.0).sum(axis=1) >= cfg.multi_zero_channels)
    integrity.append(
        _criterion(
            "I5", "Current telemetry is fresh (no initialized or partially zeroed rows)",
            FAIL if leading_zero else WARNING if all_zero.any() or multi_zero.size else PASS,
            {
                "all_zero_rows": int(all_zero.sum()),
                "leading_all_zero_rows": leading_zero,
                "rows_with_many_zero_channels": int(multi_zero.size),
                "many_zero_channel_threshold": cfg.multi_zero_channels,
                "first_last_such_row": [int(multi_zero[0]), int(multi_zero[-1])] if multi_zero.size else None,
                "rule": "leading all-zero rows match RobotState zero initialization; simultaneous "
                "exact zeros on several channels cannot be separated from genuine idle current "
                "using the CSV alone",
            },
        )
    )
    channel_findings = {}
    plateau_findings = {}
    for joint in range(6):
        values = current[:, joint]
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            continue
        run_len = _longest_identical_run(finite)
        problems = []
        if np.unique(finite).size == 1:
            problems.append("constant")
        if run_len > cfg.frozen_run_fraction * finite.size:
            problems.append(f"identical run of {run_len}/{finite.size} samples")
        if (finite == 0.0).all():
            problems.append("all zero")
        if problems:
            channel_findings[f"current{joint + 1}"] = problems
        top = int((finite == finite.max()).sum())
        bottom = int((finite == finite.min()).sum())
        if max(top, bottom) >= cfg.plateau_min_samples:
            plateau_findings[f"current{joint + 1}"] = {
                "samples_at_max": top, "samples_at_min": bottom,
                "min_mA": float(finite.min()), "max_mA": float(finite.max()),
            }
    integrity.append(
        _criterion(
            "I6", "No flat, frozen, or all-zero current channel",
            WARNING if channel_findings else PASS,
            channel_findings or "all channels vary",
        )
    )
    integrity.append(
        _criterion(
            "I7", "Current clipping",
            NOT_VERIFIED,
            {
                "note": "no documented current limit; extremum repeats are reported, not judged",
                "repeated_extrema": plateau_findings,
            },
        )
    )

    # ---- Timing -----------------------------------------------------------------
    dt = np.diff(t)
    dt_valid = dt[np.isfinite(dt)]
    negative = int((dt_valid < 0).sum())
    zero = int((dt_valid == 0).sum())
    duplicate_fraction = zero / len(dt_valid) if len(dt_valid) else 0.0
    integrity.append(
        _criterion(
            "T1", "Timestamps never decrease",
            FAIL if negative else PASS,
            {"negative_dt": negative},
        )
    )
    integrity.append(
        _criterion(
            "T2", "Timestamps are unique (one STATUS per host timestamp)",
            FAIL if duplicate_fraction > cfg.duplicate_timestamp_fail_fraction
            else WARNING if zero else PASS,
            {
                "dt_zero_count": zero,
                "dt_zero_fraction": duplicate_fraction,
                "fail_above_fraction": cfg.duplicate_timestamp_fail_fraction,
                "cause": "several STATUS messages parsed from one socket read share its receive time; "
                "samples are distinct but their true spacing is unknown",
            },
        )
    )
    unique_t = np.unique(t[np.isfinite(t)])
    group_dt = np.diff(unique_t)
    median_group_dt = float(np.median(group_dt)) if group_dt.size else float("nan")
    gap_limit = cfg.gap_factor * median_group_dt if math.isfinite(median_group_dt) else float("nan")
    gaps = group_dt[group_dt > gap_limit] if math.isfinite(gap_limit) else group_dt[:0]
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    dt_stats = {
        "message_dt_s": _stats(dt_valid),
        "unique_timestamp_dt_s": _stats(group_dt),
        "gap_limit_s": gap_limit if math.isfinite(gap_limit) else None,
        "gap_count": int(gaps.size),
        "largest_gap_s": float(gaps.max()) if gaps.size else None,
        "dt_histogram_unique": _histogram(group_dt),
    }
    integrity.append(
        _criterion(
            "T3", "No unexpected recording gaps",
            WARNING if gaps.size else PASS,
            {"gap_count": int(gaps.size), "gap_limit_s": dt_stats["gap_limit_s"],
             "largest_gap_s": dt_stats["largest_gap_s"]},
        )
    )
    sample_index = np.array([_to_float(row.get("sample_index"))[0] for row in rows])
    has_index = "sample_index" in run["fieldnames"]
    index_gaps = int((np.diff(sample_index) != 1).sum()) if has_index and len(sample_index) > 1 else 0
    integrity.append(
        _criterion(
            "T4", "sample_index is contiguous",
            NOT_VERIFIED if not has_index else FAIL if index_gaps else PASS,
            {"discontinuities": index_gaps},
        )
    )
    message_rate = (len(t) - 1) / duration if duration > 0 else None
    unique_rate = (unique_t.size - 1) / duration if duration > 0 and unique_t.size > 1 else None
    sampling = {
        "rows": len(rows),
        "duration_s": duration,
        "message_rate_hz": message_rate,
        "unique_timestamp_rate_hz": unique_rate,
        "unique_timestamp_count": int(unique_t.size),
    }
    integrity.append(
        _criterion(
            "T5", "Effective sampling rate measured",
            PASS if message_rate else FAIL,
            sampling,
        )
    )

    # ---- Logger sidecar ---------------------------------------------------------
    sidecar = Path(sidecar_path) if sidecar_path else Path(f"{csv_path}.json")
    sidecar_data: dict[str, Any] | None = None
    if sidecar.exists():
        try:
            sidecar_data = json.loads(sidecar.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            sidecar_data = None
    if sidecar_data is None:
        integrity.append(_criterion("L1", "Logger writer reported no drops or errors", NOT_VERIFIED, "no readable JSON sidecar"))
    else:
        dropped = sidecar_data.get("writer_dropped_sample_count", 0)
        writer_error = sidecar_data.get("writer_error")
        integrity.append(
            _criterion(
                "L1", "Logger writer reported no drops or errors",
                FAIL if dropped or writer_error else PASS,
                {"dropped": dropped, "writer_error": writer_error,
                 "logger_quality_status": sidecar_data.get("quality_status")},
            )
        )

    # ---- Excitation -------------------------------------------------------------
    joints = _joint_summaries(q, current, t, cfg)
    excitation_labels = [j.get("excitation", "NOT EXERCISED") for j in joints]
    excitation_status = (
        PASS if all(label == "EXCITED" for label in excitation_labels)
        else WARNING if any(label != "NOT EXERCISED" for label in excitation_labels)
        else FAIL
    )
    integrity_status = _worst([c["status"] for c in integrity])
    structural_fail = integrity_status == FAIL or readability_status == FAIL

    excited = [j["joint"] for j in joints if j.get("excitation") == "EXCITED"]
    unexcited = [j["joint"] for j in joints if j.get("excitation") != "EXCITED"]
    if structural_fail:
        dry = FAIL
    elif excited and duration >= cfg.min_dry_duration_s:
        dry = PASS
    else:
        dry = WARNING
    if structural_fail:
        full = FAIL
    elif excitation_status == PASS and duration >= cfg.min_full_duration_s:
        full = PASS
    else:
        full = WARNING

    if structural_fail:
        data_use = "not usable for identification until the failed criteria are resolved"
    elif not excited:
        data_use = "logger dry test only"
    elif unexcited:
        data_use = f"preliminary Model A for joints {excited} only"
    else:
        data_use = "preliminary Model A for all joints; more repeats required for Model B and holdout"

    criteria.extend(readability)
    criteria.extend(integrity)
    report.update(
        timing=dt_stats,
        sampling=sampling,
        joints=joints,
        excitation_summary={"status": excitation_status, "excited_joints": excited, "not_fully_excited_joints": unexcited},
        batch_duplicate_rows=int((~_batch_info(t)["is_last"]).sum()),
        conclusions={
            "basic_csv_readability": readability_status,
            "logger_signal_integrity": integrity_status,
            "initial_0g_dry_test": dry,
            "full_six_joint_campaign": full,
        },
        overall_status=FAIL if structural_fail else _worst([dry, full, integrity_status]),
        data_use=data_use,
        not_verified=[c["id"] for c in criteria if c["status"] == NOT_VERIFIED],
        campaign_note="A single run never certifies the identification campaign; the plan requires "
        "10-15 trajectory recipes with independent repeats.",
    )
    return report


def _stats(values: np.ndarray) -> dict[str, float | None]:
    if values.size == 0:
        return {"min": None, "median": None, "mean": None, "p95": None, "p99": None, "max": None, "std": None}
    return {
        "min": float(values.min()),
        "median": float(np.median(values)),
        "mean": float(values.mean()),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(values.max()),
        "std": float(values.std()),
    }


def _histogram(values: np.ndarray, bins: int = 10) -> dict[str, list[float]] | None:
    if values.size == 0:
        return None
    counts, edges = np.histogram(values, bins=bins)
    return {"counts": counts.tolist(), "edges_s": edges.tolist()}


def format_report(report: dict[str, Any]) -> str:
    """Human-readable audit text."""
    lines = [
        f"IDENTIFICATION RUN AUDIT: {report['overall_status']}",
        f"source: {report['source_file']}",
        f"sha256: {report['source_sha256']}",
        "",
        "Conclusions",
    ]
    for key, value in report["conclusions"].items():
        lines.append(f"  {key.replace('_', ' '):<28} {value}")
    lines.append(f"  data use: {report.get('data_use')}")
    lines.append("")
    lines.append("Criteria")
    for item in report["criteria"]:
        lines.append(f"  [{item['status']:<12}] {item['id']} {item['name']}")
        lines.append(f"      {json.dumps(item['evidence'], default=str)}")
    if report.get("sampling"):
        s = report["sampling"]
        lines += ["", "Sampling",
                  f"  rows={s['rows']} duration_s={s['duration_s']:.3f} message_rate_hz={s['message_rate_hz']} "
                  f"unique_timestamp_rate_hz={s['unique_timestamp_rate_hz']}"]
    if report.get("joints"):
        lines += ["", "Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)"]
        for j in report["joints"]:
            if "q_range_deg" not in j:
                lines.append(f"  J{j['joint']}: {j.get('note')}")
                continue
            lines.append(
                f"  J{j['joint']}: {j['excitation']:<13} q[{j['q_min_deg']:.2f},{j['q_max_deg']:.2f}] "
                f"range={j['q_range_deg']:.2f} travel+/-={j['travel_positive_deg']:.2f}/{j['travel_negative_deg']:.2f} "
                f"speed[{_fmt(j['speed_min_deg_s'])},{_fmt(j['speed_max_deg_s'])}] "
                f"current[{j['current_min_mA']:.0f},{j['current_max_mA']:.0f}] std={j['current_std_mA']:.1f} "
                f"moving_fraction={_fmt(j['moving_fraction'])}"
            )
    if report.get("campaign_note"):
        lines += ["", report["campaign_note"]]
    return "\n".join(lines) + "\n"


def _fmt(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f}"


# ---- Processed dataset for offline modeling -------------------------------------

PROCESSED_UNITS = {
    "t_s": "s since first sample (host receive clock)",
    "q*_rad": "rad", "qdot*_rad_s": "rad/s", "qddot*_rad_s2": "rad/s^2", "current*_A": "A",
}


def _gradient(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    result = np.full(values.shape, np.nan)
    if values.size >= 3:
        result[1:-1] = (values[2:] - values[:-2]) / (times[2:] - times[:-2])
    return result


def build_processed_dataset(
    csv_path: str | Path,
    output_path: str | Path,
    smoothing_window: int = 1,
) -> dict[str, Any]:
    """Write a derived CSV (SI units) and provenance JSON; the raw CSV is only read.

    Velocity and acceleration are reconstructed from q using only the last row
    of each timestamp batch (the row closest to its true receive time). Boundary
    rows stay NaN: no extrapolation. smoothing_window>1 applies a centered
    moving average to velocity before the second derivative.
    """
    source = Path(csv_path)
    destination = Path(output_path)
    if destination.resolve() == source.resolve():
        raise ValueError("processed output must not overwrite the raw CSV")
    if smoothing_window < 1 or smoothing_window % 2 == 0:
        raise ValueError("smoothing_window must be an odd integer >= 1")
    run = load_run(source)
    rows = run["rows"]
    if run["csv_error"] or not rows:
        raise ValueError(f"cannot process {source}: {run['csv_error'] or 'no data rows'}")
    t, _ = _column(rows, "timestamp")
    q_deg = np.column_stack([_column(rows, c)[0] for c in Q_VECTOR_COLUMNS])
    current_mA = np.column_stack([_column(rows, c)[0] for c in CURRENT_VECTOR_COLUMNS])
    batch = _batch_info(t)
    usable = batch["is_last"] & np.isfinite(t)
    increasing = usable.copy()
    kept_indices = np.flatnonzero(usable)
    kept_t = t[kept_indices]
    if kept_t.size > 1 and np.any(np.diff(kept_t) <= 0):
        raise ValueError("timestamps are not strictly increasing between batches; run failed audit")

    qdot = np.full((len(rows), 6), np.nan)
    qddot = np.full((len(rows), 6), np.nan)
    q_rad = np.deg2rad(q_deg)
    for joint in range(6):
        qk = q_rad[kept_indices, joint]
        if not np.isfinite(qk).all():
            continue
        velocity = _gradient(qk, kept_t)
        if smoothing_window > 1:
            velocity = _moving_average(velocity, smoothing_window)
        acceleration = _gradient(np.where(np.isfinite(velocity), velocity, np.nan), kept_t)
        qdot[kept_indices, joint] = velocity
        qddot[kept_indices, joint] = acceleration

    fieldnames = [
        "source_file", "source_row", "sample_index", "t_host_s", "t_s",
        *METADATA_COLUMNS, "batch_size", "batch_position", "flag_duplicate_timestamp",
        "flag_current_invalid", "flag_use_for_derivatives",
        *(f"q{j}_rad" for j in range(1, 7)),
        *(f"qdot{j}_rad_s" for j in range(1, 7)),
        *(f"qddot{j}_rad_s2" for j in range(1, 7)),
        *(f"current{j}_A" for j in range(1, 7)),
    ]
    destination.parent.mkdir(parents=True, exist_ok=True)
    t0 = float(t[np.isfinite(t)][0])
    with open(destination, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, row in enumerate(rows):
            out: dict[str, Any] = {
                "source_file": source.name,
                "source_row": index,
                "sample_index": row.get("sample_index", ""),
                "t_host_s": repr(float(t[index])),
                "t_s": repr(float(t[index] - t0)),
                **{key: row.get(key, "") for key in METADATA_COLUMNS},
                "batch_size": int(batch["size"][index]),
                "batch_position": int(batch["position"][index]),
                "flag_duplicate_timestamp": int(batch["size"][index] > 1),
                "flag_current_invalid": int(not np.isfinite(current_mA[index]).all()),
                "flag_use_for_derivatives": int(increasing[index]),
            }
            for joint in range(6):
                out[f"q{joint + 1}_rad"] = repr(float(q_rad[index, joint]))
                out[f"qdot{joint + 1}_rad_s"] = repr(float(qdot[index, joint]))
                out[f"qddot{joint + 1}_rad_s2"] = repr(float(qddot[index, joint]))
                out[f"current{joint + 1}_A"] = repr(float(current_mA[index, joint] / 1000.0))
            writer.writerow(out)
    provenance = {
        "source_file": str(source),
        "source_sha256": _sha256(source),
        "processed_file": str(destination),
        "units": PROCESSED_UNITS,
        "raw_units": {"q": "deg", "current": "mA"},
        "method": {
            "velocity": "central difference of q (rad) on the last row of each identical-timestamp batch, "
            "using measured timestamps",
            "acceleration": "central difference of reconstructed velocity; boundary rows left NaN",
            "smoothing_window_samples": smoothing_window,
            "non_batch_last_rows": "derivatives NaN (true sample time unknown); raw values preserved",
            "live_qddot": "ignored; not trusted as acceleration ground truth",
        },
        "rows": len(rows),
        "derivative_rows": int(np.isfinite(qdot[:, 0]).sum()),
        "run_id": sorted({row.get("run_id", "") for row in rows}),
        "model_features": {
            "A": "[1, qdot, tanh(qdot/v_s), sin(q), cos(q)]",
            "B": "[1, qddot, qdot, tanh(qdot/v_s), sin(q), cos(q)]",
        },
    }
    with open(Path(f"{destination}.provenance.json"), "w", encoding="utf-8") as handle:
        json.dump(provenance, handle, indent=2)
    return provenance


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    result = np.full(values.shape, np.nan)
    half = window // 2
    for index in range(half, len(values) - half):
        segment = values[index - half : index + half + 1]
        if np.isfinite(segment).all():
            result[index] = segment.mean()
    return result


def model_features(
    processed_csv: str | Path, joint: int, model: str = "A", v_s: float = 0.1
) -> tuple[np.ndarray, np.ndarray]:
    """Design matrix and current target (A) for one joint, valid rows only. No fitting."""
    if model not in {"A", "B"}:
        raise ValueError("model must be 'A' or 'B'")
    if v_s <= 0:
        raise ValueError("v_s must be positive")
    with open(processed_csv, newline="", encoding="utf-8") as handle:
        data = list(csv.DictReader(handle))
    j = int(joint)
    q = np.array([float(r[f"q{j}_rad"]) for r in data])
    qd = np.array([float(r[f"qdot{j}_rad_s"]) for r in data])
    qdd = np.array([float(r[f"qddot{j}_rad_s2"]) for r in data])
    y = np.array([float(r[f"current{j}_A"]) for r in data])
    use = np.array([r["flag_use_for_derivatives"] == "1" and r["flag_current_invalid"] == "0" for r in data])
    valid = use & np.isfinite(q) & np.isfinite(qd) & np.isfinite(y)
    columns = [np.ones(valid.sum())]
    if model == "B":
        valid &= np.isfinite(qdd)
        columns = [np.ones(valid.sum()), qdd[valid]]
    columns += [qd[valid], np.tanh(qd[valid] / v_s), np.sin(q[valid]), np.cos(q[valid])]
    return np.column_stack(columns), y[valid]


def whole_run_holdout_splits(run_ids: list[str]) -> Iterator[tuple[list[int], list[int]]]:
    """Leave-one-run-out splits over row indices; adjacent samples are never separated."""
    unique = sorted(set(run_ids))
    if len(unique) < 2:
        raise ValueError("whole-run holdout needs at least two distinct run IDs")
    for held_out in unique:
        train = [i for i, rid in enumerate(run_ids) if rid != held_out]
        test = [i for i, rid in enumerate(run_ids) if rid == held_out]
        yield train, test


# ---- Diagnostics merge guard ---------------------------------------------------


def describe_csv_schema(path: str | Path) -> dict[str, Any]:
    run = load_run(path)
    first = run["rows"][0] if run["rows"] else {}
    return {
        "path": str(path),
        "columns": run["fieldnames"],
        "rows": len(run["rows"]),
        "first_row": first,
    }


def assess_merge_feasibility(
    identification_csv: str | Path, diagnostics_csv: str | Path
) -> dict[str, Any]:
    """Decide whether the diagnostics CSV can be combined with an identification run.

    Supported only when both files share a run_id and a common clock column.
    Row order, equal first-sample offsets, or similar-looking signals never count.
    """
    ident = describe_csv_schema(identification_csv)
    diag = describe_csv_schema(diagnostics_csv)
    ident_cols, diag_cols = set(ident["columns"]), set(diag["columns"])
    reasons = []
    if "run_id" not in diag_cols:
        reasons.append("diagnostics file has no run_id/experiment identity to prove the same physical execution")
    elif {r.get("run_id") for r in load_run(diagnostics_csv)["rows"]} != {r.get("run_id") for r in load_run(identification_csv)["rows"]}:
        reasons.append("run_id values differ between the files")
    shared_clock = {"timestamp"} & ident_cols & diag_cols
    if not shared_clock:
        reasons.append(
            "no shared host-clock column: identification 'timestamp' is a CRI receive time, "
            "diagnostics 'Time (s)' is execution-relative and GUI-sampled"
        )
    mismatched_units = [c for c in diag["columns"] if "(mA)" in c or "(deg)" in c]
    if mismatched_units:
        reasons.append("signals use different names/units (deg, mA) and are GUI-sampled, not raw CRI receive samples")
    return {
        "mergeable": not reasons,
        "reasons": reasons,
        "identification_schema": ident["columns"],
        "diagnostics_schema": diag["columns"],
        "identification_rows": ident["rows"],
        "diagnostics_rows": diag["rows"],
    }


def require_mergeable(identification_csv: str | Path, diagnostics_csv: str | Path) -> None:
    assessment = assess_merge_feasibility(identification_csv, diagnostics_csv)
    if not assessment["mergeable"]:
        raise UnsupportedMergeError("; ".join(assessment["reasons"]))
