"""Post-run report for one identification CSV.

Writes reports/identification_runs/<run_id>/ without touching the raw CSV.
Decisions A/B/C are assessed independently and use only evidence present in
the file; anything that needs hardware or device timestamps is NOT VERIFIED.
"""
from __future__ import annotations

import csv
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from core.identification_audit import audit_run, build_processed_dataset, format_report

PASS, WARNING, FAIL, NV = "PASS", "WARNING", "FAIL", "NOT VERIFIED"
RECEIVE_DT_CV_LIMIT = 0.25  # heuristic: receive-interval jitter acceptable for derivatives
MIN_RECEIVE_DT_S = 0.01
REPORT_ROOT = Path("reports") / "identification_runs"


def _f(rows: list[dict[str, str]], name: str) -> np.ndarray:
    out = []
    for row in rows:
        try:
            out.append(float(row.get(name, "")))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return np.asarray(out, dtype=float)


def _load(csv_path: Path) -> list[dict[str, str]]:
    with open(csv_path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def timing_diagnostics(rows: list[dict[str, str]]) -> dict[str, Any]:
    ts = _f(rows, "timestamp")
    t_rx = _f(rows, "t_frame_s")
    chunk = _f(rows, "rx_chunk_id")
    frame_counter = _f(rows, "frame_counter")
    status_counter = _f(rows, "status_frame_counter")
    out: dict[str, Any] = {"rows": len(rows)}
    out["unique_timestamps"] = int(len(np.unique(ts[np.isfinite(ts)])))
    out["rows_sharing_timestamp"] = int(len(ts) - len(np.unique(ts[np.isfinite(ts)]))) if len(ts) else 0
    out["unique_chunks"] = int(len(np.unique(chunk[np.isfinite(chunk)])))
    labels: dict[str, int] = {}
    for row in rows:
        label = row.get("timing_class", "")
        labels[label] = labels.get(label, 0) + 1
    out["timing_class_counts"] = labels
    if len(np.unique(chunk[np.isfinite(chunk)])) > 2:
        first = {}
        for c, t in zip(chunk, t_rx):
            first.setdefault(c, t)
        spacing = np.diff(np.asarray(list(first.values())))
        spacing = spacing[np.isfinite(spacing)]
        if len(spacing):
            out["chunk_dt_s"] = {
                "min": float(spacing.min()), "median": float(np.median(spacing)),
                "max": float(spacing.max()), "mean": float(spacing.mean()),
                "cv": float(spacing.std() / spacing.mean()) if spacing.mean() > 0 else None,
                "fraction_below_min_dt": float(np.mean(spacing < MIN_RECEIVE_DT_S)),
            }
    for name, arr in (("frame_counter", frame_counter), ("status_frame_counter", status_counter)):
        valid = arr[np.isfinite(arr) & (arr >= 0)]
        if len(valid) > 1:
            gaps = np.diff(valid)
            out[name] = {
                "first": int(valid[0]), "last": int(valid[-1]), "span": int(valid[-1] - valid[0] + 1),
                "rows": int(len(valid)), "max_gap": int(gaps.max()),
                "gaps_gt1": int(np.sum(gaps > 1)),
                "non_increasing": int(np.sum(gaps <= 0)),
            }
    return out


def current_freshness(rows: list[dict[str, str]]) -> dict[str, Any]:
    result = {}
    for j in range(1, 7):
        v = _f(rows, f"current{j}")
        finite = v[np.isfinite(v)]
        changes = int(np.sum(np.diff(finite) != 0)) if len(finite) > 1 else 0
        result[f"J{j}"] = {
            "finite": int(len(finite)), "exact_zero": int(np.sum(finite == 0)),
            "value_changes": changes,
            "unique_values": int(len(np.unique(finite))),
        }
    return result


def _crit(audit: dict[str, Any], cid: str) -> tuple[str, Any]:
    for item in audit["criteria"]:
        if item["id"] == cid:
            return item["status"], item["evidence"]
    return NV, "criterion not evaluated"


def _worst(statuses: list[str]) -> str:
    for level in (FAIL, WARNING, NV):
        if level in statuses:
            return level
    return PASS


def decisions(audit: dict[str, Any], timing: dict[str, Any], currents: dict[str, Any]) -> dict[str, Any]:
    """Independent decisions; the readiness verdict is derived from them only.

    Genuine zero current and stationary joints are never failures by themselves.
    Any missing/non-finite raw value, missing column, malformed row, invalid
    metadata, lost data or writer error is a FAIL.
    """
    st = {c["id"]: c["status"] for c in audit["criteria"]}
    i4_status, i4 = _crit(audit, "I4")
    bad_cols = i4.get("bad_columns", {}) if isinstance(i4, dict) else {}
    bad_q = [c for c in bad_cols if c.startswith("q")]
    bad_i = [c for c in bad_cols if c.startswith("current")]
    structural = [k for k in ("R1", "R2", "R3") if st.get(k) == FAIL]
    meta = [k for k in ("I1", "I2", "I3") if st.get(k) == FAIL]
    ts_issue = isinstance(i4, dict) and any(i4.get("timestamp_issues", {}).values())

    d: dict[str, dict[str, Any]] = {}
    d["structure_and_metadata"] = {
        "status": FAIL if structural or meta else PASS,
        "reason": f"failed: {structural + meta}" if structural or meta else "columns, rows and metadata valid",
    }
    d["raw_positions"] = {
        "status": FAIL if bad_q or structural or ts_issue else PASS,
        "reason": f"missing/non-finite: {bad_q}" if bad_q else "all q finite (a constant joint is not a failure)",
    }
    constant = st.get("I6") == WARNING
    d["raw_current"] = {
        "status": FAIL if bad_i or structural else WARNING if constant else PASS,
        "reason": f"missing/non-finite: {bad_i}" if bad_i
        else "constant/zero channels present (stationary or unverified; genuine zeros are not a failure)" if constant
        else "all current finite and varying",
    }
    i8, i8e = _crit(audit, "I8")
    d["current_freshness"] = {
        "status": i8,
        "reason": {PASS: "every row has FRESH current and position from its own STATUS message",
                   FAIL: "rows with MISSING/NON_FINITE telemetry", NV: "no explicit freshness flags in this recording"}[i8],
        "evidence": i8e,
    }
    l2, l2e = _crit(audit, "L2")
    d["status_frame_accounting"] = {
        "status": FAIL if st.get("T4") == FAIL or l2 == FAIL else l2 if l2 != PASS else PASS,
        "reason": "STATUS frames received during the run equal rows written; other frame types are not data loss"
        if l2 == PASS else "see L2 evidence",
        "evidence": l2e,
    }
    d["logger_integrity"] = {
        "status": FAIL if st.get("L1") == FAIL else st.get("L1", NV),
        "reason": "writer drops or errors" if st.get("L1") == FAIL else "no writer drops or errors reported",
    }
    t2 = st.get("T2", NV)
    d["receive_timing"] = {
        "status": FAIL if st.get("T1") == FAIL or t2 == FAIL else WARNING,
        "reason": "host receive time only; STATUS carries no device timestamp; batched rows share a timestamp (labelled, not repaired)",
    }
    chunk = timing.get("chunk_dt_s")
    if chunk is None:
        vel = acc = (NV, "no chunk timing")
    else:
        frac, cv = chunk["fraction_below_min_dt"], chunk["cv"]
        jitter = (cv is not None and cv > RECEIVE_DT_CV_LIMIT) or frac > 0.05
        vel = (WARNING, "finite differences use host receive intervals >= 10 ms; velocity noise unquantified")
        acc = (FAIL if jitter else WARNING,
               f"receive-interval CV={cv:.2f}, fraction<min dt={frac:.2f}: " + (
                   "acceleration from receive times is unreliable" if jitter else "needs hardware timing validation"))
    d["derived_velocity"] = {"status": vel[0], "reason": vel[1]}
    d["derived_acceleration"] = {"status": acc[0], "reason": acc[1]}
    raw_ok = all(d[k]["status"] != FAIL for k in
                 ("structure_and_metadata", "raw_positions", "raw_current", "status_frame_accounting", "logger_integrity"))
    d["model_a_exploratory"] = {
        "status": WARNING if raw_ok else FAIL,
        "reason": "offline exploration only: position/velocity features on time-valid rows; not final training data"
        if raw_ok else "raw telemetry failed",
    }
    d["model_b"] = {
        "status": FAIL if not raw_ok or acc[0] == FAIL else WARNING,
        "reason": "acceleration regressor unreliable with this timing" if acc[0] == FAIL else "unverified",
    }
    exc = audit.get("excitation_summary", {})
    d["excitation"] = {
        "status": exc.get("status", NV),
        "reason": f"excited joints {exc.get('excited_joints')}; not fully excited {exc.get('not_fully_excited_joints')}",
    }
    gate = [d[k]["status"] for k in ("structure_and_metadata", "raw_positions", "raw_current",
                                      "current_freshness", "status_frame_accounting", "logger_integrity", "receive_timing")]
    if FAIL in gate or audit.get("overall_status") == FAIL:
        verdict = "NOT READY"
    elif all(s == PASS for s in gate[:-1]) and gate[-1] == WARNING and d["current_freshness"]["status"] == PASS \
            and d["raw_current"]["status"] == PASS:
        verdict = "READY FOR CONTROLLED 0 G DRY TEST"
    else:
        verdict = "CONDITIONALLY READY"
    d["overall_readiness"] = verdict
    return d

def _plots(rows: list[dict[str, str]], out_dir: Path) -> list[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # optional dependency
        return [f"plots skipped: matplotlib unavailable ({exc})"]
    t = _f(rows, "t_frame_s")
    t = t - np.nanmin(t) if len(t) else t
    made = []
    for fname, prefix, ylabel in (
        ("joint_position_plot.png", "q", "joint position [deg]"),
        ("joint_current_plot.png", "current", "joint current [mA]"),
    ):
        fig, ax = plt.subplots(figsize=(9, 4))
        for j in range(1, 7):
            ax.plot(t, _f(rows, f"{prefix}{j}"), label=f"J{j}")
        ax.set_xlabel("receive time [s]"); ax.set_ylabel(ylabel); ax.legend(ncol=6, fontsize=7)
        fig.tight_layout(); fig.savefig(out_dir / fname, dpi=110); plt.close(fig)
        made.append(fname)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5))
    chunk = _f(rows, "rx_chunk_id")
    first: dict[float, float] = {}
    for c, tt in zip(chunk, _f(rows, "t_frame_s")):
        first.setdefault(c, tt)
    spacing = np.diff(np.asarray(list(first.values()))) * 1000.0
    finite_spacing = spacing[np.isfinite(spacing)]
    if len(finite_spacing):
        axes[0].hist(finite_spacing, bins=min(40, max(1, len(finite_spacing))))
    axes[0].set_xlabel("receive-chunk interval [ms]")
    for j in range(1, 7):
        qd = _f(rows, f"qddot{j}")
        axes[1].plot(t, np.where(np.isfinite(qd), qd, np.nan), lw=0.6)
    axes[1].set_xlabel("receive time [s]"); axes[1].set_ylabel("live qddot [deg/s^2]")
    fig.tight_layout(); fig.savefig(out_dir / "derivative_quality_plot.png", dpi=110); plt.close(fig)
    made.append("derivative_quality_plot.png")
    return made


def generate_run_report(csv_path: str | Path, out_root: str | Path = REPORT_ROOT) -> Path:
    csv_path = Path(csv_path)
    rows = _load(csv_path)
    run_id = csv_path.stem
    out_dir = Path(out_root) / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    audit = audit_run(csv_path)
    timing = timing_diagnostics(rows)
    currents = current_freshness(rows)
    decided = decisions(audit, timing, currents)

    sidecar_path = Path(str(csv_path) + ".json")
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8")) if sidecar_path.exists() else None

    report = {
        "run_id": run_id, "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_file": str(csv_path), "source_sha256": _sha(csv_path),
        "decisions": decided, "timing": timing, "current_freshness": currents,
        "audit_overall_status": audit["overall_status"], "audit": audit,
        "sidecar_frame_accounting": (sidecar or {}).get("frame_accounting"),
        "sidecar_logger_accounting": (sidecar or {}).get("logger_accounting"),
    }
    (out_dir / "RUN_AUDIT.json").write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")

    md = [f"# Run audit: {run_id}", "", f"Readiness: **{decided['overall_readiness']}**", "", "## Decisions", ""]
    for key, val in decided.items():
        if isinstance(val, dict):
            md.append(f"- **{key}**: {val['status']} - {val['reason']}")
    md += ["", "## Timing", "", "```json", json.dumps(timing, indent=2), "```", "",
           "## Current freshness", "", "```json", json.dumps(currents, indent=2), "```", "",
           "## Detailed audit", "", "```", format_report(audit), "```", ""]
    (out_dir / "RUN_AUDIT.md").write_text("\n".join(md), encoding="utf-8")

    cols = ["sample_index", "timestamp", "t_frame_s", "rx_chunk_id", "frame_counter",
            "status_frame_counter", "frame_in_chunk", "frames_in_chunk", "timing_class"]
    with open(out_dir / "timing_diagnostics.csv", "w", newline="", encoding="utf-8") as handle:
        w = csv.writer(handle)
        w.writerow(cols)
        for row in rows:
            w.writerow([row.get(c, "") for c in cols])

    plots = _plots(rows, out_dir)
    # Derived offline dataset, separate from the raw CSV; skipped when the audit failed.
    processed = out_dir / "processed_dataset.csv"
    if audit["overall_status"] != FAIL:
        try:
            build_processed_dataset(csv_path, processed)
        except ValueError as exc:
            plots.append(f"processed dataset skipped: {exc}")
    manifest = {
        "run_id": run_id, "source_file": str(csv_path), "source_sha256": report["source_sha256"],
        "files": sorted(p.name for p in out_dir.iterdir()), "plots": plots,
    }
    (out_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return out_dir


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Write the post-run report for one identification CSV")
    parser.add_argument("csv_path")
    parser.add_argument("--out-root", default=str(REPORT_ROOT))
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    out = generate_run_report(args.csv_path, args.out_root)
    verdict = json.loads((out / "RUN_AUDIT.json").read_text(encoding="utf-8"))["decisions"]["overall_readiness"]
    print(out)
    print(verdict)
    return 1 if verdict == "NOT READY" else 0


if __name__ == "__main__":
    raise SystemExit(main())