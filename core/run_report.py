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

from core.identification_audit import audit_run, format_report

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


def decisions(audit: dict[str, Any], timing: dict[str, Any], currents: dict[str, Any]) -> dict[str, Any]:
    crit = {c["id"]: c for c in audit["criteria"]}
    # A. raw telemetry integrity
    a_fail = [k for k in ("R1", "R2", "R3", "I1", "I2", "I3") if k in crit and crit[k]["status"] == FAIL]
    stuck = [j for j, s in currents.items() if s["finite"] and s["exact_zero"] == s["finite"]]
    if a_fail:
        a = (FAIL, f"criteria failed: {a_fail}")
    elif stuck:
        a = (WARNING, f"channels exactly zero on every row: {stuck}; may be an unmoved/unloaded joint or a missing field (hardware check needed)")
    else:
        a = (PASS, "structural checks pass")
    # B. timestamp provenance
    t2 = crit.get("T2", {}).get("status", NV)
    b = (t2, "host receive time only; no device timestamp exists in CRI STATUS frames; batched rows share a timestamp")
    if t2 == PASS:
        b = (WARNING, b[1])
    # C. Model A (offline, current vs. position-derived features)
    chunk = timing.get("chunk_dt_s")
    if chunk is None:
        c_model = (NV, "no chunk timing")
    else:
        frac = chunk["fraction_below_min_dt"]
        cv = chunk["cv"]
        if cv is not None and cv > RECEIVE_DT_CV_LIMIT or frac > 0.05:
            c_model = (FAIL, f"receive-interval CV={cv:.2f}, fraction<min dt={frac:.2f}: acceleration from receive times is unreliable (Model B)")
        else:
            c_model = (WARNING, "receive intervals acceptable but still not device-time; verify on hardware")
    a_model = (WARNING if a[0] != FAIL else FAIL,
               "Model A (position/velocity-light, one sample per unique timestamp) usable offline only if currents are valid; excitation not assessed here")
    ready = FAIL if FAIL in (a[0], b[0]) else (WARNING if WARNING in (a[0], b[0], c_model[0]) else PASS)
    verdict = {FAIL: "NOT READY", WARNING: "CONDITIONALLY READY", PASS: "READY FOR CONTROLLED 0 G DRY TEST"}[ready]
    return {
        "A_raw_telemetry_integrity": {"status": a[0], "reason": a[1]},
        "B_timestamp_provenance": {"status": b[0], "reason": b[1]},
        "C_model_A_offline": {"status": a_model[0], "reason": a_model[1]},
        "C_model_B_live_derivatives": {"status": c_model[0], "reason": c_model[1]},
        "overall_readiness": verdict,
    }


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
    manifest = {
        "run_id": run_id, "source_file": str(csv_path), "source_sha256": report["source_sha256"],
        "files": sorted(p.name for p in out_dir.iterdir()), "plots": plots,
    }
    (out_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return out_dir


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print("usage: python -m core.run_report <identification.csv>")
        return 2
    print(generate_run_report(args[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
