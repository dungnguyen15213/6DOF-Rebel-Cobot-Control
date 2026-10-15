#!/usr/bin/env python3
"""Audit an identification CSV without modifying it.

Usage:
    python tools/audit_identification_run.py <identification_csv>
        [--out-dir DIR] [--diagnostics robot_diagnostics.csv]
        [--processed] [--smoothing-window N] [--no-plots]

Exit code: 0 for PASS/WARNING, 1 for FAIL, 2 for usage or file errors.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from core.identification_audit import (  # noqa: E402
    FAIL,
    AuditConfig,
    assess_merge_feasibility,
    audit_run,
    build_processed_dataset,
    format_report,
    load_run,
)
from core.identification_logger import CURRENT_VECTOR_COLUMNS, Q_VECTOR_COLUMNS  # noqa: E402


def write_plots(csv_path: Path, out_dir: Path) -> list[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        return []
    rows = load_run(csv_path)["rows"]
    if not rows:
        return []

    def col(name: str) -> "np.ndarray":
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except (KeyError, TypeError, ValueError):
                values.append(float("nan"))
        return np.array(values)

    t = col("timestamp")
    t = t - t[np.isfinite(t)][0]
    written = []
    for group, columns, label in (
        ("positions", Q_VECTOR_COLUMNS, "deg"),
        ("currents", CURRENT_VECTOR_COLUMNS, "mA"),
    ):
        figure, axes = plt.subplots(6, 1, sharex=True, figsize=(9, 11))
        for axis, column in zip(axes, columns):
            axis.plot(t, col(column), linewidth=0.8)
            axis.set_ylabel(f"{column} [{label}]")
            axis.grid(alpha=0.3)
        axes[-1].set_xlabel("host receive time since first sample [s]")
        figure.tight_layout()
        target = out_dir / f"{csv_path.stem}_{group}.png"
        figure.savefig(target, dpi=110)
        plt.close(figure)
        written.append(str(target))
    dt = np.diff(t)
    figure, axes = plt.subplots(2, 1, figsize=(9, 6))
    axes[0].plot(t[1:], dt * 1000.0, ".", markersize=3)
    axes[0].set_ylabel("dt [ms]")
    axes[0].set_xlabel("time [s]")
    axes[1].hist(dt[dt > 0] * 1000.0, bins=40)
    axes[1].set_xlabel("positive dt [ms]")
    axes[1].set_ylabel("count")
    figure.tight_layout()
    target = out_dir / f"{csv_path.stem}_dt.png"
    figure.savefig(target, dpi=110)
    plt.close(figure)
    written.append(str(target))
    return written


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="report directory (default: audit_reports/<csv stem>)")
    parser.add_argument("--diagnostics", type=Path, default=None,
                        help="robot_diagnostics.csv to assess for merge feasibility (never merged)")
    parser.add_argument("--processed", action="store_true",
                        help="also write the SI-unit processed dataset for offline modeling")
    parser.add_argument("--smoothing-window", type=int, default=1)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--payload-g", type=float, default=0.0)
    parser.add_argument("--contact-label", default="NORMAL")
    parser.add_argument("--grasp-context", default="NONE")
    args = parser.parse_args(argv)

    if not args.csv_path.is_file():
        print(f"CSV file not found: {args.csv_path}", file=sys.stderr)
        return 2
    out_dir = args.out_dir or REPO_ROOT / "audit_reports" / args.csv_path.stem
    if out_dir.resolve() == args.csv_path.resolve().parent:
        print("--out-dir must not be the raw data directory", file=sys.stderr)
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)

    config = AuditConfig(
        expected_payload_g=args.payload_g,
        expected_contact_label=args.contact_label.upper(),
        expected_grasp_context=args.grasp_context.upper(),
    )
    report = audit_run(args.csv_path, config)
    if args.diagnostics is not None:
        if not args.diagnostics.is_file():
            print(f"Diagnostics file not found: {args.diagnostics}", file=sys.stderr)
            return 2
        report["diagnostics_merge_assessment"] = assess_merge_feasibility(args.csv_path, args.diagnostics)
    if args.processed:
        if report["overall_status"] == FAIL:
            report["processed_dataset"] = "not written: audit FAIL"
        else:
            processed_path = out_dir / f"{args.csv_path.stem}_processed.csv"
            report["processed_dataset"] = build_processed_dataset(
                args.csv_path, processed_path, args.smoothing_window
            )
    if not args.no_plots:
        report["plots"] = write_plots(args.csv_path, out_dir)

    (out_dir / "audit_report.json").write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
    text = format_report(report)
    if "diagnostics_merge_assessment" in report:
        merge = report["diagnostics_merge_assessment"]
        text += "\nDiagnostics merge: " + ("SUPPORTED" if merge["mergeable"] else "UNSUPPORTED") + "\n"
        text += "".join(f"  - {reason}\n" for reason in merge["reasons"])
    (out_dir / "audit_report.txt").write_text(text, encoding="utf-8")
    print(text)
    print(f"Reports written to: {out_dir}")
    return 1 if report["overall_status"] == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
