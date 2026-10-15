"""Package one identification run for external (e.g. ChatGPT) review.

usage:
    python tools/export_identification_handoff.py --run <identification.csv>
    python tools/export_identification_handoff.py --code-only      # no run data

The run is always the explicit --run argument; nothing is substituted from
another run. Every run-specific artifact is checked against the CSV SHA-256 and
run_id. Missing artifacts are listed in the README, never replaced.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.run_report import generate_run_report  # noqa: E402

FIX = "reports/identification_fix/"
CONTEXT = [
    "Pathways/trajectory_export.csv",
    "IDENTIFICATION_DATA_COLLECTION.md",
    FIX + "COPILOT_CHANGE_REPORT.md",
    FIX + "root_cause_analysis.md",
    FIX + "PREPROCESSING_POLICY.md",
    FIX + "change_manifest.json",
    FIX + "git_diff.patch",
    FIX + "git_status.txt",
    FIX + "test_results.txt",
]
REQUIRED_REPORT_FILES = [
    "RUN_AUDIT.md", "RUN_AUDIT.json", "timing_diagnostics.csv", "joint_position_plot.png",
    "joint_current_plot.png", "derivative_quality_plot.png", "run_manifest.json",
    "processed_dataset.csv", "processed_dataset.csv.provenance.json",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_run_ids(path: Path) -> set[str]:
    with open(path, newline="", encoding="utf-8") as handle:
        return {row.get("run_id", "") for row in csv.DictReader(handle)}


def build(run: Path | None, out_dir: Path) -> Path:
    entries: list[tuple[Path, str]] = []
    missing: list[str] = []
    problems: list[str] = []
    run_id = "code_only"
    run_hash = None
    if run is not None:
        run = run.resolve()
        if not run.is_file():
            raise SystemExit(f"run CSV not found: {run}")
        run_id = run.stem
        run_hash = sha256(run)
        sidecar = Path(str(run) + ".json")
        entries.append((run, f"raw/{run.name}"))
        if sidecar.is_file():
            entries.append((sidecar, f"raw/{sidecar.name}"))
            side = json.loads(sidecar.read_text(encoding="utf-8"))
            declared = Path(str(side.get("file_path", run))).name
            if declared != run.name:
                problems.append(f"sidecar file_path {declared} != {run.name}")
        else:
            missing.append(f"sidecar {sidecar.name}")
        report_dir = generate_run_report(run, ROOT / "reports" / "identification_runs")
        audit = json.loads((report_dir / "RUN_AUDIT.json").read_text(encoding="utf-8"))
        if audit["source_sha256"] != run_hash or Path(audit["source_file"]).name != run.name:
            problems.append("RUN_AUDIT.json does not match the selected run")
        manifest = json.loads((report_dir / "run_manifest.json").read_text(encoding="utf-8"))
        if manifest["source_sha256"] != run_hash:
            problems.append("run_manifest.json hash mismatch")
        prov = Path(str(report_dir / "processed_dataset.csv") + ".provenance.json")
        if prov.is_file() and json.loads(prov.read_text(encoding="utf-8"))["source_sha256"] != run_hash:
            problems.append("processed dataset provenance hash mismatch")
        for name in REQUIRED_REPORT_FILES:
            if (report_dir / name).is_file():
                entries.append((report_dir / name, f"run_report/{name}"))
            else:
                missing.append(f"run_report/{name}")
        if problems:
            raise SystemExit("identity validation failed, no ZIP written: " + "; ".join(problems))
    for rel in CONTEXT:
        path = ROOT / rel
        if path.is_file():
            entries.append((path, f"context/{rel}"))
        else:
            missing.append(rel)

    listing = {arc: sha256(path) for path, arc in entries}
    readme = [
        f"# Identification handoff: {run_id}", "",
        "Run-specific files are under `raw/` (unmodified recorder output) and `run_report/` (derived, hash-checked against the CSV).",
        "`context/` holds the trajectory, operator docs and code-change evidence; it is not run-specific.",
        f"Selected run SHA-256: {run_hash or 'n/a (code-only package; no hardware run is claimed)'}",
        "Timestamps are host receive times; STATUS frames carry no device timestamp.", "",
        "## Unavailable artifacts", *([f"- {m}" for m in missing] or ["- none"]), "",
        "## File hashes (SHA-256)", *[f"- {a}: {h}" for a, h in sorted(listing.items())],
    ]
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / f"ChatGPT_Identification_Handoff_{run_id}.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for path, arc in entries:
            archive.write(path, arc)
        archive.writestr("README.md", "\n".join(readme) + "\n")
        archive.writestr("handoff_manifest.json", json.dumps(
            {"run_id": run_id, "source_sha256": run_hash, "files": listing, "unavailable": missing}, indent=2))
    return zip_path


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run", help="raw identification CSV (explicit; never auto-selected)")
    group.add_argument("--code-only", action="store_true", help="package code-change evidence only")
    parser.add_argument("--out-dir", default=str(ROOT / "reports" / "handoff"))
    args = parser.parse_args()
    print(build(Path(args.run) if args.run else None, Path(args.out_dir)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
