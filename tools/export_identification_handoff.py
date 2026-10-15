"""Package one identification run for external (e.g. ChatGPT) review.

usage: python tools/export_identification_handoff.py --run <identification.csv>
Raw files are copied unmodified; missing optional artifacts are listed in README.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.run_report import generate_run_report  # noqa: E402

OPTIONAL = [
    "Pathways/trajectory_export.csv",
    "IDENTIFICATION_DATA_COLLECTION.md",
    "reports/identification_fix/COPILOT_CHANGE_REPORT.md",
    "reports/identification_fix/root_cause_analysis.md",
    "reports/identification_fix/change_manifest.json",
    "reports/identification_fix/git_diff.patch",
    "reports/identification_fix/git_status.txt",
    "reports/identification_fix/test_results.txt",
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="raw identification CSV")
    ap.add_argument("--out-dir", default=str(ROOT / "reports" / "handoff"))
    args = ap.parse_args()
    csv_path = Path(args.run).resolve()
    sidecar = Path(str(csv_path) + ".json")
    report_dir = generate_run_report(csv_path, ROOT / "reports" / "identification_runs")

    entries: list[tuple[Path, str]] = [(csv_path, f"raw/{csv_path.name}")]
    missing: list[str] = []
    if sidecar.exists():
        entries.append((sidecar, f"raw/{sidecar.name}"))
    else:
        missing.append(f"sidecar {sidecar.name}")
    for p in sorted(report_dir.iterdir()):
        entries.append((p, f"run_report/{p.name}"))
    for rel in OPTIONAL:
        p = ROOT / rel
        if p.exists():
            entries.append((p, f"context/{rel}"))
        else:
            missing.append(rel)
    run_id = csv_path.stem
    manifest = {
        "run_id": run_id,
        "files": {arc: hashlib.sha256(p.read_bytes()).hexdigest() for p, arc in entries},
        "unavailable": missing,
    }
    readme = [
        f"# Identification handoff: {run_id}", "",
        "raw/ holds unmodified recorder output; run_report/ is derived (see RUN_AUDIT.md);",
        "context/ holds the trajectory, operator docs and change evidence.",
        "Timestamps are host receive times; STATUS frames carry no device timestamp.", "",
        "## Unavailable artifacts", *([f"- {m}" for m in missing] or ["- none"]),
    ]
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    zip_path = out_dir / f"ChatGPT_Identification_Handoff_{run_id}.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for p, arc in entries:
            z.write(p, arc)
        z.writestr("README.md", "\n".join(readme) + "\n")
        z.writestr("handoff_manifest.json", json.dumps(manifest, indent=2))
    print(zip_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
