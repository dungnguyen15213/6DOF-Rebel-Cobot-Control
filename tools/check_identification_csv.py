#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.identification_logger import _print_report, analyze_identification_csv


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python tools/check_identification_csv.py <identification_csv>")
        return 1

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f"CSV file not found: {csv_path}")
        return 1

    try:
        report = analyze_identification_csv(csv_path)
    except (OSError, UnicodeError, csv.Error) as exc:
        print(f"FAIL: {csv_path} is unreadable or corrupt: {exc}")
        return 2
    _print_report(report)
    return 2 if report["quality_status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
