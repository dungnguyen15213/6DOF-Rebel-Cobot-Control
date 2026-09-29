#!/usr/bin/env python3

import sys
from pathlib import Path

from core.identification_logger import analyze_identification_csv


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python tools/check_identification_csv.py <identification_csv>")
        return 1

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f"CSV file not found: {csv_path}")
        return 1

    report = analyze_identification_csv(csv_path)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
