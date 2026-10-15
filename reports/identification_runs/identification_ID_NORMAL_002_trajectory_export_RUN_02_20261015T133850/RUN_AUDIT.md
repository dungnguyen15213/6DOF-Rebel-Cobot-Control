# Run audit: identification_ID_NORMAL_002_trajectory_export_RUN_02_20261015T133850

Readiness: **NOT READY**

## Decisions

- **A_raw_telemetry_integrity**: WARNING - channels exactly zero on every row: ['J6']; may be an unmoved/unloaded joint or a missing field (hardware check needed)
- **B_timestamp_provenance**: FAIL - host receive time only; no device timestamp exists in CRI STATUS frames; batched rows share a timestamp
- **C_model_A_offline**: WARNING - Model A (position/velocity-light, one sample per unique timestamp) usable offline only if currents are valid; excitation not assessed here
- **C_model_B_live_derivatives**: FAIL - receive-interval CV=0.32, fraction<min dt=0.04: acceleration from receive times is unreliable (Model B)

## Timing

```json
{
  "rows": 632,
  "unique_timestamps": 421,
  "rows_sharing_timestamp": 211,
  "unique_chunks": 421,
  "timing_class_counts": {
    "BATCH_EARLIER": 292,
    "BATCH_LAST": 184,
    "UNIQUE": 156
  },
  "chunk_dt_s": {
    "min": 0.00019729999985429458,
    "median": 0.04591970000001311,
    "max": 0.06775969999944209,
    "mean": 0.04101413404761756,
    "cv": 0.316141216292272,
    "fraction_below_min_dt": 0.04285714285714286
  },
  "frame_counter": {
    "first": 1912,
    "last": 2819,
    "span": 908,
    "rows": 632,
    "max_gap": 7,
    "gaps_gt1": 157,
    "non_increasing": 0
  }
}
```

## Current freshness

```json
{
  "J1": {
    "finite": 632,
    "exact_zero": 20,
    "value_changes": 605,
    "unique_values": 363
  },
  "J2": {
    "finite": 632,
    "exact_zero": 48,
    "value_changes": 584,
    "unique_values": 357
  },
  "J3": {
    "finite": 632,
    "exact_zero": 18,
    "value_changes": 605,
    "unique_values": 181
  },
  "J4": {
    "finite": 632,
    "exact_zero": 0,
    "value_changes": 439,
    "unique_values": 7
  },
  "J5": {
    "finite": 632,
    "exact_zero": 34,
    "value_changes": 590,
    "unique_values": 317
  },
  "J6": {
    "finite": 632,
    "exact_zero": 632,
    "value_changes": 0,
    "unique_values": 1
  }
}
```

## Detailed audit

```
IDENTIFICATION RUN AUDIT: FAIL
source: D:\01_VGU\03_IHMS\03_Project\06_6DOFCobotController\identification_data\identification_ID_NORMAL_002_trajectory_export_RUN_02_20261015T133850.csv
sha256: 3fa35a0ab623dd8592594ac6a594dbe03bdd9706e04697769eead561d923973d

Conclusions
  basic csv readability        PASS
  logger signal integrity      FAIL
  initial 0g dry test          FAIL
  full six joint campaign      FAIL
  data use: not usable for identification until the failed criteria are resolved

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "632 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["ID_NORMAL_002"], "trajectory_id": ["trajectory_export"], "run_id": ["RUN_02"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [WARNING     ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 34, "many_zero_channel_threshold": 3, "first_last_such_row": [15, 631], "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [WARNING     ] I6 No flat, frozen, or all-zero current channel
      {"current6": ["constant", "identical run of 632/632 samples", "all zero"]}
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current1": {"samples_at_max": 1, "samples_at_min": 20, "min_mA": 0.0, "max_mA": 640.0}, "current2": {"samples_at_max": 1, "samples_at_min": 48, "min_mA": 0.0, "max_mA": 786.0}, "current3": {"samples_at_max": 1, "samples_at_min": 18, "min_mA": 0.0, "max_mA": 789.0}, "current5": {"samples_at_max": 1, "samples_at_min": 34, "min_mA": 0.0, "max_mA": 634.0}, "current6": {"samples_at_max": 632, "samples_at_min": 632, "min_mA": 0.0, "max_mA": 0.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [FAIL        ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 292, "BATCH_LAST": 184, "UNIQUE": 156}, "labels_inconsistent_with_timestamps": 87, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 211, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.2295589999994263, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 632, "duration_s": 17.22597560000031, "message_rate_hz": 36.63072644779484, "unique_timestamp_rate_hz": 24.38178305558452, "unique_timestamp_count": 421}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=632 duration_s=17.226 message_rate_hz=36.63072644779484 unique_timestamp_rate_hz=24.38178305558452

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: LIMITED       q[0.00,48.01] range=48.01 travel+/-=48.01/-0.00 speed[0.00,474.83] current[0,640] std=146.3 moving_fraction=0.85
  J2: LIMITED       q[-40.21,-11.45] range=28.76 travel+/-=28.76/-0.00 speed[0.00,284.90] current[0,786] std=161.5 moving_fraction=0.76
  J3: LIMITED       q[102.04,114.99] range=12.95 travel+/-=12.95/-0.00 speed[0.00,189.93] current[0,789] std=163.2 moving_fraction=0.37
  J4: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[10,16] std=1.0 moving_fraction=0.00
  J5: LIMITED       q[65.96,69.55] range=3.59 travel+/-=0.01/3.59 speed[-22.82,0.32] current[0,634] std=123.0 moving_fraction=0.01
  J6: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
