# Run audit: identification_ID_NORMAL_003_trajectory_export_RUN_03_20261015T135600

Readiness: **CONDITIONALLY READY**

## Decisions

- **structure_and_metadata**: PASS - columns, rows and metadata valid
- **raw_positions**: PASS - all q finite (a constant joint is not a failure)
- **raw_current**: WARNING - constant/zero channels present (stationary or unverified; genuine zeros are not a failure)
- **current_freshness**: NOT VERIFIED - no explicit freshness flags in this recording
- **status_frame_accounting**: PASS - STATUS frames received during the run equal rows written; other frame types are not data loss
- **logger_integrity**: PASS - no writer drops or errors reported
- **receive_timing**: WARNING - host receive time only; STATUS carries no device timestamp; batched rows share a timestamp (labelled, not repaired)
- **derived_velocity**: WARNING - finite differences use host receive intervals >= 10 ms; velocity noise unquantified
- **derived_acceleration**: FAIL - receive-interval CV=0.34, fraction<min dt=0.05: acceleration from receive times is unreliable
- **model_a_exploratory**: WARNING - offline exploration only: position/velocity features on time-valid rows; not final training data
- **model_b**: FAIL - acceleration regressor unreliable with this timing
- **excitation**: WARNING - excited joints []; not fully excited [1, 2, 3, 4, 5, 6]

## Timing

```json
{
  "rows": 632,
  "unique_timestamps": 414,
  "rows_sharing_timestamp": 218,
  "unique_chunks": 414,
  "timing_class_counts": {
    "UNIQUE": 197,
    "BATCH_EARLIER": 218,
    "BATCH_LAST": 217
  },
  "chunk_dt_s": {
    "min": 0.00023220000002766028,
    "median": 0.045983500000147615,
    "max": 0.09458690000064962,
    "mean": 0.041771650121064534,
    "cv": 0.3390334827915779,
    "fraction_below_min_dt": 0.048426150121065374
  },
  "frame_counter": {
    "first": 1810,
    "last": 2718,
    "span": 909,
    "rows": 632,
    "max_gap": 9,
    "gaps_gt1": 158,
    "non_increasing": 0
  },
  "status_frame_counter": {
    "first": 1353,
    "last": 1984,
    "span": 632,
    "rows": 632,
    "max_gap": 1,
    "gaps_gt1": 0,
    "non_increasing": 0
  }
}
```

## Current freshness

```json
{
  "J1": {
    "finite": 632,
    "exact_zero": 31,
    "value_changes": 596,
    "unique_values": 359
  },
  "J2": {
    "finite": 632,
    "exact_zero": 53,
    "value_changes": 576,
    "unique_values": 357
  },
  "J3": {
    "finite": 632,
    "exact_zero": 22,
    "value_changes": 600,
    "unique_values": 175
  },
  "J4": {
    "finite": 632,
    "exact_zero": 0,
    "value_changes": 269,
    "unique_values": 2
  },
  "J5": {
    "finite": 632,
    "exact_zero": 37,
    "value_changes": 584,
    "unique_values": 295
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
IDENTIFICATION RUN AUDIT: WARNING
source: D:\01_VGU\03_IHMS\03_Project\06_6DOFCobotController\identification_data\identification_ID_NORMAL_003_trajectory_export_RUN_03_20261015T135600.csv
sha256: 4235e1c94a7b34c2642377d1a5c6a01b0829ef0dabd6f9e347c8320dc8329a12

Conclusions
  basic csv readability        PASS
  logger signal integrity      WARNING
  initial 0g dry test          WARNING
  full six joint campaign      WARNING
  data use: logger dry test only

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "632 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["ID_NORMAL_003"], "trajectory_id": ["trajectory_export"], "run_id": ["RUN_03"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [NOT VERIFIED] I8 Explicit per-row freshness of position and current telemetry
      "legacy recording without position_validity/current_validity columns"
  [WARNING     ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 40, "many_zero_channel_threshold": 3, "first_last_such_row": [6, 631], "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [WARNING     ] I6 No flat, frozen, or all-zero current channel
      {"current6": ["constant", "identical run of 632/632 samples", "all zero"]}
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current1": {"samples_at_max": 1, "samples_at_min": 31, "min_mA": 0.0, "max_mA": 805.0}, "current2": {"samples_at_max": 1, "samples_at_min": 53, "min_mA": 0.0, "max_mA": 813.0}, "current3": {"samples_at_max": 1, "samples_at_min": 22, "min_mA": 0.0, "max_mA": 841.0}, "current4": {"samples_at_max": 416, "samples_at_min": 216, "min_mA": 18.0, "max_mA": 19.0}, "current5": {"samples_at_max": 1, "samples_at_min": 37, "min_mA": 0.0, "max_mA": 605.0}, "current6": {"samples_at_max": 632, "samples_at_min": 632, "min_mA": 0.0, "max_mA": 0.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [WARNING     ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 218, "BATCH_LAST": 217, "UNIQUE": 197}, "labels_inconsistent_with_timestamps": 0, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 218, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.22997649999524583, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 632, "duration_s": 17.25165399999969, "message_rate_hz": 36.57620307015266, "unique_timestamp_rate_hz": 23.939733546708474, "unique_timestamp_count": 414}
  [PASS        ] L2 All received STATUS frames are accounted for
      {"status_counter_contiguous": true, "rows": 632, "status_frames_received": 632, "discarded_frames_delta": 0, "parse_errors_delta": 0, "callback_errors_delta": 0, "other_frame_types_ignored_here": true}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=632 duration_s=17.252 message_rate_hz=36.57620307015266 unique_timestamp_rate_hz=23.939733546708474

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: LIMITED       q[0.00,48.01] range=48.01 travel+/-=48.01/-0.00 speed[0.00,196.00] current[0,805] std=147.1 moving_fraction=0.86
  J2: LIMITED       q[-40.21,-11.45] range=28.76 travel+/-=28.76/-0.00 speed[0.00,117.60] current[0,813] std=169.8 moving_fraction=0.75
  J3: LIMITED       q[102.04,114.99] range=12.95 travel+/-=12.95/-0.00 speed[0.00,58.80] current[0,841] std=167.6 moving_fraction=0.34
  J4: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[18,19] std=0.5 moving_fraction=0.00
  J5: LIMITED       q[65.96,69.55] range=3.59 travel+/-=0.01/3.59 speed[-19.60,0.20] current[0,605] std=123.9 moving_fraction=0.02
  J6: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
