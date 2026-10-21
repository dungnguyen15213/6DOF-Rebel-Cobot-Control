# Run audit: identification_ID_NORMAL_006_trajectory_export_RUN_06_20261022T002058

Readiness: **CONDITIONALLY READY**

## Decisions

- **structure_and_metadata**: PASS - columns, rows and metadata valid
- **raw_positions**: PASS - all q finite (a constant joint is not a failure)
- **raw_current**: WARNING - constant/zero channels present (stationary or unverified; genuine zeros are not a failure)
- **current_freshness**: PASS - every row has FRESH current and position from its own STATUS message
- **status_frame_accounting**: PASS - STATUS frames received during the run equal rows written; other frame types are not data loss
- **logger_integrity**: PASS - no writer drops or errors reported
- **receive_timing**: WARNING - host receive time only; STATUS carries no device timestamp; batched rows share a timestamp (labelled, not repaired)
- **derived_velocity**: WARNING - finite differences use host receive intervals >= 10 ms; velocity noise unquantified
- **derived_acceleration**: FAIL - receive-interval CV=0.29, fraction<min dt=0.03: acceleration from receive times is unreliable
- **model_a_exploratory**: WARNING - offline exploration only: position/velocity features on time-valid rows; not final training data
- **model_b**: FAIL - acceleration regressor unreliable with this timing
- **excitation**: FAIL - excited joints []; not fully excited [1, 2, 3, 4, 5, 6]

## Timing

```json
{
  "rows": 899,
  "unique_timestamps": 594,
  "rows_sharing_timestamp": 305,
  "unique_chunks": 594,
  "timing_class_counts": {
    "UNIQUE": 289,
    "BATCH_EARLIER": 305,
    "BATCH_LAST": 305
  },
  "chunk_dt_s": {
    "min": 0.0004794999986188486,
    "median": 0.04587380000157282,
    "max": 0.07110419999662554,
    "mean": 0.041252861045526036,
    "cv": 0.2901920380088886,
    "fraction_below_min_dt": 0.025295109612141653
  },
  "frame_counter": {
    "first": 3275,
    "last": 4451,
    "span": 1177,
    "rows": 899,
    "max_gap": 6,
    "gaps_gt1": 182,
    "non_increasing": 0
  },
  "status_frame_counter": {
    "first": 2469,
    "last": 3367,
    "span": 899,
    "rows": 899,
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
    "finite": 899,
    "exact_zero": 0,
    "value_changes": 0,
    "unique_values": 1
  },
  "J2": {
    "finite": 899,
    "exact_zero": 0,
    "value_changes": 337,
    "unique_values": 21
  },
  "J3": {
    "finite": 899,
    "exact_zero": 0,
    "value_changes": 0,
    "unique_values": 1
  },
  "J4": {
    "finite": 899,
    "exact_zero": 899,
    "value_changes": 0,
    "unique_values": 1
  },
  "J5": {
    "finite": 899,
    "exact_zero": 0,
    "value_changes": 0,
    "unique_values": 1
  },
  "J6": {
    "finite": 899,
    "exact_zero": 0,
    "value_changes": 228,
    "unique_values": 4
  }
}
```

## Detailed audit

```
IDENTIFICATION RUN AUDIT: WARNING
source: identification_data\identification_ID_NORMAL_006_trajectory_export_RUN_06_20261022T002058.csv
sha256: 715e0499da4c07fb0d1946380719f33da3d2204c16b757ce6f5ef5a36d3f1cc1

Conclusions
  basic csv readability        PASS
  logger signal integrity      WARNING
  initial 0g dry test          WARNING
  full six joint campaign      WARNING
  data use: logger dry test only

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "899 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["ID_NORMAL_006"], "trajectory_id": ["trajectory_export"], "run_id": ["RUN_06"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [PASS        ] I8 Explicit per-row freshness of position and current telemetry
      {"counts": {"position": {"FRESH": 899, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}, "current": {"FRESH": 899, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}}, "rule": "FRESH = field present in the same STATUS and finite; a measured 0 is FRESH"}
  [PASS        ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 0, "many_zero_channel_threshold": 3, "first_last_such_row": null, "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [WARNING     ] I6 No flat, frozen, or all-zero current channel
      {"current1": ["constant", "identical run of 899/899 samples"], "current3": ["constant", "identical run of 899/899 samples"], "current4": ["constant", "identical run of 899/899 samples", "all zero"], "current5": ["constant", "identical run of 899/899 samples"]}
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current1": {"samples_at_max": 899, "samples_at_min": 899, "min_mA": 19.0, "max_mA": 19.0}, "current3": {"samples_at_max": 899, "samples_at_min": 899, "min_mA": 361.0, "max_mA": 361.0}, "current4": {"samples_at_max": 899, "samples_at_min": 899, "min_mA": 0.0, "max_mA": 0.0}, "current5": {"samples_at_max": 899, "samples_at_min": 899, "min_mA": 9.0, "max_mA": 9.0}, "current6": {"samples_at_max": 764, "samples_at_min": 2, "min_mA": 36.0, "max_mA": 39.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [WARNING     ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 305, "BATCH_LAST": 305, "UNIQUE": 289}, "labels_inconsistent_with_timestamps": 0, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 305, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.22941949999221833, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 899, "duration_s": 24.46297819999745, "message_rate_hz": 36.70853126133651, "unique_timestamp_rate_hz": 24.24071162357745, "unique_timestamp_count": 594}
  [PASS        ] L2 All received STATUS frames are accounted for
      {"status_counter_contiguous": true, "rows": 899, "status_frames_received": 899, "discarded_frames_delta": 0, "parse_errors_delta": 0, "callback_errors_delta": 0, "other_frame_types_ignored_here": true}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=899 duration_s=24.463 message_rate_hz=36.70853126133651 unique_timestamp_rate_hz=24.24071162357745

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[19,19] std=0.0 moving_fraction=0.00
  J2: NOT EXERCISED q[0.01,0.01] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[38,188] std=27.3 moving_fraction=0.00
  J3: NOT EXERCISED q[90.00,90.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[361,361] std=0.0 moving_fraction=0.00
  J4: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00
  J5: NOT EXERCISED q[90.00,90.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[9,9] std=0.0 moving_fraction=0.00
  J6: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[36,39] std=0.4 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
