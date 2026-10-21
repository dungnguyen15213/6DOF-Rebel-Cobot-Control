# Run audit: identification_Stationary_UNSPECIFIED_TRAJECTORY_RUN_04_20261015T154629

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
- **derived_acceleration**: FAIL - receive-interval CV=0.31, fraction<min dt=0.03: acceleration from receive times is unreliable
- **model_a_exploratory**: WARNING - offline exploration only: position/velocity features on time-valid rows; not final training data
- **model_b**: FAIL - acceleration regressor unreliable with this timing
- **excitation**: FAIL - excited joints []; not fully excited [1, 2, 3, 4, 5, 6]

## Timing

```json
{
  "rows": 697,
  "unique_timestamps": 464,
  "rows_sharing_timestamp": 233,
  "unique_chunks": 464,
  "timing_class_counts": {
    "UNIQUE": 231,
    "BATCH_EARLIER": 233,
    "BATCH_LAST": 233
  },
  "chunk_dt_s": {
    "min": 0.00024420000045211054,
    "median": 0.046201299999665935,
    "max": 0.05971849999696133,
    "mean": 0.041086433045356446,
    "cv": 0.3130429201159179,
    "fraction_below_min_dt": 0.032397408207343416
  },
  "frame_counter": {
    "first": 4209,
    "last": 5120,
    "span": 912,
    "rows": 697,
    "max_gap": 6,
    "gaps_gt1": 139,
    "non_increasing": 0
  },
  "status_frame_counter": {
    "first": 3187,
    "last": 3883,
    "span": 697,
    "rows": 697,
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
    "finite": 697,
    "exact_zero": 0,
    "value_changes": 423,
    "unique_values": 5
  },
  "J2": {
    "finite": 697,
    "exact_zero": 0,
    "value_changes": 582,
    "unique_values": 12
  },
  "J3": {
    "finite": 697,
    "exact_zero": 0,
    "value_changes": 349,
    "unique_values": 4
  },
  "J4": {
    "finite": 697,
    "exact_zero": 0,
    "value_changes": 68,
    "unique_values": 2
  },
  "J5": {
    "finite": 697,
    "exact_zero": 697,
    "value_changes": 0,
    "unique_values": 1
  },
  "J6": {
    "finite": 697,
    "exact_zero": 697,
    "value_changes": 0,
    "unique_values": 1
  }
}
```

## Detailed audit

```
IDENTIFICATION RUN AUDIT: WARNING
source: identification_data\identification_Stationary_UNSPECIFIED_TRAJECTORY_RUN_04_20261015T154629.csv
sha256: 981c34b45cf896bc4dbc53f85147fc1f51d1501bbfff548137bc82aea6c32680

Conclusions
  basic csv readability        PASS
  logger signal integrity      WARNING
  initial 0g dry test          WARNING
  full six joint campaign      WARNING
  data use: logger dry test only

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "697 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["Stationary"], "trajectory_id": ["UNSPECIFIED_TRAJECTORY"], "run_id": ["RUN_04"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [PASS        ] I8 Explicit per-row freshness of position and current telemetry
      {"counts": {"position": {"FRESH": 697, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}, "current": {"FRESH": 697, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}}, "rule": "FRESH = field present in the same STATUS and finite; a measured 0 is FRESH"}
  [PASS        ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 0, "many_zero_channel_threshold": 3, "first_last_such_row": null, "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [WARNING     ] I6 No flat, frozen, or all-zero current channel
      {"current5": ["constant", "identical run of 697/697 samples", "all zero"], "current6": ["constant", "identical run of 697/697 samples", "all zero"]}
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current1": {"samples_at_max": 11, "samples_at_min": 12, "min_mA": 5.0, "max_mA": 9.0}, "current3": {"samples_at_max": 35, "samples_at_min": 49, "min_mA": 594.0, "max_mA": 597.0}, "current4": {"samples_at_max": 661, "samples_at_min": 36, "min_mA": 8.0, "max_mA": 9.0}, "current5": {"samples_at_max": 697, "samples_at_min": 697, "min_mA": 0.0, "max_mA": 0.0}, "current6": {"samples_at_max": 697, "samples_at_min": 697, "min_mA": 0.0, "max_mA": 0.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [WARNING     ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 233, "BATCH_LAST": 233, "UNIQUE": 231}, "labels_inconsistent_with_timestamps": 0, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 233, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.23111749998861342, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 697, "duration_s": 19.02302520000012, "message_rate_hz": 36.58724060356055, "unique_timestamp_rate_hz": 24.338925861276632, "unique_timestamp_count": 464}
  [PASS        ] L2 All received STATUS frames are accounted for
      {"status_counter_contiguous": true, "rows": 697, "status_frames_received": 697, "discarded_frames_delta": 0, "parse_errors_delta": 0, "callback_errors_delta": 0, "other_frame_types_ignored_here": true}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=697 duration_s=19.023 message_rate_hz=36.58724060356055 unique_timestamp_rate_hz=24.338925861276632

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: NOT EXERCISED q[47.91,47.91] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[5,9] std=0.8 moving_fraction=0.00
  J2: NOT EXERCISED q[-11.25,-11.25] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[168,179] std=1.8 moving_fraction=0.00
  J3: NOT EXERCISED q[115.06,115.06] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[594,597] std=0.7 moving_fraction=0.00
  J4: NOT EXERCISED q[0.33,0.33] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[8,9] std=0.2 moving_fraction=0.00
  J5: NOT EXERCISED q[64.91,64.91] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00
  J6: NOT EXERCISED q[-0.25,-0.25] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
