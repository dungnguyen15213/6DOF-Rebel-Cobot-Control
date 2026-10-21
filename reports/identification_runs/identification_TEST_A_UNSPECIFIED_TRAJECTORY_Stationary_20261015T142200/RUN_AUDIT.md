# Run audit: identification_TEST_A_UNSPECIFIED_TRAJECTORY_Stationary_20261015T142200

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
  "rows": 2124,
  "unique_timestamps": 1408,
  "rows_sharing_timestamp": 716,
  "unique_chunks": 1408,
  "timing_class_counts": {
    "BATCH_EARLIER": 716,
    "BATCH_LAST": 716,
    "UNIQUE": 692
  },
  "chunk_dt_s": {
    "min": 0.000273600000582519,
    "median": 0.046065300000918796,
    "max": 0.06402210000123887,
    "mean": 0.04121070739161323,
    "cv": 0.3069708965308508,
    "fraction_below_min_dt": 0.030561478322672354
  },
  "frame_counter": {
    "first": 1648,
    "last": 4429,
    "span": 2782,
    "rows": 2124,
    "max_gap": 6,
    "gaps_gt1": 423,
    "non_increasing": 0
  },
  "status_frame_counter": {
    "first": 1233,
    "last": 3356,
    "span": 2124,
    "rows": 2124,
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
    "finite": 2124,
    "exact_zero": 0,
    "value_changes": 0,
    "unique_values": 1
  },
  "J2": {
    "finite": 2124,
    "exact_zero": 0,
    "value_changes": 1887,
    "unique_values": 17
  },
  "J3": {
    "finite": 2124,
    "exact_zero": 0,
    "value_changes": 1684,
    "unique_values": 10
  },
  "J4": {
    "finite": 2124,
    "exact_zero": 0,
    "value_changes": 888,
    "unique_values": 3
  },
  "J5": {
    "finite": 2124,
    "exact_zero": 0,
    "value_changes": 1745,
    "unique_values": 10
  },
  "J6": {
    "finite": 2124,
    "exact_zero": 2124,
    "value_changes": 0,
    "unique_values": 1
  }
}
```

## Detailed audit

```
IDENTIFICATION RUN AUDIT: WARNING
source: identification_data\identification_TEST_A_UNSPECIFIED_TRAJECTORY_Stationary_20261015T142200.csv
sha256: 6b741677bb958666f154019e0ed9ad2e19670d91e0ab5854ff0b049ae24e56e7

Conclusions
  basic csv readability        PASS
  logger signal integrity      WARNING
  initial 0g dry test          WARNING
  full six joint campaign      WARNING
  data use: logger dry test only

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "2124 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["TEST A"], "trajectory_id": ["UNSPECIFIED_TRAJECTORY"], "run_id": ["Stationary"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [PASS        ] I8 Explicit per-row freshness of position and current telemetry
      {"counts": {"position": {"FRESH": 2124, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}, "current": {"FRESH": 2124, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}}, "rule": "FRESH = field present in the same STATUS and finite; a measured 0 is FRESH"}
  [PASS        ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 0, "many_zero_channel_threshold": 3, "first_last_such_row": null, "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [WARNING     ] I6 No flat, frozen, or all-zero current channel
      {"current1": ["constant", "identical run of 2124/2124 samples"], "current6": ["constant", "identical run of 2124/2124 samples", "all zero"]}
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current1": {"samples_at_max": 2124, "samples_at_min": 2124, "min_mA": 9.0, "max_mA": 9.0}, "current3": {"samples_at_max": 6, "samples_at_min": 1, "min_mA": 620.0, "max_mA": 629.0}, "current4": {"samples_at_max": 1469, "samples_at_min": 1, "min_mA": 17.0, "max_mA": 19.0}, "current5": {"samples_at_max": 2, "samples_at_min": 9, "min_mA": 122.0, "max_mA": 131.0}, "current6": {"samples_at_max": 2124, "samples_at_min": 2124, "min_mA": 0.0, "max_mA": 0.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [WARNING     ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 716, "BATCH_LAST": 716, "UNIQUE": 692}, "labels_inconsistent_with_timestamps": 0, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 716, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.23037500000100408, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 2124, "duration_s": 57.983472200001415, "message_rate_hz": 36.61388184338413, "unique_timestamp_rate_hz": 24.26553544684007, "unique_timestamp_count": 1408}
  [PASS        ] L2 All received STATUS frames are accounted for
      {"status_counter_contiguous": true, "rows": 2124, "status_frames_received": 2124, "discarded_frames_delta": 0, "parse_errors_delta": 0, "callback_errors_delta": 0, "other_frame_types_ignored_here": true}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=2124 duration_s=57.983 message_rate_hz=36.61388184338413 unique_timestamp_rate_hz=24.26553544684007

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: NOT EXERCISED q[48.01,48.01] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[9,9] std=0.0 moving_fraction=0.00
  J2: NOT EXERCISED q[-11.44,-11.44] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[296,313] std=2.3 moving_fraction=0.00
  J3: NOT EXERCISED q[114.99,114.99] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[620,629] std=1.5 moving_fraction=0.00
  J4: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[17,19] std=0.5 moving_fraction=0.00
  J5: NOT EXERCISED q[65.96,65.96] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[122,131] std=1.5 moving_fraction=0.00
  J6: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[0,0] std=0.0 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
