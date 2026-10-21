# Run audit: identification_ID_NORMAL_005_trajectory_export_RUN_05_20261019T221637

Readiness: **READY FOR CONTROLLED 0 G DRY TEST**

## Decisions

- **structure_and_metadata**: PASS - columns, rows and metadata valid
- **raw_positions**: PASS - all q finite (a constant joint is not a failure)
- **raw_current**: PASS - all current finite and varying
- **current_freshness**: PASS - every row has FRESH current and position from its own STATUS message
- **status_frame_accounting**: PASS - STATUS frames received during the run equal rows written; other frame types are not data loss
- **logger_integrity**: PASS - no writer drops or errors reported
- **receive_timing**: WARNING - host receive time only; STATUS carries no device timestamp; batched rows share a timestamp (labelled, not repaired)
- **derived_velocity**: WARNING - finite differences use host receive intervals >= 10 ms; velocity noise unquantified
- **derived_acceleration**: FAIL - receive-interval CV=0.30, fraction<min dt=0.04: acceleration from receive times is unreliable
- **model_a_exploratory**: WARNING - offline exploration only: position/velocity features on time-valid rows; not final training data
- **model_b**: FAIL - acceleration regressor unreliable with this timing
- **excitation**: WARNING - excited joints []; not fully excited [1, 2, 3, 4, 5, 6]

## Timing

```json
{
  "rows": 635,
  "unique_timestamps": 416,
  "rows_sharing_timestamp": 219,
  "unique_chunks": 416,
  "timing_class_counts": {
    "BATCH_EARLIER": 219,
    "BATCH_LAST": 218,
    "UNIQUE": 198
  },
  "chunk_dt_s": {
    "min": 0.0003391000209376216,
    "median": 0.04582649999065325,
    "max": 0.06355240003904328,
    "mean": 0.04158263662647658,
    "cv": 0.3046877945990934,
    "fraction_below_min_dt": 0.03855421686746988
  },
  "frame_counter": {
    "first": 7677,
    "last": 8587,
    "span": 911,
    "rows": 635,
    "max_gap": 8,
    "gaps_gt1": 160,
    "non_increasing": 0
  },
  "status_frame_counter": {
    "first": 5785,
    "last": 6419,
    "span": 635,
    "rows": 635,
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
    "finite": 635,
    "exact_zero": 2,
    "value_changes": 606,
    "unique_values": 341
  },
  "J2": {
    "finite": 635,
    "exact_zero": 54,
    "value_changes": 585,
    "unique_values": 348
  },
  "J3": {
    "finite": 635,
    "exact_zero": 17,
    "value_changes": 610,
    "unique_values": 173
  },
  "J4": {
    "finite": 635,
    "exact_zero": 0,
    "value_changes": 468,
    "unique_values": 36
  },
  "J5": {
    "finite": 635,
    "exact_zero": 30,
    "value_changes": 591,
    "unique_values": 278
  },
  "J6": {
    "finite": 635,
    "exact_zero": 0,
    "value_changes": 455,
    "unique_values": 18
  }
}
```

## Detailed audit

```
IDENTIFICATION RUN AUDIT: WARNING
source: identification_data\identification_ID_NORMAL_005_trajectory_export_RUN_05_20261019T221637.csv
sha256: b0a9e6e3bf44b32bb0fc588c80efac65cd7df6dd56cf797061733be7cf2c247b

Conclusions
  basic csv readability        PASS
  logger signal integrity      WARNING
  initial 0g dry test          WARNING
  full six joint campaign      WARNING
  data use: logger dry test only

Criteria
  [PASS        ] R1 CSV parses and has data rows
      "635 data rows"
  [PASS        ] R2 Required columns present
      "all required columns present"
  [PASS        ] R3 No malformed or truncated rows
      {"malformed_rows": 0, "final_line_terminated": true}
  [PASS        ] I1 Metadata constant for the whole run
      {"inconsistent": {}, "empty_fields": [], "values": {"experiment_id": ["ID_NORMAL_005"], "trajectory_id": ["trajectory_export"], "run_id": ["RUN_05"], "commanded_speed": ["50.0"], "payload_g": ["0.0"], "contact_label": ["NORMAL"], "grasp_context": ["NONE"], "thermal_condition": ["COLD"]}}
  [PASS        ] I2 Experiment conditions match configured expectation
      {"expected": {"payload_g": 0.0, "contact_label": "NORMAL", "grasp_context": "NONE"}, "mismatches": {}}
  [PASS        ] I3 Thermal condition declared
      {"reported": ["COLD"], "note": "operator-declared label; no temperature signal is verified by this audit"}
  [PASS        ] I4 q and current are finite for every row
      {"bad_columns": {}, "timestamp_issues": {"missing": 0, "nonfinite": 0, "invalid": 0}}
  [PASS        ] I8 Explicit per-row freshness of position and current telemetry
      {"counts": {"position": {"FRESH": 635, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}, "current": {"FRESH": 635, "MISSING": 0, "NON_FINITE": 0, "UNKNOWN": 0}}, "rule": "FRESH = field present in the same STATUS and finite; a measured 0 is FRESH"}
  [PASS        ] I5 Current telemetry is fresh (no initialized or partially zeroed rows)
      {"all_zero_rows": 0, "leading_all_zero_rows": 0, "rows_with_many_zero_channels": 0, "many_zero_channel_threshold": 3, "first_last_such_row": null, "rule": "leading all-zero rows match RobotState zero initialization; simultaneous exact zeros on several channels cannot be separated from genuine idle current using the CSV alone"}
  [PASS        ] I6 No flat, frozen, or all-zero current channel
      "all channels vary"
  [NOT VERIFIED] I7 Current clipping
      {"note": "no documented current limit; extremum repeats are reported, not judged", "repeated_extrema": {"current2": {"samples_at_max": 1, "samples_at_min": 54, "min_mA": 0.0, "max_mA": 831.0}, "current3": {"samples_at_max": 1, "samples_at_min": 17, "min_mA": 0.0, "max_mA": 826.0}, "current5": {"samples_at_max": 1, "samples_at_min": 30, "min_mA": 0.0, "max_mA": 574.0}}}
  [PASS        ] T1 Timestamps never decrease
      {"negative_dt": 0}
  [WARNING     ] T2 Timestamps are unique (one STATUS per host timestamp)
      {"timing_provenance": "present", "label_counts": {"BATCH_EARLIER": 219, "BATCH_LAST": 218, "UNIQUE": 198}, "labels_inconsistent_with_timestamps": 0, "usable_rows_with_nonincreasing_timestamp": 0, "dt_zero_count": 219, "note": "batched rows are labelled; derivatives use only UNIQUE/BATCH_LAST rows. True sample times inside a batch are unknown and not reconstructed."}
  [PASS        ] T3 No unexpected recording gaps
      {"gap_count": 0, "gap_limit_s": 0.22895000001881272, "largest_gap_s": null}
  [PASS        ] T4 sample_index is contiguous
      {"discontinuities": 0}
  [PASS        ] T5 Effective sampling rate measured
      {"rows": 635, "duration_s": 17.25678739999421, "message_rate_hz": 36.739167337728965, "unique_timestamp_rate_hz": 24.04850858857653, "unique_timestamp_count": 416}
  [PASS        ] L2 All received STATUS frames are accounted for
      {"status_counter_contiguous": true, "rows": 635, "status_frames_received": 635, "discarded_frames_delta": 0, "parse_errors_delta": 0, "callback_errors_delta": 0, "other_frame_types_ignored_here": true}
  [PASS        ] L1 Logger writer reported no drops or errors
      {"dropped": 0, "writer_error": null, "logger_quality_status": "WARNING"}

Sampling
  rows=635 duration_s=17.257 message_rate_hz=36.739167337728965 unique_timestamp_rate_hz=24.04850858857653

Per-joint coverage (q deg, current mA; derived from last row of each timestamp batch)
  J1: LIMITED       q[0.00,48.01] range=48.01 travel+/-=48.01/-0.00 speed[0.00,240.71] current[0,668] std=131.4 moving_fraction=0.86
  J2: LIMITED       q[-40.21,-11.45] range=28.76 travel+/-=28.76/-0.00 speed[0.00,160.47] current[0,831] std=155.6 moving_fraction=0.78
  J3: LIMITED       q[102.04,114.99] range=12.95 travel+/-=12.95/-0.00 speed[0.00,106.98] current[0,826] std=156.8 moving_fraction=0.37
  J4: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[39,225] std=19.9 moving_fraction=0.00
  J5: LIMITED       q[65.96,69.55] range=3.59 travel+/-=0.00/3.59 speed[-26.75,0.00] current[0,574] std=103.7 moving_fraction=0.01
  J6: NOT EXERCISED q[-0.00,-0.00] range=0.00 travel+/-=0.00/-0.00 speed[0.00,0.00] current[33,60] std=2.5 moving_fraction=0.00

A single run never certifies the identification campaign; the plan requires 10-15 trajectory recipes with independent repeats.

```
