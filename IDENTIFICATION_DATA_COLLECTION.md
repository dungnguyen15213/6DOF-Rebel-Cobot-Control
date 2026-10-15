# Identification Data Collection

## Purpose and Scope

The identification logger captures unmodified CRI STATUS position and per-joint motor-current telemetry for later offline system identification. It does not move the robot when a recording is started. The existing explicit robot-motion workflow remains operator initiated. This stage is data acquisition only; it does not implement or tune Model A/B, RLS, VFF-RLS, CUSUM, payload estimation, collision logic, or a context state machine.

The first campaign condition is normal operation with no attached payload and no intentional external contact:

- `payload_g = 0`
- `contact_label = NORMAL`
- `grasp_context = NONE`
- `thermal_condition = COLD` or `WARM`, as appropriate

The GUI confirms before starting a run with other payload/contact/grasp labels. It does not prevent later, explicitly classified experiments.

## Signal Provenance and Units

The parser reads CRI STATUS `POSJOINTCURRENT` into the joint state and `CURRENTJOINTS` into the per-axis current state. These fields are raw API values; the logger does not convert them or replace them with analytics output. The controller captures `time.perf_counter()` immediately after the socket `recv` returns and passes that receive time to the callback. It is host monotonic time, not a robot-controller timestamp. Multiple STATUS messages in one received socket chunk can share that timestamp; duplicates are retained and reported rather than fabricated into a fixed sample period.

| Signal | CSV columns | Source / method | Units |
| --- | --- | --- | --- |
| Receive time | `timestamp` | Host monotonic time at CRI STATUS reception | seconds |
| Sample sequence | `sample_index` | Zero-based callback sequence; gaps reveal queue loss | count |
| Joint position | `q1` ... `q6` | Raw CRI `POSJOINTCURRENT` | degrees |
| Joint velocity | `qdot1` ... `qdot6` | Derived by finite difference using measured timestamp deltas; first sample unavailable | degrees/second |
| Joint current | `current1` ... `current6` | Raw CRI `CURRENTJOINTS` | milliamps (mA) |
| Candidate acceleration | `qddot1` ... `qddot6` | Derived from consecutive derived velocities with measured deltas; initial samples unavailable; not a trusted model input | degrees/second squared |

Positions and currents remain in live/API units. No radians or amperes conversion is performed. The repository has no verified motor-current clipping limit configured; reports show observed current extrema and state that the clipping limit is unknown.

## CSV Schema and Metadata

Every CSV row has these columns in this order:

```text
timestamp,sample_index,q1,q2,q3,q4,q5,q6,qdot1,qdot2,qdot3,qdot4,qdot5,qdot6,current1,current2,current3,current4,current5,current6,qddot1,qddot2,qddot3,qddot4,qddot5,qddot6,experiment_id,trajectory_id,run_id,commanded_speed,payload_g,contact_label,grasp_context,thermal_condition
```

Run metadata is snapshotted once when recording starts and remains constant in the run:

| Field | Meaning |
| --- | --- |
| `experiment_id` | Campaign/condition identifier |
| `trajectory_id` | Joint excitation or path identifier; repeated executions retain the same trajectory ID |
| `run_id` | Unique physical execution ID, changed for every repeat |
| `commanded_speed` | Operator-selected CRI speed setting recorded for the run |
| `payload_g` | Attached payload mass label in grams; use 0 for the no-load campaign |
| `contact_label` | `NORMAL` for no intentional contact; other conditions must be labeled explicitly |
| `grasp_context` | `NONE` for the initial no-load campaign; otherwise explicitly labeled |
| `thermal_condition` | Operator-assessed `COLD` or `WARM` label; it is not a measured motor temperature |

The GUI refuses a run ID already present in an earlier identification CSV, so assign a fresh ID to every physical execution, including repeats.

Illustrative metadata only:

```text
experiment_id: ID_NORMAL_COLD_001
trajectory_id: J2_SWEEP_MEDIUM
run_id: RUN_01
commanded_speed: 40
payload_g: 0
contact_label: NORMAL
grasp_context: NONE
thermal_condition: COLD
```

Unavailable or incomplete numeric telemetry is written as `nan`, never substituted with zero. A measured `0.0` remains distinguishable from unavailable telemetry. In particular, current values are accepted only when `CURRENTJOINTS` is present in that same STATUS packet; a previously cached RobotState current vector is not reused.

## Files, Buffering, and Lifecycle

Runs are written under `identification_data/` using:

```text
identification_<experiment>_<trajectory>_<run>_<local-datetime>.csv
identification_<same-name>.csv.json
```

Filename components are sanitized. Files are created exclusively; if a name already exists, a numbered suffix is selected. A JSON sidecar records schema version, metadata, wall-clock start/end, monotonic receive-time span, row count, units, signal provenance, dropped-sample count, writer errors, and automatic quality results.

The CRI receive callback snapshots six positions, six currents, packet field-presence flags, and the receive timestamp while holding the robot-state lock. It enqueues an immutable sample with a non-blocking bounded queue (capacity 4096); the dedicated worker computes time-aware derivatives and writes CSV. It flushes after 128 records or at least once per second. It does not `fsync` each sample. If the queue is full, that sample is dropped, the drop count increases, `sample_index` exposes the gap, and the run fails its integrity check. The callback does not access Qt widgets or perform CSV writes/flushes.

Stop detaches the active logger under a short lock, then sends a queue sentinel and waits for all accepted records to drain, flushes and closes the CSV, runs the quality analysis, and writes the sidecar. Robot disconnect and normal application close use the same close path. A disconnect detected by the GUI timer also closes the run. The final CSV path and quality status are shown after a manual Stop & Save.

## Operator Workflow

1. Start the application and connect to the robot.
2. Confirm the robot and surrounding workspace are physically safe for the intended movement.
3. Confirm no external payload is attached and no intentional contact will occur.
4. Open **Identification Settings...** and set a descriptive experiment ID, trajectory ID, unique run ID, commanded speed, `payload_g = 0`, `contact_label = NORMAL`, `grasp_context = NONE`, and the appropriate `COLD`/`WARM` label.
5. Click **Start Identification Run**. This starts logging only; it does not command robot motion.
6. Use the existing safe operator control workflow to perform a short movement. For the first physical logger test, keep it approximately 30–120 seconds and use only operator-selected safe motions/speeds within existing robot/controller limits.
7. Click **Stop & Save Run**. Wait for the final path and PASS/WARNING/FAIL result.
8. Inspect the quality result and CSV/JSON artifacts before collecting longer data.

For a trajectory execution using the existing **Run** workflow, the application keeps its current confirmation and preparation behavior; the identification recording starts after reaching the trajectory start pose and closes when that task execution ends. Do not use this as a logger-only test if a physical path execution is not intended.

## Quality Check

The application runs the checker automatically after Stop & Save and stores its detailed report in the sidecar. To rerun it manually from the repository root:

```powershell
.\igus_env\Scripts\python.exe tools\check_identification_csv.py identification_data\<run-file>.csv
```

The report includes the input path, rows and duration, timestamp monotonicity, duplicate timestamps, `dt <= 0` and negative-`dt` counts, minimum/median/mean/maximum `dt`, P1/P5/P95/P99, approximate sample-rate statistics from positive measured `dt`, missing/non-finite values by column, incomplete current rows, all-zero current rows, repeated adjacent current values, q/qdot/current/qddot ranges, metadata values/consistency, sample-index gaps, schema, and units. A gap greater than five times that run's median `dt` is reported as a timing-gap heuristic, not a scientific acceptance threshold.

- **PASS** means the CSV has rows and required columns, raw q/current are present, timestamps are strictly usable with no duplicates or regressions, metadata is complete and constant, and no writer samples were dropped.
- **WARNING** means core raw-data integrity passed but derived initial values are unavailable, timing gaps were detected, or repeated/all-zero current values deserve inspection. A measured zero is not automatically treated as missing.
- **FAIL** means the run has no rows, missing raw position/current data, missing required columns, invalid/duplicate/nonmonotonic timestamps, inconsistent or incomplete metadata, queue loss, or a writer error. Do not use a failed run as a clean identification run.

These are acquisition-integrity checks only. They do not define anomaly thresholds or certify physical safety. Inspect the reported current ranges; no clipping limit is asserted without a verified configured value.

## Repeated No-Load Run Strategy

Keep each physical execution in its own file. For an initial conservative campaign, repeat safe excitation patterns with distinct `run_id` values, including positive and negative motion for individual joints, multiple operator-selected safe commanded speeds, and representative multi-joint paths. Reuse `trajectory_id` for the same path and change `run_id` for every execution. Use the robot/controller's established limits; this guide does not prescribe joint limits or safe speeds. Later offline validation should hold out entire run IDs, not randomly split samples from a run.

Keep the CSV, its JSON sidecar, the exact trajectory/path artifact, the quality-check output, and notes on the physical setup/thermal label together for every run. Do not begin longer collection until the 30–120 second dry test confirms real current availability, realistic timestamp spacing, no unexplained missing samples, and no unacceptable effect on CRI receive timing.

## Audit, Processed Dataset, and Dry-Lab Workflow

After every run, audit it before trusting it:

```powershell
.\.venv\Scripts\python.exe tools\audit_identification_run.py identification_data\<run-file>.csv --no-plots
# optional: --diagnostics robot_diagnostics.csv  --processed  --smoothing-window 5
```

Reports go to `audit_reports\<run-file>\` (`audit_report.json`, `audit_report.txt`, plots if matplotlib is installed). Exit code: 0 PASS/WARNING, 1 FAIL, 2 file/usage error. Raw CSVs are never modified. Criteria cover readability, metadata, finite q/current, stale/zeroed current rows, timestamp uniqueness/gaps, per-joint excitation (range and both directions), and logger drop counters. Clipping is reported NOT VERIFIED unless a current limit is documented. All thresholds are configurable heuristics in `AuditConfig`, not scientific limits.

**Timestamp caveat.** The timestamp is the host receive time of one socket read. Several STATUS messages parsed from one read share it (`dt == 0`); their true spacing is unrecoverable and is not fabricated. A run with more than 5% duplicate timestamps FAILs T2. Derivatives and the processed dataset use only the last row of each identical-timestamp batch.

**Processed dataset** (`--processed`, only written when the audit does not FAIL): SI-unit CSV (rad, rad/s, A) plus `.provenance.json`, with flags `flag_duplicate_timestamp`, `flag_current_invalid`, `flag_use_for_derivatives`, `batch_size`, `batch_position`. Live `qddot` is ignored; boundary derivative rows stay NaN. `core.identification_audit.model_features` builds Model A/B design matrices (no fitting), and `whole_run_holdout_splits` splits by run ID.

**Merging `robot_diagnostics.csv`** is refused (UNSUPPORTED): it has no `run_id`, uses an execution-relative `Time (s)` instead of the host clock, and is GUI-sampled. Do not align by guessing.

**Conservative 0 g dry-lab sequence**
1. Check workspace and E-stop; no payload, no intended contact.
2. Connect; confirm fresh telemetry (currents changing).
3. Confirm payload/contact/grasp/thermal labels in Identification Settings.
4. Record 30–60 s stationary.
5. Run one operator-approved low-speed bidirectional trajectory.
6. Stop & Save; close cleanly.
7. Run the audit immediately.
8. Reject or investigate any FAIL; do not repair raw data.
9. Only then begin the 10–15 trajectory campaign with distinct `run_id`s, including J4/J6 and both directions on every joint.
