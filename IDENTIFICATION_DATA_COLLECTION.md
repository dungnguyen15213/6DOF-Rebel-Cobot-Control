# Identification Data Collection

This repository now includes a dedicated raw telemetry logger for offline system-identification experiments. The logger preserves the raw CRI/STATUS values and records the metadata needed to organize repeated runs.

## 1. Start an identification recording

1. Connect to the robot from the application.
2. In the Execution tab, fill in the identification metadata fields:
   - Experiment ID
   - Trajectory ID
   - Run ID
   - Commanded speed (taken from the execution speed spin box unless you override it in the metadata panel)
   - Payload (g)
   - Contact label
   - Grasp context
   - Thermal condition
3. Click Start Identification Run.
4. Run the desired motion or trajectory.

## 2. Stop and save a run

1. When the motion is complete, click Stop & Save Run.
2. The logger closes the active CSV file and keeps the run separate from any prior files.
3. The filename is created as:
   identification_<experiment_id>_<trajectory_id>_<run_id>_<timestamp>.csv
4. If the exact file already exists, the logger automatically creates a numbered suffix instead of overwriting the previous file.

## 3. Metadata fields

Every row in the identification CSV includes:

- experiment_id
- trajectory_id
- run_id
- commanded_speed
- payload_g
- contact_label
- grasp_context
- thermal_condition

The first normal-operation identification condition is:

- payload_g = 0
- contact_label = NORMAL
- grasp_context = NONE
- thermal_condition = COLD or WARM depending on the lab environment

## 4. Output location

The logger writes to the repository folder:

- identification_data/

The exact file path is shown in the GUI when the recording starts.

## 5. CSV schema

The raw CSV columns are:

- timestamp
- q1..q6
- qdot1..qdot6
- current1..current6
- qddot1..qddot6
- experiment_id
- trajectory_id
- run_id
- commanded_speed
- payload_g
- contact_label
- grasp_context
- thermal_condition

The values are recorded in the same raw units used by the live application:

- q1..q6: degrees
- qdot1..qdot6: deg/s (finite-difference derivative of the raw joint samples)
- qddot1..qddot6: deg/s^2 (candidate acceleration; derived only as a later offline check)
- current1..current6: mA
- timestamp: seconds from a monotonic clock (`time.perf_counter` / CRI receive timestamp)

## 6. Raw telemetry rules

The following are intentionally preserved unchanged:

- timestamp
- q1..q6 (raw CRI STATUS joint positions)
- current1..current6 (raw CRI STATUS motor-current telemetry)

No raw value is overwritten by FF-RLS, CUSUM, payload estimation, filtering, or other analytics. Derived values such as qdot and qddot are stored in clearly named columns and never replace the raw telemetry columns.

## 7. Data-quality checker

Run the standalone quality checker with:

python tools/check_identification_csv.py identification_data/<file_name>.csv

It reports:

1. total number of rows
2. first and last timestamps
3. timestamp monotonicity
4. duplicate timestamps
5. invalid or negative dt values
6. dt minimum / maximum / mean / median
7. dt percentiles (1%, 5%, 95%, 99%)
8. missing / NaN values per column
9. q ranges for each joint
10. velocity ranges for each joint
11. current extrema per joint
12. acceleration ranges if present
13. experiment_id / trajectory_id / run_id values found
14. metadata values found
15. schema and units summary

No clipping limit is inferred unless it is explicitly documented in the repository; if no verified value is available, the report states that clearly.

## 8. How CRI telemetry enters the logger

The raw CRI STATUS values enter the code through:

- [cri_lib/cri_protocol_parser.py](cri_lib/cri_protocol_parser.py)
- [cri_lib/cri_controller.py](cri_lib/cri_controller.py)

The parser reads `POSJOINTCURRENT` into `RobotState.joints_current` and `CURRENTJOINTS` into `RobotState.current_joints`; the controller calls the live status callback with a monotonic receive timestamp. The identification logger is attached in the GUI callback in [gui/main_window.py](gui/main_window.py), which ensures the raw telemetry is recorded exactly as received.

## 9. Known uncertainty

The repository exposes raw joint positions and raw per-joint currents from CRI STATUS. Joint velocities are not directly reported as a separate STATUS field in the parser; they are derived from finite differences of raw joint-position samples. Acceleration is similarly a candidate derivative and is only recorded as a later offline check.
