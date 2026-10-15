# Phase 1 - RUN_03 investigation (before the P0/P1 edits)

Run: `identification_ID_NORMAL_003_trajectory_export_RUN_03_20261015T135600.csv` (+ `.json`), 632 rows, unmodified. Audit at HEAD b56df16: WARNING.

## Verified facts
- STATUS accounting is complete: sidecar `frame_accounting` end-start `status_frames` = 1985-1353 = 632 = rows written = offered; `status_frame_counter` is 1353..1984 with every step 1; parse errors 0, callback errors 0, writer drops 0. Other frame types (RUNSTATE, VARIABLES, LOGMSG, GRIPPERSTATE, CYCLESTAT, CMDACK...) moved the all-frame counter separately, so earlier "gaps" were not STATUS loss.
- Only J1, J2, J3, J5 moved (q ranges J1 0..48, J2 -40..-11, J3 102..115, J5 66..70 deg). **J4 and J6 stayed at 0 deg.** J4 current is 18-19 mA (2 unique values) and J6 is 0 mA. Both are therefore *unexcited*, not proven faulty. The earlier statement "only J6 did not move" understated this.
- Current is nonnegative on every channel (min 0). Sign is not available or at least unverified.
- 2 of 631 consecutive current vectors are identical (little evidence of repeated/stale vectors).

## Code findings
- `I4` (finite q/current) is in `integrity`, so `_worst(integrity)` includes it and the top-level `audit_run` conclusions/overall_status already propagate it. **That earlier suspicion is not true for `audit_run`.**
- It IS true for `core/run_report.py::decisions`: its raw-integrity decision only looked at R1-R3, I1-I3. It ignored I4, T1, T4, L1 and the audit's overall FAIL, so a run with NaN currents could be shown as WARNING/CONDITIONALLY READY. Fixed in this task.
- Current freshness: the parser resets `status_fields_seen` on every STATUS and the GUI only copies `current_joints` when `CURRENTJOINTS` is in that set, otherwise NaN. Stale/initialised values are therefore not logged as measurements. But the CSV records no explicit per-row validity flag, so the audit cannot prove it (I5 relies on a zero-pattern heuristic that can also fire on genuine idle zeros and FAILs on leading zero rows). Fixed by adding explicit validity columns.
- Live derivative gating (`MIN_DERIVATIVE_DT_S`=10 ms) is applied in the logger; the offline processed dataset did not apply it. Fixed.
- No dependency file exists in the repo; PyQt6 is imported by `gui/` but not documented.
