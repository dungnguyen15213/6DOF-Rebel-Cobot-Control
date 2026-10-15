# Offline preprocessing policy

Raw CSV files in `identification_data/` are never modified. `core.identification_audit.build_processed_dataset` (run automatically by `core.run_report`, or by `tools/audit_identification_run.py --processed`) writes a separate `processed_dataset.csv` plus `processed_dataset.csv.provenance.json` (source SHA-256, parameters, exclusion counts, limitations).

- Every raw row is kept, with `source_row` and `sample_index`; `status_frame_counter`, `timing_class`, `position_validity` and `current_validity` are copied through.
- Representative samples: the last row of each identical-timestamp batch (policy, recorded in the provenance). Other rows keep raw values but have no derivatives.
- Units: q deg -> rad, current mA -> A (division by 1000; values are not otherwise altered). Current sign is unverified (all channels nonnegative so far).
- Candidate velocity: central difference of q on representative rows. Candidate acceleration: central difference of that velocity. Both require both neighbouring receive intervals >= `MIN_DERIVATIVE_DT_S` (10 ms, heuristic) and finite inputs; otherwise the value is NaN, never 0. Boundary rows are NaN. Smoothing is off unless requested and is then recorded.
- Counts of excluded rows (not batch-last, no valid velocity, no valid acceleration) are in the provenance.
- Limits: times are host receive times, not sample times, and STATUS carries no device timestamp. Intervals are not assumed equal. Acceleration is exploratory and must not be treated as physical ground truth. Model A/B fitting stays offline, outside the robot application.
