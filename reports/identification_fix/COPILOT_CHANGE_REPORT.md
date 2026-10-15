# Copilot change report

Changed: `cri_lib/frame_timing.py` (per-type counters, STATUS-only batching), `cri_lib/cri_controller.py` (STATUS-only stamping, parse/callback error counters, `frame_stats()`), `core/identification_logger.py` (`status_frame_counter`, min-dt derivative gate, accounting in sidecar), `gui/main_window.py` (frame-stat snapshots, background post-run report), new `core/run_report.py`, `tools/export_identification_handoff.py`, `tests/test_run_report.py`, docs.
Not changed: trajectory planner, motion control, FF-RLS, CUSUM, thresholds, payload inference, VFF-RLS, state machine. Raw data untouched.
Evidence: `root_cause_analysis.md`, `test_results.txt` (real output; only `test_move_to_start_latency` errors, PyQt6 not installed), `git_diff.patch`, `git_status.txt`.
Unverified (hardware): reason for ~22 Hz read rate, J6 zero current, zero-current rows, STATUS-only frame_counter gaps.
Verdict: CONDITIONALLY READY for a controlled 0 g dry test with new logger (RUN_03); RUN_02 itself is NOT READY as evidence.
