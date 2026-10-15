# Change report: 0 g identification integrity after RUN_03

No new hardware run was performed. Code, tests and docs only. Raw CSVs untouched. Robot motion, planner, FF-RLS, CUSUM, collision, payload, state machine, VFF-RLS and regressors untouched. No thresholds weakened.

1. Problems investigated: see phase1_run03_investigation.md and root_cause_analysis.md.
2. Fixes: per-row `position_validity`/`current_validity` in the logger; audit criteria I8 (freshness) and L2 (STATUS frame accounting); I5 PASS when all rows FRESH; `run_report.decisions()` derived from all audit criteria (previously ignored I4/T1/T4/L1/overall FAIL); CLI exit 1 on NOT READY.
3. Processed dataset: min-dt gated derivatives (NaN, never 0), extra provenance columns, exclusions and limitations; written automatically by the post-run report.
4. Exporter: explicit `--run`, hash/run identity validation, no substitution, README with missing artifacts and file hashes.
5. Docs: current semantics/freshness, Tests A/B/C, PREPROCESSING_POLICY.md, requirements.txt.
6. Tests: 104 tests OK headless (test_results.txt), including 19 audit-readiness and 2 handoff tests.
7. RUN_03 verdict: CONDITIONALLY READY (commissioning-grade, exploratory only). Freshness NOT VERIFIED (legacy file), J4/J6 unexcited, current sign unverified, acceleration FAIL, Model B FAIL.
8. Not verified: J6 behaviour, current sign/index mapping, freshness on hardware.
9. Next physical tests: A, B, C in IDENTIFICATION_DATA_COLLECTION.md.
10. Commands: `python -m core.run_report <csv>`; `python tools\audit_identification_run.py <csv>`; `python tools\export_identification_handoff.py --run <csv>`.
