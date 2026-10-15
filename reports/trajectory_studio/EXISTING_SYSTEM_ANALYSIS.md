# Existing system analysis
- Execution authority: `core/path_executor.py`. CSV columns `Phase,Step,J1..J6 (deg),Velocity (%)`. First PREPARATION row = start pose; TASK rows = waypoints.
- The executor uses ONE velocity for the whole file (last value read). Dwell is unsupported. The studio therefore keeps velocity per trajectory and does not model dwell.
- Execution UI: Pathways tab in `gui/main_window.py` (load -> experiment/trajectory/run IDs -> condition confirmation -> Yes/No -> run). Motion speed range 1-100.
- No authoritative joint limits, collision model or time parameterisation exist in the repository (`config.yaml`, `rebel_config.json`, `core/kinematics.py` checked). Limits are operator-declared only and always reported NOT VERIFIED.
- Fixture: `Pathways/trajectory_export.csv` (28 rows, 1 PREPARATION + 27 TASK, 50%).
