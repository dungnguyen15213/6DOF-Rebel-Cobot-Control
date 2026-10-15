# Root cause analysis: RUN_02 (T133850) identification data

Evidence: `identification_data/..._RUN_02_20261015T133850.csv` (632 rows, unmodified).

| Observation | Cause | Status |
|---|---|---|
| 421 unique timestamps for 632 rows; many dt = 0 | CRI STATUS frames have no device timestamp. The timestamp is host `perf_counter()` after `recv()`; several frames in one socket read share it. | Confirmed from code and data |
| Chunk receive spacing median 45.9 ms (~21.8 Hz), min 0.21 ms, max 67.8 ms (CV 0.32) | Reads arrive irregularly. Whether this is receive-thread scheduling, TCP coalescing or robot send behaviour is unknown. | NOT CONFIRMED (needs hardware test) |
| 87 rows labelled BATCH_* although they are alone on their timestamp | `frames_in_chunk` counted every frame type (CMDACK, RUNSTATE, ...), not STATUS only. | Confirmed; fixed: labels now count STATUS frames only |
| `frame_counter` 1912..2819 (907) vs 632 rows, gaps up to 7 | Counter counted all frame types. Gaps are probably non-STATUS frames. | Probable; new per-type counters and `status_frame_counter` allow confirming on RUN_03 |
| Live qddot up to ~570,000 deg/s^2 | Quantised positions divided by sub-ms receive intervals (0.2 ms). | Confirmed; fixed: live derivatives are NaN when dt < 10 ms (heuristic, documented, raw data unchanged) |
| `current6` exactly 0 on all rows | Unknown: J6 may be idle in this trajectory, or the field is not reported. | NOT CONFIRMED (hardware item) |
| Zero currents on J1/J2/J3/J5 on 18-48 rows | Cannot separate genuine idle current from zero-initialised state from the CSV alone. | NOT CONFIRMED |
| Writer drops | None (`sample_index` contiguous). | Confirmed |

Nothing in the fix invents timestamps or adds epsilon offsets. Timestamp quality remains limited by host receive time.

## RUN_03 update
See phase1_run03_investigation.md. I4 was already in audit_run aggregation; the old run_report.decisions() ignored it (fixed). Freshness is now explicit per row; STATUS accounting verified (632 rows = 1985-1353).
