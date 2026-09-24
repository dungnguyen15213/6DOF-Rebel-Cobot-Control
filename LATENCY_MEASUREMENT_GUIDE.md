# CRI Latency Measurement Method

## Time Bases

Latency intervals use `time.perf_counter()`, a monotonic high-resolution clock. Wall-clock timestamps, when required for external experiment records, must be captured separately and are not subtracted from `perf_counter()` values.

## Communication Metrics

For a command with CRI command ID $i$, the controller records the timestamp immediately before `sendall` as $t_{tx,i}$. A command-specific numeric ACK records the socket-buffer receive timestamp $t_{rx,i}$ immediately after `recv` returns.

$$
RTT_i = (t_{rx,i} - t_{tx,i}) \times 1000\ \mathrm{ms}
$$

Only successful `CMDACK` responses matching a pending command ID contribute to CRI ACK RTT samples. `CMDERROR` response times are tracked separately and STATUS or unsolicited messages do not affect RTT. The displayed one-way value is an estimate:

$$
D_{oneway,i} \approx RTT_i / 2
$$

This assumes approximately symmetric network delay and is not a direct one-way measurement. Peak consecutive jitter is:

$$
J_{peak} = \max_i |RTT_i - RTT_{i-1}|
$$

The tracker keeps a bounded history and removes unmatched command IDs after a configurable timeout. This is Method 1, a CRI command-response communication experiment.

## Telemetry-Observed Motion Response

A controlled jog trial captures GUI intent $t_{gui}$ at the input event and binds it to the first matching, nonzero `ALIVEJOG` transmission for the selected joint and direction. STATUS joint positions are differentiated into velocity. Motion onset requires the requested direction, a threshold above both the configured minimum and stationary-noise multiplier, and consecutive confirmation samples. The timestamp of the first crossing is retained, so confirmation does not inflate measured latency.

$$
L_{gui-motion} = (t_{motion} - t_{gui}) \times 1000\ \mathrm{ms}
$$

$$
L_{tx-motion} = (t_{motion} - t_{tx}) \times 1000\ \mathrm{ms}
$$

$$
L_{gui-tx} = (t_{tx} - t_{gui}) \times 1000\ \mathrm{ms}
$$

This is Method 2, a JOG GUI/TX-to-telemetry-observed motion-response experiment, not pure actuator latency. It includes GUI scheduling, transport, controller and servo processing, STATUS sampling, return transport, and local packet handling. A trial can end as `DETECTED`, `TIMEOUT`, `INVALID_ALREADY_MOVING`, or `CANCELLED`; only `DETECTED` produces a motion-latency sample. Method 1 and Method 2 are separate measurements: $RTT/2$ must not be added to or interpreted as physical response time.

## Trial Event Records

`latency_experiments.csv` is event based: the application writes exactly one row for every finalized trial, including detected, timed-out, invalid, and cancelled trials. Each row is identified by a session-scoped `LAT-<session>-xxxxxx` trial ID and includes wall-clock metadata, monotonic timestamp values, GUI-to-TX, TX-to-motion, GUI-to-motion, RTT diagnostics, threshold/noise metadata, and final status. Paper execution-latency statistics must use this event log rather than the high-frequency analytics telemetry CSV.

## Experimental Procedure

1. Hold the selected joint stationary until the noise estimator has observed STATUS velocity samples.
2. Issue one jog input at a time and allow one matching nonzero command to arm the trial.
3. Release the jog before starting another trial. Release and disconnect cancel the active trial.
4. Record RTT distributions separately from motion-response distributions. Do not report RTT/2 as measured physical response time.
5. Export raw CSV data and report sample count, mean, standard deviation, median, P95, P99, min, max, and peak consecutive jitter alongside robot configuration, requested speed, joint, command type, and controller/firmware versions.
