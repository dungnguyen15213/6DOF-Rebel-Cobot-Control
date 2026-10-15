# CRI Path Export Guide

The digital twin can plan geometry, but robot execution is driven by a saved trajectory CSV. The execution workflow is now independent from the last GUI planning session:

1. Design a pathway or preview a planned path.
2. Export the result to the project `Pathways/` folder.
3. Restart or keep the application open.
4. Connect the robot.
5. Select the saved CSV in the Execution tab.
6. Execute. The application automatically moves to the CSV-defined start pose, then runs only the measured task segment while timing and identification logging are active.

This mirrors the real robot workflow: the robot's current position is only known at execution time, so the stored CSV explicitly defines the start pose and the task motion.

## Default trajectory directory

The default export and browse location is the project-level `Pathways/` directory. Trajectory CSVs are reusable execution artifacts and are not implicitly tied to the last run of the planning tab.

## Phase-aware trajectory format

The recommended format separates the preparation move from the actual experiment path:

```csv
Phase,Step,J1 (deg),J2 (deg),J3 (deg),J4 (deg),J5 (deg),J6 (deg),Velocity (%)
PREPARATION,0,0.0000,-10.0000,135.0000,0.0000,20.0000,0.0000,50.0
TASK,1,1.0000,-9.5000,134.5000,0.0000,20.5000,0.0000,50.0
TASK,2,2.0000,-9.0000,134.0000,0.0000,21.0000,0.0000,50.0
```

Interpretation:

- `PREPARATION`: the required robot start pose, which is reached before the measured motion begins.
- `TASK`: the trajectory segment executed after the robot confirms it has reached the start pose.
- The task path is the only motion included in the experiment timer and identification logger.

## Legacy compatibility

The application still accepts a legacy file shaped like this:

```csv
Step,CRI Command
0,"CMD Move Joint 0.0000 -10.0000 135.0000 0.0000 20.0000 0.0000 0.0 0.0 0.0 50.0"
1,"CMD Move Joint 1.0000 -9.5000 134.5000 0.0000 20.5000 0.0000 0.0 0.0 0.0 50.0"
```

For the legacy format, the first waypoint is treated as the start pose and the ordered sequence is treated as the task trajectory.

## Execution semantics

The execution flow is intentionally split:

1. Move from the robot's current physical position to the CSV-defined preparation/start target.
2. Wait for `EXECEND` confirmation.
3. Start the experiment timer and identification logger.
4. Execute the task trajectory.
5. Stop timing and logging as soon as the final task point completes.

Only the task segment is included in the measured experiment duration.