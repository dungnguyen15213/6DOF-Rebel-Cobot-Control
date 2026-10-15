# Pathway / Trajectory Export Reference

`PathExecutor` stores a trajectory as a start pose plus a task trajectory. The canonical saved artifact is a CSV in the project `Pathways/` directory, with one row per joint target and explicit phase separation.

| Format | Content |
|---|---|
| Phase-aware trajectory CSV | `PREPARATION` start pose + `TASK` segment |
| Legacy CRI CSV | Ordered `CMD Move Joint` payloads, still parsed for compatibility |
| Joint CSV | Ordered six-axis targets with controller velocity |
| JSON | Raw target payloads and extension axes |

The start pose is always explicit because the robot's actual starting joint state is only known at execution time. The execution engine therefore moves to the start pose first and then runs only the task motion.

The standard field layout is:

```csv
Phase,Step,J1 (deg),J2 (deg),J3 (deg),J4 (deg),J5 (deg),J6 (deg),Velocity (%)
PREPARATION,0,0.0000,-10.0000,135.0000,0.0000,20.0000,0.0000,50.0
TASK,1,1.0000,-9.5000,134.5000,0.0000,20.5000,0.0000,50.0
```

The controller is still responsible for target timing and motion profiles, but the saved file now separates the robot-preparation motion from the measured experiment segment.