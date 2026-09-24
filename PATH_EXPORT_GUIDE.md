# CRI Path Export Guide

The digital twin plans geometry only. It previews ordered joint or Cartesian waypoints, then sends each joint target to the Igus controller as a CRI `CMD Move Joint` command. The robot controller owns timing, acceleration, and velocity shaping.

## GUI Workflow

1. Select a joint-interpolated or Cartesian straight-line path.
2. Choose the number of preview waypoints and calculate the path.
3. Review the path in the 3D viewer.
4. Export CRI commands or move to the first waypoint and execute the path.

The velocity percentage is sent as part of each CRI command. It is a controller setting, not a local motion profile.

## Export Format

The CRI path CSV contains an ordered command per waypoint:

```csv
Step,CRI Command
0,"CMD Move Joint 0.0000 -10.0000 135.0000 0.0000 20.0000 0.0000 0.0 0.0 0.0 50.0"
```

Each command has the form:

```text
CMD Move Joint A1 A2 A3 A4 A5 A6 E1 E2 E3 velocity
```

No timestamps, local time steps, or acceleration vectors are exported.