# Quick Start: CRI Path Planning

## Plan and Preview

1. Start the application with `python main.py`.
2. In **Path Planning**, set the start and target Cartesian positions.
3. Choose either **Joint interpolation** or **Cartesian straight line**.
4. Set the waypoint count and select **Calculate Path**.
5. Use the playback controls and 3D viewer to review the spatial path.

## Export and Execute

1. Connect and enable the Igus ReBeL controller.
2. Set the CRI velocity percentage.
3. Select **Export CRI Path CSV** to save ordered `CMD Move Joint` payloads.
4. Use **Move to Start** to safely position the robot at the first target.
5. Select **Execute Path** to stream the target sequence over CRI.

The digital twin produces only waypoint geometry. Timing, acceleration, and velocity profiles are performed by the CRI-compliant controller.

See [PATH_EXPORT_GUIDE.md](PATH_EXPORT_GUIDE.md) for the command format and export details.
