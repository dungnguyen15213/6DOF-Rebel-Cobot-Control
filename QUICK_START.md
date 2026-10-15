# Quick Start: Trajectory Execution

## Design and export

1. Start the application with `python main.py`.
2. In **Path Planning**, set the start and target Cartesian positions.
3. Choose either **Joint interpolation** or **Cartesian straight line**.
4. Set the waypoint count and select **Calculate Path**.
5. Review the path in the 3D viewer.
6. Export the trajectory to the project `Pathways/` folder.

## Execute a saved trajectory

1. Connect and enable the Igus ReBeL controller.
2. In the **Execution** tab, select and load an existing CSV from `Pathways/` or browse to another valid trajectory file.
3. Set the motion speed and open **Identification Settings** to edit experiment metadata.
4. Click **Run**.

The run now follows this sequence automatically:

1. Move from the robot's actual current position to the CSV-defined start pose.
2. Wait for the robot to confirm the start pose (`EXECEND`).
3. Start the experiment timer and identification logger.
4. Run only the task path from the CSV.
5. Stop timing and logging when the final task waypoint completes; the identification CSV is saved automatically and its path is shown in the completion message.

The digital twin still produces waypoint geometry, but the saved CSV is now a reusable execution artifact instead of a GUI-only plan.

See [PATH_EXPORT_GUIDE.md](PATH_EXPORT_GUIDE.md) for the schema and compatibility details.
