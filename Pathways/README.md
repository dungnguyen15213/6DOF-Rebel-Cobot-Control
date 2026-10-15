# Pathways

This directory is the project default location for reusable robot trajectory CSV files.

Supported CSV formats:
- Phase-aware trajectory exports with explicit start pose and task motion.
- Legacy `Step,CRI Command` files for backward compatibility.

The application loads trajectories from this folder and executes them by:
1. moving to the CSV-defined start pose,
2. waiting for physical completion,
3. running only the task segment while a timed identification run is active.
