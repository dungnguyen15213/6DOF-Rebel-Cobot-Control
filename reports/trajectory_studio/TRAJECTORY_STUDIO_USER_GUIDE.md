# Trajectory Studio user guide
**Open:** Pathways tab -> "Trajectory Studio...". The studio never moves the robot.

**Edit a waypoint:** import a CSV (or generate a template), double-click a cell, or use the six-joint editor ("Set on rows"). Ctrl+Z / Ctrl+Y undo/redo. Lock a joint to prevent edits.

**Test B for J4 / J6:** Generate -> Bidirectional shell for J4 or J6 (or open the `TEST_B_J4/J6_TEMPLATE_NON_EXECUTABLE` recipe). It is blocked until YOU enter the start pose, positive/negative amplitude and speed from an approved source, optionally declaring joint limits and their source. Validate, then export.

**15-recipe campaign:** Experiment planner tab -> Create default plan (T01-T15, shells with no angles). Fill each recipe with approved values, save to the library, run each via Pathways, then link recorded runs for coverage/traceability.

**Preview / validate / export:** the plot and Validate panel are static. Export is blocked on any ERROR. NOT VERIFIED items (limits, feasibility, collision, speed meaning) always remain for the operator.

**Execute:** "Send to Pathways" exports and loads the CSV only. Run it from the Pathways tab with the normal ID entry and confirmations.
