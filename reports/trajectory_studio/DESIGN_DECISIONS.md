# Design decisions
1. Studio is authoring/preview/validation only; the package `trajectory_studio/` never imports hardware, `cri_lib` or core executor code (enforced by a test).
2. "Send to Pathways" exports a validated CSV and emits `loadRequested`; the existing Pathways confirmations still gate execution.
3. Export is byte-identical for the existing CSV format (verified on the fixture).
4. No invented limits, angles or speeds. Templates raise `IncompleteTemplate` until the operator supplies values. Test B J4/J6 shells are NOT executable.
5. Recipes are JSON with immutable revisions (`<name>.vN.recipe.json`); archive moves, never deletes. Run links go to a separate file; run CSV/JSON are only read.
6. Preview plots against waypoint index (no verified time model). 3D/FK/IK/collision deferred.
7. Undo/redo via QUndoStack snapshots.
