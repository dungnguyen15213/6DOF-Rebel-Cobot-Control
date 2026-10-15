# Known limitations
- Nothing was run on hardware; only static validation and offscreen GUI tests were executed.
- Joint limits, feasibility, collision and physical meaning of Velocity (%) are NOT VERIFIED.
- Per-waypoint speed and dwell are unsupported by the executor and are not exported.
- Interpolation is linear in joint space for authoring only; the controller path may differ.
- Preview has no time axis; velocity/acceleration plots are not provided.
- Release 3 deferred: verified 3D/FK/IK model, collision checking, time parameterisation, advanced design of experiments.
- Planned-vs-recorded comparison is a range comparison without time alignment.
- Screenshots are offscreen renders of the real window; the planner tab image shows an empty campaign.
