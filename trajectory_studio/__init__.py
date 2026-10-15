"""Hardware-independent trajectory-authoring engine for the ReBeL cobot.

Nothing in this package imports the CRI/robot layer, so previewing, validating
and exporting can never command motion. Execution stays in the existing
operator-controlled Pathways workflow.
"""

from trajectory_studio.models import (  # noqa: F401
    CATEGORIES,
    JOINT_NAMES,
    Trajectory,
    Waypoint,
)
