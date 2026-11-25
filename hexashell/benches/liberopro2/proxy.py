"""The ``robot`` object a cell sees in LIBERO-Pro 2.0: LIBERO-PRO's (:mod:`hexashell.benches.libero.proxy`), a
Panda arm with a parallel gripper whose base is fixed beside a table (or on the floor).  Every call travels to
the host process, which runs the simulator and the perception models; results come back as plain types.

Units: metres, degrees.  Arm frame: origin at the arm's base plate, x forward (from the arm toward the
workspace), y to its left, z up; the robot never moves, so it holds all episode.  Pixels are (x, y) = (column,
row) from the top-left corner.  The camera models and the robot's readings are the robot's own; the docstrings
below say where the benchmark may make them differ from the world.
"""

from __future__ import annotations

from ..libero import proxy as _libero
from ..libero.proxy import NotExecuted, RobotError  # noqa: F401 - the names cells catch


class Robot(_libero.Robot):
    @property
    def task(self):
        """The task: the instruction as it stands (the benchmark may change it during the episode), step budget
        (used / left), the hand's start pose."""
        return self._call("task", (), {})

    def camera(self, cam):
        """Camera model as the robot has it calibrated: focal length (px), image centre, pose in the arm frame
        now.  The real camera may sit or zoom otherwise; compare where things appear with where they are."""
        return self._call("camera", (cam,), {})
