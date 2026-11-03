"""Moving LIBERO's Panda through its official controller (server side).

Every step sends LIBERO's 7-dim action of robosuite's BASIC composite controller with the arm under
OSC_POSE (delta input, base frame) and the gripper under GRIP:

    arm[0:6] (OSC_POSE deltas) | gripper[6] (sign)

The arm and the gripper move as :mod:`hexashell.benches.arm` describes.
"""

from __future__ import annotations

import numpy as np

from ..arm import DT, Arm
from . import protocol as P

if abs(DT - P.STEP_S) > 1e-9:
    raise RuntimeError("the arm control assumes LIBERO's control period")

#: lowest the grasp point may be asked to go (arm frame: the table or floor the robot works on is
#: near z = 0 in every LIBERO scene)
FLOOR_Z = -0.05


class Robot(Arm):
    """The arm and gripper of one episode.  Rebuild after every reset."""

    def __init__(self, sim, log=None):
        robot = sim.rs.robots[0]
        if int(sim.rs.action_dim) != 7:
            raise RuntimeError(f"unexpected action dim {sim.rs.action_dim}")
        super().__init__(sim, robot.composite_controller.part_controllers["right"], robot.gripper["right"], log=log)

    def action(self, arm6: np.ndarray, grip: float, **extra) -> np.ndarray:
        out = np.zeros(7)
        out[0:6] = arm6
        out[6] = grip
        return out
