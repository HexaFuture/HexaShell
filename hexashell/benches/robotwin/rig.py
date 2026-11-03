"""RoboTwin 2.0's two arms (aloha-agilex: two ARX X5 arms on a mobile base) as a motion
:class:`~hexashell.motion.rig.Rig`.

The arms' chains come from RoboTwin's own URDF (``assets/robotwin_aloha_arms.urdf``, the arm part of
``arx5_description_isaac.urdf``); each arm's base link stands where the robot's root pose puts it
(the robot at (0, -0.65, 0) turned 90 degrees, so the arms face +y across the table).  The flange is
link6; the grasp point (between the finger pads) is 0.143 m along its x axis, the fingertips 0.156 m.
The gripper is read from the finger joint (0 .. 0.055 m of travel per finger from fully closed to
fully open): a reading of 0.18 has the pads touching, 1.0 leaves 9 cm between them.

The table top is lowered by up to 3 cm per episode (10 cm more in one task), so an episode's rig is
:data:`ROBOTWIN` moved to the height measured in the start pictures (:func:`rig_for`).
"""

from __future__ import annotations

import math

from ... import world as W
from ...motion.rig import ASSETS, GripperModel, Rig
from . import protocol as P

#: metres between the pads for a reading: the finger joint is 0.055 r - 0.01 per finger, the pads
#: touch at joint 0
OPEN_GAP_M = 0.09


def finger_gap(reading: float, squeezing: bool) -> float:
    return max(0.0, 0.11 * float(reading) - 0.02)


ROBOTWIN = Rig(
    name="robotwin",
    bases={"left": ((-0.297, -0.4195, 0.782), math.pi / 2 + 0.02),
           "right": ((0.3063, -0.4185, 0.781), math.pi / 2 + 0.01)},
    urdf=ASSETS / "robotwin_aloha_arms.urdf",
    tips={"left": "fl_link6", "right": "fr_link6"},
    # the same arm as RoboDojo's (ARX X5): the URDF's limits are placeholders (+-10 rad)
    joint_ranges=W.JOINT_RANGES,
    grasp_reach=0.143,
    tip_reach=0.156,
    table_z=P.TABLE_TOP_Z,
    table_x=(-P.TABLE_X_HALF, P.TABLE_X_HALF),
    table_y=(-P.TABLE_Y_HALF, P.TABLE_Y_HALF),
    # the reading is the fingers' own position: closed on nothing it stops at 0.18 (pads touching)
    gripper=GripperModel(gap=finger_gap, closed=0.186, jammed=0.97, leads=False, open_gap_m=OPEN_GAP_M),
    step_s=P.STEP_S,
)


def rig_for(task: str, table_z: float) -> Rig:
    """The rig of an episode of ``task`` whose table top is at ``table_z``."""
    return ROBOTWIN.on_table(table_z, x=P.table_x(task), y=P.table_y())
