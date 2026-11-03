"""The host side of ``robot.*`` in LIBERO-PRO: the arm, gripper and cameras of
:class:`~hexashell.benches.armhost.ArmHost` for a Panda whose base does not move.

Poses cross to the server as 4x4 tool poses in the arm frame (see :mod:`.sim`).  ``home`` returns
to the pose the hand started in.
"""

from __future__ import annotations

from ..armhost import ARM_CALLS, ArmHost, camera_checker, describe  # noqa: F401  (describe: for the session)
from . import protocol as P

PUBLIC = ARM_CALLS
CAMS = tuple(P.CAMERAS)
ALIASES = {"agent": "agentview", "agent_view": "agentview", "front": "agentview", "eye_in_hand": "wrist",
           "robot0_eye_in_hand": "wrist", "hand": "wrist"}
check_cam = camera_checker(CAMS, ALIASES)
#: the gap between the finger pads when fully open (m)
FINGER_STROKE_M = 0.08


class LiberoHost(ArmHost):
    public = PUBLIC
    cams = CAMS
    aliases = ALIASES
    finger_stroke_m = FINGER_STROKE_M
    step_s = P.STEP_S
    unreachable_note = ("the arm cannot get there (its base does not move): ask for another pitch or height, or a "
                        "point nearer the arm")
    stalled_advice = "ask for another pitch, or come from nearer the arm"
    stretched_advice = "aim at a point nearer the arm, or at another pitch"
    jammed_advice = "to get there, come from another side or pitch, or aim nearer the arm"
