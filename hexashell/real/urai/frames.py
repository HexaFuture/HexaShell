"""URAI's frames: its world frame and its TCP are used as they are; only the tool's axes are renamed.

URAI gives a tool's orientation as ``rpy_deg``: x-y-z Euler angles about the fixed axes, in degrees,
R = Rz(c) Ry(b) Rx(a) (scipy's ``"xyz"``).  HexaShell's pitch, yaw and roll
(:mod:`hexashell.motion.pose`) turn a tool that approaches along its +x and closes along its +y.

The hand's own axes are HexaShell's tool axes: forward (where the fingers point), left (along the closing
line) and up (forward x left; world up for a level hand at roll 0).  ``goto(hand_by=..., hand_turn=...)``
moves in them (:func:`hand_move`); :func:`hand_move_between` gives the two for a recorded motion.
"""

from __future__ import annotations

import math

import numpy as np

from ...geometry import exp_so3, homogeneous, log_so3, rot_x, rot_y, rot_z
from ...motion.pose import angles_of, rotation_of

#: PiPER's approach axis (+z) and closing axis (+x) are HexaShell's +x and +y.
C_G_P = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
#: |cos b| under this: b is +-90 degrees, where only a - c (or a + c) counts; c is then given as 0, as scipy does
_LOCKED = 1e-7
#: the largest hand_turn of one move (degrees): URAI turns from the measured orientation the shorter way
#: round, and this leaves room for a hand a few degrees off the orientation last asked
HAND_TURN_MAX_DEG = 170.0


def to_rpy(pitch: float, yaw: float, roll: float) -> list[float]:
    """URAI's ``rpy_deg`` of a tool turned to HexaShell's pitch, yaw and roll (degrees)."""
    return rpy_of(rotation_of(pitch, yaw, roll) @ C_G_P)


def from_rpy(rpy_deg) -> tuple[float, float, float]:
    """HexaShell's (pitch, yaw, roll) of a tool at URAI's ``rpy_deg``."""
    return angles_of(rotation_rpy(rpy_deg) @ C_G_P.T)


def tcp_pose(state: dict) -> np.ndarray:
    """The 4x4 pose of a TCP as URAI reports it (``xyz``, ``rpy_deg``), in URAI's tool axes."""
    return homogeneous(rotation_rpy(state["rpy_deg"]), state["xyz"])


def hand_axes(rpy_deg) -> np.ndarray:
    """The hand's own axes at URAI's ``rpy_deg``, in the world: the columns are forward (URAI's tool +z),
    left (its +x, the closing line) and up (its +y)."""
    return rotation_rpy(rpy_deg) @ C_G_P.T


def hand_move(xyz, rpy_deg, hand_by=(0.0, 0.0, 0.0), hand_turn=(0.0, 0.0, 0.0)) -> tuple[np.ndarray, list[float]]:
    """Where ``hand_by`` (metres along the hand's axes) and ``hand_turn`` (a rotation vector in them, degrees)
    take a TCP at ``xyz`` and ``rpy_deg``: both in the axes the hand has at the start, the turn about the TCP.
    As 4x4 poses in the hand's axes, end = start @ [exp(hand_turn), hand_by].  The end's xyz and rpy_deg."""
    axes = hand_axes(rpy_deg)
    end = np.asarray(xyz, float) + axes @ np.asarray(hand_by, float)
    return end, rpy_of(axes @ exp_so3(np.radians(np.asarray(hand_turn, float))) @ C_G_P)


def hand_move_between(start: dict, end: dict) -> tuple[list[float], list[float]]:
    """The ``hand_by`` (m) and ``hand_turn`` (degrees) that take a TCP from ``start`` to ``end`` (each ``xyz``
    and ``rpy_deg``, as URAI reports a TCP): the inverse of :func:`hand_move`, for writing recorded motions as
    goto calls.  The turn comes out as the shorter way round (at most a half turn)."""
    axes = hand_axes(start["rpy_deg"])
    shift = axes.T @ (np.asarray(end["xyz"], float) - np.asarray(start["xyz"], float))
    turn = np.degrees(log_so3(axes.T @ hand_axes(end["rpy_deg"])))
    return [float(v) + 0.0 for v in shift], [float(v) + 0.0 for v in turn]


def rotation_rpy(rpy_deg) -> np.ndarray:
    a, b, c = (math.radians(float(v)) for v in rpy_deg)
    return rot_z(c) @ rot_y(b) @ rot_x(a)


def rpy_of(rot: np.ndarray) -> list[float]:
    """``rpy_deg`` of a rotation, b in [-90, 90]."""
    r = np.asarray(rot, float)
    cos_b = math.hypot(r[0, 0], r[1, 0])
    b = math.atan2(-r[2, 0], cos_b)
    if cos_b < _LOCKED:
        a, c = math.atan2(-r[1, 2], r[1, 1]), 0.0
    else:
        a, c = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    return [math.degrees(v) + 0.0 for v in (a, b, c)]  # + 0.0: no -0.0
