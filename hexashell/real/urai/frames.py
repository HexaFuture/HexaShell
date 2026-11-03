"""URAI's frames: its world frame and its TCP are used as they are; only the tool's axes are renamed.

URAI gives a tool's orientation as ``rpy_deg``: x-y-z Euler angles about the fixed axes, in degrees,
R = Rz(c) Ry(b) Rx(a) (scipy's ``"xyz"``).  HexaShell's pitch, yaw and roll
(:mod:`hexashell.motion.pose`) turn a tool that approaches along its +x and closes along its +y.
"""

from __future__ import annotations

import math

import numpy as np

from ...geometry import homogeneous, rot_x, rot_y, rot_z
from ...motion.pose import angles_of, rotation_of

#: PiPER's approach axis (+z) and closing axis (+x) are HexaShell's +x and +y.
C_G_P = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
#: |cos b| under this: b is +-90 degrees, where only a - c (or a + c) counts; c is then given as 0, as scipy does
_LOCKED = 1e-7


def to_rpy(pitch: float, yaw: float, roll: float) -> list[float]:
    """URAI's ``rpy_deg`` of a tool turned to HexaShell's pitch, yaw and roll (degrees)."""
    return rpy_of(rotation_of(pitch, yaw, roll) @ C_G_P)


def from_rpy(rpy_deg) -> tuple[float, float, float]:
    """HexaShell's (pitch, yaw, roll) of a tool at URAI's ``rpy_deg``."""
    return angles_of(rotation_rpy(rpy_deg) @ C_G_P.T)


def tcp_pose(state: dict) -> np.ndarray:
    """The 4x4 pose of a TCP as URAI reports it (``xyz``, ``rpy_deg``), in URAI's tool axes."""
    return homogeneous(rotation_rpy(state["rpy_deg"]), state["xyz"])


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
