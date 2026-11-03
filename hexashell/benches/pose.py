"""The hand's orientation in pitch, yaw and roll (degrees), in a frame with x forward, y left, z up.

* ``pitch``: how far the approach direction (wrist -> between the fingers) points below the
  horizontal; 0 level, 90 straight down.
* ``yaw``: heading of the approach direction: 0 forward (+x), +90 to the left (+y).  Pointing
  straight down, yaw instead turns the fingers about the vertical: at yaw 0 they close along y
  (left-right), at yaw 90 along x.
* ``roll``: turn of the fingers about the approach direction.

Tool rotation (columns: approach, finger line, approach x finger line) = Rz(yaw) Ry(pitch) Rx(roll).
Pointing straight down, yaw and roll turn about the same axis: :func:`angles_of` then reports it all
as yaw, roll 0.  Results print :func:`shown_angles`, which does the same within ``FOLD_DEG`` of
vertical (the small tilt then reads as the same tilt the other way); farther from vertical they
print as near the angles asked as the rotation allows (:func:`shown_near`: a pitch may read past 90).
"""

from __future__ import annotations

import math

import numpy as np

from ..geometry import rot_x, rot_y, rot_z, wrap_deg

#: Below this horizontal share of the approach direction the hand counts as vertical.
_VERTICAL = math.sin(math.radians(1.0))


def rotation_of(pitch: float, yaw: float, roll: float) -> np.ndarray:
    r = math.radians
    return rot_z(r(yaw)) @ rot_y(r(pitch)) @ rot_x(r(roll))


def angles_of(rot: np.ndarray) -> tuple[float, float, float]:
    """(pitch, yaw, roll) in degrees of a tool rotation."""
    approach = rot[:, 0]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(approach[2])))))
    if math.hypot(approach[0], approach[1]) < _VERTICAL:
        fingers = rot[:, 1]  # pointing down or up the finger line is (-sin yaw, cos yaw, 0)
        yaw = math.degrees(math.atan2(-fingers[0], fingers[1]))
        return _round(pitch, wrap_deg(yaw), 0.0)
    yaw = math.degrees(math.atan2(approach[1], approach[0]))
    frame = rot_z(math.radians(yaw)) @ rot_y(math.radians(pitch))
    rest = frame.T @ rot
    roll = math.degrees(math.atan2(rest[2, 1], rest[1, 1]))
    return _round(pitch, wrap_deg(yaw), wrap_deg(roll))


#: Within this many degrees of straight down (or up), the angles are given with roll 0.
FOLD_DEG = 10.0


def folded(angles) -> tuple[float, float, float]:
    """The same angles with the roll folded into the yaw when the hand points within ``FOLD_DEG`` of
    vertical: exact pointing straight down or up, otherwise the tilt changes side (at most
    2 * FOLD_DEG).  Pointing down, Rz(y) Ry(90) Rx(r) = Rz(y - r) Ry(90); pointing up, Rz(y + r)."""
    pitch, yaw, roll = (float(v) for v in angles)
    if abs(pitch) < 90.0 - FOLD_DEG or roll == 0.0:
        return (pitch, yaw, roll)
    return _round(pitch, wrap_deg(yaw - roll if pitch > 0 else yaw + roll), 0.0)


def shown_angles(rot: np.ndarray) -> tuple[float, float, float]:
    """(pitch, yaw, roll) for printing: :func:`angles_of`, folded near vertical."""
    return folded(angles_of(rot))


def past_vertical(angles) -> tuple[float, float, float]:
    """The other angles of the same rotation, with the pitch past vertical (beyond 90 or -90):
    Rz(y) Ry(p) Rx(r) = Rz(y + 180) Ry(180 - p) Rx(r + 180)."""
    pitch, yaw, roll = (float(v) for v in angles)
    return _round(wrap_deg(180.0 - pitch), wrap_deg(yaw + 180.0), wrap_deg(roll + 180.0))


def nearest(angles, asked) -> tuple[float, float, float]:
    """Of ``angles`` and :func:`past_vertical` of them, the set nearer ``asked``: a hand asked for
    pitch 90 that tipped 15 degrees back reads pitch 105, not pitch 75 with yaw and roll turned
    round, and one asked for pitch 150 reads 150."""
    def off(a):
        return sum(abs(wrap_deg(float(x) - float(y))) for x, y in zip(a, asked))
    other = past_vertical(angles)
    return other if off(other) < off(angles) else tuple(float(v) for v in angles)


def shown_near(rot: np.ndarray, asked) -> tuple[float, float, float]:
    """:func:`shown_angles`, away from vertical as the set nearer the angles ``asked``."""
    plain = shown_angles(rot)
    return plain if abs(plain[0]) >= 90.0 - FOLD_DEG else nearest(plain, asked)


def _round(pitch: float, yaw: float, roll: float) -> tuple[float, float, float]:
    return (round(pitch, 2), round(yaw, 2), round(roll, 2))


def turn_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle (degrees) of the rotation between two rotation matrices."""
    c = (float(np.trace(a.T @ b)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))
