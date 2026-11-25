"""The G2_crsB body (the WBC board's robot): its torso leans and goes down on three pitch joints, and its head
turns and tips on the torso (genie_sim 3.6.0 ``G2_crsB_90D.urdf``, read as data).

The chain (x forward, y left, z up in ``base_link``; each pitch joint turns about y, a positive angle tips
what it carries forward):

- ``idx01_body_joint1`` at (0.102, 0, 0.144) on ``base_link``, -1.0822..0 (the lower leg leans back);
- ``idx02_body_joint2`` 0.36 m up the lower leg, 0..2.653 (the knee: the thigh tips forward);
- ``idx03_body_joint3`` 0.38 m up the thigh (the hip), -1.907..1.039; ``idx04`` (roll about x, +-0.4364) at the
  same point; ``idx05`` (the torso's turn about its own long axis, +-3.0456) 0.149 m up;
- ``arm_base_link`` 0.3085 m above ``idx05``, turned -90 degrees about x: the shoulders sit level with its
  origin, 0.243 m to each side, so their height is the origin's; the torso's lean is q1 + q2 + q3;
- the head: ``idx11`` (turn, +-0.781) at (-0.013, 0, 0.496) on ``body_link5``, ``idx12`` (roll, +-0.151)
  (0.03335, 0, 0.07) further, ``idx13`` (tip down, -0.0181..0.5231) (-0.03035, 0, 0) further.

These reproduce the observations' ``arm_base_link`` to 0.1 mm (checked on pick_up_trash: the
start pose, the benchmark's "standard" body [-0.5327, 0.8683, 0] and a crouch at a 48-degree lean); the body
followed 90-120-row ramps to within 0.01 rad by the next request (the simulator waits for the whole body of
a free-base robot before it asks again).

A posture is asked as the torso's lean (degrees forward from upright) and the shoulders' height; the hip's
place along x is free, and the solution nearest the joints now wins (the body moves as little as it can),
within the joints' limits and with the hip between ``HIP_X`` and at least ``HIP_MIN_Z`` high.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: where the first body joint sits on base_link (x, z), the lower leg, the thigh, the hip to arm_base_link
J1_X, J1_Z = 0.102, 0.144
SHIN_M, THIGH_M = 0.36, 0.38
HIP_TO_TORSO_TURN_M = 0.149
TORSO_TURN_TO_ARM_BASE_M = 0.3085
TORSO_M = HIP_TO_TORSO_TURN_M + TORSO_TURN_TO_ARM_BASE_M
#: the body joints' limits (rad, the URDF's) and the margin kept from them
LIMITS = ((-1.0822, 0.0), (0.0, 2.653), (-1.907, 1.039), (-0.4364, 0.4364), (-3.0456, 3.0456))
LIMIT_MARGIN = 0.02
#: the hip's place (base_link x, m) and lowest height (z, m) the posture keeps: about the range of the
#: benchmark's own start postures (hip x 0.04-0.17, the lowest 0.43 m in sending_food) with some room
HIP_X = (-0.05, 0.25)
HIP_MIN_Z = 0.40
#: the head's joints (rad): turn about z, roll about x, tip down about y
HEAD_LIMITS = ((-0.781, 0.781), (-0.151, 0.151), (-0.0181, 0.5231))
HEAD_TURN, HEAD_TIP = 0, 2
#: the head camera's tilt on its mount (degrees down from the head's level), from a calibrated pose:
#: 81.9 degrees down at a 31.6-degree lean and the head tipped 25.8 degrees
HEAD_CAMERA_TILT_DEG = 24.5


def _t(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _rx(a: float) -> np.ndarray:
    m = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    m[1, 1], m[1, 2], m[2, 1], m[2, 2] = c, -s, s, c
    return m


def _ry(a: float) -> np.ndarray:
    m = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def _rz(a: float) -> np.ndarray:
    m = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    return m


_ARM_BASE_ON_LINK5 = _t(z=TORSO_TURN_TO_ARM_BASE_M) @ _rx(-math.pi / 2)


def link5(q) -> np.ndarray:
    """``body_link5`` (after the torso's turn) in base_link for the five body joints."""
    q1, q2, q3, q4, q5 = (float(v) for v in np.asarray(q, float).reshape(5))
    return (_t(J1_X, 0.0, J1_Z) @ _ry(q1) @ _t(z=SHIN_M) @ _ry(q2) @ _t(z=THIGH_M) @ _ry(q3) @ _rx(q4)
            @ _t(z=HIP_TO_TORSO_TURN_M) @ _rz(q5))


def arm_base(q) -> np.ndarray:
    """``arm_base_link`` in base_link for the five body joints."""
    return link5(q) @ _ARM_BASE_ON_LINK5


def head_in_arm_base(h) -> np.ndarray:
    """``head_link3`` (the head camera's mount) in arm_base_link for the three head joints: it rides on the
    torso, so only the head's joints move it there."""
    h1, h2, h3 = (float(v) for v in np.asarray(h, float).reshape(3))
    on_link5 = _t(-0.013, 0.0, 0.496) @ _rz(h1) @ _t(0.03335, 0.0, 0.07) @ _rx(h2) @ _t(-0.03035, 0.0, 0.0) @ _ry(h3)
    return np.linalg.inv(_ARM_BASE_ON_LINK5) @ on_link5


@dataclass(frozen=True)
class Posture:
    """Where the body's joints put the torso: its lean (rad, forward from upright), the shoulders' height and
    the arm base's place along x (m, base_link), the hip's place (x, z)."""

    lean: float
    shoulder_z: float
    shoulder_x: float
    hip: tuple[float, float]


def posture(q) -> Posture:
    """The posture of five body joints (roll and turn as they are)."""
    q = np.asarray(q, float).reshape(5)
    ab = arm_base(q)
    hip = _hip(q[0], q[1])
    return Posture(lean=float(q[0] + q[1] + q[2]), shoulder_z=float(ab[2, 3]), shoulder_x=float(ab[0, 3]),
                   hip=(float(hip[0]), float(hip[1])))


def _hip(q1: float, q2: float) -> np.ndarray:
    a, b = q1, q1 + q2
    return np.array([J1_X + SHIN_M * math.sin(a) + THIGH_M * math.sin(b),
                     J1_Z + SHIN_M * math.cos(a) + THIGH_M * math.cos(b)])


def within(q, margin: float = LIMIT_MARGIN) -> bool:
    q = np.asarray(q, float).reshape(-1)
    return all(lo + margin - 1e-9 <= v <= hi - margin + 1e-9 for v, (lo, hi) in zip(q[:3], LIMITS))


def _legs(hx: float, hz: float):
    """(q1, q2) with the knee bent forward that put the hip at (hx, hz), or None."""
    dx, dz = hx - J1_X, hz - J1_Z
    d2 = dx * dx + dz * dz
    c = (d2 - SHIN_M ** 2 - THIGH_M ** 2) / (2.0 * SHIN_M * THIGH_M)
    if not -1.0 <= c <= 1.0:
        return None
    q2 = math.acos(c)
    q1 = math.atan2(dx, dz) - math.atan2(THIGH_M * math.sin(q2), SHIN_M + THIGH_M * math.cos(q2))
    return q1, q2


#: the hip's places tried along x (m)
_HIP_STEP_M = 0.002


def solve(lean: float, shoulder_z: float, q_now) -> np.ndarray | None:
    """Body joints (five; roll and turn kept from ``q_now``) for a torso leaning ``lean`` (rad) with the
    shoulders ``shoulder_z`` high, the nearest to ``q_now`` of those within the limits; None if there is none."""
    q_now = np.asarray(q_now, float).reshape(5)
    hz = shoulder_z - TORSO_M * math.cos(lean)
    if hz < HIP_MIN_Z - 1e-9:
        return None
    best, best_cost = None, math.inf
    for hx in np.arange(HIP_X[0], HIP_X[1] + 1e-9, _HIP_STEP_M):
        legs = _legs(float(hx), hz)
        if legs is None:
            continue
        q1, q2 = legs
        q = np.array([q1, q2, lean - q1 - q2, q_now[3], q_now[4]])
        if not within(q):
            continue
        cost = float(np.sum((q[:3] - q_now[:3]) ** 2))
        if cost < best_cost:
            best, best_cost = q, cost
    return best


def heights(lean: float, q_now) -> tuple[float, float] | None:
    """The lowest and highest shoulders (m) the body reaches at a lean (rad), or None if it cannot lean so."""
    found = [z for z in np.arange(0.30, 1.40, 0.005) if solve(lean, float(z), q_now) is not None]
    if not found:
        return None
    return float(min(found)), float(max(found))


def leans(shoulder_z: float, q_now, lo_deg: float = -20.0, hi_deg: float = 110.0) -> tuple[float, float] | None:
    """The least and most lean (rad) from ``lo_deg`` to ``hi_deg`` (degrees) the body reaches with the shoulders at
    a height (m), or None."""
    found = [a for a in np.radians(np.arange(lo_deg, hi_deg + 1e-9, 0.5))
             if solve(float(a), shoulder_z, q_now) is not None]
    if not found:
        return None
    return float(min(found)), float(max(found))


__all__ = ["Posture", "arm_base", "head_in_arm_base", "heights", "leans", "link5", "posture", "solve", "within"]
