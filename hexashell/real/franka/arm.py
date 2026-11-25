"""The FR3 and its hand as the planner and the pictures need them: forward kinematics, the Jacobian of the
TCP, and capsules round the links for the table and self-collision checks.

The joints come from ``assets/fr3.urdf``.  The TCP is Franka's: 0.1034 m along the flange's z axis from
``fr3_link8``, between the finger tips.  Longer fingers need ``tool_length_m`` (the gripping centre) and
``tip_past_tcp_m`` (how far the tips reach past it), or the table check passes a finger that hits the table.  Its axes
follow URAI's convention, which :mod:`hexashell.real.urai.frames` turns into pitch, yaw and roll: it approaches
along +z and closes along +x.  Franka's hand closes along its y axis, hence :data:`TOOL_AXES`.

The capsules are our own conservative fit round the links, not a Franka model; they keep the arm from the table
and from itself, and show the model which pixels are the robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ...geometry import homogeneous, rot_z
from ...motion.chain import _turn, read_urdf_joints

URDF = Path(__file__).resolve().parents[2] / "assets" / "fr3.urdf"
#: Franka's "ready" pose: the hand straight down in front of the base
READY_Q = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)
#: fr3_link8 -> the hand: 45 degrees about the flange's z
HAND = homogeneous(rot_z(-math.pi / 4), (0.0, 0.0, 0.0))
#: Franka's TCP frame -> URAI's tool axes (+x along Franka's closing axis y, +z still the approach)
TOOL_AXES = homogeneous(rot_z(math.pi / 2), (0.0, 0.0, 0.0))
TOOL_LENGTH_M = 0.1034
#: How far the stock fingers' tips reach past that TCP
TIP_PAST_TCP_M = 0.009
#: The radius of the capsule round the fingers
FINGER_RADIUS_M = 0.012
#: The hand's maximum opening (m), Franka's data sheet; the hardware reports its own
HAND_OPENING_M = 0.08
#: Clearance (m) above the table: of the arm's capsules, and of the TCP
BODY_CLEARANCE_M = 0.01
TIP_CLEARANCE_M = 0.002
#: Two capsules whose surfaces come closer than this (m) count as touching
SELF_CLEARANCE_M = 0.005
#: Capsules that may not touch each other (the neighbours along the chain always do)
_FIXED = ("shoulder", "upper_arm")
_FAR = ("forearm_end", "wrist", "flange", "hand", "hand_bar", "fingers")
_NEAR_TOOL = ("hand", "hand_bar", "fingers")
SELF_PAIRS = tuple((a, b) for a in _FAR for b in _FIXED) + tuple((a, "elbow") for a in _NEAR_TOOL)


@dataclass(frozen=True)
class LinkCapsule:
    name: str
    a: np.ndarray
    b: np.ndarray
    radius_m: float


def segment_distance(p1, q1, p2, q2) -> float:
    """The shortest distance between two segments (Ericson, Real-Time Collision Detection, 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e <= 1e-12:
            s, t = min(max(-c / a, 0.0), 1.0), 0.0
        else:
            b = float(d1 @ d2)
            denominator = a * e - b * b
            s = min(max((b * f - c * e) / denominator, 0.0), 1.0) if denominator > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = min(max(-c / a, 0.0), 1.0), 0.0
            elif t > 1.0:
                s, t = min(max((b - c) / a, 0.0), 1.0), 1.0
    return float(np.linalg.norm(p1 + d1 * s - p2 - d2 * t))


class Fr3:
    def __init__(self, t_world_base=None, *, table_z: float = 0.0, tool_length_m: float = TOOL_LENGTH_M,
                 tip_past_tcp_m: float = TIP_PAST_TCP_M):
        self.t_world_base = np.eye(4) if t_world_base is None else np.asarray(t_world_base, float)
        self.table_z = float(table_z)
        self.tool_length_m = float(tool_length_m)
        self.tip_past_tcp_m = float(tip_past_tcp_m)
        joints = read_urdf_joints(URDF)
        self.joints = [joints[f"fr3_link{i}"] for i in range(1, 8)]
        self.flange_joint = joints["fr3_link8"]
        limits = _joint_limits(URDF)
        self.names = [joint.name for joint in self.joints]
        self.low = np.array([limits[name][0] for name in self.names])
        self.high = np.array([limits[name][1] for name in self.names])
        self.t_flange_tcp = HAND @ homogeneous(np.eye(3), (0.0, 0.0, self.tool_length_m)) @ TOOL_AXES

    def frames(self, q) -> list[np.ndarray]:
        """World frames of fr3_link1 .. fr3_link7 and the flange (fr3_link8)."""
        t = self.t_world_base
        out = []
        for joint, angle in zip(self.joints, q):
            t = t @ joint.origin @ _turn(joint.axis, float(angle))
            out.append(t)
        out.append(t @ self.flange_joint.origin)
        return out

    def flange(self, q) -> np.ndarray:
        return self.frames(q)[-1]

    def tcp(self, q) -> np.ndarray:
        return self.flange(q) @ self.t_flange_tcp

    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        """(the 6x7 world Jacobian of the TCP, the TCP's pose)."""
        frames = self.frames(q)
        tip = frames[-1] @ self.t_flange_tcp
        jac = np.zeros((6, 7))
        for i, (joint, frame) in enumerate(zip(self.joints, frames)):
            axis = frame[:3, :3] @ joint.axis
            jac[:3, i] = np.cross(axis, tip[:3, 3] - frame[:3, 3])
            jac[3:, i] = axis
        return jac, tip

    def within_limits(self, q, slack: float = 1e-6) -> bool:
        q = np.asarray(q, float)
        return bool(np.all(q >= self.low - slack) and np.all(q <= self.high + slack))

    def capsules(self, q) -> list[LinkCapsule]:
        f = self.frames(q)
        at = lambda frame, x=0.0, y=0.0, z=0.0: (frame @ np.array([x, y, z, 1.0]))[:3]  # noqa: E731
        hand = f[7] @ HAND
        base = self.t_world_base[:3, 3]
        return [
            LinkCapsule("shoulder", base, at(f[0]), 0.085),
            LinkCapsule("upper_arm", at(f[1]), at(f[2]), 0.07),
            LinkCapsule("elbow", at(f[2]), at(f[3]), 0.07),
            LinkCapsule("forearm", at(f[3]), at(f[4]), 0.06),
            LinkCapsule("forearm_end", at(f[4], z=-0.2), at(f[4]), 0.055),
            LinkCapsule("wrist", at(f[5]), at(f[6]), 0.055),
            LinkCapsule("flange", at(f[6]), at(f[7]), 0.05),
            LinkCapsule("hand", at(hand), at(hand, z=0.045), 0.045),
            LinkCapsule("hand_bar", at(hand, y=-0.08, z=0.03), at(hand, y=0.08, z=0.03), 0.03),
            LinkCapsule("fingers", at(hand, z=0.0584),
                        at(hand, z=self.tool_length_m + max(0.0, self.tip_past_tcp_m - FINGER_RADIUS_M)),
                        FINGER_RADIUS_M),
        ]

    def problems(self, q) -> list[str]:
        """What is wrong with joint angles ``q``: the joint limits, the table, the arm's own links."""
        if not self.within_limits(q):
            return ["a joint beyond its limits"]
        out = []
        capsules = {c.name: c for c in self.capsules(q)}
        for capsule in capsules.values():
            if capsule.name != "shoulder" and min(capsule.a[2], capsule.b[2]) - capsule.radius_m \
                    < self.table_z + (TIP_CLEARANCE_M if capsule.name == "fingers" else BODY_CLEARANCE_M):
                out.append(f"the {capsule.name.replace('_', ' ')} below the table top")
        for one, other in SELF_PAIRS:
            a, b = capsules[one], capsules[other]
            if segment_distance(a.a, a.b, b.a, b.b) < a.radius_m + b.radius_m + SELF_CLEARANCE_M:
                out.append(f"the {one.replace('_', ' ')} against the {other.replace('_', ' ')}")
        return out


def _joint_limits(path: Path) -> dict[str, tuple[float, float]]:
    import xml.etree.ElementTree as ET

    return {el.get("name"): (float(el.find("limit").get("lower")), float(el.find("limit").get("upper")))
            for el in ET.parse(path).getroot().findall("joint") if el.find("limit") is not None}
