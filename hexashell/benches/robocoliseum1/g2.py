"""The G2's arms: forward and inverse kinematics of each seven-joint arm, from ``arm_base_link`` to the
grasp point between the finger pads.

The numbers are those of genie_sim v3.5.0's robot description (``G2_omnipicker_from_usd.urdf`` of
``data_collection/config/robot_cfg/G2``; the arm chains are the same in ``utils/IK-SDK/G2_NO_GRIPPER.urdf``,
which the benchmark's IK uses): seven revolute joints per arm, ``arm_{l,r}_end_link`` 0.088 m past joint 7,
the gripper's base 5.37 mm along the end link's z, and the point between the pads (``gripper_*_center_link``)
143.08 mm farther, with the end link's orientation.  The fingers sit at +-19.5 mm on the gripper base's y,
which is the end link's x: the fingers close along the end link's x and point along its z.

``arm_base_link`` moves with the waist; where it is in ``base_link`` comes with every observation (each hand's
pose in both frames), so the arms are solved in ``arm_base_link`` and moved to ``base_link`` with that
transform (:func:`arm_base_from_obs`).

The hand's orientation is given in pitch, yaw and roll (:mod:`hexashell.benches.pose`: the frame x forward,
y left, z up, which is ``base_link``'s); the tool frame there has the approach along its x and the finger
line along its y, so ``R_tool = R_end @ TOOL_IN_END`` (columns: end z, end x, end y) for the right hand,
and that turned half a turn about the approach for the left (its end link is mounted mirrored: so both
hands start near roll 0, and their angles mirror each other).

The inverse kinematics are damped least squares with the joint limits and, for the seventh degree of
freedom, a pull toward the seed's joints (the arm moves as little as it can); a pose is solved from
several seeds and the solution nearest the seed that reaches it wins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ...geometry import homogeneous, invert, log_so3, rpy

ARMS = ("left", "right")
HALF_PI = 1.5707963267948966

#: (origin xyz, origin rpy, joint limits) of joints 1..7; every axis is the joint frame's z
_LEFT = (
    ((0.0, 0.0, 0.069), (0.0, 0.0, 0.0), (-3.1067, 3.1067)),
    ((0.0, 0.0, 0.1745), (HALF_PI, 0.0, 0.0), (-2.0944, 2.0944)),
    ((0.0, 0.0, 0.0), (-HALF_PI, 0.0, 0.0), (-3.1067, 3.1067)),
    ((0.018, 0.0, 0.287), (HALF_PI, 0.0, 0.0), (-2.5307, 1.0472)),
    ((-0.018, 0.0, 0.0), (-HALF_PI, 0.0, 0.0), (-3.1067, 3.1067)),
    ((0.0, 0.0, 0.314), (HALF_PI, 0.0, 0.0), (-1.0472, 1.0472)),
    ((0.0, 0.0, 0.0), (HALF_PI, 0.0, HALF_PI), (-1.5708, 1.5708)),
)
_RIGHT = (((0.0, 0.0, -0.069), (math.pi, 0.0, 0.0), (-3.1067, 3.1067)),) + _LEFT[1:]
#: joint 7 -> arm_*_end_link
_END = ((0.088, 0.0, 0.0), (math.pi, -HALF_PI, 0.0))
#: the point between the finger pads, along the end link's z (5.37 mm to the gripper base, 143.08 mm on)
GRASP_REACH = 0.00537 + 0.14308
#: the fingertips reach this far along the end link's z (10.3 mm past the point between the pads)
TIP_REACH = GRASP_REACH + 0.0103
#: tool frame (approach, finger line, approach x finger line) in the end link's axes: the right hand's;
#: the left hand's is turned half a turn about the approach, so that the two hands' angles mirror each
#: other (both start near roll 0: the left's end link is mounted mirrored)
TOOL_IN_END = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
TOOL_IN_END_OF = {"right": TOOL_IN_END, "left": TOOL_IN_END @ np.diag([1.0, -1.0, -1.0])}

#: joint ranges kept to (radians): the limits less a margin
LIMIT_MARGIN = 0.02


def _origin(xyz, r) -> np.ndarray:
    return homogeneous(rpy(*r), xyz)


@dataclass
class Solution:
    q: np.ndarray
    reached: bool
    pos_err_m: float
    rot_err_deg: float
    iterations: int = 0


class Arm:
    """One arm's chain in ``arm_base_link``."""

    def __init__(self, name: str):
        if name not in ARMS:
            raise ValueError(f"arm must be 'left' or 'right', not {name!r}")
        self.name = name
        spec = _LEFT if name == "left" else _RIGHT
        self.origins = [_origin(xyz, r) for xyz, r, _ in spec]
        self.lower = np.array([lim[0] for _, _, lim in spec]) + LIMIT_MARGIN
        self.upper = np.array([lim[1] for _, _, lim in spec]) - LIMIT_MARGIN
        self.end = _origin(*_END)
        self.grasp_offset = homogeneous(np.eye(3), (0.0, 0.0, GRASP_REACH))
        self.tool = self.end @ self.grasp_offset
        #: the shoulder (joint 1 turns about it, joints 2 and 3 meet there) and the farthest the grasp
        #: point gets from it (the arm straight): upper arm, forearm, wrist to the end link, the pads
        self.shoulder = self.frames(np.zeros(7))[1][:3, 3].copy()
        self.reach = (math.hypot(spec[3][0][0], spec[3][0][2]) + math.hypot(spec[4][0][0], spec[5][0][2])
                      + _END[0][0] + GRASP_REACH)

    # -- forward -------------------------------------------------------------------------------
    def frames(self, q) -> list[np.ndarray]:
        """The joint frames 1..7 (after each joint's turn) in arm_base_link."""
        q = np.asarray(q, float).reshape(7)
        t = np.eye(4)
        out = []
        turn = np.eye(4)
        for k in range(7):
            c, s = math.cos(q[k]), math.sin(q[k])
            turn[0, 0], turn[0, 1], turn[1, 0], turn[1, 1] = c, -s, s, c
            t = t @ self.origins[k] @ turn
            out.append(t)
        return out

    def end_pose(self, q) -> np.ndarray:
        """arm_*_end_link in arm_base_link (the pose the benchmark reports as ``end_pose``)."""
        return self.frames(q)[-1] @ self.end

    def grasp_pose(self, q) -> np.ndarray:
        """The point between the pads, with the end link's orientation, in arm_base_link."""
        return self.frames(q)[-1] @ self.tool

    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        """(6x7 geometric Jacobian of the grasp point in arm_base_link: linear rows first, the grasp pose)."""
        fr = self.frames(q)
        tip = fr[-1] @ self.tool
        p = tip[:3, 3]
        jac = np.zeros((6, 7))
        for k, f in enumerate(fr):
            z = f[:3, 2]
            jac[:3, k] = np.cross(z, p - f[:3, 3])
            jac[3:, k] = z
        return jac, tip

    # -- inverse -------------------------------------------------------------------------------
    def solve(self, target: np.ndarray, seed, *, iterations: int = 200, pos_tol: float = 5e-4,
              rot_tol_deg: float = 0.3, rot_weight: float = 0.3, null_gain: float = 0.05,
              rest=None) -> Solution:
        """Joints that put the grasp point at ``target`` (4x4 in arm_base_link), from ``seed``; the
        redundant freedom leans toward ``rest`` (default: the seed).

        A joint at a limit that the step would push further is left out of that step (the others take
        over its share); the best joints met are returned when none reaches."""
        q = np.clip(np.asarray(seed, float).reshape(7), self.lower, self.upper)
        rest = q.copy() if rest is None else np.clip(np.asarray(rest, float).reshape(7), self.lower, self.upper)
        target = np.asarray(target, float)
        lam = 0.01
        mid = 0.5 * (self.lower + self.upper)
        span = self.upper - self.lower
        best = None
        err_pos = err_rot = float("inf")
        for it in range(1, iterations + 1):
            jac, tip = self.jacobian(q)
            dp = target[:3, 3] - tip[:3, 3]
            dr = tip[:3, :3] @ log_so3(tip[:3, :3].T @ target[:3, :3])
            err_pos, err_rot = float(np.linalg.norm(dp)), math.degrees(float(np.linalg.norm(dr)))
            if err_pos < pos_tol and err_rot < rot_tol_deg:
                return Solution(q, True, err_pos, err_rot, it)
            score = err_pos + 0.001 * err_rot
            if best is None or score < best[0]:
                best = (score, q.copy(), err_pos, err_rot)
            # cap the step so the linearisation holds
            scale = min(1.0, 0.05 / max(err_pos, 1e-9))
            e = np.r_[dp * scale, rot_weight * dr * min(1.0, 0.3 / max(np.linalg.norm(dr), 1e-9))]
            jw = jac.copy()
            jw[3:] *= rot_weight
            # the redundant direction: toward the seed, away from the limits
            pull = (rest - q) + 0.2 * (mid - q) * ((2.0 * np.abs(q - mid) / span) > 0.8)
            at_lo = q <= self.lower + 1e-4
            at_hi = q >= self.upper - 1e-4
            free = np.ones(7, bool)
            dq = np.zeros(7)
            for _ in range(4):
                jf = jw[:, free]
                inv = np.linalg.inv(jf @ jf.T + (lam ** 2) * np.eye(6))
                pinv = jf.T @ inv
                step = pinv @ e + null_gain * ((np.eye(int(free.sum())) - pinv @ jf) @ pull[free])
                dq = np.zeros(7)
                dq[free] = step
                push = free & ((at_lo & (dq < 0)) | (at_hi & (dq > 0)))
                if not push.any():
                    break
                free &= ~push
            big = float(np.max(np.abs(dq)))
            if big > 0.2:
                dq *= 0.2 / big
            q = np.clip(q + dq, self.lower, self.upper)
        _, q, err_pos, err_rot = best
        return Solution(q, err_pos < pos_tol * 4 and err_rot < rot_tol_deg * 4, err_pos, err_rot, iterations)

    def ik(self, target: np.ndarray, seeds, **kw) -> Solution:
        """The best of :meth:`solve` from each seed: one that reaches, nearest the first seed (the joints now)."""
        seeds = [np.asarray(s, float).reshape(7) for s in seeds]
        best = None
        for s in seeds:
            sol = self.solve(target, s, **kw)
            if sol.reached:
                travel = float(np.max(np.abs(sol.q - seeds[0])))
                key = (0, travel)
            else:
                key = (1, sol.pos_err_m + 0.01 * sol.rot_err_deg)
            if best is None or key < best[0]:
                best = (key, sol)
            if sol.reached and key[1] < 0.3:
                break
        return best[1]

    def beyond_reach(self, point, margin: float = 0.005) -> bool:
        """Whether a point (arm_base_link) is farther from the shoulder than the grasp point ever gets."""
        return float(np.linalg.norm(np.asarray(point, float)[:3] - self.shoulder)) > self.reach + margin

    def within(self, q, margin: float = 0.0) -> bool:
        q = np.asarray(q, float)
        return bool(np.all(q >= self.lower - LIMIT_MARGIN + margin) and np.all(q <= self.upper + LIMIT_MARGIN - margin))


def pose_of(position, quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return homogeneous(r, position)


def arm_base_from_obs(end_pose: dict) -> np.ndarray:
    """``T_base_link <- arm_base_link`` from an observation's ``end_pose`` (each hand in both frames): it
    depends on the waist only, so the left hand's two poses give it."""
    a = end_pose["base_link"]["left_arm"]
    b = end_pose["arm_base_link"]["left_arm"]
    return pose_of(a["position"], a["orientation"]) @ invert(pose_of(b["position"], b["orientation"]))


def tool_rotation(end_rotation: np.ndarray, arm: str) -> np.ndarray:
    """The tool frame of :mod:`hexashell.benches.pose` (approach, finger line, ...) of an arm's end link
    rotation."""
    return np.asarray(end_rotation, float) @ TOOL_IN_END_OF[arm]


def end_rotation(tool_rot: np.ndarray, arm: str) -> np.ndarray:
    return np.asarray(tool_rot, float) @ TOOL_IN_END_OF[arm].T


ARM_MODELS = {name: Arm(name) for name in ARMS}
