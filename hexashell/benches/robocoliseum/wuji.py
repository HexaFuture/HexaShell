"""The Wuji hands of the G2_crsB_wuji (the dexmanip board): five fingers of four joints each, every joint driven
on its own (no coupling), from the robot's URDF (genie_sim 3.6.0 ``app/robot_cfg/G2_crsB_wuji``, read as data).

Wire: per hand 20 values, thumb, index, middle, ring, pinky, each joint 1..4, in radians: the joints' absolute
position targets, passed through unchanged (no offset, scale, mirror or limit on the way) and played as they are
in every row.  The readings come back in the same order.  The drives are stiff (a joint follows its target with
about 0.1 s of lag) and their torque is capped at 10 N m: a finger stopped by something squeezes it with that
much and stays short of its target.  The fingers do not collide with each other or the palm.

Joints (the right hand in its end link's axes: the fingers along +z, the thumb on the +x side, the palm facing
-y; the end link is ``arm_r_end_link``, the one the arm's kinematics end at):

- index to pinky: joint 1 bends the finger at the knuckle toward the palm (+), joint 2 spreads it (+ toward the
  thumb on the right hand, - on the left), joints 3 and 4 bend its middle and last joints (+ curls);
- thumb: joint 1 swings it across in front of the palm, joint 2 moves it off the palm (- out and down, + back
  toward the palm's plane), joints 3 and 4 bend it.

Mirror: the left hand at joints q is the right hand at q' with every point's y (end link) negated, q' being q with
the four fingers' joint 2 negated (the URDF chains agree to 0.02 mm at the pads).  The grasp shapes below are the
right hand's; the left's follow by that rule.

Shapes (:data:`SHAPES`): each a working point (the arm's grasp point while the hand is in that shape), the line it
closes along, and its joints open (opening 1) and closed (0); the joints in between are linear in the opening
(the pinch's gap is then 3.0 cm times the opening, within 1 mm):

- ``flat``: every finger straight, the thumb beside the index (its joint 1 at its lower limit); the working point
  is the middle of the palm's face (it pushes and holds with it);
- ``pinch``: the thumb's pad against the index finger's pad (the two pads' faces 48 degrees apart, each 24 off
  the line they close along), the other three fingers half curled out of the way; the working point is where the
  pads meet, 1.15 cm in front of the palm's face;
- ``wrap``: the four fingers curl around a bar lying across the palm (along the end link's x), the thumb along
  the index finger's side; opening 1 fits a bar about 10 cm thick, 0 is a fist (bars of 3 cm and thicker are
  held in between); the working point is a 6 cm bar's axis;
- ``point``: the index finger straight, the other fingers curled (the thumb tucked over them); the working point
  is the index fingertip.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ...geometry import homogeneous, rpy

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
SIDES = ("left", "right")
#: the index of each finger's first joint among a hand's 20
FIRST = {f: 4 * i for i, f in enumerate(FINGERS)}
#: the four fingers' spreading joints (joint 2), negated by the mirror
SPREAD = (5, 9, 13, 17)

# -- the URDF chains (m, rad): hand base in the end link, then per finger joints 1..4 (origin xyz, rpy in the frame
# before; every axis y except the left thumb's joint 2, -y) and the pad link on link 4 -------------------------------
_BASE = {"left": ((0.0, 0.0, 0.0341), (0.0, 0.0, math.pi / 2)),
         "right": ((0.0, 0.0, 0.0341), (0.0, 0.0, -math.pi / 2))}
_CHAINS = {
    "left": {
        "thumb": (((0.0084399, -0.020587, 0.028893), (1.328, -0.37161, 0.7307)),
                  ((-0.0047549, 0.0058191, 0.014132), (0.57428, -0.23677, -1.8034)),
                  ((0.00026671, 0.0009915, 0.034666), (-0.049049, 0.017311, 1.9085)),
                  ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.00105, 0.0002, 0.0313)),
        "index": (((-0.0058701, -0.029434, 0.091917), (0.11071, -0.071139, 0.21822)),
                  ((0.0022, 0.0, 0.004000006), (0.0, 0.0, 1.570796)),
                  ((0.0, 0.004749999, 0.04361769), (0.0, 0.0, -1.570528)),
                  ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.0010521, 0.0002108692, 0.0267)),
        "middle": (((-0.01046, -0.0074615, 0.089544), (5.0894e-05, -0.087266, -0.00058387)),
                   ((0.0022, 0.0, 0.004000006), (0.0, 0.0, 1.570796)),
                   ((0.0, 0.004749999, 0.04361769), (0.0, 0.0, -1.570528)),
                   ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                   (-0.0010504, 0.0002026723, 0.0267)),
        "ring": (((-0.0076649, 0.013712, 0.084429), (-0.090375, -0.083436, -0.047127)),
                 ((0.0022, 0.0, 0.004000005), (0.0, 0.0, 1.570796)),
                 ((0.0, 0.004749999, 0.04361769), (0.0, 0.0, -1.570528)),
                 ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                 (-0.001049983, 0.0002025572, 0.0267)),
        "pinky": (((-0.002268651, 0.03400111, 0.07428134), (-0.1924837, -0.09251723, -0.1576728)),
                  ((0.002200001, 0.0, 0.004000005), (0.0, 0.0, 1.570796)),
                  ((0.0, 0.00475, 0.04361769), (0.0, 0.0, -1.570528)),
                  ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.001050407, 0.0002010472, 0.0267)),
    },
    "right": {
        "thumb": (((0.0084399, 0.020587, 0.028893), (-1.328, -0.37161, -0.7307)),
                  ((-0.0047549, -0.0058191, 0.014132), (0.57428, 0.23677, -1.3382)),
                  ((-0.000266705, 0.0009915016, 0.03466562), (0.04904945, 0.01731124, 1.233052)),
                  ((0.0, 0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.00105, -0.0002, 0.0313)),
        "index": (((-0.0058701, 0.029434, 0.091917), (-0.11071, -0.071139, -0.21822)),
                  ((0.0022, 0.0, 0.004000006), (0.0, 0.0, 1.570796)),
                  ((0.0, 0.004750282, 0.04361766), (0.0, 0.0, -1.571065)),
                  ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.00105, 0.0002, 0.0267)),
        "middle": (((-0.01046, 0.0074615, 0.089544), (-5.0886e-05, -0.087266, 0.00058387)),
                   ((0.0022, 0.0, 0.004000006), (0.0, 0.0, 1.570796)),
                   ((0.0, 0.00475, 0.04361769), (0.0, 0.0, -1.571065)),
                   ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                   (-0.001049087, 0.0002010395, 0.0267)),
        "ring": (((-0.0076649, -0.013712, 0.084429), (0.090375, -0.083436, 0.047127)),
                 ((0.002200001, 0.0, 0.004000005), (0.0, 0.0, 1.570796)),
                 ((0.0, 0.004749999, 0.04361769), (0.0, 0.0, -1.571065)),
                 ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                 (-0.00105961, 0.0001904508, 0.0267)),
        "pinky": (((-0.002268651, -0.03400111, 0.07428134), (0.1924837, -0.09251723, 0.1576728)),
                  ((0.002200001, 0.0, 0.004000005), (0.0, 0.0, 1.570796)),
                  ((0.0, 0.004749999, 0.04361769), (0.0, 0.0, -1.571065)),
                  ((0.0, -0.0002, 0.0295), (0.0, 0.0, 0.0)),
                  (-0.001044639, 0.0002113095, 0.0267)),
    },
}
#: the joints' ranges (rad), the same on both hands (joint 2 of the four fingers is symmetric)
LIMITS = np.array([
    (0.0475, 1.6033), (-0.1387, 0.9324), (-0.4642, 1.5623), (-0.4699, 1.5568),
    (-0.1585, 1.5604), (-0.37, 0.37), (-0.4777, 1.5485), (-0.4683, 1.5753),
    (-0.1644, 1.5516), (-0.37, 0.37), (-0.4739, 1.5512), (-0.4684, 1.5745),
    (-0.1554, 1.5585), (-0.37, 0.37), (-0.4765, 1.5487), (-0.4777, 1.5634),
    (-0.1626, 1.5585), (-0.37, 0.37), (-0.4768, 1.5490), (-0.4683, 1.5735),
])
#: targets are kept this far (rad) inside the ranges
LIMIT_MARGIN = 0.005
#: the palmar face of each finger's last link (centre, outward normal) in that link's frame (m), from its mesh
_PATCH = {"thumb": ((0.01185, 0.0, 0.012), (0.998, 0.0, -0.059)),
          "index": ((0.00907, 0.0, 0.010), (0.965, 0.0, 0.262)),
          "middle": ((0.00907, 0.0, 0.010), (0.965, 0.0, 0.262)),
          "ring": ((0.00902, 0.0, 0.010), (0.965, 0.0, 0.262)),
          "pinky": ((0.00967, 0.0, 0.010), (0.997, 0.0, 0.083))}
#: the middle of the palm's face in the end link (m; the right hand: the palm faces -y) and the left's
PALM = {"right": (0.0, -0.0252, 0.0791), "left": (0.0, 0.0233, 0.0791)}

_ORIGINS = {side: {f: [homogeneous(rpy(*r), xyz) for xyz, r in chain[:4]] + [homogeneous(np.eye(3), chain[4])]
                   for f, chain in fingers.items()}
            for side, fingers in _CHAINS.items()}
_BASES = {side: homogeneous(rpy(*r), xyz) for side, (xyz, r) in _BASE.items()}


def _turn_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    m = np.eye(4)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = c, s, -s, c
    return m


def mirror_q(q) -> np.ndarray:
    """The other hand's joints for the mirrored pose (the four fingers' joint 2 negated)."""
    out = np.array(q, float).reshape(20)
    out[list(SPREAD)] *= -1.0
    return out


def clamp(q) -> np.ndarray:
    """Joint targets within the ranges (less LIMIT_MARGIN)."""
    return np.clip(np.asarray(q, float).reshape(20), LIMITS[:, 0] + LIMIT_MARGIN, LIMITS[:, 1] - LIMIT_MARGIN)


def finger_frames(side: str, q) -> dict[str, list[np.ndarray]]:
    """Per finger, its frames in the end link: after each of joints 1..4 turned, then its pad (the fingertip)."""
    q = np.asarray(q, float).reshape(20)
    out = {}
    for i, f in enumerate(FINGERS):
        t = _BASES[side]
        frames = []
        for k in range(4):
            angle = q[4 * i + k]
            if side == "left" and f == "thumb" and k == 1:
                angle = -angle  # the left thumb's joint 2 turns about -y
            t = t @ _ORIGINS[side][f][k] @ _turn_y(angle)
            frames.append(t)
        frames.append(t @ _ORIGINS[side][f][4])
        out[f] = frames
    return out


def points(side: str, q) -> dict[str, np.ndarray]:
    """End-link points (m): each finger's pad (fingertip), the index finger's last link (the benchmark's
    'gripper' point of a hand), the palm's middle."""
    fr = finger_frames(side, q)
    out = {f: fr[f][4][:3, 3].copy() for f in FINGERS}
    out["index_link4"] = fr["index"][3][:3, 3].copy()
    out["palm"] = np.array(PALM[side], float)
    return out


def patch(side: str, q, finger: str) -> tuple[np.ndarray, np.ndarray]:
    """A finger's palmar pad face (centre, outward normal) in the end link."""
    t = finger_frames(side, q)[finger][3]
    c, n = _PATCH[finger]
    return (t @ np.r_[c, 1.0])[:3], t[:3, :3] @ np.asarray(n, float)


def pinch_gap(side: str, q) -> float:
    """The distance (m) between the thumb's and the index finger's pad faces."""
    a, _ = patch(side, q, "thumb")
    b, _ = patch(side, q, "index")
    return float(np.linalg.norm(a - b))


def skeleton(side: str, q) -> list[tuple[np.ndarray, np.ndarray]]:
    """Line segments (end link, m) along each finger, from its first joint to its fingertip: for masks."""
    fr = finger_frames(side, q)
    segs = []
    for f in FINGERS:
        pts = [fr[f][0][:3, 3]] + [fr[f][k][:3, 3] for k in (2, 3)] + [fr[f][4][:3, 3]]
        segs += [(pts[k], pts[k + 1]) for k in range(len(pts) - 1)]
    return segs


# -- shapes ---------------------------------------------------------------------------------------------
def _q(**fingers) -> tuple:
    q = np.zeros(20)
    for f, v in fingers.items():
        q[FIRST[f]:FIRST[f] + 4] = v
    return tuple(float(x) for x in q)


FIST = (1.45, 0.0, 1.40, 1.00)
HALF_FIST = tuple(0.5 * v for v in FIST)
#: the thumb tucked over the curled fingers (the benchmark's own fist, ``ARITHMETIC_STATES``, right hand)
THUMB_TUCKED = (1.4696, 0.2615, 1.2836, 1.4121)
#: the thumb along the index finger's side (the wrap's: the cylinder grasps of 6-10 cm)
THUMB_ALONG = (0.0475, 0.9324, 1.34, -0.29)


@dataclass(frozen=True)
class Shape:
    """A grasp shape of the right hand: its joints open (opening 1) and closed (0); its working point and the line
    it closes along (end link, m; the left's mirrored); the fingers that hold in it (those whose contact counts as
    a grasp); the gap at opening 1 (m, None: no gap to speak of)."""

    name: str
    open_q: tuple
    closed_q: tuple
    point_mm: tuple
    closes_along: tuple
    holders: tuple = ()
    open_gap_m: float | None = None

    @property
    def fixed(self) -> bool:
        return self.open_q == self.closed_q

    def joints(self, side: str, opening: float) -> np.ndarray:
        g = min(max(float(opening), 0.0), 1.0)
        q = np.asarray(self.closed_q, float) + g * (np.asarray(self.open_q, float) - np.asarray(self.closed_q, float))
        q = clamp(q)
        return mirror_q(q) if side == "left" else q

    def point(self, side: str) -> np.ndarray:
        p = np.asarray(self.point_mm, float) / 1000.0
        return p * np.array([1.0, -1.0, 1.0]) if side == "left" else p

    def tool_in_end(self, side: str) -> np.ndarray:
        """The tool frame (columns: the approach = the end link's z, the closing line's part across it, their cross
        product) in the end link's axes; the left's is the right's mirrored with its y turned round, so that both
        hands share the meaning of pitch, yaw and roll (as the ctek90d's do)."""
        x = np.array([0.0, 0.0, 1.0])
        c = np.asarray(self.closes_along, float)
        y = c - np.dot(c, x) * x
        y /= np.linalg.norm(y)
        m = np.column_stack([x, y, np.cross(x, y)])
        if side == "left":
            m = np.diag([1.0, -1.0, 1.0]) @ m @ np.diag([1.0, -1.0, 1.0])
        return m


#: the palm's normal (right hand), the line flat, wrap and point "close" along
_PALM_NORMAL = (0.0, -1.0, 0.0)
FLAT = Shape("flat", _q(thumb=(0.0475, 0.0, 0.0, 0.0)), _q(thumb=(0.0475, 0.0, 0.0, 0.0)),
             (0.0, -25.2, 79.1), _PALM_NORMAL)
#: the pulp pinch (thumb pad to index pad): the gap 30.2 mm x opening; the pads' line thumb -> index
PINCH = Shape("pinch",
              _q(thumb=(0.3102, 0.9324, 0.8627, -0.2223), index=(1.2026, -0.3043, 0.8936, 1.1558), middle=HALF_FIST,
                 ring=HALF_FIST, pinky=HALF_FIST),
              _q(thumb=(0.6307, 0.6590, 1.0817, -0.2779), index=(1.0102, -0.2170, 1.3631, 1.3443), middle=HALF_FIST,
                 ring=HALF_FIST, pinky=HALF_FIST),
              (12.0, -36.7, 132.5), (-0.665, -0.640, -0.385), ("thumb", "index"), 0.030)
#: the fingers at half (about a 10 cm bar) to a full fist; the thumb along the index finger
WRAP = Shape("wrap",
             _q(thumb=THUMB_ALONG, index=HALF_FIST, middle=HALF_FIST, ring=HALF_FIST, pinky=HALF_FIST),
             _q(thumb=THUMB_ALONG, index=FIST, middle=FIST, ring=FIST, pinky=FIST),
             (0.0, -52.6, 94.1), _PALM_NORMAL, ("index", "middle", "ring", "pinky"), 0.10)
POINT = Shape("point", _q(thumb=THUMB_TUCKED, middle=FIST, ring=FIST, pinky=FIST),
              _q(thumb=THUMB_TUCKED, middle=FIST, ring=FIST, pinky=FIST), (42.9, 21.6, 227.3), _PALM_NORMAL)
SHAPES = {s.name: s for s in (FLAT, PINCH, WRAP, POINT)}
#: the shape the hands start in (the benchmark starts them about flat)
START_SHAPE = "flat"
#: names the model may use for a shape
SHAPE_WORDS = {"open": "flat", "flat": "flat", "pinch": "pinch", "wrap": "wrap", "fist": "wrap", "point": "point"}


# -- the hand cameras -------------------------------------------------------------------------------------
#: each hand's camera (the robot's USD and robot config, read as data): an OpenCV fisheye of
#: 1920 x 1536 (the requests mark its calibration unavailable: "unsupported lens model"), its picture a disc of
#: about 76 degrees' half angle; fx, fy, cx, cy and k1..k4 at that size
CAMERA_SIZE = (1920, 1536)
CAMERA_K = (726.2366, 726.33655, 965.7987, 745.5315)
CAMERA_D = (-0.0024797623, -6.388e-7, -0.0013551259, 0.00023899999)
#: per hand, the camera's optical frame (x right, y down, z forward) in the end link: its centre (m) and its
#: rotation's rows; it sits at the heel of the palm on the little finger's side, about 2 cm proud of the palm,
#: looking along the fingers
CAMERA_MOUNT = {
    "right": ((-0.0220, -0.0473, 0.0298),
              ((0.4833, -0.84883, 0.21425), (0.87545, 0.4686, -0.11828), (0.0, 0.24473, 0.96959))),
    "left": ((-0.0220, 0.0459, 0.0300),
             ((-0.49603, -0.85988, 0.12068), (0.86611, -0.48009, 0.13917), (-0.06173, 0.17356, 0.98289))),
}
#: the pinhole camera the pictures are undistorted into: its size and focal length (px)
PINHOLE_SIZE = (800, 640)
PINHOLE_F = 260.0


def camera_mount(side: str) -> np.ndarray:
    """The hand camera's optical frame in its end link (4x4; the rotation made orthonormal)."""
    centre, rows = CAMERA_MOUNT[side]
    u, _, vt = np.linalg.svd(np.asarray(rows, float))
    return homogeneous(u @ vt, np.asarray(centre, float))


def shape_of(name) -> Shape:
    key = SHAPE_WORDS.get(str(name).lower()) if isinstance(name, str) else None
    if key is None:
        raise ValueError(f"a hand shape is one of {', '.join(repr(s) for s in SHAPES)}, not {name!r}")
    return SHAPES[key]


def finger_joints(finger: str, value, side: str) -> np.ndarray:
    """One finger's four joint targets (rad) from what the model gives: a curl 0..1 (0 straight, 1 curled: the
    fist's joints, the thumb tucked over the fingers) or four joint angles in degrees."""
    if finger not in FINGERS:
        raise ValueError(f"fingers are {', '.join(FINGERS)}, not {finger!r}")
    if isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool):
        c = float(value)
        if not 0.0 <= c <= 1.0:
            raise ValueError(f"a finger's curl is 0 (straight) .. 1 (curled), not {value!r}")
        curled = np.array(THUMB_TUCKED if finger == "thumb" else FIST)
        straight = np.array((LIMITS[0, 0], 0.0, 0.0, 0.0)) if finger == "thumb" else np.zeros(4)
        q = straight + c * (curled - straight)
    else:
        try:
            q = np.radians(np.asarray(value, float).reshape(4))
        except (TypeError, ValueError):
            raise ValueError(f"a finger is a curl 0..1 or its four joint angles in degrees, not {value!r}") from None
        if side == "left" and finger != "thumb":
            q[1] = -q[1]  # the spread given as on the right hand: + toward the thumb
    return q


__all__ = ["FINGERS", "SHAPES", "Shape", "camera_mount", "clamp", "finger_frames", "finger_joints", "mirror_q", "pinch_gap", "points",
           "shape_of", "skeleton"]
