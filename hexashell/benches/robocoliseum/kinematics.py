"""The robots' arms in Season II: forward and inverse kinematics of each arm, from ``arm_base_link`` to the
grasp point between the fingers, for the arm builds and grippers of the benchmark's configurations (genie_sim
3.6.0, ``app/robot_cfg``; the chains are the URDFs' joints, read as data).

Arm builds (revolute joints: each turns about its axis in its own frame, which sits at its origin in the frame
before it, after the fixed joints between them):

- ``G2`` (``G2_omnipicker``, the crsemb board): seven joints per arm, every axis z; Season I's numbers
  (``G2_omnipicker_from_usd.urdf``);
- ``G2_crsB`` (``G2_crsB_90D/G2_crsB_90D.urdf``; the Wuji build is the same arm): the elbow offset 10.5 mm
  (G2: 18 mm), the upper arm 0.332 m (0.287), the forearm 0.278 m (0.314), joint 7 to the end link 0.09 m
  (0.088), and narrower joint ranges;
- ``G1`` (``G1_omnipicker/G1_omnipicker.urdf``): seven joints per arm, every axis z, the upper arm 0.305 m, the
  forearm 0.1975 m, the wrist 0.1805 m; ``arm_base_link`` rides on the body's lift and pitch;
- ``ARX`` (``arx/arx.urdf``): six joints per arm, axes z, y, -y, -y, -z, x, mounted on ``base_link`` (the
  benchmark's ``arm_base_link`` is ``base_link`` there);
- ``Agilex`` (``agilex/agilex.urdf``): six joints per arm, every axis z, the shoulders on a frame 0.775 m above
  ``arm_base_link`` (which is 0.15 m below ``base_link``).

The forward kinematics reproduce the reported ``end_pose`` (``arm_*_end_link`` in ``arm_base_link``) to
0.00 mm / 0.00 degrees on all five.

Hands (each the same on both arms of a robot; the approach, the line the fingers close along, in the end
link's axes):

- ``omnipicker`` on the G2: the point between the pads 148.45 mm along the end link's z; the fingers close
  along its x; wire value -0.785 open .. 0 closed (the reading the same);
- ``omnipicker`` on the G1: the same gripper straight on the end link (no 5.37 mm spacer), turned a quarter
  turn about z: the point between the pads 143.08 mm along z, the fingers along x;
- ``ctek90d`` (finemanip, WBC): a parallel gripper on a four-bar linkage.  Its base sits 7 mm along end z,
  turned a quarter turn about it, so the fingers close along the end link's **y**.  Each finger hangs on an
  outer link of 55 mm pivoting at (|y| 30.1 mm, z 111.2 mm), 47.3 degrees off the approach when open; the
  wire value is that joint's angle (0 open .. -0.910 closed, the joint's limit -52.14 degrees; a positive
  value is clamped to 0).  Closing swings the fingers inward and 17.5 mm forward: open, the pads face each
  other 89.6 mm apart over z 143.5..212 mm; closed they meet over z 161..229.5 mm.  The grasp point is
  taken 190 mm along end z (the pads' middle at a 40 mm grip).  An empty close reads about -0.906;
- ``arx_gripper``: two fingers sliding apart along end y, 0..44 mm each (the simulator's travel; the wire
  value is -3.5 open .. 0 closed, ``name_utils``'s scale), the fingertips about 155 mm along end **x**; the
  grasp point 130 mm along x (the pads' middle; the benchmark's ``gripper_*_center`` is at 150 mm);
- ``agilex_gripper``: two fingers sliding apart along end x, 0..40 mm each (their faces meet at 0), 71 mm
  long, the fingertips 140 mm along end z (the benchmark's ``gripper_*_center_link``); wire value -3.5 open
  .. 0 closed; the grasp point 115 mm along z (the pads' middle);
- ``wuji`` (dexmanip): five-finger hands of 20 joints (:mod:`.wuji`); the grasp point is the working point of the
  hand's shape (the palm's middle when flat, where the pads meet in a pinch, ...: ``Hand.tcp_in_end``, off the
  approach), and the tool frame's y is the line that shape closes along; one :class:`Hand` per shape
  (:data:`WUJI_HANDS`), the arm's model changing with the shape (``arm_models(robot, shape)``).

The hand's orientation is given in pitch, yaw and roll (:mod:`hexashell.benches.pose`: x forward, y left,
z up, ``base_link``'s axes); the tool frame has the approach along its x and the finger line along its y, so
``R_tool = R_end @ tool_in_end`` (columns: the approach, the finger line, their cross product).  The left
hand's tool frame is the right's turned half a turn about the approach where that makes the two hands start
near roll 0 (``Hand.left_flip``: the G2 omnipicker's left end link is mounted mirrored; the others are not).

``arm_base_link`` moves with the waist (G2: its turn; G1: its lift and pitch); where it is in ``base_link``
comes with every observation (each hand's pose in both frames), so the arms are solved in ``arm_base_link``
and moved to ``base_link`` with that transform (:func:`arm_base_from_obs`).

The inverse kinematics are damped least squares with the joint limits and, for a seventh degree of freedom,
a pull toward the seed's joints (the arm moves as little as it can); a pose is solved from several seeds and
the solution nearest the seed that reaches it wins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ...geometry import homogeneous, invert, log_so3, rpy
from . import wuji

ARMS = ("left", "right")
HALF_PI = 1.5707963267948966
PI = math.pi
Z_AXIS = (0.0, 0.0, 1.0)
NO_TURN = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Joint:
    """A revolute joint: where its frame sits in the frame before it ((xyz, rpy) pieces composed in order: the
    fixed joints before it, then its own origin), the axis it turns about in its own frame, its range (rad)."""

    origin: tuple
    axis: tuple
    limits: tuple


@dataclass(frozen=True)
class Chain:
    """An arm from ``arm_base_link``: its joints, and from the last joint's frame to the end link ((xyz, rpy)
    pieces)."""

    joints: tuple
    end: tuple = ()


@dataclass(frozen=True)
class Build:
    name: str
    left: Chain
    right: Chain
    #: whether the way from arm_base_link's origin to the first joint is the robot's own body (the ARX's and
    #: the Agilex's arms stand on mounts of their own, away from it)
    body_to_first_joint: bool = True
    #: the joints' velocity limits (rad/s, joints 1..n) and the time constant of their position drives (s:
    #: damping / stiffness), from the robots' USD as the simulator plays them: a joint follows its
    #: target about like dq/dt = (target - q) / tau, never faster than its limit
    velocity: tuple = ()
    tau_s: float = 0.1


def _g2_chain(elbow: float, upper: float, forearm: float, end_x: float, limits, *, right: bool) -> Chain:
    """A G2 arm: joints 1..7, every axis z; the right arm hangs from arm_base_link the other way (joint 1 at
    z -0.069, turned half a turn about x)."""
    l1, l2, l3, l4, l5, l6, l7 = limits
    first = ((0.0, 0.0, -0.069), (PI, 0.0, 0.0)) if right else ((0.0, 0.0, 0.069), NO_TURN)
    return Chain((
        Joint((first,), Z_AXIS, l1),
        Joint((((0.0, 0.0, 0.1745), (HALF_PI, 0.0, 0.0)),), Z_AXIS, l2),
        Joint((((0.0, 0.0, 0.0), (-HALF_PI, 0.0, 0.0)),), Z_AXIS, l3),
        Joint((((elbow, 0.0, upper), (HALF_PI, 0.0, 0.0)),), Z_AXIS, l4),
        Joint((((-elbow, 0.0, 0.0), (-HALF_PI, 0.0, 0.0)),), Z_AXIS, l5),
        Joint((((0.0, 0.0, forearm), (HALF_PI, 0.0, 0.0)),), Z_AXIS, l6),
        Joint((((0.0, 0.0, 0.0), (HALF_PI, 0.0, HALF_PI)),), Z_AXIS, l7),
    ), (((end_x, 0.0, 0.0), (PI, -HALF_PI, 0.0)),))


def _g2_build(name: str, elbow: float, upper: float, forearm: float, end_x: float, limits, velocity) -> Build:
    return Build(name, _g2_chain(elbow, upper, forearm, end_x, limits, right=False),
                 _g2_chain(elbow, upper, forearm, end_x, limits, right=True), velocity=velocity, tau_s=0.1)


def _g1_chain(*, right: bool) -> Chain:
    """A G1 arm (``G1_omnipicker.urdf``: ``idx11/12_arm_*_base_joint``, then joints 1..7)."""
    if right:
        base = (((0.0, -0.025, 0.0), (-HALF_PI, 0.0, PI)), ((0.0, 0.0, 0.188), NO_TURN))
        second, second_limits = ((0.0, 0.0, 0.0), (-HALF_PI, 0.0, -HALF_PI)), (-1.48, 2.09)
        third = ((0.0, -0.305, 0.0), (-HALF_PI, -HALF_PI, PI))
    else:
        base = (((0.0, 0.025, 0.0), (-HALF_PI, 0.0, 0.0)), ((0.0, 0.0, 0.1859), NO_TURN))
        second, second_limits = ((0.0, 0.0, 0.0), (-HALF_PI, 0.0, HALF_PI)), (-2.09, 1.48)
        third = ((0.0, -0.305, 0.0), (-HALF_PI, HALF_PI, PI))
    return Chain((
        Joint(base, Z_AXIS, (-3.14, 3.14)),
        Joint((second,), Z_AXIS, second_limits),
        Joint((third,), Z_AXIS, (-3.1, 3.1)),
        Joint((((0.0, 0.0, 0.0), (-HALF_PI, 0.0, PI)),), Z_AXIS, (-1.48, 1.48)),
        Joint((((0.0, -0.1975, 0.0), (-HALF_PI, 0.0, -PI)),), Z_AXIS, (-3.1, 3.1)),
        Joint((((0.0, 0.0, 0.0), (-HALF_PI, 0.0, 0.0)),), Z_AXIS, (-1.74, 1.74)),
        Joint((((0.0, -0.1805, 0.0), (HALF_PI, 0.0, 0.0)),), Z_AXIS, (-3.1, 3.1)),
    ))


def _arx_chain(*, right: bool) -> Chain:
    """An ARX arm (``arx.urdf``: joints 1..6 on base_link; the end link sits on joint 6)."""
    y = -0.25 if right else 0.25
    return Chain((
        Joint((((-0.01602, y, 0.0215), NO_TURN),), (0.0, 0.0, 1.0), (-3.14159, 3.14159)),
        Joint((((0.02, 0.0, 0.04), NO_TURN),), (0.0, 1.0, 0.0), (-0.05, 3.5)),
        Joint((((-0.264, 3e-05, 0.0), NO_TURN),), (0.0, -1.0, 0.0), (0.0, 3.2)),
        Joint((((0.245, -3e-05, 0.056), NO_TURN),), (0.0, -1.0, 0.0), (-1.6, 1.55)),
        Joint((((0.06775, 0.0, 0.0845), NO_TURN),), (0.0, 0.0, -1.0), (-1.57, 1.57)),
        Joint((((0.02895, 0.0, -0.0845), NO_TURN),), (1.0, 0.0, 0.0), (-2.0, 2.0)),
    ))


def _agilex_chain(*, right: bool) -> Chain:
    """An Agilex arm (``agilex.urdf``: ``arm_f*_base_joint``, then joints 1..6; the end link sits on joint 6)."""
    y = -0.3 if right else 0.3
    return Chain((
        Joint((((0.23875, y, 0.775), NO_TURN), ((0.0, 0.0, 0.123), (0.0, 0.0, -1.5708))), Z_AXIS, (-2.618, 2.618)),
        Joint((((0.0, 0.0, 0.0), (1.5708, 0.0, -1.5708)),), Z_AXIS, (0.0, 3.14)),
        Joint((((0.28358, 0.028726, 0.0), (0.0, 0.0, 0.10095)),), Z_AXIS, (-2.697, 0.0)),
        Joint((((-0.24221, 0.068514, 0.0), (-1.5708, 0.0, 1.3826)),), Z_AXIS, (-1.832, 1.832)),
        Joint((((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)),), Z_AXIS, (-1.22, 1.22)),
        Joint((((0.0, 0.091, 0.0014165), (-1.5708, -1.5708, 0.0)),), Z_AXIS, (-3.14, 3.14)),
    ))


BUILDS = {b.name: b for b in (
    # drives (USD): G2 and G2_crsB stiffness 10000, damping 1000; G1 10000 / 500; ARX and Agilex
    # 50000 / 5000
    _g2_build("G2", 0.018, 0.287, 0.314, 0.088,
              ((-3.1067, 3.1067), (-2.0944, 2.0944), (-3.1067, 3.1067), (-2.5307, 1.0472), (-3.1067, 3.1067),
               (-1.0472, 1.0472), (-1.5708, 1.5708)), (3.14,) * 7),
    _g2_build("G2_crsB", 0.0105, 0.332, 0.278, 0.09,
              ((-3.0718, 3.0718), (-2.0595, 2.0595), (-3.0718, 3.0718), (-2.4959, 1.0123), (-3.0718, 3.0718),
               (-1.0123, 1.0123), (-1.5359, 1.5359)), (3.05, 3.05, 3.76, 3.76, 3.35, 3.35, 3.35)),
    Build("G1", _g1_chain(right=False), _g1_chain(right=True), velocity=(3.14, 3.14) + (3.92,) * 5, tau_s=0.05),
    Build("ARX", _arx_chain(right=False), _arx_chain(right=True), body_to_first_joint=False, velocity=(5.0,) * 6,
          tau_s=0.1),
    Build("Agilex", _agilex_chain(right=False), _agilex_chain(right=True), body_to_first_joint=False,
          velocity=(3.0,) * 6, tau_s=0.1),
)}


# -- the grippers ------------------------------------------------------------------------------------
#: the omnipicker's gap between the pads (mm) at its readings (-0.785 open .. 0 closed; Season I's table)
_OP_READING = np.array([-0.785, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0])
_OP_GAP_MM = np.array([67.6, 61.8, 54.3, 46.3, 37.7, 28.7, 19.4, 9.8, 0.0])

#: the ctek90d's linkage (m, degrees): the outer link's pivot (|y|, z) and length, its angle off the
#: approach when open, the pads' inner faces when open (|y|) and the link's far end then (|y|)
_C90_PIVOT = (0.0301, 0.1112)
_C90_LINK = 0.055
_C90_OPEN_DEG = 47.3
_C90_PAD_Y = 0.0448
_C90_TIP_Y = _C90_PIVOT[0] + _C90_LINK * math.sin(math.radians(_C90_OPEN_DEG))
_C90_TIP_Z = _C90_PIVOT[1] + _C90_LINK * math.cos(math.radians(_C90_OPEN_DEG))


def _c90_gap(theta: float) -> float:
    """The ctek90d's gap (m) with its finger joint turned ``theta`` (rad, >= 0) inward."""
    a = math.radians(_C90_OPEN_DEG) - theta
    y = _C90_PIVOT[0] + _C90_LINK * math.sin(a)
    return max(0.0, 2.0 * (_C90_PAD_Y - (_C90_TIP_Y - y)))


def _c90_forward(theta: float) -> float:
    """How far (m) the ctek90d's pads have moved along the approach at ``theta``."""
    a = math.radians(_C90_OPEN_DEG) - theta
    return _C90_PIVOT[1] + _C90_LINK * math.cos(a) - _C90_TIP_Z


@dataclass(frozen=True)
class Hand:
    name: str
    #: the grasp point and the fingertips (open) along the approach (m)
    grasp_reach: float
    tip_reach: float
    #: the right hand's tool frame (approach, finger line, approach x finger line) in its end link's axes
    tool_in_end: tuple
    #: whether the left hand's tool frame is the right's turned half a turn about the approach
    left_flip: bool
    #: wire values fully open and fully closed (the reading is on the same scale)
    open_value: float
    closed_value: float
    #: the gap (m) across the pads when fully open
    open_gap: float
    #: the grasp point in the right hand's end link (m) when it is not on the approach (the Wuji hand's shapes),
    #: and whether the left hand is the right's mirror image (its point's y negated, its tool frame mirrored with
    #: its y turned round)
    tcp_in_end: tuple | None = None
    mirror_left: bool = False
    #: a five-finger hand's shape (:mod:`.wuji`), and whether the hand is one (its effector is 20 joints)
    shape: str = ""
    dexterous: bool = False

    def opening(self, reading: float) -> float:
        """0 closed .. 1 open, from a reading (clipped)."""
        g = (float(reading) - self.closed_value) / (self.open_value - self.closed_value)
        return min(max(g, 0.0), 1.0)

    def command(self, opening: float) -> float:
        """The wire value of an opening 0..1."""
        g = min(max(float(opening), 0.0), 1.0)
        return self.closed_value + g * (self.open_value - self.closed_value)

    def gap_m(self, reading: float) -> float:
        """The gap (m) between the finger pads at a reading."""
        if self.name == "omnipicker":
            return float(np.interp(float(reading), _OP_READING, _OP_GAP_MM)) / 1000.0
        if self.name == "ctek90d":
            return _c90_gap(abs(min(float(reading), 0.0)))
        return self.open_gap * self.opening(reading)  # sliding fingers

    def forward_m(self, reading: float) -> float:
        """How far (m) the pads have moved along the approach since fully open (the ctek90d's swing)."""
        if self.name != "ctek90d":
            return 0.0
        return _c90_forward(abs(min(float(reading), 0.0)))

    def tool_in_end_of(self, arm: str) -> np.ndarray:
        m = np.asarray(self.tool_in_end, float)
        if arm == "left" and self.mirror_left:
            return np.diag([1.0, -1.0, 1.0]) @ m @ np.diag([1.0, -1.0, 1.0])
        return m @ np.diag([1.0, -1.0, -1.0]) if (arm == "left" and self.left_flip) else m

    def tcp_of(self, arm: str) -> np.ndarray:
        """The grasp point in the end link (m)."""
        if self.tcp_in_end is None:
            return self.approach_in_end * self.grasp_reach
        p = np.asarray(self.tcp_in_end, float)
        return p * np.array([1.0, -1.0, 1.0]) if (arm == "left" and self.mirror_left) else p

    @property
    def approach_in_end(self) -> np.ndarray:
        return np.asarray(self.tool_in_end, float)[:, 0]


#: the G2's omnipicker: tool x = end z, y = end x, z = end y
OMNIPICKER = Hand("omnipicker", 0.00537 + 0.14308, 0.00537 + 0.14308 + 0.0103,
                  ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)), True, -0.785, 0.0, 0.0676)
#: the G1's omnipicker: tool x = end z, y = -end x, z = -end y on both hands (they then start at roll -21 / +21
#: with the hands pointing inward, pour_50ml_g1)
OMNIPICKER_G1 = Hand("omnipicker", 0.14308, 0.14308 + 0.0103,
                     ((0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)), False, -0.785, 0.0, 0.0676)
#: the ctek90d: tool x = end z, y = -end y, z = end x, on both hands (both then start near roll 0:
#: phone_packaging -3 / +4, open_the_door -6 / +5, the Wuji build's pick_up_a_package -1 / +1)
CTEK90D = Hand("ctek90d", 0.19, 0.212, ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0)), False, 0.0, -0.910,
               2.0 * _C90_PAD_Y)
#: the ARX's gripper: tool = end (x the approach, y the finger line); both hands start at pitch 39, roll 0
ARX_GRIPPER = Hand("arx_gripper", 0.13, 0.155, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), False,
                   -3.5, 0.0, 2.0 * 0.044)
#: the Agilex's gripper: tool x = end z, y = end x, z = end y; both hands start at pitch 24, roll 0
AGILEX_GRIPPER = Hand("agilex_gripper", 0.115, 0.14, ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)), False,
                      -3.5, 0.0, 2.0 * 0.04)
HANDS = {h.name: h for h in (OMNIPICKER, CTEK90D, ARX_GRIPPER, AGILEX_GRIPPER)}


def _wuji_hand(shape: wuji.Shape) -> Hand:
    """The Wuji hand in one shape: the grasp point its working point, the tool frame its closing line's."""
    tcp = tuple(float(v) for v in shape.point("right"))
    tool = tuple(tuple(float(v) for v in row) for row in shape.tool_in_end("right"))
    return Hand("wuji", float(tcp[2]), float(tcp[2]), tool, False, 1.0, 0.0, float(shape.open_gap_m or 0.0),
                tcp_in_end=tcp, mirror_left=True, shape=shape.name, dexterous=True)


#: the Wuji hand per shape (dexmanip)
WUJI_HANDS = {name: _wuji_hand(s) for name, s in wuji.SHAPES.items()}

#: joint ranges kept to (radians): the limits less a margin
LIMIT_MARGIN = 0.02
#: joint samples for an arm's reach (the farthest its grasp point gets from the shoulder), and the margin added
REACH_SAMPLES = 20000
REACH_MARGIN_M = 0.01


def _compose(pieces) -> np.ndarray:
    t = np.eye(4)
    for xyz, r in pieces:
        t = t @ homogeneous(rpy(*r), xyz)
    return t


def _skew(axis) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


@dataclass
class Solution:
    q: np.ndarray
    reached: bool
    pos_err_m: float
    rot_err_deg: float
    iterations: int = 0


class Arm:
    """One arm's chain in ``arm_base_link``, with its hand."""

    def __init__(self, name: str, build: Build | str = "G2", hand: Hand | str = OMNIPICKER):
        if name not in ARMS:
            raise ValueError(f"arm must be 'left' or 'right', not {name!r}")
        self.name = name
        self.build = BUILDS[build] if isinstance(build, str) else build
        self.hand = HANDS[hand] if isinstance(hand, str) else hand
        chain = self.build.left if name == "left" else self.build.right
        self.n = len(chain.joints)
        self.origins = [_compose(j.origin) for j in chain.joints]
        self.axes = [np.asarray(j.axis, float) / np.linalg.norm(j.axis) for j in chain.joints]
        self._about_z = [bool(np.allclose(a, (0.0, 0.0, 1.0))) for a in self.axes]
        self._skews = [_skew(a) for a in self.axes]
        self.lower = np.array([j.limits[0] for j in chain.joints]) + LIMIT_MARGIN
        self.upper = np.array([j.limits[1] for j in chain.joints]) - LIMIT_MARGIN
        #: the joints' velocity limits (rad/s) and the drives' time constant (s), see Build
        self.velocity = np.array(self.build.velocity or (3.0,) * self.n, float)
        self.tau_s = float(self.build.tau_s)
        self.end = _compose(chain.end)
        self.grasp_offset = homogeneous(np.eye(3), self.hand.tcp_of(name))
        self.tool = self.end @ self.grasp_offset
        self.tool_in_end = self.hand.tool_in_end_of(name)
        #: the shoulder (where the second joint's frame sits) and the farthest the grasp point gets from it
        self.shoulder = self.frames(np.zeros(self.n))[1][:3, 3].copy()
        self.reach = self._reach()

    # -- forward -------------------------------------------------------------------------------
    def _turn(self, k: int, angle: float) -> np.ndarray:
        m = np.eye(4)
        c, s = math.cos(angle), math.sin(angle)
        if self._about_z[k]:
            m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
        else:
            kk = self._skews[k]
            m[:3, :3] = np.eye(3) + s * kk + (1.0 - c) * (kk @ kk)
        return m

    def frames(self, q) -> list[np.ndarray]:
        """The joint frames (after each joint's turn) in arm_base_link."""
        q = np.asarray(q, float).reshape(self.n)
        t = np.eye(4)
        out = []
        for k in range(self.n):
            t = t @ self.origins[k] @ self._turn(k, q[k])
            out.append(t)
        return out

    def end_pose(self, q) -> np.ndarray:
        """arm_*_end_link in arm_base_link (the pose the benchmark reports as ``end_pose``)."""
        return self.frames(q)[-1] @ self.end

    def grasp_pose(self, q) -> np.ndarray:
        """The grasp point, with the end link's orientation, in arm_base_link."""
        return self.frames(q)[-1] @ self.tool

    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        """(6 x n geometric Jacobian of the grasp point in arm_base_link: linear rows first, the grasp pose)."""
        fr = self.frames(q)
        tip = fr[-1] @ self.tool
        p = tip[:3, 3]
        jac = np.zeros((6, self.n))
        for k, f in enumerate(fr):
            axis = f[:3, 2] if self._about_z[k] else f[:3, :3] @ self.axes[k]
            jac[:3, k] = np.cross(axis, p - f[:3, 3])
            jac[3:, k] = axis
        return jac, tip

    def _reach(self) -> float:
        """The farthest the grasp point gets from the shoulder: the most over joint samples (the same every
        time), plus a margin."""
        rng = np.random.default_rng(0)
        qs = rng.uniform(self.lower, self.upper, size=(REACH_SAMPLES, self.n))
        t = np.broadcast_to(np.eye(4), (REACH_SAMPLES, 4, 4))
        for k in range(self.n):
            s = np.sin(qs[:, k])[:, None, None]
            c = np.cos(qs[:, k])[:, None, None]
            kk = self._skews[k]
            turn = np.zeros((REACH_SAMPLES, 4, 4))
            turn[:, :3, :3] = np.eye(3) + s * kk + (1.0 - c) * (kk @ kk)
            turn[:, 3, 3] = 1.0
            t = (t @ self.origins[k]) @ turn
        tips = (t @ self.tool)[:, :3, 3]
        return float(np.max(np.linalg.norm(tips - self.shoulder, axis=1))) + REACH_MARGIN_M

    # -- inverse -------------------------------------------------------------------------------
    def solve(self, target: np.ndarray, seed, *, iterations: int = 200, pos_tol: float = 5e-4,
              rot_tol_deg: float = 0.3, rot_weight: float = 0.3, null_gain: float = 0.05,
              rest=None) -> Solution:
        """Joints that put the grasp point at ``target`` (4x4 in arm_base_link), from ``seed``; a redundant
        freedom leans toward ``rest`` (default: the seed).

        A joint at a limit that the step would push further is left out of that step (the others take
        over its share); the best joints met are returned when none reaches."""
        n = self.n
        q = np.clip(np.asarray(seed, float).reshape(n), self.lower, self.upper)
        rest = q.copy() if rest is None else np.clip(np.asarray(rest, float).reshape(n), self.lower, self.upper)
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
            free = np.ones(n, bool)
            dq = np.zeros(n)
            for _ in range(4):
                jf = jw[:, free]
                inv = np.linalg.inv(jf @ jf.T + (lam ** 2) * np.eye(6))
                pinv = jf.T @ inv
                step = pinv @ e + null_gain * ((np.eye(int(free.sum())) - pinv @ jf) @ pull[free])
                dq = np.zeros(n)
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
        seeds = [np.asarray(s, float).reshape(self.n) for s in seeds]
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

    # -- the tool frame ------------------------------------------------------------------------
    def tool_rotation(self, end_rotation: np.ndarray) -> np.ndarray:
        """The tool frame of :mod:`hexashell.benches.pose` (approach, finger line, ...) of an end link rotation."""
        return np.asarray(end_rotation, float) @ self.tool_in_end

    def end_rotation(self, tool_rot: np.ndarray) -> np.ndarray:
        return np.asarray(tool_rot, float) @ self.tool_in_end.T


def pose_of(position, quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return homogeneous(r, position)


def quat_xyzw_of(r) -> list[float]:
    """The unit quaternion (x, y, z, w) of a rotation matrix (the inverse of :func:`pose_of`'s)."""
    r = np.asarray(r, float)[:3, :3]
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)
    v = np.asarray(q, float)
    v /= np.linalg.norm(v)
    return [float(x) for x in v]


def arm_base_from_obs(end_pose: dict) -> np.ndarray:
    """``T_base_link <- arm_base_link`` from an observation's ``end_pose`` (each hand in both frames): it
    depends on the waist only, so the left hand's two poses give it."""
    a = end_pose["base_link"]["left_arm"]
    b = end_pose["arm_base_link"]["left_arm"]
    return pose_of(a["position"], a["orientation"]) @ invert(pose_of(b["position"], b["orientation"]))


#: the arm build and hand of each robot of the protocol (``protocol.ROBOTS`` keys)
MODEL_OF_ROBOT = {
    "g2_omnipicker": ("G2", OMNIPICKER),
    "g2_90d": ("G2_crsB", CTEK90D),
    "g1": ("G1", OMNIPICKER_G1),
    "arx": ("ARX", ARX_GRIPPER),
    "agilex": ("Agilex", AGILEX_GRIPPER),
    "g2_wuji": ("G2_crsB", WUJI_HANDS[wuji.START_SHAPE]),
}
_MODELS: dict[tuple, dict[str, Arm]] = {}


def arm_models(robot: str, shape: str | None = None) -> dict[str, Arm]:
    """{'left': Arm, 'right': Arm} of a robot (``protocol.ROBOTS`` key): both arms, also where only one moves
    (the pictures show both).  ``shape``: a five-finger hand's shape (its grasp point and tool frame; None: the
    one it starts in)."""
    if robot not in MODEL_OF_ROBOT:
        raise KeyError(f"no arm model for robot {robot!r}")
    build, hand = MODEL_OF_ROBOT[robot]
    if shape is not None and shape != hand.shape:
        if not hand.dexterous or shape not in WUJI_HANDS:
            raise KeyError(f"no hand shape {shape!r} on robot {robot!r}")
        hand = WUJI_HANDS[shape]
    key = (robot, hand.shape)
    if key not in _MODELS:
        _MODELS[key] = {arm: Arm(arm, build, hand) for arm in ARMS}
    return _MODELS[key]
