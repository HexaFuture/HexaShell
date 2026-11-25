"""The hand and finger commands, carried out through the official step (:mod:`.sim`).

Runs in the server process.  The official step takes, per arm, a change of the wrist pose (the attachment
site: a translation and a rotation vector in the scene frame, added to the pose the environment measures at
that step; at most the task's bounds per axis) and the hand's 20 joint targets.  The environment's own
operational-space controller then pulls the wrist toward that pose for the step.  Measured on the tasks:
with the one-arm bounds (0.02 m, 0.03 rad; its stiff controller) the wrist moves about 0.83 of the
change asked in a step, with the two-arm and Tower of Hanoi bounds (0.08 m, 0.30 rad; a softer controller)
about 0.12; and an arm sent no change stays where it is (it does not sag).

So every arm always has a reference pose, where it should be (``Executor.ref``), and every step sends it
LOOP_GAIN times the way from where it is to the reference, scaled down as a whole to the bounds: with gain 1
(one arm) and 4 (two arms, Hanoi) a 10 cm, 30 degree change settles to 2 mm in 10-12 steps without
overshoot.  A motion moves the reference along a path: the chosen point of the hand (palm, grasp point or a
fingertip) on a straight line, the hand's orientation turning evenly (slerp), at most ``speed``; an arc turns
the hand about a line.  A motion starts from the arm's reference, or from where the hand is when it is further off
its reference than a path waits for (``path_start``: fingers closing on something push the hand off its
reference).  An arm that falls behind its reference by more than BLOCKED_M (or BLOCKED_DEG) for HELD_STEPS steps
in a row is held up by something: the motion stops there and the reference becomes where the hand is (it does
not keep pushing).  The path waits while an arm falls further behind it than it normally follows (WAIT_M,
WAIT_DEG), and an arm it waits for that comes no nearer for STALL_STEPS steps, its joints at rest, is held up as
well (an arm with a joint at the end of its travel works its way out of that end slowly, its joints moving).  At
the end the arm settles on the goal.  The other arm keeps its reference meanwhile.

Grasps (measured on the official Wuji demonstrations): what a closed hand holds sits about 5 cm out from
the palm's face at the finger roots (GRASP_POINT), between the curled fingers and the thumb swung across under
it; a handle held in a power grip runs across the fingers at a slant (GRIP_SLANT: its thumb end further out
along the fingers), and a hand turned so (``grip_frame``) holds a hammer by its handle where a square grip lets it
turn out of the fingers.

Fingers: position actuators (stiff: kp 120); a finger setting moves the targets evenly to the new ones over
``seconds`` and settles; a finger that something stops keeps pressing toward its target (as the official
demonstrations do: their targets lie well past where the fingers stop).  The named shapes (SHAPE_TARGETS) are
the targets the demonstrations closed with; per finger, ``curl`` 0 (straight) .. 1 (curled in) sets J1, J3, J4
(FINGER_CURL), and the thumb's curl and its opposition (swung across the palm) and the four fingers' spread
are set apart.  A finger presses on something when something outside the hand pushes on it (``pressing``,
FINGER_PUSH_RAD): the push from the joints' motion and the hand's own model (``DexSim.outside_push``).  Not
from a finger short of its target: the fingers of a closed hand meet each other and the palm, and of 387
settings closed on an empty hand in the air (hammer_nail's right hand) 132 left a finger more than 0.15 rad
short of its target and 82 its thumb and another finger (a hold; bimanual_photograph's left hand: 131 and 71),
while the push read no finger at any of the 5031 steps of either hand's closings (0 to 1e-6 rad).  Nor
from where the fingers would have closed to with nothing in the hand: an object that pushes a finger aside while
the hand closes can leave the fingers meeting each other another way, short of that with nothing in the hand.
"""

from __future__ import annotations

import math

import numpy as np

from . import protocol as P
from .sim import PALM_POINT, DexSim

DT = P.STEP_S
#: the loop's gain per bound (one-arm tasks: 0.02 m; the others 0.08 m), see the module's text
LOOP_GAIN = {0.02: 1.0, 0.08: 4.0}
#: the share of a step's change the wrist makes (measured, see the module's text): it sets the lag of a motion
FOLLOW_SHARE = {0.02: 0.83, 0.08: 0.117}
#: a motion's default and highest speeds of the hand's point (m/s) and turn (deg/s), per bound
SPEED = P.HAND_SPEED
TURN_SPEED = P.TURN_SPEED
#: an arm this far (m, deg) behind its reference, beyond its expected lag, is held up by something
BLOCKED_M = 0.03
BLOCKED_DEG = 15.0
#: steps in a row the hand must lag beyond those before a motion counts as blocked (the arm's controller can lag
#: 1-3 cm for a few steps when a long turn swings the elbow, and catches up)
HELD_STEPS = 4
#: a motion's path waits while an arm is further behind it than it normally follows by this much (m, deg): the
#: soft arms of the two-arm tasks fall behind a hand's turn near a stretched wrist (a 180-degree roll of the right
#: hand in the microwave task fell 16 deg behind at 30 deg/s); an arm the path waits for and that comes no
#: nearer to it for STALL_STEPS steps is held up; the path waits at most max(its steps, WAIT_STEPS_MIN) steps
WAIT_M = 0.015
WAIT_DEG = 8.0
WAIT_STEPS_MIN = 40
#: how a turn counts against a distance when an arm's progress toward its path is judged (m per degree)
GAP_M_PER_DEG = 0.001
#: a goal is reached within this much (m, deg)
REACHED_M = 0.004
REACHED_DEG = 2.0
#: most steps of settling at the end of a motion, and of settling without getting nearer by SETTLE_GAIN_M
SETTLE_STEPS = {0.02: 20, 0.08: 40}
STALL_STEPS = 8
SETTLE_GAIN_M = 0.0005
#: an arm whose joints move less than this (rad, the largest over STALL_STEPS steps) is at rest.  An arm the path
#: waits for that comes no nearer is held up only once it is at rest: an arm with a joint at the end of its travel
#: turns about its own redundancy, slowly, until the joint comes off its end, and then catches up with the turn
#: lagging (a two-arm hand's 120-degree roll with joint 4 at its end: 0.03-0.4 rad per STALL_STEPS, there after
#: 130 steps); arms pressed on something came to rest within 4-24 steps and drifted 0.02 at most then (29 stopped
#: motions of six learning episodes)
JOINTS_AT_REST_RAD = 0.02
#: the point things a closed hand holds sit at (attachment frame: x along the fingers, z out of the palm's face):
#: about 5 cm out from the palm's face (z 0.045) at the finger roots; the official demonstrations hold the hammer's
#: handle, the boxed food, the hot dog and the Hanoi disks there
GRASP_POINT = np.array([0.15, 0.0, 0.095])
#: a handle held in a power grip, in the right hand's attachment frame, pointing to the thumb's side: across the
#: fingers at a slant of about 20 degrees, its thumb end further out along the fingers (the left hand mirrors y)
GRIP_SLANT = np.array([0.355, -1.0, 0.0]) / math.hypot(0.355, 1.0)
#: an approach: the hand comes the last part of its way along its palm's normal at this speed (m/s)
APPROACH_SPEED = 0.06
#: finger settings: a curl of 1 sets J1, J3, J4 of a finger to these (rad); spread 1 sets J2 to SPREAD_RAD
#: (index toward the thumb's side, ring and little away from it); the thumb's curl sets its J2, J3, J4 and its
#: opposition J1 (rad at 1)
FINGER_CURL = np.array([-1.45, 0.0, 1.45, 1.30])
SPREAD_RAD = 0.40
SPREAD_SIGN = {"index": 1.0, "middle": 0.0, "ring": -1.0, "little": -1.0}
THUMB_CURL = np.array([0.0, 0.70, 1.10, 0.90])
THUMB_OPPOSE_RAD = 1.55
#: the named shapes: the 20 joint targets (thumb, index, middle, ring, little; J1..J4 each) the official Wuji
#: demonstrations closed with (right hand; the left hand's joints mirror it, so the same numbers); "open" is the
#: hand's start (the environment's home).  power: the hammer's handle; wrap: the boxed food; hook: the basket's
#: handle; pinch: a Hanoi disk; tripod: the hot dog; point: the tablet's digits and the microwave's start button
SHAPE_TARGETS = {
    "flat": [0.0] * 20,
    "power": [1.36, 0.23, 0.07, 0.05, -0.65, 0.2, 0.76, 0.29, -1.02, 0.19, 0.51, 0.48, -0.98, 0.21, 0.52, 0.72,
              -0.84, 0.16, 0.42, 0.58],
    "wrap": [1.13, -0.06, 0.38, 0.07, -0.86, -0.12, 0.83, 0.37, -0.91, 0.0, 1.08, 0.52, -0.83, -0.16, 1.0, 0.46,
             -0.8, -0.1, 0.94, 0.42],
    "hook": [1.27, -0.04, 0.51, 0.15, -1.03, -0.08, 0.97, 0.45, -1.18, 0.0, 1.12, 0.56, -0.89, -0.1, 0.8, 0.34,
             -0.75, -0.05, 0.67, 0.26],
    "pinch": [1.26, 0.2, 0.0, 0.63, -0.48, 0.18, 1.07, 0.7, -0.5, 0.0, 0.91, 0.61, -0.37, 0.14, 0.67, 0.45,
              -0.37, 0.14, 0.67, 0.45],
    "tripod": [1.25, -0.17, 0.52, 0.1, -0.79, -0.1, 0.48, 0.2, -0.89, 0.0, 0.78, 0.34, -0.6, -0.11, 0.46, 0.13,
               -0.57, -0.06, 0.24, 0.01],
    "point": [1.1, 0.32, 0.07, 0.05, -0.01, -0.12, 0.14, 0.34, -1.08, -0.01, 0.91, 0.78, -0.96, -0.02, 0.99, 0.82,
              -0.76, -0.09, 1.05, 0.96],
}
#: a finger that something outside the hand pushes on more than this (on any joint, as the joint motor's
#: equivalent: rad; 0.6 N m at kp 120, some 15 N at a fingertip) presses on something.  Over 21 scripted
#: episodes (the six smoke episodes and 15 grasp checks: 402 readings after finger settings and motions, 12896
#: control steps) the hold this gives (the thumb and one more finger) read 69 of the 82 readings at which the
#: thumb and another part each pushed at least 0.5 N on one thing and 1 of the 320 without; per step 1673 of
#: 1881 and 23 of 11015, all in Hanoi, the thumb and a finger on different things, which the joints cannot tell
#: apart.  At 0.002, 76 of 82 and 2 (and fingers resting on a pressed finger read as pressed); the earlier rule (a
#: finger 0.15 rad short of the self-model's closing) 17 of 82.  Of fingers pushing at least 3 N on something
#: half pushed more than 0.08, a quarter less than 0.01: a push along the finger (the thumb's tip) turns no joint
#: and is not felt
FINGER_PUSH_RAD = 0.005
#: settling steps after a finger setting
FINGER_SETTLE_STEPS = 5
#: an arm joint this near an end of its range (rad) is at the end of its travel
JOINT_LIMIT_NEAR_RAD = 0.03
#: a stopped hand whose goal the arm's kinematics cannot reach within this much (m, deg) stopped for that
OUT_OF_REACH_M = 0.01
OUT_OF_REACH_DEG = 5.0
#: a wrist camera's view (``hand(point='camera', look_at=...)``): the turns of the picture about the line of sight
#: tried in this order (deg, from its top upward, or for a view steeper than VIEW_STEEP_COS away from the robot,
#: +x) until the arm reaches one
VIEW_ROLLS_DEG = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, -150.0, 180.0)
VIEW_STEEP_COS = 0.97
#: samples of a way home along the joints, to measure its length and turn (for its duration)
HOME_SAMPLES = 40
#: a reach check (dry runs, why a hand stopped, a wrist view's turn) follows the motion's path at REACH_SAMPLES
#: points with at most REACH_ITERATIONS inverse-kinematics steps at each, IK_ITERATIONS at its end
REACH_SAMPLES = 16
REACH_ITERATIONS = 30
IK_ITERATIONS = 150
#: how strongly its solutions are drawn toward the start joints within the arm's redundancy, its damping, and the
#: margin it keeps from the ends of the joints' travel (rad).  Checked against real motions on four tasks (180
#: motions): every motion it found the arm falling behind on (BLOCKED_M / BLOCKED_DEG, as far as a motion
#: lets a hand fall before it counts as held up) or ending more than 1 cm / 5 degrees off stopped short; of those
#: it found ending within 5 mm / 2 degrees, nine in ten got there (the rest touched the table or an object, or
#: were the soft arms' large turns); between the two, half did.  The damping lets part of the pull into the
#: hand's motion where the arm is stretched or has a joint at the end of its travel, as the controller's own pull
#: keeps the hand off its goal there: with the pull kept to the exact null space, or left out of the last steps,
#: the check called 16-20 more of the 180 motions reachable that stopped short
NULL_PULL = 0.1
IK_DAMPING = 0.05
JOINT_MARGIN = 0.005


def rotvec(R: np.ndarray) -> np.ndarray:
    """The rotation vector of a rotation matrix."""
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle > math.pi - 1e-4:
        # near a half turn: the axis from the symmetric part
        M = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(M), 0.0, None))
        k = int(np.argmax(axis))
        axis = M[:, k] / max(1e-12, math.sqrt(max(M[k, k], 1e-12)))
        if float(axis @ w) < 0.0:
            axis = -axis
        return axis / np.linalg.norm(axis) * angle
    return w / (2.0 * math.sin(angle)) * angle


def rotation(v: np.ndarray) -> np.ndarray:
    """The rotation matrix of a rotation vector (Rodrigues)."""
    v = np.asarray(v, float)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def turn_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return math.degrees(float(np.linalg.norm(rotvec(Ra @ Rb.T))))


def frame_of(normal, finger, R_now: np.ndarray) -> np.ndarray:
    """The hand's orientation (columns: finger direction x, y = z cross x, palm normal z) for a palm normal
    and/or a finger direction; a part left out stays as near as it can to the hand's now."""
    z = R_now[:, 2] if normal is None else np.asarray(normal, float) / np.linalg.norm(normal)
    x = R_now[:, 0] if finger is None else np.asarray(finger, float) / np.linalg.norm(finger)
    if normal is None and finger is not None:
        z = z - x * float(x @ z)
        if np.linalg.norm(z) < 1e-6:
            z = np.cross(x, R_now[:, 1])
    else:
        x = x - z * float(x @ z)
        if np.linalg.norm(x) < 1e-6:  # the fingers along the new normal: keep the hand's y
            x = np.cross(R_now[:, 1], z)
    z = z / np.linalg.norm(z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def view_frame(axis_hand, up_hand, a, w) -> np.ndarray:
    """The hand's orientation that turns a camera on it - its line of sight ``axis_hand`` and its picture's up
    ``up_hand``, both in the attachment frame - to look along ``a`` with the picture's up ``w`` (scene frame, w
    square to a)."""
    e1 = np.asarray(axis_hand, float) / np.linalg.norm(axis_hand)
    e2 = np.asarray(up_hand, float) - e1 * float(np.asarray(up_hand, float) @ e1)
    e2 = e2 / np.linalg.norm(e2)
    a = np.asarray(a, float) / np.linalg.norm(a)
    w = np.asarray(w, float) - a * float(np.asarray(w, float) @ a)
    w = w / np.linalg.norm(w)
    return np.column_stack([a, w, np.cross(a, w)]) @ np.column_stack([e1, e2, np.cross(e1, e2)]).T


def view_up(a) -> np.ndarray:
    """The picture's preferred up for a line of sight ``a``: upward, or for a steep view away from the robot
    (+x); square to a."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    pref = np.array([0.0, 0.0, 1.0]) if abs(float(a[2])) < VIEW_STEEP_COS else np.array([1.0, 0.0, 0.0])
    w = pref - a * float(pref @ a)
    return w / np.linalg.norm(w)


def shape_targets(sim: DexSim, side: str, shape: str, amount: float = 1.0) -> np.ndarray:
    """A named shape's 20 joint targets; ``amount`` < 1 stops that share of the way from open (the hand's
    home) to it."""
    home = np.asarray(sim.hand_home[side], float)
    if shape == "open":
        q = home.copy()
    else:
        q = home + (np.asarray(SHAPE_TARGETS[shape], float) - home) * float(amount)
    lo, hi = sim.hand_range[side][:, 0], sim.hand_range[side][:, 1]
    return np.clip(q, lo, hi)


def pressing(push) -> list[str]:
    """The fingers that something outside the hand pushes on more than FINGER_PUSH_RAD (on any joint; ``push``:
    the 20 joints' ``DexSim.outside_push``): they press on something."""
    push = np.abs(np.asarray(push, float))
    return [f for k, f in enumerate(P.FINGERS) if float(push[4 * k:4 * k + 4].max()) > FINGER_PUSH_RAD]


def finger_settings(sim: DexSim, side: str, base: np.ndarray, curl: dict | None = None, thumb: float | None = None,
                    oppose: float | None = None, spread: float | None = None) -> np.ndarray:
    """``base`` (20 joint targets) with some fingers set: ``curl`` {finger: 0 straight .. 1 curled in} sets the
    finger's J1, J3, J4 (FINGER_CURL); ``thumb`` (0..1) the thumb's J2, J3, J4 (THUMB_CURL); ``oppose`` (0..1)
    the thumb's J1, how far it swings across the palm; ``spread`` (-1..1) the four fingers' J2."""
    q = np.asarray(base, float).copy()
    for k, f in enumerate(P.FINGERS[1:], start=1):
        if curl and f in curl and curl[f] is not None:
            c = float(curl[f])
            q[4 * k], q[4 * k + 2], q[4 * k + 3] = FINGER_CURL[0] * c, FINGER_CURL[2] * c, FINGER_CURL[3] * c
        if spread is not None:
            q[4 * k + 1] = SPREAD_RAD * float(spread) * SPREAD_SIGN[f]
    if thumb is not None:
        q[1:4] = THUMB_CURL[1:] * float(thumb)
    if oppose is not None:
        q[0] = THUMB_OPPOSE_RAD * float(oppose)
    lo, hi = sim.hand_range[side][:, 0], sim.hand_range[side][:, 1]
    return np.clip(q, lo, hi)


def grip_frame(side: str, along, hint) -> np.ndarray:
    """The hand's orientation for a power grip of a handle that runs along ``along`` (pointing to the thumb's
    side): turned so the handle lies across the fingers at GRIP_SLANT, then rolled about the handle so that the
    palm's normal comes as near ``hint`` as it can."""
    D = GRIP_SLANT.copy()
    if side == "left":
        D[1] = -D[1]
    a = np.asarray(along, float)
    a = a / np.linalg.norm(a)
    v = np.cross(D, a)
    s, c = float(np.linalg.norm(v)), float(D @ a)
    if s < 1e-9:
        if c > 0:
            R0 = np.eye(3)
        else:  # half a turn about any axis square to D
            w = np.cross(D, [0.0, 0.0, 1.0])
            R0 = rotation(w / np.linalg.norm(w) * math.pi)
    else:
        R0 = rotation(v / s * math.atan2(s, c))
    z = R0[:, 2]
    h = np.asarray(hint, float)
    zp, hp = z - (z @ a) * a, h - (h @ a) * a
    if np.linalg.norm(hp) < 1e-6:
        return R0
    phi = math.atan2(float(np.cross(zp, hp) @ a), float(zp @ hp))
    return rotation(a * phi) @ R0


def measured_curl(q: np.ndarray) -> dict[str, float]:
    """How far each finger is curled (0 straight .. 1 as FINGER_CURL / THUMB_CURL), from its joints."""
    out = {}
    t = q[0:4]
    out["thumb"] = round(float(np.clip((t[2] / THUMB_CURL[2] + t[3] / THUMB_CURL[3]) / 2.0, 0.0, 1.2)), 2)
    for k, f in enumerate(P.FINGERS[1:], start=1):
        j = q[4 * k:4 * k + 4]
        c = (j[0] / FINGER_CURL[0] + j[2] / FINGER_CURL[2] + j[3] / FINGER_CURL[3]) / 3.0
        out[f] = round(float(np.clip(c, 0.0, 1.2)), 2)
    return out


class Executor:
    """The commands of one episode: references for the arms, targets for the hands."""

    def __init__(self, sim: DexSim):
        self.sim = sim
        bound = round(sim.translation_bound, 3)
        self.bound = bound
        self.gain = LOOP_GAIN[bound]
        self.follow = FOLLOW_SHARE[bound]
        self.speed_default, self.speed_max = SPEED[bound]
        self.turn_default, self.turn_max = TURN_SPEED[bound]
        self.settle_steps = SETTLE_STEPS[bound]
        self.ref = {s: sim.tcp(s).copy() for s in sim.sides}
        self.start = {s: sim.tcp(s).copy() for s in sim.sides}
        self.start_q = {s: sim.arm_q(s).copy() for s in sim.sides}
        self.targets = {s: sim.hand_targets(s) for s in sim.sides}
        self.shape = {s: "open" for s in sim.sides}

    # -- one step ----------------------------------------------------------------------------------
    def delta(self, side: str) -> np.ndarray:
        """The wrist change of this step toward the arm's reference, within the bounds."""
        base = self.sim.base_tcp(side)
        ref = self.ref[side]
        d = self.gain * (ref[:3, 3] - base[:3, 3])
        r = self.gain * rotvec(ref[:3, :3] @ base[:3, :3].T)
        tb, rb = 0.999 * self.sim.translation_bound, 0.999 * self.sim.rotation_bound
        d = d / max(1.0, float(np.abs(d).max()) / tb)
        r = r / max(1.0, float(np.abs(r).max()) / rb)
        return np.r_[d, r]

    def step(self) -> bool:
        commands = {s: np.r_[self.delta(s), self.targets[s]] for s in self.sim.sides}
        return self.sim.step(commands)

    def hold_steps(self, n: int) -> int:
        t0 = self.sim.steps
        for _ in range(int(n)):
            if not self.step():
                break
        return self.sim.steps - t0

    # -- points of the hand ------------------------------------------------------------------------
    def offset(self, side: str, point: str) -> np.ndarray:
        """Where a point of the hand is in its attachment frame now: 'palm', 'grasp', a finger's tip or the
        wrist camera ('camera')."""
        if point == "palm":
            return PALM_POINT.copy()
        if point == "grasp":
            return GRASP_POINT.copy()
        if point == "camera":
            return self.camera_mount(side)[0]
        if point in P.FINGERS:
            T = self.sim.tcp(side)
            return T[:3, :3].T @ (self.sim.tips(side)[point] - T[:3, 3])
        raise ValueError(f"point {point!r}: one of {', '.join(P.HAND_POINTS)}")

    def camera_mount(self, side: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The hand's wrist camera in its attachment frame: where it is, its line of sight, its picture's up."""
        T = self.sim.tcp(side)
        c = np.asarray(self.sim.camera(f"{side}_wrist")["pose"], float)
        Rt = T[:3, :3]
        return Rt.T @ (c[:3, 3] - T[:3, 3]), Rt.T @ -c[:3, 2], Rt.T @ c[:3, 1]

    def view(self, side: str, cam_goal, look_at) -> tuple[np.ndarray, dict]:
        """The hand's orientation that puts its wrist camera at ``cam_goal`` looking at ``look_at``: the first of
        the picture's turns VIEW_ROLLS_DEG the arm reaches (or, if none, the one it comes nearest), and that
        turn's reach."""
        off, axis, up = self.camera_mount(side)
        cam_goal = np.asarray(cam_goal, float)
        a = np.asarray(look_at, float) - cam_goal
        a = a / np.linalg.norm(a)
        w0 = view_up(a)
        best = None
        for roll in VIEW_ROLLS_DEG:
            R = view_frame(axis, up, a, rotation(a * math.radians(roll)) @ w0)
            goal = np.eye(4)
            goal[:3, :3], goal[:3, 3] = R, cam_goal - R @ off
            r = self.reach(side, goal, off)
            if r["error_m"] <= REACHED_M and r["error_deg"] <= REACHED_DEG and r["lost_at"] is None:
                return R, r
            score = r["error_m"] + r["error_deg"] / 1000.0
            if best is None or score < best[1]["error_m"] + best[1]["error_deg"] / 1000.0:
                best = (R, r)
        return best

    def point_now(self, side: str, point: str) -> np.ndarray:
        T = self.sim.tcp(side)
        return T[:3, :3] @ self.offset(side, point) + T[:3, 3]

    # -- motions -------------------------------------------------------------------------------------
    def move(self, goals: dict[str, dict], speed: float | None = None, turn_speed: float | None = None) -> dict:
        """Move hands at once: per side {'point': name, 'xyz': goal of that point, 'R': goal orientation or
        None}.  The points go in straight lines, the orientations turn evenly; all sides take the same time."""
        speed = self.speed_default if speed is None else min(float(speed), self.speed_max)
        turn_speed = self.turn_default if turn_speed is None else min(float(turn_speed), self.turn_max)
        plans = {}
        duration = 0.0
        for side, g in goals.items():
            off = self.offset(side, g["point"])
            T0 = self.ref[side] = self.path_start(side)
            p0 = T0[:3, :3] @ off + T0[:3, 3]
            R0 = T0[:3, :3]
            R1 = R0 if g.get("R") is None else np.asarray(g["R"], float)
            p1 = np.asarray(g["xyz"], float)
            turn = rotvec(R1 @ R0.T)
            plans[side] = {"off": off, "p0": p0, "p1": p1, "R0": R0, "turn": turn, "R1": R1}
            duration = max(duration, float(np.linalg.norm(p1 - p0)) / speed,
                           math.degrees(float(np.linalg.norm(turn))) / turn_speed)
        n = max(1, int(math.ceil(duration / DT)))

        def ref_at(plan, s):
            R = rotation(plan["turn"] * s) @ plan["R0"]
            p = plan["p0"] + (plan["p1"] - plan["p0"]) * s
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = p - R @ plan["off"]
            return T

        return self._follow(plans, n, ref_at)

    def place(self, side: str, point: str, xyz, R=None, speed: float | None = None,
              approach: float | None = None) -> dict:
        """Move one hand's ``point`` to ``xyz`` with orientation ``R`` (None: as the hand's reference now).  With
        ``approach`` (m) the hand first goes to the goal set back that far along the goal's palm normal, then comes
        in along the normal at APPROACH_SPEED (or ``speed``, when slower)."""
        R_goal = self.ref[side][:3, :3].copy() if R is None else np.asarray(R, float)
        xyz = np.asarray(xyz, float)
        if not approach:
            return self.move({side: {"point": point, "xyz": xyz, "R": R_goal}}, speed=speed)
        back = xyz - float(approach) * R_goal[:, 2]
        first = self.move({side: {"point": point, "xyz": back, "R": R_goal}}, speed=speed)
        if first["sides"][side]["blocked"] or self.sim.over:
            first["stage"] = "on the way to the approach's start"
            return first
        slow = APPROACH_SPEED if speed is None else min(APPROACH_SPEED, float(speed))
        second = self.move({side: {"point": point, "xyz": xyz, "R": R_goal}}, speed=slow)
        second["steps"] += first["steps"]
        if second["sides"][side]["blocked"]:
            second["stage"] = "on the approach"
        return second

    def arc(self, side: str, center, axis, angle_rad: float, turn_speed: float | None = None) -> dict:
        """Turn the hand by ``angle_rad`` about the line through ``center`` along ``axis`` (the hand turns with
        it), at most ``turn_speed`` (deg/s) and the hand's point speed limit."""
        turn_speed = self.turn_default if turn_speed is None else min(float(turn_speed), self.turn_max)
        c = np.asarray(center, float)
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        self.ref[side] = self.path_start(side)
        T0 = self.ref[side].copy()
        radius = float(np.linalg.norm(np.cross(a, T0[:3, :3] @ PALM_POINT + T0[:3, 3] - c)))
        duration = max(math.degrees(abs(angle_rad)) / turn_speed, radius * abs(angle_rad) / self.speed_max)
        n = max(1, int(math.ceil(duration / DT)))
        plans = {side: {"T0": T0, "p1": None}}

        def ref_at(plan, s):
            Rr = rotation(a * angle_rad * s)
            T = np.eye(4)
            T[:3, :3] = Rr @ plan["T0"][:3, :3]
            T[:3, 3] = Rr @ (plan["T0"][:3, 3] - c) + c
            return T

        return self._follow(plans, n, ref_at)

    def _follow(self, plans: dict, n: int, ref_at) -> dict:
        sim = self.sim
        t0 = sim.steps
        lag = (1.0 / (self.follow * self.gain) - 1.0)
        blocked: dict[str, str] = {}
        limits: dict[str, list[int]] = {}
        beyond: dict[str, float] = {}
        held = dict.fromkeys(plans, 0)
        stalled = dict.fromkeys(plans, 0)
        joints = {side: [sim.arm_q(side)] for side in plans}
        best = dict.fromkeys(plans, math.inf)
        ends = {side: ref_at(plan, 1.0) for side, plan in plans.items()}
        k = waited = 0
        while k < n:
            active = [side for side in plans if side not in blocked]
            if not active:
                break
            # the path waits while an arm falls behind it more than it follows (a hard turn of the soft arms)
            s0 = k / n
            step_len, step_deg, lagging = {}, {}, []
            for side in active:
                a, b = ref_at(plans[side], s0), ref_at(plans[side], min(1.0, (k + 1) / n))
                step_len[side] = float(np.linalg.norm((b[:3, :3] - a[:3, :3]) @ PALM_POINT + b[:3, 3] - a[:3, 3]))
                step_deg[side] = turn_deg(a[:3, :3], b[:3, :3])
                e_m, e_deg = self._behind(side)
                if e_m > WAIT_M + 1.5 * lag * step_len[side] or e_deg > WAIT_DEG + 1.5 * lag * step_deg[side]:
                    lagging.append(side)
            if lagging and waited < max(n, WAIT_STEPS_MIN):
                waited += 1
            else:
                k += 1
            s = k / n
            for side in active:
                self.ref[side] = ref_at(plans[side], s)
            if not self.step():
                break
            for side in active:
                why = self._held_up(side, BLOCKED_M + 1.5 * lag * step_len[side])
                held[side] = held[side] + 1 if why else 0
                joints[side] = (joints[side] + [sim.arm_q(side)])[-(STALL_STEPS + 1):]
                # an arm that the path waits for and that comes no nearer to it is held up as well, once it is at
                # rest
                if side in lagging:
                    e_m, e_deg = self._behind(side)
                    gap = e_m + e_deg * GAP_M_PER_DEG
                    if gap < best[side] - SETTLE_GAIN_M:
                        best[side], stalled[side] = gap, 0
                    elif self._moving(joints[side]):
                        stalled[side] = 0
                    else:
                        stalled[side] += 1
                else:
                    best[side], stalled[side] = math.inf, 0
                if held[side] >= HELD_STEPS or stalled[side] >= STALL_STEPS:
                    if why is None:
                        e_m, e_deg = self._behind(side)
                        why = (f"the hand stays {100 * e_m:.1f} cm and {e_deg:.0f} deg behind its path and comes no "
                               "nearer (something holds it up)")
                    blocked[side] = self._why(side, why, limits, ends[side], beyond)
                    self.ref[side] = sim.tcp(side)
        # settle on the goals
        if not sim.over:
            best = {side: math.inf for side in plans if side not in blocked}
            still = {side: 0 for side in best}
            for _ in range(self.settle_steps):
                if not best:
                    break
                if all(self._near(side, ends[side]) for side in best):
                    break
                if not self.step():
                    break
                for side in list(best):
                    e_m, e_deg = self._off(side, ends[side])
                    if e_m < best[side] - SETTLE_GAIN_M:
                        best[side], still[side] = e_m, 0
                    else:
                        still[side] += 1
                    if still[side] >= STALL_STEPS and (e_m > REACHED_M or e_deg > REACHED_DEG):
                        if e_m > 0.01 or e_deg > 5.0:
                            blocked[side] = self._why(side, f"it stopped {100 * e_m:.1f} cm short (something holds "
                                                            "it up)", limits, ends[side], beyond)
                            self.ref[side] = sim.tcp(side)
                        best.pop(side)
        out = {"steps": sim.steps - t0, "sides": {}}
        for side in plans:
            e_m, e_deg = self._off(side, ends[side])
            out["sides"][side] = {"blocked": blocked.get(side), "error_m": e_m, "error_deg": e_deg,
                                  "reached": side not in blocked and e_m <= 2.5 * REACHED_M
                                  and e_deg <= 2.5 * REACHED_DEG, "at_limit": limits.get(side, []),
                                  "beyond_reach_m": beyond.get(side)}
        return out

    def joints_at_limit(self, side: str) -> list[int]:
        """The arm's joints (1-7, shoulder to wrist) at an end of their travel now."""
        q = self.sim.arm_q(side)
        lo, hi = self.sim.arm_range[side][:, 0], self.sim.arm_range[side][:, 1]
        return [k + 1 for k in range(7) if q[k] - lo[k] < JOINT_LIMIT_NEAR_RAD or hi[k] - q[k] < JOINT_LIMIT_NEAR_RAD]

    def _why(self, side: str, why: str, limits: dict, goal: np.ndarray | None = None,
             beyond: dict | None = None) -> str:
        """Why a hand stopped: the arm at the end of a joint's travel (proprioception), the goal out of the arm's
        reach (its kinematics: ``reach`` from the joints now), or what held it up."""
        joints = self.joints_at_limit(side)
        if joints:
            limits[side] = joints
            return (f"the arm is at the end of its travel (joint {', '.join(map(str, joints))}): it gets no farther "
                    "this way")
        if goal is not None and beyond is not None:
            r = self.reach(side, goal)
            if r["lost_at"] is not None:
                beyond[side] = r["lost_m"]
                return (f"the way to the goal is out of the arm's reach with the hand held this way (from here the arm "
                        f"falls {100 * r['lost_m']:.1f} cm, {r['lost_deg']:.0f} degrees behind it "
                        f"{100 * r['lost_at']:.0f}% of the way)")
            if r["error_m"] > OUT_OF_REACH_M or r["error_deg"] > OUT_OF_REACH_DEG:
                beyond[side] = r["error_m"]
                return (f"the goal is out of the arm's reach with the hand held this way (the arm gets no nearer than "
                        f"{100 * r['error_m']:.1f} cm, {r['error_deg']:.0f} degrees)")
        return why

    def _off(self, side: str, goal: np.ndarray) -> tuple[float, float]:
        T = self.sim.tcp(side)
        off = PALM_POINT
        p = T[:3, :3] @ off + T[:3, 3]
        q = goal[:3, :3] @ off + goal[:3, 3]
        return float(np.linalg.norm(p - q)), turn_deg(T[:3, :3], goal[:3, :3])

    def _near(self, side: str, goal: np.ndarray) -> bool:
        e_m, e_deg = self._off(side, goal)
        return e_m < REACHED_M and e_deg < REACHED_DEG

    def _behind(self, side: str) -> tuple[float, float]:
        """How far the hand is from its reference now: the palm's distance (m) and the turn (deg)."""
        T = self.sim.tcp(side)
        ref = self.ref[side]
        return (float(np.linalg.norm((T[:3, :3] - ref[:3, :3]) @ PALM_POINT + T[:3, 3] - ref[:3, 3])),
                turn_deg(T[:3, :3], ref[:3, :3]))

    def path_start(self, side: str) -> np.ndarray:
        """Where a motion of the hand starts: its reference, or the hand's pose when the hand is further off its
        reference than a path waits for (WAIT_M, WAIT_DEG).  Fingers closing on something push the hand off its
        reference (2-7 cm and 8-24 degrees after grasps on the official tasks); a path from the reference
        would count that offset as the hand held up as soon as it starts."""
        e_m, e_deg = self._behind(side)
        if e_m > WAIT_M or e_deg > WAIT_DEG:
            return self.sim.tcp(side).copy()
        return self.ref[side]

    @staticmethod
    def _moving(joints: list) -> bool:
        """Whether the arm still moves its joints: more than JOINTS_AT_REST_RAD over the last STALL_STEPS steps
        (``joints``: its joint readings of those steps, oldest first)."""
        return len(joints) > STALL_STEPS and float(np.abs(joints[-1] - joints[0]).max()) > JOINTS_AT_REST_RAD

    def _held_up(self, side: str, allowed_m: float) -> str | None:
        e_m, e_deg = self._behind(side)
        if e_m > allowed_m:
            return f"the hand fell {100 * e_m:.1f} cm behind its path (something holds it up)"
        if e_deg > BLOCKED_DEG:
            return f"the hand's turn fell {e_deg:.0f} deg behind (something holds it up)"
        return None

    def home(self, sides) -> dict:
        """Every hand of ``sides`` back to its start pose along the arm's joints: the wrist's reference follows
        the poses of the seven joints turning evenly from where they are to where they started, a path the arm
        can always take.  (A straight line with the shortest turn, as other motions go, can drive a joint to the
        end of its travel: a hand stopped at joint 5's limit after a large turn could not get home that way.)"""
        import mujoco

        sim = self.sim
        m = sim.m
        d = mujoco.MjData(m)
        d.qpos[:] = sim.shadow.qpos
        plans = {}

        def pose_at(side, q):
            d.qpos[sim.arm_qpos[side]] = q
            mujoco.mj_kinematics(m, d)
            site = sim.tcp_site[side]
            T = np.eye(4)
            T[:3, :3] = d.site_xmat[site].reshape(3, 3)
            T[:3, 3] = d.site_xpos[site]
            return T

        duration = 0.0
        for side in sides:
            q0, q1 = sim.arm_q(side).copy(), self.start_q[side].copy()
            plans[side] = {"side": side, "q0": q0, "q1": q1}
            poses = [pose_at(side, q0 + (q1 - q0) * s) for s in np.linspace(0.0, 1.0, HOME_SAMPLES + 1)]
            length = sum(float(np.linalg.norm((b[:3, :3] - a[:3, :3]) @ PALM_POINT + b[:3, 3] - a[:3, 3]))
                         for a, b in zip(poses, poses[1:]))
            turned = sum(turn_deg(a[:3, :3], b[:3, :3]) for a, b in zip(poses, poses[1:]))
            duration = max(duration, length / self.speed_default, turned / self.turn_default)
        n = max(1, int(math.ceil(duration / DT)))

        def ref_at(plan, s):
            return pose_at(plan["side"], plan["q0"] + (plan["q1"] - plan["q0"]) * s)

        return self._follow(plans, n, ref_at)

    # -- fingers -----------------------------------------------------------------------------------
    def fingers(self, side: str, goal: np.ndarray, seconds: float) -> dict:
        """Moves the hand's targets evenly to ``goal`` over ``seconds`` and settles FINGER_SETTLE_STEPS steps; the
        fingers something outside the hand pushes on then are ``pressing`` (``pressing``)."""
        sim = self.sim
        t0 = sim.steps
        start = self.targets[side].copy()
        goal = np.asarray(goal, float)
        n = max(1, int(round(float(seconds) / DT)))
        for k in range(1, n + FINGER_SETTLE_STEPS + 1):
            if k <= n:
                s = k / n
                self.targets[side] = start + (goal - start) * (s * s * (3.0 - 2.0 * s))
            else:
                self.targets[side] = goal.copy()
            if not self.step():
                break
        q = sim.hand_q(side)
        return {"steps": sim.steps - t0, "pressing": pressing(sim.outside_push(side)), "curl": measured_curl(q),
                "asked": measured_curl(goal)}

    # -- reach (dry runs) ----------------------------------------------------------------------------
    def reach(self, side: str, goal: np.ndarray, off=None, via: np.ndarray | None = None) -> dict:
        """Whether the arm can follow a motion to ``goal`` (4x4: the attachment site's pose), through ``via`` (the
        approach's start) when given: the arm's kinematics only, not what the hand or arm would touch.  The
        motion's path - the hand's point ``off`` (the palm by default) on straight lines from where the motion
        starts (``path_start``), the hand turning evenly, as ``move`` makes it - is followed by damped least
        squares over the arm's seven joints on a copy of the state, from the joints now.  Each step is drawn toward
        the start joints within the arm's redundancy, as the environment's controller draws the arm toward its home
        posture (which it starts in), and leaves out the joints at an end of their travel that it would push
        further.  Gives the
        error left at the goal, the joints at the end of their travel there, and where along the path (0..1, by
        the point's distance) the arm first fell more than BLOCKED_M / BLOCKED_DEG behind it, how far, and the
        joints at the end of their travel then (``lost_at`` None: nowhere).  (An inverse kinematics of the goal
        alone, from the joints now and without the pull, found poses the controller cannot take.)"""
        import mujoco

        sim = self.sim
        m = sim.m
        d = mujoco.MjData(m)
        d.qpos[:] = sim.shadow.qpos
        site = sim.tcp_site[side]
        qadr, dadr = sim.arm_qpos[side], sim.arm_dof[side]
        lo, hi = sim.arm_range[side][:, 0] + JOINT_MARGIN, sim.arm_range[side][:, 1] - JOINT_MARGIN
        home = self.start_q[side]
        off = PALM_POINT if off is None else np.asarray(off, float)
        poses = [self.path_start(side)] + ([] if via is None else [np.asarray(via, float)]) + [np.asarray(goal, float)]
        points = [T[:3, :3] @ off + T[:3, 3] for T in poses]
        lengths = [float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:])]
        total = sum(lengths)
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        lost = lost_limits = None
        lost_err = (0.0, 0.0)

        def error(p, R):
            mujoco.mj_kinematics(m, d)
            ep = p - d.site_xpos[site]
            er = rotvec(R @ d.site_xmat[site].reshape(3, 3).T)
            return ep, er, float(np.linalg.norm(ep)), math.degrees(float(np.linalg.norm(er)))

        def limits_now():
            q = d.qpos[qadr]
            return [k + 1 for k in range(7) if q[k] <= lo[k] + 1e-3 or q[k] >= hi[k] - 1e-3]

        def solve(ep, er, q):
            mujoco.mj_comPos(m, d)
            mujoco.mj_jacSite(m, d, jacp, jacr, site)
            J0 = np.vstack([jacp[:, dadr], 0.2 * jacr[:, dadr]])
            e = np.r_[ep, 0.2 * er]
            free = np.ones(7, bool)
            for _ in range(4):
                J = J0 * free
                Jp = J.T @ np.linalg.inv(J @ J.T + IK_DAMPING ** 2 * np.eye(6))
                dq = Jp @ e + (np.eye(7) - Jp @ J) @ (NULL_PULL * (home - q) * free)
                push = free & (((q <= lo + 1e-4) & (dq < 0.0)) | ((q >= hi - 1e-4) & (dq > 0.0)))
                if not push.any():
                    break
                free &= ~push
            return dq

        e_m = e_deg = 0.0
        done = 0.0
        for leg, (Ta, Tb) in enumerate(zip(poses, poses[1:])):
            Ra = Ta[:3, :3]
            turn = rotvec(Tb[:3, :3] @ Ra.T)
            pa, pb = points[leg], points[leg + 1]
            for i in range(1, REACH_SAMPLES + 1):
                s = i / REACH_SAMPLES
                end = i == REACH_SAMPLES
                last = end and leg == len(poses) - 2
                R = Tb[:3, :3] if end else rotation(turn * s) @ Ra
                p = (pb if end else pa + (pb - pa) * s) - R @ off
                for _ in range(IK_ITERATIONS if last else REACH_ITERATIONS):
                    ep, er, e_m, e_deg = error(p, R)
                    if e_m < 0.001 and e_deg < 0.5:
                        break
                    q = d.qpos[qadr].copy()
                    dq = solve(ep, er, q)
                    step = float(np.abs(dq).max())
                    if step > 0.2:
                        dq *= 0.2 / step
                    d.qpos[qadr] = np.clip(q + dq, lo, hi)
                ep, er, e_m, e_deg = error(p, R)
                if lost is None and not last and (e_m > BLOCKED_M or e_deg > BLOCKED_DEG):
                    along = (done + s * lengths[leg]) / total if total > 1e-6 else (leg + s) / (len(poses) - 1)
                    lost, lost_limits, lost_err = round(along, 2), limits_now(), (e_m, e_deg)
            done += lengths[leg]
        return {"error_m": e_m, "error_deg": e_deg, "at_limit": limits_now(), "lost_at": lost,
                "lost_limits": lost_limits, "lost_m": lost_err[0], "lost_deg": lost_err[1]}
