"""One Panda arm through robosuite's OSC_POSE controller in delta mode (simulator server side).

Shared by the benchmarks whose arm is a Panda under robosuite's operational-space controller
(RoboCasa's PandaOmron, LIBERO's Panda).  The benchmark's sim object supplies the readings (see
:class:`Arm`) and its robot class turns the arm's six numbers and the gripper's sign into the
environment's own action.

A motion moves the grasp point on a straight line while it turns, rest to rest, on one trapezoidal
profile (at most ARM_SPEED, ARM_ACCEL and ARM_TURN, or a lower speed asked for, down to MIN_SPEED,
with its acceleration and turning slowed in proportion), planned in the frame of ``robot0_link0``;
where the way ahead leaves the elbow little room (SLOW_ROOM) it goes slower.
OSC_POSE (kp 150, critically damped) lags a moving goal by kd/kp = 0.16 s and does not compensate a
held load, so each step's delta aims past the planned pose: feed-forward of the plan's velocity and
acceleration, damping of the hand's velocity error, and while plan and hand rest a slow integral of
what is left.  A motion is blocked when the hand stays behind its plan without keeping up with it (its speed along
the plan's direction) or very far behind it, or after the plan stays far from the goal without closing in.  (A
hand that falls behind while the arm comes out of a nearly straight elbow keeps up again once it bends: it is
not blocked.)
A motion also stops when the elbow straightens to within STRETCH_STOP of straight, or would within
STRETCH_AHEAD at the speed the wrist moves away from the shoulder (``stretched``), and the arm gives way
back along its joints toward where the motion started (GIVE_WAY_BEND).  The controller keeps the elbow
bent as far as the hand's pose allows (:class:`Elbow`), so how straight it gets follows from where the
hand is: a hand a centimetre or two beyond its goal, far out, straightens it a lot.
After the plan, the hand settles until it is within the arm's settle tolerance (SETTLE by default) and at
rest (REST), or its lag stops shrinking (STALL_*: near the edge of the arm's reach, or touching something);
it counts as arrived within ARRIVED.  A benchmark may loosen the tolerance and drop the rest: what is left
of the lag then goes on shrinking while the arm holds the reached pose (during a grip or a wait), and a
next motion plans from where the hand is.
Between motions the arm holds the pose it last reached, relative to ``robot0_link0``.  The deltas
go to the OSC in the frame of its origin site, from MuJoCo's world frame.
A push (:meth:`Arm.push_arc`) follows no plan: in short cycles, each holding a goal a few centimetres past
the hand, it presses a hinged part around its hinge, the hand turning with the part as far as the hand's
own track shows it turned (PRESS_CHUNK).

The gripper's sign action moves a level from -1 (open) to +1 (closed) by 0.2 per step; the opening
asked for is a level, held while other things move.
"""

from __future__ import annotations

import math

import numpy as np

#: control period of the environments (20 Hz)
DT = 0.05
ARM_SPEED = 0.25  # m/s of the grasp point
ARM_ACCEL = 1.0  # m/s^2
ARM_TURN = math.radians(60.0)  # rad/s of the hand
MIN_SPEED = 0.01  # m/s: the slowest a motion may ask for
FEEDFORWARD = 0.75
DAMP = 0.15  # s
REST = (0.01, math.radians(3.0))  # m/s, rad/s: the hand is still
KI = 0.1
I_ZONE = (0.015, math.radians(4.0))
I_MAX = (0.03, math.radians(10.0))
BEHIND = (0.02, math.radians(8.0))  # behind the plan by this much while not keeping up: blocked
FAR_BEHIND = (0.06, math.radians(20.0))  # after the plan, this far from the goal and not closing in: blocked
VERY_FAR = (0.12, math.radians(40.0))  # this far behind the plan: blocked, keeping up or not
KEEP_UP = 0.3  # share of the planned speed
#: after the plan, the hand closes in when its lag (in units of FAR_BEHIND) shrinks below this share per step
CLOSING = 0.95
BLOCKED_STEPS = 5
#: a motion or a hold stops when the elbow straightens to less than this (rad) short of straight, or
#: would within STRETCH_AHEAD (s) at the speed the wrist moves out, and the arm gives way: the controller
#: does not bend a stretched arm back (it turns the wrist instead), the arm's momentum carries the elbow
#: on through straight to the end of its travel, and a stopped arm held there creeps on outward.  After
#: the stop the wrist coasts on outward for about 0.17 s at the speed it had; a longer look ahead would
#: stop planned approaches at full speed to targets 30 deg short of straight (LIBERO's least)
STRETCH_STOP = math.radians(15.0)
STRETCH_AHEAD = 0.15
#: near straight the controller has little hold on the elbow: a fast motion carries the wrist on past
#: its plan, centimetres farther out, and the hand's turn lags and overshoots.  So a motion goes slower
#: where its way from there to SLOW_AHEAD (m) on leaves the elbow less room (Elbow.room) than
#: SLOW_ROOM: down to SLOWEST of the arm's speeds at SLOWEST_ROOM and less, and all its acceleration
#: by the least of those shares (the room sampled at ROOM_SAMPLES + 1 points of the way).  Five
#: replayed approaches that took the elbow to within 3 deg of straight or past it at full speed (to
#: targets of 30-40 deg room, or by a way through less) kept it 22-27 deg from straight at 0.4 of the
#: speeds all the way, and as far this way in a quarter fewer steps; 0.6 all the way, slowing from
#: 0.05 m ahead, or only where the plan's own room was low, left some within 9 deg of straight or past
SLOW_ROOM = math.radians(50.0)
SLOWEST_ROOM = math.radians(40.0)
SLOWEST = 0.4
SLOW_AHEAD = 0.1
ROOM_SAMPLES = 32
#: a motor of the arm at this share of its torque limit strains: the arm presses on something.  Pressed
#: on with the elbow bent less than STRAIN_BEND short of straight, the arm can run away: the wrist's weak
#: motors give out and the elbow snaps on through straight to the end of its travel.  Holding the pose,
#: following the hand, or lifting it does not stop that; pulling the hand toward the shoulder does (it
#: bends the elbow back).  So such a strain stops a motion, a hold or a drive at once (``jammed``) and the
#: arm gives way: the hand goes GIVE_WAY_M toward the shoulder, then holds until the joints rest
#: (GIVE_WAY_STEPS at most)
STRAIN = 0.99
STRAIN_BEND = math.radians(90.0)
GIVE_WAY_M = 0.05
GIVE_WAY_STEPS = 20
#: when the elbow straightens too far (STRETCH_STOP), the arm gives way back along its joints toward where
#: the motion started (a hold: where the last one started), until the elbow is this much (rad) more bent
#: or all the way, at GIVE_WAY_SPEED of the arm's speeds, then holds as above.  The joints' way bends the
#: elbow; pulling the hand toward the shoulder near straight does not (the wrist swings out instead: in two
#: replayed stops the elbow ended within 1 deg of straight that way, 37-48 deg going back by the joints).
#: When the way back would bend it less than GIVE_WAY_MIN, the hand is pulled in (GIVE_WAY_M) instead
GIVE_WAY_BEND = math.radians(20.0)
GIVE_WAY_MIN = math.radians(5.0)
GIVE_WAY_SPEED = 0.6
#: an elbow bent backwards past straight (pressed on through it) stays there: pulling the hand in, on the
#: way home or giving way after a strain, bends it on backwards against the end of its travel, and every
#: way home is BLOCKED.  So the way home first moves the grip site this far (m) out along the line from
#: the shoulder to the wrist, its turn kept, not stopped for a strain (:meth:`Arm.unbend`): the elbow comes
#: back to about straight.  Replayed from five such stops (casa_dev4, casa_val1, val2, val6), the arm got
#: home from 2 without it, from 4 after 3 cm out (from 3 after 2 cm out stopped by a strain, which three
#: of them met); in the fifth the hand was caught
UNBEND_M = 0.03
SETTLE = (0.003, math.radians(1.0))
ARRIVED = (0.01, math.radians(3.0))
SETTLE_STEPS = 40
#: settling stops when the lag shrank by less than this share over that many steps
STALL_STEPS, STALL_SHARE = 8, 0.9
GRIP_STEP = 0.2
FINGERS_STILL = 0.02  # m/s
GRIP_STEPS_MAX = 20
#: a coarse workspace check before the inverse kinematics: a ball about the shoulder (robot0_link2)
REACH_M = 0.95
#: push_arc (:meth:`Arm.push_arc`): cycles of PRESS_CHUNK steps, each holding one goal (short, so that
#: pressing never counts as blocked); the touch first goes TOUCH_STEP a cycle along the push until two
#: cycles running advance less than TOUCH_MOVED each (TOUCH_MAX at most; a hand starting from rest, or in a
#: slow pose, can take one cycle that short in the open); then each goal lies the press (PRESS_RANGE) past
#: the hand, deeper by PRESS_GROW every 4 cycles while the part turns less than PRESS_SLOW in 4; the part's
#: angle grows by at most PRESS_FOLLOW a cycle and shrinks by at most PRESS_BACK; stalled after PRESS_STALL
#: cycles at the deepest press without PRESS_GAIN; the hand's track adds a point every TRACK_STEP; the
#: circle fitted to it moves the hinge line (pulled to the given one, sigma FIT_PULL) by FIT_SHIFT at most,
#: to a radius of FIT_RADIUS at least; the hand turns no further while it is TURN_LAG from its asked turn;
#: the hand slid off (or touched nothing) when its distance from the fitted hinge line changed by SLID_M:
#: on a part the hand keeps its distance, a free hand pressed toward the hinge (the push leans only its
#: tilt off it) spirals in
PRESS_CHUNK = 4
PRESS_RANGE = (0.01, 0.05)
PRESS_GROW = 0.005
PRESS_SLOW = math.radians(6.0)
PRESS_FOLLOW = math.radians(8.0)
PRESS_BACK = math.radians(5.0)
PRESS_STALL = 12
PRESS_GAIN = math.radians(2.0)
PRESS_CYCLES = 200
TOUCH_STEP = 0.01
TOUCH_MOVED = 0.004
TOUCH_MAX = 0.12
TRACK_STEP = 0.003
FIT_PULL = 0.03
FIT_SHIFT = 0.15
FIT_RADIUS = 0.05
TURN_LAG = math.radians(15.0)
SLID_M = 0.03

#: the Panda gripper's grip-site axes -> tool axes: tool x = site z (approach), tool y = site x (the
#: fingers slide along it: the finger bodies are turned +90 degrees about z inside the gripper and
#: slide along their own y), tool z = site y
PANDA_SITE_TO_TOOL = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
PANDA_PADS = ("gripper0_right_finger1_pad_collision", "gripper0_right_finger2_pad_collision")
PANDA_HAND = "gripper0_right_right_gripper"


# -- rotations -------------------------------------------------------------------------------------
def rotvec(r: np.ndarray) -> np.ndarray:
    """Axis times angle of a rotation matrix."""
    r = np.asarray(r, float)
    cos = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    angle = math.acos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if angle > math.pi - 1e-6:
        # half a turn: the axis from the symmetric part
        m = (r + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(m), 0.0, None))
        k = int(np.argmax(axis))
        axis = m[:, k] / max(axis[k], 1e-12)
        return axis / np.linalg.norm(axis) * angle
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2.0 * math.sin(angle))
    return w * angle


def rotation(v) -> np.ndarray:
    """Rotation matrix of an axis-angle vector."""
    v = np.asarray(v, float)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx)


def turn_between(a: np.ndarray, b: np.ndarray) -> float:
    """Angle (rad) between two rotation matrices."""
    return float(np.linalg.norm(rotvec(b @ a.T)))


def quat_matrix(quat) -> np.ndarray:
    """Rotation matrix of a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = (float(v) for v in quat)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def planar_pose(x: float, y: float, yaw: float, z: float = 0.0) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    pose = np.eye(4)
    pose[:2, :2] = [[c, -s], [s, c]]
    pose[:3, 3] = (x, y, z)
    return pose


def planar(pose: np.ndarray) -> tuple[float, float, float]:
    return float(pose[0, 3]), float(pose[1, 3]), float(np.arctan2(pose[1, 0], pose[0, 0]))


def _clip_norm(v: np.ndarray, limit: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v if n <= limit else v * (limit / n)


# -- paths -----------------------------------------------------------------------------------------
def trapezoid(length: float, speed: float, accel: float):
    """(duration, distance(t)) of a rest-to-rest profile over ``length``."""
    if length <= 0.0:
        return 0.0, lambda t: 0.0
    if length < speed * speed / accel:
        ramp = math.sqrt(length / accel)
        peak, duration = accel * ramp, 2.0 * ramp
    else:
        ramp = speed / accel
        peak, duration = speed, length / speed + ramp

    def distance(t: float) -> float:
        if t <= 0.0:
            return 0.0
        if t < ramp:
            return 0.5 * accel * t * t
        if t < duration - ramp:
            return 0.5 * accel * ramp * ramp + peak * (t - ramp)
        return length - 0.5 * accel * max(0.0, duration - t) ** 2

    return duration, distance


def fractions(duration: float, distance, length: float) -> np.ndarray:
    """Share of the way done at the end of each control step."""
    if duration <= 0.0 or length <= 0.0:
        return np.zeros(0)
    n = max(1, int(math.ceil(duration / DT - 1e-9)))
    out = np.array([distance(duration * (i + 1) / n) / length for i in range(n)])
    out[-1] = 1.0
    return np.clip(out, 0.0, 1.0)


def speed_cap(room: float) -> float:
    """Share of the arm's speeds a motion keeps where its plan leaves the elbow ``room`` (rad)."""
    share = (room - SLOWEST_ROOM) / (SLOW_ROOM - SLOWEST_ROOM)
    return float(SLOWEST + (1.0 - SLOWEST) * np.clip(share, 0.0, 1.0))


def capped_fractions(length: float, speed: float, accel: float, caps) -> np.ndarray | None:
    """Share of the way done at the end of each control step, rest to rest at most ``accel``, and at
    most ``speed * caps[k]`` at share k / (len(caps) - 1) of the way (speeds between the points
    accelerating evenly).  None when no cap is below 1: the trapezoid's profile is then the same."""
    caps = np.asarray(caps, float)
    if length <= 0.0 or caps.size < 2 or float(caps.min()) >= 1.0:
        return None
    dx = length / (caps.size - 1)
    v = speed * np.clip(caps, 1e-3, 1.0)
    v[0] = 0.0
    for k in range(1, v.size):
        v[k] = min(v[k], math.sqrt(v[k - 1] ** 2 + 2.0 * accel * dx))
    v[-1] = 0.0
    for k in range(v.size - 2, -1, -1):
        v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2.0 * accel * dx))
    t = np.concatenate([[0.0], np.cumsum(2.0 * dx / (v[:-1] + v[1:]))])
    duration = float(t[-1])
    n = max(1, int(math.ceil(duration / DT - 1e-9)))
    out = np.empty(n)
    for i in range(n):
        now = duration * (i + 1) / n
        k = int(np.clip(np.searchsorted(t, now, side="right") - 1, 0, v.size - 2))
        tau = now - t[k]
        a = (v[k + 1] ** 2 - v[k] ** 2) / (2.0 * dx)
        out[i] = (k * dx + v[k] * tau + 0.5 * a * tau * tau) / length
    out[-1] = 1.0
    return np.clip(out, 0.0, 1.0)


def _shares(length: float, pose_at, room, speed: float = ARM_SPEED) -> np.ndarray:
    """Share of the way done at the end of each step: the trapezoid at ``speed`` (m/s, MIN_SPEED to
    ARM_SPEED; the acceleration in proportion), slowed (speed_cap) where the way from there to
    SLOW_AHEAD on leaves the elbow little room (``room`` of a pose, see :meth:`Elbow.room`; None:
    never), its acceleration too."""
    speed = float(np.clip(speed, MIN_SPEED, ARM_SPEED))
    accel = ARM_ACCEL * speed / ARM_SPEED
    if room is not None and length > 0.0:
        rooms = np.array([room(pose_at(float(s))) for s in np.linspace(0.0, 1.0, ROOM_SAMPLES + 1)])
        reach = int(math.ceil(SLOW_AHEAD / (length / ROOM_SAMPLES) - 1e-9))
        ahead = np.array([rooms[k:k + reach + 1].min() for k in range(rooms.size)])
        caps = np.array([speed_cap(float(r)) for r in ahead])
        out = capped_fractions(length, speed, accel * float(caps.min()), caps)
        if out is not None:
            return out
    duration, distance = trapezoid(length, speed, accel)
    return fractions(duration, distance, length)


def hand_path(start: np.ndarray, goal: np.ndarray, room=None, speed: float = ARM_SPEED) -> list[np.ndarray]:
    """Poses (4x4) at the end of each step: a straight line while turning about one axis, at most
    ``speed`` (m/s; the turning slowed in proportion); slower where the way ahead leaves the elbow
    little ``room`` (of a pose: :meth:`Elbow.room`)."""
    move = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
    turn = rotvec(goal[:3, :3] @ start[:3, :3].T)
    angle = float(np.linalg.norm(turn))
    length = max(move, angle * ARM_SPEED / ARM_TURN)

    def pose_at(f: float) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, 3] = start[:3, 3] + f * (goal[:3, 3] - start[:3, 3])
        pose[:3, :3] = rotation(f * turn) @ start[:3, :3]
        return pose

    return [pose_at(float(f)) for f in _shares(length, pose_at, room, speed)]


def joint_path(sim, goal_joints, samples: int = 64, speed: float = 1.0) -> list[np.ndarray]:
    """Grip-site poses (frame of robot0_link0) at the end of each step while the arm's joints go in a
    straight line from where they are to ``goal_joints``, rest to rest at ``speed`` times the hand's
    speeds; the poses are the forward kinematics on a scratch copy of the state.  Unlike the hand's
    straight line, the joints stay within their limits on the way.  ``sim`` as for :func:`reach`, with
    ``link0_id``."""
    import mujoco

    m = sim.model
    scratch = mujoco.MjData(m)
    scratch.qpos[:] = sim.data.qpos
    qadr = np.array(sim.arm_qpos)
    q0 = scratch.qpos[qadr].copy()
    q1 = np.clip(np.asarray(goal_joints, float), sim.arm_limits[:, 0], sim.arm_limits[:, 1])

    def pose_at(s: float) -> np.ndarray:
        scratch.qpos[qadr] = q0 + s * (q1 - q0)
        mujoco.mj_kinematics(m, scratch)
        link0, site = np.eye(4), np.eye(4)
        link0[:3, :3] = scratch.xmat[sim.link0_id].reshape(3, 3)
        link0[:3, 3] = scratch.xpos[sim.link0_id]
        site[:3, :3] = scratch.site_xmat[sim.site_id].reshape(3, 3)
        site[:3, 3] = scratch.site_xpos[sim.site_id]
        return np.linalg.inv(link0) @ site

    # the hand's way along the joints' line, in the units of hand_path: metres, or turning at ARM_TURN
    s = np.linspace(0.0, 1.0, samples + 1)
    coarse = [pose_at(float(v)) for v in s]
    legs = [max(float(np.linalg.norm(b[:3, 3] - a[:3, 3])), turn_between(a[:3, :3], b[:3, :3]) * ARM_SPEED / ARM_TURN)
            for a, b in zip(coarse, coarse[1:])]
    done = np.concatenate([[0.0], np.cumsum(legs)])
    length = float(done[-1])
    duration, distance = trapezoid(length, speed * ARM_SPEED, speed * ARM_ACCEL)
    poses = []
    for f in fractions(duration, distance, length):
        i = int(np.clip(np.searchsorted(done, f * length), 1, samples))
        w = (f * length - done[i - 1]) / max(1e-12, done[i] - done[i - 1])
        poses.append(pose_at(float(s[i - 1] + w * (s[i] - s[i - 1]))))
    return poses


def arc_path(start: np.ndarray, center, axis, angle: float, room=None,
             speed: float = ARM_SPEED) -> list[np.ndarray]:
    """Poses (4x4) at the end of each step turning ``start`` by ``angle`` (rad) about the line through
    ``center`` along ``axis``; the hand turns with it, as it would holding a door's handle.  ``room``
    and ``speed`` as for :func:`hand_path`."""
    axis = np.asarray(axis, float) / max(float(np.linalg.norm(axis)), 1e-12)
    center = np.asarray(center, float)
    arm = start[:3, 3] - center
    radius = float(np.linalg.norm(arm - (arm @ axis) * axis))
    length = abs(angle) * max(radius, ARM_SPEED / ARM_TURN)

    def pose_at(f: float) -> np.ndarray:
        r = rotation(axis * angle * f)
        pose = np.eye(4)
        pose[:3, :3] = r @ start[:3, :3]
        pose[:3, 3] = center + r @ arm
        return pose

    return [pose_at(float(f)) for f in _shares(length, pose_at, room, speed)]


def fit_circle(points, prior, pull: float = FIT_PULL, noise: float = 0.001,
               iterations: int = 10) -> tuple[np.ndarray, float]:
    """Centre (2-D) and radius of the circle through ``points`` [n, 2] (sigma ``noise``), the centre
    pulled toward ``prior`` (sigma ``pull``): a short arc of track alone does not fix it."""
    pts = np.asarray(points, float)
    centre = np.asarray(prior, float).copy()
    radius = float(np.mean(np.linalg.norm(pts - centre, axis=1)))
    w = noise / pull
    n = len(pts)
    for _ in range(iterations):
        d = pts - centre
        dist = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
        jac = np.zeros((n + 2, 3))
        jac[:n, :2] = -d / dist[:, None]
        jac[:n, 2] = -1.0
        jac[n:, :2] = w * np.eye(2)
        res = np.concatenate([dist - radius, w * (centre - prior)])
        step = np.linalg.lstsq(jac, -res, rcond=None)[0]
        centre, radius = centre + step[:2], radius + float(step[2])
    return centre, radius


# -- the arm -----------------------------------------------------------------------------------------
class Arm:
    """The arm and gripper of one episode.  Rebuild after every reset (it rebuilds the robot).

    ``sim`` gives ``model`` / ``data`` (MuJoCo), ``site_id`` (the grip site), ``link0_id``,
    ``link0()`` and ``site_pose()`` (4x4 in the episode's frame), ``tcp()``, ``finger_width()``,
    ``finger_speed()``, ``steps``, ``over`` and ``step(action)``.  Subclasses build the environment's
    action (:meth:`action`) and may add numbers to each step's record row (:meth:`row_extra`).
    """

    #: numbers per record row: step, grasp point xyz, hand rotation vector, finger gap, then the extras
    ROW = 8
    #: settling after a motion's plan stops within this lag (m, rad) once the hand rests, or when the lag
    #: shrank by less than STALL_SHARE over this many steps
    settle_tol = SETTLE
    stall_steps = STALL_STEPS
    #: whether settling also waits for the hand to rest (REST) once it is within settle_tol
    settle_rest = True

    def __init__(self, sim, osc, gripper_model, log=None):
        self.sim = sim
        self.log = log or (lambda row: None)
        if osc.name != "OSC_POSE" or osc.input_type != "delta":
            raise RuntimeError(f"unexpected arm controller: {osc.name} {osc.input_type}")
        self.osc = osc
        self.gripper_model = gripper_model
        self.scale = np.abs(osc.output_max - osc.output_min) / np.abs(osc.input_max - osc.input_min)
        self.out_mid = (osc.output_max + osc.output_min) / 2.0
        self.in_mid = (osc.input_max + osc.input_min) / 2.0
        self.integral = np.zeros(6)
        self.elbow = Elbow(sim)
        self.motors, self.torque_max = arm_motors(sim.model, sim.arm_dof)
        #: hold steps that gave way (see :meth:`hold_step`), how many of them because the elbow
        #: straightened (the others strained), whether one went back by the joints, and how far (m) the
        #: give-ways moved the hand; whoever reports them resets them (:meth:`count_anew`)
        self.gave_way = 0
        self.gave_way_stretched = 0
        self.gave_way_back = False
        self.gave_way_m = 0.0
        #: the joints where the last motion started: the way back of a hold whose elbow straightens
        self.came_from = sim.arm_joints()
        #: of the last give-way: (whether it went back by the joints, how far (m) the hand moved)
        self.last_give_way = (False, 0.0)
        _, site = self.site_now()
        self.hold = site  # grip site pose to hold, frame of robot0_link0
        self.start_site = site.copy()
        self.grip_goal = self.level()  # the level the gripper is kept at
        self.history: list[np.ndarray] = []

    # -- what subclasses fill in -----------------------------------------------------------------
    def action(self, arm6: np.ndarray, grip: float, **extra) -> np.ndarray:
        raise NotImplementedError

    def row_extra(self) -> list[float]:
        return []

    # -- readings ------------------------------------------------------------------------------
    def site_now(self) -> tuple[np.ndarray, np.ndarray]:
        """(robot0_link0 in the episode frame, grip site in the frame of robot0_link0)."""
        link0 = self.sim.link0()
        return link0, np.linalg.inv(link0) @ self.sim.site_pose()

    def velocity(self) -> tuple[np.ndarray, np.ndarray]:
        """Linear and angular velocity of the grip site relative to robot0_link0, in its frame."""
        import mujoco

        m, d, sim = self.sim.model, self.sim.data, self.sim
        site, body = np.zeros(6), np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, sim.site_id, site, 0)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, sim.link0_id, body, 0)
        rot = d.xmat[sim.link0_id].reshape(3, 3)
        arm = d.site_xpos[sim.site_id] - d.xpos[sim.link0_id]
        linear = site[3:] - body[3:] - np.cross(body[:3], arm)
        return rot.T @ linear, rot.T @ (site[:3] - body[:3])

    def resting(self) -> bool:
        v, w = self.velocity()
        return float(np.linalg.norm(v)) < REST[0] and float(np.linalg.norm(w)) < REST[1]

    def level(self) -> float:
        return float(np.asarray(self.gripper_model.current_action).reshape(-1)[-1])

    def bend(self) -> float:
        """How far (rad) the elbow is bent short of straight (negative: bent backwards past it)."""
        return self.elbow.straight - float(self.sim.arm_joints()[ELBOW_J])

    def straining(self) -> bool:
        """A motor of the arm at its torque limit (STRAIN) with the elbow bent less than STRAIN_BEND."""
        torque = np.abs(self.sim.data.ctrl[self.motors]) / self.torque_max
        return bool(torque.max() >= STRAIN) and self.bend() < STRAIN_BEND

    def joints_still(self) -> bool:
        return bool(np.all(np.abs(self.sim.data.qvel[self.sim.arm_dof]) < REST[1]))

    # -- one step ------------------------------------------------------------------------------
    def _grip_sign(self) -> float:
        level = self.level()
        if self.grip_goal > level + 1e-6:
            return 1.0
        if self.grip_goal < level - 1e-6:
            return -1.0
        return 0.0

    def arm_command(self, previous: np.ndarray, target: np.ndarray, following: np.ndarray) -> np.ndarray:
        """The OSC part of the action for a step whose planned end pose is ``target`` (link0 frame;
        ``previous`` / ``following``: the planned poses a step before and after)."""
        osc = self.osc
        _, now = self.site_now()
        kp, kd = float(osc.kp[0]), float(osc.kd[0])
        kp_r, kd_r = float(osc.kp[3]), float(osc.kd[3])
        vel = (following[:3, 3] - previous[:3, 3]) / (2.0 * DT)
        acc = (following[:3, 3] - 2.0 * target[:3, 3] + previous[:3, 3]) / DT ** 2
        w_in = rotvec(target[:3, :3] @ previous[:3, :3].T) / DT
        w_out = rotvec(following[:3, :3] @ target[:3, :3].T) / DT
        spin, spin_acc = 0.5 * (w_in + w_out), (w_out - w_in) / DT
        error = np.concatenate([target[:3, 3] - now[:3, 3], rotvec(target[:3, :3] @ now[:3, :3].T)])
        linear, angular = self.velocity()
        still_plan = float(np.linalg.norm(vel)) < 1e-9 and float(np.linalg.norm(spin)) < 1e-9
        if still_plan and float(np.linalg.norm(linear)) < REST[0] and float(np.linalg.norm(angular)) < REST[1]:
            for part, zone, cap in ((slice(0, 3), I_ZONE[0], I_MAX[0]), (slice(3, 6), I_ZONE[1], I_MAX[1])):
                if float(np.linalg.norm(error[part])) < zone:
                    self.integral[part] = _clip_norm(self.integral[part] + KI * error[part], cap)
        goal_p = target[:3, 3] + FEEDFORWARD * (kd * vel + acc) / kp + self.integral[:3] - DAMP * (linear - vel)
        goal_r = rotation(FEEDFORWARD * (kd_r * spin + spin_acc) / kp_r + self.integral[3:]
                          - DAMP * (angular - spin)) @ target[:3, :3]
        # the OSC takes its deltas in the frame of its origin site, whose orientation it keeps in MuJoCo's
        # world frame: turn link0-frame vectors with link0's world orientation (not the episode frame's)
        link0_world = self.sim.data.xmat[self.sim.link0_id].reshape(3, 3)
        to_origin = np.asarray(osc.origin_ori, float).T @ link0_world
        delta = np.concatenate([to_origin @ (goal_p - now[:3, 3]), to_origin @ rotvec(goal_r @ now[:3, :3].T)])
        return np.clip((delta - self.out_mid) / self.scale + self.in_mid, osc.input_min, osc.input_max)

    def step(self, arm6: np.ndarray, **extra) -> bool:
        alive = self.sim.step(self.action(np.asarray(arm6, float), self._grip_sign(), **extra))
        self._record()
        return alive

    def count_anew(self) -> None:
        self.gave_way = self.gave_way_stretched = 0
        self.gave_way_back, self.gave_way_m = False, 0.0

    def hold_step(self, **extra) -> tuple[int, bool]:
        """A step holding the pose (``extra``: the benchmark's other parts of the action).  Straining, or
        with the elbow straightening too far (a hold far out creeps outward), the arm gives way
        (:meth:`give_way`, its steps without ``extra``; stretched, back toward where the last motion
        started) and holds where that leaves it (``gave_way`` and the others count it).  Returns
        (steps, alive)."""
        margin = self.elbow.margin(self.bend())
        alive = self.step(self.arm_command(self.hold, self.hold, self.hold), **extra)
        if not alive:
            return 1, alive
        strained = self.straining()
        if not strained and not self._stretching(margin)[0]:
            return 1, alive
        self.gave_way += 1
        self.gave_way_stretched += not strained
        steps, alive = self.give_way(None if strained else self.came_from)
        back, moved = self.last_give_way
        self.gave_way_back = self.gave_way_back or back
        self.gave_way_m += moved
        return 1 + steps, alive

    def give_way(self, back: np.ndarray | None = None) -> tuple[int, bool]:
        """With ``back`` (the joints where the motion started), go back along the joints' line toward
        them until the elbow is GIVE_WAY_BEND more bent, or all the way (at GIVE_WAY_SPEED); without it,
        or when that would bend the elbow less than GIVE_WAY_MIN, pull the hand GIVE_WAY_M straight
        toward the shoulder (its turn kept).  Then hold there until the joints rest (GIVE_WAY_STEPS at
        most); the arm holds where that leaves it.  Returns (steps, alive); ``last_give_way`` says how
        it went."""
        self.integral[:] = 0.0
        _, now = self.site_now()
        gain = None if back is None else (self.elbow.straight - float(back[ELBOW_J])) - self.bend()
        by_joints = gain is not None and gain >= GIVE_WAY_MIN
        if by_joints:
            q = self.sim.arm_joints()
            share = min(1.0, GIVE_WAY_BEND / gain)
            path = joint_path(self.sim, q + share * (np.asarray(back, float) - q), speed=GIVE_WAY_SPEED)
        else:
            toward = self.elbow.shoulder - now[:3, 3]
            pulled = now.copy()
            pulled[:3, 3] += GIVE_WAY_M * toward / max(1e-9, float(np.linalg.norm(toward)))
            path = hand_path(now, pulled)
        goal = path[-1] if path else now
        steps, alive = 0, not self.sim.over
        for i, pose in enumerate(path):
            if not alive:
                break
            before = path[i - 1] if i > 0 else now
            after = path[i + 1] if i + 1 < len(path) else pose
            alive = self.step(self.arm_command(before, pose, after))
            steps += 1
        for _ in range(GIVE_WAY_STEPS):
            if not alive or self.joints_still():
                break
            alive = self.step(self.arm_command(goal, goal, goal))
            steps += 1
        self.integral[:] = 0.0
        _, self.hold = self.site_now()
        self.last_give_way = (by_joints, float(np.linalg.norm(self.hold[:3, 3] - now[:3, 3])))
        return steps, alive

    def unbend(self) -> dict:
        """With the elbow bent backwards past straight: move the grip site UNBEND_M out along the line
        from the shoulder to the wrist, its turn kept, not stopped for a strain (:meth:`follow`)."""
        d = self.sim.data
        link0_r = d.xmat[self.sim.link0_id].reshape(3, 3)
        out = link0_r.T @ (d.xanchor[self.elbow.wrist_id] - d.xpos[self.sim.link0_id]) - self.elbow.shoulder
        _, start = self.site_now()
        goal = start.copy()
        goal[:3, 3] += UNBEND_M * out / max(1e-9, float(np.linalg.norm(out)))
        return self.follow(hand_path(start, goal), start, guard=False, strain=False)

    def _record(self) -> None:
        tcp = self.sim.tcp()
        self.history.append(np.array([self.sim.steps, *tcp[:3, 3], *rotvec(tcp[:3, :3]), self.sim.finger_width(),
                                      *self.row_extra()], dtype=np.float32))

    def take_history(self) -> np.ndarray:
        rows = np.stack(self.history) if self.history else np.zeros((0, self.ROW), np.float32)
        self.history = []
        return rows

    # -- arm -------------------------------------------------------------------------------------
    def move(self, goal_site: np.ndarray) -> dict:
        """Move the grip site to ``goal_site`` (frame of robot0_link0) on a straight line."""
        _, start = self.site_now()
        return self.follow(hand_path(start, goal_site, self.elbow.room), start)

    def _stretching(self, margin_before: float) -> tuple[bool, float]:
        """(Whether the elbow straightens too far, the wrist's margin now): ``margin_before`` was a step
        ago (see :meth:`Elbow.margin`).  Within STRETCH_STOP of straight, or within it STRETCH_AHEAD from
        now at this step's speed, the wrist moving out; never once the elbow is past straight (pulling the
        hand in would then bend it on backwards)."""
        bend = self.bend()
        margin = self.elbow.margin(bend)
        out = (margin_before - margin) / DT
        return bend > 0.0 and out > 0.0 and margin - STRETCH_AHEAD * out < self.elbow.margin(STRETCH_STOP), margin

    def follow(self, path: list[np.ndarray], start: np.ndarray, guard: bool = True, strain: bool = True) -> dict:
        """Run a plan of grip-site poses (frame of robot0_link0, one per step from ``start``), then
        settle at its last pose.  ``strain``: a strain (STRAIN) stops it and the arm gives way (``jammed``,
        which counts as blocked).  ``guard``: the elbow straightening too far (STRETCH_STOP) stops it too
        (``stretched``), and the arm gives way back toward the joints it started from."""
        goal_site = path[-1] if path else start
        self.came_from = self.sim.arm_joints()
        self.integral[:] = 0.0
        steps, behind, blocked, alive = 0, 0, False, not self.sim.over
        stretched, jammed, margin = False, False, self.elbow.margin(self.bend())
        for i, pose in enumerate(path):
            if not alive:
                break
            before = path[i - 1] if i > 0 else start
            after = path[i + 1] if i + 1 < len(path) else pose
            alive = self.step(self.arm_command(before, pose, after))
            steps += 1
            if strain and self.straining():
                jammed = True
                break
            stretched, margin = self._stretching(margin)
            if guard and stretched:
                break
            stretched = False
            lag_m, lag_r = self._lag(pose)
            if lag_m > VERY_FAR[0] or lag_r > VERY_FAR[1] or self._not_keeping_up(lag_m, lag_r, before, after):
                behind += 1
            else:
                behind = 0
            if behind >= BLOCKED_STEPS:
                blocked = True
                break
        settled, stalled = 0, False
        if not blocked and not stretched and not jammed:
            scores: list[float] = []
            tol, stall = self.settle_tol, self.stall_steps
            far_before = None
            for _ in range(SETTLE_STEPS):
                if not alive:
                    break
                lag_m, lag_r = self._lag(goal_site)
                if lag_m < tol[0] and lag_r < tol[1] and (not self.settle_rest or self.resting()):
                    break
                # still far from the goal after the plan and not closing in (something holds the hand
                # back): blocked, the count going on from the plan's last steps
                far = lag_m / FAR_BEHIND[0] + lag_r / FAR_BEHIND[1]
                if (lag_m > FAR_BEHIND[0] or lag_r > FAR_BEHIND[1]) \
                        and not (far_before is not None and far < CLOSING * far_before):
                    behind += 1
                    if behind >= BLOCKED_STEPS:
                        blocked = True
                        break
                else:
                    behind = 0
                far_before = far
                # the lag in units of the settle tolerance; no longer shrinking: stop waiting
                scores.append(lag_m / tol[0] + lag_r / tol[1])
                if len(scores) > stall and scores[-1] > STALL_SHARE * scores[-1 - stall]:
                    stalled = True
                    break
                alive = self.step(self.arm_command(goal_site, goal_site, goal_site))
                steps += 1
                settled += 1
                if strain and self.straining():
                    jammed = True
                    break
                stretched, margin = self._stretching(margin)
                if guard and stretched:
                    break
                stretched = False
        gave_way, (back, moved) = 0, (False, 0.0)
        if jammed or stretched:
            gave_way, alive = self.give_way(None if jammed else self.came_from)
            back, moved = self.last_give_way
            steps += gave_way
        err_m, err_r = self._lag(goal_site)
        reached = not blocked and not stretched and not jammed and err_m < ARRIVED[0] and err_r < ARRIVED[1]
        _, now = self.site_now()
        self.hold = goal_site.copy() if reached else now
        if not reached:
            self.integral[:] = 0.0
        return {"steps": steps, "planned": len(path), "settle": settled, "blocked": blocked or jammed,
                "jammed": jammed, "gave_way": gave_way, "gave_way_back": back, "gave_way_m": moved,
                "reached": reached, "stalled": stalled and not reached,
                "stretched": stretched, "bend_deg": math.degrees(self.bend()), "err_m": err_m,
                "err_deg": math.degrees(err_r), "over": self.sim.over}

    def _lag(self, target: np.ndarray) -> tuple[float, float]:
        _, now = self.site_now()
        return (float(np.linalg.norm(target[:3, 3] - now[:3, 3])), turn_between(now[:3, :3], target[:3, :3]))

    def _not_keeping_up(self, lag_m, lag_r, before, after) -> bool:
        """Behind the plan and not catching up: the hand's speed along the planned direction (of
        travel, of turning) is below KEEP_UP of the plan's.  Moving fast the wrong way (pushed back by
        something) is not keeping up."""
        move = after[:3, 3] - before[:3, 3]
        turn = rotvec(after[:3, :3] @ before[:3, :3].T)
        want_v = float(np.linalg.norm(move)) / (2.0 * DT)
        want_w = float(np.linalg.norm(turn)) / (2.0 * DT)
        v, w = self.velocity()
        slow_v = want_v < 1e-6 or float(v @ move) / (2.0 * DT * want_v) < KEEP_UP * want_v
        slow_w = want_w < 1e-6 or float(w @ turn) / (2.0 * DT * want_w) < KEEP_UP * want_w
        return (lag_m > BEHIND[0] and slow_v) or (lag_r > BEHIND[1] and slow_w)

    def push_arc(self, center: np.ndarray, axis: np.ndarray, angle: float, push: np.ndarray, press: float,
                 tilt: float = 0.0) -> dict:
        """Push the hinged part the hand touches around the line through ``center`` along ``axis`` (frame of
        robot0_link0) by up to ``angle`` (rad; its sign the way round; see PRESS_CHUNK).  First the hand goes
        along ``push`` until it touches; then each cycle's goal lies ``press`` past the grip site along
        ``push`` leaned ``tilt`` (rad) toward the way the part turns there (so that friction turns it too),
        turned by the part's angle so far, the hand turned with it.  The angle comes from the grip site's
        track about the line.  A strain or a stretched elbow stops it as it stops :meth:`follow`; the arm
        then holds where the hand is (it does not press on)."""
        a = np.asarray(axis, float) / float(np.linalg.norm(axis))
        u = np.asarray(push, float) / float(np.linalg.norm(push))
        c = np.asarray(center, float)
        way, end = (1.0 if angle >= 0.0 else -1.0), abs(float(angle))
        press = float(np.clip(press, *PRESS_RANGE))
        self.came_from = self.sim.arm_joints()
        self.integral[:] = 0.0
        _, start = self.site_now()
        turn0 = start[:3, :3].copy()
        steps, alive = 0, not self.sim.over
        stretched, jammed, margin = False, False, self.elbow.margin(self.bend())

        def run(goal: np.ndarray) -> bool:
            """PRESS_CHUNK steps holding ``goal``; False once a strain, a stretched elbow or the end stops them."""
            nonlocal steps, alive, stretched, jammed, margin
            for _ in range(PRESS_CHUNK):
                if not alive:
                    return False
                alive = self.step(self.arm_command(goal, goal, goal))
                steps += 1
                if self.straining():
                    jammed = True
                    return False
                stretched, margin = self._stretching(margin)
                if stretched:
                    return False
            return alive

        touched, touch_m, k, short = False, 0.0, 0, 0
        while touch_m < TOUCH_MAX and k < 2 * TOUCH_MAX / TOUCH_STEP:
            _, now = self.site_now()
            goal = now.copy()
            goal[:3, :3] = turn0
            goal[:3, 3] = now[:3, 3] + TOUCH_STEP * u
            if not run(goal):
                break
            moved = float((self.site_now()[1][:3, 3] - now[:3, 3]) @ u)
            touch_m += max(0.0, moved)
            k += 1
            short = short + 1 if k > 1 and moved < TOUCH_MOVED else 0  # the first cycle starts from rest
            if short >= 2:
                touched = True
                break
        _, now = self.site_now()
        radial = now[:3, 3] - c
        e1 = radial - a * float(radial @ a)
        e1 = e1 / max(float(np.linalg.norm(e1)), 1e-9)
        e2 = np.cross(a, e1)

        def plane(p: np.ndarray) -> np.ndarray:
            q = p - c
            return np.array([float(q @ e1), float(q @ e2)])

        # the press leans from the push toward the way the part turns where the hand met it
        ahead = way * np.cross(a, e1)
        ahead = ahead - u * float(ahead @ u)
        if float(np.linalg.norm(ahead)) > 1e-6:
            u = math.cos(tilt) * u + math.sin(tilt) * ahead / float(np.linalg.norm(ahead))
        track, hinge, contact = [plane(now[:3, 3])], np.zeros(2), now[:3, 3].copy()
        th = best = hand = 0.0
        best_at = raised_at = cycles = 0
        history, trace, drift = [], [], 0.0
        stop = None if touched else "no touch"
        while stop is None:
            if cycles >= PRESS_CYCLES:
                stop = "cycles"
                break
            _, now = self.site_now()
            q = plane(now[:3, 3])
            if float(np.linalg.norm(q - track[-1])) > TRACK_STEP:
                track.append(q)
            if len(track) >= 3:
                fit, radius = fit_circle(track, np.zeros(2))
                if float(np.linalg.norm(fit)) < FIT_SHIFT and radius > FIT_RADIUS:
                    hinge = fit
            v0, v1 = track[0] - hinge, q - hinge
            est = way * float(np.angle(np.exp(1j * (math.atan2(v1[1], v1[0]) - math.atan2(v0[1], v0[0])))))
            th = max(th - PRESS_BACK, min(max(est, 0.0), end + PRESS_BACK, th + PRESS_FOLLOW))
            history.append(th)
            drift = float(np.linalg.norm(v1)) - float(np.linalg.norm(v0))
            if cycles % 4 == 0:
                trace.append((round(math.degrees(th), 1), round(press, 3), round(100.0 * drift, 1),
                              round(100.0 * float(np.linalg.norm(hinge)), 1)))
            if abs(drift) > SLID_M:
                stop = "slid"
                break
            if th > best + PRESS_GAIN:
                best, best_at = th, cycles
            if th >= end:
                stop = "angle"
                break
            if len(history) > 4 and history[-1] - history[-5] < PRESS_SLOW and press < PRESS_RANGE[1] \
                    and cycles - raised_at >= 4:
                press, raised_at = min(PRESS_RANGE[1], press + PRESS_GROW), cycles
            if cycles - best_at >= PRESS_STALL and press >= PRESS_RANGE[1]:
                stop = "stalled"
                break
            # the hand turns with the part, unless it lags the turn it was last asked for
            if turn_between(now[:3, :3], rotation(a * way * hand) @ turn0) < TURN_LAG:
                hand = th
            goal = np.eye(4)
            goal[:3, :3] = rotation(a * way * hand) @ turn0
            goal[:3, 3] = now[:3, 3] + press * (rotation(a * way * th) @ u)
            # in the plane the part turns in, through where the hand met it (pressed at a slant, a hand
            # creeps along the hinge and off the part)
            goal[:3, 3] += a * float((contact - goal[:3, 3]) @ a)
            cycles += 1
            if not run(goal):
                break
        if jammed or stretched:
            stop = "jammed" if jammed else "stretched"
        elif stop is None or not alive:
            stop = "over"
        gave_way, (back, moved) = 0, (False, 0.0)
        if jammed or stretched:
            gave_way, alive = self.give_way(None if jammed else self.came_from)
            back, moved = self.last_give_way
            steps += gave_way
        _, self.hold = self.site_now()
        self.integral[:] = 0.0
        trace.append((round(math.degrees(th), 1), round(press, 3)))
        return {"steps": steps, "touched": touched, "touch_m": touch_m, "cycles": cycles, "angle": way * th,
                "stop": stop, "press": press, "hinge_shift_m": float(np.linalg.norm(hinge)), "drift_m": drift,
                "hand_angle": way * hand, "trace": trace, "blocked": jammed, "jammed": jammed,
                "stretched": stretched, "gave_way": gave_way, "gave_way_back": back, "gave_way_m": moved,
                "reached": stop == "angle", "bend_deg": math.degrees(self.bend()), "over": self.sim.over}

    # -- gripper ---------------------------------------------------------------------------------
    def set_grip(self, opening: float) -> None:
        """Start moving the gripper to ``opening`` (0 closed .. 1 open), in steps of 0.1."""
        level = 1.0 - 2.0 * float(np.clip(opening, 0.0, 1.0))
        self.grip_goal = float(np.clip(round(level / GRIP_STEP) * GRIP_STEP, -1.0, 1.0))

    def grip(self, opening: float) -> dict:
        """Move the gripper to ``opening`` and wait until the fingers stop."""
        self.set_grip(opening)
        steps, alive = 0, not self.sim.over
        self.count_anew()
        while alive and steps < GRIP_STEPS_MAX:
            at_goal = abs(self.level() - self.grip_goal) < 1e-6
            if at_goal and steps >= 2 and self.sim.finger_speed() < FINGERS_STILL:
                break
            n, alive = self.hold_step()
            steps += n
        return {"steps": steps, "width": self.sim.finger_width(), "level": self.level(), "goal": self.grip_goal,
                **self.gave_way_counts(), "over": self.sim.over}

    def wait(self, n: int) -> dict:
        steps, alive = 0, not self.sim.over
        self.count_anew()
        while alive and steps < n:
            k, alive = self.hold_step()
            steps += k
        return {"steps": steps, **self.gave_way_counts(), "over": self.sim.over}

    def gave_way_counts(self) -> dict:
        """How the holds since :meth:`count_anew` gave way, for a reply."""
        return {"gave_way": self.gave_way, "stretched": self.gave_way_stretched, "gave_way_back": self.gave_way_back,
                "gave_way_m": self.gave_way_m}


# -- checks and reachability -----------------------------------------------------------------------------
def check_tool_axes(model, data, site_id: int, site_to_tool: np.ndarray = PANDA_SITE_TO_TOOL,
                    pads=PANDA_PADS, hand: str = PANDA_HAND) -> None:
    """The tool axes must match the hand: y along the line between the finger pads, x pointing from
    the hand out between them."""
    m, d = model, data
    ends = [d.geom_xpos[m.geom(name).id].copy() for name in pads]
    site = d.site_xpos[site_id]
    tool = d.site_xmat[site_id].reshape(3, 3) @ site_to_tool
    line = (ends[0] - ends[1]) / max(1e-9, float(np.linalg.norm(ends[0] - ends[1])))
    base = d.xpos[m.body(hand).id]
    out = (site - base) / max(1e-9, float(np.linalg.norm(site - base)))
    if abs(float(line @ tool[:, 1])) < 0.95 or float(out @ tool[:, 0]) < 0.95:
        raise RuntimeError(f"tool axes do not match the gripper: finger line . y = {float(line @ tool[:, 1]):.2f}, "
                           f"hand->site . x = {float(out @ tool[:, 0]):.2f}")


def reach(sim, goal_site: np.ndarray, iterations: int = 200, damping: float = 0.05,
          start: np.ndarray | None = None) -> dict:
    """Whether the arm can put the grip site at ``goal_site`` (episode frame) with everything else
    where it is: damped least squares over the seven arm joints within their limits, in small steps,
    on a scratch copy of the simulation state.  The search starts from the joints now; when that
    does not get there (it can stall against a joint limit), again from the joints now shaken a
    little.  Starts far from the joints now find poses the arm cannot move into from where it is
    (the controller only follows the hand's path), so there are none.  ``sim`` adds ``to_start``
    (world -> episode frame), ``arm_qpos``, ``arm_dof`` and ``arm_limits``.  ``start``: joints to
    search from instead of the joints now (the arm moved there first, as ``home`` moves it).

    ``progress``: how much of the way from the hand's pose now the best search got (0: none)."""
    import mujoco

    m, d = sim.model, sim.data
    scratch = mujoco.MjData(m)
    scratch.qpos[:] = d.qpos
    world = np.linalg.inv(sim.to_start) @ goal_site
    qadr = np.array(sim.arm_qpos)
    dadr = np.array(sim.arm_dof)
    low, high = sim.arm_limits[:, 0], sim.arm_limits[:, 1]
    now = d.qpos[qadr].copy() if start is None else np.clip(np.asarray(start, float), low, high)
    starts = [now]
    shake = np.random.default_rng(0)
    while len(starts) < IK_STARTS:
        starts.append(np.clip(now + shake.normal(0.0, IK_SHAKE, now.size), low, high))
    first = best = None
    for tries, q0 in enumerate(starts, 1):
        found = _solve(m, scratch, sim.site_id, world, qadr, dadr, low, high, q0, iterations, damping)
        first = first or found
        if best is None or found[0] + 0.1 * found[1] < best[0] + 0.1 * best[1]:
            best = found
        if best[0] < ARRIVED[0] and best[1] < ARRIVED[1]:
            break
    e_m, e_r, q, _ = best
    s_m, s_r = first[3]  # the error at the joints now
    margin = np.minimum(q - low, high - q)
    progress = 1.0 - (e_m + 0.1 * e_r) / max(1e-9, s_m + 0.1 * s_r)
    return {"ok": bool(e_m < ARRIVED[0] and e_r < ARRIVED[1]), "err_m": e_m, "err_deg": math.degrees(e_r),
            "joints": q, "limit_margin": float(margin.min()), "tries": tries, "progress": float(progress)}


#: indices among the arm's joints of the Panda's joints 2 (shoulder), 4 (elbow), 6 (wrist) and 7
SHOULDER_J, ELBOW_J, WRIST_J, LAST_J = 1, 3, 5, 6


class Elbow:
    """How straight the Panda's elbow is, from the model's geometry.

    The distance from the shoulder (joint 2's anchor) to the wrist (joint 6's anchor) depends on the
    elbow's angle (joint 4) alone and peaks where the arm is straight (``straight``, well inside the
    joint's travel: beyond it the elbow bends backwards).  With the grip site at a pose, joint 7's axis
    is fixed and the wrist can be anywhere on a circle about it, so the distance lies between two
    bounds (:meth:`spread`).  The controller keeps the wrist near the nearest of them: the elbow about as
    bent as the hand's pose allows (:meth:`room`; in replayed approaches 0.1 to 6 deg less, never more).
    Near straight that changes fast: the margin to straight grows with the square of the bend (about
    6 mm at 15 deg, 24 mm at 30, 43 mm at 40), so a hand a centimetre farther out can take 10 deg off.
    ``sim`` as for :func:`reach`."""

    def __init__(self, sim, samples: int = 721):
        import mujoco

        m = sim.model
        scratch = mujoco.MjData(m)
        scratch.qpos[:] = sim.data.qpos
        qadr = np.array(sim.arm_qpos)
        ids = [int(np.flatnonzero(m.jnt_qposadr == a)[0]) for a in qadr]
        shoulder, wrist, last = ids[SHOULDER_J], ids[WRIST_J], ids[LAST_J]
        #: joint 6, whose anchor is the wrist
        self.wrist_id = wrist
        mujoco.mj_kinematics(m, scratch)
        link0_r = scratch.xmat[sim.link0_id].reshape(3, 3)
        #: the shoulder in the frame of robot0_link0
        self.shoulder = link0_r.T @ (scratch.xanchor[shoulder] - scratch.xpos[sim.link0_id])
        # in the grip site's frame: joint 7's axis, and the circle of the wrist about it
        site_r, site_p = scratch.site_xmat[sim.site_id].reshape(3, 3), scratch.site_xpos[sim.site_id]
        axis = site_r.T @ scratch.xaxis[last]
        self.axis = axis / np.linalg.norm(axis)
        on_axis = site_r.T @ (scratch.xanchor[last] - site_p)
        w = site_r.T @ (scratch.xanchor[wrist] - site_p)
        self.center = on_axis + float((w - on_axis) @ self.axis) * self.axis
        self.radius = float(np.linalg.norm(w - self.center))
        # the shoulder-wrist distance against the elbow's angle
        low, high = sim.arm_limits[ELBOW_J]
        grid = np.linspace(low, high, samples)
        dist = np.zeros(samples)
        for k, v in enumerate(grid):
            scratch.qpos[qadr[ELBOW_J]] = v
            mujoco.mj_kinematics(m, scratch)
            dist[k] = np.linalg.norm(scratch.xanchor[wrist] - scratch.xanchor[shoulder])
        top = int(np.argmax(dist))
        #: the elbow's angle (rad) where the arm is straight, and the shoulder-wrist distance there
        self.straight, self.longest = float(grid[top]), float(dist[top])
        self._grid, self._dist = grid[:top + 1], np.maximum.accumulate(dist[:top + 1])

    def spread(self, site: np.ndarray) -> tuple[float, float]:
        """Nearest and farthest the wrist can be from the shoulder with the grip site at ``site``
        (frame of robot0_link0)."""
        v = site[:3, :3].T @ (self.shoulder - site[:3, 3]) - self.center
        along = float(v @ self.axis)
        across = float(np.linalg.norm(v - along * self.axis))
        return math.hypot(along, across - self.radius), math.hypot(along, across + self.radius)

    def angle(self, distance: float) -> float:
        """The elbow's angle (rad) bent so that the shoulder and the wrist are ``distance`` apart (the
        straight angle when that is as far as they get)."""
        return float(np.interp(distance, self._dist, self._grid))

    def room(self, site: np.ndarray) -> float:
        """The most (rad) the elbow can be bent short of straight with the grip site at ``site`` (frame of
        robot0_link0): the wrist nearest the shoulder.  The controller keeps the elbow there."""
        return self.straight - self.angle(self.spread(site)[0])

    def margin(self, bend: float) -> float:
        """How much nearer (m) the wrist is to the shoulder than with the arm straight, the elbow bent
        ``bend`` (rad) short of straight, either way."""
        return self.longest - float(np.interp(self.straight - abs(bend), self._grid, self._dist))


#: the inverse kinematics: largest change of the joints per iteration (rad), how many starting points,
#: how much the joints now are shaken for all but the first (rad)
IK_STEP = 0.1
IK_STARTS = 4
IK_SHAKE = 0.1


def _solve(m, scratch, site_id: int, world: np.ndarray, qadr, dadr, low, high, q0, iterations: int,
           damping: float):
    """(error m, error rad, joints) of the best pose found from joints ``q0``, and the error at ``q0``."""
    import mujoco

    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    scratch.qpos[qadr] = q0
    best, start = (np.inf, np.inf, np.array(q0, float)), None
    for _ in range(iterations):
        mujoco.mj_kinematics(m, scratch)
        mujoco.mj_comPos(m, scratch)
        p = scratch.site_xpos[site_id]
        r = scratch.site_xmat[site_id].reshape(3, 3)
        err = np.concatenate([world[:3, 3] - p, rotvec(world[:3, :3] @ r.T)])
        e_m, e_r = float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))
        start = start or (e_m, e_r)
        if e_m + 0.1 * e_r < best[0] + 0.1 * best[1]:
            best = (e_m, e_r, scratch.qpos[qadr].copy())
        if e_m < 1e-4 and e_r < 1e-3:
            break
        mujoco.mj_jacSite(m, scratch, jp, jr, site_id)
        jac = np.vstack([jp[:, dadr], jr[:, dadr]])
        dq = _clip_norm(jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(6), err), IK_STEP)
        scratch.qpos[qadr] = np.clip(scratch.qpos[qadr] + dq, low, high)
    return (*best, start)


def arm_joints(model, prefix: str = "robot0_joint", count: int = 7):
    """(qpos addresses, dof addresses, limits (n x 2)) of the arm's joints."""
    qpos, dof, limits = [], [], []
    for k in range(1, count + 1):
        j = model.joint(f"{prefix}{k}").id
        qpos.append(int(model.jnt_qposadr[j]))
        dof.append(int(model.jnt_dofadr[j]))
        limits.append(model.jnt_range[j])
    return qpos, dof, np.array(limits, float)


def arm_motors(model, dof) -> tuple[np.ndarray, np.ndarray]:
    """(actuator ids, torque limits) of the motors that drive the arm's joints (dof addresses ``dof``)."""
    ids, limits = [], []
    for j in model.dof_jntid[np.asarray(dof, int)]:
        found = np.flatnonzero((model.actuator_trntype == 0) & (model.actuator_trnid[:, 0] == j))
        if not len(found):
            raise RuntimeError(f"no motor drives the joint {model.joint(int(j)).name}")
        ids.append(int(found[0]))
        limits.append(float(np.max(np.abs(model.actuator_ctrlrange[found[0]]))))
    return np.array(ids), np.array(limits)
