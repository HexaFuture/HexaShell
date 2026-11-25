"""HITCH-style body commands carried out with SIMPLE's whole-body controllers: AMO (the MP tasks,
``AmoExecutor``) and GR00T's decoupled WBC (the teleoperation tasks, ``WbcExecutor``).

Runs in the server process with :mod:`.sim`.  Each command closes the loop on the simulator's own
measurements (pelvis, torso, palms) and returns what was asked and what was reached.  HITCH's execute
carries out its parts one after another: body, hands, fingers, walk, gaze (``execute``).  The two
controllers share everything but their step command (``Executor``): each has its own way of asking for a
heading, a pelvis height and a torso pitch, and its own ranges for them.

AMO (SIMPLE robots/policy/AMO_Policy.py) takes at every 0.02 s step
[vx, target_yaw, vy, d_height, torso_yaw, torso_pitch, torso_roll, turn_flag] for the legs, the three
waist joints' targets (equal to the torso angles: SIMPLE's evaluation agents send them so) and targets
for the 14 arm and 14 hand joints.  It stands in place while |vx| and |vy| are below 0.1 m/s and
turn_flag below 0.5 (it then ignores the heading); target_yaw is the heading relative to the pelvis
IMU's orientation at the episode's first step; the pelvis height asked for is 0.75 m + d_height.
Measured open loop: it walks 29-94 % of the distance asked, squats 63-74 % of the depth asked
and leans 113-116 % of the pitch asked, so every command here is closed on measurements.

AMO also reads the shoulder and elbow joints (positions and speeds).  Fast arm motions upset it: with
arm commands jumping up to 0.35 rad in a step it slid 2.7 m and turned 110 deg while "standing",
while the same arm postures held still moved it at most 3 cm.  Arm commands are therefore limited to
ARM_SPEED, and the arm's redundancy is pulled toward AMO's training mean posture.

The decoupled WBC (.wbc.SimClockWbc, on simulated time) takes at every step the 31 upper-body targets
(waist, arms, fingers), a base height command (m, absolute; the teleoperation range 0.2-0.74) and a walking
command (vx, vy, turn flag, target heading).  Its lower body is an RL policy with a standing and a walking
network: it stands while |(vx, vy, flag)| < 0.1; with the flag at 0.1 or more it turns toward the target
heading (absolute: the pelvis's yaw in the world) at the heading error / 0.5 s, at most 1 rad/s.  It turns
the waist targets into the torso orientation it keeps.  Here the flag is on while walking (the heading is
held) and while turning.

Posture: a pelvis height or torso pitch once asked for is held (a slow loop at every step) until the next
body command changes it; a part never asked for is left to AMO (its natural lean when squatting).
Hands: after a hand command the arm's joints stay put, so the palm follows the torso; a hand command can
instead ask the palm to stay where it is in the world or relative to the pelvis while the body moves
(HITCH's hold modes), which later body commands then keep by inverse kinematics.

Coordinates are world coordinates (SIMPLE's); HITCH's observation frame O is ``sim.frame()``.
"""

from __future__ import annotations

import math

import numpy as np

from . import protocol as P
from .sim import FINGER_AXIS, HAND_SQUEEZE, PALM_NORMAL, PALM_POINT, SimpleSim

DT = P.STEP_S
WAIST = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
ARM = {s: tuple(f"{s}_{j}_joint" for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                                          "wrist_roll", "wrist_pitch", "wrist_yaw")) for s in P.SIDES}
HAND = {s: tuple(f"{s}_hand_{j}_joint" for j in ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1",
                                                "middle_0", "middle_1")) for s in P.SIDES}

#: the wide-open hand: thumb_1 swung back from the fingers (+ right, - left; the rest open), so that a hand
#: coming around a box's end does not meet it with the thumb (the end grips of the Teleop scripts)
THUMB_WIDE_RAD = 0.5
HAND_WIDE = {"right": (0.0, THUMB_WIDE_RAD, 0.0, 0.0, 0.0, 0.0, 0.0),
             "left": (0.0, -THUMB_WIDE_RAD, 0.0, 0.0, 0.0, 0.0, 0.0)}

#: AMO stands in place below this speed; walking commands are kept at or above WALK_MIN
DEAD_BAND = 0.1
WALK_MIN = 0.12
WALK_MAX = {"forward": 0.5, "back": 0.3, "side": 0.3}
WALK_GAIN = 1.5            # m/s per m of position error
#: the walking loop stops after 10 steps inside WALK_TOL_M; a walk is done when, after stopping, turning
#: and standing still, the pelvis is within WALK_DONE_M (else it walks again, at most WALK_ROUNDS times)
WALK_TOL_M = 0.03
WALK_DONE_M = 0.05
WALK_ROUNDS = 3
TURN_TOL = math.radians(3.0)
#: on walks shorter than FACE_TARGET_M a heading change above TURN_FIRST is made before walking; longer
#: walks more than FACE_OFF_AXIS off the heading are made facing the target
TURN_FIRST = math.radians(45.0)
FACE_TARGET_M = 0.8
FACE_OFF_AXIS = math.radians(30.0)
WALK_SETTLE_STEPS = 25
#: a WBC walk that has not come WALK_STALL_M closer to its goal in WALK_STALL_S is given up as blocked (by a
#: table's slab against the hips, say) instead of pushing on until its time is up
WALK_STALL_S = 1.5
WALK_STALL_M = 0.01
#: AMO's walks: when its legs come to stand after a walk, the pelvis
#: settles 2.2 cm back and 4.1 cm to the left of where the walking loop brought it (24 of 26 walks of 0.06-2.2 m,
#: forward, back, sideways and turning alike), so the loop aims that much the other way (AMO_SETTLE_OFFSET_M,
#: forward and left in the walk's final heading).  While it steps the pelvis sways 4-5 cm to each side over a
#: 1.5 s gait cycle, which the loop chased for many seconds near the target (a 0.25 m walk: 213 steps to come
#: within 5 cm, 580 in all; in a model episode the full 20 s), so a round also ends once its nearest approach
#: has not improved by WALK_STALL_M in AMO_NEAR_STALL_S when that approach came within WALK_NEAR_M (the legs then
#: stand, and the next round starts if the pelvis is still beyond WALK_DONE_M), or in AMO_FAR_STALL_S when it
#: did not, as blocked (AMO starts slowly: a 1 s command of 0.2 m/s from standing moves it 2 cm).  On the 26
#: walks: mean error 4.6 -> 1.7 cm (max 5.8 -> 4.3), time 228 -> 149 s in all
AMO_SETTLE_OFFSET_M = (-0.022, 0.041)
AMO_NEAR_STALL_S = 1.5
AMO_FAR_STALL_S = 2.5
WALK_NEAR_M = 0.08
#: the body command's loop moves the command by GAIN x error per step; HOLD_GAIN keeps it afterwards
HEIGHT_GAIN = 0.08
PITCH_GAIN = 0.08
HOLD_GAIN = 0.02
HEIGHT_TOL_M = 0.01
PITCH_TOL = math.radians(2.0)
#: a torso yaw closed on its measurement (WbcExecutor)
YAW_GAIN = 0.08
YAW_TOL = math.radians(2.0)
INSIDE_STEPS = 10
#: AMO's height command (d_height) and torso pitch command are kept inside these
D_HEIGHT_RANGE = (-0.45, 0.05)
TORSO_PITCH_CMD_RANGE = (-0.5, 1.1)
#: a body change is ramped at these rates before the loop corrects it
HEIGHT_RATE = 0.3          # m/s
PITCH_RATE = math.radians(60.0)
YAW_RATE = math.radians(60.0)
BODY_MAX_S = 4.0
#: arms: inverse kinematics on a copy of the robot's state, arm joints only
ARM_SPEED = 2.0            # rad/s, per joint, of the arm commands
SLOW_MAX = 4               # a hand segment may take up to this many times its asked time
IK_ITERS = 20
IK_STEP = 0.1              # rad per iteration
IK_DAMPING = 0.03
IK_ORIENT_WEIGHT = 0.4      # m per rad
IK_POSTURE_GAIN = 0.05      # pull toward REST in the arm's redundant directions, per iteration
#: AMO's training mean of the shoulder and elbow joints (adapter_norm_stats), wrists straight
REST = {"left": np.array([-0.1886, 0.4174, -0.2966, 0.6869, 0.0, 0.0, 0.0]),
        "right": np.array([-0.2080, -0.4032, 0.2750, 0.7165, 0.0, 0.0, 0.0])}
#: after the last waypoint the arm keeps tracking until the palm is within HAND_TOL_M for 5 steps
HAND_TOL_M = 0.01
HAND_SETTLE_MAX_STEPS = 40
HOLD_MODES = ("torso", "world", "pelvis")
FINGER_STEPS = 25


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q) -> float:
    """ZYX yaw of a wxyz quaternion."""
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_mul(a, b) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conj(q) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def align_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rotation vector (axis x angle) that turns unit vector ``current`` onto ``target``."""
    c = np.cross(current, target)
    s, d = np.linalg.norm(c), float(np.dot(current, target))
    angle = math.atan2(s, d)
    if s < 1e-9:
        if d > 0:
            return np.zeros(3)
        axis = np.cross(current, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(current, [0.0, 1.0, 0.0])
        return unit(axis) * math.pi
    return c / s * angle


def angle_between(a, b) -> float:
    return math.degrees(math.atan2(np.linalg.norm(np.cross(a, b)), float(np.dot(a, b))))


def rotate_toward(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """Unit vector a fraction ``s`` of the way from unit vector ``a`` to ``b`` on the great circle."""
    r = align_error(a, b) * s
    angle = np.linalg.norm(r)
    if angle < 1e-12:
        return a.copy()
    k = r / angle
    return a * math.cos(angle) + np.cross(k, a) * math.sin(angle) + k * np.dot(k, a) * (1 - math.cos(angle))


def min_jerk(s: float) -> float:
    s = min(1.0, max(0.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def limit_step(q_from: np.ndarray, q_to: np.ndarray, max_step: float) -> np.ndarray:
    """Move from q_from toward q_to by at most max_step in any joint (all joints scaled together)."""
    dq = q_to - q_from
    top = float(np.max(np.abs(dq)))
    return q_to.copy() if top <= max_step else q_from + dq * (max_step / top)


class ArmIK:
    """Damped least squares on the arm's seven joints, on a scratch copy of the robot's whole state (so
    the pelvis, waist and the other arm stay where they are now); the arm's redundant directions are
    pulled toward REST."""

    def __init__(self, sim: SimpleSim):
        import mujoco

        self.mujoco = mujoco
        self.sim = sim
        self.m = sim.model
        self.d = mujoco.MjData(self.m)
        self.qadr = {s: np.array([sim.qadr[j] for j in ARM[s]]) for s in P.SIDES}
        self.dadr = {s: np.array([sim.dadr[j] for j in ARM[s]]) for s in P.SIDES}
        self.lo = {s: np.array([sim.limits[j][0] for j in ARM[s]]) for s in P.SIDES}
        self.hi = {s: np.array([sim.limits[j][1] for j in ARM[s]]) for s in P.SIDES}
        self.jacp = np.zeros((3, self.m.nv))
        self.jacr = np.zeros((3, self.m.nv))

    def palm(self, side: str):
        i = self.sim.wrist_id[side]
        R = self.d.xmat[i].reshape(3, 3)
        return self.d.xpos[i] + R @ PALM_POINT[side], R @ PALM_NORMAL[side], R @ FINGER_AXIS

    def solve(self, side: str, point, normal=None, finger=None, q0=None,
              iters: int = IK_ITERS) -> tuple[np.ndarray, dict]:
        mj, m, d = self.mujoco, self.m, self.d
        d.qpos[:] = self.sim.data.qpos
        qa, da = self.qadr[side], self.dadr[side]
        q = np.array(d.qpos[qa] if q0 is None else q0, dtype=float)
        point = np.asarray(point, dtype=float)
        wid = self.sim.wrist_id[side]
        rest = REST[side]
        for _ in range(iters):
            d.qpos[qa] = q
            mj.mj_kinematics(m, d)
            mj.mj_comPos(m, d)
            p, n, f = self.palm(side)
            mj.mj_jac(m, d, self.jacp, self.jacr, p, wid)
            rows, errs = [self.jacp[:, da]], [point - p]
            er = np.zeros(3)
            if normal is not None:
                er += align_error(n, normal)
            if finger is not None:
                er += align_error(f, finger)
            if normal is not None or finger is not None:
                rows.append(IK_ORIENT_WEIGHT * self.jacr[:, da])
                errs.append(IK_ORIENT_WEIGHT * er)
            J, e = np.vstack(rows), np.concatenate(errs)
            if np.linalg.norm(e[:3]) < 5e-4 and (len(e) == 3 or np.linalg.norm(e[3:]) < 2e-3):
                break
            Jp = J.T @ np.linalg.inv(J @ J.T + IK_DAMPING ** 2 * np.eye(len(e)))
            dq = Jp @ e + (np.eye(7) - Jp @ J) @ (IK_POSTURE_GAIN * (rest - q))
            top = np.max(np.abs(dq))
            if top > IK_STEP:
                dq *= IK_STEP / top
            q = np.clip(q + dq, self.lo[side], self.hi[side])
        d.qpos[qa] = q
        mj.mj_kinematics(m, d)
        p, n, f = self.palm(side)
        res = {"position_error_m": float(np.linalg.norm(point - p))}
        if normal is not None:
            res["normal_error_deg"] = angle_between(n, normal)
        if finger is not None:
            res["finger_error_deg"] = angle_between(f, finger)
        at_limit = [ARM[side][k] for k in range(7)
                    if min(q[k] - self.lo[side][k], self.hi[side][k] - q[k]) < math.radians(1.0)]
        if at_limit:
            res["at_limit"] = at_limit
        return q, res


class Executor:
    """Holds the command the controller gets at every step and changes one part of it per command.
    A controller gives the step (``send``), its heading commands (``_turn_command``, ``_turn_end``,
    ``_walk_command``, ``_stop_walking``) and the ranges of its height and torso pitch commands
    (``height_cmd_range``, ``pitch_cmd_range``)."""

    #: steps a fingers command takes
    finger_steps = FINGER_STEPS
    #: rad/s, per joint, of the arm commands
    arm_speed = ARM_SPEED
    #: m/s, mean speed of a body command's height ramp
    height_rate = HEIGHT_RATE
    #: the last walk stopped for lack of progress
    walk_blocked = False
    #: s without WALK_STALL_M of progress after which a walk is given up as blocked (None: never)
    walk_stall_s = None
    #: (near, far): s without WALK_STALL_M of improvement on a round's nearest approach after which the round
    #: ends, as done when that approach came within WALK_NEAR_M (near), as blocked when not (far); None: never
    walk_best_stall_s = None
    #: the last walk's round ended near its target without getting nearer (walk_best_stall_s)
    walk_near_stopped = False
    #: (forward, left) m in the walk's final heading from where the walking loop stops the pelvis to where it
    #: stands; the loop aims that much the other way
    walk_settle_offset = (0.0, 0.0)
    walk_min = WALK_MIN
    walk_max = WALK_MAX
    #: the height command's name in a body command's report
    height_cmd_name = "height_command"
    #: a body command's torso yaw closed on its measurement and then held like the height and pitch (else
    #: the waist yaw command is the yaw asked)
    close_yaw = False
    yaw_cmd_range = (-math.pi, math.pi)

    def __init__(self, sim: SimpleSim, upper: dict, waist: dict, height_cmd: float):
        self.sim = sim
        self.ik = ArmIK(sim)
        #: arm and finger targets, by joint
        self.upper = upper
        #: waist yaw, roll, pitch targets = torso command
        self.waist = waist
        self.vx = self.vy = 0.0
        self.turn = 0.0
        self.height_cmd = height_cmd
        #: posture held between body commands: pelvis height (m) and torso pitch (rad), None = the
        #: controller's own
        self.posture = {"height": None, "pitch": None}
        #: per hand: hold mode and the palm pose kept in that mode (world, or the pelvis frame)
        self.hold = {s: {"mode": "torso"} for s in P.SIDES}

    # -- the controller's own ----------------------------------------------------------------------
    def send(self, hold_posture: bool = True) -> bool:
        """One step with the current command; the held posture is corrected slowly."""
        raise NotImplementedError

    def _turn_command(self, heading_world: float, err: float) -> None:
        """Turn in place toward a world heading (err: its difference from the pelvis's heading now)."""
        raise NotImplementedError

    def _turn_end(self) -> None:
        raise NotImplementedError

    def _walk_command(self, vx: float, vy: float, travel_h: float, h: float) -> None:
        """Walk at (vx, vy) in the pelvis's frame holding world heading travel_h (h: the heading now)."""
        raise NotImplementedError

    def _stop_walking(self) -> None:
        raise NotImplementedError

    # -- command -----------------------------------------------------------------------------------
    def arm_q(self, side: str) -> np.ndarray:
        return np.array([self.upper[j] for j in ARM[side]])

    def set_arm(self, side: str, q: np.ndarray) -> None:
        for j, v in zip(ARM[side], q):
            self.upper[j] = float(v)

    def _hold_posture(self) -> None:
        """The held posture's slow correction (once per step)."""
        sim = self.sim
        if self.posture["height"] is not None:
            self.height_cmd = float(np.clip(self.height_cmd + HOLD_GAIN * (self.posture["height"] - sim.pelvis_height()),
                                            *self.height_cmd_range))
        if self.posture["pitch"] is not None:
            self.waist["waist_pitch_joint"] = float(np.clip(
                self.waist["waist_pitch_joint"] + HOLD_GAIN * (self.posture["pitch"] - sim.torso()["pitch"]),
                *self.pitch_cmd_range))
        if self.posture.get("yaw") is not None:
            self.waist["waist_yaw_joint"] = float(np.clip(
                self.waist["waist_yaw_joint"] + HOLD_GAIN * (self.posture["yaw"] - sim.torso()["yaw"]),
                *self.yaw_cmd_range))

    def hold_steps(self, steps: int) -> bool:
        for _ in range(int(steps)):
            if not self.send():
                return False
        return True

    def ended(self, out: dict | None = None) -> dict:
        return {**(out or {}), "episode_over": True, "status": self.sim.status()}

    # -- hands kept in place while the body moves ------------------------------------------------
    def _pelvis_pose(self) -> tuple[np.ndarray, np.ndarray]:
        i = self.sim.pelvis_id
        return self.sim.data.xmat[i].reshape(3, 3).copy(), self.sim.data.xpos[i].copy()

    def _held_target(self, side: str):
        h = self.hold[side]
        if h["mode"] == "world":
            return h["point"], h["normal"], h["finger"]
        R, p = self._pelvis_pose()
        return p + R @ h["point"], R @ h["normal"], R @ h["finger"]

    def _track_held_hands(self) -> None:
        for side in P.SIDES:
            if self.hold[side]["mode"] == "torso":
                continue
            point, normal, finger = self._held_target(side)
            q_ik, _ = self.ik.solve(side, point, normal, finger, q0=self.arm_q(side), iters=8)
            self.set_arm(side, limit_step(self.arm_q(side), q_ik, self.arm_speed * DT))

    def _hand_report(self) -> dict:
        out = {}
        for side in P.SIDES:
            if self.hold[side]["mode"] == "torso":
                continue
            point, normal, finger = self._held_target(side)
            now = self.sim.palm(side)
            out[side] = {"hold": self.hold[side]["mode"],
                         "position_error_m": round(float(np.linalg.norm(now["point"] - point)), 4),
                         "normal_error_deg": round(angle_between(now["normal"], normal), 2)}
        return out

    # -- body --------------------------------------------------------------------------------------
    def body(self, height: float | None = None, pitch_deg: float | None = None,
             yaw_deg: float | None = None, max_s: float = BODY_MAX_S) -> dict:
        """Pelvis height above the ground (m), torso pitch (deg below horizontal, forward lean) and torso
        yaw (deg left of the pelvis's heading), each closed on its measurement and then held.  After the
        ramp, the correction goes on until every one asked for has been within its tolerance for
        INSIDE_STEPS steps in a row, for at most ``max_s`` seconds ("reached" says which)."""
        sim = self.sim
        t0 = sim.steps
        if height is not None:
            lo, hi = P.PELVIS_HEIGHT_RANGE_M
            height = min(hi, max(lo, float(height)))
        pitch = None if pitch_deg is None else math.radians(min(P.TORSO_PITCH_RANGE_DEG[1],
                                                                max(P.TORSO_PITCH_RANGE_DEG[0], float(pitch_deg))))
        yaw = None if yaw_deg is None else math.radians(min(P.TORSO_YAW_RANGE_DEG[1],
                                                            max(P.TORSO_YAW_RANGE_DEG[0], float(yaw_deg))))
        if height is not None:
            self.posture["height"] = height
        if pitch is not None:
            self.posture["pitch"] = pitch
        if yaw is not None and self.close_yaw:
            self.posture["yaw"] = yaw
        h_goal, p_goal = self.posture["height"], self.posture["pitch"]
        start = {"pelvis_height": sim.pelvis_height(), **sim.torso()}
        # ramp the commands by the change asked for, then correct on measurements
        steps = 1
        if height is not None:
            steps = max(steps, int(abs(height - start["pelvis_height"]) / self.height_rate / DT))
        if pitch is not None:
            steps = max(steps, int(abs(pitch - start["pitch"]) / PITCH_RATE / DT))
        if yaw is not None:
            steps = max(steps, int(abs(yaw - self.waist["waist_yaw_joint"]) / YAW_RATE / DT))
        c0 = (self.height_cmd, self.waist["waist_pitch_joint"], self.waist["waist_yaw_joint"])
        c1 = (c0[0] if height is None else c0[0] + (height - start["pelvis_height"]),
              c0[1] if pitch is None else c0[1] + (pitch - start["pitch"]),
              c0[2] if yaw is None else yaw)
        for k in range(1, steps + 1):
            s = min_jerk(k / steps)
            self.height_cmd = float(np.clip(c0[0] + s * (c1[0] - c0[0]), *self.height_cmd_range))
            self.waist["waist_pitch_joint"] = float(np.clip(c0[1] + s * (c1[1] - c0[1]), *self.pitch_cmd_range))
            self.waist["waist_yaw_joint"] = c0[2] + s * (c1[2] - c0[2])
            self._track_held_hands()
            if not self.send(hold_posture=False):
                return self.ended()
        inside = 0
        for k in range(int(max_s / DT)):
            h, t = sim.pelvis_height(), sim.torso()
            eh = 0.0 if h_goal is None else h_goal - h
            ep = 0.0 if p_goal is None else p_goal - t["pitch"]
            ey = yaw - t["yaw"] if self.close_yaw and yaw is not None else 0.0
            inside = inside + 1 if abs(eh) < HEIGHT_TOL_M and abs(ep) < PITCH_TOL and abs(ey) < YAW_TOL else 0
            if inside >= INSIDE_STEPS:
                break
            self.height_cmd = float(np.clip(self.height_cmd + HEIGHT_GAIN * eh, *self.height_cmd_range))
            self.waist["waist_pitch_joint"] = float(np.clip(self.waist["waist_pitch_joint"] + PITCH_GAIN * ep,
                                                            *self.pitch_cmd_range))
            if ey:
                self.waist["waist_yaw_joint"] = float(np.clip(self.waist["waist_yaw_joint"] + YAW_GAIN * ey,
                                                              *self.yaw_cmd_range))
            self._track_held_hands()
            if not self.send(hold_posture=False):
                return self.ended()
        end = {"pelvis_height": sim.pelvis_height(), **sim.torso()}
        out = {"seconds": round((sim.steps - t0) * DT, 2), "reached": inside >= INSIDE_STEPS,
               "height_m": {"asked": None if h_goal is None else round(h_goal, 4),
                            "reached": round(end["pelvis_height"], 4)},
               "pitch_deg": {"asked": None if p_goal is None else round(math.degrees(p_goal), 2),
                             "reached": round(math.degrees(end["pitch"]), 2)},
               "yaw_deg": {"asked": round(math.degrees(self.waist["waist_yaw_joint"] if yaw is None else yaw), 2),
                           "reached": round(math.degrees(end["yaw"]), 2)},
               "command": {self.height_cmd_name: round(self.height_cmd, 4),
                           "torso_pitch_rad": round(self.waist["waist_pitch_joint"], 4),
                           "torso_yaw_rad": round(self.waist["waist_yaw_joint"], 4)}}
        limits = []
        if h_goal is not None and self.height_cmd in self.height_cmd_range:
            limits.append("height command at its limit")
        if p_goal is not None and self.waist["waist_pitch_joint"] in self.pitch_cmd_range:
            limits.append("pitch command at its limit")
        if limits:
            out["limits"] = limits
        hands = self._hand_report()
        if hands:
            out["hands"] = hands
        return out

    # -- walk --------------------------------------------------------------------------------------
    def _pelvis(self):
        sim = self.sim
        p = sim.data.xpos[sim.pelvis_id]
        from .sim import heading
        return float(p[0]), float(p[1]), heading(sim.data.xmat[sim.pelvis_id].reshape(3, 3))

    def turn_to(self, heading_world: float, max_s: float = 8.0) -> bool:
        """Turn in place to a world heading (rad)."""
        inside = 0
        for _ in range(int(max_s / DT)):
            x, y, h = self._pelvis()
            err = wrap(heading_world - h)
            inside = inside + 1 if abs(err) < TURN_TOL else 0
            if inside >= 10:
                break
            self.vx = self.vy = 0.0
            self._turn_command(heading_world, err)
            if not self.send():
                return False
        self._turn_end()
        return True

    def _walk_to(self, x: float, y: float, travel_h: float, max_steps: int) -> tuple[bool, bool]:
        """Position loop holding a heading; returns (episode still running, reached).  Sets
        ``walk_blocked`` when it stops for lack of progress."""
        inside = 0
        stall = None if self.walk_stall_s is None else int(self.walk_stall_s / DT)
        best_stall = None if self.walk_best_stall_s is None else tuple(int(s / DT) for s in self.walk_best_stall_s)
        dists = []
        best = []                   # the nearest approach so far, per step
        for _ in range(max_steps):
            px, py, h = self._pelvis()
            ex_w, ey_w = x - px, y - py
            c, s = math.cos(h), math.sin(h)
            ef, el = c * ex_w + s * ey_w, -s * ex_w + c * ey_w
            d = math.hypot(ef, el)
            inside = inside + 1 if d < WALK_TOL_M else 0
            if inside >= 10:
                self._stop_walking()
                return True, True
            dists.append(d)
            if stall is not None and len(dists) > stall and dists[-1 - stall] - d < WALK_STALL_M:
                self.walk_blocked = True
                self._stop_walking()
                return True, False
            best.append(min(d, best[-1]) if best else d)
            if best_stall is not None:
                near = best[-1] < WALK_NEAR_M
                window = best_stall[0] if near else best_stall[1]
                if len(best) > window and best[-1 - window] - best[-1] < WALK_STALL_M:
                    self.walk_near_stopped = near
                    self.walk_blocked = not near
                    self._stop_walking()
                    return True, False
            if d < WALK_TOL_M:
                vx = vy = 0.0
            else:
                vx = max(-self.walk_max["back"], min(self.walk_max["forward"], WALK_GAIN * ef))
                vy = max(-self.walk_max["side"], min(self.walk_max["side"], WALK_GAIN * el))
                top = max(abs(vx), abs(vy))
                if top < self.walk_min:
                    vx, vy = vx * self.walk_min / top, vy * self.walk_min / top
            self._walk_command(vx, vy, travel_h, h)
            if not self.send():
                self._stop_walking()
                return False, False
        self._stop_walking()
        return True, False

    def walk(self, x: float, y: float, heading_deg: float | None = None, max_s: float = 20.0) -> dict:
        """Walk the pelvis to world (x, y) and turn to a world heading (deg; None keeps the heading)."""
        sim = self.sim
        t0 = sim.steps
        x0, y0, h0 = self._pelvis()
        goal_h = h0 if heading_deg is None else math.radians(float(heading_deg))
        dist = math.hypot(x - x0, y - y0)
        bearing = math.atan2(y - y0, x - x0)
        travel_h = h0
        if dist <= FACE_TARGET_M and abs(wrap(goal_h - h0)) > TURN_FIRST:
            travel_h = goal_h                       # short walk, big turn: turn first
        elif dist > FACE_TARGET_M and abs(wrap(bearing - h0)) > FACE_OFF_AXIS:
            travel_h = bearing                      # long walk off to the side: face the target
        elif abs(wrap(goal_h - h0)) <= TURN_FIRST:
            travel_h = goal_h                       # small turns are made while walking
        if abs(wrap(travel_h - h0)) > TURN_FIRST and not self.turn_to(travel_h):
            return self.ended()
        reached = False
        rounds = 0
        deadline = t0 + int(max_s / DT)
        self.walk_blocked = self.walk_near_stopped = False
        fo, lo = self.walk_settle_offset
        aim_x = x - (math.cos(goal_h) * fo - math.sin(goal_h) * lo)
        aim_y = y - (math.sin(goal_h) * fo + math.cos(goal_h) * lo)
        while rounds < WALK_ROUNDS and sim.steps < deadline:
            rounds += 1
            running, reached = self._walk_to(aim_x, aim_y, travel_h, deadline - sim.steps)
            if not running:
                return self.ended()
            if abs(wrap(goal_h - self._pelvis()[2])) > TURN_TOL and not self.turn_to(goal_h):
                return self.ended()
            travel_h = goal_h
            if not self.hold_steps(WALK_SETTLE_STEPS):
                return self.ended()
            px, py, _ = self._pelvis()
            if math.hypot(x - px, y - py) <= WALK_DONE_M or self.walk_blocked:
                break
        px, py, h = self._pelvis()
        err = math.hypot(x - px, y - py)
        out = {"seconds": round((sim.steps - t0) * DT, 2), "reached": err <= WALK_DONE_M, "rounds": rounds,
               "asked": [round(x, 4), round(y, 4), round(math.degrees(goal_h), 2)],
               "reached_pose": [round(px, 4), round(py, 4), round(math.degrees(h), 2)],
               "position_error_m": round(err, 4),
               "heading_error_deg": round(math.degrees(wrap(goal_h - h)), 2),
               "distance_asked_m": round(dist, 4)}
        if self.walk_blocked:
            secs = self.walk_stall_s if self.walk_best_stall_s is None else self.walk_best_stall_s[1]
            out["blocked"] = f"no {WALK_STALL_M * 100:.0f} cm of progress in {secs} s"
        elif self.walk_near_stopped and err > WALK_DONE_M:
            out["near_stopped"] = True
        return out

    # -- hands -------------------------------------------------------------------------------------
    def hand(self, side: str, waypoints: list[dict], hold: str = "torso") -> dict:
        """Move a palm through waypoints (world): each {"point": xyz, "normal": xyz or None,
        "finger": xyz or None, "seconds": s}; straight lines with a minimum-jerk timing, the arm's
        joints from inverse kinematics at every step on the robot's current state, at most arm_speed;
        where that speed cannot keep up, the path waits for the arm (the move takes longer, reported as
        "slowed_by_speed_limit_s").
        ``hold`` says what the palm does in later body commands: "torso" (the arm's joints stay, so it
        moves with the torso), "world" (it stays where it is) or "pelvis" (it moves with the pelvis)."""
        if side not in P.SIDES:
            raise ValueError(f"side must be one of {P.SIDES}")
        if not 1 <= len(waypoints) <= P.HAND_WAYPOINTS_MAX:
            raise ValueError(f"1..{P.HAND_WAYPOINTS_MAX} waypoints")
        if hold not in HOLD_MODES:
            raise ValueError(f"hold must be one of {HOLD_MODES}")
        sim = self.sim
        t0 = sim.steps
        cur = sim.palm(side)
        p0, n0, f0 = cur["point"].copy(), cur["normal"].copy(), cur["finger"].copy()
        q = self.arm_q(side)
        max_step = self.arm_speed * DT
        slowed = 0
        for wp in waypoints:
            p1 = np.asarray(wp["point"], dtype=float)
            n1 = None if wp.get("normal") is None else unit(wp["normal"])
            f1 = None if wp.get("finger") is None else unit(wp["finger"])
            seconds = min(P.HAND_SEGMENT_S[1], max(P.HAND_SEGMENT_S[0], float(wp.get("seconds", 1.0))))
            steps = max(1, int(round(seconds / DT)))
            # the path waits for the arm: a path point the speed limit keeps the joints from reaching in
            # one step is held until they get there (moving the joints straight toward a far solution
            # would take the palm off the line), up to SLOW_MAX times the asked time
            k, used = 1, 0
            while k <= steps:
                s = min_jerk(k / steps)
                p = p0 + s * (p1 - p0)
                n = None if n1 is None else rotate_toward(n0, n1, s)
                f = None if f1 is None else rotate_toward(f0, f1, s)
                q_ik, _ = self.ik.solve(side, p, n, f, q0=q)
                q_new = limit_step(q, q_ik, max_step)
                behind = float(np.max(np.abs(q_ik - q_new)))
                q = q_new
                self.set_arm(side, q)
                if not self.send():
                    return self.ended()
                used += 1
                if behind < 1e-9 or used >= SLOW_MAX * steps:
                    k += 1
                else:
                    slowed += 1
            p0 = p1
            n0 = n0 if n1 is None else n1
            f0 = f0 if f1 is None else f1
        # keep tracking the last waypoint until the palm is there (or the settle time is up)
        last = waypoints[-1]
        n1 = None if last.get("normal") is None else unit(last["normal"])
        f1 = None if last.get("finger") is None else unit(last["finger"])
        inside = 0
        res = {}
        for _ in range(HAND_SETTLE_MAX_STEPS):
            q_ik, res = self.ik.solve(side, p0, n1, f1, q0=q)
            q = limit_step(q, q_ik, max_step)
            self.set_arm(side, q)
            if not self.send():
                return self.ended()
            inside = inside + 1 if np.linalg.norm(sim.palm(side)["point"] - p0) < HAND_TOL_M else 0
            if inside >= 5:
                break
        now = sim.palm(side)
        out = {"seconds": round((sim.steps - t0) * DT, 2), "side": side,
               "reached_point": [round(float(v), 4) for v in now["point"]],
               "position_error_m": round(float(np.linalg.norm(now["point"] - p0)), 4),
               "ik_position_error_m": round(res.get("position_error_m", float("nan")), 4)}
        if n1 is not None:
            out["normal_error_deg"] = round(angle_between(now["normal"], n1), 2)
        if f1 is not None:
            out["finger_error_deg"] = round(angle_between(now["finger"], f1), 2)
        if "at_limit" in res:
            out["at_limit"] = res["at_limit"]
        if slowed:
            out["slowed_by_speed_limit_s"] = round(slowed * DT, 2)
        # what the palm does in later body commands
        if hold == "torso":
            self.hold[side] = {"mode": "torso"}
        else:
            point = p0
            normal = now["normal"] if n1 is None else n1
            finger = now["finger"] if f1 is None else f1
            if hold == "pelvis":
                R, p = self._pelvis_pose()
                point, normal, finger = R.T @ (point - p), R.T @ normal, R.T @ finger
            self.hold[side] = {"mode": hold, "point": np.array(point, dtype=float),
                               "normal": np.array(normal, dtype=float), "finger": np.array(finger, dtype=float)}
        out["hold"] = hold
        return out

    # -- fingers -----------------------------------------------------------------------------------
    def fingers(self, side: str, closed: bool, wide: bool = False) -> dict:
        """Open (all zeros; ``wide``: the thumb swung back from the fingers, THUMB_WIDE) or close one hand's
        fingers; closing curls them until they touch and then squeezes (HAND_SQUEEZE), so "short of target"
        is large when something is held."""
        t0 = self.sim.steps
        target = HAND_SQUEEZE[side] if closed else HAND_WIDE[side] if wide else (0.0,) * 7
        for j, v in zip(HAND[side], target):
            self.upper[j] = float(v)
        if not self.hold_steps(self.finger_steps):
            return self.ended()
        q = self.sim.joints(HAND[side])
        return {"seconds": round((self.sim.steps - t0) * DT, 2), "side": side, "closed": bool(closed),
                "short_of_target_rad": [round(float(t - v), 3) for t, v in zip(target, q)]}

    # -- gaze --------------------------------------------------------------------------------------
    def gaze(self, yaw_deg: float, pitch_deg: float) -> dict:
        """Turn the gaze camera at the pan-tilt's speed (the robot holds still meanwhile)."""
        sim = self.sim
        t0 = sim.steps
        y0, p0 = (float(v) for v in sim.gaze_deg)
        y1 = min(P.GAZE_YAW_RANGE_DEG[1], max(P.GAZE_YAW_RANGE_DEG[0], float(yaw_deg)))
        p1 = min(P.GAZE_PITCH_RANGE_DEG[1], max(P.GAZE_PITCH_RANGE_DEG[0], float(pitch_deg)))
        steps = int(math.ceil(max(abs(y1 - y0), abs(p1 - p0)) / P.GAZE_SPEED_DEG_S / DT))
        for k in range(1, steps + 1):
            s = k / steps
            sim.set_gaze(y0 + s * (y1 - y0), p0 + s * (p1 - p0))
            if not self.send():
                return self.ended()
        sim.set_gaze(y1, p1)
        return {"seconds": round((sim.steps - t0) * DT, 2), "gaze_deg": [round(y1, 1), round(p1, 1)]}

    # -- HITCH's execute ---------------------------------------------------------------------------
    def execute(self, body: dict | None = None, hands: dict | None = None, fingers: dict | None = None,
                walk: dict | None = None, gaze: dict | None = None) -> dict:
        """The parts given, in HITCH's order: body, hands, fingers, walk, gaze.  ``hands`` maps a side to
        its waypoints, or to {"waypoints": [...], "hold": mode}."""
        out = {}
        if body:
            out["body"] = self.body(**body)
        if hands and not self.sim.over:
            out["hands"] = {}
            for side, spec in hands.items():
                if isinstance(spec, dict):
                    out["hands"][side] = self.hand(side, spec["waypoints"], spec.get("hold", "torso"))
                else:
                    out["hands"][side] = self.hand(side, spec)
        if fingers and not self.sim.over:
            out["fingers"] = {side: self.fingers(side, closed) for side, closed in fingers.items()}
        if walk and not self.sim.over:
            out["walk"] = self.walk(**walk)
        if gaze and not self.sim.over:
            out["gaze"] = self.gaze(**gaze)
        out["status"] = self.sim.status()
        return out


class AmoExecutor(Executor):
    """The commands carried out with AMO (the MP tasks)."""

    height_cmd_name = "d_height"
    walk_best_stall_s = (AMO_NEAR_STALL_S, AMO_FAR_STALL_S)
    walk_settle_offset = AMO_SETTLE_OFFSET_M

    def __init__(self, sim: SimpleSim):
        from simple.core.action import ActionCmd

        joints = sim.proprio()["joints"]
        super().__init__(sim, upper={j: float(joints[j]) for s in P.SIDES for j in ARM[s] + HAND[s]},
                         waist={j: float(joints[j]) for j in WAIST}, height_cmd=0.0)
        self.ActionCmd = ActionCmd
        self.target_yaw = self.amo_yaw()

    # AMO's height command is d_height (the pelvis height asked for is 0.75 m + d_height); the ranges are
    # read when used, so a script may widen them
    @property
    def height_cmd_range(self) -> tuple:
        return D_HEIGHT_RANGE

    @property
    def pitch_cmd_range(self) -> tuple:
        return TORSO_PITCH_CMD_RANGE

    def amo_yaw(self) -> float:
        """The pelvis heading in AMO's terms: relative to the IMU's orientation at the first step."""
        d = self.sim.data
        q = np.array(d.sensor("orientation").data, dtype=float)
        q0 = self.sim.robot.amo_policy._initial_quat
        if q0 is None:
            return 0.0
        return quat_yaw(quat_mul(quat_conj(np.asarray(q0, dtype=float)), q))

    def send(self, hold_posture: bool = True) -> bool:
        """One step with the current command; the held posture is corrected slowly."""
        if hold_posture:
            self._hold_posture()
        w = self.waist
        cmd = self.ActionCmd(
            "eval_move_actuators", target_qpos=dict(self.upper),
            action_command=[self.vx, self.target_yaw, self.vy, self.height_cmd, w["waist_yaw_joint"],
                            w["waist_pitch_joint"], w["waist_roll_joint"], self.turn],
            waist_qpos=dict(w))
        return self.sim.step(cmd)

    def _turn_command(self, heading_world: float, err: float) -> None:
        self.turn = 1.0
        self.target_yaw = self.amo_yaw() + err

    def _turn_end(self) -> None:
        self.turn = 0.0
        self.target_yaw = self.amo_yaw()

    def _walk_command(self, vx: float, vy: float, travel_h: float, h: float) -> None:
        self.vx, self.vy, self.turn = vx, vy, 0.0
        self.target_yaw = self.amo_yaw() + wrap(travel_h - h)

    def _stop_walking(self) -> None:
        self.vx = self.vy = 0.0


#: the WBC's height command (m) is kept inside this (the teleoperation's range)
WBC_HEIGHT_CMD_RANGE = (0.2, 0.74)
#: the turn flag while walking and turning (the WBC turns toward the heading while the flag is >= 0.1)
WBC_TURN_FLAG = 1.0
#: SIMPLE's hand PD under the WBC is soft (kp 5 thumb, 2.5 index and middle, kd 1): about 0.3 s (thumb) and
#: 0.5 s (index, middle) to 63 % of a move (check_wbc_fingers.py)
WBC_FINGER_STEPS = 50


class WbcExecutor(Executor):
    """The commands carried out with GR00T's decoupled WBC (the teleoperation tasks; ``sim.wbc``).

    SIMPLE's G1Sonic drives the WBC's arm targets by PD alone (kp 100, 100, 40, 40, 20, 20, 20 from the
    shoulder to the wrist; no gravity compensation, GR00T's default), so an arm held out sags: on reachable
    targets the palm stayed 2.4-5.8 cm (median 4.6 cm) short.  Each arm target sent is therefore
    raised by the gravity torque at the commanded posture over the joint's stiffness (``gravity_feedforward``;
    the rest of the body as it is, standing still; a held object's weight is not included: a palm holding the
    0.1 kg cracker box trails a moving target by about 3 cm, so a move that ends next to something is best
    made as two hand commands, the arm settling at the first).  The torso's yaw follows the waist yaw command
    loosely (30 deg asked: 21 deg after 0.7 s, 33 deg after 1.7 s), so a body command's yaw is closed on its
    measurement like the pitch.

    Found on the teleoperation tasks' scenes: the hips (hip roll and yaw links) meet a table's 10 cm
    top slab when the pelvis is about 0.19-0.21 m from its edge, walking forward or sideways or turning in
    place, and the WBC then pushes on without getting closer (hence ``walk_stall_s``).  Deep squats are
    unsteady (after a walk the pelvis drifted back 20-31 cm in one), so low reaches lean the torso from a
    shallower squat, and a body command asking for more than the WBC gives runs out its ``max_s`` (callers cap
    it).  A palm held back by contact is still solved for from the commanded arm posture; after about 45
    steps the solution can flip the elbow and drag the palm away, so hand commands into contact are kept
    short.  Turning in place goes at the heading error / 0.5 s: 176 deg asked reached 157-175 deg in 3.6 s."""

    finger_steps = WBC_FINGER_STEPS
    height_cmd_range = WBC_HEIGHT_CMD_RANGE
    close_yaw = True
    gravity_feedforward = True
    walk_stall_s = WALK_STALL_S

    def __init__(self, sim: SimpleSim):
        import mujoco

        if sim.wbc is None:
            raise ValueError(f"{sim.spec.env_id} has no decoupled WBC (an MP task: AmoExecutor)")
        self.wbc = sim.wbc
        goal = self.wbc.targets()  # the WBC's goal after the stand, by the joints driven
        super().__init__(sim, upper={j: float(goal[j]) for s in P.SIDES for j in ARM[s] + HAND[s]},
                         waist={j: float(goal[j]) for j in WAIST}, height_cmd=float(self.wbc.height))
        #: the waist pitch target is the torso pitch command: kept inside the joint's range
        self.pitch_cmd_range = tuple(sim.limits["waist_pitch_joint"])
        self.yaw_cmd_range = tuple(sim.limits["waist_yaw_joint"])
        #: world heading the WBC turns toward while the turn flag is on
        self.heading = self._pelvis()[2]
        self._mj = mujoco
        self._gravity_data = mujoco.MjData(sim.model)
        self._tau = np.zeros(sim.model.nv)
        self._arm_kp = {s: np.array([self.wbc.body_kp[j] for j in ARM[s]]) for s in P.SIDES}

    def gravity_offsets(self) -> dict:
        """Per arm joint (rad): the gravity torque at the commanded arm postures (the rest of the robot as it
        is now, nothing moving) over the joint's PD stiffness."""
        mj, m, d = self._mj, self.sim.model, self._gravity_data
        d.qpos[:] = self.sim.data.qpos
        for s in P.SIDES:
            d.qpos[self.ik.qadr[s]] = self.arm_q(s)
        d.qvel[:] = 0.0
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)
        mj.mj_comVel(m, d)
        mj.mj_rne(m, d, 0, self._tau)
        out = {}
        for s in P.SIDES:
            for j, v in zip(ARM[s], self._tau[self.ik.dadr[s]] / self._arm_kp[s]):
                out[j] = float(v)
        return out

    def send(self, hold_posture: bool = True) -> bool:
        """One step with the current command; the held posture is corrected slowly."""
        if hold_posture:
            self._hold_posture()
        w = self.wbc
        if self.gravity_feedforward:
            sent = dict(self.upper)
            for j, v in self.gravity_offsets().items():
                sent[j] += v
            w.set_targets(sent)
        else:
            w.set_targets(self.upper)
        w.set_targets(self.waist)
        w.navigate = np.array([self.vx, self.vy, self.turn, self.heading], dtype=float)
        w.height = self.height_cmd
        return self.sim.step(w.action())

    def _turn_command(self, heading_world: float, err: float) -> None:
        self.turn = WBC_TURN_FLAG
        self.heading = wrap(heading_world)

    def _turn_end(self) -> None:
        self.turn = 0.0

    def _walk_command(self, vx: float, vy: float, travel_h: float, h: float) -> None:
        self.vx, self.vy, self.turn = vx, vy, WBC_TURN_FLAG
        self.heading = wrap(travel_h)

    def _stop_walking(self) -> None:
        self.vx = self.vy = 0.0
        self.turn = 0.0
