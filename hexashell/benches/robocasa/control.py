"""Moving the PandaOmron through RoboCasa's official controllers (server side).

Every step sends the official 12-dim action of PandaOmron's HYBRID_MOBILE_BASE controller:

    arm[0:6] (OSC_POSE deltas) | gripper[6] (sign) | base[7:10] (joint velocities) | torso[10] | mode[11]

always in arm mode (mode -1: the OSC goal is the achieved pose plus the delta).  The torso stays.
The arm and the gripper move as :mod:`hexashell.benches.arm` describes.

Base.  The three base joints (two slides and a yaw hinge 0.21 m behind the base origin) follow a
trapezoidal path from the base pose to the target (straight, or turning about a floor point): joint velocities of the plan plus a term on the
joint error, turned into the action by inverting the official velocity controller (it rotates the
x/y action by the yaw since its reset) with the joints' Coulomb friction added back.  A drive is
blocked when the base stops moving short of the target, or when it is held or pushed off the
planned path (by more than BASE_OFF for BASE_OFF[2] steps): pressing on a counter, the base turns
about the corner that touches it and goes somewhere else.
"""

from __future__ import annotations

import math

import numpy as np

from ..arm import ARRIVED, DT, Arm, fractions, quat_matrix, trapezoid
from . import protocol as P

if abs(DT - P.STEP_S) > 1e-9:
    raise RuntimeError("the arm control assumes RoboCasa's control period")

#: lowest the grasp point may be asked to go (start frame: the floor is z = 0)
FLOOR_Z = 0.05

BASE_SPEED = 0.4
BASE_ACCEL = 0.8
BASE_TURN = 1.0
BASE_TURN_ACCEL = 2.0
BASE_GAIN = 4.0
BASE_SETTLE_GAIN = 10.0
BASE_SETTLE_STEPS = 20
BASE_TOL = (0.01, math.radians(1.0))
BASE_STUCK = (0.002, math.radians(0.5), 10)  # moved less than this over that many steps: blocked
BASE_OFF = (0.03, math.radians(3.0), 3)  # this far off the planned pose for that many steps: blocked (free
# drives stay within about 1 cm and 0.3 degrees of the plan)
BASE_JOINTS = ("mobilebase0_joint_mobile_forward", "mobilebase0_joint_mobile_side", "mobilebase0_joint_mobile_yaw")
BASE_MIN_SPEED = 1e-3


class Robot(Arm):
    """The arm, gripper and base of one episode.  Rebuild after every reset (it rebuilds the robot)."""

    ROW = 11  # ... and the base: x, y, heading
    #: settle much less than the default (3 mm and 1 degree at rest, stall window 8 steps): the
    #: official horizons bind (25 of casa_dev2a's 32 failures ran out of steps or stopped for want of
    #: them), and settling took 13 % of its steps even at 5 mm and 2 degrees.  So a motion ends once the
    #: hand is within ARRIVED, still moving or not; the arm then holds the goal (the rest of the lag
    #: goes on shrinking during a grip or a wait).  Replaying casa_dev2a's 38 successes this way took 12 % fewer steps (the
    #: gotos 18 % fewer), and lost no more of them than a stall window of 6 instead of 5 did (6 each:
    #: open-loop replays drift).
    settle_tol = ARRIVED
    settle_rest = False
    stall_steps = 5

    def __init__(self, sim, log=None):
        robot = sim.env.robots[0]
        composite = robot.composite_controller
        if int(sim.env.action_dim) != 12:
            raise RuntimeError(f"unexpected action dim {sim.env.action_dim}")
        super().__init__(sim, composite.part_controllers["right"], robot.gripper["right"], log=log)
        self.base = Base(sim, composite.part_controllers["base"])

    def action(self, arm6: np.ndarray, grip: float, base=(0.0, 0.0, 0.0)) -> np.ndarray:
        out = np.zeros(12)
        out[0:6] = arm6
        out[6] = grip
        out[7:10] = base
        out[11] = -1.0
        return out

    def row_extra(self) -> list[float]:
        return [float(v) for v in self.sim.base()]

    def drive(self, target, about=None) -> dict:
        """Drive the base to ``target`` = (x, y, heading rad), start frame; the arm holds its pose.  With
        ``about`` = (x, y), the base turns rigidly about that floor point instead of going straight
        (``target`` is where that turn ends): the base and the hand go round it on circles."""
        return self.base.run(np.asarray(target, float), self, None if about is None else np.asarray(about, float))


class Base:
    """Kinematics of the base joints and the inverse of the official base velocity controller."""

    def __init__(self, sim, controller):
        self.sim = sim
        self.controller = controller
        m, d = sim.model, sim.data
        ids = [m.joint(name).id for name in BASE_JOINTS]
        self.qpos = np.array([m.jnt_qposadr[j] for j in ids])
        self.dof = np.array([m.jnt_dofadr[j] for j in ids])
        if not np.allclose(m.jnt_axis[ids[2]], (0.0, 0.0, 1.0)):
            raise RuntimeError(f"unexpected base yaw axis {m.jnt_axis[ids[2]]}")
        self.axes = np.array([m.jnt_axis[j][:2] for j in ids[:2]], float).T
        self.pivot = np.array(m.jnt_pos[ids[2]][:2], float)
        body = int(m.jnt_bodyid[ids[0]])
        parent = sim.body(int(m.body_parentid[body]))  # start frame
        local = np.eye(4)
        local[:3, :3] = quat_matrix(m.body_quat[body])
        local[:3, 3] = m.body_pos[body]
        self.frame = parent @ local  # where the joints act, start frame (fixed)
        self.heading0 = math.atan2(self.frame[1, 0], self.frame[0, 0])
        actuators = [next(a for a in range(m.nu) if int(m.actuator_trnid[a, 0]) == j) for j in ids]
        kv = np.array([m.actuator_gainprm[a, 0] for a in actuators], float)
        self.friction = np.array([m.dof_frictionloss[k] for k in self.dof], float) / kv
        low = np.asarray(controller.actuator_min, float)
        high = np.asarray(controller.actuator_max, float)
        self.low, self.high = low, high
        self.mid, self.half = 0.5 * (high + low), 0.5 * (high - low)
        kind = type(controller).__name__
        if kind == "LegacyMobileBaseJointVelocityController":
            if controller._check_forward_joint_reversed():
                raise RuntimeError("base forward joint along y: not supported")
            self.sense = 1.0  # joint velocity = R(theta) action
        elif kind == "MobileBaseJointVelocityController":
            self.sense = -1.0  # joint velocity = R(-theta) action
        else:
            raise RuntimeError(f"unsupported base controller {kind}")
        self.kind = kind

    def joints(self) -> np.ndarray:
        return self.sim.data.qpos[self.qpos].copy()

    def pose(self, q=None) -> np.ndarray:
        """(x, y, heading) of the base origin in the start frame, the heading continuous."""
        q = self.joints() if q is None else np.asarray(q, float)
        local = self.axes @ q[:2] + self.pivot - _rot2(q[2]) @ self.pivot
        p = self.frame @ np.array([local[0], local[1], 0.0, 1.0])
        return np.array([p[0], p[1], q[2] + self.heading0])

    def inverse(self, pose) -> np.ndarray:
        x, y, heading = (float(v) for v in pose)
        q_yaw = heading - self.heading0
        local = np.linalg.inv(self.frame) @ np.array([x, y, 0.0, 1.0])
        slides = np.linalg.solve(self.axes, local[:2] - self.pivot + _rot2(q_yaw) @ self.pivot)
        return np.array([slides[0], slides[1], q_yaw])

    def _theta(self) -> float:
        _, ori = self.controller.get_base_pose()
        now = math.atan2(ori[1, 0], ori[0, 0])
        init = np.asarray(self.controller.init_ori, float)
        return now - math.atan2(init[1, 0], init[0, 0])

    def action(self, joint_velocity) -> np.ndarray:
        cmd = np.array(joint_velocity, float)
        moving = np.abs(cmd) > BASE_MIN_SPEED
        cmd[~moving] = 0.0
        cmd[moving] += np.sign(cmd[moving]) * self.friction[moving]
        cmd = np.clip(cmd, self.low, self.high)
        norm = (cmd - self.mid) / self.half
        xy = _rot2(self.sense * self._theta()).T @ norm[:2]
        return np.array([xy[0], xy[1], norm[2]])

    def run(self, target: np.ndarray, robot: Robot, about: np.ndarray | None = None) -> dict:
        start = self.pose()
        target = target.copy()
        # the shorter way round
        target[2] = start[2] + _wrap(target[2] - start[2])
        turn = abs(float(target[2] - start[2]))
        if about is None:
            move = float(np.hypot(*(target[:2] - start[:2])))
        else:
            # round a point: the farther of the base origin and the hand goes the longest way
            hand = robot.sim.tcp()[:2, 3]
            move = turn * max(float(np.hypot(*(start[:2] - about))), float(np.hypot(*(hand - about))))
        d_move, f_move = trapezoid(move, BASE_SPEED, BASE_ACCEL)
        d_turn, f_turn = trapezoid(turn, BASE_TURN, BASE_TURN_ACCEL)
        if d_move >= d_turn:
            shares = fractions(d_move, f_move, move)
        else:
            shares = fractions(d_turn, f_turn, turn)
        plan = [self.inverse(_along(start, target, s, about)) for s in shares]
        final = self.inverse(target)
        previous = self.inverse(start)
        trail = [start]
        steps, blocked, alive = 0, False, not self.sim.over
        off, off_steps, worst, pushed, jammed = None, 0, (0.0, 0.0), False, False
        robot.count_anew()
        for i in range(len(plan) + BASE_SETTLE_STEPS):
            if not alive:
                break
            q = self.joints()
            planned = None
            if i < len(plan):
                want = plan[i]
                velocity = (want - previous) / DT + BASE_GAIN * (previous - q)
                previous = want
                planned = self.pose(want)
            else:
                if _within(self.pose(), target):
                    break
                velocity = BASE_SETTLE_GAIN * (final - q)
            n, alive = robot.hold_step(base=self.action(velocity))
            steps += n
            if robot.gave_way:
                # the arm pressed on something with the elbow far out (see arm.STRAIN), or the elbow
                # straightened (arm.STRETCH_STOP): driving on would press it harder, so the base stopped and
                # the arm gave way
                jammed = blocked = True
                break
            pose = self.pose()
            trail.append(pose)
            if planned is not None:
                off = _offset(pose, planned)
                worst = (max(worst[0], float(np.hypot(*off[:2]))), max(worst[1], abs(off[2])))
                if float(np.hypot(*off[:2])) > BASE_OFF[0] or abs(off[2]) > BASE_OFF[1]:
                    off_steps += 1
                    if off_steps >= BASE_OFF[2]:
                        blocked = pushed = True
                        break
                else:
                    off_steps = 0
            n = BASE_STUCK[2]
            if len(trail) > n and not _within(pose, target):
                moved = float(np.hypot(*(pose[:2] - trail[-1 - n][:2])))
                turned = abs(float(pose[2] - trail[-1 - n][2]))
                if moved < BASE_STUCK[0] and turned < BASE_STUCK[1]:
                    blocked = True
                    break
        # stop: one step without base command lets the friction hold it
        pose = self.pose()
        err_m = float(np.hypot(*(pose[:2] - target[:2])))
        err_r = abs(_wrap(float(pose[2] - target[2])))
        arrived = err_m < BASE_TOL[0] and err_r < BASE_TOL[1]
        out = {"steps": steps, "planned": len(plan), "blocked": blocked or (alive and not arrived),
               "reached": arrived, "pose": pose, "err_m": err_m, "err_deg": math.degrees(err_r), "over": self.sim.over,
               "worst_off_m": worst[0], "worst_off_deg": math.degrees(worst[1]), "jammed": jammed,
               "stretched": bool(robot.gave_way_stretched), "gave_way_back": robot.gave_way_back,
               "gave_way_m": robot.gave_way_m}
        if pushed:
            # where the base is relative to where the plan had it: ahead (+) / behind, left (+) / right, turned
            out["off"] = [float(off[0]), float(off[1]), math.degrees(float(off[2]))]
        return out


def _along(start, target, share: float, about) -> np.ndarray:
    """The pose ``share`` of the way from ``start`` to ``target``: straight, or round ``about``."""
    if about is None:
        return start + share * (target - start)
    turn = share * float(target[2] - start[2])
    xy = about + _rot2(turn) @ (start[:2] - about)
    return np.array([xy[0], xy[1], start[2] + turn])


def _offset(pose, planned) -> np.ndarray:
    """``pose`` relative to ``planned`` (x, y, heading): forward and left in the planned heading, turn."""
    d = _rot2(float(planned[2])).T @ (np.asarray(pose[:2], float) - np.asarray(planned[:2], float))
    return np.array([d[0], d[1], _wrap(float(pose[2] - planned[2]))])


def _within(pose, target) -> bool:
    return (float(np.hypot(*(np.asarray(pose[:2]) - np.asarray(target[:2])))) < BASE_TOL[0]
            and abs(_wrap(float(pose[2] - target[2]))) < BASE_TOL[1])


def _wrap(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def _rot2(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])
