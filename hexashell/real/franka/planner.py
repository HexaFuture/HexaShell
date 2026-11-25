"""Plans for the FR3: inverse kinematics with a null space, straight lines and joint swings, checked against the
table and the arm itself, and timed.  A plan is a joint path, which the driver follows with franky's joint motion, so
the path that was checked is the path that runs."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ...geometry import angle_between, slerp
from ...motion.ik import pose_error
from .arm import READY_Q, Fr3

#: A pose counts as reached within these
POS_TOL_M = 0.001
ROT_TOL_RAD = math.radians(0.5)
#: Random starts when the near ones fail, and how many tries to compare once one has worked
SOLVE_RESTARTS = 40
SOLVE_ENOUGH = 10
#: A straight line has a waypoint per this much travel or turn
LINE_STEP_M = 0.01
LINE_STEP_RAD = math.radians(3.0)
#: Consecutive line waypoints may not differ by more than this in any joint: a singularity or an elbow flip
LINE_JUMP_RAD = 0.25
#: A swing's joint path is checked and followed at this many waypoints
SWING_SAMPLES = 24
#: Speeds the planner keeps to: the TCP's turn (rad/s) and each joint (rad/s; the first four are the larger joints)
TURN_SPEED_RAD_S = 0.6
JOINT_SPEED_RAD_S = np.array([0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8])
#: Speed up and slow down over this long at the ends of a path (s), from this share of the full speed
RAMP_S = 0.3
RAMP_FLOOR = 0.25
LEG_FLOOR_S = 0.02


class Refusal(Exception):
    """A motion that will not be planned; the message says why."""


@dataclass
class Solution:
    q: np.ndarray
    pos_err: float
    rot_err: float
    problems: list[str] = field(default_factory=list)

    @property
    def reached(self) -> bool:
        return self.pos_err <= POS_TOL_M and self.rot_err <= ROT_TOL_RAD


@dataclass
class TimedPath:
    """Joint waypoints with their times (s) and velocities (rad/s); the first is where the arm starts."""

    q: np.ndarray
    t: np.ndarray
    dq: np.ndarray
    kind: str

    @property
    def duration_s(self) -> float:
        return float(self.t[-1])


def refine(arm: Fr3, target: np.ndarray, seed, *, iterations: int = 200, rest=READY_Q) -> Solution:
    """Damped least squares from ``seed`` toward the TCP pose ``target``; the spare joint leans toward ``rest``."""
    rest = np.asarray(rest, float)
    margin = 1e-3
    q = np.clip(np.asarray(seed, float), arm.low + margin, arm.high - margin)
    jac, pose = arm.jacobian(q)
    err = pose_error(target, pose)
    for _ in range(iterations):
        if np.linalg.norm(err[:3]) < POS_TOL_M / 4 and np.linalg.norm(err[3:]) < ROT_TOL_RAD / 4:
            break
        # far off: damped, and leaning toward rest; close: nearly undamped, so it converges beside a singularity
        close = np.linalg.norm(err[:3]) < 0.01 and np.linalg.norm(err[3:]) < 0.05
        inverse = jac.T @ np.linalg.inv(jac @ jac.T + (1e-7 if close else 1e-4) * np.eye(6))
        step = inverse @ err + (np.eye(7) - inverse @ jac) @ ((0.0 if close else 0.05) * (rest - q))
        largest = float(np.max(np.abs(step)))
        if largest > 0.2:
            step *= 0.2 / largest
        q = np.clip(q + step, arm.low + margin, arm.high - margin)
        jac, pose = arm.jacobian(q)
        err = pose_error(target, pose)
    return Solution(q, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:])), arm.problems(q))


def solve(arm: Fr3, target: np.ndarray, current) -> Solution:
    """The clear solution nearest ``current``; failing that one that reaches but is not clear, or the closest miss."""
    current = np.asarray(current, float)
    rng = np.random.default_rng(7)
    # the first seeds lean toward the arm's posture; a stuck one ends at a joint limit, so the rest start anywhere
    seeds = [(current, current), (np.asarray(READY_Q), current)]
    seeds += [(seed, seed) for seed in rng.uniform(arm.low, arm.high, (SOLVE_RESTARTS, 7))]
    found = []
    for i, (seed, rest) in enumerate(seeds):
        solution = refine(arm, target, seed, rest=rest)
        found.append(solution)
        if solution.reached and not solution.problems and np.max(np.abs(solution.q - current)) < 1.0:
            return solution
        if i >= SOLVE_ENOUGH and any(s.reached and not s.problems for s in found):
            break
    good = [s for s in found if s.reached and not s.problems]
    if good:
        return min(good, key=lambda s: float(np.sum((s.q - current) ** 2)))
    # a pose that can be reached but only against something says more than the closest miss
    return min(found, key=lambda s: (not s.reached, s.pos_err + 0.12 * s.rot_err))


def goal_of(arm: Fr3, target: np.ndarray, current) -> np.ndarray:
    solution = solve(arm, target, current)
    if not solution.reached:
        raise Refusal(f"out of reach: the closest solution misses by {solution.pos_err * 1000:.0f} mm and "
                      f"{math.degrees(solution.rot_err):.0f} degrees")
    if solution.problems:
        raise Refusal(f"the pose would put {solution.problems[0]}")
    return solution.q


def plan_line(arm: Fr3, current, target: np.ndarray, speed_m_s: float) -> TimedPath:
    """A straight line of the TCP from where ``current`` puts it to ``target``."""
    current = np.asarray(current, float)
    start = arm.tcp(current)
    distance = float(np.linalg.norm(target[:3, 3] - start[:3, 3]))
    turn = angle_between(start[:3, :3], target[:3, :3])
    count = max(1, math.ceil(distance / LINE_STEP_M), math.ceil(turn / LINE_STEP_RAD))
    q, out = current, [current]
    for i in range(1, count + 1):
        share = i / count
        pose = np.eye(4)
        pose[:3, :3] = slerp(start[:3, :3], target[:3, :3], share)
        pose[:3, 3] = (1 - share) * start[:3, 3] + share * target[:3, 3]
        solution = refine(arm, pose, q, iterations=60, rest=q)
        where = f"{100 * share:.0f}% of the way"
        if not solution.reached:
            raise Refusal(f"the straight line leaves the arm's reach {where}")
        if solution.problems:
            raise Refusal(f"the straight line would put {solution.problems[0]} {where}")
        if float(np.max(np.abs(solution.q - q))) > LINE_JUMP_RAD:
            raise Refusal(f"the straight line passes a singularity {where}")
        q = solution.q
        out.append(q)
    return time_path(arm, np.array(out), speed_m_s, "line")


def plan_joints(arm: Fr3, current, goal, speed_m_s: float, kind: str = "swing") -> TimedPath:
    """The joints interpolated directly from ``current`` to ``goal``."""
    current, goal = np.asarray(current, float), np.asarray(goal, float)
    out = [current + (goal - current) * (i / SWING_SAMPLES) for i in range(SWING_SAMPLES + 1)]
    for i, q in enumerate(out[1:], 1):
        problems = arm.problems(q)
        if problems:
            raise Refusal(f"a joint swing would put {problems[0]} {100 * i / SWING_SAMPLES:.0f}% of the way")
    return time_path(arm, np.array(out), speed_m_s, kind)


def time_path(arm: Fr3, q: np.ndarray, speed_m_s: float, kind: str) -> TimedPath:
    """Times for joint waypoints: each leg as long as its slowest limit needs, longer near the two ends."""
    poses = [arm.tcp(c) for c in q]
    legs = np.zeros(len(q) - 1)
    for i in range(len(legs)):
        travel = float(np.linalg.norm(poses[i + 1][:3, 3] - poses[i][:3, 3]))
        turn = angle_between(poses[i][:3, :3], poses[i + 1][:3, :3])
        joints = float(np.max(np.abs(q[i + 1] - q[i]) / JOINT_SPEED_RAD_S))
        legs[i] = max(travel / speed_m_s, turn / TURN_SPEED_RAD_S, joints, LEG_FLOOR_S)
    middle = np.cumsum(legs) - legs / 2
    from_end = np.minimum(middle, legs.sum() - middle)
    legs = legs / np.clip(from_end / RAMP_S, RAMP_FLOOR, 1.0)
    t = np.concatenate([[0.0], np.cumsum(legs)])
    dq = np.zeros_like(q)
    if len(q) > 2:
        dq[1:-1] = (q[2:] - q[:-2]) / (t[2:] - t[:-2])[:, None]
    return TimedPath(q, t, dq, kind)
