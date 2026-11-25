"""The G2's motions behind the robot calls: both seven-joint arms, their grippers and the waist's turn.

Frames: the world of the robot calls is ``base_link`` (x forward, y left, z up; it moves only when the
chassis does, which these calls never do).  An arm is solved in its ``arm_base_link`` (:mod:`.g2`), which
the waist carries; where it is comes with every observation.

A step is one row of joint targets the benchmark plays; each motion is sent one row per reply, so the
benchmark plays it, lets the arm settle and sends the next request: every waypoint is reached (or not)
before the next is sent, and a motion that meets something stops at the step it stalls.

- ``goto``: a straight line of the grasp point (waypoints every ``STEP_M`` and ``STEP_DEG``, each solved
  from the one before with its spare freedom leaning toward the joints that reach the target; a joint
  jumping more than ``JUMP_RAD`` between two waypoints gets waypoints between them, up to ``SPLITS``
  halvings, and still jumping it breaks the line; the turn is spread evenly, else in the line's first
  or second half, ``SCHEDULES``) or a swing of the joints (every joint at most ``SWING_RAD`` per step).
  'auto' is the line, else a swing that keeps the grasp point within ``SWING_OFF_M`` of it and at most
  ``SWING_DIP_M`` below its lower end.  An open, empty gripper with roll left out may arrive with its
  fingers half a turn round (the same grasp): when the asked way has no line, or pointing at least
  ``FLIP_PITCH`` down when that saves ``FLIP_SAVE_STEPS`` and ``FLIP_SAVE_SHARE`` of the steps; the
  result's note says so, and after a steep one a closed gripper keeps that heading (``ALIAS_DEG``).
  Blocked: the grasp point stalled ``LAG_M`` behind its command (it then holds where it is); dropped:
  what the gripper held slipped out (the fingers closed further by ``DROP_CLOSE``).
- ``gripper``: the command is -0.785 x opening (-0.785 open, 0 closed); rows until the reading settles.  A
  close that ends with the fingers apart holds something.
- ``home``: lift a low hand, then swing the arms to their start joints (both: the waist too).
- ``wait``: hold everything for n steps.
- ``waist``: turn the torso about the vertical (the arms turn with it).

The other arm, the grippers and the waist hold what was last sent.
"""

from __future__ import annotations

import math

import numpy as np

from ...errors import Refused
from ...geometry import angle_between, homogeneous, invert, slerp, wrap_deg
from ...robot import results as R
from ..pose import angles_of, rotation_of, shown_angles, shown_near, turn_deg
from . import g2
from .link import HELD_AT_START, EpisodeLink, copy_row
from .mask import gap_m

#: line waypoints: at most this far (m) and this much turn (degrees) apart
STEP_M = 0.025
STEP_DEG = 8.0
#: a swing moves every joint at most this much (rad) per step
SWING_RAD = 0.15
#: no joint moves more than this (rad) between two waypoints of a line: a bigger jump is halved (the
#: waypoint between solved) up to SPLITS times, and a line that still jumps is no line (the wrist would flip)
JUMP_RAD = 0.25
SPLITS = 3
#: how the hand's turn is spread along a line (the grasp point always moves evenly along it): with
#: the move, in its first half, in its second half - tried in this order
SCHEDULES = ("even", "turn_first", "turn_last")
#: path='auto' swings the joints only when no line leads there and the grasp point then stays this
#: close (m) to the straight line and dips at most SWING_DIP_M below its lower end (RoboDojo's limits)
SWING_OFF_M = 0.03
SWING_DIP_M = 0.01
#: the fingers may arrive half a turn round (the same grasp for an open gripper; roll left out):
#: pointing at least this steeply, whichever way is shorter; less steeply, only when the asked way has
#: no line (the wrist camera then sees upside down, which misleads)
FLIP_PITCH = 85.0
#: ... and there, the turned way is taken when its line is shorter by at least both of these
FLIP_SAVE_STEPS = 5
FLIP_SAVE_SHARE = 0.25
#: results print a hand pointing at least this steeply with its heading as the yaw (roll 0)
FOLD_PITCH = 80.0
#: after such a turned approach, a closed gripper asked for the yaw first asked (within this) keeps the
#: fingers' heading: the turn would take what it holds half a turn round (RoboDojo's ALIAS_DEG)
ALIAS_DEG = 20.0
#: blocked: the grasp point this far (m) behind its command and moving less than STALL_M in a step
LAG_M = 0.02
STALL_M = 0.003
#: after the last waypoint, up to SETTLE_ROWS more of it while the grasp point is more than SETTLE_M behind
SETTLE_ROWS = 2
SETTLE_M = 0.004
#: a move within these of its target reached it
REACHED_MM = 10.0
REACHED_DEG = 3.0
#: a pose the arm misses by at most these is accepted: it goes to the closest pose it reaches (noted)
NEAR_MM = REACHED_MM
NEAR_DEG = REACHED_DEG
#: the gripper: command at 'open', rows at most, settled when the reading changes less than GRIP_STILL
#: twice or is within GRIP_AT of the command
GRIP_OPEN = -0.785
GRIP_ROWS = 30
GRIP_STILL = 0.003
GRIP_AT = 0.01
#: a close that ends at a reading below this (the fingers about 3 mm apart or more) holds something
EMPTY_CLOSE = -0.03
#: holding: the fingers closing this much further (reading) means it slipped out
DROP_CLOSE = 0.05
WAIT_MAX = 100
HOLD_CHUNK = 30
#: the waist: its yaw joint (body joint 5, about the vertical), how far (rad) per step, the most (deg)
WAIST_YAW = 4
WAIST_STEP = 0.08
WAIST_LIMIT_DEG = 170.0
#: the yaw joint trails its command by a step or two (about 10 degrees when the ramp ends) and a motion
#: started before it arrives lands centimetres off, so a turn holds its heading WAIST_HOLD steps at a time, up to
#: WAIST_SETTLE_ROWS steps, until the reading is within WAIST_AT of the goal or moves less than WAIST_STILL
WAIST_HOLD = 2
WAIST_SETTLE_ROWS = 30
WAIST_AT = math.radians(0.5)
WAIST_STILL = math.radians(0.1)
#: home lifts a hand first when it is this much (m) below its start height
HOME_LIFT_M = 0.05
#: simulated seconds of a step (the benchmark renders at 30 Hz)
STEP_S = 1.0 / 30.0
#: random IK seeds tried when the joints now, the last command and the start do not reach a pose
RANDOM_SEEDS = 12

GRASP = homogeneous(np.eye(3), (0.0, 0.0, g2.GRASP_REACH))


def check_arm(arm) -> str:
    if arm not in g2.ARMS:
        raise Refused(f"arm must be 'left' or 'right' (or 'both' where a call allows it), not {arm!r}")
    return arm


def opening_of(value) -> float:
    words = {"open": 1.0, "close": 0.0, "closed": 0.0}
    if isinstance(value, str):
        if value.lower() not in words:
            raise Refused("an opening is 'open', 'close' or a number 0..1")
        return words[value.lower()]
    try:
        g = float(value)
    except (TypeError, ValueError):
        raise Refused("an opening is 'open', 'close' or a number 0..1") from None
    if not (0.0 <= g <= 1.0):
        raise Refused("an opening is 'open', 'close' or a number 0..1")
    return g


def vector_of(value, name: str = "by") -> np.ndarray:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres") from None
    if v.size != 3 or not np.all(np.isfinite(v)):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres")
    return v


def grasp_of(frame, arm: str) -> np.ndarray:
    """The grasp point's pose (end link orientation) in base_link, as read."""
    return frame.end_link(arm) @ GRASP


def opening_read(reading: float) -> float:
    return round(min(max(float(reading) / GRIP_OPEN, 0.0), 1.0), 3)


def _xyz(p) -> str:
    return "(" + ", ".join(f"{float(v):.3f}" for v in p) + ")"


def _tup(p, nd: int = 4) -> tuple:
    return tuple(round(float(v), nd) for v in p)


def _miss(sol) -> float:
    return sol.pos_err_m + 0.01 * sol.rot_err_deg


def _tidy(angles) -> tuple[float, float, float]:
    """Angles that would print as -180 print as 180."""
    return tuple(float(v) + 360.0 if -180.5 < float(v) <= -179.5 else float(v) for v in angles)


def _share(schedule: str, t: float) -> float:
    """How much of the turn is done at ``t`` (0..1) of the way along a line."""
    if schedule == "turn_first":
        return min(1.0, 2.0 * t)
    if schedule == "turn_last":
        return max(0.0, 2.0 * t - 1.0)
    return t


def half_turn(angles) -> tuple[float, float, float]:
    """The same hand with its fingers half a turn round about the way they point:
    Rz(y) Ry(p) Rx(r + 180) = Rz(y + 180) Ry(180 - p) Rx(r) - pointing steeply, written with the yaw
    turned (as results print it), otherwise with the roll."""
    p, y, r = (float(v) for v in angles)
    if abs(p) >= FOLD_PITCH:
        return (round(wrap_deg(180.0 - p), 2), round(wrap_deg(y + 180.0), 2), r)
    return (p, y, round(wrap_deg(r + 180.0), 2))


class G2Driver:
    """Moves the G2 through an :class:`~.link.EpisodeLink`.  ``on_frame(frame, rows)``: after every reply."""

    def __init__(self, link: EpisodeLink, *, log=None, on_frame=None, seed: int = 0):
        self.link = link
        first = link.first
        self.frame = link.latest()
        self.log = log or (lambda event: None)
        self.on_frame = on_frame or (lambda frame, rows: None)
        self.rng = np.random.default_rng(seed)
        self.start_q = {arm: first.arm_joints(arm) for arm in g2.ARMS}
        self.start_pose = {arm: grasp_of(first, arm) for arm in g2.ARMS}
        self.start_angles = {arm: shown_angles(g2.tool_rotation(self.start_pose[arm][:3, :3], arm)) for arm in g2.ARMS}
        self.start_grippers = {arm: opening_read(first.gripper(arm)) for arm in g2.ARMS}
        #: the angles goto keeps when some are left out: the last asked for
        self.asked = dict(self.start_angles)
        self.grip_goal = {arm: opening_read(link.command["grip"][arm]) for arm in g2.ARMS}
        self._holding: dict[str, bool | None] = {arm: (True if first.gripper(arm) > HELD_AT_START else None)
                                                 for arm in g2.ARMS}
        self.held_reading = {arm: (first.gripper(arm) if self._holding[arm] else None) for arm in g2.ARMS}
        #: closes that caught something, per arm (the perception hears of each)
        self.catches = {arm: 0 for arm in g2.ARMS}
        #: per arm, after an approach whose fingers arrived half a turn round pointing steeply: (the yaw
        #: asked, the yaw it has), until the gripper opens again (see ALIAS_DEG)
        self.alias: dict[str, tuple[float, float] | None] = {arm: None for arm in g2.ARMS}
        #: steps sent by the robot calls in this episode
        self.rows = 0
        #: whether a motion asks for the depth pictures where it ends (False: only :meth:`hold_for_depth`
        #: asks, which the perception calls when the latest request has none; online, where a request with
        #: depth takes seconds more to arrive)
        self.motion_depth = True

    # -- reading -----------------------------------------------------------------------------------
    def grasp_pose(self, arm: str) -> np.ndarray:
        return grasp_of(self.frame, arm)

    def holding(self, arm: str) -> bool | None:
        return self._holding[arm]

    def home_offset(self, arm: str) -> tuple[float, float]:
        now, start = self.grasp_pose(arm), self.start_pose[arm]
        return (round(float(np.linalg.norm(now[:3, 3] - start[:3, 3])), 4),
                round(turn_deg(now[:3, :3], start[:3, :3]), 1))

    def angles_now(self, arm: str, asked=None) -> tuple[float, float, float]:
        rot = g2.tool_rotation(self.grasp_pose(arm)[:3, :3], arm)
        return _tidy(shown_near(rot, asked) if asked is not None else shown_angles(rot))

    def refresh(self) -> None:
        """Read the latest request (hold replies may have come since the last motion)."""
        self.frame = self.link.latest()

    # -- sending -----------------------------------------------------------------------------------
    def _row(self, **arms) -> dict:
        row = copy_row(self.link.command)
        for arm, q in arms.items():
            row[arm] = np.asarray(q, float).reshape(7)
        return row

    def _send(self, rows: list[dict], need_depth: bool, *, force: bool = False):
        frame = self.link.act(rows, force or (need_depth and self.motion_depth))
        self.frame = frame
        self.rows += len(rows)
        self.on_frame(frame, rows)
        return frame

    def hold_for_depth(self):
        """One step that holds everything and asks for the depth pictures."""
        return self._send([self._row()], True, force=True)

    def hold(self):
        """One step that holds everything (with the depth pictures when motions ask for them)."""
        return self._send([self._row()], True)

    # -- solving -----------------------------------------------------------------------------------
    def _random_seeds(self, arm: str, n: int) -> list[np.ndarray]:
        m = g2.ARM_MODELS[arm]
        return [self.rng.uniform(m.lower, m.upper) for _ in range(n)]

    def _solve(self, arm: str, goal_ab: np.ndarray, q_now: np.ndarray, *, quick: bool = False) -> g2.Solution:
        model = g2.ARM_MODELS[arm]
        seeds = [q_now, self.link.command[arm], self.start_q[arm]]
        if model.beyond_reach(goal_ab[:3, 3]):
            return model.solve(goal_ab, q_now, iterations=60)  # how near it gets, for the message
        sol = model.ik(goal_ab, seeds, iterations=120 if quick else 200)
        if sol.reached or quick:
            return sol
        more = model.ik(goal_ab, [q_now] + self._random_seeds(arm, RANDOM_SEEDS))
        return more if more.reached or (more.pos_err_m + 0.01 * more.rot_err_deg
                                        < sol.pos_err_m + 0.01 * sol.rot_err_deg) else sol

    def _line(self, arm: str, cur: np.ndarray, target: np.ndarray, q0: np.ndarray, arm_base: np.ndarray, *,
              q_goal: np.ndarray | None = None, schedule: str = "even"):
        """Joints along the straight line of the grasp point from ``cur`` to ``target`` (base_link), the
        turn spread by ``schedule``; each waypoint is solved from the one before, its spare freedom
        leaning toward the joints that reach the target (``q_goal``).  None: no such line."""
        model = g2.ARM_MODELS[arm]
        inv = invert(arm_base)
        p0, p1 = cur[:3, 3], target[:3, 3]
        r0, r1 = cur[:3, :3], target[:3, :3]
        dist = float(np.linalg.norm(p1 - p0))
        ang = math.degrees(angle_between(r0, r1))
        if dist < 0.001 and ang < 0.5:
            return []
        if schedule != "even" and (ang < STEP_DEG or dist < STEP_M):
            return None  # about the even line: nothing new to try
        n = max(1, math.ceil(dist / STEP_M), math.ceil((ang if schedule == "even" else 2.0 * ang) / STEP_DEG))
        q = np.asarray(q0, float)
        lean = None if q_goal is None else np.asarray(q_goal, float) - q
        prev, rest_prev = cur, (None if lean is None else q.copy())
        out: list[np.ndarray] = []
        for i in range(1, n + 1):
            t = i / n
            s = _share(schedule, t)
            pose = homogeneous(slerp(r0, r1, s), p0 + t * (p1 - p0))
            rest = None if lean is None else np.asarray(q0, float) + s * lean
            sol = model.solve(inv @ pose, q, iterations=100, rest=rest)
            if not sol.reached:
                sol = model.solve(inv @ pose, q, iterations=300, rest=rest)
                if not sol.reached:
                    return None
            piece = self._split(model, inv, prev, q, pose, sol.q, rest_prev, rest, SPLITS)
            if piece is None:
                return None
            out += piece
            q, prev, rest_prev = sol.q, pose, rest
        return out

    def _split(self, model, inv, pa, qa, pb, qb, ra, rb, depth: int):
        """The joints from ``qa`` (at pose ``pa``) to ``qb`` (at ``pb``), with waypoints between them
        while a joint jumps more than JUMP_RAD; None when it still jumps after ``depth`` halvings."""
        if float(np.max(np.abs(qb - qa))) <= JUMP_RAD:
            return [qb]
        if depth <= 0:
            return None
        mid = homogeneous(slerp(pa[:3, :3], pb[:3, :3], 0.5), 0.5 * (pa[:3, 3] + pb[:3, 3]))
        rm = None if ra is None or rb is None else 0.5 * (ra + rb)
        sol = model.solve(inv @ mid, qa, iterations=150, rest=rm)
        if not sol.reached:
            return None
        first = self._split(model, inv, pa, qa, mid, sol.q, ra, rm, depth - 1)
        if first is None:
            return None
        second = self._split(model, inv, mid, sol.q, pb, qb, rm, rb, depth - 1)
        return None if second is None else first + second

    @staticmethod
    def _swing(q0: np.ndarray, q1: np.ndarray) -> list[np.ndarray]:
        dq = np.asarray(q1, float) - np.asarray(q0, float)
        big = float(np.max(np.abs(dq)))
        if big < 1e-4:
            return []
        n = max(1, math.ceil(big / SWING_RAD))
        return [np.asarray(q0, float) + dq * (i / n) for i in range(1, n + 1)]

    @staticmethod
    def _off_line(arm: str, plan, p0: np.ndarray, p1: np.ndarray, arm_base: np.ndarray) -> tuple[float, float]:
        """How far (m) the grasp point leaves the straight line from p0 to p1 along a plan, and how far it
        dips below the line's lower end."""
        model = g2.ARM_MODELS[arm]
        d = p1 - p0
        dd = float(d @ d)
        low = min(float(p0[2]), float(p1[2]))
        off = dip = 0.0
        for q in plan:
            p = (arm_base @ model.grasp_pose(q))[:3, 3]
            t = 0.0 if dd < 1e-12 else min(max(float((p - p0) @ d) / dd, 0.0), 1.0)
            off = max(off, float(np.linalg.norm(p - (p0 + t * d))))
            dip = max(dip, low - float(p[2]))
        return off, dip

    def _plan(self, arm, cur, options, q_now, path, arm_base, *, shortest: bool):
        """The motion to the first of ``options`` ((label, angles, target, solution), each reachable, the
        asked one first) that has one: (joints per step or None, 'line' / 'swing', notes, the option).
        ``shortest``: a later option whose line is clearly shorter (FLIP_SAVE_*) wins over the first."""
        if path in ("auto", "line"):
            found = []
            for opt in options:
                for schedule in SCHEDULES:
                    line = self._line(arm, cur, opt[2], q_now, arm_base, q_goal=opt[3].q, schedule=schedule)
                    if line is not None:
                        found.append((line, opt))
                        break
                if found and not shortest:
                    break
            if found:
                line, opt = found[0]
                for other, o in found[1:]:
                    save = len(line) - len(other)
                    if save >= FLIP_SAVE_STEPS and save >= FLIP_SAVE_SHARE * len(line):
                        line, opt = other, o
                return line, "line", [], opt
            if path == "line":
                return None, "line", ["no straight line of the grasp point leads there from here (a joint would "
                                      "have to turn over on the way); path='swing' swings the joints instead"], None
        if path == "swing":
            opt = options[0]
            return self._swing(q_now, opt[3].q), "swing", [], opt
        worst = None
        for opt in options:
            swing = self._swing(q_now, opt[3].q)
            off, dip = self._off_line(arm, swing, cur[:3, 3], opt[2][:3, 3], arm_base)
            if off <= SWING_OFF_M and dip <= SWING_DIP_M:
                return swing, "swing", [f"no straight line from here: the joints swung (the grasp point left the line "
                                        f"by up to {100 * off:.0f} cm)"], opt
            if worst is None:
                worst = (off, dip)
        off, dip = worst
        below = f", {100 * dip:.0f} cm below its lower end" if dip > SWING_DIP_M else ""
        return None, "swing", [f"no straight line leads there from here, and a swing of the joints would take the "
                               f"grasp point up to {100 * off:.0f} cm off it{below}: path='swing' allows that, or go "
                               "by a point in between (a higher one, or these angles first where the hand is)"], None

    def _alternatives(self, arm, cur, p, angles, q_now, arm_base) -> list[str]:
        inv = invert(arm_base)
        out = []
        far = g2.ARM_MODELS[arm].beyond_reach((inv @ np.r_[p, 1.0])[:3])

        def reaches(a) -> bool:
            goal = homogeneous(g2.end_rotation(rotation_of(*a), arm), p)
            return self._solve(arm, inv @ goal, q_now, quick=True).reached

        if not far and abs(angles[0]) >= FOLD_PITCH:
            # pointing down: the fingers a quarter turn round, or tilted toward the point (the yaw its heading
            # from the arm's base, the roll that keeps the fingers' line seen from above within 45°)
            quarter = (angles[0], round(wrap_deg(angles[1] + 90.0)), 0.0)
            if reaches(quarter):
                out.append(f"yaw {quarter[1]:.0f} at the same point (the fingers a quarter turn round)")
            o = arm_base[:3, 3]
            heading = float(round(math.degrees(math.atan2(p[1] - o[1], p[0] - o[0]))))
            off = (angles[1] - heading + 90.0) % 180.0 - 90.0
            roll = 0.0 if abs(off) <= 45.0 else 90.0
            for pitch in (75.0, 60.0, 45.0):
                if reaches((pitch, heading, roll)):
                    out.append(f"pitch {pitch:.0f} yaw {heading:.0f} roll {roll:.0f} at the same point (tilted "
                               "toward it; seen from above the fingers close along about the same line)")
                    break
        else:
            for dp in () if far else (15.0, -15.0, 30.0, -30.0):
                a = (angles[0] + dp, angles[1], angles[2])
                if reaches(a):
                    out.append(f"pitch {a[0]:.0f} at the same point")
                    if len(out) >= 2:
                        break
        rot = g2.end_rotation(rotation_of(*angles), arm)
        p0 = cur[:3, 3]
        lo, hi = 0.0, 1.0
        if self._solve(arm, inv @ homogeneous(rot, p0), q_now, quick=True).reached:
            for _ in range(5):
                mid = 0.5 * (lo + hi)
                if self._solve(arm, inv @ homogeneous(rot, p0 + mid * (p - p0)), q_now, quick=True).reached:
                    lo = mid
                else:
                    hi = mid
            if lo > 0.05:
                q = p0 + lo * (p - p0)
                out.append(f"{_xyz(q)} with these angles ({100 * (1 - lo) * float(np.linalg.norm(p - p0)):.0f} cm "
                           "short, on the way there)")
        return out

    def _lag(self, arm: str, frame, q_cmd) -> float:
        model = g2.ARM_MODELS[arm]
        a = model.grasp_pose(frame.arm_joints(arm))[:3, 3]
        b = model.grasp_pose(q_cmd)[:3, 3]
        return float(np.linalg.norm(a - b))

    def _slipped(self, arm: str, frame) -> bool:
        if self._holding[arm] is not True or self.held_reading[arm] is None:
            return False
        if frame.gripper(arm) > self.held_reading[arm] + DROP_CLOSE:
            self._holding[arm] = False
            self.held_reading[arm] = None
            self.log({"kind": "dropped", "arm": arm, "reading": round(frame.gripper(arm), 4)})
            return True
        return False

    def _closed(self, arm: str, reading: float) -> bool:
        """After a close: whether the fingers stopped apart (they hold something)."""
        holding = reading < EMPTY_CLOSE
        if holding and self._holding[arm] is not True:
            self.catches[arm] += 1
        self._holding[arm] = holding
        self.held_reading[arm] = reading if holding else None
        return holding

    def _grip_asked(self, arm: str, g: float) -> None:
        if g > 0.05 and self.grip_goal[arm] <= 0.05:
            self.alias[arm] = None  # it lets go: a later approach starts afresh
        self.grip_goal[arm] = g
        if g > 0.05 and self._holding[arm] is not None:
            self._holding[arm] = None
            self.held_reading[arm] = None

    # -- goto --------------------------------------------------------------------------------------
    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
             dry_run=False, contact=False):
        if path not in ("auto", "line", "swing"):
            raise Refused("path is 'auto', 'line' or 'swing'")
        if arm == "both":
            return self._goto_both(xyz, by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip, path=path,
                                   dry_run=dry_run, contact=contact)
        arm = check_arm(arm)
        frame = self.frame
        cur = grasp_of(frame, arm)
        q_now = frame.arm_joints(arm)
        asked = list(self.asked[arm])
        for i, v in enumerate((pitch, yaw, roll)):
            if v is not None:
                asked[i] = float(v)
        if yaw is not None and roll is None and abs(asked[0]) >= FOLD_PITCH:
            asked[2] = 0.0  # pointing steeply, the yaw asked is the fingers' heading
        angles = tuple(asked)
        p = cur[:3, 3].copy() if xyz is None else np.asarray(xyz, float).reshape(3)
        if by is not None:
            p = p + vector_of(by)
        gcmd = None if grip is None else GRIP_OPEN * opening_of(grip)
        open_after = self.grip_goal[arm] if grip is None else opening_of(grip)
        notes: list[str] = []
        # a closed gripper whose fingers arrived half a turn from the yaw asked keeps their heading when
        # asked for that yaw again (see ALIAS_DEG)
        alias = self.alias[arm]
        if (alias is not None and yaw is not None and roll is None and open_after <= 0.05
                and abs(angles[0]) >= FLIP_PITCH and abs(wrap_deg(float(yaw) - alias[0])) <= ALIAS_DEG):
            kept = float(yaw)
            angles = (angles[0], round(wrap_deg(alias[1] + kept - alias[0]), 2), angles[2])
            notes.append(f"kept the fingers' heading, yaw {angles[1]:.0f}: yaw {kept:.0f} names the same grasp half "
                         "a turn round, and turning there would take what it holds half a turn round (with roll "
                         "given too, it turns)")
        at_now, ang_now = _tup(cur[:3, 3]), self.angles_now(arm, angles)

        def refused(why, alternatives=()):
            return R.Move(arm=arm, reached=False, moved=False, target=_tup(p), target_angles=angles, at=at_now,
                          angles=ang_now, dry_run=bool(dry_run), notes=notes + list(why),
                          alternatives=list(alternatives))

        # the same grasp with the fingers half a turn round, for an open, empty gripper with roll left out
        choices = [("", angles)]
        if roll is None and open_after >= 0.9 and self._holding[arm] is not True:
            choices.append(("half a turn", half_turn(angles)))
        inv = invert(frame.arm_base)
        model = g2.ARM_MODELS[arm]
        p_ab = (inv @ np.r_[p, 1.0])[:3]
        far = model.beyond_reach(p_ab)
        options, near = [], None
        for label, a in choices:
            goal = homogeneous(g2.end_rotation(rotation_of(*a), arm), p)
            sol = self._solve(arm, inv @ goal, q_now)
            if sol.reached:
                options.append((label, a, goal, sol))
            elif near is None or _miss(sol) < _miss(near[3]):
                near = (label, a, goal, sol)
        if not options and near is not None and not far and 1000 * near[3].pos_err_m <= NEAR_MM \
                and near[3].rot_err_deg <= NEAR_DEG:
            label, a, _, sol = near
            closest = frame.arm_base @ model.grasp_pose(sol.q)
            options.append((label, a, closest, g2.Solution(q=sol.q, reached=True, pos_err_m=0.0, rot_err_deg=0.0)))
            notes.append(f"the arm reaches that pose only to within {1000 * sol.pos_err_m:.0f} mm / "
                         f"{sol.rot_err_deg:.0f}°: it went to the closest pose it reaches")
        if not options:
            if far:
                past = float(np.linalg.norm(p_ab - model.shoulder)) - model.reach
                why = f"the {arm} arm cannot reach that point: it lies {100 * past:.0f} cm beyond the arm's reach"
            else:
                why = (f"the {arm} arm cannot reach that pose (at best {1000 * near[3].pos_err_m:.0f} mm / "
                       f"{near[3].rot_err_deg:.0f}° off)")
            return refused([why], self._alternatives(arm, cur, p, angles, q_now, frame.arm_base))
        plan, how, more, option = self._plan(arm, cur, options, q_now, path, frame.arm_base,
                                             shortest=abs(angles[0]) >= FLIP_PITCH)
        notes += more
        if plan is None:
            return refused([])
        label, used, target, _ = option
        heading = _tidy(shown_near(rotation_of(*used), used))[1]
        if label:
            if abs(angles[0]) >= FOLD_PITCH:
                notes.append(f"the fingers arrive half a turn round, at yaw {heading:.0f} (the same grasp for an open "
                             f"gripper), so the {arm}_wrist picture is upside down from before: pass yaw={heading:.0f} "
                             f"from now on - once the gripper closes, yaw={angles[1]:.0f} is taken as "
                             f"yaw={heading:.0f} (with roll given too, it turns the wrist half a turn)")
            else:
                notes.append(f"the fingers arrive half a turn round, at roll {used[2]:.0f} (the same grasp for an open "
                             f"gripper; with roll {angles[2]:.0f} no path leads there), so the {arm}_wrist picture is "
                             f"upside down from before; roll {used[2]:.0f} is kept for the next moves")
        if dry_run:
            return R.Move(arm=arm, reached=True, moved=False, target=_tup(p), target_angles=used, at=at_now,
                          angles=ang_now, steps=len(plan), path=how, dry_run=True, notes=notes)
        if gcmd is not None:
            self._grip_asked(arm, opening_of(grip))
        if label and abs(angles[0]) >= FOLD_PITCH:
            self.alias[arm] = (float(angles[1]), float(heading))
        elif not label and open_after >= 0.9:
            self.alias[arm] = None
        angles = used
        self.asked[arm] = angles
        start = self.rows
        halt, blocked, dropped, skipped, lag = "", False, False, 0, 0.0
        prev = cur[:3, 3]
        for k, q in enumerate(plan):
            last = k == len(plan) - 1
            row = self._row(**{arm: q})
            if gcmd is not None:
                row["grip"][arm] = gcmd
            frame = self._send([row], last)
            at = grasp_of(frame, arm)[:3, 3]
            lag = self._lag(arm, frame, q)
            if self._slipped(arm, frame):
                dropped, halt, skipped = True, "Dropped", len(plan) - k - 1
                break
            if not contact and not last and lag > LAG_M and float(np.linalg.norm(at - prev)) < STALL_M:
                blocked, halt, skipped = True, "Blocked", len(plan) - k - 1
                break
            prev = at
        else:
            if plan:
                for _ in range(SETTLE_ROWS):
                    if lag <= SETTLE_M:
                        break
                    before = lag
                    frame = self._send([self._row()], True)
                    lag = self._lag(arm, frame, plan[-1])
                    if self._slipped(arm, frame):
                        dropped, halt = True, "Dropped"
                        break
                    if before - lag < 0.001:
                        break
                if not dropped and not contact and lag > LAG_M:
                    blocked, halt = True, "Blocked"
        if blocked or (dropped and skipped):
            # hold where it is (no pushing on), and look
            frame = self._send([self._row(**{arm: frame.arm_joints(arm)})], True)
        if gcmd is not None and opening_of(grip) <= 0.05:
            self._closed(arm, frame.gripper(arm))
        move = self._move_result(arm, frame, p, angles, start, how, notes, blocked=blocked, dropped=dropped,
                                 lag=lag, skipped=skipped, halt=halt, contact=contact, aim=target)
        self.log({"kind": "goto", "arm": arm, "target": _tup(p), "angles": angles, "path": how, "steps": move.steps,
                  "err_mm": move.err_mm, "lag_mm": move.lag_mm, "halt": halt})
        return move

    def _move_result(self, arm, frame, p, angles, start, how, notes, *, blocked, dropped, lag, skipped, halt,
                     contact, aim=None) -> R.Move:
        """The errors are from the pose asked; reached is judged at the pose the plan went to (``aim``: the
        closest pose the arm reaches, when it reaches the one asked only nearly)."""
        now = grasp_of(frame, arm)
        target_rot = rotation_of(*angles)
        err_mm = 1000.0 * float(np.linalg.norm(now[:3, 3] - p))
        err_deg = turn_deg(g2.tool_rotation(now[:3, :3], arm), target_rot)
        if aim is None:
            reached = err_mm <= REACHED_MM and err_deg <= REACHED_DEG
        else:
            reached = (1000.0 * float(np.linalg.norm(now[:3, 3] - aim[:3, 3])) <= REACHED_MM
                       and turn_deg(now[:3, :3], aim[:3, :3]) <= REACHED_DEG)
        notes = list(notes)
        if dropped:
            notes.insert(0, "what the gripper held slipped out on the way")
        if contact and lag > LAG_M:
            notes.append(f"the grasp point ended {1000 * lag:.0f} mm behind its command: it is pressing on something")
        return R.Move(arm=arm, reached=reached, moved=True, target=_tup(p),
                      target_angles=angles, at=_tup(now[:3, 3]), angles=self.angles_now(arm, angles),
                      err_mm=round(err_mm, 1), err_deg=round(err_deg, 1), steps=self.rows - start, path=how,
                      blocked=blocked, dropped=dropped, lag_mm=round(1000 * lag, 1), skipped=skipped, halt=halt,
                      notes=notes)

    def _goto_both(self, xyz, *, by, pitch, yaw, roll, grip, path, dry_run, contact):
        if xyz is not None or by is None or any(v is not None for v in (pitch, yaw, roll)) or grip is not None:
            raise Refused("goto('both', by=(dx, dy, dz)): both hands move together by the same offset along a straight "
                          "line, keeping their angles; nothing else is given with 'both'")
        if path == "swing":
            raise Refused("goto('both', ...) moves along a straight line only")
        d = vector_of(by)
        frame = self.frame
        plans, targets, curs = {}, {}, {}
        inv = invert(frame.arm_base)
        for arm in g2.ARMS:
            cur = grasp_of(frame, arm)
            target = cur.copy()
            target[:3, 3] = cur[:3, 3] + d
            curs[arm], targets[arm] = cur, target
            sol = self._solve(arm, inv @ target, frame.arm_joints(arm), quick=True)
            plans[arm] = self._line(arm, cur, target, frame.arm_joints(arm), frame.arm_base,
                                    q_goal=sol.q if sol.reached else None)
        angles = {arm: self.angles_now(arm, self.asked[arm]) for arm in g2.ARMS}
        bad = [arm for arm in g2.ARMS if plans[arm] is None]
        if bad or dry_run:
            out = []
            for arm in g2.ARMS:
                ok = plans[arm] is not None
                notes = [] if ok else [f"the {arm} hand cannot move along that straight line from here"]
                out.append(R.Move(arm=arm, reached=ok and not bad, moved=False, target=_tup(targets[arm][:3, 3]),
                                  target_angles=angles[arm], at=_tup(curs[arm][:3, 3]), angles=angles[arm],
                                  steps=len(plans[arm] or []), path="line", dry_run=bool(dry_run), notes=notes))
            return out
        n = max(len(plans["left"]), len(plans["right"]))
        for arm in g2.ARMS:  # the same length: pad the shorter one with its end
            if plans[arm] and len(plans[arm]) < n:
                plans[arm] = plans[arm] + [plans[arm][-1]] * (n - len(plans[arm]))
            elif not plans[arm]:
                plans[arm] = [frame.arm_joints(arm)] * n
        start = self.rows
        halt, blocked, dropped, skipped = "", {a: False for a in g2.ARMS}, {a: False for a in g2.ARMS}, 0
        lags = {a: 0.0 for a in g2.ARMS}
        prev = {a: curs[a][:3, 3] for a in g2.ARMS}
        for k in range(n):
            last = k == n - 1
            frame = self._send([self._row(left=plans["left"][k], right=plans["right"][k])], last)
            stop = False
            for arm in g2.ARMS:
                at = grasp_of(frame, arm)[:3, 3]
                lags[arm] = self._lag(arm, frame, plans[arm][k])
                if self._slipped(arm, frame):
                    dropped[arm], halt, stop = True, "Dropped", True
                elif not contact and not last and lags[arm] > LAG_M and float(np.linalg.norm(at - prev[arm])) < STALL_M:
                    blocked[arm], halt, stop = True, halt or "Blocked", True
                prev[arm] = at
            if stop:
                skipped = n - k - 1
                break
        else:
            for arm in g2.ARMS:
                if not contact and lags[arm] > LAG_M:
                    blocked[arm], halt = True, halt or "Blocked"
        if any(blocked.values()) or (any(dropped.values()) and skipped):
            frame = self._send([self._row(left=frame.arm_joints("left"), right=frame.arm_joints("right"))], True)
        out = []
        for arm in g2.ARMS:
            m = self._move_result(arm, frame, targets[arm][:3, 3], self.asked[arm], start, "line", [],
                                  blocked=blocked[arm], dropped=dropped[arm], lag=lags[arm], skipped=skipped,
                                  halt=halt if (blocked[arm] or dropped[arm]) else "", contact=contact)
            out.append(m)
        self.log({"kind": "goto", "arm": "both", "by": _tup(d), "steps": self.rows - start, "halt": halt})
        return out

    # -- gripper -----------------------------------------------------------------------------------
    def gripper(self, arm, opening) -> R.Grip:
        arm = check_arm(arm)
        g = opening_of(opening)
        cmd = GRIP_OPEN * g
        was_holding = self._holding[arm]
        self._grip_asked(arm, g)
        start = self.rows
        prev = self.frame.gripper(arm)
        still = 0
        for _ in range(GRIP_ROWS):
            row = self._row()
            row["grip"][arm] = cmd
            frame = self._send([row], True)
            r = frame.gripper(arm)
            if abs(r - cmd) <= GRIP_AT:
                break
            still = still + 1 if abs(r - prev) < GRIP_STILL else 0
            if still >= 2:
                break
            prev = r
        r = self.frame.gripper(arm)
        notes = []
        if g <= 0.05:
            holding = self._closed(arm, r)
        else:
            holding = None
            if was_holding:
                notes.append("let go of what it held")
        grip = R.Grip(arm=arm, opening=opening_read(r), width_m=round(gap_m(r), 4), holding=holding,
                      steps=self.rows - start, goal=g, notes=notes)
        self.log({"kind": "gripper", "arm": arm, "goal": g, "reading": round(r, 4), "holding": holding,
                  "steps": grip.steps})
        return grip

    # -- home --------------------------------------------------------------------------------------
    def home(self, arm="both") -> R.Homing:
        arms = g2.ARMS if arm == "both" else (check_arm(arm),)
        start = self.rows
        notes, dropped = [], False
        for a in arms:
            cur = self.grasp_pose(a)
            if cur[2, 3] < self.start_pose[a][2, 3] - HOME_LIFT_M:
                up = cur.copy()
                up[2, 3] += HOME_LIFT_M
                plan = self._line(a, cur, up, self.frame.arm_joints(a), self.frame.arm_base)
                if plan is None:
                    notes.append(f"the {a} hand could not lift straight up first")
                    continue
                for q in plan:
                    frame = self._send([self._row(**{a: q})], False)
                    dropped |= self._slipped(a, frame)
        q0 = {a: self.frame.arm_joints(a) for a in arms}
        big = max(float(np.max(np.abs(self.start_q[a] - q0[a]))) for a in arms)
        n = max(1, math.ceil(big / SWING_RAD)) if big > 1e-4 else 0
        # both arms: the waist turns back to its start with them
        w_from = w_to = None
        w, w0 = self.frame.waist, self.link.first.waist
        if arm == "both" and w is not None and w0 is not None and len(w) > WAIST_YAW:
            w_from = np.asarray(self.link.command["waist"] if self.link.command.get("waist") is not None else w, float)
            w_to = np.asarray(w0, float)
            turn = float(np.max(np.abs(w_to - w_from)))
            if turn < 1e-3:
                w_from = w_to = None
            else:
                n = max(n, math.ceil(turn / WAIST_STEP))
        for i in range(1, n + 1):
            t = i / n
            row = self._row(**{a: q0[a] + t * (self.start_q[a] - q0[a]) for a in arms})
            if w_from is not None:
                row["waist"] = w_from + t * (w_to - w_from)
            frame = self._send([row], i == n)
            for a in arms:
                dropped |= self._slipped(a, frame)
        for _ in range(SETTLE_ROWS):
            lag = max(self._lag(a, self.frame, self.start_q[a]) for a in arms)
            if lag <= SETTLE_M:
                break
            self._send([self._row()], True)
        for a in arms:
            self.asked[a] = self.angles_now(a, self.start_angles[a])
        out = {}
        for a in arms:
            dist, turn = self.home_offset(a)
            out[a] = (round(1000 * dist, 1), turn)
        if dropped:
            notes.insert(0, "what a gripper held slipped out on the way")
        self.log({"kind": "home", "arms": list(arms), "steps": self.rows - start, "off": out})
        return R.Homing(arms=out, steps=self.rows - start, notes=notes, dropped=dropped)

    # -- wait --------------------------------------------------------------------------------------
    def wait(self, n: int) -> int:
        if isinstance(n, bool) or not isinstance(n, (int, np.integer)) or not 1 <= n <= WAIT_MAX:
            raise Refused(f"wait(n): n is a whole number of steps, 1..{WAIT_MAX}")
        n = int(n)
        left = n
        while left > 0:
            k = min(HOLD_CHUNK, left)
            self._send([self._row()] * k, left == k)
            left -= k
        return n

    # -- waist -------------------------------------------------------------------------------------
    def waist(self, yaw=None, *, by=None) -> str:
        w = self.frame.waist
        if w is None or len(w) <= WAIST_YAW:
            raise Refused("the waist's readings are not available in this episode")
        if (yaw is None) == (by is None):
            raise Refused("waist(yaw=degrees) turns the torso to that heading (0 = the start's), waist(by=degrees) "
                          "turns it by that much; positive turns left")
        try:
            value = float(yaw if yaw is not None else by)
        except (TypeError, ValueError):
            raise Refused("waist: the angle is a number of degrees") from None
        if not math.isfinite(value):
            raise Refused("waist: the angle is a number of degrees")
        base = np.asarray(self.link.command["waist"] if self.link.command.get("waist") is not None else w, float).copy()
        now = float(w[WAIST_YAW])
        start_yaw = float(self.link.first.waist[WAIST_YAW]) if self.link.first.waist is not None else 0.0
        goal = start_yaw + math.radians(value) if yaw is not None else now + math.radians(value)
        lim = math.radians(WAIST_LIMIT_DEG)
        clipped = abs(goal) > lim
        goal = max(-lim, min(lim, goal))
        n = math.ceil(abs(goal - now) / WAIST_STEP)
        start = self.rows
        dropped = False
        row = None
        for i in range(1, n + 1):
            row = self._row()
            wv = base.copy()
            wv[WAIST_YAW] = now + (goal - now) * (i / n)
            row["waist"] = wv
            frame = self._send([row], i == n)
            for a in g2.ARMS:
                dropped |= self._slipped(a, frame)
        ramp = self.rows - start
        last = self._waist_yaw(goal)
        while row is not None and self.rows - start - ramp < WAIST_SETTLE_ROWS and abs(goal - last) > WAIST_AT:
            frame = self._send([row] * WAIST_HOLD, True)
            for a in g2.ARMS:
                dropped |= self._slipped(a, frame)
            moved, last = abs(self._waist_yaw(goal) - last), self._waist_yaw(goal)
            if moved < WAIST_STILL:
                break
        for a in g2.ARMS:  # the hands turned with the torso: keep the angles they have now
            self.asked[a] = self.angles_now(a)
        got = self._waist_yaw(goal)
        hands = "; ".join(f"{a} grasp point now {_xyz(self.grasp_pose(a)[:3, 3])}" for a in g2.ARMS)
        text = (f"Waist: yaw {math.degrees(got - start_yaw):.0f}° from the start's ({math.degrees(got - now):+.0f}°), "
                f"{self.rows - start} steps; the arms turned with it: {hands}")
        if clipped:
            text += f"\n  note: the waist turns at most {WAIST_LIMIT_DEG:.0f}° either way"
        if abs(goal - got) > math.radians(2.0):
            text += (f"\n  note: the waist stopped {math.degrees(abs(goal - got)):.0f}° short of the heading asked "
                     "(something may be in the way of an arm or the torso)")
        if dropped:
            text += "\n  note: what a gripper held slipped out on the way"
        self.log({"kind": "waist", "goal_rad": round(goal, 4), "read_rad": round(got, 4), "steps": self.rows - start,
                  "settle_steps": self.rows - start - ramp})
        return text

    def _waist_yaw(self, default: float) -> float:
        w = self.frame.waist
        return float(w[WAIST_YAW]) if w is not None and len(w) > WAIST_YAW else default

    # -- status --------------------------------------------------------------------------------------
    def arm_status(self, arm: str) -> R.ArmStatus:
        pose = self.grasp_pose(arm)
        dist, turn = self.home_offset(arm)
        return R.ArmStatus(arm=arm, at=_tup(pose[:3, 3]), angles=self.angles_now(arm),
                           gripper=opening_read(self.frame.gripper(arm)), goal=float(self.grip_goal[arm]),
                           holding=self._holding[arm], joints=_tup(self.frame.arm_joints(arm)),
                           home_dist_m=dist, home_turn_deg=turn)

    def start_info(self) -> dict:
        return {arm: (_tup(self.start_pose[arm][:3, 3], 3), self.start_angles[arm]) for arm in g2.ARMS}


__all__ = ["G2Driver", "angles_of", "check_arm", "grasp_of", "opening_of", "vector_of"]
