"""The host side of ``robot.*`` on a robot run by URAI: the motions are planned and executed by the URAI
service; HexaShell displays its RGB-D pictures and computes pixel/world coordinates without segmentation.

``dispatch`` is what the kernel calls for each robot call.  The step budget counts what URAI took to
execute, in steps of 0.04 s, and is checked before each call; once it is spent the call answers
:class:`~hexashell.kernel.host.Stop`.  A motion URAI will not plan raises NotExecuted with URAI's
reason; one URAI reports as an empty grasp raises Dropped.  Positions are URAI's world frame (metres,
z up) and its TCP; angles are HexaShell's pitch, yaw and roll (:mod:`.frames`).  goto's ``hand_by`` and
``hand_turn`` move in the hand's own axes instead (:func:`.frames.hand_move`); URAI gets the world pose.
``both`` is RoboDojo's ``robot.both``: both arms' motions in one URAI plan, on URAI's one clock.

Unlike RoboDojo's :class:`~hexashell.robot.host.RobotHost`, nothing here simulates or plans the arm: the
robot's own software does, as the benchmarks' servers do for :class:`~hexashell.benches.armhost.ArmHost`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import httpx
import numpy as np

from ...errors import Refused
from ...geometry import angle_between
from ...kernel.host import CallFailed, Stop
from ...robot import results as R
from ...vision import overlay
from ...vision.cameras import check_cam
from ...vision.rgbd import spot
from . import texts
from .frames import HAND_TURN_MAX_DEG, from_rpy, hand_move, tcp_pose, to_rpy
from .observation import CAMERA_TO_URAI, Observation

PUBLIC = ("goto", "gripper", "both", "home", "wait", "to_world", "to_pixel", "task", "state", "image", "look",
          "camera")
#: what each arm of a both() call may ask for: goto's arguments, without dry_run (as RoboDojo's robot.both)
BOTH_ARGS = ("xyz", "by", "pitch", "yaw", "roll", "hand_by", "hand_turn", "grip", "path", "contact", "gentle")
#: both(together=True): how many times the arm that would end first is slowed and the pair planned again
TOGETHER_TRIES = 6
#: the slowest speed URAI plans (m/s)
MIN_SPEED_M_S = 0.001
#: path="auto" takes the joint-space path when the straight line cannot be planned, if it leaves the line by at
#: most SWING_OFF_M and dips at most SWING_DIP_M under its lower end (RoboDojo's rule)
SWING_OFF_M, SWING_DIP_M = 0.03, 0.01
#: gentle=True: RoboDojo's gentle speed at the ends, 0.02 m per 0.04 s action (URAI's own angle and joint
#: limits are stricter still)
GENTLE_SPEED_M_S = 0.02 / 0.04
#: URAI's execution time is counted in steps of this (s)
STEP_S = 0.04
#: how the robot's points are named in what it prints
POINT = "TCP"


@dataclass
class Goto:
    """One arm's goto as URAI is asked for it: from where the arm was measured, to where and at what angles
    (HexaShell's, and URAI's rpy_deg), the opening sent as it starts, and URAI's goto primitive."""

    arm: str
    before: dict
    target: np.ndarray
    angles: tuple[float, float, float]
    rpy: list[float]
    goal: float | None
    spec: dict
    path: str
    contact: bool
    notes: list[str] = field(default_factory=list)
    #: why URAI will not run it (see UraiHost._route)
    refusal: str = ""


class UraiHost:
    #: HexaShell's camera names -> the service's
    cameras = CAMERA_TO_URAI
    #: the camera whose picture a plan is checked against
    plan_camera = "head"
    #: what the model is told
    texts = texts
    gentle_speed_m_s = GENTLE_SPEED_M_S
    #: the robot's calls (the FR3, one arm, has no both)
    public = PUBLIC

    def __init__(self, client, instruction: str, *, task_name: str | None = None, step_limit: int | None = None,
                 speed_m_s: float = 0.12, log, attach, record_state):
        self.client = client
        self.model = client.model()
        self.instruction, self.task_name = instruction, task_name
        self.step, self.step_limit = 0, step_limit
        self.speed_m_s = speed_m_s
        self.log = log
        self.attach = attach
        self.record_state = record_state
        self.observations: dict[str, Observation] = {}
        self.arms: dict[str, dict] = {}
        self.refresh(list(self.cameras))
        self.start = {arm: dict(state) for arm, state in self.arms.items()}
        #: the angles last asked of each arm, and URAI's rpy_deg for them (a goto that gives none keeps them)
        self.asked = {arm: from_rpy(state["rpy_deg"]) for arm, state in self.arms.items()}
        self.aim = {arm: state["rpy_deg"] for arm, state in self.arms.items()}
        #: the opening last asked of each gripper (share of its maximum)
        self.grip_goal = {arm: state["gripper_mm"] / self.model["arms"][arm]["max_opening_mm"]
                          for arm, state in self.arms.items()}
        self.methods = {name: getattr(self, name) for name in self.public}

    @property
    def steps_left(self) -> int | None:
        return None if self.step_limit is None else max(0, self.step_limit - self.step)

    @property
    def over(self) -> bool:
        return self.step_limit is not None and self.step >= self.step_limit

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        if self.over:
            raise Stop(self.texts.BUDGET.format(step=self.step, limit=self.step_limit))
        if name not in self.methods:
            raise CallFailed("RobotError", f"robot has no '{name}'")
        try:
            return self.methods[name](*args, **kwargs)
        except R.NotFound as exc:
            raise CallFailed("NotFound", str(exc)) from exc
        except Refused as exc:
            raise CallFailed("ValueError", str(exc)) from exc

    # -- pictures --------------------------------------------------------------------------------
    def refresh(self, cams: list[str]) -> dict:
        """New pictures of ``cams`` (and the arms as they were taken)."""
        names = {remote: name for name, remote in self.cameras.items()}
        for cam in cams:
            observation = Observation.from_packet(self.client.observe(self.cameras[cam]), step=self.step, names=names)
            self.observations[cam] = observation
            self.arms = observation.arms
        return {cam: self.observations[cam].view for cam in cams}

    def views(self, cams: list[str]) -> dict:
        return self.refresh(cams)

    # -- motion ----------------------------------------------------------------------------------
    def holding(self, arm: str) -> bool | None:
        """From the opening asked and the one measured: False when the fingers are where they were sent,
        None when they stayed about fully open, else True."""
        width = self.arms[arm]["gripper_mm"]
        maximum = self.model["arms"][arm]["max_opening_mm"]
        if width <= maximum * self.grip_goal[arm] + 1.5:
            return False
        return None if width >= maximum - 3.0 else True

    def _plan(self, specs: dict, dry_run: bool = False) -> dict:
        self.refresh([self.plan_camera])
        return self.client.plan(self.observations[self.plan_camera].id, specs, planning_only=dry_run)

    def _execute(self, plan: dict, goals: dict | None = None) -> int:
        """Run a plan (``goals``: the openings it sends the grippers to); its steps."""
        if goals:
            self.grip_goal.update(goals)
        status = self.client.execute(plan)
        execution = status["execution"]
        self.arms = status["arms"]
        steps = math.ceil(execution["execution_time_s"] / STEP_S)
        self.step += steps
        self.record_state({"step": self.step, "arms": self.arms, "execution": execution})
        if "result" in execution and execution["result"].get("outcome") == "empty_grasp":
            raise CallFailed("Dropped", execution["last_error"])
        if execution["state"] == "error":
            raise RuntimeError(execution["error"])
        if execution["state"] == "cancelled":
            raise CallFailed("RobotError", self.texts.CANCELLED)
        return steps

    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, hand_by=None, hand_turn=None,
             grip=None, path="auto", dry_run=False, contact=False, gentle=False):
        move = self._goto(arm, xyz, by=by, pitch=pitch, yaw=yaw, roll=roll, hand_by=hand_by, hand_turn=hand_turn,
                          grip=grip, path=path, contact=contact, gentle=gentle)
        plan = self._route(move, dry_run)
        if plan is None:
            return self._refused(move, dry_run)
        if dry_run:
            return R.Move(arm, True, False, tuple(move.target), move.angles, tuple(move.before["xyz"]),
                          from_rpy(move.before["rpy_deg"]), steps=math.ceil(plan["duration_s"] / STEP_S),
                          path=move.spec["motion"], dry_run=True, notes=move.notes, steps_left=self.steps_left)
        self.asked[arm], self.aim[arm] = move.angles, move.rpy
        steps = self._execute(plan, None if move.goal is None else {arm: move.goal})
        return self._moved(move, plan, steps)

    def _goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, hand_by=None, hand_turn=None,
              grip=None, path="auto", contact=False, gentle=False) -> Goto:
        """What goto's arguments ask of ``arm`` from where it is (measured here), before anything is planned."""
        if path not in ("auto", "line", "swing"):
            raise Refused("path must be 'auto', 'line' or 'swing'")
        requested = (pitch, yaw, roll)
        hand = hand_by is not None or hand_turn is not None
        if hand:
            if xyz is not None or by is not None or requested != (None, None, None):
                raise Refused("hand_by and hand_turn move from the hand's last asked pose along its own axes: "
                              "give them without xyz, by, pitch, yaw or roll")
            shift = (0.0, 0.0, 0.0) if hand_by is None else three_numbers(
                hand_by, "hand_by is three numbers (forward, left, up) in metres")
            turn = (0.0, 0.0, 0.0) if hand_turn is None else three_numbers(
                hand_turn, "hand_turn is three numbers (about forward, about left, about up) in degrees")
            if np.linalg.norm(turn) > HAND_TURN_MAX_DEG:
                raise Refused(f"hand_turn turns {np.linalg.norm(turn):.1f} degrees; one move turns at most "
                              f"{HAND_TURN_MAX_DEG:.0f}: split the turn into two moves")
        self.arms = self.client.state()["arms"]
        before = self.arms[arm]
        if hand:
            # from the measured TCP (as by=) in the axes last asked (as omitted angles); a shift alone keeps
            # the asked orientation as it is
            target, rpy = hand_move(before["xyz"], self.aim[arm], shift, turn)
            if hand_turn is None:
                angles, rpy = self.asked[arm], self.aim[arm]
            else:
                angles = from_rpy(rpy)
        else:
            target = np.array(before["xyz"]) if xyz is None else point_of(xyz)
            if by is not None:
                target = target + point_of(by)
            angles = tuple(old if new is None else float(new) for old, new in zip(self.asked[arm], requested))
            rpy = self.aim[arm] if requested == (None, None, None) else to_rpy(*angles)
        goal = None if grip is None else parse_opening(grip)
        spec = {"primitive": "goto", "xyz": target.tolist(), "rpy_deg": rpy,
                "motion": "swing" if path == "swing" else "line",
                "speed_m_s": min(self.speed_m_s, self.gentle_speed_m_s) if gentle else self.speed_m_s}
        if goal is not None:
            spec["opening_mm"] = goal * self.model["arms"][arm]["max_opening_mm"]
        return Goto(arm, before, target, angles, rpy, goal, spec, path, contact)

    def _route(self, move: Goto, dry_run: bool) -> dict | None:
        """URAI's plan of ``move`` alone, its motion set: path "auto" takes the line, else a joint path near it.
        None when URAI refuses it or the joint path is too far from the line; ``move.refusal`` says why."""
        arm, spec = move.arm, move.spec
        for motion in ("line", "swing") if move.path == "auto" else (move.path,):
            spec["motion"] = motion
            try:
                plan = self._plan({arm: spec}, dry_run)
            except httpx.HTTPStatusError as exc:
                if exc.response.status_code != 409:
                    raise
                reason = exc.response.json()["detail"]
                if move.path == "auto" and motion == "line":
                    move.notes.append(reason)
                    continue
                move.refusal = reason
                return None
            if move.path == "auto" and motion == "swing":
                off, dip = swing_curve(plan["arms"][arm]["xyz"])
                if off > SWING_OFF_M or dip > SWING_DIP_M:
                    move.refusal = self.texts.SWING_REFUSED.format(off=off * 1000, dip=dip * 1000)
                    return None
                move.notes.append(self.texts.SWING_USED.format(off=off * 1000, dip=dip * 1000))
            if move.contact:
                move.notes.append(self.texts.CONTACT)
            return plan

    def _refused(self, move: Goto, dry_run: bool) -> R.Move:
        state = self.arms[move.arm]
        refused = R.Move(move.arm, False, False, tuple(move.target), move.angles, tuple(state["xyz"]),
                         from_rpy(state["rpy_deg"]), dry_run=dry_run, notes=[move.refusal], steps_left=self.steps_left)
        if not dry_run:
            raise CallFailed("NotExecuted", str(refused))
        return refused

    def _moved(self, move: Goto, plan: dict, steps: int) -> R.Move:
        """The Move of an executed ``move``: where the arm is against where it was sent."""
        state = self.arms[move.arm]
        actual, wanted = tcp_pose(state), tcp_pose({"xyz": move.target, "rpy_deg": move.rpy})
        err_mm = float(np.linalg.norm(actual[:3, 3] - move.target) * 1000)
        err_deg = math.degrees(angle_between(actual[:3, :3], wanted[:3, :3]))
        tolerance = plan["pose_tolerance"]
        return R.Move(move.arm, err_mm <= tolerance["position_mm"] and err_deg <= tolerance["orientation_deg"], True,
                      tuple(move.target), move.angles, tuple(state["xyz"]), from_rpy(state["rpy_deg"]),
                      err_mm=err_mm, err_deg=err_deg, steps=steps, path=move.spec["motion"], notes=move.notes,
                      steps_left=self.steps_left)

    def gripper(self, arm, opening):
        goal = parse_opening(opening)
        plan = self._plan({arm: self._gripper_spec(arm, goal)})
        steps = self._execute(plan, {arm: goal})
        return self._gripped(arm, goal, steps)

    def _gripper_spec(self, arm: str, goal: float) -> dict:
        maximum = self.model["arms"][arm]["max_opening_mm"]
        return {"primitive": "gripper", "opening_mm": goal * maximum, "effort": 1000,
                "duration_s": maximum / 100.0 + 0.3}

    def _gripped(self, arm: str, goal: float, steps: int) -> R.Grip:
        maximum = self.model["arms"][arm]["max_opening_mm"]
        width = self.arms[arm]["gripper_mm"]
        return R.Grip(arm, width / maximum, width / 1000, self.holding(arm), steps,
                      goal=goal, steps_left=self.steps_left)

    def both(self, left=None, right=None, *, together=False):
        """RoboDojo's robot.both on URAI: each arm's goto (or gripper) in one plan of both arms, which URAI runs
        on one clock (both start at once, the arm that ends first holds) and checks for the arms' clearance at
        every sample; it costs the steps of the longer motion.  Each arm is first planned alone, as goto's dry
        run; a refusal there or of the pair moves nothing."""
        asked = {}
        for arm, given in (("left", left), ("right", right)):
            if given is None:
                continue
            if not isinstance(given, dict):
                raise Refused(f"both(): {arm} is a dict of goto's arguments, e.g. dict(by=(0, 0, 0.1)) or "
                              "dict(grip='close')")
            unknown = set(given) - set(BOTH_ARGS)
            if unknown:
                raise Refused(f"both(): {arm} has no {sorted(unknown)}; it takes {list(BOTH_ARGS)}")
            if arm not in self.arms:
                raise Refused(f"both(): this robot has no {arm} arm")
            asked[arm] = given
        if not asked:
            raise Refused("both() needs a motion for at least one arm: left=dict(...) or right=dict(...)")
        grips = {arm: parse_opening(given["grip"]) for arm, given in asked.items() if set(given) == {"grip"}}
        moves = {arm: self._goto(arm, **given) for arm, given in asked.items() if arm not in grips}
        for move in moves.values():
            if self._route(move, True) is None:
                raise CallFailed("NotExecuted", f"{self._refused(move, True)}\n{self.texts.BOTH_NOT_EXECUTED}")
        specs = {arm: moves[arm].spec if arm in moves else self._gripper_spec(arm, grips[arm]) for arm in asked}
        plan = self._pair_plan(specs)
        if together and len(moves) == 2:
            plan = self._together(specs, plan)
        for arm, move in moves.items():
            self.asked[arm], self.aim[arm] = move.angles, move.rpy
        goals = {**{arm: m.goal for arm, m in moves.items() if m.goal is not None}, **grips}
        steps = self._execute(plan, goals or None)
        done = {arm: self._moved(moves[arm], plan, own_steps(plan, arm, steps)) if arm in moves
                else self._gripped(arm, grips[arm], own_steps(plan, arm, steps)) for arm in asked}
        return R.Both(left=done.get("left"), right=done.get("right"), steps=steps, steps_left=self.steps_left)

    def _pair_plan(self, specs: dict) -> dict:
        try:
            return self._plan(specs)
        except httpx.HTTPStatusError as exc:
            if exc.response.status_code != 409:
                raise
            raise CallFailed("NotExecuted", f"both(): {exc.response.json()['detail']}\n"
                                            f"{self.texts.BOTH_NOT_EXECUTED}") from None

    def _together(self, specs: dict, plan: dict) -> dict:
        """both(together=True): slow the arm that would end first until both end in the same step.  URAI times
        the two on one clock and stretches both where either needs it, so a guess from URAI's speed profile of
        a line is followed by a search between the speeds that ended it too early and too late."""
        ends = {arm: float(plan["arms"][arm]["time_s"][-1]) for arm in specs}
        slow = min(specs, key=ends.get)
        other = next(arm for arm in specs if arm != slow)
        fastest = specs[slow]["speed_m_s"]
        early, late = None, None  # (speed, gap) of the plans that ended the slowed arm too early (gap > 0), too late
        for attempt in range(TOGETHER_TRIES + 1):
            ends = {arm: float(plan["arms"][arm]["time_s"][-1]) for arm in specs}
            speed, gap = specs[slow]["speed_m_s"], ends[other] - ends[slow]
            if abs(gap) <= STEP_S or attempt == TOGETHER_TRIES:
                break
            if gap > 0.0:
                early = (speed, gap)
            else:
                late = (speed, gap)
            if late is None:
                arm = plan["arms"][slow]
                length, accel = path_length(arm["xyz"]), arm.get("accel_m_s2")
                slower = slowed_speed(length, accel, move_time(length, accel, speed) + gap)
                if slower is None:
                    break
            else:
                # regula falsi on 1/speed, kept inside the middle of the bracket so that it shrinks
                (va, ga), (vb, gb) = early, late
                pa, pb = 1.0 / va, 1.0 / vb
                share = min(max(ga / (ga - gb), 0.1), 0.9)
                slower = 1.0 / (pa + share * (pb - pa))
            slower = min(max(slower, MIN_SPEED_M_S), fastest)
            if slower == speed:
                break
            specs[slow]["speed_m_s"] = slower
            plan = self._pair_plan(specs)
        if abs(gap) <= STEP_S:
            return plan
        raise CallFailed("NotExecuted", self.texts.TOGETHER_REFUSED.format(arm=slow, gap=abs(gap)))

    def home(self, arm="both"):
        arms = list(self.arms) if arm == "both" else [arm]
        plan = self._plan({a: {"primitive": "home", "joints_deg": self.start[a]["joints_deg"],
                               "speed_m_s": self.speed_m_s} for a in arms})
        steps = self._execute(plan)
        offsets = {}
        for a in arms:
            actual, start = tcp_pose(self.arms[a]), tcp_pose(self.start[a])
            offsets[a] = (float(np.linalg.norm(actual[:3, 3] - start[:3, 3]) * 1000),
                          math.degrees(angle_between(actual[:3, :3], start[:3, :3])))
            self.asked[a] = from_rpy(self.arms[a]["rpy_deg"])
            self.aim[a] = self.arms[a]["rpy_deg"]
        return R.Homing(offsets, steps, self.steps_left)

    def wait(self, n):
        if not 1 <= n <= 200 or int(n) != n:
            raise Refused("wait(n): n is a whole number of steps, 1..200")
        plan = self._plan({a: {"primitive": "wait", "duration_s": n * STEP_S} for a in self.arms})
        return R.Waited(self._execute(plan), self.steps_left)

    # -- image geometry --------------------------------------------------------------------------
    def check_cam(self, cam) -> str:
        return check_cam(cam)

    def to_world(self, cam, xy, *, z=None, pair=None):
        cam = self.check_cam(cam)
        names = [cam] if pair is None else [cam, self.check_cam(pair[0])]
        return spot(self.refresh(names), cam, xy, z=z, pair=pair, check_cam=self.check_cam)

    def to_pixel(self, xyz, cam="head"):
        cam = self.check_cam(cam)
        return self.refresh([cam])[cam].camera.pixel(point_of(xyz))

    # -- reading ---------------------------------------------------------------------------------
    def task(self):
        return R.TaskInfo(self.instruction, self.step, self.step_limit,
                          {a: (tuple(s["xyz"]), from_rpy(s["rpy_deg"])) for a, s in self.start.items()},
                          self.task_name, point=POINT)

    def state(self):
        self.arms = self.client.state()["arms"]
        arms = {}
        for arm, state in self.arms.items():
            actual, start = tcp_pose(state), tcp_pose(self.start[arm])
            arms[arm] = R.ArmStatus(arm, tuple(state["xyz"]), from_rpy(state["rpy_deg"]),
                                    state["gripper_mm"] / self.model["arms"][arm]["max_opening_mm"],
                                    self.grip_goal[arm], self.holding(arm), tuple(np.radians(state["joints_deg"])),
                                    float(np.linalg.norm(actual[:3, 3] - start[:3, 3])),
                                    math.degrees(angle_between(actual[:3, :3], start[:3, :3])), point=POINT)
        return R.Status(self.step, self.step_limit, arms)

    def image(self, cam="head"):
        cam = self.check_cam(cam)
        return self.refresh([cam])[cam].image.copy()

    def camera(self, cam="head"):
        cam = self.check_cam(cam)
        return self.refresh([cam])[cam].camera.info()

    def look(self, cams="head", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None):
        names = list(self.cameras) if cams in ("all", None) else [cams] if isinstance(cams, str) else list(cams)
        names = [self.check_cam(c) for c in names]
        if not names:
            raise Refused("look: name at least one camera")
        if crop is not None and len(names) > 1:
            raise Refused("look: crop applies to one camera at a time")
        views = self.refresh(names)
        rendered, notes = [], []
        for cam in names:
            camera = views[cam].camera
            img, more = overlay.render(views[cam].image, cam=cam, grid=bool(grid), marks=marks,
                                       outlines=self._outlines(masks, cam), crop=crop, zoom=zoom,
                                       project=lambda p, c=camera: c.pixel(point_of(p)))
            rendered.append((cam, img))
            notes += more
        extras = [w for w, on in (("grid", grid), ("marks", marks), ("outlines", masks), ("crop", crop)) if on]
        what = ", ".join(c for c, _ in rendered) + (f" ({', '.join(extras)})" if extras else "")
        caption = f"look: {what}" + (f" — {note}" if note else "")
        if tile and len(rendered) > 1:
            self.attach("look", overlay.side_by_side([im for _, im in rendered]), caption + " (side by side)")
        else:
            for cam, img in rendered:
                self.attach("look", img, f"look: {cam}" + (f" ({', '.join(extras)})" if extras else "")
                            + (f" — {note}" if note else ""))
        return R.Picture(cams=[c for c, _ in rendered], what=what + (", side by side" if tile and len(rendered) > 1
                                                                    else ""), notes=notes)

    def _outlines(self, masks, cam: str) -> list[tuple[np.ndarray, str]]:
        if masks is None:
            return []
        out = []
        for item in R.mask_items(masks):
            if isinstance(item, R.Mask):
                if item.cam == cam:
                    out.append((item.pixels, f"#{item.id} {item.label}"))
            elif isinstance(item, R.Thing):
                m = item.masks.get(cam)
                if m is not None:
                    out.append((m.pixels, f"T{item.id} {item.label}"))
            else:
                raise Refused("look(masks=...): give supplied Masks, Things or a Thing's .masks")
        return out

    def close(self) -> None:
        self.client.close()


def swing_curve(points) -> tuple[float, float]:
    """How far a path (URAI's samples of the TCP) leaves the straight line between its ends, and how far it
    dips under the lower end (m)."""
    points = np.asarray(points)
    a, b = points[0], points[-1]
    ab = b - a
    length2 = float(ab @ ab)
    t = np.zeros(len(points)) if length2 < 1e-12 else np.clip((points - a) @ ab / length2, 0, 1)
    off = np.linalg.norm(points - (a + t[:, None] * ab), axis=1).max()
    dip = max(0.0, min(a[2], b[2]) - points[:, 2].min())
    return float(off), float(dip)


def own_steps(plan: dict, arm: str, steps: int) -> int:
    """The steps of ``arm``'s own motion in a plan that took ``steps``: URAI's arms share one clock and the
    one that ends first holds."""
    times = plan["arms"][arm].get("time_s")
    return steps if not times else min(steps, math.ceil(float(times[-1]) / STEP_S))


def path_length(points) -> float:
    """The length of a path of TCP samples (m)."""
    return float(np.linalg.norm(np.diff(np.asarray(points, float), axis=0), axis=1).sum()) if len(points) > 1 else 0.0


def move_time(length: float, accel: float | None, speed: float) -> float:
    """The time of a move of ``length`` from rest to rest: up to ``speed`` and down again at ``accel`` (URAI's
    trapezoid; a triangle when it is too short to reach the speed), or at ``speed`` throughout without ``accel``."""
    if not accel:
        return length / speed
    if speed * speed >= accel * length:
        return 2.0 * math.sqrt(length / accel)
    return length / speed + speed / accel


def slowed_speed(length: float, accel: float | None, duration: float) -> float | None:
    """The top speed at which a move of ``length`` takes ``duration`` (the inverse of :func:`move_time`);
    None when nothing can stretch it so: no length, or ``duration`` shorter than the move at any speed."""
    if length < 1e-4 or duration <= 0.0:
        return None
    if not accel:
        return length / duration
    disc = (accel * duration) ** 2 - 4.0 * accel * length
    return None if disc < 0.0 else (accel * duration - math.sqrt(disc)) / 2.0


def point_of(value) -> np.ndarray:
    """A world point from a tuple, an array, a Spot, a Thing (its centre) or an ArmStatus (its TCP)."""
    if isinstance(value, R.Thing):
        value = value.center
    elif isinstance(value, R.Spot):
        if value.xyz is None:
            raise R.NotFound(f"the Spot has no world point: {value.warning}")
        value = value.xyz
    elif isinstance(value, R.ArmStatus):
        value = value.at
    try:
        p = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused("a world point is three numbers (x, y, z) in metres") from None
    if p.size != 3 or not np.all(np.isfinite(p)):
        raise Refused("a world point is three numbers (x, y, z) in metres")
    return p


def three_numbers(value, refusal: str) -> np.ndarray:
    """Three finite numbers (a tuple, a list or an array), or Refused(``refusal``)."""
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(refusal) from None
    if v.size != 3 or not np.all(np.isfinite(v)):
        raise Refused(refusal)
    return v


def parse_opening(opening) -> float:
    if isinstance(opening, str):
        word = opening.strip().lower()
        if word in ("open", "opened"):
            return 1.0
        if word in ("close", "closed", "shut"):
            return 0.0
        raise Refused(f"gripper opening must be 0..1, 'open' or 'close', not {opening!r}")
    try:
        value = float(opening)
    except (TypeError, ValueError):
        raise Refused(f"gripper opening must be 0..1, 'open' or 'close', not {opening!r}") from None
    if not 0.0 <= value <= 1.0:
        raise Refused(f"gripper opening must be between 0 (closed) and 1 (open), not {value}")
    return value
