"""The host side of ``robot.*`` on a robot run by URAI: the motions are planned and executed by the URAI
service; HexaShell displays its RGB-D pictures and computes pixel/world coordinates without segmentation.

``dispatch`` is what the kernel calls for each robot call.  The step budget counts what URAI took to
execute, in steps of 0.04 s, and is checked before each call; once it is spent the call answers
:class:`~hexashell.kernel.host.Stop`.  A motion URAI will not plan raises NotExecuted with URAI's
reason; one URAI reports as an empty grasp raises Dropped.  Positions are URAI's world frame (metres,
z up) and its TCP; angles are HexaShell's pitch, yaw and roll (:mod:`.frames`).

Unlike RoboDojo's :class:`~hexashell.robot.host.RobotHost`, nothing here simulates or plans the arm: the
robot's own software does, as the benchmarks' servers do for :class:`~hexashell.benches.armhost.ArmHost`.
"""

from __future__ import annotations

import math
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
from .frames import from_rpy, tcp_pose, to_rpy
from .observation import CAMERA_TO_URAI, Observation

PUBLIC = ("goto", "gripper", "home", "wait", "to_world", "to_pixel", "task", "state", "image", "look", "camera")
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


class UraiHost:
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
        self.refresh(list(CAMERA_TO_URAI))
        self.start = {arm: dict(state) for arm, state in self.arms.items()}
        #: the angles last asked of each arm, and URAI's rpy_deg for them (a goto that gives none keeps them)
        self.asked = {arm: from_rpy(state["rpy_deg"]) for arm, state in self.arms.items()}
        self.aim = {arm: state["rpy_deg"] for arm, state in self.arms.items()}
        #: the opening last asked of each gripper (share of its maximum)
        self.grip_goal = {arm: state["gripper_mm"] / self.model["arms"][arm]["max_opening_mm"]
                          for arm, state in self.arms.items()}
        self.methods = {name: getattr(self, name) for name in PUBLIC}

    @property
    def steps_left(self) -> int | None:
        return None if self.step_limit is None else max(0, self.step_limit - self.step)

    @property
    def over(self) -> bool:
        return self.step_limit is not None and self.step >= self.step_limit

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        if self.over:
            raise Stop(texts.BUDGET.format(step=self.step, limit=self.step_limit))
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
        for cam in cams:
            observation = Observation.from_packet(self.client.observe(CAMERA_TO_URAI[cam]), step=self.step)
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
        self.refresh(["head"])
        return self.client.plan(self.observations["head"].id, specs, planning_only=dry_run)

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
            raise CallFailed("RobotError", "URAI execution cancelled")
        return steps

    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
             dry_run=False, contact=False, gentle=False):
        if path not in ("auto", "line", "swing"):
            raise Refused("path must be 'auto', 'line' or 'swing'")
        self.arms = self.client.state()["arms"]
        before = self.arms[arm]
        target = np.array(before["xyz"]) if xyz is None else point_of(xyz)
        if by is not None:
            target = target + point_of(by)
        requested = (pitch, yaw, roll)
        angles = tuple(old if new is None else float(new) for old, new in zip(self.asked[arm], requested))
        rpy = self.aim[arm] if requested == (None, None, None) else to_rpy(*angles)
        goal = None if grip is None else parse_opening(grip)
        spec = {"primitive": "goto", "xyz": target.tolist(), "rpy_deg": rpy,
                "motion": "swing" if path == "swing" else "line",
                "speed_m_s": min(self.speed_m_s, GENTLE_SPEED_M_S) if gentle else self.speed_m_s}
        if goal is not None:
            spec["opening_mm"] = goal * self.model["arms"][arm]["max_opening_mm"]
        notes = []
        motions = ("line", "swing") if path == "auto" else (path,)
        for motion in motions:
            spec["motion"] = motion
            try:
                plan = self._plan({arm: spec}, dry_run)
            except httpx.HTTPStatusError as exc:
                if exc.response.status_code != 409:
                    raise
                reason = exc.response.json()["detail"]
                if path == "auto" and motion == "line":
                    notes.append(reason)
                    continue
                return self._refused(arm, target, angles, dry_run, reason)
            if path == "auto" and motion == "swing":
                off, dip = swing_curve(plan["arms"][arm]["xyz"])
                if off > SWING_OFF_M or dip > SWING_DIP_M:
                    reason = texts.SWING_REFUSED.format(off=off * 1000, dip=dip * 1000)
                    return self._refused(arm, target, angles, dry_run, reason)
                notes.append(texts.SWING_USED.format(off=off * 1000, dip=dip * 1000))
            break
        if contact:
            notes.append(texts.CONTACT)
        if dry_run:
            return R.Move(arm, True, False, tuple(target), angles, tuple(before["xyz"]),
                          from_rpy(before["rpy_deg"]), steps=math.ceil(plan["duration_s"] / STEP_S),
                          path=spec["motion"], dry_run=True, notes=notes, steps_left=self.steps_left)
        self.asked[arm], self.aim[arm] = angles, rpy
        steps = self._execute(plan, None if goal is None else {arm: goal})
        state = self.arms[arm]
        actual, wanted = tcp_pose(state), tcp_pose({"xyz": target, "rpy_deg": rpy})
        err_mm = float(np.linalg.norm(actual[:3, 3] - target) * 1000)
        err_deg = math.degrees(angle_between(actual[:3, :3], wanted[:3, :3]))
        tolerance = plan["pose_tolerance"]
        return R.Move(arm, err_mm <= tolerance["position_mm"] and err_deg <= tolerance["orientation_deg"], True,
                      tuple(target), angles, tuple(state["xyz"]), from_rpy(state["rpy_deg"]),
                      err_mm=err_mm, err_deg=err_deg, steps=steps, path=spec["motion"], notes=notes,
                      steps_left=self.steps_left)

    def _refused(self, arm, target, angles, dry_run, reason):
        state = self.arms[arm]
        move = R.Move(arm, False, False, tuple(target), angles, tuple(state["xyz"]), from_rpy(state["rpy_deg"]),
                      dry_run=dry_run, notes=[reason], steps_left=self.steps_left)
        if not dry_run:
            raise CallFailed("NotExecuted", str(move))
        return move

    def gripper(self, arm, opening):
        goal = parse_opening(opening)
        maximum = self.model["arms"][arm]["max_opening_mm"]
        plan = self._plan({arm: {"primitive": "gripper", "opening_mm": goal * maximum,
                                 "effort": 1000, "duration_s": maximum / 100.0 + 0.3}})
        steps = self._execute(plan, {arm: goal})
        width = self.arms[arm]["gripper_mm"]
        return R.Grip(arm, width / maximum, width / 1000, self.holding(arm), steps,
                      goal=goal, steps_left=self.steps_left)

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
    def to_world(self, cam, xy, *, z=None, pair=None):
        cam = check_cam(cam)
        names = [cam] if pair is None else [cam, check_cam(pair[0])]
        return spot(self.refresh(names), cam, xy, z=z, pair=pair, check_cam=check_cam)

    def to_pixel(self, xyz, cam="head"):
        cam = check_cam(cam)
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
        cam = check_cam(cam)
        return self.refresh([cam])[cam].image.copy()

    def camera(self, cam="head"):
        cam = check_cam(cam)
        return self.refresh([cam])[cam].camera.info()

    def look(self, cams="head", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None):
        names = list(CAMERA_TO_URAI) if cams in ("all", None) else [cams] if isinstance(cams, str) else list(cams)
        names = [check_cam(c) for c in names]
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
