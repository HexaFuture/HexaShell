"""The host side of ``robot.*`` on the robots of RoboColiseum Season II (the G2's omnipicker and ctek90d
builds, the G1, the ARX and the Agilex): RoboDojo's robot calls (same names, arguments and result types) on
the arms that move, plus ``waist`` (the G2's turn), and on the WBC board ``drive`` (the chassis), ``body`` (the
torso's lean and height) and ``head``.

Not a :class:`hexashell.robot.host.RobotHost` (that one is built round the six-joint arms of its
:class:`~hexashell.motion.driver.Driver`): the motions are :class:`.motion.RobotDriver`'s, the pictures and
the readings come with the benchmark's requests (:mod:`.link`, :mod:`.obs`), and perception is
:class:`~hexashell.vision.rgbd.DepthScene` on the depth pictures, with the robot's own pixels from the
arms' shapes (:mod:`.mask`).  Errors reach the cell as RobotHost's do.

Each robot motion is one chunk of rows, one reply to the benchmark: one dispatch, what the benchmark
counts against each stage (:mod:`.budget`).  The model is told how many motions it has made, never the
budgets (it is told the scoring mechanism, no numbers).
Looking costs nothing: every motion asks for the depth pictures, so the pictures after it have depth and
calibration, and perception needs no motion of its own except for a black (dropped) picture.

The native runtimes (dexmanip on libuipc, Newton, MuJoCo) play one span of rows for the whole episode
(``protocol.EPISODE_ROWS``, MuJoCo's 60 s of simulated time): a fact of the runtime, so robot.task, robot.state()
and every motion's result say how many steps of it are left (:meth:`ColiseumHost.span`); their requests carry no
depth, so to_world takes a height or a second picture and measure() says that it cannot work.
"""

from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

from ...errors import Refused
from ...geometry import angle_between, invert
from ...kernel.host import CallFailed, Stop
from ...motion.driver import EpisodeOver, MotionError
from ...robot import results as R
from ...vision import overlay
from ...vision.rgbd import DepthView, spot
from ..armhost import camera_checker
from . import mask as M
from . import obs as O
from . import protocol as P
from .footprint import SQUARE_ASPECT, even_extent, square_footprint
from .link import NATIVE
from .motion import RobotDriver

PUBLIC = ("goto", "gripper", "hand", "home", "wait", "waist", "drive", "body", "head", "find", "measure",
          "to_world", "to_pixel", "task", "state", "image", "look", "camera", "things")
#: the calls that move the robot (each costs steps)
MOTIONS = ("goto", "gripper", "hand", "home", "wait", "waist", "drive", "body", "head")
check_cam = camera_checker(O.CAMERAS, {"left": "left_wrist", "right": "right_wrist", "hand_left": "left_wrist",
                                       "hand_right": "right_wrist", "cam_head": "head",
                                       "cam_left_wrist": "left_wrist", "cam_right_wrist": "right_wrist"})
#: a camera that moved more than this (m, degrees) since the model last saw its picture: to_world warns
MOVED_M = 0.005
MOVED_DEG = 1.0
#: seconds over_reason waits for the benchmark's verdict of an episode it ended
VERDICT_WAIT_S = 20.0
#: one-row holds, at most, for a fresh picture while a camera's latest one is black (a dropped frame); each
#: is a dispatch
DROP_TRIES = 2
#: the native runtimes send no depth pictures: what to_world and measure say there instead of reading them
NO_DEPTH_PLACE = ("to_world(cam, (x, y), z=h) gives the point where that pixel's ray meets the height h (a height "
                  "you know: the table's top, read from where a hand stopped on it, or a thing's top from its "
                  "height above the table), and to_world(cam, (x, y), pair=(cam2, (x2, y2))) the point seen at "
                  "a pixel in each of two pictures (a wrist picture and the head's, or both wrists')")
NO_DEPTH_TO_WORLD = ("to_world(cam, (x, y)) alone reads the depth picture, and this task's cameras give no depth "
                     "pictures: " + NO_DEPTH_PLACE)
NO_DEPTH_MEASURE = ("measure() cannot work in this task: it reads sizes and places from the depth pictures, and "
                    "this task's cameras give none. Instead, {find}" + NO_DEPTH_PLACE)
NO_DEPTH_FIND = "find(cam, 'words') outlines a thing in a picture (its Mask's centroid and box are pixels); "


def point_of(value) -> np.ndarray:
    """A world point from three numbers, a Spot, a Thing (its centre) or an ArmStatus (its grasp point)."""
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


def _half_turn(deg: float) -> float:
    """A grasp yaw (a half turn gives the same grasp) in (-90, 90]."""
    while deg <= -90.0:
        deg += 180.0
    while deg > 90.0:
        deg -= 180.0
    return deg


def _num(v, name: str):
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a number of degrees") from None
    if not math.isfinite(f):
        raise Refused(f"{name} is a number of degrees")
    return f


def describe_verdict(verdict: dict | None) -> str:
    v = verdict or {}
    if v.get("abandoned"):
        return f"the evaluation stopped this episode ({v['abandoned']})"
    if v.get("success"):
        return "the benchmark reports SUCCESS: every stage of the task is done"
    if v.get("success") is False and v.get("sub_steps"):
        done = sum(1 for s in v["sub_steps"] if s.get("score", 0) >= 1.0)
        return (f"the benchmark ended the episode with {done} of {len(v['sub_steps'])} stages done "
                "(a stage ran out of time, or something left the allowed region)")
    return "the benchmark ended the episode"


#: the native runtimes: a step's simulated time and the episode's span in words (protocol.STEP_S, EPISODE_ROWS)
STEP_WORDS = {"libuipc": "1/30 s (33 ms)", "newton": "1/60 s (17 ms)", "mujoco": "1/15 s (67 ms)"}
SPAN_WORDS = {"libuipc": "2400 steps (80 s)", "newton": "600 steps (10 s)",
              "mujoco": "60 s of simulated time (900 steps of 1/15 s)"}
#: what the benchmark's steps are, in the native runtimes' words (their prompts speak of a task judged in parts, in
#: no order: the steps describe the task)
NATIVE_STEPS_WORDS = ("they describe the task, not how it is judged: several steps can make one part, and a step may "
                      "not be judged at all")


#: the native runtimes' ends for what happened in the task (link.TASK_ENDS), in the model's words
END_WORDS = {"joint_limit": "a joint was driven past its range",
             "arm_speed_limit": "an arm joint moved faster than it may",
             "self_collision": "the robot touched its own body",
             "cross_arm_collision": "the two arms (or hands) touched each other",
             "drop": "the cube dropped below where it started",
             "wok_below_table": "the pan went below the table's top"}
#: the card: what else ends a native episode at once (MuJoCo and Newton play one task each here, the Rubik's cube and
#: the wok)
EARLY_END_WORDS = {"mujoco": ("a joint driven past its range or moving faster than it may, the two arms or hands "
                              "touching each other, the robot touching its own body, the cube dropping below where "
                              "it started"),
                   "newton": "the pan going below the table's top"}


def describe_native_end(verdict: dict | None, runtime: str, span: tuple[int, int] | None = None) -> str:
    """How a native runtime's episode ended, in the model's words: the cause the runtime gave (``verdict["end"]``,
    link.end_of) - its span used up, a joint past its range, a failure of the simulator - else the span used up by
    our own count (``span``: steps used, steps of the span), and the parts done."""
    v = verdict or {}
    if v.get("abandoned"):
        return f"the evaluation stopped this episode ({v['abandoned']})"
    if v.get("success"):
        return "the benchmark reports SUCCESS: every part of the task is done"
    cause = (v.get("end") or {}).get("cause")
    if cause is None and span is not None and span[0] >= span[1]:
        cause = "span"
    if cause == "span":
        text = f"its span of {SPAN_WORDS[runtime]} is used up, which ends it wherever the task stands"
    elif cause in END_WORDS:
        text = f"the simulator stopped it because {END_WORDS[cause]}"
    elif cause == "failure":
        text = "a failure of the simulator ended it"
    elif cause:
        text = f"the simulator ended it ({cause.replace('_', ' ')})"
    elif span is not None:
        text = f"the simulator ended it at step {span[0]} of its span of {span[1]}, without a reason yet"
    else:
        text = "the benchmark ended it"
    steps = v.get("sub_steps") or []
    if steps and cause != "failure":  # a failed episode is not scored (its parts all read 0)
        text += f"; {sum(1 for s in steps if s.get('score', 0) >= 1.0)} of {len(steps)} parts done"
    return text


class TaskStages(R.TaskInfo):
    """TaskInfo with the benchmark's stage texts (the requests' ``segments``); ``native``: in a native runtime's
    words (:data:`NATIVE_STEPS_WORDS`)."""

    def __init__(self, *args, stages=(), native=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.stages = [str(s) for s in stages]
        self.native = bool(native)

    def __repr__(self) -> str:
        text = super().__repr__()
        if self.stages:
            # the steps of the benchmark's description, not its stages (cover_cup_lid's step 1 is judged by no
            # stage)
            what = (NATIVE_STEPS_WORDS if self.native else
                    "a description, not the stages it judges: several steps can make one stage, and a step may not "
                    "be judged at all")
            text += (f"\nThe benchmark's own description of the task, step by step ({what}):\n"
                     + "\n".join(f"  {k}. {s}" for k, s in enumerate(self.stages, 1)))
        return text

    __str__ = __repr__


def stage_texts(segments) -> list[str]:
    """The stage sentences of a request's ``segments`` (a list of texts, or of dicts with a text)."""
    out = []
    for s in segments or []:
        if isinstance(s, str):
            text = s
        elif isinstance(s, dict):
            text = next((s[k] for k in ("text", "segment", "description", "instruction", "name") if s.get(k)), "")
        else:
            text = ""
        text = " ".join(str(text).split())
        if text:
            out.append(text)
    return out


class ColiseumHost:
    """``link``: the episode's :class:`.link.EpisodeLink`; ``task_name``: what robot.task names (the base
    task, never a perturbation); ``rec``: the episode's records (frames after motions, one state line per
    request); ``physics_hz``: the configuration's physics rate (a row's simulated time); ``mobile``: the chassis
    drives (the WBC board)."""

    def __init__(self, link, *, task_name: str | None = None, scene=None, log: Callable[[dict], None] | None = None,
                 attach: Callable[[str, np.ndarray, str], None] | None = None, rec=None, seed: int = 0,
                 physics_hz: int | None = None, mobile: bool = False):
        self.link = link
        self.task_name = task_name
        self.log = log or (lambda event: None)
        self.attach = attach or (lambda kind, image, caption: None)
        self.rec = rec
        self.driver = RobotDriver(link, log=self.log, on_frame=self._on_frame, seed=seed, physics_hz=physics_hz,
                                  mobile=mobile)
        self.verdict: dict | None = None
        self.scene = scene
        self.carry = None
        self._views: tuple[int | None, dict[str, DepthView]] = (None, {})
        #: per camera, its pose when the model last saw its picture (the start pictures, then look's)
        first = link.first
        self.shown: dict[str, np.ndarray] = {cam: c.pose.copy() for cam in O.CAMERAS
                                             if (c := first.camera(cam)) is not None}
        self.looked: dict[str, np.ndarray] = {}
        #: per camera, the drives done when the model last saw its picture (and when look took one this turn)
        self.shown_drives: dict[str, int] = {cam: 0 for cam in self.shown}
        self.looked_drives: dict[str, int] = {}
        self.motions = 0
        if scene is not None:
            scene.bind(self.views, hands={arm: (lambda arm=arm: self.driver.grasp_pose(arm)) for arm in self.driver.arms})
            self.carry = scene

    # -- the episode -------------------------------------------------------------------------------
    @property
    def native(self) -> bool:
        """The benchmark runs this task on a native runtime (dexmanip on Newton, libuipc, MuJoCo): no depth."""
        return getattr(self.link, "runtime", "isaac") in NATIVE

    def span(self) -> tuple[int, int] | None:
        """On a native runtime, (the steps used, the steps of the episode's one span); None on Isaac (its stages'
        allowances are not told).  Steps are the rows played; MuJoCo's span is 60 s of simulated time, so there they
        are counted from the request's ``simulation_time`` at the nominal step (a row a fast joint stretched counts
        for more), else from the rows."""
        runtime = getattr(self.link, "runtime", "isaac")
        limit = P.EPISODE_ROWS.get(runtime) if runtime in NATIVE else None
        if not limit:
            return None
        used = int(getattr(self.link, "rows", 0))
        t = getattr(self.frame, "sim_time", None)
        if runtime == "mujoco" and t is not None:
            used = max(0, math.ceil(t / P.STEP_S[runtime] - 1e-6))
        return min(used, limit), limit

    def _left(self, result):
        """A motion's result (a list: each one) with the steps left of the episode's span, on a native runtime."""
        span = self.span()
        if span is not None:
            for r in result if isinstance(result, list) else [result]:
                if hasattr(r, "steps_left"):
                    r.steps_left = span[1] - span[0]
        return result

    def over(self) -> bool:
        return self.verdict is not None or self.link.over

    def over_reason(self) -> str:
        v = self.link.verdict
        if v is not None and v.get("ended") and v.get("success") is None:
            v = self.link.wait_verdict(VERDICT_WAIT_S)
        if self.verdict is None or (v is not None and v.get("success") is not None):
            self.verdict = v or {}
        if self.native:
            return describe_native_end(self.verdict, self.link.runtime, self.span())
        return describe_verdict(self.verdict)

    @property
    def frame(self) -> O.Frame:
        """The latest request's readings and pictures."""
        return self.link.latest()

    # -- records -----------------------------------------------------------------------------------
    def _on_frame(self, frame: O.Frame, rows: list[dict]) -> None:
        if self.rec is None:
            return
        record = O.summary(frame)
        record["rows"] = len(rows)
        record["sent"] = self.driver.rows
        record["dispatches"] = self.link.dispatches
        self.rec.state(record)

    def _record_frames(self, name: str) -> None:
        if self.rec is None:
            return
        try:
            frame = self.frame
            self.rec.frames(f"m{self.motions:03d}_{name}", {cam: img for cam, img in frame.images.items()})
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            self.log({"kind": "frames_failed", "error": str(exc)})

    # -- dispatch ----------------------------------------------------------------------------------
    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        if self.over():
            raise Stop(self.over_reason())
        if name not in PUBLIC:
            raise CallFailed("RobotError", f"robot has no '{name}'")
        method = getattr(self, "_" + name)
        start_rows = self.driver.rows
        moving = name in MOTIONS
        hands = self._hands() if self.carry is not None and moving else None
        if moving:
            self.driver.refresh()
        try:
            return method(*args, **kwargs)
        except EpisodeOver:
            into = self.driver.rows - start_rows
            reason = self.over_reason()
            self.log({"kind": "episode_over", "during": name, "into": into, "verdict": self.verdict})
            if self.native:
                # the chunk that was playing is not counted in ``into`` ("0 steps into it" after rows ran)
                raise Stop(f"the episode ended during robot.{name}(): {reason}") from None
            raise Stop(f"{reason} (during robot.{name}(), {into} steps into it)") from None
        except R.NotFound as exc:
            raise CallFailed("NotFound", str(exc)) from None
        except (MotionError, Refused) as exc:
            raise CallFailed("ValueError", str(exc)) from None
        except TypeError as exc:
            if "argument" in str(exc):
                raise CallFailed("TypeError", f"robot.{name}: {exc}") from None
            raise
        finally:
            if hands is not None:
                self._carry(hands)
            if moving and self.driver.rows > start_rows:
                self.motions += 1
                self._record_frames(name)

    def _hands(self) -> dict:
        d = self.driver
        return {arm: (d.grasp_pose(arm), d.squeezing(arm), d.holding(arm) is True, d.catches[arm])
                for arm in self.driver.arms}

    def _carry(self, before: dict) -> None:
        """Tell the perception what a motion did with the hands (as RobotHost does): let go (the fingers
        opened, what they held slipped out, or they closed anew) and closed on something."""
        d = self.driver
        for arm, (start, squeezing, holding, caught) in before.items():
            opened = squeezing and not d.squeezing(arm)
            anew = d.catches[arm] != caught
            if opened or anew or (holding and d.holding(arm) is not True):
                self.carry.released(start if opened else d.grasp_pose(arm), arm)
            if anew and d.holding(arm) is True:
                self.carry.grasped(d.grasp_pose(arm), arm, None)

    # -- pictures ----------------------------------------------------------------------------------
    def _fresh(self, names) -> O.Frame:
        """The latest request, after holding a step or a few while one of these cameras' pictures is black
        (the camera dropped that frame: the camera-quality tasks)."""
        frame, tries = self.frame, 0
        while tries < DROP_TRIES and not self.link.over and any(c in frame.dropped for c in names):
            frame = self.driver.hold()
            tries += 1
            self.log({"kind": "drop_step", "cams": sorted(c for c in names if c in frame.dropped)})
        return frame

    def views(self, cams=None) -> dict[str, DepthView]:
        """The cameras' pictures now with depth and the robot's own pixels (a one-row hold to get depth when
        the latest request has none, or a fresh picture for a black one: a dispatch each); a camera whose
        picture is still black is left out."""
        names = list(O.CAMERAS) if cams is None else [check_cam(c) for c in cams]
        frame = self._fresh(names)
        # the native runtimes (dexmanip on Newton, libuipc, MuJoCo) never send depth: no hold for it there
        if not self.native and any(c in frame.images and c not in frame.depth for c in names):
            if not self.link.over:
                frame = self.driver.hold_for_depth()
                self.log({"kind": "depth_step", "cams": names})
        if self._views[0] != frame.seq:
            self._views = (frame.seq, {})
        have = self._views[1]
        missing = [c for c in names if c not in have]
        if missing:
            arms = M.frame_arms(frame)
            for cam in missing:
                image, camera = frame.images.get(cam), frame.camera(cam)
                if image is None or camera is None or cam in frame.dropped:
                    continue
                depth = frame.depth.get(cam)
                if depth is None:
                    depth = np.full(image.shape[:2], np.nan, np.float32)
                robot = M.robot_mask(camera, depth, arms)
                have[cam] = DepthView(cam, image, depth, camera, robot, step=self.driver.rows, frame_id=str(frame.seq))
        return {c: have[c] for c in names if c in have}

    def _pictures(self, names) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        frame = self.frame
        images, cams = {}, {}
        for cam in names:
            if cam in frame.images:
                images[cam] = frame.images[cam]
                cams[cam] = frame.camera(cam)
        return images, cams

    def pictures_shown(self) -> None:
        """The pictures look took this turn have reached the model."""
        self.shown.update(self.looked)
        self.shown_drives.update(self.looked_drives)
        self.looked = {}
        self.looked_drives = {}

    # -- motion ------------------------------------------------------------------------------------
    def _goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
              dry_run=False, contact=False, gentle=False, speed=None):
        if isinstance(xyz, R.Thing) and self._before_drive(xyz):
            raise Refused(f"{xyz!r} was measured before the robot drove, from where it stood then: measure it again")
        moves = self._left(self.driver.goto(arm, None if xyz is None else point_of(xyz), by=by,
                                            pitch=_num(pitch, "pitch"), yaw=_num(yaw, "yaw"), roll=_num(roll, "roll"),
                                            grip=grip, path=path, dry_run=bool(dry_run), contact=bool(contact),
                                            speed=speed))
        many = isinstance(moves, list)
        listed = moves if many else [moves]
        text = "\n".join(str(m) for m in listed)
        if any(not m.moved and not m.dry_run for m in listed):
            raise CallFailed("NotExecuted", text)
        if any(m.halt == "Dropped" for m in listed):
            rest = ("the rest of the motion and the cell were skipped - look where it went"
                    if any(m.skipped for m in listed) else "the rest of the cell was skipped so you can look")
            raise CallFailed("Dropped", f"{text}\n{rest}")
        if any(m.halt == "Blocked" for m in listed):
            raise CallFailed("Blocked", f"{text}\nthe move ran; the rest of the cell was skipped so you can look")
        return moves

    def _gripper(self, arm, opening):
        return self._dropped(self._left(self.driver.gripper(arm, opening)))

    def _hand(self, arm, shape=None, opening=None, *, fingers=None, dry_run=False):
        return self._dropped(self._left(self.driver.hand(arm, shape, opening, fingers=fingers, dry_run=bool(dry_run))))

    @staticmethod
    def _dropped(result):
        """A five-finger hand's motion during which the other hand lost what it held raises Dropped."""
        if getattr(result, "halt", "") == "Dropped":
            raise CallFailed("Dropped", f"{result}\nthe rest of the cell was skipped so you can look")
        return result

    def _home(self, arm="both"):
        return self._left(self.driver.home(arm))

    def _wait(self, n):
        if isinstance(n, float) and n.is_integer():
            n = int(n)
        steps = self.driver.wait(n)
        # a step's simulated time: the runtime's own on a native one (our rows are paced at row_s everywhere)
        step_s = P.STEP_S.get(getattr(self.link, "runtime", "isaac"), self.driver.row_s)
        return self._left(R.Waited(steps=steps, step_s=round(step_s, 4)))

    def _waist(self, yaw=None, *, by=None):
        return self.driver.waist(yaw, by=by)

    def _drive(self, forward=0.0, left=0.0, turn=0.0):
        text = self.driver.drive(forward, left, turn)
        self._rebase(self.driver.moved_by)
        return text

    def _body(self, lean=None, height=None, *, dry_run=False):
        return self.driver.body(lean, height, dry_run=dry_run)

    def _head(self, yaw=None, pitch=None):
        return self.driver.head(yaw, pitch)

    def _before_drive(self, thing: R.Thing) -> bool:
        """Whether a Thing was measured in pictures taken before the last drive."""
        return bool(self.driver.drives) and thing.step is not None and thing.step < self.driver.drive_end

    def _rebase(self, move: np.ndarray) -> None:
        """The base drove: ``move`` is its pose now in the frame it started from (as the head pictures read it,
        else as asked).  What perception knows of the things no hand holds is re-expressed in the new frame as if
        the drive went so (a held one rode with the hand), and the outlines of the old pictures are forgotten
        (their cameras stood elsewhere)."""
        scene = self.scene
        if scene is None:
            return
        back = invert(move)
        turn = math.degrees(math.atan2(move[1, 0], move[0, 0]))
        for thing_id, rec in scene.records.items():
            if thing_id in scene.carried:
                continue
            thing = rec.thing
            if thing.center_xyz is not None:
                c = back[:3, :3] @ np.asarray(thing.center_xyz, float) + back[:3, 3]
                thing.center_xyz = tuple(round(float(v), 4) for v in c)
            if thing.grip_yaw is not None:
                thing.grip_yaw = round(_half_turn(thing.grip_yaw - turn), 1)
            if rec.points is not None and len(rec.points):
                rec.points = np.asarray(rec.points, float) @ back[:3, :3].T + back[:3, 3]
            if rec.moved is not None:
                rec.moved = back @ rec.moved @ move
        scene.masks.clear()
        self.log({"kind": "rebase", "things": sorted(i for i in scene.records if i not in scene.carried)})

    # -- perception --------------------------------------------------------------------------------
    def _need_scene(self):
        if self.scene is None:
            raise Refused("find and measure are not available in this run (no SAM3 service); to_world and the "
                          "pictures work")
        return self.scene

    def _find(self, cam="head", text=None, *, at=None, points=None, labels=None, box=None):
        return self._need_scene().find(check_cam(cam), text, at=at, points=points, labels=labels, box=box)

    def _measure(self, target, *, cams="auto"):
        if self.native:
            raise Refused(NO_DEPTH_MEASURE.format(find=NO_DEPTH_FIND if self.scene is not None else ""))
        scene = self._need_scene()
        if isinstance(target, R.Thing) and self._before_drive(target) and target.id in scene.records:
            target = scene.records[target.id].thing  # where it is taken to be from here
        thing = scene.measure(target, cams=cams)
        if getattr(thing, "found", False) and self.driver.dexterous:
            self._even_out(scene, thing)
        # the shared measurement calls a square block round (its points spread alike every way) or gives it a
        # yaw from a slight stretch of its points: a square's faces come from its outline
        size = getattr(thing, "size", None)
        if getattr(thing, "found", False) and (thing.round or (size and size[1] > 0 and size[0] / size[1] < SQUARE_ASPECT)):
            rec = scene.records.get(thing.id)
            sq = None if rec is None else square_footprint(rec.points, scene.closing_yaw)
            if sq is not None:
                thing.round = False
                thing.grip_yaw = round(sq.grip_yaw, 1)
                if size:
                    thing.size = (max(sq.across_m, sq.other_m), min(sq.across_m, sq.other_m), size[2])
                thing.notes.append(f"its footprint is about square: grip_yaw closes the fingers across a pair of its "
                                   f"faces (yaw {sq.grip_yaw - 90 if sq.grip_yaw > 0 else sq.grip_yaw + 90:.0f}: across "
                                   "the other pair), not across its corners")
        return thing

    @staticmethod
    def _even_out(scene, thing) -> None:
        """Five-finger hands' wrist cameras look along the fingers from close by: the extents of what one of them
        saw are fitted again on its points thinned evenly (:func:`.footprint.even_extent`)."""
        rec = scene.records.get(thing.id)
        if rec is None or not any(str(v).endswith("_wrist") for v in thing.views):
            return
        ext = even_extent(rec.points, scene.closing_yaw)
        if ext is None:
            return
        thing.center_xyz = tuple(round(v, 4) for v in ext.center)
        thing.size = tuple(round(v, 4) for v in ext.size)
        thing.top = round(ext.top, 4)
        if isinstance(thing.on, str) and thing.on.startswith("z "):
            thing.on = f"z {ext.bottom:.3f}"
        thing.on_z = round(ext.bottom, 4)
        thing.round = ext.round
        thing.grip_yaw = None if ext.grip_yaw is None else round(ext.grip_yaw, 1)

    def _to_world(self, cam, xy, *, z=None, pair=None):
        cam = check_cam(cam)
        if self.native and z is None and pair is None:
            raise Refused(NO_DEPTH_TO_WORLD)
        need = [cam]
        if pair is not None:
            try:
                need.append(check_cam(pair[0]))
            except (TypeError, IndexError):
                raise Refused("to_world(pair=(cam2, (x2, y2))): a pixel of the same point in another camera") from None
        views = self.views(need)
        found = spot(views, cam, xy, z=z, pair=pair, check_cam=check_cam)
        notes = []
        for c in dict.fromkeys(need):
            moved = self._moved_since_shown(c, views.get(c))
            if moved:
                notes.append(moved)
        if notes:
            note = "; ".join(notes)
            found.warning = note if found.warning is None else f"{found.warning}; {note}"
        return found

    def _moved_since_shown(self, cam: str, view: DepthView | None) -> str | None:
        seen = self.shown.get(cam)
        if view is None or seen is None:
            return None
        if self.shown_drives.get(cam, 0) != self.driver.drives:
            return (f"the robot drove since the {cam} picture you last saw: that pixel may show something else now - "
                    "look again")
        now = view.camera.pose
        dist = float(np.linalg.norm(now[:3, 3] - seen[:3, 3]))
        turn = math.degrees(angle_between(seen[:3, :3], now[:3, :3]))
        if dist <= MOVED_M and turn <= MOVED_DEG:
            return None
        return (f"the {cam} camera moved {1000 * dist:.0f} mm / {turn:.0f}° since the {cam} picture you last saw: "
                "that pixel may show something else now - look again")

    def _to_pixel(self, xyz, cam="head"):
        cam = check_cam(cam)
        camera = self.frame.camera(cam)
        if camera is None:
            raise Refused(f"no {cam} camera model in the latest observation")
        px = camera.pixel(point_of(xyz))
        return None if px is None else (round(px[0], 1), round(px[1], 1))

    def _things(self):
        if self.scene is None:
            return R.ThingList()
        return R.ThingList(self.scene.things())

    # -- reading -----------------------------------------------------------------------------------
    def _task(self):
        step, limit = self.span() or (None, None)
        return TaskStages(instruction=self.link.prompt, step=step, step_limit=limit, start=self.driver.start_info(),
                          task_name=self.task_name, stages=stage_texts(self.link.first.segments), native=self.native)

    def _state(self):
        self.driver.refresh()
        step, limit = self.span() or (None, None)
        arms = {arm: self.driver.arm_status(arm) for arm in self.driver.arms}
        return R.Status(step=step, step_limit=limit, arms=arms)

    def _image(self, cam="head"):
        cam = check_cam(cam)
        image = self.frame.images.get(cam)
        if image is None:
            raise Refused(f"no {cam} picture in the latest observation")
        return image.copy()

    def _camera(self, cam="head"):
        cam = check_cam(cam)
        camera = self.frame.camera(cam)
        if camera is None:
            raise Refused(f"no {cam} camera model in the latest observation")
        return camera.info()

    def _look(self, cams="head", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None):
        names = list(O.CAMERAS) if cams in ("all", None) else [cams] if isinstance(cams, str) else list(cams)
        names = [check_cam(c) for c in names]
        if not names:
            raise Refused("look: name at least one camera")
        if crop is not None and len(names) > 1:
            raise Refused("look: crop applies to one camera at a time")
        dropped = self._fresh(names).dropped
        pictures, cameras = self._pictures(names)
        rendered, notes = [], []
        for cam in names:
            image = pictures.get(cam)
            if image is None:
                notes.append(f"no {cam} picture")
                continue
            if cam in dropped:
                notes.append(f"the {cam} picture is black: that camera dropped this moment's picture - look again")
            camera = cameras.get(cam)
            project = None if camera is None else (lambda p, c=camera: c.pixel(point_of(p)))
            img, more = overlay.render(image, cam=cam, grid=bool(grid), marks=marks, outlines=self._outlines(masks, cam),
                                       crop=crop, zoom=zoom, project=project)
            rendered.append((cam, img))
            notes += more
            if camera is not None:
                self.looked[cam] = camera.pose.copy()
                self.looked_drives[cam] = self.driver.drives
        if not rendered:
            raise Refused("look: none of those pictures is available")
        extras = [w for w, on in (("grid", grid), ("marks", marks), ("outlines", masks), ("crop", crop)) if on]
        what = ", ".join(c for c, _ in rendered) + (f" ({', '.join(extras)})" if extras else "")
        caption = f"look: {what}" + (f" — {note}" if note else "")
        if tile and len(rendered) > 1:
            self.attach("look", overlay.side_by_side([im for _, im in rendered]), caption + " (side by side)")
        else:
            for cam, img in rendered:
                self.attach("look", img, f"look: {cam}" + (f" ({', '.join(extras)})" if extras else "")
                            + (f" — {note}" if note else ""))
        return R.Picture(cams=[c for c, _ in rendered],
                         what=what + (", side by side" if tile and len(rendered) > 1 else ""), notes=notes)

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
                raise Refused("look(masks=...): give Masks (from find), Things (from measure) or a Thing's .masks")
        return out
