"""The host side of ``robot.*`` on RoboColiseum's G2: RoboDojo's robot calls (same names, arguments and
result types) on the G2's two seven-joint arms, plus ``waist``.

Not a :class:`hexashell.robot.host.RobotHost` (that one is built round the six-joint arms of its
:class:`~hexashell.motion.driver.Driver`): the motions are :class:`.motion.G2Driver`'s, the pictures and
the readings come with the benchmark's requests (:mod:`.link`, :mod:`.obs`), and perception is
:class:`~hexashell.vision.rgbd.DepthScene` on the depth pictures, with the robot's own pixels from the
arms' shapes (:mod:`.mask`).  Errors reach the cell as RobotHost's do.

There is no step count or step limit for the model: the benchmark's are per stage and not told
(it is told the scoring mechanism, no numbers).
"""

from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

from ...errors import Refused
from ...geometry import angle_between
from ...kernel.host import CallFailed, Stop
from ...motion.driver import EpisodeOver, MotionError
from ...robot import results as R
from ...vision import overlay
from ...vision.rgbd import DepthView, spot
from ..armhost import camera_checker
from . import g2
from . import mask as M
from . import obs as O
from .footprint import SQUARE_ASPECT, square_footprint
from .motion import STEP_S, G2Driver

PUBLIC = ("goto", "gripper", "home", "wait", "waist", "find", "measure", "to_world", "to_pixel", "task", "state",
          "image", "look", "camera", "things")
#: the calls that move the robot (each costs steps)
MOTIONS = ("goto", "gripper", "home", "wait", "waist")
check_cam = camera_checker(O.CAMERAS, {"left": "left_wrist", "right": "right_wrist", "hand_left": "left_wrist",
                                       "hand_right": "right_wrist", "cam_head": "head",
                                       "cam_left_wrist": "left_wrist", "cam_right_wrist": "right_wrist"})
#: a camera that moved more than this (m, degrees) since the model last saw its picture: to_world warns
MOVED_M = 0.005
MOVED_DEG = 1.0
#: seconds over_reason waits for the benchmark's verdict of an episode it ended
VERDICT_WAIT_S = 20.0
#: steps held, at most, for a fresh picture while a camera's latest one is black (a dropped frame)
DROP_TRIES = 3


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
                "(a stage ran out of steps, or something left the allowed region)")
    return "the benchmark ended the episode"


class G2Host:
    """``link``: the episode's :class:`.link.EpisodeLink`; ``task_name``: what robot.task names (the base
    task, never a perturbation); ``rec``: the episode's records (frames after motions, one state line per
    request)."""

    def __init__(self, link, *, task_name: str | None = None, scene=None, log: Callable[[dict], None] | None = None,
                 attach: Callable[[str, np.ndarray, str], None] | None = None, rec=None, seed: int = 0):
        self.link = link
        self.task_name = task_name
        self.log = log or (lambda event: None)
        self.attach = attach or (lambda kind, image, caption: None)
        self.rec = rec
        self.driver = G2Driver(link, log=self.log, on_frame=self._on_frame, seed=seed)
        self.verdict: dict | None = None
        self.scene = scene
        self.carry = None
        self._views: tuple[int | None, dict[str, DepthView]] = (None, {})
        #: per camera, its pose when the model last saw its picture (the start pictures, then look's)
        first = link.first
        self.shown: dict[str, np.ndarray] = {cam: c.pose.copy() for cam in O.CAMERAS
                                             if (c := first.camera(cam)) is not None}
        self.looked: dict[str, np.ndarray] = {}
        self.motions = 0
        if scene is not None:
            scene.bind(self.views, hands={arm: (lambda arm=arm: self.driver.grasp_pose(arm)) for arm in g2.ARMS})
            self.carry = scene

    # -- the episode -------------------------------------------------------------------------------
    def over(self) -> bool:
        return self.verdict is not None or self.link.over

    def over_reason(self) -> str:
        v = self.link.verdict
        if v is not None and v.get("ended") and v.get("success") is None:
            v = self.link.wait_verdict(VERDICT_WAIT_S)
        if self.verdict is None or (v is not None and v.get("success") is not None):
            self.verdict = v or {}
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
        return {arm: (d.grasp_pose(arm), d.grip_goal[arm] <= 0.05, d.holding(arm) is True, d.catches[arm])
                for arm in g2.ARMS}

    def _carry(self, before: dict) -> None:
        """Tell the perception what a motion did with the hands (as RobotHost does): let go (the fingers
        opened, what they held slipped out, or they closed anew) and closed on something."""
        d = self.driver
        for arm, (start, squeezing, holding, caught) in before.items():
            opened = squeezing and d.grip_goal[arm] > 0.05
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
        """The cameras' pictures now with depth and the robot's own pixels (a step that holds still to get
        depth when the latest request has none, or a fresh picture for a black one); a camera whose picture
        is still black is left out."""
        names = list(O.CAMERAS) if cams is None else [check_cam(c) for c in cams]
        frame = self._fresh(names)
        if any(c in frame.images and c not in frame.depth for c in names):
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
        self.looked = {}

    # -- motion ------------------------------------------------------------------------------------
    def _goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
              dry_run=False, contact=False, gentle=False):
        moves = self.driver.goto(arm, None if xyz is None else point_of(xyz), by=by, pitch=_num(pitch, "pitch"),
                                 yaw=_num(yaw, "yaw"), roll=_num(roll, "roll"), grip=grip, path=path,
                                 dry_run=bool(dry_run), contact=bool(contact))
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
        return self.driver.gripper(arm, opening)

    def _home(self, arm="both"):
        return self.driver.home(arm)

    def _wait(self, n):
        if isinstance(n, float) and n.is_integer():
            n = int(n)
        steps = self.driver.wait(n)
        return R.Waited(steps=steps, step_s=round(STEP_S, 4))

    def _waist(self, yaw=None, *, by=None):
        return self.driver.waist(yaw, by=by)

    # -- perception --------------------------------------------------------------------------------
    def _need_scene(self):
        if self.scene is None:
            raise Refused("find and measure are not available in this run (no SAM3 service); to_world and the "
                          "pictures work")
        return self.scene

    def _find(self, cam="head", text=None, *, at=None, points=None, labels=None, box=None):
        return self._need_scene().find(check_cam(cam), text, at=at, points=points, labels=labels, box=box)

    def _measure(self, target, *, cams="auto"):
        scene = self._need_scene()
        thing = scene.measure(target, cams=cams)
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

    def _to_world(self, cam, xy, *, z=None, pair=None):
        cam = check_cam(cam)
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
        return R.TaskInfo(instruction=self.link.prompt, step=None, step_limit=None, start=self.driver.start_info(),
                          task_name=self.task_name)

    def _state(self):
        self.driver.refresh()
        return R.Status(step=None, step_limit=None, arms={arm: self.driver.arm_status(arm) for arm in g2.ARMS})

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
