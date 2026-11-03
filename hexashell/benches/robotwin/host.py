"""The host side of ``robot.*`` for RoboTwin 2.0: RoboDojo's robot calls on RoboTwin's two arms.

The calls, their arguments and their results are RoboDojo's (:class:`hexashell.robot.host.RobotHost`
and its motion :class:`~hexashell.motion.driver.Driver`); what differs is underneath:

- the arms stand where RoboTwin puts them and the table top is at the height measured in the start
  pictures (:mod:`.rig`);
- a step is one ``take_action`` of the simulator server (:mod:`.server`);
- the pictures come with depth, so perception is :class:`~hexashell.vision.rgbd.DepthScene` (an
  outline's pixels back-project to the surface the camera sees) and ``to_world`` without ``z`` or
  ``pair`` reads the depth picture.  Without depth (the setting ``depth`` false), the pictures come
  with the robot's parts in them instead and perception is RoboDojo's, from the colours and the
  camera models alone (:class:`.scene.TwinScene`); the table top is then matched between the head
  picture and the wrist cameras' (:func:`stereo_table_height`), or read from where its far edge shows
  in the head picture where it is too plain to match (:func:`edge_table_height`).

Pictures are rendered when a call needs them, once per step and camera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from ...errors import Refused
from ...kernel.host import CallFailed, Stop
from ...motion.driver import EpisodeOver
from ...robot.host import RobotHost
from ...state import ArmReading, Snapshot
from ...vision.cameras import Camera, check_cam
from ...vision.depth import level_depth
from ...vision.rgbd import DepthView, spot
from ...vision.spots import on_plane, pixel_of
from ..session import SimError, SimRefused
from . import protocol as P
from .rig import rig_for

ARMS = ("left", "right")
#: the table top is looked for between these heights (m) in the first head picture
TABLE_SEARCH_Z = (0.5, 0.9)
TABLE_BIN_M = 0.002
#: Without depth, the table top is matched between pictures at head pixels every TABLE_GRID_PX over the
#: table, at heights from TABLE_LOWEST (m) below the task's usual top up (the most an episode lowers
#: it).  The heights found group where they lie within TABLE_GAP (m) of each other, and the table top
#: is the lowest group of at least TABLE_MIN_MATCHES no higher than its usual top: objects stand on it,
#: and where the table is plain only objects match.  (Learning seeds 10-28, 24 start pictures against
#: their depth pictures: within 0.6 mm in 21, too few on the 3 plain tables; the most common height, the
#: rule before, was an object's top in 2 of them - 43 and 106 mm up - and found none in 5.)
TABLE_GRID_PX = 10
TABLE_LOWEST = 0.03
TABLE_GAP = 0.003
TABLE_MIN_MATCHES = 5
#: Where too few heights agree, the table top is where its far edge (y = TABLE_Y_HALF) shows in the
#: head picture: the height, in TABLE_EDGE_STEP_M steps from TABLE_LOWEST + 1 cm below the usual top to
#: 1 cm above it, at which the colours change most across the edge's line (1 px either side, summed over
#: RGB) at the median of its points off the robot; none when that change is under TABLE_EDGE_MIN_STEP or
#: under TABLE_EDGE_STANDOUT times its median over the heights tried.  (The same 24 start pictures: within
#: 6 mm in all, and 0.8, 0.8 and 4.3 mm low on the 3 plain tables, where the change was 60-69, against
#: 7-8 with the edge painted over.)
TABLE_EDGE_STEP_M = 0.0005
TABLE_EDGE_MIN_STEP = 20.0
TABLE_EDGE_STANDOUT = 2.0
#: spots this far (m) off the table are warned of
TABLE_MARGIN = 0.05


@dataclass
class Shot:
    """One camera's picture at one step without depth: the colours, the camera model and the robot's
    parts (numbers of :data:`.protocol.ROBOT_PARTS`, 0 where the robot is not)."""

    cam: str
    image: np.ndarray  # RGB uint8 (H, W, 3)
    camera: Camera
    parts: np.ndarray  # uint8 (H, W)
    step: int = 0

    @property
    def robot(self) -> np.ndarray:
        return self.parts > 0


def closing_yaw(direction_xy) -> float:
    """Yaw at which a gripper pointing down closes along ``direction_xy`` (as on RoboDojo: at yaw 0
    the fingers close along world x), in (-90, 90]."""
    yaw = math.degrees(math.atan2(float(direction_xy[1]), float(direction_xy[0])))
    while yaw <= -90.0:
        yaw += 180.0
    while yaw > 90.0:
        yaw -= 180.0
    return round(yaw, 1)


def snapshot(proprio: dict, *, seq: int, step: int | None, facts: dict) -> Snapshot:
    arms = {arm: ArmReading(joints=np.asarray(proprio[arm]["joints"], float).reshape(6),
                            flange=np.asarray(proprio[arm]["flange"], float).reshape(4, 4),
                            gripper=float(proprio[arm]["gripper"])) for arm in ARMS}
    return Snapshot(seq=seq, arms=arms, images={}, instruction=str(facts.get("instruction") or ""), step=step,
                    step_limit=int(facts.get("step_limit") or P.step_limit(str(facts.get("task")))),
                    info={"task_name": facts.get("task"), "seed": facts.get("seed")})


def table_height(view: DepthView, x_range, y_range) -> float | None:
    """Height of the table top seen by the head camera: the most common height of the bare surface
    within the table's extent."""
    valid = view.valid() & ~view.robot
    ys, xs = np.nonzero(valid)
    if not len(xs):
        return None
    pts = view.back_project(xs.astype(float), ys.astype(float), view.depth[ys, xs].astype(float))
    lo, hi = TABLE_SEARCH_Z
    keep = ((pts[:, 2] > lo) & (pts[:, 2] < hi) & (pts[:, 0] > x_range[0] + 0.02) & (pts[:, 0] < x_range[1] - 0.02)
            & (pts[:, 1] > y_range[0] + 0.02) & (pts[:, 1] < y_range[1] - 0.02))
    z = pts[keep, 2]
    if len(z) < 500:
        return None
    return most_common(z, lo, hi)


def most_common(z: np.ndarray, lo: float, hi: float) -> float:
    """The most common of these heights (TABLE_BIN_M bins between lo and hi), refined by the median of
    those within 4 mm of it."""
    counts, edges = np.histogram(z, bins=np.arange(lo, hi + TABLE_BIN_M, TABLE_BIN_M))
    k = int(np.argmax(counts))
    mode = 0.5 * (edges[k] + edges[k + 1])
    near = z[np.abs(z - mode) < 0.004]
    return float(np.median(near))


def stereo_table_height(shots: dict[str, Shot], x_range, y_range,
                        top: float = P.TABLE_TOP_Z) -> tuple[float | None, int, int]:
    """Height of the table top (at ``top`` before the episode lowers it) from the start pictures without
    depth: at head pixels over the table, the height at which the level patch around the pixel matches
    the wrist cameras' pictures (:func:`level_depth`); the table top from those heights (see TABLE_GAP;
    None when no group has enough), how many put it there and how many heights were found."""
    head = shots["head"]
    camera = head.camera
    xs, ys = np.meshgrid(np.arange(TABLE_GRID_PX // 2, camera.width, TABLE_GRID_PX),
                         np.arange(TABLE_GRID_PX // 2, camera.height, TABLE_GRID_PX))
    pixels = np.stack([xs.ravel(), ys.ravel()], axis=1).astype(float)
    pts = camera.on_plane(pixels, top)
    with np.errstate(invalid="ignore"):
        over = ((pts[:, 0] > x_range[0] + 0.02) & (pts[:, 0] < x_range[1] - 0.02) & (pts[:, 1] > y_range[0] + 0.02)
                & (pts[:, 1] < y_range[1] - 0.02))
    floor = top - TABLE_LOWEST
    zs = []
    for x, y in pixels[over]:
        if head.robot[int(y), int(x)]:
            continue
        match = level_depth(shots, "head", (x, y), floor=floor)
        if match.point is not None:
            zs.append(match.height)
    heights = np.sort(np.array(zs))
    heights = heights[(heights > floor - 0.01) & (heights < top + TABLE_GAP)]
    for group in np.split(heights, np.nonzero(np.diff(heights) > TABLE_GAP)[0] + 1):
        if len(group) >= TABLE_MIN_MATCHES:
            return most_common(group, group[0] - TABLE_BIN_M, group[-1] + TABLE_BIN_M), len(group), len(zs)
    return None, 0, len(zs)


def edge_table_height(shot: Shot, x_range, top: float) -> tuple[float | None, float]:
    """Height of the table top (at ``top`` before the episode lowers it) from where its far edge shows in
    the head picture (see TABLE_EDGE_STEP_M; None when the edge does not stand out), and how much the
    colours change across it."""
    camera = shot.camera
    image = shot.image.astype(np.float32)
    xs = np.linspace(x_range[0] + 0.02, x_range[1] - 0.02, 400)
    zs = np.arange(top - TABLE_LOWEST - 0.01, top + 0.01 + 1e-9, TABLE_EDGE_STEP_M)
    change = np.full(len(zs), np.nan)
    for k, z in enumerate(zs):
        px, depth = camera.project(np.stack([xs, np.full_like(xs, P.TABLE_Y_HALF), np.full_like(xs, z)], axis=1))
        seen = ((depth > 0) & (px[:, 0] >= 0) & (px[:, 0] <= camera.width - 1) & (px[:, 1] >= 2)
                & (px[:, 1] <= camera.height - 3))
        u, v = np.round(px[seen, 0]).astype(int), px[seen, 1]
        off = ~shot.robot[np.round(v).astype(int), u]
        if off.sum() < 100:
            continue
        u, v = u[off], v[off]
        rows = []
        for r in (v - 1.0, v + 1.0):  # the colours 1 px above and below the line, between rows
            r0 = np.floor(r).astype(int)
            f = (r - r0)[:, None]
            rows.append((1 - f) * image[r0, u] + f * image[r0 + 1, u])
        change[k] = float(np.median(np.abs(rows[0] - rows[1]).sum(axis=1)))
    if not np.isfinite(change).any():
        return None, 0.0
    k = int(np.nanargmax(change))
    if change[k] < max(TABLE_EDGE_MIN_STEP, TABLE_EDGE_STANDOUT * float(np.nanmedian(change))):
        return None, float(change[k])
    return float(zs[k]), float(change[k])


class TwinHost(RobotHost):
    """``session``: the :class:`~.session.RoboTwinSession` (call, status, over, fail, over_reason)."""

    def __init__(self, session, opened: dict, *, scene=None, log: Callable[[dict], None] | None = None,
                 attach: Callable[[str, np.ndarray, str], None] | None = None,
                 on_step: Callable[[int, dict, Any], None] | None = None, speed=None, depth: bool = True):
        self.session = session
        self.facts = dict(opened["facts"])
        self.seq = 0
        self.log = log or (lambda event: None)
        #: whether the pictures come with depth (else with the robot's parts: see the module's notes)
        self.depth = depth
        self._views: tuple[int | None, dict[str, DepthView | Shot]] = (None, {})
        first = snapshot(opened["start"], seq=0, step=session.step, facts=self.facts)
        self.snap = first
        x_range, y_range = P.table_x(self.facts["task"]), P.table_y()
        how = {}
        if depth:
            z = table_height(self.views(["head"])["head"], x_range, y_range)
        else:
            shots, top = self.views(), P.table_top(self.facts["task"])
            z, matches, found = stereo_table_height(shots, x_range, y_range, top)
            how = {"by": "matching pictures", "matches": matches, "heights_found": found}
            if z is None:
                z, change = edge_table_height(shots["head"], x_range, top)
                how.update({"by": "the far edge" if z is not None else None, "edge_change": round(change, 1)})
        self.table_measured = z is not None
        #: "the far edge" when the table top was measured from where its far edge shows (good to about 6 mm)
        self.table_by = how.get("by")
        self.table_z = round(z if z is not None else P.table_top(self.facts["task"]), 4)
        rig = rig_for(self.facts["task"], self.table_z)
        super().__init__(None, first, scene=scene, log=log, attach=attach, on_step=on_step, speed=speed, rig=rig)
        self.start_grippers = {arm: first.arms[arm].gripper for arm in ARMS}
        self.log({"kind": "table", "z": self.table_z, "measured": self.table_measured, **how})

    def bind_scene(self, scene) -> None:
        if not self.depth:
            super().bind_scene(scene)  # RoboDojo's Scene reads the pictures from this host
            return
        scene.bind(self.views, hands={arm: (lambda arm=arm: self.driver.grasp_pose(arm)) for arm in self.rig.arms})
        self.carry = scene

    # -- the episode -------------------------------------------------------------------------------
    def over(self) -> bool:
        return self.session.over

    def over_reason(self) -> str:
        return self.session.over_reason()

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        try:
            return super().dispatch(name, args, kwargs)
        except SimRefused as exc:
            raise CallFailed("ValueError", str(exc)) from None
        except SimError as exc:
            self.session.fail(str(exc).splitlines()[0])
            raise Stop(self.session.over_reason()) from None

    # -- the driver's link to the simulator ------------------------------------------------------
    def act(self, action: dict) -> Snapshot:
        if self.session.over:
            raise EpisodeOver(self.session.verdict)
        q = np.concatenate([np.r_[np.asarray(action[f"{arm}_arm_joint_state"], float).reshape(6),
                                  np.asarray(action[f"{arm}_ee_joint_state"], float).reshape(-1)[:1]]
                            for arm in ARMS])
        reply = self.session.call("act", action=q)
        self.seq += 1
        self.snap = snapshot(reply["proprio"], seq=self.seq, step=self.session.step, facts=self.facts)
        self.on_step(self.seq, reply["proprio"], q)
        return self.snap

    # -- pictures --------------------------------------------------------------------------------
    def views(self, cams=None) -> dict[str, DepthView | Shot]:
        """The cameras at this step (each rendered once per step); ``cams``: only those."""
        names = list(P.CAMERAS) if cams is None else [check_cam(c) for c in cams]
        step = self.session.step
        if self._views[0] != step:
            self._views = (step, {})
        have = self._views[1]
        missing = [c for c in names if c not in have]
        if missing:
            obs = self.session.call("observe", cams=missing, robot_mask=self.depth, depth=self.depth,
                                    parts=not self.depth)["obs"]
            for cam in missing:
                c = obs["cameras"][cam]
                camera = Camera(cam, float(c["focal"]), np.asarray(c["pose"], float), int(c["width"]),
                                int(c["height"]))
                image = obs["images"][cam]
                if not self.depth:
                    have[cam] = Shot(cam, image, camera, np.asarray(obs["parts"][cam], np.uint8), step=int(obs["step"]))
                    continue
                robot = obs["robot"].get(cam)
                if robot is None:
                    robot = np.zeros(image.shape[:2], bool)
                have[cam] = DepthView(cam, image, np.asarray(obs["depth"][cam], np.float32), camera,
                                      np.asarray(robot, bool), step=int(obs["step"]))
            self.log({"kind": "render", "cams": missing, "step": obs["step"], "s": obs.get("render_s")})
        return {c: have[c] for c in names}

    def cached_views(self) -> dict[str, DepthView | Shot]:
        """The pictures already rendered at this step (none rendered for this)."""
        step, have = self._views
        return dict(have) if step == self.session.step else {}

    def pictures(self, cams=None) -> dict[str, np.ndarray]:
        return {cam: v.image for cam, v in self.views(cams).items()}

    def pictures_now(self, cams=None) -> dict[str, np.ndarray]:
        return self.pictures(cams)

    def cameras_now(self, cams=None) -> dict:
        return {cam: v.camera for cam, v in self.views(cams).items()}

    # -- perception --------------------------------------------------------------------------------
    def _to_world(self, cam, xy, *, z=None, pair=None):
        cam = check_cam(cam)
        need = [cam]
        if pair is not None:
            try:
                need.append(check_cam(pair[0]))
            except (TypeError, IndexError):
                raise Refused("to_world(pair=(cam2, (x2, y2))): a pixel of the same point in another camera") from None
        if self.depth or z is not None or pair is not None:
            return spot(self.views(need), cam, xy, z=z, pair=pair, check_cam=check_cam)
        # without depth: the surface under the pixel as RoboDojo finds it, else the table top
        if self.scene is not None:
            found = self.scene.surface_spot(cam, xy)
        else:
            camera = self.views(need)[cam].camera
            found = on_plane(camera, pixel_of(xy, camera), self.table_z, "the table top (no perception in this run)",
                             table_z=self.table_z, area=self.table_area())
            note = "assumes the pixel shows the table top; for a point on an object give z= or pair="
            found.warning = note if found.warning is None else f"{note}; {found.warning}"
        if not self.table_measured and found.xyz is not None and abs(found.xyz[2] - self.table_z) < 0.0005:
            note = (f"the table top's height could not be measured (z = {self.table_z:.3f} is assumed; it may be up to "
                    "3 cm lower), so a point on it may be off by a few cm along the camera's line of sight")
            found.warning = note if found.warning is None else f"{note}; {found.warning}"
        robot = self.views(need)[cam].robot
        h, w = robot.shape
        x, y = found.px
        if robot[min(max(int(round(y)), 0), h - 1), min(max(int(round(x)), 0), w - 1)]:
            note = "that pixel shows the robot"
            found.warning = note if found.warning is None else f"{note}; {found.warning}"
        return found

    def table_area(self) -> tuple:
        """The table's extent with a margin (spots off it are warned of)."""
        (x0, x1), (y0, y1) = self.rig.table_x, self.rig.table_y
        return (x0 - TABLE_MARGIN, x1 + TABLE_MARGIN), (y0 - TABLE_MARGIN, y1 + TABLE_MARGIN)

    def _need_scene(self):
        if self.scene is None:
            raise Refused("find and measure are not available in this run (no SAM3 service); to_world and the "
                          "pictures work")
        return self.scene
