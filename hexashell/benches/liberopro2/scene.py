"""LIBERO-Pro 2.0's perception without depth: RoboDojo's :class:`~hexashell.vision.scene.Scene` on LIBERO's
pictures and its one arm (the setting ``depth`` false, as the official environment renders no depth).

As on RoboDojo and RoboTwin, 3-D positions come from the colour pictures and the camera models alone: an outline
found again in the other camera and the outlines intersected (:mod:`hexashell.vision.hull`), the same patch of
surface matched between the pictures (:func:`hexashell.vision.depth.level_depth`), a pixel's ray met with a plane
or with the other camera's ray.  What LIBERO-Pro 2.0 gives otherwise: the pictures with the cameras' nominal
models, and the robot's pixels part by part (the arm, its fingers, the rest of the robot) as the server predicts
them from the joint readings on the nominal model (:class:`.sim.RobotPixels`: never the simulator's segmentation);
the table top the objects stand on is the nominal model's (:func:`.sim.nominal_table`; no perturbation moves it).
There is one arm, ``"arm"``, with the wrist camera on its hand; agentview is the head camera.  Where to look from
is asked of the server (``look_from``): whether the arm reaches a pose, and where the robot would show then.  A
camera the camera check does not trust (the host's ``untrusted``, :mod:`.calib`) is left out.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ...errors import Refused
from ...geometry import homogeneous
from ...motion.pose import rotation_of
from ...state import Snapshot
from ...vision import hull
from ...vision.scene import HOVER_SHARE, LOOK_CLEAR, Scene, View, _free_share, solid_points
from ..libero.host import check_cam
from ..robotwin.scene import OBJECT_POINT_NOTE
from . import protocol as P

ARM = "arm"
BODY = P.ROBOT_PARTS.index("body") + 1
FINGERS = P.ROBOT_PARTS.index("fingers") + 1
#: Where to look from: the wrist camera LOOK_FROM (m) from the object's top and looking at it (farther for a big
#: object, whose top then fills HOVER_SHARE of the picture), the hand at the first of LOOK_PITCHES, heading away
#: from the arm's base turned by the first of LOOK_TURNS (degrees), that the arm reaches while leaving LOOK_CLEAR of
#: the object free in the wrist and the agentview picture (else the pose that leaves the most free).  (As
#: RoboTwin's: without depth the agentview camera has to see the object too, for its height.)
LOOK_FROM = 0.20
LOOK_PITCHES = (90, 75, 60, 45)
LOOK_TURNS = (0, 30, -30)
#: Spots this far (m) off the table are warned of.
TABLE_MARGIN = 0.05
#: Where objects are looked for on a floor (arm frame), which has no edge.
FLOOR_AREA = ((-0.3, 1.2), (-0.9, 0.9))



def untrusted_words(cam: str) -> str:
    """Why positions do not come from camera ``cam`` (the host's ``untrusted``)."""
    return (f"the {cam} pictures do not show the robot where the robot's model has that camera (the camera check at "
            "the start), so positions do not come from them in this episode: use the other camera (the "
            f"{cam} pictures still show what is there)")


@dataclass(frozen=True, eq=False)
class LiberoRig:
    """What the perception needs of the robot and its table: the table top's height and extent (on a floor, the
    area objects are looked for in), the one arm, whose grasp point is the hand's pose (no reach to add)."""

    table_z: float
    table_x: tuple
    table_y: tuple
    floor: bool = False
    arms: tuple = (ARM,)
    grasp_reach: float = 0.0


def rig_of(table: dict) -> LiberoRig:
    """The rig of the table :func:`.sim.nominal_table` found."""
    if table.get("floor") or table.get("x") is None:
        return LiberoRig(float(table["z"]), FLOOR_AREA[0], FLOOR_AREA[1], floor=True)
    return LiberoRig(float(table["z"]), tuple(table["x"]), tuple(table["y"]))


@dataclass
class LiberoView(View):
    """A LIBERO picture, with the robot's parts in it as the server predicts them."""

    parts: np.ndarray | None = field(default=None, repr=False)  # uint8 H x W: P.ROBOT_PARTS numbers, 0 off the robot

    def __post_init__(self) -> None:
        self._robot = self.parts > 0

    def labels(self) -> np.ndarray:
        """The arm's parts (the rest of the robot does not move aside, so it counts as no arm)."""
        return np.where(self.parts == BODY, 0, self.parts)

    @staticmethod
    def arm_of(label: int) -> str:
        return ARM

    def arm_mask(self, arm: str) -> np.ndarray:
        return (self.parts > 0) & (self.parts != BODY)

    def fingers(self, arm: str) -> np.ndarray:
        return self.parts == FINGERS


class LiberoScene(Scene):
    """RoboDojo's perception for a LIBERO-Pro 2.0 host whose pictures come without depth."""

    #: SAM3's words for the bare surface the objects stand on, table top or floor (see TABLE_WORDS): in 50 start
    #: pictures (both cameras; dev and learning starts, unperturbed, S14 and S22) they outline a median 98% of the
    #: bare surface (none in 2: matching decides there), with at most 49 pixels on things over 1 cm high, but some
    #: 550 by the stoves and 8-16k on the cabinet and microwave tops of the kitchen scenes 4 and 6 (training tasks
    #: only).  "table" and "table top" miss the table in most wrist pictures or take in objects (up to 41k pixels);
    #: "floor" misses it in most agentview pictures.
    table_words = "work surface"
    head = "agentview"
    order = ("agentview", "wrist")

    def __init__(self, url: str, *, log=None, client=None):
        super().__init__(url, log=log, client=client)
        #: the wrist camera's pose in the frame of the grasp point (it rides on the hand)
        self.mount: np.ndarray | None = None

    # -- the robot's names -----------------------------------------------------------------------
    def wrist_of(self, arm: str) -> str:
        return "wrist"

    def arm_words(self, arm: str) -> str:
        return "arm"

    def arms_words(self, arms) -> str:
        return "arm"

    def gripper_words(self, arm: str) -> str:
        return "gripper"

    def wrist_words(self, arm: str) -> str:
        return "wrist camera"

    def held_words(self, arm: str) -> str:
        return "held by the gripper"

    def check_cam(self, cam) -> str:
        return check_cam(cam)

    def side_words(self, normal) -> str:
        x, y = float(normal[0]), float(normal[1])
        if abs(x) >= abs(y):
            return "+x (far)" if x > 0 else "-x (near)"
        return "+y (left)" if y > 0 else "-y (right)"

    def grip_yaw(self, shape: hull.Shape) -> float | None:
        """LIBERO's convention: pointing down at yaw 0 the fingers close along y, so across the short side at the
        long side's own yaw."""
        if shape.yaw is None:
            return None
        g = float(shape.yaw)
        while g <= -90.0:
            g += 180.0
        while g > 90.0:
            g -= 180.0
        return round(g, 1)

    def _covering(self, arm: str, cam: str, what: str) -> str:
        if cam == "wrist":
            return f"the gripper covers {what} in the wrist picture"
        return f"the arm covers {what} in the {cam} picture"

    def look_command(self, arm: str, point, got) -> str:
        return f"robot.goto(({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}), {self._look_angles(got)})"

    def _look_angles(self, got) -> str:
        return f"pitch={got[2]}, yaw={got[4]}, roll=0"

    # -- the host's robot ------------------------------------------------------------------------
    def _untrusted(self) -> frozenset:
        return getattr(self.host, "untrusted", frozenset())

    def _pictures(self, snap: Snapshot) -> dict[str, View]:
        untrusted = self._untrusted()
        return {cam: LiberoView(cam, snap.seq, shot.camera, shot.image, snap.arms_in_pictures, parts=shot.parts)
                for cam, shot in self.host.views().items() if cam not in untrusted}

    def _cams(self, cams) -> list[str]:
        out = super()._cams(cams)
        kept = [cam for cam in out if cam not in self._untrusted()]
        if not kept:
            raise Refused(untrusted_words(out[0]))
        return kept

    def view(self, cam: str) -> View:
        if cam in self._untrusted():
            raise Refused(untrusted_words(cam))
        return super().view(cam)

    def refresh(self) -> None:
        """The cameras changed (the camera check): the pictures and the wrist camera's mount found again."""
        self._views = None
        self.mount = None

    def _grasp_pose(self, arm: str) -> np.ndarray:
        """The hand as the pictures show it (where it is, but while the readings lag the commands): what is
        measured in them goes by it, and what the hand carries moves with it."""
        return np.asarray(self.host._seen_tcp(), float)

    def _holding(self, arm: str) -> bool:
        return self.host.holding is True

    def grasped(self, tcp, hand: str = ARM, thing: int | None = None) -> None:
        super().grasped(tcp, hand, thing)

    def released(self, tcp, hand: str = ARM) -> None:
        super().released(tcp, hand)

    def _area(self) -> tuple | None:
        if self.rig.floor:
            return None
        (x0, x1), (y0, y1) = self.rig.table_x, self.rig.table_y
        return (x0 - TABLE_MARGIN, x1 + TABLE_MARGIN), (y0 - TABLE_MARGIN, y1 + TABLE_MARGIN)

    def _support(self, shape: hull.Shape, me: int):
        on, on_z, arm = super()._support(shape, me)
        return ("floor" if on == "table" and self.rig.floor else on), on_z, arm

    def _spot_on_object(self, view: View, xy):
        spot, why = super()._spot_on_object(view, xy)
        if spot is not None and spot.warning is None:
            spot.warning = OBJECT_POINT_NOTE
        return spot, why

    # -- where to look from ----------------------------------------------------------------------
    def _mount(self, view: View | None) -> np.ndarray | None:
        if self.mount is None and view is not None:
            self.mount = np.linalg.inv(self._grasp_pose(ARM)) @ view.camera.pose
        return self.mount

    def _look_pose(self, shape: hull.Shape, busy: set[str]):
        """(arm, grasp point, pitch, {camera: share of the object left free}, yaw) of the first pose (see
        LOOK_FROM) that leaves LOOK_CLEAR of the object free in both pictures, else of the reachable one that
        leaves the most free; None when the arm holds something (it would hide the object) or reaches none."""
        if ARM in busy or not shape.ok or shape.size is None or shape.top is None:
            return None
        views = self.views()
        wrist = views.get("wrist")
        mount = self._mount(wrist)
        if wrist is None or mount is None:
            return None
        camera = wrist.camera
        bottom = shape.bottom if shape.bottom is not None else shape.top - shape.size[2]
        corners = solid_points(shape.center, shape.size, shape.yaw, shape.round, bottom, shape.top)
        target = np.array([shape.center[0], shape.center[1], shape.top])
        fits = camera.focal * math.sqrt(float(shape.size[0] * shape.size[1])
                                        / (HOVER_SHARE * camera.width * camera.height))
        heading = math.degrees(math.atan2(float(target[1]), float(target[0])))
        best = None
        for pitch in LOOK_PITCHES:
            for turn_deg in LOOK_TURNS:
                yaw = int(round(heading + turn_deg))
                turn = rotation_of(pitch, yaw, 0.0)
                point = target + max(LOOK_FROM, fits) * (turn @ mount[:3, 2]) - turn @ mount[:3, 3]
                there = self.host.look_from(homogeneous(turn, point))
                if there is None:
                    continue
                clear = {cam: _free_share(there["cameras"][cam], corners, there["parts"][cam] > 0)
                         for cam in self.order if cam in there["parts"]}
                got = (ARM, point, pitch, clear, yaw)
                if min(clear.values()) >= LOOK_CLEAR:
                    return got
                if best is None or min(clear.values()) > min(best[3].values()):
                    best = got
                if pitch == 90:
                    break  # pointing straight down, the heading only turns the wrist picture
        return best
