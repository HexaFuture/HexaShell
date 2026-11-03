"""RoboTwin 2.0's perception without depth: RoboDojo's :class:`~hexashell.vision.scene.Scene` on
RoboTwin's pictures (the setting ``depth`` false).

As on RoboDojo, 3-D positions come from the colour pictures and the camera models alone: an outline
found again in the other cameras and the outlines intersected (:mod:`hexashell.vision.hull`), the
same patch of surface matched between pictures (:func:`hexashell.vision.depth.level_depth`), a
pixel's ray met with a plane or with a second camera's ray.  What RoboTwin gives otherwise: the
pictures, the camera models and the robot's pixels, part by part (each arm, its fingers, the rest of
the robot), come from the simulator at the current step (:meth:`.host.TwinHost.views`); the table and
the hands are the episode's rig (the table top at the height measured in the start pictures); and the
advice where to look from is worked out for RoboTwin's wrist cameras, whose place on the hand is read
from the pictures, with the arm at the pose advised drawn along its links (RoboTwin draws the robot
only where it is).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from ...geometry import homogeneous
from ...motion import ik
from ...motion.chain import ArmChain, arm_chain, link_points
from ...motion.pose import rotation_of
from ...state import Snapshot
from ...vision import hull
from ...vision.cameras import Camera
from ...vision.scene import ARM_SPAN, HOVER_SHARE, LOOK_CLEAR, Scene, View, _free_share, solid_points
from . import protocol as P

BODY = P.ROBOT_PARTS.index("body") + 1
#: Where to look from: the wrist camera LOOK_FROM (m) from the object's top and looking at it (farther
#: for a big object, whose top then fills HOVER_SHARE of the picture), the hand at the first of
#: LOOK_PITCHES and turned by the first of LOOK_TURNS (degrees of yaw) to come from the arm's own side
#: that an arm reaches while leaving LOOK_CLEAR of the object free in that wrist picture and the head
#: picture, the nearer arm first (else the pose that leaves the most free).  (RoboTwin's notes: the hand
#: pointing straight down, its grasp point 10-15 cm over the top, the wrist camera about 20 cm over it;
#: without depth the head camera has to see the object too, for its height.  Learning seeds 10-28: a
#: plate advised with the grasp point slid 16 cm aside instead was cut off at the wrist picture's edge.)
#: The arm at a pose it is not in is drawn as discs ARM_RADIUS (m) wide along its links' centre line.
LOOK_FROM = 0.20
LOOK_PITCHES = (90, 75, 60, 45)
LOOK_TURNS = (0, 20, 40)
ARM_RADIUS = 0.04
#: Spots this far (m) off the table are warned of.
TABLE_MARGIN = 0.05
#: What a spot on a measured object says when nothing else is wrong with it.  (Learning seeds 10-28, 11
#: start pictures, head pixels on raised objects measured from two views: 21 mm off the depth picture's
#: point at the median, 29 of 40 over 1 cm; the point lies on the pixel's ray, so the error is along it.)
OBJECT_POINT_NOTE = ("where the pixel's ray enters the object's measured box: often 1-5 cm off along the ray; for "
                     "where the object is, robot.measure, for one point on it, pair=")


def part(name: str) -> int:
    return P.ROBOT_PARTS.index(name) + 1


@dataclass
class TwinView(View):
    """A RoboTwin picture, with the robot's parts in it as the simulator drew them."""

    parts: np.ndarray | None = field(default=None, repr=False)  # uint8 H x W: P.ROBOT_PARTS numbers, 0 off the robot

    def __post_init__(self) -> None:
        self._robot = self.parts > 0

    def labels(self) -> np.ndarray:
        """The arms' parts (the rest of the robot does not move aside, so it counts as no arm)."""
        return np.where(self.parts == BODY, 0, self.parts)

    @staticmethod
    def arm_of(label: int) -> str:
        return P.ROBOT_PARTS[label - 1].split()[0]

    def arm_mask(self, arm: str) -> np.ndarray:
        return np.isin(self.parts, (part(f"{arm} arm"), part(f"{arm} fingers")))

    def fingers(self, arm: str) -> np.ndarray:
        return self.parts == part(f"{arm} fingers")


class TwinScene(Scene):
    """RoboDojo's perception for a :class:`~.host.TwinHost` whose pictures come without depth."""

    #: SAM3's word for RoboTwin's table top.  (Learning seeds 10-28, 10 start pictures against their depth
    #: pictures: "desk" covered a median 0.89 of the bare table top in the head picture, at least 0.77, and
    #: no raised object; RoboDojo's "floor" found nothing in 5 of the 10 head pictures.)
    table_words = "desk"

    def __init__(self, url: str, *, log=None, client=None):
        super().__init__(url, log=log, client=client)
        #: per arm, its wrist camera's pose in the frame of the grasp point
        self.mounts: dict[str, np.ndarray] = {}

    def _pictures(self, snap: Snapshot) -> dict[str, View]:
        return {cam: TwinView(cam, snap.seq, shot.camera, shot.image, snap.arms, parts=shot.parts)
                for cam, shot in self.host.views().items()}

    def _area(self) -> tuple:
        (x0, x1), (y0, y1) = self.rig.table_x, self.rig.table_y
        return (x0 - TABLE_MARGIN, x1 + TABLE_MARGIN), (y0 - TABLE_MARGIN, y1 + TABLE_MARGIN)

    def _mount(self, arm: str, view: View | None) -> np.ndarray | None:
        """The arm's wrist camera's pose in the frame of its grasp point (it rides on the hand)."""
        if arm not in self.mounts and view is not None:
            self.mounts[arm] = np.linalg.inv(self._grasp_pose(arm)) @ view.camera.pose
        return self.mounts.get(arm)

    def _chain(self, arm: str) -> ArmChain:
        if arm not in self._chains:
            self._chains[arm] = arm_chain(arm, self.rig)
        return self._chains[arm]

    def _look_pose(self, shape: hull.Shape, busy: set[str]):
        """(arm, grasp point, pitch, {camera: share of the object left free}, yaw) of the first pose that has
        a wrist camera look at the object's top from LOOK_FROM with LOOK_CLEAR of it free there and in the
        head picture, else of the pose that leaves the most free, or None when no arm reaches one."""
        snap = getattr(self.host, "snap", None)
        if snap is None or not shape.ok or shape.size is None or shape.top is None:
            return None
        views = self.views()
        head = views.get("head")
        bottom = shape.bottom if shape.bottom is not None else shape.top - shape.size[2]
        corners = solid_points(shape.center, shape.size, shape.yaw, shape.round, bottom, shape.top)
        target = np.array([shape.center[0], shape.center[1], shape.top])
        near = "left" if shape.center[0] < 0 else "right"
        arms = [a for a in (near, "right" if near == "left" else "left") if a not in busy] or [near]
        best = None
        for pitch in LOOK_PITCHES:
            for degrees in LOOK_TURNS:
                for arm in arms:
                    view = views.get(f"{arm}_wrist")
                    mount = self._mount(arm, view)
                    if mount is None:
                        continue
                    camera = view.camera
                    fits = camera.focal * math.sqrt(float(shape.size[0] * shape.size[1])
                                                    / (HOVER_SHARE * camera.width * camera.height))
                    yaw = -degrees if arm == "left" else degrees  # the hand points away from its own side
                    turn = rotation_of(pitch, yaw, 0.0)
                    point = target + max(LOOK_FROM, fits) * (turn @ mount[:3, 2]) - turn @ mount[:3, 3]
                    base = np.asarray(self.rig.bases[arm][0], float)
                    if np.linalg.norm(point - self.rig.grasp_reach * turn[:, 0] - base) > ARM_SPAN:
                        continue
                    pose = homogeneous(turn, point)
                    sol, _ = ik.solve(self._chain(arm), pose, snap.arms[arm].joints)
                    if not (sol.reached and sol.clear):
                        continue
                    wrist = Camera(camera.name, camera.focal, pose @ mount, camera.width, camera.height)
                    clear = {wrist.name: _free_share(wrist, corners, view.arm_mask(arm))}
                    if head is not None:
                        other = "right" if arm == "left" else "left"
                        robot = head.arm_mask(other) | self._drawn(head.camera, arm, sol.q)
                        clear["head"] = _free_share(head.camera, corners, robot)
                    if min(clear.values()) >= LOOK_CLEAR:
                        return arm, point, pitch, clear, yaw
                    if best is None or min(clear.values()) > min(best[3].values()):
                        best = (arm, point, pitch, clear, yaw)
        return best

    def _spot_on_object(self, view: View, xy):
        spot, why = super()._spot_on_object(view, xy)
        if spot is not None and spot.warning is None:
            spot.warning = OBJECT_POINT_NOTE
        return spot, why

    def _look_angles(self, got) -> str:
        # roll too: an open hand asked to point down may otherwise arrive half a turn round (see
        # motion.driver.FLIP_PITCH), its camera then on the other side of it
        return f"pitch={got[2]}, yaw={got[4]}, roll=0"

    def _drawn(self, camera: Camera, arm: str, q) -> np.ndarray:
        """Where the arm at joint angles ``q`` would show in a camera's picture (see ARM_RADIUS)."""
        mask = np.zeros((camera.height, camera.width), np.uint8)
        px, depth = camera.project(link_points(self._chain(arm), q))
        span = 2 * (camera.width + camera.height)
        for (x, y), d in zip(px, depth):
            if d > 0.02 and abs(x) < span and abs(y) < span:
                cv2.circle(mask, (int(round(x)), int(round(y))), max(1, int(round(ARM_RADIUS * camera.focal / d))),
                           1, -1)
        return mask > 0
