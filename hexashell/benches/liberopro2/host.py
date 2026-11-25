"""The host side of ``robot.*`` in LIBERO-Pro 2.0: LIBERO-PRO's (:mod:`hexashell.benches.libero.host`), with the
instruction as it stands (a perturbation may change it during the episode) and the suite, not the perturbation
case, as the task's name.

Without depth (the setting ``depth`` false, the official environment's pictures) the pictures come with the
robot's parts in them as the server predicts them, and perception is RoboDojo's, from the colours and the camera
models alone (:class:`.scene.LiberoScene`); ``to_world`` without ``z`` or ``pair`` then finds the surface under
the pixel as that perception does, and the table top is the nominal model's (:func:`.sim.nominal_table`).

While the readings lag the commands (:mod:`.timing`) the hand the results give is where the commands sent leave it
(as predicted), and the pictures show it where the readings have it: perception goes by the hand as pictured
(``pictured_arms``), and the results say how far that is from where it is now (:data:`LAG_NOTE_M`).

The camera check at the start (:mod:`.calib`) may re-fit the cameras (the server's pictures and camera models
then go through its fixes) or find a camera's pictures off where the robot's model has that camera: positions then
do not come from that camera (``untrusted``: perception, find, to_world and to_pixel go without it).
"""

from __future__ import annotations

import numpy as np

from ...errors import Refused
from ...state import ArmReading, Snapshot
from ...vision.cameras import Camera
from ...vision.spots import on_plane, pixel_of
from .. import results as R
from ..libero.host import FINGER_STROKE_M, LiberoHost  # noqa: F401  (FINGER_STROKE_M: for the session)
from ..pose import shown_angles, turn_deg
from ..robotwin.host import Shot
from . import protocol as P
from .scene import ARM, TABLE_MARGIN, rig_of, untrusted_words

#: While the readings lag the commands, the results say so when the readings (and so the pictures) show the hand this
#: far (m, degrees) or farther from where it is now.
LAG_NOTE_M, LAG_NOTE_DEG = 0.003, 2.0



def camera_of(cam: str, c: dict) -> Camera:
    return Camera(cam, float(c["focal"]), np.asarray(c["pose"], float), int(c["width"]), int(c["height"]))


class LiberoPro2Host(LiberoHost):
    step_s = P.STEP_S

    def __init__(self, session, opened: dict, *, scene=None, log=None, attach=None, on_rows=None):
        #: whether the pictures come with depth (else with the robot's parts: see the module's notes)
        self.depth = bool(getattr(session, "depth", True))
        super().__init__(session, opened, scene=scene if self.depth else None, log=log, attach=attach,
                         on_rows=on_rows)
        #: the table top (or floor) the objects stand on, as the nominal model has it, and the perception's rig
        self.table = dict(self.facts.get("table") or {})
        self.rig = rig_of(self.table) if self.table else None
        if not self.depth and scene is not None:
            self.scene = scene
            scene.bind(self)
        #: the cameras the camera check (:mod:`.calib`) does not trust: positions do not come from them
        self.untrusted: frozenset = frozenset()

    def recalibrated(self, found: dict) -> None:
        """After the camera check (:func:`.calib.calibrate`) changed the cameras: the pictures taken again (through
        its fixes), and positions without the cameras it does not trust."""
        self.untrusted = frozenset([cam for cam in ("wrist",) if found.get(cam) == "untrusted"])
        self._views = None
        if self.scene is not None:
            self.scene.refresh()

    def trusted(self, cam) -> str:
        """The camera's name (:meth:`check_cam`); refused for a camera positions do not come from."""
        cam = self.check_cam(cam)
        if cam in self.untrusted:
            raise Refused(untrusted_words(cam))
        return cam

    def dispatch(self, name: str, args: tuple, kwargs: dict):
        out = super().dispatch(name, args, kwargs)
        note = self._lag_note()
        if note is not None and isinstance(getattr(out, "notes", None), list):
            out.notes.append(note)
        return out

    def _seen_tcp(self) -> np.ndarray:
        """The hand as the readings (and the pictures) show it: where it is (:meth:`_tcp`) but while they lag."""
        seen = self.proprio.get("seen_tcp")
        return self._tcp() if seen is None else np.asarray(seen, float)

    def _lag_note(self) -> str | None:
        lag = int(self.proprio.get("lag_steps") or 0)
        if not lag:
            return None
        seen, now = self._seen_tcp(), self._tcp()
        off_m, off_deg = float(np.linalg.norm(seen[:3, 3] - now[:3, 3])), turn_deg(seen[:3, :3], now[:3, :3])
        if off_m < LAG_NOTE_M and off_deg < LAG_NOTE_DEG:
            return None
        return (f"the readings and the pictures lag the commands by {lag} steps here (as the robot measures it from "
                f"its own commands): they show the hand {1000 * off_m:.0f} mm and {off_deg:.0f} deg from where it is "
                "now, the pose the results give (predicted from the commands); held still, it shows where it is "
                f"once the readings catch up (robot.wait({lag}))")

    def _task(self):
        return R.TaskInfo(instruction=str(self.session.instruction or ""), step=self.session.step,
                          step_limit=self.session.step_limit,
                          start_at=tuple(round(float(v), 3) for v in self.start_tcp[:3, 3]),
                          start_angles=shown_angles(self.start_tcp[:3, :3]), task_name=self.session.suite)

    # -- for the perception without depth -------------------------------------------------------
    @property
    def snap(self) -> Snapshot:
        """The robot now, as RoboDojo's perception reads it (one arm, ``"arm"``; seq: the step); while the readings
        lag the commands, with the arm as they and the pictures show it (``pictured_arms``)."""
        step = self.session.step
        gripper = min(1.0, max(0.0, self._width() / self.finger_stroke_m))
        reading = ArmReading(joints=np.asarray(self.proprio.get("joints", ()), float), flange=self._tcp(),
                             gripper=gripper)
        snap = Snapshot(seq=int(step or 0), arms={ARM: reading}, images={},
                        instruction=str(self.session.instruction or ""), step=step,
                        step_limit=self.session.step_limit)
        if self.proprio.get("seen_tcp") is not None:
            snap.pictures_behind = int(self.proprio.get("lag_steps") or 0)
            snap.pictured_arms = {ARM: ArmReading(joints=np.asarray(self.proprio["seen_joints"], float),
                                                  flange=self._seen_tcp(), gripper=gripper)}
        return snap

    def views(self, cams=None) -> dict:
        if self.depth:
            return super().views(cams)
        step = self.session.step
        if self._views is None or self._views[0] != step:
            obs = self._call("observe", cams=list(self.cams), robot_mask=False, parts=True)["obs"]
            self._views = (step, {cam: Shot(cam, image, camera_of(cam, obs["cameras"][cam]),
                                            np.asarray(obs["parts"][cam], np.uint8), step=int(obs["step"]))
                                  for cam, image in obs["images"].items()})
        return self._views[1]

    def look_from(self, pose) -> dict | None:
        """With the hand at tool pose ``pose``: {"cameras": {cam: Camera}, "parts": {cam: the robot's parts}}, or
        None when the arm does not reach it (the server's goto check)."""
        reply = self._call("look_from", pose=np.asarray(pose, float), cams=list(self.cams))
        if not reply.get("reaches"):
            return None
        return {"cameras": {cam: camera_of(cam, c) for cam, c in reply["cameras"].items()},
                "parts": {cam: np.asarray(m, np.uint8) for cam, m in reply["parts"].items()}}

    def table_area(self) -> tuple | None:
        """The table's extent with a margin (spots off it are warned of); None on a floor."""
        if self.rig is None or self.rig.floor:
            return None
        (x0, x1), (y0, y1) = self.rig.table_x, self.rig.table_y
        return (x0 - TABLE_MARGIN, x1 + TABLE_MARGIN), (y0 - TABLE_MARGIN, y1 + TABLE_MARGIN)

    def _find(self, cam, text=None, **prompt):
        return super()._find(self.trusted(cam), text, **prompt)

    def _to_pixel(self, xyz, cam):
        return super()._to_pixel(xyz, self.trusted(cam))

    def _to_world(self, cam, xy, *, z=None, pair=None):
        cam = self.trusted(cam)
        if isinstance(pair, (tuple, list)) and pair:
            self.trusted(pair[0])
        if self.depth or z is not None or pair is not None:
            return super()._to_world(cam, xy, z=z, pair=pair)
        cam = self.check_cam(cam)
        if self.scene is not None:
            found = self.scene.surface_spot(cam, xy)
        else:
            camera = self.views()[cam].camera
            top = self.rig.table_z
            found = on_plane(camera, pixel_of(xy, camera), top, "the table top (no perception in this run)",
                             table_z=top, area=self.table_area())
            note = "assumes the pixel shows the table top; for a point on an object give z= or pair="
            found.warning = note if found.warning is None else f"{note}; {found.warning}"
        robot = self.views()[cam].robot
        h, w = robot.shape
        x, y = found.px
        if robot[min(max(int(round(y)), 0), h - 1), min(max(int(round(x)), 0), w - 1)]:
            note = "that pixel shows the robot"
            found.warning = note if found.warning is None else f"{note}; {found.warning}"
        return found

    def _depth(self, cam):
        if self.depth:
            return super()._depth(cam)
        raise Refused("there are no depth pictures in this run: positions come from the colour pictures and the "
                      "camera models (robot.measure, robot.to_world)")

