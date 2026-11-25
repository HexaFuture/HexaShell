"""The LIBERO-Pro 2.0 simulator server: ``python -m hexashell.benches.liberopro2.server <fd>``.

Started by the episode side (:class:`hexashell.benches.session.BenchSession`) with one end of a socket pair as
file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in the LIBERO-Pro 2.0 Python
environment.  It answers the requests of :class:`hexashell.benches.armserver.ArmServer` (open, observe, state,
reach, goto, arc, home, gripper, wait, close), ``look_from`` and the camera check's ``calib_check``, ``calib_move``
and ``calib_fit`` (:mod:`.calib`); ``open`` takes the initial state and whether the cameras render depth,
``observe`` whether to draw the robot's parts (predicted: see :class:`.sim.RobotPixels`), ``look_from`` where the
robot would show with the hand at a pose.  Poses are 4x4 in
the arm frame (see :mod:`.sim`); the hand's are of the tool axes.  ``home`` returns the hand to where the episode
started.  Every reply's status carries the instruction as it stands (a perturbation may change it).
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np

from ..armserver import ArmServer, Refused, main as serve_main, prepare  # noqa: F401 - prepare: for the warm server
from . import calib as C, control, sim as S


class Server(ArmServer):
    floor_z = control.FLOOR_Z
    reach_where = "(the arm's base does not move)"
    #: as LIBERO-PRO's (hexashell.benches.libero.server): the same robots and scenes
    elbow_room = math.radians(30.0)

    def __init__(self):
        super().__init__()
        self.sim: S.LiberoProSim | None = None
        self.robot: control.Robot | None = None
        self.start_tcp: np.ndarray | None = None
        #: the episode's camera check (:class:`.calib.Calibration`), once asked for
        self.calib: C.Calibration | None = None

    def open(self, task: str, seed: int, size: int = 512, init_state: int = 0, depth: bool = False) -> dict:
        t0 = time.time()
        if self.sim is None or self.sim.task != task or self.sim.size != int(size) or self.sim.depth != bool(depth):
            if self.sim is not None:
                self.sim.close()
            self.sim = None
            self.sim = S.LiberoProSim(task, size=int(size), depth=bool(depth))
        facts = self.sim.reset(int(seed), int(init_state))
        self.calib = None
        self.robot = control.Robot(self.sim)
        self.start_tcp = self.sim.tcp()
        self.home_joints = self.sim.arm_joints()
        return {"facts": facts, "start": self.sim.proprio(), "seconds": round(time.time() - t0, 2),
                "level": self.robot.level(), "record": self.sim.record()}

    def home_goal(self) -> np.ndarray:
        return self.start_tcp.copy()

    def op_observe(self, cams=None, robot_mask: bool = True, parts: bool = False) -> dict:
        return {"obs": self.sim.observe(cams, robot_mask=bool(robot_mask), parts=bool(parts))}

    def op_look_from(self, pose, cams=None) -> dict:
        """Whether the hand reaches tool pose ``pose`` (as a goto's check), and if it does, the cameras and the
        robot's predicted pixels with the arm there (:meth:`.sim.LiberoProSim.pictured_at`).  (The verdict is
        ``reaches``, never ``ok``: the reply's own ``ok`` says whether the call worked.)"""
        verdict = self._check(np.asarray(pose, float))
        out = {"reaches": bool(verdict["ok"]), "why": verdict.get("why")}
        if out["reaches"]:
            out.update(self.sim.pictured_at(verdict["joints"], cams))
        return out


    # -- the camera check (:mod:`.calib`) --------------------------------------------------------
    def op_calib_check(self, mask) -> dict:
        """The check at the start, before a step is taken (``mask``: SAM3's outline of the robot in the start's
        agentview picture): {"found": :meth:`.calib.Calibration.check`'s}."""
        if self.sim.depth:
            raise Refused("the camera check is for the pictures without depth")
        if self.sim.steps or self.sim.fixes:
            raise Refused("the camera check is made at the start, before a step is taken")
        self.calib = C.Calibration(self.sim)
        return {"found": self.calib.check(np.asarray(mask, bool))}

    def op_calib_move(self) -> dict:
        """The hand's move for the calibration (:meth:`.calib.Calibration.move`), once the check called for it."""
        if self.calib is None or not self.calib.found.get("need") or "move" in self.calib.found:
            raise Refused("the calibration's move follows a check that calls for it, once")
        return self.calib.move(self)

    def op_calib_fit(self, masks) -> dict:
        """The cameras fitted to the move's shots (``masks``: {step: SAM3's outline of the robot in agentview's
        picture} for its images): {"found": :meth:`.calib.Calibration.fit`'s}."""
        if self.calib is None or "move" not in self.calib.found or "fixes" in self.calib.found:
            raise Refused("the calibration's fit follows its move, once")
        return {"found": self.calib.fit({int(k): np.asarray(v, bool) for k, v in dict(masks).items()})}


if __name__ == "__main__":
    serve_main(sys.argv[1:], Server())
