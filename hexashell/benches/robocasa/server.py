"""The RoboCasa simulator server: ``python -m hexashell.benches.robocasa.server <fd>``.

Started by the episode side (:class:`hexashell.benches.session.BenchSession`) with one end of a
socket pair as file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in the RoboCasa Python environment.  It answers the requests
of :class:`hexashell.benches.armserver.ArmServer` (open, observe, state, reach, goto, arc,
push_arc, home, gripper, wait, close) and ``drive``.  The log rows of a motion end with the base's x, y and
heading.  Poses are 4x4 in the start frame (see :mod:`.sim`); the hand's are of the tool axes.
"""

from __future__ import annotations

import sys
import time

import numpy as np

from ..armserver import ArmServer, Refused, main as serve_main, prepare  # noqa: F401 - for the warm server
from . import control, sim as S


class Server(ArmServer):
    floor_z = control.FLOOR_Z
    reach_where = "with the base where it is"

    def __init__(self):
        super().__init__()
        self.sim: S.CasaSim | None = None
        self.robot: control.Robot | None = None
        self.start_tcp_base: np.ndarray | None = None

    def open(self, task: str, seed: int, size: int = 512, split: str = "pretrain50") -> dict:
        t0 = time.time()
        if self.sim is None or self.sim.task != task or self.sim.size != int(size) or self.sim.split != split:
            if self.sim is not None:
                self.sim.close()
            self.sim = S.CasaSim(task, size=int(size), split=split)
        facts = self.sim.reset(int(seed))
        self.robot = control.Robot(self.sim)
        tcp = self.sim.tcp()
        bx, by, byaw = self.sim.base()
        self.start_tcp_base = np.linalg.inv(S.planar_pose(bx, by, byaw)) @ tcp
        self.home_joints = self.sim.arm_joints()
        return {"facts": facts, "start": self.sim.proprio(), "seconds": round(time.time() - t0, 2),
                "base_controller": self.robot.base.kind, "level": self.robot.level()}

    def home_goal(self) -> np.ndarray:
        """The start pose relative to the base, where the base is now."""
        bx, by, byaw = self.sim.base()
        return S.planar_pose(bx, by, byaw) @ self.start_tcp_base

    def op_drive(self, target, about=None) -> dict:
        target = np.asarray(target, float).reshape(-1)
        if target.size != 3 or not np.all(np.isfinite(target)):
            raise Refused("drive: the target is (x, y, heading in radians)")
        if about is not None:
            about = np.asarray(about, float).reshape(-1)
            if about.size != 2 or not np.all(np.isfinite(about)):
                raise Refused("drive: about is a floor point (x, y)")
        return self._reply(self.robot.drive(target, about))


if __name__ == "__main__":
    serve_main(sys.argv[1:], Server())
