"""The LIBERO-PRO simulator server: ``python -m hexashell.benches.libero.server <fd>``.

Started by the episode side (:class:`hexashell.benches.session.BenchSession`) with one end of a
socket pair as file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in the LIBERO-PRO Python environment.  It answers the
requests of :class:`hexashell.benches.armserver.ArmServer` (open, observe, state, reach, goto, arc,
home, gripper, wait, close).  Poses are 4x4 in the arm frame (see :mod:`.sim`); the hand's are of
the tool axes.  ``home`` returns the hand to where the episode started.
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np

from ..armserver import ArmServer, main as serve_main, prepare  # noqa: F401 - prepare: for the warm server
from . import control, sim as S


class Server(ArmServer):
    floor_z = control.FLOOR_Z
    reach_where = "(the arm's base does not move)"
    #: the base cannot drive closer, and targets 30-35 deg short of straight are common: 9 of 111 moves in
    #: seven libero_test replays (7 reached them), against 10 of 401 in RoboCasa.  A motion that
    #: straightens past them gives way (arm.STRETCH_STOP)
    elbow_room = math.radians(30.0)

    def __init__(self):
        super().__init__()
        self.sim: S.LiberoSim | None = None
        self.robot: control.Robot | None = None
        self.start_tcp: np.ndarray | None = None

    def open(self, task: str, seed: int, size: int = 512) -> dict:
        t0 = time.time()
        if self.sim is None or self.sim.task != task or self.sim.size != int(size):
            if self.sim is not None:
                self.sim.close()
            self.sim = None
            self.sim = S.LiberoSim(task, size=int(size))
        facts = self.sim.reset(int(seed))
        self.robot = control.Robot(self.sim)
        self.start_tcp = self.sim.tcp()
        self.home_joints = self.sim.arm_joints()
        return {"facts": facts, "start": self.sim.proprio(), "seconds": round(time.time() - t0, 2),
                "level": self.robot.level()}

    def home_goal(self) -> np.ndarray:
        return self.start_tcp.copy()


if __name__ == "__main__":
    serve_main(sys.argv[1:], Server())
