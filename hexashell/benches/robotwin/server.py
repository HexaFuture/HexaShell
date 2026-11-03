"""The RoboTwin 2.0 simulator server: ``python -m hexashell.benches.robotwin.server <fd>``.

Started by the episode side (:class:`hexashell.benches.robotwin.session.RoboTwinSession`) with one
end of a socket pair as file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in
RoboTwin's Python environment.  The motion is planned on the episode side (as for RoboDojo); this
process only runs the environment:

- ``open(task, seed)``: build the episode; replies with its facts and the robot's start readings;
- ``act(action)``: one step (14 joint targets and openings); replies with the robot's readings;
- ``observe(cams, robot_mask, depth, parts)``: render the cameras;
- ``state``, ``ping``, ``close``.

Every reply carries the episode status (step, horizon, success, over).  The GPU that renders is
``$HEXA_EGL_DEVICE`` (an nvidia-smi index): SAPIEN renders on the CUDA device it is given, so the
index becomes ``CUDA_VISIBLE_DEVICES`` before SAPIEN is imported.
"""

from __future__ import annotations

import os
import socket
import sys
import time


def prepare() -> None:
    """Before SAPIEN is imported: the GPU, the Vulkan driver, the assets."""
    gpu = os.environ.get("HEXA_EGL_DEVICE", "").strip()
    if gpu:
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"  # nvidia-smi's numbering
        os.environ["CUDA_VISIBLE_DEVICES"] = gpu
    if os.path.exists("/etc/vulkan/icd.d/nvidia_icd.json"):
        os.environ.setdefault("VK_ICD_FILENAMES", "/etc/vulkan/icd.d/nvidia_icd.json")
    root = os.environ.get("ROBOTWIN_ASSETS_ROOT")
    if root:
        os.environ.setdefault("ASSETS_PATH", root)
        os.environ.setdefault("ROBOTWIN_ASSETS_PATH", root)


class Server:
    #: RoboTwin's evaluation empties SAPIEN's cache of meshes and textures after every 5th episode
    clear_cache_every = 5

    def __init__(self):
        from .sim import TwinSim

        self.sim = TwinSim()
        self.warm = False
        self.closed = 0

    def handle(self, op: str, args: dict) -> dict:
        from ..armserver import Refused

        if op == "ping":
            return {"pid": os.getpid()}
        if op == "open":
            return self.open(**args)
        if op == "close":  # every episode builds its own scene; a warm server keeps only the process
            if self.sim.env is not None:
                self.closed += 1
                self.sim.close(clear_cache=not self.warm or self.closed % self.clear_cache_every == 0)
            return {}
        if self.sim.env is None:
            raise Refused("no episode: send open first")
        if op == "act":
            try:
                self.sim.act(args["action"])
            except ValueError as exc:
                raise Refused(str(exc)) from None
            return {"proprio": self.sim.proprio()}
        if op == "state":
            return {"proprio": self.sim.proprio()}
        if op == "observe":
            try:
                return {"obs": self.sim.observe(args.get("cams"), robot_mask=bool(args.get("robot_mask", True)),
                                                depth=bool(args.get("depth", True)),
                                                parts=bool(args.get("parts", False)))}
            except ValueError as exc:
                raise Refused(str(exc)) from None
        raise Refused(f"unknown operation {op!r}")

    def status(self) -> dict:
        return self.sim.status()

    def open(self, task: str, seed: int) -> dict:
        t0 = time.time()
        facts = self.sim.open(str(task), int(seed))
        return {"facts": facts, "start": self.sim.proprio(), "seconds": round(time.time() - t0, 2)}


def main(argv: list[str]) -> None:
    prepare()
    from ..armserver import serve

    sock = socket.socket(fileno=int(argv[0]))
    try:
        serve(sock, Server())
    finally:
        sock.close()


if __name__ == "__main__":
    main(sys.argv[1:])
