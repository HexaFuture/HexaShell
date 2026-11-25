"""The SIMPLE simulator server: ``python -m hexashell.benches.simple.server <fd>``.

Started by the episode side (:class:`hexashell.benches.simple.session.SimpleSession`) with one end of a
socket pair as file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in SIMPLE's Python
environment (bench-simple).  It runs the environment (:mod:`.sim`) and the body commands (:mod:`.control`:
AMO for the MP tasks, GR00T's decoupled WBC for the teleoperation tasks):

- ``open(task, seed, renderer, official_test)``: a fresh layout (protocol, Episodes), or for a test seed its
  official evaluation episode, which needs ``official_test`` true (the episode side sends it only when the run
  allows the test: protocol.OFFICIAL_TEST_VARIABLE) and the evaluation data under ``$SIMPLE_EVAL_DIR``; replies
  with its facts and the robot's start readings.  ``renderer``: who draws the pictures (default
  ``$SIMPLE_RENDERER``, else Isaac Sim);
- ``body(height, pitch, yaw)``, ``walk(x, y, heading)``, ``hand(side, waypoints, hold, dry_run)``,
  ``fingers(side, closed)``, ``gaze(yaw, pitch)``, ``wait(n)``: the commands; each replies with its result,
  the robot's readings and one log row per step it took;
- ``observe(cams, robot_mask)``: the gaze camera's RGB-D picture;
- ``state``, ``ping``, ``close``.

Coordinates in requests and replies are in the start frame (protocol): origin on the ground under the pelvis
at the episode's first step, x along its heading then, y to its left, z up; angles in degrees.  The commands
work in SIMPLE's world, so they are converted here.  Every reply carries the episode status.

Before the simulator is imported (``prepare``): MuJoCo renders on the EGL device of ``$HEXA_EGL_DEVICE`` (an
nvidia-smi index), Isaac Sim on that CUDA device; the X display and the proxies are dropped (an inherited
display can hang Isaac Sim at exit, and the simulator needs no network).
"""

from __future__ import annotations

import math
import os
import socket
import sys
import time

import numpy as np

from . import protocol as P

#: a hand's parts, by the start of its MuJoCo body names (after "<side>_hand_"); the palm is the wrist-yaw link
HAND_PARTS = ("thumb", "index", "middle", "palm")
#: rows of the per-step log: step, pelvis x y z, heading, pelvis height, torso pitch and yaw, left palm xyz,
#: right palm xyz (start frame, degrees)
LOG_COLUMNS = 14


def prepare() -> None:
    """Before SIMPLE (MuJoCo, Isaac Sim) is imported: the GPU, no display, no proxy, and Isaac Sim's EULA
    accepted non-interactively (as SIMPLE's own eval entry points do; the server has no terminal to ask on)."""
    for name in ("DISPLAY", "XAUTHORITY", "http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "all_proxy",
                 "ALL_PROXY"):
        os.environ.pop(name, None)
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("PRIVACY_CONSENT", "N")
    gpu = os.environ.get("HEXA_EGL_DEVICE", "").strip()
    if gpu:
        from .. import egl

        # Isaac Sim's GPU first: asking EGL for its devices' CUDA ordinals starts CUDA in this process,
        # which then keeps the devices it saw (egl.pin, which clears the variable for robosuite, would
        # leave Isaac Sim with all of them, rendering on the first)
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"  # nvidia-smi's numbering
        os.environ["CUDA_VISIBLE_DEVICES"] = gpu
        os.environ["MUJOCO_EGL_DEVICE_ID"] = str(egl.egl_index(int(gpu)))  # MuJoCo's EGL device


def wrap_deg(a: float) -> float:
    return (float(a) + 180.0) % 360.0 - 180.0


def rounded(v, n: int = 4) -> list[float]:
    return [round(float(x), n) for x in v]


class Server:
    def __init__(self):
        self.sim = None
        self.ex = None
        self.warm = False
        #: world <- start frame, and its inverse
        self.start = np.eye(4)
        self.start_inv = np.eye(4)
        self.start_heading = 0.0
        self.rows: list[list[float]] = []
        #: per hand, its parts that touched something (not the robot) at any step of the current command
        self.touched: dict[str, set] = {s: set() for s in P.SIDES}
        self.fingers = {s: "open" for s in P.SIDES}
        self.hand_bodies: dict[int, tuple[str, str]] = {}

    # -- requests --------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        from ..armserver import Refused

        if op == "ping":
            return {"pid": os.getpid()}
        if op == "open":
            return self.open(**args)
        if op == "close":  # a warm server keeps the task's environment for its next episode
            if self.sim is not None and not self.warm:
                self.sim.close()
                self.sim = None
            self.ex = None
            return {}
        if self.sim is None or self.ex is None:
            raise Refused("no episode: send open first")
        method = getattr(self, "op_" + op, None)
        if method is None:
            raise Refused(f"unknown operation {op!r}")
        self.rows = []
        self.touched = {s: set() for s in P.SIDES}
        out = method(**args)
        out["proprio"] = self.proprio()
        out["proprio"]["touched"] = {s: [k for k in HAND_PARTS if k in self.touched[s]] for s in P.SIDES}
        if self.rows:
            out["log"] = np.asarray(self.rows, dtype=float)
        return out

    def status(self) -> dict:
        return {} if self.sim is None or self.ex is None else self.sim.status()

    def open(self, task: str, seed: int, renderer: str | None = None, size=None, official_test: bool = False) -> dict:
        """``size``: what one-arm sessions send (their pictures' size); the gaze camera's is fixed.
        ``official_test``: a test seed (an official evaluation episode) may be opened."""
        from ..armserver import Refused
        from .control import AmoExecutor, WbcExecutor
        from .sim import SimpleSim

        t0 = time.time()
        try:
            P.check_task(str(task))
            P.check_seed(int(seed), official_test=bool(official_test))
        except ValueError as exc:
            raise Refused(str(exc)) from None
        renderer = renderer or os.environ.get("SIMPLE_RENDERER") or "isaac"
        if renderer not in P.RENDERERS:
            raise Refused(f"renderer {renderer!r}: one of {', '.join(P.RENDERERS)}")
        self.ex = None
        if self.sim is not None and (self.sim.task_name != task or self.sim.picture_renderer != renderer):
            self.sim.close()
            self.sim = None
        if self.sim is None:
            self.sim = SimpleSim(str(task), renderer=renderer)
            self._log_steps()
        sim = self.sim
        try:
            facts = dict(sim.reset(int(seed), official_test=bool(official_test)))
        except ValueError as exc:  # an official episode without the evaluation data
            raise Refused(str(exc)) from None
        self._hand_bodies()
        self.ex = WbcExecutor(sim) if sim.spec.controller == "decoupled_wbc" else AmoExecutor(sim)
        self.start = sim.frame()
        self.start_inv = np.linalg.inv(self.start)
        self.start_heading = math.degrees(math.atan2(self.start[1, 0], self.start[0, 0]))
        self.fingers = {s: "open" for s in P.SIDES}
        facts.update({"split": P.split_of(int(seed)), "renderer": renderer,
                      "start_frame_in_world": self.start.tolist()})
        return {"facts": facts, "start": self.proprio(), "seconds": round(time.time() - t0, 2)}

    # -- frames ----------------------------------------------------------------------------------
    def _point(self, p) -> list[float]:
        """World point -> start frame."""
        return rounded(self.start_inv[:3, :3] @ np.asarray(p, float) + self.start_inv[:3, 3])

    def _dir(self, v) -> list[float]:
        return rounded(self.start_inv[:3, :3] @ np.asarray(v, float))

    def _world_point(self, p) -> np.ndarray:
        return self.start[:3, :3] @ np.asarray(p, float) + self.start[:3, 3]

    def _world_dir(self, v) -> np.ndarray | None:
        return None if v is None else self.start[:3, :3] @ np.asarray(v, float)

    def _heading(self) -> float:
        from .sim import heading

        return wrap_deg(math.degrees(heading(self.sim.data.xmat[self.sim.pelvis_id].reshape(3, 3)))
                        - self.start_heading)

    # -- readings --------------------------------------------------------------------------------
    def _log_steps(self) -> None:
        """Every control step the commands take adds a row to the reply's log, and what each hand touched
        (``touched``: a hand can knock something over on its way and touch nothing at the end)."""
        sim, step = self.sim, self.sim.step

        def logged(cmd) -> bool:
            running = step(cmd)
            self.rows.append(self._row())
            for side, parts in self.touching().items():
                self.touched[side].update(parts)
            return running

        sim.step = logged

    def _row(self) -> list[float]:
        sim = self.sim
        p = self._point(sim.data.xpos[sim.pelvis_id])
        t = sim.torso()
        palms = [v for s in P.SIDES for v in self._point(sim.palm(s)["point"])]
        return [float(sim.steps), *p, round(self._heading(), 2), round(sim.pelvis_height(), 4),
                round(math.degrees(t["pitch"]), 2), round(math.degrees(t["yaw"]), 2), *palms]

    def _hand_bodies(self) -> None:
        m = self.sim.model
        self.hand_bodies = {}
        for b in range(m.nbody):
            name = m.body(b).name
            for s in P.SIDES:
                if name == f"{s}_wrist_yaw_link":
                    self.hand_bodies[b] = (s, "palm")
                elif name.startswith(f"{s}_hand_"):
                    part = next((k for k in HAND_PARTS if k in name), "palm")
                    self.hand_bodies[b] = (s, part)

    def touching(self) -> dict[str, list[str]]:
        """Per hand, its parts in contact with something that is not the robot (the Dex3's touch sensors)."""
        sim = self.sim
        m, d = sim.model, sim.data
        robot = set(int(g) for g in sim.robot_geoms)
        out: dict[str, set] = {s: set() for s in P.SIDES}
        n = int(d.ncon)
        for g1, g2 in zip(np.asarray(d.contact.geom1)[:n], np.asarray(d.contact.geom2)[:n]):
            for mine, other in ((int(g1), int(g2)), (int(g2), int(g1))):
                hit = self.hand_bodies.get(int(m.geom_bodyid[mine]))
                if hit is not None and other not in robot:
                    out[hit[0]].add(hit[1])
        return {s: [k for k in HAND_PARTS if k in out[s]] for s in P.SIDES}

    def proprio(self) -> dict:
        sim = self.sim
        t = sim.torso()
        palms = {}
        for s in P.SIDES:
            palm = sim.palm(s)
            palms[s] = {"point": self._point(palm["point"]), "normal": self._dir(palm["normal"]),
                        "finger": self._dir(palm["finger"])}
        return {"step": int(sim.steps), "pelvis": self._point(sim.data.xpos[sim.pelvis_id]),
                "heading": round(self._heading(), 2), "pelvis_height": round(sim.pelvis_height(), 4),
                "torso_pitch": round(math.degrees(t["pitch"]), 2), "torso_yaw": round(math.degrees(t["yaw"]), 2),
                "palms": palms, "fingers": dict(self.fingers), "touching": self.touching(),
                "holds": {s: self.ex.hold[s]["mode"] for s in P.SIDES} if self.ex is not None else {},
                "gaze": rounded(sim.gaze_deg, 1)}

    # -- pictures --------------------------------------------------------------------------------
    def op_observe(self, cams=None, robot_mask=True) -> dict:
        from ..armserver import Refused

        for cam in cams or P.CAMERAS:
            if cam not in P.CAMERAS:
                raise Refused(f"camera {cam!r}: the robot has {', '.join(P.CAMERAS)}")
        obs = self.sim.observe(robot_mask=bool(robot_mask))
        cameras = {}
        for cam, c in obs["cameras"].items():
            cameras[cam] = {**c, "pose": (self.start_inv @ np.asarray(c["pose"], float)).tolist()}
        return {"obs": {"step": obs["step"], "images": obs["images"], "depth": obs["depth"], "robot": obs["robot"],
                        "cameras": cameras}}

    def op_state(self) -> dict:
        return {}

    # -- commands --------------------------------------------------------------------------------
    def _over(self, res: dict) -> dict:
        return {"result": {**res, "episode_over": bool(res.get("episode_over") or self.sim.over)}}

    def op_body(self, height=None, pitch=None, yaw=None) -> dict:
        return self._over(self.ex.body(height=height, pitch_deg=pitch, yaw_deg=yaw))

    def op_walk(self, x, y, heading=None) -> dict:
        target = self._world_point([float(x), float(y), 0.0])
        h = None if heading is None else float(heading) + self.start_heading
        res = self.ex.walk(float(target[0]), float(target[1]), heading_deg=h)
        if "reached_pose" in res:
            px, py, ph = res["reached_pose"]
            at = self._point([px, py, 0.0])
            res["reached_pose"] = [at[0], at[1], round(wrap_deg(ph - self.start_heading), 2)]
            ax, ay, ah = res["asked"]
            asked = self._point([ax, ay, 0.0])
            res["asked"] = [asked[0], asked[1], round(wrap_deg(ah - self.start_heading), 2)]
        return self._over(res)

    def _waypoints(self, waypoints) -> list[dict]:
        out = []
        for wp in waypoints:
            out.append({"point": self._world_point(wp["point"]), "normal": self._world_dir(wp.get("normal")),
                        "finger": self._world_dir(wp.get("finger")), "seconds": float(wp.get("seconds", 1.0))})
        return out

    def op_hand(self, side, waypoints, hold="torso", dry_run=False) -> dict:
        from ..armserver import Refused
        from .control import HOLD_MODES

        if side not in P.SIDES:
            raise Refused(f"side {side!r}: one of {', '.join(P.SIDES)}")
        if hold not in HOLD_MODES:
            raise Refused(f"hold {hold!r}: one of {', '.join(HOLD_MODES)}")
        if not 1 <= len(waypoints) <= P.HAND_WAYPOINTS_MAX:
            raise Refused(f"a hand command has 1 to {P.HAND_WAYPOINTS_MAX} waypoints")
        wps = self._waypoints(waypoints)
        if dry_run:
            last = wps[-1]
            q, res = self.ex.ik.solve(side, last["point"], last["normal"], last["finger"], q0=self.ex.arm_q(side),
                                      iters=60)
            p, n, f = self.ex.ik.palm(side)
            out = {"dry_run": True, "side": side, "reached_point": self._point(p),
                   "position_error_m": round(res["position_error_m"], 4)}
            for k in ("normal_error_deg", "finger_error_deg"):
                if k in res:
                    out[k] = round(res[k], 2)
            if "at_limit" in res:
                out["at_limit"] = res["at_limit"]
            return {"result": out}
        res = self.ex.hand(side, wps, hold=hold)
        if "reached_point" in res:
            res["reached_point"] = self._point(res["reached_point"])
        return self._over(res)

    def op_fingers(self, side, closed) -> dict:
        from ..armserver import Refused

        if side not in P.SIDES:
            raise Refused(f"side {side!r}: one of {', '.join(P.SIDES)}")
        wide = closed == "wide"
        closed = False if wide else bool(closed)
        res = self.ex.fingers(side, closed, wide=wide)
        if "seconds" in res:
            self.fingers[side] = "closed" if closed else "wide" if wide else "open"
            res["touching"] = self.touching()[side]
        return self._over(res)

    def aim(self, at) -> tuple[float, float]:
        """The gaze angles (deg) that point the camera at a start-frame point, from where the torso is now."""
        from .sim import HEAD_CAMERA_POSITION

        sim = self.sim
        R = sim.data.xmat[sim.torso_id].reshape(3, 3)
        c = sim.data.xpos[sim.torso_id] + R @ HEAD_CAMERA_POSITION
        p = R.T @ (self._world_point(at) - c)
        return math.degrees(math.atan2(p[1], p[0])), math.degrees(math.atan2(-p[2], math.hypot(p[0], p[1])))

    def op_gaze(self, yaw=None, pitch=None, at=None) -> dict:
        y0, p0 = (float(v) for v in self.sim.gaze_deg)
        if at is not None:
            yaw, pitch = self.aim(at)
        asked = [round(float(y0 if yaw is None else yaw), 1), round(float(p0 if pitch is None else pitch), 1)]
        res = self.ex.gaze(asked[0], asked[1])
        if "gaze_deg" in res:
            res["asked_deg"] = asked
        return self._over(res)

    def op_wait(self, n) -> dict:
        t0 = self.sim.steps
        self.ex.hold_steps(int(n))
        return self._over({"steps": int(self.sim.steps - t0)})


def main(argv: list[str]) -> None:
    prepare()
    from ..armserver import serve

    sock = socket.socket(fileno=int(argv[0]))
    try:
        serve(sock, Server())
    finally:
        sock.close()
    os._exit(0)  # Isaac Sim's shutdown can hang; the episode side has every reply


if __name__ == "__main__":
    main(sys.argv[1:])
