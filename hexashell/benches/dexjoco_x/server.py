"""The DexJoCo-X simulator server: ``python -m hexashell.benches.dexjoco_x.server <fd>``.

Started by the episode side (:class:`hexashell.benches.dexjoco_x.session.DexterousSession`) with one end of a socket
pair as file descriptor ``fd`` (or kept warm by :mod:`hexashell.benches.daemon`), in DexJoCo-X's Python
environment (bench-dexjoco-x, ``$DEXJOCO_X_ROOT`` the release).  It runs the official episode (:mod:`.sim`) and
the hand commands (:mod:`.control`):

- ``open(task, seed, size, official_test)``: the episode of that scene seed (a test seed only with
  ``official_test``, which the episode side sends only when the run allows the test:
  protocol.OFFICIAL_TEST_VARIABLE); then the robot holds still for START_HOLD_STEPS steps while the objects
  settle (they are placed a little above their rest and drop, tip or roll in the first steps); replies with the
  episode's facts and the robot's readings;
- ``hand(side, point, xyz, normal, finger, along, speed, approach, dry_run, look_at)`` (look_at: the wrist camera,
  point 'camera', looks at that point: control.Executor.view), ``arc(side, center, axis, angle, speed)``,
  ``fingers(side, shape, amount, curl, thumb, oppose, spread, seconds)``, ``home(side)``, ``wait(n)``: the
  commands; each replies with its result, the robot's readings and one log row per step it took;
- ``observe(cams, robot_mask)``: the cameras' RGB-D pictures;
- ``state``, ``ping``, ``close``.

Coordinates are the scene's (MuJoCo's world frame: metres, z up, the robot faces +x); angles of requests in
degrees.  Every reply carries the episode status.

The readings are the robot's own (proprioception): where the arms put the hands (forward kinematics), and per
finger the push from outside the hand on its joints, from what the joints do and the hand's own model
(``DexSim.outside_push``: the hand's self-model, a copy of the model in which only the hand's own parts collide,
gives the joints' accelerations with nothing outside the hand; the fingers of a closed hand meet each other and
the palm, which pushes nothing).  The hands have no touch sensors, so what the model hears of contact comes from
the fingers: a finger pushed on (FINGER_PUSH_RAD) presses on something, a hand whose thumb and one more finger
are pushed on holds something (``holds``), and a finger pushed on at any step of a command touched something on
the hand's way (``pressed``).  The contacts of the physics (what really touches what) go only to the per-step
log, for the records.
"""

from __future__ import annotations

import math
import os
import socket
import sys
import time

import numpy as np

from . import protocol as P

#: rows of the per-step log: step, right palm xyz, left palm xyz (nan for an arm the task has not), success, and
#: for the records only (the model never sees them) whether the physics has each hand holding something (its thumb
#: and one more part pushing on the same free object; right, left; nan for an arm the task has not)
LOG_COLUMNS = 10
#: steps the robot holds still after the episode opens, while the objects settle (measured: the hammer
#: tips over within 10 steps, the hot dog drops 1.4 cm, the camera 1 cm)
START_HOLD_STEPS = 10
#: a motion's slowest speed (m/s)
SPEED_MIN = 0.01
#: longest approach (m)
APPROACH_MAX_M = 0.25
#: a wrist camera sent to look at a point stays at least this far from it (m)
LOOK_MIN_M = 0.10
#: a finger setting's shortest and longest time (s)
FINGER_SECONDS = (0.1, 3.0)


def prepare() -> None:
    """Before MuJoCo is imported: render on ``$HEXA_EGL_DEVICE`` (EGL), no display, no proxy (the simulator
    needs no network)."""
    for name in ("DISPLAY", "XAUTHORITY", "http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY", "all_proxy",
                 "ALL_PROXY"):
        os.environ.pop(name, None)
    os.environ["MUJOCO_GL"] = "egl"
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    from ..armserver import prepare as pin_gpu

    pin_gpu()


def holds(pressing) -> bool:
    """A hand holds something when its thumb and one more finger press on something (``pressing``: the hand's
    fingers that do, ``Server.fingers_pressing``)."""
    parts = set(pressing or ())
    return "thumb" in parts and len(parts) >= 2


def rounded(v, n: int = 4) -> list[float]:
    return [round(float(x), n) for x in v]


def unit(v, name: str) -> np.ndarray:
    from ..armserver import Refused

    try:
        a = np.asarray(v, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a direction: three numbers") from None
    n = float(np.linalg.norm(a)) if a.size == 3 and np.all(np.isfinite(a)) else 0.0
    if n < 1e-6:
        raise Refused(f"{name} is a direction: three numbers, not all zero")
    return a / n


def point3(v, name: str) -> np.ndarray:
    from ..armserver import Refused

    try:
        a = np.asarray(v, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a point: three numbers (m)") from None
    if a.size != 3 or not np.all(np.isfinite(a)):
        raise Refused(f"{name} is a point: three numbers (m)")
    return a


class Server:
    def __init__(self):
        self.sim = None
        self.ex = None
        self.warm = False
        self.rows: list[list[float]] = []
        #: per hand, its fingers something outside the hand pushed on at a step of the current command
        self.pressed: dict[str, set] = {}

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
        self.pressed = {s: set() for s in self.sim.sides}
        out = method(**args)
        out["proprio"] = self.proprio()
        out["proprio"]["pressed"] = {s: [f for f in P.FINGERS if f in self.pressed[s]] for s in self.sim.sides}
        if self.rows:
            out["log"] = np.asarray(self.rows, dtype=float)
        return out

    def status(self) -> dict:
        return {} if self.sim is None or self.ex is None else self.sim.status()

    def open(self, task: str, seed: int, size=None, official_test: bool = False) -> dict:
        from ..armserver import Refused
        from .control import Executor
        from .sim import DexSim

        t0 = time.time()
        try:
            P.check_task(str(task))
            P.check_seed(str(task), int(seed), official_test=bool(official_test))
        except ValueError as exc:
            raise Refused(str(exc)) from None
        size = int(size or P.IMAGE_SIZE)
        self.ex = None
        if self.sim is not None and (self.sim.task != task or self.sim.size != size):
            self.sim.close()
            self.sim = None
        if self.sim is None:
            self.sim = DexSim(str(task), size=size)
            self._log_steps()
        sim = self.sim
        facts = dict(sim.reset(int(seed)))
        self.ex = Executor(sim)
        self.rows = []
        self.pressed = {s: set() for s in sim.sides}
        self.ex.hold_steps(START_HOLD_STEPS)
        facts.update({"split": P.split_of(str(task), int(seed)), "start_hold_steps": START_HOLD_STEPS,
                      "hand_points": list(P.HAND_POINTS), "shapes": list(P.SHAPES)})
        return {"facts": facts, "start": self.proprio(), "seconds": round(time.time() - t0, 2)}

    # -- readings --------------------------------------------------------------------------------
    def fingers_pressing(self, side: str) -> list[str]:
        """The fingers of a hand pressing on something now: something outside the hand pushes on them more than
        FINGER_PUSH_RAD (control.pressing, DexSim.outside_push)."""
        from .control import pressing

        return pressing(self.sim.outside_push(side))

    def _log_steps(self) -> None:
        """Every control step a command takes adds a row to the reply's log, and the fingers something outside
        the hand pushed on at that step (``pressed``: a hand can knock something over on its way and touch
        nothing at the end)."""
        sim, step = self.sim, self.sim.step

        def logged(cmd) -> bool:
            running = step(cmd)
            self.rows.append(self._row())
            for side in sim.sides:
                self.pressed.setdefault(side, set()).update(self.fingers_pressing(side))
            return running

        sim.step = logged

    def _row(self) -> list[float]:
        sim = self.sim
        row = [float(sim.steps)]
        for side in ("right", "left"):
            row += list(sim.palm(side)["point"]) if side in sim.sides else [math.nan] * 3
        row.append(1.0 if sim.success else 0.0)
        truth = sim.holding()
        row += [float(truth[side]) if side in sim.sides else math.nan for side in ("right", "left")]
        return row

    def proprio(self) -> dict:
        from .control import GRASP_POINT, measured_curl

        sim, ex = self.sim, self.ex
        hands = {}
        for side in sim.sides:
            T = sim.tcp(side)
            palm = sim.palm(side)
            pressing = self.fingers_pressing(side)
            cam = np.asarray(sim.camera(f"{side}_wrist")["pose"], float)
            hands[side] = {
                "palm": rounded(palm["point"]), "normal": rounded(palm["normal"], 3),
                "finger": rounded(palm["finger"], 3), "grasp": rounded(T[:3, :3] @ GRASP_POINT + T[:3, 3]),
                "tips": {f: rounded(v) for f, v in sim.tips(side).items()},
                "camera": rounded(cam[:3, 3]), "view": rounded(-cam[:3, 2], 3), "picture_up": rounded(cam[:3, 1], 3),
                "shape": ex.shape[side], "curl": measured_curl(sim.hand_q(side)),
                "asked": measured_curl(ex.targets[side]), "pressing": pressing, "holding": holds(pressing),
            }
        return {"step": int(sim.steps), "hands": hands}

    # -- pictures --------------------------------------------------------------------------------
    def op_observe(self, cams=None, robot_mask=True) -> dict:
        from ..armserver import Refused

        cams = list(cams or self.sim.cams)
        for cam in cams:
            if cam not in self.sim.cams:
                raise Refused(f"camera {cam!r}: this task has {', '.join(self.sim.cams)}")
        return {"obs": self.sim.observe(cams=cams, robot_mask=bool(robot_mask))}

    def op_state(self) -> dict:
        return {}

    # -- commands --------------------------------------------------------------------------------
    def _side(self, side) -> str:
        from ..armserver import Refused

        if side not in self.sim.sides:
            raise Refused(f"side {side!r}: this task's hands are {', '.join(self.sim.sides)}")
        return str(side)

    def _over(self, res: dict) -> dict:
        return {"result": {**res, "episode_over": bool(self.sim.over)}}

    def _speed(self, speed) -> float | None:
        from ..armserver import Refused

        if speed is None:
            return None
        v = float(speed)
        if not math.isfinite(v) or v < SPEED_MIN:
            raise Refused(f"speed is in m/s, at least {SPEED_MIN:g}")
        return min(v, self.ex.speed_max)

    def _orientation(self, side: str, normal, finger, along) -> np.ndarray | None:
        """The goal orientation, or None to keep the hand's."""
        from ..armserver import Refused
        from .control import frame_of, grip_frame

        R_now = self.ex.ref[side][:3, :3]
        n = None if normal is None else unit(normal, "normal")
        f = None if finger is None else unit(finger, "finger")
        if along is not None:
            if f is not None:
                raise Refused("hand: along (the handle's way, for a power grip) or finger, not both")
            return grip_frame(side, unit(along, "along"), R_now[:, 2] if n is None else n)
        if n is None and f is None:
            return None
        if n is not None and f is not None and abs(float(n @ f)) > math.cos(math.radians(60.0)):
            raise Refused("hand: the palm's normal and the fingers' direction must be at least 60 degrees apart "
                          "(they are at right angles on the hand)")
        return frame_of(n, f, R_now)

    def op_hand(self, side, xyz, point="palm", normal=None, finger=None, along=None, speed=None, approach=None,
                dry_run=False, look_at=None) -> dict:
        from ..armserver import Refused
        from .control import turn_deg

        side = self._side(side)
        if point not in P.HAND_POINTS:
            raise Refused(f"point {point!r}: one of {', '.join(P.HAND_POINTS)}")
        target = point3(xyz, "xyz")
        if look_at is not None:
            if point != "camera" or normal is not None or finger is not None or along is not None:
                raise Refused("look_at turns the hand so that its wrist camera looks at that point: with "
                              "point='camera' and no other orientation")
            seen = point3(look_at, "look_at")
            if float(np.linalg.norm(seen - target)) < LOOK_MIN_M:
                raise Refused(f"look_at: at least {LOOK_MIN_M:g} m from where the camera goes")
            R = self.ex.view(side, target, seen)[0]
        else:
            R = self._orientation(side, normal, finger, along)
        speed = self._speed(speed)
        if approach is not None:
            approach = float(approach)
            if not 0.0 <= approach <= APPROACH_MAX_M:
                raise Refused(f"approach is a distance in metres, 0 to {APPROACH_MAX_M:g}")
        ex = self.ex
        R_goal = ex.ref[side][:3, :3] if R is None else R
        off = ex.offset(side, point)
        goal = np.eye(4)
        goal[:3, :3] = R_goal
        goal[:3, 3] = target - R_goal @ off
        if dry_run:
            via = None
            if approach:
                via = goal.copy()
                via[:3, 3] = goal[:3, 3] - approach * R_goal[:, 2]
            reach = ex.reach(side, goal, off, via=via)
            return {"result": {"dry_run": True, "side": side, "point": point, "target": rounded(target),
                               "error_m": round(reach["error_m"], 4), "error_deg": round(reach["error_deg"], 2),
                               "at_limit": reach["at_limit"], "lost_at": reach["lost_at"],
                               "lost_limits": reach["lost_limits"], "lost_m": round(reach["lost_m"], 4),
                               "lost_deg": round(reach["lost_deg"], 2)}}
        res = ex.place(side, point, target, R=R_goal, speed=speed, approach=approach or None)
        s = res["sides"][side]
        T = self.sim.tcp(side)
        at = T[:3, :3] @ off + T[:3, 3] if point in ("palm", "grasp", "camera") else ex.point_now(side, point)
        out = {"side": side, "point": point, "target": rounded(target), "at": rounded(at),
               "error_m": round(float(np.linalg.norm(at - target)), 4),
               "error_deg": round(turn_deg(T[:3, :3], R_goal), 2), "reached": bool(s["reached"]),
               "blocked": s["blocked"], "at_limit": list(s.get("at_limit") or []),
               "beyond_reach_m": s.get("beyond_reach_m"), "steps": int(res["steps"])}
        if res.get("stage"):
            out["stage"] = res["stage"]
        return self._over(out)

    def op_arc(self, side, center, axis, angle, speed=None) -> dict:
        from ..armserver import Refused
        from .control import rotvec

        side = self._side(side)
        c = point3(center, "center")
        a = unit(axis, "axis")
        ang = float(angle)
        if not math.isfinite(ang) or abs(ang) > 180.0 or ang == 0.0:
            raise Refused("arc: angle in degrees, -180 to 180, not 0")
        turn_speed = None if speed is None else float(speed)
        if turn_speed is not None and not (1.0 <= turn_speed <= 90.0):
            raise Refused("arc: speed in degrees per second, 1 to 90")
        T0 = self.sim.tcp(side)
        res = self.ex.arc(side, c, a, math.radians(ang), turn_speed=turn_speed)
        s = res["sides"][side]
        T1 = self.sim.tcp(side)
        turned = math.degrees(float(rotvec(T1[:3, :3] @ T0[:3, :3].T) @ a))
        return self._over({"side": side, "turned_deg": round(turned, 1), "asked_deg": round(ang, 1),
                           "reached": bool(s["reached"]), "blocked": s["blocked"],
                           "at_limit": list(s.get("at_limit") or []), "beyond_reach_m": s.get("beyond_reach_m"),
                           "steps": int(res["steps"]),
                           "palm": rounded(self.sim.palm(side)["point"])})

    def op_fingers(self, side, shape=None, amount=1.0, curl=None, thumb=None, oppose=None, spread=None,
                   seconds=0.5) -> dict:
        from ..armserver import Refused
        from .control import finger_settings, shape_targets

        side = self._side(side)
        ex = self.ex
        if shape is not None and shape not in P.SHAPES:
            raise Refused(f"shape {shape!r}: one of {', '.join(P.SHAPES)}")
        amount = float(amount)
        if not 0.0 <= amount <= 1.0:
            raise Refused("amount: 0 (open) to 1 (the whole shape)")
        sec = float(seconds)
        if not FINGER_SECONDS[0] <= sec <= FINGER_SECONDS[1]:
            raise Refused(f"seconds: {FINGER_SECONDS[0]:g} to {FINGER_SECONDS[1]:g}")
        curl = dict(curl or {})
        for f, c in curl.items():
            if f not in P.FINGERS[1:]:
                raise Refused(f"curl: fingers are {', '.join(P.FINGERS[1:])} (the thumb has thumb=)")
            if not -0.1 <= float(c) <= 1.2:
                raise Refused("a finger's curl is 0 (straight) to 1 (curled in), up to 1.2")
        bounds = ((thumb, "thumb", 0.0, 1.2), (oppose, "oppose", 0.0, 1.0), (spread, "spread", -1.0, 1.0))
        for v, name, lo, hi in bounds:
            if v is not None and not lo <= float(v) <= hi:
                raise Refused(f"{name}: {lo:g} to {hi:g}")
        base = shape_targets(self.sim, side, shape, amount) if shape is not None else ex.targets[side]
        goal = finger_settings(self.sim, side, base, curl=curl, thumb=thumb, oppose=oppose, spread=spread)
        res = ex.fingers(side, goal, sec)
        if shape is not None and not curl and thumb is None and oppose is None and spread is None:
            ex.shape[side] = shape if amount >= 0.999 else f"{shape} {amount:.2f}"
        else:
            ex.shape[side] = "custom"
        return self._over({"side": side, "shape": ex.shape[side], "curl": res["curl"], "asked": res["asked"],
                           "pressing": res["pressing"], "holding": holds(res["pressing"]), "steps": int(res["steps"])})

    def op_home(self, side=None) -> dict:
        sides = list(self.sim.sides) if side is None else [self._side(side)]
        res = self.ex.home(sides)
        return self._over({"sides": {s: {"reached": bool(v["reached"]), "blocked": v["blocked"],
                                         "at_limit": list(v.get("at_limit") or [])}
                                     for s, v in res["sides"].items()}, "steps": int(res["steps"])})

    def op_wait(self, n) -> dict:
        from ..armserver import Refused

        n = int(n)
        if not 1 <= n <= P.WAIT_MAX_STEPS:
            raise Refused(f"wait: 1 to {P.WAIT_MAX_STEPS} steps ({P.STEP_S:g} s each)")
        t0 = self.sim.steps
        self.ex.hold_steps(n)
        return self._over({"steps": int(self.sim.steps - t0)})


def main(argv: list[str]) -> None:
    prepare()
    from ..armserver import serve

    sock = socket.socket(fileno=int(argv[0]))
    try:
        serve(sock, Server())
    finally:
        sock.close()
    os._exit(0)


if __name__ == "__main__":
    main(sys.argv[1:])
