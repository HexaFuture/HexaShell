"""A fake RoboCasa kitchen for the bench tests: a counter with a red box (and, if asked, an open square
dish), three RGB-D cameras, an arm that gets where it is sent at once (counting the steps it would
take), a colour-based SAM3.

:class:`FakeKitchen` answers the operations of :mod:`hexashell.benches.robocasa.server` with replies
of the same shape, and :class:`FakeLink` passes requests and replies through the wire encoding, so
the episode side (session, host, perception, the cells' robot) runs as it does against the real
server.  Start frame as in RoboCasa: origin on the floor under the base, x forward, y left, z up.
"""

from __future__ import annotations

import json
import math
import struct

import numpy as np

from hexashell.benches import wire
from hexashell.benches.robocasa import protocol as P
from hexashell.benches.arm import rotvec
from hexashell.benches.robocasa.host import CAMS, check_cam
from hexashell.benches.pose import rotation_of
from hexashell.benches.robocasa.session import CasaSession
from hexashell.benches.session import SimError, SimRefused
from hexashell.vision.sam3_client import Segment

FLOOR, COUNTER, BOX, HAND, DISH = 1, 2, 3, 4, 5
COLOURS = {0: (60, 70, 90), FLOOR: (45, 40, 40), COUNTER: (150, 140, 120), BOX: (200, 30, 30), HAND: (230, 230, 235),
           DISH: (90, 150, 200)}
COUNTER_BOX = (np.array([0.45, -1.5, 0.0]), np.array([1.6, 1.5, 0.90]))
COUNTER_TOP = 0.90
BOX_SIZE = np.array([0.08, 0.05, 0.08])
BOX_START = np.array([0.72, -0.05, COUNTER_TOP + 0.04])
#: the dish: square, walls DISH_WALL thick and DISH_H tall around DISH_HALF of its middle, on a thin floor; its
#: near wall (toward -x) is held by a close with the grasp point within 2 cm of it, 0.3-3.5 cm under its top
DISH_START = np.array([0.75, 0.25, COUNTER_TOP])
DISH_HALF = 0.12
DISH_H = 0.07
DISH_WALL = 0.006
DISH_PINCH = 0.02
#: the shoulder in the base frame, and how far the grasp point gets from it
SHOULDER = np.array([0.20, 0.0, 1.05])
REACH = 0.85
STEP_M = 0.0125  # grasp point travel per step
STEP_DEG = 3.0
BASE_STEP_M = 0.02
BASE_STEP_DEG = 2.5
BASE_FRONT = 0.30  # the base's front edge, ahead of its origin
HAND_R = 0.035
START_TCP_BASE = (np.array([0.50, 0.0, 1.10]), (45.0, 0.0, 0.0))
BASE_CAMS = {"left": ((0.0, 0.22, 1.40), (0.8, 0.0, 0.9)), "right": ((0.0, -0.22, 1.40), (0.8, 0.0, 0.9))}
FOVY = 70.0


class Refusal(Exception):
    pass


def planar(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    pose = np.eye(4)
    pose[:2, :2] = [[c, -s], [s, c]]
    pose[:2, 3] = (x, y)
    return pose


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Camera-to-world pose of a camera at ``eye`` looking at ``target`` (MuJoCo axes: looks along -z)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye
    f /= np.linalg.norm(f)
    right = np.cross(f, up)
    right /= np.linalg.norm(right)
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2], pose[:3, 3] = right, np.cross(right, f), -f, eye
    return pose


def render(pose: np.ndarray, focal: float, size, boxes, spheres=()):
    """RGB, depth along the optical axis and the id of what each pixel shows; ``size``: n (a square
    picture) or (width, height)."""
    w, h = (int(size), int(size)) if np.isscalar(size) else (int(size[0]), int(size[1]))
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    ys, xs = np.mgrid[0:h, 0:w].astype(float)
    local = np.stack([(xs - cx) / focal, -(ys - cy) / focal, -np.ones_like(xs)], -1).reshape(-1, 3)
    d = local @ pose[:3, :3].T  # local z is -1: the ray parameter is the depth along the axis
    o = pose[:3, 3]
    best = np.full(len(d), np.inf)
    ids = np.zeros(len(d), int)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -o[2] / d[:, 2]
        hit = np.isfinite(t) & (t > 1e-6) & (t < best)
        best[hit], ids[hit] = t[hit], FLOOR
        for lo, hi, k in boxes:
            t1, t2 = (lo - o) / d, (hi - o) / d
            near = np.nanmax(np.minimum(t1, t2), axis=1)
            far = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (far >= near) & (near > 1e-6) & (near < best)
            best[hit], ids[hit] = near[hit], k
    for centre, r, k in spheres:
        oc = o - np.asarray(centre, float)
        a = np.einsum("ij,ij->i", d, d)
        b = 2.0 * d @ oc
        disc = b * b - 4.0 * a * (oc @ oc - r * r)
        ok = disc >= 0
        t = np.where(ok, (-b - np.sqrt(np.where(ok, disc, 0.0))) / (2.0 * a), np.inf)
        hit = ok & (t > 1e-6) & (t < best)
        best[hit], ids[hit] = t[hit], k
    ids = ids.reshape(h, w)
    depth = np.where(np.isfinite(best), best, 0.0).reshape(h, w).astype(np.float32)
    rgb = np.zeros((h, w, 3), np.uint8)
    for k, colour in COLOURS.items():
        rgb[ids == k] = colour
    return rgb, depth, ids


class FakeKitchen:
    """The simulator side: state, operations, status."""

    def __init__(self, horizon: int = 600, size: int = 256, dish: bool = False):
        self.horizon = horizon
        self.size = size
        self.with_dish = dish
        self.closed = False
        self.ops: list[str] = []

    # -- state -----------------------------------------------------------------------------------
    def reset(self) -> None:
        self.steps = 0
        self.success = False
        self.base = np.zeros(3)
        p, angles = START_TCP_BASE
        self.tcp_base = np.eye(4)
        self.tcp_base[:3, :3] = rotation_of(*angles)
        self.tcp_base[:3, 3] = p
        self.start_tcp_base = self.tcp_base.copy()
        self.level = -1.0
        self.width = 0.08
        self.box = BOX_START.copy()
        self.held: np.ndarray | None = None  # box centre minus grasp point while held
        self.dish = DISH_START.copy() if self.with_dish else None  # the middle of its floor's underside
        self.dish_held: np.ndarray | None = None  # ... minus the grasp point while held
        #: the next closes on the dish's wall this many times come out empty
        self.dish_empty = 0
        #: (dx, dy): the next motion pushes the base by this, after the arm got where it was asked
        #: relative to the base (as when the hand presses on a fixture)
        self.shove: np.ndarray | None = None
        #: the joints the next motion ends with at the end of their travel (the server's at_limit)
        self.at_limit: list | None = None
        #: the next motion stops with the elbow this many degrees short of straight (the server's stretched)
        self.stretch: float | None = None
        #: the next motion strains with the elbow this many degrees short of straight: it stops and the arm
        #: gives way (the server's jammed)
        self.jam: float | None = None
        #: the elbow is this many degrees short of straight now: gotos and arcs are refused until home (the
        #: server's START_ROOM)
        self.stretched_now: float | None = None
        #: gotos are refused until home as poses the arm gets to only from home (the server's from_home)
        self.home_only = False
        #: the next grip or wait reports that the arm gave way this many times
        self.give = 0
        #: ... each time as the elbow crept nearly straight, not as the arm strained (the server's stretched),
        #: going back by the joints (the server's gave_way_back) unless creep_back is off
        self.creep = False
        self.creep_back = True
        #: the next drive stops as the arm strains (the server's jammed)
        self.drive_jam = False
        #: ... as the elbow straightens (jammed and stretched)
        self.drive_stretch = False
        #: the next push_arc turns this share of its angle and stops so (the server's ``stop``)
        self.push_stop: tuple[float, str] | None = None

    @property
    def over(self) -> bool:
        return self.success or self.steps >= self.horizon

    def status(self) -> dict:
        return {"step": self.steps, "horizon": self.horizon, "success": self.success, "over": self.over}

    def base_pose(self) -> np.ndarray:
        return planar(*self.base)

    def tcp(self) -> np.ndarray:
        return self.base_pose() @ self.tcp_base

    def shoulder(self) -> np.ndarray:
        return (self.base_pose() @ np.r_[SHOULDER, 1.0])[:3]

    def proprio(self) -> dict:
        return {"tcp": self.tcp(), "base": self.base.copy(), "width": float(self.width), "joints": np.zeros(7),
                "torso": 0.0}

    def row(self) -> list[float]:
        tcp = self.tcp()
        return [self.steps, *tcp[:3, 3], *rotvec(tcp[:3, :3]), self.width, *self.base]

    def _tick(self, rows: list) -> bool:
        """One step: carry the held box, judge; False once the episode is over."""
        self.steps += 1
        if self.held is not None:
            self.box = self.tcp()[:3, 3] + self.held
            if self.box[2] - BOX_SIZE[2] / 2 >= COUNTER_TOP + 0.08:
                self.success = True
        if self.dish_held is not None:
            self.dish = self.tcp()[:3, 3] + self.dish_held
        rows.append(self.row())
        return not self.over

    # -- operations ------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        self.ops.append(op)
        if op == "open":
            self.reset()
            return {"facts": {"task": args["task"], "seed": args["seed"], "task_set": P.task_set(args["task"]),
                              "seed_set": P.seed_set(args["seed"]), "layout_id": 7, "style_id": 9, "kitchen": [7, 9],
                              "instruction": "pick up the red box from the counter", "horizon": self.horizon,
                              "step_s": P.STEP_S},
                    "start": self.proprio(), "seconds": 0.01, "base_controller": "fake", "level": self.level}
        method = getattr(self, "op_" + op, None)
        if method is None:
            raise Refusal(f"unknown operation {op!r}")
        return method(**args)

    def _reply(self, result: dict, rows: list) -> dict:
        log = np.asarray(rows, float) if rows else np.zeros((0, 11))
        return {"result": result, "proprio": self.proprio(), "log": log}

    def op_observe(self, cams=None, robot_mask=True) -> dict:
        out = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "cameras": {}}
        focal = 0.5 * self.size / math.tan(math.radians(FOVY) / 2)
        boxes = [(*COUNTER_BOX, COUNTER), (self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2, BOX)] + self._dish_boxes()
        tcp = self.tcp()
        for cam in cams or CAMS:
            if cam == "wrist":
                pose = np.eye(4)
                pose[:3, 0], pose[:3, 1], pose[:3, 2] = tcp[:3, 1], -tcp[:3, 2], -tcp[:3, 0]
                pose[:3, 3] = tcp[:3, 3] - 0.10 * tcp[:3, 0]
                spheres = []
            else:
                eye, target = BASE_CAMS[cam]
                pose = self.base_pose() @ look_at(eye, target)
                spheres = [(tcp[:3, 3], HAND_R, HAND)]
            rgb, depth, ids = render(pose, focal, self.size, boxes, spheres)
            out["images"][cam], out["depth"][cam] = rgb, depth
            out["robot"][cam] = ids == HAND
            out["cameras"][cam] = {"pose": pose, "focal": focal, "width": self.size, "height": self.size}
        out["proprio"] = self.proprio()
        return {"obs": out}

    def _dish_boxes(self) -> list:
        if self.dish is None:
            return []
        x, y, z = self.dish
        a, t, h = DISH_HALF, DISH_WALL, DISH_H
        walls = [((x - a, y - a), (x - a + t, y + a)), ((x + a - t, y - a), (x + a, y + a)),
                 ((x - a, y - a), (x + a, y - a + t)), ((x - a, y + a - t), (x + a, y + a))]
        out = [(np.array([*lo, z]), np.array([*hi, z + h]), DISH) for lo, hi in walls]
        return out + [(np.array([x - a, y - a, z]), np.array([x + a, y + a, z + 0.008]), DISH)]

    def op_state(self) -> dict:
        return {"proprio": self.proprio(), "level": self.level, "grip_goal": self.level}

    def _check(self, pose: np.ndarray) -> dict:
        p = pose[:3, 3]
        dist = float(np.linalg.norm(p - self.shoulder()))
        if p[2] < 0.05:
            return {"ok": False, "why": "below the floor limit", "err_m": 0.0, "err_deg": 0.0}
        if dist > REACH:
            return {"ok": False, "why": f"{dist:.2f} m from the shoulder (the arm reaches {REACH} m)",
                    "err_m": dist - REACH, "err_deg": 0.0}
        return {"ok": True, "err_m": 0.0005, "err_deg": 0.1, "limit_margin": 0.6, "joints": np.zeros(7)}

    def _run(self, poses: list[np.ndarray]) -> tuple[list, bool]:
        """Follow world poses of the hand, one per step; stops at the counter top (blocked)."""
        rows, blocked = [], False
        for pose in poses:
            if self.over:
                break
            if self.held is None and pose[0, 3] > COUNTER_BOX[0][0] and pose[2, 3] < COUNTER_TOP + 0.01:
                blocked = True
                break
            self.tcp_base = np.linalg.inv(self.base_pose()) @ pose
            if not self._tick(rows):
                break
        return rows, blocked

    def _line(self, goal: np.ndarray, speed=None) -> list[np.ndarray]:
        start = self.tcp()
        dist = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
        turn = math.degrees(np.linalg.norm(rotvec(goal[:3, :3] @ start[:3, :3].T)))
        share = 1.0 if speed is None else speed / 0.25
        n = 3 + math.ceil(dist / (STEP_M * share)) + math.ceil(turn / (STEP_DEG * share))
        out = []
        for k in range(1, n + 1):
            f = k / n
            pose = goal.copy()
            pose[:3, 3] = start[:3, 3] + f * (goal[:3, 3] - start[:3, 3])
            pose[:3, :3] = _turn(start[:3, :3], goal[:3, :3], f)
            out.append(pose)
        return out

    def _motion(self, poses: list[np.ndarray], goal: np.ndarray, verdict: dict, extra: dict) -> dict:
        rows, blocked = self._run(poses)
        err = float(np.linalg.norm(self.tcp()[:3, 3] - goal[:3, 3]))
        if self.shove is not None:
            self.base[:2] += self.shove
            self.shove = None
        if self.stretch is not None:
            extra = {**extra, "stretched": True, "gave_way": 5, "gave_way_back": True, "gave_way_m": 0.07,
                     "bend_deg": self.stretch}
            self.stretch = None
        if self.jam is not None:
            extra = {**extra, "jammed": True, "gave_way": 5, "bend_deg": self.jam}
            blocked, self.jam = True, None
        return self._reply({"moved": True, "reach": verdict, "steps": len(rows), "blocked": blocked,
                            "reached": not blocked and err < 0.01 and not extra.get("stretched"), "err_m": err,
                            "err_deg": 0.1, "at_limit": self._limits(), **extra}, rows)

    def _stretched_now(self) -> dict | None:
        if self.stretched_now is None:
            return None
        return {"ok": False, "stretched_now": True, "elbow_now_deg": self.stretched_now,
                "why": f"the arm is stretched almost straight now (the elbow {self.stretched_now:.0f} deg short of "
                       "straight): from there it gets stuck. robot.home() first: it bends the elbow back"}

    def _limits(self) -> list:
        out, self.at_limit = self.at_limit or [], None
        return out

    def op_goto(self, pose, grip=None, dry_run=False, check=True, speed=None) -> dict:
        goal = np.asarray(pose, float)
        verdict = self._stretched_now() or self._check(goal)
        if verdict["ok"] and self.home_only:
            verdict = {"ok": False, "progress": 0.6, "err_m": 0.03, "err_deg": 25.0, "from_home": True,
                       "why": "the hand gets within 3.0 cm and 25 deg of it from the arm's pose now; from home "
                              "(robot.home()) it gets there"}
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict}, [])
        if grip is not None:
            self._set_grip(float(grip))
        return self._motion(self._line(goal, speed), goal, verdict, {})

    def op_arc(self, center, axis, angle, grip=None, dry_run=False, speed=None) -> dict:
        center, axis = np.asarray(center, float), np.asarray(axis, float)
        if abs(angle) > math.radians(180.0) or np.linalg.norm(axis) < 1e-6:
            raise Refusal("arc: at most 180 degrees at a time, about an axis that is not zero")
        axis = axis / np.linalg.norm(axis)
        start = self.tcp()
        n = 3 + math.ceil(abs(math.degrees(angle)) / (STEP_DEG * (1.0 if speed is None else speed / 0.25)))
        poses = []
        for k in range(1, n + 1):
            r = _axis_angle(axis, angle * k / n)
            pose = np.eye(4)
            pose[:3, :3] = r @ start[:3, :3]
            pose[:3, 3] = center + r @ (start[:3, 3] - center)
            poses.append(pose)
        verdict = self._stretched_now() or {"ok": True}
        for k in sorted({len(poses) // 2, len(poses) - 1}) if verdict["ok"] else []:
            verdict = self._check(poses[k])
            if not verdict["ok"]:
                verdict["why"] = ("the end of the arc: " if k == len(poses) - 1 else "half way: ") + verdict["why"]
                break
        end = poses[-1]
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict, "end": end}, [])
        if grip is not None:
            self._set_grip(float(grip))
        return self._motion(poses, end, verdict, {"end": end})

    def op_push_arc(self, center, axis, angle, push, press=0.03, tilt=0.0, dry_run=False) -> dict:
        center, axis = np.asarray(center, float), np.asarray(axis, float)
        if not 0.0 < abs(angle) <= math.radians(180.0) or np.linalg.norm(axis) < 1e-6:
            raise Refusal("push_arc: at most 180 degrees, about an axis that is not zero")
        axis = axis / np.linalg.norm(axis)
        verdict = self._stretched_now() or {"ok": True}
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict}, [])
        # the hand goes around the hinge as far as the part turns: all the way unless push_stop says
        share, stop = self.push_stop or (1.0, "angle")
        self.push_stop = None
        start = self.tcp()
        n = 3 + math.ceil(abs(math.degrees(angle * share)) / STEP_DEG)
        poses = []
        for k in range(1, n + 1):
            r = _axis_angle(axis, angle * share * k / n)
            pose = np.eye(4)
            pose[:3, :3] = r @ start[:3, :3]
            pose[:3, 3] = center + r @ (start[:3, 3] - center)
            poses.append(pose)
        rows, _ = self._run(poses)
        return self._reply({"moved": True, "reach": verdict, "steps": len(rows), "touched": stop != "no touch",
                            "touch_m": 0.08 if stop == "no touch" else 0.01, "cycles": n, "angle": angle * share,
                            "stop": stop, "press": press, "hinge_shift_m": 0.0,
                            "drift_m": -0.04 if stop == "slid" else 0.0, "blocked": False,
                            "reached": stop == "angle", "at_limit": self._limits()}, rows)

    def _set_grip(self, opening: float) -> None:
        self.level = float(np.clip(round((1.0 - 2.0 * opening) / 0.2) * 0.2, -1.0, 1.0))
        opening = (1.0 - self.level) / 2.0
        if opening <= 0.05:
            if self._grasps():
                self.width = float(BOX_SIZE[1])
                self.held = self.box - self.tcp()[:3, 3]
            elif self._pinches() and self.dish_empty > 0:
                self.dish_empty -= 1
                self.width = 0.003
            elif self._pinches():
                self.width = DISH_PINCH
                self.dish_held = self.dish - self.tcp()[:3, 3]
            else:
                self.width = 0.0
        elif self.dish_held is not None:
            self.width = 0.08 * opening
            self.dish_held = None
            self.dish[2] = COUNTER_TOP
        elif self.held is not None and 0.08 * opening < BOX_SIZE[1]:
            self.width = float(BOX_SIZE[1])  # set narrower than the box: the fingers stay on it
        else:
            self.width = 0.08 * opening
            if self.held is not None:
                self.held = None
                self.box[2] = COUNTER_TOP + BOX_SIZE[2] / 2 if self.box[0] > COUNTER_BOX[0][0] else BOX_SIZE[2] / 2

    def _grasps(self) -> bool:
        tcp = self.tcp()
        p, fingers = tcp[:3, 3], tcp[:3, 1]
        off = np.abs(p - self.box)
        inside = off[0] <= BOX_SIZE[0] / 2 + 0.005 and off[1] <= BOX_SIZE[1] / 2 + 0.01 \
            and -BOX_SIZE[2] / 2 + 0.005 <= p[2] - self.box[2] <= BOX_SIZE[2] / 2
        return bool(inside and abs(fingers[1]) > math.cos(math.radians(20.0)))

    def _pinches(self) -> bool:
        if self.dish is None or self.dish_held is not None:
            return False
        p, (x, y, z) = self.tcp()[:3, 3], self.dish
        return bool(abs(p[0] - (x - DISH_HALF + DISH_WALL / 2)) <= 0.02 and abs(p[1] - y) <= DISH_HALF
                    and z + DISH_H - 0.035 <= p[2] <= z + DISH_H - 0.003)

    def op_gripper(self, opening) -> dict:
        self._set_grip(float(opening))
        rows = []
        for _ in range(10):
            if not self._tick(rows):
                break
        return self._reply({"steps": len(rows), "width": self.width, "level": self.level, "goal": self.level,
                            **self._gave_way(), "over": self.over}, rows)

    def _gave_way(self) -> dict:
        give, self.give = self.give, 0
        crept, self.creep = give if self.creep else 0, False
        back = bool(crept) and self.creep_back
        return {"gave_way": give, "stretched": crept, "gave_way_back": back, "gave_way_m": 0.06 if back else 0.05 * give}

    def op_home(self) -> dict:
        goal = self.base_pose() @ self.start_tcp_base
        poses = self._line(goal)
        rows, blocked = self._run(poses)
        self.stretched_now, self.home_only = None, False
        tcp = self.tcp()
        return self._reply({"moved": True, "steps": len(rows), "blocked": blocked,
                            "home_m": float(np.linalg.norm(tcp[:3, 3] - goal[:3, 3])), "home_deg": 0.0,
                            "goal": goal, "at_limit": self._limits()}, rows)

    def op_wait(self, n) -> dict:
        rows = []
        for _ in range(int(n)):
            if not self._tick(rows):
                break
        return self._reply({"steps": len(rows), **self._gave_way(), "over": self.over}, rows)

    def op_drive(self, target, about=None) -> dict:
        target = np.asarray(target, float)
        start = self.base.copy()
        dist = float(np.hypot(*(target[:2] - start[:2])))
        turn = abs(math.degrees(_wrap(target[2] - start[2])))
        n = 3 + math.ceil(dist / BASE_STEP_M) + math.ceil(turn / BASE_STEP_DEG)
        rows, blocked = [], False
        for k in range(1, n + 1):
            f = k / n
            nxt = start + f * np.r_[target[:2] - start[:2], _wrap(target[2] - start[2])]
            if about is not None:
                a = f * _wrap(target[2] - start[2])
                c, s = math.cos(a), math.sin(a)
                rel = start[:2] - np.asarray(about, float)
                nxt[:2] = np.asarray(about, float) + np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]])
            if nxt[0] + BASE_FRONT > COUNTER_BOX[0][0]:
                blocked = True
                break
            self.base = nxt
            if not self._tick(rows):
                break
        err = float(np.hypot(*(self.base[:2] - target[:2])))
        stretched, self.drive_stretch = self.drive_stretch, False
        jammed, self.drive_jam = self.drive_jam or stretched, False
        blocked = blocked or jammed
        return self._reply({"steps": len(rows), "blocked": blocked, "reached": not blocked and err < 0.01,
                            "pose": self.base.copy(), "err_m": err, "jammed": jammed, "stretched": stretched,
                            "gave_way_back": stretched, "gave_way_m": 0.06 if stretched else 0.05 * jammed,
                            "err_deg": abs(math.degrees(_wrap(self.base[2] - target[2]))), "over": self.over}, rows)

    def op_close(self) -> dict:
        self.closed = True
        return {}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _turn(a: np.ndarray, b: np.ndarray, f: float) -> np.ndarray:
    v = rotvec(b @ a.T)
    n = float(np.linalg.norm(v))
    return a if n < 1e-9 else _axis_angle(v / n, f * n) @ a


def round_trip(payload):
    """``payload`` as the other side of the socket would read it."""
    data = wire.encode(payload)
    (n,) = struct.unpack(">Q", data[:8])
    header = json.loads(data[8:8 + n])
    blobs, pos = [], 8 + n
    for meta in header["blobs"]:
        blobs.append(data[pos:pos + meta["nbytes"]])
        pos += meta["nbytes"]
    return wire.decode(header, blobs)


class FakeLink:
    """Stands in for SimServer: the same call(), replies through the wire encoding."""

    def __init__(self, kitchen: FakeKitchen):
        self.kitchen = kitchen

    def call(self, op: str, **args) -> dict:
        request = round_trip({"op": op, "args": args})
        try:
            reply = {"ok": True, **self.kitchen.handle(request["op"], request["args"])}
        except Refusal as exc:
            reply = {"ok": False, "kind": "refused", "error": str(exc)}
        except Exception as exc:  # noqa: BLE001 - as the server reports a crash
            reply = {"ok": False, "kind": "crash", "error": f"{type(exc).__name__}: {exc}"}
        reply["status"] = self.kitchen.status() if hasattr(self.kitchen, "steps") else {}
        reply = round_trip(reply)
        if not reply["ok"]:
            if reply.get("kind") == "refused":
                raise SimRefused(reply["error"], reply["status"] or None)
            raise SimError(f"simulator server: {reply['error']}", reply["status"] or None)
        return reply

    def close(self) -> None:
        self.kitchen.closed = True


class FakeSam3:
    """Outlines by colour: words name a colour class ('red ...', 'counter'), a point picks the
    connected region of its colour."""

    WORDS = {"red": BOX, "counter": COUNTER, "floor": FLOOR}

    def __init__(self):
        self.requests: list[dict] = []

    def health(self) -> dict:
        return {"ok": True}

    def close(self) -> None:
        pass

    @staticmethod
    def classes(image: np.ndarray) -> np.ndarray:
        out = np.zeros(image.shape[:2], int)
        for k, colour in COLOURS.items():
            out[np.all(image == colour, axis=-1)] = k
        return out

    def segment(self, image, *, text=None, points=None, labels=None, box=None, max_masks=8, min_score=0.0,
                multimask=None) -> list[Segment]:
        import cv2

        self.requests.append({"text": text, "points": points})
        cls = self.classes(np.asarray(image))
        if text is not None:
            kind = next((k for word, k in self.WORDS.items() if word in text.lower()), None)
            if kind is None:
                return []
            n, labels_img = cv2.connectedComponents((cls == kind).astype(np.uint8), connectivity=8)
            parts = [labels_img == i for i in range(1, n)]
            parts.sort(key=lambda m: -int(m.sum()))
            return [_segment(m, 0.9 - 0.1 * i) for i, m in enumerate(parts[:max_masks]) if m.sum() >= 4]
        if points:
            x, y = (int(round(v)) for v in points[0])
            kind = cls[y, x]
            if kind == 0:
                return []
            n, labels_img = cv2.connectedComponents((cls == kind).astype(np.uint8), connectivity=8)
            return [_segment(labels_img == labels_img[y, x], 0.88)]
        return []


def _segment(mask: np.ndarray, score: float) -> Segment:
    ys, xs = np.nonzero(mask)
    return Segment(mask=mask, score=float(score), box=(int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
                   area=int(mask.sum()))


class FakeCasaSession(CasaSession):
    """A CasaSession whose server is the fake kitchen and whose SAM3 is the colour one."""

    def __init__(self, task: str = "PickPlaceCounterToCabinet", seed: int = 0, *, size: int = 256,
                 sam3: bool = True, kitchen: FakeKitchen | None = None):
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused", size=size)
        self.kitchen = kitchen or FakeKitchen(size=size)
        self.sam3 = FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = FakeLink(self.kitchen)

    def _scene(self, settings):
        if self.sam3 is None:
            return None
        from hexashell.vision.rgbd import DepthScene

        return DepthScene("fake", cams=list(CAMS), check_cam=check_cam, log=self.rec.event, client=self.sam3)
