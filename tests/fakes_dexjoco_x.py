"""A fake DexJoCo-X scene for the bench tests: Franka arms (the task's) with five-fingered hands in front of a
table with a red box on it, the task's cameras (RGB-D), hands that get where they are sent if the arm reaches
(counting the steps they would take; the table's top stops them), fingers that hold the box when they close with
the grasp point next to it, a colour-based SAM3 (:class:`fakes_casa.FakeSam3`).

:class:`FakeScene` answers the operations of :mod:`hexashell.benches.dexjoco_x.server` with replies of the same
shape; :class:`fakes_casa.FakeLink` passes them through the wire encoding, so the episode side (session, host,
perception, the cells' robot) runs as it does against the real server.  The fake's world is the scene frame.
"""

from __future__ import annotations

import math

import numpy as np

import fakes_casa as C
from hexashell.benches.dexjoco_x import control as K
from hexashell.benches.dexjoco_x import protocol as P
from hexashell.benches.dexjoco_x.session import DexterousSession
from hexashell.benches.dexjoco_x.sim import PALM_POINT

TABLE, BOX, HAND = C.COUNTER, C.BOX, C.HAND
TABLE_TOP = 0.90
TABLE_LO, TABLE_HI = np.array([-0.30, -0.70, 0.0]), np.array([0.60, 0.70, TABLE_TOP])
BOX_SIZE = np.array([0.06, 0.06, 0.10])
BOX_START = np.array([0.15, -0.25, TABLE_TOP + 0.05])
#: the box lifted (the task done) this far over where it stood
LIFTED_M = 0.05
#: the shoulders and how far the palm gets from them
SHOULDER = {"right": np.array([-0.80, 0.0, 1.30]), "left": np.array([-0.80, 0.0, 1.30])}
REACH = 1.25
#: the palms at the start: one arm in the middle, two arms apart
START_Y = {1: {"right": 0.0}, 2: {"right": -0.40, "left": 0.40}}
START_PALM_X, START_PALM_Z = -0.348, 1.4725
#: the fingertips in the attachment frame (x along the fingers, z out of the palm; the left hand mirrors y)
TIPS = {"thumb": (0.13, -0.06, 0.07), "index": (0.24, -0.035, 0.05), "middle": (0.25, 0.0, 0.05),
        "ring": (0.24, 0.03, 0.05), "little": (0.22, 0.055, 0.05)}
#: a closing hand whose grasp point is this near the box (m) holds it
GRIP_GAP = 0.05
#: shapes that close (the others open the hand)
CLOSING = ("power", "wrap", "hook", "pinch", "tripod")
SCENE_FOCAL_FOVY = 45.0
WRIST_FOVY = 60.0
#: the wrist camera in the attachment frame: where it is, its line of sight, its picture's up
WRIST_CAMERA = np.array([-0.15, 0.0, 0.0])
WRIST_AXIS = np.array([math.cos(math.radians(30.0)), 0.0, math.sin(math.radians(30.0))])
WRIST_UP = np.array([math.sin(math.radians(30.0)), 0.0, -math.cos(math.radians(30.0))])
HAND_STEP_M = 0.0075
TURN_STEP_DEG = 1.25


def rotation(axis, angle_rad: float) -> np.ndarray:
    return K.rotation(np.asarray(axis, float) / np.linalg.norm(axis) * angle_rad)


class FakeScene:
    """The simulator side: state, operations, status."""

    def __init__(self, task: str = "hammer_nail", size: int = 256):
        self.task = task
        self.size = size
        self.closed = False
        self.ops: list[str] = []
        #: a test sets it: the next motion lets the held box slip out
        self.slip = False

    # -- state -----------------------------------------------------------------------------------
    def reset(self, task: str) -> None:
        self.task = task
        self.sides = P.arms(task)
        self.horizon = P.TASKS[task].steps
        self.steps = 0
        self.success = False
        self.T = {}
        for s in self.sides:
            T = np.eye(4)
            T[:3, :3] = np.diag([1.0, -1.0, -1.0])  # x forward, z (the palm) down
            T[:3, 3] = np.array([START_PALM_X, START_Y[len(self.sides)][s], START_PALM_Z]) - T[:3, :3] @ PALM_POINT
            self.T[s] = T
        self.start = {s: self.T[s].copy() for s in self.sides}
        self.shape = {s: "open" for s in self.sides}
        self.asked = {s: dict.fromkeys(P.FINGERS, 0.0) for s in self.sides}
        self.box = BOX_START.copy()
        self.carried: tuple[str, np.ndarray] | None = None
        self.pressed = {s: set() for s in self.sides}

    @property
    def over(self) -> bool:
        return self.success or self.steps >= self.horizon

    def status(self) -> dict:
        return {"step": self.steps, "seconds": round(self.steps * P.STEP_S, 2), "horizon": self.horizon,
                "success": self.success, "over": self.over, "terminated": False, "truncated": False, "failure": None}

    def point(self, side: str, name: str) -> np.ndarray:
        T = self.T[side]
        if name == "palm":
            off = PALM_POINT
        elif name == "grasp":
            off = K.GRASP_POINT
        elif name == "camera":
            off = WRIST_CAMERA
        else:
            off = np.array(TIPS[name], float)
            if side == "left":
                off[1] = -off[1]
        return T[:3, :3] @ off + T[:3, 3]

    def pressing(self, side: str) -> list[str]:
        if self.carried is not None and self.carried[0] == side:
            return ["thumb", "index", "middle"]
        return ["index", "middle"] if self._brushes(side) else []

    def proprio(self) -> dict:
        hands = {}
        for s in self.sides:
            pressing = self.pressing(s)
            curl = dict(self.asked[s])
            for f in pressing:
                curl[f] = round(0.6 * curl[f], 2)
            hands[s] = {"palm": [round(float(v), 4) for v in self.point(s, "palm")],
                        "normal": [round(float(v), 3) for v in self.T[s][:3, 2]],
                        "finger": [round(float(v), 3) for v in self.T[s][:3, 0]],
                        "grasp": [round(float(v), 4) for v in self.point(s, "grasp")],
                        "tips": {f: [round(float(v), 4) for v in self.point(s, f)] for f in P.FINGERS},
                        "camera": [round(float(v), 4) for v in self.point(s, "camera")],
                        "view": [round(float(v), 3) for v in self.T[s][:3, :3] @ WRIST_AXIS],
                        "picture_up": [round(float(v), 3) for v in self.T[s][:3, :3] @ WRIST_UP],
                        "shape": self.shape[s], "curl": curl, "asked": dict(self.asked[s]),
                        "pressing": pressing, "holding": "thumb" in pressing and len(pressing) >= 2}
        return {"step": self.steps, "hands": hands}

    def row(self) -> list[float]:
        row = [float(self.steps)]
        for s in ("right", "left"):
            row += list(self.point(s, "palm")) if s in self.sides else [math.nan] * 3
        row.append(1.0 if self.success else 0.0)
        row += [float(self.carried is not None and self.carried[0] == s) if s in self.sides else math.nan
                for s in ("right", "left")]
        return row

    def _tick(self, rows: list) -> bool:
        """One step; False once the episode is over."""
        if self.over:
            return False
        self.steps += 1
        if self.carried is not None:
            side, offset = self.carried
            self.box = self.point(side, "grasp") + offset
            if not self.success and self.box[2] >= BOX_START[2] + LIFTED_M:
                self.success = True
        for s in self.sides:
            if (self.carried is None or self.carried[0] != s) and self._brushes(s):
                self.pressed[s].update(("index", "middle"))
        rows.append(self.row())
        return not self.over

    def _brushes(self, side: str) -> bool:
        tip = self.point(side, "middle")
        return bool(np.all(np.abs(tip - self.box) <= BOX_SIZE / 2 + 0.005))

    # -- requests --------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        self.ops.append(op)
        if op == "open":
            return self.op_open(**args)
        if op == "close":
            return {}
        if not hasattr(self, "sides"):
            raise C.Refusal("no episode: send open first")
        method = getattr(self, "op_" + op, None)
        if method is None:
            raise C.Refusal(f"unknown operation {op!r}")
        rows: list = []
        self.pressed = {s: set() for s in self.sides}
        out = method(rows, **args)
        if self.slip and self.carried is not None and op in ("hand", "arc", "home", "wait"):
            self.slip = False
            self.carried = None
            self.box = np.array([self.box[0], self.box[1], BOX_START[2]])
        out["proprio"] = self.proprio()
        out["proprio"]["pressed"] = {s: [f for f in P.FINGERS if f in self.pressed[s]] for s in self.sides}
        if rows:
            out["log"] = np.asarray(rows, float)
        return out

    def _over(self, res: dict) -> dict:
        return {"result": {**res, "episode_over": self.over}}

    def op_open(self, task, seed, size=None, official_test=False) -> dict:
        try:
            P.check_task(task)
            P.check_seed(task, int(seed), official_test=bool(official_test))
        except ValueError as exc:
            raise C.Refusal(str(exc)) from None
        self.opened_with = {"task": task, "seed": int(seed), "official_test": bool(official_test)}
        self.size = int(size or self.size)
        self.reset(task)
        rows: list = []
        for _ in range(10):
            self._tick(rows)
        facts = {"task": task, "seed": int(seed), "hand": P.HAND, "instruction": "Pick up the red box.",
                 "steps": self.horizon, "arms": list(self.sides), "translation_bound_m": P.TASKS[task].translation_m,
                 "rotation_bound_rad": P.TASKS[task].rotation_rad,
                 "cameras": {c: c for c in P.cameras(task)}, "split": P.split_of(task, int(seed)),
                 "start_hold_steps": 10, "hand_points": list(P.HAND_POINTS), "shapes": list(P.SHAPES)}
        return {"facts": facts, "start": self.proprio(), "seconds": 0.0}

    # -- pictures --------------------------------------------------------------------------------
    def camera_pose(self, cam: str) -> tuple[np.ndarray, float]:
        if cam == "scene":
            if P.TASKS[self.task].scene_camera == "across":
                pose = C.look_at((1.3, 0.0, 1.6), (-0.1, 0.0, 0.95))
            else:
                pose = C.look_at((-1.3, 0.0, 2.2), (0.3, 0.0, 0.6))
            fovy = SCENE_FOCAL_FOVY
        else:
            side = cam.split("_")[0]
            T = self.T[side]
            eye = self.point(side, "camera")
            pose = C.look_at(eye, eye + T[:3, :3] @ WRIST_AXIS)
            fovy = WRIST_FOVY
        return pose, 0.5 * self.size / math.tan(math.radians(0.5 * fovy))

    def op_observe(self, rows, cams=None, robot_mask=True) -> dict:
        cams = list(cams or P.cameras(self.task))
        for cam in cams:
            if cam not in P.cameras(self.task):
                raise C.Refusal(f"camera {cam!r}: this task has {', '.join(P.cameras(self.task))}")
        boxes = [(TABLE_LO, TABLE_HI, TABLE), (self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2, BOX)]
        spheres = [(self.point(s, "palm"), 0.04, HAND) for s in self.sides]
        obs = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "cameras": {}}
        for cam in cams:
            pose, focal = self.camera_pose(cam)
            rgb, depth, ids = C.render(pose, focal, self.size, boxes, spheres)
            obs["images"][cam], obs["depth"][cam] = rgb, depth
            if robot_mask:
                obs["robot"][cam] = ids == HAND
            obs["cameras"][cam] = {"pose": pose, "focal": focal, "width": self.size, "height": self.size}
        return {"obs": obs}

    def op_state(self, rows) -> dict:
        return {}

    # -- motions ---------------------------------------------------------------------------------
    def _goal_rotation(self, side, normal, finger, along) -> np.ndarray:
        R_now = self.T[side][:3, :3]
        if along is not None:
            return K.grip_frame(side, along, R_now[:, 2] if normal is None else normal)
        if normal is None and finger is None:
            return R_now.copy()
        return K.frame_of(normal, finger, R_now)

    def _reach(self, side: str, p: np.ndarray) -> tuple[np.ndarray, bool]:
        d = p - SHOULDER[side]
        n = float(np.linalg.norm(d))
        return (p, False) if n <= REACH else (SHOULDER[side] + d * (REACH / n), True)

    def _move(self, rows, side: str, point: str, target: np.ndarray, R_goal: np.ndarray, speed: float) -> str | None:
        """Moves the hand's point in a straight line, the orientation with it; None, or why it stopped."""
        T0 = self.T[side].copy()
        off = T0[:3, :3].T @ (self.point(side, point) - T0[:3, 3])
        p0 = self.point(side, point)
        turn = K.rotvec(R_goal @ T0[:3, :3].T)
        n = max(1, int(math.ceil(max(float(np.linalg.norm(target - p0)) / (speed * P.STEP_S),
                                     math.degrees(float(np.linalg.norm(turn))) / (25.0 * P.STEP_S)))))
        for k in range(1, n + 1):
            s = k / n
            R = K.rotation(turn * s) @ T0[:3, :3]
            p = p0 + (target - p0) * s
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = R, p - R @ off
            palm = T[:3, :3] @ PALM_POINT + T[:3, 3]
            if TABLE_LO[0] <= palm[0] <= TABLE_HI[0] and TABLE_LO[1] <= palm[1] <= TABLE_HI[1] \
                    and palm[2] < TABLE_TOP + 0.03:
                return f"the hand fell {100 * float(np.linalg.norm(target - self.point(side, point))):.1f} cm " \
                       "behind its path (something holds it up)"
            self.T[side] = T
            if not self._tick(rows):
                return None
        return None

    def op_hand(self, rows, side, xyz, point="palm", normal=None, finger=None, along=None, speed=None, approach=None,
                dry_run=False, look_at=None) -> dict:
        if side not in self.sides:
            raise C.Refusal(f"side {side!r}")
        target = np.asarray(xyz, float)
        if look_at is not None:
            if point != "camera":
                raise C.Refusal("look_at needs point='camera'")
            a = np.asarray(look_at, float) - target
            R_goal = K.view_frame(WRIST_AXIS, WRIST_UP, a, K.view_up(a))
        else:
            R_goal = self._goal_rotation(side, normal, finger, along)
        reached_p, limit = self._reach(side, target)
        if dry_run:
            return {"result": {"dry_run": True, "side": side, "point": point,
                               "target": [round(float(v), 4) for v in target],
                               "error_m": round(float(np.linalg.norm(reached_p - target)), 4), "error_deg": 0.0,
                               "at_limit": [4] if limit else []}}
        t0 = self.steps
        sp = 0.15 if speed is None else float(speed)
        blocked, stage = None, None
        if approach:
            blocked = self._move(rows, side, point, reached_p - float(approach) * R_goal[:, 2], R_goal, sp)
            stage = "on the way to the approach's start" if blocked else None
        if blocked is None and not self.over:
            blocked = self._move(rows, side, point, reached_p, R_goal, min(sp, K.APPROACH_SPEED) if approach else sp)
            stage = "on the approach" if blocked and approach else stage
        at = self.point(side, point)
        err = float(np.linalg.norm(at - target))
        out = {"side": side, "point": point, "target": [round(float(v), 4) for v in target],
               "at": [round(float(v), 4) for v in at], "error_m": round(err, 4), "error_deg": 0.0,
               "reached": blocked is None and err <= 0.01, "blocked": blocked, "at_limit": [], "steps": self.steps - t0}
        if blocked is None and limit:
            out.update(blocked="the arm is at the end of its travel (joint 4): it gets no farther this way",
                       at_limit=[4], reached=False)
        if stage:
            out["stage"] = stage
        return self._over(out)

    def op_arc(self, rows, side, center, axis, angle, speed=None) -> dict:
        c, a = np.asarray(center, float), np.asarray(axis, float)
        T0 = self.T[side].copy()
        n = max(1, int(math.ceil(abs(float(angle)) / TURN_STEP_DEG)))
        t0 = self.steps
        for k in range(1, n + 1):
            Rr = rotation(a, math.radians(float(angle)) * k / n)
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = Rr @ T0[:3, :3], Rr @ (T0[:3, 3] - c) + c
            self.T[side] = T
            if not self._tick(rows):
                break
        return self._over({"side": side, "turned_deg": round(float(angle) * k / n, 1), "asked_deg": float(angle),
                           "reached": True, "blocked": None, "steps": self.steps - t0,
                           "palm": [round(float(v), 4) for v in self.point(side, "palm")]})

    def op_home(self, rows, side=None) -> dict:
        sides = list(self.sides) if side is None else [side]
        t0 = self.steps
        for s in sides:
            T = self.start[s]
            self._move(rows, s, "palm", T[:3, :3] @ PALM_POINT + T[:3, 3], T[:3, :3], 0.15)
        return self._over({"sides": {s: {"reached": True, "blocked": None} for s in sides}, "steps": self.steps - t0})

    def op_fingers(self, rows, side, shape=None, amount=1.0, curl=None, thumb=None, oppose=None, spread=None,
                   seconds=0.5) -> dict:
        asked = dict(self.asked[side])
        if shape is not None:
            base = 0.0 if shape in ("open", "flat") else 0.6 if shape == "point" else 0.7
            asked = {f: round(base * float(amount), 2) for f in P.FINGERS}
        for f, c in (curl or {}).items():
            asked[f] = round(float(c), 2)
        if thumb is not None:
            asked["thumb"] = round(float(thumb), 2)
        self.asked[side] = asked
        settings = bool(curl) or thumb is not None or oppose is not None or spread is not None
        self.shape[side] = "custom" if settings or shape is None else shape if amount >= 0.999 else \
            f"{shape} {float(amount):.2f}"
        closing = (shape in CLOSING and amount >= 0.5) or (shape is None and sum(asked.values()) / 5 >= 0.5)
        if not closing and self.carried is not None and self.carried[0] == side:
            self.carried = None
            self.box = np.array([self.box[0], self.box[1], BOX_START[2]])  # it drops back onto the table
        t0 = self.steps
        for _ in range(max(1, int(round(float(seconds) / P.STEP_S))) + 5):
            if not self._tick(rows):
                return self._over({"side": side, "shape": self.shape[side], "curl": asked, "asked": asked,
                                   "pressing": [], "holding": False, "steps": self.steps - t0})
        g = self.point(side, "grasp")
        near = np.clip(g, self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2)
        if closing and self.carried is None and float(np.linalg.norm(near - g)) <= GRIP_GAP:
            self.carried = (side, self.box - g)
        pressing = self.pressing(side)
        return self._over({"side": side, "shape": self.shape[side], "curl": self.proprio()["hands"][side]["curl"],
                           "asked": asked, "pressing": pressing, "holding": "thumb" in pressing and len(pressing) >= 2,
                           "steps": self.steps - t0})

    def op_wait(self, rows, n) -> dict:
        t0 = self.steps
        for _ in range(int(n)):
            if not self._tick(rows):
                break
        return self._over({"steps": self.steps - t0})


class FakeDexterousSession(DexterousSession):
    """A DexterousSession whose server is the fake scene and whose SAM3 is the colour one."""

    def __init__(self, task: str = "hammer_nail", seed: int | None = None, *, sam3: bool = True,
                 scene: FakeScene | None = None, size: int = 256):
        seed = P.seeds(task, "learning")[0] if seed is None else seed
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused")
        self.size = size
        self.room = scene or FakeScene(task, size=size)
        self.sam3 = C.FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = C.FakeLink(self.room)

    def _scene(self, settings):
        if self.sam3 is None:
            return None
        from hexashell.vision.rgbd import DepthScene

        return DepthScene("fake", cams=list(self.cams), check_cam=self.check_cam, log=self.rec.event,
                          client=self.sam3)
