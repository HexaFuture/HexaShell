"""A fake SIMPLE room for the bench tests: a humanoid that stands in front of a table with a red box on it, a gaze
camera on its head (RGB-D), palms that get where they are sent if the arm reaches (counting the steps they would
take), fingers that hold the box when the palm is next to it, a colour-based SAM3 (:class:`fakes_casa.FakeSam3`).

:class:`FakeRoom` answers the operations of :mod:`hexashell.benches.simple.server` with replies of the same shape;
:class:`fakes_casa.FakeLink` passes them through the wire encoding, so the episode side (session, host,
perception, the cells' robot) runs as it does against the real server.  The fake's world is the start frame.
"""

from __future__ import annotations

import math

import numpy as np

import fakes_casa as C
from hexashell.benches.simple import protocol as P
from hexashell.benches.simple.session import SimpleSession

FLOOR, TABLE, BOX, HAND = C.FLOOR, C.COUNTER, C.BOX, C.HAND
TABLE_LO, TABLE_HI = np.array([0.55, -0.6, 0.0]), np.array([1.15, 0.6, 0.70])
BOX_SIZE = np.array([0.06, 0.16, 0.20])
BOX_START = np.array([0.62, -0.10, TABLE_HI[2] + 0.10])
#: the box is lifted (the task done) this far over where it stood
LIFTED_M = 0.05
STAND_HEIGHT = 0.75
#: the lowest the legs' controller takes the pelvis (AMO's floor)
LOWEST = 0.54
#: the shoulders over the pelvis, in the torso's axes, and how far the palm gets from them
SHOULDER = {"left": np.array([0.0, 0.15, 0.30]), "right": np.array([0.0, -0.15, 0.30])}
REACH = 0.55
PALM_START = {"left": np.array([0.20, 0.20, 0.05]), "right": np.array([0.20, -0.20, 0.05])}
CAMERA = np.array([0.06, 0.0, 0.45])
FOCAL = 0.5 * P.GAZE_WIDTH / math.tan(math.radians(0.5 * P.GAZE_HFOV_DEG))
HAND_STEP_M = 0.01
WALK_STEP_M = 0.01
TURN_STEP_DEG = 2.0
#: the pelvis cannot get nearer the table's front than this (the legs hit it)
TABLE_CLEARANCE = 0.25
#: a palm this near the box's surface (m) closes on it
GRIP_GAP = 0.05
#: an open hand whose palm comes this close to the box (0: into it) brushes it with its fingers
BRUSH_M = 0.0


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class FakeRoom:
    """The simulator side: state, operations, status."""

    def __init__(self, task: str = "bend_pick", horizon: int = P.RELAXED_STEPS):
        self.task = task
        self.horizon = horizon
        self.closed = False
        self.ops: list[str] = []

    # -- state -----------------------------------------------------------------------------------
    def reset(self) -> None:
        self.steps = 0
        self.first_success = None
        self.pelvis = np.zeros(2)
        self.heading = 0.0
        self.height = STAND_HEIGHT
        self.pitch = 0.0
        self.yaw = 0.0
        self.gaze = np.array(P.GAZE_START_DEG, float)
        self.palms = {s: self.to_world(PALM_START[s]) for s in P.SIDES}
        self.normals = {"left": np.array([0.0, -1.0, 0.0]), "right": np.array([0.0, 1.0, 0.0])}
        self.fingers = {s: "open" for s in P.SIDES}
        self.holds = {s: "torso" for s in P.SIDES}
        self.box = BOX_START.copy()
        self.carried: tuple[str, np.ndarray] | None = None

    @property
    def over(self) -> bool:
        return self.first_success is not None or self.steps >= self.horizon

    def status(self) -> dict:
        first = self.first_success
        official = P.TASKS[self.task].official_steps
        return {"step": self.steps, "seconds": round(self.steps * P.STEP_S, 2), "horizon": self.horizon,
                "official_steps": official, "success": first is not None, "first_success_step": first,
                "official_success": first is not None and first <= official, "over": self.over}

    def to_world(self, local) -> np.ndarray:
        """A point in the torso's axes over the pelvis -> start frame."""
        base = np.array([self.pelvis[0], self.pelvis[1], self.height])
        return base + rot_z(self.heading + self.yaw) @ np.asarray(local, float)

    def proprio(self) -> dict:
        palms = {}
        for s in P.SIDES:
            palms[s] = {"point": [round(float(v), 4) for v in self.palms[s]],
                        "normal": [round(float(v), 4) for v in self.normals[s]],
                        "finger": [round(float(v), 4) for v in rot_z(self.heading) @ np.array([1.0, 0.0, 0.0])]}
        return {"step": self.steps, "pelvis": [round(float(self.pelvis[0]), 4), round(float(self.pelvis[1]), 4),
                                               round(self.height, 4)],
                "heading": round(self.heading, 2), "pelvis_height": round(self.height, 4),
                "torso_pitch": round(self.pitch, 2), "torso_yaw": round(self.yaw, 2), "palms": palms,
                "fingers": dict(self.fingers), "touching": {s: self.touching(s) for s in P.SIDES},
                "holds": dict(self.holds), "gaze": [round(float(v), 1) for v in self.gaze]}

    def row(self) -> list[float]:
        return [float(self.steps), float(self.pelvis[0]), float(self.pelvis[1]), self.height, self.heading,
                self.height, self.pitch, self.yaw, *self.palms["left"], *self.palms["right"]]

    def _tick(self, rows: list) -> bool:
        """One step; False once the episode is over."""
        if self.over:
            return False
        self.steps += 1
        if self.carried is not None:
            side, offset = self.carried
            self.box = self.palms[side] + offset
        if self.first_success is None and self.carried is not None and self.box[2] >= BOX_START[2] + LIFTED_M:
            self.first_success = self.steps
        for s in P.SIDES:
            self.touched[s].update(self.touching(s) or self.brushing(s))
        rows.append(self.row())
        return not self.over

    def brushing(self, side: str) -> list[str]:
        """An open hand's fingers brush the box when the palm comes within BRUSH_M of it."""
        if self.carried is not None:
            return []
        near = np.clip(self.palms[side], self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2)
        return ["index", "middle"] if np.linalg.norm(near - self.palms[side]) <= BRUSH_M else []

    def touching(self, side: str) -> list[str]:
        if self.fingers[side] != "closed":
            return []
        near = np.clip(self.palms[side], self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2)
        return ["thumb", "index", "middle"] if np.linalg.norm(near - self.palms[side]) <= GRIP_GAP else []

    # -- requests --------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        self.ops.append(op)
        if op == "open":
            return self.op_open(**args)
        if op == "close":
            return {}
        method = getattr(self, "op_" + op, None)
        if method is None:
            raise C.Refusal(f"unknown operation {op!r}")
        rows: list = []
        self.touched = {s: set() for s in P.SIDES}
        out = method(rows, **args)
        out["proprio"] = self.proprio()
        out["proprio"]["touched"] = {s: sorted(self.touched[s]) for s in P.SIDES}
        if rows:
            out["log"] = np.asarray(rows, float)
        return out

    def _ended(self) -> dict:
        return {"result": {"episode_over": True, "status": self.status()}}

    def op_open(self, task, seed, renderer=None, size=None, official_test=False) -> dict:
        P.check_seed(int(seed), official_test=bool(official_test))
        self.opened_with = {"seed": int(seed), "official_test": bool(official_test)}
        self.task = task
        self.reset()
        spec = P.TASKS[task]
        facts = {"task": task, "seed": int(seed), "instruction": spec.instruction.format("cracker box"),
                 "official_steps": spec.official_steps, "relaxed_steps": self.horizon, "step_s": P.STEP_S,
                 "controller": spec.controller, "split": P.split_of(int(seed)), "renderer": renderer or "fake"}
        return {"facts": facts, "start": self.proprio(), "seconds": 0.0}

    def camera_pose(self) -> np.ndarray:
        eye = self.to_world(CAMERA)
        yaw = self.heading + self.yaw + self.gaze[0]
        pitch = min(85.0, self.pitch + self.gaze[1])
        d = rot_z(yaw) @ np.array([math.cos(math.radians(pitch)), 0.0, -math.sin(math.radians(pitch))])
        return C.look_at(eye, eye + d)

    def op_observe(self, rows, cams=None, robot_mask=True) -> dict:
        pose = self.camera_pose()
        boxes = [(TABLE_LO, TABLE_HI, TABLE), (self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2, BOX)]
        spheres = [(self.palms[s], 0.035, HAND) for s in P.SIDES]
        rgb, depth, ids = C.render(pose, FOCAL, (P.GAZE_WIDTH, P.GAZE_HEIGHT), boxes, spheres)
        cam = P.GAZE_CAMERA
        return {"obs": {"step": self.steps, "images": {cam: rgb}, "depth": {cam: depth},
                        "robot": {cam: ids == HAND} if robot_mask else {},
                        "cameras": {cam: {"pose": pose.tolist(), "focal": FOCAL, "width": P.GAZE_WIDTH,
                                          "height": P.GAZE_HEIGHT}}}}

    def op_state(self, rows) -> dict:
        return {}

    def op_body(self, rows, height=None, pitch=None, yaw=None) -> dict:
        h = self.height if height is None else max(LOWEST, float(height))
        p = self.pitch if pitch is None else float(pitch)
        y = self.yaw if yaw is None else float(yaw)
        n = 20
        start = (self.height, self.pitch, self.yaw)
        local = {s: rot_z(-(self.heading + self.yaw)) @ (self.palms[s] - self.to_world(np.zeros(3))) for s in P.SIDES}
        for k in range(1, n + 1):
            f = k / n
            self.height, self.pitch, self.yaw = (a + f * (b - a) for a, b in zip(start, (h, p, y)))
            for s in P.SIDES:
                if self.holds[s] == "torso":
                    self.palms[s] = self.to_world(local[s])
            if not self._tick(rows):
                return self._ended()
        res = {"seconds": round(n * P.STEP_S, 2), "reached": height is None or h <= float(height) + 0.01,
               "height_m": {"asked": None if height is None else float(height), "reached": round(self.height, 4)},
               "pitch_deg": {"asked": pitch, "reached": round(self.pitch, 2)},
               "yaw_deg": {"asked": round(y, 2), "reached": round(self.yaw, 2)}, "command": {}}
        if height is not None and float(height) < LOWEST:
            res["limits"] = ["height command at its limit"]
        return {"result": res}

    def op_walk(self, rows, x, y, heading=None) -> dict:
        goal = np.array([float(x), float(y)])
        goal_h = self.heading if heading is None else float(heading)
        asked = goal.copy()
        blocked = False
        if abs(goal[1]) < TABLE_HI[1] and goal[0] > TABLE_LO[0] - TABLE_CLEARANCE:
            goal[0] = TABLE_LO[0] - TABLE_CLEARANCE
            blocked = True
        start_p, start_h = self.pelvis.copy(), self.heading
        dist = float(np.linalg.norm(goal - start_p))
        turn = wrap(goal_h - start_h)
        n = max(1, int(math.ceil(max(dist / WALK_STEP_M, abs(turn) / TURN_STEP_DEG))))
        local = {s: rot_z(-(self.heading + self.yaw)) @ (self.palms[s] - self.to_world(np.zeros(3))) for s in P.SIDES}
        for k in range(1, n + 1):
            f = k / n
            self.pelvis = start_p + f * (goal - start_p)
            self.heading = wrap(start_h + f * turn)
            for s in P.SIDES:
                if self.holds[s] != "world":
                    self.palms[s] = self.to_world(local[s])
            if not self._tick(rows):
                return self._ended()
        err = float(np.linalg.norm(asked - self.pelvis))
        res = {"seconds": round(n * P.STEP_S, 2), "reached": err <= 0.05, "rounds": 1,
               "asked": [float(x), float(y), round(goal_h, 2)],
               "reached_pose": [round(float(self.pelvis[0]), 4), round(float(self.pelvis[1]), 4),
                                round(self.heading, 2)],
               "position_error_m": round(err, 4), "heading_error_deg": round(wrap(goal_h - self.heading), 2),
               "distance_asked_m": round(float(np.linalg.norm(asked - start_p)), 4)}
        if blocked:
            res["blocked"] = "no 1 cm of progress in 1.5 s"
        return {"result": res}

    def _reachable(self, side: str, point: np.ndarray) -> tuple[np.ndarray, bool]:
        shoulder = self.to_world(SHOULDER[side])
        d = point - shoulder
        n = float(np.linalg.norm(d))
        p = point if n <= REACH else shoulder + d * (REACH / n)
        if TABLE_LO[0] <= p[0] <= TABLE_HI[0] and TABLE_LO[1] <= p[1] <= TABLE_HI[1]:
            p = np.array([p[0], p[1], max(p[2], TABLE_HI[2] + 0.02)])
        return p, n > REACH

    def op_hand(self, rows, side, waypoints, hold="torso", dry_run=False) -> dict:
        if side not in P.SIDES:
            raise C.Refusal(f"side {side!r}")
        last = np.asarray(waypoints[-1]["point"], float)
        if dry_run:
            p, limit = self._reachable(side, last)
            out = {"dry_run": True, "side": side, "reached_point": [round(float(v), 4) for v in p],
                   "position_error_m": round(float(np.linalg.norm(p - last)), 4)}
            if limit:
                out["at_limit"] = [f"{side}_elbow_joint"]
            return {"result": out}
        t0, limit = self.steps, False
        for wp in waypoints:
            goal, lim = self._reachable(side, np.asarray(wp["point"], float))
            limit = limit or lim
            start = self.palms[side].copy()
            n = max(int(round(float(wp.get("seconds", 1.0)) / P.STEP_S)),
                    int(math.ceil(np.linalg.norm(goal - start) / HAND_STEP_M)), 1)
            for k in range(1, n + 1):
                self.palms[side] = start + (k / n) * (goal - start)
                if not self._tick(rows):
                    return self._ended()
            if wp.get("normal") is not None:
                self.normals[side] = np.asarray(wp["normal"], float)
        self.holds[side] = hold
        res = {"seconds": round((self.steps - t0) * P.STEP_S, 2), "side": side,
               "reached_point": [round(float(v), 4) for v in self.palms[side]],
               "position_error_m": round(float(np.linalg.norm(self.palms[side] - last)), 4),
               "ik_position_error_m": 0.0, "hold": hold}
        if waypoints[-1].get("normal") is not None:
            res["normal_error_deg"] = 0.0
        if limit:
            res["at_limit"] = [f"{side}_elbow_joint"]
        return {"result": res}

    def op_fingers(self, rows, side, closed) -> dict:
        wide = closed == "wide"
        closed = False if wide else bool(closed)
        self.fingers[side] = "closed" if closed else "wide" if wide else "open"
        if not closed and self.carried is not None and self.carried[0] == side:
            self.carried = None
            self.box = np.array([self.box[0], self.box[1], BOX_START[2]])  # it drops back onto the table
        for _ in range(50):
            if not self._tick(rows):
                return self._ended()
        touching = self.touching(side)
        if closed and touching:
            self.carried = (side, self.box - self.palms[side])
        return {"result": {"seconds": 1.0, "side": side, "closed": bool(closed), "short_of_target_rad": [],
                           "touching": touching}}

    def op_gaze(self, rows, yaw=None, pitch=None, at=None) -> dict:
        y0, p0 = (float(v) for v in self.gaze)
        if at is not None:
            eye = self.to_world(CAMERA)
            d = rot_z(-(self.heading + self.yaw)) @ (np.asarray(at, float) - eye)
            yaw, pitch = math.degrees(math.atan2(d[1], d[0])), math.degrees(math.atan2(-d[2], math.hypot(d[0], d[1])))
        asked = [round(float(y0 if yaw is None else yaw), 1), round(float(p0 if pitch is None else pitch), 1)]
        y1 = min(P.GAZE_YAW_RANGE_DEG[1], max(P.GAZE_YAW_RANGE_DEG[0], asked[0]))
        p1 = min(P.GAZE_PITCH_RANGE_DEG[1], max(P.GAZE_PITCH_RANGE_DEG[0], asked[1]))
        n = int(math.ceil(max(abs(y1 - y0), abs(p1 - p0)) / P.GAZE_SPEED_DEG_S / P.STEP_S))
        for k in range(1, n + 1):
            self.gaze = np.array([y0 + k / n * (y1 - y0), p0 + k / n * (p1 - p0)])
            if not self._tick(rows):
                return self._ended()
        self.gaze = np.array([y1, p1])
        return {"result": {"seconds": round(n * P.STEP_S, 2), "gaze_deg": [round(y1, 1), round(p1, 1)],
                           "asked_deg": asked}}

    def op_wait(self, rows, n) -> dict:
        t0 = self.steps
        for _ in range(int(n)):
            if not self._tick(rows):
                break
        return {"result": {"steps": self.steps - t0}}


class FakeSimpleSession(SimpleSession):
    """A SimpleSession whose server is the fake room and whose SAM3 is the colour one."""

    def __init__(self, task: str = "bend_pick", seed: int = 1001, *, sam3: bool = True, room: FakeRoom | None = None):
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused")
        self.room = room or FakeRoom(task)
        self.sam3 = C.FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = C.FakeLink(self.room)

    def _scene(self, settings):
        if self.sam3 is None:
            return None
        from hexashell.vision.rgbd import DepthScene

        return DepthScene("fake", cams=list(P.CAMERAS), check_cam=self.check_cam, log=self.rec.event, client=self.sam3)
