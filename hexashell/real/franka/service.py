"""The service: one owner of the robot and its cameras, answering the part of URAI's HTTP interface that
:class:`hexashell.real.urai.client.UraiClient` uses (a motion is a draft, a preview, an execution).

A plan belongs to the latest picture of the planning camera and to the joints it started from; executing checks
them again and runs a plan once.  Without ``allow_motion`` it plans and reads, and nothing can reach the driver's
acting calls.
"""

from __future__ import annotations

import base64
import io
import threading
import time
import uuid
from dataclasses import dataclass

import numpy as np

from ...geometry import homogeneous
from ..urai.frames import rotation_rpy, rpy_of
from .planner import Refusal, goal_of, plan_joints, plan_line

ARM = "arm"
#: What a finished goto may be off by; the host's Move.ok compares with these
POSE_TOLERANCE = {"position_mm": 5.0, "orientation_deg": 2.0}
#: The arm must still be this close to where a plan starts (rad)
START_TOLERANCE_RAD = 0.03
#: The arm counts as having moved since a picture beyond this (rad)
MOVED_RAD = 0.002


class Conflict(Exception):
    """A request that cannot be done now or as asked: HTTP 409 with the reason."""


class Forbidden(Exception):
    """A request a read-only service does not do: HTTP 403."""


@dataclass
class Plan:
    id: str
    actions: list
    start_q: np.ndarray
    planning_only: bool
    summary: dict
    used: bool = False


def encode_array(array: np.ndarray) -> dict:
    stream = io.BytesIO()
    np.save(stream, array, allow_pickle=False)
    return {"encoding": "npy-base64", "data": base64.b64encode(stream.getvalue()).decode("ascii")}


class FrankaService:
    def __init__(self, rig, driver, cameras: dict, *, allow_motion: bool = False):
        self.rig, self.driver, self.cameras = rig, driver, cameras
        self.arm = rig.arm()
        self.allow_motion = allow_motion
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.observation_id: str | None = None
        self.observation_q: np.ndarray | None = None
        self.draft: dict | None = None
        self.revision = 0
        self.plan: Plan | None = None
        self.execution = {"id": None, "state": "idle", "execution_time_s": 0.0, "error": None, "last_error": None}

    # -- reading ---------------------------------------------------------------------------------
    def arm_state(self, q: np.ndarray | None = None) -> dict:
        q = self.driver.joints() if q is None else q
        tcp = self.arm.tcp(q)
        return {"xyz": tcp[:3, 3].tolist(), "rpy_deg": rpy_of(tcp[:3, :3]), "joints_deg": np.degrees(q).tolist(),
                "gripper_mm": self.driver.gripper_width_m() * 1000.0, "feedback_age_s": 0.0,
                "enabled": not self.driver.health()["has_errors"]}

    def backend(self) -> dict:
        return {"name": self.driver.name, "mode": self.driver.mode, "family": "franka_fr3",
                "read_only": not self.allow_motion, "cameras": list(self.cameras),
                "joint_names": self.arm.names, "pose_reference": "the rig's world frame, Franka's TCP",
                "health": self.driver.health()}

    def state(self) -> dict:
        q = self.driver.joints()
        stale = self.observation_q is None or float(np.max(np.abs(q - self.observation_q))) > MOVED_RAD
        return {"backend": self.backend(), "execution": dict(self.execution), "arms": {ARM: self.arm_state(q)},
                "task_queue": {}, "observation_id": self.observation_id, "observation_stale": stale}

    def model(self) -> dict:
        return {"name": self.driver.name, "arm_names": [ARM], "table_z": self.arm.table_z,
                "cameras": list(self.cameras),
                "arms": {ARM: {"t_world_base": self.arm.t_world_base.tolist(),
                               "tcp_offset_mm": self.arm.tool_length_m * 1000.0, "table_z": self.arm.table_z,
                               "tip_past_tcp_mm": self.arm.tip_past_tcp_m * 1000.0,
                               "max_opening_mm": self.driver.max_width_m * 1000.0,
                               "joint_limits_deg": np.degrees([self.arm.low, self.arm.high]).T.tolist()}}}

    def capture(self, camera: str):
        if camera not in self.cameras:
            raise ValueError(f"no camera {camera!r}; the cameras are {sorted(self.cameras)}")
        q = self.driver.joints()
        return self.cameras[camera].capture(), q

    def observe(self, camera: str) -> dict:
        with self.lock:
            picture, q = self.capture(camera)
            frame_id = uuid.uuid4().hex
            if camera == self.rig.plan_camera:
                self.observation_id, self.observation_q = frame_id, q
            capsules = {ARM: [{"name": c.name, "a": c.a.tolist(), "b": c.b.tolist(), "radius_m": c.radius_m}
                              for c in self.arm.capsules(q)]}
            return {"id": frame_id, "timestamp": picture.timestamp, "camera": camera,
                    "rgb": encode_array(picture.rgb), "depth": encode_array(picture.depth),
                    "k": picture.k.tolist(), "t_world_camera": self.rig.cameras[camera].pose(self.arm, q).tolist(),
                    "arms": {ARM: self.arm_state(q)}, "capsules": capsules}

    def jpeg(self, camera: str) -> bytes:
        import cv2

        picture, _ = self.capture(camera)
        ok, data = cv2.imencode(".jpg", cv2.cvtColor(picture.rgb, cv2.COLOR_RGB2BGR))
        if not ok:
            raise RuntimeError(f"camera {camera}: the picture could not be encoded")
        return data.tobytes()

    # -- planning --------------------------------------------------------------------------------
    def put_draft(self, body: dict) -> dict:
        with self.lock:
            if body["observation_id"] != self.observation_id:
                raise Conflict("the draft names an old picture; observe the planning camera again")
            arms = body["arms"]
            if set(arms) != {ARM}:
                raise ValueError(f"the arm is {ARM!r}, not {sorted(arms)}")
            self.revision += 1
            self.draft = {"arms": arms, "revision": self.revision}
            return {"revision": self.revision}

    def _running(self) -> bool:
        return self.execution["state"] in ("running", "cancelling")

    def preview(self, body: dict, *, full: bool) -> dict:
        with self.lock:
            planning_only = bool(body.get("planning_only", False))
            if self.draft is None or body.get("expected_revision") != self.draft["revision"]:
                raise Conflict("the preview does not name the latest draft")
            if self._running():
                raise Conflict("a motion is running")
            if not planning_only and not self.allow_motion:
                raise Conflict("this service is read-only (started without --allow-motion): it plans but does not move")
            q = self.driver.joints()
            actions, summary = self._compile(self.draft["arms"][ARM], q)
            plan_id = uuid.uuid4().hex
            duration = sum(_action_duration(action) for action in actions)
            summary = summary if full else {key: value for key, value in summary.items() if key != "xyz"}
            answer = {"id": plan_id, "duration_s": duration, "pose_tolerance": dict(POSE_TOLERANCE),
                      "planning_only": planning_only, "arms": {ARM: summary}}
            self.plan = Plan(plan_id, actions, q, planning_only, answer)
            return answer

    def _compile(self, spec: dict, q: np.ndarray) -> tuple[list, dict]:
        primitive = spec["primitive"]
        speed = min(float(spec.get("speed_m_s", self.rig.max_speed_m_s)), self.rig.max_speed_m_s)
        try:
            if primitive == "goto":
                target = homogeneous(rotation_rpy(spec["rpy_deg"]), spec["xyz"])
                if spec["motion"] == "line":
                    path = plan_line(self.arm, q, target, speed)
                elif spec["motion"] == "swing":
                    path = plan_joints(self.arm, q, goal_of(self.arm, target, q), speed)
                else:
                    raise ValueError(f"unknown motion {spec['motion']!r}")
                grip = [("grip", self._width(spec["opening_mm"]))] if "opening_mm" in spec else []
                return [*grip, ("path", path)], self._path_summary(path)
            if primitive == "home":
                goal = np.radians(spec["joints_deg"])
                path = plan_joints(self.arm, q, goal, speed, kind="home")
                return [("path", path)], self._path_summary(path)
        except Refusal as exc:
            raise Conflict(str(exc)) from exc
        here = {"xyz": [self.arm.tcp(q)[:3, 3].tolist()], "path": primitive}
        if primitive == "gripper":
            return [("grip", self._width(spec["opening_mm"]))], here
        if primitive == "wait":
            return [("wait", float(spec["duration_s"]))], here
        raise ValueError(f"unknown primitive {primitive!r}")

    def _width(self, opening_mm: float) -> float:
        width = float(opening_mm) / 1000.0
        if not 0.0 <= width <= self.driver.max_width_m + 1e-9:
            raise Conflict(f"opening {opening_mm} mm is outside 0..{self.driver.max_width_m * 1000.0:.1f} mm")
        return min(width, self.driver.max_width_m)

    def _path_summary(self, path) -> dict:
        return {"xyz": [self.arm.tcp(q)[:3, 3].tolist() for q in path.q], "path": path.kind,
                "joints_deg": np.degrees(path.q[-1]).tolist()}

    # -- acting ----------------------------------------------------------------------------------
    def execute(self, body: dict) -> dict:
        with self.lock:
            if not self.allow_motion:
                raise Forbidden("this service is read-only (started without --allow-motion)")
            plan = self.plan
            if plan is None or plan.id != body["preview_id"] or plan.used or plan.planning_only:
                raise Conflict("that plan is not the latest unexecuted one")
            if self._running():
                raise Conflict("a motion is running")
            health = self.driver.health()
            if health["has_errors"]:
                raise Conflict(f"the robot reports errors and an operator must recover it first: {health['errors']}")
            moved = float(np.max(np.abs(self.driver.joints() - plan.start_q)))
            if moved > START_TOLERANCE_RAD:
                raise Conflict(f"the arm has moved {moved:.3f} rad since the plan was made; plan again")
            plan.used = True
            self.cancel_event.clear()
            self.execution = {"id": plan.id, "state": "running", "execution_time_s": 0.0, "error": None,
                              "last_error": None}
            self.thread = threading.Thread(target=self._run, args=(plan,), daemon=True)
            self.thread.start()
            return dict(self.execution)

    def _run(self, plan: Plan) -> None:
        start = time.monotonic()
        state, error = "completed", None
        try:
            for kind, value in plan.actions:
                if self.cancel_event.is_set():
                    break
                if kind == "path":
                    self.driver.follow(value, self.cancel_event)
                elif kind == "grip":
                    self.driver.grip(value, self.cancel_event)
                else:
                    self.cancel_event.wait(value)
            if self.cancel_event.is_set():
                state = "cancelled"
        except Exception as exc:  # noqa: BLE001 - whatever ended the motion is the execution's result
            state, error = "error", f"{type(exc).__name__}: {exc}"
        with self.lock:
            self.execution = {"id": plan.id, "state": state, "execution_time_s": time.monotonic() - start,
                              "error": error, "last_error": error}

    def cancel(self) -> dict:
        with self.lock:
            if self._running():
                self.cancel_event.set()
                self.execution = {**self.execution, "state": "cancelling"}
            return dict(self.execution)

    def recover(self) -> dict:
        with self.lock:
            if not self.allow_motion:
                raise Forbidden("this service is read-only (started without --allow-motion)")
            if self._running():
                raise Conflict("a motion is running")
            previous = dict(self.execution)
            recovered = bool(self.driver.recover())
            return {"recovered": recovered, "previous_execution": previous, "health": self.driver.health()}

    def wait_idle(self, timeout_s: float = 60.0) -> None:
        thread = self.thread
        if thread is not None:
            thread.join(timeout_s)


def _action_duration(action) -> float:
    kind, value = action
    if kind == "path":
        return value.duration_s
    if kind == "wait":
        return value
    return 0.0


