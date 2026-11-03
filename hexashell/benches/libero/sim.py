"""The LIBERO-PRO environment in the server process: build, reset, step, render, read.

Runs in the LIBERO-PRO Python environment (``liberopro`` on robosuite 1.5, MuJoCo).  Nothing here is
seen by the model directly; :mod:`.server` answers the episode side.

Frame: everything the episode side gets is in the *arm frame*, the frame of the robot's base (body
``robot0_base``): origin at the arm's base plate, +x forward (from the arm toward the middle of the
workspace), +y left, +z up.  LIBERO's robot never moves, so this frame holds for the whole episode.
Hand poses are the forward kinematics of the robot's own joints (proprioception); cameras are given
as pinhole models in the same frame.

Hand orientation ("tool" axes): x points from the wrist out between the fingers (the approach
direction), y is the line the fingers open along, z = x × y.
"""

from __future__ import annotations

import os
import time

import numpy as np

from .. import arm as A
from ..gpucheck import check_depth, check_time
from . import protocol as P

GRIP_SITE = "gripper0_right_grip_site"
BASE_BODY = "robot0_base"
ARM_BASE_BODY = "robot0_link0"
SHOULDER_BODY = "robot0_link2"
FINGER_JOINTS = ("gripper0_right_finger_joint1", "gripper0_right_finger_joint2")
ROBOT_PREFIXES = ("robot0_", "gripper0_", "mount0_")
SITE_TO_TOOL = A.PANDA_SITE_TO_TOOL
#: the still action of the settling steps: no arm motion, the gripper opening
SETTLE_ACTION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
RESET_TRIES = 20


def task_of(suite_name: str, index: int):
    """(task, its initial states, the path of its BDDL file) from LIBERO-PRO's registry."""
    from liberopro.liberopro import benchmark, get_libero_path

    suites = benchmark.get_benchmark_dict()
    if suite_name not in suites:
        raise ValueError(f"{suite_name!r} is not a LIBERO-PRO suite")
    suite = suites[suite_name]()
    if not 0 <= int(index) < suite.n_tasks:
        raise ValueError(f"{suite_name} has tasks 0-{suite.n_tasks - 1}, not {index}")
    task = suite.get_task(int(index))
    states = suite.get_task_init_states(int(index))
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    return task, states, bddl


def make(bddl: str, steps: int, size: int = P.IMAGE_SIZE):
    """LIBERO's off-screen environment for episodes of ``steps`` control steps (pictures are rendered on
    demand only, not every step)."""
    from liberopro.liberopro.envs import OffScreenRenderEnv

    return OffScreenRenderEnv(bddl_file_name=bddl, camera_names=list(P.CAMERAS.values()), camera_heights=size,
                              camera_widths=size, camera_depths=False, use_camera_obs=False,
                              horizon=steps + P.SETTLE_STEPS + 1, hard_reset=False)


class LiberoSim:
    """One task's environment; ``reset(seed)`` starts an episode."""

    def __init__(self, task: str, size: int = P.IMAGE_SIZE):
        self.suite, self.index = P.parse_task(task)
        self.task = task
        self.size = int(size)
        self.horizon = P.official_steps(task)
        self.info, self.init_states, self.bddl = task_of(self.suite, self.index)
        self.env = make(self.bddl, self.horizon, self.size)
        self.steps = 0
        self.success = False
        self.drawn = False  # a picture rendered since the episode began
        self.facts: dict = {}

    @property
    def rs(self):
        """The robosuite environment inside LIBERO's wrapper."""
        return self.env.env

    # -- episode ---------------------------------------------------------------------------------
    def reset(self, seed: int) -> dict:
        from robosuite.utils.errors import RandomizationError

        env, seed = self.env, int(seed)
        np.random.seed(seed)  # the fixtures are placed with the global generator
        env.seed(seed)
        for _ in range(RESET_TRIES):  # LIBERO's own reset retries a failed placement for ever
            try:
                self.rs.reset()
                break
            except RandomizationError:
                continue
        else:
            raise RuntimeError(f"LIBERO could not place the fixtures in {RESET_TRIES} tries")
        init = P.init_index(seed, len(self.init_states))
        env.set_init_state(self.init_states[init])
        self.model = self.rs.sim.model._model
        self.data = self.rs.sim.data._data
        for _ in range(P.SETTLE_STEPS):
            env.step(np.array(SETTLE_ACTION))
        self.steps = 0
        self.success = False
        self.drawn = False
        self.to_start = np.linalg.inv(self.body_world(BASE_BODY))
        self._ids()
        A.check_tool_axes(self.model, self.data, self.site_id, SITE_TO_TOOL)
        self.facts = {
            "task": self.task, "suite": self.suite, "task_index": self.index, "seed": seed, "init_state": init,
            "instruction": str(self.info.language).strip(), "task_label": str(self.info.name),
            "robot": type(self.rs.robots[0].robot_model).__name__, "horizon": self.horizon, "step_s": P.STEP_S,
            "settle_steps": P.SETTLE_STEPS,
        }
        return self.facts

    def _ids(self) -> None:
        import mujoco

        m = self.model
        self.site_id = m.site(GRIP_SITE).id
        self.link0_id = m.body(ARM_BASE_BODY).id
        self.shoulder_id = m.body(SHOULDER_BODY).id
        self.finger_qpos = [int(m.jnt_qposadr[m.joint(j).id]) for j in FINGER_JOINTS]
        self.finger_dof = [int(m.jnt_dofadr[m.joint(j).id]) for j in FINGER_JOINTS]
        self.cam_ids = {cam: m.camera(name).id for cam, name in P.CAMERAS.items()}
        robot = []
        for g in range(m.ngeom):
            body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or ""
            if body.startswith(ROBOT_PREFIXES):
                robot.append(g)
        self.robot_geoms = np.array(sorted(robot), dtype=np.int64)
        self.arm_qpos, self.arm_dof, self.arm_limits = A.arm_joints(m)

    @property
    def over(self) -> bool:
        return self.success or self.steps >= self.horizon

    def step(self, action: np.ndarray) -> bool:
        """One control step (20 Hz) with LIBERO's 7-dim action; then the task's success check.
        Returns False once the episode is over."""
        if self.over:
            return False
        self.env.step(np.asarray(action, dtype=np.float64))
        self.steps += 1
        if bool(self.env.check_success()):
            self.success = True
        return not self.over

    def status(self) -> dict:
        return {"step": self.steps, "horizon": self.horizon, "success": self.success, "over": self.over}

    # -- poses (arm frame) -----------------------------------------------------------------------
    def body_world(self, name: str) -> np.ndarray:
        m, d = self.rs.sim.model._model, self.rs.sim.data._data
        i = m.body(name).id
        pose = np.eye(4)
        pose[:3, :3] = d.xmat[i].reshape(3, 3)
        pose[:3, 3] = d.xpos[i]
        return pose

    def body(self, index: int) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, :3] = self.data.xmat[index].reshape(3, 3)
        pose[:3, 3] = self.data.xpos[index]
        return self.to_start @ pose

    def site_pose(self) -> np.ndarray:
        """The grip site in the arm frame (MuJoCo site axes)."""
        pose = np.eye(4)
        pose[:3, :3] = self.data.site_xmat[self.site_id].reshape(3, 3)
        pose[:3, 3] = self.data.site_xpos[self.site_id]
        return self.to_start @ pose

    def tcp(self) -> np.ndarray:
        """The grasp point with the tool axes, arm frame."""
        pose = self.site_pose()
        pose[:3, :3] = pose[:3, :3] @ SITE_TO_TOOL
        return pose

    def link0(self) -> np.ndarray:
        return self.body(self.link0_id)

    def shoulder(self) -> np.ndarray:
        return self.body(self.shoulder_id)[:3, 3]

    def finger_width(self) -> float:
        q = self.data.qpos[self.finger_qpos]
        return float(q[0] - q[1])

    def finger_speed(self) -> float:
        v = self.data.qvel[self.finger_dof]
        return float(abs(v[0] - v[1]))

    def arm_joints(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos].copy()

    def proprio(self) -> dict:
        return {"tcp": self.tcp(), "width": self.finger_width(), "joints": self.arm_joints()}

    # -- pictures --------------------------------------------------------------------------------
    def camera(self, cam: str) -> dict:
        i = self.cam_ids[cam]
        pose = np.eye(4)
        pose[:3, :3] = self.data.cam_xmat[i].reshape(3, 3)  # MuJoCo: looks along -z, +y up the picture
        pose[:3, 3] = self.data.cam_xpos[i]
        fovy = np.radians(float(self.model.cam_fovy[i]))
        return {"pose": self.to_start @ pose, "focal": 0.5 * self.size / np.tan(0.5 * fovy),
                "width": self.size, "height": self.size}

    def picture(self, cam: str, **kw):
        """robosuite's picture of camera ``cam`` (``depth=True``: with depth; ``segmentation=True``: of the
        geoms), when the GPU drew it in time (gpucheck.check_time)."""
        t0 = time.time()
        out = self.rs.sim.render(width=self.size, height=self.size, camera_name=P.CAMERAS[cam], **kw)
        check_time(time.time() - t0, cam, first=not self.drawn)
        self.drawn = True
        return out

    def observe(self, cams=None, robot_mask: bool = True) -> dict:
        """RGB, depth (m along the optical axis) and the robot's own pixels of each camera, with the
        camera models and the proprioception at this step."""
        import mujoco
        from robosuite.utils.camera_utils import get_real_depth_map

        sim = self.rs.sim
        out = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "cameras": {}}
        for cam in cams or list(P.CAMERAS):
            rgb, depth = self.picture(cam, depth=True)
            out["images"][cam] = np.ascontiguousarray(rgb[::-1])
            depth = check_depth(depth[::-1], cam)
            out["depth"][cam] = np.ascontiguousarray(
                get_real_depth_map(sim, np.clip(depth, 0.0, 1.0)).astype(np.float32))
            if robot_mask:
                seg = self.picture(cam, segmentation=True)[::-1]
                geom = seg[..., 0] == int(mujoco.mjtObj.mjOBJ_GEOM)
                out["robot"][cam] = np.ascontiguousarray(geom & np.isin(seg[..., 1], self.robot_geoms))
            out["cameras"][cam] = self.camera(cam)
        out["proprio"] = self.proprio()
        return out

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:  # noqa: BLE001
            pass
