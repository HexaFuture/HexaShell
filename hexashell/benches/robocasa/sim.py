"""The official RoboCasa environment in the server process: build, reset, step, render, read.

Runs in the RoboCasa Python environment (robosuite master, robocasa with the determinism patch,
MuJoCo).  Nothing here is seen by the model directly; :mod:`.server` answers the episode side.

Frame: everything the episode side gets is in the *start frame*, the base frame at the start of the
episode: origin on the floor under the base (body ``mobilebase0_base``), +x forward, +y left, +z up.
It stays fixed while the base drives.  Base and hand poses are the forward kinematics of the robot's
own joints (proprioception); cameras are given as pinhole models in the same frame.

Hand orientation ("tool" axes): x points from the wrist out between the fingers (the approach
direction), y is the line the fingers open along, z = x × y.
"""

from __future__ import annotations

import contextlib
import time

import numpy as np

from .. import arm as A
from ..arm import planar, planar_pose  # noqa: F401  (the server builds base poses with them)
from ..gpucheck import check_depth, check_time
from . import protocol as P

GRIP_SITE = "gripper0_right_grip_site"
BASE_BODY = "mobilebase0_base"
ARM_BASE_BODY = "robot0_link0"
SHOULDER_BODY = "robot0_link2"
FINGER_JOINTS = ("gripper0_right_finger_joint1", "gripper0_right_finger_joint2")
FINGER_STROKE_M = 0.08
TORSO_JOINT = "mobilebase0_joint_torso_height"
ROBOT_PREFIXES = ("robot0_", "gripper0_", "mobilebase0_")
SITE_TO_TOOL = A.PANDA_SITE_TO_TOOL
#: stream that keeps the reset's unseeded generators apart from env.rng (seeded with the bare seed)
FALLBACK_STREAM = 0x5EED


def make(task: str, size: int = P.IMAGE_SIZE, split: str = "pretrain50"):
    """The environment as robocasa's create_env builds it for the split (pictures are rendered on
    demand only)."""
    import robocasa  # noqa: F401  (registers the kitchen tasks)
    import robosuite
    from robosuite.controllers import load_composite_controller_config

    if P.split_of(split) == "pretrain":  # create_env(split="pretrain"): the seed draws the kitchen
        scene = dict(layout_ids=P.PRETRAIN_SCENES, style_ids=P.PRETRAIN_SCENES, layout_and_style_ids=None)
    else:  # reset() narrows the list to the kitchen of the seed
        scene = dict(layout_ids=None, style_ids=None, layout_and_style_ids=[tuple(k) for k in P.PRETRAIN50])
    return robosuite.make(
        env_name=task,
        robots=P.ROBOT,
        controller_configs=load_composite_controller_config(controller=None, robot=P.ROBOT),
        camera_names=list(P.CAMERAS.values()),
        camera_widths=size,
        camera_heights=size,
        has_renderer=False,
        has_offscreen_renderer=True,
        use_camera_obs=False,
        use_object_obs=True,
        translucent_robot=False,
        generative_textures=None,
        randomize_cameras=False,
        obj_instance_split="pretrain",
        **scene,
        ignore_done=True,
    )


@contextlib.contextmanager
def _seeded_fallbacks(seed: int):
    """While open, ``np.random.default_rng()`` without a seed draws from a stream of ``seed`` (parts of
    the reset ask for a fresh generator instead of using env.rng)."""
    original = np.random.default_rng
    stream = np.random.SeedSequence([int(seed), FALLBACK_STREAM])

    def default_rng(seed=None):
        return original(stream.spawn(1)[0] if seed is None else seed)

    np.random.default_rng = default_rng
    try:
        yield
    finally:
        np.random.default_rng = original


class CasaSim:
    """One task's environment; ``reset(seed)`` starts an episode."""

    def __init__(self, task: str, size: int = P.IMAGE_SIZE, split: str = "pretrain50"):
        P.task_set(task)
        self.task = task
        self.size = int(size)
        self.split = P.split_of(split)
        self.horizon = P.HORIZON[task]
        self.env = make(task, self.size, self.split)
        self.steps = 0
        self.success = False
        self.drawn = False  # a picture rendered since the episode began
        self.facts: dict = {}

    # -- episode ---------------------------------------------------------------------------------
    def reset(self, seed: int) -> dict:
        env, seed = self.env, int(seed)
        if self.split == "pretrain50":
            layout, style = P.kitchen(seed, env.EXCLUDE_LAYOUTS, env.EXCLUDE_STYLES, P.skipped(self.task, seed))
            env.layout_and_style_ids = [(layout, style)]
        env.seed = seed
        env.rng = np.random.default_rng(seed)
        np.random.seed(seed)
        if env.sim is not None and env.hard_reset and not env.deterministic_reset:
            env._destroy_sim()  # robosuite frees the old sim only for its own viewer: it would leak
        with _seeded_fallbacks(seed):
            env.reset()
        if self.split != "pretrain50":  # the kitchen the seed drew
            layout, style = int(env.layout_id), int(env.style_id)
        self.model = env.sim.model._model
        self.data = env.sim.data._data
        self.steps = 0
        self.success = False
        self.drawn = False
        base = self.body_world(BASE_BODY)
        x, y, yaw = planar(base)
        self.to_start = np.linalg.inv(planar_pose(x, y, yaw, float(base[2, 3])))
        self._ids()
        self._check_tool_axes()
        meta = env.get_ep_meta() if hasattr(env, "get_ep_meta") else {}
        self.facts = {
            "task": self.task, "seed": seed, "split": self.split, "task_set": P.task_set(self.task),
            "seed_set": P.seed_set(seed) if self.split == "pretrain50" else None,
            "layout_id": int(getattr(env, "layout_id", layout)), "style_id": int(getattr(env, "style_id", style)),
            "kitchen": [layout, style], "instruction": str(meta.get("lang", "")).strip(),
            "horizon": self.horizon, "step_s": P.STEP_S,
        }
        return self.facts

    def _ids(self) -> None:
        import mujoco

        m = self.model
        self.site_id = m.site(GRIP_SITE).id
        self.base_id = m.body(BASE_BODY).id
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

    def _check_tool_axes(self) -> None:
        A.check_tool_axes(self.model, self.data, self.site_id, SITE_TO_TOOL)

    @property
    def over(self) -> bool:
        return self.success or self.steps >= self.horizon

    def step(self, action: np.ndarray) -> bool:
        """One control step (20 Hz) with the official 12-dim action; then the official success check.
        Returns False once the episode is over."""
        if self.over:
            return False
        self.env.step(np.asarray(action, dtype=np.float64))
        self.steps += 1
        if bool(self.env._check_success()):
            self.success = True
        return not self.over

    def status(self) -> dict:
        return {"step": self.steps, "horizon": self.horizon, "success": self.success, "over": self.over}

    # -- poses (start frame) ---------------------------------------------------------------------
    def body_world(self, name: str) -> np.ndarray:
        m, d = self.env.sim.model._model, self.env.sim.data._data
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
        """The grip site in the start frame (MuJoCo site axes)."""
        pose = np.eye(4)
        pose[:3, :3] = self.data.site_xmat[self.site_id].reshape(3, 3)
        pose[:3, 3] = self.data.site_xpos[self.site_id]
        return self.to_start @ pose

    def tcp(self) -> np.ndarray:
        """The grasp point with the tool axes, start frame."""
        pose = self.site_pose()
        pose[:3, :3] = pose[:3, :3] @ SITE_TO_TOOL
        return pose

    def base(self) -> np.ndarray:
        """(x, y, heading rad) of the base, start frame."""
        return np.array(planar(self.body(self.base_id)))

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

    def torso(self) -> float:
        m = self.model
        return float(self.data.qpos[m.jnt_qposadr[m.joint(TORSO_JOINT).id]])

    def proprio(self) -> dict:
        return {"tcp": self.tcp(), "base": self.base(), "width": self.finger_width(), "joints": self.arm_joints(),
                "torso": self.torso()}

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
        out = self.env.sim.render(width=self.size, height=self.size, camera_name=P.CAMERAS[cam], **kw)
        check_time(time.time() - t0, cam, first=not self.drawn)
        self.drawn = True
        return out

    def observe(self, cams=None, robot_mask: bool = True) -> dict:
        """RGB, depth (m along the optical axis) and the robot's own pixels of each camera, with the
        camera models and the proprioception at this step."""
        from robosuite.utils.camera_utils import get_real_depth_map

        sim = self.env.sim
        out = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "cameras": {}}
        for cam in cams or list(P.CAMERAS):
            rgb, depth = self.picture(cam, depth=True)
            out["images"][cam] = np.ascontiguousarray(rgb[::-1])
            depth = check_depth(depth[::-1], cam)
            out["depth"][cam] = np.ascontiguousarray(
                get_real_depth_map(sim, np.clip(depth, 0.0, 1.0)).astype(np.float32))
            if robot_mask:
                seg = self.picture(cam, segmentation=True)[::-1]
                import mujoco

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

