"""RoboTwin 2.0 in the server process: one episode, its steps, its pictures.

Runs in RoboTwin's own Python environment (``robotwin``, SAPIEN, mplib), with ``CUDA_VISIBLE_DEVICES``
set to the GPU that renders and ``ROBOTWIN_ASSETS_ROOT`` to the assets folder (:mod:`.server` sees
to both before SAPIEN is imported).

An episode is built the way RoboTwin's evaluation builds it: the ``demo_randomized`` settings, the
aloha-agilex embodiment, ``setup_demo(seed)``, then the task's step limit.  A step is one
``take_action`` with joint targets (14 numbers: left arm, left gripper, right arm, right gripper).
The cameras are rendered only when pictures are asked for, with RoboTwin's renderer (ray traced)
and RoboTwin's conversions: RGB = colour x 255 clipped, depth = minus the camera-frame z of the
rendered surface, zero where nothing was rendered.

What leaves this process: the robot's own readings (joint angles, link6 poses, finger positions),
the pictures (with depth, or without for perception from the colours alone), the robot's pixels in
them (or its parts there: :data:`.protocol.ROBOT_PARTS`), the camera models, the step count and whether
the environment's success check has passed.
"""

from __future__ import annotations

import contextlib
import copy
import glob
import importlib
import io
import os
import time
import types

import numpy as np

from . import protocol as P

ARMS = ("left", "right")
#: the head camera is one of the embodiment's static cameras; the wrist cameras ride on the arms
STATIC = {"head_camera"}
#: the tasks that pick an object's model among its files in the order the folder lists them
MODEL_FILE_TASKS = ("place_a2b_left", "place_a2b_right", "place_object_scale", "place_object_stand",
                    "put_object_cabinet")
#: shifted builds made before a scene of a task with a shifted table (its own build is the next one)
TABLE_SHIFTS_BEFORE = 3
#: the tasks whose ``get_info`` runs the expert's motions on the scene (place_bread_basket when one arm
#: takes the breads): their scene is built again after it
MOVING_INFO_TASKS = ("blocks_ranking_size", "place_bread_basket", "stack_blocks_three")
#: the clutter function's own default ranges, as found before the first build
_CLUTTER_DEFAULTS: dict = {}


def assets_root() -> str:
    root = os.environ.get("ROBOTWIN_ASSETS_ROOT")
    if not root:
        raise RuntimeError("ROBOTWIN_ASSETS_ROOT is not set: the folder that holds RoboTwin's assets/")
    os.environ.setdefault("ASSETS_PATH", root)  # what RoboTwin's task code reads
    return root


@contextlib.contextmanager
def quiet():
    """RoboTwin prints progress on every step; keep it out of the server log."""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


def task_args(task: str, save_dir: str) -> dict:
    """The arguments of ``setup_demo``, as RoboTwin's vectorised evaluation makes them."""
    import yaml
    from robotwin.config import load_task_config

    root = assets_root()
    args = load_task_config("demo_randomized")
    # mplib: cuRobo's planner is only needed by the expert demonstrations; both build the same scene
    args.update(task_name=task, task_config="demo_randomized", step_lim=P.step_limit(task), planner_backend="mplib",
                eval_mode=True, eval_video_log=False, render_freq=0, save_data=False)
    cameras = load_task_config("_camera_config")
    args["head_camera_h"] = cameras["D435"]["h"]
    args["head_camera_w"] = cameras["D435"]["w"]
    kind = args["embodiment"]
    if len(kind) != 1:
        raise RuntimeError(f"expected one embodiment for both arms, not {kind}")
    robot_file = os.path.join(root, load_task_config("_embodiment_config")[kind[0]]["file_path"])
    with open(os.path.join(robot_file, "config.yml"), encoding="utf-8") as f:
        embodiment = yaml.load(f.read(), Loader=yaml.FullLoader)
    args.update(left_robot_file=robot_file, right_robot_file=robot_file, dual_arm_embodied=True,
                left_embodiment_config=embodiment, right_embodiment_config=copy.deepcopy(embodiment),
                embodiment_name=str(kind[0]), rdt_step=10, n_envs=1, action_dim=14, clear_cache_freq=5,
                save_path=os.path.join(save_dir, f"{task}_reward"))
    return args


def make_instruction(task: str, info: dict, seed: int) -> str:
    """An episode's instruction, made as RoboTwin's evaluation makes one: a description template of the
    episode's kind (:func:`.protocol.instruction_kind`: 'unseen' for a test episode, 'seen' for a
    development one) filled with this episode's objects, in descriptions of the same kind, and arm
    (``info``: ``get_info()``)."""
    from robotwin.description import generate_episode_descriptions

    kind = P.instruction_kind(seed)
    found = generate_episode_descriptions(task, [info], 1, int(seed))
    if not found or not found[0].get(kind):
        raise RuntimeError(f"no {kind} instruction could be made for {task} seed {seed}")
    return str(found[0][kind][0])


def numbered_glob(pattern: str, *args, **kwargs) -> list[str]:
    """``glob.glob``, with RoboTwin's model files (``model_data<N>.json``) in the order of their numbers."""
    def number(path: str):
        name = os.path.basename(path).replace("model_data", "").replace(".json", "")
        return (0, int(name), path) if name.isdigit() else (1, 0, path)

    found = glob.glob(pattern, *args, **kwargs)
    return sorted(found, key=number) if "model_data" in pattern else found


def reset_clutter_area(env, task: str) -> None:
    """Before a build: the clutter area as RoboTwin's published evaluation had it for this task.

    ``get_cluttered_table`` moves its default x and y ranges (lists shared by every build) by the
    table's offset in place, so every build of a task with a shifted table (:data:`.protocol.TABLE_SHIFT_X`)
    moves the clutter area of all later builds of the process (fixed upstream on 2026-09-18, not in this
    runtime).  From the fourth shifted build on, no clutter fits on the table: RoboTwin's
    evaluation, one process per task, built every published seed of those tasks so.  The ranges are set
    back to the function's own before every build; a task with a shifted table then has them moved as
    three builds before would have moved them, its own build moves them once more: no clutter, as
    published.  Every other task gets the clutter area of a fresh process."""
    fn = next((vars(cls)["get_cluttered_table"] for cls in type(env).__mro__
               if "get_cluttered_table" in vars(cls)), None)
    if fn is None or not fn.__defaults__:
        return
    own = _CLUTTER_DEFAULTS.setdefault(fn, copy.deepcopy(fn.__defaults__))
    code = fn.__code__
    names = code.co_varnames[code.co_argcount - len(own):code.co_argcount]
    defaults = dict(zip(names, copy.deepcopy(own)))
    shift = P.TABLE_SHIFT_X.get(task, 0.0)
    for _ in range(TABLE_SHIFTS_BEFORE if shift else 0):
        defaults["xlim"][0] += shift  # as the function itself adds it
        defaults["xlim"][1] += shift
    fn.__defaults__ = tuple(defaults[name] for name in names)


def fix_task_module(task: str, module) -> None:
    """Two fixes to a task's module.

    The tasks of :data:`MODEL_FILE_TASKS` pick an object's model with ``np.random.choice`` among the
    model files in the order ``glob`` lists them, the file system's order, so the same seed builds
    other models on another file system; they get the files in the order of their numbers, as upstream
    has them since PR #490 (2026-09-24, not in this runtime).

    ``place_fan`` builds its reward (for reinforcement learning; the evaluation's success check does
    not use it) as ``Reward(subtasks=..., transition_rewards=...)``, which this runtime's ``Reward`` (a
    factory: ``build``, ``build_top``) does not take, so its scenes cannot be built.  It gets the reward
    every other task builds: ``Reward.build_top({"type": "Serial", ...})``."""
    if task in MODEL_FILE_TASKS and getattr(module, "glob", None) is glob:
        module.glob = types.SimpleNamespace(glob=numbered_glob)
    original = getattr(module, "Reward", None)
    if task != "place_fan" or original is None or not hasattr(original, "build_top") \
            or getattr(original, "serial_call", False):
        return

    class Reward(original):
        serial_call = True

        def __new__(cls, **config):
            return original.build_top({"type": "Serial", **config})

    module.Reward = Reward


def build(env, task: str, seed: int, args: dict) -> None:
    """``setup_demo`` of an episode's scene, the clutter area reset before it; an unstable scene raises
    UnStableError."""
    reset_clutter_area(env, task)
    try:
        with quiet():
            env.setup_demo(now_ep_num=seed, seed=seed, **args)
    except Exception:
        with contextlib.suppress(Exception), quiet():
            env.close_env(clear_cache=True)  # as RoboTwin's evaluation does before the next seed
        raise
    env.step_lim = P.step_limit(task)


def pose_matrix(pose) -> np.ndarray:
    """4x4 of a SAPIEN pose (quaternion w, x, y, z)."""
    w, x, y, z = (float(v) for v in pose.q)
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [float(v) for v in pose.p]
    return m


class TwinSim:
    """One RoboTwin episode at a time."""

    def __init__(self, save_dir: str | None = None):
        self.save_dir = save_dir or os.path.join(os.getcwd(), "robotwin_scratch")
        self.env = None
        self.task: str | None = None
        self.seed: int | None = None
        self.instruction: str | None = None
        self.cams: dict = {}
        self.robot_ids: np.ndarray = np.zeros(0, int)
        self.part_lut: np.ndarray = np.zeros(1, np.uint8)  # segmentation id -> part number (P.ROBOT_PARTS)
        self.flanges: dict = {}
        self.fingers: dict = {}

    # -- the episode ---------------------------------------------------------------------------------
    def open(self, task: str, seed: int) -> dict:
        P.check_task(task)
        seed = int(seed)
        self.close(clear_cache=False)
        t0 = time.time()
        args = task_args(task, self.save_dir)
        module = importlib.import_module(f"robotwin.envs.{task}")
        fix_task_module(task, module)
        env = getattr(module, task)()
        P.check_seed(task, seed)
        build(env, task, seed, args)
        # Some success checks read what the expert decides from the first state: which arm counts
        # (open_laptop, place_object_scale, put_object_cabinet) and the object's height then
        # (put_object_cabinet).  RoboTwin's evaluation runs the expert (play_once) on the same task
        # object before it builds the scene again; get_info sets the same from the scene as built.
        info = env.get_info()
        if task in MOVING_INFO_TASKS:  # get_info has moved the robot, maybe the objects: the scene again
            with quiet():
                env.close_env(clear_cache=False)
            build(env, task, seed, args)
        self.env, self.task, self.seed = env, task, seed
        self.instruction = make_instruction(task, info, seed)
        env.set_instruction(instruction=self.instruction)
        self._bind()
        return {"task": task, "seed": seed, "instruction": self.instruction,
                "instruction_from": f"the task's {P.instruction_kind(seed)} descriptions",
                "step_limit": P.step_limit(task), "setup_s": round(time.time() - t0, 1), "cameras": list(self.cams)}

    def _bind(self) -> None:
        env = self.env
        robot = env.robot
        cams = env.cameras
        static = dict(zip(cams.static_camera_name, cams.static_camera_list))
        self.cams = {"head": static["head_camera"], "left_wrist": cams.left_camera,
                     "right_wrist": cams.right_camera}
        names = {}
        for entity in {id(e): e for e in (robot.left_entity, robot.right_entity)}.values():
            for link in entity.get_links():
                names[int(link.entity.per_scene_id)] = link.get_name()
        self.robot_ids = np.array(sorted(names), int)
        self.part_lut = np.zeros(max(names, default=0) + 1, np.uint8)
        for i, name in names.items():
            self.part_lut[i] = P.part_of(name)
        self.flanges, self.fingers = {}, {}
        for arm, entity, tip, gripper, scale in (
                ("left", robot.left_entity, "fl_link6", robot.left_gripper, robot.left_gripper_scale),
                ("right", robot.right_entity, "fr_link6", robot.right_gripper, robot.right_gripper_scale)):
            self.flanges[arm] = next(link for link in entity.get_links() if link.get_name() == tip)
            joint, mult, offset = gripper[0]
            index = entity.get_active_joints().index(joint)
            self.fingers[arm] = (entity, index, float(mult), float(offset), float(scale[0]), float(scale[1]))

    @property
    def step(self) -> int:
        return int(self.env.take_action_cnt) if self.env is not None else 0

    @property
    def success(self) -> bool:
        return bool(self.env is not None and self.env.eval_success)

    @property
    def over(self) -> bool:
        return self.success or self.step >= P.step_limit(self.task)

    def status(self) -> dict:
        if self.env is None:
            return {}
        return {"step": self.step, "horizon": P.step_limit(self.task), "success": self.success, "over": self.over}

    # -- the robot -----------------------------------------------------------------------------------
    def gripper_reading(self, arm: str) -> float:
        """The fingers' opening, 0 (squeezing as far as the command goes) .. 1 (fully open), read from
        the finger joint itself (the command RoboTwin reports runs ahead of the fingers)."""
        entity, index, mult, offset, lo, hi = self.fingers[arm]
        q = float(entity.get_qpos()[index])
        return ((q - offset) / mult - lo) / (hi - lo)

    def proprio(self) -> dict:
        robot = self.env.robot
        real = {"left": robot.get_left_arm_real_jointState(), "right": robot.get_right_arm_real_jointState()}
        targets = {"left": robot.get_left_arm_jointState(), "right": robot.get_right_arm_jointState()}
        out = {}
        for arm in ARMS:
            out[arm] = {"joints": np.asarray(real[arm][:6], float),
                        "flange": pose_matrix(self.flanges[arm].get_pose()),
                        "gripper": self.gripper_reading(arm),
                        "target": np.asarray(targets[arm][:6], float),
                        "grip_command": float(targets[arm][6])}
        return out

    def act(self, action) -> None:
        """One step: joint targets and gripper openings of both arms (14 numbers)."""
        a = np.asarray(action, float).reshape(-1)
        if a.size != 14 or not np.all(np.isfinite(a)):
            raise ValueError("an action is 14 finite numbers: left joints 1-6, left gripper, right joints, right gripper")
        a[6] = min(max(a[6], 0.0), 1.0)
        a[13] = min(max(a[13], 0.0), 1.0)
        if self.over:
            return
        with quiet():
            self.env.take_action(a, action_type="qpos")

    # -- pictures ------------------------------------------------------------------------------------
    def parts_of(self, seg: np.ndarray) -> np.ndarray:
        """The robot's part numbers (P.ROBOT_PARTS) of a segmentation picture's ids, 0 off the robot."""
        lut = self.part_lut
        seg = np.asarray(seg).astype(np.int64)
        inside = (seg >= 0) & (seg < len(lut))
        return np.where(inside, lut[np.clip(seg, 0, len(lut) - 1)], 0).astype(np.uint8)

    def observe(self, cams=None, robot_mask: bool = True, depth: bool = True, parts: bool = False) -> dict:
        """The cameras' pictures now: colours, and as asked depth, the robot's pixels and its parts."""
        names = list(self.cams) if not cams else [str(c) for c in cams]
        for name in names:
            if name not in self.cams:
                raise ValueError(f"no camera {name!r} (cameras: {', '.join(self.cams)})")
        t0 = time.time()
        self.env._update_render()  # the wrist cameras follow the arms; lights as RoboTwin sets them
        images, depths, robot, part_maps, cameras = {}, {}, {}, {}, {}
        for name in names:
            cam = self.cams[name]
            cam.take_picture()
            rgba = cam.get_picture("Color")
            rgba8 = (rgba * 255).clip(0, 255).astype(np.uint8)
            images[name] = np.ascontiguousarray(rgba8[..., :3])
            if depth:
                position = cam.get_picture("Position")
                depths[name] = (-position[..., 2] * (rgba8[..., 3] / 255.0)).astype(np.float32)
            if robot_mask or parts:
                seg = cam.get_picture("Segmentation")[..., 1]
                if robot_mask:
                    robot[name] = np.isin(seg, self.robot_ids)
                if parts:
                    part_maps[name] = self.parts_of(seg)
            k = np.asarray(cam.get_intrinsic_matrix(), float)
            cameras[name] = {"focal": float(k[0, 0]), "focal_y": float(k[1, 1]), "center": (float(k[0, 2]),
                             float(k[1, 2])), "pose": np.asarray(cam.get_model_matrix(), float),
                             "width": int(cam.get_width()), "height": int(cam.get_height())}
        out = {"images": images, "depth": depths, "robot": robot, "cameras": cameras, "step": self.step,
               "render_s": round(time.time() - t0, 2)}
        if parts:
            out["parts"] = part_maps
        return out

    def close(self, clear_cache: bool = True) -> None:
        if self.env is not None:
            env, self.env = self.env, None
            with quiet():
                env.close_env(clear_cache=clear_cache)
