"""SIMPLE's G1 environments in the server process: build, reset, step, read, take pictures.

Runs in the SIMPLE Python environment (bench-simple: Python 3.10, MuJoCo 3.3.6, SIMPLE's own package).
Physics and the official success check are MuJoCo's.  Pictures are Isaac Sim's (``renderer="isaac"``, as in
the official evaluation, where Isaac Sim only draws: MuJoCo's state is copied into it and nothing comes back)
or MuJoCo's own (``renderer="mujoco"``: no room around the scene, for quick tests).  Isaac Sim draws only
when a picture is taken (SIMPLE's evaluation copies the state and draws every camera at every step).
Nothing here is seen by the model directly.

Stepping: ``step`` does what the environment's own step does for the physics and the success check
(SIMPLE's ``LocoManipulationEnv.step`` for the MP tasks: the command, the AMO robot's ten MuJoCo substeps;
``SonicLocoManipEnv.step`` for the teleoperation tasks: the command and a MuJoCo step, four times; then the
step count and the task's reward and success check on the objects' poses) without drawing every camera at
every step; pictures are taken on demand.  The stand before the first step is the official evaluation's:
SIMPLE's StandStabilizationWrapper for the MP tasks, the stabilisation loop of its teleoperation evaluation
for the others (``_stand``).  Teleoperation tasks run GR00T's decoupled whole-body controller on simulated
time (wbc.py); ``self.wbc`` turns its goal into the step's command.

Layouts: learning and validation layouts are made the way the official episodes were made (protocol,
Episodes): drawn, stood on, saved with the robot where the stand left it, restored through SIMPLE's own
reset, stood on again.  A test seed's layout is its official evaluation episode's saved configuration
(protocol.official_environment_config), restored and stood on as SIMPLE's evaluations do.

Frames: SIMPLE's world has z up and the ground is the plane at ``floor_z``.  HITCH's observation frame O
(``frame()``): origin on the ground under the pelvis, x along the pelvis's heading, y to its left, z up;
it is taken anew at each observation.

Hands: the palm point is the centre of the palm's face; ``normal`` points out of the palm (the side the
fingers close to), ``finger`` from the wrist towards the finger roots.  Both are fixed in the wrist-yaw
link (the Dex3 palm is a fixed part of it in SIMPLE's MJCF).

Gaze camera: added to the task's cameras before the scene is built (SIMPLE's camera randomiser leaves
cameras as configured and runs after everything else is drawn, so the layouts do not change); it hangs
at SIMPLE's head-camera position (MuJoCo: on ``torso_link``; Isaac Sim: SIMPLE puts head cameras under
the robot's ``d435_link``, which is pitched 47.6 deg down) and is turned by setting its orientation (it
takes no part in the physics).  Its pose for a picture is the pose of the camera that drew it.

Depth: along the optical axis (m); 0 where nothing is within GAZE_FAR_M (Isaac Sim gives no distance
there, MuJoCo its far plane).
"""

from __future__ import annotations

import copy
import json
import math
import os
import random
import time

import numpy as np

from ..gpucheck import check_depth, check_time
from . import protocol as P

PELVIS = "pelvis"
TORSO = "torso_link"
WRIST = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
#: palm point in the wrist-yaw link: the palm link's origin (URDF *_hand_palm_joint: x 0.0415, y -/+0.003)
#: plus the centre of the palm's face in it (the palm mesh spans x 0 .. 0.0867, its face at y +/-0.020)
PALM_POINT = {"left": np.array([0.0865, -0.017, 0.0]), "right": np.array([0.0865, 0.017, 0.0])}
#: out of the palm's face: the fingers close towards +y (right hand) and -y (left hand) of the palm link
PALM_NORMAL = {"left": np.array([0.0, -1.0, 0.0]), "right": np.array([0.0, 1.0, 0.0])}
FINGER_AXIS = np.array([1.0, 0.0, 0.0])
#: SIMPLE's head camera on torso_link (engines/mujoco.py, mount "eye_in_head")
HEAD_CAMERA_POSITION = np.array([0.05366004 + 0.0039635, 0.01752999, 0.4738702 - 0.044])
#: SIMPLE's head camera frame in torso_link (engines/mujoco.py DEFAULT_HEAD_CAM_ORIENTATION, w x y z):
#: 47.6 deg about y, i.e. pitched down; in Isaac Sim this is the robot's d435_link
HEAD_CAMERA_ORIENTATION = np.array([0.91496, 0.0, 0.40355, 0.0])
#: an Isaac-style camera frame (x forward, y left, z up) -> MuJoCo's (looks along -z, +y up)
ISAAC_TO_MUJOCO_CAMERA = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
#: SIMPLE's Dex3 presets (robots/g1_wholebody.py: controller_cfg.*_eef.close_qpos; open is all zeros).
#: Not used for closing: at these angles the thumb and the curled fingers stay about 8 cm apart, so they
#: do not hold the 7.3 cm cracker box (BendPick); kept for reference.
HAND_CLOSE = {
    "left": (0.3523, -0.0964, 0.2790, -0.5058, -1.1950, -0.5389, -0.9835),
    "right": (0.02331954, -0.02398408, -0.22170663, 0.25662386, 1.3371105, 0.3085137, 0.9805285),
}
#: closing = squeezing: targets near the fully curled end of each joint's range (joint order thumb_0,
#: thumb_1, thumb_2, index_0, index_1, middle_0, middle_1), so the fingers curl until they touch the
#: object and then press with the joints' torque limits (3-5 N m, SIMPLE's PD)
HAND_SQUEEZE = {
    "left": (0.0, 1.0, 1.7, -1.5, -1.7, -1.5, -1.7),
    "right": (0.0, -1.0, -1.7, 1.5, 1.7, 1.5, 1.7),
}


def segment(r) -> np.ndarray:
    """A MuJoCo Renderer's segmentation (object id, object type per pixel) for the scene it last updated.
    Pixels whose ID colour matches no geom of the scene (colours blended at edges by the offscreen
    buffer's multisampling) read as background (-1, -1); MuJoCo's own Renderer.render raises IndexError
    on them and leaves the scene in ID-colour mode."""
    flags = r._scene.flags.copy()
    r.enable_segmentation_rendering()
    try:
        return r.render().copy()
    except IndexError:
        from mujoco import _render

        raw = np.empty((r.height, r.width, 3), dtype=np.uint8)
        _render.mjr_render(r._rect, r._scene, r._mjr_context)
        _render.mjr_readPixels(raw, None, r._rect, r._mjr_context)
        ids = raw.astype(np.int64)
        seg = ids[..., 0] + ids[..., 1] * 256 + ids[..., 2] * 65536
        n = r._scene.ngeom
        table = np.full((n + 1, 2), -1, dtype=np.int32)
        for g in r._scene.geoms[:n]:
            if g.segid != -1:
                table[g.segid + 1] = (g.objid, g.objtype)
        seg[(seg < 0) | (seg > n)] = 0
        out = table[seg]
        return np.flipud(out).copy() if r._gl_context else out
    finally:
        np.copyto(r._scene.flags, flags)
        r.disable_segmentation_rendering()


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def heading(R: np.ndarray) -> float:
    """Heading (rad, left positive) of a body's x axis on the ground."""
    return math.atan2(R[1, 0], R[0, 0])


def pitch_down(R: np.ndarray) -> float:
    """How far (rad) a body's x axis points below the horizontal (leaning forward > 0)."""
    return math.asin(max(-1.0, min(1.0, -R[2, 0])))


def mat2quat(R: np.ndarray) -> np.ndarray:
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, dtype=np.float64).reshape(9))
    return q


def quat2mat(q) -> np.ndarray:
    """Rotation matrix of a quaternion (w, x, y, z)."""
    import mujoco

    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=np.float64) / np.linalg.norm(q))
    return R.reshape(3, 3)


def far_clip(depth: np.ndarray) -> np.ndarray:
    """Depth beyond the gaze camera's range (or none: Isaac Sim's inf) set to 0, no depth."""
    out = np.array(depth, dtype=np.float32)
    out[(out > P.GAZE_FAR_M) | np.isposinf(out)] = 0.0
    return out


def pose(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def make(task: str, sim_mode: str = "mujoco"):
    """SIMPLE's environment for ``task`` as its official evaluation builds it, not yet reset: gym.make with
    the evaluation's render rate; MP tasks with the MP evaluation's success criterion, teleoperation tasks
    with GR00T's simulation config and no success criterion (the teleoperation evaluation passes none, so
    each task keeps its own)."""
    import gymnasium as gym

    import simple.envs  # noqa: F401  (registers the environments)

    spec = P.check_task(task)
    extra = {}
    if spec.target_object is not None:
        extra["target_object" if spec.controller == "amo" else "target"] = spec.target_object
    if spec.controller == "amo":
        extra["success_criteria"] = P.SUCCESS_CRITERIA
    else:
        from simple.cli.eval_decoupled_wbc import _make_sonic_config

        extra["sonic_config"] = _make_sonic_config()
    env = gym.make(spec.env_id, sim_mode=sim_mode, render_hz=P.RENDER_HZ, headless=True, **extra)
    if spec.fresh_layout_robot_x_range_m is not None:
        set_robot_x_range(env.unwrapped.task, spec.fresh_layout_robot_x_range_m)
    if spec.fresh_layout_table_height_range_m is not None:
        set_table_height_range(env.unwrapped.task, spec.fresh_layout_table_height_range_m)
    return env


def set_robot_x_range(task, x_range) -> None:
    """Fresh layouts of ``task`` (a SIMPLE task) draw the robot's x from ``x_range`` (protocol.Task); the robot
    region's other coordinates stay as configured.  The ends keep the config's order (SIMPLE draws
    uniform(low, high)), so a seed draws the same fraction of the new range as of the old one.  The spatial
    randomiser and the task get their own copies of the config (the task class's config objects are shared by
    its instances); the task's copy is what its state dict records as the DR config."""
    from simple.dr.types import Box

    old = task.dr.randomizers["spatial"].cfg
    region = old.robot_region
    low_x, high_x = sorted(x_range, reverse=region.low[0] > region.high[0])
    new = copy.copy(old)
    new.robot_region = Box(low=[low_x, *region.low[1:]], high=[high_x, *region.high[1:]])
    task.dr.randomizers["spatial"].cfg = new
    task.dr_cfgs = {**task.dr_cfgs, "spatial": new}


def set_table_height_range(task, height_range) -> None:
    """Fresh layouts of ``task`` (a SIMPLE task) draw the table top's height from ``height_range``
    (protocol.Task), as set_robot_x_range does the robot's x: ends in the config's order (SIMPLE draws
    uniform(low, high)), own copies of the scene randomiser's config for the randomiser and the task."""
    from simple.dr.types import Box

    old = task.dr.randomizers["scene"].cfg
    box = old.table_height
    low, high = sorted(height_range, reverse=box.low > box.high)
    new = copy.copy(old)
    new.table_height = Box(low=low, high=high)
    task.dr.randomizers["scene"].cfg = new
    task.dr_cfgs = {**task.dr_cfgs, "scene": new}


class SimpleSim:
    """One task's environment; ``reset(seed)`` starts an episode on a fresh layout.  ``renderer``: who draws
    the pictures (protocol.RENDERERS); Isaac Sim starts with the first environment of the process."""

    def __init__(self, task: str, renderer: str = "isaac", isaac_frames: int = P.ISAAC_FRAMES_PER_PICTURE):
        self.spec = P.check_task(task)
        if renderer not in P.RENDERERS:
            raise ValueError(f"renderer {renderer!r}: one of {', '.join(P.RENDERERS)}")
        self.task_name = task
        self.picture_renderer = renderer  # (self.renderer is MuJoCo's Renderer of the gaze camera)
        self.isaac_frames = int(isaac_frames)
        self.env = make(task, sim_mode="mujoco_isaac" if renderer == "isaac" else "mujoco")
        self.base = self.env.unwrapped
        self.task = self.base.task
        self.robot = self.task.robot
        self.isaac = self.base.isaac  # None with MuJoCo's renderer
        #: teleoperation tasks: GR00T's decoupled whole-body controller on simulated time (wbc.py); its
        #: commands are SIMPLE's ActionCmd "decoupled_wbc" (``self.wbc.action()``)
        self.wbc = None
        if self.spec.controller == "decoupled_wbc":
            from .wbc import SimClockWbc

            self.wbc = SimClockWbc(self.robot, self.base.sonic_config)
        self.stand_steps: list[int] = []
        self._annotators: dict = {}
        self._add_gaze_camera()
        self.steps = 0
        self.first_success: int | None = None
        self.drawn = False
        self.drawn_layout: dict | None = None
        self.saved_layout: dict | None = None
        self.gaze_deg = np.array(P.GAZE_START_DEG, dtype=float)  # (yaw, pitch)
        self._isaac_gaze_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._last_camera: dict | None = None
        self._last_isaac_segmentation = None
        self.facts: dict = {}

    def _add_gaze_camera(self) -> None:
        from simple.sensors.config import CameraCfg

        cfgs = dict(self.task.sensor_cfgs)
        cfgs[P.GAZE_CAMERA] = CameraCfg(
            uid="hitch_gaze", mount="eye_in_head", width=P.GAZE_WIDTH, height=P.GAZE_HEIGHT,
            fov=math.radians(P.GAZE_HFOV_DEG), near=P.GAZE_NEAR_M, far=P.GAZE_FAR_M, focal_length=1.93,
            pose=dict(position=[0.0, 0.0, 0.0]))
        self.task.sensor_cfgs = cfgs  # an instance attribute: the task class's own dict is left alone

    # -- episode ---------------------------------------------------------------------------------
    def reset(self, seed: int, official_test: bool = False) -> dict:
        """Start an episode: a fresh layout for a learning or validation seed (any other seed than a test seed
        draws one too), the official evaluation episode for a test seed (only with ``official_test``:
        protocol.check_seed).

        A fresh layout is made with ``seed`` the way the official episodes were made (protocol, Episodes):
        drawn from Python's and NumPy's global generators (SIMPLE's domain randomisation draws from them,
        not from reset's seed), stood on, saved with the robot where that stand left it, restored as the
        official evaluation restores an episode, and stood on again.

        ``drawn_layout`` and ``saved_layout`` keep the two state dicts (the second is the counterpart of
        an official episode's environment_config).  The first stand takes no pictures, so Isaac Sim sits
        it out (it never moves the physics) and builds its scene only for the restored layout.  The stand
        is the task's official one (``_stand``); ``stand_steps`` keeps how many steps each took."""
        seed = int(seed)
        if P.split_of(seed) == "test":
            P.check_seed(seed, official_test=official_test)
            return self._reset_official(seed)
        random.seed(seed)
        np.random.seed(seed)
        uid = str(self.robot.uid)
        isaac, self.base.isaac = self.base.isaac, None
        self.stand_steps = []
        try:
            # the unwrapped environment: gymnasium's checker would compare the observation with the space it
            # took at make time, which lacks the gaze camera (nothing else in the wrappers touches the physics)
            self.base.reset(seed=seed)
            self._ids()
            drawn = self._state_dict()
            self.stand_steps.append(self._stand())
            saved = copy.deepcopy(drawn)
            saved["dr_state_dict"]["spatial"][uid] = self._robot_pose_on_ground(
                z=drawn["dr_state_dict"]["spatial"][uid]["position"][2])
        finally:
            self.base.isaac = isaac
        self._restore_and_stand(saved)
        self.drawn_layout, self.saved_layout = drawn, saved
        self.facts = {"task": self.task_name, "seed": seed, "instruction": str(self.task.instruction),
                      "official_steps": self.spec.official_steps, "relaxed_steps": P.RELAXED_STEPS,
                      "step_s": P.STEP_S, "controller": self.spec.controller,
                      "stand_steps": list(self.stand_steps),
                      "robot_drawn_pose": drawn["dr_state_dict"]["spatial"][uid],
                      "robot_saved_pose": saved["dr_state_dict"]["spatial"][uid]}
        return self.facts

    def _reset_official(self, seed: int) -> dict:
        """A test seed's official evaluation episode, started as SIMPLE's evaluations start it: its saved
        environment_config from meta/episodes.jsonl (protocol.official_environment_config: the evaluation data
        under $SIMPLE_EVAL_DIR) given to the environment's reset as the state dict, then the stand.  Nothing is
        drawn: lighting, materials, objects and the robot's position all come from the saved configuration.
        The global generators are seeded with the test seed first (SIMPLE's evaluation leaves them as they
        are), so the episode starts the same way each time."""
        data_dir = os.environ.get(P.OFFICIAL_DATA_VARIABLE, "").strip()
        if not data_dir:
            raise ValueError(f"seed {seed}: an official evaluation episode needs ${P.OFFICIAL_DATA_VARIABLE} (the "
                             "folder SIMPLE's evaluation data unpack into) in the server's environment")
        conf, entry = P.official_environment_config(self.task_name, seed, data_dir)
        random.seed(seed)
        np.random.seed(seed)
        self.stand_steps = []
        self._restore_and_stand(conf)
        self.drawn_layout, self.saved_layout = None, conf
        uid = str(self.robot.uid)
        level, index = P.official_episode(seed)
        self.facts = {"task": self.task_name, "seed": seed, "instruction": str(self.task.instruction),
                      "official_steps": self.spec.official_steps, "relaxed_steps": P.RELAXED_STEPS,
                      "step_s": P.STEP_S, "controller": self.spec.controller,
                      "stand_steps": list(self.stand_steps),
                      "official_episode": {"level": level, "episode_index": index,
                                           "instruction": (entry.get("tasks") or [None])[0]},
                      "robot_saved_pose": conf["dr_state_dict"]["spatial"].get(uid)}
        return self.facts

    def _restore_and_stand(self, saved: dict) -> None:
        """Restore a saved layout as SIMPLE's evaluations restore an episode, stand, and start the count."""
        # no DR level, as the official evaluation: every randomiser's state is loaded.  SIMPLE keeps the dict
        # it is given as the randomisers' state and writes into it, so it gets a copy.
        self.base.reset(options={"state_dict": copy.deepcopy(saved)})
        self._ids()
        if self.isaac is not None:
            self._isaac_annotators()
        self._last_camera = None
        self._last_isaac_segmentation = None
        self.stand_steps.append(self._stand())
        self.base.step_count = 0
        self.base._success = False
        self.steps = 0
        self.first_success = None
        self.drawn = False
        self.set_gaze(*P.GAZE_START_DEG)

    def _state_dict(self) -> dict:
        """The task's state dict as SIMPLE's episode data holds it (JSON types; a copy)."""
        from simple.utils import NumpyArrayEncoder

        return json.loads(json.dumps(self.task.state_dict(), cls=NumpyArrayEncoder))

    def _stand(self) -> int:
        """The official evaluation's stand before an episode (the caller resets the count); returns its steps.

        MP tasks: SIMPLE's StandStabilizationWrapper, STAND_STEPS stand commands.  Teleoperation tasks: the
        stabilisation loop of SIMPLE's eval_decoupled_wbc.py: the controller is reset and its RL policy engaged,
        then its stabilisation goal is given until the robot reports that it stands still (G1Sonic.stabilized,
        read once before each step as there) or for at most STABILIZE_MAX_STEPS steps; then the gait phase
        restarts.  Here on simulated time, without the loop's sleeps."""
        if self.wbc is None:
            from simple.core.action import ActionCmd

            stand = ActionCmd("loco_command", command=[0, 0, 0, 0, 0, 0, 0, 0], motion_type="stand",
                              keep_waist_pose=False)
            for _ in range(P.STAND_STEPS):
                self._physics_step(stand)
            return P.STAND_STEPS
        from .wbc import STABILIZE_MAX_STEPS

        w = self.wbc
        w.reset()
        w.engage()
        n = 0
        while not self.robot.stabilized and n < STABILIZE_MAX_STEPS:
            self._physics_step(w.stabilize_action())
            n += 1
        w.start_policy()
        return n

    def _robot_pose_on_ground(self, z: float) -> dict:
        """The robot's pose in the form of a layout's: the pelvis's x and y, height ``z``, and a quaternion
        (w, x, y, z) of the pelvis's heading alone."""
        p = self.data.xpos[self.pelvis_id]
        h = heading(self.data.xmat[self.pelvis_id].reshape(3, 3))
        return {"position": [float(p[0]), float(p[1]), float(z)],
                "quaternion": [math.cos(h / 2.0), 0.0, 0.0, math.sin(h / 2.0)]}

    def _ids(self) -> None:
        import mujoco

        self.model = m = self.base.mujoco.mjModel
        self.data = self.base.mujoco.mjData
        self.pelvis_id = m.body(PELVIS).id
        self.torso_id = m.body(TORSO).id
        self.wrist_id = {s: m.body(WRIST[s]).id for s in P.SIDES}
        root = int(m.body_rootid[self.pelvis_id])
        self.robot_geoms = np.array([g for g in range(m.ngeom) if int(m.body_rootid[m.geom_bodyid[g]]) == root],
                                    dtype=np.int64)
        planes = [float(m.geom_pos[g][2]) for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
        self.floor_z = max(planes) if planes else 0.0
        self.gaze_id = m.camera(P.GAZE_CAMERA).id
        self.names = list(self.robot.joint_names)  # WHOLE_BODY_JOINTS order (43)
        self.qadr = {j: int(m.jnt_qposadr[m.joint(j).id]) for j in self.names}
        self.dadr = {j: int(m.jnt_dofadr[m.joint(j).id]) for j in self.names}
        self.limits = {j: tuple(float(v) for v in m.joint(j).range) for j in self.names}
        self.renderer = self.base.mujoco.renderers[P.GAZE_CAMERA]

    def _isaac_annotators(self) -> None:
        """Depth and instance segmentation of Isaac Sim's gaze camera, attached once to its render product
        (which lives across resets).  They are read straight from the annotators: the camera's own frame
        is only refreshed at its frequency of simulated time, and Isaac's time moves only when it draws."""
        import omni.replicator.core as rep

        self.icam = self.isaac.cameras[P.GAZE_CAMERA]
        path = self.icam.get_render_product_path()
        if self._annotators.get("path") != path:
            self._annotators = {"path": path}
            for name in ("distance_to_image_plane", "instance_id_segmentation"):
                a = rep.AnnotatorRegistry.get_annotator(name)
                a.attach([path])
                self._annotators[name] = a
        self.isaac_robot_prefix = f"{self.isaac.workspace_prim_path}/Robot/"

    def _physics_step(self, cmd) -> bool:
        """What the environment's step does to the physics and the success check, without pictures:
        LocoManipulationEnv.step (MP tasks: the command once, one MuJoCo step, which runs the AMO robot's
        substeps), SonicLocoManipEnv.step (teleoperation tasks: the command and one 0.005 s MuJoCo step,
        control_decimal = 4 times; Isaac Sim is not copied into at every substep, only for pictures)."""
        b = self.base
        for _ in range(1 if self.wbc is None else int(b.control_decimal)):
            b.mujoco.apply_action(cmd)
            b.mujoco.step(render=False)
        b.step_count += 1
        info = b._get_info()
        b.task.compute_reward(info, mujoco_env=b.mujoco)
        b._success = bool(b.task.check_success(info, mujoco_env=b.mujoco))
        return b._success

    @property
    def over(self) -> bool:
        return self.first_success is not None or self.steps >= P.RELAXED_STEPS

    def step(self, cmd) -> bool:
        """One 0.02 s control step with SIMPLE's ActionCmd; False once the episode is over."""
        if self.over:
            return False
        ok = self._physics_step(cmd)
        self.steps = int(self.base.step_count)
        if ok and self.first_success is None:
            self.first_success = self.steps
        return not self.over

    def status(self) -> dict:
        first = self.first_success
        return {"step": self.steps, "seconds": round(self.steps * P.STEP_S, 2), "horizon": P.RELAXED_STEPS,
                "official_steps": self.spec.official_steps, "success": first is not None,
                "first_success_step": first, "official_success": P.official_success(first, self.task_name),
                "over": self.over}

    # -- the robot (world frame) -----------------------------------------------------------------
    def body_pose(self, body_id: int) -> np.ndarray:
        return pose(self.data.xmat[body_id].reshape(3, 3), self.data.xpos[body_id])

    def frame(self) -> np.ndarray:
        """world <- O: HITCH's observation frame now (on the ground under the pelvis, x its heading)."""
        p = self.data.xpos[self.pelvis_id]
        R = self.data.xmat[self.pelvis_id].reshape(3, 3)
        return pose(rot_z(heading(R)), (p[0], p[1], self.floor_z))

    def pelvis_height(self) -> float:
        return float(self.data.xpos[self.pelvis_id][2] - self.floor_z)

    def torso(self) -> dict:
        """Torso pitch (forward lean below horizontal) and yaw relative to the pelvis's heading (rad)."""
        Rt = self.data.xmat[self.torso_id].reshape(3, 3)
        Rp = self.data.xmat[self.pelvis_id].reshape(3, 3)
        yaw = (heading(Rt) - heading(Rp) + math.pi) % (2 * math.pi) - math.pi
        return {"pitch": pitch_down(Rt), "yaw": yaw}

    def palm(self, side: str) -> dict:
        """Palm point, normal and finger direction (world)."""
        i = self.wrist_id[side]
        R = self.data.xmat[i].reshape(3, 3)
        return {"point": self.data.xpos[i] + R @ PALM_POINT[side], "normal": R @ PALM_NORMAL[side],
                "finger": R @ FINGER_AXIS}

    def joints(self, names) -> np.ndarray:
        return np.array([self.data.qpos[self.qadr[j]] for j in names])

    def joint_speeds(self, names) -> np.ndarray:
        return np.array([self.data.qvel[self.dadr[j]] for j in names])

    def proprio(self) -> dict:
        """The robot's own state, world frame."""
        Rp = self.data.xmat[self.pelvis_id].reshape(3, 3)
        return {"pelvis": self.data.xpos[self.pelvis_id].copy(), "heading": heading(Rp),
                "pelvis_height": self.pelvis_height(), "torso": self.torso(),
                "palms": {s: self.palm(s) for s in P.SIDES}, "gaze_deg": self.gaze_deg.copy(),
                "joints": dict(zip(self.names, self.joints(self.names)))}

    # -- gaze camera -----------------------------------------------------------------------------
    def set_gaze(self, yaw_deg: float, pitch_deg: float) -> None:
        """Point the gaze camera (degrees: yaw left of the torso's forward, pitch below its horizontal)."""
        import mujoco

        lo, hi = P.GAZE_YAW_RANGE_DEG
        yaw = min(hi, max(lo, float(yaw_deg)))
        lo, hi = P.GAZE_PITCH_RANGE_DEG
        pitch = min(hi, max(lo, float(pitch_deg)))
        R = rot_z(math.radians(yaw)) @ rot_y(math.radians(pitch))  # in torso_link, x forward, z up
        self.model.cam_quat[self.gaze_id] = mat2quat(R @ ISAAC_TO_MUJOCO_CAMERA)
        self.model.cam_pos[self.gaze_id] = HEAD_CAMERA_POSITION
        mujoco.mj_camlight(self.model, self.data)
        # Isaac Sim: the same direction in d435_link (Isaac's camera axes: x forward, z up)
        self._isaac_gaze_quat = mat2quat(quat2mat(HEAD_CAMERA_ORIENTATION).T @ R)
        self.gaze_deg = np.array([yaw, pitch])

    def _mujoco_camera(self) -> dict:
        i = self.gaze_id
        fovy = math.radians(float(self.model.cam_fovy[i]))
        return {"pose": pose(self.data.cam_xmat[i].reshape(3, 3), self.data.cam_xpos[i]),
                "focal": 0.5 * P.GAZE_HEIGHT / math.tan(0.5 * fovy), "width": P.GAZE_WIDTH,
                "height": P.GAZE_HEIGHT}

    def camera(self) -> dict:
        """The gaze camera of the last picture as a pinhole (world frame; looks along -z, +y up the
        picture); with MuJoCo's renderer and no picture yet, where it is now."""
        if self._last_camera is not None:
            return self._last_camera
        if self.isaac is not None:
            raise RuntimeError("no picture taken yet in this episode")
        return self._mujoco_camera()

    def picture(self, robot_mask: bool = True) -> tuple:
        """RGB, depth (m along the optical axis, 0 = none) and the robot's own pixels of the gaze camera."""
        t0 = time.time()
        if self.isaac is not None:
            rgb, depth, mask, cam = self._isaac_picture(robot_mask)
        else:
            rgb, depth, mask, cam = self._mujoco_picture(robot_mask)
        check_time(time.time() - t0, P.GAZE_CAMERA, first=not self.drawn)
        self.drawn = True
        self._last_camera = cam
        return rgb, far_clip(check_depth(depth, P.GAZE_CAMERA)), mask

    def _mujoco_picture(self, robot_mask: bool) -> tuple:
        import mujoco

        mujoco.mj_camlight(self.model, self.data)
        r = self.renderer
        r.update_scene(self.data, camera=P.GAZE_CAMERA)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        depth = r.render().copy()
        r.disable_depth_rendering()
        mask = None
        if robot_mask:
            seg = segment(r)
            mask = (seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(seg[..., 0], self.robot_geoms)
        return rgb, depth.astype(np.float32), mask, self._mujoco_camera()

    def _isaac_picture(self, robot_mask: bool) -> tuple:
        """Copy MuJoCo's state into Isaac Sim and draw (SIMPLE's IsaacSimSimulator.step: objects, joints
        and the robot's base; one physics step of Isaac's, which only moves what it draws)."""
        cam = self.icam
        cam.set_local_pose(translation=np.zeros(3), orientation=self._isaac_gaze_quat, camera_axes="world")
        for _ in range(max(1, self.isaac_frames)):
            self.isaac.step(self.base.mujoco)
        h, w = P.GAZE_HEIGHT, P.GAZE_WIDTH
        rgba = np.asarray(cam.get_rgba())
        if rgba.shape[:2] != (h, w):
            raise RuntimeError(f"Isaac Sim gave a picture of shape {rgba.shape} for the gaze camera")
        rgb = rgba[..., :3].astype(np.uint8).copy()
        depth = np.array(self._annotators["distance_to_image_plane"].get_data(), dtype=np.float32).reshape(h, w)
        depth[np.isposinf(depth)] = 0.0  # nothing hit (NaN, which a GPU that draws wrongly gives, stays)
        seg =self._annotators["instance_id_segmentation"].get_data()
        ids = np.asarray(seg["data"]).reshape(h, w)
        labels = {int(k): str(v) for k, v in seg["info"]["idToLabels"].items()}
        self._last_isaac_segmentation = (ids, labels)
        mask = None
        if robot_mask:
            robot_ids = [k for k, v in labels.items() if v.startswith(self.isaac_robot_prefix)]
            mask = np.isin(ids, robot_ids)
        p, q = cam.get_world_pose(camera_axes="usd")  # looks along -z, +y up, as MuJoCo's cameras
        K = np.asarray(cam.get_intrinsics_matrix(), dtype=float)
        if abs(K[0, 0] - K[1, 1]) > 1e-3 * K[0, 0]:
            raise RuntimeError(f"Isaac Sim's gaze camera has fx {K[0, 0]:.3f} != fy {K[1, 1]:.3f}")
        camera = {"pose": pose(quat2mat(q), np.asarray(p, dtype=float)), "focal": float(K[0, 0]),
                  "width": w, "height": h}
        return rgb, depth, mask, camera

    def object_pixels(self, actor: str) -> np.ndarray:
        """Pixels of a layout actor (e.g. "target") in the last picture: ground truth, for scripted tests
        only (a model picks pixels itself)."""
        if self.isaac is not None:
            if self._last_isaac_segmentation is None:
                raise RuntimeError("no picture taken yet in this episode")
            ids, labels = self._last_isaac_segmentation
            label = self.task.layout.actors[actor].asset.label
            prefix = self.isaac.objects[label]["object_prim_path"] + "/"
            return np.isin(ids, [k for k, v in labels.items() if v.startswith(prefix)])
        import mujoco

        r = self.renderer
        mujoco.mj_camlight(self.model, self.data)
        r.update_scene(self.data, camera=P.GAZE_CAMERA)
        seg = segment(r)
        body = self.base.mujoco.mj_objects[actor].id
        geoms = [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == body]
        return (seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(seg[..., 0], geoms)

    def observe(self, robot_mask: bool = True) -> dict:
        rgb, depth, mask = self.picture(robot_mask)
        cam = P.GAZE_CAMERA
        return {"step": self.steps, "images": {cam: rgb}, "depth": {cam: depth},
                "robot": {cam: mask} if mask is not None else {}, "cameras": {cam: self.camera()},
                "frame": self.frame(), "proprio": self.proprio()}

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:  # noqa: BLE001
            pass


def pixel_to_world(camera: dict, depth: np.ndarray, u: float, v: float) -> np.ndarray | None:
    """World point seen at pixel (u right, v down; pixel centres at +0.5) with that pixel's depth."""
    h, w = depth.shape
    col, row = min(w - 1, max(0, int(u))), min(h - 1, max(0, int(v)))
    z = float(depth[row, col])
    if not np.isfinite(z) or z <= 0:
        return None
    f = camera["focal"]
    x = (u - 0.5 * w) * z / f
    y = (v - 0.5 * h) * z / f
    T = camera["pose"]
    return T[:3, 3] + T[:3, :3] @ np.array([x, -y, -z])
