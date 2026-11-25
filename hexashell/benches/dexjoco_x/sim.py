"""DexJoCo-X's environments in the server process: build, reset, step, read, take pictures.

Runs in DexJoCo-X's Python environment (bench-dexjoco-x: Python 3.11, MuJoCo 3.4.0) with the release's code
under ``$DEXJOCO_X_ROOT``.  The episode is the official evaluation's: its ``Runtime`` (evaluation/runtime.py,
loaded from the release unchanged) builds the task's environment with the Wuji hand for the scene seed
(``make_env``, reset with that seed), checks the evaluation contract (actuators, joints, cameras, the hand's
mount) and steps it (``Runtime.step``: the action clipped to the contract's bounds, then the environment's own
step).  Every opening builds a new Runtime, as the official evaluation does for every episode.  The episode
ends as the official rollout ends it: at the first step whose info reports success, when the environment
terminates or truncates it, or at the contract's step limit.  Nothing here is seen by the model directly.

Readings and pictures are taken on the runtime's ``shadow`` copy of the state (forward kinematics only, as
the official observation does), which leaves the physics' caches as the official run has them.  The wrist
command's base is the environment's own (``tcp``: the attachment site on the live data, which its step
reads).  What a hand feels is the push from outside it on its joints (``outside_push``), from what the joints
do and the hand's own model, not from the physics' contacts (``contacts`` and ``holding`` are for the records).

Pictures: our own MuJoCo renderer of the official cameras (the profile's front camera and the wrist
cameras), square at protocol.IMAGE_SIZE (the official images are 256 square: the same cameras and fields of
view, more pixels), without multisampling (``offsamples`` 0: with it the same state can draw a few pixels
differently).  Depth is along the optical axis (m), 0 where nothing is within FAR_M.  The robot's own pixels
(the arms and hands) come from the renderer's segmentation.

Frame: MuJoCo's world (the scene frame): z up, the robot faces +x (the arms' bases stand at x -0.8, at
-1.05 in the microwave task), y to its left.

Hands: each hand's attachment frame (its ``attachment_site``) has x along the fingers and z out of the palm
(the palm faces +z); the thumb is on its -y side on the right hand, +y on the left.  The palm point is the
centre of the palm's face (PALM_POINT); a fingertip is the middle of its pad (TIP_PAD).  The hand's 20
joints are four per finger, thumb first (protocol.FINGERS): on a finger J1 bends it at the knuckle (toward
the palm as it goes negative), J2 turns it sideways (positive: toward the thumb's side), J3 and J4 curl it;
on the thumb J1 swings it across the palm, J2 lifts it toward the fingers, J3 and J4 curl it.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from ..gpucheck import check_depth, check_time
from . import protocol as P

#: the layout of the official 80-number action (the official evaluation's default)
LAYOUT = "grouped80_v1"
#: no depth beyond this (m)
FAR_M = 10.0
#: the centre of the palm's face in the attachment frame (m): the palm mesh spans x 0.041..0.146,
#: y -0.038..0.043, its face at z 0.044
PALM_POINT = np.array([0.095, 0.0, 0.045])
#: the tip bodies of the fingers (after "benchhand_<side>_native_wuji_")
TIP_BODIES = {"thumb": "IP_thumb", "index": "tipmount_index", "middle": "tipmount_middle", "ring": "tipmount_ring",
              "little": "tipmount_pinky"}
#: the middle of each fingertip's pad in its tip body's frame (m), right hand; the left hand's is mirrored (x and
#: y negated)
TIP_PAD = {"thumb": np.array([-0.0276, 0.0026, 0.0001]), "finger": np.array([-0.0249, 0.0028, -0.0001])}
#: a hand's parts, by words in its body names: the thumb's links, each finger's, the rest is the palm
PART_WORDS = (("thumb", ("thumb", "CMC")), ("index", ("index",)), ("middle", ("middle",)), ("ring", ("ring",)),
              ("little", ("pinky",)))
PARTS = ("thumb", "index", "middle", "ring", "little", "palm")
#: a contact pushing at least this hard (N) is a touch
TOUCH_N = 0.02
#: info keys whose values the records keep (small numbers and words of the environment's own checks)
INFO_KEEP = 40


def release_root() -> Path:
    root = os.environ.get("DEXJOCO_X_ROOT", "").strip()
    if not root:
        raise RuntimeError("DEXJOCO_X_ROOT is not set: the DexJoCo-X release's folder (its evaluation/runtime.py); "
                           "config/env.example.sh says where it is set")
    path = Path(root)
    if not (path / "evaluation" / "runtime.py").is_file():
        raise RuntimeError(f"DEXJOCO_X_ROOT={root}: no evaluation/runtime.py there")
    return path


_RUNTIME = None


def official():
    """The release's evaluation/runtime.py as a module (loaded once per process: it puts the release's simulator
    package on sys.path and sets the official camera profile)."""
    global _RUNTIME
    if _RUNTIME is None:
        path = release_root() / "evaluation" / "runtime.py"
        spec = importlib.util.spec_from_file_location("dexjoco_x_official_runtime", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        _RUNTIME = module
    return _RUNTIME


def segment(r) -> np.ndarray:
    """A MuJoCo Renderer's segmentation (object id, object type per pixel) for the scene it last updated.
    Pixels whose ID colour matches no geom of the scene read as background (-1, -1); MuJoCo's own
    Renderer.render raises IndexError on them and leaves the scene in ID-colour mode.  (As SIMPLE's.)"""
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


def pose(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def plain(value):
    """An info value as plain Python (numbers, words, small lists), or None."""
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return round(float(value), 5) if math.isfinite(float(value)) else None
    if isinstance(value, str):
        return value[:200]
    if isinstance(value, (list, tuple, np.ndarray)) and len(value) <= 8:
        out = [plain(v) for v in value]
        return out if all(v is not None for v in out) else None
    if isinstance(value, dict):
        out = {str(k): plain(v) for k, v in list(value.items())[:INFO_KEEP]}
        return {k: v for k, v in out.items() if v is not None}
    return None


def failure_of(task: str, info: dict) -> str | None:
    """The environment's own words for why it ended an episode without success, if it gives them."""
    if task == "single_hand_hanoi":
        reason = info.get("illegal_reason")
        return f"the tower is in an illegal state ({reason})" if reason else None
    if task == "bimanual_unlock_ipad":
        metrics = info.get("ipad_task_metrics") or {}
        reason = metrics.get("failure") if isinstance(metrics, dict) else None
        return f"the tablet task failed ({reason})" if reason else None
    return None


class DexSim:
    """One task's environment; ``reset(seed)`` starts an episode (a new official Runtime)."""

    def __init__(self, task: str, size: int = P.IMAGE_SIZE):
        self.spec = P.check_task(task)
        self.task = task
        self.size = int(size)
        self.rt = None
        self.renderer = None
        self.steps = 0
        self.success = False
        self.terminated = False
        self.truncated = False
        self.failure: str | None = None
        self.info: dict = {}
        self.clipped = 0
        self.drawn = False
        self._fresh = False
        self._self_models: dict = {}

    # -- the episode -----------------------------------------------------------------------------
    def reset(self, seed: int) -> dict:
        """A new episode on the scene ``seed`` (the caller checked that the run may open it)."""
        self.close()
        runtime = official()
        rt = runtime.Runtime(self.task, P.HAND, int(seed), LAYOUT, representation="native", render=False)
        self.rt, self.e, self.m, self.d, self.shadow = rt, rt.e, rt.m, rt.d, rt.shadow
        self.seed = int(seed)
        self.limit = int(self.e.contract["max_episode_steps"])
        bounds = (float(self.e._translation_bound), float(self.e._rotation_bound))
        if self.limit != self.spec.steps or bounds != (self.spec.translation_m, self.spec.rotation_rad):
            raise RuntimeError(f"{self.task}: the release's contract (limit {self.limit}, bounds {bounds}) is not "
                               f"protocol.TASKS' ({self.spec}): the release changed")
        self.translation_bound, self.rotation_bound = bounds
        self.sides = tuple(s for s in P.SIDES if s in self.e._native_ctrl_ids)
        if set(self.sides) != set(self.spec.arms):
            raise RuntimeError(f"{self.task}: arms {self.sides}, protocol says {self.spec.arms}")
        self.steps = 0
        self.success = self.terminated = self.truncated = False
        self.failure = None
        self.info = {}
        self.clipped = 0
        self.drawn = False
        self._ids()
        self._renderer()
        self._fresh = False
        self._self_models = {}
        self.sync()
        return self.facts()

    def facts(self) -> dict:
        c = self.e.contract
        return {"task": self.task, "seed": self.seed, "hand": P.HAND,
                "instruction": str(self.rt.contract["instruction"]),
                "steps": self.limit, "arms": list(self.sides), "translation_bound_m": self.translation_bound,
                "rotation_bound_rad": self.rotation_bound, "cameras": {cam: self.cam_names[cam] for cam in self.cams},
                "model_binding": dict(self.rt.model_binding), "contract_steps": int(c["max_episode_steps"])}

    def _ids(self) -> None:
        m, e = self.m, self.e
        self.tcp_site = {s: int(e._tcp_sites[s]) for s in self.sides}
        self.ctrl_ids = {s: np.asarray(e._native_ctrl_ids[s], dtype=int) for s in self.sides}
        self.hand_qpos = {s: np.asarray([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in self.ctrl_ids[s]], dtype=int)
                          for s in self.sides}
        self.hand_dof = {s: np.asarray([m.jnt_dofadr[m.actuator_trnid[a, 0]] for a in self.ctrl_ids[s]], dtype=int)
                         for s in self.sides}
        self.hand_range = {s: np.asarray(m.actuator_ctrlrange[self.ctrl_ids[s]], float) for s in self.sides}
        self.hand_home = {s: np.asarray(e.hand_metadata["sides"][s]["home"], float) for s in self.sides}
        # the arm: the seven hinges from the hand's attachment body up to the base
        self.arm_joints, self.arm_qpos, self.arm_dof, self.arm_range, self.arm_root = {}, {}, {}, {}, {}
        for s in self.sides:
            b = int(m.site_bodyid[self.tcp_site[s]])
            joints = []
            while b > 0:
                for j in range(int(m.body_jntadr[b]), int(m.body_jntadr[b]) + int(m.body_jntnum[b])):
                    if int(m.jnt_type[j]) == 3:  # hinge
                        joints.append(j)
                root = b
                b = int(m.body_parentid[b])
            joints = joints[::-1]
            if len(joints) != 7:
                raise RuntimeError(f"{self.task}: the {s} arm has {len(joints)} hinges up from its hand, not 7")
            self.arm_joints[s] = np.asarray(joints, dtype=int)
            self.arm_qpos[s] = np.asarray([m.jnt_qposadr[j] for j in joints], dtype=int)
            self.arm_dof[s] = np.asarray([m.jnt_dofadr[j] for j in joints], dtype=int)
            self.arm_range[s] = np.asarray([m.jnt_range[j] for j in joints], float)
            self.arm_root[s] = root
        # the robot's bodies: each arm's subtree from its root; the hand's parts under its attachment body
        self.body_part: dict[int, tuple[str, str]] = {}
        robot_bodies = set()
        for s in self.sides:
            hand_root = int(m.site_bodyid[self.tcp_site[s]])
            for b in range(m.nbody):
                if self._under(b, self.arm_root[s]):
                    robot_bodies.add(b)
                    if self._under(b, hand_root):
                        name = m.body(b).name
                        part = next((p for p, words in PART_WORDS if any(w in name for w in words)), "palm")
                        self.body_part[b] = (s, part)
                    else:
                        self.body_part[b] = (s, "arm")
        self.robot_bodies = robot_bodies
        self.robot_geoms = np.asarray([g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in robot_bodies], dtype=int)
        # the things that move freely (a free joint): what a hand can hold
        self.free_roots = {int(m.body_rootid[m.jnt_bodyid[j]]) for j in range(m.njnt) if int(m.jnt_type[j]) == 0
                           and int(m.jnt_bodyid[j]) not in robot_bodies}
        self.tip_body = {}
        for s in self.sides:
            for f, name in TIP_BODIES.items():
                self.tip_body[(s, f)] = int(m.body(f"benchhand_{s}_native_wuji_{name}").id)
        # cameras: ours -> the contract's
        self.cams = P.cameras(self.task)
        self.cam_names, self.cam_ids = {}, {}
        for cam in self.cams:
            role = P.CAMERA_ROLES[cam]
            if role not in self.rt.cameras:
                raise RuntimeError(f"{self.task}: the contract has no {role} camera ({sorted(self.rt.cameras)})")
            name = self.rt.cameras[role]["camera_name"]
            self.cam_names[cam] = name
            self.cam_ids[cam] = int(m.camera(name).id)

    def _under(self, b: int, root: int) -> bool:
        while b > 0:
            if b == root:
                return True
            b = int(self.m.body_parentid[b])
        return root == 0

    def _renderer(self) -> None:
        import mujoco

        m = self.m
        m.vis.quality.offsamples = 0
        m.vis.global_.offwidth = max(int(m.vis.global_.offwidth), self.size)
        m.vis.global_.offheight = max(int(m.vis.global_.offheight), self.size)
        self.renderer = mujoco.Renderer(m, height=self.size, width=self.size)

    @property
    def over(self) -> bool:
        return self.success or self.terminated or self.truncated or self.steps >= self.limit

    def step(self, commands: dict) -> bool:
        """One control step: per arm of the task 6 + 20 numbers (the wrist's change: translation, rotation
        vector; the hand's 20 joint targets).  False once the episode is over."""
        if self.over:
            return False
        action = self.rt.encode_action({s: np.asarray(commands[s], float) for s in self.sides})
        success, terminated, truncated, clipped, info = self.rt.step(action)
        self.steps += 1
        self.clipped += int(clipped)
        self.info = info
        self.success = bool(success)
        self.terminated = bool(terminated)
        self.truncated = bool(truncated)
        if (terminated or truncated) and not success and self.failure is None:
            self.failure = failure_of(self.task, info) or ("the environment ended the episode" if terminated else None)
        self._fresh = False
        return not self.over

    def status(self) -> dict:
        if self.rt is None:
            return {}
        return {"step": self.steps, "seconds": round(self.steps * P.STEP_S, 2), "horizon": self.limit,
                "success": bool(self.success), "over": bool(self.over), "terminated": bool(self.terminated),
                "truncated": bool(self.truncated), "failure": self.failure}

    def info_row(self) -> dict:
        """The environment's own numbers of the last step (for the records)."""
        out = plain({k: v for k, v in (self.info or {}).items() if k != "succeed"}) or {}
        return out

    # -- readings (scene frame) ------------------------------------------------------------------
    def sync(self) -> None:
        """The shadow at the state now (forward kinematics only; the physics' data untouched)."""
        if self._fresh:
            return
        import mujoco

        for name in ("qpos", "qvel", "act", "ctrl", "mocap_pos", "mocap_quat", "qacc_warmstart"):
            getattr(self.shadow, name)[:] = getattr(self.d, name)
        self.shadow.time = self.d.time
        mujoco.mj_forward(self.m, self.shadow)
        self._fresh = True

    def tcp(self, side: str) -> np.ndarray:
        """The attachment site's pose now (shadow)."""
        self.sync()
        sid = self.tcp_site[side]
        return pose(self.shadow.site_xmat[sid].reshape(3, 3), self.shadow.site_xpos[sid])

    def base_tcp(self, side: str) -> np.ndarray:
        """The pose the environment's step adds the wrist change to (its own ``tcp``, the live data)."""
        return np.asarray(self.e.tcp(side), float)

    def palm(self, side: str) -> dict:
        T = self.tcp(side)
        return {"point": T[:3, :3] @ PALM_POINT + T[:3, 3], "normal": T[:3, 2].copy(), "finger": T[:3, 0].copy()}

    def tips(self, side: str) -> dict:
        self.sync()
        out = {}
        for f in P.FINGERS:
            b = self.tip_body[(side, f)]
            pad = TIP_PAD["thumb" if f == "thumb" else "finger"].copy()
            if side == "left":
                pad[:2] *= -1.0
            out[f] = self.shadow.xmat[b].reshape(3, 3) @ pad + self.shadow.xpos[b]
        return out

    def hand_q(self, side: str) -> np.ndarray:
        self.sync()
        return self.shadow.qpos[self.hand_qpos[side]].copy()

    def hand_targets(self, side: str) -> np.ndarray:
        return self.d.ctrl[self.ctrl_ids[side]].copy()

    def arm_q(self, side: str) -> np.ndarray:
        self.sync()
        return self.shadow.qpos[self.arm_qpos[side]].copy()

    def outside_push(self, side: str) -> np.ndarray:
        """Per joint of a hand (20, thumb first): the push from outside the hand on it now, as the joint motor's
        equivalent (rad: the torque over the motor's stiffness, how far the push holds the joint off where its
        motor alone would take it; signed).  From what the joints do and the hand's own model: the hand's
        self-model - a copy of the model in which only this hand's own parts collide - gives the accelerations
        the joints would have at the state now with nothing outside the hand; the difference to the ones they
        have, times the inertia, is the torque from outside.  The fingers of a closed hand meet each other and
        the palm (a thumb swung across the palm and curled stops on the index or the middle finger, fingers
        curled far stop on the palm): that is in the self-model too and pushes nothing.  The physics' data is
        untouched."""
        import copy

        import mujoco

        self.sync()
        if side not in self._self_models:
            m2 = copy.deepcopy(self.m)
            for g in range(m2.ngeom):
                part = self.body_part.get(int(m2.geom_bodyid[g]))
                if part is None or part[0] != side or part[1] == "arm":
                    m2.geom_contype[g] = 0
                    m2.geom_conaffinity[g] = 0
            self._self_models[side] = (m2, mujoco.MjData(m2), np.zeros(m2.nv))
        m2, d2, torque = self._self_models[side]
        now = self.shadow
        for name in ("qpos", "qvel", "act", "ctrl", "mocap_pos", "mocap_quat", "qacc_warmstart"):
            getattr(d2, name)[:] = getattr(now, name)
        d2.qfrc_applied[:] = 0.0
        d2.xfrc_applied[:] = 0.0
        d2.time = now.time
        mujoco.mj_forward(m2, d2)
        mujoco.mj_mulM(m2, d2, torque, now.qacc - d2.qacc)
        kp = self.m.actuator_gainprm[self.ctrl_ids[side], 0]
        return torque[self.hand_dof[side]] / kp

    def contacts(self) -> dict[str, dict[str, set]]:
        """Per hand, per part: the objects (root body ids) it pushes on at least TOUCH_N (not the robot)."""
        import mujoco

        self.sync()
        m, d = self.m, self.shadow
        out = {s: {p: set() for p in PARTS} for s in self.sides}
        force = np.zeros(6)
        for i in range(int(d.ncon)):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            for mine, other in ((b1, b2), (b2, b1)):
                hit = self.body_part.get(mine)
                if hit is None or hit[1] == "arm" or other in self.robot_bodies:
                    continue
                mujoco.mj_contactForce(m, d, i, force)
                if abs(float(force[0])) >= TOUCH_N:
                    out[hit[0]][hit[1]].add(int(m.body_rootid[other]))
        return out

    def touching(self) -> dict[str, list[str]]:
        c = self.contacts()
        return {s: [p for p in PARTS if c[s][p]] for s in self.sides}

    def holding(self, contacts: dict | None = None) -> dict[str, bool]:
        """Per hand: whether its thumb and one more part (a finger or the palm) push on the same thing that moves
        freely."""
        c = self.contacts() if contacts is None else contacts
        out = {}
        for s in self.sides:
            thumb = c[s]["thumb"] & self.free_roots
            others = set().union(*(c[s][p] for p in PARTS if p != "thumb"))
            out[s] = bool(thumb & others)
        return out

    def arm_touching(self) -> dict[str, bool]:
        """Per arm: whether a link of the arm (not the hand) touches something that is not the robot."""
        self.sync()
        m, d = self.m, self.shadow
        out = {s: False for s in self.sides}
        for i in range(int(d.ncon)):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            for mine, other in ((b1, b2), (b2, b1)):
                hit = self.body_part.get(mine)
                if hit is not None and hit[1] == "arm" and other not in self.robot_bodies and float(c.dist) < 0.0:
                    out[hit[0]] = True
        return out

    # -- pictures --------------------------------------------------------------------------------
    def camera(self, cam: str) -> dict:
        """The camera as a pinhole (scene frame; looks along -z, +y up the picture)."""
        self.sync()
        i = self.cam_ids[cam]
        fovy = math.radians(float(self.m.cam_fovy[i]))
        return {"pose": pose(self.shadow.cam_xmat[i].reshape(3, 3), self.shadow.cam_xpos[i]),
                "focal": 0.5 * self.size / math.tan(0.5 * fovy), "width": self.size, "height": self.size}

    def picture(self, cam: str, robot_mask: bool = True) -> tuple:
        """RGB, depth (m along the optical axis, 0 = none) and the robot's own pixels of one camera."""
        import mujoco

        self.sync()
        t0 = time.time()
        r = self.renderer
        r.update_scene(self.shadow, camera=self.cam_ids[cam])
        rgb = r.render().copy()
        r.enable_depth_rendering()
        depth = r.render().copy().astype(np.float32)
        r.disable_depth_rendering()
        mask = None
        if robot_mask:
            seg = segment(r)
            mask = (seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(seg[..., 0], self.robot_geoms)
        check_time(time.time() - t0, cam, first=not self.drawn)
        self.drawn = True
        depth[(depth > FAR_M) | ~np.isfinite(depth)] = 0.0
        return rgb, check_depth(depth, cam), mask

    def observe(self, cams=None, robot_mask: bool = True) -> dict:
        images, depths, robot, cameras = {}, {}, {}, {}
        for cam in cams or self.cams:
            rgb, depth, mask = self.picture(cam, robot_mask)
            images[cam], depths[cam], cameras[cam] = rgb, depth, self.camera(cam)
            if mask is not None:
                robot[cam] = mask
        return {"step": self.steps, "images": images, "depth": depths, "robot": robot, "cameras": cameras}

    def close(self) -> None:
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:  # noqa: BLE001
                pass
            self.renderer = None
        if self.rt is not None:
            try:
                self.rt.e.close()
            except Exception:  # noqa: BLE001
                pass
            self.rt = None
