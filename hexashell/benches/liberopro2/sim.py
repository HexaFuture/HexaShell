"""The LIBERO-Pro 2.0 environment in the server process: build, reset, step, read.

Runs in the LIBERO-Pro 2.0 Python environment (``bench-libero-pro2``: Python 3.8, robosuite 1.4, MuJoCo 3.2,
the evaluation bundle's ``libero`` and ``libero_remote`` installed from its sources).  Nothing here is seen
by the model directly; :mod:`.server` answers the episode side.

The environment is the official evaluation's (libero_remote/environment/libero.py): the OffScreenRenderEnv of
the instance's BDDL file from the task manifest (``$LIBERO_REMOTE_TASK_MANIFEST``), with its defaults (hard
reset, horizon 1000, OSC_POSE taking world-frame deltas), started the same way (:meth:`LiberoProSim.reset`);
the perturbations act inside its own step.  The cameras render 512 x 512 (the official client chooses 16 to
1024; its default is 256) every step, as the official observations are made; without depth, as the official
environment's (``depth``: with depth too, for the depth mode).

What the episode side gets comes from those observations alone, as a policy's would: the two cameras' pictures (and
depth), the arm's joint angles and speeds, the finger joints' positions and speeds.  The perturbations act on them
(a camera moved or its field of view widened, the joint readings offset, noise, delays ...).  Where the hand and the
wrist camera are is the forward kinematics of the joint readings on the scene's nominal model (:class:`Belief`: the
model as built, before any perturbation changed it), and the cameras' calibration is the nominal one.  The gripper's
level is the integral of the commands sent (:class:`CommandedGripper`), as robosuite's gripper integrates them.
Besides the observations, the episode side reads one thing of the robot: the torques its controller commands to the
arm's motors (the arm's strain guard, :meth:`hexashell.benches.arm.Arm.straining`).  A perturbation may delay the
readings or the commands: the readings then show the arm some steps back (:mod:`.timing` follows how many, the
*lag*, from the commands sent and the readings alone), and ``data`` is the arm predicted ahead of them, where the
commands already sent will leave it (``seen`` the readings' own, the pictures' and the cameras'; one and the same
with no lag).  The robot's own pixels are not read from the simulator (its segmentation would show where a
perturbation put the robot or a camera): without depth they are predicted from the readings on the nominal model
(:class:`RobotPixels`), and the table top the objects stand on is the nominal model's (:func:`nominal_table`; no
perturbation moves it).

Frame: as LIBERO-PRO's (:mod:`hexashell.benches.libero.sim`), the *arm frame* of the robot's base (body
``robot0_base``) in the nominal model: origin at the arm's base plate, +x forward, +y left, +z up; the robot
never drives.  Hand orientation ("tool" axes): x points from the wrist out between the fingers, y is the line
the fingers open along, z = x × y.
"""

from __future__ import annotations

import copy
import json
import os
import time

import numpy as np

from .. import arm as A
from ..gpucheck import check_depth, check_time
from . import protocol as P
from . import timing as TM

GRIP_SITE = "gripper0_grip_site"
BASE_BODY = "robot0_base"
ARM_BASE_BODY = "robot0_link0"
SHOULDER_BODY = "robot0_link2"
HAND_BODY = "gripper0_right_gripper"
PADS = ("gripper0_finger1_pad_collision", "gripper0_finger2_pad_collision")
FINGER_JOINTS = ("gripper0_finger_joint1", "gripper0_finger_joint2")
SITE_TO_TOOL = A.PANDA_SITE_TO_TOOL
#: the observations of the robot it is read from (robosuite 1.4's names)
JOINTS, JOINT_SPEEDS = "robot0_joint_pos", "robot0_joint_vel"
FINGERS, FINGER_SPEEDS = "robot0_gripper_qpos", "robot0_gripper_qvel"
#: robosuite 1.4's Panda gripper: in each physics step of a control step its level moves by this much the way of
#: the action's sign, so 0.25 a control step (25 physics steps of 2 ms)
GRIP_SPEED, SUBSTEPS = 0.01, 25
#: the wrist camera on the hand (body robot0_right_hand): position, quaternion (w x y z), vertical field of view
#: (deg), the same in every scene; and the agentview camera's field of view
WRIST_MOUNT = ((0.05, 0.0, 0.0), (0.0, 0.70710678, 0.70710678, 0.0), 75.0)
AGENTVIEW_FOVY = 45.0
#: The robot's own pixels: each of its visual geoms (MuJoCo group VISUAL_GROUP, the ones the cameras draw; bodies
#: named ROBOT_BODIES ...) where the readings put it on the nominal model, drawn through the nominal camera as the
#: convex hull of its points (a mesh's hull, a box's corners) cut NEAR_M in front of the camera; what is below the
#: table top (:func:`nominal_table`) left out where the table top is between it and the camera (below a floor:
#: always).  Numbered by :data:`.protocol.ROBOT_PARTS` (FINGER_BODIES; STILL_BODIES and the stand, mount0_*; the
#: rest is the arm), each part widened by MARGIN_PX, the body drawn first and the fingers last.  Nothing else is
#: known to stand in front of the robot, so where something does, its pixels count as the robot's (blind): in the
#: study scene the desk caddy and the wall hide the arm's base from agentview, some 5k pixels.
VISUAL_GROUP = 1
ROBOT_BODIES = ("robot0_", "gripper0_", "mount0_")
FINGER_BODIES = ("gripper0_leftfinger", "gripper0_rightfinger")
STILL_BODIES = ("robot0_base", "robot0_link0")
NEAR_M = 0.01
MARGIN_PX = 2
#: The table top (or floor) the objects stand on, from the nominal model: rays cast straight down every
#: TABLE_GRID_M over TABLE_SEARCH (arm frame, from TABLE_RAY_Z), past the robot, the objects that move (under a
#: free joint) and the geoms nothing collides with; its height the most common height hit over TABLE_WORK, in
#: front of the arm (TABLE_BIN_M bins), refined by the median of those within TABLE_NEAR_M of it; its extent where
#: the hits that close to it lie.  A floor (a plane) has no extent.  (Where the objects stand: the table drawn in
#: the pictures may lie lower, 4.5 mm in the study scene and 11 mm in the living room ones, 0.5 mm elsewhere.)
TABLE_SEARCH = ((-0.4, 1.8), (-1.3, 1.3))
TABLE_WORK = ((0.3, 0.8), (-0.4, 0.4))
TABLE_GRID_M = 0.02
TABLE_RAY_Z = 2.0
TABLE_BIN_M = 0.002
TABLE_NEAR_M = 0.005

_MANIFEST = None


def manifest():
    """The official task manifest (it checks every BDDL file's hash as it loads)."""
    global _MANIFEST
    if _MANIFEST is None:
        from libero_remote.environment.task_manifest import TaskManifest

        _MANIFEST = TaskManifest.load(os.environ["LIBERO_REMOTE_TASK_MANIFEST"])
    return _MANIFEST


def task_of(suite_name: str, index: int, case: str):
    """(the manifest's entry, the path of its BDDL file, the base task's initial states)."""
    from libero.libero import benchmark

    entry = manifest().require(suite_name, index, case)
    if int(entry["max_steps"]) != P.MAX_STEPS[suite_name] \
            or int(entry.get("reset_stabilization_steps", P.SETTLE_STEPS)) != P.SETTLE_STEPS:
        raise RuntimeError(f"the manifest's limits of {suite_name} {index} {case} are not the protocol's")
    bddl = str(manifest().resolve_bddl(suite_name, index, case))
    states = benchmark.get_benchmark_dict()[suite_name]().get_task_init_states(int(index))
    return entry, bddl, states


#: how many times building the environment is tried.  robosuite places the objects once while it builds the
#: environment, from the global random state, before any seed is set; on some instances that placing fails now and
#: then (it is placed again at the reset, after the seed, with the official retries).  The start does not depend on
#: the building's placing (the same official fingerprints after the reset), so a failed building is tried again.
BUILD_ATTEMPTS = 10


def make(bddl: str, size: int = P.IMAGE_SIZE, depth: bool = False):
    """The official evaluation's environment, its cameras at ``size`` pixels (``depth``: with depth); built again
    when the objects could not be placed while building it (:data:`BUILD_ATTEMPTS`)."""
    from libero.libero.envs import OffScreenRenderEnv
    from robosuite.utils.errors import RandomizationError

    for attempt in range(1, BUILD_ATTEMPTS + 1):
        try:
            return OffScreenRenderEnv(bddl_file_name=bddl, camera_names=list(P.CAMERAS.values()),
                                      camera_heights=size, camera_widths=size, camera_depths=bool(depth))
        except RandomizationError:
            if attempt == BUILD_ATTEMPTS:
                raise


def plain(value):
    """``value`` as JSON has it (the perturbation plan, for the record)."""
    return json.loads(json.dumps(value, default=str))


class CommandedGripper:
    """The gripper's level as robosuite 1.4's Panda gripper integrates the actions sent: in each physics step
    of a control step it moves by ``speed`` the way of the action's sign (-1 open .. +1 closed), held at the
    ends.  ``current_action`` as the gripper's own (:meth:`hexashell.benches.arm.Arm.level` reads it)."""

    def __init__(self, current_action, speed: float = GRIP_SPEED, substeps: int = SUBSTEPS):
        self.current_action = np.array(current_action, float).reshape(-1)
        self.speed, self.substeps = float(speed), int(substeps)

    def command(self, action: float) -> None:
        sign = float(np.sign(action))
        for _ in range(self.substeps):
            self.current_action = np.clip(self.current_action + np.array([-1.0, 1.0]) * self.speed * sign,
                                          -1.0, 1.0)


class Belief:
    """The robot as its readings show it: a copy of the scene's model with the fields the perturbations change
    put back as built (the runtime's baseline), its arm and finger joints at the observed positions and speeds,
    everything else at the model's rest pose; MuJoCo's kinematics of that (no physics).  ``data.ctrl``: the
    torques the arm's controller commands, copied in each step."""

    def __init__(self, model, baseline: dict, arm_qpos, arm_dof, finger_qpos, finger_dof):
        import mujoco

        self.model = copy.deepcopy(model)
        for field, value in baseline.items():
            getattr(self.model, field)[:] = value
        self.data = mujoco.MjData(self.model)
        self.arm_qpos, self.arm_dof = list(arm_qpos), list(arm_dof)
        self.finger_qpos, self.finger_dof = list(finger_qpos), list(finger_dof)

    def update(self, obs: dict, ctrl: np.ndarray) -> None:
        import mujoco

        m, d = self.model, self.data
        d.qpos[self.arm_qpos] = np.asarray(obs[JOINTS], float).reshape(-1)[:len(self.arm_qpos)]
        d.qvel[self.arm_dof] = np.asarray(obs[JOINT_SPEEDS], float).reshape(-1)[:len(self.arm_dof)]
        d.qpos[self.finger_qpos] = np.asarray(obs[FINGERS], float).reshape(-1)[:len(self.finger_qpos)]
        d.qvel[self.finger_dof] = np.asarray(obs[FINGER_SPEEDS], float).reshape(-1)[:len(self.finger_dof)]
        d.ctrl[:] = ctrl
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_camlight(m, d)
        mujoco.mj_comVel(m, d)


def _geom_points(model, g: int) -> np.ndarray | None:
    """Points of geom ``g`` in its own frame whose convex hull is the geom (about, for a round one)."""
    import mujoco

    kind, size = int(model.geom_type[g]), np.asarray(model.geom_size[g], float)
    T = mujoco.mjtGeom
    if kind == T.mjGEOM_MESH:
        mesh = int(model.geom_dataid[g])
        start, n = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        return np.asarray(model.mesh_vert[start:start + n], float)
    if kind == T.mjGEOM_BOX:
        return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float) * size
    turn = np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False)
    ring = np.stack([np.cos(turn), np.sin(turn), np.zeros_like(turn)], axis=1)
    if kind in (T.mjGEOM_SPHERE, T.mjGEOM_ELLIPSOID):
        radii = size if kind == T.mjGEOM_ELLIPSOID else np.full(3, size[0])
        rings = [ring * np.cos(a) + np.array([0.0, 0.0, np.sin(a)]) for a in np.linspace(-1.2, 1.2, 5)]
        return np.concatenate(rings + [np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])]) * radii
    if kind in (T.mjGEOM_CYLINDER, T.mjGEOM_CAPSULE):
        r, half = size[0], size[1] + (size[0] if kind == T.mjGEOM_CAPSULE else 0.0)
        return np.concatenate([ring * r + np.array([0.0, 0.0, z]) for z in (-half, half)])
    return None


def _hull(points: np.ndarray) -> tuple:
    """(the indices of the convex hull's vertices among ``points`` (N, 3), its edges as index pairs into those);
    a flat set: all its points, every pair an edge."""
    from scipy.spatial import ConvexHull

    try:
        hull = ConvexHull(points)
    except Exception:  # noqa: BLE001 - flat
        n = len(points)
        return np.arange(n), np.array([(i, j) for i in range(n) for j in range(i + 1, n)], int).reshape(-1, 2)
    index = {int(v): k for k, v in enumerate(hull.vertices)}
    pairs = {tuple(sorted((index[int(u)], index[int(v)])))
             for tri in hull.simplices for u, v in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[0], tri[2]))}
    return np.asarray(hull.vertices, int), np.array(sorted(pairs), int)


def _side(points: np.ndarray, edges: np.ndarray, values: np.ndarray, limit: float) -> np.ndarray:
    """The points whose hull is the part of a convex hull (vertices ``points``, edges ``edges``) where ``values``
    (one per vertex, linear in space) are at least ``limit``: the vertices there and where the edges cross it."""
    keep = values >= limit
    if keep.all() or not keep.any():
        return points[keep]
    a, b = edges[:, 0], edges[:, 1]
    cross = keep[a] != keep[b]
    va, vb = values[a[cross]], values[b[cross]]
    pa, pb = points[a[cross]], points[b[cross]]
    return np.concatenate([points[keep], pa + ((va - limit) / (va - vb))[:, None] * (pb - pa)])


class RobotPixels:
    """The robot's own pixels in a picture, predicted from the readings (see VISUAL_GROUP): what the episode side
    takes for the robot, never the simulator's segmentation."""

    def __init__(self, model):
        #: (geom, part number, hull points (N, 3) in the geom's frame, hull edges (E, 2))
        self.geoms = []
        for g in range(model.ngeom):
            body = model.body(int(model.geom_bodyid[g])).name
            if int(model.geom_group[g]) != VISUAL_GROUP or float(model.geom_rgba[g, 3]) <= 0.0 \
                    or not body.startswith(ROBOT_BODIES):
                continue
            points = _geom_points(model, g)
            if points is None or not len(points):
                continue
            vertices, edges = _hull(points)
            if body in FINGER_BODIES:
                part = "fingers"
            elif body in STILL_BODIES or body.startswith("mount0_"):
                part = "body"
            else:
                part = "arm"
            self.geoms.append((g, P.ROBOT_PARTS.index(part) + 1, points[vertices], edges))
        #: the part numbers in the order they are drawn: the body, the arm, the fingers over them
        self.order = tuple(P.ROBOT_PARTS.index(part) + 1 for part in ("body", "arm", "fingers"))
        if not any(item[1] == self.order[-1] for item in self.geoms):
            raise RuntimeError("no visual geoms of the fingers: the robot's bodies are not the expected ones")
        self.kernel = np.ones((2 * MARGIN_PX + 1,) * 2, np.uint8)

    @staticmethod
    def table_pixels(camera: dict, table: dict) -> np.ndarray | None:
        """bool (H, W): the pixels whose ray meets the table top (``table`` as :func:`nominal_table` gives it; a
        floor anywhere) in front of the camera, which must be above it; None if it is not."""
        pose = np.asarray(camera["pose"], float)
        z = float(table["z"])
        if pose[2, 3] <= z:
            return None
        w, h, f = int(camera["width"]), int(camera["height"]), float(camera["focal"])
        u = ((np.arange(w) - (w - 1) / 2.0) / f)[None, :]
        v = (-(np.arange(h) - (h - 1) / 2.0) / f)[:, None]
        rot, origin = pose[:3, :3], pose[:3, 3]
        down = rot[2, 0] * u + rot[2, 1] * v - rot[2, 2]  # the rays (u, v, -1) in the camera's axes
        hit = down < -1e-9
        if table.get("floor") or table.get("x") is None:
            return np.broadcast_to(hit, (h, w)).copy()
        t = np.where(hit, (z - origin[2]) / np.where(hit, down, -1.0), 0.0)
        x = origin[0] + t * (rot[0, 0] * u + rot[0, 1] * v - rot[0, 2])
        y = origin[1] + t * (rot[1, 0] * u + rot[1, 1] * v - rot[1, 2])
        (x0, x1), (y0, y1) = table["x"], table["y"]
        return hit & (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)

    def draw(self, data, camera: dict, to_start: np.ndarray, table: dict | None = None) -> np.ndarray:
        """uint8 (H, W): the part numbers of the robot's pixels (0 elsewhere) in a picture of ``camera`` (as
        :meth:`LiberoProSim.camera` gives it: arm frame, MuJoCo's axes) with the geoms where ``data`` has them;
        ``table``: the table top that hides what is below it."""
        import cv2

        w, h, f = int(camera["width"]), int(camera["height"]), float(camera["focal"])
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        to_cam = np.linalg.inv(np.asarray(camera["pose"], float)) @ to_start
        hidden = None if not table else self.table_pixels(camera, table)
        floor = hidden is not None and (table.get("floor") or table.get("x") is None)
        tz = float(table["z"]) if hidden is not None else 0.0
        layers = {part: np.zeros((h, w), np.uint8) for part in self.order}
        below = {part: np.zeros((h, w), np.uint8) for part in self.order}
        for g, part, points, edges in self.geoms:
            rot, pos = data.geom_xmat[g].reshape(3, 3), data.geom_xpos[g]
            # camera-frame x y z and arm-frame z of the hull's vertices
            local = np.concatenate([points @ (to_cam[:3, :3] @ rot).T + (to_cam[:3, :3] @ pos + to_cam[:3, 3]),
                                    (points @ (to_start[2, :3] @ rot) + (to_start[2, :3] @ pos + to_start[2, 3]))[:, None]],
                                   axis=1)
            pieces = [(local, edges, layers[part])]
            if hidden is not None and (local[:, 3] < tz).any():
                pieces = [(_side(local, edges, local[:, 3], tz), None, layers[part])]
                if not floor:
                    pieces.append((_side(local, edges, -local[:, 3], -tz), None, below[part]))
            for piece, piece_edges, out in pieces:
                if len(piece) < 3:
                    continue
                depth = -piece[:, 2]
                if not (depth > NEAR_M).any():
                    continue
                if not (depth > NEAR_M).all():  # cut at the near plane, where the hull's edges cross it
                    if piece_edges is None:
                        keep, piece_edges = _hull(piece[:, :3])
                        piece, depth = piece[keep], depth[keep]
                    piece = _side(piece, piece_edges, depth, NEAR_M)
                    depth = -piece[:, 2]
                x, y = f * piece[:, 0] / depth + cx, -f * piece[:, 1] / depth + cy
                if x.max() < 0 or y.max() < 0 or x.min() > w - 1 or y.min() > h - 1:
                    continue
                poly = cv2.convexHull(np.stack([x, y], axis=1).astype(np.float32))
                cv2.fillConvexPoly(out, np.round(poly).astype(np.int32).reshape(-1, 2), 1)
        out = np.zeros((h, w), np.uint8)
        for part in self.order:
            mask = layers[part]
            if hidden is not None and not floor:
                mask = mask | (below[part] & ~hidden)
            if MARGIN_PX:
                mask = cv2.dilate(mask, self.kernel)
            out[mask > 0] = part
        return out


def _moves(model, body: int) -> bool:
    """Whether the body hangs from a free joint (an object that can be moved about)."""
    import mujoco

    while body > 0:
        start, n = int(model.body_jntadr[body]), int(model.body_jntnum[body])
        if any(int(model.jnt_type[j]) == mujoco.mjtJoint.mjJNT_FREE for j in range(start, start + n)):
            return True
        body = int(model.body_parentid[body])
    return False


def nominal_table(model, data, to_start: np.ndarray) -> dict:
    """The surface the objects stand on in the nominal model (see TABLE_SEARCH), arm frame: {"z", "x": (x0, x1) or
    None, "y", "floor": whether it is a plane, "geoms": the names hit at its height}."""
    import mujoco

    m = copy.deepcopy(model)
    d = mujoco.MjData(m)
    d.qpos[:] = data.qpos
    mujoco.mj_kinematics(m, d)
    hidden = 5
    for g in range(m.ngeom):
        body = int(m.geom_bodyid[g])
        if m.body(body).name.startswith(ROBOT_BODIES) or _moves(m, body) \
                or (int(m.geom_contype[g]) == 0 and int(m.geom_conaffinity[g]) == 0):
            m.geom_group[g] = hidden
    groups = np.array([1, 1, 1, 1, 1, 0], np.uint8)
    from_start = np.linalg.inv(to_start)
    if not np.allclose(from_start[:3, 2], (0.0, 0.0, 1.0), atol=1e-6):
        raise RuntimeError("the arm frame's z is not the world's up")
    down = np.array([0.0, 0.0, -1.0])
    (x0, x1), (y0, y1) = TABLE_SEARCH
    xs = np.arange(x0, x1 + 1e-9, TABLE_GRID_M)
    ys = np.arange(y0, y1 + 1e-9, TABLE_GRID_M)
    hits, found = [], np.zeros(1, np.int32)
    for x in xs:
        for y in ys:
            origin = from_start[:3, :3] @ np.array([x, y, TABLE_RAY_Z]) + from_start[:3, 3]
            dist = mujoco.mj_ray(m, d, origin, down, groups, 1, -1, found)
            if dist >= 0:
                hits.append((x, y, TABLE_RAY_Z - dist, int(found[0])))
    if not hits:
        raise RuntimeError("no surface under the workspace in the nominal model")
    hits = np.array(hits, float)
    (wx0, wx1), (wy0, wy1) = TABLE_WORK
    work = hits[(hits[:, 0] >= wx0) & (hits[:, 0] <= wx1) & (hits[:, 1] >= wy0) & (hits[:, 1] <= wy1), 2]
    if len(work) < 10:
        raise RuntimeError("no surface in front of the arm in the nominal model")
    counts, bins = np.histogram(work, bins=np.arange(work.min(), work.max() + 2 * TABLE_BIN_M, TABLE_BIN_M))
    k = int(np.argmax(counts))
    z = float(np.median(work[np.abs(work - 0.5 * (bins[k] + bins[k + 1])) <= TABLE_NEAR_M]))
    level = hits[np.abs(hits[:, 2] - z) <= TABLE_NEAR_M]
    names = sorted({m.geom(int(g)).name or f"geom {int(g)}" for g in level[:, 3]})
    floor = any(int(m.geom_type[int(g)]) == mujoco.mjtGeom.mjGEOM_PLANE for g in set(level[:, 3]))
    half = 0.5 * TABLE_GRID_M
    return {"z": round(z, 4), "floor": bool(floor), "geoms": names,
            "x": None if floor else (round(float(level[:, 0].min()) - half, 3), round(float(level[:, 0].max()) + half, 3)),
            "y": None if floor else (round(float(level[:, 1].min()) - half, 3), round(float(level[:, 1].max()) + half, 3))}


class LiberoProSim:
    """One instance's environment; ``reset(seed, init_state)`` starts an episode."""

    def __init__(self, task: str, size: int = P.IMAGE_SIZE, depth: bool = False):
        self.suite, self.index, self.case = P.parse_task(task)
        self.task = task
        self.size = int(size)
        #: whether the cameras render depth too (the depth mode)
        self.depth = bool(depth)
        self.horizon = P.max_steps(task)
        self.entry, self.bddl, self.init_states = task_of(self.suite, self.index, self.case)
        self.env = make(self.bddl, self.size, self.depth)
        self.robot_pixels: RobotPixels | None = None
        self.table: dict = {}
        self.steps = 0
        self.success = False
        self.done = False
        self.obs: dict = {}
        self.facts: dict = {}
        #: {camera: :class:`.calib.Fix`}: the cameras as the camera check re-fitted them (none: the nominal ones);
        #: ``on_step(sim)`` after every step while the check moves the hand.  Both cleared at the reset.
        self.fixes: dict = {}
        self.on_step = None

    @property
    def rs(self):
        """The robosuite environment inside LIBERO's wrapper."""
        return self.env.env

    # -- episode ---------------------------------------------------------------------------------
    def reset(self, seed: int, init_state: int = P.INIT_STATE) -> dict:
        """The official start: the seeds, the reset, the initial state (the official plan's is 0: see
        :func:`.protocol.init_state_of`), the settling steps, the perturbation seed again, the rollout begun, the
        controller taking deltas."""
        env, seed, init_state = self.env, int(seed), int(init_state)
        if not 0 <= init_state < len(self.init_states):
            raise ValueError(f"{self.task} has initial states 0-{len(self.init_states) - 1}, not {init_state}")
        t0 = time.time()
        env.seed(seed)
        env.set_perturbation_seed(seed)
        env.reset()
        env.set_init_state(self.init_states[init_state])
        for _ in range(P.SETTLE_STEPS):
            env.step(np.array(P.SETTLE_ACTION, dtype=np.float32))
        env.set_perturbation_seed(seed)
        obs = env.begin_perturbation_rollout()
        robot = self.rs.robots[0]
        robot.controller.use_delta = True
        check_time(time.time() - t0, "agentview", first=True)
        self.steps, self.success, self.done = 0, False, False
        sim = self.rs.sim
        self.real_model, self.real_data = sim.model._model, sim.data._data
        runtime = self.rs.perturbation_runtime
        if runtime._baseline_model_id != id(sim.model):
            raise RuntimeError("the perturbation runtime's baseline is not of this model")
        self._ids(self.real_model)
        self.belief = Belief(self.real_model, runtime._model_baseline, self.arm_qpos, self.arm_dof,
                             self.finger_qpos, self.finger_dof)
        self.model = self.belief.model
        #: the robot as read (the pictures' and the cameras'), and as the controller steers it: the same data but
        #: while the readings lag the commands (:meth:`_predict`)
        self.seen = self.data = self.belief.data
        self._ahead = None
        self._check_mounts()
        substeps = int(round(self.rs.control_timestep / self.rs.model_timestep))
        if abs(float(robot.gripper.speed) - GRIP_SPEED) > 1e-12 or substeps != SUBSTEPS:
            raise RuntimeError(f"unexpected gripper: speed {robot.gripper.speed}, {substeps} physics steps a step")
        self.gripper = CommandedGripper(robot.gripper.current_action)
        self._take(obs)
        self.replica = TM.Replica(self.model, self.seen.qpos, robot.controller, robot.torque_limits, self.site_id,
                                  self.arm_qpos, self.arm_dof, self.finger_qpos, self.finger_dof,
                                  A.arm_motors(self.model, self.arm_dof)[0], substeps)
        self.timing = TM.Timing(self.replica.run, *self._reading(), scaled=self.replica.scaled)
        self.to_start = np.linalg.inv(self.body_world(BASE_BODY))
        A.check_tool_axes(self.model, self.data, self.site_id, SITE_TO_TOOL, PADS, HAND_BODY)
        if self.robot_pixels is None:
            self.robot_pixels = RobotPixels(self.model)
        self.table = nominal_table(self.model, self.data, self.to_start)
        self.fixes, self.on_step = {}, None
        self.facts = {
            "task": self.task, "suite": self.suite, "task_index": self.index, "case": self.case,
            "kind": P.kind(self.case), "seed": seed, "init_state": init_state,
            "instruction": self.instruction(), "robot": type(robot.robot_model).__name__, "horizon": self.horizon,
            "step_s": P.STEP_S, "settle_steps": P.SETTLE_STEPS, "manifest": manifest().version,
            "bddl_sha256": self.entry.get("bddl_sha256"), "depth": self.depth, "table": self.table,
        }
        return self.facts

    def record(self) -> dict:
        """For the record of the opening, never the model: the perturbation plan and the official fingerprints of
        it and of the starting state, and how far the readings put the grasp point from where it is."""
        from libero_remote.reproducibility import canonical_fingerprint, numerical_state_fingerprint

        plan = self.env.get_perturbation_plan()
        site = self.real_data.site_xpos[self.site_id]
        belief = self.data.site_xpos[self.site_id]
        return {"plan": plain(plan), "plan_fingerprint": canonical_fingerprint(plan),
                "state_fingerprint": numerical_state_fingerprint(self.env.get_sim_state()),
                "reading_error_mm": round(1000.0 * float(np.linalg.norm(belief - site)), 3)}

    def _ids(self, m) -> None:
        self.site_id = m.site(GRIP_SITE).id
        self.link0_id = m.body(ARM_BASE_BODY).id
        self.shoulder_id = m.body(SHOULDER_BODY).id
        self.finger_qpos = [int(m.jnt_qposadr[m.joint(j).id]) for j in FINGER_JOINTS]
        self.finger_dof = [int(m.jnt_dofadr[m.joint(j).id]) for j in FINGER_JOINTS]
        self.cam_ids = {cam: m.camera(name).id for cam, name in P.CAMERAS.items()}
        self.arm_qpos, self.arm_dof, self.arm_limits = A.arm_joints(m)

    def _check_mounts(self) -> None:
        """The nominal cameras must be the scenes' own: the wrist camera on the hand, agentview's field of view."""
        m = self.model
        pos, quat, fovy = WRIST_MOUNT
        i, j = self.cam_ids["wrist"], self.cam_ids["agentview"]
        if not (np.allclose(m.cam_pos[i], pos, atol=1e-6) and np.allclose(m.cam_quat[i], quat, atol=1e-6)
                and abs(float(m.cam_fovy[i]) - fovy) < 1e-6 and abs(float(m.cam_fovy[j]) - AGENTVIEW_FOVY) < 1e-6):
            raise RuntimeError("the nominal cameras are not the scene's (the baseline holds a perturbation?)")

    def _take(self, obs: dict) -> None:
        self.obs = obs
        self.belief.update(obs, self.real_data.ctrl)

    def _reading(self) -> tuple:
        """(the arm's joint positions, joint velocities) as read."""
        n = len(self.arm_qpos)
        return (np.asarray(self.obs[JOINTS], float).reshape(-1)[:n].copy(),
                np.asarray(self.obs[JOINT_SPEEDS], float).reshape(-1)[:n].copy())

    def _jacobian(self) -> np.ndarray:
        """The grip site's Jacobian (linear rows, then angular; the arm's joints) where the readings put the arm."""
        import mujoco

        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.seen, jp, jr, self.site_id)
        return np.vstack([jp[:, self.arm_dof], jr[:, self.arm_dof]])

    def _predict(self) -> None:
        """``data``: the readings' own while nothing lags; else the arm as :class:`.timing.Timing` predicts it
        (the fingers, the speeds of the rest and the torques as read)."""
        import mujoco

        if self.timing.lag == 0:
            self.data = self.seen
            return
        if self._ahead is None:
            self._ahead = mujoco.MjData(self.model)
        d, s = self._ahead, self.seen
        q, qd = self.timing.predicted()
        d.qpos[:] = s.qpos
        d.qvel[:] = s.qvel
        d.qpos[self.arm_qpos] = q
        d.qvel[self.arm_dof] = qd
        d.ctrl[:] = s.ctrl
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_comPos(self.model, d)
        mujoco.mj_camlight(self.model, d)
        mujoco.mj_comVel(self.model, d)
        self.data = d

    @property
    def lag(self) -> int:
        """How many steps the readings lag the commands (:mod:`.timing`)."""
        return int(self.timing.lag)

    @property
    def over(self) -> bool:
        return self.done or self.success or self.steps >= self.horizon

    def step(self, action: np.ndarray) -> bool:
        """One control step (20 Hz) with LIBERO's 7-dim action, as the official step: the task's success check
        after it.  Returns False once the episode is over."""
        if self.over:
            return False
        a = np.asarray(action, dtype=np.float32)
        t0 = time.time()
        obs, _, done, _ = self.env.step(a)
        check_time(time.time() - t0, "agentview", first=False)
        self.gripper.command(float(a[6]))
        self.steps += 1
        self.success = bool(self.env.check_success())
        self.done = bool(done)
        self._take(obs)
        self.replica.fingers = np.asarray(obs[FINGERS], float).reshape(-1)[:len(self.finger_qpos)].copy()
        self.timing.update(np.asarray(a[:6], float), *self._reading(), self._jacobian())
        self._predict()
        if self.on_step is not None:
            self.on_step(self)
        return not self.over

    def instruction(self) -> str:
        """The instruction now (the official step's info): a perturbation may change it during the episode."""
        return str(self.env.language_instruction).strip()

    def status(self) -> dict:
        return {"step": self.steps, "horizon": self.horizon, "success": self.success, "over": self.over,
                "instruction": self.instruction()}

    # -- poses (arm frame: the arm as predicted, or as read with ``data=self.seen``) -----------------------
    def body_world(self, name: str) -> np.ndarray:
        i = self.model.body(name).id
        pose = np.eye(4)
        pose[:3, :3] = self.data.xmat[i].reshape(3, 3)
        pose[:3, 3] = self.data.xpos[i]
        return pose

    def body(self, index: int) -> np.ndarray:
        pose = np.eye(4)
        pose[:3, :3] = self.data.xmat[index].reshape(3, 3)
        pose[:3, 3] = self.data.xpos[index]
        return self.to_start @ pose

    def site_pose(self, data=None) -> np.ndarray:
        """The grip site in the arm frame (MuJoCo site axes)."""
        data = self.data if data is None else data
        pose = np.eye(4)
        pose[:3, :3] = data.site_xmat[self.site_id].reshape(3, 3)
        pose[:3, 3] = data.site_xpos[self.site_id]
        return self.to_start @ pose

    def tcp(self, data=None) -> np.ndarray:
        """The grasp point with the tool axes, arm frame."""
        pose = self.site_pose(data)
        pose[:3, :3] = pose[:3, :3] @ SITE_TO_TOOL
        return pose

    def link0(self) -> np.ndarray:
        return self.body(self.link0_id)

    def shoulder(self) -> np.ndarray:
        return self.body(self.shoulder_id)[:3, 3]

    def finger_width(self) -> float:
        q = np.asarray(self.obs[FINGERS], float).reshape(-1)
        return float(q[0] - q[1])

    def finger_speed(self) -> float:
        v = np.asarray(self.obs[FINGER_SPEEDS], float).reshape(-1)
        return float(abs(v[0] - v[1]))

    def arm_joints(self) -> np.ndarray:
        return self.data.qpos[self.arm_qpos].copy()

    def proprio(self) -> dict:
        """The hand and the joints (as predicted), the fingers' gap (as read); while the readings lag, the lag
        (``lag_steps``) and the hand and the joints as read too (``seen_tcp``, ``seen_joints``)."""
        out = {"tcp": self.tcp(), "width": self.finger_width(), "joints": self.arm_joints()}
        if self.data is not self.seen:
            out.update(lag_steps=self.lag, seen_tcp=self.tcp(self.seen),
                       seen_joints=self.seen.qpos[self.arm_qpos].copy())
        return out

    # -- pictures --------------------------------------------------------------------------------
    def camera(self, cam: str, data=None) -> dict:
        """The pinhole model of camera ``cam`` that the pictures (:meth:`observe`) are of: the nominal one
        (:meth:`nominal_camera`), or as the camera check re-fitted it (:mod:`.calib`)."""
        out = self.nominal_camera(cam, data)
        fix = self.fixes.get(cam)
        return out if fix is None else fix.camera(out)

    def nominal_camera(self, cam: str, data=None) -> dict:
        """The nominal pinhole model of camera ``cam`` (the wrist camera where the readings put the hand, or where
        ``data`` has it)."""
        i, data = self.cam_ids[cam], self.seen if data is None else data
        pose = np.eye(4)
        pose[:3, :3] = data.cam_xmat[i].reshape(3, 3)  # MuJoCo: looks along -z, +y up the picture
        pose[:3, 3] = data.cam_xpos[i]
        fovy = np.radians(float(self.model.cam_fovy[i]))
        return {"pose": self.to_start @ pose, "focal": 0.5 * self.size / np.tan(0.5 * fovy),
                "width": self.size, "height": self.size}

    def depth_picture(self, cam: str) -> np.ndarray:
        """Depth (m along the optical axis) of the step's observation; 0 where it has none (a value of exactly 0)."""
        from robosuite.utils.camera_utils import get_real_depth_map

        raw = np.asarray(self.obs[f"{P.CAMERAS[cam]}_depth"], np.float64)[::-1]
        raw = check_depth(raw.reshape(raw.shape[0], raw.shape[1]), cam)
        out = get_real_depth_map(self.rs.sim, np.clip(raw, 0.0, 1.0))
        out[raw <= 0.0] = 0.0
        return np.ascontiguousarray(out.astype(np.float32))

    def picture(self, cam: str) -> np.ndarray:
        """The step's picture from camera ``cam`` as taken (uint8, the top row first)."""
        image = np.asarray(self.obs[f"{P.CAMERAS[cam]}_image"])[::-1]
        return np.ascontiguousarray(image.astype(np.uint8))

    def observe(self, cams=None, robot_mask: bool = True, parts: bool = False) -> dict:
        """The step's pictures (with depth in the depth mode) with the cameras' models (:meth:`camera`: as the
        camera check corrected the pictures, if it did) and the readings' proprioception; ``parts``: the robot's own
        pixels as predicted (:class:`RobotPixels`).  (None from the simulator's segmentation: ``robot_mask`` is
        ignored.)"""
        out = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "parts": {}, "cameras": {}}
        for cam in cams or list(P.CAMERAS):
            image, fix = self.picture(cam), self.fixes.get(cam)
            out["images"][cam] = image if fix is None else fix.picture(image)
            out["cameras"][cam] = camera = self.camera(cam)
            if self.depth:
                out["depth"][cam] = self.depth_picture(cam)
            if parts:
                out["parts"][cam] = self.robot_pixels.draw(self.seen, camera, self.to_start, self.table)
        out["proprio"] = self.proprio()
        return out

    def pictured_at(self, joints, cams=None) -> dict:
        """With the arm's joints at ``joints`` (the fingers as they are): the cameras' models (:meth:`camera`) and
        the robot's pixels as predicted (:class:`RobotPixels`) - where the robot would show if the arm stood so."""
        import mujoco

        d = mujoco.MjData(self.model)
        d.qpos[:] = self.seen.qpos
        d.qpos[self.arm_qpos] = np.asarray(joints, float).reshape(-1)[:len(self.arm_qpos)]
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_camlight(self.model, d)
        out = {"cameras": {}, "parts": {}}
        for cam in cams or list(P.CAMERAS):
            out["cameras"][cam] = camera = self.camera(cam, d)
            out["parts"][cam] = self.robot_pixels.draw(d, camera, self.to_start, self.table)
        return out

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:  # noqa: BLE001
            pass
