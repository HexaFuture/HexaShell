"""What one request of the Season II benchmark brings, made ready for the robot calls.

A request (``{"method": "infer", "params": {...}}``, msgpack; genie_sim 3.6.0's ``corobotpolicy.py``) carries
the three cameras' colour pictures (JPEG), the robot's readings (both arms' joints, the effectors, the waist,
the head, the chassis on the G2, each hand's pose in ``base_link`` and in ``arm_base_link``), the instruction
(``prompt``), the stage texts (``segments``), the robot (``robot_type``, ``ee_type``), the scene's name and
whether the episode has just ended (``episode_done``).  The episode's first request, and every request after a
reply with ``need_depth``, adds each camera's depth picture (uint16 millimetres at the colour picture's size)
and its calibration (OpenCV pinhole with distortion; the optical frame in ``base_link``).

Here:

- the pictures are undistorted (colour bilinear, depth nearest) into a pinhole camera with the same
  intrinsics, and the hand cameras, which come at 640x528 (omnipicker) or 1280x1056 (ctek90d), are brought to
  640x528 (the intrinsics halved, depth every second pixel);
- cameras are named ``head``, ``left_wrist`` and ``right_wrist`` (the benchmark's ``head``, ``hand_left``,
  ``hand_right``) and modelled as :class:`hexashell.vision.cameras.Camera` in ``base_link`` (x forward, y left,
  z up), the world frame of the robot calls;
- a calibration marked unavailable only because the colour pixels were perturbed (``pose_checked_against_render``)
  is used: only the colour is perturbed, not the geometry or the depth;
- a camera without calibration in a request keeps the last calibration seen for it (:class:`Calibrations`, one
  per episode): a hand camera's pose is carried along with its hand (it sits fixed on the gripper), the head
  camera's with ``arm_base_link`` (both sit on the torso; nothing of ours turns the head);
- a black colour picture (all pixels about 0) is a dropped frame: listed in ``Frame.dropped``;
- the simulator's depth is noisy (per pixel, a standard deviation of about 1.6% of the depth: 2.4 cm at the
  1.4 m from the WBC robot's head to the floor, enough to add 4-5 cm to a height read from the far ends of a
  measured spread); on the WBC board (``parse(..., smooth=True)``) the depth pictures are smoothed, keeping
  the edges between things apart in depth (:func:`smooth_depth`).  The table boards keep the raw depth.

The robot is known from ``robot_type`` and ``ee_type`` (``protocol.ROBOTS``); the joint and effector counts are
the robot's.

The G2's Wuji hands (dexmanip) carry fisheye cameras whose calibration the requests mark unavailable: their
pictures are undistorted (colour bilinear, depth nearest; the depth is z along the optical axis, the same for
both models) into a pinhole of ``wuji.PINHOLE_SIZE``, placed by the cameras' fixed mounts on the end links
(:data:`.wuji.CAMERA_MOUNT`).

Native runtimes (dexmanip on Newton, libuipc, MuJoCo) send no calibration and no depth, ever; Newton also no
``end_pose``: there it is computed from the joints (:func:`end_pose_from_joints`; it reproduced the reported one
to 0.00 mm on Isaac, libuipc and MuJoCo requests of the G2_crsB) and marked ``computed``.  Their cameras
get a static model per runtime (``parse(..., runtime=...)``, :data:`NATIVE_LENSES`): the head's lens with its
pose from our kinematics (the head's joints in the request on the mount :data:`HEAD_MOUNT`), the hands' lenses on
the same mounts as on Isaac (:data:`.wuji.CAMERA_MOUNT`); the head picture is undistorted into a pinhole with its
own intrinsics, the hand pictures into the pinhole of ``wuji.PINHOLE_SIZE`` (the same pictures as on Isaac).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from ...geometry import invert
from ...vision.cameras import Camera
from . import body
from . import kinematics
from . import protocol as P
from . import wuji

#: the benchmark's camera names -> ours
WIRE_CAMS = {"head": "head", "hand_left": "left_wrist", "hand_right": "right_wrist"}
CAMERAS = ("head", "left_wrist", "right_wrist")
#: the hand a wrist camera rides on
CAM_ARM = {"left_wrist": "left", "right_wrist": "right"}
#: the size the hand cameras are brought to
HAND_SIZE = (640, 528)
#: depth readings (m) outside this range are none
NEAR_M, FAR_M = 0.02, 8.0
#: the simulator's depth noise (every board's cameras): per pixel, independent, about uniform, with a
#: standard deviation of about this share of the depth (0.3 cm at 0.25 m, 1.5 cm at 1 m, 2.4 cm at 1.4 m)
DEPTH_NOISE_SHARE = 0.016
#: depth smoothing (:func:`smooth_depth`): the window (pixels) and how many noise deviations a neighbour's
#: reading may differ from the pixel's own and still count
SMOOTH_WINDOW = 7
SMOOTH_DEVIATIONS = 3.0
#: a colour picture whose brightest pixel is at most this is black: a dropped frame
BLACK_MAX = 12
#: (robot_type, ee_type) of the requests -> protocol.ROBOTS key
ROBOT_OF_WIRE = {(r.robot_type, r.ee_type): r.key for r in P.ROBOTS.values()}


class ObsError(ValueError):
    """A request that cannot be read."""


@dataclass
class Calib:
    k: np.ndarray  # 3x3 at the picture's own size (after normalising)
    pose: np.ndarray  # 4x4 optical frame (x right, y down, z forward) in base_link
    width: int
    height: int
    snapshot: str = ""
    carried: bool = False  # not in this request: the last one seen, moved with what carries the camera


@dataclass
class Frame:
    """One request's contents.  ``seq`` counts requests over the server's life."""

    seq: int
    t: float
    task_name: str
    episode_idx: int | None
    prompt: str
    done: bool
    segments: Any
    robot: str  # protocol.ROBOTS key
    joints: np.ndarray  # (2 n,) left n, right n
    effectors: np.ndarray  # (2 m,) left m, right m
    waist: np.ndarray | None
    head: np.ndarray | None
    chassis: np.ndarray | None
    end_pose: dict
    arm_base: np.ndarray  # T base_link <- arm_base_link
    images: dict[str, np.ndarray] = field(default_factory=dict)  # cam -> RGB uint8
    depth: dict[str, np.ndarray] = field(default_factory=dict)  # cam -> float32 metres (nan: none)
    calib: dict[str, Calib] = field(default_factory=dict)
    dropped: set[str] = field(default_factory=set)  # cams whose colour picture is black (a dropped frame)
    bytes: int = 0
    timestamp: Any = None
    robot_type: str = ""
    ee_type: str = ""
    #: the simulated seconds since the episode began (MuJoCo's requests: ``simulation_time``), or None
    sim_time: float | None = None

    @property
    def end_pose_computed(self) -> bool:
        """Whether ``end_pose`` came from our kinematics (the request had none)."""
        return bool(isinstance(self.end_pose, dict) and self.end_pose.get("computed"))

    @property
    def arm_joint_count(self) -> int:
        return len(self.joints) // 2

    def arm_joints(self, arm: str) -> np.ndarray:
        n = self.arm_joint_count
        k = kinematics.ARMS.index(arm)
        return self.joints[n * k: n * k + n].copy()

    def effector(self, arm: str) -> np.ndarray:
        m = len(self.effectors) // 2
        k = kinematics.ARMS.index(arm)
        return self.effectors[m * k: m * k + m].copy()

    def gripper(self, arm: str) -> float:
        """A two-finger gripper's reading (its one value)."""
        return float(self.effector(arm)[0])

    def camera(self, cam: str) -> Camera | None:
        c = self.calib.get(cam)
        if c is None:
            return None
        return Camera.from_cv(cam, c.k, c.pose, c.width, c.height)

    def has_depth(self, cams=CAMERAS) -> bool:
        return all(c in self.depth for c in cams if c in self.images)

    def end_link(self, arm: str) -> np.ndarray:
        """arm_*_end_link in base_link, as reported."""
        p = self.end_pose["base_link"][f"{arm}_arm"]
        return kinematics.pose_of(p["position"], p["orientation"])


# -- pictures --------------------------------------------------------------------------------------
_maps_lock = threading.Lock()
_maps: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}


def _undistort_maps(k: np.ndarray, dist: np.ndarray, w: int, h: int):
    key = (w, h, tuple(np.round(k.ravel(), 6)), tuple(np.round(dist.ravel(), 8)))
    with _maps_lock:
        maps = _maps.get(key)
        if maps is None:
            maps = cv2.initUndistortRectifyMap(k, dist, None, k, (w, h), cv2.CV_32FC1)
            if len(_maps) > 64:
                _maps.clear()
            _maps[key] = maps
        return maps


def _fisheye_maps(w: int, h: int):
    """Maps from the Wuji hand camera's fisheye picture (``w`` x ``h``: its own size or a scaled one) to the
    pinhole of wuji.PINHOLE_SIZE, and that pinhole's intrinsics."""
    key = ("wuji", w, h)
    with _maps_lock:
        got = _maps.get(key)
        if got is None:
            sx, sy = w / wuji.CAMERA_SIZE[0], h / wuji.CAMERA_SIZE[1]
            fx, fy, cx, cy = wuji.CAMERA_K
            k = np.array([[fx * sx, 0.0, cx * sx], [0.0, fy * sy, cy * sy], [0.0, 0.0, 1.0]])
            pw, ph = wuji.PINHOLE_SIZE
            k_new = np.array([[wuji.PINHOLE_F, 0.0, (pw - 1) / 2.0], [0.0, wuji.PINHOLE_F, (ph - 1) / 2.0],
                              [0.0, 0.0, 1.0]])
            mx, my = cv2.fisheye.initUndistortRectifyMap(k, np.asarray(wuji.CAMERA_D, float).reshape(4, 1), np.eye(3),
                                                         k_new, (pw, ph), cv2.CV_32FC1)
            got = (mx, my, k_new)
            if len(_maps) > 64:
                _maps.clear()
            _maps[key] = got
        return got


def _wuji_hand(cam: str, frame: "Frame", rgb: np.ndarray | None, depth: np.ndarray | None):
    """A Wuji hand camera's picture as a pinhole one: (rgb, depth, its calibration)."""
    h, w = (rgb if rgb is not None else depth).shape[:2]
    mx, my, k = _fisheye_maps(w, h)
    if rgb is not None:
        rgb = cv2.remap(rgb, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    if depth is not None:
        depth = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    pw, ph = wuji.PINHOLE_SIZE
    pose = frame.end_link(CAM_ARM[cam]) @ wuji.camera_mount(CAM_ARM[cam])
    return rgb, depth, Calib(k=k, pose=pose, width=pw, height=ph, snapshot="wuji_fixed_mount")


# -- the native runtimes' cameras (their requests carry no calibration) ------------------------------------
@dataclass(frozen=True)
class Lens:
    """A camera's lens as a runtime renders it: OpenCV ``fisheye`` (``d`` = k1..k4) or ``pinhole`` (``d`` = k1, k2,
    p1, p2, k3), the intrinsics ``k`` (fx, fy, cx, cy) at the picture size ``size`` (w, h)."""

    model: str
    size: tuple[int, int]
    k: tuple[float, float, float, float]
    d: tuple[float, ...]

    def matrix(self, w: int, h: int) -> np.ndarray:
        """The intrinsics at a picture of w x h (scaled from ``size``)."""
        fx, fy, cx, cy = self.k
        sx, sy = w / self.size[0], h / self.size[1]
        return np.array([[fx * sx, 0.0, cx * sx], [0.0, fy * sy, cy * sy], [0.0, 0.0, 1.0]])


#: the head camera's mount on ``head_link3``, the same on every runtime: its centre (m) and the rotation (w, x, y,
#: z) of the USD camera frame (x right, y up, looking along -z; the optical frame is it turned half round x).
#: Source: genie_sim 3.6.0 geniesim_benchmark/.../benchmark/config/llm_task/wuji_ipc/cameras.yaml
#: (``extrinsic``); the same in genie_sim_bringup/config/launcher_newton_wuji_eval.yaml (``eye``/``look_at``/``up``)
#: and in backends/mujoco/rubik/camera_calibration.json (``position``/``forward``/``up``), read as data.
#: Our head chain on it reproduced the head optical frame of the Isaac requests to 0.0 mm.
HEAD_MOUNT = ((0.10237, 0.02375, 0.10256),
              (-0.5945104121873722, -0.38683164617055127, 0.37872914728340196, 0.5945442627086477))
#: the head's intrinsics at 640 x 400 on every runtime (the Isaac calibration's too); cameras.yaml ``intrinsic``
HEAD_K = (306.69110107421875, 306.5507507324219, 319.90093994140625, 201.29141235351562)
#: libuipc and Newton: cameras.yaml (wuji_ipc) and launcher_newton_wuji_eval.yaml ``newton_rgb_views_json`` (the
#: same numbers; Newton's Isaac renderer authors them as the OpenCV lens, genie_sim_render
#: scripts/_isaac_snapshot_scene.py): the head an OpenCV pinhole with only p2, the hands an OpenCV fisheye of
#: 960 x 768 (half the Isaac hand cameras' 1920 x 1536, wuji.CAMERA_K)
_HEAD_IPC = Lens("pinhole", (640, 400), HEAD_K, (0.0, 0.0, 0.0, 3.2653999824105995e-06, 0.0))
_HAND_IPC = Lens("fisheye", (960, 768),
                 (363.1182861328125, 363.16827392578125, 482.89935302734375, 372.7657470703125),
                 (-0.002479762304574251, -6.387999746948481e-07, -0.0013551259180530906, 0.00023899998632259667))
#: MuJoCo (rubik): backends/mujoco/rubik/camera_calibration.json, rendered as OpenCV pinholes with that
#: distortion (rubik/isaac_worker.py): the head 640 x 400, the hands 1280 x 1056, each hand its own
_HEAD_MUJOCO = Lens("pinhole", (640, 400), HEAD_K,
                    (-0.031128238886594772, 0.03271407261490822, 0.0002466687001287937, 3.2653999824105995e-06,
                     -0.011283845640718937))
_LEFT_MUJOCO = Lens("pinhole", (1280, 1056),
                    (486.1373291015625, 485.9415283203125, 614.3196411132812, 529.999755859375),
                    (-0.04644061625003815, -0.011465538293123245, 0.00033718859776854515, -0.00019672630878631026,
                     0.002211029641330242))
_RIGHT_MUJOCO = Lens("pinhole", (1280, 1056),
                     (465.1792907714844, 465.0162048339844, 630.6480102539062, 527.8828125),
                     (-0.036002930253744125, -0.014382577501237392, -0.00022836736752651632, -0.00030626903753727674,
                      0.0026454415637999773))
#: per native runtime, each camera's lens (the hands sit on wuji.CAMERA_MOUNT on all three: the files' mounts
#: agree with it to 0.05 mm and 0.001 degrees)
NATIVE_LENSES: dict[str, dict[str, Lens]] = {
    "libuipc": {"head": _HEAD_IPC, "left_wrist": _HAND_IPC, "right_wrist": _HAND_IPC},
    "newton": {"head": _HEAD_IPC, "left_wrist": _HAND_IPC, "right_wrist": _HAND_IPC},
    "mujoco": {"head": _HEAD_MUJOCO, "left_wrist": _LEFT_MUJOCO, "right_wrist": _RIGHT_MUJOCO},
}


def _lens_maps(lens: Lens, w: int, h: int, k_new: np.ndarray, out_size: tuple[int, int]):
    """Maps from a picture of w x h through ``lens`` to a pinhole of intrinsics ``k_new`` and size ``out_size``."""
    key = ("lens", lens, w, h, tuple(np.round(k_new.ravel(), 6)), out_size)
    with _maps_lock:
        got = _maps.get(key)
        if got is None:
            k = lens.matrix(w, h)
            if lens.model == "fisheye":
                got = cv2.fisheye.initUndistortRectifyMap(k, np.asarray(lens.d, float).reshape(4, 1), np.eye(3), k_new,
                                                          out_size, cv2.CV_32FC1)
            else:
                got = cv2.initUndistortRectifyMap(k, np.asarray(lens.d, float), None, k_new, out_size, cv2.CV_32FC1)
            if len(_maps) > 64:
                _maps.clear()
            _maps[key] = got
        return got


def native_head_pose(frame: "Frame") -> np.ndarray | None:
    """The head camera's optical frame in base_link from the head's joints in the request (None without them)."""
    if frame.head is None or len(frame.head) < 3:
        return None
    (x, y, z), (qw, qx, qy, qz) = HEAD_MOUNT
    mount = kinematics.pose_of((x, y, z), (qx, qy, qz, qw)) @ np.diag([1.0, -1.0, -1.0, 1.0])
    return frame.arm_base @ body.head_in_arm_base(frame.head[:3]) @ mount


def _native_camera(cam: str, lens: Lens, runtime: str, frame: "Frame", rgb: np.ndarray | None,
                   depth: np.ndarray | None):
    """A native runtime's picture through its static model: (rgb, depth, its calibration or None).  The head comes
    out a pinhole with its own intrinsics, a hand camera the pinhole of wuji.PINHOLE_SIZE."""
    h, w = (rgb if rgb is not None else depth).shape[:2]
    if cam in CAM_ARM:
        pw, ph = wuji.PINHOLE_SIZE
        k = np.array([[wuji.PINHOLE_F, 0.0, (pw - 1) / 2.0], [0.0, wuji.PINHOLE_F, (ph - 1) / 2.0], [0.0, 0.0, 1.0]])
        size = (pw, ph)
        pose = frame.end_link(CAM_ARM[cam]) @ wuji.camera_mount(CAM_ARM[cam])
    else:
        k = lens.matrix(w, h)
        size = (w, h)
        pose = native_head_pose(frame)
    if lens.model == "fisheye" or any(abs(v) > 0 for v in lens.d) or size != (w, h):
        mx, my = _lens_maps(lens, w, h, k, size)
        if rgb is not None:
            rgb = cv2.remap(rgb, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        if depth is not None:
            depth = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)
    if pose is None:
        return rgb, depth, None
    return rgb, depth, Calib(k=k, pose=pose, width=size[0], height=size[1], snapshot=f"{runtime}_static_model")


def _k_of(cal: dict) -> np.ndarray:
    return np.array([[float(cal["fx"]), 0.0, float(cal["cx"])], [0.0, float(cal["fy"]), float(cal["cy"])],
                     [0.0, 0.0, 1.0]])


def _usable(cal: dict | None) -> bool:
    """The calibration's numbers hold for this capture: available, or checked against the render and
    marked unavailable only because the colour pixels were perturbed."""
    if cal is None or "fx" not in cal or cal.get("optical_to_frame") is None:
        return False
    return bool(cal.get("available", True)) or bool(cal.get("pose_checked_against_render"))


def _decode_jpeg(data: bytes) -> np.ndarray:
    arr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if arr is None:
        raise ObsError("a picture could not be decoded")
    return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)


def _picture(entry: dict) -> np.ndarray | None:
    data = entry.get("image_data")
    if not data:
        return None
    enc = str(entry.get("encoding") or "").upper()
    if enc in ("JPEG", "JPG", "PNG"):
        return _decode_jpeg(data)
    if enc in ("RAW_UINT16", "UINT16"):
        h, w = int(entry["height"]), int(entry["width"])
        return np.frombuffer(data, dtype=np.uint16).reshape(h, w)
    if enc in ("RAW_RGB", "RGB8", "RAW"):
        h, w = int(entry["height"]), int(entry["width"])
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3).copy()
    raise ObsError(f"unknown picture encoding {enc!r}")


def smooth_depth(depth: np.ndarray) -> np.ndarray:
    """The depth picture (float metres, nan: none) with its per-pixel noise evened out; none stays none.

    Each reading becomes the mean of the readings in the SMOOTH_WINDOW around it that differ from it by at most
    SMOOTH_DEVIATIONS noise deviations at its depth.  A floor or table top comes out about five times smoother
    (the floor 1.4 m below the WBC head camera: a standard deviation of 2.3 cm before, 0.5 cm after), while the
    edge between things far apart in depth (an arm over the floor) stays where it was.  A step smaller than the
    noise (a 2 cm lid seen from 1.4 m) is blurred over the window, as the noise blurs it anyway."""
    d = np.asarray(depth, np.float32)
    h, w = d.shape
    pad = SMOOTH_WINDOW // 2
    padded = np.pad(d, pad, constant_values=np.nan)
    tol = np.float32(SMOOTH_DEVIATIONS * DEPTH_NOISE_SHARE) * d
    total = np.zeros_like(d)
    count = np.zeros_like(d)
    with np.errstate(invalid="ignore"):
        for dy in range(SMOOTH_WINDOW):
            for dx in range(SMOOTH_WINDOW):
                near = padded[dy:dy + h, dx:dx + w]
                ok = np.abs(near - d) <= tol
                total += np.where(ok, near, np.float32(0.0))
                count += ok
    out = total / np.maximum(count, np.float32(1.0))
    out[count == 0] = np.nan  # only where there was no reading
    return out


def _normalise(cam: str, rgb: np.ndarray | None, depth: np.ndarray | None, k: np.ndarray):
    """The hand cameras at HAND_SIZE: halve a 1280x1056 picture (any other size is kept)."""
    if cam == "head":
        return rgb, depth, k
    w0, h0 = HAND_SIZE
    shape = (rgb if rgb is not None else depth).shape
    h, w = shape[0], shape[1]
    if (w, h) == (w0, h0) or (w, h) != (2 * w0, 2 * h0):
        return rgb, depth, k
    k2 = k.copy()
    k2[:2] /= 2.0
    if rgb is not None:
        rgb = cv2.resize(rgb, (w0, h0), interpolation=cv2.INTER_AREA)
    if depth is not None:
        depth = depth[::2, ::2].copy()
    return rgb, depth, k2


#: robots whose head camera is carried by the head's joints (read in each request) rather than by arm_base_link
HEAD_ON_JOINTS = ("g2_90d",)


class Calibrations:
    """The calibrations an episode has seen, for its requests that carry none."""

    def __init__(self):
        self.last: dict[str, Calib] = {}
        #: a camera's pose on what carries it: its hand's end link (wrist cameras), arm_base_link (the head; the
        #: G2_crsB's head link, HEAD_ON_JOINTS)
        self.mount: dict[str, np.ndarray] = {}

    def carrier(self, cam: str, frame: "Frame") -> np.ndarray:
        if cam in CAM_ARM:
            return frame.end_link(CAM_ARM[cam])
        if cam == "head" and frame.robot in HEAD_ON_JOINTS and frame.head is not None and len(frame.head) >= 3:
            # the G2_crsB's head turns and tips on the torso (the WBC board's robot.head): the camera rides on it
            return frame.arm_base @ body.head_in_arm_base(frame.head[:3])
        return frame.arm_base

    def seen(self, cam: str, calib: Calib, frame: "Frame") -> None:
        self.last[cam] = calib
        self.mount[cam] = invert(self.carrier(cam, frame)) @ calib.pose

    def carried(self, cam: str, frame: "Frame", width: int, height: int) -> Calib | None:
        prev = self.last.get(cam)
        if prev is None:
            return None
        k = prev.k.copy()
        if (prev.width, prev.height) != (width, height):
            k[0] *= width / prev.width
            k[1] *= height / prev.height
        pose = self.carrier(cam, frame) @ self.mount[cam]
        return Calib(k=k, pose=pose, width=width, height=height, snapshot=prev.snapshot, carried=True)


#: the arm chain of robots without an arm model of their own yet (only their end links are computed: the
#: chain to the end link does not depend on the hand)
CHAIN_OF_ROBOT: dict[str, str] = {}
_CHAINS: dict[str, dict[str, kinematics.Arm]] = {}


def _chains(robot: str) -> dict[str, kinematics.Arm]:
    if robot in kinematics.MODEL_OF_ROBOT:
        return kinematics.arm_models(robot)
    if robot not in CHAIN_OF_ROBOT:
        raise ObsError(f"no arm chain for robot {robot!r}")
    if robot not in _CHAINS:
        _CHAINS[robot] = {arm: kinematics.Arm(arm, CHAIN_OF_ROBOT[robot], kinematics.CTEK90D)
                          for arm in kinematics.ARMS}
    return _CHAINS[robot]


def end_pose_from_joints(robot: str, joints: np.ndarray, waist: np.ndarray | None) -> dict:
    """An ``end_pose`` like the benchmark's (each arm's end link in ``base_link`` and in ``arm_base_link``,
    position and xyzw orientation) from the arm joints and the five body joints, marked ``computed``."""
    if waist is None or len(waist) != 5:
        raise ObsError("no end_pose and no five waist readings to compute it from")
    chains = _chains(robot)
    arm_base = body.arm_base(waist)
    out: dict[str, Any] = {"base_link": {}, "arm_base_link": {}, "computed": True}
    n = len(joints) // 2
    for k, arm in enumerate(kinematics.ARMS):
        e_ab = chains[arm].end_pose(joints[n * k:n * k + n])
        e_b = arm_base @ e_ab
        out["base_link"][f"{arm}_arm"] = {"position": [float(v) for v in e_b[:3, 3]],
                                         "orientation": kinematics.quat_xyzw_of(e_b[:3, :3])}
        out["arm_base_link"][f"{arm}_arm"] = {"position": [float(v) for v in e_ab[:3, 3]],
                                             "orientation": kinematics.quat_xyzw_of(e_ab[:3, :3])}
    return out


def robot_of(params: dict) -> str:
    key = ROBOT_OF_WIRE.get((str(params.get("robot_type") or ""), str(params.get("ee_type") or "")))
    if key is None:
        raise ObsError(f"unknown robot {params.get('robot_type')!r} / {params.get('ee_type')!r}")
    return key


def _seconds(value) -> float | None:
    """A finite number of seconds, or None."""
    try:
        t = float(value)
    except (TypeError, ValueError):
        return None
    return t if math.isfinite(t) else None


def parse(payload: dict, seq: int, calibrations: Calibrations | None = None, *, smooth: bool = False,
          runtime: str = "isaac") -> Frame:
    """A Frame from one request (raises ObsError for one that is not an inference request); ``smooth``: the
    depth pictures smoothed (:func:`smooth_depth`); ``runtime``: the benchmark's physics runtime (a native one's
    cameras without calibration get its static model, :data:`NATIVE_LENSES`)."""
    lenses = NATIVE_LENSES.get(runtime, {})
    if not isinstance(payload, dict) or payload.get("method", "infer") != "infer":
        raise ObsError("not an inference request")
    p = payload.get("params")
    if not isinstance(p, dict):
        raise ObsError("no params")
    robot = P.ROBOTS[robot_of(p)]
    st = p.get("states") or {}
    def vec(name):
        v = st.get(name)
        return None if v is None or len(v) == 0 else np.asarray(v, float).reshape(-1)

    try:
        joints = np.asarray(st["arm_joint_states"], float).reshape(2 * robot.arm_joints)
        effectors = np.asarray(st["gripper_states"], float).reshape(2 * robot.effector)
        end_pose = st["end_pose"]
        if end_pose is None:  # Newton
            end_pose = end_pose_from_joints(robot.key, joints, vec("waist_joint_states"))
        arm_base = kinematics.arm_base_from_obs(end_pose)
    except (KeyError, TypeError, ValueError) as exc:
        raise ObsError(f"the robot's readings are incomplete: {exc}") from None

    frame = Frame(seq=seq, t=time.time(), task_name=str(p.get("task_name") or ""),
                  episode_idx=p.get("episode_idx"), prompt=str(p.get("prompt") or ""),
                  done=bool(p.get("episode_done") is True), segments=p.get("segments"), robot=robot.key,
                  joints=joints, effectors=effectors, waist=vec("waist_joint_states"), head=vec("head_joint_states"),
                  chassis=vec("chassis_states"), end_pose=end_pose, arm_base=arm_base,
                  timestamp=p.get("timestamps") or p.get("timestamp"), robot_type=str(p.get("robot_type") or ""),
                  ee_type=str(p.get("ee_type") or ""), sim_time=_seconds(p.get("simulation_time")))
    images = p.get("images") or {}
    for wire, cam in WIRE_CAMS.items():
        entry = images.get(wire)
        if not isinstance(entry, dict):
            continue
        rgb = _picture(entry)
        dent = images.get(f"{wire}_depth")
        raw_depth = _picture(dent) if isinstance(dent, dict) else None
        cal = entry.get("calibration") if isinstance(entry.get("calibration"), dict) else None
        if rgb is None and raw_depth is None:
            continue
        h, w = (rgb if rgb is not None else raw_depth).shape[:2]
        depth = None if raw_depth is None else raw_depth.astype(np.float32) * float(
            (cal or {}).get("depth_scale_m") or 0.001)
        if rgb is not None and rgb.ndim == 3 and int(rgb.max()) <= BLACK_MAX:
            frame.dropped.add(cam)
        calib = None
        if cam in lenses and not _usable(cal):
            rgb, depth, calib = _native_camera(cam, lenses[cam], runtime, frame, rgb, depth)
        elif robot.key == "g2_wuji" and cam in CAM_ARM and not _usable(cal):
            rgb, depth, calib = _wuji_hand(cam, frame, rgb, depth)
        elif _usable(cal):
            k = _k_of(cal)
            cw, ch = int(cal.get("width") or w), int(cal.get("height") or h)
            if (cw, ch) != (w, h):  # intrinsics at another size: scale them
                k[0] *= w / cw
                k[1] *= h / ch
            dist = np.asarray(cal.get("distortion") or [], float)
            if dist.size and np.any(np.abs(dist) > 0):
                mx, my = _undistort_maps(k, dist, w, h)
                if rgb is not None:
                    rgb = cv2.remap(rgb, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                if depth is not None:
                    depth = cv2.remap(depth, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                                      borderValue=0.0)
            rgb, depth, k = _normalise(cam, rgb, depth, k)
            hh, ww = (rgb if rgb is not None else depth).shape[:2]
            pose = np.asarray(cal["optical_to_frame"], float).reshape(4, 4)
            calib = Calib(k=k, pose=pose, width=ww, height=hh, snapshot=str(cal.get("snapshot_id") or ""))
            if calibrations is not None:
                calibrations.seen(cam, calib, frame)
        else:
            rgb, depth, _ = _normalise(cam, rgb, depth, np.eye(3))
            hh, ww = (rgb if rgb is not None else depth).shape[:2]
            if calibrations is not None:
                calib = calibrations.carried(cam, frame, ww, hh)
        if rgb is not None:
            frame.images[cam] = np.ascontiguousarray(rgb)
        if depth is not None:
            d = depth.astype(np.float32)
            d[(d <= NEAR_M) | (d >= FAR_M)] = np.nan
            frame.depth[cam] = smooth_depth(d) if smooth else d
        if calib is not None:
            frame.calib[cam] = calib
    return frame


def summary(frame: Frame) -> dict:
    """A short record of a frame (for states.jsonl)."""
    out = {"seq": frame.seq, "joints": [round(float(v), 4) for v in frame.joints],
           "effectors": [round(float(v), 4) for v in frame.effectors]}
    if frame.waist is not None:
        out["waist"] = [round(float(v), 4) for v in frame.waist]
    for arm in kinematics.ARMS:
        e = frame.end_link(arm)
        out[f"{arm}_end"] = [round(float(v), 4) for v in e[:3, 3]]
    out["depth"] = sorted(frame.depth)
    if frame.sim_time is not None:
        out["sim_time"] = round(frame.sim_time, 3)
    return out


def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
