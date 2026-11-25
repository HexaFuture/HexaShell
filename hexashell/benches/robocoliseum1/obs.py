"""What one request of the benchmark brings, made ready for the robot calls.

A request (``{"method": "infer", "params": {...}}``, msgpack) carries the three cameras' colour pictures
(JPEG) with their calibration, their depth pictures when the reply before asked for them (uint16
millimetres, aligned with the colour), the robot's readings (both arms' seven joints, the grippers, the
waist, the head, the chassis, each hand's pose in ``base_link`` and in ``arm_base_link``), the instruction
(``prompt``), the episode's sub-steps (``segments``, some tasks), the sub-task's name and whether the
episode has just ended (``episode_done``).

Here:

- the pictures are undistorted (the calibration's OpenCV distortion; colour bilinear, depth nearest) into
  a pinhole camera with the same intrinsics, and the hand cameras, which come at 640x528 or 1280x1056 by
  task, are brought to 640x528 (the intrinsics halved, depth every second pixel);
- cameras are named ``head``, ``left_wrist`` and ``right_wrist`` (the benchmark's ``head``, ``hand_left``,
  ``hand_right``) and modelled as :class:`hexashell.vision.cameras.Camera` in ``base_link`` (x forward,
  y left, z up), which is the world frame of the robot calls;
- where the benchmark perturbs the colour pictures (noise, smudges, now and then a black picture for a
  dropped frame: the camera-quality tasks), it marks the calibration unavailable ("alignment not
  verified") but still sends its numbers, checked against that very render
  (``pose_checked_against_render``); only the colour pixels are perturbed - not the camera's geometry,
  not the depth - so those numbers are used.  A calibration that failed carries no numbers;
- a camera without calibration numbers in a request keeps the last calibration seen for it; a hand
  camera's pose is then carried along with its hand (the camera sits fixed on the gripper);
- a camera no request has brought a calibration for (the online simulator sends none) gets the nominal
  model of :mod:`.cameras` in every request: the local calibration's lens, the pose from the readings;
- a black colour picture (all pixels about 0) is a dropped frame: listed in ``Frame.dropped``.
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
from . import cameras, g2

#: the benchmark's camera names -> ours
WIRE_CAMS = {"head": "head", "hand_left": "left_wrist", "hand_right": "right_wrist"}
CAMERAS = ("head", "left_wrist", "right_wrist")
#: the hand a wrist camera rides on
CAM_ARM = {"left_wrist": "left", "right_wrist": "right"}
#: the size the hand cameras are brought to
HAND_SIZE = (640, 528)
#: depth readings (m) outside this range are none
NEAR_M, FAR_M = 0.02, 8.0
#: a colour picture whose brightest pixel is at most this is black: a dropped frame
BLACK_MAX = 12


class ObsError(ValueError):
    """A request that cannot be read."""


@dataclass
class Calib:
    k: np.ndarray  # 3x3 at the picture's own size (after normalising)
    pose: np.ndarray  # 4x4 optical frame (x right, y down, z forward) in base_link
    width: int
    height: int
    snapshot: str = ""
    nominal: bool = False  # not in this request: carried over


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
    joints: np.ndarray  # (14,) left 7, right 7
    grippers: np.ndarray  # (2,) left, right: -0.785 open .. 0 closed
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

    def arm_joints(self, arm: str) -> np.ndarray:
        k = g2.ARMS.index(arm)
        return self.joints[7 * k: 7 * k + 7].copy()

    def gripper(self, arm: str) -> float:
        return float(self.grippers[g2.ARMS.index(arm)])

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
        return g2.pose_of(p["position"], p["orientation"])


# -- pictures --------------------------------------------------------------------------------------
_maps_lock = threading.Lock()
_maps: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}
#: per camera, the latest calibration seen (for requests without one) and, for the hand cameras, the
#: camera's pose on its hand's end link
_last: dict[str, Calib] = {}
_on_hand: dict[str, np.ndarray] = {}
#: cameras a request has brought a calibration for (from then on a request without one carries it over;
#: a camera never calibrated by a request gets the nominal model of :mod:`.cameras`)
_seen_real: set[str] = set()


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


def parse(payload: dict, seq: int) -> Frame:
    """A Frame from one request (raises ObsError for one that is not an inference request)."""
    if not isinstance(payload, dict) or payload.get("method", "infer") != "infer":
        raise ObsError("not an inference request")
    p = payload.get("params")
    if not isinstance(p, dict):
        raise ObsError("no params")
    st = p.get("states") or {}
    try:
        joints = np.asarray(st["arm_joint_states"], float).reshape(14)
        grippers = np.asarray(st["gripper_states"], float).reshape(-1)[:2]
        end_pose = st["end_pose"]
        arm_base = g2.arm_base_from_obs(end_pose)
    except (KeyError, TypeError, ValueError) as exc:
        raise ObsError(f"the robot's readings are incomplete: {exc}") from None

    def vec(name):
        v = st.get(name)
        return None if v is None else np.asarray(v, float).reshape(-1)

    frame = Frame(seq=seq, t=time.time(), task_name=str(p.get("task_name") or ""),
                  episode_idx=p.get("episode_idx"), prompt=str(p.get("prompt") or ""),
                  done=bool(p.get("episode_done") is True), segments=p.get("segments"), joints=joints,
                  grippers=grippers, waist=vec("waist_joint_states"), head=vec("head_joint_states"),
                  chassis=vec("chassis_states"), end_pose=end_pose, arm_base=arm_base,
                  timestamp=p.get("timestamps") or p.get("timestamp"), robot_type=str(p.get("robot_type") or ""))
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
        if _usable(cal) and not cal.get("nominal"):
            _seen_real.add(cam)
        elif cal is None and cam not in _seen_real:
            # a simulator that sends no calibration at all (the online one): the nominal model
            cal = cameras.nominal(wire, arm_base=frame.arm_base, head=frame.head,
                                  end_links={arm: frame.end_link(arm) for arm in g2.ARMS},
                                  task_name=frame.task_name)
        h, w = (rgb if rgb is not None else raw_depth).shape[:2]
        depth = None if raw_depth is None else raw_depth.astype(np.float32) * float(
            (cal or {}).get("depth_scale_m") or 0.001)
        if rgb is not None and rgb.ndim == 3 and int(rgb.max()) <= BLACK_MAX:
            frame.dropped.add(cam)
        if _usable(cal):
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
            _last[cam] = calib
            if cam in CAM_ARM:
                _on_hand[cam] = invert(frame.end_link(CAM_ARM[cam])) @ pose
        else:
            prev = _last.get(cam)
            k = prev.k.copy() if prev is not None else None
            rgb, depth, _ = _normalise(cam, rgb, depth, k if k is not None else np.eye(3))
            hh, ww = (rgb if rgb is not None else depth).shape[:2]
            calib = None
            if prev is not None:
                pose = prev.pose
                if cam in _on_hand:
                    pose = frame.end_link(CAM_ARM[cam]) @ _on_hand[cam]
                if (prev.width, prev.height) != (ww, hh):
                    k[0] *= ww / prev.width
                    k[1] *= hh / prev.height
                calib = Calib(k=k, pose=pose, width=ww, height=hh, nominal=True)
        if rgb is not None:
            frame.images[cam] = np.ascontiguousarray(rgb)
        if depth is not None:
            d = depth.astype(np.float32)
            d[(d <= NEAR_M) | (d >= FAR_M)] = np.nan
            frame.depth[cam] = d
        if calib is not None:
            frame.calib[cam] = calib
    return frame


def hold_row(frame: Frame) -> dict:
    """The joints and grippers as read (for a hold before anything was commanded)."""
    return {"left": frame.arm_joints("left"), "right": frame.arm_joints("right"),
            "grip": {"left": frame.gripper("left"), "right": frame.gripper("right")}}


def summary(frame: Frame) -> dict:
    """A short record of a frame (for states.jsonl)."""
    out = {"seq": frame.seq, "joints": [round(float(v), 4) for v in frame.joints],
           "grippers": [round(float(v), 4) for v in frame.grippers]}
    if frame.waist is not None:
        out["waist"] = [round(float(v), 4) for v in frame.waist]
    for arm in g2.ARMS:
        e = frame.end_link(arm)
        out[f"{arm}_end"] = [round(float(v), 4) for v in e[:3, 3]]
    out["depth"] = sorted(frame.depth)
    return out


def angle_wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
