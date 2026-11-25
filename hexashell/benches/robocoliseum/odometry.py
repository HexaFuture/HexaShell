"""How far the base went in a drive, from the head camera's pictures before and after it.

Nothing in the observations reads the chassis (the simulator reports it as zero at every request), so a
drive's reply could only repeat what was asked - and in the first WBC episodes drives next to a door left the
base where it was, one into a shutter turned it away, and one with a tray turned it about 70 degrees, all
reported "as asked".  The head camera rides on the torso, which keeps its posture through a drive, and every
request after a motion carries depth: the static things seen before and after a drive, matched by their looks
(ORB features of the colour pictures, away from the robot's own pixels and from depth edges) and lifted to 3-D
with the depth pictures, give the base's motion - the turn about the vertical and the shift along the floor
that carry the after-drive points onto the before-drive ones (RANSAC over pairs of matches, then a least-
squares fit on the points that agree).

The depth pictures are noisy, more so the farther (a standard deviation of about 1.5 cm at 1 m and 2.4 cm at
1.4 m, where the WBC robot's head sees the floor): features take the mean depth of a small window, and edges
are judged on that smoothed depth.  In a WBC probe (pick_up_trash, a learning instance) the reading
matched the simulator's own chassis tracking within 5 mm and 0.15 degrees on open floor, and read "hardly
moved" for a drive against a wall.

A turn that leaves the head camera too little of what it saw before (beyond about 60 degrees with this
camera), a bare wall filling the picture or a scene that moves as a whole gives no estimate (None).
"""

from __future__ import annotations

import math
import os
import pickle
import time
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np

from . import mask as M

#: ORB features per picture; the ratio test of the two best matches
FEATURES = 2000
RATIO = 0.8
#: depth (m) a feature may have: nearer is the robot or a blur, farther too coarse
NEAR_M, FAR_M = 0.3, 7.0
#: the depth pictures are noisy (in the WBC scenes each pixel off by up to about 4 cm, a standard deviation of
#: about 2.3 cm on a flat floor): a feature's depth is the mean of this window (px) around it, and it has none
#: where the window holds a hole or a depth edge (the smoothed depth spans more than EDGE_SHARE of itself)
WINDOW_PX = 7
EDGE_SHARE = 0.1
#: points nearer than this to the base's vertical axis (m, in base_link) belong to the robot
BODY_RADIUS_M = 0.45
#: a match agrees with a move when it lands within AGREE_M + AGREE_PER_M per metre of depth
AGREE_M = 0.03
AGREE_PER_M = 0.02
#: pairs of matches tried, and their least spread on the floor (m)
TRIES = 400
SPREAD_M = 0.25
#: matches that must agree for an estimate, and their least share of all matches
MIN_AGREE = 15
MIN_SHARE = 0.2
#: a directory to keep the frames before and after each drive in, for replaying them offline (a probe's
#: setting; unset or empty: none kept)
DUMP_ENV = "HEXASHELL_DRIVE_DUMP"


class BaseMove(NamedTuple):
    forward: float  # m, along the base's x before the drive
    left: float  # m, along its y
    turn_deg: float  # + to the left
    agree: int  # matches that agree with it
    matched: int  # matches tried

    def text(self) -> str:
        return (f"forward {self.forward:.2f} m, left {self.left:.2f} m, turned {self.turn_deg:+.0f}° "
                f"({self.agree} of {self.matched} matched points agree)")


class _Seen(NamedTuple):
    desc: np.ndarray  # ORB descriptors (N, 32)
    points: np.ndarray  # (N, 3) in base_link
    depth: np.ndarray  # (N,) m from the camera


def _seen(image, depth, camera, robot, info: dict | None = None) -> _Seen | None:
    """The head picture's features that lie on still, solid things, with their 3-D points in base_link
    (``info``, when given, gets the counts on the way)."""
    info = {} if info is None else info
    if image is None or depth is None or camera is None:
        info["missing"] = [n for n, v in (("image", image), ("depth", depth), ("camera", camera)) if v is None]
        return None
    grey = cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_RGB2GRAY) if np.ndim(image) == 3 else \
        np.ascontiguousarray(image, np.uint8)
    d = np.asarray(depth, np.float32)
    h, w = d.shape
    ok = np.isfinite(d) & (d > NEAR_M) & (d < FAR_M)
    if robot is not None:
        ok &= ~np.asarray(robot, bool)
    allowed = cv2.erode(ok.astype(np.uint8) * 255, np.ones((WINDOW_PX + 2, WINDOW_PX + 2), np.uint8))
    info["usable_px"] = int(np.count_nonzero(allowed))
    kps, desc = cv2.ORB_create(FEATURES).detectAndCompute(grey, allowed)
    info["features"] = 0 if desc is None else len(kps)
    if desc is None or len(kps) < MIN_AGREE:
        return None
    z_map, flat = _smoothed(d)
    uv = np.array([k.pt for k in kps], float)
    us = np.clip(np.round(uv[:, 0]).astype(int), 0, w - 1)
    vs = np.clip(np.round(uv[:, 1]).astype(int), 0, h - 1)
    z = z_map[vs, us]
    good = flat[vs, us] & (z > NEAR_M) & (z < FAR_M)
    info["solid"] = int(good.sum())
    local = np.stack([(uv[:, 0] - camera.cx) / camera.fx * z, -(uv[:, 1] - camera.cy) / camera.fy * z, -z], axis=1)
    pts = local @ np.asarray(camera.pose, float)[:3, :3].T + np.asarray(camera.pose, float)[:3, 3]
    good &= np.hypot(pts[:, 0], pts[:, 1]) > BODY_RADIUS_M
    info["off_body"] = int(good.sum())
    if good.sum() < MIN_AGREE:
        return None
    return _Seen(desc[good], pts[good], z[good])


def _smoothed(depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The depth picture's mean over WINDOW_PX windows, and where it is flat: no hole in the window and no edge
    (the mean spans less than EDGE_SHARE of itself over the window)."""
    d = np.asarray(depth, np.float32)
    valid = np.isfinite(d) & (d > NEAR_M) & (d < FAR_M)
    size = (WINDOW_PX, WINDOW_PX)
    count = cv2.boxFilter(valid.astype(np.float32), -1, size, normalize=False, borderType=cv2.BORDER_CONSTANT)
    total = cv2.boxFilter(np.where(valid, d, 0.0).astype(np.float32), -1, size, normalize=False,
                          borderType=cv2.BORDER_CONSTANT)
    mean = total / np.maximum(count, 1.0)
    kernel = np.ones(size, np.uint8)
    spread = cv2.dilate(mean, kernel) - cv2.erode(mean, kernel)
    flat = (count > WINDOW_PX * WINDOW_PX - 0.5) & (spread < EDGE_SHARE * mean)
    return mean, flat


def _fit(p: np.ndarray, q: np.ndarray) -> tuple[float, np.ndarray]:
    """The turn (rad) and shift (2,) on the floor that carry q's (x, y) onto p's, least squares."""
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    a, b = p - pc, q - qc
    angle = math.atan2(float(np.sum(b[:, 0] * a[:, 1] - b[:, 1] * a[:, 0])), float(np.sum(b[:, 0] * a[:, 0] + b[:, 1] * a[:, 1])))
    c, s = math.cos(angle), math.sin(angle)
    return angle, pc - np.array([c * qc[0] - s * qc[1], s * qc[0] + c * qc[1]])


def _residuals(p, q, angle, shift) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    moved = np.stack([c * q[:, 0] - s * q[:, 1] + shift[0], s * q[:, 0] + c * q[:, 1] + shift[1]], axis=1)
    return np.hypot(moved[:, 0] - p[:, 0], moved[:, 1] - p[:, 1])


def estimate(before: _Seen | None, after: _Seen | None, *, seed: int = 0, info: dict | None = None) -> BaseMove | None:
    """The base's move from the head's features before and after it (None: they cannot tell; ``info``, when
    given, gets the counts on the way)."""
    info = {} if info is None else info
    if before is None or after is None:
        return None
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(after.desc, before.desc, k=2)
    idx = [(m[0].queryIdx, m[0].trainIdx) for m in pairs if len(m) == 2 and m[0].distance < RATIO * m[1].distance]
    info["matched"] = len(idx)
    if len(idx) < MIN_AGREE:
        return None
    qi, pi = np.array(idx).T
    q, p = after.points[qi], before.points[pi]
    # the base stays on the floor and the torso keeps its posture: a point keeps its height
    level = np.abs(q[:, 2] - p[:, 2]) < AGREE_M + AGREE_PER_M * np.maximum(after.depth[qi], before.depth[pi])
    q, p = q[level], p[level]
    rng_m = np.maximum(after.depth[qi], before.depth[pi])[level]
    n = len(q)
    info["level"] = n
    if n < MIN_AGREE:
        return None
    tol = AGREE_M + AGREE_PER_M * rng_m
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(TRIES):
        i, j = rng.choice(n, 2, replace=False)
        if math.hypot(*(q[i, :2] - q[j, :2])) < SPREAD_M:
            continue
        angle, shift = _fit(p[[i, j], :2], q[[i, j], :2])
        agree = _residuals(p[:, :2], q[:, :2], angle, shift) < tol
        if best is None or agree.sum() > best.sum():
            best = agree
    info["ransac_agree"] = None if best is None else int(best.sum())
    if best is None or best.sum() < MIN_AGREE:
        return None
    for _ in range(2):
        angle, shift = _fit(p[best, :2], q[best, :2])
        best = _residuals(p[:, :2], q[:, :2], angle, shift) < tol
        if best.sum() < MIN_AGREE:
            return None
    info["agree"] = int(best.sum())
    if best.sum() < MIN_SHARE * n:
        return None
    return BaseMove(float(shift[0]), float(shift[1]), math.degrees(angle), int(best.sum()), n)


def seen_by_head(frame, cam: str = "head", info: dict | None = None) -> _Seen | None:
    """The features of a frame's head picture (None without its picture, depth or calibration)."""
    info = {} if info is None else info
    image, depth, camera = frame.images.get(cam), frame.depth.get(cam), frame.camera(cam)
    if cam in frame.dropped:
        info["missing"] = ["image (a dropped frame)"]
        return None
    robot = None
    if image is not None and depth is not None and camera is not None:
        try:
            robot = M.robot_mask(camera, depth, M.frame_arms(frame))
            info["robot_px"] = int(np.count_nonzero(robot))
        except Exception as exc:  # noqa: BLE001 - without the arms' readings, the body radius keeps the robot out
            info["robot_mask"] = f"{type(exc).__name__}: {exc}"[:200]
    return _seen(image, depth, camera, robot, info)


def base_move(before, after, info: dict | None = None) -> BaseMove | None:
    """The base's move between two frames, from their head pictures (``info``, when given, gets the counts
    on the way: ``before``, ``after`` and the matching's)."""
    info = {} if info is None else info
    info["before"], info["after"] = {}, {}
    return estimate(seen_by_head(before, info=info["before"]), seen_by_head(after, info=info["after"]), info=info)


def dump(before, after, n: int, info: dict) -> None:
    """Keep the frames before and after the n-th drive (pickled) where DUMP_ENV says; nothing when unset."""
    folder = os.environ.get(DUMP_ENV, "").strip()
    if not folder:
        return
    try:
        path = Path(folder) / f"drive_{n:03d}_{time.strftime('%H%M%S')}.pkl"
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as fh:
            pickle.dump({"before": before, "after": after, "info": info}, fh, protocol=pickle.HIGHEST_PROTOCOL)
        info["dump"] = str(path)
    except Exception as exc:  # noqa: BLE001 - a probe's extra, never the drive's
        info["dump_error"] = f"{type(exc).__name__}: {exc}"[:200]
