"""Which pixels of a picture show the robot's own arms and grippers, from the depth picture.

Each pixel with depth is back-projected to ``base_link``; it shows the robot when its point lies inside
the arms' shapes as the joint readings place them:

- the arm: capsules on the chain from ``arm_base_link`` through the joints to the end link
  (``CAPSULE_M`` radius, the wrist ``WRIST_M``);
- the gripper: a box in the end link's frame (``GRIPPER_BOX``: across the fingers, along the pads, along
  the approach) less the gap between the fingers (``SLOT``: what the fingers hold is not the robot), and
  the wrist camera's housing (a ball).

The mask is then widened by ``MARGIN_PX`` (the depth at the robot's edges blurs into the background).
Pixels without depth are not marked.  The shapes are rough on purpose: a little of what touches the
robot is marked with it.
"""

from __future__ import annotations

import cv2
import numpy as np

from ...vision.cameras import Camera
from . import g2

#: capsule radii (m): the arm's links, and the last segment (joint 7 to the end link)
CAPSULE_M = 0.065
WRIST_M = 0.055
#: the gripper in the end link's frame: |x| (the fingers' closing direction), |y| (along the pads), z range
#: (the approach; the fingertips reach g2.TIP_REACH)
GRIPPER_BOX = (0.075, 0.06, (-0.06, g2.TIP_REACH + 0.008))
#: the gap between the fingers, from this far along the approach: (start z, clearance to each finger)
SLOT = (0.085, 0.003)
#: the wrist camera's housing: a ball of this radius (m) about the camera (its pose comes with the pictures;
#: without it, at the left camera's place in its end link's frame, as measured)
HOUSING_M = 0.035
HOUSING_IN_END = (0.0012, -0.0898, 0.0661)
#: depth (m) used
DEPTH_RANGE = (0.02, 3.0)
MARGIN_PX = 3

#: the gripper's gap (mm) at its readings (-0.785 open .. 0 closed)
_GAP_READING = np.array([-0.785, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0])
_GAP_MM = np.array([67.6, 61.8, 54.3, 46.3, 37.7, 28.7, 19.4, 9.8, 0.0])


def gap_m(reading: float) -> float:
    """The gap between the finger pads (m) at a gripper reading."""
    return float(np.interp(float(reading), _GAP_READING, _GAP_MM)) / 1000.0


def skeleton(arm: str, q, arm_base: np.ndarray) -> np.ndarray:
    """The chain's points in base_link: arm_base_link, the joint frames (repeats left out), the end link."""
    model = g2.ARM_MODELS[arm]
    frames = model.frames(q)
    pts = [arm_base[:3, 3]] + [(arm_base @ f)[:3, 3] for f in frames] + [(arm_base @ frames[-1] @ model.end)[:3, 3]]
    out = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - out[-1]) > 1e-4:
            out.append(p)
    return np.array(out)


def _near_segment(pts: np.ndarray, a: np.ndarray, b: np.ndarray, r: float) -> np.ndarray:
    ab = b - a
    t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    d = pts - (a + t[:, None] * ab)
    return np.einsum("ij,ij->i", d, d) <= r * r


def inside_arm(pts: np.ndarray, arm: str, q, arm_base: np.ndarray, end_link: np.ndarray, reading: float,
               housing=None) -> np.ndarray:
    """Which of the points (N, 3, base_link) lie inside the arm's and its gripper's shapes (``housing``: the
    wrist camera's position in base_link, if known)."""
    out = np.zeros(len(pts), bool)
    chain = skeleton(arm, q, arm_base)
    # the gripper, in the end link's frame
    local = (pts - end_link[:3, 3]) @ end_link[:3, :3]
    hx, hy, (z0, z1) = GRIPPER_BOX
    box = (np.abs(local[:, 0]) <= hx) & (np.abs(local[:, 1]) <= hy) & (local[:, 2] >= z0) & (local[:, 2] <= z1)
    slot_z, clearance = SLOT
    half = max(gap_m(reading) / 2.0 - clearance, 0.0)
    box &= ~((np.abs(local[:, 0]) < half) & (local[:, 2] >= slot_z))
    out |= box
    if housing is None:
        d = local - np.asarray(HOUSING_IN_END)
    else:
        d = pts - np.asarray(housing, float)
    out |= np.einsum("ij,ij->i", d, d) <= HOUSING_M * HOUSING_M
    # the links
    for k in range(len(chain) - 1):
        radius = WRIST_M if k == len(chain) - 2 else CAPSULE_M
        out |= _near_segment(pts, chain[k], chain[k + 1], radius)
    return out


def robot_mask(camera: Camera, depth: np.ndarray, arms: dict, *, margin_px: int = MARGIN_PX) -> np.ndarray:
    """bool (H, W): pixels showing the robot.  ``arms``: arm -> (joints, arm_base 4x4, end link 4x4 in base_link,
    gripper reading, wrist camera position or None)."""
    d = np.asarray(depth, np.float32)
    h, w = d.shape
    valid = np.isfinite(d) & (d > DEPTH_RANGE[0]) & (d < DEPTH_RANGE[1])
    ys, xs = np.nonzero(valid)
    mask = np.zeros((h, w), bool)
    if len(xs) == 0:
        return mask
    z = d[ys, xs].astype(float)
    local = np.stack([(xs - camera.cx) / camera.fx * z, -(ys - camera.cy) / camera.fy * z, -z], axis=1)
    pts = local @ camera.pose[:3, :3].T + camera.pose[:3, 3]
    hit = np.zeros(len(pts), bool)
    for arm, (q, arm_base, end_link, reading, housing) in arms.items():
        chain = skeleton(arm, q, arm_base)
        lo = np.minimum(chain.min(axis=0), end_link[:3, 3]) - 0.25
        hi = np.maximum(chain.max(axis=0), end_link[:3, 3]) + 0.25
        near = np.all((pts >= lo) & (pts <= hi), axis=1) & ~hit
        if not near.any():
            continue
        idx = np.nonzero(near)[0]
        hit[idx] |= inside_arm(pts[idx], arm, q, arm_base, end_link, reading, housing)
    mask[ys[hit], xs[hit]] = True
    if margin_px > 0 and mask.any():
        k = np.ones((2 * margin_px + 1, 2 * margin_px + 1), np.uint8)
        mask = cv2.dilate(mask.astype(np.uint8), k).astype(bool)
    return mask


def frame_arms(frame) -> dict:
    """``arms`` for :func:`robot_mask` from an observation (:class:`.obs.Frame`)."""
    out = {}
    for arm in g2.ARMS:
        cal = frame.calib.get(f"{arm}_wrist")
        out[arm] = (frame.arm_joints(arm), frame.arm_base, frame.end_link(arm), frame.gripper(arm),
                    None if cal is None else cal.pose[:3, 3].copy())
    return out


def end_from_chain(arm: str, q, arm_base: np.ndarray) -> np.ndarray:
    """The end link in base_link by forward kinematics (to compare with the reported one)."""
    return arm_base @ g2.ARM_MODELS[arm].end_pose(q)

