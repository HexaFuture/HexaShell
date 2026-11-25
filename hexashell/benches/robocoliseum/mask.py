"""Which pixels of a picture show the robot's own arms and grippers, from the depth picture.

Each pixel with depth is back-projected to ``base_link``; it shows the robot when its point lies inside
the arms' shapes as the joint readings place them:

- the arm: capsules on the chain from ``arm_base_link`` (or the first joint, where the arm stands on a mount
  of its own) through the joints to the end link (``CAPSULE_M`` radius, the wrist ``WRIST_M``);
- the gripper: a box in the hand's tool frame (:class:`.kinematics.Hand`: x the approach, y the line the fingers close
  along, z along the pads; ``SHAPES`` per hand) reaching to the fingertips (they move forward as the ctek90d
  closes) less the gap between the fingers (what the fingers hold is not the robot), and the wrist camera's
  housing (a ball about the camera);
- a five-finger hand (the Wuji, :mod:`.wuji`): capsules along its fingers as their readings place them
  (``FINGER_M``), and a box about its palm (``PALM_BOX``, end link).

The mask is then widened by ``MARGIN_PX`` (the depth at the robot's edges blurs into the background).
Pixels without depth are not marked.  The shapes are rough on purpose: a little of what touches the
robot is marked with it.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ...vision.cameras import Camera
from . import kinematics
from . import wuji

#: capsule radii (m): the arm's links, and the last segment (the last joint to the end link)
CAPSULE_M = 0.065
WRIST_M = 0.055
#: the wrist camera's housing: a ball of this radius (m) about the camera
HOUSING_M = 0.035
#: depth (m) used
DEPTH_RANGE = (0.02, 3.0)
MARGIN_PX = 3
#: how far (m) past the fingertips the box reaches
TIP_MARGIN_M = 0.008
#: a five-finger hand: the fingers' capsule radius (m), and the palm's box in the right hand's end link (m: x, y, z
#: ranges; the left's y mirrored): from the hand's base to past the knuckles, the palm's face to its back
FINGER_M = 0.011
PALM_BOX = ((-0.049, 0.044), (-0.031, 0.022), (0.029, 0.136))


@dataclass(frozen=True)
class HandShape:
    """A gripper's box in its tool frame (m): half its extent across the fingers and along the pads, where it
    starts along the approach (behind the end link: negative), from where along the approach the gap between
    the fingers is free, and the clearance kept to each finger there; the wrist camera's place in the end
    link's frame when no calibration has placed it (None: no housing then)."""

    across: float
    pads: float
    back: float
    slot_from: float
    clearance: float
    housing_in_end: tuple | None = None


SHAPES = {
    # Season I's (measured): the left camera's housing in its end link
    "omnipicker": HandShape(0.075, 0.06, -0.06, 0.085, 0.003, (0.0012, -0.0898, 0.0661)),
    # the outer links' far ends sit 0.0705 m off the approach when open (kinematics._C90_TIP_Y), the pads start
    # past the links' pivots (z 0.111)
    "ctek90d": HandShape(0.082, 0.05, -0.06, 0.13, 0.003),
    # the ARX's: the fingers slide from 27 mm to 71 mm off the approach, their slides start 84 mm along it
    "arx_gripper": HandShape(0.08, 0.04, -0.05, 0.09, 0.003),
    # the Agilex's: 24.5 mm thick fingers sliding to 40 mm off the approach, 56 mm wide, from 69 mm along it
    "agilex_gripper": HandShape(0.07, 0.035, -0.05, 0.075, 0.003),
}


@dataclass
class ArmShape:
    """One arm as a frame places it: its model, joints, arm_base_link and end link in base_link, gripper
    reading, and the wrist camera's position in base_link (None: not known)."""

    model: kinematics.Arm
    q: np.ndarray
    arm_base: np.ndarray
    end_link: np.ndarray
    reading: float
    housing: np.ndarray | None = None
    #: a five-finger hand's 20 joint readings
    effector: np.ndarray | None = None


def skeleton(model: kinematics.Arm, q, arm_base: np.ndarray) -> np.ndarray:
    """The chain's points in base_link: arm_base_link (where the body reaches the first joint), the joint
    frames (repeats left out), the end link."""
    frames = model.frames(q)
    pts = ([arm_base[:3, 3]] if model.build.body_to_first_joint else []) + [(arm_base @ f)[:3, 3] for f in frames] \
        + [(arm_base @ frames[-1] @ model.end)[:3, 3]]
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


def _side(shape: ArmShape) -> str:
    return shape.model.name


def inside_hand(pts: np.ndarray, shape: ArmShape) -> np.ndarray:
    """Which of the points (N, 3, base_link) lie inside a five-finger hand: its fingers and its palm."""
    side = _side(shape)
    end = shape.end_link
    local = (pts - end[:3, 3]) @ end[:3, :3]
    (x0, x1), (y0, y1), (z0, z1) = PALM_BOX
    if side == "left":
        y0, y1 = -y1, -y0
    out = ((local[:, 0] >= x0) & (local[:, 0] <= x1) & (local[:, 1] >= y0) & (local[:, 1] <= y1)
           & (local[:, 2] >= z0) & (local[:, 2] <= z1))
    q = np.zeros(20) if shape.effector is None else np.asarray(shape.effector, float)
    for a, b in wuji.skeleton(side, q):
        out |= _near_segment(local, a, b, FINGER_M)
    cam = np.asarray(wuji.CAMERA_MOUNT[side][0], float)
    d = local - cam
    out |= np.einsum("ij,ij->i", d, d) <= HOUSING_M * HOUSING_M
    return out


def inside_arm(pts: np.ndarray, shape: ArmShape) -> np.ndarray:
    """Which of the points (N, 3, base_link) lie inside the arm's and its gripper's shapes."""
    model, hand = shape.model, shape.model.hand
    out = np.zeros(len(pts), bool)
    if hand.dexterous:
        out |= inside_hand(pts, shape)
        chain = skeleton(model, shape.q, shape.arm_base)
        for k in range(len(chain) - 1):
            radius = WRIST_M if k == len(chain) - 2 else CAPSULE_M
            out |= _near_segment(pts, chain[k], chain[k + 1], radius)
        return out
    box_of = SHAPES[hand.name]
    # the gripper, in its tool frame
    tool = shape.end_link[:3, :3] @ model.tool_in_end
    local = (pts - shape.end_link[:3, 3]) @ tool
    tip = hand.tip_reach + hand.forward_m(shape.reading) + TIP_MARGIN_M
    box = ((local[:, 0] >= box_of.back) & (local[:, 0] <= tip) & (np.abs(local[:, 1]) <= box_of.across)
           & (np.abs(local[:, 2]) <= box_of.pads))
    half = max(hand.gap_m(shape.reading) / 2.0 - box_of.clearance, 0.0)
    box &= ~((np.abs(local[:, 1]) < half) & (local[:, 0] >= box_of.slot_from))
    out |= box
    housing = shape.housing
    if housing is None and box_of.housing_in_end is not None:
        housing = (shape.end_link @ np.r_[np.asarray(box_of.housing_in_end, float), 1.0])[:3]
    if housing is not None:
        d = pts - np.asarray(housing, float)
        out |= np.einsum("ij,ij->i", d, d) <= HOUSING_M * HOUSING_M
    # the links
    chain = skeleton(model, shape.q, shape.arm_base)
    for k in range(len(chain) - 1):
        radius = WRIST_M if k == len(chain) - 2 else CAPSULE_M
        out |= _near_segment(pts, chain[k], chain[k + 1], radius)
    return out


def robot_mask(camera: Camera, depth: np.ndarray, arms: dict[str, ArmShape], *,
               margin_px: int = MARGIN_PX) -> np.ndarray:
    """bool (H, W): pixels showing the robot."""
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
    for shape in arms.values():
        chain = skeleton(shape.model, shape.q, shape.arm_base)
        lo = np.minimum(chain.min(axis=0), shape.end_link[:3, 3]) - 0.3
        hi = np.maximum(chain.max(axis=0), shape.end_link[:3, 3]) + 0.3
        near = np.all((pts >= lo) & (pts <= hi), axis=1) & ~hit
        if not near.any():
            continue
        idx = np.nonzero(near)[0]
        hit[idx] |= inside_arm(pts[idx], shape)
    mask[ys[hit], xs[hit]] = True
    if margin_px > 0 and mask.any():
        k = np.ones((2 * margin_px + 1, 2 * margin_px + 1), np.uint8)
        mask = cv2.dilate(mask.astype(np.uint8), k).astype(bool)
    return mask


def frame_arms(frame) -> dict[str, ArmShape]:
    """The arms of an observation (:class:`.obs.Frame`) for :func:`robot_mask`."""
    models = kinematics.arm_models(frame.robot)
    out = {}
    for arm in kinematics.ARMS:
        cal = frame.calib.get(f"{arm}_wrist")
        out[arm] = ArmShape(models[arm], frame.arm_joints(arm), frame.arm_base, frame.end_link(arm),
                            frame.gripper(arm), None if cal is None else cal.pose[:3, 3].copy(),
                            effector=frame.effector(arm) if models[arm].hand.dexterous else None)
    return out


def end_from_chain(model: kinematics.Arm, q, arm_base: np.ndarray) -> np.ndarray:
    """The end link in base_link by forward kinematics (to compare with the reported one)."""
    return arm_base @ model.end_pose(q)
