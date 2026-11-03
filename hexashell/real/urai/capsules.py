"""Rays against URAI's collision capsules of the arms: where the robot itself is in a picture.

Copied from URAI's urai/robot/scene.py (``Capsule``, ``_capsule_hit``, ``_capsule_window``,
``_camera_rays``, ``_pixel_rays``, ``_window``; Apache License 2.0, Copyright 2026 Hexafuture Inc.
and the URAI authors, see NOTICE), so that HexaShell does not import URAI.  Changed: the names are
public and a capsule has no colour.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass

import numpy as np

_EPS = 1e-9
#: the near plane (m): a box that reaches behind the camera is cut there
NEAR_M = 1e-3
#: Edges of a box whose corners are ordered as ``[(x, y, z) for x in .. for y in .. for z in ..]``.
BOX_EDGES = tuple((i, i | bit) for i in range(8) for bit in (1, 2, 4) if not i & bit)


@dataclass(frozen=True)
class Capsule:
    """Segment ``p``-``q`` (world, metres) swept by a sphere of ``radius``."""

    p: np.ndarray
    q: np.ndarray
    radius: float


def capsule_hit(o: np.ndarray, d: np.ndarray, cap: Capsule) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """First intersection (t, outward normal, hit) of rays ``o + t d`` with a capsule."""
    scale = np.linalg.norm(d, axis=1)
    rd = d / scale[:, None]
    ba = cap.q - cap.p
    oa = o - cap.p
    baba = float(ba @ ba)
    bard = rd @ ba
    baoa = oa @ ba
    rdoa = np.einsum("ij,ij->i", rd, oa)
    oaoa = np.einsum("ij,ij->i", oa, oa)
    a = baba - bard * bard
    b = baba * rdoa - baoa * bard
    c = baba * oaoa - baoa * baoa - cap.radius ** 2 * baba
    h = b * b - a * c
    with np.errstate(divide="ignore", invalid="ignore"):
        t_body = (-b - np.sqrt(np.maximum(h, 0.0))) / a
    y = baoa + t_body * bard
    body = (h >= 0.0) & (a > 1e-12) & (y > 0.0) & (y < baba)
    end = np.where(((y <= 0.0) | ~np.isfinite(y))[:, None], oa, o - cap.q)
    bc = np.einsum("ij,ij->i", rd, end)
    hc = bc * bc - (np.einsum("ij,ij->i", end, end) - cap.radius ** 2)
    t_cap = -bc - np.sqrt(np.maximum(hc, 0.0))
    t_unit = np.where(body, t_body, np.where(hc > 0.0, t_cap, np.nan))
    hit = np.isfinite(t_unit) & (t_unit > _EPS)
    pos = o + rd * np.where(hit, t_unit, 0.0)[:, None]
    pa = pos - cap.p
    along = np.clip((pa @ ba) / baba, 0.0, 1.0) if baba > 0.0 else np.zeros(len(o))
    normal = (pa - along[:, None] * ba) / cap.radius
    return np.where(hit, t_unit / scale, np.inf), normal, hit


def capsule_window(cap: Capsule, k: np.ndarray, t_cam_world: np.ndarray, width: int, height: int) -> np.ndarray | None:
    """Flat pixel indices covered by the projection of the capsule's bounding box."""
    r = cap.radius
    lo, hi = np.minimum(cap.p, cap.q) - r, np.maximum(cap.p, cap.q) + r
    return window(np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]),
                  k, t_cam_world, width, height)


@functools.lru_cache(maxsize=8)
def camera_rays(fx: float, fy: float, cx: float, cy: float, width: int, height: int) -> np.ndarray:
    """Camera-frame ray directions with unit z (the ray parameter is the depth along the optical axis)."""
    us, vs = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    rays = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us)], axis=-1).reshape(-1, 3)
    rays.flags.writeable = False
    return rays


def pixel_rays(k: np.ndarray, width: int, height: int) -> np.ndarray:
    return camera_rays(float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]), int(width), int(height))


def window(corners_world: np.ndarray, k: np.ndarray, t_cam_world: np.ndarray, width: int, height: int
           ) -> np.ndarray | None:
    """Flat indices of the pixels covered by a box (8 corners, see :data:`BOX_EDGES`); None when off screen.

    The box is clipped against the near plane first, so boxes that reach behind the camera still get a
    tight window.
    """
    cam = corners_world @ t_cam_world[:3, :3].T + t_cam_world[:3, 3]
    front = cam[cam[:, 2] >= NEAR_M]
    cuts = [cam[i] + (cam[j] - cam[i]) * (NEAR_M - cam[i, 2]) / (cam[j, 2] - cam[i, 2])
            for i, j in BOX_EDGES if (cam[i, 2] - NEAR_M) * (cam[j, 2] - NEAR_M) < 0.0]
    points = np.vstack([front, *cuts]) if cuts else front
    if len(points) == 0:
        return None
    u = k[0, 0] * points[:, 0] / points[:, 2] + k[0, 2]
    v = k[1, 1] * points[:, 1] / points[:, 2] + k[1, 2]
    u0, u1 = max(int(np.floor(u.min())), 0), min(int(np.ceil(u.max())), width - 1)
    v0, v1 = max(int(np.floor(v.min())), 0), min(int(np.ceil(v.max())), height - 1)
    if u0 > u1 or v0 > v1:
        return None
    cols, rows = np.meshgrid(np.arange(u0, u1 + 1), np.arange(v0, v1 + 1))
    return (rows * width + cols).ravel()
