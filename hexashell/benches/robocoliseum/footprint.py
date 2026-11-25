"""Square footprints.

The shared measurement (:class:`hexashell.vision.rgbd.DepthScene`) calls a footprint round when its points
spread about equally every way: a disc, but a square block too, whose grasp yaw it then leaves free
(``grip_yaw any``).  Fingers closing across a square's corners slip; across a pair of faces they hold.
:func:`square_footprint` tells a square from a disc by how much of its smallest enclosing box the outline of
its points fills (a square all of it, a disc about 0.79, a hexagon 0.75, a triangle 0.5) and gives the
yaw that closes down-pointing fingers across a pair of its faces.

Even extents.  The shared measurement takes an object's extents from the percentiles of all its views' depth
points together, so a camera close to the object outvotes a far one: the Wuji hands' wrist cameras look along
the fingers from a few centimetres off, their points crowd the near part, and a 35 cm carton came out 22 cm
long (its far end a few per cent of the points).  :func:`even_extent` fits the extents again, alike, with every
part counted by its size: the heights from the points thinned to one per ``EVEN_CELL_M`` cube, the footprint
(its axes, length and width) from one per square of that side (a wall seen from the side stacks many points
on one line of the footprint).
"""

from __future__ import annotations

import math
from typing import Callable, NamedTuple

import cv2
import numpy as np

from ...vision.rgbd import HIGH_Q, LOW_Q, ROUND_RATIO

#: the outline's hull fills at least this share of its smallest enclosing box: a square
SQUARE_FILL = 0.88
#: ... and that box is about square
SQUARE_ASPECT = 1.25
MIN_POINTS = 60
#: a point is kept when its 3 x 3 cells around hold at least this many points (cells about the spacing)
MIN_AROUND = 4
#: sides differing by more than this share: close across the shorter one, else at the yaw nearer 0
UNEQUAL = 1.15


#: the cell (m) the points are thinned to for even extents: about the head camera's point spacing on the table
EVEN_CELL_M = 0.005
EVEN_MIN_POINTS = 30


class Extent(NamedTuple):
    center: tuple[float, float, float]
    size: tuple[float, float, float]  # length (the longer side), width, height
    top: float
    bottom: float
    grip_yaw: float | None  # None: round
    round: bool


def even_points(points, cell: float = EVEN_CELL_M) -> np.ndarray:
    """The points (N, d) thinned to one per cell of side ``cell`` (a cube, or a square for d = 2): the mean of
    those in it."""
    pts = np.asarray(points, float)
    pts = pts.reshape(-1, pts.shape[-1] if pts.ndim > 1 else 3)
    if not len(pts):
        return pts
    _, inv = np.unique(np.floor(pts / cell).astype(np.int64), axis=0, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    sums = np.zeros((int(inv.max()) + 1, pts.shape[1]))
    np.add.at(sums, inv, pts)
    return sums / np.bincount(inv).astype(float)[:, None]


def even_extent(points, closing_yaw: Callable[[tuple[float, float]], float]) -> Extent | None:
    """The shared measurement's extents (percentiles along the footprint's axes, and of the heights) on the points
    thinned by :func:`even_points`; None for too few points."""
    raw = np.asarray(points, float).reshape(-1, 3)
    pts = even_points(raw)
    xy = even_points(raw[:, :2])
    if len(pts) < EVEN_MIN_POINTS or len(xy) < EVEN_MIN_POINTS:
        return None
    lo_z, hi_z = np.percentile(pts[:, 2], [LOW_Q, HIGH_Q])
    mid = np.median(xy, axis=0)
    _, vecs = np.linalg.eigh(np.cov((xy - mid).T))
    major, minor = vecs[:, 1], vecs[:, 0]
    a_lo, a_hi = np.percentile((xy - mid) @ major, [LOW_Q, HIGH_Q])
    b_lo, b_hi = np.percentile((xy - mid) @ minor, [LOW_Q, HIGH_Q])
    centre = mid + major * (a_lo + a_hi) / 2.0 + minor * (b_lo + b_hi) / 2.0
    length, width = float(a_hi - a_lo), float(b_hi - b_lo)
    is_round = width > 0 and length / max(width, 1e-6) < ROUND_RATIO
    return Extent((float(centre[0]), float(centre[1]), float(lo_z + hi_z) / 2.0), (length, width, float(hi_z - lo_z)),
                  float(hi_z), float(lo_z), None if is_round else float(closing_yaw(minor)), bool(is_round))


class Square(NamedTuple):
    grip_yaw: float  # closes down-pointing fingers across a pair of faces
    across_m: float  # about how far apart those faces are (what the cameras see of them)
    other_m: float  # the other pair


def square_footprint(points, closing_yaw: Callable[[tuple[float, float]], float]) -> Square | None:
    """The grasp across a pair of faces when the points' footprint (x, y) is about a square, else None.
    The pair: across the shorter sides when they differ, else the one whose yaw is nearer 0."""
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] < 2 or len(pts) < MIN_POINTS:
        return None
    xy = pts[:, :2]
    lo, hi = np.percentile(xy, 1, axis=0), np.percentile(xy, 99, axis=0)
    xy = xy[np.all((xy >= lo - 0.005) & (xy <= hi + 0.005), axis=1)]
    if len(xy) < MIN_POINTS:
        return None
    # leave out stray points: few others close by
    span = xy.max(axis=0) - xy.min(axis=0)
    spacing = math.sqrt(max(float(span[0] * span[1]), 1e-8) / len(xy))
    cell = max(spacing, 0.001)
    ij = np.floor((xy - xy.min(axis=0)) / cell).astype(int) + 1
    cols, rows = ij.max(axis=0) + 2
    if cols * rows > 1_000_000:
        return None
    counts = np.zeros((rows, cols), np.float32)
    np.add.at(counts, (ij[:, 1], ij[:, 0]), 1.0)
    around = cv2.boxFilter(counts, -1, (3, 3), normalize=False, borderType=cv2.BORDER_CONSTANT)
    xy = xy[around[ij[:, 1], ij[:, 0]] >= MIN_AROUND]
    if len(xy) < MIN_POINTS:
        return None
    mm = ((xy - xy.mean(axis=0)) * 1000.0).astype(np.float32)
    hull = cv2.convexHull(mm)
    (_, _), (w, h), angle = cv2.minAreaRect(hull)
    if min(w, h) < 5.0:  # mm
        return None
    if max(w, h) / min(w, h) > SQUARE_ASPECT or cv2.contourArea(hull) / (w * h) < SQUARE_FILL:
        return None
    a = math.radians(angle)
    pairs = []
    for d in ((math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))):
        lo_a, hi_a = np.percentile(mm.astype(float) @ np.array(d), [0.5, 99.5])
        pairs.append((float(closing_yaw(d)), float(hi_a - lo_a) / 1000.0))
    (y1, e1), (y2, e2) = pairs
    if max(e1, e2) > UNEQUAL * min(e1, e2):
        pick = 0 if e1 < e2 else 1
    else:
        pick = 0 if abs(y1) <= abs(y2) else 1
    other = pairs[1 - pick][1]
    return Square(pairs[pick][0], pairs[pick][1], other)
