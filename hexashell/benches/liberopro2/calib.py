"""The cameras checked against the robot they picture, and re-fitted when they are off (LIBERO-Pro 2.0 only).

Without depth every measurement goes through the cameras' models (:mod:`.scene`), and a perturbation may change a
camera: move it, change its lens, or wrap its pictures around (every row or column shifted along, what is pushed
past one edge coming back in at the other).  The robot is in the agentview picture, and its joint readings say
where it is: SAM3's outline of the robot (ROBOT_WORDS) against the robot drawn where the readings put it
(:class:`Outline`) through a camera model tells how well that model explains the picture (:func:`score`).  So, as a
hand-eye calibration on a real robot, from the pictures and the readings alone (never the simulator's cameras):

1. At the start, before a step is taken (:meth:`Calibration.check`): whether the pictures are wrapped
   (:func:`roll_of`), and how much better than the nominal camera a camera pose fitted to the start's outline
   explains it (the *gain*).  Unwrapped pictures and a gain under GAIN: nothing more is done, and the episode goes on
   as without this module.
2. Else the hand goes MOVE from the start (:meth:`Calibration.move`; the episode goes on from there), with shots
   (both pictures and the readings) every EVERY steps of the way and at its end, and agentview's camera is fitted
   to all their outlines in two ways (MODELS: its pose, its lens); :func:`choose` keeps the nominal camera unless
   one gains GAIN.
3. The wrist camera rides on the hand: the hand and the fingers it pictures stay put while the scene behind them
   moves.  The pixels that stay still over the shots (:func:`still_pixels`) against the robot drawn through its
   nominal model (:func:`self_score`): under TRUST the wrist camera is not where the robot's model has it, and
   perception goes without it (:class:`.scene.LiberoScene`).  When agentview's focal length changed by at least
   WRIST_FOCAL, the wrist camera's is fitted the same way and taken when it changes by that much too.
4. The fixes (:class:`Fix`) go to the simulator (``fixes`` of :class:`.sim.LiberoProSim`): the pictures un-wrapped
   and undistorted, the camera models re-fitted; all that reads the cameras (the pictures, the robot's predicted
   pixels, look_from) goes through them.

A camera (:func:`camera_of`) is the nominal one turned and moved in its own frame (a rotation vector, degrees; a
shift, cm), its focal length scaled (``exp(theta[6] / 100)``) and one radial term ``k1`` (``theta[7] / 100``): a ray
the pinhole puts ``n`` half-widths off the centre is pictured ``n (1 + k1 |n|^2)`` off.

The server half (:class:`Calibration`, :class:`Fix`, the fits) runs in the simulator's environment; the host half
(:func:`calibrate`) on the episode side, where SAM3 is.
"""

from __future__ import annotations

import math
import time

import numpy as np

from . import protocol as P

#: SAM3's words for the robot; its masks for them with at least ROBOT_MIN_SCORE, united, are the robot's outline
ROBOT_WORDS, ROBOT_MIN_SCORE, ROBOT_MAX_MASKS = "robot arm", 0.3, 8
#: Wrapped pictures (:func:`roll_of`), per axis: every picture must show its last and first lines continuing each
#: other (the jump across that boundary at most WRAP_RATIO times the larger of the jumps next to it, plus
#: WRAP_FLOOR), and a seam (a one-line step: the jump across it more than STEP_RATIO times both neighbouring ones,
#: plus STEP_FLOOR, along at least SEAM_SHARE of the line) at the same place, within a line, in all of them.  A
#: jump: the colour difference of two lines, summed over the channels, per position along them.
WRAP_RATIO, WRAP_FLOOR = 3.0, 6.0
STEP_RATIO, STEP_FLOOR = 2.0, 12.0
SEAM_SHARE = 0.2
#: The robot drawn (:class:`Hulls`): each visual geom's hull by its points farthest along DIRECTIONS directions
#: (inside the hull: on a start picture 0.6 % of the robot's pixels fewer than with all of them), cut NEAR_M in
#: front of the camera.
DIRECTIONS = 256
NEAR_M = 0.01
#: The score of a camera (:func:`score`): the overlap of the outline and the robot drawn through it, the drawn
#: pixels outside the outline counted LAM (the robot hidden behind something, or missed by SAM3, is likelier than
#: SAM3 outlining something else).  The fits draw at FIT_SIZE pixels (the outlines scaled down to it).
LAM = 0.5
FIT_SIZE = 256
#: The camera models fitted: the entries of theta (see :func:`camera_of`) each frees
MODELS = {"pose": (0, 1, 2, 3, 4, 5), "lens": (6, 7)}
#: Where the fits start from (:func:`fit`): the best FIT_STARTS of a grid of shifts (cm, the camera's x, y and z)
#: and focal changes (theta[6]) and no change; Nelder-Mead's first steps STEPS; it stops after FIT_EVALS
#: evaluations per entry fitted, or with the simplex within XTOL of its best point and the scores within FTOL.
GRID_X = GRID_Y = (-15.0, -7.5, 0.0, 7.5, 15.0)
GRID_Z = (-20.0, -10.0, 0.0, 10.0, 20.0)
GRID_FOCAL = tuple(float(v) for v in np.linspace(-50.0, 15.0, 14))
FIT_STARTS = 4
STEPS = (1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 2.0)
FIT_EVALS, XTOL, FTOL = 250, 0.02, 1e-5
#: The choice (:func:`choose`): the nominal camera unless a model gains GAIN; then the lens, which has fewer entries
#: free, unless the pose scores MARGIN better.  (On the move's shots of the learning starts that called for the
#: calibration the pose scored either 0.10 to 0.57 better than the lens or between 0.045 worse and 0.012 better;
#: fitting all the entries together gained at most 0.06 more and was never nearer.)  The start check calls for
#: the calibration at the same GAIN when the camera fitted also covers more of the outline than the nominal one (by
#: more than COVER_GAIN: SAM3 may outline only part of a robot that looks unusual, and a camera farther off then
#: explains the part better while covering less of it), or when the pictures are wrapped.
GAIN, MARGIN = 0.10, 0.05
COVER_GAIN = 0.0
#: The move (:meth:`Calibration.move`): the hand MOVE (m, arm frame) from the start, up and aside, clear of what
#: stands on the table (some 22 steps); a shot every EVERY steps of the way and at its end.  When the arm cannot go
#: there, the move mirrored left to right.
MOVE = (0.0, -0.15, 0.05)
EVERY = 3
#: The wrist camera (:func:`still_pixels`, :func:`self_score`): a pixel is still when its colour stays within STILL
#: (summed over the channels) of the first shot's in every shot, opened by a STILL_OPEN square; the still pixels off
#: the drawn robot count SPARE (bare surfaces stay still too); under TRUST at the nominal camera it is not trusted.
STILL, STILL_OPEN = 24, 5
SPARE = 0.3
TRUST = 0.45
#: The wrist camera's focal length (theta[6] alone: with the radial term free too, a still band across the
#: picture is explained as well by a picture zoomed in and bent) is taken when it changes by at least WRIST_FOCAL
#: (a share) and scores better than the nominal one; the fit starts from the best WRIST_STARTS of WRIST_GRID.
WRIST_FOCAL = 0.05
WRIST_GRID = tuple(float(v) for v in np.linspace(-60.0, 20.0, 17))
WRIST_STARTS = 3

AGENTVIEW, WRIST = "agentview", "wrist"


# -- wrapped pictures ------------------------------------------------------------------------------------------
def _jumps(image: np.ndarray, axis: int) -> np.ndarray:
    """(n, m): the jump across each boundary between line k and k + 1 (k = n - 1: the last line to the first) at
    each of the m positions along the lines; lines are rows (axis 0) or columns (axis 1)."""
    a = np.moveaxis(np.asarray(image, np.float32), axis, 0)
    return np.abs(np.roll(a, -1, axis=0) - a).sum(axis=2)


def roll_of(images) -> tuple:
    """(rows, columns) the pictures are wrapped by: the line of each picture that shows the scene's first line, or
    0 (see WRAP_RATIO)."""
    out = []
    for axis in (0, 1):
        jumps = [_jumps(image, axis) for image in images]
        wraps = True
        for j in jumps:
            med = np.median(j, axis=1)
            wraps = wraps and med[-1] <= WRAP_RATIO * max(med[-2], med[0]) + WRAP_FLOOR
        if not wraps:
            out.append(0)
            continue
        shares = []
        for j in jumps:
            side = np.maximum(np.roll(j, 1, axis=0), np.roll(j, -1, axis=0))
            shares.append((j > STEP_RATIO * side + STEP_FLOOR).mean(axis=1)[:-1])
        best = [int(np.argmax(s)) for s in shares]
        same = all(abs(b - best[0]) <= 1 for b in best) and all(s[b] >= SEAM_SHARE for s, b in zip(shares, best))
        out.append(best[0] + 1 if same else 0)
    return tuple(out)


def unroll(image: np.ndarray, roll) -> np.ndarray:
    """The picture (or mask) un-wrapped: the scene's first line first again."""
    rows, cols = int(roll[0]), int(roll[1])
    return np.roll(image, (-rows, -cols), axis=(0, 1)) if (rows or cols) else image


# -- cameras ---------------------------------------------------------------------------------------------------
def _rotation(vector) -> np.ndarray:
    """The rotation matrix of a rotation vector (radians)."""
    v = np.asarray(vector, float)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    cross = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * cross @ cross


def camera_of(theta, pose0, focal0) -> tuple:
    """(pose, focal, k1) of the nominal camera (``pose0``: arm frame, MuJoCo's axes; ``focal0``) changed by theta:
    turned by theta[0:3] (rotation vector, degrees) and moved by theta[3:6] (cm), both in the camera's own frame,
    the focal length scaled by exp(theta[6] / 100), k1 = theta[7] / 100."""
    theta = np.asarray(theta, float)
    change = np.eye(4)
    change[:3, :3] = _rotation(np.radians(theta[:3]))
    change[:3, 3] = theta[3:6] / 100.0
    return np.asarray(pose0, float) @ change, float(focal0) * math.exp(theta[6] / 100.0), float(theta[7]) / 100.0


def _project(local: np.ndarray, focal: float, k1: float, c: float) -> np.ndarray:
    """Pixels (N, 2, float32) of camera-frame points (N, 3) in front of a camera of ``focal`` and ``k1`` whose
    centre is (c, c); with k1 < 0 the points beyond the radius where the distortion turns back are pulled in to it."""
    depth = -local[:, 2]
    nx, ny = focal * local[:, 0] / depth / c, -focal * local[:, 1] / depth / c
    if k1:
        r2 = nx * nx + ny * ny
        if k1 < 0.0:
            fold = 1.0 / (3.0 * -k1)
            pull = np.sqrt(np.minimum(r2, fold) / np.maximum(r2, 1e-12))
            nx, ny, r2 = nx * pull, ny * pull, np.minimum(r2, fold)
        g = 1.0 + k1 * r2
        nx, ny = nx * g, ny * g
    return np.stack([c + c * nx, c + c * ny], axis=1).astype(np.float32)


def _fill(out: np.ndarray, px: np.ndarray) -> None:
    import cv2

    size = out.shape[0]
    if px[:, 0].max() < 0 or px[:, 1].max() < 0 or px[:, 0].min() > size - 1 or px[:, 1].min() > size - 1:
        return
    hull = cv2.convexHull(px)
    cv2.fillConvexPoly(out, np.round(hull).astype(np.int32).reshape(-1, 2), 1)


def _directions(n: int) -> np.ndarray:
    """n directions spread evenly over the sphere (a Fibonacci lattice)."""
    i = np.arange(n) + 0.5
    polar = np.arccos(1.0 - 2.0 * i / n)
    turn = math.pi * (1.0 + math.sqrt(5.0)) * i
    return np.stack([np.cos(turn) * np.sin(polar), np.sin(turn) * np.sin(polar), np.cos(polar)], axis=1)


class Hulls:
    """The robot's visual geoms (:class:`.sim.RobotPixels`), each by the points of its hull farthest along
    DIRECTIONS directions, with their own hull's edges."""

    def __init__(self, robot_pixels):
        from .sim import _hull

        directions = _directions(DIRECTIONS)
        self.geoms = []
        for g, part, points, _ in robot_pixels.geoms:
            keep = np.unique(np.argmax((points - points.mean(axis=0)) @ directions.T, axis=0))
            vertices, edges = _hull(points[keep])
            self.geoms.append((g, part, points[keep][vertices], edges))

    def at(self, data, to_start: np.ndarray) -> list:
        """[(part, vertices (N, 3) in the arm frame, edges)] with the geoms where ``data`` has them."""
        rot0, pos0 = to_start[:3, :3], to_start[:3, 3]
        out = []
        for g, part, points, edges in self.geoms:
            rot, pos = data.geom_xmat[g].reshape(3, 3), data.geom_xpos[g]
            out.append((part, (points @ rot.T + pos) @ rot0.T + pos0, edges))
        return out


class Outline:
    """The robot of one shot (:meth:`Hulls.at`) to draw through candidate cameras; ``table_z``: what is below the
    table top left out (agentview looks down at the table, so that is behind it; None: nothing left out)."""

    def __init__(self, hulls: list, table_z: float | None = None):
        from .sim import _side

        self.pieces = []
        for _, points, edges in hulls:
            if table_z is not None and (points[:, 2] < table_z).any():
                points, edges = _side(points, edges, points[:, 2], table_z), None
                if len(points) < 3:
                    continue
            self.pieces.append((points, edges))
        self.points = np.concatenate([p for p, _ in self.pieces]) if self.pieces else np.zeros((0, 3))
        self.cuts = np.cumsum([len(p) for p, _ in self.pieces])[:-1]

    def draw(self, pose: np.ndarray, focal: float, k1: float, size: int) -> np.ndarray:
        """bool (size, size): the robot's pixels through the camera (pose: arm frame, MuJoCo's axes)."""
        from .sim import _hull, _side

        out = np.zeros((size, size), np.uint8)
        if not len(self.points):
            return out > 0
        c = (size - 1) / 2.0
        local = (self.points - pose[:3, 3]) @ pose[:3, :3]
        if (-local[:, 2] > NEAR_M).all():
            for px in np.split(_project(local, focal, k1, c), self.cuts):
                _fill(out, px)
            return out > 0
        for (_, edges), piece in zip(self.pieces, np.split(local, self.cuts)):
            depth = -piece[:, 2]
            if not (depth > NEAR_M).any():
                continue
            if not (depth > NEAR_M).all():  # cut at the near plane, where the hull's edges cross it
                if edges is None:
                    keep, edges = _hull(piece)
                    piece, depth = piece[keep], depth[keep]
                piece = _side(piece, edges, depth, NEAR_M)
                if len(piece) < 3:
                    continue
            _fill(out, _project(piece, focal, k1, c))
        return out > 0


def score(drawn: np.ndarray, mask: np.ndarray, lam: float = LAM) -> float:
    """The overlap of the robot drawn and its outline, the drawn pixels outside the outline counted ``lam``."""
    both = np.count_nonzero(drawn & mask)
    whole = both + np.count_nonzero(mask & ~drawn) + lam * np.count_nonzero(drawn & ~mask)
    return both / whole if whole else 0.0


def covered(drawn: np.ndarray, mask: np.ndarray) -> float:
    """The share of the outline the robot drawn covers."""
    n = np.count_nonzero(mask)
    return np.count_nonzero(drawn & mask) / n if n else 0.0


def scaled(mask: np.ndarray, size: int) -> np.ndarray:
    """A mask at size x size pixels (a pixel kept where at least half of what it covers is)."""
    import cv2

    mask = np.asarray(mask, bool)
    if mask.shape[0] == size:
        return mask
    return cv2.resize(mask.astype(np.float32), (size, size), interpolation=cv2.INTER_AREA) >= 0.5


# -- the fits --------------------------------------------------------------------------------------------------
def nelder_mead(cost, x0, steps, max_evals: int, xtol: float = XTOL, ftol: float = FTOL) -> tuple:
    """(the best point, its cost, the evaluations made): Nelder and Mead's simplex search (reflection 1, expansion
    2, contraction and shrink 0.5, as scipy's) from ``x0`` with the first simplex's edges ``steps``."""
    x0 = np.asarray(x0, float)
    n = len(x0)
    simplex = [x0] + [x0 + np.eye(n)[i] * steps[i] for i in range(n)]
    costs = [cost(x) for x in simplex]
    evals = n + 1
    while evals < max_evals:
        order = np.argsort(costs, kind="stable")
        simplex, costs = [simplex[i] for i in order], [costs[i] for i in order]
        if max(float(np.abs(x - simplex[0]).max()) for x in simplex[1:]) <= xtol \
                and max(abs(f - costs[0]) for f in costs[1:]) <= ftol:
            break
        centre = np.mean(simplex[:-1], axis=0)
        worst = simplex[-1]
        xr = 2.0 * centre - worst
        fr = cost(xr)
        evals += 1
        if fr < costs[0]:
            xe = 3.0 * centre - 2.0 * worst
            fe = cost(xe)
            evals += 1
            simplex[-1], costs[-1] = (xe, fe) if fe < fr else (xr, fr)
            continue
        if fr < costs[-2]:
            simplex[-1], costs[-1] = xr, fr
            continue
        if fr < costs[-1]:
            xc = centre + 0.5 * (xr - centre)
            fc = cost(xc)
            evals += 1
            if fc <= fr:
                simplex[-1], costs[-1] = xc, fc
                continue
        else:
            xc = centre + 0.5 * (worst - centre)
            fc = cost(xc)
            evals += 1
            if fc < costs[-1]:
                simplex[-1], costs[-1] = xc, fc
                continue
        for i in range(1, n + 1):
            simplex[i] = simplex[0] + 0.5 * (simplex[i] - simplex[0])
            costs[i] = cost(simplex[i])
        evals += n
    k = int(np.argmin(costs))
    return simplex[k], costs[k], evals


def _grid(free) -> list:
    """The thetas of the grid the fits start from (the entries ``free`` leaves fixed at 0)."""
    focal = GRID_FOCAL if 6 in free else (0.0,)
    xs, ys, zs = (GRID_X if 3 in free else (0.0,)), (GRID_Y if 4 in free else (0.0,)), (GRID_Z if 5 in free else (0.0,))
    return [np.array([0.0, 0.0, 0.0, x, y, z, f, 0.0]) for f in focal for z in zs for x in xs for y in ys]


def fit(outlines: list, masks: list, poses: list, focal0: float, size: int, free, *, starts=None,
        lam: float = LAM) -> dict:
    """The camera (theta, see :func:`camera_of`; the entries ``free`` fitted, the rest 0) that best explains the
    masks (one per outline: the robot in several shots, ``poses`` the nominal camera's in each), drawn at ``size``
    pixels with the focal length ``focal0`` there: {"theta", "score", "nominal" (the score of theta 0), "evals"}.
    ``starts``: thetas to start from as well as the grid's best FIT_STARTS and no change."""
    free = tuple(free)
    count = [0]

    def cost_of(theta) -> float:
        count[0] += 1
        total = 0.0
        for outline, mask, pose0 in zip(outlines, masks, poses):
            pose, focal, k1 = camera_of(theta, pose0, focal0)
            total += score(outline.draw(pose, focal, k1, size), mask, lam)
        return -total / len(outlines)

    def full(x) -> np.ndarray:
        theta = np.zeros(8)
        theta[list(free)] = x
        return theta

    nominal = -cost_of(np.zeros(8))
    grid = _grid(free)
    firsts = [np.zeros(8)] + [np.asarray(s, float) for s in (starts or ())]
    if len(grid) > 1:
        ranked = sorted(((cost_of(theta), k) for k, theta in enumerate(grid)), key=lambda t: t[0])
        firsts += [grid[k] for _, k in ranked[:FIT_STARTS]]
    steps = np.array(STEPS)[list(free)]
    best = None
    for theta0 in firsts:
        x, f, _ = nelder_mead(lambda x: cost_of(full(x)), theta0[list(free)], steps, FIT_EVALS * len(free))
        if best is None or f < best[1]:
            best = (x, f)
    return {"theta": full(best[0]), "score": -best[1], "nominal": nominal, "evals": count[0]}


def choose(fits: dict) -> str:
    """The model to take (see GAIN) of fits {"pose", "lens": fit's result}: "nominal", "lens" or "pose"."""
    pose, lens = fits["pose"], fits["lens"]
    if max(pose["score"], lens["score"]) - pose["nominal"] < GAIN:
        return "nominal"
    if lens["score"] - lens["nominal"] >= GAIN and pose["score"] - lens["score"] < MARGIN:
        return "lens"
    return "pose"


# -- the wrist camera ------------------------------------------------------------------------------------------
def still_pixels(images: list) -> np.ndarray:
    """bool (H, W): the pixels whose colour stays within STILL of the first picture's in all the others."""
    import cv2

    first = np.asarray(images[0], np.int16)
    change = np.max([np.abs(np.asarray(im, np.int16) - first).sum(axis=2) for im in images[1:]], axis=0)
    still = (change <= STILL).astype(np.uint8)
    return cv2.morphologyEx(still, cv2.MORPH_OPEN, np.ones((STILL_OPEN, STILL_OPEN), np.uint8)) > 0


def self_score(drawn: np.ndarray, still: np.ndarray) -> float:
    """How well the robot drawn in every shot (``drawn``: the pixels it covers in all of them) is the still
    pixels, those off it counted SPARE."""
    both = np.count_nonzero(drawn & still)
    whole = both + SPARE * np.count_nonzero(still & ~drawn) + np.count_nonzero(drawn & ~still)
    return both / whole if whole else 0.0


def wrist_fit(outlines: list, poses: list, focal0: float, size: int, still: np.ndarray, lens: bool) -> dict:
    """The wrist camera's self-view: {"nominal": :func:`self_score` at the nominal camera, and with ``lens`` the
    focal length that scores best: "theta", "score"}."""

    def drawn(theta) -> np.ndarray:
        out = None
        for outline, pose0 in zip(outlines, poses):
            pose, focal, k1 = camera_of(theta, pose0, focal0)
            one = outline.draw(pose, focal, k1, size)
            out = one if out is None else out & one
        return out

    out = {"nominal": self_score(drawn(np.zeros(8)), still)}
    if not lens:
        return out

    def full(x) -> np.ndarray:
        theta = np.zeros(8)
        theta[6] = x[0]
        return theta

    cost = lambda x: -self_score(drawn(full(x)), still)  # noqa: E731
    ranked = sorted((cost(np.array([f])), f) for f in WRIST_GRID)
    best = None
    for _, f0 in ranked[:WRIST_STARTS]:
        x, f, _ = nelder_mead(cost, np.array([f0]), np.array([STEPS[6]]), FIT_EVALS)
        if best is None or f < best[1]:
            best = (x, f)
    out.update(theta=full(best[0]), score=-best[1])
    return out


# -- the fixes -------------------------------------------------------------------------------------------------
class Fix:
    """A camera's correction: its pictures un-wrapped (``roll``: see :func:`roll_of`) and, with k1, undistorted to
    the pinhole of the fitted focal length; its model changed by ``theta`` (:func:`camera_of`)."""

    def __init__(self, roll=(0, 0), theta=None):
        self.roll = (int(roll[0]), int(roll[1]))
        self.theta = np.zeros(8) if theta is None else np.asarray(theta, float)
        self._maps = None

    @property
    def k1(self) -> float:
        return float(self.theta[7]) / 100.0

    def camera(self, camera: dict) -> dict:
        """The camera model (as :meth:`.sim.LiberoProSim.camera` gives the nominal one) changed."""
        pose, focal, _ = camera_of(self.theta, camera["pose"], camera["focal"])
        return {**camera, "pose": pose, "focal": focal}

    def picture(self, image: np.ndarray) -> np.ndarray:
        """The picture un-wrapped and undistorted: at a pixel ``n`` half-widths off the centre, what the camera
        pictured ``n (1 + k1 |n|^2)`` off."""
        import cv2

        image = unroll(np.asarray(image), self.roll)
        if self.k1:
            h, w = image.shape[:2]
            if self._maps is None or self._maps[0] != (h, w):
                c = (w - 1) / 2.0
                xs, ys = np.meshgrid((np.arange(w) - c) / c, (np.arange(h) - (h - 1) / 2.0) / c)
                g = 1.0 + self.k1 * (xs * xs + ys * ys)
                self._maps = ((h, w), (c + c * xs * g).astype(np.float32),
                              ((h - 1) / 2.0 + c * ys * g).astype(np.float32))
            image = cv2.remap(image, self._maps[1], self._maps[2], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return np.ascontiguousarray(image)

    def brief(self) -> dict:
        return {"roll": list(self.roll), "theta": [round(float(v), 3) for v in self.theta]}


# -- the server half -------------------------------------------------------------------------------------------
class Shot:
    """What a shot keeps: its step, the robot where the readings put it (:meth:`Hulls.at`), the nominal cameras and
    the pictures as taken."""

    def __init__(self, step: int, hulls: list, cameras: dict, images: dict):
        self.step, self.hulls, self.cameras, self.images = int(step), hulls, cameras, images


class Calibration:
    """One episode's camera check (server side; see the module's notes): :meth:`check`, then when it calls for
    it :meth:`move` and :meth:`fit`."""

    def __init__(self, sim):
        self.sim = sim
        self.hulls = Hulls(sim.robot_pixels)
        self.shots: list = []
        self.roll = (0, 0)
        self.masks: dict = {}
        self.found: dict = {}

    def shoot(self) -> Shot:
        sim = self.sim
        shot = Shot(sim.steps, self.hulls.at(sim.seen, sim.to_start),
                    {cam: sim.nominal_camera(cam) for cam in P.CAMERAS},
                    {cam: sim.picture(cam) for cam in P.CAMERAS})
        self.shots.append(shot)
        return shot

    def _agentview(self, shots: list) -> tuple:
        """(outlines, masks, poses, focal) of agentview in ``shots`` at FIT_SIZE."""
        size, k = self.sim.size, FIT_SIZE / self.sim.size
        table = self.sim.table
        table_z = float(table["z"]) if table else None
        outlines = [Outline(s.hulls, table_z) for s in shots]
        masks = [scaled(unroll(self.masks[s.step], self.roll), FIT_SIZE) for s in shots]
        poses = [np.asarray(s.cameras[AGENTVIEW]["pose"], float) for s in shots]
        assert all(int(s.cameras[AGENTVIEW]["width"]) == size for s in shots)
        return outlines, masks, poses, float(shots[0].cameras[AGENTVIEW]["focal"]) * k

    def check(self, mask) -> dict:
        """The start's check (no step taken): {"need": whether to calibrate, "roll", "gain" (of the camera pose
        fitted to the start's outline), "score" (the nominal camera's), "cover" (the share of the outline the robot
        drawn covers: through the nominal camera, through the one fitted)}."""
        t0 = time.time()
        self.shots = []
        start = self.shoot()
        self.roll = roll_of([start.images[cam] for cam in P.CAMERAS])
        self.masks = {start.step: np.asarray(mask, bool)}
        outlines, masks, poses, focal = self._agentview([start])
        got = fit(outlines, masks, poses, focal, FIT_SIZE, MODELS["pose"])
        gain = got["score"] - got["nominal"]
        cover = [covered(outlines[0].draw(*camera_of(theta, poses[0], focal), FIT_SIZE), masks[0])
                 for theta in (np.zeros(8), got["theta"])]
        need = any(self.roll) or (gain >= GAIN and cover[1] - cover[0] > COVER_GAIN)
        self.found = {"need": bool(need), "roll": list(self.roll), "gain": round(gain, 4),
                      "score": round(got["nominal"], 4), "cover": [round(c, 4) for c in cover],
                      "start_theta": [round(float(v), 3) for v in got["theta"]], "check_s": round(time.time() - t0, 2)}
        return dict(self.found)

    def _on_step(self, sim) -> None:
        if sim.steps % EVERY == 0:
            self.shoot()

    def move(self, server) -> dict:
        """The hand MOVE from where it is (or that mirrored left to right, when the arm cannot go there), with shots
        every EVERY steps of the way and at its end (``server``: the simulator's server, whose goto moves the arm):
        the goto's reply, its result with the move's, and "images": {step: agentview's picture} of the shots after
        the start."""
        sim = self.sim
        start = sim.tcp()
        rows, out, reply = [], None, None
        for offset in (np.asarray(MOVE, float), np.asarray(MOVE, float) * (1.0, -1.0, 1.0)):
            goal = start.copy()
            goal[:3, 3] = start[:3, 3] + offset
            dry = server.op_goto(goal, dry_run=True)
            rows.append(dry["log"])
            if not dry["result"]["reach"]["ok"]:
                continue
            first = sim.steps
            sim.on_step = self._on_step
            try:
                reply = server.op_goto(goal)
            finally:
                sim.on_step = None
            rows.append(reply["log"])
            result = reply["result"]
            out = {"offset": [round(float(v), 3) for v in offset], "steps": sim.steps - first,
                   "moved": bool(result.get("moved")), "blocked": bool(result.get("blocked")),
                   "off_mm": round(1000.0 * float(np.linalg.norm(sim.tcp()[:3, 3] - goal[:3, 3])), 1)}
            if self.shots[-1].step != sim.steps:
                self.shoot()
            break
        self.found["move"] = out
        rows = [r for r in rows if len(r)]
        return {"result": {**(reply or {}).get("result", {}), "moved": bool(out and out["moved"]), "move": out},
                "proprio": sim.proprio(), "log": np.concatenate(rows) if rows else server.robot.take_history(),
                "images": {str(s.step): s.images[AGENTVIEW] for s in self.shots[1:]}}

    def fit(self, masks: dict) -> dict:
        """The cameras fitted to the shots' outlines (``masks``: {step: agentview's robot outline} of the shots
        after the start), chosen and set in the simulator's ``fixes``: {"agentview": the model taken, "wrist":
        "trusted" / "lens" / "untrusted", "fixes": {cam: Fix.brief()}, ...}.  When the hand did not move, only
        the pictures' wrap is undone."""
        t0 = time.time()
        sim = self.sim
        for step, mask in masks.items():
            self.masks[int(step)] = np.asarray(mask, bool)
        shots = [s for s in self.shots if s.step in self.masks]
        model, theta, fits, wrist = "nominal", np.zeros(8), {}, {"verdict": "trusted"}
        if (self.found.get("move") or {}).get("moved") and len(shots) > 1:
            outlines, scaled_masks, poses, focal = self._agentview(shots)
            fits["pose"] = fit(outlines, scaled_masks, poses, focal, FIT_SIZE, MODELS["pose"],
                               starts=[self.found["start_theta"]])
            fits["lens"] = fit(outlines, scaled_masks, poses, focal, FIT_SIZE, MODELS["lens"])
            model = choose(fits)
            if model != "nominal":
                theta = fits[model]["theta"]
            wrist = self._wrist(shots, lens=abs(math.expm1(theta[6] / 100.0)) >= WRIST_FOCAL)
        fixes = {}
        if model != "nominal" or any(self.roll):
            fixes[AGENTVIEW] = Fix(self.roll, theta)
        if wrist["verdict"] == "lens":
            fixes[WRIST] = Fix(self.roll, wrist["theta"])
        elif any(self.roll):
            fixes[WRIST] = Fix(self.roll)
        sim.fixes = fixes
        self.found.update(
            agentview=model, wrist=wrist["verdict"], shots=[s.step for s in shots],
            fits={name: {"theta": [round(float(v), 3) for v in f["theta"]], "score": round(f["score"], 4),
                         "evals": f["evals"]} for name, f in fits.items()},
            fixes={cam: fix.brief() for cam, fix in fixes.items()}, fit_s=round(time.time() - t0, 2))
        if fits:
            self.found["nominal"] = round(fits["pose"]["nominal"], 4)
        if "nominal" in wrist:
            self.found["wrist_score"] = round(wrist["nominal"], 4)
        if "theta" in wrist:
            self.found.update(wrist_theta=[round(float(v), 3) for v in wrist["theta"]],
                              wrist_lens_score=round(wrist["score"], 4))
        return dict(self.found)

    def _wrist(self, shots: list, lens: bool) -> dict:
        """The wrist camera's verdict on its self-view over the shots: "untrusted" (under TRUST), "lens" (its focal
        length re-fitted: ``lens`` asks for the fit) or "trusted"."""
        size = self.sim.size
        images = [unroll(s.images[WRIST], self.roll) for s in shots]
        still = still_pixels(images)
        outlines = [Outline(s.hulls) for s in shots]
        poses = [np.asarray(s.cameras[WRIST]["pose"], float) for s in shots]
        focal = float(shots[0].cameras[WRIST]["focal"])
        got = wrist_fit(outlines, poses, focal, size, still, lens)
        if got["nominal"] < TRUST:
            return {**got, "verdict": "untrusted"}
        if lens and got["score"] > got["nominal"] and abs(math.expm1(got["theta"][6] / 100.0)) >= WRIST_FOCAL:
            return {**got, "verdict": "lens"}
        return {**got, "verdict": "trusted"}


# -- the host half ---------------------------------------------------------------------------------------------
def robot_outline(sam, image: np.ndarray) -> np.ndarray:
    """bool (H, W): SAM3's outline of the robot in a picture (see ROBOT_WORDS)."""
    out = np.zeros(image.shape[:2], bool)
    for seg in sam.segment(image, text=ROBOT_WORDS, max_masks=ROBOT_MAX_MASKS, min_score=ROBOT_MIN_SCORE):
        out |= np.asarray(seg.mask, bool)
    return out


def calibrate(host, log) -> dict | None:
    """The camera check at the start (host side): SAM3's outlines of the robot for the server's check, move and
    fit (see the module's notes).  Returns what it found ("need" false: nothing done), logged as a
    ``camera_check`` event; None when it could not be done (no perception, or a failure: then logged too, and the
    cameras stay as they are)."""
    scene = host.scene
    if scene is None:
        return None
    t0 = time.time()
    try:
        start = host.views()[AGENTVIEW].image
        found = dict(host._call("calib_check", mask=robot_outline(scene.sam, start))["found"])
        if found.get("need"):
            moved = host._call("calib_move")
            masks = {str(step): robot_outline(scene.sam, np.asarray(image, np.uint8))
                     for step, image in moved["images"].items()}
            found = dict(host._call("calib_fit", masks=masks)["found"])
    except Exception as exc:  # noqa: BLE001 - the episode goes on with the cameras as they are
        log({"kind": "camera_check_failed", "error": f"{type(exc).__name__}: {exc}"})
        return None
    found["seconds"] = round(time.time() - t0, 2)
    log({"kind": "camera_check", **found})
    return found
