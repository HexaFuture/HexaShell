"""RoboColiseum Season II: the base's move in a drive, read from the head pictures before and after it."""

from __future__ import annotations

import math

import numpy as np
import pytest

from hexashell.benches.robocoliseum import odometry as Od


class _Cam:
    """A pinhole camera like the observations' (it looks along -z, +y up the picture; pose in base_link)."""

    def __init__(self, pose, fx=306.7, w=640, h=400):
        self.pose, self.fx, self.fy, self.cx, self.cy, self.w, self.h = pose, fx, fx, (w - 1) / 2, (h - 1) / 2, w, h


def _head_pose(tilt_deg=35.0, at=(0.27, 0.02, 1.48)):
    t = math.radians(tilt_deg)
    view = np.array([math.cos(t), 0.0, -math.sin(t)])
    right = np.array([0.0, -1.0, 0.0])
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2], pose[:3, 3] = right, np.cross(right, view), -view, at
    return pose


def _move(forward, left, turn_deg):
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:2, 3] = forward, left
    return m


_RNG = np.random.default_rng(7)
_TEXTURES = [_RNG.integers(0, 256, (97, 89)).astype(np.uint8) for _ in range(4)]


#: the simulator's depth noise: uniform, up to this many metres per square metre of depth either way (about
#: 3.9 cm at 1.32 m in the WBC scenes' head pictures)
NOISE_PER_M2 = 0.022


def _render(cam: _Cam, world_pose: np.ndarray, noise_seed: int | None = None):
    """A room (walls 3 m ahead and 2 m to each side, the floor), each face a blocky random pattern of 6 cm
    cells: the picture (grey) and its depth as the camera sees it from world_pose (with the simulator's kind of
    depth noise when noise_seed is given)."""
    v, u = np.mgrid[0:cam.h, 0:cam.w].astype(float)
    ray = np.stack([(u - cam.cx) / cam.fx, -(v - cam.cy) / cam.fy, -np.ones_like(u)], axis=-1)
    d = ray @ world_pose[:3, :3].T
    o = world_pose[:3, 3]
    best = np.full(u.shape, np.inf)
    grey = np.zeros(u.shape, np.uint8)
    faces = ((0, 3.0, (1, 2)), (1, 2.0, (0, 2)), (1, -2.0, (0, 2)), (2, 0.0, (0, 1)))
    for k, (axis, at, (a, b)) in enumerate(faces):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (at - o[axis]) / d[..., axis]
        hit = np.isfinite(t) & (t > 0) & (t < best)
        p = o + t[..., None] * d
        tex = _TEXTURES[k]
        ia = np.floor(p[..., a] / 0.06).astype(np.int64) % tex.shape[0]
        ib = np.floor(p[..., b] / 0.06).astype(np.int64) % tex.shape[1]
        best = np.where(hit, t, best)
        grey = np.where(hit, tex[ia, ib], grey)
    z = best.astype(np.float32)  # the rays have z = -1 in the camera: t is the depth along the optical axis
    if noise_seed is not None:
        r = np.random.default_rng(noise_seed)
        z = z + (r.uniform(-1.0, 1.0, z.shape) * NOISE_PER_M2 * z ** 2).astype(np.float32)
        z = np.round(z * 1000.0) / 1000.0  # millimetre steps, as the pictures come
    return grey.astype(np.uint8), z.astype(np.float32)


def _seen_after(move: np.ndarray, cam_pose=None, noise_seed=None):
    pose = _head_pose() if cam_pose is None else cam_pose
    cam = _Cam(pose)
    grey, depth = _render(cam, move @ pose, noise_seed)
    return Od._seen(grey, depth, cam, None)


@pytest.mark.parametrize("forward, left, turn", [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.3, -0.2, 15.0),
                                                 (0.0, 0.0, -30.0), (-0.4, 0.1, 5.0)])
def test_the_head_pictures_give_the_base_move(forward, left, turn):
    before = _seen_after(np.eye(4))
    move = Od.estimate(before, _seen_after(_move(forward, left, turn)))
    assert move is not None and move.agree >= Od.MIN_AGREE
    assert move.forward == pytest.approx(forward, abs=0.03) and move.left == pytest.approx(left, abs=0.03)
    assert move.turn_deg == pytest.approx(turn, abs=1.0)
    assert "matched points agree" in move.text()


#: the WBC head camera as it stands at the start: 8 degrees off straight down, 1.26 m above base_link's origin
_DOWN = (82.0, (0.59, 0.02, 1.26))


@pytest.mark.parametrize("forward, left, turn", [(0.3, 0.0, 0.0), (0.0, 0.4, 0.0), (0.0, 0.0, 30.0),
                                                 (0.6, 0.0, 15.0), (0.0, 0.0, 0.0)])
@pytest.mark.parametrize("tilt, at", [(35.0, (0.27, 0.02, 1.48)), _DOWN])
def test_noisy_depth_still_gives_the_base_move(forward, left, turn, tilt, at):
    """The simulator's depth is noisy (a few cm at a metre and more): the move still comes out within 1 cm
    and 0.5 degrees."""
    pose = _head_pose(tilt, at)
    before = _seen_after(np.eye(4), pose, noise_seed=1)
    move = Od.estimate(before, _seen_after(_move(forward, left, turn), pose, noise_seed=2))
    assert move is not None and move.agree >= 25
    assert move.forward == pytest.approx(forward, abs=0.01) and move.left == pytest.approx(left, abs=0.01)
    assert move.turn_deg == pytest.approx(turn, abs=0.5)


def test_a_half_turn_shares_too_little_to_tell():
    assert Od.estimate(_seen_after(np.eye(4)), _seen_after(_move(0.0, 0.0, 180.0))) is None


def test_no_pictures_no_estimate():
    assert Od.estimate(None, _seen_after(np.eye(4))) is None
    grey, depth = _render(_Cam(_head_pose()), _head_pose())
    assert Od._seen(grey, np.full_like(depth, np.nan), _Cam(_head_pose()), None) is None


def _seeing_link(goes: float):
    """A FakeLink of the WBC robot whose head sees the room from where the base is: the base goes ``goes`` of
    each drive asked (0: it is held)."""
    import dataclasses

    from test_robocoliseum_motion import FakeLink

    from hexashell.benches.robocoliseum import obs as O

    class SeeingLink(FakeLink):
        def __init__(self):
            super().__init__("g2_90d")
            self.base = np.eye(4)
            self.first = self._frame = self._seen(self._frame)

        def _seen(self, f):
            pose = _head_pose()
            cam = _Cam(pose)
            grey, depth = _render(cam, self.base @ pose)
            k = np.array([[cam.fx, 0.0, cam.cx], [0.0, cam.fy, cam.cy], [0.0, 0.0, 1.0]])
            calib = O.Calib(k, pose @ np.diag([1.0, -1.0, -1.0, 1.0]), cam.w, cam.h)
            return dataclasses.replace(f, images={"head": np.repeat(grey[..., None], 3, axis=2)},
                                       depth={"head": depth}, calib={"head": calib})

        def act(self, rows, need_depth):
            f = super().act(rows, need_depth)
            w = rows[-1].get("wheels")
            if w is not None:
                self.base = self.base @ _move(goes * w[0], goes * w[1], goes * math.degrees(w[2]))
            self._frame = self._seen(f)
            return self._frame

    return SeeingLink()


def test_a_drive_reports_the_move_the_head_pictures_read():
    from hexashell.benches.robocoliseum import motion
    d = motion.RobotDriver(_seeing_link(1.0), mobile=True)
    text = d.drive(forward=0.5, turn=10)
    assert "asked forward 0.50 m" in text and "the head pictures before and after read forward" in text
    assert "note: the base" not in text
    assert np.allclose(d.moved_by[:2, 3], [0.5, 0.0], atol=0.03)
    assert math.degrees(math.atan2(d.moved_by[1, 0], d.moved_by[0, 0])) == pytest.approx(10.0, abs=1.0)


def test_a_drive_that_did_not_move_or_fell_short_says_so():
    from hexashell.benches.robocoliseum import motion
    d = motion.RobotDriver(_seeing_link(0.0), mobile=True)
    text = d.drive(forward=0.5)
    assert "the base hardly moved" in text and np.allclose(d.moved_by, np.eye(4), atol=0.03)
    d = motion.RobotDriver(_seeing_link(0.5), mobile=True)
    text = d.drive(forward=0.6)
    assert "note: the base ended 0.30 m" in text and d.moved_by[0, 3] == pytest.approx(0.3, abs=0.03)
