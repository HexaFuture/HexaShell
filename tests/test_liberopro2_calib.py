"""Checks of LIBERO-Pro 2.0's camera check (:mod:`hexashell.benches.liberopro2.calib`) that need neither its
simulator nor SAM3: wrapped pictures found and undone, the camera models and the fixes, the fit on a drawn robot,
and the choice of model.  Run: ``python tests/test_liberopro2_calib.py`` (or with pytest) from the checkout, in the
policy environment.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from hexashell.benches.liberopro2 import calib as C  # noqa: E402

SIZE = 128


def _picture(seed: int) -> np.ndarray:
    """A smooth picture whose first and last lines differ (as a scene's top and bottom do)."""
    rng = np.random.default_rng(seed)
    ys, xs = np.mgrid[0:SIZE, 0:SIZE].astype(float)
    out = np.stack([ys * 255.0 / SIZE, xs * 255.0 / SIZE, 128 + 40 * np.sin((xs + ys) / 9.0 + rng.random())], axis=2)
    return np.clip(out, 0, 255).astype(np.uint8)


def test_wrapped_pictures():
    pictures = [_picture(0), _picture(1)]
    assert C.roll_of(pictures) == (0, 0)
    rolled = [np.roll(p, (37, 90), axis=(0, 1)) for p in pictures]
    assert C.roll_of(rolled) == (37, 90)
    assert all(np.array_equal(C.unroll(r, (37, 90)), p) for r, p in zip(rolled, pictures))
    # one axis only, and pictures that disagree on the seam are not taken as wrapped
    assert C.roll_of([np.roll(p, 50, axis=1) for p in pictures]) == (0, 50)
    assert C.roll_of([np.roll(pictures[0], 20, axis=0), np.roll(pictures[1], 60, axis=0)])[0] == 0


def test_camera_and_fix():
    pose0 = np.eye(4)
    pose0[:3, 3] = (0.5, 0.0, 1.0)
    theta = np.array([0.0, 0.0, 90.0, 1.0, 2.0, -3.0, 100.0 * np.log(0.8), 4.0])
    pose, focal, k1 = C.camera_of(theta, pose0, 300.0)
    assert np.allclose(pose[:3, 3], (0.5 + 0.01, 0.02, 1.0 - 0.03)) and np.allclose(pose[:3, 0], (0, 1, 0))
    assert abs(focal - 240.0) < 1e-9 and abs(k1 - 0.04) < 1e-12
    fix = C.Fix((3, 5), theta)
    cam = fix.camera({"pose": pose0, "focal": 300.0, "width": SIZE, "height": SIZE})
    assert np.allclose(cam["pose"], pose) and abs(cam["focal"] - focal) < 1e-9 and cam["width"] == SIZE
    assert fix.brief()["roll"] == [3, 5]
    # roll only: the picture un-wrapped as it was
    picture = _picture(2)
    assert np.array_equal(C.Fix((3, 5)).picture(np.roll(picture, (3, 5), axis=(0, 1))), picture)
    # with k1: what the camera pictured n (1 + k1 |n|^2) half-widths off shows n off, where the pinhole has it
    fix = C.Fix((0, 0), np.array([0, 0, 0, 0, 0, 0, 0, 8.0]))
    c = (SIZE - 1) / 2.0
    n = np.array([0.5, -0.3])
    pictured = c + c * n * (1.0 + 0.08 * float(n @ n))
    image = np.zeros((SIZE, SIZE, 3), np.uint8)
    x0, y0 = (int(round(v)) for v in pictured)
    image[y0 - 1:y0 + 2, x0 - 1:x0 + 2] = 255
    y, x = np.unravel_index(np.argmax(fix.picture(image)[..., 0]), (SIZE, SIZE))
    assert abs(x - (c + c * n[0])) <= 1.5 and abs(y - (c + c * n[1])) <= 1.5, (x, y, c + c * n)
    # the drawing agrees: a point drawn through the camera with k1 lands where the camera pictured it
    local = np.array([[n[0] * 0.5 * c / 100.0, -n[1] * 0.5 * c / 100.0, -0.5]])
    assert np.allclose(C._project(local, 100.0, 0.08, c)[0], pictured, atol=1e-3)


def _robot() -> list:
    """A toy robot: three boxes in front of a camera at the origin looking down -z."""
    out = []
    for centre, half in (((0.0, 0.0, -1.0), (0.10, 0.04, 0.04)), ((0.12, 0.06, -1.1), (0.04, 0.10, 0.04)),
                         ((-0.08, -0.1, -0.9), (0.05, 0.05, 0.05))):
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
        out.append(("arm", np.asarray(centre) + corners * half, None))
    return out


def test_fit_finds_a_changed_lens():
    pose0 = np.eye(4)
    outline = C.Outline(_robot())
    truth = np.array([0, 0, 0, 0, 0, 0, -25.0, 0.0])
    mask = outline.draw(*C.camera_of(truth, pose0, 100.0), 64)
    assert mask.any() and C.score(mask, mask) == 1.0 and C.covered(mask, mask) == 1.0
    got = C.fit([outline], [mask], [pose0], 100.0, 64, C.MODELS["lens"])
    assert got["nominal"] < 0.8 and got["score"] > 0.97, got
    assert abs(got["theta"][6] - truth[6]) < 3.0, got["theta"]


def test_choice():
    def fits(pose, lens, nominal=0.6):
        return {"pose": {"score": pose, "nominal": nominal}, "lens": {"score": lens, "nominal": nominal}}

    assert C.choose(fits(0.65, 0.66)) == "nominal"
    assert C.choose(fits(0.95, 0.70)) == "pose"
    assert C.choose(fits(0.90, 0.92)) == "lens"
    assert C.choose(fits(0.93, 0.90)) == "lens"  # within MARGIN: the lens, with fewer entries free
    assert C.choose(fits(0.96, 0.90)) == "pose"
    assert C.choose(fits(0.75, 0.65)) == "pose"  # the lens gains too little by itself


def test_nelder_mead():
    x, f, evals = C.nelder_mead(lambda v: float(((v - (1.0, -2.0)) ** 2).sum()), np.zeros(2), (0.5, 0.5), 400,
                                xtol=1e-6, ftol=1e-12)
    assert np.allclose(x, (1.0, -2.0), atol=1e-4) and f < 1e-8 and evals <= 400


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
