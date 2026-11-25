"""RoboColiseum Season II: the simulator's noisy depth pictures, smoothed on the WBC board."""

from __future__ import annotations

import numpy as np

from hexashell.benches.robocoliseum import obs as O


def _noisy(z: np.ndarray, seed: int = 0) -> np.ndarray:
    """Depth as the simulator gives it: uniform per-pixel noise with a deviation of DEPTH_NOISE_SHARE of the
    depth, in millimetre steps."""
    r = np.random.default_rng(seed)
    half = O.DEPTH_NOISE_SHARE * np.sqrt(3.0)  # a uniform's deviation is its half-width over sqrt(3)
    noisy = z * (1.0 + r.uniform(-half, half, z.shape))
    return (np.round(noisy * 1000.0) / 1000.0).astype(np.float32)


def _scene():
    """What the WBC head camera sees, roughly: the floor 1.4 m away, a box top 5 cm above it, an arm 0.5 m
    away in one corner and a hole without readings."""
    z = np.full((200, 320), 1.4, np.float32)
    z[80:120, 140:190] = 1.35  # the box (40 x 50 px)
    z[:60, :100] = 0.5  # the arm
    z[150:160, 250:270] = np.nan
    return z


def test_smoothing_evens_out_the_noise():
    truth = _scene()
    raw = _noisy(truth)
    out = O.smooth_depth(raw)
    floor = (slice(130, 190), slice(20, 230))
    assert np.nanstd(raw[floor]) > 0.02 and np.nanstd(out[floor]) < 0.3 * np.nanstd(raw[floor])
    top = (slice(84, 116), slice(144, 186))  # the box top away from its edges
    assert abs(float(np.mean(out[top])) - 1.35) < 0.003
    lo, hi = np.percentile(out[top], [3, 97])
    assert hi - lo < 0.025 < np.subtract(*np.percentile(raw[top], [97, 3]))


def test_smoothing_keeps_edges_and_holes():
    truth = _scene()
    out = O.smooth_depth(_noisy(truth))
    # the arm's edge stays where it was: no reading in between the arm and the floor
    assert np.all(out[:60, :100] < 0.53) and np.all(out[60:62, :100] > 1.3) and np.all(out[:60, 100:102] > 1.3)
    # none stays none, and nothing else becomes none
    assert np.array_equal(np.isnan(out), np.isnan(truth))


def _request(depth_mm: np.ndarray) -> dict:
    h, w = depth_mm.shape
    cal = {"fx": 300.0, "fy": 300.0, "cx": (w - 1) / 2, "cy": (h - 1) / 2, "width": w, "height": h,
           "optical_to_frame": np.eye(4).tolist(), "available": True}
    rgb = np.full((h, w, 3), 128, np.uint8)
    images = {"head": {"image_data": rgb.tobytes(), "encoding": "RAW_RGB", "height": h, "width": w,
                       "calibration": cal},
              "head_depth": {"image_data": depth_mm.astype(np.uint16).tobytes(), "encoding": "RAW_UINT16",
                             "height": h, "width": w}}
    arms = [1.732, -1.1514, -1.5989, -1.9, -0.4202, 0.0041, 0.0008,
            -1.7277, -1.1473, 1.6009, -1.8991, 0.4222, -0.0013, -0.0007]
    return {"method": "infer", "params": {
        "robot_type": "g2a_sim", "ee_type": "wuji", "prompt": "p", "task_name": "t", "episode_idx": 0,
        "episode_done": False, "images": images,
        "states": {"arm_joint_states": arms, "gripper_states": [0.0] * 40,
                   "waist_joint_states": [0.0, 0.0, 0.349066, 0.0, 0.0], "head_joint_states": [0.0, 0.0, 0.0],
                   "end_pose": None}}}


def test_parse_smooths_the_depth_only_when_asked():
    raw = _noisy(np.full((48, 64), 1.4, np.float32))
    payload = _request(np.round(raw * 1000.0))
    plain = O.parse(payload, 1).depth["head"]
    smoothed = O.parse(payload, 1, smooth=True).depth["head"]
    assert np.allclose(plain, raw, atol=1e-6)
    assert np.std(smoothed) < 0.3 * np.std(plain)
