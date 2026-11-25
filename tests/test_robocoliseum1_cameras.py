"""benches/robocoliseum1/cameras.py and its use in obs.parse: the nominal camera models for requests without a
calibration (the online simulator's), and the online request fields dropped unread."""

import math

import cv2
import msgpack
import numpy as np
import pytest

from hexashell.benches.robocoliseum1 import cameras as C
from hexashell.benches.robocoliseum1 import fakegate as F
from hexashell.benches.robocoliseum1 import g2
from hexashell.benches.robocoliseum1 import obs as O
from hexashell.benches.robocoliseum1 import tunnel as T
from hexashell.geometry import invert

MOUNT_HEAD = C._t(0.05, 0.0, 0.08) @ C._r((0, 1, 0), 0.3) @ C._r((0, 0, 1), -math.pi / 2)
MOUNT_HAND = C._t(0.0, 0.03, 0.06) @ C._r((1, 0, 0), 0.2)
MODEL = {
    "head": {"size": [640, 400], "fx": 300.0, "fy": 300.0, "cx": 320.0, "cy": 200.0, "distortion": [0.0] * 8,
             "mount": MOUNT_HEAD.tolist()},
    "hand_left": {"size": [640, 528], "fx": 250.0, "fy": 250.0, "cx": 320.0, "cy": 264.0, "distortion": [0.0] * 8,
                  "mount": MOUNT_HAND.tolist()},
    "hand_right": {"size": [640, 528], "fx": 250.0, "fy": 250.0, "cx": 320.0, "cy": 264.0, "distortion": [0.0] * 8,
                   "mount": MOUNT_HAND.tolist()},
}
WAIST = [-0.83, 1.22, 0.1, 0.0, 0.3]
HEAD = [0.1, -0.05, 0.2]


def _quat(m):
    r = m[:3, :3]
    w = math.sqrt(max(0.0, 1 + np.trace(r))) / 2
    return [(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w]


def _pose(m):
    return {"position": m[:3, 3].tolist(), "orientation": _quat(m)}


def _jpeg(w, h):
    return cv2.imencode(".jpg", np.full((h, w, 3), 90, np.uint8))[1].tobytes()


def request(calibrated: bool = False, hand=(1280, 1056)):
    arm_base = C.arm_base_from_waist(WAIST)
    end_abl = {"left": C._t(0.3, 0.2, 0.1), "right": C._t(0.3, -0.2, 0.1) @ C._r((0, 0, 1), 0.4)}
    end_pose = {"base_link": {f"{a}_arm": _pose(arm_base @ end_abl[a]) for a in g2.ARMS},
                "arm_base_link": {f"{a}_arm": _pose(end_abl[a]) for a in g2.ARMS}}
    images = {"head": {"encoding": "JPEG", "width": 640, "height": 400, "image_data": _jpeg(640, 400)}}
    for wire in ("hand_left", "hand_right"):
        images[wire] = {"encoding": "JPEG", "width": hand[0], "height": hand[1], "image_data": _jpeg(*hand)}
    if calibrated:
        for wire, entry in images.items():
            k = MODEL[wire]
            entry["calibration"] = {"available": True, "fx": k["fx"], "fy": k["fy"], "cx": k["cx"], "cy": k["cy"],
                                    "width": k["size"][0], "height": k["size"][1], "distortion": [0.0] * 8,
                                    "optical_to_frame": np.eye(4).tolist()}
    return {"method": "infer", "params": {
        "images": images, "prompt": "p", "task_name": "t", "episode_done": False, "task_progress": [{"x": 1}],
        "states": {"arm_joint_states": [0.0] * 14, "gripper_states": [0.0, 0.0], "end_pose": end_pose,
                   "waist_joint_states": WAIST, "head_joint_states": HEAD}}}


@pytest.fixture
def model(monkeypatch):
    monkeypatch.setattr(C, "_model", MODEL)
    monkeypatch.setattr(O, "_seen_real", set())
    monkeypatch.setattr(O, "_last", {})
    monkeypatch.setattr(O, "_on_hand", {})


def test_nominal_cameras_follow_the_readings(model):
    frame = O.parse(request(), 1)
    arm_base = C.arm_base_from_waist(WAIST)
    assert np.allclose(frame.arm_base, arm_base, atol=1e-9)
    head = frame.calib["head"]
    want = arm_base @ invert(C.B5_AB) @ C.B5_HB @ C.chain(C.HEAD, HEAD) @ MOUNT_HEAD
    assert np.allclose(head.pose, want, atol=1e-9) and (head.width, head.height) == (640, 400)
    left = frame.calib["left_wrist"]
    assert np.allclose(left.pose, frame.end_link("left") @ MOUNT_HAND, atol=1e-9)
    # 1280x1056 is brought to 640x528 with the model's intrinsics (given at 640x528)
    assert (left.width, left.height) == (640, 528) and frame.images["left_wrist"].shape[:2] == (528, 640)
    assert np.isclose(left.k[0, 0], 250.0) and np.isclose(left.k[1, 2], 264.0)
    assert frame.camera("head") is not None and frame.camera("right_wrist") is not None


def test_no_head_reading_takes_the_start_head(model):
    req = request()
    req["params"]["states"]["head_joint_states"] = []
    frame = O.parse(req, 1)
    want = C.arm_base_from_waist(WAIST) @ invert(C.B5_AB) @ C.B5_HB @ C.chain(C.HEAD, C.HEAD_REST) @ MOUNT_HEAD
    assert np.allclose(frame.calib["head"].pose, want, atol=1e-9)
    cal = C.nominal("head", arm_base=C.arm_base_from_waist(WAIST), head=None, end_links={}, task_name="scoop_popcorn")
    assert cal["head_rest"] and np.allclose(
        cal["optical_to_frame"],
        C.arm_base_from_waist(WAIST) @ invert(C.B5_AB) @ C.B5_HB @ C.chain(C.HEAD, (0, 0, 0)) @ MOUNT_HEAD, atol=1e-9)
    assert C.head_rest("g2op_manip_take_wrong_item_shelf") == (0.0, 0.0, 0.1745)
    assert not C.nominal("head", arm_base=np.eye(4), head=HEAD, end_links={})["head_rest"]


def test_a_real_calibration_wins(model):
    O.parse(request(calibrated=True, hand=(640, 528)), 1)
    frame = O.parse(request(), 2)  # no calibration now: the one seen is carried over, not the model
    assert np.allclose(frame.calib["head"].pose, np.eye(4))
    assert frame.calib["head"].nominal


def test_no_model_no_camera(monkeypatch):
    monkeypatch.setattr(C, "_model", None)
    monkeypatch.setattr(C, "MODEL_FILE", C.MODEL_FILE.with_name("missing.json"))
    monkeypatch.setattr(O, "_seen_real", set())
    monkeypatch.setattr(O, "_last", {})
    frame = O.parse(request(), 1)
    assert frame.camera("head") is None and "head" in frame.images


def test_model_file_fits_the_head_chain():
    """The shipped model's numbers are usable: sizes as the cameras send them, a rigid mount."""
    m = C.model()
    if m is None:
        pytest.skip("no g2_cameras.json")
    assert m["head"]["size"] == [640, 400] and m["hand_left"]["size"] == [640, 528]
    for wire in C.WIRE:
        r = np.asarray(m[wire]["mount"])[:3, :3]
        assert np.allclose(r.T @ r, np.eye(3), atol=1e-6) and np.isclose(np.linalg.det(r), 1.0, atol=1e-6)
    # the fitted head chain: HEAD's axes, origins a few mm from the URDF's
    chain = m["head"]["chain"]
    assert [tuple(ax) for _, ax in chain] == [tuple(ax) for _, ax in C.HEAD]
    assert max(np.abs(np.subtract(o, o0)).max() for (o, _), (o0, _) in zip(chain, C.HEAD)) < 0.006


def test_tunnel_drops_task_progress():
    payload = msgpack.packb(request(), use_bin_type=True)
    plain, changed = T.plain_request(payload)
    back = msgpack.unpackb(plain, raw=False)
    assert changed and "task_progress" not in back["params"] and back["params"]["prompt"] == "p"
    probe = T.calibration_probe(plain)
    assert probe["states_lengths"]["head_joint_states"] == 3 and "task_progress" not in probe["params_keys"]


def test_fakegate_like_online():
    p = request(calibrated=True, hand=(640, 528))["params"]
    p["images"]["hand_left_depth"] = {"encoding": "RAW_UINT16", "width": 640, "height": 528,
                                      "image_data": np.arange(640 * 528, dtype=np.uint16).tobytes()}
    p["segments"] = ["a"]
    F.like_online(p)
    assert all("calibration" not in e for e in p["images"].values()) and "segments" not in p
    assert p["states"]["head_joint_states"] == []
    assert (p["images"]["hand_left"]["width"], p["images"]["hand_left"]["height"]) == (1280, 1056)
    d = np.frombuffer(p["images"]["hand_left_depth"]["image_data"], np.uint16).reshape(1056, 1280)
    assert d[1, 1] == 0 and d[2, 2] == 641
    assert p["task_progress"]
