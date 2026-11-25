"""The real robot run by URAI (hexashell/real/urai), without URAI: the frames, the capsules and the cameras
against closed forms, the host against the fake service of ``tests/fakes_urai.py`` through the real HTTP
client, and episodes end to end.  The checks against URAI's own simulator are in test_real_urai_sim.py
(they need URAI installed).
"""

from __future__ import annotations

import json
import math
import os
import re

import numpy as np
import pytest

import fakes
import fakes_urai as F
from hexashell.geometry import angle_between, exp_so3, homogeneous, rot_x, rot_y, rot_z
from hexashell.kernel.host import CallFailed, Stop
from hexashell.motion.pose import rotation_of
from hexashell.real.urai import texts
from hexashell.real.urai.capsules import Capsule, capsule_hit
from hexashell.real.urai.frames import (from_rpy, hand_axes, hand_move, hand_move_between, rotation_rpy, rpy_of,
                                        tcp_pose, to_rpy)
from hexashell.real.urai.host import STEP_S, UraiHost
from hexashell.real.urai.observation import CAMERA_TO_URAI, Observation
from hexashell.vision.cameras import Camera, check_cam
from hexashell.vision.rgbd import DepthScene, DepthView, spot

TOP = (F.BLOCK_XY[0], F.BLOCK_XY[1], F.TABLE_Z + F.EDGE_M)  # the middle of the block's top


def _host(fake=None, **kw):
    fake = fake or F.FakeUrai()
    events, states, pictures = [], [], []
    host = UraiHost(fake.client(events.append), "Move the red block to (0.20, -0.02).", task_name="move_block",
                    log=events.append, attach=lambda kind, image, caption: pictures.append((kind, image, caption)),
                    record_state=states.append, **kw)
    return fake, host, events, states, pictures


def _scene(log=None):
    return DepthScene("fake", cams=list(CAMERA_TO_URAI), check_cam=check_cam, log=log, client=F.FakeSam3())


def _steps(plans) -> int:
    return sum(math.ceil((p["duration_s"] + F.FakeUrai().overhead_s) / STEP_S) for p in plans)


def _failed(kind: str, host, name: str, *args, **kwargs) -> str:
    """The text of the ``kind`` error the robot call raises in the sandbox."""
    with pytest.raises(CallFailed) as caught:
        host.dispatch(name, args, kwargs)
    assert caught.value.kind == kind, (caught.value.kind, caught.value.text)
    return caught.value.text


# -- frames ----------------------------------------------------------------------------------------
@pytest.mark.parametrize("yaw, closing", [(0.0, (-1.0, 0.0, 0.0)), (90.0, (0.0, -1.0, 0.0))])
def test_top_down_tool_axes(yaw, closing):
    """URAI's tool approaches along its +z and closes along its +x."""
    rot = rotation_rpy(to_rpy(90.0, yaw, 0.0))
    assert np.allclose(rot[:, 2], (0.0, 0.0, -1.0), atol=1e-12) and np.allclose(rot[:, 0], closing, atol=1e-12)


def test_roll_and_rpy_order():
    rot = rotation_rpy(to_rpy(0.0, 0.0, 30.0))
    assert np.allclose(rot[:, 2], (0.0, 1.0, 0.0), atol=1e-12)
    assert np.allclose(rot[:, 0], (-math.sqrt(3) / 2, 0.0, 0.5), atol=1e-12)
    # about the fixed axes, x first: R = Rz(c) Ry(b) Rx(a)
    assert np.allclose(rotation_rpy([90.0, 0.0, 90.0])[:, 2], (1.0, 0.0, 0.0), atol=1e-12)
    assert np.allclose(rotation_rpy([180.0, 0.0, 180.0]), np.diag([-1.0, 1.0, -1.0]), atol=1e-12)


def test_rpy_round_trips():
    rng = np.random.default_rng(7)
    for a, b, c in rng.uniform((-179, -89, -179), (179, 89, 179), (200, 3)):
        assert np.allclose(rpy_of(rotation_rpy([a, b, c])), (a, b, c), atol=1e-9)
    for b in (90.0, -90.0):  # gimbal lock: c is given as 0, the rotation stays the same
        back = rpy_of(rotation_rpy([30.0, b, 20.0]))
        assert back[1] == pytest.approx(b) and back[2] == 0.0
        assert np.allclose(rotation_rpy(back), rotation_rpy([30.0, b, 20.0]), atol=1e-12)
    for angles in ((90.0, 0.0, 0.0), (90.0, 45.0, 0.0), (60.0, -30.0, 10.0), (0.0, 0.0, 30.0), (90.0, -120.0, 0.0)):
        assert np.allclose(from_rpy(to_rpy(*angles)), angles, atol=1e-9), angles
    assert np.allclose(from_rpy(F.DOWN_RPY), (90.0, 0.0, 0.0), atol=1e-12)
    pose = tcp_pose({"xyz": [0.1, 0.2, 0.3], "rpy_deg": [10.0, 20.0, 30.0]})
    assert np.allclose(pose, homogeneous(rotation_rpy([10.0, 20.0, 30.0]), (0.1, 0.2, 0.3)))


def test_rpy_is_scipys_xyz():
    Rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    rng = np.random.default_rng(3)
    for rpy in rng.uniform((-180, -90, -180), (180, 90, 180), (200, 3)):
        rot = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
        assert np.allclose(rotation_rpy(rpy), rot, atol=1e-12)
        assert np.allclose(rpy_of(rot), Rotation.from_matrix(rot).as_euler("xyz", degrees=True), atol=1e-8)


# -- the hand's own axes ---------------------------------------------------------------------------
@pytest.mark.parametrize("angles", [(0.0, 0.0, 0.0), (90.0, 0.0, 0.0), (45.0, 30.0, -20.0), (-30.0, 150.0, 170.0)])
def test_hand_axes_are_the_tool_axes_of_pitch_yaw_roll(angles):
    """forward = where the fingers point (URAI's tool +z), left = the closing line (its +x), up = forward x left."""
    axes, tool = hand_axes(to_rpy(*angles)), rotation_rpy(to_rpy(*angles))
    assert np.allclose(axes, rotation_of(*angles), atol=1e-12)
    assert np.allclose(axes[:, 0], tool[:, 2], atol=1e-12) and np.allclose(axes[:, 1], tool[:, 0], atol=1e-12)
    assert np.allclose(axes[:, 2], np.cross(axes[:, 0], axes[:, 1]), atol=1e-12)


def test_hand_axes_of_a_level_and_a_downward_hand():
    level = hand_axes(to_rpy(0.0, 0.0, 0.0))  # pointing along world +y: left is world -x, up is world up
    assert np.allclose(level, [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], atol=1e-12)
    down = hand_axes(to_rpy(90.0, 0.0, 0.0))  # pointing down, closing along world x
    assert np.allclose(down[:, 0], (0.0, 0.0, -1.0), atol=1e-12) and np.allclose(down[:, 1], (-1.0, 0.0, 0.0), atol=1e-12)
    assert np.allclose(down[:, 2], (0.0, 1.0, 0.0), atol=1e-12)


def _hand_pose(tcp) -> np.ndarray:
    return homogeneous(hand_axes(tcp["rpy_deg"]), tcp["xyz"])


def _random_tcp(rng) -> dict:
    return {"xyz": rng.uniform(-0.5, 0.5, 3).tolist(), "rpy_deg": rng.uniform((-180, -90, -180), (180, 90, 180)).tolist()}


def test_hand_move_composes_in_the_start_axes_and_hand_move_between_inverts_it():
    """end = start @ [exp(hand_turn), hand_by] as poses in the hand's axes; the labels come back exactly, up to
    turns of 179 degrees."""
    rng = np.random.default_rng(11)
    for _ in range(300):
        start = _random_tcp(rng)
        shift = rng.uniform(-0.2, 0.2, 3)
        axis = rng.normal(size=3)
        turn = axis / np.linalg.norm(axis) * rng.uniform(0.0, 179.0)
        xyz, rpy = hand_move(start["xyz"], start["rpy_deg"], shift, turn)
        end = {"xyz": xyz.tolist(), "rpy_deg": rpy}
        assert np.allclose(_hand_pose(end), _hand_pose(start) @ homogeneous(exp_so3(np.radians(turn)), shift), atol=1e-9)
        back_shift, back_turn = hand_move_between(start, end)
        assert np.allclose(back_shift, shift, atol=1e-9) and np.allclose(back_turn, turn, atol=1e-6)
    # the shift alone keeps the orientation; a turn about forward alone is a roll
    xyz, rpy = hand_move((0.2, 0.0, 0.2), to_rpy(45.0, 30.0, -20.0), (0.05, 0.0, 0.0))
    assert np.allclose(xyz, np.add((0.2, 0.0, 0.2), 0.05 * rotation_of(45.0, 30.0, -20.0)[:, 0]), atol=1e-12)
    assert np.allclose(rotation_rpy(rpy), rotation_rpy(to_rpy(45.0, 30.0, -20.0)), atol=1e-12)
    _, rpy = hand_move((0.2, 0.0, 0.2), to_rpy(45.0, 30.0, -20.0), hand_turn=(25.0, 0.0, 0.0))
    assert np.allclose(from_rpy(rpy), (45.0, 30.0, 5.0), atol=0.01)


def test_hand_moves_do_not_depend_on_the_world_frame():
    """The same two TCP poses seen from another frame (say the other arm's base) give the same hand_by and
    hand_turn: a recorded motion's hand moves need no calibration between the bases."""
    rng = np.random.default_rng(5)
    other = homogeneous(rotation_rpy([10.0, -20.0, 75.0]), (0.1, -0.6, 0.02))

    def seen(tcp):
        pose = other @ tcp_pose(tcp)
        return {"xyz": pose[:3, 3].tolist(), "rpy_deg": rpy_of(pose[:3, :3])}

    for _ in range(100):
        a, b = _random_tcp(rng), _random_tcp(rng)
        assert np.allclose(np.ravel(hand_move_between(a, b)), np.ravel(hand_move_between(seen(a), seen(b))), atol=1e-6)


# -- capsules and pictures -----------------------------------------------------------------------
def test_capsule_hit_matches_closed_form():
    """Rays straight down (direction length 2) onto a capsule along x at z = 0.3: its body, its ends, misses
    (none of them grazes it)."""
    cap = Capsule(np.array([-0.1, 0.0, 0.3]), np.array([0.1, 0.0, 0.3]), 0.05)
    xs, ys = np.meshgrid(np.linspace(-0.2, 0.2, 40), np.linspace(-0.08, 0.08, 16))
    o = np.stack([xs.ravel(), ys.ravel(), np.ones(xs.size)], axis=1)
    d = np.tile([0.0, 0.0, -2.0], (len(o), 1))
    t, normal, hit = capsule_hit(o, d, cap)
    end = np.clip(o[:, 0], -0.1, 0.1)  # the nearest point of the segment, seen from above
    reach2 = 0.05 ** 2 - (o[:, 0] - end) ** 2 - o[:, 1] ** 2
    assert np.array_equal(hit, reach2 > 0)
    z = 0.3 + np.sqrt(reach2[hit])
    assert np.allclose(t[hit], (1.0 - z) / 2.0, atol=1e-12) and np.all(np.isinf(t[~hit]))
    expected = np.stack([o[hit, 0] - end[hit], o[hit, 1], z - 0.3], axis=1) / 0.05
    assert np.allclose(normal[hit], expected, atol=1e-9)


def test_camera_from_cv_keeps_the_calibration():
    k = np.array([[420.0, 0.0, 113.2], [0.0, 610.0, 72.7], [0.0, 0.0, 1.0]])
    pose = homogeneous(rot_z(0.16) @ rot_y(-0.56) @ rot_x(0.3), (0.2, -0.1, 0.5))
    camera = Camera.from_cv("head", k, pose, 240, 160)
    local = np.array([[0.0, 0.0, 1.0], [0.05, -0.03, 0.8], [-0.08, 0.04, 1.4]])  # x right, y down, z ahead
    world = local @ pose[:3, :3].T + pose[:3, 3]
    pixels, depths = camera.project(world)
    assert np.allclose(pixels, np.stack([420.0 * local[:, 0] / local[:, 2] + 113.2,
                                         610.0 * local[:, 1] / local[:, 2] + 72.7], axis=1), atol=1e-9)
    assert np.allclose(depths, local[:, 2], atol=1e-12)
    rays = world - pose[:3, 3]
    assert np.allclose(camera.rays(pixels), rays / np.linalg.norm(rays, axis=1, keepdims=True), atol=1e-12)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == (420.0, 610.0, 113.2, 72.7)
    assert np.allclose(camera.pose, pose @ np.diag([1.0, -1.0, -1.0, 1.0]))
    info = camera.info()
    assert info.focal_px == 420.0 and info.fy_px == 610.0 and np.array_equal(info.k, k)
    plain = Camera("head", 420.0, pose, 240, 160).info()
    assert plain.fy_px is None and plain.k is None and plain.center_px == (119.5, 79.5)


def test_camera_with_its_own_matrix():
    k = np.array([[200.0, 0.0, 39.5], [0.0, 200.0, 29.5], [0.0, 0.0, 1.0]])
    camera = Camera("head", 200.0, np.eye(4), 80, 60, k=k)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == (200.0, 200.0, 39.5, 29.5)
    assert np.allclose(camera.project([[0.1, 0.2, -1.0]])[0], [[59.5, -10.5]])


def _top_view(fx, fy, name="head", x=0.0, depth=0.5):
    k = np.array([[fx, 0.0, 63.0], [0.0, fy, 60.0], [0.0, 0.0, 1.0]])
    camera = Camera.from_cv(name, k, F.looking_down((x, 0.0, 1.0)), 128, 128)
    return DepthView(name, np.zeros((128, 128, 3), np.uint8), np.full((128, 128), depth, np.float32), camera,
                     np.zeros((128, 128), bool), frame_id=f"{name}-frame")


@pytest.mark.parametrize("fx, fy", [(120.0, 240.0), (240.0, 120.0)])
def test_depth_and_spots_use_both_focal_lengths(fx, fy):
    view = _top_view(fx, fy)
    pixels = np.array([[30.0, 45.0], [90.0, 100.0]])
    cloud = view.back_project(pixels[:, 0], pixels[:, 1], np.full(2, 0.5))
    expected = np.stack([(pixels[:, 0] - 63.0) / fx * 0.5, -(pixels[:, 1] - 60.0) / fy * 0.5, np.full(2, 0.5)], 1)
    assert np.allclose(cloud, expected, atol=1e-12)
    assert np.allclose(view.camera.project(cloud)[0], pixels)
    for plane in (None, 0.5):
        measured = spot({"head": view}, "head", (63.0, 60.0), z=plane)
        assert np.allclose(measured.xyz, (0.0, 0.0, 0.5))
        assert measured.mm_per_px == pytest.approx(round(500.0 / min(fx, fy), 1 if plane else 2))


def test_paired_spot_uses_each_cameras_focal_lengths():
    views = {name: _top_view(240.0, 120.0, name, x) for name, x in (("head", -0.15), ("left_wrist", 0.15))}
    point = np.array([0.0, 0.0, 0.2])
    first, second = views["head"].camera, views["left_wrist"].camera
    result = spot(views, "head", first.pixel(point), pair=("left_wrist", second.pixel(point)))
    assert np.allclose(result.xyz, point)
    angle = 2.0 * np.arctan(0.15 / 0.8)
    assert result.mm_per_px == round(1000.0 * np.linalg.norm(point - first.position) / 120.0 / np.sin(angle), 1)


def test_measurement_without_depth_keeps_the_masks_frame():
    view = _top_view(120.0, 180.0, depth=np.nan)
    outline = np.zeros((128, 128), bool)
    outline[50:70, 50:70] = True
    scene = DepthScene("fake", cams=["head"], check_cam=lambda name: name, client=F.FakeSam3())
    try:
        measured = scene.measure([scene.register(view, outline, "block", 1.0, {})])
        assert not measured.found and measured.frame_ids == {"head": "head-frame"}
    finally:
        scene.close()


def test_packet_to_view_and_the_robots_pixels():
    """A sphere-shaped capsule under a camera: its pixels are the robot's, except where something stands in
    front of it or the depth is missing; the arrays come through as they were."""
    fx, fy, cx, cy, width, height = 150.0, 140.0, 63.5, 47.5, 128, 96
    pose = F.looking_down((0.0, 0.0, 1.0))
    us, vs = np.meshgrid(np.arange(width, dtype=float), np.arange(height, dtype=float))
    d = np.stack([(us - cx) / fx, -(vs - cy) / fy, -np.ones_like(us)], -1)  # world rays with unit depth
    n2 = (d ** 2).sum(-1)
    disc = 0.25 - n2 * (0.25 - 0.1 ** 2)  # ray from (0, 0, 1) against the ball of radius 0.1 at (0, 0, 0.5)
    on = disc > 1e-6
    depth = np.ones((height, width), np.float32)  # the table at z = 0
    depth[on] = ((0.5 - np.sqrt(disc[on])) / n2[on]).astype(np.float32)
    in_front = np.zeros_like(on)
    in_front[44:52, 60:68] = True
    depth[in_front] = 0.3
    depth[47, 50] = np.nan
    rgb = np.random.default_rng(1).integers(0, 255, (height, width, 3), dtype=np.uint8)
    arms = {"left": {"xyz": [0.0, 0.0, 0.5]}}
    packet = {"id": "frame-7", "timestamp": 3.5, "camera": "hand_left", "rgb": F.encode(rgb), "depth": F.encode(depth),
              "k": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], "t_world_camera": pose.tolist(), "arms": arms,
              "capsules": {"left": [{"name": "wrist", "a": [0.0, 0.0, 0.5], "b": [0.0, 0.0, 0.5], "radius_m": 0.1}],
                           "right": []}}
    observation = Observation.from_packet(packet, step=17)
    view = observation.view
    assert observation.id == view.frame_id == "frame-7" and observation.timestamp == 3.5 and observation.arms == arms
    assert view.cam == view.camera.name == "left_wrist" and view.step == 17
    assert view.image.dtype == np.uint8 and np.array_equal(view.image, rgb)
    assert view.depth.dtype == np.float32 and np.array_equal(view.depth, depth, equal_nan=True)
    assert (view.camera.fx, view.camera.fy, view.camera.cx, view.camera.cy) == (fx, fy, cx, cy)
    expected = on & ~in_front
    expected[47, 50] = False
    assert expected.sum() > 300 and np.array_equal(view.robot, expected)


# -- the host against the fake service ------------------------------------------------------------
def test_host_reads_the_robot():
    fake, host, events, states, pictures = _host(step_limit=500)
    status = host.state()
    assert status.step == 0 and status.step_limit == 500
    left = status["left"]
    assert np.allclose(left.at, F.START_XYZ["left"]) and np.allclose(left.angles, (90.0, 0.0, 0.0))
    assert left.gripper == 1.0 and left.holding is False and left.home_dist_m == 0.0 and left.point == "TCP"
    assert "TCP" in str(status) and "grasp point" not in str(status)
    task = host.task()
    assert task.instruction.startswith("Move the red block") and task.task_name == "move_block"
    assert "TCP" in str(task) and "grasp point" not in str(task)
    info = host.camera("head")
    assert info.fy_px == 220.0 and info.k == F.HEAD_K and info.focal_px == 240.0
    assert host.image("left_wrist").shape == (F.HEIGHT, F.WIDTH, 3)
    xy = host.to_pixel(TOP, "head")
    assert np.allclose(host.to_world("head", xy, z=TOP[2]).xyz, TOP, atol=1e-9)
    assert np.allclose(host.to_world("head", xy).xyz, TOP, atol=0.002)  # from the depth picture
    assert [r["body"]["camera"] for r in fake.requests if r["path"] == "/api/harness/observe"][:3] == [
        "head", "hand_left", "hand_right"]
    assert not fake.executed and host.step == 0


def test_goto_moves_through_urai_and_counts_its_time():
    fake, host, events, states, _ = _host(step_limit=500)
    move = host.goto("left", (0.27, -0.12, 0.12), pitch=90, yaw=0, roll=0, grip="open", path="line")
    assert move.ok and move.moved and move.reached and move.path == "line" and move.err_mm < 1e-6
    assert move.steps == host.step == _steps(fake.executed) > 0 and move.steps_left == 500 - host.step
    draft = fake.drafts[-1]
    assert draft["completion"] == "hold" and draft["observation_id"] == fake.head_id
    spec = draft["arms"]["left"]
    assert spec == {"primitive": "goto", "xyz": [0.27, -0.12, 0.12], "rpy_deg": to_rpy(90.0, 0.0, 0.0),
                    "motion": "line", "speed_m_s": 0.12, "opening_mm": F.MAX_OPENING_MM}
    preview = next(r for r in reversed(fake.requests) if r["path"] == "/api/preview")
    assert preview["query"] == {"trajectory": "full"} and preview["body"]["planning_only"] is False
    # the client read the state till the execution ended, and logged it
    assert [e["execution"]["state"] for e in events if e["kind"] == "urai_execution"] == ["completed"]
    assert states[-1]["step"] == host.step and states[-1]["execution"]["state"] == "completed"
    # angles left out keep the last ones asked for
    second = host.goto("left", by=(0.0, 0.0, -0.05), yaw=30)
    assert second.ok and second.target_angles == (90.0, 30.0, 0.0)
    third = host.goto("left", by=(0.01, 0.0, 0.0))
    assert third.ok and third.target_angles == second.target_angles
    assert fake.drafts[-1]["arms"]["left"]["rpy_deg"] == fake.drafts[-2]["arms"]["left"]["rpy_deg"]
    assert np.allclose(third.target, np.add(second.target, (0.01, 0.0, 0.0)))
    host.goto("left", by=(0.0, 0.0, 0.01), gentle=True)
    assert fake.drafts[-1]["arms"]["left"]["speed_m_s"] == 0.12  # gentle caps at 0.5 m/s
    # a goto that ends off its goal is not reached
    fake.miss = (0.0, 0.0, 0.01)
    missed = host.goto("left", by=(0.0, 0.0, 0.01))
    assert missed.moved and not missed.reached and missed.err_mm == pytest.approx(10.0)
    assert host.step == _steps(fake.executed)


def test_hand_by_moves_along_the_asked_axes_from_the_measured_tcp():
    fake, host, *_ = _host()
    fake.miss = (0.002, 0.0, 0.0)
    first = host.goto("left", (0.27, -0.12, 0.12), pitch=45, yaw=30, roll=-20)
    fake.miss = (0.0, 0.0, 0.0)
    asked_rpy = fake.drafts[-1]["arms"]["left"]["rpy_deg"]
    fake.arms["left"]["rpy_deg"] = to_rpy(48.0, 33.0, -20.0)  # the hand a few degrees off what was asked
    move = host.goto("left", hand_by=(0.03, -0.01, 0.02))
    spec = fake.drafts[-1]["arms"]["left"]
    axes = rotation_of(45.0, 30.0, -20.0)
    assert np.allclose(spec["xyz"], np.add(first.at, axes @ (0.03, -0.01, 0.02)), atol=1e-12)
    assert spec["rpy_deg"] == asked_rpy and move.target_angles == first.target_angles == (45.0, 30.0, -20.0)
    assert move.ok and np.allclose(move.target, spec["xyz"])
    # the same motion as by= with the world offset axes @ hand_by
    host.goto("left", by=tuple(axes @ (0.0, 0.0, -0.02)), dry_run=True)
    host.goto("left", hand_by=(0.0, 0.0, -0.02), dry_run=True)
    by_spec, hand_spec = (fake.drafts[i]["arms"]["left"] for i in (-2, -1))
    assert np.allclose(hand_spec.pop("xyz"), by_spec.pop("xyz"), atol=1e-12) and hand_spec == by_spec


def test_hand_turn_turns_about_the_tcp_and_what_follows_keeps_it():
    fake, host, *_ = _host()
    first = host.goto("left", (0.27, -0.12, 0.12), pitch=45, yaw=30, roll=-20)
    rolled = host.goto("left", hand_turn=(25.0, 0.0, 0.0))  # about forward: a roll
    assert rolled.ok and np.allclose(rolled.target, first.at) and np.allclose(rolled.target_angles, (45.0, 30.0, 5.0))
    start = hand_axes(host.aim["left"])
    turn = (10.0, -30.0, 40.0)
    turned = host.goto("left", hand_turn=turn)
    spec = fake.drafts[-1]["arms"]["left"]
    assert turned.ok and np.allclose(spec["xyz"], rolled.at, atol=1e-12)
    assert np.allclose(hand_axes(spec["rpy_deg"]), start @ exp_so3(np.radians(turn)), atol=1e-12)
    assert turned.target_angles == host.asked["left"] == from_rpy(spec["rpy_deg"]) and host.aim["left"] == spec["rpy_deg"]
    # omitted angles, by= and hand_by keep the turned orientation exactly
    assert host.goto("left", by=(0.0, 0.0, 0.01)).target_angles == turned.target_angles
    assert fake.drafts[-1]["arms"]["left"]["rpy_deg"] == spec["rpy_deg"]
    host.goto("left", hand_by=(0.01, 0.0, 0.0))
    assert fake.drafts[-1]["arms"]["left"]["rpy_deg"] == spec["rpy_deg"]
    assert np.allclose(np.subtract(fake.drafts[-1]["arms"]["left"]["xyz"], fake.drafts[-2]["arms"]["left"]["xyz"]),
                       0.01 * hand_axes(spec["rpy_deg"])[:, 0], atol=1e-12)
    # and the opposite turn comes back
    host.goto("left", hand_turn=tuple(-v for v in turn))
    assert math.degrees(angle_between(hand_axes(fake.drafts[-1]["arms"]["left"]["rpy_deg"]), start)) < 1e-9


def test_hand_by_with_hand_turn_shifts_along_the_axes_before_the_turn():
    fake, host, *_ = _host()
    first = host.goto("left", (0.27, -0.12, 0.12), pitch=60, yaw=-40, roll=10)
    move = host.goto("left", hand_by=(0.02, 0.01, -0.01), hand_turn=(0.0, 0.0, 90.0), grip=0.5, path="line")
    spec = fake.drafts[-1]["arms"]["left"]
    axes = rotation_of(60.0, -40.0, 10.0)
    assert np.allclose(spec["xyz"], np.add(first.at, axes @ (0.02, 0.01, -0.01)), atol=1e-12)
    assert np.allclose(hand_axes(spec["rpy_deg"]), axes @ rot_z(math.pi / 2), atol=1e-12)
    assert spec["motion"] == "line" and spec["opening_mm"] == 0.5 * F.MAX_OPENING_MM and move.ok


@pytest.mark.parametrize("world", [dict(xyz=(0.2, 0.0, 0.2)), dict(by=(0.0, 0.0, 0.01)), dict(pitch=90.0),
                                   dict(yaw=0.0), dict(roll=0.0)])
def test_hand_moves_take_no_world_target_or_angles(world):
    fake, host, *_ = _host()
    xyz = world.pop("xyz", None)
    for hand in (dict(hand_by=(0.01, 0.0, 0.0)), dict(hand_turn=(0.0, 0.0, 10.0))):
        text = _failed("ValueError", host, "goto", "left", xyz, **hand, **world)
        assert "without xyz, by, pitch, yaw or roll" in text
    assert not fake.drafts and host.step == 0


def test_hand_moves_refuse_what_is_not_three_numbers_and_long_turns():
    fake, host, *_ = _host()
    assert "(forward, left, up) in metres" in _failed("ValueError", host, "goto", "left", hand_by=(0.01, 0.0))
    assert "(forward, left, up) in metres" in _failed("ValueError", host, "goto", "left", hand_by=(0.01, math.nan, 0.0))
    assert "in degrees" in _failed("ValueError", host, "goto", "left", hand_turn="roll")
    assert "at most 170" in _failed("ValueError", host, "goto", "left", hand_turn=(0.0, 0.0, 171.0))
    assert "173.2 degrees" in _failed("ValueError", host, "goto", "left", hand_turn=(100.0, 100.0, 100.0))
    assert not fake.drafts and host.step == 0
    assert host.goto("left", hand_turn=np.array([0.0, 0.0, 170.0]), dry_run=True).reached


def test_hand_moves_dry_run_and_refusals_keep_the_asked_pose():
    fake, host, *_ = _host()
    host.goto("left", (0.27, -0.12, 0.12), pitch=45, yaw=30, roll=0)
    asked, aim = host.asked["left"], host.aim["left"]
    dry = host.goto("left", hand_turn=(0.0, 0.0, 45.0), dry_run=True)
    assert dry.reached and not dry.moved and dry.target_angles != asked
    assert host.asked["left"] == asked and host.aim["left"] == aim
    assert "NOT executed" in _failed("NotExecuted", host, "goto", "left", hand_by=(5.0, 0.0, 0.0))
    assert host.asked["left"] == asked and host.aim["left"] == aim
    # the next hand move still starts from what was asked
    host.goto("left", hand_by=(0.0, 0.0, 0.01))
    assert fake.drafts[-1]["arms"]["left"]["rpy_deg"] == aim


def test_proxy_passes_hand_moves_only_when_given():
    from hexashell.real.urai.proxy import Robot

    calls = []
    robot = Robot(lambda name, args, kwargs: calls.append((name, args, kwargs)))
    robot.goto("left", by=(0.0, 0.0, 0.01))
    robot.goto("right", hand_by=(0.01, 0.0, 0.0), hand_turn=(0.0, 0.0, 10.0), path="line")
    assert calls[0] == ("goto", ("left", None), dict(by=(0.0, 0.0, 0.01), pitch=None, yaw=None, roll=None, grip=None,
                                                     path="auto", dry_run=False, contact=False, gentle=False))
    assert calls[1][2]["hand_by"] == (0.01, 0.0, 0.0) and calls[1][2]["hand_turn"] == (0.0, 0.0, 10.0)
    assert calls[1][2]["path"] == "line" and "hand_by" in Robot.goto.__doc__ and "hand_turn" in Robot.goto.__doc__


def test_both_moves_the_two_arms_in_one_plan():
    fake, host, events, states, _ = _host(step_limit=500)
    both = host.both(left=dict(by=(0.0, 0.0, 0.05)), right=dict(xyz=(0.25, -0.3, 0.15), pitch=90, yaw=30, roll=0,
                                                                  path="line"))
    # each arm planned alone first, as goto's dry run; then the pair: one plan, one execution
    assert [sorted(d["arms"]) for d in fake.drafts] == [["left"], ["right"], ["left", "right"]]
    assert [r["body"]["planning_only"] for r in fake.requests if r["path"] == "/api/preview"] == [True, True, False]
    assert fake.drafts[-1]["arms"] == {"left": fake.drafts[0]["arms"]["left"], "right": fake.drafts[1]["arms"]["right"]}
    assert len(fake.executed) == 1 and len(states) == 1
    assert both.steps == host.step == _steps(fake.executed) and both.steps_left == 500 - host.step
    assert both.left.ok and np.allclose(both.left.at, np.add(F.START_XYZ["left"], (0.0, 0.0, 0.05)))
    assert both.right.ok and np.allclose(both.right.at, (0.25, -0.3, 0.15))
    # one clock: the shorter motion ends first and holds, its own steps are fewer
    assert both.left.steps == math.ceil(0.05 / 0.12 / STEP_S) < both.right.steps <= both.steps
    assert host.asked == {"left": (90.0, 0.0, 0.0), "right": (90.0, 30.0, 0.0)}
    assert str(both).startswith(f"Both: {both.steps} steps together  [{both.steps_left} steps left]\nMove left: at")


def test_both_takes_a_gripper_alone_and_hand_moves():
    fake, host, *_ = _host()
    both = host.both(left=dict(grip="close"), right=dict(hand_by=(0.02, 0.0, 0.0), grip=0.5))
    # a gripper alone is not planned first; the hand moves forward (down) from the measured TCP
    assert [sorted(d["arms"]) for d in fake.drafts] == [["right"], ["left", "right"]]
    assert fake.drafts[-1]["arms"]["left"] == {"primitive": "gripper", "opening_mm": 0.0, "effort": 1000,
                                               "duration_s": F.MAX_OPENING_MM / 100.0 + 0.3}
    assert fake.drafts[-1]["arms"]["right"]["opening_mm"] == 0.5 * F.MAX_OPENING_MM
    assert type(both.left).__name__ == "Grip" and both.left.goal == 0.0 and both.left.holding is False
    assert both.right.ok and np.allclose(both.right.at, np.add(F.START_XYZ["right"], (0.0, 0.0, -0.02)))
    assert host.grip_goal == {"left": 0.0, "right": 0.5} and fake.arms["left"]["gripper_mm"] == 0.0
    assert both.left.steps == math.ceil((F.MAX_OPENING_MM / 100.0 + 0.3) / STEP_S) and both.steps == host.step
    assert both.right.steps == math.ceil(0.02 / 0.12 / STEP_S)
    one = host.both(right=dict(by=(0.0, 0.0, 0.01)))
    assert one.left is None and one.right.ok and sorted(fake.drafts[-1]["arms"]) == ["right"]


def test_both_refusals_move_nothing():
    fake, host, *_ = _host()
    before, asked = json.dumps(fake.arms), dict(host.asked)
    for bad, words in [(dict(left=(0.2, 0.0, 0.2)), "is a dict of goto's arguments"),
                       (dict(left=dict(by=(0.0, 0.0, 0.01), dry_run=True)), "has no ['dry_run']"),
                       (dict(), "at least one arm"),
                       (dict(left=dict(grip="half")), "gripper opening"),
                       (dict(right=dict(hand_by=(0.01, 0.0, 0.0), by=(0.0, 0.0, 0.01))), "hand_by and hand_turn")]:
        assert words in _failed("ValueError", host, "both", **bad)
    assert not fake.drafts
    # an arm refused alone: its dry run says why
    text = _failed("NotExecuted", host, "both", left=dict(by=(0.0, 0.0, 0.01)), right=dict(xyz=(9.0, 9.0, 9.0)))
    assert text.startswith("Move right (dry run): NOT reachable") and "目标超出工作空间" in text
    assert text.endswith("\n" + texts.BOTH_NOT_EXECUTED)
    # the pair refused: URAI's reason (here the arms would meet)
    text = _failed("NotExecuted", host, "both", left=dict(xyz=(0.25, -0.1, 0.25)), right=dict(xyz=(0.25, -0.15, 0.25)))
    assert text.startswith("both(): left would hit the right arm") and text.endswith("\n" + texts.BOTH_NOT_EXECUTED)
    assert not fake.executed and host.step == 0 and json.dumps(fake.arms) == before and host.asked == asked


def test_both_chooses_each_arms_path_as_goto_does():
    fake, host, *_ = _host()
    fake.no_line, fake.bend = True, 0.02
    both = host.both(left=dict(by=(0.05, 0.0, 0.0)), right=dict(by=(0.05, 0.0, 0.0), path="swing", contact=True))
    assert [(sorted(d["arms"]), [spec["motion"] for spec in d["arms"].values()]) for d in fake.drafts] == [
        (["left"], ["line"]), (["left"], ["swing"]), (["right"], ["swing"]), (["left", "right"], ["swing", "swing"])]
    assert both.left.path == both.right.path == "swing"
    assert both.left.notes == ["left: 直线路径与桌面碰撞", texts.SWING_USED.format(off=20.0, dip=0.0)]
    assert both.right.notes == [texts.CONTACT]
    fake.bend = 0.05  # too far from the line for "auto"
    text = _failed("NotExecuted", host, "both", left=dict(by=(0.05, 0.0, 0.0)), right=dict(by=(0.0, 0.0, 0.01)))
    assert texts.SWING_REFUSED.format(off=50.0, dip=0.0) in text and len(fake.executed) == 1


def test_together_slows_the_arm_that_would_end_first():
    fake, host, *_ = _host()
    apart = host.both(left=dict(by=(0.0, 0.0, 0.05)), right=dict(by=(0.0, 0.0, 0.1)))
    assert apart.left.steps < apart.right.steps
    together = host.both(left=dict(by=(0.0, 0.0, -0.05)), right=dict(by=(0.0, 0.0, -0.1)), together=True)
    # planned once as asked, then with the left arm at the speed that ends it with the right (0.05 m in 0.1/0.12 s)
    assert [d["arms"]["left"]["speed_m_s"] for d in fake.drafts[-2:]] == [0.12, pytest.approx(0.06)]
    assert fake.drafts[-1]["arms"]["right"]["speed_m_s"] == 0.12 and len(fake.executed) == 2
    ends = [fake.executed[-1]["arms"][arm]["time_s"][-1] for arm in ("left", "right")]
    assert ends[0] == pytest.approx(ends[1]) and together.left.steps == together.right.steps
    assert together.left.ok and together.right.ok and together.steps == apart.steps
    # a gripper alone is not stretched
    host.both(left=dict(grip="open"), right=dict(by=(0.0, 0.0, 0.01)), together=True)
    assert len(fake.executed) == 3 and fake.drafts[-1]["arms"]["right"]["speed_m_s"] == 0.12
    # an arm that only turns, and ends first, has no speed to slow
    text = _failed("NotExecuted", host, "both", left=dict(hand_turn=(0.0, 0.0, 30.0)),
                   right=dict(by=(0.0, 0.0, 0.1)), together=True)
    assert text == texts.TOGETHER_REFUSED.format(arm="left", gap=0.1 / 0.12 - 0.12)
    assert len(fake.executed) == 3


def test_both_is_on_the_urai_robot_only():
    from hexashell.real.franka.proxy import Robot as Fr3Robot
    from hexashell.real.franka.session import FrankaHost
    from hexashell.real.urai.proxy import Robot

    calls = []
    robot = Robot(lambda name, args, kwargs: calls.append((name, args, kwargs)))
    robot.both(left=dict(by=(0.0, 0.0, 0.01)))
    robot.both(right=dict(grip="close"), together=True)
    assert calls == [("both", (), dict(left=dict(by=(0.0, 0.0, 0.01)), right=None, together=False)),
                     ("both", (), dict(left=None, right=dict(grip="close"), together=True))]
    assert "both" in repr(robot) and "together" in Robot.both.__doc__
    assert "both" in UraiHost.public and "both" not in FrankaHost.public
    assert set(UraiHost.public) - set(FrankaHost.public) == {"both"}
    fr3 = Fr3Robot(lambda name, args, kwargs: None)
    assert not hasattr(fr3, "both") and "both" not in repr(fr3) and hasattr(fr3, "goto")


def test_gentle_caps_a_fast_host():
    fake, host, *_ = _host(speed_m_s=0.8)
    host.goto("left", by=(0.0, 0.0, 0.01), gentle=True, dry_run=True)
    host.goto("left", by=(0.0, 0.0, 0.01), dry_run=True)
    assert [d["arms"]["left"]["speed_m_s"] for d in fake.drafts] == [0.5, 0.8]


def test_dry_run_and_refusals():
    fake, host, *_ = _host()
    before = json.dumps(fake.arms)
    dry = host.goto("left", by=(0.01, 0.0, 0.0), grip="open", path="line", dry_run=True)
    assert dry.reached and not dry.moved and dry.dry_run
    assert dry.steps == math.ceil(fake.preview["duration_s"] / STEP_S) == 3 and host.step == 0
    blocked = host.goto("left", (9.0, 9.0, 9.0), dry_run=True)
    assert not blocked.reached and not blocked.moved and blocked.steps == 0 and blocked.notes == ["left: 目标超出工作空间"]
    assert [r["body"]["planning_only"] for r in fake.requests if r["path"] == "/api/preview"] == [True, True, True]
    text = _failed("NotExecuted", host, "goto", "left", (9.0, 9.0, 9.0), path="line")
    assert "NOT executed" in text and "目标超出工作空间" in text
    assert not fake.executed and host.step == 0 and json.dumps(fake.arms) == before
    _failed("ValueError", host, "goto", "left", (0.2, 0.0, 0.2), path="zigzag")
    _failed("ValueError", host, "goto", "left", (0.2, 0.0))


def test_auto_takes_a_swing_only_near_the_line():
    fake, host, *_ = _host()
    fake.no_line, fake.bend = True, 0.02
    move = host.goto("left", by=(0.05, 0.0, 0.0))
    assert move.ok and move.path == "swing" and move.notes[0] == "left: 直线路径与桌面碰撞"
    assert move.notes[1] == texts.SWING_USED.format(off=20.0, dip=0.0)
    assert [d["arms"]["left"]["motion"] for d in fake.drafts] == ["line", "swing"]
    fake.bend = 0.05  # too far from the line
    text = _failed("NotExecuted", host, "goto", "left", by=(0.05, 0.0, 0.0))
    assert texts.SWING_REFUSED.format(off=50.0, dip=0.0) in text
    fake.bend = -0.015  # under its lower end
    text = _failed("NotExecuted", host, "goto", "left", by=(0.05, 0.0, 0.0))
    assert texts.SWING_REFUSED.format(off=15.0, dip=15.0) in text
    executed = len(fake.executed)
    assert host.goto("left", by=(0.05, 0.0, 0.0), path="swing").ok  # asked for: taken as URAI plans it
    assert len(fake.executed) == executed + 1
    _failed("NotExecuted", host, "goto", "left", by=(0.05, 0.0, 0.0), path="line")
    assert texts.CONTACT in host.goto("left", by=(0.0, 0.0, 0.01), path="swing", contact=True).notes


def test_gripper_holding_and_the_step_budget():
    fake, host, *_ = _host(step_limit=200)
    assert host.goto("left", (0.27, -0.12, 0.08), pitch=90, yaw=0, roll=0, grip="open").ok
    assert host.goto("left", (0.27, -0.12, 0.02), path="line").ok
    grip = host.gripper("left", "close")
    assert grip.holding is True and grip.width_m == pytest.approx(0.04) and grip.goal == 0.0
    assert fake.drafts[-1]["arms"]["left"] == {"primitive": "gripper", "opening_mm": 0.0, "effort": 1000,
                                               "duration_s": F.MAX_OPENING_MM / 100.0 + 0.3}
    assert host.state()["left"].holding is True
    assert host.goto("left", by=(0.0, 0.0, 0.1)).ok and fake.block[2] == pytest.approx(0.12)
    let_go = host.gripper("left", "open")
    assert let_go.holding is False and fake.held_by is None and fake.block[2] == pytest.approx(F.EDGE_M / 2)
    empty = host.gripper("left", 0.3)
    assert empty.holding is False and empty.opening == pytest.approx(0.3)
    host.arms["left"]["gripper_mm"], host.grip_goal["left"] = 68.0, 0.0  # a close the fingers did not follow
    assert host.holding("left") is None
    homing = host.home("left")
    assert homing.ok and homing.arms["left"] == (0.0, 0.0) and fake.arms["left"]["gripper_mm"] == 21.0
    assert fake.drafts[-1]["arms"] == {"left": {"primitive": "home", "joints_deg": host.start["left"]["joints_deg"],
                                                "speed_m_s": 0.12}}
    waited = host.wait(2)
    assert waited.steps == math.ceil((2 * STEP_S + fake.overhead_s) / STEP_S) == 3
    assert set(fake.drafts[-1]["arms"]) == {"left", "right"}
    for n in (0, 201, 1.5):
        _failed("ValueError", host, "wait", n)
    assert host.step == _steps(fake.executed)
    host.step_limit = host.step + 1
    host.dispatch("wait", (1,), {})  # within the budget when it starts; it ends past it
    with pytest.raises(Stop) as stop:
        host.dispatch("state", (), {})
    assert str(stop.value) == texts.BUDGET.format(step=host.step, limit=host.step_limit)


def test_failed_executions_and_unknown_calls():
    fake, host, *_ = _host()
    assert _failed("RobotError", host, "drive") == "robot has no 'drive'"
    fake.fail = "error"
    with pytest.raises(RuntimeError, match="关节 3 跟踪误差超限"):
        host.dispatch("wait", (1,), {})
    fake.fail = "empty_grasp"
    assert _failed("Dropped", host, "gripper", "left", "close") == "夹爪闭合后未检测到物体"
    fake.fail = "cancelled"
    assert "cancelled" in _failed("RobotError", host, "goto", "left", by=(0.0, 0.0, 0.02))
    assert host.step == _steps(fake.executed[:2]) + math.ceil((fake.executed[2]["duration_s"] + 0.004) / 2 / STEP_S)


def test_urai_has_no_segmentation_api():
    from hexashell.real.urai.proxy import Robot

    _, host, *_ = _host()
    robot = Robot(host.dispatch)
    for name in ("find", "measure", "things"):
        assert not hasattr(robot, name)
        assert name not in repr(robot)
        assert _failed("RobotError", host, name) == f"robot has no '{name}'"


def test_gripper_commands_take_effect_at_the_start_of_motion():
    """Check measured holding and the fake service's block, without an object-tracking scene."""
    fake, host, *_ = _host()
    try:
        initial = fake.block.copy()
        assert host.goto("left", (*F.BLOCK_XY, 0.08), pitch=90, yaw=0, roll=0, grip="open").ok
        assert host.goto("left", (*F.BLOCK_XY, 0.02), path="line").ok
        assert host.goto("left", by=(0.0, 0.0, 0.1), grip="close").ok
        assert fake.held_by == "left" and host.holding("left")
        assert np.allclose(fake.block, initial + (0.0, 0.0, 0.1))
        assert host.goto("left", (0.2, -0.02, 0.12)).ok
        release_xy = fake.block[:2].copy()
        assert host.goto("left", by=(-0.05, 0.0, 0.0), grip="open").ok
        assert fake.held_by is None and not host.holding("left")
        assert np.allclose(fake.block[:2], release_xy)
        assert fake.block[2] == pytest.approx(F.TABLE_Z + F.EDGE_M / 2)
    finally:
        host.close()


def test_look_and_its_outlines():
    scene = _scene()
    fake, host, _, _, pictures = _host()
    try:
        view = host.observations["head"].view
        rgb = view.image.astype(int)
        outline = (rgb[..., 0] > 2 * rgb[..., 1]) & (rgb[..., 0] > 2 * rgb[..., 2])
        mask = scene.register(view, outline, "red block", 1.0, {})
        thing = scene.measure([mask], cams=["head"])
        assert thing.found
        assert host._outlines(None, "head") == [] and host._outlines(mask, "left_wrist") == []
        assert [label for _, label in host._outlines([mask, (thing,), thing.masks], "head")] == [
            f"#{mask.id} red block", f"T{thing.id} red block", f"#{mask.id} red block"]
        picture = host.look("head", masks=[mask, (thing,), thing.masks])
        assert picture.cams == ["head"] and pictures[-1][0] == "look"
        assert pictures[-1][1].shape[:2] == (F.HEIGHT, F.WIDTH)
        _failed("ValueError", host, "look", "head", masks=["red block"])
        host.look("all", tile=True, note="全部")
        assert len(pictures) == 2 and pictures[-1][2] == "look: head, left_wrist, right_wrist — 全部 (side by side)"
        host.look(["head", "right_wrist"], grid=True)
        assert [c for _, _, c in pictures[2:]] == ["look: head (grid)", "look: right_wrist (grid)"]
        _failed("ValueError", host, "look", ["head", "left_wrist"], crop=(0, 0, 10, 10))
    finally:
        scene.close()
        host.close()


# -- episodes --------------------------------------------------------------------------------------
class _Gateway(fakes.ScriptedGateway):
    tools: list = []

    def ask(self, messages, tools, **kw):
        type(self).tools = tools
        return super().ask(messages, tools, **kw)


def _fake_client(monkeypatch, fake):
    from hexashell.real.urai import session

    monkeypatch.setattr(session, "UraiClient", lambda url, token=None, *, log: fake.client(log, token))
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        monkeypatch.delenv(key)


def test_scripted_episode(monkeypatch, tmp_path):
    from hexashell.real.urai.session import UraiSession
    from hexashell.runtime import card, episode

    fake = F.FakeUrai()
    _fake_client(monkeypatch, fake)
    _Gateway.script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nprint(robot.camera('head'))\n"
                           "robot.look('all', tile=True)")],
        [fakes.python_call("print(robot.goto('left', (0.27, -0.12, 0.08), pitch=90, yaw=0, roll=0, grip='open'))\n"
                           "print(robot.goto('left', (0.27, -0.12, 0.02), path='line'))\n"
                           "g = robot.gripper('left', 'close')\nprint(g)\nassert g.holding")],
        [fakes.python_call("print(robot.goto('left', by=(0, 0, 0.1)))\nprint(robot.goto('left', (0.2, -0.02, 0.12)))\n"
                           "print(robot.goto('left', by=(0, 0, -0.1)))\nprint(robot.gripper('left', 'open'))\n"
                           "print(robot.home('left'))")],
        [fakes.python_call("try:\n    robot.goto('left', (9, 9, 9))\n"
                           "except NotExecuted as e:\n    print('refused:', e)\n"
                           "robot.find('head', 'red block')\nprint('not reached')")],
        [fakes.finish_call("done", "The block was placed at (0.20, -0.02).")],
    ]
    _Gateway.seen = []
    monkeypatch.setattr(episode, "Gateway", _Gateway)
    session = UraiSession(F.URL, "Move the red block to (0.20, -0.02).", task_name="move_block",
                          token="s3cret", step_limit=400)
    settings = episode.Settings(max_turns=10, budget_s=300)
    ep = episode.Episode(session, settings, tmp_path)
    ep.run()
    folder = ep.rec.dir
    assert folder.name.startswith("urai-move_block-")
    result = json.loads((folder / "result.json").read_text())
    assert next(iter(result)) == "bench" and result["bench"] == "urai"
    assert result["end"] == "finish" and result["errors"] == [] and result["cells"] == 4, result
    assert result["finish"] == {"status": "done", "summary": "The block was placed at (0.20, -0.02)."}
    assert result["success"] is None and result["agent_report"] == result["finish"]
    assert result["evaluation"] == {"source": "none"} and result["progress"] is None
    assert result["step"] == _steps(fake.executed) > 0 and result["step_limit"] == 400
    assert result["robot_calls"]["goto"] == 6 and result["robot_calls"].get("find", 0) == 0, result["robot_calls"]
    assert np.allclose(fake.block, (0.2, -0.02, F.EDGE_M / 2)) and fake.held_by is None
    assert ep.deadline - ep.t_agent == pytest.approx(settings.budget_s)  # one budget: the robot does not wait
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert all(c["error"] is None for c in cells[:3]), cells
    assert "Grip left: closed on something" in cells[1]["output"] and "TCP" in cells[0]["output"]
    assert "refused: Move left: NOT executed" in cells[3]["output"] and "not reached" not in cells[3]["output"]
    assert "AttributeError" in cells[3]["error"] and "find" in cells[3]["error"]
    # what the model was told: the English prompt, card, tools and answers
    messages = _Gateway.seen[-1]
    assert messages[0]["content"] == card.system_prompt("urai")
    assert messages[0]["content"].startswith("# 1. Your job and the rules")
    first = messages[1]["content"]
    text = first if isinstance(first, str) else " ".join(p.get("text", "") for p in first if isinstance(p, dict))
    assert text.startswith("# Task card\n\nMove the red block to (0.20, -0.02).") and "fake-piper" in text
    assert f"You have up to 10 replies and 5 minutes for this episode; {texts.TIMING}\n" in text
    assert "[picture 1: head at the start]" in text
    finish = next(t["function"] for t in _Gateway.tools if t["function"]["name"] == "finish")
    assert finish["description"] == texts.FINISH and finish["parameters"]["properties"]["status"]["enum"] == [
        "done", "no_budget", "stuck", "keep_partial"]
    assert episode.TOOLS[1]["function"]["parameters"]["properties"]["status"]["enum"][0] != "done"  # untouched
    transcript = (folder / "transcript.json").read_text()
    assert json.dumps(texts.FINISHED, ensure_ascii=False)[1:-1] in transcript
    assert (folder / "prompt.md").read_text().startswith(messages[0]["content"])
    assert {r["token"] for r in fake.requests} == {"Bearer s3cret"}
    states = [json.loads(line) for line in (folder / "states.jsonl").read_text().splitlines()]
    assert states[0]["step"] == 0 and states[-1]["step"] == result["step"]
    assert [s["execution"]["id"] for s in states if "execution" in s] == [p["id"] for p in fake.executed]
    assert len(list((folder / "pictures").iterdir())) == 4  # three start pictures and the tile
    assert len(list((folder / "frames").iterdir())) >= 3


def test_session_ignores_sam3_settings(monkeypatch, tmp_path):
    from hexashell.real.urai.session import UraiSession
    from hexashell.runtime import episode
    from hexashell.vision.sam3_client import Sam3Client

    fake = F.FakeUrai()
    _fake_client(monkeypatch, fake)
    connections = []

    def no_sam(self, *args, **kwargs):
        connections.append((args, kwargs))
        raise AssertionError("URAI must not create a SAM3 client")

    monkeypatch.setattr(Sam3Client, "__init__", no_sam)
    _Gateway.script = [[fakes.python_call(
        "robot.look('head', grid=True)\n"
        "assert robot.image('head').ndim == 3\n"
        "xy = robot.to_pixel((.27, -.12, .04), 'head')\n"
        "point = robot.to_world('head', xy)\n"
        "assert np.linalg.norm(np.array(point.xyz) - (.27, -.12, .04)) < .003\n"
        "assert robot.wait(1).steps >= 1")]]
    _Gateway.seen = []
    monkeypatch.setattr(episode, "Gateway", _Gateway)
    session = UraiSession(F.URL, "Inspect the block and hold for one step.")
    ep = episode.Episode(session, episode.Settings(sam3_url="http://sam3.invalid", max_turns=3), tmp_path)
    ep.run()
    assert connections == []
    result = json.loads((ep.rec.dir / "result.json").read_text())
    assert result["end"] == "finish" and result["errors"] == [] and result["step"] >= 1, result
    cells = [json.loads(line) for line in (ep.rec.dir / "cells.jsonl").read_text().splitlines()]
    assert all(cell["error"] is None for cell in cells), cells
    events = [json.loads(line) for line in (ep.rec.dir / "events.jsonl").read_text().splitlines()]
    assert not any(e["kind"] in ("sam3", "perception_unavailable") for e in events)
    assert next(e for e in events if e["kind"] == "start")["perception"] is False


def test_command_line(monkeypatch, tmp_path, capsys):
    from hexashell.real.urai import session

    fake = F.FakeUrai()
    _fake_client(monkeypatch, fake)
    monkeypatch.setenv("URAI_TOKEN", "t0k")
    monkeypatch.setenv("SAM3_URL", "http://sam3-environment.invalid")
    monkeypatch.setenv("HEXASHELL_SAM3_URL", "http://sam3-shared-environment.invalid")
    from hexashell.vision.sam3_client import Sam3Client

    connections = []

    def no_sam(self, *args, **kwargs):
        connections.append((args, kwargs))
        raise AssertionError("URAI must not create a SAM3 client")

    monkeypatch.setattr(Sam3Client, "__init__", no_sam)
    script = tmp_path / "cells.py"
    script.write_text("# %% Read the interface and state\nhelp(robot)\nhelp(robot.goto)\nprint(robot.state())\n"
                      "# %% Hold for one step\nassert robot.wait(1).steps >= 1\n")
    config = tmp_path / "settings.json"
    config.write_text(json.dumps({"budget_s": 60, "max_turns": 5, "sam3_url": "http://sam3-config.invalid"}))
    code = session.main(["--url", F.URL, "--task", "Read the measured state and hold for one step.",
                         "--name", "read", "--script", str(script),
                         "--config", str(config), "--results", str(tmp_path / "runs"), "--step-limit", "50",
                         "--speed", "0.2"])
    assert code == 0 and connections == []
    folder = tmp_path / "runs" / capsys.readouterr().out.strip().rsplit("/", 1)[-1]
    result = json.loads((folder / "result.json").read_text())
    assert result["end"] == "finish" and result["errors"] == [] and result["cells"] == 2, result
    assert result["finish"]["status"] == "keep_partial" and result["agent_report"] == result["finish"]
    assert result["settings"]["budget_s"] == 60 and result["settings"]["script"] == str(script)
    assert result["step"] == 2 and result["step_limit"] == 50 and folder.name.startswith("urai-read-")
    assert {r["token"] for r in fake.requests} == {"Bearer t0k"}
    # Inspect real records, including help text emitted inside the worker, not just prompt files.
    for name in ("prompt.md", "transcript.json", "cells.jsonl", "result.json"):
        recorded = (folder / name).read_text()
        if name.endswith(".json"):
            recorded = json.dumps(json.loads(recorded), ensure_ascii=False)
        elif name.endswith(".jsonl"):
            recorded = json.dumps([json.loads(line) for line in recorded.splitlines()], ensure_ascii=False)
        assert not re.search(r"[\u3400-\u9fff]", recorded), name
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert "Move the selected arm's URAI TCP" in cells[0]["output"]
    assert "Write all model-generated text in English" in (folder / "prompt.md").read_text()
    monkeypatch.delenv("URAI_URL", raising=False)
    with pytest.raises(SystemExit) as stopped:
        session.main(["--task", "Read the state.", "--results", str(tmp_path / "none")])
    assert stopped.value.code == 2 and "--url" in capsys.readouterr().err
    with pytest.raises(SystemExit) as help_exit:
        session.main(["--help"])
    assert help_exit.value.code == 0 and "--sam3-url" not in capsys.readouterr().out
    assert not (tmp_path / "none").exists()
