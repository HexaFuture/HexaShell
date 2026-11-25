"""hexashell/real/franka: the FR3's kinematics against Franka's numbers, the planner's paths, the service over real
HTTP with the kinematic simulator, whole episodes through the real client, host and sandboxed cells, and the driver's
use of the real franky package (installed for the tests; nothing connects to a robot)."""

from __future__ import annotations

import base64
import io
import json
import math
import re
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import httpx
import numpy as np
import pytest

from hexashell.geometry import angle_between
from hexashell.real.franka import planner, session
from hexashell.real.franka.arm import READY_Q, Fr3, segment_distance
from hexashell.real.franka.cameras import depth_metres
from hexashell.real.franka.driver import RobotFault, fault_types, grip_command, joint_waypoint_motion
from hexashell.real.franka.rig import load_rig, rig_from_dict
from hexashell.real.franka.server import serve
from hexashell.real.franka.service import FrankaService
from hexashell.real.franka.simulated import BLOCK_EDGE_M, SimulatedCamera, SimulatedDriver, simulated_rig
from hexashell.real.urai.client import UraiClient
from hexashell.real.urai.frames import rotation_rpy, to_rpy


def random_clear_q(arm: Fr3, rng, count: int = 40) -> list[np.ndarray]:
    out = []
    while len(out) < count:
        q = rng.uniform(arm.low, arm.high)
        if not arm.problems(q):
            out.append(q)
    return out


# -- kinematics --------------------------------------------------------------------------------------------------
def test_joint_limits_are_frankas():
    arm = Fr3()
    assert np.allclose(arm.low, [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
    assert np.allclose(arm.high, [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])


def test_ready_pose_is_frankas():
    """The ready pose under ``assets/fr3.urdf`` with the hand's TCP, pointing straight down."""
    arm = Fr3()
    tcp = arm.tcp(READY_Q)
    assert np.allclose(tcp[:3, 3], (0.3069, 0.0, 0.4869), atol=5e-4), tcp[:3, 3]
    assert np.allclose(tcp[:3, 2], (0, 0, -1), atol=1e-6)  # approach
    assert np.allclose(np.abs(tcp[:3, 0]), (0, 1, 0), atol=1e-6)  # closing axis, the hand's y
    assert arm.problems(READY_Q) == []


def test_the_kinematics_match_a_real_fr3():
    """O_T_EE read from an FR3 whose Desk control frame is z 0.1034, yaw -45 degrees."""
    tcp = Fr3().tcp((-0.0039, -0.0913, -0.1312, -2.04, 0.0592, 1.9723, 0.6591))
    assert np.allclose(tcp[:3, 3], (0.5301, -0.0629, 0.3819), atol=5e-4), tcp[:3, 3]


@pytest.mark.parametrize("tool, tip, clear_m, refused", [
    (0.195, 0.021, 0.015, True),  # the long fingers' tips would be under the table
    (0.195, 0.021, 0.030, False),
    (0.1034, 0.009, 0.010, True),  # the stock fingers: the capsule's radius still decides
    (0.1034, 0.009, 0.020, False),
])
def test_the_table_check_covers_the_finger_tips(tool, tip, clear_m, refused):
    arm = Fr3(tool_length_m=tool, tip_past_tcp_m=tip)
    tcp_z = float(arm.tcp(READY_Q)[2, 3])
    table = Fr3(tool_length_m=tool, tip_past_tcp_m=tip, table_z=tcp_z - clear_m)
    assert ("the fingers below the table top" in table.problems(READY_Q)) is refused


@pytest.mark.parametrize("key", ["tool_length_m", "tip_past_tcp_m"])
def test_a_rig_must_state_its_fingers(key):
    data = _rig_data()
    data.pop(key)
    with pytest.raises(TypeError, match=key):
        rig_from_dict(data)


def test_a_rig_with_long_fingers_builds_that_arm():
    data = _rig_data()
    data.update(tool_length_m=0.195, tip_past_tcp_m=0.021)
    arm = rig_from_dict(data).arm()
    assert arm.tool_length_m == 0.195 and arm.tip_past_tcp_m == 0.021


def test_base_pose_and_tool_length_move_the_tcp():
    shifted = np.eye(4)
    shifted[:3, 3] = (0.1, -0.2, 0.05)
    assert np.allclose(Fr3(shifted).tcp(READY_Q)[:3, 3], np.array((0.3069 + 0.1, -0.2, 0.4869 + 0.05)), atol=5e-4)
    assert Fr3(tool_length_m=0.1534).tcp(READY_Q)[2, 3] == pytest.approx(0.4869 - 0.05, abs=5e-4)


def test_jacobian_matches_finite_differences():
    arm = Fr3()
    q = np.array([0.3, -0.5, 0.2, -2.0, 0.4, 1.8, 0.6])
    jac, pose = arm.jacobian(q)
    for i in range(7):
        step = np.zeros(7)
        step[i] = 1e-6
        ahead = arm.tcp(q + step)
        assert np.allclose(jac[:3, i], (ahead[:3, 3] - pose[:3, 3]) / 1e-6, atol=1e-4)
        assert np.allclose(jac[3:, i], planner.pose_error(ahead, pose)[3:] / 1e-6, atol=1e-4)


def test_segment_distance_on_known_shapes():
    a = np.array
    assert segment_distance(a([0., 0, 0]), a([1., 0, 0]), a([0., 1, 0]), a([1., 1, 0])) == pytest.approx(1.0)
    assert segment_distance(a([0., 0, 0]), a([1., 0, 0]), a([.5, -1, 1]), a([.5, 1, 1])) == pytest.approx(1.0)
    assert segment_distance(a([0., 0, 0]), a([1., 0, 0]), a([2., 0, 0]), a([3., 0, 0])) == pytest.approx(1.0)
    assert segment_distance(a([0., 0, 0]), a([0., 0, 0]), a([0., 3, 4]), a([0., 3, 4])) == pytest.approx(5.0)


def test_inverse_kinematics_reaches_poses_the_arm_can_take():
    arm = Fr3()
    rng = np.random.default_rng(3)
    start = np.array(READY_Q)
    for q in random_clear_q(arm, rng):
        target = arm.tcp(q)
        solution = planner.solve(arm, target, start)
        assert solution.reached, (q, solution.pos_err, solution.rot_err)
        reached = arm.tcp(solution.q)
        assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) <= planner.POS_TOL_M
        assert angle_between(reached[:3, :3], target[:3, :3]) <= planner.ROT_TOL_RAD
        assert arm.within_limits(solution.q)


def test_pitch_and_yaw_mean_what_hexashell_says_on_the_real_kinematics():
    """Through the same conversion the host uses: pitch 90 down, yaw 0/90 closes along world x/y."""
    arm = Fr3()
    for yaw, closing in ((0.0, (1, 0, 0)), (90.0, (0, 1, 0))):
        target = np.eye(4)
        target[:3, :3] = rotation_rpy(to_rpy(90.0, yaw, 0.0))
        target[:3, 3] = (0.45, 0.0, 0.2)
        q = planner.goal_of(arm, target, READY_Q)
        pose = arm.tcp(q)
        assert np.allclose(pose[:3, 2], (0, 0, -1), atol=2e-2), pose[:3, 2]
        assert np.allclose(np.abs(pose[:3, 0]), closing, atol=2e-2), pose[:3, 0]


# -- plans -------------------------------------------------------------------------------------------------------
def _down_at(x, y, z):
    pose = np.eye(4)
    pose[:3, :3] = rotation_rpy(to_rpy(90.0, 0.0, 0.0))
    pose[:3, 3] = (x, y, z)
    return pose


def test_a_line_is_straight_and_keeps_to_its_speeds():
    arm = Fr3()
    start = planner.goal_of(arm, _down_at(0.4, -0.15, 0.25), READY_Q)
    target = _down_at(0.5, 0.15, 0.12)
    path = planner.plan_line(arm, start, target, 0.12)
    assert np.allclose(arm.tcp(path.q[-1])[:3, 3], target[:3, 3], atol=planner.POS_TOL_M)
    points = np.array([arm.tcp(q)[:3, 3] for q in path.q])
    line = points[-1] - points[0]
    off = points - points[0] - np.outer((points - points[0]) @ line / (line @ line), line)
    assert np.linalg.norm(off, axis=1).max() < 2e-3
    legs = np.diff(path.t)
    assert np.all(np.linalg.norm(np.diff(points, axis=0), axis=1) / legs <= 0.12 + 1e-6)
    assert np.all(np.abs(np.diff(path.q, axis=0)) / legs[:, None] <= planner.JOINT_SPEED_RAD_S + 1e-6)
    assert np.all(path.dq[0] == 0) and np.all(path.dq[-1] == 0)
    assert legs[0] > legs[len(legs) // 2]  # it speeds up from the start


@pytest.mark.parametrize("pose, reason", [
    (_down_at(2.0, 0.0, 0.3), "out of reach"),
    (_down_at(0.45, 0.0, 0.005), "fingers below the table"),
])
def test_impossible_poses_are_refused_with_the_reason(pose, reason):
    arm = Fr3()
    with pytest.raises(planner.Refusal, match=reason):
        planner.goal_of(arm, pose, READY_Q)


def test_a_line_through_the_table_is_refused_on_the_way():
    arm = Fr3(table_z=0.15)
    start = planner.goal_of(arm, _down_at(0.45, 0.0, 0.3), READY_Q)
    with pytest.raises(planner.Refusal, match="table"):
        planner.plan_line(arm, start, _down_at(0.45, 0.0, 0.1), 0.1)


def test_the_arm_may_not_run_through_itself():
    arm = Fr3()
    rng = np.random.default_rng(1)
    against = None
    for _ in range(5000):
        q = rng.uniform(arm.low, arm.high)
        if any(" against " in p for p in arm.problems(q)):
            against = q
            break
    assert against is not None
    with pytest.raises(planner.Refusal, match="swing would put"):
        planner.plan_joints(arm, READY_Q, against, 0.1)


def test_a_path_of_no_length_is_still_a_path():
    arm = Fr3()
    path = planner.plan_line(arm, READY_Q, arm.tcp(READY_Q), 0.1)
    assert path.duration_s > 0 and np.allclose(path.q[-1], READY_Q, atol=1e-2)


# -- the service over HTTP ---------------------------------------------------------------------------------------
@contextmanager
def served(*, allow_motion=True, time_scale=0.02, token=None, block_xy=(0.45, 0.12)):
    rig = simulated_rig()
    driver = SimulatedDriver(rig.arm(), time_scale=time_scale, block_xy=block_xy)
    cameras = {name: SimulatedCamera(mount, driver) for name, mount in rig.cameras.items()}
    service = FrankaService(rig, driver, cameras, allow_motion=allow_motion)
    server = serve(service, "127.0.0.1", 0, token)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", service, driver
    finally:
        server.shutdown()
        server.server_close()
        service.wait_idle()


def _array(packet):
    return np.load(io.BytesIO(base64.b64decode(packet["data"])), allow_pickle=False)


def _goto(*, z=0.3, motion="swing", speed=0.2):
    return {"primitive": "goto", "xyz": [0.45, 0.0, z], "rpy_deg": to_rpy(90.0, 0.0, 0.0), "motion": motion,
            "speed_m_s": speed}


def _client(url):
    return UraiClient(url, log=lambda event: None)


def test_model_state_and_pictures():
    with served() as (url, service, driver):
        client = _client(url)
        model = client.model()
        assert model["name"] == "Simulated Franka FR3" and model["arm_names"] == ["arm"]
        assert model["arms"]["arm"]["tcp_offset_mm"] == pytest.approx(103.4)
        assert model["arms"]["arm"]["max_opening_mm"] == pytest.approx(80.0)
        state = client.state()
        assert state["backend"]["mode"] == "sim" and state["backend"]["family"] == "franka_fr3"
        arm = state["arms"]["arm"]
        assert np.allclose(arm["xyz"], (0.3069, 0.0, 0.4869), atol=5e-4)
        assert np.allclose(arm["joints_deg"], np.degrees(READY_Q))
        assert arm["enabled"] is True and arm["gripper_mm"] == pytest.approx(80.0)
        packet = client.observe("external")
        rgb, depth = _array(packet["rgb"]), _array(packet["depth"])
        assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8 and depth.shape == (480, 640)
        assert np.nanmedian(depth) == pytest.approx(1.1, abs=0.01)  # the camera is 1.1 m over the table
        k, pose = np.array(packet["k"]), np.array(packet["t_world_camera"])
        # the block's top centre shows at its projection, nearer than the table by its height
        top = np.array([0.45, 0.12, BLOCK_EDGE_M])
        cam = pose[:3, :3].T @ (top - pose[:3, 3])
        u, v = (k @ (cam / cam[2]))[:2].round().astype(int)
        assert rgb[v, u, 0] > 150 and rgb[v, u, 1] < 80
        assert depth[v, u] == pytest.approx(1.1 - BLOCK_EDGE_M, abs=1e-3)
        assert {c["name"] for c in packet["capsules"]["arm"]} >= {"forearm", "hand", "fingers"}
        wrist = client.observe("wrist")
        assert wrist["id"] != packet["id"]
        # a wrist picture is not the plan's picture
        assert service.observation_id == packet["id"]
        moved = np.array(wrist["t_world_camera"])
        assert not np.allclose(moved, pose)
        client.close()


def test_motion_goes_draft_preview_execute_and_ends_where_it_was_sent():
    with served() as (url, service, driver):
        client = _client(url)
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()})
        assert plan["duration_s"] > 0 and plan["pose_tolerance"]["position_mm"] == 5.0
        assert len(plan["arms"]["arm"]["xyz"]) == planner.SWING_SAMPLES + 1
        status = client.execute(plan)
        assert status["execution"]["state"] == "completed" and status["execution"]["execution_time_s"] > 0
        assert np.allclose(status["arms"]["arm"]["xyz"], (0.45, 0.0, 0.3), atol=2e-3)
        client.close()


def test_a_plan_runs_once_and_only_from_where_it_started():
    with served() as (url, service, driver):
        client = _client(url)
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()})
        client.execute(plan)
        with pytest.raises(httpx.HTTPStatusError) as twice:
            client.request("POST", "/api/execute", {"preview_id": plan["id"]})
        assert twice.value.response.status_code == 409 and "not the latest" in twice.value.response.json()["detail"]

        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto(z=0.25)})
        driver.q = driver.q + 0.1  # someone moved the arm between the plan and the execution
        with pytest.raises(httpx.HTTPStatusError) as moved:
            client.request("POST", "/api/execute", {"preview_id": plan["id"]})
        assert moved.value.response.status_code == 409 and "has moved" in moved.value.response.json()["detail"]
        client.close()


def test_a_draft_must_name_the_latest_picture_and_a_preview_the_latest_draft():
    with served() as (url, service, driver):
        client = _client(url)
        old = client.observe("external")["id"]
        client.observe("external")
        with pytest.raises(httpx.HTTPStatusError) as stale:
            client.plan(old, {"arm": _goto()})
        assert stale.value.response.status_code == 409 and "old picture" in stale.value.response.json()["detail"]
        latest = client.observe("external")["id"]
        draft = client.request("PUT", "/api/draft",
                               {"observation_id": latest, "completion": "hold", "arms": {"arm": _goto()}})
        with pytest.raises(httpx.HTTPStatusError) as behind:
            client.request("POST", "/api/preview", {"expected_revision": draft["revision"] - 1})
        assert behind.value.response.status_code == 409
        with pytest.raises(httpx.HTTPStatusError) as unknown:
            client.request("PUT", "/api/draft", {"observation_id": latest, "arms": {"left": _goto()}})
        assert unknown.value.response.status_code == 400
        client.close()


def test_a_refused_plan_answers_409_with_the_reason():
    with served() as (url, service, driver):
        client = _client(url)
        observation = client.observe("external")["id"]
        for spec in (dict(_goto(), xyz=[2.0, 0.0, 0.3]), dict(_goto(), xyz=[0.45, 0.0, 0.005]),
                     {"primitive": "gripper", "opening_mm": 200.0}):
            with pytest.raises(httpx.HTTPStatusError) as refused:
                client.plan(observation, {"arm": spec})
            assert refused.value.response.status_code == 409 and refused.value.response.json()["detail"]
        client.close()


def test_a_read_only_service_plans_and_never_moves():
    with served(allow_motion=False) as (url, service, driver):
        client = _client(url)
        start = driver.joints()
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()}, planning_only=True)
        assert plan["planning_only"] is True and plan["duration_s"] > 0
        with pytest.raises(httpx.HTTPStatusError) as acting:
            client.plan(observation, {"arm": _goto()})
        assert acting.value.response.status_code == 409 and "read-only" in acting.value.response.json()["detail"]
        for path, body in (("/api/execute", {"preview_id": plan["id"]}), ("/api/recover", {})):
            with pytest.raises(httpx.HTTPStatusError) as refused:
                client.request("POST", path, body)
            assert refused.value.response.status_code == 403
        time.sleep(0.05)
        assert np.array_equal(driver.joints(), start)
        client.close()


def test_a_plan_made_only_to_look_cannot_be_executed():
    with served() as (url, service, driver):
        client = _client(url)
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()}, planning_only=True)
        with pytest.raises(httpx.HTTPStatusError) as refused:
            client.request("POST", "/api/execute", {"preview_id": plan["id"]})
        assert refused.value.response.status_code == 409
        client.close()


def test_cancel_stops_a_motion_part_way():
    with served(time_scale=1.0) as (url, service, driver):
        client = _client(url)
        goal = np.array([0.45, 0.0, 0.3])
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto(speed=0.05)})
        client.request("POST", "/api/execute", {"preview_id": plan["id"]})
        time.sleep(0.2)
        assert client.state()["execution"]["state"] == "running"
        answer = client.request("POST", "/api/cancel", {})
        assert answer["state"] == "cancelling"
        service.wait_idle(5)
        state = client.state()
        assert state["execution"]["state"] == "cancelled"
        assert np.linalg.norm(np.array(state["arms"]["arm"]["xyz"]) - goal) > 0.02
        client.close()


def test_a_fault_ends_the_motion_and_only_an_operator_clears_it():
    with served() as (url, service, driver):
        client = _client(url)
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()})
        driver.fail_at = 0.5
        status = client.execute(plan)
        assert status["execution"]["state"] == "error" and "cartesian_reflex" in status["execution"]["error"]
        assert status["arms"]["arm"]["enabled"] is False
        assert status["backend"]["health"]["robot_mode"] == "Reflex"
        observation = client.observe("external")["id"]
        plan = client.plan(observation, {"arm": _goto()})
        with pytest.raises(httpx.HTTPStatusError) as blocked:
            client.request("POST", "/api/execute", {"preview_id": plan["id"]})
        assert blocked.value.response.status_code == 409 and "recover" in blocked.value.response.json()["detail"]
        assert driver.fault == "cartesian_reflex"  # nothing cleared it on the way
        answer = client.request("POST", "/api/recover", {})
        assert answer["recovered"] is True and answer["previous_execution"]["state"] == "error"
        assert client.state()["arms"]["arm"]["enabled"] is True
        client.close()


def test_the_token_and_the_address_rule():
    with served(token="s3cret") as (url, service, driver):
        assert httpx.get(f"{url}/api/state", trust_env=False).status_code == 401
        wrong = httpx.get(f"{url}/api/state", headers={"Authorization": "Bearer no"}, trust_env=False)
        assert wrong.status_code == 401
        client = UraiClient(url, "s3cret", log=lambda event: None)
        assert client.state()["backend"]["family"] == "franka_fr3"
        client.close()
        with pytest.raises(ValueError, match="needs a token"):
            serve(service, "0.0.0.0", 0, None)


def test_pictures_for_the_workbench_and_its_state_fields():
    """What a monitor reads: JPEGs by camera, /api/live.jpg, and the state's backend and arm fields."""
    import cv2

    with served() as (url, service, driver):
        for path in ("/api/cameras/external/image.jpg", "/api/cameras/wrist/image.jpg", "/api/live.jpg"):
            answer = httpx.get(url + path, trust_env=False)
            assert answer.status_code == 200 and answer.headers["content-type"] == "image/jpeg"
            image = cv2.imdecode(np.frombuffer(answer.content, np.uint8), cv2.IMREAD_COLOR)
            assert image.shape == (480, 640, 3)
        assert httpx.get(f"{url}/api/cameras/nope/image.jpg", trust_env=False).status_code == 400
        state = httpx.get(f"{url}/api/state", trust_env=False).json()
        assert state["backend"]["joint_names"][0] == "fr3_joint1" and len(state["backend"]["joint_names"]) == 7
        assert state["arms"]["arm"]["feedback_age_s"] == 0.0 and state["observation_stale"] is True
        assert httpx.get(f"{url}/api/nope", trust_env=False).status_code == 404


# -- episodes ----------------------------------------------------------------------------------------------------
PICK = """# %% See where things are
help(robot.goto)
print(robot.task)
print(robot.state())
print(robot.camera())
robot.look('all', tile=True)
# %% Find the red block in the fixed camera, and go over it
image = robot.image('external')
red = (image[..., 0] > 150) & (image[..., 1] < 80)
rows, cols = np.nonzero(red)
pixel = (float(cols.mean()), float(rows.mean()))
spot = robot.to_world('external', pixel)
print(spot)
x, y, top = spot.xyz
print(robot.goto((x, y, 0.15), pitch=90, yaw=0, roll=0, grip='open'))
print(robot.to_pixel((x, y, top), 'external'))
print(robot.goto((x, y, 0.15), dry_run=True))
# %% Grasp it
print(robot.goto((x, y, 0.02), path='line', gentle=True))
grip = robot.gripper('close')
print(grip)
assert grip.holding, grip
print(robot.goto(by=(0, 0, 0.1), path='line'))
print('lifted', robot.state())
# %% Refusals do not stop the session
try:
    robot.goto((3, 0, 0.3))
except NotExecuted as e:
    print('refused:', e)
print(robot.goto((3, 0, 0.3), dry_run=True))
# %% Put it down elsewhere and go home
print(robot.goto((0.45, -0.1, 0.1)))
print(robot.gripper('open'))
print(robot.wait(3))
print(robot.home())
print(robot.state())
"""


def test_a_whole_episode_picks_the_block_up_and_puts_it_down(tmp_path, capsys):
    script = tmp_path / "cells.py"
    script.write_text(PICK)
    with served() as (url, service, driver):
        code = session.main(["--url", url, "--task", "Move the red block to (0.45, -0.1).", "--name", "move_block",
                             "--script", str(script), "--results", str(tmp_path), "--step-limit", "4000"])
        assert code == 0
        folder = Path(capsys.readouterr().out.strip().splitlines()[-1])
        assert folder.name.startswith("franka-move_block-")
        result = json.loads((folder / "result.json").read_text())
        assert next(iter(result)) == "bench" and result["bench"] == "franka"
        assert result["errors"] == [] and result["success"] is None and result["evaluation"] == {"source": "none"}
        assert result["step"] > 0 and result["robot_calls"]["goto"] >= 6
        cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
        assert [c["error"] for c in cells] == [None] * 5, cells
        output = "\n".join(c["output"] for c in cells)
        assert "Grip arm: closed on something" in output and "refused: Move arm: NOT executed" in output
        assert "out of reach" in output and "TCP" in output
        assert "Move the TCP to" in cells[0]["output"]  # help(robot.goto) shows the FR3's words
        # the block really went where the cells sent it, and the arm came back
        assert np.allclose(driver.block[:2], (0.45, -0.1), atol=0.01)
        assert driver.block[2] == pytest.approx(BLOCK_EDGE_M / 2) and driver.held_offset is None
        assert np.allclose(driver.joints(), READY_Q, atol=1e-6)
        prompt = (folder / "prompt.md").read_text()
        assert prompt.startswith("# 1. Your job and the rules") and "Franka Research 3" in prompt
        assert "PiPER" not in prompt and "URAI" not in prompt
        assert "Simulated Franka FR3" in prompt
        assert not re.search(r"[一-鿿]", prompt + output)
        states = [json.loads(line) for line in (folder / "states.jsonl").read_text().splitlines()]
        assert states[0]["step"] == 0 and states[-1]["step"] == result["step"]
        assert len(list((folder / "pictures").iterdir())) >= 3


def test_an_episode_on_a_read_only_service_stops_at_the_refusal(tmp_path, capsys):
    script = tmp_path / "cells.py"
    script.write_text("# %% Plan, then try to move\n"
                      "print(robot.goto((0.45, 0.0, 0.3), pitch=90, yaw=0, roll=0, path='swing', dry_run=True))\n"
                      "try:\n    robot.goto((0.45, 0.0, 0.3), path='swing')\n"
                      "except NotExecuted as e:\n    print('refused:', e)\n")
    with served(allow_motion=False) as (url, service, driver):
        start = driver.joints()
        assert session.main(["--url", url, "--task", "Stay.", "--script", str(script),
                             "--results", str(tmp_path)]) == 0
        folder = Path(capsys.readouterr().out.strip().splitlines()[-1])
        cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
        assert cells[0]["error"] is None and "read-only" in cells[0]["output"]
        assert np.array_equal(driver.joints(), start)


def test_the_command_needs_the_address(monkeypatch, tmp_path):
    monkeypatch.delenv("FRANKA_URL", raising=False)
    with pytest.raises(SystemExit):
        session.main(["--task", "Read the state.", "--results", str(tmp_path)])


def test_the_policy_side_needs_neither_franky_nor_realsense():
    code = ("import sys, hexashell.real.franka.session, hexashell.real.franka.proxy, hexashell.real.franka.server\n"
            "assert 'franky' not in sys.modules and 'pyrealsense2' not in sys.modules\n")
    subprocess.run([sys.executable, "-c", code], check=True)


# -- rigs --------------------------------------------------------------------------------------------------------
def _rig_data():
    fixed, on_hand = np.eye(4).tolist(), np.eye(4).tolist()
    return {"name": "cell", "robot_ip": "192.0.2.1", "table_z": 0.01, "tool_length_m": 0.1034, "tip_past_tcp_m": 0.009,
            "plan_camera": "external",
            "t_world_base": np.eye(4).tolist(),
            "cameras": {"external": {"serial": "1", "t_world_camera": fixed},
                        "wrist": {"serial": "2", "width": 424, "height": 240, "fps": 15, "t_flange_camera": on_hand}}}


def test_a_rig_is_read_from_its_file(tmp_path):
    path = tmp_path / "rig.json"
    path.write_text(json.dumps(_rig_data()))
    rig = load_rig(path)
    assert rig.arm().table_z == 0.01 and rig.cameras["wrist"].width == 424 and rig.dynamics_factor == 0.1
    arm = rig.arm()
    assert np.allclose(rig.cameras["wrist"].pose(arm, READY_Q), arm.flange(READY_Q))
    assert np.allclose(rig.cameras["external"].pose(arm, READY_Q), np.eye(4))


@pytest.mark.parametrize("change, message", [
    (lambda d: d["cameras"]["wrist"].pop("t_flange_camera"), "exactly one"),
    (lambda d: d["cameras"]["wrist"].update(t_world_camera=np.eye(4).tolist()), "exactly one"),
    (lambda d: d["cameras"]["wrist"].update(t_flange_camera=[[1, 0], [0, 1]]), "4x4"),
    (lambda d: d.update(plan_camera="other"), "plan_camera"),
    (lambda d: d.update(t_world_base=[1, 2, 3]), "4x4"),
])
def test_a_wrong_rig_says_what_is_wrong(change, message):
    data = _rig_data()
    change(data)
    with pytest.raises(ValueError, match=message):
        rig_from_dict(data)


def test_a_rig_must_measure_its_table():
    data = _rig_data()
    data.pop("table_z")
    with pytest.raises(TypeError, match="table_z"):
        rig_from_dict(data)


# -- what the hardware path does without hardware ------------------------------------------------------------------
def test_depth_units_and_missing_depth():
    depth = depth_metres(np.array([[0, 1000], [2500, 65535]], np.uint16), 0.001)
    assert math.isnan(depth[0, 0]) and depth[0, 1] == pytest.approx(1.0) and depth[1, 0] == pytest.approx(2.5)


def test_opening_is_a_move_and_closing_a_grasp_that_takes_any_block():
    assert grip_command(0.08, 0.02, 0.08, speed_m_s=0.05, force_n=20.0) == {"kind": "move", "width": 0.08, "speed": 0.05}
    closing = grip_command(0.0, 0.08, 0.08, speed_m_s=0.05, force_n=20.0)
    assert closing["kind"] == "grasp" and closing["force"] == 20.0
    assert closing["width"] - closing["epsilon_inner"] <= 0.0
    assert closing["width"] + closing["epsilon_outer"] >= 0.08  # blocked anywhere up to fully open counts
    middle = grip_command(0.03, 0.08, 0.08, speed_m_s=0.05, force_n=20.0)
    assert middle["epsilon_inner"] == pytest.approx(0.03) and middle["epsilon_outer"] == pytest.approx(0.05)
    for wrong in (dict(width_m=0.09), dict(speed_m_s=0.5), dict(force_n=90.0), dict(force_n=0.0)):
        arguments = dict(width_m=0.03, speed_m_s=0.05, force_n=20.0) | wrong
        with pytest.raises(ValueError):
            grip_command(arguments.pop("width_m"), 0.08, 0.08, **arguments)


def test_the_driver_builds_franky_motions_the_real_library_accepts():
    franky = pytest.importorskip("franky")
    arm = Fr3()
    start = planner.goal_of(arm, _down_at(0.4, 0.0, 0.25), READY_Q)
    path = planner.plan_line(arm, start, _down_at(0.5, 0.05, 0.15), 0.1)
    motion = joint_waypoint_motion(franky, path)
    assert isinstance(motion, franky.JointWaypointMotion)


def test_what_the_driver_calls_exists_on_the_real_library():
    franky = pytest.importorskip("franky")
    for name in ("current_joint_state", "state", "has_errors", "move", "poll_motion", "join_motion", "stop",
                 "recover_from_errors"):
        assert hasattr(franky.Robot, name), name
    for name in ("max_width", "width", "move_async", "grasp_async", "stop"):
        assert hasattr(franky.Gripper, name), name
    for name in ("robot_mode", "current_errors", "control_command_success_rate"):
        assert hasattr(franky.RobotState, name), name
    assert hasattr(franky.BoolFuture, "wait") and hasattr(franky.BoolFuture, "get")
    assert {"Enforce", "Ignore"} <= set(franky.RealtimeConfig.__members__)
    for name in ("ControlException", "CommandException", "NetworkException", "RealtimeException", "GripperException"):
        assert issubclass(getattr(franky, name), fault_types(franky)), name
    # the keywords the driver passes are the library's
    for keyword in ("relative_dynamics_factor", "realtime_config"):
        assert keyword in franky.Robot.__init__.__doc__
    for keyword in ("width", "speed", "force", "epsilon_inner", "epsilon_outer"):
        assert keyword in franky.Gripper.grasp_async.__doc__
    assert issubclass(RobotFault, RuntimeError)
