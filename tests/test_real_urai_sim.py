"""hexashell/real/urai against URAI's own simulator of its PiPER-X rig (kinematic arms, rendered RGB-D)
over real HTTP: the end-to-end, carrying, host, vision, command-line and edge-case tests of the real-robot
harness this adapter was ported from, and its frame and RGB-D tests.

URAI is only a test dependency here (the package never imports it): these tests need urai with its
server (scipy, fastapi, uvicorn) and the harness routes (urai.routes.harness), and are skipped without them. No segmentation service is used.
The checks that need no URAI are in test_real_urai.py.
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
from dataclasses import replace
from pathlib import Path

import httpx
import numpy as np
import pytest

pytest.importorskip("urai")
pytest.importorskip("urai.routes.harness")
pytest.importorskip("scipy")
pytest.importorskip("fastapi")
uvicorn = pytest.importorskip("uvicorn")

from fastapi.testclient import TestClient  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from urai.app import create_app  # noqa: E402
from urai.backend import PiperBackend  # noqa: E402
from urai.geometry import Frame  # noqa: E402
from urai.robot.calibration import example_calibration_path, load_calibration  # noqa: E402
from urai.robot.rig import RigConfig, build_models  # noqa: E402
from urai.robot.scene import Box, Capsule, Scene, SceneObject, Table, render_rgbd  # noqa: E402
from urai.robot.simulated import build_simulated_runtime, default_scene  # noqa: E402
from urai.service import Service  # noqa: E402

import fakes_urai  # noqa: E402
from hexashell.errors import Refused  # noqa: E402
from hexashell.kernel.host import CallFailed, Kernel  # noqa: E402
from hexashell.real.urai.client import UraiClient  # noqa: E402
from hexashell.real.urai.frames import from_rpy, tcp_pose, to_rpy  # noqa: E402
from hexashell.real.urai.host import UraiHost  # noqa: E402
from hexashell.real.urai.observation import CAMERA_TO_URAI, Observation  # noqa: E402
from hexashell.real.urai.session import UraiSession  # noqa: E402
from hexashell.robot.results import mask_items  # noqa: E402
from hexashell.runtime.episode import Episode, Settings  # noqa: E402
from hexashell.runtime.records import Records  # noqa: E402
from hexashell.runtime.script import ScriptGateway  # noqa: E402
from hexashell.vision.cameras import Camera, check_cam  # noqa: E402
from hexashell.vision.overlay import jpeg  # noqa: E402
from hexashell.vision.rgbd import DepthScene, DepthView, spot  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


# -- URAI served -----------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def arm_models():
    """Both PiPER-X arm models of URAI's example rig (world frame = left arm base)."""
    return build_models(load_calibration(example_calibration_path()), RigConfig())


@pytest.fixture
def piper_service():
    """URAI's service on its PiPER-X backend driving the kinematic simulator."""
    runtime = build_simulated_runtime()
    backend = PiperBackend(runtime)
    yield Service(backend)
    backend.close()
    runtime.close()


def http_client(service) -> TestClient:
    """URAI's app in process; without a token it answers loopback host names only."""
    return TestClient(create_app(service), base_url="http://127.0.0.1")


def wait_until(predicate, timeout=10.0, interval=0.005):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition not reached in time"
        time.sleep(interval)


@pytest.fixture
def serve_robot(arm_models, tmp_path):
    """URAI's server on a free local port over the simulator, its red block at (0.27, -0.12) on the table;
    URAI logs each execution under tmp_path/urai_logs."""

    @contextmanager
    def serve(*, allow_motion=True, nonzero=False):
        original = default_scene(arm_models)
        block_pose = np.eye(4)
        block_pose[:3, 3] = [0.27, -0.12, original.table.z]
        objects = [replace(item, t_world_obj=block_pose) if item.name == "red_block" else item
                   for item in original.objects()]
        runtime = build_simulated_runtime(
            rig=RigConfig(head_width=320, head_height=240, wrist_width=320, wrist_height=240),
            scene=Scene(original.table, objects), allow_motion=allow_motion)
        backend = PiperBackend(runtime)
        service = Service(backend, log_dir=tmp_path / "urai_logs")
        if nonzero:
            q = [5.0, 10.0, -10.0, 0.0, 5.0, 0.0]
            runtime.arms["left"].command_joints(q)
            wait_until(lambda: np.allclose(runtime.arms["left"].joints(), q, atol=0.001))
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        server = uvicorn.Server(uvicorn.Config(create_app(service), log_level="error", ws="none"))
        thread = threading.Thread(target=server.run, kwargs={"sockets": [sock]}, daemon=True)
        thread.start()
        wait_until(lambda: server.started)
        try:
            yield f"http://127.0.0.1:{sock.getsockname()[1]}", runtime, service
        finally:
            service.cancel()
            if service.worker is not None:
                service.worker.join(30)
            server.should_exit = True
            thread.join(10)
            sock.close()
            backend.close()
            runtime.close()

    return serve


@contextmanager
def opened_cells(url, root, *, step_limit=None):
    """A session on ``url`` and the cell kernel an episode would give the model, without the model."""
    session = UraiSession(url, "验证真实机器人接口的边界", step_limit=step_limit)
    records = Records(root, session.info, name=session.record_name())
    pictures = []

    def attach(kind, image, caption):
        pictures.append(caption)
        records.picture(len(pictures), kind, jpeg(image, 90))

    kernel = Kernel(records.workspace, session.dispatch, proxy=session.proxy, log=records.event)
    try:
        session.open(Settings(), records, attach)
        kernel.start()
        yield session, kernel, records
    finally:
        kernel.close()
        session.close()
        records.close()


def episode_for(url, root, script, *, step_limit=None):
    source = root / "cells.py"
    source.write_text(script)
    session = UraiSession(url, "操作并验证真实仿真臂；红色方块为 40 mm 立方体", step_limit=step_limit)
    return Episode(session, Settings(script=str(source), max_turns=20, budget_s=300),
                   root / "runs")


def host_for(url, instruction, **kw):
    """A host straight on ``url`` and what it records, using URAI images and depth."""
    events, states, pictures = [], [], []
    client = UraiClient(url, log=events.append)
    host = UraiHost(client, instruction, log=events.append,
                    attach=lambda kind, image, caption: pictures.append((kind, image, caption)),
                    record_state=states.append, **kw)
    return client, host, pictures


def red_outline(view):
    rgb = view.image.astype(int)
    return (rgb[:, :, 0] > 2 * rgb[:, :, 1]) & (rgb[:, :, 0] > 2 * rgb[:, :, 2])


def json_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def execution_logs(root):
    return [json.loads(path.read_text()) for path in (root / "urai_logs").glob("*.json")]


def assert_arms_unchanged(before, after):
    for arm in before:
        np.testing.assert_allclose(after[arm]["joints_deg"], before[arm]["joints_deg"], atol=0.001)
        np.testing.assert_allclose(after[arm]["xyz"], before[arm]["xyz"], atol=1e-6)
        assert after[arm]["gripper_mm"] == pytest.approx(before[arm]["gripper_mm"], abs=0.01)


# -- an episode moves the block ------------------------------------------------------------------
MOVE_BLOCK = """# %% 读取与像素变换
print(robot.task)
print(robot.state())
print(robot.camera('head'))
assert robot.image('head').shape == (240, 320, 3)
pixel = robot.to_pixel((.27, -.12, .0435), 'head')
spot = robot.to_world('head', pixel, z=.0435)
assert np.linalg.norm(np.asarray(spot.xyz) - [.27, -.12, .0435]) < .001
robot.look('all', tile=True, grid=True, marks=[('world', (.27, -.12, .0435), 'block')])
# %% 到方块上方，保持 URAI TCP 指尖语义
approach = robot.goto('left', (.27, -.12, .12), pitch=90, yaw=-180, roll=0,
                      grip='open', path='swing')
assert approach.ok
# %% 下探并闭爪
assert robot.goto('left', (.27, -.12, .0235), path='line').ok
grip = robot.gripper('left', 'close')
assert grip.holding and .035 < grip.width_m < .05
# %% 真正抬起
assert robot.goto('left', by=(0, 0, .10), path='line').ok
robot.wait(1)
robot.look('head')
# %% 平移
assert robot.goto('left', (.20, -.02, .1235), path='line').ok
# %% 下降与释放
assert robot.goto('left', (.20, -.02, .0235), path='line').ok
assert not robot.gripper('left', 'open').holding
# %% 退开、回会话起点
assert robot.goto('left', by=(0, 0, .10), path='line').ok
assert robot.home('left').ok
assert robot.state().left.gripper > .99
robot.look('head')
"""


def test_episode_cells_carry_the_block_over_http(serve_robot, tmp_path):
    script = tmp_path / "move_block.py"
    script.write_text(MOVE_BLOCK)
    with serve_robot(nonzero=True) as (url, runtime, service):
        start = service.backend.state()
        initial = runtime.cameras.scene.pose("red_block").copy()
        session = UraiSession(url, "把红色方块搬到 (0.20, -0.02)，释放后回起点", task_name="move_block")
        episode = Episode(session, Settings(script=str(script), max_turns=12, budget_s=240), tmp_path / "runs")
        thread = threading.Thread(target=episode.run)
        thread.start()
        held_positions = []
        while thread.is_alive():
            if runtime.arms["left"].held_object() == "red_block":
                held_positions.append(runtime.cameras.scene.pose("red_block")[:3, 3].copy())
            thread.join(0.02)
        result = json.loads((episode.rec.dir / "result.json").read_text())
        cells = json_lines(episode.rec.dir / "cells.jsonl")
        assert result["errors"] == [], result
        assert result["end"] == "finish" and result["cells"] == 7, result
        assert all(cell["error"] is None for cell in cells), cells
        assert result["bench"] == "urai" and result["success"] is None
        assert result["agent_report"] == result["finish"] and result["evaluation"] == {"source": "none"}
        held = np.asarray(held_positions)
        assert len(held) and held[:, 2].max() > initial[2, 3] + 0.08
        assert np.linalg.norm(held[:, :2] - [0.20, -0.02], axis=1).min() < 0.015
        final = runtime.cameras.scene.pose("red_block")
        np.testing.assert_allclose(final[:2, 3], [0.20, -0.02], atol=0.015)
        assert final[2, 3] == pytest.approx(runtime.cameras.scene.table.z, abs=1e-6)
        assert runtime.arms["left"].held_object() is None
        after = service.backend.state()
        np.testing.assert_allclose(after["left"]["joints_deg"], start["left"]["joints_deg"], atol=0.2)
        np.testing.assert_allclose(after["right"]["joints_deg"], start["right"]["joints_deg"], atol=0.001)
        logs = execution_logs(tmp_path)
        assert len(logs) == 10
        assert all(log["preview"]["completion"] == "hold" for log in logs)
        assert all("return_home" not in log["result"] for log in logs)
        states = json_lines(episode.rec.dir / "states.jsonl")
        executed = {row["execution"]["id"] for row in states if "execution" in row}
        assert executed == {log["preview"]["id"] for log in logs}
        assert result["step"] == sum(math.ceil(log["execution"]["execution_time_s"] / 0.04) for log in logs)
        for name in ("events.jsonl", "transcript.json", "prompt.md"):
            assert (episode.rec.dir / name).stat().st_size > 0
        assert len(list((episode.rec.dir / "pictures").iterdir())) >= 6


# -- the host on the simulator --------------------------------------------------------------------
def test_goto_grip_carries_the_block_from_the_start_of_the_motion(serve_robot):
    """The fingers close as the motion starts: the block is carried from the pose the motion began in."""
    with serve_robot() as (url, runtime, _):
        client, host, _ = host_for(url, "夹爪在动作开始时生效")
        try:
            initial = runtime.cameras.scene.pose("red_block")[:3, 3].copy()
            assert host.goto("left", (0.27, -0.12, 0.12), pitch=90, yaw=-180, roll=0, grip="open", path="swing").ok
            assert host.goto("left", (0.27, -0.12, 0.0235), path="line").ok
            assert host.goto("left", by=(0, 0, 0.10), grip="close", path="line").ok
            assert runtime.arms["left"].held_object() == "red_block"
            assert host.holding("left")
            raised = runtime.cameras.scene.pose("red_block")[:3, 3]
            assert raised[2] > initial[2] + 0.08
            assert host.goto("left", by=(-0.05, 0, 0), grip="open", path="line").ok
            assert runtime.arms["left"].held_object() is None
            np.testing.assert_allclose(runtime.cameras.scene.pose("red_block")[:2, 3], [0.27, -0.12], atol=0.015)
        finally:
            host.close()
            client.close()


@pytest.mark.parametrize("hold", [False, True])
def test_empty_grasp_is_told_by_the_executions_outcome(serve_robot, tmp_path, hold):
    """Dropped comes from URAI's structured outcome (which it gives when the motion does not hold); a
    held motion's failure comes up as URAI's own error, not guessed from its words."""
    with serve_robot() as (url, _, service), opened_cells(url, tmp_path / "runs") as (session, _, _):
        client, host = session.client, session.host
        frame = client.observe()
        position = frame["arms"]["left"]["xyz"]
        draft = {"observation_id": frame["id"], "arms": {"left": {
            "path": {"mode": "waypoints", "points": [position, position]}, "speed": 0.03,
            "orientation": {"mode": "hold"},
            "gripper_events": [{"s": 0.0, "opening_mm": 0.0, "hold_s": 0.1,
                                "wait_for_arrival": True, "verify": "holding"}]}}}
        if hold:
            draft["completion"] = "hold"
        client.request("PUT", "/api/draft", draft)
        plan = client.request("POST", "/api/preview", {})
        if hold:
            with pytest.raises(RuntimeError) as raised:
                host._execute(plan)
            assert str(raised.value) == service.execution["error"]
            assert "result" not in service.execution
        else:
            with pytest.raises(CallFailed) as raised:
                host._execute(plan)
            assert raised.value.kind == "Dropped"
            assert service.execution["result"]["outcome"] == "empty_grasp"
        assert host.step > 0


def test_look_outlines_take_nested_masks_and_things_only(serve_robot):
    with serve_robot() as (url, _, _):
        client, host, pictures = host_for(url, "核实轮廓的实际输入类型")
        try:
            view = host.observations["head"].view
            scene = DepthScene("fake", cams=["head"], check_cam=check_cam, client=fakes_urai.FakeSam3())
            try:
                mask = scene.register(view, red_outline(view), "red block", 1.0, {})
                thing = scene.measure([mask], cams=["head"])
            finally:
                scene.close()
            assert thing.found
            assert host._outlines(None, "head") == []
            assert host._outlines(mask, "left_wrist") == []
            assert host._outlines(thing, "left_wrist") == []
            nested = [mask, (thing,), thing.masks]
            assert [label for _, label in host._outlines(nested, "head")] == [
                f"#{mask.id} red block", f"T{thing.id} red block", f"#{mask.id} red block"]
            picture = host.look("head", masks=nested)
            assert picture.cams == ["head"]
            assert pictures[0][0] == "look" and pictures[0][1].shape[:2] == view.image.shape[:2]
            assert mask_items(["red block"]) == ["red block"]
            with pytest.raises(Refused, match=r"look\(masks=\.\.\.\)"):
                host.look("head", masks=["red block"])
        finally:
            host.close()
            client.close()


@pytest.mark.parametrize("speed, expected", [(0.12, 0.12), (0.8, 0.5)])
def test_gentle_caps_the_speed_urai_plans_with(serve_robot, speed, expected):
    with serve_robot(allow_motion=False) as (url, _, service):
        client, host, _ = host_for(url, "规划端点速度", speed_m_s=speed)
        try:
            before = service.backend.state()
            move = host.goto("left", by=(0.01, 0, 0), gentle=True, dry_run=True, path="line")
            assert move.reached and not move.moved
            plan = service.preview_data
            assert plan["input_draft"]["arms"]["left"]["speed_m_s"] == expected
            assert move.steps == math.ceil(plan["duration_s"] / 0.04) > 0
            assert host.step == 0
            for arm, state in before.items():
                np.testing.assert_allclose(service.backend.state()[arm]["joints_deg"], state["joints_deg"])
            assert plan["motion_limits"]["joint_speed_deg_s"] <= math.degrees(0.15 / 0.04)
        finally:
            host.close()
            client.close()


# -- the command line ----------------------------------------------------------------------------
def _without(*names):
    gone = {*names, "HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy"}
    return {key: value for key, value in os.environ.items() if key not in gone}


@pytest.mark.parametrize("url_source", ["argument", "environment"])
def test_command_line_records_real_cells(serve_robot, tmp_path, url_source):
    script = tmp_path / "read_and_wait.py"
    script.write_text("print(robot.state())\nassert robot.wait(1).steps >= 1\n")
    with serve_robot() as (url, _, _):
        config = tmp_path / "settings.json"
        config.write_text(json.dumps({"sam3_url": url, "budget_s": 60}))  # shared configuration must not enable segmentation
        env = _without("SAM3_URL", "HEXASHELL_SAM3_URL", "URAI_URL")
        url_args = ["--url", url] if url_source == "argument" else []
        if url_source == "environment":
            env["URAI_URL"] = url
        result = subprocess.run([sys.executable, "-m", "hexashell.real.urai", *url_args, "--task", "读取实测状态并等待一拍",
                                 "--config", str(config), "--script", str(script), "--results", str(tmp_path / "runs")],
                                cwd=ROOT, env=env, text=True, capture_output=True, timeout=120)
        assert result.returncode == 0, result.stdout + result.stderr
        records = list((tmp_path / "runs").glob("*/result.json"))
        assert len(records) == 1 and result.stdout.strip() == str(records[0].parent)
        report = json.loads(records[0].read_text())
        assert report["settings"]["sam3_url"] == url
        assert report["end"] == "finish" and report["success"] is None and report["step"] >= 1
        assert report["errors"] == [], report
        events = json_lines(records[0].parent / "events.jsonl")
        assert not any(e["kind"] in ("sam3", "perception_unavailable") for e in events)
        states = json_lines(records[0].parent / "states.jsonl")
        assert states[-1]["execution"]["state"] == "completed"


@pytest.mark.parametrize("url_env", [None, ""], ids=["missing", "empty"])
def test_command_line_needs_the_urai_url_first(tmp_path, url_env):
    env = _without("URAI_URL")
    if url_env is not None:
        env["URAI_URL"] = url_env
    result = subprocess.run([sys.executable, "-m", "hexashell.real.urai", "--task", "读取状态",
                             "--config", str(tmp_path / "missing.json"), "--results", str(tmp_path / "runs")],
                            cwd=ROOT, env=env, text=True, capture_output=True, timeout=60)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "--url" in result.stderr and "URAI_URL" in result.stderr
    assert "Traceback" not in result.stderr
    assert not (tmp_path / "runs").exists()


# -- edges -----------------------------------------------------------------------------------------
@pytest.mark.parametrize("path", ["line", "auto", "swing"])
def test_unreachable_call_stops_the_cell_dry_run_is_free_and_a_catch_goes_on(serve_robot, tmp_path, path):
    with serve_robot(nonzero=True) as (url, _, service), opened_cells(url, tmp_path / "runs") as (session, kernel, rec):
        before = service.backend.state()
        result = kernel.run(f"robot.goto('left', (9., 9., 9.), path={path!r})\n"
                            "robot.gripper('left', 'open')\nprint('must not run')")
        assert "NotExecuted" in result.error
        assert [call["name"] for call in result.calls] == ["goto"]
        assert "must not run" not in result.output
        assert session.step == 0 and execution_logs(tmp_path) == []
        assert_arms_unchanged(before, service.backend.state())
        dry = kernel.run("before_step = robot.state().step\n"
                         f"reachable = robot.goto('left', by=(.01, 0., 0.), grip='open', path={path!r}, dry_run=True)\n"
                         "assert reachable.reached and not reachable.moved and reachable.steps > 0\n"
                         f"blocked = robot.goto('left', (9., 9., 9.), path={path!r}, dry_run=True)\n"
                         "assert not blocked.reached and not blocked.moved and blocked.steps == 0\n"
                         "assert robot.state().step == before_step")
        assert dry.error is None, dry.error
        assert session.step == 0 and execution_logs(tmp_path) == []
        assert_arms_unchanged(before, service.backend.state())
        caught = kernel.run(f"try:\n    robot.goto('left', (9., 9., 9.), path={path!r})\n"
                            "except NotExecuted:\n    print('caught refusal')\n"
                            "waited = robot.wait(1)\nassert waited.steps >= 1\nprint('continued')")
        assert caught.error is None, caught.error
        assert "caught refusal" in caught.output and "continued" in caught.output
        assert [call["name"] for call in caught.calls] == ["goto", "wait"]
        logs = execution_logs(tmp_path)
        assert len(logs) == 1 and logs[0]["execution"]["state"] == "completed"
        assert session.step == math.ceil(logs[0]["execution"]["execution_time_s"] / 0.04)
        assert_arms_unchanged(before, service.backend.state())
        states = json_lines(rec.dir / "states.jsonl")
        assert len(states) == 2 and states[-1]["step"] == session.step


def test_readonly_service_plans_and_refuses_motion_before_the_gripper(serve_robot, tmp_path):
    with serve_robot(allow_motion=False) as (url, runtime, service), \
            opened_cells(url, tmp_path / "runs") as (session, kernel, _):
        before = service.backend.state()
        assert not runtime.motion_allowed
        dry = kernel.run("planned = robot.goto('left', by=(.01, 0., 0.), grip='open', path='line', dry_run=True)\n"
                         "assert planned.reached and not planned.moved and planned.steps > 0")
        assert dry.error is None, dry.error
        assert session.step == 0 and execution_logs(tmp_path) == []
        assert_arms_unchanged(before, service.backend.state())
        refused = kernel.run("robot.goto('left', by=(.01, 0., 0.), grip='open', path='line')\n"
                             "robot.gripper('left', 'open')")
        assert "RobotError" in refused.error and "motion is not allowed" in refused.error
        assert [call["name"] for call in refused.calls] == ["goto"]
        # URAI checks the motion gate as its execution starts: the error keeps the time it took
        logs = execution_logs(tmp_path)
        assert len(logs) == 1 and logs[0]["execution"]["state"] == "error"
        assert "motion is not allowed" in logs[0]["execution"]["error"]
        assert session.step == math.ceil(logs[0]["execution"]["execution_time_s"] / 0.04)
        assert_arms_unchanged(before, service.backend.state())


def test_cancelled_wait_stops_the_cell_and_counts_the_time_it_ran(serve_robot, tmp_path):
    script = "robot.wait(200)\nrobot.gripper('left', 'open')\nprint('must not run')"
    with serve_robot() as (url, _, service), httpx.Client(base_url=url, trust_env=False) as client:
        before = service.backend.state()
        episode = episode_for(url, tmp_path, script)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(episode.run)
            wait_until(lambda: client.get("/api/state").json()["execution"]["state"] == "running", timeout=30)
            wait_until(lambda: client.get("/api/state").json()["execution"].get("elapsed_s", 0.0) >= 0.08)
            assert client.post("/api/cancel").status_code == 200
            future.result(timeout=30)
        assert isinstance(episode.gateway, ScriptGateway)
        cells = json_lines(episode.rec.dir / "cells.jsonl")
        assert len(cells) == 1 and "cancelled" in cells[0]["error"]
        assert [call["name"] for call in cells[0]["calls"]] == ["wait"]
        assert "must not run" not in cells[0]["output"]
        logs = execution_logs(tmp_path)
        assert len(logs) == 1 and logs[0]["execution"]["state"] == "cancelled"
        elapsed = logs[0]["execution"]["execution_time_s"]
        assert 0.08 <= elapsed < 8.0
        result = json.loads((episode.rec.dir / "result.json").read_text())
        assert result["errors"] == [] and result["step"] == math.ceil(elapsed / 0.04)
        assert_arms_unchanged(before, service.backend.state())
        assert json_lines(episode.rec.dir / "states.jsonl")[-1]["execution"]["state"] == "cancelled"


def test_wait_200_runs_eight_seconds_and_holds_both_arms(serve_robot, tmp_path):
    with serve_robot(nonzero=True) as (url, _, service), opened_cells(url, tmp_path / "runs") as (session, kernel, rec):
        before = service.backend.state()
        samples = []
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(kernel.run, "waited = robot.wait(200)\nassert waited.steps >= 200\n"
                                             "print('whole wait returned')")
            while not future.done():
                samples.append(service.backend.state())
                time.sleep(0.05)
            result = future.result()
        assert result.error is None and "whole wait returned" in result.output
        assert [call["name"] for call in result.calls] == ["wait"]
        assert len(samples) >= 80
        for state in samples:
            assert_arms_unchanged(before, state)
        logs = execution_logs(tmp_path)
        assert len(logs) == 1 and logs[0]["execution"]["state"] == "completed"
        elapsed = logs[0]["execution"]["execution_time_s"]
        assert elapsed >= 8.0 and session.step == math.ceil(elapsed / 0.04) >= 200
        assert_arms_unchanged(before, service.backend.state())
        states = json_lines(rec.dir / "states.jsonl")
        assert len(states) == 2 and states[-1]["step"] == session.step


def test_right_relative_moves_keep_omitted_angles_and_home_keeps_the_grippers(serve_robot, tmp_path):
    with serve_robot(nonzero=True) as (url, _, service), opened_cells(url, tmp_path / "runs") as (session, kernel, rec):
        before = service.backend.state()
        result = kernel.run("initial = robot.state()\n"
                            "first = robot.goto('right', by=(.01, 0., 0.), grip=.4, path='line')\n"
                            "assert first.ok and np.allclose(first.target_angles, initial.right.angles)\n"
                            "assert np.allclose(first.target, np.asarray(initial.right.at) + [.01, 0., 0.])\n"
                            "second = robot.goto('right', by=(0., 0., .005), yaw=initial.right.angles[1]+3.,\n"
                            "                    path='line')\n"
                            "assert second.ok\n"
                            "assert second.target_angles == (initial.right.angles[0], initial.right.angles[1]+3., "
                            "initial.right.angles[2])\n"
                            "third = robot.goto('right', by=(0., .005, 0.), path='line')\n"
                            "assert third.ok and third.target_angles == second.target_angles\n"
                            "assert robot.goto('left', by=(.01, 0., 0.), path='line').ok\n"
                            "robot.gripper('left', .3)\n"
                            "opened = robot.state()\n"
                            "assert abs(opened.left.gripper - .3) < .01 and abs(opened.right.gripper - .4) < .01\n"
                            "assert robot.home('both').ok\n"
                            "returned = robot.state()\n"
                            "assert abs(returned.left.gripper - opened.left.gripper) < .001\n"
                            "assert abs(returned.right.gripper - opened.right.gripper) < .001\n"
                            "import json\n"
                            "json.dump({'initial_angles': initial.right.angles,\n"
                            "           'partial_angles': second.target_angles,\n"
                            "           'omitted_angles': third.target_angles}, open('angles.json', 'w'))")
        assert result.error is None, result.error
        after = service.backend.state()
        for arm in before:
            np.testing.assert_allclose(after[arm]["joints_deg"], before[arm]["joints_deg"], atol=0.2)
        maximum = {arm: session.host.model["arms"][arm]["max_opening_mm"] for arm in ("left", "right")}
        assert after["left"]["gripper_mm"] == pytest.approx(0.3 * maximum["left"], abs=0.1)
        assert after["right"]["gripper_mm"] == pytest.approx(0.4 * maximum["right"], abs=0.1)
        angles = json.loads((rec.workspace / "angles.json").read_text())
        assert angles["partial_angles"] == angles["omitted_angles"]
        logs = execution_logs(tmp_path)
        assert len(logs) == 6 and all(log["execution"]["state"] == "completed" for log in logs)
        assert all(log["preview"]["completion"] == "hold" for log in logs)
        assert session.step == sum(math.ceil(log["execution"]["execution_time_s"] / 0.04) for log in logs)


def test_step_budget_is_checked_before_the_next_call_not_during_a_motion(serve_robot, tmp_path):
    script = ("waited = robot.wait(4)\nassert waited.steps >= 4\nprint('whole wait returned')\n"
              "robot.gripper('left', 'open')\nprint('must not run')")
    with serve_robot() as (url, _, service):
        before = service.backend.state()
        episode = episode_for(url, tmp_path, script, step_limit=2)
        episode.run()
        assert isinstance(episode.gateway, ScriptGateway)
        cells = json_lines(episode.rec.dir / "cells.jsonl")
        assert len(cells) == 1 and cells[0]["stopped"]
        assert "whole wait returned" in cells[0]["output"] and "must not run" not in cells[0]["output"]
        assert [call["name"] for call in cells[0]["calls"]] == ["wait", "gripper"]
        assert "stop" in cells[0]["calls"][1]
        logs = execution_logs(tmp_path)
        assert len(logs) == 1 and logs[0]["execution"]["state"] == "completed"
        assert logs[0]["execution"]["execution_time_s"] >= 0.16
        result = json.loads((episode.rec.dir / "result.json").read_text())
        assert result["end"] == "episode_over" and result["step_limit"] == 2
        assert result["step"] == math.ceil(logs[0]["execution"]["execution_time_s"] / 0.04) >= 4
        assert_arms_unchanged(before, service.backend.state())


def test_server_gone_fails_the_call_and_nothing_after_it(serve_robot, tmp_path):
    with ExitStack() as cells:
        with serve_robot() as (url, _, service):
            session, kernel, _ = cells.enter_context(opened_cells(url, tmp_path / "runs"))
            assert kernel.run("assert robot.state().step == 0").error is None
            before = service.backend.state()
        with pytest.raises(httpx.ConnectError):
            session.client.state()
        failed = kernel.run("robot.state()\nrobot.gripper('left', 'open')\nprint('must not run')")
        assert "ConnectError" in failed.error
        assert [call["name"] for call in failed.calls] == ["state"]
        assert "must not run" not in failed.output
        assert session.step == 0 and execution_logs(tmp_path) == []
        assert_arms_unchanged(before, session.host.arms)


# -- frames, cameras and pictures against URAI's own geometry -------------------------------------
@pytest.mark.parametrize("arm", ["left", "right"])
def test_fk_read_back_in_urais_world_and_tcp(arm_models, arm):
    fk = arm_models[arm].fk_tcp_world([10.0, 20.0, -20.0, 0.0, 5.0, 0.0])
    state = {"xyz": fk[:3, 3].tolist(),
             "rpy_deg": Rotation.from_matrix(fk[:3, :3]).as_euler("xyz", degrees=True).tolist()}
    np.testing.assert_allclose(tcp_pose(state), fk, atol=1e-12)
    back = Rotation.from_euler("xyz", to_rpy(*from_rpy(state["rpy_deg"])), degrees=True).as_matrix()
    np.testing.assert_allclose(back, fk[:3, :3], atol=1e-4)


def encoded(array):
    stream = io.BytesIO()
    np.save(stream, array, allow_pickle=False)
    return {"encoding": "npy-base64", "data": base64.b64encode(stream.getvalue()).decode("ascii")}


def packet(rgb, depth, k, pose, capsules=()):
    return {"id": "geometry-frame", "timestamp": 1.0, "camera": "head",
            "rgb": encoded(rgb), "depth": encoded(depth), "k": k.tolist(), "t_world_camera": pose.tolist(), "arms": {},
            "capsules": {"left": [{"name": f"link-{i}", "a": c.p.tolist(), "b": c.q.tolist(), "radius_m": c.radius}
                                  for i, c in enumerate(capsules)]}}


def top_camera():
    k = np.array([[120.0, 0.0, 63.0], [0.0, 180.0, 60.0], [0.0, 0.0, 1.0]])
    pose = np.diag([1.0, -1.0, -1.0, 1.0])
    pose[2, 3] = 1.0
    return k, pose


def test_camera_from_cv_projects_as_urai_does():
    k = np.array([[420.0, 0.0, 113.2], [0.0, 610.0, 72.7], [0.0, 0.0, 1.0]])
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [17.0, -32.0, 9.0], degrees=True).as_matrix()
    pose[:3, 3] = [0.2, -0.1, 0.5]
    camera = Camera.from_cv("head", k, pose, 240, 160)
    world = np.array([[0.0, 0.0, 1.0], [0.05, -0.03, 0.8], [-0.08, 0.04, 1.4]]) @ pose[:3, :3].T + pose[:3, 3]
    frame = Frame(np.zeros((160, 240, 3), np.uint8), np.ones((160, 240)), k, pose)
    np.testing.assert_allclose(camera.project(world)[0], frame.project(world), atol=1e-12)


@pytest.mark.parametrize("fx, fy", [(120.0, 240.0), (240.0, 120.0)])
def test_back_projection_lands_on_urais_world_points(fx, fy):
    k, pose = top_camera()
    k[0, 0], k[1, 1] = fx, fy
    rgb = np.zeros((128, 128, 3), np.uint8)
    depth = np.full((128, 128), 0.5, np.float32)
    camera = Camera.from_cv("head", k, pose, 128, 128)
    view = DepthView("head", rgb, depth, camera, np.zeros(depth.shape, bool), frame_id="back-projection-frame")
    pixels = np.array([[30.0, 45.0], [90.0, 100.0]])
    cloud = view.back_project(pixels[:, 0], pixels[:, 1], np.full(2, 0.5))
    world = Frame(rgb, depth, k, pose).world_points()
    np.testing.assert_allclose(cloud, world[pixels[:, 1].astype(int), pixels[:, 0].astype(int)])
    measured = spot({"head": view}, "head", (camera.cx, camera.cy))
    np.testing.assert_allclose(measured.xyz, [0.0, 0.0, 0.5])


def test_observation_decodes_urais_frames_and_camera_names(piper_service):
    assert CAMERA_TO_URAI == {"head": "head", "left_wrist": "hand_left", "right_wrist": "hand_right"}
    with http_client(piper_service) as client:
        for name, remote in CAMERA_TO_URAI.items():
            response = client.post("/api/harness/observe", json={"camera": remote})
            assert response.status_code == 200, response.text
            data = response.json()
            observation = Observation.from_packet(data, step=17)
            view = observation.view
            assert observation.id == data["id"] == view.frame_id
            assert observation.timestamp == data["timestamp"] and observation.arms == data["arms"]
            assert view.cam == name and view.camera.name == name and view.step == 17
            np.testing.assert_array_equal(view.camera.k, data["k"])
            for field, array in [("rgb", view.image), ("depth", view.depth)]:
                expected = np.load(io.BytesIO(base64.b64decode(data[field]["data"])), allow_pickle=False)
                np.testing.assert_array_equal(array, expected)
                assert array.dtype == expected.dtype
            assert view.robot.dtype == bool and view.robot.shape == view.depth.shape
            if name == "head":
                assert view.robot.any() and not view.robot.all()
                ys, xs = np.nonzero(view.valid())
                every = np.arange(0, len(xs), 100)
                points = view.back_project(xs[every], ys[every], view.depth[ys[every], xs[every]])
                np.testing.assert_allclose(points, piper_service.frame.world_points()[ys[every], xs[every]], atol=1e-7)


def test_robot_pixels_leave_out_what_hides_the_arm_and_the_background():
    k, pose = top_camera()
    capsule = Capsule(np.array([-0.25, 0.0, 0.3]), np.array([0.25, 0.0, 0.3]), 0.07, (30, 120, 230))
    block_pose = np.eye(4)
    block_pose[:3, 3] = [0.0, 0.0, 0.5]
    block = SceneObject("occluder", [Box((0.12, 0.2, 0.1), (240, 20, 20))], block_pose)
    rgb, depth = render_rgbd(k, pose, 128, 128, Table(0.0, (-1.0, 1.0), (-1.0, 1.0)), [block], [capsule])
    view = Observation.from_packet(packet(rgb, depth, k, pose, [capsule]), step=0).view
    visible_capsule = rgb[:, :, 2] > 2 * rgb[:, :, 0].astype(int)
    foreground = rgb[:, :, 0] > 2 * rgb[:, :, 2].astype(int)
    assert visible_capsule.any() and foreground.any()
    assert view.robot[visible_capsule].all()
    assert not view.robot[foreground].any()
    assert not view.robot[depth > 0.9].any()


def test_measurements_keep_their_frames_and_each_arm_carries_its_own():
    k, pose = top_camera()
    objects = []
    for x, color in [(-0.2, (240, 20, 20)), (0.2, (20, 20, 240))]:
        transform = np.eye(4)
        transform[:3, 3] = [x, 0.0, 0.0]
        objects.append(SceneObject(str(x), [Box((0.09, 0.06, 0.05), color)], transform))
    rgb, depth = render_rgbd(k, pose, 128, 128, Table(0.0, (-1.0, 1.0), (-1.0, 1.0)), objects)
    view = Observation.from_packet(packet(rgb, depth, k, pose), step=23).view
    scene = DepthScene("fake", cams=["head"], check_cam=lambda name: name, client=fakes_urai.FakeSam3())
    hands = {"left": np.eye(4), "right": np.eye(4)}
    try:
        scene.bind(lambda names: {"head": view}, hands={arm: (lambda arm=arm: hands[arm]) for arm in hands})
        things = {}
        for arm, channel in [("left", 0), ("right", 2)]:
            outline = rgb[:, :, channel] > 2 * rgb[:, :, 2 - channel].astype(int)
            mask = scene.register(view, outline, arm, 1.0, {})
            assert mask.frame_id == view.frame_id
            things[arm] = scene.measure([mask])
            assert things[arm].frame_ids == {"head": view.frame_id} and things[arm].step == 23
            hands[arm][:3, 3] = things[arm].center
            scene.grasped(hands[arm], arm)
        hands["left"][0, 3] += 0.1
        hands["right"][1, 3] += 0.2
        for arm, delta in [("left", [0.1, 0.0, 0.0]), ("right", [0.0, 0.2, 0.0])]:
            thing = things[arm]
            np.testing.assert_allclose(scene._place(thing.id, scene.records[thing.id]),
                                       np.asarray(thing.center) + delta)
        scene.released(hands["left"], "left")
        assert things["left"].id not in scene.carried and things["right"].id in scene.carried
        hands["right"][1, 3] += 0.1
        right = things["right"]
        np.testing.assert_allclose(scene._place(right.id, scene.records[right.id]),
                                   np.asarray(right.center) + [0.0, 0.3, 0.0])
    finally:
        scene.close()


def test_depth_locates_a_real_frame_without_segmentation(piper_service):
    with http_client(piper_service) as client:
        response = client.post("/api/harness/observe", json={"camera": "head"})
        assert response.status_code == 200, response.text
        view = Observation.from_packet(response.json(), step=0).view
    item = piper_service.backend.runtime.cameras.scene.objects()[0]
    _, high = item.local_bounds()
    top = (item.t_world_obj @ [0.0, 0.0, high[2], 1.0])[:3]
    xy = view.camera.pixel(top)
    assert xy is not None
    point = spot({"head": view}, "head", xy)
    assert point.xyz is not None
    np.testing.assert_allclose(point.xyz, top, atol=0.01)
