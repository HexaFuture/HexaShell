"""Checks of the benchmark adapters that need neither a simulator nor the model (RoboCasa so far).

The RoboCasa side runs against the fake kitchen of ``tests/fakes_casa.py``: same operations, same
replies, through the same wire encoding.  Run: ``python tests/test_benches.py`` (or with pytest)
from the checkout, in the policy environment.
"""

from __future__ import annotations

import contextlib
import dataclasses
import json
import math
import os
import re
import shutil
import socket
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from hexashell.benches import results as R  # noqa: E402
from hexashell.benches import wire  # noqa: E402
from hexashell.benches.robocasa import protocol as P  # noqa: E402
from hexashell.benches.pose import angles_of, rotation_of, turn_deg  # noqa: E402
from hexashell.kernel.host import CallFailed, Stop  # noqa: E402
from hexashell.robot import results as RR  # noqa: E402
from hexashell.runtime.episode import Settings  # noqa: E402
from hexashell.runtime.records import Records  # noqa: E402
from hexashell.vision.rgbd import closing_yaw_robocasa  # noqa: E402


@contextlib.contextmanager
def raises(kind, text: str | None = None):
    try:
        yield
    except kind as exc:
        if text is not None:
            assert text in str(exc), f"{text!r} not in {exc!r}"
        return
    raise AssertionError(f"{kind.__name__} not raised")


def _failed(etype: str):
    """CallFailed of that kind (the cell sees it as that exception)."""

    class Match:
        exc = None

    @contextlib.contextmanager
    def check():
        try:
            yield Match
        except CallFailed as exc:
            assert exc.kind == etype, (exc.kind, exc.text)
            Match.exc = exc
            return
        raise AssertionError(f"CallFailed({etype}) not raised")

    return check()


# -- wire ------------------------------------------------------------------------------------------
def test_wire_round_trip():
    a, b = socket.socketpair()
    payload = {"x": np.arange(6, dtype=np.float32).reshape(2, 3), "mask": np.eye(3, dtype=bool),
               "img": np.full((4, 5, 3), 7, np.uint8), "raw": b"\x00\x01", "t": (1, 2.5, None, "s"),
               "n": np.int64(3), "f": np.float32(0.5), "ok": np.bool_(True), "nested": [{"y": np.zeros(0)}]}
    try:
        wire.send(a, payload)
        got = wire.recv(b)
    finally:
        a.close()
        b.close()
    assert got["x"].dtype == np.float32 and got["x"].shape == (2, 3) and got["x"][1, 2] == 5
    assert got["mask"].dtype == bool and got["mask"].sum() == 3
    assert got["img"].shape == (4, 5, 3) and got["raw"] == b"\x00\x01"
    assert got["t"] == [1, 2.5, None, "s"] and got["n"] == 3 and got["f"] == 0.5 and got["ok"] is True
    assert got["nested"][0]["y"].shape == (0,)
    with raises(wire.WireError):
        wire.encode({"bad": np.array([1 + 2j])})
    with raises(wire.WireError):
        wire.encode({"bad": object()})
    with raises(wire.WireError):
        wire.decode({"body": {"__blob__": 3}, "blobs": []}, [])


def test_read_env_file():
    from hexashell.benches.session import read_env_file

    folder = Path(tempfile.mkdtemp(prefix="hexashell-env-"))
    path = folder / "env.sh"
    path.write_text("# a comment: run \"$X\" ...\nexport A=/x/y\nexport B=pro   # RLinf's routing\n"
                    "C='a b'\nexport D=\"$HOME/z\"  # it's here\nnot a setting\n")
    env = read_env_file(path)
    assert env == {"A": "/x/y", "B": "pro", "C": "a b", "D": os.path.expanduser("~") + "/z"}, env
    shutil.rmtree(folder)


def test_sim_server_and_bench_session():
    from hexashell.benches.session import BenchSession, SimError, SimRefused, SimServer, read_env_file

    out = Path(tempfile.mkdtemp(prefix="hexashell-bench-"))
    env = out / "env.sh"
    env.write_text("# a comment\nexport HEXA_TEST_A=1\nHEXA_TEST_B='two words'\nexport HEXA_TEST_C=$HOME/x\n")
    values = read_env_file(env)
    assert values["HEXA_TEST_A"] == "1" and values["HEXA_TEST_B"] == "two words"
    assert values["HEXA_TEST_C"] == os.path.expandvars("$HOME/x")

    class EchoSession(BenchSession):
        server_module = "hexashell.benches.echo"

    session = EchoSession({"task_name": "t", "eval_seed": 0}, python=sys.executable, env_file=str(env))
    try:
        session.start_server(out / "server.log")
        reply = session.call("echo", a=np.arange(3, dtype=np.int32), b=(1, 2))
        assert reply["args"]["a"].tolist() == [0, 1, 2] and reply["args"]["b"] == [1, 2]
        assert session.step == 1 and session.step_limit == 100 and not session.over
        with raises(SimRefused, "not like that"):
            session.call("refuse")
        with raises(SimError, "ZeroDivisionError"):
            session.call("crash")
        assert session.verdict == {"success": False, "step": 3, "step_limit": 100, "finished": False}
        session.fail("it broke")
        assert session.over and session.verdict["failure"] == "it broke"
        assert session.errors == ["simulator: it broke"]
        with raises(SimError):
            session.call("exit")
    finally:
        session.close()
    with raises(ValueError):
        SimServer("os", sys.executable)
    shutil.rmtree(out)


def test_failed_open_takes_no_status():
    """A failed open may carry the status of the server's previous episode: it is not taken."""
    from hexashell.benches.session import BenchSession, SimError

    class Stale:
        def call(self, op, **args):
            raise SimError("simulator server: RuntimeError: could not initialize task",
                           {"step": 1824, "horizon": 2250, "success": True, "over": True})

    session = BenchSession({"task_name": "t", "eval_seed": 3}, python=sys.executable)
    session.server = Stale()
    with raises(SimError, "could not initialize"):
        session.call("open", size=64)
    assert session.status == {} and session.verdict is None and not session.over
    with raises(SimError):
        session.call("goto")  # a request in the episode: its status is the episode's
    assert session.verdict["success"] and session.over


def test_bench_score_keeps_a_verdict_over_later_errors():
    import importlib.util

    spec = importlib.util.spec_from_file_location("bench_score", ROOT / "scripts" / "bench_score.py")
    score = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(score)
    out = Path(tempfile.mkdtemp(prefix="hexashell-score-"))
    records = {"A_s0-1": {"task": "A", "seed": 0, "success": True, "end": "episode_over"},
               "A_s0-2": {"task": "A", "seed": 0, "success": False, "end": "gateway_error"},
               "B_s0-1": {"task": "B", "seed": 0, "success": True, "end": "error"},  # the last episode's
               "C_s0-1": {"task": "C", "seed": 0, "success": False, "end": "gateway_error"},
               "C_s0-2": {"task": "C", "seed": 0, "success": True, "end": "episode_over"}}
    for name, r in records.items():
        (out / name).mkdir()
        (out / name / "result.json").write_text(json.dumps(r))
    best = score.load(str(out))
    assert {k[0]: (v["success"], v["end"]) for k, v in best.items()} == {
        "A": (True, "episode_over"), "B": (False, "error"), "C": (True, "episode_over")}
    shutil.rmtree(out)


def test_warm_server():
    """One server process plays episode after episode (python -m hexashell.benches.daemon)."""
    import subprocess

    from hexashell.benches.session import BenchSession, SimError, SimRefused, SimServer, server_env

    out = Path(tempfile.mkdtemp(prefix="hexashell-warm-"))
    sock = str(out / "s.sock")

    class EchoSession(BenchSession):
        server_module = "hexashell.benches.echo"

    def daemon(*extra):
        return subprocess.Popen([sys.executable, "-m", "hexashell.benches.daemon", "hexashell.benches.echo", sock,
                                 *extra], env=server_env(None, "7"), cwd=str(out), stdin=subprocess.DEVNULL,
                                stdout=open(out / "daemon.log", "ab"), stderr=subprocess.STDOUT)

    proc = daemon("--max-episodes", "4")
    try:
        # the first episode waits for the server to listen
        one = EchoSession({"task_name": "t", "eval_seed": 0}, python=sys.executable)
        one.sim_socket = sock
        (out / "one").mkdir()
        try:
            one.start_server(out / "one" / "server.log")
            reply = one.call("echo", a=1)
            assert reply["pid"] == proc.pid and reply["warm"] and reply["prepared"] == ["7"]
            assert one.server.warm == {"pid": proc.pid, "episode": 1}
            assert one.server_facts() == {"warm_server": {"pid": proc.pid, "episode": 1}}
            with raises(SimRefused, "not like that"):
                one.call("refuse")
            with raises(SimError, "ZeroDivisionError"):
                one.call("crash")
            assert one.step == 3  # the server's own count of requests
            one.call("win")
            assert one.verdict["success"]
        finally:
            one.close()
        # the next episode's scene cannot be built: the server still has the last one, whose success
        # is not this episode's
        stale = EchoSession({"task_name": "t", "eval_seed": 1}, python=sys.executable)
        stale.sim_socket = sock
        try:
            stale.start_server(out / "stale.log")
            try:
                stale.call("open", fail=True)
                raise AssertionError("the open did not fail")
            except SimError as exc:
                assert "could not build the scene" in str(exc) and not exc.status, exc.status
            assert stale.status == {} and stale.verdict is None
        finally:
            stale.close()
        # another server module is turned away
        with raises(SimError, "warm server of hexashell.benches.echo"):
            SimServer("hexashell.benches.libero.server", sys.executable, address=sock).start()
        # the second episode goes away without closing: the server closes it itself
        two = SimServer("hexashell.benches.echo", sys.executable, address=sock, log_path=out / "two.log")
        two.start()
        reply = two.call("echo", b=2)
        assert reply["pid"] == proc.pid and reply["closed"] == 2 and two.warm["episode"] == 3
        two.sock.close()
        two.sock = None
        three = SimServer("hexashell.benches.echo", sys.executable, address=sock, log_path=out / "three.log")
        three.start()
        assert three.call("echo")["closed"] == 3 and three.warm["episode"] == 4
        three.close()
        assert proc.wait(timeout=30) == 0  # after its 4 episodes
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    assert not os.path.exists(sock)
    log_one = (out / "one" / "server.log").read_text()
    assert f"[warm server pid {proc.pid}: episode 1 of this process]" in log_one and "echo {'a': 1}" in log_one
    assert "echo {'b': 2}" in (out / "two.log").read_text() and "echo {'b': 2}" not in log_one
    log = (out / "daemon.log").read_text()
    assert "ready on" in log and "episode 3: the episode's side went away; closed here" in log
    assert "refused a hello for 'hexashell.benches.libero.server'" in log and "episode 4: closed" in log
    # nothing listening: the episode gives up after connect_s
    late = SimServer("hexashell.benches.echo", sys.executable, address=str(out / "none.sock"))
    late.connect_s = 0.5
    with raises(SimError, "none.sock"):
        late.start()
    # an idle server ends by itself
    proc = daemon("--idle-s", "1")
    try:
        assert proc.wait(timeout=60) == 0
    finally:
        if proc.poll() is None:
            proc.kill()
    assert "no episode for 1 s: ending" in (out / "daemon.log").read_text()
    shutil.rmtree(out)


# -- RoboCasa: protocol, poses, results ----------------------------------------------------------------
def test_protocol_sets_and_kitchens():
    sets = {name: [t for t, _ in tasks] for name, tasks in P.TASKS.items()}
    assert [len(sets[k]) for k in ("atomic_seen", "composite_seen", "composite_unseen")] == [18, 16, 16]
    assert len(P.HORIZON) == 50 and len(set(P.PRETRAIN50)) == 50
    assert P.task_set("CloseFridge") == "atomic_seen" and P.task_set("ArrangeTea") == "composite_unseen"
    with raises(ValueError):
        P.task_set("MakeCoffeeTwice")
    assert [P.seed_set(s) for s in (0, 9, 10, 19, 20, 49, 50)] == ["learn", "learn", "val", "val", "test", "test",
                                                                   "learn"]
    for seed in range(50):
        assert P.kitchen(seed) == P.PRETRAIN50[seed]
    # a kitchen the task cannot use is replaced by the next of the same seed set, wrapping inside it
    k5, k6 = P.PRETRAIN50[5], P.PRETRAIN50[6]
    assert P.kitchen(5, unbuildable=[k5]) == k6
    assert P.kitchen(9, unbuildable=[P.PRETRAIN50[9]]) == P.PRETRAIN50[0]
    assert P.kitchen(5, exclude_layouts=[k5[0]]) == (k6[0], k5[1])
    # (layout, style, seed) entries skip a kitchen for that seed only
    assert P.skipped("SeparateFreezerRack", 38) == [(30, 37), (37, 42), (27, 51)]
    assert P.skipped("SeparateFreezerRack", 39) == [(30, 37), (37, 42)]
    assert P.skipped("CloseFridge", 0) == []
    assert P.REPLIES[P.task_set("CloseFridge")] == 40 and P.EPISODE_S == 7200.0
    # the default split, and RoboCasa's own evaluation split
    assert P.split_of(None) == P.split_of("") == "pretrain50" and P.split_of("pretrain") == "pretrain"
    with raises(ValueError):
        P.split_of("target")


def test_pose_angles():
    rng = np.random.default_rng(0)
    for _ in range(200):
        p, y, r = rng.uniform(-80, 80), rng.uniform(-179, 179), rng.uniform(-179, 179)
        back = angles_of(rotation_of(p, y, r))
        assert np.allclose(back, (p, y, r), atol=0.06), ((p, y, r), back)
    # pointing straight down, roll folds into yaw
    for y in (-120.0, -30.0, 0.0, 45.0, 170.0):
        for r in (0.0, 20.0):
            p2, y2, r2 = angles_of(rotation_of(90.0, y, r))
            assert p2 == 90.0 and r2 == 0.0 and abs(((y2 - (y - r)) + 180) % 360 - 180) < 0.06
    down = rotation_of(90.0, 0.0, 0.0)
    assert np.allclose(down[:, 0], (0, 0, -1)) and np.allclose(down[:, 1], (0, 1, 0))
    assert np.allclose(rotation_of(0.0, 90.0, 0.0)[:, 0], (0, 1, 0))
    assert abs(turn_deg(rotation_of(90, 0, 0), rotation_of(90, 30, 0)) - 30.0) < 1e-6
    # near vertical the roll folds into the yaw (exact pointing straight down or up)
    from hexashell.benches.pose import folded, shown_angles

    assert folded((86.8, 180.0, 180.0)) == (86.8, 0.0, 0.0) and folded((45.0, 10.0, 20.0)) == (45.0, 10.0, 20.0)
    assert folded((-85.0, 30.0, 20.0)) == (-85.0, 50.0, 0.0) and folded((90.0, 30.0, 0.0)) == (90.0, 30.0, 0.0)
    for y, r in ((180.0, 180.0), (30.0, -50.0), (-120.0, 75.0)):
        assert np.allclose(rotation_of(90.0, y, r), rotation_of(*folded((90.0, y, r))), atol=1e-9)
        assert np.allclose(rotation_of(-90.0, y, r), rotation_of(*folded((-90.0, y, r))), atol=1e-9)
    tilted = rotation_of(86.8, 180.0, 180.0)
    assert shown_angles(tilted) == (86.8, 0.0, 0.0) and turn_deg(tilted, rotation_of(86.8, 0.0, 0.0)) < 6.5
    # farther from vertical, the angles print as near the asked ones as the rotation allows: a pitch may
    # read past 90 (the same rotation as pitch 180 - p with yaw and roll turned half round)
    from hexashell.benches.pose import nearest, past_vertical, shown_near

    back = rotation_of(105.0, 0.0, 0.0)
    assert shown_angles(back) == (75.0, 180.0, 180.0) or shown_angles(back) == (75.0, -180.0, -180.0) \
        or abs(shown_angles(back)[1]) > 179.9, shown_angles(back)
    assert np.allclose(shown_near(back, (90.0, 0.0, 0.0)), (105.0, 0.0, 0.0), atol=0.02)
    assert np.allclose(shown_near(rotation_of(150.0, -90.0, 0.0), (150.0, -90.0, 0.0)), (150.0, -90.0, 0.0), atol=0.02)
    assert np.allclose(shown_near(rotation_of(30.0, 20.0, 5.0), (30.0, 25.0, 0.0)), (30.0, 20.0, 5.0), atol=0.02)
    for a in ((30.0, 20.0, 5.0), (-60.0, 170.0, -100.0), (75.0, 180.0, 180.0)):
        assert np.allclose(rotation_of(*a), rotation_of(*past_vertical(a)), atol=1e-6)
        assert nearest(a, a) == a and nearest(past_vertical(a), a) == a



def test_closing_yaw_matches_the_fingers():
    for yaw in (-80.0, -45.0, 0.0, 30.0, 89.0):
        fingers = rotation_of(90.0, yaw, 0.0)[:, 1]
        assert abs(closing_yaw_robocasa(fingers[:2]) - yaw) < 0.11, yaw
        assert abs(closing_yaw_robocasa(-fingers[:2]) - yaw) < 0.11, yaw
    assert closing_yaw_robocasa((0.0, 1.0)) == 0.0 and closing_yaw_robocasa((1.0, 0.0)) == 90.0


def test_results_print():
    ok = R.Move(reached=True, moved=True, target=(0.6, 0, 1.0), target_angles=(90, 0, 0), at=(0.6, 0, 1.0),
                angles=(90, 0, 0), err_mm=1, steps=18, steps_left=500)
    assert ok.ok and str(ok).startswith("Move: at (0.600, 0.000, 1.000) pitch 90") and "[500 steps left]" in str(ok)
    blocked = R.Move(reached=False, moved=True, target=(0.6, 0, 0.8), target_angles=(90, 0, 0), at=(0.6, 0, 0.91),
                     angles=(90, 0, 0), err_mm=110, steps=9, blocked=True, notes=["pressing"])
    assert not blocked.ok and "BLOCKED" in str(blocked) and "note: pressing" in str(blocked)
    no = R.Move(reached=False, moved=False, target=(3, 3, 3), target_angles=(0, 0, 0), at=(0.5, 0, 1.1),
                angles=(45, 0, 0), why="too far")
    assert str(no).startswith("Move: NOT executed - too far")
    dry = R.Move(reached=True, moved=False, target=(0.6, 0, 1.0), target_angles=(90, 0, 0), at=(0.5, 0, 1.1),
                 angles=(45, 0, 0), dry_run=True)
    assert str(dry).startswith("Move (dry run): reachable")
    assert "holding" in str(R.Grip(opening=0, width_m=0.05, holding=True, goal=0))
    assert "closed fully" in str(R.Grip(opening=0, width_m=0.0, holding=False, goal=0))
    assert "open 1.00" in str(R.Grip(opening=1, width_m=0.08, holding=False, goal=1))
    d = R.Drive(reached=True, blocked=False, at=(0.1, 0), heading=0, target=(0.1, 0), target_heading=0, steps=8)
    assert str(d).startswith("Drive: base at (0.100, 0.000) heading 0")
    s = R.Status(step=10, step_limit=600, at=(0.5, 0, 1.1), angles=(45, 0, 0), gripper=1.0, width_m=0.08, goal=1.0,
                 holding=False, base=(0, 0), heading=0, home_mm=0, home_deg=0)
    assert "step 10 of 600 (590 left)" in str(s) and "(home)" in str(s) and s.steps_left == 590
    t = R.TaskInfo(instruction="open the fridge", step=0, step_limit=900, start_at=(0.5, 0, 1.1),
                   start_angles=(45, 0, 0))
    assert str(t).startswith("Task: open the fridge") and t.steps_left == 900


def test_thing_masks_for_look():
    import io
    import pickle

    from hexashell.kernel.wire import HostUnpickler

    def mask(i, cam):
        return RR.Mask(id=i, cam=cam, label="cup", score=0.9, pixels=np.zeros((2, 2), bool), box=(0, 0, 1, 1),
                       centroid=(0.5, 0.5), area=1)

    a = RR.Thing(id=1, label="cup", found=True, masks={"left": mask(1, "left"), "right": mask(2, "right")})
    b = RR.Thing(id=2, label="plate", found=True, masks={"left": mask(3, "left")})
    assert isinstance(a.masks, RR.Views) and a.masks["left"].id == 1 and isinstance(RR.Thing(0, "x", False).masks,
                                                                                       RR.Views)
    assert [m.id for m in a.masks + b.masks] == [1, 2, 3]
    assert [m.id for m in [mask(9, "left")] + a.masks] == [9, 1, 2]
    assert [getattr(m, "id", None) for m in RR.mask_items([a, b.masks, (mask(4, "wrist"),)])] == [1, 3, 4]
    assert RR.mask_items(None) == [] and RR.mask_items(a) == [a]
    # the worker may send a Thing's masks back to the host
    got = HostUnpickler(io.BytesIO(pickle.dumps(a.masks + b.masks))).load()
    assert [m.id for m in got] == [1, 2, 3]
    got = HostUnpickler(io.BytesIO(pickle.dumps(a))).load()
    assert isinstance(got.masks, RR.Views) and got.masks["right"].id == 2


# -- RoboCasa: perception on the fake kitchen ------------------------------------------------------------
def _views(kitchen):
    from hexashell.vision.cameras import Camera
    from hexashell.vision.rgbd import DepthView

    obs = kitchen.op_observe(["left", "right", "wrist"])["obs"]
    views = {}
    for cam in obs["images"]:
        c = obs["cameras"][cam]
        camera = Camera(cam, float(c["focal"]), np.asarray(c["pose"]), int(c["width"]), int(c["height"]))
        views[cam] = DepthView(cam, obs["images"][cam], obs["depth"][cam], camera, obs["robot"][cam], step=obs["step"])
    return views


def test_depth_scene_measures_a_box():
    import fakes_casa as F
    from hexashell.benches.robocasa.host import check_cam
    from hexashell.errors import Refused
    from hexashell.vision.rgbd import DepthScene, spot

    kitchen = F.FakeKitchen(size=256)
    kitchen.reset()
    views = _views(kitchen)
    scene = DepthScene("fake", cams=["left", "right", "wrist"], check_cam=check_cam, client=F.FakeSam3())
    asked = []
    scene.bind(lambda cams: asked.append(list(cams)) or views)
    try:
        box, top = F.BOX_START, F.BOX_START[2] + F.BOX_SIZE[2] / 2
        thing = scene.measure("red box")
        assert thing.found and set(thing.views) >= {"left", "right"}, thing
        assert np.allclose(thing.center[:2], box[:2], atol=0.01), thing
        assert abs(thing.top - top) < 0.006 and abs(thing.center[2] - box[2]) < 0.025, thing
        length, width, _ = thing.size
        assert abs(length - F.BOX_SIZE[0]) < 0.015 and abs(width - F.BOX_SIZE[1]) < 0.015, thing
        assert thing.grip_yaw is not None and abs(thing.grip_yaw) < 10, thing
        assert thing.on.startswith("z ") and abs(float(thing.on[2:]) - F.COUNTER_TOP) < 0.03, thing
        assert abs(thing.on_z - float(thing.on[2:])) < 0.001 and repr(thing).count(" z ") == 1, thing

        assert asked[0] == ["left", "right", "wrist"], asked
        masks = scene.find("left", "red box")
        assert len(masks) == 1 and masks[0].cam == "left" and masks[0].area > 50, masks
        assert asked[-1] == ["left"], asked  # find asks for the one picture it outlines
        by_mask = scene.measure(masks[0])
        assert by_mask.found and "right" in by_mask.views and np.allclose(by_mask.center, thing.center, atol=0.01)
        cx, cy = masks[0].centroid
        by_pixel = scene.measure(("left", (cx, cy)))
        assert by_pixel.found and np.allclose(by_pixel.center, thing.center, atol=0.01), by_pixel
        again = scene.measure(thing)
        assert again.id == thing.id and not any("moved" in n for n in again.notes)
        assert [t.id for t in scene.things()] == [1, 2, 3]  # words, mask, pixel; measuring #1 again keeps it
        # the words outline nothing now: measured where it was instead, when what is there has its size
        scene.sam.WORDS = {"counter": F.COUNTER}
        placed = scene.measure(thing)
        assert placed.found and placed.id == thing.id and "outlined nothing now" in placed.notes[0], placed
        assert np.allclose(placed.center, thing.center, atol=0.01), placed
        huge = dataclasses.replace(thing, size=(0.5, 0.5, 0.5))
        lost = scene.measure(huge)
        assert not lost.found and any("it may have moved" in n for n in lost.notes), lost
        assert scene.records[thing.id].thing is placed  # the rejected outline did not replace the record
        del scene.sam.WORDS
        missing = scene.measure("green bottle")
        assert not missing.found and "no 'green bottle'" in missing.why_missing
        with raises(RR.NotFound):
            missing.center

        # points: depth (the right camera sees the top's middle; the hand hides it from the left one),
        # a plane, two rays
        on_top_r = views["right"].camera.pixel((box[0], box[1], top))
        s = spot(views, "right", on_top_r)
        assert s.source == "the depth picture" and np.allclose(s.xyz, (box[0], box[1], top), atol=0.006), s
        assert s.warning is None, s
        on_top = views["left"].camera.pixel((box[0], box[1], top))
        s = spot(views, "left", on_top)
        assert s.warning and "robot" in s.warning, s
        s = spot(views, "left", on_top, z=top)
        assert s.source.startswith("plane z") and np.allclose(s.xyz, (box[0], box[1], top), atol=0.002), s
        other = views["right"].camera.pixel((box[0], box[1], top))
        s = spot(views, "left", on_top, pair=("right", other))
        assert s.gap_mm < 1 and np.allclose(s.xyz, (box[0], box[1], top), atol=0.002) and s.warning is None, s
        s = spot(views, "left", on_top, pair=("right", (other[0] + 40, other[1])))
        assert s.xyz is None or s.warning, s
        assert spot(views, "left", (128, 250)).xyz is not None  # the counter's front or the floor
        assert spot(views, "left", (0, 0)).xyz is None  # above the counter: nothing within reach of the depth
        with raises(Refused):
            spot(views, "left", on_top, pair=("left", on_top))
        with raises(Refused):
            spot(views, "left", ("a", 1))
        with raises(Refused):
            scene.measure(42)
    finally:
        scene.close()


def test_depth_view_leaves_out_the_robot_and_the_background():
    import fakes_casa as F

    kitchen = F.FakeKitchen(size=256)
    kitchen.reset()
    view = _views(kitchen)["left"]
    hand = view.camera.pixel(kitchen.tcp()[:3, 3])
    assert hand is not None and view.robot[int(round(hand[1])), int(round(hand[0]))]
    disc = np.zeros(view.robot.shape, bool)
    yy, xx = np.mgrid[0:256, 0:256]
    disc[(xx - hand[0]) ** 2 + (yy - hand[1]) ** 2 < 15 ** 2] = True
    pts = view.points(disc)
    assert len(pts) > 0 and np.all(np.linalg.norm(pts - kitchen.tcp()[:3, 3], axis=1) > F.HAND_R - 0.003)
    assert len(view.points(disc, robot_too=True)) > len(pts)
    # an outline spilling over the box's edge onto the counter far behind keeps the box
    box_px = view.camera.pixel(F.BOX_START)
    edge = np.zeros(view.robot.shape, bool)
    edge[int(box_px[1]) - 4:int(box_px[1]) + 4, int(box_px[0]) - 4:int(box_px[0]) + 30] = True
    pts = view.points(edge)
    assert np.median(pts[:, 2]) > F.COUNTER_TOP


def test_depth_scene_flags_what_does_not_stand_out_in_depth():
    import fakes_casa as F
    from hexashell.benches.robocasa.host import check_cam
    from hexashell.vision.rgbd import DepthScene

    kitchen = F.FakeKitchen(size=256)
    kitchen.reset()
    views = _views(kitchen)

    def measure(views):
        scene = DepthScene("fake", cams=["left", "right", "wrist"], check_cam=check_cam, client=F.FakeSam3())
        scene.bind(lambda cams: views)
        return scene.measure("red box")

    solid = measure(views)
    assert solid.found and solid.warning is None and not any("sees through" in n for n in solid.notes), solid
    # the same box see-through to the depth picture (as a clear plastic box is in MuJoCo): the depth of its
    # pixels is the counter's
    clear = {}
    for cam, v in views.items():
        _, bare, _ = F.render(v.camera.pose, v.camera.focal, (v.camera.width, v.camera.height),
                              [(*F.COUNTER_BOX, F.COUNTER)])
        box = np.all(v.image == F.COLOURS[F.BOX], axis=-1)
        clear[cam] = dataclasses.replace(v, depth=np.where(box, bare, v.depth).astype(np.float32))
    thing = measure(clear)
    assert thing.found and thing.size[2] < 0.005, thing
    assert "sees through" in thing.warning and f"(z {F.COUNTER_TOP:.3f})" in thing.warning, thing


def test_surroundings():
    from hexashell.vision.rgbd import surroundings

    xs, ys = np.meshgrid(np.linspace(0.3, 0.7, 40), np.linspace(-0.2, 0.2, 40))
    flat = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, 0.9)], axis=1)
    band = flat[(np.abs(flat[:, 0] - 0.5) > 0.12) | (np.abs(flat[:, 1]) > 0.12)]
    inside = flat[(np.abs(flat[:, 0] - 0.5) < 0.1) & (np.abs(flat[:, 1]) < 0.1)]
    seen_through = surroundings([band], inside)
    assert seen_through.level and seen_through.flush and abs(seen_through.z - 0.9) < 1e-6, seen_through
    board = surroundings([band], inside + (0, 0, 0.025))  # only the top of a 2.5 cm board shows
    assert board.level and not board.flush and abs(board.above - 0.025) < 1e-6, board
    wall = np.stack([np.full(xs.size, 1.0), xs.ravel() - 0.5, 0.6 + ys.ravel() + 0.3], axis=1)
    poster = surroundings([wall[np.abs(wall[:, 1]) > 0.12]], wall[np.abs(wall[:, 1]) < 0.1])
    assert not poster.level and poster.flush, poster
    rng = np.random.default_rng(0)
    assert surroundings([band + rng.normal(0, 0.05, band.shape)], inside) is None  # no one surface around
    # things standing next to it fill a third of the band: the surface is still the counter's
    cluttered = band.copy()
    cluttered[: len(band) // 3, 2] += rng.uniform(0.006, 0.05, len(band) // 3)
    beside = surroundings([cluttered], inside)
    assert beside.level and beside.flush and abs(beside.z - 0.9) < 1e-6, beside


# -- RoboCasa: the host against the fake server ------------------------------------------------------
def _open(sam3=True, horizon=600, dish=False):
    import fakes_casa as F

    out = Path(tempfile.mkdtemp(prefix="hexashell-robocasa-"))
    session = F.FakeCasaSession(sam3=sam3, kitchen=F.FakeKitchen(horizon=horizon, size=256, dish=dish))
    rec = Records(out, session.info, name=session.record_name())
    pictures = []
    session.open(Settings(sam3_url="fake" if sam3 else ""), rec, lambda kind, image, caption: pictures.append(caption))
    return session, rec, pictures, out


def test_casa_host_moves():
    import fakes_casa as F

    session, rec, pictures, out = _open(horizon=700)
    host = session.host
    call = host.dispatch
    try:
        st = call("state", (), {})
        assert isinstance(st, R.Status) and st.base == (0.0, 0.0) and st.heading == 0 and st.gripper == 1.0
        assert st.home_mm == 0 and st.angles == (45.0, 0.0, 0.0) and st.step == 0, st
        info = call("task", (), {})
        assert info.instruction == "pick up the red box from the counter" and info.step_limit == 700

        with _failed("NotExecuted") as f:
            call("goto", ((3, 3, 3),), {})
        assert "cannot get there" in f.exc.text and "shoulder" in f.exc.text
        dry = call("goto", ((3, 3, 3),), {"dry_run": True})
        assert not dry.reached and not dry.moved and session.step == 0

        m = call("goto", ((0.6, 0.1, 1.1),), {"pitch": 90})
        assert m.ok and m.steps > 5 and np.allclose(m.at, (0.6, 0.1, 1.1), atol=0.002) and m.angles[0] == 90, m
        assert m.steps_left == 700 - session.step
        m = call("goto", (), {"by": (0, 0, -0.05)})
        assert m.ok and abs(m.at[2] - 1.05) < 0.002 and m.target_angles == (90.0, 0.0, 0.0), m
        # slower when asked: the same way back up in more steps; a speed out of range is refused
        slow = call("goto", (), {"by": (0, 0, 0.05), "speed": 0.05})
        assert slow.ok and abs(slow.at[2] - 1.1) < 0.002 and slow.steps > 2 * m.steps, (slow, m)
        for bad in (0.3, 0.001, "fast"):
            with _failed("ValueError") as f:
                call("goto", (), {"by": (0, 0, -0.05), "speed": bad})
            assert "speed" in f.exc.text, f.exc.text
        call("goto", (), {"by": (0, 0, -0.05)})
        m = call("goto", ((0.7, 0.3, 0.85),), {})
        assert m.blocked and not m.ok and any("pressing" in n for n in m.notes), m
        # stopped with the elbow straight: the note says so instead of pressing
        session.kitchen.at_limit = [[4, "high"]]
        m = call("goto", ((0.7, 0.3, 0.85),), {})
        assert m.blocked and any("joint 4 (the elbow) straight" in n and "base closer" in n for n in m.notes), m
        assert not any("pressing" in n for n in m.notes), m
        # stopped as the elbow straightened: the arm gave way, and the note says so
        session.kitchen.stretch = 13.0
        m = call("goto", (), {"by": (0, 0, 0.02)})
        assert m.moved and not m.ok and "short of target" in str(m), m
        assert any("nearly straight on the way" in n and "went back part of the way it came" in n
                   and "the hand moved 7 cm" in n and "drive the base closer" in n for n in m.notes), m
        assert not any("pressed on something" in n for n in m.notes), m
        # pressed on something with the arm stretched out: stopped and pulled back, and the note says so
        session.kitchen.jam = 55.0
        m = call("goto", (), {"by": (0, 0, 0.02)})
        assert m.moved and m.blocked and not m.ok, m
        assert any("5 cm back toward the shoulder" in n and "drive the base closer first" in n for n in m.notes), m
        assert not any("pressing on something" in n for n in m.notes), m
        # stretched almost straight now: gotos and arcs are refused (dry runs say so too) until home
        session.kitchen.stretched_now = 12.0
        d = call("goto", (), {"by": (0, 0, 0.02), "dry_run": True})
        assert not d.ok and not d.moved and "robot.home() first" in str(d), d
        assert not any("cannot get there" in n for n in d.notes), d
        with _failed("NotExecuted") as f:
            call("arc", ((0.5, 0.0, 1.0), 10), {})
        assert "stretched almost straight now (the elbow 12 deg" in f.exc.text, f.exc.text
        call("home", (), {})
        assert call("goto", (), {"by": (0, 0, 0.02), "dry_run": True}).ok
        # a pose the arm gets to from home but not from its pose now: the note says home first, not drive
        session.kitchen.home_only = True
        with _failed("NotExecuted") as f:
            call("goto", (), {"by": (0, 0, 0.02)})
        assert "from home (robot.home()) it gets there" in f.exc.text and "robot.home() first, then the same move" \
            in f.exc.text and "drive the base closer" not in f.exc.text, f.exc.text
        call("home", (), {})
        assert call("goto", (), {"by": (0, 0, 0.02), "dry_run": True}).ok
        # the arm gave way while the fingers moved
        session.kitchen.give = 3
        g = call("gripper", (0.9,), {})
        assert any("5 cm back toward the shoulder" in n for n in g.notes), g
        assert not call("wait", (5,), {}).notes
        session.kitchen.give = 1
        w = call("wait", (5,), {})
        assert any("5 cm back toward the shoulder" in n for n in w.notes) and "note:" in str(w), w
        assert not any("crept on" in n for n in w.notes), w
        # ... as the hold crept until the elbow was nearly straight: the note says that instead
        session.kitchen.give, session.kitchen.creep = 1, True
        w = call("wait", (5,), {})
        assert any("crept on until its elbow was nearly straight" in n and "went back part of the way it came" in n
                   for n in w.notes), w
        assert not any("pressed on something" in n for n in w.notes), w
        # ... and with no way back that bends the elbow, pulled the hand in
        session.kitchen.give, session.kitchen.creep, session.kitchen.creep_back = 1, True, False
        w = call("wait", (5,), {})
        assert any("crept on until" in n and "5 cm back toward the shoulder" in n for n in w.notes), w
        session.kitchen.creep_back = True
        session.kitchen.drive_jam = True
        dr = call("drive", (), {"to": (0, 0), "heading": 0})
        assert dr.blocked and any("the base stopped: the arm" in n and "5 cm back toward" in n for n in dr.notes), dr
        assert any("pressed on something" in n for n in dr.notes), dr
        session.kitchen.drive_stretch = True
        dr = call("drive", (), {"to": (0, 0), "heading": 0})
        assert dr.blocked and any("straightening its elbow" in n and "went back part of the way it came" in n
                                  for n in dr.notes), dr
        assert not any("pressed on something" in n for n in dr.notes), dr
        g = call("gripper", ("close",), {})
        assert g.holding is False and "closed fully" in str(g), g
        g = call("gripper", (0.5,), {})
        assert g.holding is False and abs(g.width_m - 0.04) < 1e-6, g

        x0 = host._tcp()[0, 3]
        d = call("drive", (0.1,), {})
        assert d.reached and d.at == (0.1, 0.0) and abs(host._tcp()[0, 3] - x0 - 0.1) < 1e-6, d
        d = call("drive", (0.5,), {})
        assert d.blocked and any("stopped short" in n for n in d.notes), d
        d = call("drive", (), {"turn": 30})
        assert d.reached and d.heading == 30.0 and host.asked[1] == 30.0, d
        d = call("drive", (), {"to": (0, 0), "heading": 0})
        assert d.reached and d.at == (0.0, 0.0) and d.heading == 0.0 and host.asked[1] == 0.0, d
        with _failed("ValueError"):
            call("drive", (1.0,), {"to": (0, 0)})
        with _failed("ValueError"):
            call("drive", (5.0,), {})
        # turning about a floor point: the base and the hand go round it
        hand0, pivot, t = host._tcp()[:2, 3].copy(), np.array([0.3, -0.4]), math.radians(40)
        rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        d = call("drive", (), {"turn": 40, "about": tuple(pivot)})
        assert d.reached and d.heading == 40.0 and np.allclose(d.at, pivot - rot @ pivot, atol=1e-3), d
        assert np.allclose(host._tcp()[:2, 3], pivot + rot @ (hand0 - pivot), atol=1e-3) and host.asked[1] == 40.0
        for kwargs in ({"about": (0, 0)}, {"turn": 200, "about": (0, 0)}, {"turn": 20, "about": "hinge"},
                       {"forward": 0.1, "turn": 20, "about": (0, 0)}, {"turn": 20, "about": (0, 0), "heading": 0}):
            with _failed("ValueError"):
                call("drive", (), kwargs)
        call("drive", (), {"to": (0, 0), "heading": 0})
        # a dry run says where the base and the hand would end, and which way the hand first goes; nothing moves
        hand0, step0 = host._tcp()[:3, 3].copy(), session.step
        dry = call("drive", (), {"turn": 40, "about": tuple(pivot), "dry_run": True})
        assert dry.dry_run and dry.steps == 0 and session.step == step0 and np.allclose(host._tcp()[:3, 3], hand0), dry
        assert np.allclose(dry.target, pivot - rot @ pivot, atol=1e-3) and dry.target_heading == 40.0, dry
        assert np.allclose(dry.hand[:2], pivot + rot @ (hand0[:2] - pivot), atol=1e-3) and dry.hand[2] == round(hand0[2], 3)
        assert str(dry).startswith("Drive (dry run): the base would end at") and "first back (toward the base)" in str(dry)
        assert not any("pushes the door shut" in n for n in dry.notes), dry
        d = call("drive", (0.2,), {"dry_run": True})
        assert d.target == (0.2, 0.0) and np.allclose(d.hand, hand0 + [0.2, 0, 0], atol=1e-3) and not d.notes, d
        # with something held, a turn that sends the hand forward would push a door shut: said before and after
        host.holding = True
        dry = call("drive", (), {"turn": -40, "about": tuple(pivot), "dry_run": True})
        assert any("first forward (away from the base)" in n and "turn=40" in n for n in dry.notes), dry
        session.kitchen.drive_jam = True
        d = call("drive", (), {"turn": -6, "about": tuple(pivot)})
        assert d.blocked and any("pushes the door shut" in n and "turn=6" in n for n in d.notes), d
        host.holding = False
        call("drive", (), {"to": (0, 0), "heading": 0})

        # pressing on a fixture pushed the base back: the arm got there relative to the base only
        session.kitchen.shove = np.array([-0.02, 0.0])
        m = call("goto", (), {"by": (0.03, 0, 0)})
        assert m.moved and not m.ok and m.err_mm > 15 and any("base was pushed" in n for n in m.notes), m
        assert "short of target" in str(m), m
        call("drive", (), {"to": (0, 0), "heading": 0})

        h = call("home", (), {})
        assert h.ok and host.asked == (45.0, 0.0, 0.0), h
        start = host._tcp()[:3, 3].copy()
        hinge = start + np.array([0.0, -0.3, 0.0])
        dry = call("arc", (tuple(hinge), 30), {"dry_run": True})
        # counter-clockwise seen from above: the hand swings from +y of the hinge toward -x
        expect = hinge + np.array([-0.3 * math.sin(math.radians(30)), 0.3 * math.cos(math.radians(30)), 0.0])
        assert dry.dry_run and dry.reached and np.allclose(dry.target, expect, atol=1e-3), dry
        m = call("arc", (tuple(hinge), 30), {"axis": "z"})
        assert m.ok and np.allclose(m.at, expect, atol=0.002) and abs(m.angles[1] - 30) < 0.5, m
        slow = call("arc", (tuple(hinge), -30), {"axis": "z", "speed": 0.05})
        assert slow.ok and slow.steps > 2 * m.steps, (slow, m)
        call("arc", (tuple(hinge), 30), {"axis": "z"})
        # holding something 0.3 m from the line (a door's handle), an arc asked for no speed goes at 0.1 m/s until
        # arcs holding it have turned 30 degrees about that line (the start of a door's swing); 0.1 m from it (a
        # tap), at the arm's own
        host.holding = True
        held = call("arc", (tuple(hinge), -30), {"axis": "z"})
        assert held.ok and held.steps > 1.5 * m.steps, (held, m)
        # the same line (3 cm off it, along it the other way): turned 30 degrees already, so at the arm's speed
        back = call("arc", (tuple(hinge + np.array([0.03, 0.0, 0.2])), -30), {"axis": "-z"})
        assert back.ok and back.steps < 1.2 * m.steps and len(host.swung) == 1 and host.swung[0][2] == 0, back
        held = call("arc", (tuple(hinge), -30), {"axis": "z"})
        assert held.ok and held.steps > 1.5 * m.steps and host.swung[0][2] == -30, (held, m)
        near = start + np.array([0.0, -0.1, 0.0])
        tap = call("arc", (tuple(near), 30), {"axis": "z"})
        assert tap.ok and tap.steps == m.steps and len(host.swung) == 2, (tap, m)
        call("arc", (tuple(near), -30), {"axis": "z"})
        arcs = [e for e in map(json.loads, (rec.dir / "events.jsonl").read_text().splitlines()) if e["kind"] == "arc"]
        assert [(e.get("held_slow", False), e["swung"]) for e in arcs[-5:]] == [
            (True, 0.0), (False, 30.0), (True, 0.0), (False, 0.0), (False, 30.0)], arcs[-5:]
        host.holding = False
        with _failed("ValueError"):
            call("arc", (tuple(hinge), 200), {})
        with _failed("ValueError"):
            call("arc", (tuple(hinge), 30), {"axis": (0, 0, 0)})
        call("home", (), {})
        # a push about the same hinge: the hand goes around it as far as the part turns
        p = call("push_arc", (tuple(hinge), 30), {})
        assert p.ok and p.turned == 30 and "went 30 of 30 deg around the hinge (all the way)" in str(p), p
        assert np.allclose(p.at, expect, atol=0.002) and abs(host.asked[1] - 30) < 0.5 and not p.notes, p
        session.kitchen.push_stop = (0.5, "stalled")
        p = call("push_arc", (tuple(hinge), -30), {"push": (1, 0, 0), "press": 0.05})
        assert not p.ok and p.stop == "stalled" and any("stopped turning at 15 deg" in n for n in p.notes), p
        session.kitchen.push_stop = (0.0, "no touch")
        p = call("push_arc", (tuple(hinge), 30), {})
        assert p.stop == "no touch" and any("8 cm along the push without meeting" in n for n in p.notes), p
        session.kitchen.push_stop = (0.4, "slid")
        p = call("push_arc", (tuple(hinge), 30), {})
        assert not p.ok and p.stop == "slid" and any("after 12 deg the hand came 4 cm nearer to the hinge" in n
                                                     for n in p.notes), p
        assert "the hand slid off" in str(p), p
        d = call("push_arc", (tuple(hinge), 30), {"dry_run": True})
        assert d.dry_run and d.reachable and not d.moved and str(d).startswith("Push (dry run): reachable"), d
        for args, kwargs in (((tuple(hinge), 30), {"press": 0.2}), ((tuple(hinge), 0), {}),
                             ((tuple(host._tcp()[:3, 3]), 30), {})):  # the last: the hand on the hinge line
            with _failed("ValueError"):
                call("push_arc", args, kwargs)
        session.kitchen.stretched_now = 12.0
        with _failed("NotExecuted") as f:
            call("push_arc", (tuple(hinge), 30), {})
        assert "Push: NOT executed" in f.exc.text and "stretched almost straight now" in f.exc.text, f.exc.text
        session.kitchen.stretched_now = None
        call("home", (), {})

        with _failed("ValueError"):
            call("wait", (0,), {})
        w = call("wait", (5,), {})
        assert w.steps == 5 and "0.25 s" in str(w)
        with _failed("RobotError"):
            call("fly", (), {})
        with _failed("ValueError"):
            call("image", ("head",), {})
        with _failed("TypeError"):
            call("wait", (), {})
        assert call("image", ("left",), {}).shape == (256, 256, 3)
        assert call("depth", ("wrist",), {}).dtype == np.float32
        assert call("camera", ("right",), {}).width == 256
        pic = call("look", ("all",), {"tile": True})
        assert pic.cams == ["left", "right", "wrist"] and pictures[-1].endswith("(side by side)")
        call("look", ("left",), {"grid": True, "crop": (60, 60, 160, 160), "zoom": 2,
                                 "marks": [("world", tuple(F.BOX_START), "box")]})
        assert "grid" in pictures[-1] and "crop" in pictures[-1]

        t = call("measure", ("red box",), {})
        assert t.found and np.allclose(t.center[:2], F.BOX_START[:2], atol=0.01), t
        px = call("to_pixel", (t, "left"), {})
        masks = call("find", ("left", "red box"), {})
        assert abs(px[0] - masks[0].centroid[0]) < 6 and abs(px[1] - masks[0].centroid[1]) < 6, (px, masks)
        s = call("to_world", ("left", masks[0].centroid), {})
        half = np.asarray(F.BOX_SIZE) / 2 + 0.005  # a point of the box's visible surface
        assert np.all(np.abs(np.asarray(s.xyz) - np.asarray(F.BOX_START)) <= half), s
        call("look", (["left", "right"],), {"masks": [t, masks[0]]})
        # a Thing's .masks (by camera), and two of them added, are taken too
        call("look", (["left", "right"],), {"masks": t.masks})
        call("look", ("left",), {"masks": t.masks + masks})
        with _failed("ValueError"):
            call("look", ("left",), {"masks": ["red box"]})
        assert len(call("things", (), {})) == 1

        # the pick: above it, measure again, down, close, up -> the environment reports success
        m = call("goto", ((t.center[0], t.center[1], t.top + 0.08),), {"pitch": 90, "yaw": t.grip_yaw, "grip": "open"})
        assert m.ok, m
        t = call("measure", (t,), {})
        assert "wrist" in t.views, t
        call("goto", ((t.center[0], t.center[1], t.top - 0.03),), {})
        g = call("gripper", ("close",), {})
        assert g.holding and abs(g.width_m - F.BOX_SIZE[1]) < 1e-6, g
        st = call("state", (), {})
        assert st.holding and "holding" in str(st)
        m = call("goto", (), {"by": (0, 0, 0.002)})
        assert not any("changed" in n for n in m.notes), m
        session.kitchen.width += 0.006  # the box turned in the fingers
        m = call("goto", (), {"by": (0, 0, 0.002)})
        assert any("changed from 5.0 to 5.6 cm" in n for n in m.notes), m
        m = call("goto", (), {"by": (0, 0, 0.002)})
        assert not any("changed" in n for n in m.notes), m  # said once
        real_drive = session.kitchen.op_drive

        def pushed(target):
            reply = real_drive(target)
            reply["result"].update(blocked=True, reached=False, off=[-0.04, 0.02, 3.0])
            return reply

        session.kitchen.op_drive = pushed
        d = call("drive", (-0.05,), {})
        assert d.blocked and any("held off its path and stopped (4 cm behind and 2 cm to the left, turned 3 "
                                 "degrees left of" in n for n in d.notes), d
        session.kitchen.op_drive = real_drive
        with raises(Stop, "SUCCESS"):
            call("goto", (), {"by": (0, 0, 0.15)})
        with raises(Stop, "SUCCESS"):
            call("state", (), {})
        assert session.over and session.verdict["success"] and session.verdict["step"] == session.step
        assert "the environment reports SUCCESS" in session.over_text()
    finally:
        session.close()
        rec.close()
    states = (rec.dir / "states.jsonl").read_text().splitlines()
    assert len(states) == session.kitchen.steps, (len(states), session.kitchen.steps)
    first = json.loads(states[0])
    assert first["step"] == 1 and len(first["tcp"]) == 3 and len(first["base"]) == 3
    events = [json.loads(line)["kind"] for line in (rec.dir / "events.jsonl").read_text().splitlines()]
    assert "sim_open" in events and "goto" in events and "measure" in events and "sam3" in events
    assert "push_arc" in events
    assert session.kitchen.closed
    shutil.rmtree(out)


def test_casa_host_measures_a_carried_thing_where_the_hand_set_it():
    import fakes_casa as F

    session, rec, pictures, out = _open(horizon=900)
    call, scene = session.host.dispatch, session.host.scene
    try:
        t = call("measure", ("red box",), {})
        call("goto", ((t.center[0], t.center[1], t.top + 0.08),), {"pitch": 90, "yaw": t.grip_yaw, "grip": "open"})
        call("goto", ((t.center[0], t.center[1], t.top - 0.03),), {})
        assert call("gripper", ("close",), {}).holding and list(scene.carried) == [t.id]
        call("goto", (), {"by": (0, 0.25, 0)})
        call("gripper", ("open",), {})
        assert not scene.carried
        call("goto", (), {"by": (0, 0, 0.1)})
        # another red box where the first one stood: measuring the first again finds it where it was set
        lo, hi = F.BOX_START - F.BOX_SIZE / 2, F.BOX_START + F.BOX_SIZE / 2
        session.kitchen._dish_boxes = lambda: [(lo, hi, F.BOX)]
        again = call("measure", (t,), {})
        assert again.id == t.id and np.allclose(again.center[:2], F.BOX_START[:2] + (0, 0.25), atol=0.015), again
        assert again.notes[0].startswith("moved 2") and "other 'red box' seen" in " ".join(again.notes), again
        # set down again by a motion that opens the fingers as it starts: it stays where it was let go
        call("goto", ((again.center[0], again.center[1], again.top + 0.08),), {"grip": "open"})
        call("goto", ((again.center[0], again.center[1], again.top - 0.03),), {})
        assert call("gripper", ("close",), {}).holding and list(scene.carried) == [t.id]
        call("goto", (), {"by": (0, -0.12, 0)})
        call("goto", (), {"by": (0, 0, 0.1), "grip": "open"})
        assert not scene.carried
        third = call("measure", (t,), {})
        assert np.allclose(third.center[:2], F.BOX_START[:2] + (0, 0.13), atol=0.015), third
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_stops_the_cell_when_the_load_slips_out():
    session, rec, pictures, out = _open()
    host = session.host
    try:
        host.holding, host.held_width = True, 0.05
        session.kitchen.width = 0.0  # the fingers close on nothing during the next move: it slipped out
        with _failed("RobotError") as f:
            host.dispatch("goto", (), {"by": (0, 0, 0.02)})
        assert f.exc.text.startswith("Move:") and "slipped out" in f.exc.text, f.exc.text
        assert f.exc.text.endswith("so the cell stops here") and host.holding is False, f.exc.text
        assert host.dispatch("goto", (), {"by": (0, 0, -0.02)}).ok  # nothing held: nothing to stop for
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_stops_a_drive_once_when_the_grip_slips():
    session, rec, pictures, out = _open()
    host, kitchen = session.host, session.kitchen
    try:
        host.grip_goal, host.holding, host.held_width, host.grasp_width = 0.0, True, 0.05, 0.05
        kitchen.width = 0.045  # it settles 5 mm narrower on the first move: what the stop measures from
        assert host.dispatch("drive", (0.05,), {}).reached
        assert "closed, holding (4.5 cm; 5.0 cm when it closed)" in str(host.dispatch("state", (), {}))
        kitchen.width = 0.042  # 3 mm narrower than after the first move: it holds
        host.dispatch("wait", (1,), {})
        assert not host.dispatch("home", (), {}).blocked
        kitchen.width = 0.040  # 5 mm: slipping out (the gap the last call ended with counts)
        host.dispatch("wait", (1,), {})
        step = session.step
        with _failed("RobotError") as f:
            host.dispatch("home", (), {})
        assert f.exc.text.startswith("robot.home() did not start") and \
            "4.0 cm, 4.5 cm after the first move with it" in f.exc.text and session.step == step, f.exc.text
        assert host.dispatch("drive", (0.05,), {}).reached and session.step > step  # once per grasp
        host.dispatch("gripper", (1.0,), {})
        host.holding, host.held_width, host.grasp_width = True, 0.05, 0.05  # the next grasp
        kitchen.width = 0.045
        host.dispatch("wait", (1,), {})
        assert host.dispatch("goto", (), {"by": (0, 0, 0.02), "dry_run": True}).dry_run  # neither moves it
        assert host.dispatch("drive", (0.05,), {}).reached  # the first move
        kitchen.width = 0.040
        host.dispatch("wait", (1,), {})
        with _failed("RobotError") as f:
            host.dispatch("drive", (0.05,), {})
        assert f.exc.text.startswith("robot.drive() did not start"), f.exc.text
        kitchen.width = 0.03
        host.holding, host.grasp_width = False, None  # nothing held: nothing to stop for
        assert host.dispatch("drive", (-0.05,), {}).reached
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_notes_a_close_that_ends_nearly_open():
    session, rec, pictures, out = _open()
    kitchen, plain = session.kitchen, session.kitchen._set_grip

    def resting(opening):  # the fingers rest on something and hardly close
        plain(opening)
        if opening <= 0.05:
            kitchen.width = 0.078

    try:
        kitchen._set_grip = resting
        g = session.host.dispatch("gripper", ("close",), {})
        assert g.holding and any("nearly fully open" in n for n in g.notes), g
        kitchen._set_grip = plain
        g = session.host.dispatch("gripper", ("close",), {})
        assert not any("nearly fully open" in n for n in g.notes), g
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_notes_a_close_at_the_top_of_a_low_thing():
    session, rec, pictures, out = _open()
    kitchen, plain, host = session.kitchen, session.kitchen._set_grip, session.host

    def on_something(opening):  # something 3 cm wide between the fingers
        plain(opening)
        kitchen.width = max(kitchen.width, 0.03)

    try:
        kitchen._set_grip = on_something
        x, y, z = host._tcp()[:3, 3]

        def close_on(top, high, dx=0.01, opening="close", wide=0.06, on="table"):
            steak = RR.Thing(id=7, label="steak", found=True, center_xyz=(x + dx, y, top - high / 2), top=top,
                             size=(0.15, wide, high), on=on, on_z=top - high)
            host.scene.things = lambda: [steak]
            g = host.dispatch("gripper", (opening,), {})
            assert g.holding, g
            return [n for n in g.notes if "the grasp point is" in n]

        said = close_on(z + 0.002, 0.02)
        assert len(said) == 1 and "0.2 cm under the top of Thing#7 'steak' as last measured (2.0 cm high)" in said[0] \
            and f"down to the height it stands on (z {z + 0.002 - 0.02:.3f};" in said[0], said
        assert "0.3 cm over the top" in close_on(z - 0.003, 0.02)[0]
        assert f"about half its height (z {z + 0.002 - 0.025:.3f})" in close_on(z + 0.002, 0.05)[0]
        assert not close_on(z + 0.015, 0.02)  # 1.5 cm under its top
        assert not close_on(z + 0.002, 0.08)  # a tall thing
        assert not close_on(z + 0.002, 0.02, dx=0.08)  # not under the hand
        assert not close_on(z + 0.002, 0.02, opening=0.3)  # a partial close
        assert not close_on(z + 0.002, 0.02, wide=0.11)  # wider than the open fingers: they hold a part (a knob)
        assert not close_on(z + 0.002, 0.02, on="held by left")  # last measured in the hand
        assert not close_on(z + 0.002, 0.008)  # a flat thing gripped at the surface: nothing lower to go to
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_notes_a_descent_that_stops_on_a_thing():
    session, rec, pictures, out = _open()
    kitchen, host = session.kitchen, session.host
    real_goto = kitchen.op_goto

    def stopping_at(z_stop):  # the hand stops at z_stop on the way down, as on something under it
        def goto(pose, grip=None, dry_run=False, check=True, speed=None):
            goal = np.asarray(pose, float)
            stop = goal.copy()
            stop[2, 3] = max(goal[2, 3], z_stop)
            reply = real_goto(stop, grip=grip, dry_run=dry_run, check=check, speed=speed)
            if not dry_run and stop[2, 3] > goal[2, 3]:
                reply["result"].update(reached=False, stalled=True)
            return reply
        return goto

    try:
        assert host.dispatch("goto", ((0.6, 0.0, 1.10),), {"pitch": 90, "grip": "open"}).ok
        x, y, _ = host._tcp()[:3, 3]

        def descend(top, high, dx=0.01, wide=0.03, stop=1.04, to=1.01, pitch=90):
            handle = RR.Thing(id=7, label="kettle handle", found=True, center_xyz=(x + dx, y, top - high / 2),
                              top=top, size=(0.12, wide, high), on="table", on_z=top - high)
            host.scene.things = lambda: [handle]
            kitchen.op_goto = stopping_at(stop)
            m = host.dispatch("goto", ((x, y, to),), {"pitch": pitch})
            kitchen.op_goto = real_goto
            host.dispatch("goto", ((x, y, 1.10),), {"pitch": 90})
            return [n for n in m.notes if "the fingers rest on it" in n]

        said = descend(1.043, 0.025)
        assert len(said) == 1 and "stopped 3.0 cm above the target with the grasp point 0.3 cm under the top of " \
            "Thing#7 'kettle handle' as last measured, 2.2 cm over what it stands on" in said[0], said
        assert "they held about 1 time in 5" in said[0]
        assert "0.5 cm over the top" in descend(1.035, 0.025)[0]
        # lower down on a taller thing: 3 cm under its top, 3 cm over its surface
        assert "they held about 1 time in 3" in descend(1.07, 0.06)[0]
        assert not descend(1.045, 0.012)  # at the surface it stands on (0.7 cm over it)
        assert not descend(1.043, 0.025, dx=0.08)  # not under the hand
        assert not descend(1.043, 0.025, wide=0.11)  # wider than the open fingers
        assert not descend(1.043, 0.025, stop=1.0)  # got there
        assert not descend(1.043, 0.025, stop=1.04, to=1.038)  # stopped 2 mm short: as good as there
        assert not descend(1.043, 0.025, pitch=30)  # not pointing down
        host.dispatch("gripper", ("close",), {})
        assert not descend(1.043, 0.025)  # the fingers closed
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_tells_a_held_back_opening_from_a_partial_hold():
    session, rec, pictures, out = _open()
    kitchen, plain = session.kitchen, session.kitchen._set_grip

    def walled(opening):  # a rim around the fingers keeps them from opening fully
        plain(opening)
        if opening > 0.05:
            kitchen.width = min(kitchen.width, 0.071)

    def on_something(opening):  # something 5 cm wide between the fingers
        plain(opening)
        kitchen.width = max(kitchen.width, 0.05)

    try:
        g = session.host.dispatch("gripper", ("close",), {})
        kitchen._set_grip = walled
        g = session.host.dispatch("gripper", ("open",), {})
        assert g.holding is False and any("opened only to 7.1 cm of the 8.0" in n for n in g.notes), g
        assert not any("between them:" in n for n in g.notes), g
        kitchen._set_grip = plain
        m = session.host.dispatch("goto", (), {"by": (0.0, 0.0, 0.05)})
        assert not any("turned or slid" in n for n in m.notes), m  # the fingers came free: no hold to watch
        g = session.host.dispatch("gripper", (0.45,), {})  # set as 0.5 (steps of 0.1): 4.0 cm, no note
        assert not g.notes and not g.holding, g
        g = session.host.dispatch("gripper", ("open",), {})
        assert not g.notes, g
        kitchen._set_grip = on_something
        g = session.host.dispatch("gripper", (0.3,), {})
        assert g.holding and any("stopped at 5.0 cm, short of the 2.4 cm asked: something is between" in n
                                 for n in g.notes), g
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_eases_a_light_close_to_a_light_hold():
    session, rec, pictures, out = _open()
    host, kitchen, plain = session.host, session.kitchen, session.kitchen._set_grip
    across = [0.05]

    def on_something(opening):  # something `across` wide between the fingers
        plain(opening)
        kitchen.width = max(kitchen.width, across[0])

    try:
        kitchen._set_grip = on_something
        g = host.dispatch("gripper", ("light",), {})  # 5 cm (0.62): set at the 0.1 step 0.05 under it
        assert g.holding and g.goal == 0.5 and "closed on something ~5.0 cm wide (holding)" in str(g), g
        assert any("eased to a light hold: the fingers are set at 4.0 cm" in n for n in g.notes), g
        assert "closed, holding (5.0 cm)" in str(host.dispatch("state", (), {}))
        kitchen.width = 0.043  # 3 mm over where they are set: it holds
        assert host.dispatch("goto", (), {"by": (0, 0, 0.02)}).ok and host.holding
        kitchen.width = 0.041  # in to where they are set: it slipped out
        with _failed("RobotError") as f:
            host.dispatch("goto", (), {"by": (0, 0, 0.02)})
        assert "closed in to where they were set: what they held slipped out" in f.exc.text, f.exc.text
        assert host.holding is False and host.hold_gap is None
        host.dispatch("gripper", ("open",), {})
        across[0] = 0.01  # too thin to set under it in steps of 0.1: the full squeeze
        g = host.dispatch("gripper", ("light",), {})
        assert g.holding and g.goal == 0.0 and any("too thin to ease" in n for n in g.notes), g
        host.dispatch("gripper", ("open",), {})
        across[0] = 0.05
        g = host.dispatch("gripper", ("close",), {})  # a plain close keeps squeezing
        assert g.holding and g.goal == 0.0 and not any("eased" in n for n in g.notes), g
        with _failed("ValueError"):
            host.dispatch("goto", (), {"by": (0, 0, 0.02), "grip": "light"})  # only robot.gripper eases
    finally:
        session.close()
        rec.close()
    events = [json.loads(line) for line in (rec.dir / "events.jsonl").read_text().splitlines()]
    grips = [e for e in events if e["kind"] == "gripper"]
    assert grips[0]["light"] and grips[0]["eased"] == 0.5 and grips[0]["opening"] == 0.0, grips[0]
    assert "eased" not in grips[2] and grips[2]["light"], grips[2]
    shutil.rmtree(out)


def test_limit_note():
    from hexashell.benches.armhost import limit_note

    assert limit_note([]) is None and limit_note(None) is None
    assert "elbow) fully bent" in limit_note([[4, "low"]]) and "move the base back" in limit_note([[4, "low"]])
    note = limit_note([[6, "high"], [7, "low"]])
    assert "joint 6 (the wrist) at the end of its travel and joint 7" in note and "other angles" in note, note


def test_bend_words_and_zoom_cap():
    from hexashell.benches.armserver import bend_words
    from hexashell.errors import Refused
    from hexashell.vision.overlay import View

    assert bend_words(math.radians(24.0)) == "the elbow 24 deg short of straight"
    assert bend_words(math.radians(-23.0)) == "the elbow bent 23 deg backwards past straight"  # not "-23 short"
    assert bend_words(0.0) == "the elbow straight"
    image = np.zeros((480, 640, 3), np.uint8)
    assert View(image, (265, 235, 288, 262), 10).zoom == 8.0  # capped, not refused
    with raises(Refused, "zoom"):
        View(image, None, float("nan"))


def test_casa_host_rim_pick():
    import fakes_casa as F

    session, rec, _, out = _open(sam3=False, horizon=1500, dish=True)
    call = session.host.dispatch
    kitchen = session.kitchen
    middle = (0.75, 0.25, F.COUNTER_TOP + F.DISH_H)
    try:
        with _failed("ValueError") as f:
            call("rim_pick", ((0.72, -0.05, 0.98),), {})  # the red box
        assert re.search(r"rim_pick: what stands there is [5-9] cm across: not a dish", f.exc.text), f.exc.text
        with _failed("ValueError") as f:
            call("rim_pick", (middle,), {"pitch": 80})
        assert "pitch is 30 to 60" in f.exc.text, f.exc.text
        # a dry run: the rim from the depth points, nothing moves
        d = call("rim_pick", (middle,), {"dry_run": True})
        assert d.dry_run and not d.holding and session.step == 0, d
        assert np.allclose(d.center, (0.75, 0.25), atol=0.005) and abs(d.top - middle[2]) < 0.003, d
        # the fake dish is square: the circle of its outline falls between its half side and half diagonal
        assert F.DISH_HALF - 0.006 < d.radius < 1.15 * F.DISH_HALF and abs(d.on_z - F.COUNTER_TOP) < 0.003, d
        assert d.angles[::2] == (45.0, 90.0) and abs(d.angles[1] - 18.4) < 0.5, d  # yaw: toward the middle
        assert str(d).startswith("RimPick (dry run): reachable"), d
        # the near wall's outer face is at x 0.63: the pinch ~1.2 cm in along the way to the middle (seen at a slant,
        # the square's outermost points in the strip lie a little outside the face), 1.5 cm down
        assert 0.628 < d.pinch[0] < 0.645, d
        assert abs(d.pinch[2] - (middle[2] - 0.015)) < 0.003, d
        o = call("rim_pick", (middle,), {"dry_run": True, "outer": True})  # one dish: its outermost rim is its rim
        assert np.allclose(o.pinch, d.pinch, atol=0.003) and session.step == 0, o
        # the first close comes out empty: once more 0.7 cm nearer the edge, then it lifts
        kitchen.dish_empty = 1
        r = call("rim_pick", (middle,), {})
        assert r.ok and r.tries == 2 and r.width_m == F.DISH_PINCH and r.steps == session.step, r
        assert abs(kitchen.dish[2] - (F.COUNTER_TOP + 0.10)) < 0.003 and abs(r.at[2] - (r.pinch[2] + 0.10)) < 0.003, r
        assert any("the first pinch, 1.2 cm in from the rim's edge, closed on nothing; this one is 0.5 cm in" in n
                   for n in r.notes), r
        assert str(r).startswith("RimPick: holding it by the near rim (closed on ~2.0 cm) and lifted"), r
        assert call("state", (), {}).holding
        # a dish held by its rim settles on in the fingers: a drive still starts 6 mm narrower, not 13 mm
        kitchen.width = F.DISH_PINCH - 0.006
        call("wait", (1,), {})  # read the new gap
        call("drive", (), {"forward": -0.02})
        kitchen.width = F.DISH_PINCH - 0.013
        call("wait", (1,), {})
        with _failed("RobotError") as f:
            call("drive", (), {"forward": 0.02})
        assert "did not start: the gap between the fingers is 0.7 cm, 2.0 cm after" in f.exc.text, f.exc.text
        call("drive", (), {"forward": 0.02})  # once per grasp
        kitchen.width = F.DISH_PINCH
        with _failed("ValueError") as f:
            call("rim_pick", (middle,), {})
        assert "set it down and open first" in f.exc.text, f.exc.text
        call("gripper", ("open",), {})
        assert abs(kitchen.dish[2] - F.COUNTER_TOP) < 1e-9
        call("home", (), {})
        # both closes empty: not holding, the hand back over the dish and open
        kitchen.dish_empty = 2
        r = call("rim_pick", (middle,), {})
        assert not r.ok and r.tries == 2 and "nothing between them, at 0.5 cm in" in r.why, r
        assert "(and 1.2 cm in before)" in r.why and not call("state", (), {}).holding, r
        call("home", (), {})
        # the dish off to the side: turn first; out of reach: how to drive
        call("drive", (), {"turn": 70})
        with _failed("ValueError") as f:
            call("rim_pick", (middle,), {})
        assert "of the base's heading: turn the base toward it first (robot.drive(turn=-52))" in f.exc.text, f.exc.text
        call("drive", (), {"turn": -70})
        call("drive", (-0.5,), {})
        d = call("rim_pick", (middle,), {"dry_run": True})
        assert d.dry_run and not d.executed and "NOT reachable" in str(d), d
        assert any("robot.drive(forward=0.6" in n and "puts it 0.6 m straight ahead" in n for n in d.notes), d
        with _failed("NotExecuted") as f:
            call("rim_pick", (middle,), {})
        assert "RimPick: NOT executed" in f.exc.text, f.exc.text
        events = [e["kind"] for e in map(json.loads, (rec.dir / "events.jsonl").read_text().splitlines())]
        assert events.count("rim_pick") == 6, events  # the six that got as far as the rim
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_host_budget_failure_and_no_sam3():
    import fakes_casa as F

    session, rec, _, out = _open(sam3=False, horizon=30)
    call = session.host.dispatch
    try:
        with _failed("ValueError") as f:
            call("measure", ("red box",), {})
        assert "SAM3" in f.exc.text
        assert call("to_world", ("left", (128, 200)), {}).xyz is not None
        with raises(Stop, "step budget is used up (30 of 30)"):
            call("wait", (100,), {})
        assert session.verdict == {"success": False, "step": 30, "step_limit": 30, "finished": False}
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)

    session, rec, _, out = _open(sam3=False)
    call = session.host.dispatch
    try:
        def broken(**args):
            raise RuntimeError("mujoco exploded")

        session.kitchen.op_wait = broken
        with raises(Stop, "the simulator failed"):
            call("wait", (3,), {})
        assert session.failure and "mujoco exploded" in session.failure and session.frames() == {}
        with raises(Stop):
            call("state", (), {})
        assert session.verdict["failure"] == session.failure
        assert session.errors and session.errors[0].startswith("simulator:")
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_casa_card():
    session, rec, _, out = _open(sam3=False)
    try:
        text, used = session.card(Settings(skills=True))
        assert "Instruction (from the benchmark; it is authoritative): 'pick up the red box from the counter'" in text
        assert "Step budget: 600 steps (30 s of simulated time)" in text
        assert "pitch 45 yaw 0 roll 0, gripper opening 1.00" in text and "x forward, y to the left" in text
        # the task's notes (skills/robocasa/<task>.md) follow the card when skills are on
        assert "Pictures 1-3 below: left (" in text and used == {"task": "PickPlaceCounterToCabinet", "skill": True}
        assert "\n## Notes for this task\n\n# PickPlaceCounterToCabinet\n" in text
        plain, used = session.card(Settings(skills=False))
        assert "Notes for this task" not in plain and not used["skill"] and plain.strip() in text
        assert [c for c, _ in session.start_pictures()] == ["left", "right", "wrist"]
        assert session.prompt_folder() == "robocasa" and session.record_name() == "PickPlaceCounterToCabinet_s0"
        fields = session.result_fields({})
        assert fields["split"] == "pretrain50" and fields["seed_set"] == "learn" and fields["layout_id"] == 7
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_run_settings():
    from hexashell.benches import run

    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    try:
        cfg = str(ROOT / "config" / "benches.yml")
        s = run.settings_for("robocasa", "CloseFridge", cfg, [])
        assert s.max_turns == 40 and s.budget_s == 7200.0 and not s.ladder and s.stream
        assert s.sam3_url == "http://127.0.0.1:18300"
        s = run.settings_for("robocasa", "ArrangeTea", cfg, ["max_turns=7", "skills=yes", "budget_s=60"])
        assert s.max_turns == 7 and s.skills is True and s.budget_s == 60.0
        os.environ["HEXASHELL_MAX_TURNS"] = "9"
        assert run.settings_for("robocasa", "ArrangeTea", None, []).max_turns == 9
        with raises(SystemExit):
            run.settings_for("robocasa", "ArrangeTea", cfg, ["no_such=1"])
        os.environ.pop("HEXASHELL_MAX_TURNS")
        s = run.settings_for("libero", "libero_goal_swap_t3", cfg, [])
        assert s.max_turns == 40 and s.budget_s == 7200.0 and not s.ladder
        with raises(ValueError):
            run.settings_for("libero", "x", cfg, [])
        s = run.settings_for("robotwin", "adjust_bottle", cfg, [])
        assert s.max_turns == 40 and s.budget_s == 7200.0 and not s.ladder
        with raises(ValueError):
            run.settings_for("robotwin", "x", cfg, [])
        twin = run.make_session("robotwin", "adjust_bottle", 7, python="/x/python", env_file=None, egl="3")
        assert twin.bench == "robotwin" and twin.server_timeout_s == 1800.0 and twin.egl_device == "3"
        # the split goes to the server's open; pretrain50 unless asked, and only RoboCasa has one
        casa = run.make_session("robocasa", "CloseFridge", 1000, python="/x/python", env_file=None, egl="3",
                                split="pretrain")
        assert casa.bench == "robocasa" and casa.egl_device == "3" and casa.split == "pretrain"
        assert casa.open_args() == {"task": "CloseFridge", "seed": 1000, "split": "pretrain"}
        plain = run.make_session("robocasa", "CloseFridge", 0, python="/x/python", env_file=None, egl=None)
        assert plain.split == "pretrain50" and plain.open_args()["split"] == "pretrain50"
        with raises(ValueError):
            run.make_session("robocasa", "CloseFridge", 0, python="/x/python", env_file=None, egl=None, split="target")
        with raises(ValueError):
            run.make_session("libero", "libero_goal_swap_t3", 0, python="/x/python", env_file=None, egl=None,
                             split="pretrain")
        with raises(ValueError):
            run.settings_for("calvin", "x", cfg, [])
    finally:
        os.environ.clear()
        os.environ.update(saved)


def test_scripted_casa_episode():
    import fakes
    import fakes_casa as F
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-robocasa-run-"))
    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nrobot.look('all', tile=True)\n"
                           "t = robot.measure('red box')\nprint(t)")],
        [fakes.python_call("m = robot.goto((t.center[0], t.center[1], t.top + 0.08), pitch=90, yaw=t.grip_yaw, "
                           "grip='open')\nprint(m)\nt = robot.measure(t)\nprint(t)\n"
                           "print(robot.goto((t.center[0], t.center[1], t.top - 0.03)))\n"
                           "g = robot.gripper('close')\nprint(g)\nassert g.holding")],
        [fakes.python_call("try:\n    robot.goto((3, 3, 3))\nexcept NotExecuted as e:\n    print('caught', e)\n"
                           "print(robot.drive(turn=0.0))\nprint(robot.goto(by=(0, 0, 0.15)))\nprint('not reached')")],
    ]
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        session = F.FakeCasaSession(kitchen=F.FakeKitchen(size=256))
        settings = Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30)
        ep = episode.Episode(session, settings, out)
        ep.run()
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    folder = ep.rec.dir
    result = json.loads((folder / "result.json").read_text())
    assert result["bench"] == "robocasa" and result["success"] is True, result
    assert result["end"] == "episode_over" and result["turns"] == 3 and result["split"] == "pretrain50", result
    assert result["robot_calls"]["goto"] == 4 and result["robot_calls"]["measure"] == 2, result["robot_calls"]
    assert not result["errors"], result["errors"]
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert len(cells) == 3 and not cells[1]["error"], cells[1]
    assert "Thing#1 'red box'" in cells[0]["output"] and "Grip: closed on something" in cells[1]["output"]
    assert "caught" in cells[2]["output"] and "not reached" not in cells[2]["output"] and cells[2]["stopped"]
    messages = fakes.ScriptedGateway.seen[-1]
    system = messages[0]["content"]
    assert system.startswith("# 1. Your job and the rules") and "RoboCasa" in system
    card = messages[1]["content"]
    card_text = card if isinstance(card, str) else " ".join(p.get("text", "") for p in card if isinstance(p, dict))
    assert "Task: PickPlaceCounterToCabinet (RoboCasa)" in card_text and "You have up to 10 replies" in card_text
    assert "the environment reports SUCCESS" in session.over_text()  # the model is told this after the cell
    assert len(list((folder / "frames").iterdir())) >= 2
    shutil.rmtree(out)



# -- LIBERO-PRO: the protocol, the host against the fake table, a scripted episode -----------------
def test_libero_protocol():
    from hexashell.benches.libero import protocol as LP

    tasks = LP.test_tasks()
    assert len(tasks) == 156 and len(set(tasks)) == 156 and tasks[0] == "libero_spatial_swap_t0"
    assert tasks[39] == "libero_10_swap_t9" and tasks[-1] == "libero_10_task_t9"
    assert "libero_spatial_task_t3" not in tasks and "libero_10_object_t5" in tasks
    assert list(LP.LEARN_SEEDS) == list(range(10)) and list(LP.VALIDATION_SEEDS) == list(range(10, 20))
    assert list(LP.TEST_SEEDS) == list(range(20, 30))
    assert LP.official_steps("libero_10_swap_t9") == 520 and LP.official_steps("libero_object_task_t1") == 280
    assert LP.official_steps("libero_spatial_lan_t0") == 220 and LP.official_steps("libero_goal_t0") == 300
    with raises(ValueError):
        LP.official_steps("libero_90_t0")
    assert LP.parse_task("libero_10_swap_t9") == ("libero_10_swap", 9)
    assert LP.parse_task(LP.task_name("libero_goal_swap", 3)) == ("libero_goal_swap", 3)
    for bad in ("libero_goal_swap", "goal_swap_t1", "libero_goal_swap_t-1", "LIBERO_10_t1"):
        with raises(ValueError):
            LP.parse_task(bad)
    assert LP.init_index(0) == 0 and LP.init_index(52) == 2
    assert LP.SETTLE_STEPS == 10 and LP.REPLIES == 40 and LP.EPISODE_S == 7200.0


def _open_libero(sam3=True, horizon=600):
    import fakes_libero as L

    out = Path(tempfile.mkdtemp(prefix="hexashell-libero-"))
    session = L.FakeLiberoSession(sam3=sam3, table=L.FakeTable(horizon=horizon, size=256))
    rec = Records(out, session.info, name=session.record_name())
    pictures = []
    session.open(Settings(sam3_url="fake" if sam3 else ""), rec, lambda kind, image, caption: pictures.append(caption))
    return session, rec, pictures, out


def test_libero_host_moves():
    import fakes_libero as L

    session, rec, pictures, out = _open_libero()
    host = session.host
    call = host.dispatch
    try:
        st = call("state", (), {})
        assert st.gripper == 1.0 and st.home_mm == 0 and st.angles == (86.8, 0.0, 0.0) and st.step == 0, st
        assert "base" not in str(st) and host.asked == (86.8, 0.0, 0.0), (st, host.asked)
        info = call("task", (), {})
        assert info.instruction == "pick up the red box" and info.step_limit == 600
        with _failed("NotExecuted") as f:
            call("goto", ((2, 2, 2),), {})
        assert "base does not move" in f.exc.text, f.exc.text
        with _failed("RobotError"):
            call("drive", (0.1,), {})
        m = call("goto", ((0.5, 0.1, 0.2),), {"pitch": 90})  # turns 3 degrees, not half round
        assert m.ok and np.allclose(m.at, (0.5, 0.1, 0.2), atol=0.002) and m.angles == (90.0, 0.0, 0.0), m
        assert m.steps < 25, m  # a half turn would add 60 steps
        m = call("goto", ((0.5, 0.1, -0.03),), {})
        assert m.blocked and not m.ok, m
        h = call("home", (), {})
        assert h.ok, h
        assert call("image", ("agentview",), {}).shape == (256, 256, 3)
        assert call("image", ("front",), {}).shape == (256, 256, 3)  # an alias
        with _failed("ValueError"):
            call("image", ("left",), {})
        pic = call("look", ("all",), {"tile": True})
        assert pic.cams == ["agentview", "wrist"], pic.cams

        t = call("measure", ("red box",), {})
        assert t.found and np.allclose(t.center[:2], L.BOX_START[:2], atol=0.015), t
        assert abs(t.top - L.BOX_SIZE[2]) < 0.01, t
        m = call("goto", ((t.center[0], t.center[1], t.top + 0.08),), {"pitch": 90, "yaw": t.grip_yaw, "grip": "open"})
        assert m.ok, m
        t = call("measure", (t,), {})
        assert "wrist" in t.views, t
        call("goto", ((t.center[0], t.center[1], t.top - 0.03),), {})
        g = call("gripper", ("close",), {})
        assert g.holding and abs(g.width_m - L.BOX_SIZE[1]) < 1e-6, g
        with raises(Stop, "SUCCESS"):
            call("goto", (), {"by": (0, 0, 0.15)})
        assert session.over and session.verdict["success"] and session.verdict["step"] == session.step
    finally:
        session.close()
        rec.close()
    states = (rec.dir / "states.jsonl").read_text().splitlines()
    assert len(states) == session.kitchen.steps, (len(states), session.kitchen.steps)
    first = json.loads(states[0])
    assert first["step"] == 1 and len(first["tcp"]) == 3 and "base" not in first, first
    assert session.kitchen.closed and "drive" not in session.kitchen.ops
    shutil.rmtree(out)


def test_libero_card():
    session, rec, _, out = _open_libero(sam3=False)
    try:
        text, used = session.card(Settings(skills=True))
        assert "Instruction (from the benchmark; it is authoritative): 'pick up the red box'" in text
        assert "Task: libero_object_swap_t0 (LIBERO-PRO, task 0 of libero_object_swap)" in text
        assert "Step budget: 600 steps (30 s of simulated time)" in text
        assert "pitch 87 yaw 0 roll 0, gripper opening 1.00" in text and "the robot does not move" in text
        assert "Pictures 1-2 below: agentview (" in text and used == {"task": "libero_object_swap_t0", "skill": True}
        assert "\n## Notes for this task\n\n# libero_object_swap_t0\n" in text  # skills/libero/libero_object_swap_t0.md
        text, used = session.card(Settings(skills=False))
        assert "## Notes for this task" not in text and used["skill"] is False
        assert [c for c, _ in session.start_pictures()] == ["agentview", "wrist"]
        assert session.prompt_folder() == "libero" and session.record_name() == "libero_object_swap_t0_s0"
        fields = session.result_fields({})
        assert fields["suite"] == "libero_object_swap" and fields["task_index"] == 0 and fields["init_state"] == 0
        assert session.labels() == {"task": "libero_object_swap_t0", "seed": 0, "suite": "libero_object_swap"}
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_libero_prompt_folder():
    from hexashell.runtime import card

    text = card.system_prompt("libero")
    heads = [line for line in text.splitlines() if line.startswith("# ")]
    assert [h.split(".")[0] for h in heads] == ["# 1", "# 2", "# 3", "# 4", "# 5", "# 6"], heads
    assert "LIBERO" in text and "robot.drive" not in text and "'agentview' and 'wrist'" in text
    assert "left of the agentview picture" in text  # how the instructions say left and right


def test_scripted_libero_episode():
    import fakes
    import fakes_libero as L
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-libero-run-"))
    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nrobot.look('all', tile=True)\n"
                           "t = robot.measure('red box')\nprint(t)")],
        [fakes.python_call("m = robot.goto((t.center[0], t.center[1], t.top + 0.08), pitch=90, yaw=t.grip_yaw, "
                           "grip='open')\nprint(m)\nt = robot.measure(t)\nprint(t)\n"
                           "print(robot.goto((t.center[0], t.center[1], t.top - 0.03)))\n"
                           "g = robot.gripper('close')\nprint(g)\nassert g.holding\n"
                           "print(robot.goto(by=(0, 0, 0.15)))\nprint('not reached')")],
    ]
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        session = L.FakeLiberoSession(table=L.FakeTable(size=256))
        settings = Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30)
        ep = episode.Episode(session, settings, out)
        ep.run()
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    folder = ep.rec.dir
    result = json.loads((folder / "result.json").read_text())
    assert result["bench"] == "libero" and result["success"] is True, result
    assert result["end"] == "episode_over" and result["turns"] == 2 and result["suite"] == "libero_object_swap", result
    assert result["task"] == "libero_object_swap_t0" and result["seed"] == 0, result
    assert not result["errors"], result["errors"]
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert len(cells) == 2 and "Thing#1 'red box'" in cells[0]["output"], cells[0]
    assert "not reached" not in cells[1]["output"] and cells[1]["stopped"], cells[1]
    messages = fakes.ScriptedGateway.seen[-1]
    system = messages[0]["content"]
    assert system.startswith("# 1. Your job and the rules") and "LIBERO" in system
    card = messages[1]["content"]
    card_text = card if isinstance(card, str) else " ".join(p.get("text", "") for p in card if isinstance(p, dict))
    assert "Task: libero_object_swap_t0 (LIBERO-PRO" in card_text and "You have up to 10 replies" in card_text
    shutil.rmtree(out)


# -- RoboTwin 2.0: the protocol, the rig, the host against the fake table, a scripted episode ------
def test_robotwin_protocol():
    from hexashell.benches.robotwin import protocol as TP

    tasks = TP.tasks()
    assert len(tasks) == 50 and len(set(tasks)) == 50 and tasks[0] == "adjust_bottle"
    published = TP.published_episodes()
    assert len(published) == 250 and len(set(published)) == 250 and {t for t, _ in published} == set(tasks)
    assert all(TP.is_eval_seed(seed) for _, seed in published)
    episodes = TP.test_episodes()
    assert len(episodes) == 750 and len(set(episodes)) == 750 and {t for t, _ in episodes} == set(tasks)
    assert [TP.PART_SEEDS[part] for part in TP.PARTS] == [10, 10, 15]
    for t in tasks:
        learn, validation, test = (TP.seeds(t, part) for part in TP.PARTS)
        assert len(learn) == len(validation) == 10 and len(test) == 15
        assert max(learn) < min(validation) and max(validation) < TP.FIRST_EVAL_SEED <= min(test)
        assert test == sorted(test)
    with raises(ValueError):
        TP.seeds("adjust_bottle", "train")
    with raises(ValueError):
        TP.check_seed("adjust_bottle", 100001)  # unstable: not a test seed
    TP.check_seed("adjust_bottle", 1)  # development seeds are free
    assert TP.instruction_kind(100000) == "unseen" and TP.instruction_kind(99999) == "seen"
    assert not TP.is_eval_seed(99999)
    # RoboTwin's own step limits
    limits = TP.STEP_LIMITS.values()
    assert set(TP.STEP_LIMITS) == set(tasks) and min(limits) == 400 and max(limits) == 1700
    assert TP.step_limit("adjust_bottle") == 400 and TP.step_limit("stack_blocks_three") == 1200
    assert TP.step_limit("open_microwave") == 1500 and TP.step_limit("put_bottles_dustbin") == 1700
    for bad in ("no_such_task", "Adjust_Bottle", ""):
        with raises(ValueError):
            TP.check_task(bad)
        with raises(ValueError):
            TP.step_limit(bad)
    assert TP.table_x("adjust_bottle") == (-0.6, 0.6) and np.allclose(TP.table_x("put_bottles_dustbin"), (-0.3, 0.9))
    assert TP.REPLIES == 40 and TP.EPISODE_S == 7200.0 and TP.STEP_S == 0.2
    assert list(TP.CAMERAS) == ["head", "left_wrist", "right_wrist"]


def test_robotwin_rig():
    from hexashell.benches.robotwin.rig import ROBOTWIN, rig_for
    from hexashell.motion.chain import arm_chain, slide

    # where RoboTwin's simulator puts the grasp points at the start (all joints at zero)
    for arm, grasp in (("left", (-0.30078, -0.17083, 0.94201)), ("right", (0.30500, -0.16981, 0.94101))):
        flange = arm_chain(arm, ROBOTWIN).flange(np.zeros(6))
        assert np.allclose(slide(flange, ROBOTWIN.grasp_reach)[:3, 3], grasp, atol=2e-5), (arm, flange)
        assert flange[1, 0] > 0.999  # pointing away from the robot
    gap = ROBOTWIN.gripper.gap
    assert abs(gap(0.1818, False)) < 1e-3 and abs(gap(1.0, False) - 0.09) < 1e-9 and gap(0.0, True) == 0.0
    rig = rig_for("put_bottles_dustbin", 0.716)
    assert rig.table_z == 0.716 and np.allclose(rig.table_x, (-0.3, 0.9)) and rig.step_s == 0.2
    assert rig.bases == ROBOTWIN.bases and ROBOTWIN.table_z == 0.74


def _open_twin(sam3=True, horizon=600, seed=None, depth=True):
    import fakes_twin as T

    out = Path(tempfile.mkdtemp(prefix="hexashell-twin-"))
    session = T.FakeTwinSession(seed=T.SEED if seed is None else seed, sam3=sam3, twin=T.FakeTwin(horizon=horizon))
    rec = Records(out, session.info, name=session.record_name())
    pictures = []
    session.open(Settings(sam3_url="fake" if sam3 else "", depth=depth), rec,
                 lambda kind, image, caption: pictures.append(caption))
    return session, rec, pictures, out


def test_robotwin_host_moves():
    import fakes_twin as T

    session, rec, pictures, out = _open_twin()
    twin = session.kitchen
    host = session.host
    call = host.dispatch
    try:
        assert host.table_measured and abs(host.table_z - T.TABLE_TOP) < 0.002, host.table_z
        assert twin.observed[0] == ["head"], twin.observed  # the table is measured in the head picture alone
        st = call("state", (), {})
        assert st.step == 0 and st.arms["left"].gripper == 1.0 and "roll -0" not in str(st), st
        info = call("task", (), {})
        assert info.instruction == "pick up the red box" and info.step_limit == 600, info
        with _failed("NotExecuted") as f:
            call("goto", ("left", (-0.15, 0.0, 0.70)), {})
        assert "(0.728)" in f.exc.text, f.exc.text
        with _failed("RobotError"):
            call("drive", (0.1,), {})
        w = call("wait", (5,), {})
        assert w.steps == 5 and "(1.00 s)" in str(w), w

        # pictures are rendered when a call needs them, only for the cameras it needs, once per step
        twin.observed.clear()
        masks = call("find", ("head", "red box"), {})
        assert masks and masks[0].cam == "head" and twin.observed == [["head"]], twin.observed
        pic = call("look", ("all",), {"tile": True})
        assert pic.cams == ["head", "left_wrist", "right_wrist"], pic.cams
        assert twin.observed == [["head"], ["left_wrist", "right_wrist"]], twin.observed
        assert call("image", ("head",), {}).shape == (240, 320, 3) and len(twin.observed) == 2
        assert call("image", ("left",), {}).shape == (240, 320, 3)  # an alias

        px = call("to_pixel", ((0.2, 0.1, T.TABLE_TOP), "head"), {})
        assert px is not None and 0 <= px[0] < 320 and 0 <= px[1] < 240, px
        sp = call("to_world", ("head", px), {})
        assert np.allclose(sp.xyz, (0.2, 0.1, T.TABLE_TOP), atol=0.004) and "depth picture" in str(sp), sp
        sp = call("to_world", ("head", px), {"z": 0.80})
        assert abs(sp.xyz[2] - 0.80) < 1e-6, sp

        t = call("measure", ("red box",), {})
        assert t.found and np.allclose(t.center[:2], T.BOX_START[:2], atol=0.015), t
        assert abs(t.top - (T.TABLE_TOP + T.BOX_SIZE[2])) < 0.01, t
        m = call("goto", ("left", (t.center[0], t.center[1], t.top + 0.08)),
                 {"pitch": 90, "yaw": t.grip_yaw, "grip": "open"})
        assert m.ok and m.angles[0] == 90, m
        t = call("measure", (t,), {})
        assert "left_wrist" in t.views, t
        m = call("goto", ("left", (t.center[0], t.center[1], t.top - 0.03)), {})
        assert m.ok, m
        g = call("gripper", ("left", "close"), {})
        assert g.holding and abs(g.width_m - T.BOX_SIZE[0]) < 0.003, g
        st = call("state", (), {})
        assert st.arms["left"].holding and not st.arms["right"].holding, st
        with raises(Stop, "SUCCESS"):
            call("goto", ("left",), {"by": (0, 0, 0.15)})
        assert session.over and session.verdict["success"] and session.verdict["step"] == session.step
        with raises(Stop, "SUCCESS"):
            call("state", (), {})
    finally:
        session.close()
        rec.close()
    states = (rec.dir / "states.jsonl").read_text().splitlines()
    assert len(states) == twin.steps + 1, (len(states), twin.steps)
    assert twin.closed and "drive" not in twin.ops
    shutil.rmtree(out)


def test_robotwin_host_measures_a_carried_thing_where_the_hand_set_it():
    import fakes_twin as T

    session, rec, pictures, out = _open_twin()
    call, scene = session.host.dispatch, session.host.scene
    try:
        t = call("measure", ("red box",), {})
        call("goto", ("left", (t.center[0], t.center[1], t.top + 0.08)), {"pitch": 90, "yaw": t.grip_yaw, "grip": "open"})
        call("goto", ("left", (t.center[0], t.center[1], t.top - 0.03)), {})
        assert call("gripper", ("left", "close"), {}).holding and list(scene.carried) == [t.id]
        assert scene.by_hand == {t.id: "left"}
        call("goto", ("left",), {"by": (0, 0, 0.04)})  # (8 cm up is the fake's success)
        call("goto", ("left",), {"by": (0.12, 0, 0)})
        call("goto", ("left",), {"by": (0, 0, -0.035)})
        call("gripper", ("left", "open"), {})
        assert not scene.carried and not scene.by_hand
        call("goto", ("left",), {"by": (0, 0, 0.1)})
        # another red box where the first one stood: measuring the first again finds it where it was set
        session.kitchen.extra = [T.BOX_START.copy()]
        again = call("measure", (t,), {})
        assert again.id == t.id and np.allclose(again.center[:2], T.BOX_START[:2] + (0.12, 0), atol=0.015), again
        assert again.notes[0].startswith("moved 1") and "other 'red box' seen" in " ".join(again.notes), again
    finally:
        session.close()
        rec.close()
    events = [json.loads(line) for line in (rec.dir / "events.jsonl").read_text().splitlines()]
    carry = [e for e in events if e["kind"] == "carry"]
    assert carry == [{**carry[0], "kind": "carry", "held": [t.id], "hand": "left"},
                     {**carry[1], "kind": "carry", "set": [t.id], "hand": "left"}], carry
    shutil.rmtree(out)


def test_depth_scene_carries_per_hand():
    """RoboTwin's two hands: what one carries, the other can take over; a host with one ``hand`` (RoboCasa,
    LIBERO) calls as before."""
    from hexashell.robot.results import Thing
    from hexashell.vision.rgbd import DepthScene, _Record

    class _Client:
        def health(self):
            return {"ok": True}

        def close(self):
            pass

    def at(x, y, z):
        pose = np.eye(4)
        pose[:3, 3] = (x, y, z)
        return pose

    events = []
    scene = DepthScene("fake", cams=["head"], check_cam=str, log=events.append, client=_Client())
    poses = {"left": at(-0.15, 0.0, 0.75), "right": at(0.3, 0.0, 0.9)}
    scene.bind(lambda cams: {}, hands={arm: (lambda arm=arm: poses[arm]) for arm in poses})
    box = Thing(id=1, label="red box", found=True, center_xyz=(-0.15, 0.0, 0.75), top=0.775, size=(0.06, 0.04, 0.05))
    scene.records[1] = _Record(box, np.zeros((0, 3)), "red box")
    try:
        scene.grasped(poses["left"], "left")
        assert scene.by_hand == {1: "left"}
        poses["left"] = at(-0.05, 0.0, 0.80)
        assert np.allclose(scene._place(1, scene.records[1]), (-0.05, 0.0, 0.80))
        poses["right"] = poses["left"].copy()  # the right hand takes it where the left has it
        scene.grasped(poses["right"], "right")
        assert scene.by_hand == {1: "right"} and np.allclose(scene.records[1].moved[:3, 3], (0.10, 0.0, 0.05))
        scene.released(poses["left"], "left")  # the left lets go: nothing of its own to set down
        poses["right"] = at(0.0, 0.05, 0.75)
        assert np.allclose(scene._place(1, scene.records[1]), (0.0, 0.05, 0.75))
        scene.released(poses["right"], "right")
        assert not scene.carried and np.allclose(scene._place(1, scene.records[1]), (0.0, 0.05, 0.75))
        assert [{k: v for k, v in e.items() if k != "kind"} for e in events if e["kind"] == "carry"] == [
            {"held": [1], "hand": "left"}, {"held": [1], "hand": "right"}, {"set": [1], "hand": "right"}]
        # one hand, bound as RoboCasa's and LIBERO's hosts do: the log as before
        events.clear()
        tcp = at(0.0, 0.05, 0.75)
        scene.bind(lambda cams: {}, hand=lambda: tcp)
        scene.grasped(tcp)
        tcp = at(0.0, 0.15, 0.75)
        assert np.allclose(scene._place(1, scene.records[1]), (0.0, 0.15, 0.75))
        scene.released(tcp)
        assert [e for e in events if e["kind"] == "carry"] == [{"kind": "carry", "held": [1]},
                                                                {"kind": "carry", "set": [1]}]
    finally:
        scene.close()


def test_robotwin_seeds():
    from hexashell.benches.robotwin import protocol as TP
    from hexashell.benches.session import SimRefused

    # a test seed: the instruction is made from the task's unseen descriptions; a development seed: seen
    for seed, kind in ((100000, "unseen"), (TP.seeds("adjust_bottle", "test")[-1], "unseen"), (7, "seen")):
        session, rec, _, out = _open_twin(sam3=False, seed=seed)
        try:
            assert session.result_fields({})["instruction_from"] == f"the task's {kind} descriptions"
        finally:
            session.close()
            rec.close()
        shutil.rmtree(out)
    # a seed of the evaluation range that is not a test seed is refused
    with raises(SimRefused, "test seeds"):
        _open_twin(sam3=False, seed=100001)


def test_robotwin_place_fan_reward():
    """place_fan calls Reward(subtasks=..., transition_rewards=...); the runtime's Reward is a factory."""
    import types

    from hexashell.benches.robotwin.sim import fix_task_module

    class Factory:
        @staticmethod
        def build_top(config):
            return ("top", config)

    module = types.SimpleNamespace(Reward=Factory)
    fix_task_module("place_fan", module)
    assert module.Reward(subtasks=[1, 2], transition_rewards=[3]) == \
        ("top", {"type": "Serial", "subtasks": [1, 2], "transition_rewards": [3]})
    fixed = module.Reward
    fix_task_module("place_fan", module)
    assert module.Reward is fixed and module.Reward.build_top is Factory.build_top
    other = types.SimpleNamespace(Reward=Factory)
    fix_task_module("adjust_bottle", other)
    assert other.Reward is Factory


def test_robotwin_open_sets_what_the_expert_decides(monkeypatch, tmp_path):
    """open_laptop's success check reads the arm the expert would take (set by play_once, or by get_info
    from the scene as built); the instruction is made from what get_info finds."""
    import importlib
    import types

    from hexashell.benches.robotwin import sim as twin_sim

    class OpenLaptop:
        def setup_demo(self, **kwargs):
            self.seed = kwargs["seed"]

        def get_info(self):
            self.arm_tag = "left"
            return {"{a}": "left"}

        def check_success(self):
            return self.arm_tag == "right"

        def set_instruction(self, instruction):
            self.instruction = instruction

        def close_env(self, clear_cache=True):
            pass

    real = importlib.import_module
    module = types.SimpleNamespace(open_laptop=OpenLaptop)
    monkeypatch.setattr(importlib, "import_module",
                        lambda name, *a: module if name == "robotwin.envs.open_laptop" else real(name, *a))
    monkeypatch.setattr(twin_sim, "task_args", lambda task, save_dir: {})
    monkeypatch.setattr(twin_sim.TwinSim, "_bind", lambda self: None)
    monkeypatch.setattr(twin_sim, "make_instruction", lambda task, info, seed: f"made for {info} {seed}")
    sim = twin_sim.TwinSim(save_dir=str(tmp_path))
    opened = sim.open("open_laptop", 100000)
    assert sim.env.check_success() is False
    assert opened["instruction"] == "made for {'{a}': 'left'} 100000" == sim.env.instruction
    assert opened["instruction_from"] == "the task's unseen descriptions" and opened["step_limit"] == 700
    opened = sim.open("open_laptop", 3)
    assert opened["instruction_from"] == "the task's seen descriptions" and sim.env.step_lim == 700


def test_robotwin_open_builds_again_after_moving_info(monkeypatch, tmp_path):
    """A task whose get_info runs the expert's motions gets its scene built again after it, as RoboTwin's
    evaluation builds the scene again after its expert's run; the others are built once."""
    import importlib
    import types

    from hexashell.benches.robotwin import sim as twin_sim

    calls = []

    class Task:
        def setup_demo(self, **kwargs):
            calls.append(("build", kwargs["seed"]))
            self.moved = False

        def get_info(self):
            calls.append("info")
            self.moved = True  # the expert's motions
            return {"{a}": "left"}

        def set_instruction(self, instruction):
            self.instruction = instruction

        def close_env(self, clear_cache=True):
            calls.append(("close", clear_cache))

    module = types.SimpleNamespace(stack_blocks_three=Task, stack_blocks_two=Task)
    real = importlib.import_module
    monkeypatch.setattr(importlib, "import_module",
                        lambda name, *a: module if name.startswith("robotwin.envs.stack_blocks") else real(name, *a))
    monkeypatch.setattr(twin_sim, "task_args", lambda task, save_dir: {})
    monkeypatch.setattr(twin_sim.TwinSim, "_bind", lambda self: None)
    monkeypatch.setattr(twin_sim, "make_instruction", lambda task, info, seed: "made")
    assert set(twin_sim.MOVING_INFO_TASKS) == {"blocks_ranking_size", "place_bread_basket", "stack_blocks_three"}
    sim = twin_sim.TwinSim(save_dir=str(tmp_path))
    sim.open("stack_blocks_three", 3)
    assert calls == [("build", 3), "info", ("close", False), ("build", 3)] and sim.env.moved is False
    assert sim.env.step_lim == twin_sim.P.step_limit("stack_blocks_three") == 1200
    sim.close()
    calls.clear()
    sim.open("stack_blocks_two", 3)
    assert calls == [("build", 3), "info"]


def test_robotwin_make_instruction(monkeypatch):
    """An episode's instruction comes from RoboTwin's description generator, given the episode's objects
    and arm: from the 'unseen' templates for a test seed (RoboTwin's evaluation of demo_randomized), from
    the 'seen' ones for a development seed."""
    import types

    from hexashell.benches.robotwin import sim as twin_sim

    calls = []

    def generate(task, episodes, max_descriptions, seed):
        calls.append((task, episodes, max_descriptions, seed))
        if task == "lift_pot":
            return [{"seen": ["lift the pot"], "unseen": []}]
        return [{"seen": [f"seen {seed}"], "unseen": [f"unseen {seed}"]}] if task == "place_fan" else []

    monkeypatch.setitem(sys.modules, "robotwin.description",
                        types.SimpleNamespace(generate_episode_descriptions=generate))
    info = {"{A}": "099_fan/base1", "{a}": "left"}
    assert twin_sim.make_instruction("place_fan", info, 100000) == "unseen 100000"
    assert twin_sim.make_instruction("place_fan", info, 7) == "seen 7"
    assert calls == [("place_fan", [info], 1, 100000), ("place_fan", [info], 1, 7)]
    with raises(RuntimeError, "no unseen instruction"):
        twin_sim.make_instruction("lift_pot", info, 100000)
    with raises(RuntimeError, "no seen instruction"):
        twin_sim.make_instruction("adjust_bottle", info, 7)  # no template fits the objects


def test_robotwin_model_file_order(tmp_path):
    """The tasks that pick a model among the files ``glob`` lists get them in the order of their numbers."""
    import glob
    import types

    from hexashell.benches.robotwin.sim import MODEL_FILE_TASKS, fix_task_module, numbered_glob

    for name in ("model_data10.json", "model_data2.json", "model_data.json", "model_data_x.json",
                 "model_data0.json", "base0.glb"):
        (tmp_path / name).write_text("{}")
    got = [Path(p).name for p in numbered_glob(str(tmp_path / "model_data*.json"))]
    assert got[:3] == ["model_data0.json", "model_data2.json", "model_data10.json"]
    assert sorted(got[3:]) == ["model_data.json", "model_data_x.json"]
    assert sorted(numbered_glob(str(tmp_path / "*.glb"))) == [str(tmp_path / "base0.glb")]
    assert set(MODEL_FILE_TASKS) == {"place_a2b_left", "place_a2b_right", "place_object_scale",
                                     "place_object_stand", "put_object_cabinet"}
    for task in MODEL_FILE_TASKS:
        module = types.SimpleNamespace(glob=glob)
        fix_task_module(task, module)
        assert module.glob.glob is numbered_glob
        fixed = module.glob
        fix_task_module(task, module)
        assert module.glob is fixed
    other = types.SimpleNamespace(glob=glob)
    fix_task_module("adjust_bottle", other)
    assert other.glob is glob


def test_robotwin_clutter_area():
    """get_cluttered_table moves its default ranges in place by the table's offset: before every build
    the ranges go back to its own; a task with a shifted table builds as a fourth shifted build would."""
    from hexashell.benches.robotwin import protocol as TP
    from hexashell.benches.robotwin.sim import reset_clutter_area

    class Base:
        table_xy_bias = [0, 0]

        def get_cluttered_table(self, cluttered_numbers=10, xlim=[-0.59, 0.59], ylim=[-0.34, 0.34], zlim=[0.741]):
            xlim[0] += self.table_xy_bias[0]
            xlim[1] += self.table_xy_bias[0]
            ylim[0] += self.table_xy_bias[1]
            ylim[1] += self.table_xy_bias[1]
            self.used = (list(xlim), list(ylim), cluttered_numbers, list(zlim))

    class Shifted(Base):
        table_xy_bias = [0.3, 0]

    class Plain(Base):
        pass

    assert TP.TABLE_SHIFT_X["dump_bin_bigbin"] == 0.3
    assert TP.table_top("adjust_bottle") == 0.74 and abs(TP.table_top("place_dual_shoes") - 0.64) < 1e-9
    fourth = [-0.59, 0.59]
    for _ in range(4):
        fourth = [fourth[0] + 0.3, fourth[1] + 0.3]
    plain, shifted = Plain(), Shifted()
    for task, env in [("adjust_bottle", plain), ("dump_bin_bigbin", shifted), ("dump_bin_bigbin", shifted),
                      ("adjust_bottle", plain), ("put_bottles_dustbin", shifted), ("beat_block_hammer", plain)]:
        reset_clutter_area(env, task)
        env.get_cluttered_table()
        want = fourth if task in TP.TABLE_SHIFT_X else [-0.59, 0.59]
        assert env.used == (want, [-0.34, 0.34], 10, [0.741]), (task, env.used)
    reset_clutter_area(object(), "adjust_bottle")  # nothing to reset
    shifted.get_cluttered_table()  # without the reset the ranges move on, build after build
    shifted.get_cluttered_table()
    assert np.allclose(shifted.used[0], [0.01, 1.19])


def test_robotwin_card():
    session, rec, _, out = _open_twin(sam3=False)
    try:
        text, used = session.card(Settings(skills=True))
        assert "Instruction (from the benchmark; it is authoritative): 'pick up the red box'" in text
        assert "Task: adjust_bottle (RoboTwin 2.0)" in text and "Step budget: 600 steps" in text
        assert "Table top: z = 0.728 (measured in the head picture at the start)." in text
        assert "- left: grasp point (-0.301, -0.171, 0.942), pitch 0 yaw 1 roll 0, gripper 1.00" in text, text
        assert "- right: grasp point (0.305, -0.170, 0.941), pitch 0 yaw 1 roll 0, gripper 1.00" in text, text
        assert "Pictures 1-3 below: head (" in text and not re.search(r"-0(?![.\d])", text), text
        assert used == {"task": "adjust_bottle", "skill": True, "table_z": 0.728, "table_measured": True,
                        "depth": True}, used
        assert "\n## Notes for this task\n\n# adjust_bottle\n" in text  # skills/robotwin/adjust_bottle.md
        plain, used = session.card(Settings(skills=False))
        assert "Notes for this task" not in plain and used["skill"] is False and plain.strip() in text
        assert [c for c, _ in session.start_pictures()] == ["head", "left_wrist", "right_wrist"]
        assert session.prompt_folder() == "robotwin" and session.record_name() == "adjust_bottle_s7"
        assert session.prompt_variant() is None
        assert session.labels() == {"task": "adjust_bottle", "seed": 7}
        fields = session.result_fields({})
        assert fields["instruction_from"] == "the task's seen descriptions" and fields["table_z"] == 0.728, fields
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_robotwin_prompt_folder():
    from hexashell.runtime import card

    text = card.system_prompt("robotwin")
    heads = [line for line in text.splitlines() if line.startswith("# ")]
    assert [h.split(".")[0] for h in heads] == ["# 1", "# 2", "# 3", "# 4", "# 5", "# 6"], heads
    assert "RoboTwin 2.0" in text and "robot.drive" not in text and "'head', 'left_wrist' and 'right_wrist'" in text
    assert "opening the gripper **fully**" in text and "(240, 320, 3)" in text and "0.143 m beyond" in text
    for word in ("RoboDojo", "Isaac", "return to origin", "score table"):
        assert word not in text, word


def test_robotwin_prompt_without_depth():
    from hexashell.runtime import card

    plain, rgb = card.system_prompt("robotwin"), card.system_prompt("robotwin", variant="rgb")
    heads = [line for line in rgb.splitlines() if line.startswith("# ")]
    assert heads == [line for line in plain.splitlines() if line.startswith("# ")], heads
    assert "Every camera gives a colour and a depth picture" in plain and "There are no depth pictures" not in plain
    assert "There are no depth pictures" in rgb and "depth picture" not in rgb.replace("no depth pictures", ""), rgb
    assert "(240, 320, 3)" in rgb and "'head', 'left_wrist' and 'right_wrist'" in rgb and "RoboDojo" not in rgb
    assert card.system_prompt("robodojo", variant="rgb") == card.system_prompt("robodojo")  # it has no rgb parts


def test_robotwin_parts():
    from hexashell.benches.robotwin import protocol as TP
    from hexashell.benches.robotwin.scene import TwinView
    from hexashell.benches.robotwin.sim import TwinSim
    from hexashell.vision.cameras import Camera

    links = {"fl_link1": "left arm", "fl_link6": "left arm", "fl_base_link": "left arm", "left_camera": "left arm",
             "fr_link3": "right arm", "right_camera": "right arm", "fl_link7": "left fingers",
             "fr_link8": "right fingers", "fl_castor_link": "body", "base_link": "body", "footprint": "body"}
    assert {link: TP.ROBOT_PARTS[TP.part_of(link) - 1] for link in links} == links
    sim = TwinSim.__new__(TwinSim)  # (looking parts up needs no simulator)
    sim.part_lut = np.array([0, 0, TP.part_of("fl_link1"), TP.part_of("fr_link7")], np.uint8)
    found = sim.parts_of(np.array([[0, 2, 3], [7, -1, 1]]))
    assert found.dtype == np.uint8 and found.tolist() == [[0, 1, 4], [0, 0, 0]], found
    parts = np.array([[0, 1, 2], [3, 4, 5]], np.uint8)  # nothing, each arm, each arm's fingers, the body
    view = TwinView("head", 0, Camera("head", 300.0, np.eye(4), 3, 2), np.zeros((2, 3, 3), np.uint8), {}, parts=parts)
    assert view.robot.tolist() == [[False, True, True], [True, True, True]]
    assert view.labels().tolist() == [[0, 1, 2], [3, 4, 0]]  # the body does not move aside
    assert [view.arm_of(k) for k in (1, 2, 3, 4)] == ["left", "right", "left", "right"]
    assert view.arm_mask("left").tolist() == [[False, True, False], [True, False, False]]
    assert view.fingers("right").tolist() == [[False, False, False], [False, True, False]]


def test_robotwin_without_depth():
    import fakes_twin as T
    from hexashell.benches.robotwin.host import Shot
    from hexashell.benches.robotwin.scene import TwinScene

    assert Settings().depth is False  # the default: no depth pictures
    session, rec, pictures, out = _open_twin(depth=False)
    twin, host = session.kitchen, session.host
    call = host.dispatch
    try:
        # the table top matched between the start pictures, which come without depth
        assert host.table_measured and abs(host.table_z - T.TABLE_TOP) < 0.002, host.table_z
        assert twin.observed[0] == ["head", "left_wrist", "right_wrist"], twin.observed
        assert all(isinstance(v, Shot) for v in host.views().values()) and isinstance(host.scene, TwinScene)
        assert session.prompt_variant() == "rgb" and session.result_fields({})["depth"] is False
        text, used = session.card(Settings())
        assert "Table top: z = 0.728 (measured in the pictures at the start)." in text and used["depth"] is False
        px = call("to_pixel", ((0.2, 0.1, T.TABLE_TOP), "head"), {})
        sp = call("to_world", ("head", px), {})
        assert np.allclose(sp.xyz, (0.2, 0.1, T.TABLE_TOP), atol=0.004) and "bare table" in str(sp), sp
        sp = call("to_world", ("head", px), {"z": 0.80})
        assert abs(sp.xyz[2] - 0.80) < 1e-6, sp

        # one view: the height is a guess, and the warning says where a wrist camera looks down on it
        t = call("measure", ("red box",), {})
        assert t.found and t.views == ["head"] and np.allclose(t.center[:2], T.BOX_START[:2], atol=0.015), t
        m = re.search(r"robot\.goto\('(\w+)', \(([-\d.]+), ([-\d.]+), ([-\d.]+)\), pitch=(\d+), yaw=(-?\d+), roll=0\)",
                      t.warning)
        assert m and m.group(1) == "left", t.warning
        look = [float(v) for v in m.groups()[1:4]]
        assert call("goto", ("left", look), {"pitch": int(m.group(5)), "yaw": int(m.group(6)), "roll": 0}).ok
        top = np.array([t.center[0], t.center[1], t.top])
        wrist = host.views()["left_wrist"].camera
        x, y = wrist.pixel(top)
        assert abs(x - 159.5) < 3 and abs(y - 119.5) < 3, (x, y)  # the guessed top in the middle of the picture
        assert abs(np.linalg.norm(top - wrist.position) - 0.20) < 0.005
        t = call("measure", (t,), {})
        assert t.views == ["head", "left_wrist"] and not t.warning, t
        assert np.allclose(t.center, T.BOX_START, atol=0.004) and abs(t.top - T.TABLE_TOP - T.BOX_SIZE[2]) < 0.005, t

        m = call("goto", ("left", (t.center[0], t.center[1], t.top + 0.08)), {"pitch": 90, "yaw": t.grip_yaw,
                                                                               "grip": "open"})
        assert m.ok, m
        assert call("goto", ("left", (t.center[0], t.center[1], t.top - 0.03)), {}).ok
        assert call("gripper", ("left", "close"), {}).holding
        with raises(Stop, "SUCCESS"):
            call("goto", ("left",), {"by": (0, 0, 0.15)})
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_robotwin_to_world_without_depth_or_perception():
    import fakes_twin as T

    session, rec, pictures, out = _open_twin(sam3=False, depth=False)
    host = session.host
    call = host.dispatch
    try:
        assert host.scene is None and abs(host.table_z - T.TABLE_TOP) < 0.002
        px = call("to_pixel", ((0.2, 0.1, T.TABLE_TOP), "head"), {})
        sp = call("to_world", ("head", px), {})
        assert np.allclose(sp.xyz, (0.2, 0.1, T.TABLE_TOP), atol=0.004) and "assumes the pixel shows the table top" \
            in sp.warning, sp
        call("goto", ("left", (-0.1, 0.0, 0.9)), {"pitch": 90})
        ys, xs = np.nonzero(host.views()["head"].parts == T.part("left fingers"))
        assert len(xs), "the left hand is in the head picture"
        sp = call("to_world", ("head", (float(xs[len(xs) // 2]), float(ys[len(ys) // 2]))), {})
        assert "that pixel shows the robot" in sp.warning, sp
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_robotwin_plain_table(monkeypatch):
    """Too few heights agree between the pictures (as on a plain table): the table top's height from where
    its far edge shows in the head picture, else the task's usual height, and spots on it warned of."""
    import fakes_twin as T
    from hexashell.benches.robotwin import host as H

    monkeypatch.setattr(H, "TABLE_MIN_MATCHES", 10**6)
    for edge in (True, False):
        if not edge:
            monkeypatch.setattr(H, "TABLE_EDGE_MIN_STEP", 1e9)
        session, rec, pictures, out = _open_twin(depth=False)
        host = session.host
        try:
            text, _ = session.card(Settings())
            px = host.dispatch("to_pixel", ((0.2, 0.1, T.TABLE_TOP), "head"), {})
            sp = host.dispatch("to_world", ("head", px), {})
            assert "bare table" in sp.source, sp
            if edge:
                assert host.table_by == "the far edge" and abs(host.table_z - T.TABLE_TOP) < 0.003, host.table_z
                assert "(measured where its far edge shows in the head picture at the start; good to about 6 mm)" \
                    in text
                assert np.allclose(sp.xyz, (0.2, 0.1, T.TABLE_TOP), atol=0.006) and sp.warning is None, sp
            else:
                assert not host.table_measured and host.table_z == 0.74, host.table_z
                assert "Table top: about z = 0.740 (it could not be measured in the pictures; it may be up to 3 cm " \
                    "lower)." in text
                assert sp.warning.startswith("the table top's height could not be measured (z = 0.740 is assumed"), sp
        finally:
            session.close()
            rec.close()
        shutil.rmtree(out)


def test_scripted_robotwin_episode():
    system, card_text = _scripted_twin(depth=True)
    assert "Every camera gives a colour and a depth picture" in system
    assert "(measured in the head picture at the start)" in card_text


def test_scripted_robotwin_episode_without_depth():
    system, card_text = _scripted_twin(depth=False)
    assert "There are no depth pictures" in system and "depth picture (" not in system
    assert "(measured in the pictures at the start)" in card_text


def _scripted_twin(depth: bool) -> tuple[str, str]:
    """A scripted episode on the fake table (with depth pictures or without): (system prompt, task card)."""
    import fakes
    import fakes_twin as T
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-twin-run-"))
    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nrobot.look('all', tile=True)\n"
                           "t = robot.measure('red box')\nprint(t)")],
        [fakes.python_call("m = robot.goto('left', (t.center[0], t.center[1], t.top + 0.08), pitch=90, "
                           "yaw=t.grip_yaw, grip='open')\nprint(m)\nt = robot.measure(t)\nprint(t)\n"
                           "print(robot.goto('left', (t.center[0], t.center[1], t.top - 0.03)))\n"
                           "g = robot.gripper('left', 'close')\nprint(g)\nassert g.holding\n"
                           "try:\n    robot.goto('right', (3, 3, 3))\nexcept NotExecuted as e:\n    print('caught')\n"
                           "print(robot.goto('left', by=(0, 0, 0.15)))\nprint('not reached')")],
    ]
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        session = T.FakeTwinSession(twin=T.FakeTwin())
        settings = Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30,
                            depth=depth)
        ep = episode.Episode(session, settings, out)
        ep.run()
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    folder = ep.rec.dir
    result = json.loads((folder / "result.json").read_text())
    assert result["bench"] == "robotwin" and result["success"] is True, result
    assert result["end"] == "episode_over" and result["turns"] == 2, result
    assert result["task"] == "adjust_bottle" and result["seed"] == 7 and result["table_z"] == 0.728, result
    assert result["depth"] is depth, result
    assert result["robot_calls"]["goto"] == 4 and result["robot_calls"]["measure"] == 2, result["robot_calls"]
    assert not result["errors"], result["errors"]
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert len(cells) == 2 and "Thing#1 'red box'" in cells[0]["output"], cells[0]
    assert "Grip left: closed on something" in cells[1]["output"] and "caught" in cells[1]["output"], cells[1]
    assert "not reached" not in cells[1]["output"] and cells[1]["stopped"], cells[1]
    messages = fakes.ScriptedGateway.seen[-1]
    system = messages[0]["content"]
    assert system.startswith("# 1. Your job and the rules") and "RoboTwin 2.0" in system
    card = messages[1]["content"]
    card_text = card if isinstance(card, str) else " ".join(p.get("text", "") for p in card if isinstance(p, dict))
    assert "Task: adjust_bottle (RoboTwin 2.0)" in card_text and "You have up to 10 replies" in card_text
    assert len((folder / "states.jsonl").read_text().splitlines()) == session.kitchen.steps + 1
    shutil.rmtree(out)
    return system, card_text


def test_arm_slows_while_the_elbow_has_little_room_ahead():
    from hexashell.benches import arm as A

    start, goal = np.eye(4), np.eye(4)
    goal[:3, 3] = [0.4, 0.0, 0.0]
    goal[:3, :3] = A.rotation(np.array([0.0, 0.0, math.radians(30.0)]))
    plain = A.hand_path(start, goal)
    # room enough all the way: the same plan as without the elbow
    roomy = A.hand_path(start, goal, room=lambda p: math.radians(80.0))
    assert len(roomy) == len(plain) and all(np.allclose(a, b) for a, b in zip(roomy, plain))

    def speeds(path):
        x = np.array([0.0] + [p[0, 3] for p in path])
        v = np.diff(x) / A.DT
        assert abs(x[-1] - 0.4) < 1e-9 and np.all(v >= -1e-12)
        assert np.abs(np.diff(np.r_[0.0, v, 0.0]) / A.DT).max() <= A.SLOWEST * A.ARM_ACCEL * 1.05
        return x[1:], v

    # toward less room (80 deg at the start, 30 at the goal: 40 deg at x = 0.32, so SLOWEST from
    # SLOW_AHEAD before): fast at first, then slow, all of it at the lower acceleration
    x, v = speeds(A.hand_path(start, goal, room=lambda p: math.radians(80.0 - 50.0 * p[0, 3] / 0.4)))
    slow_from = 0.32 - A.SLOW_AHEAD
    assert len(v) > len(plain) and v[x < slow_from - 0.1].max() > 0.9 * A.ARM_SPEED
    assert v[x > slow_from + 0.015].max() <= A.SLOWEST * A.ARM_SPEED * 1.05
    # away from it (30 deg at the start, 80 at the goal): slow at first, at full speed once the way
    # ahead has room
    x, v = speeds(A.hand_path(start, goal, room=lambda p: math.radians(30.0 + 50.0 * p[0, 3] / 0.4)))
    assert v[x < 0.06].max() <= A.SLOWEST * A.ARM_SPEED * 1.05 and v.max() > 0.95 * A.ARM_SPEED
    assert A.speed_cap(A.SLOW_ROOM) == 1.0 and A.speed_cap(A.SLOWEST_ROOM) == A.SLOWEST
    assert A.speed_cap(math.radians(30.0)) == A.SLOWEST
    # an arc too: slow from SLOW_AHEAD before where the room is least
    # (from the origin about a line 0.3 m to the right: the point swings back to x = -0.26,
    # passing x = -0.15 at 30 degrees, 0.157 m of way in)
    arc = A.arc_path(start, [0.0, -0.3, 0.0], [0.0, 0.0, 1.0], math.radians(60.0),
                     room=lambda p: math.radians(30.0) if p[0, 3] < -0.15 else math.radians(90.0))
    at = np.array([[0.0, 0.0, 0.0]] + [p[:3, 3] for p in arc])
    speed = np.linalg.norm(np.diff(at, axis=0), axis=1) / A.DT
    assert abs(at[-1, 0] + 0.3 * math.sin(math.radians(60.0))) < 1e-9
    late = speed[at[1:, 0] < -0.3 * math.sin((0.157 - A.SLOW_AHEAD + 0.02) / 0.3)]
    assert late.size and late.max() <= A.SLOWEST * A.ARM_SPEED * 1.05 < 0.9 * speed.max()


def test_fit_circle_holds_a_short_track_near_the_hinge_given():
    from hexashell.benches import arm as A

    # 40 degrees of a 20 cm circle about a point 3.6 cm from the hinge given: found
    t = np.radians(np.linspace(0.0, 40.0, 9))
    centre, radius = A.fit_circle(np.c_[0.03 + 0.2 * np.cos(t), -0.02 + 0.2 * np.sin(t)], np.zeros(2))
    assert np.linalg.norm(centre - [0.03, -0.02]) < 0.005 and abs(radius - 0.2) < 0.005, (centre, radius)
    # a few millimetres of nearly straight track fix no centre: it stays near the one given
    centre, radius = A.fit_circle(np.array([[0.2, 0.0], [0.2, 0.003], [0.2, 0.006]]), np.zeros(2))
    assert np.linalg.norm(centre) < 0.01 and abs(radius - 0.2) < 0.01, (centre, radius)


def test_arm_moves_at_the_speed_asked_for():
    from hexashell.benches import arm as A

    start, goal = np.eye(4), np.eye(4)
    goal[:3, 3] = [0.2, 0.0, 0.0]
    plain = A.hand_path(start, goal)
    assert all(np.allclose(a, b) for a, b in zip(A.hand_path(start, goal, speed=A.ARM_SPEED), plain))
    # 0.2 m at 0.05 m/s with the acceleration in proportion: 4 s at the peak plus one ramp of 0.25 s
    slow = A.hand_path(start, goal, speed=0.05)
    x = np.array([0.0] + [p[0, 3] for p in slow])
    v = np.diff(x) / A.DT
    assert len(slow) == round((0.2 / 0.05 + 0.25) / A.DT) and v.max() <= 0.05 * 1.001 and abs(x[-1] - 0.2) < 1e-9
    # below the slowest: as slow as that; turning in place slows in proportion too
    assert len(A.hand_path(start, goal, speed=0.001)) == len(A.hand_path(start, goal, speed=A.MIN_SPEED))
    turned = np.eye(4)
    turned[:3, :3] = A.rotation(np.array([0.0, 0.0, math.radians(30.0)]))
    slow = A.hand_path(start, turned, speed=0.05)
    rates = [A.turn_between(a[:3, :3], b[:3, :3]) / A.DT for a, b in zip([start] + slow, slow)]
    assert max(rates) <= A.ARM_TURN * 0.05 / A.ARM_SPEED * 1.001 and np.allclose(slow[-1], turned)
    # an arc too, the elbow's slowing on top
    room = lambda p: math.radians(30.0) if p[0, 3] < -0.15 else math.radians(90.0)  # noqa: E731
    arc = A.arc_path(start, [0.0, -0.3, 0.0], [0.0, 0.0, 1.0], math.radians(60.0), room=room, speed=0.05)
    at = np.array([[0.0, 0.0, 0.0]] + [p[:3, 3] for p in arc])
    speed = np.linalg.norm(np.diff(at, axis=0), axis=1) / A.DT
    assert speed.max() <= 0.05 * 1.001 and abs(at[-1, 0] + 0.3 * math.sin(math.radians(60.0))) < 1e-9
    assert speed[at[1:, 0] < -0.25].max() <= A.SLOWEST * 0.05 * 1.05


def _bench_pool():
    """scripts/bench_pool.py as a module (instead of running it under bash)."""
    import importlib.util

    old = os.environ.get("HEXASHELL_POOL_ENV")
    os.environ["HEXASHELL_POOL_ENV"] = "1"
    try:
        spec = importlib.util.spec_from_file_location("bench_pool", ROOT / "scripts" / "bench_pool.py")
        pool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pool)
    finally:
        if old is None:
            os.environ.pop("HEXASHELL_POOL_ENV")
        else:
            os.environ["HEXASHELL_POOL_ENV"] = old
    return pool


def test_pool_gives_each_gpu_its_own_sam3_service():
    from hexashell.benches.run import settings_for

    pool = _bench_pool()
    urls = pool.gpu_urls(" 0=http://127.0.0.1:18320, 5=http://127.0.0.1:18325,")
    assert urls == {"0": "http://127.0.0.1:18320", "5": "http://127.0.0.1:18325"} and pool.gpu_urls("") == {}
    for bad in ("0", "x=http://127.0.0.1:1", "1="):
        with raises(ValueError, "SAM3_URLS"):
            pool.gpu_urls(bad)
    sets = ["skills=true", "sam3_url=http://127.0.0.1:18300"]
    cmd = pool.episode_cmd("py", "robocasa", "OpenDrawer", "3", "/r", "5", "/s.sock", sets, urls)
    assert cmd[:3] == ["py", "-m", "hexashell.benches.run"] and cmd[cmd.index("--egl") + 1] == "5"
    assert cmd[-2:] == ["--set", "sam3_url=http://127.0.0.1:18325"]
    # the runner takes the last --set: the GPU's own service wins over the one in SETS
    words = [w for i, w in enumerate(cmd) if i and cmd[i - 1] == "--set"]
    assert settings_for("robocasa", "OpenDrawer", None, words).sam3_url == "http://127.0.0.1:18325"
    # a GPU without its own service: SETS as given
    other = pool.episode_cmd("py", "robocasa", "OpenDrawer", "3", "/r", "2", "/s.sock", sets, urls)
    assert other[-2:] == ["--set", "sam3_url=http://127.0.0.1:18300"]


def test_pool_lists_an_episode_its_gateway_ended_again_without_an_attempt():
    pool = _bench_pool()
    assert pool.relist("ok", "robocasa", "OpenDrawer", "3", 1, 0) == (None, False)
    assert pool.relist("gateway_error", "robocasa", "OpenDrawer", "3", 2, 0) == ("robocasa OpenDrawer 3 2", True)
    last = pool.GATEWAY_RELISTS
    assert pool.relist("gateway_error", "robocasa", "OpenDrawer", "3", 2, last) == ("robocasa OpenDrawer 3 3", False)
    assert pool.relist("error", "robocasa", "OpenDrawer", "3", 1, 0) == ("robocasa OpenDrawer 3 2", False)
    assert pool.relist("no record", "robocasa", "OpenDrawer", "3", pool.ATTEMPTS, 0) == (None, False)


def test_gpu_check_tells_a_gpu_that_times_out():
    from hexashell.benches import gpucheck as G

    # one machine: a healthy GPU, then one whose graphics engine timed out (NVIDIA Xid 109), then another
    # while simulator servers and SAM3 services started on the machine (no Xid): only its first frame was slow
    assert G.faults([0.239, 0.009, 0.009, 0.009, 0.009], 0) == []
    assert G.faults([4.771, 4.5], 0) == ["a frame took 4.5 s"]
    assert G.faults([4.8, 0.002, 0.002, 0.016, 0.002], 0) == []
    assert G.faults([61.0], 0) == ["the first frame took 61.0 s"]
    assert G.faults([0.3, 0.01, 4.5], 0) == ["a frame took 4.5 s"]
    assert G.faults([0.3] * 5, 7) == ["7 depth values outside 0..1"]
    depth = np.full((2, 2), 0.5, np.float32)
    assert G.check_depth(depth, "agentview") is depth
    depth[0, 1] = np.nan
    with raises(RuntimeError, "1 depth values of camera agentview are not numbers"):
        G.check_depth(depth, "agentview")
    # a picture of GPU 5 after its Xid took 128.9 s; a healthy GPU's take milliseconds, the first seconds
    assert G.check_time(0.03, "agentview") is None
    assert G.check_time(30.0, "agentview", first=True) is None
    with raises(RuntimeError, "a picture of camera agentview took 128.9 s (over 60 s)"):
        G.check_time(128.9, "agentview", first=True)
    with raises(RuntimeError, "a picture of camera wrist took 11.0 s (over 10 s)"):
        G.check_time(11.0, "wrist")


def test_simulators_stop_at_a_picture_drawn_too_slowly(monkeypatch):
    import types

    from hexashell.benches.libero import sim as L
    from hexashell.benches.robocasa import sim as C

    for module, cls, cam in ((L, L.LiberoSim, "agentview"), (C, C.CasaSim, "left")):
        clock, took, asked = [0.0], [], []

        def render(**kw):
            asked.append(kw)
            clock[0] += took.pop(0)
            return "picture"

        monkeypatch.setattr(module, "time", types.SimpleNamespace(time=lambda: clock[0]))
        sim = types.SimpleNamespace(render=render)
        s = object.__new__(cls)
        s.size, s.drawn, s.env = 8, False, types.SimpleNamespace(sim=sim, env=types.SimpleNamespace(sim=sim))
        took += [30.0, 0.02]  # the episode's first picture may take seconds, the next ones not
        assert s.picture(cam, depth=True) == "picture" and s.picture("wrist") == "picture"
        assert asked[0] == {"width": 8, "height": 8, "camera_name": module.P.CAMERAS[cam], "depth": True}
        took.append(11.0)
        with raises(RuntimeError, f"a picture of camera {cam} took 11.0 s (over 10 s)"):
            s.picture(cam, segmentation=True)


def test_gpu_check_draws_in_two_processes_at_once():
    import subprocess
    import time

    from hexashell.benches import gpucheck as G

    # two processes drawing the check's scene at once: a GPU after its Xid 109 (alone it drew 0.002 s a
    # frame), then one that timed out on every frame
    gpu5 = [{"gpu": "5", "egl": 5, "first_s": 9.897, "frames": 111, "slowest_s": 10.928, "bad_depth": 0},
            {"gpu": "5", "egl": 5, "first_s": 14.674, "frames": 1, "slowest_s": 7.946, "bad_depth": 0}]
    assert G.combine("5", gpu5) == {"gpu": "5", "egl": 5, "first_s": 14.674, "frames_s": [10.928, 7.946],
                                    "frames": [111, 1], "bad_depth": 0, "ok": False,
                                    "why": "a frame took 10.9 s" + G.WHY_SUFFIX}
    gpu3 = [{"gpu": "3", "egl": 3, "first_s": 1.134, "frames": 399, "slowest_s": 0.034, "bad_depth": 0},
            {"gpu": "3", "egl": 3, "first_s": 1.274, "frames": 399, "slowest_s": 0.021, "bad_depth": 0}]
    assert G.combine("3", gpu3)["ok"] is True
    # one whose first frame hung, one that could not draw at all
    hung = {"gpu": "3", "egl": 3, "first_s": 61.0, "frames": 0, "slowest_s": None, "bad_depth": 0}
    assert G.combine("3", [hung, gpu3[1]])["why"] == "the first frame took 61.0 s" + G.WHY_SUFFIX
    failed = {"gpu": "3", "why": "ModuleNotFoundError: No module named 'mujoco'"}
    assert G.combine("3", [gpu3[0], failed]) == {"gpu": "3", "egl": 3, "ok": False, "why": failed["why"]}
    # what the check reads of a drawing process: its next record past other lines, why it ended without one,
    # nothing by the deadline
    def run(code):
        return subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, bufsize=0)

    proc = run("import time; print('WARNING: a warning'); print('{\"ready\": true}', flush=True); time.sleep(30)")
    try:
        assert G.answer(proc, time.time() + 30) == {"ready": True}
        assert G.answer(proc, time.time() + 0.3) is None
    finally:
        proc.kill()
        proc.wait()
    assert G.answer(run("raise SystemExit(3)"), time.time() + 30) == {
        "why": "a drawing process ended (exit code 3) without a record"}


def test_pool_starts_no_server_on_a_gpu_that_cannot_draw(monkeypatch, tmp_path):
    import time

    from hexashell.benches.session import server_env

    pool = _bench_pool()
    for bench in pool.CHECKERS:
        monkeypatch.delenv(f"HEXASHELL_{bench.upper()}_PYTHON", raising=False)
        monkeypatch.delenv(f"HEXASHELL_{bench.upper()}_ENV", raising=False)
    assert pool.checker() is None
    monkeypatch.setenv("HEXASHELL_ROBOCASA_PYTHON", "/casa/python")
    assert pool.checker() == ("/casa/python", None)
    monkeypatch.setenv("HEXASHELL_LIBERO_PYTHON", "/libero/python")
    assert pool.checker() == ("/libero/python", None)
    # the check itself, in an environment without MuJoCo: it answers, and the GPU is unfit
    rec = pool.probe(sys.executable, server_env(None, "0"), "0")
    assert rec["ok"] is False and "mujoco" in rec["why"]
    assert pool.probe(str(tmp_path / "no-python"), {}, "0")["why"] == "the check gave no answer (FileNotFoundError)"
    # a GPU found unfit gets no servers until a check GPU_CHECK seconds later
    checks = []

    def probe(python, env, gpu):
        checks.append((python, env["HEXA_EGL_DEVICE"]))
        return {"gpu": gpu, "ok": gpu != "3", "why": "the first frame took 4.7 s", "first_s": 0.24, "frames_s": []}

    monkeypatch.setattr(pool, "HERE", tmp_path)
    monkeypatch.setattr(pool, "probe", probe)
    runner = pool.Pool("check", ["3", "0"], 2)
    os.rmdir(runner.sockets)
    assert not runner.gpu_fit("3") and runner.unfit("3") and runner.gpu_fit("0") and not runner.unfit("0")
    assert not runner.gpu_fit("3") and checks == [("/libero/python", "3"), ("/libero/python", "0")]
    runner.gpu_checks["3"] = (False, time.time() - pool.GPU_CHECK - 1)
    assert not runner.unfit("3") and not runner.gpu_fit("3") and len(checks) == 3
    assert "gpu=3 is not fit to render" in (tmp_path / "logs" / "check.events").read_text()


def test_egl_device_of_a_gpu():
    from hexashell.benches import egl

    assert egl._uuid("GPU-FBF5C7EF-2c82-1520-d2c2-b2e7d2c93c94") == "fbf5c7ef-2c82-1520-d2c2-b2e7d2c93c94"
    smi = {0: {"bus": "0000:04:00.0", "uuid": "aaaa"}, 1: {"bus": "0000:23:00.0", "uuid": "bbbb"}}
    # EGL lists the cards in another order, and Mesa a card a second time: the UUID decides
    rows = [{"nvidia": False, "cuda": None, "uuid": None, "bus": "0000:23:00.0"},
            {"nvidia": True, "cuda": 1, "uuid": "bbbb", "bus": None},
            {"nvidia": True, "cuda": 0, "uuid": "aaaa", "bus": None}]
    assert egl.egl_index(0, rows, smi) == 2 and egl.egl_index(1, rows, smi) == 1
    # no UUIDs: the PCI bus; no DRM node either (a user outside the video group): the CUDA ordinal
    by_bus = [dict(r, uuid=None, bus=b) for r, b in zip(rows, ("0000:23:00.0", "0000:23:00.0", "0000:04:00.0"))]
    assert egl.egl_index(1, by_bus, smi) == 1
    by_cuda = [dict(r, uuid=None, bus=None) for r in rows]
    assert egl.egl_index(0, by_cuda, smi) == 2 and egl.egl_index(1, by_cuda, {}) == 1
    # nothing known: the index itself
    assert egl.egl_index(5, [], {}) == 5


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    only = sys.argv[1:]
    failed = 0
    for name, fn in tests:
        if only and not any(o in name for o in only):
            continue
        try:
            fn()
            print(f"ok    {name}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            import traceback

            print(f"FAIL  {name}: {type(exc).__name__}: {exc}")
            traceback.print_exc()
    sys.exit(1 if failed else 0)
