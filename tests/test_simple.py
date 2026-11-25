"""SIMPLE (the humanoid): the protocol, the server's frames, the host against the fake room, a scripted episode
and the registration in benches.run.  No simulator needed."""

from __future__ import annotations

import contextlib
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from hexashell.benches import results as R  # noqa: E402
from hexashell.benches.simple import protocol as P  # noqa: E402
from hexashell.kernel.host import CallFailed, Stop  # noqa: E402
from hexashell.runtime.episode import Settings  # noqa: E402
from hexashell.runtime.records import Records  # noqa: E402


@contextlib.contextmanager
def raises(kind):
    try:
        yield
    except kind:
        return
    raise AssertionError(f"{kind.__name__} not raised")


class _Failed:
    exc: CallFailed | None = None


@contextlib.contextmanager
def failed(kind: str):
    box = _Failed()
    try:
        yield box
    except CallFailed as exc:
        assert exc.kind == kind, (exc.kind, exc.text)
        box.exc = exc
        return
    raise AssertionError(f"CallFailed({kind}) not raised")


# -- the protocol and the server's frames --------------------------------------------------------------
def test_simple_protocol():
    assert P.split_of(1000) == "learning" and P.split_of(2999) == "validation" and P.split_of(0) is None
    assert P.split_of(3000) == "test" and P.split_of(3029) == "test" and P.split_of(3030) is None
    assert P.check_seed(1500) == 1500
    for bad in (0, 999, 3030):
        with raises(ValueError):
            P.check_seed(bad)
    for bad in (0, 3030):
        with raises(ValueError):
            P.check_seed(bad, official_test=True)
    n, f = P.palm_axes("inward", "left")
    assert n == (0.0, -1.0, 0.0) and f == (1.0, 0.0, 0.0)
    assert P.palm_axes("inward", "right")[0] == (0.0, 1.0, 0.0) and P.palm_axes("outward", "right")[0][1] == -1.0
    with raises(ValueError):
        P.palm_axes("sideways", "left")
    assert P.CAMERAS == ("gaze",) and P.CAMERA_ALIASES["head"] == "gaze"
    assert P.RELAXED_STEPS == 2250 and P.official_success(400, "bend_pick") and not P.official_success(401, "bend_pick")


def test_simple_official_test_episodes():
    """Test seeds: the official evaluation episodes, numbered by level and index, refused unless allowed."""
    from hexashell.benches.armserver import Refused
    from hexashell.benches.simple import server as S

    assert P.TEST_EPISODES == 30 and P.SPLITS["test"] == (3000, 3030)
    assert [P.test_seed(level, 0) for level in P.OFFICIAL_LEVELS] == [3000, 3010, 3020]
    assert P.test_seed("dr-level-1", 3) == 3013 and P.official_episode(3013) == ("dr-level-1", 3)
    assert P.official_episode(3029) == ("dr-level-2", 9)
    assert all(P.official_episode(P.test_seed(lv, i)) == (lv, i) for lv in P.OFFICIAL_LEVELS for i in range(10))
    for bad in (("dr-level-hard", 0), ("dr-level-0", 10), ("dr-level-0", -1)):
        with raises(ValueError):
            P.test_seed(*bad)
    with raises(ValueError):
        P.official_episode(1001)
    # refused unless the run allows the test
    with raises(ValueError):
        P.check_seed(3000)
    assert P.check_seed(3000, official_test=True) == 3000
    assert not P.official_test_allowed({}) and not P.official_test_allowed({P.OFFICIAL_TEST_VARIABLE: "0"})
    assert P.official_test_allowed({P.OFFICIAL_TEST_VARIABLE: "1"})
    with raises(Refused):
        S.Server().open("bend_pick", 3000)
    # the episode's saved configuration, as SIMPLE's evaluation loads it (scene uid renamed as there)
    data = Path(tempfile.mkdtemp(prefix="hexashell-simple-eval-"))
    try:
        meta = data / "G1WholebodyHandoverTeleop-v0" / "dr-level-2" / "meta"
        meta.mkdir(parents=True)
        lines = []
        for i in range(10):
            conf = {"uid": "g1_wholebody_handover_teleop", "dr_state_dict": {"scene": {"uid": "102344280_a"},
                                                                             "spatial": {"g1_sonic": {"i": i}}}}
            lines.append(json.dumps({"episode_index": i, "tasks": [f"hand over {i}"], "length": 5,
                                     "environment_config": json.dumps(conf)}))
        (meta / "episodes.jsonl").write_text("\n".join(lines) + "\n")
        assert P.official_episodes_file("handover", "dr-level-2", data) == meta / "episodes.jsonl"
        conf, entry = P.official_environment_config("handover", 3027, data)
        assert conf["dr_state_dict"]["scene"]["uid"] == "scene3_a" and conf["dr_state_dict"]["spatial"]["g1_sonic"]["i"] == 7
        assert entry["tasks"] == ["hand over 7"] and entry["level"] == "dr-level-2" and "environment_config" not in entry
        with raises(ValueError):
            P.official_environment_config("handover", 1001, data)
    finally:
        shutil.rmtree(data, ignore_errors=True)
    # the episode side: a test seed only when the run allows it, and then the server is told so
    import fakes_simple as F

    saved = os.environ.pop(P.OFFICIAL_TEST_VARIABLE, None)
    try:
        with raises(ValueError):
            F.FakeSimpleSession("bend_pick", 3005)
        assert "official_test" not in F.FakeSimpleSession("bend_pick", 1001).open_args()
        os.environ[P.OFFICIAL_TEST_VARIABLE] = "1"
        session = F.FakeSimpleSession("bend_pick", 3005)
        assert session.open_args()["official_test"] is True
        assert session.labels()["official_episode"] == "dr-level-0 episode 5"
        assert "official_test" not in F.FakeSimpleSession("bend_pick", 1001).open_args()
        out = Path(tempfile.mkdtemp(prefix="hexashell-simple-"))
        try:
            rec = Records(out, session.info, name=session.record_name())
            session.open(Settings(sam3_url="fake"), rec, lambda kind, image, caption: None)
            assert session.room.opened_with == {"seed": 3005, "official_test": True}
            assert session.result_fields({})["split"] == "test"
        finally:
            shutil.rmtree(out, ignore_errors=True)
    finally:
        os.environ.pop(P.OFFICIAL_TEST_VARIABLE, None)
        if saved is not None:
            os.environ[P.OFFICIAL_TEST_VARIABLE] = saved


def test_simple_server_frames_and_refusals():
    from hexashell.benches.armserver import Refused
    from hexashell.benches.simple import server as S

    srv = S.Server()
    with raises(Refused):
        srv.handle("walk", {"x": 0.0, "y": 0.0})
    assert srv.handle("ping", {})["pid"] == os.getpid() and srv.status() == {}
    # the start frame: 30 degrees left of the world's x, at (1, 2) on a floor 0.1 m high
    a = math.radians(30.0)
    start = np.eye(4)
    start[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    start[:3, 3] = (1.0, 2.0, 0.1)
    srv.start, srv.start_inv, srv.start_heading = start, np.linalg.inv(start), 30.0
    world = srv._world_point([0.5, 0.0, 0.7])
    assert np.allclose(world, (1.0 + 0.5 * math.cos(a), 2.0 + 0.5 * math.sin(a), 0.8))
    assert np.allclose(srv._point(world), (0.5, 0.0, 0.7), atol=1e-4)
    assert np.allclose(srv._dir(srv._world_dir([0.0, 1.0, 0.0])), (0.0, 1.0, 0.0), atol=1e-4)
    assert S.wrap_deg(190.0) == -170.0 and S.LOG_COLUMNS == 14
    # before the simulator starts: no display, no proxy, MuJoCo on EGL
    saved = dict(os.environ)
    try:
        os.environ.update({"DISPLAY": ":1", "HTTPS_PROXY": "http://127.0.0.1:1"})
        os.environ.pop("MUJOCO_GL", None)
        os.environ.pop("HEXA_EGL_DEVICE", None)
        S.prepare()
        assert "DISPLAY" not in os.environ and "HTTPS_PROXY" not in os.environ and os.environ["MUJOCO_GL"] == "egl"
        assert os.environ["OMNI_KIT_ACCEPT_EULA"] == "YES"
        # with a GPU: Isaac Sim's CUDA device is set before EGL is asked (that starts CUDA in the process)
        from hexashell.benches import egl

        seen = {}

        def egl_index(gpu):
            seen["cuda"] = os.environ.get("CUDA_VISIBLE_DEVICES")
            return 5

        real, egl.egl_index = egl.egl_index, egl_index
        try:
            os.environ["HEXA_EGL_DEVICE"] = "3"
            S.prepare()
        finally:
            egl.egl_index = real
        assert seen["cuda"] == "3" and os.environ["CUDA_VISIBLE_DEVICES"] == "3"
        assert os.environ["MUJOCO_EGL_DEVICE_ID"] == "5" and os.environ["CUDA_DEVICE_ORDER"] == "PCI_BUS_ID"
    finally:
        os.environ.clear()
        os.environ.update(saved)


# -- the host against the fake room ----------------------------------------------------------------------
def _open(sam3=True, horizon=P.RELAXED_STEPS, task="bend_pick"):
    import fakes_simple as F

    out = Path(tempfile.mkdtemp(prefix="hexashell-simple-"))
    session = F.FakeSimpleSession(task, sam3=sam3, room=F.FakeRoom(task, horizon=horizon))
    rec = Records(out, session.info, name=session.record_name())
    pictures = []
    session.open(Settings(sam3_url="fake" if sam3 else ""), rec, lambda kind, image, caption: pictures.append(caption))
    return session, rec, pictures, out


def test_simple_host_moves():
    import fakes_simple as F

    session, rec, pictures, out = _open()
    host, call = session.host, session.host.dispatch
    try:
        st = call("state", (), {})
        assert isinstance(st, R.HumanoidStatus) and st.step == 0 and st.step_limit == 2250 and st.official_steps == 400
        assert st.pelvis[:2] == (0.0, 0.0) and st.heading == 0 and st.left.fingers == "open" and st.gaze == (0.0, 45.0)
        assert "official limit is 400" in repr(st) and "left hand: palm" in repr(st), repr(st)
        info = call("task", (), {})
        assert info.instruction == "bend to pick up the cracker box" and info.official_steps == 400, info

        # body: within HITCH's ranges, the controller's floor noted
        with failed("ValueError") as f:
            call("body", (0.2,), {})
        assert "0.3 to 0.79" in f.exc.text, f.exc.text
        with failed("ValueError"):
            call("body", (), {})
        b = call("body", (0.40,), {"pitch": 20})
        assert isinstance(b, R.BodyMove) and not b.reached and abs(b.height - F.LOWEST) < 1e-6 and b.steps == 20, b
        assert any("no lower than about 0.54 m" in n for n in b.notes), b.notes
        b = call("body", (0.75,), {"pitch": 0})
        assert b.reached and b.height == 0.75

        # walk: to a point, by a step in its own axes, blocked by the table
        w = call("walk", (0.10, 0.0), {})
        assert isinstance(w, R.Walk) and w.reached and w.at == (0.1, 0.0) and w.steps == 10, w
        assert "from the target" not in repr(w), w
        near = R.Walk(reached=True, blocked=False, at=(0.1, 0.04), heading=0.0, target=(0.1, 0.0),
                      target_heading=0.0, err_mm=41.0, steps=200)  # AMO stops within about 5 cm
        assert "41 mm from the target" in repr(near), near
        w = call("walk", (), {"by": (0.0, 0.10), "turn": 90})
        assert w.reached and abs(w.at[0] - 0.10) < 1e-6 and abs(w.at[1] - 0.10) < 1e-6 and w.heading == 90, w
        w = call("walk", (), {"by": (0.10, 0.0)})  # facing +y now: forward is +y
        assert abs(w.at[1] - 0.20) < 1e-6 and abs(w.at[0] - 0.10) < 1e-6, w
        with failed("ValueError") as f:
            call("walk", (0.5,), {})
        assert "both x and y" in f.exc.text
        with failed("ValueError"):
            call("walk", (9.0, 0.0), {})
        with failed("ValueError"):
            call("walk", (), {"heading": 0, "turn": 10})
        w = call("walk", (0.5, -0.1), {"heading": 0})
        assert w.blocked and not w.reached and abs(w.at[0] - 0.30) < 1e-6 and "BLOCKED" in repr(w), w
        assert any("in the way" in n for n in w.notes), w.notes

        # the gaze: angles in range, or a point to look at
        g = call("gaze", (10, 50), {})
        assert isinstance(g, R.Gaze) and (g.yaw, g.pitch) == (10.0, 50.0) and g.steps == 5, g  # 10 degrees at 120 per s
        with failed("ValueError"):
            call("gaze", (60, 30), {})
        g = call("gaze", (), {"at": (0.62, -0.10, 0.80)})
        assert -45 <= g.yaw <= 0 and 20 < g.pitch < 60 and not g.notes, g
        g = call("gaze", (), {"at": (0.3, 3.0, 0.8)})
        assert g.yaw == 45 and any("outside its range" in n for n in g.notes), g

        # hands: a dry run, a move with a named facing, a move out of reach
        with failed("ValueError"):
            call("hand", ("middle", (0.5, 0, 1.0)), {})
        with failed("ValueError"):
            call("hand", ("left", (0.5, 0, 1.0)), {"facing": "sideways"})
        with failed("ValueError"):
            call("hand", ("left", (0.5, 0, 1.0)), {"seconds": 20})
        with failed("ValueError") as f:
            call("hand", ("left", (0.5, 0, 1.0)), {"normal": (1, 0, 0), "finger": (1, 0.1, 0)})
        assert "60 degrees" in f.exc.text
        dry = call("hand", ("right", (0.55, -0.12, 0.85)), {"dry_run": True})
        assert dry.dry_run and dry.reached and session.step == host.session.step, dry
        before = session.step
        m = call("hand", ("right", (0.55, -0.12, 0.85)), {"facing": "inward", "seconds": 0.6})
        assert isinstance(m, R.HandMove) and m.ok and m.err_mm < 1 and session.step > before, m
        assert np.allclose(host.proprio["palms"]["right"]["normal"], (0, 1, 0)), host.proprio["palms"]["right"]
        far = call("hand", ("left", (2.0, 0.5, 1.0)), {})
        assert not far.reached and any("joint limit" in n for n in far.notes), far
        assert "short of target" in repr(far)
        with failed("ValueError"):
            call("hand", ("left",), {"via": [(0.3, 0.2, 0.9)] * 12})

        # the budget and the rows of the records
        assert far.steps_left == 2250 - session.step
        states = [json.loads(line) for line in (rec.dir / "states.jsonl").read_text().splitlines()]
        assert len(states) == session.step and states[-1]["step"] == session.step, (len(states), session.step)
        assert set(states[-1]) >= {"pelvis", "heading", "pelvis_height", "torso_pitch", "left_palm", "right_palm"}
        assert pictures == [] and session.room.ops.count("observe") >= 1
        wt = call("wait", (5,), {})
        assert isinstance(wt, R.Waited) and wt.steps == 5
        with failed("ValueError"):
            call("wait", (500,), {})
        with failed("RobotError"):
            call("goto", ((0, 0, 1),), {})
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_simple_host_grasp_carry_and_success():
    import fakes_simple as F

    session, rec, pictures, out = _open()
    call = session.host.dispatch
    try:
        call("gaze", (), {"at": F.BOX_START})
        t = call("measure", ("red box",), {})
        assert t.found and np.allclose(t.center[:2], F.BOX_START[:2], atol=0.03), t
        call("walk", (0.25, -0.10), {})
        f = call("fingers", ("right", "close"), {})
        assert isinstance(f, R.Fingers) and f.closed and not f.holding and f.touching == [], f
        assert any("closed on nothing" in n for n in f.notes)
        w = call("fingers", ("right", "wide"), {})
        assert w.wide and not w.closed and "open wide" in repr(w), w
        assert call("state", (), {}).right.fingers == "wide"
        side = (F.BOX_START[0] - F.BOX_SIZE[0] / 2 - 0.02, F.BOX_START[1], F.BOX_START[2])
        m = call("hand", ("right", side), {"facing": "forward"})
        assert m.reached, m
        f = call("fingers", ("right", True), {})
        assert f.holding and f.touching == ["thumb", "index", "middle"] and "holding" in repr(f), f
        st = call("state", (), {})
        assert st.right.holding and st.right.fingers == "closed"
        # carried: the measured box moves with the hand; the lift ends the episode with success
        with failed("ValueError"):
            call("fingers", ("right", "maybe"), {})
        with raises(Stop):
            call("hand", ("right",), {"by": (0, 0, 0.10), "hold": "pelvis"})
        assert session.over and session.verdict["success"] and "SUCCESS" in session.over_text()
        fields = session.result_fields(session.verdict)
        assert fields["first_success_step"] == session.status["first_success_step"] > 0 and fields["official_success"]
        assert fields["split"] == "learning" and fields["seed"] == 1001 and fields["task"] == "bend_pick", fields
        with raises(Stop):
            call("state", (), {})
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_simple_host_measure_moved_note():
    """A re-measured Thing is said to have moved only when its top or its ground place moved: from near and
    high the one camera sees only the top, and the centre of what it sees rises."""
    import dataclasses

    session, rec, pictures, out = _open()
    host = session.host
    try:
        t = host.dispatch("measure", ("red box",), {})
        x, y, z = t.center_xyz
        seen_from_above = dataclasses.replace(t, center_xyz=(x + 0.007, y, z + 0.075),
                                              notes=["moved 7.5 cm since it was measured"])
        host.scene.measure = lambda target, cams="auto": dataclasses.replace(seen_from_above, notes=list(seen_from_above.notes))
        again = host._measure(t)
        assert not any(n.startswith("moved") for n in again.notes), again.notes
        pushed = dataclasses.replace(t, center_xyz=(x + 0.03, y, z), notes=["moved 3.0 cm since it was measured"])
        host.scene.measure = lambda target, cams="auto": dataclasses.replace(pushed, notes=list(pushed.notes))
        assert host._measure(t).notes[0] == "moved 3.0 cm since it was measured"
        # a first measurement (words) keeps whatever the scene says
        host.scene.measure = lambda target, cams="auto": dataclasses.replace(seen_from_above, notes=list(seen_from_above.notes))
        assert host._measure("red box").notes[0].startswith("moved")
    finally:
        session.close()


def test_simple_host_touch_on_the_way_note():
    """An open hand that went through something on its way and touches nothing at the end is told so: it may
    have tipped it (a model's hand knocked the box over with no word about it)."""
    session, rec, pictures, out = _open()
    call = session.host.dispatch
    try:
        call("walk", (0.25, -0.10), {})
        before = call("hand", ("right", (0.55, -0.10, 0.85)), {"facing": "down"})
        assert not any("touch" in n for n in before.notes), before.notes
        through = call("hand", ("right", (0.70, -0.10, 0.85)), {"facing": "down"})
        assert any("touched something on the way (index, middle)" in n for n in through.notes), through
        still = call("wait", (5,), {})
        assert not any("touch" in n for n in still.notes), still.notes
        # fingers that touched something while closing and nothing at the end may have pushed it away
        room = session.room
        plain = room.op_fingers

        def pushing(rows, side, closed):
            res = plain(rows, side, closed)
            room.touched[side].add("thumb")
            return res
        room.op_fingers = pushing
        call("hand", ("right", (0.70, -0.10, 1.0)), {})  # clear of the box: closing grips nothing
        f = call("fingers", ("right", True), {})
        assert not f.holding and any("while closing they touched something (thumb)" in n for n in f.notes), f
    finally:
        session.close()


def test_simple_host_slip_and_budget_without_sam3():
    import fakes_simple as F

    session, rec, pictures, out = _open(sam3=False, horizon=300)
    call = session.host.dispatch
    try:
        with failed("ValueError") as f:
            call("measure", ("red box",), {})
        assert "no SAM3" in f.exc.text
        call("walk", (0.25, -0.10), {})
        side = (F.BOX_START[0] - F.BOX_SIZE[0] / 2 - 0.02, F.BOX_START[1], F.BOX_START[2])
        call("hand", ("right", side), {})
        assert call("fingers", ("right", True), {}).holding
        # the box slips out (the fake lets go of it): the next motion says so
        session.room.carried = None
        session.room.box = F.BOX_START + np.array([0.0, 0.3, 0.0])
        wt = call("wait", (2,), {})
        assert any("slipped out" in n for n in wt.notes) and not session.host.holding["right"], wt
        with raises(Stop):
            call("wait", (400,), {})
        assert session.over and not session.verdict["success"] and "budget is used up" in session.over_text()
        assert not session.result_fields(session.verdict)["official_success"]
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_simple_card():
    session, rec, pictures, out = _open()
    try:
        text, used = session.card(Settings(sam3_url="fake"))
        assert "Task: bend_pick (SIMPLE, Unitree G1 humanoid)" in text and "2250 steps (45 s" in text, text
        assert "official limit is 400 steps (8 s)" in text and "Picture 1 below: the gaze camera" in text, text
        assert used == {"task": "bend_pick", "skill": False}
        assert [c for c, _ in session.start_pictures()] == ["gaze"]
        assert session.start_pictures()[0][1].shape == (P.GAZE_HEIGHT, P.GAZE_WIDTH, 3)
        assert session.labels() == {"task": "bend_pick", "seed": 1001, "layout": "learning"}
        assert session.record_name() == "bend_pick_s1001"
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


# -- a scripted episode and the registration -----------------------------------------------------------
def test_scripted_simple_episode():
    import fakes
    import fakes_simple as F
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-simple-run-"))
    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nprint(robot.gaze(at=(0.62, -0.1, 0.8)))\n"
                           "robot.look('gaze')\nt = robot.measure('red box')\nprint(t)")],
        [fakes.python_call("print(robot.walk(0.25, -0.10))\n"
                           "print(robot.hand('right', (t.center[0] - 0.05, t.center[1], t.center[2]), "
                           "facing='forward'))\nf = robot.fingers('right', True)\nprint(f)\nassert f.holding")],
        [fakes.python_call("print(robot.hand('right', by=(0, 0, 0.1), hold='pelvis'))\nprint('not reached')")],
    ]
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        session = F.FakeSimpleSession()
        settings = Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30)
        ep = episode.Episode(session, settings, out)
        ep.run()
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    folder = ep.rec.dir
    result = json.loads((folder / "result.json").read_text())
    assert result["bench"] == "simple" and result["success"] is True and result["official_success"] is True, result
    assert result["end"] == "episode_over" and result["turns"] == 3 and result["split"] == "learning", result
    assert result["first_success_step"] > 0 and result["official_steps"] == 400, result
    assert result["robot_calls"]["hand"] == 2 and result["robot_calls"]["measure"] == 1, result["robot_calls"]
    assert not result["errors"], result["errors"]
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert len(cells) == 3 and not cells[1]["error"], cells[1]
    assert "Thing#1 'red box'" in cells[0]["output"] and "Fingers right: closed, holding" in cells[1]["output"], cells
    assert "not reached" not in cells[2]["output"] and cells[2]["stopped"]
    messages = fakes.ScriptedGateway.seen[-1]
    system = messages[0]["content"]
    assert system.startswith("# 1. Your job and the rules") and "humanoid" in system
    card = messages[1]["content"]
    card_text = card if isinstance(card, str) else " ".join(p.get("text", "") for p in card if isinstance(p, dict))
    assert "Task: bend_pick (SIMPLE, Unitree G1 humanoid)" in card_text and "You have up to 10 replies" in card_text
    shutil.rmtree(out)


def test_simple_run_registration():
    from hexashell.benches import run

    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    try:
        cfg = str(ROOT / "config" / "benches.yml")
        s = run.settings_for("simple", "handover", cfg, [])
        assert s.max_turns == P.REPLIES and s.budget_s == P.EPISODE_S and not s.ladder
        with raises(ValueError):
            run.settings_for("simple", "bend_pick_hard", cfg, [])
        sess = run.make_session("simple", "xmove_pick", 2001, python="/x/python", env_file=None, egl="3")
        assert sess.bench == "simple" and sess.server_timeout_s == 1800.0 and sess.egl_device == "3"
        assert sess.open_args() == {"task": "xmove_pick", "seed": 2001, "renderer": None}
        with raises(ValueError):
            run.make_session("simple", "xmove_pick", 7, python="/x/python", env_file=None, egl=None)
    finally:
        os.environ.clear()
        os.environ.update(saved)
