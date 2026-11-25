"""DexJoCo-X (Franka arms with Wuji hands): the protocol, the server's pure parts, the host against the fake
scene, the card, a scripted episode and the registration in benches.run.  No simulator needed."""

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
from hexashell.benches.dexjoco_x import protocol as P  # noqa: E402
from hexashell.kernel.host import CallFailed, Stop  # noqa: E402
from hexashell.runtime.episode import Settings  # noqa: E402
from hexashell.runtime.records import Records  # noqa: E402

HAMMER_SEED = P.seeds("hammer_nail", "learning")[0]


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


@contextlib.contextmanager
def no_hexashell_environment(**extra):
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    os.environ.update(extra)
    try:
        yield
    finally:
        os.environ.clear()
        os.environ.update(saved)


# -- the protocol and the server's pure parts ------------------------------------------------------------
def test_dexjoco_x_protocol():
    assert P.tasks() == ["bimanual_microwave_cook", "bimanual_photograph", "bimanual_unlock_ipad", "hammer_nail",
                         "pick_bucket", "single_hand_hanoi"]
    assert P.arms("hammer_nail") == ("right",) and P.arms("single_hand_hanoi") == ("right",)
    assert P.arms("bimanual_photograph") == ("left", "right")
    assert P.cameras("hammer_nail") == ("scene", "right_wrist")
    assert P.cameras("bimanual_unlock_ipad") == ("scene", "left_wrist", "right_wrist")
    with raises(ValueError):
        P.check_task("hammer")
    assert P.speeds("hammer_nail") == {"hand": (0.15, 0.30), "turn": (25.0, 27.0)}
    assert P.speeds("bimanual_microwave_cook") == {"hand": (0.12, 0.18), "turn": (30.0, 40.0)}
    # splits: the first ten demonstration scenes learn, the next ten validate; Hanoi has its own scenes
    learn, val = P.seeds("pick_bucket", "learning"), P.seeds("pick_bucket", "validation")
    assert len(learn) == len(val) == 10 and not set(learn) & set(val) and learn[0] == 101165
    assert P.seeds("single_hand_hanoi", "learning")[0] == 82001
    assert P.split_of("pick_bucket", learn[3]) == "learning" and P.split_of("pick_bucket", val[9]) == "validation"
    assert P.split_of("single_hand_hanoi", 101165) == "development" and P.split_of("hammer_nail", 7) == "development"
    # the official test scenes: TEST_SEED + 1000 * the task's index + i
    assert P.test_seed("hammer_nail", 0) == 10_003_000 and P.test_seed("bimanual_microwave_cook", 149) == 10_000_149
    assert P.split_of("hammer_nail", 10_003_049) == "test" and P.split_of("hammer_nail", 10_003_149) == "test"
    with raises(ValueError):
        P.split_of("hammer_nail", 10_002_000)  # another task's test scene
    with raises(ValueError):
        P.split_of("hammer_nail", 10_003_150)
    with raises(ValueError):
        P.test_seed("hammer_nail", 150)
    with raises(ValueError):
        P.check_seed("hammer_nail", 10_003_000)
    assert P.check_seed("hammer_nail", 10_003_000, official_test=True) == 10_003_000
    assert P.check_seed("hammer_nail", HAMMER_SEED) == HAMMER_SEED
    assert P.official_test_allowed({P.OFFICIAL_TEST_VARIABLE: "1"})
    assert not P.official_test_allowed({P.OFFICIAL_TEST_VARIABLE: ""}) and not P.official_test_allowed({})
    episodes = P.test_episodes()
    assert len(episodes) == 6 * P.TEST_EPISODES == 300 and episodes[0] == ("bimanual_microwave_cook", 10_000_000)
    assert episodes[-1] == ("single_hand_hanoi", 10_005_049)
    # named palm orientations: 'inward' toward the robot's middle
    assert P.palm_axes("inward", "right")[0] == (0.0, 1.0, 0.0) and P.palm_axes("inward", "left")[0] == (0.0, -1.0, 0.0)
    assert P.palm_axes("down", "left") == ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    with raises(ValueError):
        P.palm_axes("sideways", "right")


def test_dexjoco_x_control_frames():
    from hexashell.benches.dexjoco_x import control as K

    for v in ([0.3, -0.2, 0.5], [0.0, 0.0, 2.0], [1e-9, 0.0, 0.0], [0.0, math.pi - 1e-3, 0.0]):
        R0 = K.rotation(np.array(v))
        assert np.allclose(R0 @ R0.T, np.eye(3), atol=1e-9) and abs(np.linalg.det(R0) - 1.0) < 1e-9
        assert np.allclose(K.rotvec(R0), v, atol=1e-6), (v, K.rotvec(R0))
    # frame_of: z the palm's normal, x the fingers; a part left out stays near the hand's now
    R0 = K.frame_of([0, 0, -1], [1, 0, 0], np.eye(3))
    assert np.allclose(R0, np.diag([1.0, -1.0, -1.0]))
    R1 = K.frame_of([0, 1, 0], None, R0)
    assert np.allclose(R1[:, 2], [0, 1, 0]) and np.allclose(R1[:, 0], [1, 0, 0]), R1
    R2 = K.frame_of(None, [0, 0, -1], R0)
    assert np.allclose(R2[:, 0], [0, 0, -1]) and abs(float(R2[:, 2] @ [0, 0, -1])) < 1e-9, R2
    # grip_frame: the handle lies across the fingers at the slant a fist holds it, the palm as near the hint
    for side in ("right", "left"):
        slant = K.GRIP_SLANT.copy()
        if side == "left":
            slant[1] = -slant[1]
        for along, hint in (([0, 1, 0], [0, 0, -1]), ([1, 0, 0], [0, 0, -1]), ([0.3, -0.8, 0.2], [0, 1, 0])):
            a = np.array(along, float) / np.linalg.norm(along)
            G = K.grip_frame(side, a, hint)
            assert np.allclose(G @ G.T, np.eye(3), atol=1e-9) and abs(np.linalg.det(G) - 1.0) < 1e-9
            assert np.allclose(G @ slant, a, atol=1e-9), (side, along)
            z = G[:, 2]
            h = np.array(hint, float) - (np.array(hint, float) @ a) * a
            for phi in np.linspace(-math.pi, math.pi, 25):  # no roll about the handle brings the palm nearer
                assert float(K.rotation(a * phi) @ z @ h) <= float(z @ h) + 1e-9
    # the shapes and the finger settings
    sim = type("Sim", (), {"hand_home": {"right": np.zeros(20)},
                           "hand_range": {"right": np.column_stack([np.full(20, -2.0), np.full(20, 2.0)])}})()
    assert np.allclose(K.shape_targets(sim, "right", "open"), 0.0)
    half = K.shape_targets(sim, "right", "power", 0.5)
    assert np.allclose(half, np.clip(np.asarray(K.SHAPE_TARGETS["power"]) * 0.5, -2, 2))
    q = K.finger_settings(sim, "right", np.zeros(20), curl={"middle": 1.0}, thumb=0.5, oppose=1.0, spread=1.0)
    assert np.allclose(q[8:12], [K.FINGER_CURL[0], K.SPREAD_RAD * K.SPREAD_SIGN["middle"], K.FINGER_CURL[2],
                                 K.FINGER_CURL[3]])
    assert np.allclose(q[1:4], K.THUMB_CURL[1:] * 0.5) and abs(q[0] - min(K.THUMB_OPPOSE_RAD, 2.0)) < 1e-9
    assert abs(q[5] - K.SPREAD_RAD) < 1e-9 and abs(q[13] + K.SPREAD_RAD) < 1e-9
    curl = K.measured_curl(K.finger_settings(sim, "right", np.zeros(20), curl={"index": 0.5}))
    assert curl["index"] == 0.5 and curl["middle"] == 0.0 and curl["thumb"] == 0.0, curl


def test_dexjoco_x_motion_start_and_rest():
    """A motion starts at the hand's reference, or where the hand is when a grasp pushed it further off its
    reference than a path waits for; an arm that comes no nearer is held up only once its joints are at rest."""
    from hexashell.benches.dexjoco_x import control as K

    ex = K.Executor.__new__(K.Executor)
    ref = np.eye(4)
    ex.ref = {"right": ref}
    for shift, turn_deg, rebased in ((0.010, 0.0, False), (0.0, 7.0, False), (0.020, 0.0, True), (0.0, 12.0, True)):
        T = np.eye(4)
        T[:3, :3] = K.rotation(np.array([0.0, 0.0, math.radians(turn_deg)]))
        T[:3, 3] = [shift, 0.0, 0.0]
        ex.sim = type("Sim", (), {"tcp": lambda self, side, T=T: T.copy()})()
        start = ex.path_start("right")
        assert np.allclose(start, T if rebased else ref), (shift, turn_deg)
        assert (start is ref) == (not rebased)
    still = [np.zeros(7) + 0.001 * k for k in range(K.STALL_STEPS + 1)]
    assert not K.Executor._moving(still) and not K.Executor._moving(still[:K.STALL_STEPS])
    moving = [np.zeros(7) for _ in range(K.STALL_STEPS)] + [np.r_[np.zeros(3), 0.03, np.zeros(3)]]
    assert K.Executor._moving(moving)


def test_dexjoco_x_finger_push_reading():
    """A finger presses on something when something outside the hand pushes on any of its four joints more than
    FINGER_PUSH_RAD, either way; the thumb and one more finger doing so is a hold."""
    from hexashell.benches.dexjoco_x import control as K
    from hexashell.benches.dexjoco_x import server as S

    assert P.FINGERS[0] == "thumb" and len(P.FINGERS) == 5
    assert K.pressing(np.zeros(20)) == []
    push = np.zeros(20)
    push[0 * 4 + 2] = -2 * K.FINGER_PUSH_RAD  # thumb, third joint, pushed open
    push[1 * 4 + 0] = K.FINGER_PUSH_RAD  # index: at the threshold, not over it
    push[2 * 4 + 3] = 1.5 * K.FINGER_PUSH_RAD  # middle, last joint
    push[4 * 4 + 1] = 0.5 * K.FINGER_PUSH_RAD  # little: under it
    assert K.pressing(push) == ["thumb", "middle"]
    assert S.holds(K.pressing(push))
    push[0 * 4 + 2] = 0.0
    assert K.pressing(list(push)) == ["middle"] and not S.holds(K.pressing(push))


def test_dexjoco_x_server_without_simulator():
    from hexashell.benches.armserver import Refused
    from hexashell.benches.dexjoco_x import server as S

    assert S.holds(["thumb", "middle"]) and S.holds(["thumb", "index", "ring"])
    assert not S.holds(["index", "middle"]) and not S.holds(["thumb"]) and not S.holds([])
    assert S.rounded([0.123456, 1.0]) == [0.1235, 1.0]
    assert np.allclose(S.unit([0, 0, 2], "normal"), [0, 0, 1])
    for bad in ([0, 0, 0], [1, 0], ["a", 0, 0], [math.nan, 0, 1]):
        with raises(Refused):
            S.unit(bad, "normal")
    with raises(Refused):
        S.point3([0.1, 0.2], "xyz")
    srv = S.Server()
    assert srv.handle("ping", {})["pid"] == os.getpid() and srv.status() == {}
    with raises(Refused):
        srv.handle("hand", {"side": "right", "xyz": [0, 0, 1]})
    with raises(Refused):  # a test scene without the run's permission: refused before any simulator starts
        srv.handle("open", {"task": "hammer_nail", "seed": 10_003_000})
    with raises(Refused):
        srv.handle("open", {"task": "hammer", "seed": HAMMER_SEED})
    assert srv.sim is None
    assert S.LOG_COLUMNS == 10



def test_dexjoco_x_dry_run_verdicts():
    """A dry run's three answers: reachable, at the edge of reach, NOT reachable (at the end, or on the way)."""
    from hexashell.benches.dexjoco_x import host as H

    base = dict(side="right", point="palm", at=(0.0, 0.0, 1.1), target=(0.0, 0.0, 1.1), dry_run=True)
    ok = R.DexterousMove(reached=True, err_mm=1.0, err_deg=0.5, **base)
    assert repr(ok).startswith("Hand right (dry run): reachable - the palm would end 1 mm / 0° from "
                               "(0.000, 0.000, 1.100)")
    edge = R.DexterousMove(reached=False, edge=True, err_mm=6.0, err_deg=3.0, notes=[H.edge_note([5])], **base)
    text = repr(edge)
    assert text.startswith("Hand right (dry run): at the edge of reach - the palm would end 6 mm / 3°"), text
    assert "joint 5 (" in text and "a real motion may stop short of it" in text and not edge.ok, text
    lost = R.DexterousMove(reached=False, err_mm=2.0, err_deg=1.0, lost_at=0.81, lost_mm=34.0, lost_deg=4.0,
                           notes=[H.lost_note([4])], **base)
    text = repr(lost)
    assert text.startswith("Hand right (dry run): NOT reachable - about 81% of the way to (0.000, 0.000, 1.100) the "
                           "arm would fall 34 mm / 4° behind the palm's path"), text
    assert "there the arm would have joint 4 (the elbow) at the end of its travel" in text, text
    assert "the arm cannot follow that path" in H.lost_note([])
    far = R.DexterousMove(reached=False, err_mm=33.0, err_deg=8.0, notes=[H.far_note([2, 6])], **base)
    text = repr(far)
    assert text.startswith("Hand right (dry run): NOT reachable - the palm would end 33 mm / 8°"), text
    assert "joint 2 (the shoulder) and joint 6 (the wrist) at the end of its travel" in text, text
    assert "out of the arm's reach" in text and "edge" not in text, text
    assert H.far_note([]).startswith("that pose is out of the arm's reach with the hand held this way")


# -- the host against the fake scene -----------------------------------------------------------------
def _open(task="hammer_nail", sam3=True, seed=None):
    import fakes_dexjoco_x as F

    out = Path(tempfile.mkdtemp(prefix="hexashell-dexjoco-x-"))
    session = F.FakeDexterousSession(task, seed, sam3=sam3)
    rec = Records(out, session.info, name=session.record_name())
    pictures = []
    session.open(Settings(sam3_url="fake" if sam3 else ""), rec, lambda kind, image, caption: pictures.append(caption))
    return session, rec, pictures, out


@contextlib.contextmanager
def opened(task="hammer_nail", sam3=True, seed=None):
    session, rec, pictures, out = _open(task, sam3, seed)
    try:
        yield session, session.host.dispatch
    finally:
        session.close()
        rec.close()
        shutil.rmtree(out)


def test_dexjoco_x_host_readings_and_refusals():
    with opened() as (session, call):
        st = call("state", (), {})
        assert isinstance(st, R.DexterousStatus) and st.step == 10 and st.step_limit == 1000 and st.left is None
        assert st.right.shape == "open" and not st.right.holding and st.right.pressing == []
        assert np.allclose(st.right.palm, (-0.348, 0.0, 1.4725), atol=1e-3) and st.right.normal == (0.0, 0.0, -1.0)
        assert "right hand: palm (-0.348, 0.000, 1.472) facing (0.000, 0.000, -1.000)" in repr(st), repr(st)
        assert "holding nothing (pressing on something: none)" in repr(st) and "tips: thumb (" in repr(st)
        info = call("task", (), {})
        assert isinstance(info, R.DexterousTask) and info.arms == ("right",) and info.steps_left == 990
        assert "Arms: right. Scene frame: metres, z up" in repr(info), repr(info)
        # refusals: they cost no steps
        for args, kwargs, words in (
                (("left", (0.0, 0.0, 1.2)), {}, "only the right arm"),
                (("thumb", (0.0, 0.0, 1.2)), {}, "side is 'right'"),
                (("right", (0.0, 0.0, 1.2)), {"point": "knuckle"}, "hand(point=...)"),
                (("right", (0.0, 0.0, 1.2)), {"facing": "sideways"}, "hand(facing=...)"),
                (("right", (0.0, 0.0, 1.2)), {"normal": (0, 0, -1), "finger": (0, 0.3, -1)}, "60 degrees apart"),
                (("right", (0.0, 0.0, 1.2)), {"along": (0, 1, 0), "finger": (1, 0, 0)}, "not both"),
                (("right", (0.0, 0.0, 1.2)), {"normal": (0, 0, 0)}, "not all zero"),
                (("right", (0.0, 0.0, 1.2)), {"approach": 0.3}, "0 to 0.25"),
                (("right", (0.0, 0.0, 1.2)), {"speed": 0.3}, "0.01 to 0.25 in this task (without it 0.15)"),
                (("right", (0.0, 0.0, 1.2)), {"speed": 0.001}, "0.01 to 0.25"),
                (("right", (1.5, 0.0, 1.2)), {}, "at most 1.2 m"),
                (("right", (0.0, 0.0)), {}, "three numbers"),
                (("right",), {}, "give xyz")):
            with failed("ValueError") as f:
                call("hand", args, kwargs)
            assert words in f.exc.text, (args, kwargs, f.exc.text)
        for args, kwargs, words in ((("right", "fist"), {}, "a shape is one of open, flat, power"),
                                    (("right",), {}, "give a shape and/or settings"),
                                    (("right", "power"), {"amount": 1.5}, "fingers(amount=...)"),
                                    (("right",), {"index": 1.3}, "0 (straight) to 1"),
                                    (("right",), {"spread": -2}, "-1 to 1"),
                                    (("right", "power"), {"seconds": 5}, "0.1 to 3 s")):
            with failed("ValueError") as f:
                call("fingers", args, kwargs)
            assert words in f.exc.text, (args, kwargs, f.exc.text)
        for args, kwargs, words in ((("right", (0, 0, 1), 0), {}, "-180 to 180 and not 0"),
                                    (("right", (0, 0, 1), 200), {}, "-180 to 180"),
                                    (("right", (0, 0, 1), 30), {"speed": 30}, "1 to 27 in this task"),
                                    (("right", (0, 0, 1), 30), {"axis": "w"}, "")):
            with failed("ValueError") as f:
                call("arc", args, kwargs)
            assert words in f.exc.text, (args, kwargs, f.exc.text)
        for n in (0, 401, "a"):
            with failed("ValueError") as f:
                call("wait", (n,), {})
            assert "1..400" in f.exc.text
        with failed("ValueError") as f:
            call("look", ("left_wrist",), {})
        assert "left_wrist" in f.exc.text or "camera" in f.exc.text, f.exc.text
        with failed("RobotError"):
            call("goto", ((0, 0, 1),), {})
        assert session.step == 10 and session.room.ops.count("hand") == 0


def test_dexjoco_x_host_moves():
    import fakes_dexjoco_x as F

    with opened() as (session, call):
        m = call("hand", ("right", (-0.2, 0.0, 1.3)), {})
        assert isinstance(m, R.DexterousMove) and m.reached and m.ok and m.err_mm < 1 and m.steps > 0, m
        assert repr(m).startswith("Hand right: palm at (-0.200, 0.000, 1.300)  ±0 mm 0°"), repr(m)
        # by: from where the point is now; a fingertip as the point; facing turns the hand on the way
        b = call("hand", ("right",), {"by": (0.05, 0.0, 0.0), "point": "middle"})
        assert b.reached and b.point == "middle", b
        st = call("state", (), {})
        assert np.allclose(st.right.tips["middle"], b.at, atol=1e-3)
        f = call("hand", ("right", st.right.palm), {"facing": "inward"})
        assert f.reached and call("state", (), {}).right.normal == (0.0, 1.0, 0.0), f
        # a dry run costs no steps; out of reach it says so and names the joint at its limit
        before = session.step
        d = call("hand", ("right", (0.3, -0.2, 1.1)), {"dry_run": True})
        assert d.dry_run and d.reached and session.step == before and "reachable" in repr(d), d
        far = call("hand", ("right", (0.6, -0.3, 1.0)), {"dry_run": True, "point": "grasp"})
        assert not far.reached and far.at_limit == [4] and "NOT reachable" in repr(far), far
        assert any("joint 4" in n for n in far.notes), far.notes
        stretched = call("hand", ("right", (0.6, -0.3, 1.0)), {"point": "grasp"})
        assert not stretched.ok and stretched.blocked and len(stretched.notes) == 1, stretched
        assert "the arm has joint 4 (the elbow) at the end of its travel" in stretched.notes[0], stretched.notes
        # the table holds the hand up
        down = call("hand", ("right", (0.0, 0.3, 0.80)), {"facing": "down"})
        assert down.blocked and not down.ok and "BLOCKED" in repr(down), down
        assert any("the motion stopped there" in n for n in down.notes), down.notes
        # an open hand that goes through the box is told it touched something on the way ...
        y, z = F.BOX_START[1], F.BOX_START[2]
        call("hand", ("right", (0.0, y, 1.1)), {"point": "middle", "facing": "down"})
        beside = call("hand", ("right", (0.0, y, z)), {"point": "middle"})
        assert beside.reached and not beside.notes, beside
        through = call("hand", ("right", (0.30, y, z)), {"point": "middle"})
        assert through.reached and len(through.notes) == 1, through
        assert "on the way something pushed on the right hand's fingers (index, middle)" \
            in through.notes[0], through
        quiet = call("wait", (3,), {})
        assert isinstance(quiet, R.Waited) and quiet.steps == 3 and not quiet.notes, quiet
        # ... and once when it ends pressing on something
        into = call("hand", ("right", (0.15, y, z)), {"point": "middle"})
        assert len(into.notes) == 1 and "the right hand presses on something now (index, middle)" \
            in into.notes[0], into
        assert call("state", (), {}).right.pressing == ["index", "middle"]
        still = call("wait", (3,), {})
        assert not still.notes, still
        # an arc about a vertical line through a point beside the palm
        call("hand", ("right", (-0.2, 0.0, 1.3)), {"facing": "down"})
        palm = np.array(call("state", (), {}).right.palm)
        a = call("arc", ("right", tuple(palm + (0.1, 0.0, 0.0)), 30), {})
        assert isinstance(a, R.DexterousMove) and a.turned == 30.0 and a.ok and a.err_mm < 2, a
        assert repr(a).startswith("Arc right: turned 30° of 30°"), repr(a)
        h = call("home", (), {})
        assert isinstance(h, R.DexterousHome) and h.ok and repr(h).startswith("Home: right home"), h
        assert np.allclose(call("state", (), {}).right.palm, (-0.348, 0.0, 1.4725), atol=2e-3)
        # to_pixel takes a move's point
        px = call("to_pixel", (m, "scene"), {})
        assert px is not None and all(0 <= v < 256 for v in px), px
        # the wrist camera as the point, turned to look at a point: straight down, the picture's top away from the
        # robot; the readings give the camera and its line of sight
        box = F.BOX_START
        v = call("hand", ("right", tuple(box + (0.0, 0.0, 0.4))), {"point": "camera", "look_at": tuple(box)})
        assert v.reached and v.point == "camera", v
        assert any("the right_wrist camera looks at (0.150, -0.250, 0.950) from 0.40 m; the top of its picture is "
                   "toward +x (away from the robot)" in n for n in v.notes), v.notes
        st = call("state", (), {}).right
        assert np.allclose(st.camera, box + (0.0, 0.0, 0.4), atol=2e-3) and np.allclose(st.view, (0, 0, -1), atol=2e-3)
        assert "wrist camera at (0.150, -0.250, 1.350) looking along (" in repr(st), repr(st)
        side_view = call("hand", ("right", tuple(box + (-0.3, 0.0, 0.3))), {"point": "camera", "look_at": tuple(box)})
        assert side_view.reached and any("top of its picture is toward (" in n for n in side_view.notes), side_view
        with failed("ValueError") as f:
            call("hand", ("right", (0.0, 0.0, 1.3)), {"look_at": tuple(box)})
        assert "give point='camera'" in f.exc.text, f.exc.text
        with failed("ValueError") as f:
            call("hand", ("right", (0.0, 0.0, 1.3)), {"point": "camera", "look_at": (0.0, 0.0, 1.25)})
        assert "at least 0.1 m from what it looks at" in f.exc.text, f.exc.text
        with failed("ValueError") as f:
            call("hand", ("right", (0.0, 0.0, 1.3)), {"point": "camera", "look_at": tuple(box), "facing": "down"})
        assert "look_at sets the hand's orientation" in f.exc.text, f.exc.text
        call("home", (), {})


def test_dexjoco_x_host_grasp_carry_and_success():
    import fakes_dexjoco_x as F

    with opened() as (session, call):
        t = call("measure", ("red box",), {})
        assert t.found and np.allclose(t.center[:2], F.BOX_START[:2], atol=0.04), t
        # closing far from the box holds nothing, and says so
        f = call("fingers", ("right", "power"), {})
        assert isinstance(f, R.DexterousFingers) and not f.holding and f.pressing == [], f
        assert any("closed without pressing on anything" in n for n in f.notes), f.notes
        o = call("fingers", ("right", "open"), {})
        assert o.shape == "open" and not o.notes, o
        half = call("fingers", ("right", "wrap"), {"amount": 0.5})
        assert half.shape == "wrap 0.50", half
        custom = call("fingers", ("right",), {"index": 0.2})
        assert custom.shape == "custom" and custom.asked["index"] == 0.2, custom
        call("fingers", ("right", "open"), {})
        # in from the side along the palm's normal, then close: the thumb and two fingers press on the box
        m = call("hand", ("right", t.center), {"point": "grasp", "facing": "inward", "approach": 0.08})
        assert m.reached and m.point == "grasp", m
        assert session.room.ops[-1] == "hand"
        g = call("fingers", ("right", "wrap"), {})
        assert g.holding and g.pressing == ["thumb", "index", "middle"], g
        assert repr(g).startswith("Fingers right: wrap, holding (pressing on something: thumb, index, middle)"), g
        assert call("state", (), {}).right.holding and session.host.holding["right"]
        # carried 2 cm up: the box goes with the hand, its measurement too
        up = call("hand", ("right",), {"by": (0.0, 0.0, 0.02)})
        assert up.reached and not up.notes, up
        again = call("measure", (t,), {})
        assert again.center[2] > t.center[2] + 0.01, (t, again)
        # the lift ends the episode with success
        with raises(Stop):
            call("hand", ("right",), {"by": (0.0, 0.0, 0.05)})
        assert session.over and session.verdict["success"] and "SUCCESS" in session.over_text()
        fields = session.result_fields(session.verdict)
        assert fields["split"] == "learning" and fields["hand"] == "wuji" and fields["task"] == "hammer_nail", fields
        assert fields["environment_ended"] is None and not fields["terminated"], fields
        with raises(Stop):
            call("state", (), {})


def test_dexjoco_x_host_slip_and_budget_without_sam3():
    import fakes_dexjoco_x as F

    with opened(sam3=False) as (session, call):
        with failed("ValueError") as f:
            call("measure", ("red box",), {})
        assert "no SAM3" in f.exc.text
        call("hand", ("right", F.BOX_START), {"point": "grasp", "facing": "inward", "approach": 0.08})
        assert call("fingers", ("right", "power"), {}).holding
        session.room.slip = True
        wt = call("wait", (2,), {})
        assert any("slipped out" in n for n in wt.notes) and not session.host.holding["right"], wt
        for _ in range(2):
            call("wait", (400,), {})
        with raises(Stop):
            call("wait", (400,), {})
        assert session.over and not session.verdict["success"] and "budget is used up" in session.over_text()


def test_dexjoco_x_two_arms():
    with opened("bimanual_photograph") as (session, call):
        assert session.host.sides == ("left", "right") and session.cams == ("scene", "left_wrist", "right_wrist")
        st = call("state", (), {})
        assert np.allclose(st.left.palm, (-0.348, 0.40, 1.4725), atol=1e-3)
        assert np.allclose(st.right.palm, (-0.348, -0.40, 1.4725), atol=1e-3)
        m = call("hand", ("left", (-0.2, 0.3, 1.3)), {"facing": "inward"})
        assert m.reached and call("state", (), {}).left.normal == (0.0, -1.0, 0.0), m
        with failed("ValueError") as f:
            call("hand", ("right", (-0.2, -0.3, 1.3)), {"speed": 0.2})
        assert "0.01 to 0.18 in this task (without it 0.12)" in f.exc.text, f.exc.text
        assert call("hand", ("right", (-0.2, -0.3, 1.3)), {"speed": 0.18}).reached
        h = call("home", ("left",), {})
        assert list(h.sides) == ["left"] and h.ok
        both = call("home", (), {})
        assert repr(both).startswith("Home: left home, right home"), repr(both)
        assert session.check_cam("left") == "left_wrist" and session.check_cam("wrist") == "right_wrist"
        image = call("image", ("left",), {})
        assert image.shape == (256, 256, 3)
        text, used = session.card(Settings(sam3_url="fake"))
        assert "Franka Panda arms with five-fingered Wuji hands; the left and right arms, fixed in place" in text, text
        assert "Hand motions go at 0.12 m/s unless asked otherwise (at most 0.18); arcs at 30 degrees per second " \
               "(at most 40)." in text, text
        assert "- left: palm (-0.348, 0.400, 1.472)" in text and "- right: palm (-0.348, -0.400, 1.472)" in text
        assert "Pictures 1-3 below: scene (fixed behind and above the robot" in text, text
        assert "left_wrist (on the left wrist, looking past the fingers" in text, text
        # the task's notes (skills/dexjoco_x/<task>.md) follow the card when skills are on
        noted, used = session.card(Settings(sam3_url="fake", skills=True))
        assert used == {"task": "bimanual_photograph", "skill": True} and noted.startswith(text)
        assert "\n## Notes for this task\n\n# bimanual_photograph\n" in noted


def test_dexjoco_x_card_and_names():
    with opened() as (session, call):
        text, used = session.card(Settings(sam3_url="fake"))
        assert text.startswith("# Task card\n\nInstruction (from the benchmark; it is authoritative): "
                               "'Pick up the red box.'"), text
        assert "Task: hammer_nail (DexJoCo-X: Franka Panda arm with five-fingered Wuji hand; the right arm, fixed " \
               "in place)" in text, text
        assert "Step budget: 1000 steps (50 s of simulated time; one step is 0.05 s) for the whole episode (used " \
               "so far: 10: the robot held still for the first 10 while the objects settled)." in text, text
        assert "Hand motions go at 0.15 m/s unless asked otherwise (at most 0.25); arcs at 25 degrees per second " \
               "(at most 27)." in text, text
        assert "- right: palm (-0.348, 0.000, 1.472), facing (0.000, 0.000, -1.000), fingers along (1.000, " \
               "0.000, 0.000); grasp point (" in text, text
        assert "Pictures 1-2 below: scene (fixed across the table, facing the robot (the robot's right is on the " \
               "picture's left)); right_wrist (on the right wrist" in text, text
        assert used == {"task": "hammer_nail", "skill": False}
        noted, used_noted = session.card(Settings(sam3_url="fake", skills=True))
        assert used_noted == {"task": "hammer_nail", "skill": True} and noted.startswith(text)
        assert "\n## Notes for this task\n\n# hammer_nail\n" in noted
        assert [c for c, _ in session.start_pictures()] == ["scene", "right_wrist"]
        assert session.start_pictures()[0][1].shape == (256, 256, 3)
        assert session.labels() == {"task": "hammer_nail", "seed": HAMMER_SEED, "split": "learning", "hand": "wuji"}
        assert session.record_name() == f"hammer_nail_s{HAMMER_SEED}"
        assert session.open_args() == {"task": "hammer_nail", "seed": HAMMER_SEED}
        assert session.room.opened_with == {"task": "hammer_nail", "seed": HAMMER_SEED, "official_test": False}
    with opened("pick_bucket") as (session, call):
        text, used = session.card(Settings(sam3_url="fake"))
        assert used == {"task": "pick_bucket", "skill": False}
        assert session.card(Settings(sam3_url="fake", skills=True)) == (text, used)  # no notes for this task
    # a test scene opens only when the run allows it
    import fakes_dexjoco_x as F

    with no_hexashell_environment():
        with raises(ValueError):
            F.FakeDexterousSession("hammer_nail", 10_003_000)
    with no_hexashell_environment(**{P.OFFICIAL_TEST_VARIABLE: "1"}):
        with opened("hammer_nail", seed=10_003_007) as (session, call):
            assert session.official_test and session.open_args()["official_test"] is True
            assert session.labels()["official_episode"] == 7 and session.labels()["split"] == "test"
            assert session.room.opened_with["official_test"] is True


def test_dexjoco_x_records_rows():
    with opened() as (session, call):
        call("hand", ("right", (-0.25, 0.0, 1.35)), {})
        session.rec.close()
        rows = [json.loads(line) for line in (Path(session.rec.dir) / "states.jsonl").read_text().splitlines()]
        assert rows and rows[-1]["step"] == session.step and "right_palm" in rows[-1] and "left_palm" not in rows[-1]
        assert rows[-1]["right_holding_physics"] is False and rows[-1]["success"] is False, rows[-1]


# -- a scripted episode and the registration -----------------------------------------------------------
def test_scripted_dexjoco_x_episode():
    import fakes
    import fakes_dexjoco_x as F
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-dexjoco-x-run-"))
    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nrobot.look('all', tile=True)\n"
                           "t = robot.measure('red box')\nprint(t)")],
        [fakes.python_call("m = robot.hand('right', t.center, point='grasp', facing='inward', approach=0.08)\n"
                           "print(m)\nf = robot.fingers('right', 'wrap')\nprint(f)\nassert f.holding")],
        [fakes.python_call("print(robot.hand('right', by=(0, 0, 0.08)))\nprint('not reached')")],
    ]
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        with no_hexashell_environment():
            session = F.FakeDexterousSession()
            settings = Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20,
                                cell_idle_s=30)
            ep = episode.Episode(session, settings, out)
            ep.run()
    finally:
        episode.Gateway = real
    folder = ep.rec.dir
    result = json.loads((folder / "result.json").read_text())
    assert result["bench"] == "dexjoco_x" and result["success"] is True, result
    assert result["end"] == "episode_over" and result["turns"] == 3 and result["split"] == "learning", result
    assert result["hand"] == "wuji" and result["task"] == "hammer_nail", result
    assert result["robot_calls"]["hand"] == 2 and result["robot_calls"]["measure"] == 1, result["robot_calls"]
    assert not result["errors"], result["errors"]
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    assert len(cells) == 3 and not cells[1]["error"], cells[1]
    assert "Thing#1 'red box'" in cells[0]["output"] and "Fingers right: wrap, holding" in cells[1]["output"], cells
    assert "not reached" not in cells[2]["output"] and cells[2]["stopped"]
    messages = fakes.ScriptedGateway.seen[-1]
    system = messages[0]["content"]
    assert system.startswith("# 1. Your job and the rules") and "Wuji" in system, system[:200]
    card = messages[1]["content"]
    card_text = card if isinstance(card, str) else " ".join(p.get("text", "") for p in card if isinstance(p, dict))
    assert "Task: hammer_nail (DexJoCo-X" in card_text and "You have up to 10 replies" in card_text
    shutil.rmtree(out)


def test_dexjoco_x_run_registration():
    from hexashell.benches import run

    with no_hexashell_environment():
        cfg = str(ROOT / "config" / "benches.yml")
        s = run.settings_for("dexjoco_x", "hammer_nail", cfg, [])
        assert s.max_turns == P.REPLIES and s.budget_s == P.EPISODE_S and not s.ladder
        with raises(ValueError):
            run.settings_for("dexjoco_x", "hammer", cfg, [])
        sess = run.make_session("dexjoco_x", "pick_bucket", P.seeds("pick_bucket", "validation")[0],
                                python="/x/python", env_file=None, egl="3")
        assert sess.bench == "dexjoco_x" and sess.egl_device == "3" and sess.size == P.IMAGE_SIZE
        assert sess.open_args() == {"task": "pick_bucket", "seed": P.seeds("pick_bucket", "validation")[0]}
        with raises(ValueError):
            run.make_session("dexjoco_x", "pick_bucket", 10_004_000, python="/x/python", env_file=None, egl=None)
        assert "dexjoco_x" in run.BENCHES
