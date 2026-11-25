"""Checks of the LIBERO-Pro 2.0 adapter that need neither its simulator nor the model: the protocol (splits,
names, the leaderboard's scoring) and the episode side against LIBERO-PRO's fake table (``tests/fakes_libero.py``)
with the perturbation case kept from the model and an instruction that changes during the episode, and with the
pictures coming without depth (the robot's parts predicted, the colour SAM3, the server's look_from).
Run: ``python tests/test_liberopro2.py`` (or with pytest) from the checkout, in the policy environment.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import numpy as np  # noqa: E402

import fakes_libero as L  # noqa: E402
from hexashell.benches.liberopro2 import protocol as P  # noqa: E402
from hexashell.kernel.host import CallFailed, Stop  # noqa: E402
from hexashell.benches.liberopro2.session import LiberoPro2Session  # noqa: E402
from hexashell.runtime.episode import Settings  # noqa: E402
from hexashell.runtime.records import Records  # noqa: E402

TASK = "libero_goal_t8_D21"  # an official training task: dev
RGB_TASK = "libero_goal_t3_BASE"  # a test instance; seed 1 is a learning start
CHANGED = "Please continue: pick up the red box"
#: the fake table's top as the nominal model would give it
TABLE = {"z": L.TABLE_TOP, "floor": False, "geoms": ["table"], "x": (0.15, 1.0), "y": (-0.6, 0.6)}


def raises(kind, text=None):
    class Check:
        def __enter__(self):
            return self

        def __exit__(self, etype, exc, tb):
            assert etype is not None and issubclass(etype, kind), f"{kind.__name__} not raised"
            assert text is None or text in str(exc), f"{text!r} not in {exc!r}"
            return True

    return Check()


class FakeTable2(L.FakeTable):
    """LIBERO-PRO's fake table with LIBERO-Pro 2.0's facts and ops (the robot's parts, look_from); the instruction
    changes from step ``change_at``."""

    def __init__(self, *args, change_at: int | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.change_at = change_at
        self.opened: dict = {}
        #: how many look_from poses to call out of reach before the reach check decides
        self.refuse_looks = 0
        #: what the camera check's fit finds (None: the start's check finds the cameras right)
        self.calib_found: dict | None = None

    def status(self) -> dict:
        changed = self.change_at is not None and self.steps >= self.change_at
        return {**super().status(), "instruction": CHANGED if changed else "pick up the red box"}

    def handle(self, op: str, args: dict) -> dict:
        if op != "open":
            return super().handle(op, args)
        self.ops.append(op)
        self.opened = dict(args)
        self.reset()
        suite, index, case = P.parse_task(args["task"])
        return {"facts": {"task": args["task"], "suite": suite, "task_index": index, "case": case,
                          "kind": P.kind(case), "seed": args["seed"], "init_state": args["init_state"],
                          "instruction": "pick up the red box", "robot": "MountedPanda", "horizon": self.horizon,
                          "step_s": P.STEP_S, "settle_steps": P.SETTLE_STEPS, "depth": bool(args["depth"]),
                          "table": dict(TABLE)},
                "start": self.proprio(), "seconds": 0.01, "level": self.level,
                "record": {"plan": {"case": case}, "plan_fingerprint": "fake"}}

    def op_observe(self, cams=None, robot_mask=True, parts=False) -> dict:
        out = super().op_observe(cams, robot_mask)
        if parts:  # the hand: the fingers' number
            out["obs"]["parts"] = {cam: np.where(m, P.ROBOT_PARTS.index("fingers") + 1, 0).astype(np.uint8)
                                   for cam, m in out["obs"]["robot"].items()}
        return out

    def op_calib_check(self, mask) -> dict:
        assert self.steps == 0 and np.asarray(mask).dtype == bool
        need = self.calib_found is not None
        return {"found": {"need": need, "roll": [0, 0], "gain": 0.3 if need else 0.01}}

    def op_calib_move(self) -> dict:
        """The hand 0.15 m to the right and 0.05 m up, in 10 steps."""
        start = self.tcp_base.copy()
        poses = []
        for k in range(1, 11):
            pose = start.copy()
            pose[:3, 3] += np.array([0.0, -0.15, 0.05]) * k / 10
            poses.append(pose)
        rows, _ = self._run(poses)
        image = self.op_observe(["agentview"])["obs"]["images"]["agentview"]
        return {**self._reply({"moved": True}, rows), "images": {str(self.steps): image}}

    def op_calib_fit(self, masks) -> dict:
        assert set(masks) == {str(self.steps)}
        return {"found": {"need": True, "move": {"moved": True, "steps": 10}, **self.calib_found}}

    def op_look_from(self, pose, cams=None) -> dict:
        pose = np.asarray(pose, float)
        verdict = self._check(pose)
        if self.refuse_looks > 0:
            self.refuse_looks -= 1
            verdict = {"ok": False, "why": "out of reach (the test's)"}
        out = {"reaches": bool(verdict["ok"]), "why": verdict.get("why")}
        if out["reaches"]:
            saved, self.tcp_base = self.tcp_base, pose.copy()
            try:
                obs = self.op_observe(cams, parts=True)["obs"]
            finally:
                self.tcp_base = saved
            out.update(cameras=obs["cameras"], parts=obs["parts"])
        return out


class LaggingTable(FakeTable2):
    """The fake table whose readings, once ``lag`` is set, lag the commands by so many steps and show the hand
    ``behind`` (m, along -x) from where it is: the server's proprio then has the hand as read too."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lag, self.behind = 0, 0.0

    def proprio(self) -> dict:
        out = super().proprio()
        if self.lag:
            seen = out["tcp"].copy()
            seen[0, 3] -= self.behind
            out.update(lag_steps=self.lag, seen_tcp=seen, seen_joints=np.ones(7))
        return out


class FakeSam3(L.FakeSam3):
    WORDS = {"red": L.C.BOX, "work surface": L.C.FLOOR}  # (the fake draws its table top, at z = 0, as floor)


class FakeSession(LiberoPro2Session):
    def __init__(self, task: str = TASK, seed: int = 0, *, table: FakeTable2 | None = None, sam3: bool = False):
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused", size=256)
        self.kitchen = table or FakeTable2(size=256, horizon=P.max_steps(task))
        self.sam3 = FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = L.C.FakeLink(self.kitchen)

    def _scene(self, settings):
        if self.sam3 is None or self.depth:
            return None
        from hexashell.benches.liberopro2.scene import LiberoScene

        return LiberoScene("fake", log=self.rec.event, client=self.sam3)


def _open(task=TASK, change_at=None, seed=0, sam3=False, table=None):
    out = Path(tempfile.mkdtemp(prefix="hexashell-liberopro2-"))
    table = table or FakeTable2(size=256, horizon=P.max_steps(task), change_at=change_at)
    session = FakeSession(task, seed, table=table, sam3=sam3)
    rec = Records(out, session.info, name=session.record_name())
    session.open(Settings(sam3_url="fake" if sam3 else ""), rec, lambda kind, image, caption: None)
    return session, rec, out


def test_protocol_splits():
    sizes = {split: len(P.instances(split)) for split in P.SPLITS}
    assert sizes == {"learn": 339, "validation": 339, "test": 339, "revalidation": 339,
                     "revalidation2": 339}, sizes  # all on the test set
    assert len(set(P.instances("test"))) == 339 and P.instances("learn") == P.instances("test")
    assert [len(P.episodes(s)) for s in ("learn", "validation", "test")] == [1695, 1695, 6780]
    assert P.OFFICIAL_SEEDS == tuple(range(142, 172)) and P.TEST_SEEDS == tuple(range(152, 172))
    assert P.LEARN_SEEDS + P.VALIDATION_SEEDS + P.TEST_SEEDS == P.OFFICIAL_SEEDS
    assert P.REVALIDATION_SEEDS == tuple(range(172, 182)) and len(P.episodes("revalidation")) == 3390
    assert P.REVALIDATION2_SEEDS == tuple(range(182, 192)) and len(P.episodes("revalidation2")) == 3390
    seeds = [set(P.SPLITS[s]) for s in ("learn", "validation", "test", "revalidation", "revalidation2")]
    assert all(not (seeds[i] & seeds[j]) for i in range(5) for j in range(i + 1, 5))
    assert not set(P.MORE_LEARN_SEEDS) & (seeds[1] | seeds[2] | seeds[3] | seeds[4])
    for suite in P.SUITES:
        train, test = set(P.TRAIN_TASKS[suite]), set(P.TEST_TASKS[suite])
        assert len(train) == 8 and len(test) == 2 and not (train & test) and train | test == set(range(10))
    task = "libero_goal_t3_S15"
    assert [P.split_of(task, s) for s in (1, 4, 49, 142, 146, 147, 151, 152, 171, 172, 181, 182, 191)] == [
        "learn", "learn", "learn", "learn", "learn", "validation", "validation", "test", "test",
        "revalidation", "revalidation", "revalidation2", "revalidation2"]
    assert [P.init_state_of(task, s) for s in (1, 5, 49, 142, 147, 152, 171, 172, 181, 182, 191)] == [
        1, 5, 49, 0, 0, 0, 0, 0, 0, 0, 0]
    assert P.split_of("libero_goal_t8_S15", 0) == "dev" and P.init_state_of("libero_goal_t8_S15", 0) == 0
    for seed in (0, 50, 141, 192):  # no split starts a test instance so
        with raises(ValueError):
            P.split_of(task, seed)
    assert len(P.STATIC) == 22 and len(P.DYNAMIC) == 20 and "D19" not in P.DYNAMIC
    grouped = [c for cases in P.CATEGORIES.values() for c in cases]
    assert sorted(grouped) == sorted(P.STATIC + P.DYNAMIC) and len(grouped) == len(set(grouped))
    assert P.max_steps("libero_spatial_t3_BASE") == 220 and P.max_steps("libero_10_t0_S01") == 520


def test_protocol_names():
    assert P.parse_task("libero_goal_t8_S15") == ("libero_goal", 8, "S15")
    for bad in ("libero_goal_t8", "libero_goal_t8_D19", "libero_goal_t10_BASE", "libero_spatial_t0_S04",
                "libero_goal_t0_S02"):
        with raises(ValueError):
            P.parse_task(bad)
    name = P.record_name("libero_goal_t8_S15", 3)
    assert name.startswith("libero_goal_t8_") and name.endswith("_s3") and "S15" not in name
    assert name == P.record_name("libero_goal_t8_S15", 3) != P.record_name("libero_goal_t8_S16", 3)
    assert P.kind("BASE") == "base" and P.kind("S21") == "static" and P.kind("D21") == "dynamic"
    assert P.category("D20") == "robot" and P.category("BASE") is None


def test_protocol_score():
    outcomes = [("libero_goal_t8_BASE", True), ("libero_goal_t8_BASE", False),  # goal BASE 1/2
                ("libero_10_t0_BASE", True),  # 10 BASE 1/1
                ("libero_goal_t8_S15", False), ("libero_goal_t7_S15", True),  # one cell, two tasks pooled: 1/2
                ("libero_10_t0_S15", True),
                ("libero_goal_t8_D21", True)]
    s = P.score(outcomes)
    assert s["base"] == 0.75  # mean of the BASE cells that ran
    assert s["cells"]["libero_goal/S15"] == {"successes": 1, "episodes": 2, "rate": 0.5}
    assert s["static"]["score"] == 0.75 and s["static"]["cells"] == 2 and s["static"]["delta"] == 0.0
    assert s["static"]["categories"]["observation"]["score"] == 0.75
    assert s["static"]["categories"]["language"]["score"] is None  # no cell: left out, not 0
    assert s["dynamic"]["score"] == 1.0 and abs(s["dynamic"]["delta"] - 0.25) < 1e-12
    assert abs(s["overall"]["score"] - 2.5 / 3) < 1e-12 and s["overall"]["cells"] == 3


def test_run_settings():
    from hexashell.benches import run

    assert "liberopro2" in run.BENCHES
    assert run.bench_defaults("liberopro2", TASK) == {"max_turns": P.REPLIES, "budget_s": P.EPISODE_S,
                                                      "ladder": False}
    with raises(ValueError):
        run.bench_defaults("liberopro2", "libero_goal_t8")
    session = run.make_session("liberopro2", TASK, 0, python="unused", env_file=None, egl=None)
    assert isinstance(session, LiberoPro2Session) and session.case == "D21" and session.prompt_folder() == "liberopro2"


def test_card_keeps_the_case_from_the_model():
    session, rec, out = _open()
    try:
        text, used = session.card(Settings(skills=False))
        assert "Instruction (from the benchmark; it is authoritative): 'pick up the red box'" in text
        assert "Task: a libero_goal task of LIBERO-Pro 2.0" in text and "Step budget: 300 steps" in text
        assert "D21" not in text and "t8" not in text
        assert used == {"task": TASK, "skill": False, "notes": [], "depth": False, "table_z": L.TABLE_TOP}, used
        assert "The table top the objects stand on: z = 0.000 (the robot's model of the scene)." in text
        assert "D21" not in session.record_name() and "D21" not in rec.dir.name
        info = session.host.dispatch("task", (), {})
        assert info.task_name == "libero_goal" and info.instruction == "pick up the red box" and info.step_limit == 300
        fields = session.result_fields({})
        assert (fields["case"], fields["kind"], fields["category"], fields["split"]) == ("D21", "dynamic", "language",
                                                                                         "dev")
        assert (fields["init_state"], fields["depth"], fields["table_z"]) == (0, False, L.TABLE_TOP)
        assert session.labels() == {"task": TASK, "seed": 0, "suite": "libero_goal", "case": "D21"}
        assert session.cell_note() is None
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_card_notes_by_suite_and_base_task(monkeypatch, tmp_path):
    from hexashell.benches.liberopro2 import session as S

    (tmp_path / "common.md").write_text("all suites\n")
    (tmp_path / "libero_goal_t8.md").write_text("this base task\n")
    (tmp_path / "libero_goal_t3.md").write_text("another base task\n")
    monkeypatch.setattr(S, "SKILLS", tmp_path)
    session, rec, out = _open()
    try:
        text, used = session.card(Settings(skills=True))
        assert "## Notes\n\nall suites" in text and "## Notes for this task\n\nthis base task" in text, text
        assert "another base task" not in text and "Notes for libero_goal" not in text and "D21" not in text
        assert used["notes"] == ["common", "libero_goal_t8"] and used["skill"], used
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_instruction_change_is_told_once():
    session, rec, out = _open(change_at=3)
    try:
        session.card(Settings(skills=False))
        session.host.dispatch("goto", (), {"by": (0, 0, -0.01)})
        assert session.step >= 3 and session.instruction == CHANGED
        note = session.cell_note()
        assert note and repr(CHANGED) in note, note
        assert session.cell_note() is None
        assert session.host.dispatch("task", (), {}).instruction == CHANGED
        fields = session.result_fields({})
        assert fields["instruction"] == "pick up the red box" and fields["instruction_final"] == CHANGED
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_prompt_folder():
    from hexashell.runtime import card

    text = card.system_prompt("liberopro2")
    heads = [line for line in text.splitlines() if line.startswith("# ")]
    assert [h.split(".")[0] for h in heads] == ["# 1", "# 2", "# 3", "# 4", "# 5", "# 6"], heads
    assert "LIBERO-Pro 2.0" in text and "'agentview' and 'wrist'" in text and "left of the agentview picture" in text
    assert "steps of 0.125" in text and "left out of measurements" not in text and "robot.depth(cam)" in text
    rgb = card.system_prompt("liberopro2", variant="rgb")
    assert [line.split(".")[0] for line in rgb.splitlines() if line.startswith("# ")] == [h.split(".")[0] for h in heads]
    assert "There are no depth pictures" in rgb and "the task card gives the height" in rgb
    assert "pixels, where the robot's model of itself puts them, are left out" in rgb and "robot.depth" not in rgb
    assert "from the depth" not in rgb and "on the depth picture" not in rgb and "depth points" not in rgb
    assert "LIBERO-Pro 2.0" in rgb and "steps of 0.125" in rgb


def test_open_args():
    for seed, start in ((1, 1), (5, 5), (150, 0)):
        session = FakeSession(RGB_TASK, seed)
        assert session.open_args() == {"task": RGB_TASK, "seed": seed, "init_state": start, "depth": False}
    session.depth = True
    assert session.open_args()["depth"] is True and session.prompt_variant() is None


def test_building_tried_again_when_the_objects_could_not_be_placed(monkeypatch):
    """robosuite places the objects once while it builds the environment, before any seed; when that fails, the
    environment is built again, up to BUILD_ATTEMPTS times."""
    import types

    from hexashell.benches.liberopro2 import sim as S

    class RandomizationError(Exception):
        pass

    built = []

    def env(**kw):
        built.append(kw)
        if len(built) <= fails:
            raise RandomizationError("Cannot place all objects ):")
        return "env"

    mods = {"libero": types.ModuleType("libero"), "libero.libero": types.ModuleType("libero.libero"),
            "libero.libero.envs": types.ModuleType("libero.libero.envs"), "robosuite": types.ModuleType("robosuite"),
            "robosuite.utils": types.ModuleType("robosuite.utils"),
            "robosuite.utils.errors": types.ModuleType("robosuite.utils.errors")}
    mods["libero.libero.envs"].OffScreenRenderEnv = env
    mods["robosuite.utils.errors"].RandomizationError = RandomizationError
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)
    fails = 2
    assert S.make("task.bddl", 512) == "env" and len(built) == 3
    assert built[-1]["bddl_file_name"] == "task.bddl" and built[-1]["camera_heights"] == 512
    built.clear()
    fails = S.BUILD_ATTEMPTS
    try:
        S.make("task.bddl")
        raise AssertionError("no error after the last attempt")
    except RandomizationError:
        assert len(built) == S.BUILD_ATTEMPTS


def test_scene_names():
    from hexashell.benches.liberopro2.scene import LiberoScene, LiberoView, rig_of
    from hexashell.vision.cameras import Camera

    parts = np.array([[0, 1, 2], [3, 0, 1]], np.uint8)  # nothing, the arm, the fingers, the body
    view = LiberoView("agentview", 0, Camera("agentview", 300.0, np.eye(4), 3, 2), np.zeros((2, 3, 3), np.uint8), {},
                      parts=parts)
    assert view.robot.tolist() == [[False, True, True], [True, False, True]]
    assert view.labels().tolist() == [[0, 1, 2], [0, 0, 1]]  # the body does not move aside
    assert view.arm_of(2) == "arm" and view.arm_mask("arm").tolist() == [[False, True, True], [False, False, True]]
    assert view.fingers("arm").tolist() == [[False, False, True], [False, False, False]]
    scene = LiberoScene.__new__(LiberoScene)  # (names need no SAM3)

    class Shape:
        yaw = None

    assert scene.grip_yaw(Shape) is None
    for yaw, grip in ((0.0, 0.0), (100.0, -80.0), (-95.0, 85.0), (90.0, 90.0), (-90.0, 90.0), (270.0, 90.0)):
        Shape.yaw = yaw
        assert scene.grip_yaw(Shape) == grip, (yaw, scene.grip_yaw(Shape))
    assert [scene.side_words(n) for n in ((1, 0.2), (-1, 0), (0.1, 1), (0, -1))] == [
        "+x (far)", "-x (near)", "+y (left)", "-y (right)"]
    assert scene.look_command("arm", (0.5, -0.1, 0.2), ("arm", None, 90, {}, 12)) == \
        "robot.goto((0.500, -0.100, 0.200), pitch=90, yaw=12, roll=0)"
    floor = rig_of({"z": 0.0, "floor": True, "geoms": ["floor"], "x": None, "y": None})
    table = rig_of(TABLE)
    assert floor.floor and not table.floor and table.table_x == (0.15, 1.0) and table.arms == ("arm",)


def test_without_depth():
    from hexashell.benches.liberopro2.scene import LiberoScene
    from hexashell.benches.robotwin.host import Shot

    session, rec, out = _open(RGB_TASK, seed=1, sam3=True)
    table, host = session.kitchen, session.host
    call = host.dispatch
    try:
        assert table.opened == {"task": RGB_TASK, "seed": 1, "init_state": 1, "depth": False, "size": 256}, table.opened
        assert isinstance(host.scene, LiberoScene) and session.prompt_variant() == "rgb"
        assert all(isinstance(v, Shot) for v in host.views().values()) and set(host.views()) == {"agentview", "wrist"}
        fields = session.result_fields({})
        assert (fields["split"], fields["init_state"], fields["depth"]) == ("learn", 1, False)
        try:
            call("depth", ("agentview",), {})
            raise AssertionError("depth pictures in a run without depth")
        except CallFailed as exc:
            assert "no depth pictures" in exc.text, exc.text

        # a spot on the bare table: where the pixel's ray meets the table top
        px = call("to_pixel", ((0.7, 0.2, L.TABLE_TOP), "agentview"), {})
        sp = call("to_world", ("agentview", px), {})
        assert np.allclose(sp.xyz, (0.7, 0.2, L.TABLE_TOP), atol=0.004) and "bare table" in str(sp), sp
        sp = call("to_world", ("agentview", px), {"z": 0.05})
        assert abs(sp.xyz[2] - 0.05) < 1e-6, sp

        # one view: the height is a guess, and the warning says where the wrist camera looks down on it
        t = call("measure", ("red box",), {"cams": ["agentview"]})
        m = re.search(r"robot\.goto\(\(([-\d.]+), ([-\d.]+), ([-\d.]+)\), pitch=(\d+), yaw=(-?\d+), roll=0\)",
                      t.warning or "")
        assert t.found and t.views == ["agentview"] and m, t
        assert call("goto", ([float(v) for v in m.groups()[:3]],), {"pitch": int(m.group(4)), "yaw": int(m.group(5)),
                                                                     "roll": 0}).ok
        top = np.array([t.center[0], t.center[1], t.top])
        wrist = host.views()["wrist"].camera
        x, y = wrist.pixel(top)
        assert int(m.group(4)) == 90 and abs(x - 127.5) < 3 and abs(y - 127.5) < 3, (m.group(0), x, y)
        assert abs(np.linalg.norm(top - wrist.position) - 0.20) < 0.005
        t = call("measure", (t,), {})
        assert t.views == ["agentview", "wrist"] and not t.warning and t.on == "table", t
        assert np.allclose(t.center, L.BOX_START, atol=0.004), t

        assert call("goto", ((t.center[0], t.center[1], t.top + 0.08),), {"pitch": 90, "yaw": t.grip_yaw,
                                                                         "grip": "open"}).ok
        assert call("goto", ((t.center[0], t.center[1], t.top - 0.02),), {}).ok
        assert call("gripper", ("close",), {}).holding
        try:
            call("goto", (), {"by": (0, 0, 0.12)})
            raise AssertionError("no success")
        except Stop as exc:
            assert "SUCCESS" in str(exc), exc
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_camera_check_leaves_a_right_start_alone():
    """The cameras check out at the start: no step taken, nothing on the card, every camera used."""
    session, rec, out = _open(RGB_TASK, seed=1, sam3=True)
    table, host = session.kitchen, session.host
    try:
        assert "calib_check" in table.ops and "calib_move" not in table.ops and table.steps == 0
        assert session.camera_check["need"] is False and host.untrusted == frozenset()
        text, _ = session.card(Settings(skills=False))
        assert "Camera check" not in text and "(used so far: 0)" in text, text
        assert session.result_fields({})["camera_check"] == {"need": False, "gain": 0.01}
        assert set(host.scene.views()) == {"agentview", "wrist"}
    finally:
        session.close()
        rec.close()
    events = [json.loads(line) for line in (rec.dir / "events.jsonl").read_text().splitlines()]
    assert [e["need"] for e in events if e.get("kind") == "camera_check"] == [False]
    shutil.rmtree(out)


def test_camera_check_without_the_wrist():
    """The check moved the hand and found the wrist camera off: the card says so, positions come from agentview
    alone, and the start's pictures are the ones after the move."""
    table = FakeTable2(size=256, horizon=P.max_steps(RGB_TASK))
    table.calib_found = {"agentview": "pose", "wrist": "untrusted", "fixes": {"agentview": {"roll": [0, 0]}}}
    session, rec, out = _open(RGB_TASK, seed=1, sam3=True, table=table)
    host, call = session.host, session.host.dispatch
    try:
        assert table.ops.count("calib_move") == 1 and table.steps == 10 and session.step == 10
        assert host.untrusted == {"wrist"} and set(host.scene.views()) == {"agentview"}
        assert all(v.step == 10 for v in host.views().values())
        text, _ = session.card(Settings(skills=False))
        assert "Camera check at the start" in text and "the hand moved for it (10 steps)" in text, text
        assert "fitted again" in text and "do not come from them" in text and "(used so far: 10)" in text, text
        assert session.result_fields({})["camera_check"]["wrist"] == "untrusted"
        for name, args, kwargs in (("to_world", ("wrist", (10, 10)), {}), ("to_pixel", ((0.6, 0.0, 0.0), "wrist"), {}),
                                   ("find", ("wrist", "red box"), {}), ("measure", ("red box",), {"cams": ["wrist"]})):
            with raises(CallFailed, "camera check"):
                call(name, args, kwargs)
        t = call("measure", ("red box",), {})
        assert t.found and t.views == ["agentview"], t
        assert np.allclose(call("to_world", ("agentview", call("to_pixel", ((0.7, 0.2, L.TABLE_TOP), "agentview"), {})),
                                {}).xyz, (0.7, 0.2, L.TABLE_TOP), atol=0.004)
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_camera_check_turned_off(monkeypatch):
    from hexashell.benches.liberopro2 import session as S

    monkeypatch.setenv(S.CAMERA_CHECK_ENV, "0")
    session, rec, out = _open(RGB_TASK, seed=1, sam3=True)
    try:
        assert "calib_check" not in session.kitchen.ops and session.camera_check is None
        assert session.result_fields({})["camera_check"] is None
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_look_pose_out_of_reach():
    """A look pose the arm does not reach is skipped (it is no failure of the call): the next one is offered, and
    with none reached the Thing says so and the episode goes on."""
    session, rec, out = _open(RGB_TASK, seed=1, sam3=True)
    table, call = session.kitchen, session.host.dispatch
    try:
        table.refuse_looks = 3  # pitch 90 at every turn
        t = call("measure", ("red box",), {"cams": ["agentview"]})
        m = re.search(r"robot\.goto\(\([-\d., ]+\), pitch=(\d+), yaw=-?\d+, roll=0\)", t.warning or "")
        assert t.found and m and int(m.group(1)) < 90 and table.refuse_looks == 0, t
        table.refuse_looks = 100
        t = call("measure", (t,), {"cams": ["agentview"]})
        assert t.found and "no wrist camera can look down on it" in (t.warning or ""), t
        assert not session.finished and session.failure is None
        assert call("goto", (), {"by": (0, 0, 0.02)}).ok
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_to_world_without_perception():
    session, rec, out = _open(RGB_TASK, seed=2)
    host = session.host
    call = host.dispatch
    try:
        assert host.scene is None and np.allclose(host.table_area(), ((0.1, 1.05), (-0.65, 0.65)))
        px = call("to_pixel", ((0.7, 0.2, L.TABLE_TOP), "agentview"), {})
        sp = call("to_world", ("agentview", px), {})
        assert np.allclose(sp.xyz, (0.7, 0.2, L.TABLE_TOP), atol=0.004), sp
        assert "assumes the pixel shows the table top" in sp.warning, sp
        ys, xs = np.nonzero(host.views()["agentview"].parts > 0)
        assert len(xs), "the hand is in the agentview picture"
        sp = call("to_world", ("agentview", (float(xs[len(xs) // 2]), float(ys[len(ys) // 2]))), {})
        assert "that pixel shows the robot" in sp.warning, sp
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_readings_lagging_the_commands():
    """While the readings lag the commands, perception goes by the hand as the readings and the pictures show it, and
    the results say how far that is from where the hand is (the pose they give, as predicted)."""
    from hexashell.benches.liberopro2.scene import ARM

    def lag_notes(result):
        return [n for n in result.notes if "lag the commands" in n]

    table = LaggingTable(size=256, horizon=P.max_steps(RGB_TASK))
    session, rec, out = _open(RGB_TASK, seed=1, sam3=True, table=table)
    host, call = session.host, session.host.dispatch
    try:
        move = call("goto", (), {"by": (0, 0, 0.02)})
        assert move.ok and not lag_notes(move) and host.snap.pictured_arms is None, move
        table.lag, table.behind = 7, 0.03
        move = call("goto", (), {"by": (0, 0, 0.02)})
        notes = lag_notes(move)
        assert move.ok and len(notes) == 1 and "by 7 steps" in notes[0] and "30 mm" in notes[0] \
            and "robot.wait(7)" in notes[0], move.notes
        snap, now = host.snap, host._tcp()
        pictured = snap.arms_in_pictures[ARM].flange
        assert np.allclose(snap.arms[ARM].flange, now) and snap.pictures_behind == 7
        assert np.allclose(pictured[:3, 3], now[:3, 3] - (0.03, 0, 0)) and np.allclose(pictured[:3, :3], now[:3, :3])
        assert np.allclose(host.scene._grasp_pose(ARM), pictured)
        assert all(np.allclose(v.arms[ARM].flange, pictured) for v in host.scene.views().values())
        assert len(lag_notes(call("look", ("wrist",), {}))) == 1
        table.behind = 0.002  # the pictures show the hand about where it is: nothing to say
        assert not lag_notes(call("goto", (), {"by": (0, 0, 0.02)}))
    finally:
        session.close()
        rec.close()
    shutil.rmtree(out)


def test_scripted_episode_tells_the_change():
    import fakes
    from hexashell.runtime import episode

    out = Path(tempfile.mkdtemp(prefix="hexashell-liberopro2-run-"))
    script = [[fakes.python_call("print(robot.task)\nprint(robot.goto(by=(0, 0, -0.02)))")],
              [fakes.python_call("print(robot.task)")]]
    saved = dict(os.environ)
    for key in [k for k in os.environ if k.startswith("HEXASHELL_")]:
        os.environ.pop(key)
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        session = FakeSession(table=FakeTable2(size=256, horizon=300, change_at=2))
        settings = Settings(sam3_url="", ladder=False, max_turns=4, budget_s=600, cell_cpu_s=20, cell_idle_s=30)
        ep = episode.Episode(session, settings, out)
        ep.run()
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    result = json.loads((ep.rec.dir / "result.json").read_text())
    assert result["bench"] == "liberopro2" and result["case"] == "D21" and result["instruction_final"] == CHANGED
    assert "D21" not in ep.rec.dir.name
    messages = fakes.ScriptedGateway.seen[-1]
    texts = [m["content"] if isinstance(m["content"], str) else " ".join(p.get("text", "") for p in m["content"]
                                                                          if isinstance(p, dict))
             for m in messages if m.get("role") in ("user", "tool")]
    seen = "\n".join(texts)
    assert "D21" not in seen and "LIBERO-Pro 2.0" in seen
    assert seen.count("The benchmark changed the instruction") == 1, seen
    assert f"Task: {CHANGED}" in seen
    shutil.rmtree(out)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
