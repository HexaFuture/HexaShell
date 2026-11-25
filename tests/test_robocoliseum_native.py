"""RoboColiseum Season II on the native runtimes (dexmanip: libuipc cloth tasks, Newton wok_flip, MuJoCo Rubik's
cube): the split guard, the commands of one case, our cloth config, and the link's replies per runtime."""

from __future__ import annotations

import dataclasses
import re
import shlex
import subprocess
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import msgpack
import numpy as np
import pytest
import yaml

from hexashell.benches.robocoliseum import body, kinematics, motion, wuji
from hexashell.benches.robocoliseum import link as L
from hexashell.benches.robocoliseum import obs as O
from hexashell.benches.robocoliseum import protocol as P
from hexashell.benches.robocoliseum import sim as S
from test_robocoliseum_wuji import WujiLink

WOK = "g2crsbwuji_dexmanip_wok_flip"
RUBIK = "g2crsbwuji_dexmanip_rubik_cube_solve"
FOLD = "g2crsbwuji_dexmanip_fold_cloth"
BAG = "g2crsbwuji_dexmanip_drawstring_bag"
ROOT = "/workspace/output/hexashell"


def _frame(seq: int = 1, joints=None, waist=None) -> O.Frame:
    joints = np.zeros(14) if joints is None else np.asarray(joints, float)
    waist = np.zeros(5) if waist is None else np.asarray(waist, float)
    return O.Frame(seq=seq, t=0.0, task_name="t", episode_idx=0, prompt="p", done=False, segments=[],
                   robot="g2_wuji", joints=joints, effectors=np.zeros(40), waist=waist, head=np.zeros(3),
                   chassis=None, end_pose=O.end_pose_from_joints("g2_wuji", joints, waist),
                   arm_base=body.arm_base(waist))


def _row() -> dict:
    return {"left": np.zeros(7), "right": np.zeros(7), "grip": {"left": np.zeros(20), "right": np.zeros(20)},
            "waist": np.zeros(5), "head": np.zeros(3)}


def test_case_of_is_the_split_guard():
    assert P.case_of(FOLD, 5, "learn") == P.Case(FOLD, 5)
    assert P.case_of(WOK, 0, "learn", 101) == P.Case(WOK, 0, seed=101)
    assert P.case_of(WOK, 3, "test") == P.Case(WOK, 3)
    rubik = P.case_of(RUBIK, 1, "learn", 1001)
    assert rubik == P.Case(RUBIK, 1, seed=1001, archive=P.RUBIK_OWN_ARCHIVE)
    assert rubik.name == f"{RUBIK}_rubik_2turn_10_seed1001_i01"
    assert P.case_of(RUBIK, 3, "validation", 1014).seed == 1014
    for bad in [(FOLD, 1, "learn", None), (WOK, 0, "learn", None), (WOK, 0, "validation", 101),
                (RUBIK, 0, "learn", 1001), (RUBIK, 0, "learn", None), (RUBIK, 0, "test", 99)]:
        with pytest.raises(P.TaskError):
            P.case_of(*bad)
    # our own cases never coincide with a scored one
    assert not set(P.WOK_OWN_SEEDS["learn"] + P.WOK_OWN_SEEDS["validation"]) & {P.task(WOK).seed}
    assert P.RUBIK_OWN_ARCHIVE != P.RUBIK_SCORED_ARCHIVE


def test_commands_per_runtime():
    fold = S.commands(P.Case(FOLD, 6), 9000, "r1", watchdog_s=1800)
    assert fold == [["geniesim", "benchmark", "run", f"{ROOT}/r1/{FOLD}_i06.yaml", "--infer-host=127.0.0.1:9000",
                     "--benchmark.num_episode=1", "--benchmark.num_instances=1"]]
    wok = S.commands(P.Case(WOK, 3), 9000, "r2", watchdog_s=1800)
    assert wok == [["geniesim", "benchmark", "generate-layouts", f"--output-dir={ROOT}/r2/wok_layouts", "--count=10",
                    "--seed=1398484294"],
                   ["geniesim", "benchmark", "run-layouts", f"--bundle={ROOT}/r2/wok_layouts", "--layout-ids=3",
                    "--infer-host=127.0.0.1:9000", f"--output-dir={ROOT}/r2/wok_flip/wok_flip",
                    "--benchmark.num_episode=1", f"--startup-timeout-s={S.LAYOUT_STARTUP_S}"]]
    own_wok = S.command(P.Case(WOK, 0, seed=101), 9000, watchdog_s=1800)
    assert own_wok[3:] == [WOK, "--infer-host=127.0.0.1:9000", "--benchmark.num_episode=1",
                           "--benchmark.num_instances=1", "--benchmark.seed=101"]
    assert S.instance_args(P.Case(RUBIK, 3)) == ["--benchmark.num_instances=1", "--benchmark.seed=93"]
    assert S.instance_args(P.Case(RUBIK, 1, seed=1001, archive=P.RUBIK_OWN_ARCHIVE)) == [
        "--rubik.bundled_scene_archive=rubik_2turn_10.zip", "--benchmark.num_instances=1", "--benchmark.seed=1001"]
    for bad in (P.Case(RUBIK, 1, seed=1002, archive=P.RUBIK_OWN_ARCHIVE), P.Case(WOK, 3), P.Case(FOLD, 0, seed=1)):
        with pytest.raises(ValueError):
            S.instance_args(bad)
    with pytest.raises(ValueError):
        S.commands(P.Case(WOK, 10), 9000, "r3")
    # no watchdog flag outside Isaac: the native runners refuse flags they do not know
    assert not any(a.startswith("--app.") for c in wok + fold for a in c)


def test_script_chains_the_wok_layout_steps():
    words = shlex.split(S.script(P.Case(WOK, 3), 9000, "r2", 51))
    assert f"WARP_CACHE_PATH={S.WARP_CACHE}" in words
    inner = words[-1]
    first, second = inner.split(" && ")
    assert first.startswith(f"echo $$ > {ROOT}/r2/pid; geniesim benchmark generate-layouts")
    assert second.startswith("exec geniesim benchmark run-layouts")


def test_cloth_config(tmp_path):
    src = P.PKG / "config" / f"{BAG}.yaml"
    if not src.exists():
        pytest.skip("no genie_sim source on this machine")
    original = yaml.safe_load(src.read_text())
    out = tmp_path / "bag.yaml"
    cfg = S.write_cloth_config(P.Case(BAG, 7), out)
    assert yaml.safe_load(out.read_text()) == cfg
    lib = cfg["libuipc"]
    assert lib["instance_base_xy_offsets"] == [original["libuipc"]["instance_base_xy_offsets"][7]]
    assert cfg["benchmark"]["num_instances"] == 1
    allowed = original["generalization"]["material"]["allowed_materials"]
    assert len(allowed) == 5 and cfg["generalization"]["material"]["allowed_materials"][0] == allowed[7 % 5]
    assert sorted(cfg["generalization"]["material"]["allowed_materials"]) == sorted(allowed)
    assert lib["cameras"].startswith(f"{P.CONTAINER_SOURCE}/source/") and lib["cameras"].endswith("cameras.yaml")
    assert (P.SOURCE / lib["cameras"].removeprefix(f"{P.CONTAINER_SOURCE}/")).is_file()
    # our scene copy next to the config: the original's text with only the CPU barrier's timeout raised
    scene = Path(lib["evaluation_scene"])
    assert scene.parent == tmp_path and scene.is_file()
    name = original["libuipc"]["evaluation_scene"]
    before, after = (P.SOURCE / S.BRINGUP / name).read_text().splitlines(), scene.read_text().splitlines()
    changed = [(a, b) for a, b in zip(before, after) if a != b]
    assert len(before) == len(after) and len(changed) == 1
    assert changed[0][0].strip() == "sync_timeout_s: 0.5"
    assert changed[0][1].strip() == f"sync_timeout_s: {S.SYNC_TIMEOUT_S:g}"
    assert yaml.safe_load(scene.read_text())["libuipc"]["cpu_mujoco"]["sync_timeout_s"] == S.SYNC_TIMEOUT_S
    inside = S.write_cloth_config(P.Case(BAG, 7), out, inside="/workspace/output/hexashell/run")
    assert inside["libuipc"]["evaluation_scene"] == f"/workspace/output/hexashell/run/{BAG}_i07_scene.yaml"
    fold = S.write_cloth_config(P.Case(FOLD, 6), tmp_path / "fold.yaml")
    fold_allowed = yaml.safe_load((P.PKG / "config" / f"{FOLD}.yaml").read_text())["generalization"]["material"][
        "allowed_materials"]
    assert len(fold_allowed) == 6 and fold["generalization"]["material"]["allowed_materials"][0] == fold_allowed[0]
    with pytest.raises(ValueError):
        S.write_cloth_config(P.Case(WOK, 0, seed=101), out)


def test_scored(tmp_path):
    f = tmp_path / "evaluate_ret_00.json"
    f.write_text('{"details": [], "statistics": {}}')
    assert not S.scored(f)
    f.write_text('{"details": [{"result": {"code": 0}}]}')
    assert S.scored(f)
    assert not S.scored(tmp_path / "missing.json")


def test_pack_per_runtime():
    row = _row()
    isaac = msgpack.unpackb(L.pack([row], False), raw=False)["result"]
    assert "waist" in isaac and "head" in isaac
    for rt in L.NATIVE:
        r = msgpack.unpackb(L.pack([row, row], False, runtime=rt), raw=False)["result"]
        assert "waist" not in r and "head" not in r and len(r["left_effector"][0]) == 20
        with pytest.raises(ValueError):
            L.pack([dict(row, wheels=np.zeros(3))], False, runtime=rt)
    for rt in ("newton", "mujoco"):
        assert msgpack.unpackb(L.pack([], False, episode_done=True, runtime=rt), raw=False) == {
            "result": {"episode_done": True}}
    for rt in ("isaac", "libuipc"):
        with pytest.raises(ValueError):
            L.pack([], False, episode_done=True, runtime=rt)
    with pytest.raises(ValueError):
        L.pack([], False, runtime="newton")


def test_conclusion_per_runtime():
    for rt, n in (("isaac", L.CONCLUDE_ROWS), ("newton", 0), ("mujoco", 0)):
        ep = L.EpisodeLink(0, _frame(), runtime=rt)
        ep.conclude()
        rows, _, kind = ep.serve(None)
        assert kind == "conclude" and len(rows) == n and ep.stats["dispatches"] == (1 if n else 0)
    ep = L.EpisodeLink(0, _frame(), runtime="libuipc")
    ep.stats["rows"] = 900
    ep.conclude()
    rows, _, kind = ep.serve(None)
    assert kind == "conclude" and len(rows) == P.EPISODE_ROWS["libuipc"] - 900 and ep.stats["dispatches"] == 1


def test_hold_before_the_native_limit():
    assert L.hold_after(60, 1.0, 60) == pytest.approx(60 - L.REPLY_MARGIN_S - 1.0)
    assert L.hold_after(660, 0.5, 660) == pytest.approx(660 - L.REPLY_MARGIN_S - 0.5)
    assert L.hold_after(120, 200, 120) == L.MIN_HOLD_S
    assert L.hold_after(30, 1.0, 660) == 30


def test_capture_time():
    f, now = _frame(), 1_800_000_000.0
    f.timestamp = {"head": int((now - 3.0) * 1e9), "states": int((now - 2.0) * 1e9)}
    assert L.captured_s_ago(f, now) == pytest.approx(2.0)
    f.timestamp = {"states": 12345}  # not the wall clock
    assert L.captured_s_ago(f, now) is None
    f.timestamp = None
    assert L.captured_s_ago(f, now) is None


def test_mujoco_takes_at_most_its_rows_a_reply():
    cap = P.REPLY_ROWS["mujoco"]
    ep = L.EpisodeLink(0, _frame(), runtime="mujoco", hold_s=0.01)
    rows = [L.copy_row(ep.command) for _ in range(2 * cap + 88)]
    done = []
    t = threading.Thread(target=lambda: done.append(ep.act(rows, True)), daemon=True)
    t.start()
    sizes, kinds = [], []
    for k in range(4):
        out, _, kind = ep.serve(None if k == 0 else _frame(seq=1 + k))
        sizes.append(len(out))
        kinds.append(kind)
    t.join(timeout=10)
    assert sizes[:3] == [cap, cap, 88] and kinds[:3] == ["action"] * 3 and kinds[3] == "hold"
    assert done and done[0].seq == 4 and ep.stats["dispatches"] == 4


def test_end_pose_from_joints():
    q = np.array([0.3, -0.5, 0.2, -1.0, 0.1, 0.4, -0.2, -0.3, -0.5, -0.2, -1.0, -0.1, 0.4, 0.2])
    waist = np.array([-0.4014, 0.7524, -0.1710, 0.0, 0.1])
    ep = O.end_pose_from_joints("g2_wuji", q, waist)
    assert ep["computed"]
    assert np.allclose(kinematics.arm_base_from_obs(ep), body.arm_base(waist), atol=1e-9)
    chains = O._chains("g2_wuji")
    for k, arm in enumerate(kinematics.ARMS):
        p = ep["arm_base_link"][f"{arm}_arm"]
        want = chains[arm].end_pose(q[7 * k:7 * k + 7])
        assert np.allclose(kinematics.pose_of(p["position"], p["orientation"]), want, atol=1e-9)
    with pytest.raises(O.ObsError):
        O.end_pose_from_joints("g2_wuji", q, None)


def test_quaternion_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(300):
        q = rng.normal(size=4)
        m = kinematics.pose_of([0.0, 0.0, 0.0], q / np.linalg.norm(q))[:3, :3]
        back = kinematics.pose_of([0.0, 0.0, 0.0], kinematics.quat_xyzw_of(m))[:3, :3]
        assert np.allclose(back, m, atol=1e-9)


def test_parse_a_request_without_end_pose():
    waist = [0.0, 0.0, 0.349066, 0.0, 0.0]
    arms = [1.732, -1.1514, -1.5989, -1.9, -0.4202, 0.0041, 0.0008,
            -1.7277, -1.1473, 1.6009, -1.8991, 0.4222, -0.0013, -0.0007]
    payload = {"method": "infer", "params": {
        "robot_type": "g2a_sim", "ee_type": "wuji", "prompt": "Pan tossing", "task_name": "wok_flip",
        "episode_idx": 0, "episode_done": False, "images": {},
        "states": {"arm_joint_states": arms, "gripper_states": [0.0] * 40, "waist_joint_states": waist,
                   "head_joint_states": [0.0, 0.0, 0.0], "end_pose": None}}}
    f = O.parse(payload, 1)
    assert f.robot == "g2_wuji" and f.end_pose_computed
    left = f.end_link("left")[:3, 3]
    assert np.allclose(left, [0.477, 0.344, 0.963], atol=0.01)  # where the start pose puts the left end link


# -- the native cameras' static models --------------------------------------------------------------------
#: an Isaac request's head (pick_up_a_package's first request): the head's joints, arm_base_link and the
#: calibrated optical frame in base_link
ISAAC_HEAD = [3.1874751584837213e-06, -7.613593879796099e-07, 0.3495945930480957]
ISAAC_ARM_BASE = [[0.998497, -0.054809, 1e-05, 0.135351], [-9e-06, 1.9e-05, 1.0, -8e-06],
                  [-0.054809, -0.998497, 1.8e-05, 1.171737], [0.0, 0.0, 0.0, 1.0]]
ISAAC_HEAD_OPTICAL = [[0.001857, -0.738721, 0.674008, 0.273948], [-0.999934, 0.00626, 0.009617, 0.023735],
                      [-0.011323, -0.673982, -0.738661, 1.483402], [0.0, 0.0, 0.0, 1.0]]
WOK_WAIST = [0.0, 0.0, 0.349066, 0.0, 0.0]
WOK_ARMS = [1.732, -1.1514, -1.5989, -1.9, -0.4202, 0.0041, 0.0008,
            -1.7277, -1.1473, 1.6009, -1.8991, 0.4222, -0.0013, -0.0007]


def _jpeg(w: int, h: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((h, w, 3), 128, np.uint8))
    assert ok
    return buf.tobytes()


def _native_payload(runtime: str) -> dict:
    """A request as the native runtimes send it: no calibration, no depth (Newton's start pose)."""
    hand = (1280, 1056) if runtime == "mujoco" else (960, 768)
    images = {"head": {"encoding": "JPEG", "image_data": _jpeg(640, 400), "width": 640, "height": 400}}
    for wire in ("hand_left", "hand_right"):
        images[wire] = {"encoding": "JPEG", "image_data": _jpeg(*hand), "width": hand[0], "height": hand[1]}
    return {"method": "infer", "params": {
        "robot_type": "g2a_sim", "ee_type": "wuji", "prompt": "p", "task_name": "t", "episode_idx": 0,
        "episode_done": False, "images": images,
        "states": {"arm_joint_states": WOK_ARMS, "gripper_states": [0.0] * 40, "waist_joint_states": WOK_WAIST,
                   "head_joint_states": [0.0, -0.001, 0.45], "end_pose": None}}}


def test_the_static_head_model_is_the_calibrated_isaac_head_camera():
    f = _frame()
    f.head, f.arm_base = np.array(ISAAC_HEAD), np.array(ISAAC_ARM_BASE)
    assert np.allclose(O.native_head_pose(f), ISAAC_HEAD_OPTICAL, atol=2e-4)
    f.head = None
    assert O.native_head_pose(f) is None


@pytest.mark.parametrize("runtime", L.NATIVE)
def test_native_requests_get_a_model_for_every_camera(runtime):
    f = O.parse(_native_payload(runtime), 1, runtime=runtime)
    assert set(f.calib) == set(O.CAMERAS) and not f.depth
    head = f.calib["head"]
    assert (head.width, head.height) == (640, 400) and f.images["head"].shape == (400, 640, 3)
    assert np.allclose(head.pose, O.native_head_pose(f)) and np.allclose(head.k[0, :], [O.HEAD_K[0], 0, O.HEAD_K[2]])
    for cam in ("left_wrist", "right_wrist"):
        c = f.calib[cam]
        assert (c.width, c.height) == wuji.PINHOLE_SIZE and f.images[cam].shape == (640, 800, 3)
        assert np.allclose(c.pose, f.end_link(O.CAM_ARM[cam]) @ wuji.camera_mount(O.CAM_ARM[cam]))
        assert f.camera(cam) is not None
    # the head camera looks down at the hands, each wrist camera at its own fingertips
    for arm in kinematics.ARMS:
        tip = (f.end_link(arm) @ np.r_[wuji.points(arm, f.effector(arm))["index"], 1.0])[:3]
        assert f.camera("head").pixel(tip) is not None and f.camera(f"{arm}_wrist").pixel(tip) is not None
    # without the runtime (Isaac) a request without calibration is read as before: no head model
    assert "head" not in O.parse(_native_payload(runtime), 1).calib


def _project(lens: O.Lens, direction: np.ndarray) -> np.ndarray:
    k = lens.matrix(*lens.size)
    pt = np.asarray(direction, float).reshape(1, 1, 3)
    if lens.model == "fisheye":
        px, _ = cv2.fisheye.projectPoints(pt, np.zeros((3, 1)), np.zeros((3, 1)), k,
                                          np.asarray(lens.d, float).reshape(4, 1))
    else:
        px, _ = cv2.projectPoints(pt, np.zeros(3), np.zeros(3), k, np.asarray(lens.d, float))
    return px.reshape(2)


@pytest.mark.parametrize("runtime,cam", [("libuipc", "right_wrist"), ("newton", "left_wrist"), ("mujoco", "left_wrist"),
                                         ("mujoco", "right_wrist"), ("mujoco", "head"), ("newton", "head")])
def test_a_direction_seen_through_the_lens_lands_where_the_model_puts_it(runtime, cam):
    lens = O.NATIVE_LENSES[runtime][cam]
    w, h = lens.size
    f = _frame()
    f.head = np.zeros(3)
    for d in ([0.0, 0.0, 1.0], [0.5, 0.3, 1.0], [-0.6, 0.2, 1.0], [0.3, -0.5, 1.0], [-0.4, -0.4, 1.0]):
        d = np.asarray(d)
        u, v = _project(lens, d)
        rgb = np.zeros((h, w, 3), np.uint8)
        cv2.circle(rgb, (int(round(u)), int(round(v))), 6, (255, 255, 255), -1)
        out, _, cal = O._native_camera(cam, lens, runtime, f, rgb, None)
        ys, xs = np.nonzero(out[..., 0] > 128)
        want = cal.k @ (d / d[2])
        assert abs(xs.mean() - want[0]) < 1.5 and abs(ys.mean() - want[1]) < 1.5, (d, xs.mean(), ys.mean(), want)


def test_verdict_of_reads_the_episode_budget():
    entry = {"policy_dispatch_count": 20, "task_instruction": "Pan tossing", "result": {
        "code": 0, "scores": {"E2E": 0.0, "POLICY_BUDGET_WEIGHTED": 0.12,
                              "POLICY_BUDGET_STEPS": [{"name": "stable_lift", "score": 1.0, "points": 30.0,
                                                       "earned_points": 15.0, "factor": 0.5}]},
        "budget_evidence": {"episode_budget": {"step_out": 600, "baseline": 16, "count": 20, "over_budget": True,
                                               "penalty_fraction": 0.5, "factor": 0.5}}}}
    v = L.verdict_of(entry)
    assert v["budget"]["factor"] == 0.5 and v["budget"]["count"] == 20 and v["dispatches"] == 20
    assert v["valid"] and not v["success"] and v["score"] == 0.12


class _Container:
    """A container for sim._docker: pids carrying the run's marker; some of them ignore TERM."""

    def __init__(self, pids, ignore_term=()):
        self.pids, self.ignore_term, self.calls = set(pids), set(ignore_term), []

    def __call__(self, *args, timeout=60):
        self.calls.append(args)
        assert args[:6] == ("exec", "-u", "isaac-sim", "c", "sh", "-c")
        if "environ" in args[6]:
            assert args[7:] == ("sh", f"GENIESIM_BENCHMARK_OUTPUT_ROOT={ROOT}/r1")
            return subprocess.CompletedProcess(args, 0, "".join(f"{p}\n" for p in sorted(self.pids)) + "1\n", "")
        sig, pids = args[6].split()[1], {int(p) for p in args[8:]}
        self.pids -= pids - self.ignore_term if sig == "-TERM" else pids
        return subprocess.CompletedProcess(args, 0, "", "")


def test_end_leftovers_terms_then_kills_what_carries_the_run_root(monkeypatch):
    box = _Container({40, 41, 42}, ignore_term={41})
    monkeypatch.setattr(S, "_docker", box)
    assert S.leftovers("c", "r1") == [40, 41, 42]  # never pid 1
    assert S.end_leftovers("c", "r1", wait_s=0.05, poll_s=0.01) == {"term": [40, 41, 42], "kill": [41], "alive": []}
    assert box.pids == set()
    box.calls.clear()
    assert S.end_leftovers("c", "r1", poll_s=0.01) == {"term": [], "kill": [], "alive": []}
    assert len(box.calls) == 1  # one look, no signal


def test_leftovers_is_none_when_the_container_does_not_answer(monkeypatch):
    monkeypatch.setattr(S, "_docker", lambda *a, timeout=60: subprocess.CompletedProcess(a, 1, "", "no such container"))
    assert S.leftovers("c", "r1") is None
    assert S.end_leftovers("c", "r1", poll_s=0.01)["term"] == []


def test_stop_run_ends_leftovers_also_after_the_benchmark_exited(monkeypatch):
    from hexashell.benches.robocoliseum import run as R

    class Exited:
        returncode = 0

        def poll(self):
            return 0

    events = []
    monkeypatch.setattr(R.sim, "stop", lambda *a: pytest.fail("an ended run's group is not signalled"))
    monkeypatch.setattr(R.sim, "end_leftovers", lambda name, run_id: {"term": [7], "kill": [], "alive": []})
    assert R.stop_run(Exited(), "c", "r1", events.append) == 0
    assert events == [{"kind": "sim_leftovers", "term": [7], "kill": [], "alive": []}]
    events.clear()
    monkeypatch.setattr(R.sim, "end_leftovers", lambda name, run_id: (_ for _ in ()).throw(OSError("docker")))
    assert R.stop_run(Exited(), "c", "r1", events.append) == 0
    assert events[0]["kind"] == "sim_leftovers" and "docker" in events[0]["error"]


# -- tools without depth ----------------------------------------------------------------------------------
class _NativeLink:
    """The link of a native runtime that holds one request (nothing moves)."""

    def __init__(self, runtime: str):
        self.runtime = runtime
        self.first = O.parse(_native_payload(runtime), 1, runtime=runtime)
        self.robot = self.first.robot
        self.prompt = self.first.prompt
        self.command = L.start_command(self.first)
        self.over = False
        self.verdict = None
        self.dispatches = 0
        self.index = 0

    def latest(self) -> O.Frame:
        return self.first


def _host(runtime: str):
    from hexashell.benches.robocoliseum.host import ColiseumHost

    return ColiseumHost(_NativeLink(runtime))


@pytest.mark.parametrize("runtime", L.NATIVE)
def test_to_world_on_a_native_runtime_takes_a_height_or_a_second_picture(runtime):
    from hexashell.kernel.host import CallFailed

    host = _host(runtime)
    assert host.native
    spot = host.dispatch("to_world", ("head", (320, 300)), {"z": 0.8})
    assert spot.xyz is not None and spot.xyz[2] == pytest.approx(0.8, abs=1e-6) and spot.warning is None
    assert host.dispatch("to_pixel", (spot.xyz, "head"), {}) == pytest.approx((320, 300), abs=0.2)
    assert host.dispatch("camera", ("head",), {}).width == 640
    # a point ahead of the right hand, seen by its wrist camera and the head: the two rays meet there
    f = host.frame
    p = (f.end_link("right") @ np.r_[wuji.points("right", f.effector("right"))["index"] + [0.0, 0.0, 0.08], 1.0])[:3]
    px_wrist, px_head = (host.dispatch("to_pixel", (p, cam), {}) for cam in ("right_wrist", "head"))
    two = host.dispatch("to_world", ("right_wrist", px_wrist), {"pair": ("head", px_head)})
    assert np.allclose(two.xyz, p, atol=0.003)
    with pytest.raises(CallFailed) as no_depth:
        host.dispatch("to_world", ("head", (320, 300)), {})
    assert no_depth.value.kind == "ValueError"
    assert "no depth" in no_depth.value.text and "z=h" in no_depth.value.text and "pair=" in no_depth.value.text
    assert "picture now" not in no_depth.value.text


def test_measure_on_a_native_runtime_says_why_it_cannot_and_what_works():
    from hexashell.kernel.host import CallFailed

    host = _host("libuipc")
    with pytest.raises(CallFailed) as cannot:
        host.dispatch("measure", ("red cup",), {})
    text = cannot.value.text
    assert cannot.value.kind == "ValueError" and "cannot work" in text and "cameras give none" in text
    assert "to_world(cam, (x, y), z=h)" in text and "pair=" in text and "find(" not in text  # no SAM3 here
    host.scene = object()  # with perception, find is offered too
    with pytest.raises(CallFailed) as cannot:
        host.dispatch("measure", ("red cup",), {})
    assert "find(cam, 'words')" in cannot.value.text


# -- the native prompt, card and results: one span of steps, no waist, no stages in order (E5) ------------
class _MovingNativeLink(WujiLink):
    """The dexterous fake (arms and hands follow the rows) on a native runtime: the rows it took are the steps used
    of the episode's span, as the real link's ``rows``."""

    def __init__(self, runtime: str, **kw):
        super().__init__(**kw)
        self.runtime = runtime
        self.index = 0

    @property
    def rows(self) -> int:
        return sum(len(c) for c in self.chunks)


def _moving_host(runtime: str):
    from hexashell.benches.robocoliseum.host import ColiseumHost

    link = WujiLink() if runtime == "isaac" else _MovingNativeLink(runtime)
    return link, ColiseumHost(link)


def _prompt_parts():
    from hexashell.runtime import card

    folder = card.PROMPTS / "robocoliseum"
    return folder, sorted(p.stem for p in folder.glob("*.md") if "." not in p.stem)


def test_every_prompt_part_has_a_native_one():
    # the shared prompt takes a variant's part where there is one and the plain part otherwise (no chain to .hands)
    folder, stems = _prompt_parts()
    assert len(stems) == 6 and all((folder / f"{s}.hands_native.md").exists() for s in stems)


def test_the_native_prompt_has_one_span_no_waist_no_depth_and_no_stages():
    from hexashell.runtime import card

    text = " ".join(card.system_prompt("robocoliseum", variant="hands_native").split())
    low = text.lower()
    for gone in ("waist", "stage", "allowance", "bonus", "every 30 steps", "grip_yaw", "robot.drive", "robot.body",
                 "robot.things", "penalt", "dispatch"):
        assert gone not in low, gone
    assert low.count("in order") == 1 and "in their places in order along a row" in low  # sorted sets, not stages
    assert "**The episode is one fixed span of steps for the whole task**" in text and "[412 steps left]" in text
    assert "no depth pictures" in text and "`robot.measure()` does not work here" in text
    assert "robot.to_world(cam, (x, y), z=h)" in text and "pair=(cam2, (x2, y2))" in text
    assert "Whether the scene stands still meanwhile depends on the simulator (the task card says)" in text


def test_the_isaac_prompts_read_no_native_part():
    from hexashell.runtime import card

    folder, stems = _prompt_parts()

    def joined(names):
        return "\n\n".join((folder / n).read_text(encoding="utf-8").strip() for n in names) + "\n"

    hands = [f"{s}.hands.md" if (folder / f"{s}.hands.md").exists() else f"{s}.md" for s in stems]
    assert card.system_prompt("robocoliseum", variant="hands") == joined(hands)
    assert card.system_prompt("robocoliseum") == joined(f"{s}.md" for s in stems)
    native = " ".join(card.system_prompt("robocoliseum", variant="hands_native").split())
    for isaac in (card.system_prompt("robocoliseum", variant="hands"), card.system_prompt("robocoliseum")):
        isaac = " ".join(isaac.split())
        assert "one fixed span of steps" not in isaac and "sequence of stages, in order" in isaac
        assert isaac != native


@pytest.mark.parametrize("runtime", ["isaac", *L.NATIVE])
def test_the_native_runtimes_read_the_hands_native_parts(runtime):
    from hexashell.benches.robocoliseum import session

    link, host = _moving_host(runtime)
    want = "hands" if runtime == "isaac" else "hands_native"
    assert session.ColiseumSession.prompt_variant(SimpleNamespace(host=host)) == want
    gripper = SimpleNamespace(host=SimpleNamespace(driver=SimpleNamespace(dexterous=False), native=True))
    assert session.ColiseumSession.prompt_variant(gripper) is None


@pytest.mark.parametrize("runtime", L.NATIVE)
def test_the_native_body_does_not_turn(runtime):
    from hexashell.errors import Refused

    _, host = _moving_host(runtime)
    d = host.driver
    assert d.fixed_body and not d.turns_waist
    with pytest.raises(Refused, match="the torso does not turn in this task"):
        d.waist(by=10)
    _, isaac = _moving_host("isaac")
    assert not isaac.driver.fixed_body and isaac.driver.turns_waist  # the G2 on Isaac turns at the waist


@pytest.mark.parametrize("runtime", ["isaac", "mujoco"])
def test_home_turns_the_waist_back_only_where_it_turns(runtime):
    link, host = _moving_host(runtime)
    host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.05)})
    w = np.array(link.first.waist, float)
    w[motion.WAIST_YAW] += 0.05  # the waist reads 3 degrees off its start
    link._frame = dataclasses.replace(link._frame, waist=w)
    host.dispatch("home", (), {})
    turned = [r["waist"] is not None for r in link.chunks[-1]]
    assert all(turned) if runtime == "isaac" else not any(turned)


@pytest.mark.parametrize("runtime", L.NATIVE)
def test_the_native_card_gives_the_runtime_s_step_span_and_pause(runtime):
    from hexashell.benches.robocoliseum import session

    host = _host(runtime)
    lines = session.robot_lines(host.link.first, host.driver)
    text = "\n".join(lines)
    assert "two seven-joint arms on a torso that does not turn in this task" in lines[0] and "waist" not in text
    time = [line for line in lines if line.startswith("- Time:")]
    assert len(time) == 1 and session.STEP_WORDS[runtime] in time[0] and session.SPAN_WORDS[runtime] in time[0]
    assert ("The simulation stands still while you think." in time[0]) == P.PAUSES[runtime]
    assert ("does not stand still while you think" in time[0]) == (not P.PAUSES[runtime])
    assert ("undistorted from fisheye lenses" in text) == (runtime != "mujoco")  # MuJoCo's wrists are pinholes
    assert "physics runs at" not in text
    ends = [line for line in lines if line.startswith("- Besides the span, the episode ends at once on: ")]
    assert len(ends) == (runtime != "libuipc")
    if runtime == "mujoco":
        assert "the two arms or hands touching each other" in ends[0] and "the cube dropping" in ends[0]
    if runtime == "newton":
        assert ends[0].endswith("the pan going below the table's top.")


def test_the_isaac_card_keeps_its_words():
    from hexashell.benches.robocoliseum import session

    link, host = _moving_host("isaac")
    lines = session.robot_lines(link.first, host.driver)
    assert "two seven-joint arms on a torso that turns at the waist" in lines[0]
    assert "- Time: one step (one command row) is about 17 ms of simulated time in this scene (its physics runs at " \
           "120 Hz)." in lines
    assert "undistorted from fisheye lenses" in "\n".join(lines)
    assert not any(line.startswith("- Besides the span") for line in lines)


@pytest.mark.parametrize("runtime", ["isaac", *L.NATIVE])
def test_the_card_lists_the_benchmark_s_steps_as_a_description(runtime):
    from hexashell.benches.robocoliseum import session

    steps = ["Grasp the pan handle", "Toss the pan forward"]
    if runtime == "isaac":
        link, host = _moving_host(runtime)
    else:
        host = _host(runtime)
        link = host.link
    link.first, link.index = dataclasses.replace(link.first, segments=steps), 0
    s = session.ColiseumSession(link, {"config": WOK, "instance": 0, "episode": 0})
    s.host, s.first = host, {}
    text, used = s.card(SimpleNamespace(skills=False))
    task = repr(host.dispatch("task", (), {}))
    assert "1. Grasp the pan handle" in text and "2. Toss the pan forward" in task and used["stages"] == 2
    if runtime == "isaac":
        assert ("step by step, in order (a description, not the stages it judges: several steps can make one stage"
                in text) and "(a description, not the stages it judges" in task
    else:
        for words in (text, task):
            assert "stage" not in words and "in order" not in words
            assert "they describe the task, not how it is judged: several steps can make one part" in words


@pytest.mark.parametrize("runtime", ["isaac", *L.NATIVE])
def test_the_timing_note_per_runtime(runtime):
    from hexashell.benches.robocoliseum import session

    link, host = _moving_host(runtime)
    note = session.ColiseumSession.timing_note(SimpleNamespace(host=host, link=link))
    if runtime == "isaac":
        assert "(goto, gripper, hand, home, wait, waist)" in note and "each stage of the task allows" in note
        return
    assert "(goto, gripper, hand, home, wait)" in note and f"one span of {session.SPAN_WORDS[runtime]}" in note
    assert "stage" not in note and "measuring" not in note
    assert note.endswith("and the simulation stands still meanwhile." if P.PAUSES[runtime] else
                         "but the simulation runs on meanwhile: the robot holds its pose, and whatever is still moving "
                         "goes on moving.")


@pytest.mark.parametrize("runtime", ["libuipc", "newton"])
def test_every_motion_result_says_the_steps_left_of_the_span(runtime):
    from hexashell.benches.robocoliseum import session

    link, host = _moving_host(runtime)
    limit = P.EPISODE_ROWS[runtime]
    assert host.span() == (0, limit)
    hand = host.dispatch("hand", ("right", "pinch", 1.0), {})
    assert link.rows > 0 and hand.steps_left == limit - link.rows
    assert repr(hand).splitlines()[0].endswith(f"{hand.steps} steps  [{limit - link.rows} steps left]")
    move = host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.03)})
    assert move.steps_left == limit - link.rows and f"  [{limit - link.rows} steps left]" in repr(move)
    waited = host.dispatch("wait", (30,), {})
    assert repr(waited) == f"Waited 30 steps ({30 * P.STEP_S[runtime]:.2f} s)  [{limit - link.rows} steps left]"
    homed = host.dispatch("home", (), {})
    assert homed.steps_left == limit - link.rows and f"[{limit - link.rows} steps left]" in repr(homed)
    used = link.rows
    assert f"Budget: {limit} steps; used {used}, {limit - used} left" in repr(host.dispatch("task", (), {}))
    assert f"step {used} of {limit} ({limit - used} left)" in repr(host.dispatch("state", (), {}))
    fake = SimpleNamespace(host=host)
    assert (session.ColiseumSession.step.fget(fake), session.ColiseumSession.step_limit.fget(fake)) == (used, limit)
    dry = host.dispatch("hand", ("left", "pinch", 0.5), {"dry_run": True})
    assert dry.steps_left == limit - used and "steps left" not in repr(dry)  # a dry run uses none and says so


def test_mujoco_counts_its_span_in_simulated_time():
    link, host = _moving_host("mujoco")
    assert host.span() == (0, 900)
    host.dispatch("wait", (10,), {})
    assert host.span() == (10, 900)  # no simulation_time: the rows
    link._frame = dataclasses.replace(link._frame, sim_time=4.0)  # 60 steps of 1/15 s: some rows were stretched
    assert host.span() == (60, 900)
    waited = host.dispatch("wait", (15,), {})
    assert repr(waited) == "Waited 15 steps (1.00 s)  [840 steps left]"
    link._frame = dataclasses.replace(link._frame, sim_time=61.0)
    assert host.span() == (900, 900)


def test_isaac_results_say_nothing_of_a_span():
    from hexashell.benches.robocoliseum import session

    link, host = _moving_host("isaac")
    assert host.span() is None and session.ColiseumSession.step.fget(SimpleNamespace(host=host)) is None
    hand = host.dispatch("hand", ("right", "pinch", 1.0), {})
    move = host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.03)})
    waited = host.dispatch("wait", (30,), {})
    homed = host.dispatch("home", (), {})
    for r in (hand, move, waited, homed):
        assert r.steps_left is None and "steps left" not in repr(r)
    assert repr(waited) == f"Waited 30 steps ({30 * round(host.driver.row_s, 4):.2f} s)"
    assert "Budget" not in repr(host.dispatch("task", (), {})) and "step " not in repr(host.dispatch("state", (), {}))


def test_parse_reads_the_simulated_time():
    payload = _native_payload("mujoco")
    f = O.parse(payload, 1, runtime="mujoco")
    assert f.sim_time is None and "sim_time" not in O.summary(f)
    payload["params"]["simulation_time"] = 2.0
    f = O.parse(payload, 1, runtime="mujoco")
    assert f.sim_time == 2.0 and O.summary(f)["sim_time"] == 2.0
    for bad in (float("nan"), "soon", None):
        payload["params"]["simulation_time"] = bad
        assert O.parse(payload, 1, runtime="mujoco").sim_time is None


# -- the hands' messages (E4) ----------------------------------------------------------------------------------
@pytest.mark.parametrize("runtime", ["isaac", "mujoco"])
def test_the_note_on_fingers_held_apart_is_isaac_s(runtime):
    # native finger drives lag: an empty close reads its pads 1.0-1.2 cm apart there, so a wide gap tells nothing
    link, host = _moving_host(runtime)
    link.pinch = {"right": 0.02}
    host.dispatch("hand", ("right", "pinch"), {})
    res = host.dispatch("hand", ("right",), {"opening": 0.0})
    assert res.holding is True and abs(res.gap_m - 0.02) < 0.002 and "holding: the thumb and index" in str(res)
    assert ("something holds the fingers apart" in str(res)) == (runtime == "isaac")


@pytest.mark.parametrize("runtime", ["libuipc", "newton"])
def test_libuipc_and_newton_say_touching_or_holding_cannot_be_told(runtime):
    # libuipc: no cloth in the hand model; Newton (the wok): every close read "empty", the one that lifted the pan
    # too, and a pinch opened in the air read "holding"
    from hexashell.benches.robocoliseum import dexterous as D

    link, host = _moving_host(runtime)
    link.pinch = {"right": 0.02}
    host.dispatch("hand", ("right", "pinch"), {})
    res = host.dispatch("gripper", ("right", "close"), {})
    text = str(res)
    assert isinstance(res, D.Hand) and res.holding is None and res.touching == () and res.untold
    assert f"  {D.UNTOLD}" in text.splitlines() and "cm apart" not in text
    assert not any(w in text for w in ("holding:", "empty:", "open:", "touching: the"))
    assert host.driver.catches["right"] == 0
    assert "hand pinch, opening 0.00, touching or holding cannot be told here" in str(host.dispatch("state", (), {}))
    link.pinch = {}  # the fingers close further on the lift: nothing to call dropped where nothing was told held
    lift = host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.05)})
    assert lift.moved and lift.halt != "Dropped"
    opened = host.dispatch("gripper", ("right", "open"), {})
    assert opened.untold and "open: nothing between" not in str(opened)
    assert D.UNTOLD in str(host.dispatch("hand", ("right", "flat"), {}))  # flat touches too: cannot be told


@pytest.mark.parametrize("runtime", ["isaac", "mujoco"])
def test_elsewhere_the_hands_still_tell_holding_and_dropping(runtime):
    from hexashell.kernel.host import CallFailed

    link, host = _moving_host(runtime)
    link.pinch = {"right": 0.02}
    host.dispatch("hand", ("right", "pinch"), {})
    res = host.dispatch("gripper", ("right", "close"), {})
    assert res.holding is True and not res.untold and "cannot be told" not in str(res)
    link.pinch = {}
    with pytest.raises(CallFailed) as dropped:
        host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.05)})
    assert dropped.value.kind == "Dropped"


# -- the end of a native episode (E1) ---------------------------------------------------------------------------
def _entry(runtime: str, how: str) -> dict:
    """A details entry as each runtime writes it (the fields of recorded results)."""
    steps = {"libuipc": ["WujiFoldLeftSide", "WujiFoldRightSide", "WujiFoldHemToCollar"],
             "newton": ["stable_lift", "food_toss"], "mujoco": ["PickUp", "Turn", "Solve"]}[runtime]
    result = {"code": 0, "step": 2400, "msg": "", "scores": {"E2E": 0, "STEPS": [
        {"step": i, "name": n, "score": 1.0 if (how == "one part" and i == 0) else 0.0} for i, n in enumerate(steps)]}}
    if runtime == "libuipc":
        if how in ("ipc", "scorer"):
            error = ("IPC source is failed" if how == "ipc" else
                     "Streaming scoring worker exited 1; inspect /workspace/output/hexashell/x/_attempts")
            return {"valid": False, "result": {"code": 1, "step": 886, "msg": error},
                    "execution": {"execution_status": "failed", "termination_reason": None, "error": error}}
        return {"valid": True, "result": result,
                "execution": {"execution_status": "completed", "termination_reason": "max_steps", "error": None}}
    if runtime == "newton" and how == "pan":  # wok_evaluation.py: the pan's rim below the table's top
        return {"valid": True, "result": dict(result, step=212),
                "native_result": {"stop_reason": "wok_below_table", "failed": True,
                                  "failure_reason": "wok_below_table"}}
    if runtime == "newton":
        return {"valid": True, "result": dict(result, step=600),
                "native_result": {"stop_reason": "step_limit", "failed": False, "failure_reason": None}}
    reason = {"joint": "joint_limit", "arms": "cross_arm_collision"}.get(how, "timeout")
    return {"valid": True, "result": dict(result, step=60000),
            "native_result": {"status": "task_failure", "reason": reason, "valid": True,
                              "result": {"code": 2, "msg": reason},
                              "error": reason if how in ("joint", "arms") else None}}


def test_end_of_reads_each_runtime_s_own_cause():
    assert L.end_of(_entry("libuipc", "span")) == {"cause": "span", "reason": "max_steps"}
    assert L.end_of(_entry("newton", "span")) == {"cause": "span", "reason": "step_limit"}
    assert L.end_of(_entry("mujoco", "span")) == {"cause": "span", "reason": "timeout"}
    assert L.end_of(_entry("mujoco", "joint")) == {"cause": "joint_limit", "reason": "joint_limit"}
    assert L.end_of(_entry("mujoco", "arms")) == {"cause": "cross_arm_collision", "reason": "cross_arm_collision"}
    assert L.end_of(_entry("newton", "pan")) == {"cause": "wok_below_table", "reason": "wok_below_table"}
    assert L.end_of(_entry("libuipc", "ipc")) == {"cause": "failure", "reason": "IPC source is failed"}
    assert L.end_of(_entry("libuipc", "scorer"))["cause"] == "failure"
    v = L.verdict_of(_entry("mujoco", "joint"))
    assert v["end"]["cause"] == "joint_limit" and not v["success"] and len(v["sub_steps"]) == 3
    isaac = {"result": {"code": 0, "scores": {"E2E": 0, "STEPS": [{"name": "PickUp", "score": 0.0}]}}}
    assert L.end_of(isaac) is None and "end" not in L.verdict_of(isaac)


def test_a_native_end_names_its_cause():
    from hexashell.benches.robocoliseum.host import describe_native_end

    span = describe_native_end(L.verdict_of(_entry("newton", "one part")), "newton", (600, 600))
    assert span == ("its span of 600 steps (10 s) is used up, which ends it wherever the task stands; "
                    "1 of 2 parts done")
    joint = describe_native_end(L.verdict_of(_entry("mujoco", "joint")), "mujoco", (359, 900))
    assert joint == "the simulator stopped it because a joint was driven past its range; 0 of 3 parts done"
    arms = describe_native_end(L.verdict_of(_entry("mujoco", "arms")), "mujoco", (447, 900))
    assert arms == ("the simulator stopped it because the two arms (or hands) touched each other; "
                    "0 of 3 parts done")
    pan = describe_native_end(L.verdict_of(_entry("newton", "pan")), "newton", (212, 600))
    assert pan == "the simulator stopped it because the pan went below the table's top; 0 of 2 parts done"
    failed = describe_native_end(L.verdict_of(_entry("libuipc", "ipc")), "libuipc", (886, 2400))
    assert failed == "a failure of the simulator ended it" and "inspect" not in failed
    unscored = dict(L.verdict_of(_entry("libuipc", "scorer")), sub_steps=[{"name": "LiftHanger", "score": 0.0}])
    assert describe_native_end(unscored, "libuipc") == failed  # a failed episode is not scored: no parts
    # no verdict in time: the span by our own count, else where it stood
    assert describe_native_end({"success": None, "ended": True}, "libuipc", (2400, 2400)).startswith(
        "its span of 2400 steps (80 s) is used up")
    assert describe_native_end({"success": None, "ended": True}, "mujoco", (359, 900)) == (
        "the simulator ended it at step 359 of its span of 900, without a reason yet")
    assert describe_native_end({"success": True}, "mujoco") == (
        "the benchmark reports SUCCESS: every part of the task is done")
    for text in (span, joint, arms, pan, failed):
        assert "stage" not in text and "ran out of time" not in text


class _EndingLink(_MovingNativeLink):
    """Ends the episode with ``final`` on the chunk that would go past ``end_at`` rows."""

    def __init__(self, runtime: str, end_at: int, final: dict):
        super().__init__(runtime)
        self.end_at, self.final = end_at, final

    def act(self, rows, need_depth):
        from hexashell.motion.driver import EpisodeOver

        if self.rows + len(rows) > self.end_at:
            self.over, self.verdict = True, self.final
            raise EpisodeOver(self.final)
        return super().act(rows, need_depth)


def test_the_model_is_told_the_span_ran_out_during_its_motion():
    from hexashell.benches.robocoliseum import session
    from hexashell.benches.robocoliseum.host import ColiseumHost
    from hexashell.kernel.host import Stop

    link = _EndingLink("newton", 600, L.verdict_of(_entry("newton", "span")))
    host = ColiseumHost(link)
    for _ in range(4):
        host.dispatch("wait", (150,), {})
    assert host.span() == (600, 600)
    with pytest.raises(Stop) as stop:
        host.dispatch("wait", (10,), {})
    assert str(stop.value) == ("the episode ended during robot.wait(): its span of 600 steps (10 s) is used up, which "
                               "ends it wherever the task stands; 0 of 2 parts done")
    assert session.ColiseumSession.over_text(SimpleNamespace(host=host, link=link)) == (
        "The episode is over: its span of 600 steps (10 s) is used up, which ends it wherever the task stands; "
        "0 of 2 parts done.")
    with pytest.raises(Stop) as again:
        host.dispatch("goto", ("right",), {"by": (0.0, 0.0, 0.02)})
    assert str(again.value).startswith("its span of 600 steps (10 s) is used up")
    no_host = session.ColiseumSession.over_text(SimpleNamespace(host=None, link=link))
    assert no_host.startswith("The episode is over: its span of 600 steps (10 s) is used up")


def test_an_isaac_end_keeps_its_words():
    from hexashell.benches.robocoliseum.host import ColiseumHost, describe_verdict
    from hexashell.kernel.host import Stop
    from hexashell.motion.driver import EpisodeOver

    verdict = {"success": False, "sub_steps": [{"name": "PickUp", "score": 0.0}]}
    words = ("the benchmark ended the episode with 0 of 1 stages done (a stage ran out of time, or something left "
             "the allowed region)")
    assert describe_verdict(verdict) == words

    class Ending(WujiLink):
        def act(self, rows, need_depth):
            self.over, self.verdict = True, verdict
            raise EpisodeOver(verdict)

    host = ColiseumHost(Ending())
    with pytest.raises(Stop) as stop:
        host.dispatch("wait", (10,), {})
    assert str(stop.value) == f"{words} (during robot.wait(), 0 steps into it)"


# -- a two-hand move refused for one hand's line (E9) ---------------------------------------------------------
def _right_line_rises_at_most(d, most: float) -> None:
    """The right hand's straight lines that rise more than ``most`` fail (as the right arm's did lifting the shirt
    in recorded fold_cloth runs); its other lines and the left hand's are planned as usual."""
    line = d._line

    def planned(arm, cur, target, *a, **kw):
        if arm == "right" and target[2, 3] - cur[2, 3] > most + 1e-9:
            return None
        return line(arm, cur, target, *a, **kw)

    d._line = planned


def test_an_isaac_two_hand_move_refused_for_one_hand_keeps_its_words():
    from hexashell.kernel.host import CallFailed

    link, host = _moving_host("isaac")
    _right_line_rises_at_most(host.driver, 0.04)
    with pytest.raises(CallFailed) as refused:
        host.dispatch("goto", ("both",), {"by": (0.0, 0.0, 0.10)})
    lines = refused.value.text.splitlines()
    assert refused.value.kind == "NotExecuted" and len(lines) == 3 and not link.chunks
    assert lines[0].startswith("Move left: NOT executed") and lines[1].startswith("Move right: NOT executed")
    assert lines[2] == "  note: the right hand cannot move along that straight line from here"


@pytest.mark.parametrize("runtime", ["libuipc", "newton", "mujoco"])
def test_a_native_two_hand_move_refused_for_one_hand_says_which_can_go_and_how_far_both_can(runtime):
    from hexashell.kernel.host import CallFailed

    link, host = _moving_host(runtime)
    _right_line_rises_at_most(host.driver, 0.04)
    with pytest.raises(CallFailed) as refused:
        host.dispatch("goto", ("both",), {"by": (0.0, 0.0, 0.10)})
    assert refused.value.kind == "NotExecuted" and not link.chunks
    left, right = refused.value.text.split("Move right")
    assert ("  note: the left hand's own line is free: goto('left', by=(0.000, 0.000, 0.100)) moves it alone "
            "('both' moves both hands or neither)") in left
    assert "  note: the right hand cannot move along that straight line from here" in right
    offered = re.search(r"together both hands can go (\d+) of the 10 cm: goto\('both', by=\(([-\d., ]+)\)\)$", right)
    assert offered and offered.group(1) == "4"
    by = tuple(float(v) for v in offered.group(2).split(", "))
    assert by[:2] == (0.0, 0.0) and 0.03 <= by[2] <= 0.04
    # what it offers runs: the left hand alone by a line, and both hands together that far
    alone = host.dispatch("goto", ("left",), {"by": (0.0, 0.0, 0.10), "dry_run": True})
    assert alone.reached and alone.path == "line" and not link.chunks
    moves = host.dispatch("goto", ("both",), {"by": by})
    assert all(m.moved and m.reached for m in moves) and len(link.chunks) == 1


def test_a_native_two_hand_dry_run_says_the_same_and_keeps_contact():
    link, host = _moving_host("newton")
    _right_line_rises_at_most(host.driver, 0.0)
    moves = host.dispatch("goto", ("both",), {"by": (0.0, 0.0, 0.10), "dry_run": True, "contact": True})
    assert all(m.dry_run and not m.moved and not m.reached for m in moves) and not link.chunks
    left, right = (str(m) for m in moves)
    assert "goto('left', by=(0.000, 0.000, 0.100), contact=True) moves it alone" in left
    assert right.endswith("  note: together both hands cannot go even 1 cm of the way")


def test_a_native_two_hand_move_neither_hand_can_make_names_both():
    from hexashell.kernel.host import CallFailed

    link, host = _moving_host("libuipc")
    line = host.driver._line
    host.driver._line = lambda arm, cur, target, *a, **kw: (None if target[2, 3] - cur[2, 3] > 0.02 + 1e-9
                                                           else line(arm, cur, target, *a, **kw))
    with pytest.raises(CallFailed) as refused:
        host.dispatch("goto", ("both",), {"by": (0.0, 0.0, 0.10)})
    text = refused.value.text
    assert "own line is free" not in text and not link.chunks
    assert all(f"the {arm} hand cannot move along that straight line from here" in text for arm in ("left", "right"))
    assert text.endswith("together both hands can go 2 of the 10 cm: goto('both', by=(0.000, 0.000, 0.018))")
