"""benches/robocoliseum/tunnel.py, challenge.py, fakegate.py and online.sh (Season II online): the client's parts that
need no gateway (frames, payloads, the state file, the config of a case, what is hidden), and whole cases through the
fake gateway with made-up Season II requests and a scripted player (no model, no simulator, no platform)."""

from __future__ import annotations

import asyncio
import json
import os
import pickle
import subprocess
import threading
import time
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace

import msgpack
import msgpack_numpy
import numpy as np
import pytest

from hexashell.benches.robocoliseum import challenge as C
from hexashell.benches.robocoliseum import fakegate as F
from hexashell.benches.robocoliseum import link as L
from hexashell.benches.robocoliseum import obs as O
from hexashell.benches.robocoliseum import protocol as P
from hexashell.benches.robocoliseum import session as S
from hexashell.benches.robocoliseum import tunnel as T
from hexashell.motion.driver import EpisodeOver

ROOT = Path(__file__).resolve().parents[1]
ONLINE_SH = ROOT / "hexashell" / "benches" / "robocoliseum" / "online.sh"
#: a G2's start pose (Newton wok_flip's first request, tests/test_robocoliseum_native.py)
G2_WAIST = [0.0, 0.0, 0.349066, 0.0, 0.0]
G2_ARMS = [1.732, -1.1514, -1.5989, -1.9, -0.4202, 0.0041, 0.0008,
           -1.7277, -1.1473, 1.6009, -1.8991, 0.4222, -0.0013, -0.0007]
WIRE = {key: (r.robot_type, r.ee_type, r.effector) for key, r in P.ROBOTS.items()}
ONLINE = {"success": None, "progress": None, "online": True}


def request(robot: str, task_name: str, prompt: str, *, idx: int = 0, done: bool = False, runtime: str = "isaac",
            chassis: bool = False) -> dict:
    """A Season II request as its runtime sends it (genie_sim 3.6), without pictures: Isaac, libuipc and
    MuJoCo report end_pose, Newton sends None (our kinematics computes it from the joints and five waist readings)."""
    robot_type, ee_type, effector = WIRE[robot]
    r = P.ROBOTS[robot]
    g2 = r.arm_joints == 7 and r.waist == 5
    arms = list(G2_ARMS) if g2 else [0.0] * (2 * r.arm_joints)
    waist = list(G2_WAIST) if g2 else [0.0] * r.waist
    now = time.time_ns()
    pose = None
    if runtime != "newton":
        pose = {"base_link": {}, "arm_base_link": {}}  # read only for a G2 (the sessions here)
        if g2:
            pose = O.end_pose_from_joints(robot, np.array(arms), np.array(waist))
            pose.pop("computed")
    p = {"timestamps": {"head": now, "states": now}, "images": {},
         "states": {"head_joint_states": [0.0, 0.0, 0.45][:r.head], "arm_joint_states": arms,
                    "waist_joint_states": waist, "gripper_states": [0.0] * (2 * effector), "end_pose": pose},
         "prompt": prompt, "robot_type": robot_type, "ee_type": ee_type, "task_name": task_name,
         "episode_idx": idx, "episode_done": done}
    if chassis:
        p["states"]["chassis_states"] = [0.0, 0.0, 0.0]
    if runtime == "mujoco":
        p.update({"simulation_time": 0.0, "episode_start": True, "task_progress": [],
                  "action_contract": {"max_rows": 256}})
    elif runtime == "libuipc":
        p["task_progress"] = []
    return {"method": "infer", "params": p}


def packed(req: dict) -> bytes:
    return msgpack.packb(req, use_bin_type=True)


# -- the wire's pieces --------------------------------------------------------------------------------
def test_frames_round_trip():
    for sid, payload in [("abc", b"\x00\x01"), ("warmup", b""), ("é" * 3, b"x" * 1000)]:
        assert T.split_frame(T.join_frame(sid, payload)) == (sid, payload)
    with pytest.raises(ValueError):
        T.split_frame(b"\x00\x00")
    with pytest.raises(ValueError):
        T.split_frame(b"\x00\x00\x00\x09abc")


def test_numpy_request_made_plain_and_task_progress_dropped_unread():
    req = {"method": "infer", "params": {"states": {"arm_joint_states": np.arange(14, dtype=np.float32),
                                                    "t": np.float64(1.5)},
                                         "task_name": "fold_cloth", "task_progress": [{"stage": 1, "done": 0.25}]}}
    payload = msgpack.packb(req, default=msgpack_numpy.encode, use_bin_type=True)
    plain, changed, params, dropped = T.plain_request(payload)
    assert changed and params["task_name"] == "fold_cloth" and "task_progress" not in params
    assert dropped == {"task_progress": {"type": "list", "len": 1}}  # its kind and length, no value
    back = msgpack.unpackb(plain, raw=False)
    assert back["params"]["states"]["arm_joint_states"] == list(range(14)) and back["params"]["states"]["t"] == 1.5
    assert "task_progress" not in back["params"] and "0.25" not in json.dumps(T.request_probe(params, dropped))
    same = packed({"method": "infer", "params": {"a": [1.0, 2.0]}})  # already plain: untouched
    assert T.plain_request(same)[:2] == (same, False)


def test_object_arrays_refused():
    """msgpack_numpy unpickles an object array; ours must not."""
    evil = {b"nd": True, b"type": "|O", b"kind": b"O", b"shape": (1,), b"data": pickle.dumps([1])}
    with pytest.raises(ValueError):
        T._nd(evil)
    payload = msgpack.packb({"x": evil}, use_bin_type=True)
    assert T.plain_request(payload)[:2] == (payload, False)  # left for link.Server, which cannot read it


def test_state_file_and_season(tmp_path):
    path = tmp_path / "state.env"
    C.write_env(path, {"JOB_UUID": "u-1", "JOB_TOKEN": "a b'c", "PARALLELISM": 2})
    assert path.stat().st_mode & 0o777 == 0o600
    assert T.read_state(path) == {"JOB_UUID": "u-1", "JOB_TOKEN": "a b'c", "PARALLELISM": "2"}
    C.write_env(path, {"JOB_ID": "7"}, drop=("JOB_TOKEN",))
    assert T.read_state(path) == {"JOB_UUID": "u-1", "PARALLELISM": "2", "JOB_ID": "7"}
    os.chmod(path, 0o640)
    with pytest.raises(SystemExit):
        T.read_state(path)
    assert T.season_problem({"JOB_SEASON": "2", "JOB_BOARD": "v2"}) is None
    assert T.season_problem({"JOB_SEASON": "2"}) is None and T.season_problem({"JOB_BOARD": "v2"}) is None
    assert "manip" in T.season_problem({"JOB_BOARD": "manip"})  # a Season I job
    assert T.season_problem({"JOB_SEASON": "1", "JOB_BOARD": "v2"}) and T.season_problem({})


def test_records_never_name_the_job():
    uuid = "1d6f1c1e-5b0e-4c5e-9f1a-0123456789ab"
    tag = T.job_tag(uuid)
    assert len(tag) == 10 and tag not in uuid
    url = T.dial_url("wss://host/api/challenge/tunnel", uuid, "agent-1")
    assert url == f"wss://host/api/challenge/tunnel?job={uuid}&agent=agent-1"
    assert C.endpoint_host("ws://10.1.2.3:8080/api/challenge/tunnel") == "set (ws)" and C.endpoint_host("") == "none"


def test_scrub_hides_credentials_ids_and_links():
    resp = {"id": 5, "uuid": "u", "job_token": "t", "Token": "x", "tunnel_endpoint": "ws://h", "url": "https://s3/x",
            "items": [{"email": "e", "score": 0.5, "user_uid": "q"}]}
    assert C.scrub(resp) == {"id": 5, "uuid": "<hidden>", "job_token": "<hidden>", "Token": "<hidden>",
                             "tunnel_endpoint": "<hidden>", "url": "<hidden>",
                             "items": [{"email": "<hidden>", "score": 0.5, "user_uid": "<hidden>"}]}


def test_online_limits():
    names = {f.name for f in fields(__import__("hexashell.runtime.episode", fromlist=["Settings"]).Settings)}
    assert set(T.ONLINE_LIMITS) <= names
    assert T.ONLINE_LIMITS["budget_s"] == 240 * 60 and T.ONLINE_LIMITS["cell_total_s"] >= 3600


# -- the config of a case -----------------------------------------------------------------------------
def test_every_config_maps_back_from_its_requests():
    """What each runtime sends as task_name: corobotpolicy the sub-task, libuipc and Newton the task, MuJoCo the
    scene (rubik_cube_solve_2turn); with the robot it names the config (the trash can sub-task is shared by four)."""
    for t in P.TASKS.values():
        got = T.facts_of_request(request(t.robot, t.sub_task, "p", runtime=t.runtime, chassis=t.mobile)["params"],
                                 {"task_progress": {"type": "list", "len": 0}} if t.runtime == "libuipc" else {})
        assert got["known"] and got["match"] == "sub_task", t.config
        assert (got["config"], got["board"], got["runtime"], got["task"]) == (t.config, t.board, t.runtime, t.task)
    assert T.online_facts("stack_bowls", "g2_wuji")["config"] == "g2crsbwuji_dexmanip_stack_bowls"  # our task name
    assert T.online_facts("deliver_tissues", "g2_90d")["config"] == "g2crsb90d_wbc_deliver_tissues"


def test_a_variant_or_unknown_task_name():
    one = T.online_facts("rubik_cube_solve_1turn", "g2_wuji", runtime="mujoco")  # the MuJoCo runner's default
    assert one["known"] and one["match"] == "prefix" and one["config"] == "g2crsbwuji_dexmanip_rubik_cube_solve"
    assert T.online_facts("rubik_cube_solve_1turn", "g2_wuji", runtime="isaac")["known"] is False  # runtimes differ
    new = T.online_facts("juggle balls/../x", "g2_90d", runtime="isaac", wheels=True)
    assert not new["known"] and new["task"] == "juggle_balls_x" and new["board"] == "wbc"
    assert P.task(new["stand_in"]).board == "wbc" and new["config"] == "online_wbc_juggle_balls_x"
    assert T.online_facts("sort_socks", "g2_90d")["board"] == "finemanip"
    cloth = T.online_facts("knot_rope", "g2_wuji", runtime="libuipc")
    assert cloth["board"] == "dexmanip" and cloth["runtime"] == "libuipc" and P.task(cloth["stand_in"]).runtime == "libuipc"
    assert T.online_facts("x", "agilex")["board"] == "crsemb"


def test_runtime_of_a_request():
    for runtime in ("isaac", "mujoco", "libuipc", "newton"):
        req = request("g2_wuji", "t", "p", runtime=runtime)["params"]
        _, _, params, dropped = T.plain_request(packed({"method": "infer", "params": req}))
        assert T.runtime_of(params, dropped) == runtime
    assert T.runtime_of(request("g2_omnipicker", "t", "p", runtime="newton")["params"], {}) == "isaac"  # Wuji only


def test_prepare_sets_the_runtime_limit_and_hold_per_case():
    srv = T.TunnelServer(on_episode=lambda link: None, hold_s=None, reply_cap_s=120.0)
    assert (srv.runtime, srv.reply_limit_s, srv.hold_s) == ("isaac", 120.0, 120.0)  # before any request
    cases = [("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker", "isaac", False, 120.0, False),
             ("g2_90d", "open_the_door", "isaac", True, 120.0, True),
             ("g2_wuji", "rubik_cube_solve_2turn", "mujoco", False, 60.0, False),
             ("g2_wuji", "wok_flip", "newton", False, 120.0, False),  # Newton waits 660 s locally: the cap
             ("g2_wuji", "fold_cloth", "libuipc", False, 120.0, False)]
    for robot, task, runtime, chassis, limit, smooth in cases:
        req = request(robot, task, "p", runtime=runtime, chassis=chassis)["params"]
        _, _, params, dropped = T.plain_request(packed({"method": "infer", "params": req}))
        facts = srv.prepare(params, dropped)
        assert facts["known"] and (srv.runtime, srv.reply_limit_s, srv.hold_s, srv.smooth_depth) == \
            (runtime, limit, limit, smooth), task
        # as late as safe: the hold row reaches the simulator REPLY_MARGIN_S before its limit
        assert L.hold_after(srv.hold_s, 10.0, srv.reply_limit_s) == pytest.approx(limit - L.REPLY_MARGIN_S - 10.0)
    done = request("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker", "p", done=True)["params"]
    srv.prepare(done)  # a request that ends an episode keeps the case's facts
    assert srv.runtime == "libuipc"
    srv.prepare({"robot_type": "nobody", "ee_type": "x", "task_name": "t"})  # no robot we know: unchanged
    assert srv.runtime == "libuipc"
    short = T.TunnelServer(on_episode=lambda link: None, hold_s=30.0, reply_cap_s=120.0)
    short.prepare(request("g2_wuji", "rubik_cube_solve_2turn", "p", runtime="mujoco")["params"])
    assert (short.reply_limit_s, short.hold_s) == (60.0, 30.0)
    srv.stop()
    short.stop()


# -- the session of a case ----------------------------------------------------------------------------
class _Rec:
    def __init__(self):
        self.events = []

    def event(self, e):
        self.events.append(e)

    def state(self, record):
        pass

    def frames(self, name, images):
        pass


def _session(robot, task_name, runtime, *, depth_mode="when_needed", case=3):
    req = request(robot, task_name, "Hang the cups", runtime=runtime)["params"]
    _, _, params, dropped = T.plain_request(packed({"method": "infer", "params": req}))
    facts = T.facts_of_request(params, dropped)
    frame = O.parse({"method": "infer", "params": params}, 1, runtime=facts["runtime"])
    link = L.EpisodeLink(0, frame, hold_s=60.0, runtime=facts["runtime"])
    info = {"case": case, "slot": 1, "episode": 0, "depth_mode": depth_mode, "online": True}
    return T.OnlineSession(link, info, facts), link


SETTINGS = SimpleNamespace(depth=True, sam3_url="", skills=True)


def test_online_session_plays_the_config_of_the_case():
    sess, link = _session("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker", "isaac")
    assert sess.config == "g2op_crsemb_hang_cups_on_cup_tree" and sess.task == "hang_cups_on_cup_tree"
    assert sess.part == "online" and sess.instance == 3 and sess.info.get("eval_seed") is None
    assert sess.record_name() == "g2op_crsemb_hang_cups_on_cup_tree_online_s1_c003_e0"
    assert sess.labels()["case"] == 3
    rec = _Rec()
    sess.open(SETTINGS, rec, lambda kind, image, caption: None)
    # Isaac online, depth when needed: no depth after motions or with hold rows; hold_for_depth asks
    assert sess.host.driver.motion_depth is False and link.hold_depth is False
    assert any(e["kind"] == "online_no_depth" for e in rec.events) and sess.errors  # these requests carry none
    text, used = sess.card(SETTINGS)
    assert "Task: hang_cups_on_cup_tree" in text
    folder = S.notes_folder(P.task(sess.config))
    assert used["notes"] == [n for n, _ in S.note_names(P.task(sess.config)) if S.skill(n, folder)]  # as a local run's
    assert used["notes_folder"] == ("." if folder == S.SKILLS else folder.name)
    fields_ = sess.result_fields(dict(ONLINE))
    assert fields_["online"] and fields_["case"] == 3 and fields_["match"] == "sub_task"
    assert fields_["config"] == sess.config and fields_["board"] == "crsemb"


def test_a_task_named_in_a_subfolder_reads_all_its_notes_from_there(tmp_path, monkeypatch):
    t = P.task("g2op_crsemb_hang_cups_on_cup_tree")
    other = P.task("g2op_crsemb_keyboard_keycap_swap")
    (tmp_path / "robot_g2_omnipicker.md").write_text("default robot")
    (tmp_path / "hang_cups_on_cup_tree.md").write_text("default task")
    own = tmp_path / "skills_for_one_task"
    own.mkdir()
    (own / "tasks.txt").write_text("# a comment\nhang_cups_on_cup_tree  # another comment\n\n")
    (own / "hang_cups_on_cup_tree.md").write_text("own task\n")
    (tmp_path / "unlisted_folder").mkdir()
    monkeypatch.setattr(S, "SKILLS", tmp_path)
    assert S.notes_folder(t) == own and S.notes_folder(other) == tmp_path
    assert S.skill("hang_cups_on_cup_tree", own) == "own task"
    assert S.skill("robot_g2_omnipicker", own) is None  # missing in the subfolder = no note, not the default one
    assert S.skill("robot_g2_omnipicker") == "default robot"


def test_the_subfolders_name_known_tasks_and_hold_their_notes():
    tasks = {P.task(c).task: P.task(c) for c in P.configs()}
    folders = [f for f in S.SKILLS.iterdir() if f.is_dir() and (f / "tasks.txt").exists()]
    claimed: list[str] = []
    for folder in folders:
        names = [ln.split("#", 1)[0].strip() for ln in (folder / "tasks.txt").read_text().splitlines()]
        names = [n for n in names if n]
        assert names and all(n in tasks for n in names), folder
        claimed += names
        for n in names:
            assert S.notes_folder(tasks[n]) == folder
            assert any(S.skill(m, folder) for m, _ in S.note_names(tasks[n]))
    assert len(claimed) == len(set(claimed))  # a task in two folders would read the first only


def test_online_session_depth_modes_and_native_runtimes():
    sess, link = _session("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker", "isaac", depth_mode="every_motion")
    sess.open(SETTINGS, _Rec(), lambda *a: None)
    assert sess.host.driver.motion_depth is True and link.hold_depth is True  # as a local run
    wok, link = _session("g2_wuji", "wok_flip", "newton")
    wok.open(SETTINGS, _Rec(), lambda *a: None)
    assert link.runtime == "newton" and wok.host.native and wok.prompt_variant() == "hands_native"
    assert wok.host.driver.motion_depth is True and not wok.errors  # native: no depth ever, nothing to change


def test_online_session_of_an_unknown_task_gives_no_task_note():
    sess, _ = _session("g2_90d", "juggle_balls", "isaac")
    assert not sess.facts["known"] and sess.task == "juggle_balls" and sess.config == "online_finemanip_juggle_balls"
    sess.open(SETTINGS, _Rec(), lambda *a: None)
    text, used = sess.card(SETTINGS)
    assert "Task: juggle_balls" in text and not any(n in used["notes"] for n in ("juggle_balls", sess.facts["stand_in"]))
    assert any("unknown online task" in e for e in sess.errors)


# -- whole cases through the fake gateway ------------------------------------------------------------
def _args(out: Path, **kw):
    base = {"slot": 0, "endpoint": None, "out": str(out), "settings": None, "set": [], "hold_s": 2.5,
            "reply_cap_s": 120.0, "depth": "when_needed"}
    return SimpleNamespace(**{**base, **kw})


def _result(reply: bytes) -> dict:
    return msgpack.unpackb(reply, raw=False)["result"]


def _rows(res: dict) -> int:
    return len(res["left_arm"]["values"]) if "left_arm" in res else 0


class Script:
    """The player: per episode (in order of their start) what the model side does."""

    def __init__(self):
        self.links, self.facts, self.infos = [], [], []
        self.hold_seen = threading.Event()

    def __call__(self, link, facts, info):
        self.links.append(link)
        self.facts.append(facts)
        self.infos.append(info)
        k = len(self.links) - 1
        if k == 0:  # thinks past the hold, then one motion of two rows
            assert self.hold_seen.wait(20)
            link.act([L.copy_row(link.command)] * 2, False)
            return {"script": "hold then a motion"}
        if k == 1:  # two motions; the next request is another episode's
            link.act([L.copy_row(link.command)], False)
            try:
                link.act([L.copy_row(link.command)], False)
            except EpisodeOver:
                return {"script": "over at the boundary"}
            return {"script": "not ended"}
        return {"script": "done at once"}


def test_cases_through_the_fake_gateway(tmp_path, monkeypatch):
    monkeypatch.setattr(T, "AGENTS", tmp_path / "agents")
    script = Script()
    hang = ("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker")
    out = {}

    async def scenario():
        gate = F.Gate(parallelism=1, quiet=True)
        await gate.start()
        state = T.read_state(gate.write_state(tmp_path / "state.env"))
        assert T.season_problem(state) is None
        agent = T.Agent(_args(tmp_path / "out"), state, player=script, ready_check=lambda: None)
        run = asyncio.create_task(agent.run())
        await asyncio.wait_for(gate.ready.wait(), 10)
        assert _result(await gate.warm_check(10)) == {}  # the warmup round trip: an empty result
        a = await gate.open_case()
        replies = [_result(await a.ask(packed(request(*hang, "Hang the cups", idx=0))))]  # held: 2.5 s
        script.hold_seen.set()
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the cups", idx=0)))))  # the motion
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the cups", idx=0)))))  # done: concluded
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the cups", idx=0, done=True)))))
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the cups again", idx=1)))))  # episode 1
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the cups again", idx=1)))))
        replies.append(_result(await a.ask(packed(request(*hang, "Hang the last cup", idx=2)))))  # episode 2
        await a.close()
        # a Rubik's cube case (MuJoCo, under its runner's default name), the tunnel cut and restored in it
        b = await gate.open_case()
        rubik = ("g2_wuji", "rubik_cube_solve_1turn")
        replies.append(_result(await b.ask(packed(request(*rubik, "Solve the cube", runtime="mujoco")))))
        assert gate.cut()
        replies.append(_result(await b.ask(packed(request(*rubik, "Solve the cube", runtime="mujoco", done=True)))))
        await b.close()
        assert await gate.drain(20)
        code = await asyncio.wait_for(run, 20)
        await gate.stop()
        agent.finish(code)
        out.update(code=code, replies=replies, gate=gate, agent=agent)

    asyncio.run(asyncio.wait_for(scenario(), 120))
    r = out["replies"]
    # case 0, episode 0: a hold row (one row, depth asked as locally), the motion, the conclusion (30 held rows and
    # episode_done), the answer to the simulator's episode_done (no dispatch)
    assert _rows(r[0]) == 1 and r[0]["need_depth"] and "episode_done" not in r[0]
    assert _rows(r[1]) == 2 and not r[1]["need_depth"]
    assert _rows(r[2]) == L.CONCLUDE_ROWS and r[2]["episode_done"]
    assert _rows(r[3]) == 1 and "episode_done" not in r[3]
    # episode 1 after the end; episode 2 begins with a new instruction, which ends episode 1
    assert _rows(r[4]) == 1 and _rows(r[5]) == 1 and _rows(r[6]) == L.CONCLUDE_ROWS and r[6]["episode_done"]
    # MuJoCo: the conclusion is episode_done without rows
    assert r[7] == {"episode_done": True}
    assert _rows(r[8]) == 1
    links = script.links
    assert [ln.stats["hold_replies"] for ln in links] == [1, 0, 0, 0]
    assert [ln.dispatches for ln in links] == [3, 2, 1, 0]
    assert all(ln.wait_verdict(5) == ONLINE for ln in links)
    hang_cfg = "g2op_crsemb_hang_cups_on_cup_tree"
    assert [f["config"] for f in script.facts] == [hang_cfg] * 3 + ["g2crsbwuji_dexmanip_rubik_cube_solve"]
    assert [f["match"] for f in script.facts] == ["sub_task"] * 3 + ["prefix"] and links[3].runtime == "mujoco"
    assert [i["case"] for i in script.infos] == [0, 0, 0, 1] and links[3].reply_limit_s == 60.0
    agent, gate = out["agent"], out["gate"]
    assert out["code"] == 0 and agent.drained and gate.reconnects == 1 and agent.stats["connections"] == 2
    summary = json.loads((agent.dir / "agent.json").read_text())
    assert summary["cases"] == 2 and summary["episodes"] == 4 and summary["exit"] == 0
    assert [e["script"] for e in summary["played"]] == ["hold then a motion", "over at the boundary", "done at once",
                                                        "done at once"]
    hows = [json.loads(line).get("how") for f in sorted((agent.dir / "cases").glob("c000_*/link.jsonl"))
            for line in f.read_text().splitlines() if '"episode_over"' in line]
    assert hows == ["episode_done", "the instruction changed", "session_close"]
    tasks = [json.loads(line) for f in sorted((agent.dir / "cases").glob("c001_*/link.jsonl"))
             for line in f.read_text().splitlines() if '"case_task"' in line]
    assert tasks[0]["runtime"] == "mujoco" and tasks[0]["reply_limit_s"] == 60.0
    # no credential, job id or gateway address in anything the agent wrote
    for path in [*(tmp_path / "out").rglob("*"), *(tmp_path / "agents").rglob("*")]:
        if path.is_file():
            text = path.read_text(errors="replace")
            assert gate.token not in text and gate.job not in text and f":{gate.port}" not in text, path


class OutageScript:
    """The player through a model outage: episode 0 fails on the gateway before any action and is played again
    once the model answers; episode 1 moves, then fails (not played again); episode 2 must never start."""

    def __init__(self):
        self.links, self.infos = [], []

    def __call__(self, link, facts, info):
        self.links.append(link)
        self.infos.append(info)
        k = len(self.infos) - 1
        failed = {"end": "gateway_error", "errors": ["the gateway failed 11 times; last: 502 Upstream request failed"]}
        if k == 0:
            return {**failed, "turns": 0}
        if k == 1:
            link.act([L.copy_row(link.command)] * 2, False)
            return {"end": "finish", "turns": 3}
        if k == 2:
            link.act([L.copy_row(link.command)], False)
            return {**failed, "turns": 1}
        raise AssertionError("an episode started while the model was down")


def test_a_model_outage_holds_the_case_and_replays_an_episode_that_had_not_moved(tmp_path, monkeypatch):
    monkeypatch.setattr(T, "AGENTS", tmp_path / "agents")
    script = OutageScript()
    up = threading.Event()
    probes = []

    def model_check():
        probes.append(up.is_set())
        return None if up.is_set() else "the gateway failed 1 times; last: 502 Upstream request failed"

    hang = ("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker")
    out = {}

    async def scenario():
        gate = F.Gate(parallelism=1, quiet=True)
        await gate.start()
        state = T.read_state(gate.write_state(tmp_path / "state.env"))
        agent = T.Agent(_args(tmp_path / "out"), state, player=script, ready_check=lambda: None,
                        model_check=model_check)
        agent.model_probe_s = 0.3
        run = asyncio.create_task(agent.run())
        await asyncio.wait_for(gate.ready.wait(), 10)
        await gate.warm_check(10)
        a = await gate.open_case()
        ask = lambda prompt, idx, done=False: a.ask(packed(request(*hang, prompt, idx=idx, done=done)))  # noqa: E731
        replies = [_result(await ask("Hang the cups", 0))]  # the model is down: a hold row
        up.set()
        replies.append(_result(await ask("Hang the cups", 0)))  # played again: its motion
        replies.append(_result(await ask("Hang the cups", 0)))  # the conclusion
        replies.append(_result(await ask("Hang the cups", 0, done=True)))
        replies.append(_result(await ask("Hang the cups again", 1)))  # episode 1's motion
        up.clear()
        replies.append(_result(await ask("Hang the cups again", 1)))  # it failed after moving: concluded
        replies.append(_result(await ask("Hang the cups again", 1, done=True)))
        replies.append(_result(await ask("Hang the last cup", 2)))  # the model is down: held, never started
        await a.close()
        assert await gate.drain(20)
        code = await asyncio.wait_for(run, 20)
        await gate.stop()
        agent.finish(code)
        out.update(code=code, replies=replies, agent=agent)

    asyncio.run(asyncio.wait_for(scenario(), 120))
    r, agent = out["replies"], out["agent"]
    assert _rows(r[0]) == 1 and not r[0]["need_depth"] and "episode_done" not in r[0]  # a hold row, no depth asked
    assert _rows(r[1]) == 2 and _rows(r[2]) == L.CONCLUDE_ROWS and r[2]["episode_done"]
    assert _rows(r[4]) == 1 and _rows(r[5]) == L.CONCLUDE_ROWS and r[5]["episode_done"]
    assert _rows(r[7]) == 1 and not r[7]["need_depth"] and "episode_done" not in r[7]
    assert len(script.infos) == 3 and "replay" not in script.infos[0] and script.infos[1]["replay"] == 1
    assert "replay" not in script.infos[2] and False in probes and probes[-1] is False
    summary = json.loads((agent.dir / "agent.json").read_text())
    played = sorted(summary["played"], key=lambda e: e["episode"])
    assert [e["end"] for e in played] == ["finish", "gateway_error", "model_down"]
    first, second, third = played
    assert [x["end"] for x in first["runs"]] == ["gateway_error", "finish"] and "runs" not in second
    assert first["model_waits"][0]["ok"] and first["model_waits"][0]["probes"] >= 2
    assert first["model_waits"][0]["hold_replies"] >= 1 and "model_waits" not in second
    assert not third["model_waits"][0]["ok"] and third["model_waits"][0]["why"] == "the episode ended while waiting"
    assert [w["ok"] for w in summary["model_waits"]] == [True, False] and out["code"] == 0
    kinds = [json.loads(line)["kind"] for line in (agent.dir / "tunnel.jsonl").read_text().splitlines()]
    assert kinds.count("model_down") == 2 and kinds.count("episode_replay") == 1


def test_the_model_probe_is_one_attempt_and_says_what_failed(monkeypatch):
    from hexashell.runtime import llm

    calls = []

    def post(self, payload, timeout_s):
        calls.append((self.retries, json.loads(payload)["model"]))
        return 502, None, "Upstream request failed"

    monkeypatch.setattr(llm.Gateway, "_post", post)
    settings = SimpleNamespace(script="", api="openai", api_base="http://127.0.0.1:9/v1", model="m-1",
                               reasoning_effort="high", max_tokens=8192, stream=True, ca_file="",
                               thinking_display="summarized", api_key=lambda: "sk-local-dummy")
    problem = T.model_probe(settings, timeout_s=5)
    assert calls == [(0, "m-1")] and "502" in problem and "sk-local-dummy" not in problem
    monkeypatch.setattr(llm.Gateway, "_post", lambda self, payload, timeout_s: (
        200, {"choices": [{"message": {"content": "ready"}, "finish_reason": "stop"}]}, None))
    assert T.model_probe(settings, timeout_s=5) is None

    def no_key():
        raise RuntimeError("no API key: /x does not exist")

    assert T.model_probe(SimpleNamespace(**{**settings.__dict__, "api_key": no_key}), timeout_s=5) == "no model key"



def test_an_online_conclusion_waits_while_only_hold_rows_were_sent():
    sess, link = _session("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker", "isaac")
    rec = _Rec()
    sess.open(SETTINGS, rec, lambda *a: None)
    link.stats.update(dispatches=2, hold_replies=2)  # the model failed before the robot moved
    v = sess.conclude(60.0)
    assert v["conclusion"] == "deferred" and not link.concluding  # the guard decides (a replay may follow)
    assert any(e["kind"] == "conclusion_deferred" for e in rec.events)
    link.stats["dispatches"] = 3  # one motion: concluded as a local run (the verdict comes once it ended)
    link.end(None, 0)
    link.set_verdict(dict(ONLINE))
    assert sess.conclude(60.0)["online"] and link.concluding


class StrawScript:
    """The player of the straw case: episode 0 moves, then is done; episode 1 is done at once; then a hang case's
    episode, done at once."""

    def __init__(self):
        self.links = []

    def __call__(self, link, facts, info):
        self.links.append(link)
        if len(self.links) == 1:
            link.act([L.copy_row(link.command)] * 2, False)
        return {"end": "finish", "turns": 1}


def test_the_straw_task_waits_for_the_simulator_to_end_its_episodes(tmp_path, monkeypatch):
    monkeypatch.setattr(T, "AGENTS", tmp_path / "agents")
    script = StrawScript()
    straw = ("g2_90d", "insert_straw_into_milk_tea_cup")
    hang = ("g2_omnipicker", "hang_cups_on_cup_tree_g2_omnipicker")
    out = {}

    async def scenario():
        gate = F.Gate(parallelism=1, quiet=True)
        await gate.start()
        state = T.read_state(gate.write_state(tmp_path / "state.env"))
        agent = T.Agent(_args(tmp_path / "out"), state, player=script, ready_check=lambda: None)
        run = asyncio.create_task(agent.run())
        await asyncio.wait_for(gate.ready.wait(), 10)
        await gate.warm_check(10)
        a = await gate.open_case()
        ask = lambda prompt, idx, done=False: a.ask(packed(request(*straw, prompt, idx=idx, done=done)))  # noqa: E731
        replies = [_result(await ask("Put the straw in", 0))]  # the motion
        replies.append(_result(await ask("Put the straw in", 0)))  # done: held rows, no episode_done
        replies.append(_result(await ask("Put the straw in", 0)))  # still not ended: held rows again
        replies.append(_result(await ask("Put the straw in", 0, done=True)))  # the simulator ends it
        for _ in range(L.LINGER_REPLIES + 1):  # episode 1, done at once: held chunks, then the conclusion
            replies.append(_result(await ask("Put the straw in again", 1)))
        replies.append(_result(await ask("Put the straw in again", 1, done=True)))
        await a.close()
        b = await gate.open_case()
        replies.append(_result(await b.ask(packed(request(*hang, "Hang the cups", idx=0)))))  # concluded as before
        await b.close()
        assert await gate.drain(20)
        code = await asyncio.wait_for(run, 20)
        await gate.stop()
        agent.finish(code)
        out.update(code=code, replies=replies, agent=agent)

    asyncio.run(asyncio.wait_for(scenario(), 120))
    r, agent = out["replies"], out["agent"]
    n = L.LINGER_REPLIES
    assert _rows(r[0]) == 2
    for res in (r[1], r[2], *r[4:4 + n]):
        assert _rows(res) == L.LINGER_ROWS and "episode_done" not in res and not res["need_depth"]
    assert _rows(r[3]) == 1 and "episode_done" not in r[3]  # the answer to the simulator's end (no dispatch)
    assert _rows(r[4 + n]) == L.CONCLUDE_ROWS and r[4 + n]["episode_done"]  # the cap: the normal conclusion
    assert _rows(r[-1]) == L.CONCLUDE_ROWS and r[-1]["episode_done"]
    first, second, third = script.links
    assert first.linger and second.linger and not third.linger
    assert first.stats["linger_replies"] == 2 and first.dispatches == 3 and first.stats["conclude_replies"] == 0
    assert second.stats["linger_replies"] == n and second.dispatches == n + 1
    assert third.stats["linger_replies"] == 0 and third.dispatches == 1
    summary = json.loads((agent.dir / "agent.json").read_text())
    played = sorted(summary["played"], key=lambda e: (e["case"], e["episode"]))
    assert [e.get("waits_for_simulator_end") for e in played] == [True, True, None] and out["code"] == 0
    kinds = [json.loads(line).get("reply") for f in sorted((agent.dir / "cases").glob("c000_*/link.jsonl"))
             for line in f.read_text().splitlines() if '"request"' in line]
    assert kinds == ["action", "linger", "linger", "done", *["linger"] * n, "conclude", "done"]

def test_a_refused_dial_exits_2(tmp_path, monkeypatch):
    monkeypatch.setattr(T, "AGENTS", tmp_path / "agents")

    async def scenario():
        gate = F.Gate(quiet=True)
        await gate.start()
        state = {**gate.state_values(), "JOB_TOKEN": "not-the-token"}
        agent = T.Agent(_args(tmp_path / "out"), state, player=Script(), ready_check=lambda: None)
        code = await asyncio.wait_for(agent.run(), 20)
        await gate.stop()
        agent.finish(code)
        return code, gate

    code, gate = asyncio.run(scenario())
    assert code == 2 and any(e["kind"] == "dial_refused" and not e["token_ok"] for e in gate.events)


# -- the platform's API (no call leaves the test) -------------------------------------------------------
def test_submit_is_a_dry_run_without_yes(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(C, "STATE", tmp_path / "state.env")
    monkeypatch.setattr(C, "JOBS", tmp_path / "jobs")
    C.write_env(C.STATE, {"CHALLENGE_TOKEN": "tok-123"})
    sent = []
    job = {"id": 4242, "uuid": "job-uuid-77", "parallelism": 2, "tunnel_endpoint": "ws://10.9.8.7:81/api/challenge/tunnel",
           "job_token": "jt-999", "board": "v2"}

    def call(method, path, token=None, body=None, timeout=30.0):
        sent.append((method, path.split("?")[0], body))
        if path.startswith("/api/challenge/submission/quota"):
            return {"limit": 3, "used": 1, "remaining": 2}
        if path.startswith("/api/challenge/current-user-info"):
            return {"name": "n", "model_name": "HexaShell", "model_name_approved": True, "model_type": "agent",
                    "email": "e@x"}
        if path.startswith("/api/challenge/jobs"):
            assert "season=2" in path
            return {"items": []}
        if method == "POST" and path == "/api/challenge/job":
            return job
        raise AssertionError(path)

    monkeypatch.setattr(C, "call", call)
    assert C.main(["submit", "--name", "hexashell-s2", "--description", "d"]) == 0
    printed = capsys.readouterr().out
    assert "dry run" in printed and '"board": "v2"' in printed and "e@x" not in printed
    assert all(m == "GET" for m, _, _ in sent)
    assert C.main(["submit", "--name", "hexashell-s2", "--yes"]) == 0
    printed = capsys.readouterr().out
    assert sent[-1] == ("POST", "/api/challenge/job", {"name": "hexashell-s2", "config": {"board": "v2"}})
    for secret in ("job-uuid-77", "jt-999", "10.9.8.7", "tok-123"):
        assert secret not in printed
    saved = T.read_state(C.job_file())
    assert saved["JOB_SEASON"] == "2" and saved["JOB_BOARD"] == "v2" and saved["JOB_TOKEN"] == "jt-999"
    assert saved["CHALLENGE_TOKEN"] == "tok-123" and T.season_problem(saved) is None
    assert T.read_state(C.STATE)["JOB_ID"] == "4242"
    link = "https://github.com/HexaFuture/HexaShell"
    assert C.main(["submit", "--name", "hexashell-s2", "--paper-link", link, "--yes"]) == 0
    assert sent[-1] == ("POST", "/api/challenge/job", {"name": "hexashell-s2", "config": {"board": "v2", "paper_link": link}})
    problems = C.submit_problems({"remaining": 0}, {"model_name_approved": False, "model_type": None}, "running")
    assert len(problems) == 4


def test_submit_can_leave_an_open_job_and_keeps_its_job_file(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(C, "STATE", tmp_path / "state.env")
    monkeypatch.setattr(C, "JOBS", tmp_path / "jobs")
    C.write_env(C.STATE, {"CHALLENGE_TOKEN": "tok-123", "JOB_ID": "4241"})
    C.write_env(C.job_file(), {"CHALLENGE_TOKEN": "tok-123", "JOB_ID": "4241", "JOB_UUID": "old-uuid", "JOB_BOARD": "v2"})
    sent = []

    def call(method, path, token=None, body=None, timeout=30.0):
        sent.append((method, path.split("?")[0], body))
        if path.startswith("/api/challenge/submission/quota"):
            return {"limit": 3, "used": 1, "remaining": 2}
        if path.startswith("/api/challenge/current-user-info"):
            return {"model_name": "HexaShell", "model_name_approved": True, "model_type": "agent"}
        if path.startswith("/api/challenge/jobs"):
            return {"items": [{"id": 4241, "detailed_status": "inference_disconnected"}]}
        if method == "POST" and path == "/api/challenge/job":
            return {"id": 4242, "uuid": "new-uuid", "parallelism": 4, "tunnel_endpoint": "ws://10.9.8.7:81/t",
                    "job_token": "jt-1", "board": "v2"}
        raise AssertionError(path)

    monkeypatch.setattr(C, "call", call)
    assert C.main(["submit", "--name", "n", "--yes"]) == 1  # the open job stops it
    assert "--leave-previous-open" in capsys.readouterr().out and sent[-1][0] == "GET"
    assert C.main(["submit", "--name", "n", "--leave-previous-open"]) == 0  # a dry run keeps nothing yet
    assert not (C.JOBS / "v2_job4241.env").exists() and sent[-1][0] == "GET"
    assert C.main(["submit", "--name", "n", "--leave-previous-open", "--yes"]) == 0
    printed = capsys.readouterr().out
    kept = C.JOBS / "v2_job4241.env"
    assert C.read_env(kept)["JOB_UUID"] == "old-uuid" and kept.stat().st_mode & 0o777 == 0o600
    assert C.read_env(C.job_file())["JOB_ID"] == "4242" and "old-uuid" not in printed and "new-uuid" not in printed


# -- online.sh --------------------------------------------------------------------------------------
def _sh(*args, env=None, cwd=None):
    return subprocess.run(["bash", str(ONLINE_SH), *map(str, args)], capture_output=True, text=True, timeout=30,
                          env={**os.environ, "HEXASHELL_PYTHON": "/bin/true", **(env or {})}, cwd=cwd)


def test_online_sh_checks_the_state_file(tmp_path):
    state = tmp_path / "v2.env"
    C.write_env(state, {"JOB_UUID": "u", "JOB_TOKEN": "t", "PARALLELISM": 2, "JOB_BOARD": "manip"})
    assert _sh(1, tmp_path / "o1", state).returncode == 1  # a Season I job
    C.write_env(state, {"JOB_BOARD": "v2", "JOB_SEASON": "2"})
    assert _sh(3, tmp_path / "o2", state).returncode == 1  # over its parallelism
    os.chmod(state, 0o644)
    assert _sh(1, tmp_path / "o3", state).returncode == 1  # readable by others
    os.chmod(state, 0o600)
    done = _sh(2, tmp_path / "o4", state)  # /bin/true stands in for python: nothing dials
    assert done.returncode == 0 and len((tmp_path / "o4" / "pids").read_text().split()) == 2
    assert "u" not in done.stdout.replace("unknown", "").split() and "t" not in done.stdout.split()
    assert _sh(1, tmp_path / "o4", state).returncode == 1  # OUT exists
    rel = _sh(1, "o5", "v2.env", cwd=tmp_path)  # relative paths: the caller's, though the agents run from the checkout
    assert rel.returncode == 0 and (tmp_path / "o5" / "slot_0.out").exists() and (tmp_path / "o5" / "pids").exists()
