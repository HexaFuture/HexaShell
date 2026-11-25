"""RoboColiseum Season II batches: what a plan holds, how an attempt is judged."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from hexashell.benches.robocoliseum import batch as B
from hexashell.benches.robocoliseum import protocol as P


def test_plan_of_the_learning_part():
    units = B.plan_units(["learn"], ("finemanip", "crsemb"))
    assert len(units) == 8 * 2 + 24 * 2  # 847bb9e: 10 instances an Isaac task, learning 0 and 5
    assert len({u["id"] for u in units}) == len(units)
    assert all(P.part_of(u["instance"]) == "learn" for u in units)
    assert [u["board"] for u in units] == sorted((u["board"] for u in units), key=B.PRIORITY.get)
    assert {u["robot"] for u in units} == {"g2_90d", "g2_omnipicker", "g1", "arx", "agilex"}


def test_plan_skips_what_the_bench_cannot_play(capsys, monkeypatch):
    monkeypatch.setattr(B, "playable", lambda robot: robot != "g2_wuji")
    with pytest.raises(SystemExit, match="no units"):
        B.plan_units(["validation"], ("dexmanip",))
    err = capsys.readouterr().err
    assert "robot g2_wuji" in err and "runtime" not in err  # every runtime is wired


def test_plan_of_the_dexterous_board():
    units = B.plan_units(["validation"], ("dexmanip",))
    assert {u["robot"] for u in units} == {"g2_wuji"} and {u["runtime"] for u in units} == set(P.RUNTIMES)
    isaac = ["cover_cup_lid", "pick_up_a_package", "stack_bowls", "take_the_book", "take_the_scissors"]
    units = B.plan_units(["learn"], ("dexmanip",), tasks=isaac, arms=["sk"])
    assert [u["instance"] for u in units] == [0, 5] * 5
    assert {u["runtime"] for u in units} == {"isaac"} and {u["task"] for u in units} == set(isaac)


def test_plan_arms_and_filters():
    units = B.plan_units(["learn"], ("crsemb",), robots=["g1"], tasks=["pour_50ml_into_graduated_cylinder"],
                         arms=["sk", "ns"])
    assert [u["id"] for u in units] == [f"g1_crsemb_pour_50ml_into_graduated_cylinder_i{i:02d}_{a}"
                                        for i in (0, 5) for a in ("ns", "sk")]
    assert all(u["env"]["HEXASHELL_SKILLS"] == ("1" if u["arm"] == "sk" else "0") for u in units)
    with pytest.raises(SystemExit):
        B.plan_units(["test"], ("crsemb",), repeats=2)


def test_judge(tmp_path):
    root = tmp_path / "b"
    root.mkdir()
    (root / "units.json").write_text(json.dumps([{"id": "u", "config": "c", "instance": 0}]))
    batch = B.Batch(root)
    run = root / "units" / "u" / "a1" / "run"
    run.mkdir(parents=True)
    assert batch.judge("u", {"n": 1}, 1) == (False, "rc 1, no run.json")
    data = {"end": "sim_exit", "result_files": ["evaluate_ret_00.json"],
            "results": [{"episode": 0, "valid": True, "score": 0.16, "progress": 0.2, "dispatches": 35}]}
    (run / "run.json").write_text(json.dumps(data))
    ok, why = batch.judge("u", {"n": 1}, 0)
    assert ok and "score 0.16" in why
    (run / "run.json").write_text(json.dumps({**data, "result_files": []}))
    assert not batch.judge("u", {"n": 1}, 0)[0]
    (run / "run.json").write_text(json.dumps({**data, "results": [{**data["results"][0], "valid": False}]}))
    assert not batch.judge("u", {"n": 1}, 0)[0]


def test_judge_wants_a_score(tmp_path):
    root = tmp_path / "b"
    root.mkdir()
    (root / "units.json").write_text(json.dumps([{"id": "u", "config": "c", "instance": 0}]))
    batch = B.Batch(root)
    run = root / "units" / "u" / "a1" / "run"
    run.mkdir(parents=True)
    # cloth_hanger i05: the run went to its end, the result file the benchmark left held no score
    data = {"end": "sim_exit_hung", "result_files": ["evaluate_ret_00.json"], "scored_files": [],
            "results": [{"episode": 0, "valid": None, "score": None, "progress": None, "dispatches": None}]}
    (run / "run.json").write_text(json.dumps(data))
    ok, why = batch.judge("u", {"n": 1}, 1)
    assert not ok and "scored 0" in why
    (run / "run.json").write_text(json.dumps({**data, "scored_files": ["evaluate_ret_00.json"]}))
    assert batch.judge("u", {"n": 1}, 0) == (False, "no verdict for the episode")
    scored = {**data, "scored_files": ["evaluate_ret_00.json"],
              "results": [{"episode": 0, "valid": True, "score": 0.0, "progress": 0.0, "dispatches": 61}]}
    (run / "run.json").write_text(json.dumps(scored))
    assert batch.judge("u", {"n": 1}, 0)[0]


def _batch_with(tmp_path, units, running):
    root = tmp_path / "b"
    root.mkdir()
    (root / "units.json").write_text(json.dumps(units))
    batch = B.Batch(root)
    for uid in running:
        batch.state[uid] = {"status": "running", "attempts": [{"n": 1, "host": "h", "gpu": "6", "t0": 0.0}]}
    return batch


def test_next_queued_keeps_a_runtime_at_its_cap(tmp_path):
    units = [{"id": "bag", "config": "c", "instance": 0, "runtime": "libuipc"},
             {"id": "fold", "config": "c", "instance": 5, "runtime": "libuipc"},
             {"id": "rubik", "config": "c", "instance": 1, "runtime": "mujoco"},
             {"id": "wok", "config": "c", "instance": 0, "runtime": "newton"}]
    batch = _batch_with(tmp_path, units, ["bag"])
    host = B.Host({"name": "h", "ssh": "", "code": "/c", "python": "python", "gpus": {"4": 1, "6": 1},
                   "runtime_max": {"libuipc": 1}})
    running = batch.running_on("h")
    assert running == ["bag"]
    assert batch.next_queued(host, running) == "rubik"  # fold waits: one libuipc run is on the machine
    batch.state["rubik"]["status"] = "done"
    assert batch.next_queued(host, running) == "wok"
    batch.state["bag"]["status"] = "done"
    assert batch.next_queued(host, batch.running_on("h")) == "fold"
    # without runtime_max: the order alone
    free = B.Host({"name": "h", "ssh": "", "code": "/c", "python": "python", "gpus": {"4": 1}})
    batch.state["bag"]["status"] = "running"
    assert batch.next_queued(free, batch.running_on("h")) == "fold"
    for uid in ("fold", "wok"):
        batch.state[uid]["status"] = "failed"
    assert batch.next_queued(host, batch.running_on("h")) is None


def test_libuipc_runs_wait_longer_for_the_end_and_the_verdict():
    from hexashell.benches.robocoliseum import link as L
    from hexashell.benches.robocoliseum import run as R

    # the benchmark gives the libuipc stream scorer 3600 s since 847bb9e (900 s before)
    assert R.exit_grace_s("libuipc") > 3600.0 > 900.0 > R.exit_grace_s("isaac") == R.EXIT_GRACE_S
    assert L.verdict_wait_s("libuipc") > 3600.0 > 900.0 > L.verdict_wait_s("newton") == L.VERDICT_S


def test_report_finds_the_records_of_units_with_a_variant(tmp_path):
    # rubik and wok: the record names the variant, <config>_rubik_2turn_10_seed1001_i01_e0-<time>
    units = [{"id": "rubik", "config": "c_rubik_cube_solve", "instance": 1},
             {"id": "bag", "config": "c_drawstring_bag", "instance": 0}]
    batch = _batch_with(tmp_path, units, [])
    for uid, rec in (("rubik", "c_rubik_cube_solve_rubik_2turn_10_seed1001_i01_e0-20261115T170553"),
                     ("bag", "c_drawstring_bag_i00_e0-20261115T181902")):
        batch.state[uid] = {"status": "done", "attempts": [{"n": 1, "host": "h", "gpu": "4", "t0": 0.0}]}
        d = Path(batch.adir(uid, 1)) / rec
        d.mkdir(parents=True)
        (d / "result.json").write_text(json.dumps({"task": uid, "score": 0.16, "verdict": {"stages": []}}))
    rows = batch.report().read_text().splitlines()
    assert [r.split("\t")[0] for r in rows[1:]] == ["rubik", "bag"]
