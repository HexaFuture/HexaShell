"""benches/robocoliseum (Season II): the task catalog and split (protocol.py), the dispatch budget (budget.py)
and the container run script (sim.py)."""

import shlex

import pytest

from hexashell.benches.robocoliseum import budget as B
from hexashell.benches.robocoliseum import protocol as P
from hexashell.benches.robocoliseum import sim as S


def test_catalog_sizes():
    assert len(P.TASKS) == 50
    assert {b: len(P.configs((b,))) for b in P.BOARDS} == {"finemanip": 8, "crsemb": 24, "wbc": 8, "dexmanip": 10}
    runtimes = [t.runtime for t in P.TASKS.values()]
    assert {r: runtimes.count(r) for r in P.RUNTIMES} == {"isaac": 45, "libuipc": 3, "newton": 1, "mujoco": 1}
    for t in P.TASKS.values():
        assert t.config == f"{P.ROBOTS[t.robot].config_robot}_{t.board}_{t.task}"
        assert (t.physics_hz is not None) == (t.runtime == "isaac")


def test_crsemb_robots_and_rates():
    robots = {P.TASKS[c].robot for c in P.configs(("crsemb",))}
    assert robots == {"g2_omnipicker", "g1", "arx", "agilex"}
    write = {P.TASKS[c].robot: (P.TASKS[c].physics_hz, P.TASKS[c].render_hz)
             for c in P.configs(("crsemb",)) if P.TASKS[c].task == "write_uppercase_letter_with_pen"}
    assert write == {"g2_omnipicker": (60, 30), "g1": (60, 30), "arx": (240, 30), "agilex": (240, 30)}
    pour = [P.TASKS[c] for c in P.configs(("crsemb",)) if P.TASKS[c].task.startswith("pour_")]
    assert all(t.render_mode == "PathTracing" and t.physics_hz == 240 for t in pour)
    assert len({P.TASKS[c].sub_task for c in P.configs(("crsemb",)) if "trash" in c}) == 1


def test_robots_match_the_wire_table():
    wire = {k: (r.robot_type, r.ee_type, r.arm_joints, r.effector) for k, r in P.ROBOTS.items()}
    assert wire == {
        "g2_90d": ("g2a_sim", "zhiyuan_gripper_ctek90d", 7, 1),
        "g2_omnipicker": ("g2a_sim", "zhiyuan_gripper_omnipicker", 7, 1),
        "g1": ("a2d_sim", "zhiyuan_gripper_omnipicker", 7, 1),
        "arx": ("arx_sim", "arx_gripper", 6, 1),
        "agilex": ("agilex_sim", "agilex_gripper", 6, 1),
        "g2_wuji": ("g2a_sim", "wuji", 7, 20),
    }
    # every robot moves both arms (the scoring of some ARX and Agilex instances names the left gripper)
    assert all(r.arms == ("left", "right") for r in P.ROBOTS.values())
    assert not P.ROBOTS["arx"].eef and not P.ROBOTS["agilex"].eef
    assert [P.TASKS[c].mobile for c in P.configs(("wbc",))] == [True] * 8
    assert not any(P.TASKS[c].mobile for c in P.configs(("finemanip", "crsemb", "dexmanip")))


def test_split_counts_and_parts():
    # 847bb9e: 10 instances an Isaac task (083793f: 20, then 190 / 190 / 578)
    assert {p: len(P.all_cases(p)) for p in P.PARTS} == {"learn": 100, "validation": 100, "test": 308}
    keys = [c.key for p in P.PARTS for c in P.all_cases(p)]
    assert len(keys) == len(set(keys))
    isaac = P.configs(("finemanip",))[0]
    assert [c.instance for c in P.cases(isaac, "learn")] == [0, 5]
    assert [c.instance for c in P.cases(isaac, "validation")] == [1, 6]
    assert [c.instance for c in P.cases(isaac, "test")] == [2, 3, 4, 7, 8, 9]
    cloth = "g2crsbwuji_dexmanip_fold_cloth"
    assert [c.instance for c in P.cases(cloth, "learn")] == [0, 5]
    assert [c.instance for c in P.cases(cloth, "test")] == [2, 3, 4, 7, 8, 9]
    for config in ("g2crsbwuji_dexmanip_wok_flip", "g2crsbwuji_dexmanip_rubik_cube_solve"):
        assert [c.instance for c in P.cases(config, "test")] == list(range(10))
        assert all(c.official for c in P.cases(config, "test"))
        own = P.cases(config, "learn") + P.cases(config, "validation")
        assert len(own) == 4 and not any(c.official for c in own)
    wok = P.cases("g2crsbwuji_dexmanip_wok_flip", "learn")
    assert {c.seed for c in wok}.isdisjoint({P.TASKS["g2crsbwuji_dexmanip_wok_flip"].seed})
    rubik = P.cases("g2crsbwuji_dexmanip_rubik_cube_solve", "validation")
    assert {c.archive for c in rubik} == {"rubik_2turn_10.zip"} != {P.RUBIK_SCORED_ARCHIVE}
    with pytest.raises(P.TaskError):
        P.cases(isaac, "train")
    with pytest.raises(P.TaskError):
        P.task("g2op_if_pick_block_color")


@pytest.mark.skipif(not (P.PKG / "config").is_dir(), reason="no genie_sim 3.6 source on this machine")
def test_catalog_matches_the_benchmark():
    assert P.check_catalog() == []


def test_budget():
    assert B.baseline(450) == 12 and B.baseline(600) == 16 and B.baseline(2700) == 72
    assert B.baseline(30) == 1 and B.baseline(59) == 1
    assert B.share(450, 0) == B.share(450, 12) == 1.0
    assert B.share(450, 13) == pytest.approx(10 / 12)
    assert B.share(450, B.last_scoring(450)) > 0 and B.share(450, B.last_scoring(450) + 1) == 0
    with pytest.raises(ValueError):
        B.baseline(29)
    full = [{"points": 20, "score": 1.0, "step_out": 450, "count": 12},
            {"points": 80, "score": 1.0, "step_out": 750, "count": 20}]
    assert B.episode_score(full) == pytest.approx(1.0)
    over = [dict(full[0], count=13), full[1]]
    assert B.episode_score(over) == pytest.approx((20 * 10 / 12 + 80) / 125)
    unfinished = [full[0], dict(full[1], score=0.0, count=None)]
    assert B.episode_score(unfinished) == pytest.approx(20 / 125)


def test_run_script(monkeypatch):
    case = P.Case("g2op_crsemb_hang_cups_on_cup_tree", 5)
    text = S.script(case, 18781, "run_a", 51, watchdog_s=1800)
    words = shlex.split(text)
    assert "GENIESIM_ROBOT_INITIALIZATION_TIMEOUT_SECONDS=1800" in words
    assert "GENIESIM_BENCHMARK_OUTPUT_ROOT=/workspace/output/hexashell/run_a" in words
    assert "ROS_DOMAIN_ID=51" in words and "API_KEY=sk-local-dummy" in words
    assert not any(w.startswith("GENIESIM_POUR_TRACE") for w in words)
    inner = shlex.split(words[-1])
    assert inner[:5] == ["echo", "$$", ">", "/workspace/output/hexashell/run_a/pid;", "exec"]
    assert inner[5:] == ["geniesim", "benchmark", "run", "g2op_crsemb_hang_cups_on_cup_tree",
                         "--infer-host=127.0.0.1:18781", "--benchmark.num_episode=1", "--benchmark.instance_ids=5",
                         "--app.main_loop_stall_timeout_s=1800"]
    monkeypatch.setenv("GENIESIM_POUR_TRACE", "1")  # a developer trace switch reaches the run only when set
    assert "GENIESIM_POUR_TRACE=1" in shlex.split(S.script(case, 18781, "run_a", 51, watchdog_s=1800))
    for bad in ("", "a/b", ".."):
        with pytest.raises(ValueError):
            S.result_root(bad)
    with pytest.raises(ValueError):  # a cloth instance is picked by our copy of its config, not by a flag
        S.instance_args(P.Case("g2crsbwuji_dexmanip_fold_cloth", 0, seed=3))
    with pytest.raises(ValueError):
        S.instance_args(P.Case("g2op_crsemb_hang_cups_on_cup_tree", 5, seed=3))
    assert S.container_for(3) == "hexashell-geniesim-gpu3"
