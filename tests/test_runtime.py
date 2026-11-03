"""Runtime checks that need neither the simulator nor the model.

Run: ``python tests/test_runtime.py`` (or with pytest) from the HexaShell checkout, in the
policy environment.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from hexashell.runtime import card, conversation, episode  # noqa: E402
from hexashell.runtime.episode import Settings  # noqa: E402

LADDER_TASKS = 42


def test_robodojo_prompt_assembles():
    text = card.system_prompt("robodojo")
    for n in range(1, 7):
        assert f"# {n}. " in text, n
    assert 10_000 < len(text) < 37_000, len(text)
    # the free gap formula, not the squeeze one
    assert "10.8·g - 1.9 cm" in text


def test_ladders_match_manifest():
    folder = ROOT / "prompts" / "ladders"
    manifest = json.loads((folder / "manifest.json").read_text())
    names = [p for p in folder.glob("*.md") if p.name != "README.md"]
    assert len(names) == LADDER_TASKS, len(names)
    for path in names:
        assert manifest["files"][path.name] == hashlib.sha256(path.read_bytes()).hexdigest(), path.name
        text = card.ladder(path.stem)
        assert text and "### Scoring" in text and "### Description" in text, path.name
        assert "### Notes" not in text
    assert card.ladder("stack_bowls_random") == card.ladder("stack_bowls")
    assert card.ladder("no_such_task") is None


def test_conversation_window():
    conv = conversation.Conversation("sys", images_max=6, images_low=4, bytes_max=10**9)
    img = (np.random.default_rng(0).random((48, 64, 3)) * 255).astype(np.uint8)
    pinned = [conv.picture(img, f"start {i}", pinned=True) for i in range(3)]
    conv.user("card", pinned)
    for i in range(5):
        conv.user(f"turn {i}", [conv.picture(img, f"look {i}")])
    assert conv.live_counts()[0] == 8
    dropped = conv.trim()
    assert dropped == [4, 5, 6, 7], dropped  # oldest unpinned, down to 4 live
    assert conv.live_counts()[0] == 4
    wire = conv.wire()
    assert all("_picture" not in json.dumps(m) for m in wire)
    flat = json.dumps(wire)
    assert flat.count('"type": "image_url"') == 4
    assert "[picture 4 removed" in flat
    assert conv.trim() == []  # within limits now
    rec = conv.for_record()
    assert "base64" not in json.dumps(rec)


def test_conversation_window_bytes():
    """The byte limit counts the data URIs as sent (base64: 4/3 of the JPEG), not the JPEG bytes."""
    img = (np.random.default_rng(0).random((48, 64, 3)) * 255).astype(np.uint8)
    probe = conversation.Conversation("sys").picture(img, "probe")
    assert probe.wire_size == len(probe.url)
    conv = conversation.Conversation("sys", images_max=90, images_low=63, bytes_max=4 * len(probe.url))
    for i in range(4):
        conv.user(f"turn {i}", [conv.picture(img, f"look {i}")])
    assert conv.trim() == []  # 4 data URIs: at the limit
    conv.user("turn 4", [conv.picture(img, "look 4")])
    assert 5 * len(probe.data) < conv.bytes_max < conv.live_counts()[1]  # under it as JPEG, over it as sent
    assert conv.trim() == [1, 2, 3]  # down to 70% of the bytes: 2 left


def test_dropped_pictures_clear_later_thinking():
    """Claude's thinking blocks are bound to everything before them: when pictures are dropped, the
    replies after the first changed message lose theirs (text and calls stay); earlier ones keep them."""
    from hexashell.runtime import llm

    conv = conversation.Conversation("sys", images_max=2, images_low=1, bytes_max=10**9)
    img = (np.random.default_rng(0).random((48, 64, 3)) * 255).astype(np.uint8)

    def think(n):
        return {"type": "thinking", "thinking": f"thought {n}", "signature": f"sig{n}"}

    conv.user("card")
    conv.assistant("a", [], [think(0), {"type": "text", "text": "a"}])
    conv.user("turn 1", [conv.picture(img, "look 1")])
    conv.assistant("b", [], [think(1), {"type": "text", "text": "b"}])
    conv.user("turn 2", [conv.picture(img, "look 2")])
    conv.assistant("", [], [think(2)])
    conv.user("turn 3", [conv.picture(img, "look 3")])
    assert conv.trim() == [1, 2]
    assert [m["_blocks"] for m in conv.messages if "_blocks" in m] == [
        [think(0), {"type": "text", "text": "a"}], [{"type": "text", "text": "b"}], []]
    assert conv.thinking_cleared == 2
    _, turns = llm.to_anthropic(conv.wire())
    assert turns[5] == {"role": "assistant", "content": [{"type": "text", "text": "(no reply)"}]}


def test_task_card_skill():
    import fakes
    from hexashell.state import parse_obs

    old = card.SKILLS
    tmp = Path(tempfile.mkdtemp())
    try:
        card.SKILLS = tmp
        (tmp / "stack_bowls.md").write_text("Nest the bowls one by one.\n", encoding="utf-8")
        for task in ("stack_bowls", "stack_bowls_random"):
            first = parse_obs(fakes.FakeTable([], task=task).obs(), seq=1)
            text, used = card.task_card(first, with_ladder=False, with_skill=True)
            assert used["skill"] and "## Notes for this task" in text and "Nest the bowls one by one." in text, task
            text, used = card.task_card(first, with_ladder=False, with_skill=False)
            assert not used["skill"] and "Nest the bowls" not in text, task
        first = parse_obs(fakes.FakeTable([], task="push_T").obs(), seq=1)
        text, used = card.task_card(first, with_ladder=False, with_skill=True)
        assert not used["skill"] and "## Notes for this task" not in text
    finally:
        card.SKILLS = old
        shutil.rmtree(tmp)


def test_settings_from_env():
    old = dict(os.environ)
    try:
        os.environ["HEXASHELL_MAX_TURNS"] = "7"
        os.environ["HEXASHELL_SKILLS"] = "yes"
        os.environ["HEXASHELL_BUDGET_S"] = "60"
        s = Settings.from_config({"max_turns": 50, "ladder": False})
        assert s.max_turns == 7 and s.skills is True and s.budget_s == 60.0 and s.ladder is False
    finally:
        os.environ.clear()
        os.environ.update(old)


def test_to_world():
    import fakes
    from hexashell.errors import Refused
    from hexashell.geometry import homogeneous
    from hexashell.state import parse_obs
    from hexashell.vision import spots
    from hexashell.vision.cameras import WRIST_FOCAL, Camera, cameras_for

    snap = parse_obs(fakes.FakeTable([]).obs(), 0)
    host = type("Host", (), {"snap": snap, "scene": None})()
    head = cameras_for(snap)["head"]
    top = np.array([0.05, -0.05, 0.815])
    px = head.pixel(top)
    s = spots.to_world(host, "head", px, z=0.815)
    assert np.allclose(s.xyz, top, atol=1e-4) and s.warning is None and 2.0 < s.mm_per_px < 3.5, s
    s = spots.to_world(host, "head", px)
    assert abs(s.xyz[2] - 0.765) < 1e-6 and "table top" in s.source and s.warning, s
    assert spots.to_world(host, "head", px, z=1.5).xyz is None  # a plane above the camera
    # a wrist camera 12 cm above the point, looking straight down
    cams = {"head": head, "right_wrist": Camera("right_wrist", WRIST_FOCAL, homogeneous(np.eye(3), (0.07, -0.06, 0.94)))}
    s = spots.from_pair(cams, "head", px, ("right_wrist", cams["right_wrist"].pixel(top)))
    assert np.allclose(s.xyz, top, atol=1e-4) and s.gap_mm < 0.5 and s.angle_deg > 20 and s.warning is None, s
    for wrong in ((300, 100), (400, 300), (100, 400)):
        s = spots.from_pair(cams, "head", px, ("right_wrist", wrong))
        assert s.xyz is None or "do not show the same point" in (s.warning or ""), s
    for bad in [dict(xy=(700, 10)), dict(xy=(1, 2, 3)), dict(xy=(5, 5), z="high"), dict(xy=(5, 5), z=0.8, pair=("left", (1, 1)))]:
        xy = bad.pop("xy")
        try:
            spots.to_world(host, "head", xy, **bad)
        except Refused:
            continue
        raise AssertionError(f"accepted {xy} {bad}")
    try:
        spots.to_world(host, "top", (5, 5))
    except Refused as exc:
        assert "camera must be" in str(exc)
    else:
        raise AssertionError("accepted camera 'top'")


def _run_scripted(script, *, boxes=None, env=None, step_limit=400, table=None):
    """A whole episode against the fake table (or ``table``) with scripted replies; returns the record
    folder."""
    import fakes
    from hexashell.xpl import adapter

    out = Path(tempfile.mkdtemp(prefix="hexashell-test-"))
    saved = dict(os.environ)
    os.environ.update({"HEXASHELL_RESULT_DIR": str(out), "HEXASHELL_CELL_CPU_S": "5", "HEXASHELL_CELL_IDLE_S": "8",
                       "HEXASHELL_SAM3_URL": "", **(env or {})})
    real = episode.Gateway
    fakes.ScriptedGateway.script = script
    fakes.ScriptedGateway.seen = []
    episode.Gateway = fakes.ScriptedGateway
    try:
        table = table or fakes.FakeTable(boxes or [fakes.Box("red block", (0.05, -0.05, 0.79), (0.05, 0.05, 0.05))],
                                         step_limit=step_limit)
        host = adapter.PolicyHost({"policy_name": "HexaShell", "action_type": "joint", "poll_s": 0.2,
                                   "wrap_up_s": 60})
        verdict = fakes.drive(table, host)
        if host.thread is not None:
            host.thread.join(60)
    finally:
        episode.Gateway = real
        os.environ.clear()
        os.environ.update(saved)
    runs = list(out.iterdir())
    assert len(runs) == 1, runs
    return runs[0], verdict, fakes.ScriptedGateway.seen


def test_history_reasoning():
    """With history_reasoning on, each reply's reasoning goes back with it in the next requests; off, it does not."""
    import fakes

    script = [("first thought", [fakes.python_call("print(1)")]), ("second thought", [fakes.finish_call()])]
    for on in (True, False):
        _, _, seen = _run_scripted(script, env={"HEXASHELL_HISTORY_REASONING": "1" if on else "0"})
        sent = [m for m in seen[1] if m["role"] == "assistant"]
        assert sent and all(set(m) <= {"role", "content", "tool_calls", "reasoning"} for m in sent)
        assert [m.get("reasoning") for m in sent] == (["first thought"] if on else [None])


def test_scripted_episode():
    import fakes

    script = [
        [fakes.python_call("print(robot.task)\nprint(robot.state())\nrobot.look('all', tile=True)")],
        [fakes.python_call("m = robot.goto('right', (0.15, -0.15, 0.95), pitch=90)\nprint(m)\n"
                           "g = robot.gripper('right', 'close')\nprint(g)\nprint(robot.goto('right', by=(0, 0, 0.03)))"),
         fakes.python_call("try:\n    robot.goto('left', (3, 3, 3))\nexcept NotExecuted as e:\n    print('caught', e)\n"
                           "print(robot.goto('left', (3, 3, 3), dry_run=True))\n"
                           "print(robot.goto('left', (-0.2, -0.2, 0.9), pitch=60, dry_run=True))\nprint(robot.wait(3))\n"
                           "robot.goto('left', (0.0, 0.0, 0.70))\nprint('went on')")],
        "I am thinking without a tool call.",
        [fakes.python_call("import os")],
        [fakes.python_call("open('/etc/passwd').read()")],
        [fakes.python_call("x = 0\nwhile True:\n    x += 1")],
        [fakes.python_call("print(x > 0)\nprint(robot.home())\nprint(robot.state())")],
        [fakes.python_call("robot.look('head', grid=True, marks=[('world', (0.05, -0.05, 0.79), 'block')])\n"
                           "print(robot.to_pixel((0.05, -0.05, 0.79), 'head'))\n"
                           "print(robot.to_world('head', (343, 216), z=0.79))\nrobot.camera('top')")],
        [fakes.finish_call("stuck", "scripted test")],
    ]
    folder, verdict, seen = _run_scripted(script, env={"HEXASHELL_MAX_TURNS": "20"})
    result = json.loads((folder / "result.json").read_text())
    assert result["end"] == "finish", result
    assert result["finish"]["status"] == "stuck"
    assert result["turns"] == 9 and result["cells"] == 8, (result["turns"], result["cells"])
    assert verdict["step"] > 20, verdict
    cells = [json.loads(line) for line in (folder / "cells.jsonl").read_text().splitlines()]
    by_cell = {c["cell"]: c for c in cells}
    assert "Move right: at" in by_cell[2]["output"], by_cell[2]["output"]
    assert "closed fully (empty" in by_cell[2]["output"]
    # a refused goto raises NotExecuted (a RobotError the cell can catch); a dry run only reports
    c3 = by_cell[3]
    assert "caught Move left: NOT executed" in c3["output"], c3["output"]
    assert "Move left (dry run): NOT reachable" in c3["output"] and c3["output"].count("(dry run)") == 2
    assert "Waited 3 steps" in c3["output"] and "went on" not in c3["output"], c3["output"]
    assert "NotExecuted: Move left: NOT executed" in c3["error"] and "below the table top" in c3["error"], c3
    assert by_cell[4]["error"] and "os" in by_cell[4]["error"]
    assert by_cell[5]["error"], by_cell[5]
    assert by_cell[6]["error"] and "CellTimeout" in by_cell[6]["error"], by_cell[6]
    assert "True" in by_cell[7]["output"] and "Home:" in by_cell[7]["output"], by_cell[7]
    assert "left home" in by_cell[7]["output"] and "right home" in by_cell[7]["output"], by_cell[7]["output"]
    assert "Spot (0.05" in by_cell[8]["output"] and "plane z = 0.790" in by_cell[8]["output"], by_cell[8]
    assert by_cell[8]["error"].splitlines()[-1].startswith("ValueError: camera must be one of"), by_cell[8]["error"]
    pics = sorted(p.name for p in (folder / "pictures").iterdir())
    assert len(pics) == 5, pics  # three start pictures, one tile, one grid
    events = [json.loads(line) for line in (folder / "events.jsonl").read_text().splitlines()]
    assert any(e["kind"] == "nudge" for e in events)
    refused = [e["dry_run"] for e in events if e["kind"] == "goto_refused"]
    assert refused.count(False) == 2 and True in refused, refused
    states = (folder / "states.jsonl").read_text().splitlines()
    assert len(states) >= verdict["step"]
    # what the model saw: the card with its pictures, results with footers
    last = seen[-1]
    assert last[0]["role"] == "system" and "# Task card" in json.dumps(last[1])
    tool_texts = [m["content"] for m in last if m["role"] == "tool"]
    assert all("— step" in t for t in tool_texts), tool_texts
    shutil.rmtree(folder.parent)


def test_script_mode():
    script = Path(tempfile.mkdtemp(prefix="hexashell-script-")) / "capture.py"
    script.write_text("# %% pictures\nimport json\nnp.save('head.npy', robot.image('head'))\nprint(robot.state())\n"
                      "# %%\nprint(robot.goto('right', by=(0, 0, 0.02)))\n")
    folder, verdict, seen = _run_scripted([], env={"HEXASHELL_SCRIPT": str(script)})
    result = json.loads((folder / "result.json").read_text())
    assert result["end"] == "finish" and result["finish"]["status"] == "keep_partial", result
    assert result["cells"] == 2 and result["settings"]["sent"]["model"] == "script:capture.py", result
    assert np.load(folder / "workspace" / "head.npy").shape == (480, 640, 3)
    assert not seen  # the model was never asked
    shutil.rmtree(folder.parent)
    shutil.rmtree(script.parent)


def test_gateway_waits_out_billing():
    """A 403 'insufficient balance' is waited out like a rate limit; other 403s end at once."""
    import time

    from hexashell.runtime import llm

    answers = []

    def post(payload, timeout_s):
        return answers.pop(0)

    ok = (200, {"choices": [{"message": {"content": "hi"}, "finish_reason": "stop"}]}, None)
    broke = (403, None, '{"error": {"message": "insufficient balance", "type": "billing_error"}}')
    saved = llm.BILLING_PAUSE, llm.BILLING_WAIT
    llm.BILLING_PAUSE = (0.01, 1.0, 0.01)
    gw = llm.Gateway("http://127.0.0.1:9", "k", "m")
    gw._post = post
    try:
        answers[:] = [broke, broke, ok]
        reply = gw.ask([], [], deadline=time.monotonic() + 30)
        assert reply.text == "hi" and reply.attempts == 3
        assert 0.01 <= gw.lost_s < 1.0, gw.lost_s  # the two pauses, not the answer
        gw.lost_s = 0.0
        answers[:] = [ok]
        gw.ask([], [], deadline=time.monotonic() + 30)
        assert gw.lost_s == 0.0
        llm.BILLING_WAIT = 0.0
        answers[:] = [broke, broke]
        try:
            gw.ask([], [], deadline=time.monotonic() + 30)
            raise AssertionError("expected a GatewayError")
        except llm.GatewayError as exc:
            assert exc.kind == "failing" and exc.status == 403
        answers[:] = [(403, None, '{"error": {"message": "forbidden"}}')]
        try:
            gw.ask([], [], deadline=time.monotonic() + 30)
            raise AssertionError("expected a GatewayError")
        except llm.GatewayError as exc:
            assert exc.kind == "refused"
    finally:
        llm.BILLING_PAUSE, llm.BILLING_WAIT = saved
        gw.close()


def test_step_limit_ends_episode():
    import fakes

    loop = "for i in range(40):\n    robot.goto('right', by=(0, 0, 0.02 if i % 2 else -0.02))"
    script = [[fakes.python_call(loop)], [fakes.python_call("print('still here')")]]
    folder, verdict, seen = _run_scripted(script, step_limit=60)
    result = json.loads((folder / "result.json").read_text())
    assert verdict["step"] == 60, verdict
    assert result["end"] == "episode_over", result
    cell = json.loads((folder / "cells.jsonl").read_text().splitlines()[0])
    assert cell["stopped"], cell
    shutil.rmtree(folder.parent)


def test_episode_end_names_the_call_it_ended_in():
    """R0 and R1 ended 6 of 6 successes inside home(), which printed nothing, and three analyses took them
    for successes at the lift before.  Now the message names the call and how far into it the episode
    ended, and the events have a verdict event."""
    import re

    import fakes

    class DoneOnTheWayHome(fakes.FakeTable):
        """Succeeds once the right arm, having gone out, is nearly back at its start."""

        def apply(self, action):
            super().apply(action)
            out = float(np.max(np.abs(self.q["right"])))
            self.away = getattr(self, "away", False) or out > 0.4
            self.success = self.success or (self.away and out < 0.2)

    table = DoneOnTheWayHome([fakes.Box("red block", (0.05, -0.05, 0.79), (0.05, 0.05, 0.05))])
    cell = "print(robot.goto('right', (0.15, -0.15, 0.95), pitch=90))\nrobot.home()\nprint('after')"
    folder, verdict, seen = _run_scripted([[fakes.python_call(cell)]], table=table)
    assert verdict["success"], verdict
    record = json.loads((folder / "cells.jsonl").read_text().splitlines()[0])
    found = re.search(r"EpisodeOver: the environment reports SUCCESS: the task is complete "
                      r"\(during robot\.home\(\), (\d+) steps into it\)", record["error"] or "")
    assert found and "after" not in record["output"], record
    events = [json.loads(line) for line in (folder / "events.jsonl").read_text().splitlines()]
    went = next(e for e in events if e["kind"] == "goto")
    ended = next(e for e in events if e["kind"] == "verdict")
    assert ended["during"] == "home" and ended["step"] == verdict["step"], ended
    assert ended["into"] == int(found.group(1)) == verdict["step"] - went["steps"] > 0, (ended, went)
    shutil.rmtree(folder.parent)


def test_session_defaults_keep_the_simulators_words():
    """What a session tells the model unless it says otherwise (the real robot's does) is what the episode
    always said."""
    from hexashell.runtime.session import Session

    session = Session({})
    assert session.tools() is episode.TOOLS and session.nudge_text() == episode.NUDGE
    assert session.finished_text() == "Finishing: the episode ends now and the scene is judged as it stands."
    assert session.timing_note() == "the simulation stands still while you think."
    assert session.report(None) == {} and session.report({"status": "stuck", "summary": ""}) == {}
    assert Session.deadline_budgets == 2


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
