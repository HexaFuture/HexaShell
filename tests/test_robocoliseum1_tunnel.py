"""benches/robocoliseum1/tunnel.py and challenge.py: the parts of the online client that need no gateway (frames,
the msgpack_numpy payload, the state file, the task of a case, what is hidden from the output)."""

import os
import pickle

import msgpack
import msgpack_numpy
import numpy as np
import pytest

from hexashell.benches.robocoliseum1 import challenge as C
from hexashell.benches.robocoliseum1 import protocol as P
from hexashell.benches.robocoliseum1 import tunnel as T


def test_frames_round_trip():
    for sid, payload in [("abc", b"\x00\x01"), ("warmup", b""), ("é" * 3, b"x" * 1000)]:
        assert T.split_frame(T.join_frame(sid, payload)) == (sid, payload)
    with pytest.raises(ValueError):
        T.split_frame(b"\x00\x00")
    with pytest.raises(ValueError):
        T.split_frame(b"\x00\x00\x00\x09abc")


def test_numpy_request_made_plain():
    req = {"states": {"left_arm": np.arange(7, dtype=np.float32), "img": np.zeros((2, 3, 3), np.uint8),
                      "t": np.float64(1.5)}, "task_name": "pick_block_size", "need": [1, 2]}
    payload = msgpack.packb(req, default=msgpack_numpy.encode, use_bin_type=True)
    plain, changed = T.plain_request(payload)
    assert changed
    back = msgpack.unpackb(plain, raw=False)
    assert back["states"]["left_arm"] == list(range(7))
    assert back["states"]["img"] == np.zeros((2, 3, 3), np.uint8).tolist()
    assert back["states"]["t"] == 1.5 and back["task_name"] == "pick_block_size" and back["need"] == [1, 2]
    same = msgpack.packb({"a": [1.0, 2.0]}, use_bin_type=True)  # already plain: untouched
    assert T.plain_request(same) == (same, False)


def test_object_arrays_refused():
    """msgpack_numpy unpickles an object array; ours must not."""
    evil = {b"nd": True, b"type": "|O", b"kind": b"O", b"shape": (1,), b"data": pickle.dumps([1])}
    with pytest.raises(ValueError):
        T._nd(evil)
    payload = msgpack.packb({"x": evil}, use_bin_type=True)
    assert T.plain_request(payload) == (payload, False)  # left for link.Server, which cannot read it


def test_state_file(tmp_path):
    path = tmp_path / "state.env"
    C.write_env(path, {"JOB_UUID": "u-1", "JOB_TOKEN": "a b'c", "PARALLELISM": 2})
    assert path.stat().st_mode & 0o777 == 0o600
    assert T.read_state(path) == {"JOB_UUID": "u-1", "JOB_TOKEN": "a b'c", "PARALLELISM": "2"}
    C.write_env(path, {"JOB_ID": "7"}, drop=("JOB_TOKEN",))
    assert T.read_state(path) == {"JOB_UUID": "u-1", "PARALLELISM": "2", "JOB_ID": "7"}
    os.chmod(path, 0o640)
    with pytest.raises(SystemExit):
        T.read_state(path)


def test_records_never_name_the_job():
    uuid = "1d6f1c1e-5b0e-4c5e-9f1a-0123456789ab"
    tag = T.job_tag(uuid)
    assert len(tag) == 10 and tag not in uuid
    url = T.dial_url("ws://host/api/challenge/tunnel", uuid, "agent-1")
    assert url == f"ws://host/api/challenge/tunnel?job={uuid}&agent=agent-1"


def test_every_config_maps_to_its_task():
    for cfg in P.configs():
        facts = P.parse_config(cfg)
        _, sub, _ = P.TASKS["instruction" if facts["board"] == "robust" else facts["board"]][facts["task"]]
        got = T.online_facts(facts["board"], sub)
        assert got["known"] and got["task"] == facts["task"], cfg
        if facts["board"] == "robust":
            assert got["kind"] is None and got["config"] == f"g2op_robust_{facts['task']}"
        else:
            assert got["config"] == cfg
    assert T.online_facts("manip", "hold_pot")["task"] == "bimanual_hold_pot"
    unknown = T.online_facts("spatial", "juggle")
    assert not unknown["known"] and unknown["task"] == "juggle"


def test_scrub_hides_credentials():
    resp = {"id": 5, "uuid": "u", "job_token": "t", "Token": "x", "tunnel_endpoint": "ws://h",
            "items": [{"email": "e", "score": 0.5}]}
    assert C.scrub(resp) == {"id": 5, "uuid": "<hidden>", "job_token": "<hidden>", "Token": "<hidden>",
                             "tunnel_endpoint": "<hidden>", "items": [{"email": "<hidden>", "score": 0.5}]}


def _frame(seq: int) -> "O.Frame":
    from hexashell.benches.robocoliseum1 import obs as O
    return O.Frame(seq=seq, t=0.0, task_name="scoop_popcorn", episode_idx=None, prompt="p", done=False,
                   segments=None, joints=np.zeros(14), grippers=np.array([-0.785, -0.785]), waist=None, head=None,
                   chassis=None, end_pose={}, arm_base=np.eye(4))


def test_hold_comes_before_the_simulators_limit():
    """The simulator's 120 s per reply counts the request's download: a request long on the way gets its
    hold row sooner (online, a 7 MB depth frame took 60-90 s, then a 60 s hold passed 120 s)."""
    from hexashell.benches.robocoliseum1 import link as L
    assert L.hold_after(60, 0) == 60  # a quick request: the usual hold
    assert L.hold_after(60, 50) == pytest.approx(55)  # 50 + 55 = 105 = 120 - 15
    assert L.hold_after(60, 88) == pytest.approx(17)
    assert L.hold_after(60, 119) == L.MIN_HOLD_S  # too late to keep the margin: as soon as it can
    assert L.hold_after(60, -3) == 60
    assert L.arrival_s(None) == L.UNKNOWN_ARRIVAL_S and L.arrival_s(7.5) == 7.5
    for spent in (0, 30, 60, 88, 100):
        assert spent + L.hold_after(60, spent) <= L.SIM_REPLY_S - L.REPLY_MARGIN_S + 1e-9


def test_serve_holds_early_for_a_late_request():
    import time
    from hexashell.benches.robocoliseum1 import link as L
    ep = L.EpisodeLink(0, _frame(1), hold_s=60)
    t0 = time.monotonic()
    rows, need, kind = ep.serve(_frame(2), spent_s=L.SIM_REPLY_S - L.REPLY_MARGIN_S - 0.5)
    took = time.monotonic() - t0
    assert kind == "hold" and len(rows) == 1 and need
    assert L.MIN_HOLD_S - 0.1 <= took < L.MIN_HOLD_S + 1.5
    # an action that is ready goes at once, however late the request
    import threading
    row = L.start_command(_frame(3))
    t = threading.Thread(target=ep.act, args=([row], False), daemon=True)
    t.start()
    rows, need, kind = ep.serve(_frame(3), spent_s=200)
    assert kind == "action" and not need
    ep.serve(_frame(4), spent_s=200)  # the request after the action releases act()
    t.join(5)
    assert not t.is_alive()


def test_online_asks_for_depth_only_when_the_perception_needs_it():
    """Online a request with depth takes seconds more to arrive: motions and hold rows ask for none,
    hold_for_depth (the perception's step when the latest request has none) still does."""
    import types
    from hexashell.benches.robocoliseum1 import link as L
    from hexashell.benches.robocoliseum1 import motion as M

    asked = []
    stub = types.SimpleNamespace(rows=0, motion_depth=True, frame=None, on_frame=lambda frame, rows: None,
                                 link=types.SimpleNamespace(act=lambda rows, need: asked.append(need) or "frame"),
                                 _row=lambda: {})
    stub._send = lambda rows, need, force=False: M.G2Driver._send(stub, rows, need, force=force)
    M.G2Driver._send(stub, [{}], True)
    M.G2Driver.hold(stub)
    assert asked == [True, True]  # locally: as before
    stub.motion_depth = False
    M.G2Driver._send(stub, [{}], True)
    M.G2Driver.hold(stub)
    M.G2Driver.hold_for_depth(stub)
    M.G2Driver._send(stub, [{}], False)
    assert asked[2:] == [False, False, True, False]
    assert stub.rows == 6

    ep = L.EpisodeLink(0, _frame(1), hold_s=60)
    assert ep.hold_depth
    ep.hold_depth = False
    rows, need, kind = ep.serve(_frame(2), spent_s=L.SIM_REPLY_S - L.REPLY_MARGIN_S - 0.5)
    assert kind == "hold" and not need


def test_online_limits():
    from dataclasses import fields
    from hexashell.runtime.episode import Settings
    names = {f.name for f in fields(Settings)}
    assert set(T.ONLINE_LIMITS) <= names
    assert T.ONLINE_LIMITS["budget_s"] == 240 * 60
    assert T.ONLINE_LIMITS["cell_total_s"] >= 3600
