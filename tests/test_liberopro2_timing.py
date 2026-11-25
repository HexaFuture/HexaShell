"""Checks of LIBERO-Pro 2.0's lag of the readings behind the commands (:mod:`hexashell.benches.liberopro2.timing`)
on a toy arm: a linear model in place of the replica, run by a world that delays the commands it applies or the
readings it gives as a schedule says (from the start like a buffer filling, for a while, frozen), pushes the arm
or adds noise.  Needs neither the simulator nor MuJoCo.
Run: ``python tests/test_liberopro2_timing.py`` (or with pytest) from the checkout, in the policy environment.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from hexashell.benches.liberopro2 import timing as TM  # noqa: E402

DT = 0.05
#: the toy arm: seven joints, the grip site's Jacobian (linear rows, then angular) and the joint speeds a command
#: asks for
JAC = np.hstack([np.diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0]), np.full((6, 1), 0.1)])
GAIN = np.vstack([0.5 * np.eye(6), np.zeros((1, 6))])
PERIODS = np.array([23.0, 29.0, 31.0, 37.0, 41.0, 43.0])


def toy_run(state, commands):
    """The toy arm's states after each command: the joint speeds go 40% of the way to what it asks, a step."""
    q, qd = (np.array(v, float) for v in state)
    out = []
    for c in commands:
        qd = 0.6 * qd + 0.4 * (GAIN @ np.asarray(c, float)[:6])
        q = q + DT * qd
        out.append((q.copy(), qd.copy()))
    return out


def command(k: int) -> np.ndarray:
    return 0.6 * np.sin(2.0 * np.pi * k / PERIODS + np.arange(6))


class World:
    """The toy arm as it is (``states``: x_0 .. x_k) and as it reads: ``act(k, sent)`` is the command the arm takes
    at step k of those sent (c_1 .. c_k), ``read(k, states)`` the reading; ``push(k)``: joint speeds added."""

    def __init__(self, act=None, read=None, push=None):
        self.states = [(np.full(7, 0.1), np.zeros(7))]
        self.sent: list = []
        self.act = act or (lambda k, sent: sent[-1])
        self.read = read or (lambda k, states: states[-1])
        self.push = push

    def step(self, c):
        self.sent.append(np.asarray(c, float))
        k = len(self.sent)
        q, qd = toy_run(self.states[-1], [self.act(k, self.sent)])[0]
        if self.push is not None:
            kick = self.push(k)
            q, qd = q + DT * kick, qd + kick
        self.states.append((q, qd))
        return self.read(k, self.states)


def drive(world: World, steps: int, noise: float = 0.0, seed: int = 0, pending=lambda k: 0):
    """A Timing following the world for ``steps`` steps; with each step's gap (m) of its prediction from where the
    commands sent leave the arm (index k: step k; ``pending(k)``: how many of them it has not applied yet)."""
    rng = np.random.default_rng(seed)
    t = TM.Timing(toy_run, *world.read(0, world.states))
    off = [0.0]
    for k in range(1, steps + 1):
        q, qd = world.step(command(k))
        if noise:
            q = q + rng.normal(0.0, noise, 7)
        t.update(command(k), q, qd, JAC)
        n = pending(k)
        future = toy_run(world.states[-1], world.sent[-n:])[-1] if n else world.states[-1]
        off.append(TM.gap(t.predicted(), future, JAC))
    return t, off


def test_no_lag():
    t, off = drive(World(), 120)
    assert t.changes == [] and t.lag == 0 and max(t.checks) < 1e-9 and max(off) < 1e-12
    assert t.ahead == []


def test_buffers_filling_from_the_start():
    """The commands and the readings each go through a 7-step buffer that passes them on until it is full (the
    first 7 commands are then applied twice): no lag, the readings jump back to the start at step 7 (lag 7),
    then the commands lag too (14)."""
    def act(k, sent):
        return sent[-8] if k > 7 else sent[-1]

    def read(k, states):
        return states[-8] if len(states) > 7 else states[-1]

    t, off = drive(World(act, read), 80, pending=lambda k: 7 if k > 7 else 0)
    assert t.changes[0] == (7, 0, 7, "repeat"), t.changes
    assert len(t.changes) == 2 and t.changes[1][1:] == (7, 14, "search") and 15 <= t.changes[1][0] <= 20, t.changes
    assert t.lag == 14
    assert max(off[:7]) < 1e-12 and max(off[t.changes[1][0]:]) < 1e-9


def test_readings_delayed_for_a_while():
    """From step 30 to 49 the readings show the arm 5 steps back: they jump back, then ahead again."""
    def read(k, states):
        return states[-6] if 30 <= k < 50 else states[-1]

    t, off = drive(World(read=read), 80)
    assert t.changes == [(30, 0, 5, "repeat"), (50, 5, 0, "ahead")], t.changes
    assert max(off) < 1e-9  # the arm predicted as it is, all along


def test_commands_delayed_for_a_while():
    """From step 30 to 49 the arm takes the command sent 4 steps before (the readings are on time)."""
    def act(k, sent):
        return sent[-5] if 30 <= k < 50 else sent[-1]

    t, off = drive(World(act=act), 90, pending=lambda k: 4 if 30 <= k < 50 else 0)
    assert [c[1:] for c in t.changes] == [(0, 4, "search"), (4, 0, "search")], t.changes
    (start, _, _, _), (end, _, _, _) = t.changes
    assert 30 < start <= 36 and 50 < end <= 56, t.changes
    # (the commands of the last steps before 50 are never applied: no prediction holds for them)
    assert max(off[start:46]) < 1e-9 and max(off[end:]) < 1e-9


def test_readings_frozen():
    """From step 30 to 39 every reading is step 29's: the lag grows a step a step, then drops back at once."""
    def read(k, states):
        return states[29] if 30 <= k < 40 else states[-1]

    t, off = drive(World(read=read), 70)
    assert [c[3] for c in t.changes[:-1]] == ["repeat"] * 10 and t.changes[-1] == (40, 10, 0, "ahead"), t.changes
    assert t.lags[39] == 10 and t.lag == 0
    assert max(off) < 1e-9


def test_pushed_or_noisy_arm_keeps_the_lag():
    """A push (from step 30 to 44) or noise on the joint readings is no lag: none is taken."""
    rng = np.random.default_rng(1)
    kicks = {k: rng.normal(0.0, 0.03, 7) for k in range(30, 45)}
    t, _ = drive(World(push=lambda k: kicks.get(k, np.zeros(7))), 90)
    assert t.changes == [] and t.lag == 0 and max(t.checks) > TM.CHECK_M
    t, _ = drive(World(), 120, noise=2e-4)
    assert t.changes == [] and t.lag == 0 and max(t.checks) > TM.CHECK_M


def test_lagging_arm_pushed_keeps_the_lag():
    """With the readings 5 steps late all along (they jump back at step 5), a push is no change of the lag."""
    rng = np.random.default_rng(2)
    kicks = {k: rng.normal(0.0, 0.03, 7) for k in range(40, 55)}

    def read(k, states):
        return states[-6] if k >= 5 else states[-1]

    t, _ = drive(World(read=read, push=lambda k: kicks.get(k, np.zeros(7))), 100)
    assert t.changes == [(5, 0, 5, "repeat")], t.changes


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
