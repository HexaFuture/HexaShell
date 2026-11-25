"""robot.both: two arms in the same actions (motion/driver.py Pair, robot/host.py)."""

from __future__ import annotations

import inspect
import types

import numpy as np
import pytest

from hexashell.errors import Refused
from hexashell.kernel.host import CallFailed
from hexashell.motion.driver import Driver, PairRefused, stretch
from hexashell.robot import both_proxy, proxy
from hexashell.robot.host import RobotHost
from hexashell.runtime import card as cards
from hexashell.runtime.episode import Settings
from hexashell.world import TABLE_Z
from test_motion import _Arms, _Floor

DOWN = dict(pitch=90, yaw=0, roll=0)
L, R = (-0.25, -0.22, TABLE_Z + 0.14), (0.25, -0.22, TABLE_Z + 0.14)


def _ready():
    arms = _Arms()
    driver = arms.driver()
    assert driver.both(dict(xyz=L, **DOWN), dict(xyz=R, **DOWN))
    return arms, driver


def test_it_costs_the_longer_motion_not_the_sum():
    arms, driver = _ready()
    one = driver.actions
    driver.goto("left", by=(0, 0.12, 0))
    alone = driver.actions - one
    driver.goto("left", by=(0, -0.12, 0))
    two = driver.actions
    done = driver.both(dict(by=(0, 0.12, 0)), dict(by=(0, 0.12, 0)))
    assert driver.actions - two <= alone + 1
    assert done["left"].reached and done["right"].reached


def test_each_action_carries_both_arms_moving():
    arms, driver = _ready()
    before = {a: driver.command[a].copy() for a in ("left", "right")}
    driver.both(dict(by=(0, 0.1, 0)), dict(by=(0, 0.1, 0)))
    assert all(np.max(np.abs(driver.command[a] - before[a])) > 0.01 for a in before)


def test_together_ends_in_the_same_action():
    arms, driver = _ready()
    driver.both(dict(by=(0, 0.16, 0)), dict(by=(0, 0.04, 0)), together=True)
    ends = {a: driver.grasp_pose(a)[:3, 3] for a in ("left", "right")}
    assert abs(ends["left"][1] - (L[1] + 0.16)) < 0.01 and abs(ends["right"][1] - (R[1] + 0.04)) < 0.01


def test_a_refusal_moves_neither_arm():
    arms, driver = _ready()
    start = driver.actions
    with pytest.raises(PairRefused):
        driver.both(dict(by=(0, 0.05, 0)), dict(xyz=(5.0, 5.0, 1.0)))
    assert driver.actions == start


def test_arms_that_would_meet_are_refused():
    arms, driver = _ready()
    start = driver.actions
    with pytest.raises(PairRefused, match="one at a time"):
        driver.both(dict(xyz=(0.0, -0.22, TABLE_Z + 0.14)), dict(xyz=(0.0, -0.22, TABLE_Z + 0.14)))
    assert driver.actions == start


def test_one_arm_can_close_while_the_other_moves():
    arms, driver = _ready()
    start = driver.actions
    done = driver.both(dict(grip="close"), dict(by=(0, 0.1, 0)))
    assert driver.grip_goal["left"] <= 0.05 and done["right"].reached
    assert driver.actions - start <= done["right"].steps + 1


def test_only_one_arm_given_is_a_goto():
    arms, driver = _ready()
    done = driver.both(None, dict(by=(0, 0.05, 0)))
    assert list(done) == ["right"] and done["right"].reached


def test_stretch_keeps_the_path_and_never_shortens():
    start = np.zeros(6)
    actions = [np.full(6, 0.1), np.full(6, 0.2)]
    long = stretch(actions, start, 8)
    assert len(long) == 8 and np.allclose(long[-1], actions[-1])
    assert np.all(np.diff([a[0] for a in long]) > 0)
    assert stretch(actions, start, 2) is actions


def test_the_host_turns_a_refusal_into_notexecuted():
    arms, driver = _ready()
    host = types.SimpleNamespace(driver=driver, _solids=lambda: [], snap=driver.now, two_hands=True)
    with pytest.raises(CallFailed) as err:
        RobotHost._both(host, left=dict(xyz=(5.0, 5.0, 1.0)), right=dict(by=(0, 0.05, 0)))
    assert "nothing moved" in str(err.value)


def test_one_arm_stalling_stops_the_other():
    arms = _Floor(TABLE_Z + 0.10)
    driver = arms.driver()
    driver.both(dict(xyz=(-0.25, -0.15, TABLE_Z + 0.13), **DOWN), dict(xyz=R, **DOWN))
    done = driver.both(dict(by=(0, 0, -0.09)), dict(by=(0, 0.14, 0), gentle=True))
    assert done["left"].blocked and done["left"].skipped > 0
    assert done["right"].skipped > 0 and "other arm" in " ".join(done["right"].notes)


class _Loses(_Arms):
    """_Arms whose right gripper holds something until ``at`` actions after it is told to let it slip."""

    at = None

    def act(self, action):
        if self.at is not None:
            self.at -= 1
            if self.at <= 0:
                self.grip_floor = 0.0
        return super().act(action)


def test_a_hand_that_stays_put_losing_its_hold_stops_the_other():
    arms = _Loses()
    arms.grip_floor = 0.4
    driver = arms.driver()
    driver.keeper_stop = True  # as with the setting two_hands
    driver.both(dict(xyz=L, **DOWN), dict(xyz=R, **DOWN))
    assert driver.gripper("right", "close").holding
    arms.at = 3
    move = driver.goto("left", by=(0, 0.14, 0), contact=True)
    assert move.halt == "Dropped" and move.skipped > 0 and "right arm held" in " ".join(move.notes)


def test_off_a_hand_that_loses_its_hold_stops_nothing():
    """The driver's default, with the setting two_hands off."""
    arms = _Loses()
    arms.grip_floor = 0.4
    driver = arms.driver()
    assert driver.keeper_stop is False
    driver.both(dict(xyz=L, **DOWN), dict(xyz=R, **DOWN))
    assert driver.gripper("right", "close").holding
    arms.at = 3
    move = driver.goto("left", by=(0, 0.14, 0), contact=True)
    assert move.skipped == 0 and not move.halt


def test_both_is_off_by_default():
    assert Settings().two_hands is False
    assert inspect.signature(RobotHost.__init__).parameters["two_hands"].default is False
    assert inspect.signature(Driver.__init__).parameters["keeper_stop"].default is False
    assert not hasattr(proxy.Robot, "both") and callable(both_proxy.Robot.both)
    assert issubclass(both_proxy.Robot, proxy.Robot)
    assert "robot.both" not in cards.system_prompt("robodojo")


def test_off_the_host_has_no_both():
    host = RobotHost.__new__(RobotHost)
    host.two_hands = False
    host.over = lambda: False
    with pytest.raises(CallFailed, match="robot has no 'both'"):
        host.dispatch("both", (), {})
    with pytest.raises(Refused):
        host._both(left=dict(by=(0, 0, 0.05)))
