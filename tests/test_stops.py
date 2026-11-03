"""Stopping a goto early (motion/driver.py) and stopping the cell after it (robot/host.py): a move that
stalls against something does not send the rest of its plan, and one that stalled or ends BLOCKED by
more than RAISE_LAG_M raises Blocked; what the arm carries slipping out (except in the last 2 cm of a
descent) stops the motion and raises Dropped.  Settling ends once the arm stands still.  contact=True
turns all of it off.  (tests/test_r3_motion.py: the stops and halts R3 added.)"""

from __future__ import annotations

import types

import numpy as np

from hexashell.kernel.host import CallFailed
from hexashell.motion import ik
from hexashell.motion.driver import RAISE_LAG_M, SETTLE_ACTIONS, STOP_LAG_M
from hexashell.robot.host import RobotHost
from hexashell.world import TABLE_Z
from test_motion import _Arms, _Floor

DOWN = dict(pitch=90, yaw=0, roll=0)


def _host(driver):
    """Just enough of a RobotHost for its goto."""
    return types.SimpleNamespace(driver=driver, _solids=lambda: [])


def _goto(driver, *args, **kw):
    """RobotHost's goto: (the Move, None) or (None, the CallFailed)."""
    try:
        return RobotHost._goto(_host(driver), *args, **kw), None
    except CallFailed as exc:
        return None, exc


def _above_a_floor(floor=0.10, start=0.22):
    arms = _Floor(TABLE_Z + floor)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + start), **DOWN).ok
    return arms, driver


def test_a_stall_stops_the_plan_and_raises():
    """R1: BLOCKED descents kept pushing for the rest of their plan and 8 settle actions, and the cells
    went on to close, open or press from there."""
    arms, driver = _above_a_floor()
    target = (-0.25, -0.15, TABLE_Z + 0.03)  # 7 cm below what stops it
    planned = driver.goto("left", target, dry_run=True).steps - 2
    before = arms.seq
    move = driver.goto("left", target)
    assert move.blocked and not move.ok and move.skipped > 0 and move.halt == "Blocked", str(move)
    assert arms.seq - before == move.steps == planned - move.skipped, (arms.seq - before, move.steps, planned)
    assert f"stopped after {planned - move.skipped} of {planned} actions" in str(move), str(move)
    assert move.lag_mm > 1000 * STOP_LAG_M and "something is in the way" in str(move), str(move)
    # it holds where it stopped: later actions do not push on
    z = driver.grasp_pose("left")[2, 3]
    driver.wait(5)
    assert abs(driver.grasp_pose("left")[2, 3] - z) < 0.002
    # through the host the cell stops here
    arms, driver = _above_a_floor()
    moved, error = _goto(driver, "left", target)
    assert moved is None and error.kind == "Blocked", error
    assert error.text.startswith("Move left: BLOCKED") and error.text.endswith(
        "\nthe move ran; the rest of the cell was skipped so you can look"), error.text


def test_a_fast_move_that_lags_but_keeps_moving_goes_on(monkeypatch):
    """Fast moves lag their command by more than STOP_LAG_M (RoboDojo's joints keep up in speed, not in
    place); only a grasp point that stopped moving stops the plan."""
    from hexashell.motion import driver as D

    class Slow(_Arms):
        track = 0.35  # half as fast as _Arms: a v2 cruise lags its command by 7 cm

    def far(speed):
        arms = Slow()
        driver = arms.driver()
        driver.speed = speed
        assert driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.10), **DOWN).moved
        return driver.goto("left", (0.0, -0.30, TABLE_Z + 0.20))  # 39 cm

    for speed in (ik.SPEEDS["v1"], ik.SPEEDS["v2"]):
        move = far(speed)
        assert move.moved and move.skipped == 0 and move.halt == "" and "stopped after" not in str(move), str(move)
        assert move.err_mm < 10, str(move)
    monkeypatch.setattr(D, "STOP_MOVED", float("inf"))  # without asking whether it still moves
    monkeypatch.setattr(D, "STOP_MOVED_M", float("inf"))
    assert far(ik.SPEEDS["v2"]).skipped > 0


def test_contact_moves_push_on_and_never_raise():
    """A press, push or insertion is meant to meet resistance: no early stop, the full settle, no error."""
    arms, driver = _above_a_floor()
    target = (-0.25, -0.15, TABLE_Z + 0.03)
    planned = driver.goto("left", target, dry_run=True).steps - 2
    pressed = driver.goto("left", target, contact=True)
    assert pressed.blocked and pressed.skipped == 0 and pressed.halt == "", str(pressed)
    assert pressed.steps == planned + SETTLE_ACTIONS and pressed.lag_mm > 1000 * RAISE_LAG_M, str(pressed)
    assert "stopped after" not in str(pressed), str(pressed)
    arms, driver = _above_a_floor()
    moved, error = _goto(driver, "left", target, contact=True)
    assert error is None and moved.blocked, error


def test_blocked_at_the_end_raises_only_past_the_threshold():
    """Button presses in R1 ended 14-21 mm BLOCKED and worked; deeper blocks stop the cell.  (A press
    closes the gripper first: with it open and empty a descent stops the cell from 10 mm, see
    test_r3_motion.py.)"""
    for below, raises in ((0.012, False), (0.035, True)):
        arms, driver = _above_a_floor(floor=0.10, start=0.14)
        assert driver.gripper("left", "close").holding is False  # shut on nothing
        moved, error = _goto(driver, "left", (-0.25, -0.15, TABLE_Z + 0.10 - below))
        if raises:
            assert error is not None and error.kind == "Blocked", (below, moved)
        else:
            assert error is None and moved.blocked and moved.halt == "", (below, error)
            assert 5 < moved.lag_mm < 1000 * RAISE_LAG_M and moved.skipped == 0, str(moved)


def test_settling_ends_once_the_arm_stands_still():
    """R1: 574 gotos spent 1,739 settle actions standing still against something."""
    arms, driver = _above_a_floor(floor=0.10, start=0.14)
    target = (-0.25, -0.15, TABLE_Z + 0.09)  # 1 cm below what stops it: no early stop
    planned = driver.goto("left", target, dry_run=True).steps - 2
    stood = driver.goto("left", target)
    assert stood.blocked and stood.skipped == 0 and stood.steps < planned + SETTLE_ACTIONS, (stood.steps, planned)
    arms, driver = _above_a_floor(floor=0.10, start=0.14)
    pressed = driver.goto("left", target, contact=True)
    assert pressed.steps == planned + SETTLE_ACTIONS and abs(pressed.lag_mm - stood.lag_mm) < 2, (str(pressed), str(stood))
    free = _Arms().driver()  # a free move still settles until it arrives
    assert free.goto("left", (-0.25, -0.15, TABLE_Z + 0.14), **DOWN).ok


def _holding(arms):
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.05), **DOWN).ok
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    return driver


def test_a_slip_while_carrying_stops_the_motion_and_raises():
    """R1: 196 slips while lifting or carrying, and the cells carried the empty gripper on to place (531
    actions, then 1,144 more in their cells)."""
    arms = _Arms()
    driver = _holding(arms)
    carry = dict(xyz=(-0.05, -0.25, TABLE_Z + 0.20))
    planned = driver.goto("left", **carry, dry_run=True).steps - 2
    arms.later = {arms.seq + 3: 0.0}  # it slips out 3 actions in
    before = arms.seq
    moved, error = _goto(driver, "left", **carry)
    assert moved is None and error.kind == "Dropped", error
    sent = arms.seq - before
    assert sent < planned and f"stopped after {sent} of {planned} actions: the gripper closed fully" in error.text
    assert error.text.endswith("\nthe rest of the motion and the cell were skipped - look where it went"), error.text
    assert not driver.carrying("left") and "slipped out" not in str(driver.goto("left", by=(0, 0, 0.02)))


def test_a_slip_in_the_last_2_cm_of_a_descent_goes_on():
    """Touching down can look like a slip: in its last 2 cm a descent is not stopped, and only says so.
    (Here the fingers read shut on the 4th of 5 actions, commanded 1.6 cm above the end.)"""
    arms = _Arms()
    driver = _holding(arms)
    assert driver.goto("left", by=(0.0, 0.0, 0.10)).ok
    planned = driver.goto("left", by=(0.0, 0.0, -0.08), dry_run=True).steps - 2
    arms.later = {arms.seq + 3: 0.0}
    moved, error = _goto(driver, "left", by=(0.0, 0.0, -0.08))
    assert error is None and moved.dropped and moved.skipped == 0 and moved.steps >= planned, (error, moved)
    assert "what it held has slipped out" in str(moved), str(moved)


def test_a_slip_higher_up_in_a_descent_stops_it():
    """R1: 16 slips in descents above their last 2 cm went on to the end of the move (69 actions) and
    the cells on from there (209 steps); none came back."""
    arms = _Arms()
    driver = _holding(arms)
    assert driver.goto("left", by=(0.0, 0.0, 0.10)).ok
    planned = driver.goto("left", by=(0.0, 0.0, -0.12), dry_run=True).steps - 2
    before = arms.seq
    arms.later = {before + 1: 0.0}  # it slips out at once: the reading is shut 2 actions in
    moved, error = _goto(driver, "left", by=(0.0, 0.0, -0.12))
    assert moved is None and error.kind == "Dropped", (error, moved)
    sent = arms.seq - before
    assert f"stopped after 2 of {planned} actions: the gripper closed fully" in error.text, error.text
    assert sent < planned + 2 and error.text.endswith("the rest of the motion and the cell were skipped - look where it went")


def test_no_slip_without_a_hold_nor_in_a_contact_move():
    """An empty close pried open by a press reads like a hold; a press may squeeze a held thing flat."""
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.05), **DOWN).ok
    assert driver.gripper("left", "close").holding is False  # shut on nothing
    arms.grip_floor = 0.3  # pried apart by a press
    assert driver.goto("left", by=(0.0, 0.0, -0.02)).moved and driver.holding("left") is True
    arms.later = {arms.seq + 3: 0.0}
    moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.10))
    assert error is None and moved.ok and moved.skipped == 0, (error, moved)
    arms = _Arms()
    driver = _holding(arms)
    arms.later = {arms.seq + 3: 0.0}
    moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.10), contact=True)
    assert error is None and moved.skipped == 0 and moved.dropped, (error, moved)


def test_an_arm_that_falls_behind_for_a_moment_is_not_stopped(monkeypatch):
    """R1 (a conveyor pick): the arm followed its command slowly for a few actions, fell 7 cm behind and
    caught up; the move ended 1 mm off.  Only a grasp point that all but stops stops the plan."""
    from hexashell.motion import driver as D

    class Sticky(_Arms):
        start = 10 ** 9

        def act(self, action):
            self.track = 0.12 if 2 <= self.seq - self.start < 5 else 0.7
            return super().act(action)

    def far():
        arms = Sticky()
        driver = arms.driver()
        driver.speed = ik.SPEEDS["v2"]
        assert driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.10), **DOWN).moved
        arms.start = arms.seq
        return driver.goto("left", (0.0, -0.30, TABLE_Z + 0.20))

    move = far()
    assert move.moved and move.skipped == 0 and move.halt == "" and move.err_mm < 10, str(move)
    monkeypatch.setattr(D, "STOP_MOVED_M", float("inf"))  # the rule without the distance limit stops it
    assert far().skipped > 0


def test_a_slip_stop_that_recovers_still_stops_the_cell():
    """The fingers shut as if it slipped, then opened a little again while the gripper settled (R1: 2 of
    196 slips came back): the motion was cut short, so the cell stops, saying it may still hold it."""
    arms = _Arms()
    driver = _holding(arms)
    carry = dict(xyz=(-0.05, -0.25, TABLE_Z + 0.20), grip="close")  # the gripper settles after the stop
    planned = driver.goto("left", **carry, dry_run=True).steps - 2
    before = arms.seq
    arms.later = {before + 3: 0.0, before + 5: 0.3}  # shut 4 actions in, open a little again after it
    moved, error = _goto(driver, "left", **carry)
    assert moved is None and error.kind == "Dropped", error
    assert f"stopped after 4 of {planned} actions: the gripper closed almost fully" in error.text, error.text
    assert "it may still hold it" in error.text and driver.carrying("left"), error.text


def test_a_slip_on_the_last_action_is_not_a_stop():
    """Nothing is left to stop on the last action: the move ends where it was going and says it slipped."""
    arms = _Arms()
    driver = _holding(arms)
    carry = dict(xyz=(-0.05, -0.25, TABLE_Z + 0.20))
    planned = driver.goto("left", **carry, dry_run=True).steps - 2
    arms.later = {arms.seq + planned - 1: 0.0}  # closed fully on the last action
    moved, error = _goto(driver, "left", **carry)
    assert moved is None and error.kind == "Dropped", error
    assert "stopped after" not in error.text and "slipped out" in error.text, error.text
    assert np.linalg.norm(driver.grasp_pose("left")[:3, 3] - np.array(carry["xyz"])) < 0.01
