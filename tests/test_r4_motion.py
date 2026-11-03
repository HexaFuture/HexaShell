"""R4's changes to motion/driver.py and robot/host.py: a BLOCKED note says which way the arm was held
(held_way), and what follows from it - an open, empty descent held up 11-25 mm no longer stops the cell:
the next close on that arm is refused once (Grip "NOT closed"), and a close after an open descent held
up 8 mm or more says its hold is suspect; what the arm holds rests on something below it, or is pressed
against something (neither while it pushes the other arm); a closed, empty lift held down is hooked
under an edge.  The slip-pitch note comes once per grasp; a thin hold that reads closed fully may still
be held (the shift note that keeps quiet back at the close's width is in test_motion); after a
half-turn, a closed gripper asked for the old yaw keeps the fingers' heading; the other arm is offered
as a hand-over only when nothing else is reachable, once per turn, with steps to spare, and not while
it holds something; a BLOCKED note does not name what the hand holds, measured again."""

from __future__ import annotations

import math
import types

import numpy as np

from hexashell.geometry import angle_between
from hexashell.kernel.host import CallFailed
from hexashell.motion.driver import Solid, held_way
from hexashell.robot.host import RobotHost
from hexashell.state import STATUS_KEY
from hexashell.world import TABLE_Z
from test_motion import _Arms, _Floor
from test_stops import _goto

DOWN = dict(pitch=90, yaw=0, roll=0)
WHERE = (-0.20, -0.20, TABLE_Z + 0.15)
REST = ("the open fingertips rest on something: a close here pinches an edge and slips - grasp lower, by a "
        "narrower part, or from the side")
ONCE = " (the next close here is refused once; closing again closes)"
SUSPECT = "mm short, held up on something: holds caught after such stops were lost about half the time"
SKIPPED = "\nthe rest of the cell was skipped so you can look"


def _gripper(driver, arm, opening, things=()):
    """RobotHost's gripper: (the Grip, None) or (None, the CallFailed)."""
    host = types.SimpleNamespace(driver=driver, _solids=lambda: list(things))
    try:
        return RobotHost._gripper(host, arm, opening), None
    except CallFailed as exc:
        return None, exc


def _said(moved, error) -> str:
    return str(moved) if error is None else error.text


class _Wall(_Arms):
    """_Arms whose left grasp point cannot pass ``at`` along ``axis`` (0: x, 2: z) going the way of
    ``sign`` (+1: on or up, -1: back or down): something stands there."""

    def __init__(self, axis, at, sign):
        super().__init__()
        self.axis, self.at, self.sign = axis, at, sign

    def past(self, q) -> bool:
        return self.sign * (self.chains["left"].grasp_pose(q)[self.axis, 3] - self.at) > 1e-9

    def act(self, action):
        from hexashell.state import parse_obs

        before = self.q["left"].copy()
        super().act(action)
        after = self.q["left"]
        if self.past(after) and not self.past(before):
            lo, hi = 0.0, 1.0  # how far along this action it gets
            for _ in range(30):
                mid = (lo + hi) / 2
                if self.past(before + mid * (after - before)):
                    hi = mid
                else:
                    lo = mid
            self.q["left"] = before + lo * (after - before)
        return parse_obs(self.obs(), self.seq)


class _Counted(_Floor):
    """_Floor whose observations carry the episode step (one per action)."""

    def obs(self):
        obs = super().obs()
        obs["additional_info"] = {STATUS_KEY: {"step": self.seq, "step_limit": 1000}}
        return obs


def _rest(below, **kw):
    """An open, empty gripper going down onto something 10 cm above the table, ``below`` past its top:
    (the arms, the driver, and RobotHost's goto: the Move or None, the CallFailed or None)."""
    arms = _Floor(TABLE_Z + 0.10)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.16), **DOWN).ok
    moved, error = _goto(driver, "left", (-0.25, -0.15, TABLE_Z + 0.10 - below), **kw)
    return arms, driver, moved, error


def test_held_way_names_the_way_the_arm_was_held():
    """R3's fingertip note came 134 times, 13 of them after moves held back mostly sideways; stops on the
    round tops of eggs leaned up to 50 degrees from vertical."""
    down = (0.0, 0.0, -0.05)
    assert held_way((0.0, 0.0, -0.01), down) == "up"
    lean = math.radians(50.0)
    assert held_way((0.01 * math.sin(lean), 0.0, -0.01 * math.cos(lean)), down) == "up"
    lean = math.radians(65.0)
    assert held_way((0.01 * math.sin(lean), 0.0, -0.01 * math.cos(lean)), down) == "sideways"
    assert held_way((0.0, 0.0, 0.01), (0.0, 0.0, 0.05)) == "down"
    assert held_way((0.01, 0.0, -0.0025), (0.10, 0.0, -0.025)) == "back"
    assert held_way((0.0, 0.01, -0.002), (0.10, 0.0, -0.02)) == "sideways"
    assert held_way((0.0, 0.0, 0.0), down) == ""


def test_an_open_descent_resting_on_something_refuses_the_next_close_once():
    """R3 stopped the cell after open, empty descents held up 11-25 mm: 15 of 43 such stops cut short a
    cell whose next call was not a close (a move, a push, a look); closes right after the note kept what
    they caught 3 times in 13."""
    arms, driver, moved, error = _rest(0.012)
    assert error is None and moved.halt == "" and moved.blocked, error
    assert "held up from below" in str(moved) and REST + ONCE in str(moved), str(moved)
    before = arms.seq
    grip, error = _gripper(driver, "left", "close")
    assert grip is None and error.kind == "Blocked", grip
    assert error.text.startswith("Grip left: NOT closed (still open 1.00)  0 steps"), error.text
    assert ("note: not closed: the last move ended 12 mm short with the open fingertips resting on something, and "
            "closes there kept what they caught 3 times in 13 - grasp lower, by a narrower part, or from the side; to "
            "close here anyway, close again") in error.text and error.text.endswith(SKIPPED), error.text
    assert arms.seq == before and driver.now.arms["left"].gripper == 1.0 and not driver.squeezing("left")
    # closing again closes, and says what such holds are worth
    arms.grip_floor = 0.4
    grip, error = _gripper(driver, "left", "close")
    assert error is None and grip.holding is True and not grip.refused, error
    assert f"it closed where the open descent stopped 12 {SUSPECT}" in str(grip), str(grip)
    assert "check how it sits (its width, a picture) before carrying it far" in str(grip), str(grip)


def test_a_move_after_the_rest_clears_the_refusal():
    arms, driver, moved, error = _rest(0.012)
    assert error is None
    assert _goto(driver, "left", by=(0.0, 0.0, 0.03))[1] is None
    arms.grip_floor = 0.4
    grip, error = _gripper(driver, "left", "close")
    assert error is None and grip.holding is True and SUSPECT not in str(grip), (error, str(grip))
    # and so does home()
    arms, driver, moved, error = _rest(0.012)
    driver.home("left")
    grip, error = _gripper(driver, "left", "close")
    assert error is None and not grip.refused, error


def test_past_raise_lag_the_cell_stops_and_the_close_after_it_is_suspect():
    arms, driver, moved, error = _rest(0.035)
    assert moved is None and error.kind == "Blocked" and REST in error.text and ONCE not in error.text, (moved, error)
    arms.grip_floor = 0.4
    grip, error = _gripper(driver, "left", "close")  # not refused: the cell stopped already
    assert error is None and grip.holding is True and "it closed where the open descent stopped" in str(grip), error


def test_a_shallower_rest_only_makes_the_close_suspect():
    """fill_egg_holder (R2, R3): picks after open descents held up 8 mm or more were lost 16 times in 32."""
    arms, driver, moved, error = _rest(0.009)
    assert error is None and moved.blocked and moved.halt == "" and REST not in str(moved), error
    arms.grip_floor = 0.4
    grip, error = _gripper(driver, "left", "close")
    assert error is None and f"it closed where the open descent stopped 9 {SUSPECT}" in str(grip), str(grip)
    # not when the close catches nothing
    arms, driver, moved, error = _rest(0.009)
    grip, error = _gripper(driver, "left", "close")
    assert error is None and grip.holding is False and SUSPECT not in str(grip), str(grip)
    # 6 mm: BLOCKED, nothing more
    arms, driver, moved, error = _rest(0.006)
    assert error is None and moved.blocked and REST not in str(moved), error
    arms.grip_floor = 0.4
    grip, error = _gripper(driver, "left", "close")
    assert error is None and grip.holding is True and SUSPECT not in str(grip), str(grip)
    # contact=True: a press meant to meet resistance
    arms, driver, moved, error = _rest(0.012, contact=True)
    assert error is None and moved.blocked and moved.halt == "" and REST not in str(moved), error
    grip, error = _gripper(driver, "left", "close")
    assert error is None and not grip.refused, error


def test_a_descent_held_back_by_a_wall_does_not_rest_on_it():
    """R3: 13 of the 134 fingertip notes came after moves held back mostly sideways (insert_key's camera
    moves at pitch 30 next to the other arm), where its advice did not apply."""
    arms = _Wall(0, -0.20, +1)  # nothing past x -0.20
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).ok
    text = _said(*_goto(driver, "left", (-0.15, -0.15, TABLE_Z + 0.125)))  # 10 cm on, 2.5 cm down
    assert "held back along its path" in text and REST not in text, text
    grip, error = _gripper(driver, "left", "close")
    assert error is None and not grip.refused and SUSPECT not in str(grip), (error, grip)


def test_what_it_holds_rests_on_something_below_or_is_pressed_against_something():
    """R3: the 2 'opening here usually drops it' notes at a headset's cradle came where it already hung,
    and the opens that followed counted."""
    arms = _Floor(TABLE_Z + 0.06)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.14), **DOWN).ok
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    text = _said(*_goto(driver, "left", by=(0.0, 0.0, -0.10)))
    assert ("what it holds rests on something below it: opening here leaves it there - look whether that is where it "
            "should go") in text and "pressed against" not in text, text
    arms = _Wall(0, -0.20, +1)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).ok
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    text = _said(*_goto(driver, "left", (-0.12, -0.15, TABLE_Z + 0.15)))
    assert ("what it holds is pressed against something: opening here usually drops it - back out along the line and "
            "look first") in text and "rests on something below" not in text, text


class _Shoved(_Floor):
    """_Floor whose right arm is pushed 0.05 rad (its first joint) off its command while the left one is held up."""

    def act(self, action):
        from hexashell.state import parse_obs

        obs = super().act(action)
        if abs(self.chains["left"].grasp_pose(self.q["left"])[2, 3] - self.floor) > 1e-6:
            return obs
        self.q["right"] = np.asarray(action["right_arm_joint_state"]) + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
        return parse_obs(self.obs(), self.seq)


def test_what_it_holds_is_not_said_to_rest_or_be_pressed_when_the_hands_touch():
    """fill_pen_holder R3: in 22 cells "what it holds is pressed against something: opening here usually
    drops it" followed "it is pushing the right arm": the left hand pressed on the right one, which held
    the cup; obeyed twice (a 31-step swing)."""
    arms = _Shoved(TABLE_Z + 0.06)
    driver = arms.driver()
    assert driver.goto("right", (0.04, -0.20, TABLE_Z + 0.14), **DOWN).ok
    assert driver.goto("left", (-0.04, -0.20, TABLE_Z + 0.14), **DOWN).ok
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    text = _said(*_goto(driver, "left", by=(0.0, 0.0, -0.10)))
    assert "it is pushing the right arm" in text, text
    assert "rests on something below" not in text and "pressed against" not in text, text


def test_a_closed_empty_lift_held_down_is_hooked_under_an_edge():
    """store_laptop (R1-R3): straight lifts after a close that read 0.000 ended BLOCKED 12 times of 17, all
    at the height of the lid's edge."""
    hooked = ("the gripper is closed fully and the lift was held down: the fingers are probably hooked under an edge, "
              "not holding anything - open, back out, and grip shallower")
    for floor, said in ((0.0, True), (0.4, False)):
        arms = _Wall(2, TABLE_Z + 0.17, +1)  # nothing above 2 cm over the start
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).ok
        arms.grip_floor = floor
        driver.gripper("left", "close")
        text = _said(*_goto(driver, "left", by=(0.0, 0.0, 0.05)))
        assert "held down from above" in text and (hooked in text) == said, (floor, text)


def test_the_slip_pitch_note_comes_once_per_grasp():
    """R3 gave it 88 times in 10 runs of one task, on every tilted carry its method called for, and 1 of
    them changed a plan."""
    note = "held things slipped 1 time in 3 at pitch 15-30"
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).ok
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    first = driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), pitch=25, yaw=0, roll=0)
    second = driver.goto("left", by=(0.0, 0.02, 0.0))
    assert first.moved and second.moved and note in str(first) and note not in str(second), (str(first), str(second))
    driver.gripper("left", "open")
    assert driver.gripper("left", "close").holding is True  # a new grasp
    third = driver.goto("left", by=(0.0, -0.02, 0.0))
    assert third.moved and note in str(third), str(third)


def test_a_thin_hold_that_reads_closed_fully_may_still_be_held():
    """deposit_coin R3: every first coin lift (closes of 0.4-1.1 cm) stopped after 1-2 actions with "what it
    held has slipped out", and in 6 of 18 the coin had come up between the fingertips, turned flat (one
    agent opened on it).  After a close that caught under THIN_HOLD_M the note says either may be so; the
    stop itself is as before."""
    thin = ("what it held has slipped out, or is still held by its thin side (a hold caught ~0.9 cm wide reads "
            "the same both ways) - look before opening or gripping again")

    def held(floor):
        arms = _Arms()
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.05), **DOWN).ok
        arms.grip_floor = floor
        assert driver.gripper("left", "close").holding is True
        return arms, driver

    def lift(floor):
        arms, driver = held(floor)
        arms.later = {arms.seq + 1: 0.0}  # reads closed fully on the first action of the lift
        moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.10))
        assert moved is None and error.kind == "Dropped", (moved, error)
        assert not driver.carrying("left")
        return error.text

    said = lift(0.06)  # caught ~0.9 cm
    assert "stopped after 1 of" in said and f"the gripper closed fully - {thin}" in said, said
    said = lift(0.10)  # caught ~1.3 cm
    assert "the gripper closed fully - what it held has slipped out" in said and "thin side" not in said, said
    # the same where nothing is stopped: home only reports it
    arms, driver = held(0.06)
    assert driver.goto("left", by=(0.0, 0.0, 0.05)).ok
    arms.grip_floor = 0.0
    homing = driver.home()
    assert homing.dropped and f"the gripper closed fully on the way: {thin}" in str(homing), str(homing)
    arms, driver = held(0.10)
    arms.grip_floor = 0.0
    homing = driver.home()
    assert "has slipped out - look where it fell" in str(homing), str(homing)


def _flipped(hold=True):
    """An empty gripper whose fingers turned half a turn on the way to yaw 150 (they point to -30), then
    closed: on something when ``hold``, else fully."""
    arms = _Arms()
    driver = arms.driver()
    driver.goto("left", WHERE, **DOWN)
    turned = driver.goto("left", WHERE, pitch=90, yaw=150)
    assert ("fingers turned half a turn (the same grasp for an empty gripper), so the wrist camera sees upside down: "
            "pass yaw=-30 from now on - once the gripper closes, yaw=150 is taken as yaw=-30 (with roll=0 too, it "
            "turns the wrist half a turn)") in str(turned), str(turned)
    arms.grip_floor = 0.4 if hold else 0.0
    driver.gripper("left", "close")
    return arms, driver


def _turn(driver, **kw):
    """A move back to WHERE pointing down, and how far (degrees) the hand turned on it."""
    before = driver.grasp_pose("left")[:3, :3]
    move = driver.goto("left", WHERE, pitch=90, **kw)
    return move, math.degrees(angle_between(driver.grasp_pose("left")[:3, :3], before))


def test_after_a_half_turn_a_closed_gripper_keeps_the_fingers_heading():
    """R3: 9 requests for the old yaw while holding, 8 of them in the cell whose approach turned the fingers
    (written before its note was read); 3 shifted a bowl in the fingers."""
    kept = ("kept the fingers' heading, yaw {}: yaw {} names the same grasp half a turn round, and turning there "
            "would take what it holds half a turn round (with roll=0 too, it turns)")
    for yaw, heading, most in ((150, -30, 5.0), (160, -20, 15.0)):
        _, driver = _flipped()
        move, turned = _turn(driver, yaw=yaw)
        assert move.moved and kept.format(heading, yaw) in str(move) and turned < most, (yaw, turned, str(move))
    _, driver = _flipped()
    move, turned = _turn(driver, yaw=-30)
    assert move.moved and "kept the fingers' heading" not in str(move) and turned < 5.0, (turned, str(move))
    _, driver = _flipped()
    move, turned = _turn(driver, yaw=150, roll=0)
    assert move.moved and "kept the fingers' heading" not in str(move) and turned > 170.0, (turned, str(move))
    _, driver = _flipped(hold=False)  # closed fully: it may hold something thin
    move, turned = _turn(driver, yaw=150)
    assert "kept the fingers' heading" in str(move) and turned < 5.0, (turned, str(move))


def test_the_half_turn_alias_lasts_until_the_fingers_are_sent_to_that_yaw():
    # the note followed: an open move at yaw -30, a close, and yaw 150 still names the same grasp
    arms = _Arms()
    driver = arms.driver()
    driver.goto("left", WHERE, **DOWN)
    driver.goto("left", WHERE, pitch=90, yaw=150)
    assert driver.goto("left", by=(0.0, 0.0, -0.02), pitch=90, yaw=-30).moved
    arms.grip_floor = 0.4
    driver.gripper("left", "close")
    move, turned = _turn(driver, yaw=150)
    assert "kept the fingers' heading" in str(move) and turned < 5.0, (turned, str(move))
    # an open move at pitch 60 to yaw 150 sends the fingers there: no alias after it
    arms = _Arms()
    driver = arms.driver()
    driver.goto("left", WHERE, **DOWN)
    driver.goto("left", WHERE, pitch=90, yaw=150)
    tilted = driver.goto("left", (WHERE[0], WHERE[1], 0.883), pitch=60, yaw=150)
    assert tilted.moved and "half a turn" not in str(tilted), str(tilted)
    arms.grip_floor = 0.4
    driver.gripper("left", "close")
    move = driver.goto("left", WHERE, pitch=90, yaw=150)
    assert move.moved and "kept the fingers' heading" not in str(move), str(move)


HAND_OVER = ("only the right arm reaches this pose: hand it over, or set it down where that arm reaches it and grip it "
             "with that arm (a hand-over costs 70-110 steps)")
NOTHING = "nothing close: move the object nearer (push or hand it over) or use the other arm"
FAR = (0.30, -0.10, TABLE_Z + 0.15)


def _holding_left(hold=True):
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).moved
    arms.grip_floor = 0.4 if hold else 0.0
    driver.gripper("left", "close")
    return driver


def test_while_holding_the_other_arm_is_offered_only_when_nothing_else_is_reachable():
    """R3: 62 hand-over offers (49 in dry runs, 51 after another alternative), never taken; in an Open task
    6 hand-overs took 74-107 steps each, where a direct place at a tilted pitch takes 20-35."""
    driver = _holding_left()
    assert driver.goto("left", FAR, **DOWN, dry_run=True).alternatives == [HAND_OVER]
    # once per turn
    assert driver.goto("left", FAR, **DOWN, dry_run=True).alternatives == [NOTHING]
    driver.new_turn()
    assert driver.goto("left", FAR, **DOWN, dry_run=True).alternatives == [HAND_OVER]
    # with steps to spare
    driver.new_turn()
    driver.steps_left = lambda: 100
    assert driver.goto("left", FAR, **DOWN, dry_run=True).alternatives == [NOTHING]
    # where something else is reachable, only that
    move = _holding_left().goto("left", (0.20, -0.10, TABLE_Z + 0.15), **DOWN, dry_run=True)
    assert move.alternatives == ["pitch 30 at the same point"], str(move)
    # not holding: the other arm as before
    move = _holding_left(hold=False).goto("left", FAR, **DOWN, dry_run=True)
    assert "the right arm reaches this exact pose" in move.alternatives and HAND_OVER not in move.alternatives, str(move)
    # not to a hand that holds something (fill_pen_holder R3 l9: the hand holding the cup)
    driver = _holding_left()
    assert driver.gripper("right", "close").holding is True and driver.carrying("right")
    assert driver.goto("left", FAR, **DOWN, dry_run=True).alternatives == [NOTHING]


def _solid(id, label, center, radius, bottom, top, step):
    return Solid(id=id, label=label, center=center, half=(radius, radius), heading=0.0, round=True, bottom=bottom,
                 top=top, views=2, step=step)


def test_a_blocked_note_does_not_name_what_the_hand_holds_measured_again():
    """R3: a slide stopped by a rack's trunk said "touching Thing#4 'gold mug'?" 3 times, of the mug in the
    hand."""

    def blocked(again):
        arms = _Counted(TABLE_Z + 0.06)
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.14), **DOWN).ok
        mug = _solid(4, "gold mug", (-0.25, -0.15), 0.04, TABLE_Z + 0.10, TABLE_Z + 0.18, step=0)
        arms.grip_floor = 0.4
        assert driver.gripper("left", "close", things=[mug]).holding is True
        assert driver.held_thing["left"] == (4, "gold mug"), driver.held_thing
        # measured again next to the hand where it stops, 1.5 cm off the grasp point
        return str(driver.goto("left", by=(0.0, 0.0, -0.10), things=[mug, again(arms.seq + 1)]))

    def near(id, label, step):
        return _solid(id, label, (-0.25, -0.12), 0.015, TABLE_Z + 0.02, TABLE_Z + 0.10, step)

    for id, label in ((4, "gold mug"), (7, "Gold mug ")):
        said = blocked(lambda step: near(id, label, step))
        assert "BLOCKED" in said and "touching Thing#" not in said, (id, said)
    said = blocked(lambda step: near(9, "red box", step))
    assert "touching Thing#9 'red box'?" in said, said
    said = blocked(lambda step: near(7, "gold mug", 1))  # measured before the close: something else
    assert "touching Thing#7 'gold mug'?" in said, said


def test_the_host_tells_the_driver_when_a_turn_has_been_seen():
    seen = []
    host = types.SimpleNamespace(shown={}, looked={"head": "picture"},
                                 driver=types.SimpleNamespace(new_turn=lambda: seen.append(True)))
    RobotHost.pictures_shown(host)
    assert seen == [True] and host.shown == {"head": "picture"} and host.looked == {}
