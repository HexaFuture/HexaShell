"""R3's changes to motion/driver.py and robot/host.py: a release that leaves the fingers on what they
held stops the cell (FREE_GAP_M), and a target a few mm too low for the fingertips is raised
(CLEAR_RAISE_M).  (The slip stop in descents is in test_stops.py; R3's halt after an open descent that
rests on something, its half-turn note and its hand-over offer, as R4 changed them, in
test_r4_motion.py.)"""

from __future__ import annotations

from hexashell.motion import ik
from hexashell.motion.pose import rotation_of
from hexashell.robot.host import RobotHost
from hexashell.world import TABLE_Z
from test_motion import _Arms
from test_stops import _goto, _host

from hexashell.kernel.host import CallFailed

DOWN = dict(pitch=90, yaw=0, roll=0)
CAUGHT = "under 2 cm wider than what it held"


def _gripper(driver, arm, opening):
    """RobotHost's gripper: (the Grip, None) or (None, the CallFailed)."""
    try:
        return RobotHost._gripper(_host(driver), arm, opening), None
    except CallFailed as exc:
        return None, exc


def _holding(arms, floor=0.4, z=0.15):
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + z), **DOWN).ok
    arms.grip_floor = floor
    assert driver.gripper("left", "close").holding is True
    arms.grip_floor = 0.0  # it lets go once opened
    return driver


def test_a_release_that_leaves_the_fingers_on_it_stops_the_cell():
    """In R0's Open tasks, 7 held things came away with the hand after opens that stopped at 0.41-0.48,
    less than 2 cm wider than what they held, and the lift that followed in the cell took them along."""
    arms = _Arms()
    driver = _holding(arms)  # 4.6 cm wide
    arms.grip_cap = 0.55  # held back at a 4.0 cm gap
    grip, error = _gripper(driver, "left", "open")
    assert grip is None and error.kind == "Blocked", grip
    assert error.text.startswith("Grip left: open 0.55") and error.text.endswith(
        "\nthe rest of the cell was skipped so you can look"), error.text
    assert (f"opened only to 0.55, {CAUGHT}: something beside the fingers holds them, and it may still be caught "
            "between them - back off a little the way the hand came in, then open again") in error.text, error.text
    # near the table: the same stop, with the note for fingers pressed on it
    arms = _Arms()
    driver = _holding(arms, z=0.02)
    arms.grip_cap = 0.55
    grip, error = _gripper(driver, "left", "open")
    assert error is not None and "pressed on the table" in error.text and CAUGHT in error.text, (grip, error)


def test_a_release_that_opens_well_past_what_it_held_or_nearly_fully_goes_on():
    """R1: 19 holding opens stopped more than 0.05 short and all let go; 3 of them opened to 0.87-0.94
    around things 6-7 cm wide.  Into a narrow holder the fingers stop short but well past the width."""
    for floor, cap in ((0.2, 0.70), (0.6, 0.90)):  # 2.4 cm held, a 5.6 cm gap; 6.8 cm held, nearly open
        arms = _Arms()
        driver = _holding(arms, floor=floor)
        arms.grip_cap = cap
        grip, error = _gripper(driver, "left", "open")
        assert error is None and grip.halt == "" and f"opened only to {cap:.2f}: something beside" in str(grip), (
            floor, error)
        assert CAUGHT not in str(grip)
    # not holding anything: only the note, as before
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), **DOWN).ok
    assert driver.gripper("left", "close").holding is False
    arms.grip_cap = 0.39
    grip, error = _gripper(driver, "left", "open")
    assert error is None and "opened only to 0.39" in str(grip) and CAUGHT not in str(grip), error


def test_a_release_on_the_way_stops_the_cell_too():
    arms = _Arms()
    driver = _holding(arms)
    arms.grip_cap = 0.55
    moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.03), grip="open")
    assert moved is None and error.kind == "Blocked", moved
    assert f"opened only to 0.55, {CAUGHT}" in error.text and error.text.endswith(
        "\nthe move ran; the rest of the cell was skipped so you can look"), error.text
    arms = _Arms()
    driver = _holding(arms)
    arms.grip_cap = 0.55
    moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.03), grip="open", contact=True)
    assert error is None and moved.halt == "", error
    arms = _Arms()
    driver = _holding(arms)  # it opens fully: nothing to say
    moved, error = _goto(driver, "left", by=(0.0, 0.0, 0.03), grip="open")
    assert error is None and "opened only" not in str(moved), error


def test_a_target_a_few_mm_too_low_for_the_fingertips_is_raised():
    """R1 refused 40 poses for the fingertips; from the same joints 31 now go, 1-5 mm higher."""
    driver = _Arms().driver()
    low = driver.goto("left", (-0.15, -0.15, TABLE_Z + 0.010), **DOWN)  # 0.775: 5 mm under 0.780
    assert low.moved and abs(low.at[2] - 0.780) < 0.002 and abs(low.target[2] - 0.780) < 1e-9, str(low)
    assert "note: z 0.775 is too low for the fingertips to clear the table at these angles: going to z 0.780, 5 mm " \
           "higher" in str(low), str(low)
    dry = _Arms().driver().goto("left", (-0.15, -0.15, 0.778), **DOWN, dry_run=True)
    assert dry.reached and "going to z 0.780, 2 mm higher" in str(dry), str(dry)
    tilted = _Arms().driver().goto("left", (-0.15, -0.15, 0.775), pitch=60, yaw=0, roll=0)
    lowest = ik.lowest_z(rotation_of(60.0, 0.0, 0.0))
    assert tilted.moved and lowest - 0.001 <= tilted.target[2] <= lowest and "too low for the fingertips" in str(
        tilted), str(tilted)
    far = _Arms().driver().goto("left", (-0.15, -0.15, TABLE_Z + 0.008), **DOWN)  # 7 mm: refused
    assert not far.moved and "fingertips into the table" in str(far), str(far)
    fine = _Arms().driver().goto("left", (-0.15, -0.15, 0.780), **DOWN)
    assert fine.moved and "too low" not in str(fine), str(fine)
