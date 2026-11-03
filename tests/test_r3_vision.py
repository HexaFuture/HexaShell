"""R3 vision fixes: what measure says when its cameras cannot pin an object down.

Outlines come from the colour stand-in for SAM3 in test_vision, on FakeTable's exact pictures; where
it matters, the robot is drawn over the boxes as the real pictures show it.
"""

from __future__ import annotations

import math
import re

import numpy as np
import pytest

from fakes import Box, FakeTable
from hexashell.geometry import homogeneous
from hexashell.motion import ik
from hexashell.motion.chain import arm_chain
from hexashell.motion.pose import rotation_of
from hexashell.state import ArmReading, parse_obs
from hexashell.vision import hull
from hexashell.vision import scene as scene_module
from hexashell.vision.robotmask import label_name, robot_labels, robot_mask
from hexashell.vision.scene import Scene
from hexashell.world import ARMS, TABLE_Z
from test_vision import OutlinesByColour, _Bowl, _Host, on_table


class Bottle(Box):
    """A standing cylinder (drawn as its convex outline)."""

    def __init__(self, xy, radius=0.04, tall=0.213, color=(200, 40, 40), label="red bottle"):
        super().__init__(label, (xy[0], xy[1], TABLE_Z + tall / 2), (2 * radius, 2 * radius, tall), 0.0, color)
        a = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
        ring = np.stack([xy[0] + radius * np.cos(a), xy[1] + radius * np.sin(a)], axis=1)
        self.points = np.concatenate([np.c_[ring, np.full(len(a), TABLE_Z)],
                                      np.c_[ring, np.full(len(a), TABLE_Z + tall)]])

    def corners(self) -> np.ndarray:
        return self.points


class RobotInFront(FakeTable):
    """FakeTable with the robot drawn over the boxes, as real pictures show it: a box behind a gripper
    shows only around it, so its outline is cut there."""

    def render(self, cam):
        img = super().render(cam)
        arms = {a: ArmReading(self.q[a].copy(), self.chains[a].flange(self.q[a]), self.grip[a]) for a in ARMS}
        img[robot_mask(cam, arms, margin_px=0)] = (20, 20, 20)
        return img


def _scene(boxes, table_type=FakeTable, **kwargs):
    table = table_type(boxes, **kwargs)  # both arms at the start pose unless start_q says otherwise
    scene = Scene("http://fake", client=OutlinesByColour(table))
    scene.bind(_Host(table))
    return scene, table


# -- a second camera that sees only part of the object ------------------------------------------------
@pytest.mark.parametrize("wrist", ["right_wrist", "left_wrist"])
def test_a_bottle_a_wrist_camera_sees_only_at_its_edge_is_flagged_as_maybe_standing(wrist):
    # In front of the head, below the start-pose wrist cameras: they show its foot at their picture's
    # edge, over their own fingers, which cannot rule out the space the bottle hides from the head -
    # the box reads low and long down the head's view.  The box stays; the warning gives the head's
    # own reading and where to look from.
    scene, table = _scene([Bottle((0.0, 0.0))], RobotInFront)
    try:
        thing = scene.measure([scene.find(cam, "red bottle")[0] for cam in ("head", wrist)])
        assert thing.found and thing.views == ["head", wrist] and thing.size[2] < 0.1, thing
        said = re.search(rf"the {wrist} camera sees only part of it: it may stand (\d+) mm tall at "
                         r"\(([-\d.]+), ([-\d.]+)\), as the head picture alone suggests, not lie low - ",
                         thing.warning or "")
        assert said, thing.warning
        assert abs(int(said[1]) - 213) <= 15, thing.warning
        assert math.hypot(float(said[2]), float(said[3])) <= 0.02, thing.warning
        assert "robot.goto(" in thing.warning, thing.warning
    finally:
        scene.close()


def test_a_stick_lying_down_the_head_view_is_not_flagged():
    # The same views of a thing that does lie low and long down the head's view: the wrist outline
    # meets enough of the head's rays to rule out its standing.
    stick = Box("red stick", (0.0, 0.08, TABLE_Z + 0.015), (0.05, 0.25, 0.03))
    scene, table = _scene([stick], RobotInFront)
    try:
        thing = scene.measure([scene.find(cam, "red stick")[0] for cam in ("head", "right_wrist")])
        assert thing.found and thing.views == ["head", "right_wrist"], thing
        assert "may stand" not in (thing.warning or ""), thing.warning
        assert abs(thing.size[2] - 0.03) <= 0.01 and abs(thing.size[0] - 0.25) <= 0.02, thing
    finally:
        scene.close()


def test_a_bottle_a_wrist_camera_sees_whole_is_measured_as_before():
    scene, table = _scene([Bottle((0.2, 0.0))])
    try:
        thing = scene.measure("red bottle")
        assert thing.found and len(thing.views) >= 2, thing
        assert "may stand" not in (thing.warning or ""), thing.warning
        assert abs(thing.top - (TABLE_Z + 0.213)) <= 0.005, thing
        assert math.hypot(thing.center[0] - 0.2, thing.center[1]) <= 0.005, thing
    finally:
        scene.close()


# -- where to look at a big object from -----------------------------------------------------------------
@pytest.mark.parametrize("distance", ["fits", "close"])
def test_a_big_box_is_looked_at_from_where_it_fits_the_wrist_picture(distance, monkeypatch):
    # From the usual 23 cm a box 0.22 m square fills about half the wrist picture, and an outline over
    # MAX_PICTURE_SHARE of a picture is dropped as the table: the advised pose is farther, and measured
    # from there the wrist camera's outline is used.  From the old, closer pose it is not.
    if distance == "close":
        monkeypatch.setattr(scene_module, "HOVER_SHARE", 10.0)
    box = Box("green box", (0.05, -0.05, TABLE_Z + 0.03), (0.22, 0.22, 0.06), 0.0, (40, 160, 60))
    scene, table = _scene([box])
    try:
        shape = hull.Shape(views=["head"], top=TABLE_Z + 0.06, bottom=TABLE_Z, center=box.center, size=box.size,
                           yaw=0.0)
        arm, point, pitch, clear = scene._look_pose(shape, set())
        sol, _ = ik.solve(arm_chain(arm), homogeneous(rotation_of(pitch, 0.0, 0.0), point), table.q[arm])
        assert sol.reached and min(clear.values()) >= scene_module.LOOK_CLEAR, (arm, point, pitch, clear)
        table.q[arm] = sol.q
        scene.host.snap = parse_obs(table.obs(), 2)
        wrist = f"{arm}_wrist"
        filled = float(np.all(scene.view(wrist).image == box.color, axis=-1).mean())
        thing = scene.measure("green box", cams=["head", wrist])
        assert thing.found, thing
        if distance == "fits":
            assert filled <= 0.27 and thing.views == ["head", wrist], (filled, thing)
            assert abs(thing.size[0] - 0.22) <= 0.01 and abs(thing.size[1] - 0.22) <= 0.01, thing
            assert abs(thing.top - (TABLE_Z + 0.06)) <= 0.01, thing
        else:
            assert filled > 0.3 and thing.views == ["head"], (filled, thing)
    finally:
        scene.close()


# -- a pixel the robot's own arm covers ------------------------------------------------------------------
def _left_over(point, pitch) -> np.ndarray:
    sol, _ = ik.solve(arm_chain("left"), homogeneous(rotation_of(pitch, 0.0, 0.0), np.array(point)), np.zeros(6))
    assert sol.reached
    return sol.q


@pytest.mark.parametrize("how", ["pixel", "mask"])
def test_a_block_under_the_arm_in_the_head_picture_is_reported_covered(how):
    # The left hand looks down from between the head camera and a block: in the head picture it covers
    # the block, and SAM3 outlines the hand (or nothing) there.  Measured from a pixel picked before the
    # arm moved, or from find(at=) there, it is reported covered by the left arm, not as nothing there.
    red = on_table((0.097, -0.138), (0.04, 0.04, 0.04), 0.0, "red block", (200, 40, 40))
    blue = on_table((0.12, 0.05), (0.04, 0.04, 0.04), 0.0, "blue block", (40, 60, 200))
    scene, table = _scene([red, blue], RobotInFront,
                          start_q={"left": _left_over((0.0, -0.22, 0.91), 60), "right": np.zeros(6)})
    try:
        head = scene.view("head")
        x, y = head.camera.pixel(np.array([red.center[0], red.center[1], TABLE_Z + 0.04]))
        labels = robot_labels(head.camera, head.arms)
        assert label_name(labels[int(round(y)), int(round(x))]).startswith("left"), "the hand covers the block"
        if how == "pixel":
            thing = scene.measure(("head", (x, y)))
        else:  # SAM3's answer to a click on the hand: the hand
            hand = labels > 0
            mask = scene.register(head, hand, f"at ({x:.0f}, {y:.0f})", 0.3, {"points": [(x, y)], "labels": [1]})
            thing = scene.measure(mask)
        assert not thing.found, thing
        assert thing.why_missing == (f"the left arm covers ({x:.0f}, {y:.0f}) in the head picture: move the arm aside "
                                     "and measure again, or measure it in another camera's picture"), thing.why_missing
        other = scene.measure(("head", head.camera.pixel(np.array([blue.center[0], blue.center[1], TABLE_Z + 0.04]))))
        assert other.found and abs(other.size[2] - 0.04) <= 0.005, other
    finally:
        scene.close()


def test_a_block_clear_of_the_arms_is_measured_from_the_same_pixel():
    red = on_table((0.097, -0.138), (0.04, 0.04, 0.04), 0.0, "red block", (200, 40, 40))
    scene, table = _scene([red], RobotInFront)  # both arms at the start pose
    try:
        head = scene.view("head")
        thing = scene.measure(("head", head.camera.pixel(np.array([red.center[0], red.center[1], TABLE_Z + 0.04]))))
        assert thing.found and math.hypot(thing.center[0] - 0.097, thing.center[1] + 0.138) <= 0.005, thing
    finally:
        scene.close()


@pytest.mark.parametrize("case", ["hidden", "hidden, by words", "moved"])
def test_a_thing_whose_place_the_arm_covers_is_reported_covered_when_measured_again(case):
    # Measured with both arms at the start pose (from a pixel, or from words).  "hidden": the left hand
    # then moves between the head camera and the block, and the block is measured again from the head
    # picture alone - nothing shows there, and the reason names the arm.  "moved": the block is taken
    # away and the arms stay home, so nothing covers its place: the reason is as before.
    red = on_table((0.097, -0.138), (0.04, 0.04, 0.04), 0.0, "red block", (200, 40, 40))
    scene, table = _scene([red], RobotInFront)
    try:
        head = scene.view("head")
        if case == "hidden, by words":
            thing = scene.measure("red block")
        else:
            thing = scene.measure(("head", head.camera.pixel(np.array([red.center[0], red.center[1], TABLE_Z + 0.04]))))
        assert thing.found, thing
        if case.startswith("hidden"):
            table.q["left"] = _left_over((0.0, -0.22, 0.91), 60)
        else:
            table.boxes.remove(red)
        scene.host.snap = parse_obs(table.obs(), 2)
        again = scene.measure(thing, cams=["head"] if case.startswith("hidden") else "auto")
        assert not again.found, again
        covered = "the left arm covers that place in the head picture: move the arm aside and measure again"
        if case == "hidden":
            assert again.why_missing == (f"nothing is where it was last measured (it moved?) - {covered}, or find it "
                                         "again in the pictures"), again.why_missing
        elif case == "hidden, by words":
            assert again.why_missing.startswith("not where it was, and no 'red block' outlined in head"), again
            assert again.why_missing.endswith(f" - {covered}"), again.why_missing
        else:
            assert again.why_missing == ("nothing is where it was last measured (it moved?): find it again in the "
                                         "pictures"), again.why_missing
    finally:
        scene.close()


def test_an_outline_found_again_on_the_arm_over_a_thing_names_the_arm():
    # Measuring a remembered block again, SAM3 outlines the left hand that now covers its place (as the
    # re-find does from the words or a click there): the reason says so, and which arm covers the place.
    red = on_table((0.097, -0.138), (0.04, 0.04, 0.04), 0.0, "red block", (200, 40, 40))
    scene, table = _scene([red], RobotInFront)
    try:
        head = scene.view("head")
        thing = scene.measure(("head", head.camera.pixel(np.array([red.center[0], red.center[1], TABLE_Z + 0.04]))))
        assert thing.found, thing
        table.q["left"] = _left_over((0.0, -0.22, 0.91), 60)
        scene.host.snap = parse_obs(table.obs(), 2)
        head, rec = scene.view("head"), scene.records[thing.id]
        hand = scene.register(head, robot_labels(head.camera, head.arms) > 0, thing.label, 0.3,
                              {"again": f"Thing#{thing.id}"})
        start = scene_module._Target(hand, head, thing.label, clicks=[rec.solid.mean(axis=0)], place=rec.solid)
        again = scene._measure(start, ["head"], prior=rec)[0]
        assert not again.found, again
        assert again.why_missing == (f"Mask#{hand.id} lies on the robot's own arm or gripper - the left arm covers "
                                     "that place in the head picture: move the arm aside and measure again"), again
    finally:
        scene.close()


def test_an_outline_of_the_arm_found_by_words_is_reported_as_before():
    scene, table = _scene([], RobotInFront, start_q={"left": _left_over((0.0, -0.22, 0.91), 60), "right": np.zeros(6)})
    try:
        head = scene.view("head")
        mask = scene.register(head, robot_labels(head.camera, head.arms) > 0, "red block", 0.3, {"text": "red block"})
        thing = scene.measure(mask)
        assert not thing.found and thing.why_missing == f"Mask#{mask.id} lies on the robot's own arm or gripper", thing
    finally:
        scene.close()


# -- the note for things widest at the top -------------------------------------------------------------
@pytest.mark.parametrize("gate", ["on", "off"])
def test_a_thin_thing_read_as_hanging_from_its_rim_gets_no_bowl_note(gate, monkeypatch):
    # A thin disc a little wider on top (15 mm high) reads as hanging from its rim, like a bowl; the
    # bowl note misled on tiles and coins.  Bowls keep it (test_vision); with the gate off, it shows.
    if gate == "off":
        monkeypatch.setattr(scene_module, "HANG_NOTE_MIN", 0.0)
    centre, tall = (0.02, 0.03), 0.015
    scene, table = _scene([_Bowl(centre, 0.035, 0.02, tall)])
    try:
        camera = scene.view("head").camera
        thing = scene.measure(("head", camera.pixel((centre[0], centre[1], TABLE_Z + tall))))
        assert thing.found and thing.size[2] < 0.025, thing
        noted = any(n.startswith("widest at the top") for n in thing.notes)
        assert noted == (gate == "off"), thing.notes
    finally:
        scene.close()
