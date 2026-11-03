"""Measuring an object again after a hand carried it (vision/scene.py, robot/host.py): the host tells the
scene when a close catches something and when the hand lets go, and a re-measure looks where the hand set
it down; one that finds nothing there takes the nearest outline of the words that shows no other Thing.
(Base test 10-28: re-measures after a carry looked where the object had been, and took the nearest
outline of the words there, at times another Thing's object.)"""

from __future__ import annotations

import math

import numpy as np

from fakes import FakeTable
from hexashell.geometry import homogeneous, rot_z
from hexashell.motion.chain import slide
from hexashell.robot.host import RobotHost
from hexashell.state import parse_obs
from hexashell.vision.scene import Scene
from hexashell.world import ARMS, GRASP_REACH, TABLE_Z
from test_vision import OutlinesByColour, _Host, _hover_q, on_table

DOWN = dict(pitch=90, yaw=0, roll=0)
A_XY = (-0.15, -0.15)  # in the left arm's reach


class _GripTable(FakeTable):
    """FakeTable whose closing fingers stop 0.3 open on a box at the grasp point and hold it: it moves with
    the hand, and drops onto the table when they open."""

    def __init__(self, boxes, **kw):
        super().__init__(boxes, **kw)
        self.held: dict[str, tuple[int, np.ndarray] | None] = {arm: None for arm in ARMS}

    def apply(self, action: dict) -> None:
        super().apply(action)
        for arm in ARMS:
            grasp = slide(self.chains[arm].flange(self.q[arm]), GRASP_REACH)
            if float(action[f"{arm}_ee_joint_state"][0]) > 0.05:
                if self.held[arm] is not None:
                    box = self.boxes[self.held[arm][0]]
                    box.center = (box.center[0], box.center[1], TABLE_Z + box.size[2] / 2)
                    self.held[arm] = None
                continue
            if self.held[arm] is None:
                for i, box in enumerate(self.boxes):
                    if np.linalg.norm(np.asarray(box.center) - grasp[:3, 3]) < 0.015:
                        pose = homogeneous(rot_z(math.radians(box.yaw_deg)), box.center)
                        self.held[arm] = (i, np.linalg.inv(grasp) @ pose)
            if self.held[arm] is not None:
                self.grip[arm] = max(self.grip[arm], 0.3)
                i, local = self.held[arm]
                pose = grasp @ local
                self.boxes[i].center = tuple(float(v) for v in pose[:3, 3])
                self.boxes[i].yaw_deg = math.degrees(math.atan2(pose[1, 0], pose[0, 0]))


class _TableHost(RobotHost):
    """A RobotHost on the fake table, without a bridge."""

    def __init__(self, table: FakeTable, scene: Scene):
        self.table = table
        super().__init__(None, parse_obs(table.obs(), 0), scene=scene)

    def over(self) -> bool:
        return False

    def act(self, action: dict):
        self.table.apply(action)
        self.snap = parse_obs(self.table.obs(), self.snap.seq + 1, previous=self.snap)
        return self.snap


def _two_red_blocks():
    """Two red blocks 8 cm apart (their reds differ a little, so that the fake SAM3 tells them apart)."""
    a = on_table(A_XY, (0.04, 0.03, 0.04), 0.0, "red block", (200, 40, 40))
    b = on_table((A_XY[0], A_XY[1] + 0.08), (0.04, 0.03, 0.04), 0.0, "red block", (206, 44, 40))
    return a, b


def _gap(xyz, box) -> float:
    return math.hypot(xyz[0] - box.center[0], xyz[1] - box.center[1])


def test_a_carried_thing_is_measured_where_the_hand_set_it_down():
    a, b = _two_red_blocks()
    table = _GripTable([a, b], start_q={"left": _hover_q("left", A_XY), "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    host = _TableHost(table, scene)
    events = []
    scene.log = events.append

    def call(name, *args, **kw):
        return host.dispatch(name, args, kw)

    try:
        first = call("measure", "red block")
        assert first.found and _gap(first.center, a) < 0.005, first
        x, y = scene.view("head").camera.pixel(b.center)
        other = call("measure", ("head", (x, y)))
        assert other.found and other.id != first.id and _gap(other.center, b) < 0.005, other
        x, y, z = a.center
        assert call("goto", "left", (x, y, z + 0.10), grip=1.0, **DOWN).ok
        assert call("goto", "left", (x, y, z), **DOWN).ok
        assert call("gripper", "left", 0.0).holding and table.held["left"] is not None
        assert list(scene.carried["left"]) == [first.id]
        assert [e for e in events if e["kind"] == "carry"][-1] == {"kind": "carry", "arm": "left", "held": [first.id],
                                                                   "pick": first.id}
        # carried 13 cm to the left, set down there and let go
        assert call("goto", "left", (x, y, z + 0.08), **DOWN).ok
        assert call("goto", "left", (x - 0.12, y - 0.04, z + 0.08), **DOWN).ok
        assert call("goto", "left", (x - 0.12, y - 0.04, z + 0.005), **DOWN).ok
        assert not call("gripper", "left", 1.0).holding and table.held["left"] is None
        assert not scene.carried["left"] and scene.records[first.id].set_down
        assert _gap((x - 0.12, y - 0.04), a) < 0.002
        call("home", "left")
        # the other red block is nearer where the first one was measured than where it was set down
        again = call("measure", first)
        assert again.found and again.id == first.id and _gap(again.center, a) < 0.005, again
        assert not any(n.startswith("not found") for n in again.notes), again.notes
        assert any(n.startswith("moved 12.") for n in again.notes), again.notes
        assert _gap(call("measure", other).center, b) < 0.005
    finally:
        scene.close()


def test_a_thing_not_found_again_does_not_take_another_things_object():
    a, b = _two_red_blocks()
    table = FakeTable([a, b], start_q={"left": _hover_q("left", A_XY), "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    scene.bind(_Host(table))
    try:
        first = scene.measure("red block")
        x, y = scene.view("head").camera.pixel(b.center)
        other = scene.measure(("head", (x, y)))
        assert first.found and other.found and other.id != first.id
        # moved by something else than a hand (pushed, rolled): found again by its words, away from the other
        a.center = (a.center[0] - 0.14, a.center[1] - 0.03, a.center[2])
        scene.host.snap = parse_obs(table.obs(), 2)
        again = scene.measure(first)
        assert again.found and again.id == first.id and _gap(again.center, a) < 0.005, again
        assert again.notes[0] == "not found where it was last measured: measured the 'red block' nearest to there"
        # gone: the only red block left is the other Thing
        table.boxes.remove(a)
        scene.host.snap = parse_obs(table.obs(), 3)
        gone = scene.measure(again)
        assert not gone.found and gone.id == first.id, gone
        assert gone.why_missing.startswith("not where it was, and each 'red block' outlined in head is where another "
                                           f"Thing is: Mask#"), gone.why_missing
        assert f"(Thing#{other.id})" in gone.why_missing, gone.why_missing
        assert gone.notes == ["measure one of those Masks if it is the one you mean"], gone.notes
    finally:
        scene.close()


def test_a_hand_takes_over_what_the_other_carries():
    a, b = _two_red_blocks()
    table = FakeTable([a, b], start_q={"left": _hover_q("left", A_XY), "right": np.zeros(6)})
    scene = Scene("http://fake", client=OutlinesByColour(table))
    scene.bind(_Host(table))
    try:
        first = scene.measure("red block")
        x, y = scene.view("head").camera.pixel(b.center)
        other = scene.measure(("head", (x, y)))
        poses = {arm: homogeneous(np.eye(3), a.center) for arm in ARMS}
        scene._grasp_pose = lambda arm: poses[arm].copy()
        scene._holding = lambda arm: True
        # the driver's pick is taken when it is about at the grasp point, else the nearest one there
        scene.grasped(poses["left"], "left", thing=other.id)
        assert list(scene.carried["left"]) == [first.id]
        scene.released(poses["left"], "left")
        scene.grasped(homogeneous(np.eye(3), (a.center[0], a.center[1] + 0.04, a.center[2])), "left",
                      thing=other.id)
        assert list(scene.carried["left"]) == [other.id]
        scene.released(homogeneous(np.eye(3), (a.center[0], a.center[1] + 0.04, a.center[2])), "left")
        scene.grasped(poses["left"], "left")
        # the left hand carries it 10 cm on and turns it 90 degrees; the right takes it there; the left lets go
        poses["left"] = homogeneous(rot_z(math.pi / 2), np.asarray(a.center) + (0.0, -0.10, 0.0))
        poses["right"] = poses["left"].copy()
        scene.grasped(poses["right"], "right")
        assert list(scene.carried["right"]) == [first.id] and not scene.carried["left"]
        scene.released(poses["left"] @ homogeneous(np.eye(3), (0.0, 0.0, 0.05)), "left")  # nothing to set down
        poses["right"] = homogeneous(rot_z(math.pi / 2), np.asarray(a.center) + (-0.05, -0.10, 0.0))
        scene.released(poses["right"], "right")
        rec = scene.records[first.id]
        assert rec.set_down and not scene.carried["right"]
        assert np.allclose(rec.solid.mean(axis=0)[:2], np.asarray(a.center[:2]) + (-0.05, -0.10), atol=0.003)
        assert abs((rec.yaw - scene.records[other.id].yaw) % 180.0 - 90.0) < 1.0, rec.yaw
    finally:
        scene.close()
