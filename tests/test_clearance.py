"""Path clearance (motion/driver.py): goto and its dry run say when the planned path goes into the box
of a measured Thing below its top - the fingers, or what the arm carries - and a refusal does not offer
a joint swing that would.  It is only a note.  The Thing the move ends at (grasped, or placed into) is
left out for the hand; what it carries can still come in through its side.  Things that a close caught
since they were measured, or that were measured again under a new number, leave no box behind."""

from __future__ import annotations

import re
import types

import numpy as np

from hexashell.geometry import homogeneous
from hexashell.motion import ik
from hexashell.motion.driver import Solid, current, first_hit, path_points
from hexashell.motion.pose import rotation_of
from hexashell.robot import results as R
from hexashell.robot.host import RobotHost
from hexashell.world import TABLE_Z
from test_motion import _Arms

NOTE = "and may hit it - lift first, or come from above"


def _box(id, label, x, y, top, half=(0.03, 0.03), round=True, bottom=TABLE_Z, views=2, step=0):
    return Solid(id=id, label=label, center=(x, y), half=half, heading=0.0, round=round, bottom=bottom,
                 top=TABLE_Z + top, views=views, step=step)


SWING = dict(xyz=(-0.05, 0.0, TABLE_Z + 0.05), pitch=90, yaw=0, roll=0)


def _on_the_swing(arms):
    """A cup standing where the far swing of SWING passes, 3 cm above its grasp point."""
    driver = arms.driver()
    chain, q0 = driver.chains["left"], arms.q["left"].copy()
    goal = homogeneous(rotation_of(SWING["pitch"], SWING["yaw"], SWING["roll"]), SWING["xyz"])
    sol, _ = ik.solve(chain, goal, q0, extra_seeds=[q0])
    actions = ik.actions_for(ik.Path([q0, sol.q], "swing"), driver.speed)
    p = chain.grasp_pose(actions[len(actions) // 2])[:3, 3]
    cup = _box(1, "cup", p[0], p[1], p[2] - TABLE_Z + 0.03)
    for end in (chain.grasp_pose(q0)[:3, 3], np.array(SWING["xyz"])):
        assert np.linalg.norm(end[:2] - p[:2]) > 0.06  # it is neither where the swing starts nor ends
    return driver, cup


def test_swing_through_a_measured_thing_is_noted_and_not_offered():
    """R1: 9 of 10 runs of one task took the refusal's own offer of path='swing' to a pose beside a cup
    at grasp height; 2 of the swings knocked the cup over and lost the run."""
    driver, cup = _on_the_swing(_Arms())
    refused = driver.goto("left", **SWING, dry_run=True, things=[cup])
    assert not refused.ok and "joint swing would carry the grasp point" in str(refused), str(refused)
    assert "below the top of Thing#1 'cup'" in str(refused) and "path='swing'" not in str(refused), str(refused)
    elsewhere = _box(1, "cup", 0.30, -0.30, 0.10)
    offered = driver.goto("left", **SWING, dry_run=True, things=[elsewhere])
    assert "path='swing'" in str(offered) and "Thing#1" not in str(offered), str(offered)
    line = driver.goto("left", **SWING, path="line", dry_run=True, things=[cup])
    assert "try a waypoint: on a joint swing the fingers go" in str(line), str(line)
    planned = driver.goto("left", **SWING, path="swing", dry_run=True, things=[cup])
    assert re.search(rf"on the way the fingers go [34] cm below the top of Thing#1 'cup' {NOTE}$", str(planned)), str(planned)
    swung = driver.goto("left", **SWING, path="swing", things=[cup])  # a note, never a refusal
    assert swung.moved and str(swung).count("below the top of Thing#1") == 1, str(swung)


def _holding_a_bottle(arms):
    """The left hand holds a 10 cm bottle by its top, 8 cm above the table, lifted to 20 cm."""
    driver = arms.driver()
    bottle = _box(2, "bottle", -0.25, -0.15, 0.10, half=(0.02, 0.02))
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.08), pitch=90, yaw=0, roll=0, things=[bottle]).ok
    arms.grip_floor = 0.3
    assert driver.gripper("left", "close").holding is True
    assert driver.goto("left", by=(0.0, 0.0, 0.12), things=[bottle]).ok
    return driver, bottle


def test_carry_into_a_box_through_its_wall_is_noted():
    """R1: two carries that lowered a held shoe into a box through its near wall pushed the box out of
    reach; the hand cleared the box's top, the shoe hanging below it did not."""
    arms = _Arms()
    driver, bottle = _holding_a_bottle(arms)
    box = _box(3, "box", -0.10, -0.15, 0.10, half=(0.05, 0.05), round=False)
    things = [bottle, box]
    low = driver.goto("left", (-0.10, -0.15, TABLE_Z + 0.11), dry_run=True, things=things)
    assert f"what it holds goes 7 cm below the top of Thing#3 'box' {NOTE}" in str(low), str(low)
    assert "Thing#2" not in str(low), str(low)  # the bottle moves with the hand
    high = driver.goto("left", (-0.10, -0.15, TABLE_Z + 0.20), dry_run=True, things=things)
    assert "below the top" not in str(high), str(high)
    over = driver.goto("left", (-0.10, -0.15, TABLE_Z + 0.20), things=things)
    down = driver.goto("left", (-0.10, -0.15, TABLE_Z + 0.09), things=things)  # into it from above
    assert over.ok and down.ok and "below the top" not in str(over) + str(down), str(over) + str(down)


def test_ordinary_grasp_next_to_others_is_quiet():
    """Coming down on a Thing (open fingers) says nothing about that Thing, nor about a neighbour the
    open fingers pass; a neighbour right under a finger is named."""
    driver = _Arms().driver()
    block = _box(4, "block", -0.25, -0.15, 0.04, half=(0.02, 0.02), round=False)
    # beside it along the fingers' closing direction (x at yaw 0), the open fingers 8.9 cm apart
    near = _box(5, "red block", -0.175, -0.15, 0.04, half=(0.02, 0.02), round=False)  # 1 cm clear of a finger
    touching = _box(6, "blue block", -0.205, -0.15, 0.04, half=(0.02, 0.02), round=False)  # under a finger
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.15), pitch=90, yaw=0, roll=0, grip="open").ok
    quiet = driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.02), dry_run=True, things=[block, near])
    assert quiet.ok and "below the top" not in str(quiet), str(quiet)
    named = driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.02), dry_run=True, things=[block, touching])
    assert "the fingers go" in str(named) and "Thing#6 'blue block'" in str(named), str(named)


def test_things_measured_again_leave_no_ghost():
    """R1: a cup knocked over and measured again got a new number; its old box sat where it had stood."""
    old = _box(1, "cup", 0.0, 0.0, 0.08, step=0)
    new = _box(7, "cup", 0.3, 0.0, 0.08, step=40)
    inside = _box(8, "at (100, 200)", 0.30, 0.01, 0.08, step=50)  # the same cup measured by a pixel
    other = _box(9, "bowl", 0.0, 0.2, 0.05, step=10)
    assert [s.id for s in current([old, new, other])] == [7, 9]
    assert [s.id for s in current([old, new, inside, other])] == [8, 9]
    points = np.array([[[x, 0.0, TABLE_Z + 0.04]] for x in np.linspace(-0.2, 0.2, 41)])
    assert first_hit(points, ["fingers"], [old], end=(0.5, 0.5, 1.0)) is not None
    assert first_hit(points, ["fingers"], current([old, new]), end=(0.5, 0.5, 1.0)) is None


def test_held_items_count_deeper_and_one_view_things_are_shrunk():
    arms = _Arms()
    driver = arms.driver()
    chain = driver.chains["left"]
    pts, kinds = path_points(chain, [arms.q["left"]], 0.0, 0.05)
    assert kinds == ["fingers", "fingers", "held"] and pts.shape == (1, 3, 3)
    track = np.array([[[x, 0.0, TABLE_Z + 0.08], [x, 0.0, TABLE_Z + 0.067], [x, 0.0, TABLE_Z + 0.075]]
                      for x in np.linspace(-0.2, 0.2, 41)])
    cup = _box(1, "cup", 0.0, 0.0, 0.10)  # held point 2.5 cm below its top: not yet in
    assert first_hit(track[:, 2:], ["held"], [cup], end=(0.5, 0.5, 1.0)) is None
    deeper = track.copy()
    deeper[:, 2, 2] = TABLE_Z + 0.06
    solid, what, depth = first_hit(deeper[:, 2:], ["held"], [cup], end=(0.5, 0.5, 1.0))
    assert what == "held" and abs(depth - 0.04) < 1e-6
    one_view = _box(1, "cup", 0.0, 0.0, 0.085, views=1)  # 1.8 cm above the fingertips
    assert first_hit(track[:, :2], ["fingers", "fingers"], [one_view], end=(0.5, 0.5, 1.0)) is None
    two_views = _box(1, "cup", 0.0, 0.0, 0.085, views=2)
    assert first_hit(track[:, :2], ["fingers", "fingers"], [two_views], end=(0.5, 0.5, 1.0)) is not None


def test_host_hands_the_measured_things_to_goto_as_boxes():
    def thing(id, **kw):
        base = dict(id=id, label=f"t{id}", found=True, center_xyz=(0.1, 0.2, TABLE_Z + 0.04), top=TABLE_Z + 0.08,
                    size=(0.10, 0.04, 0.08), grip_yaw=30.0, on="table", on_z=TABLE_Z, views=["head", "left_wrist"],
                    step=12)
        base.update(kw)
        return R.Thing(**base)

    things = [thing(1), thing(2, round=True, grip_yaw=None, size=(0.07, 0.07, 0.08)), thing(3, on="held by left", on_z=None),
              thing(4, found=False, center_xyz=None), thing(5, on="Thing#1", on_z=TABLE_Z + 0.08, top=TABLE_Z + 0.12,
                                                            views=["head"])]
    scene = types.SimpleNamespace(things=lambda: things)
    solids = RobotHost._solids(types.SimpleNamespace(scene=scene))
    assert [s.id for s in solids] == [1, 2, 5]
    box, ring, stacked = solids
    assert box.half == (0.05, 0.02) and box.heading == -60.0 and not box.round and box.step == 12
    assert ring.round and ring.half == (0.035, 0.035)
    assert abs(stacked.bottom - (TABLE_Z + 0.08)) < 1e-9 and stacked.views == 1 and stacked.margin == 0.02
    assert RobotHost._solids(types.SimpleNamespace(scene=None)) == []


def test_a_raised_thing_can_be_passed_beneath():
    """A Thing on a rack or shelf: a hand passing under it does not go into it, but what the hand
    carries does when it hangs down through it from above."""
    mug = _box(3, "mug", 0.0, -0.20, 0.30, bottom=TABLE_Z + 0.20)
    end = (0.30, -0.20, TABLE_Z + 0.10)
    xs = np.linspace(-0.20, 0.20, 41)
    under = np.array([[[x, -0.20, TABLE_Z + 0.10], [x, -0.20, TABLE_Z + 0.07]] for x in xs])  # grasp point, tips
    assert first_hit(under, ["fingers", "fingers"], [mug], end) is None
    hit = first_hit(under + [0.0, 0.0, 0.15], ["fingers", "fingers"], [mug], end)  # at 25 cm: into it
    assert hit is not None and hit[0] is mug and hit[1] == "fingers", hit
    carried = np.array([[[x, -0.20, TABLE_Z + 0.33], [x, -0.20, TABLE_Z + 0.30], [x, -0.20, TABLE_Z + 0.18]]
                        for x in xs])  # the hand above its top, what it holds hanging below its bottom
    hit = first_hit(carried, ["fingers", "fingers", "held"], [mug], end)
    assert hit is not None and hit[1] == "held", hit
