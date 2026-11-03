"""Speed profiles of the motion planner (motion/ik.py): v1 is R0's, v2 is faster mid-motion and places
a line's actions by cost (none past a limit), v3 also starts and stops an empty hand faster.  And the
driver's notes: a BLOCKED note names the other arm when it is pushed, a dry run warns when it is close,
an open that stops short says why, a target too low says how low it can go.  And the angles: left-out
ones keep those last asked for, the half-turn of an empty gripper only pointing down, printing folded
from 80 degrees, and a motion that asks for no other angles keeps the exact rotation (folded angles
lose the tilt)."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from hexashell.geometry import homogeneous
from hexashell.motion import ik
from hexashell.motion.chain import arm_chain
from hexashell.motion.pose import rotation_of
from hexashell.world import TABLE_Z

V1, V2 = ik.SPEEDS["v1"], ik.SPEEDS["v2"]


def _down(chain, x, y, z):
    pose = homogeneous(rotation_of(90.0, 0.0, 0.0), (x, y, TABLE_Z + z))
    sol, _ = ik.solve(chain, pose, np.zeros(6))
    assert sol.reached and sol.clear
    return sol.q, chain.grasp_pose(sol.q)


def _line(speed, start, goal, arm="left"):
    chain = arm_chain(arm)
    q0, _ = _down(chain, *start)
    q1, pose1 = _down(chain, *goal)
    path, why = ik.line_path(chain, q0, q1, pose1, speed)
    assert path is not None, why
    return chain, path, ik.actions_for(path, speed)


def _travel(chain, q_from, actions):
    """Per action: grasp-point travel (m) and the largest joint change (rad)."""
    out, q = [], q_from
    for a in actions:
        out.append((float(np.linalg.norm(chain.grasp_pose(a)[:3, 3] - chain.grasp_pose(q)[:3, 3])),
                    float(np.max(np.abs(a - q)))))
        q = a
    return out


def test_v1_is_r0():
    assert V1.uniform and not V2.uniform
    chain, path, actions = _line(V1, (-0.30, -0.20, 0.20), (-0.05, -0.30, 0.12))
    assert len(path.waypoints) - 1 == max(math.ceil(path.dist / ik.LINE_STEP_M), math.ceil(path.turn / ik.LINE_STEP_RAD))
    same = ik.to_actions(path, 0.15)
    assert len(actions) == len(same) and all(np.array_equal(a, b) for a, b in zip(actions, same))
    with pytest.raises(ValueError):
        ik.speed_named("fast")


def test_v2_long_line_is_faster_and_slow_at_its_ends():
    start, goal = (-0.40, -0.25, 0.18), (-0.06, -0.20, 0.18)  # about 34 cm across the table
    _, _, slow = _line(V1, start, goal)
    chain, path, fast = _line(V2, start, goal)
    assert len(fast) <= len(slow) - 4
    steps = _travel(chain, path.waypoints[0], fast)
    assert all(m <= 0.0205 for m, _ in steps[:2] + steps[-2:])  # the ends as in v1
    assert max(m for m, _ in steps) <= 0.0405 and max(j for _, j in steps) <= 0.22 + 1e-9
    assert np.allclose(fast[-1], path.waypoints[-1])


def test_v2_short_descent_unchanged():
    _, _, slow = _line(V1, (-0.20, -0.25, 0.14), (-0.20, -0.25, 0.06))
    chain, path, fast = _line(V2, (-0.20, -0.25, 0.14), (-0.20, -0.25, 0.06))
    assert len(fast) == len(slow)
    assert max(m for m, _ in _travel(chain, path.waypoints[0], fast)) <= 0.0205


def _r1_count(path, speed):
    """R1's number of actions for a line: each profile interval cut to the joint limit, rounded up."""
    ends = max(path.dist / speed.line_m, path.turn / speed.line_rad)
    fast = max(path.dist / speed.cruise_m, path.turn / speed.cruise_rad)
    count, prev = 0, path.waypoints[0]
    for t, cruise in ik._profile(ends, fast, speed.ramp):
        q = ik._along(path.waypoints, t)
        count += max(1, math.ceil(float(np.max(np.abs(q - prev))) / (speed.joint_cruise if cruise else speed.joint)))
        prev = q
    return count


def test_v2_line_actions_keep_every_limit_and_round_up_once():
    """Line actions are placed by cost along the waypoints: none goes past its phase's travel, turn or
    joint limit, and a move no longer takes a whole extra action for each interval that needed a
    little more than one (a carry that turns the fingers a quarter turn took 20 actions in R1)."""
    chain = arm_chain("left")
    moves = [((-0.30, -0.15, 0.15, 0.0), (-0.05, -0.15, 0.15, 90.0)),  # carry and turn
             ((-0.30, -0.15, 0.15, 0.0), (-0.20, -0.15, 0.07, 90.0)),  # turn on the way down
             ((-0.40, -0.25, 0.18, 0.0), (-0.06, -0.20, 0.18, 0.0)),  # long, straight
             ((-0.20, -0.25, 0.14, 0.0), (-0.20, -0.25, 0.06, 0.0)),  # short descent
             ((-0.20, -0.25, 0.10, 30.0), (-0.18, -0.25, 0.10, -30.0))]  # mostly a turn
    counts = []
    for (x0, y0, z0, yaw0), (x1, y1, z1, yaw1) in moves:
        start, _ = _down(chain, x0, y0, z0)
        a = homogeneous(rotation_of(90.0, yaw0, 0.0), (x0, y0, TABLE_Z + z0))
        b = homogeneous(rotation_of(90.0, yaw1, 0.0), (x1, y1, TABLE_Z + z1))
        q0, _ = ik.solve(chain, a, start)
        q1, _ = ik.solve(chain, b, q0.q)
        path, why = ik.line_path(chain, q0.q, q1.q, chain.grasp_pose(q1.q), V2)
        assert path is not None, why
        actions = ik.actions_for(path, V2)
        assert np.allclose(actions[-1], path.waypoints[-1])
        counts.append((_r1_count(path, V2), len(actions)))
        q, pose = path.waypoints[0], chain.grasp_pose(path.waypoints[0])
        for i, act in enumerate(actions):
            end = i < V2.ramp or i >= len(actions) - V2.ramp
            m, rad, joint = (V2.line_m, V2.line_rad, V2.joint) if end else (V2.cruise_m, V2.cruise_rad, V2.joint_cruise)
            there = chain.grasp_pose(act)
            # travel and turn within 0.5 %: the joints are interpolated between waypoints 1 cm apart
            assert float(np.linalg.norm(there[:3, 3] - pose[:3, 3])) <= 1.005 * m
            assert ik.angle_between(pose[:3, :3], there[:3, :3]) <= 1.005 * rad
            assert float(np.max(np.abs(act - q))) <= joint + 1e-9
            q, pose = act, there
    assert counts[0] == (20, 12)
    assert all(new <= old for old, new in counts) and sum(old - new for old, new in counts) >= 12


def test_v3_ramps_over_one_action_only_with_an_empty_hand():
    """v3 is v2, except that a straight line by an arm that holds nothing (and does not close on the
    way) starts and stops in one end action instead of two; deploy.yml chooses between them, as it
    changes how the arms accelerate."""
    V3 = ik.SPEEDS["v3"]
    assert V3 == dataclasses.replace(V2, ramp_empty=1) and V2.ramp_empty is None
    chain, path, v2 = _line(V2, (-0.40, -0.25, 0.18), (-0.06, -0.20, 0.18))
    assert len(ik.actions_for(path, V2, empty=True)) == len(v2) == len(ik.actions_for(path, V3))
    v3 = ik.actions_for(path, V3, empty=True)
    steps = _travel(chain, path.waypoints[0], v3)
    assert len(v3) < len(v2) and max(steps[0][0], steps[-1][0]) <= 0.0205 and max(m for m, _ in steps) <= 0.0405
    assert np.allclose(v3[-1], path.waypoints[-1])

    def planned(speed, floor=None, **kw):
        arms = _Arms()
        driver = arms.driver()
        driver.speed = speed
        assert driver.goto("left", (-0.40, -0.25, TABLE_Z + 0.18), pitch=90, yaw=0, roll=0).ok
        if floor is not None:  # close on something (or, at 0, on nothing)
            arms.grip_floor = floor
            driver.gripper("left", "close")
        return driver.goto("left", (-0.06, -0.20, TABLE_Z + 0.18), dry_run=True, **kw).steps

    assert planned(V3) < planned(V2) and planned(V3, floor=0.0) < planned(V2, floor=0.0)
    assert planned(V3, floor=0.4) == planned(V2, floor=0.4)  # holding
    assert planned(V3, grip="close") == planned(V2, grip="close")  # closing on the way


def test_v2_swing_ramps():
    a, b = np.zeros(6), np.array([1.4, -0.6, 0.3, 0.0, 0.2, -0.9])
    swing = ik.Path([a, b], "swing")
    slow, fast = ik.actions_for(swing, V1), ik.actions_for(swing, V2)
    assert len(fast) < len(slow)
    jumps = [float(np.max(np.abs(q - p))) for p, q in zip([a] + fast[:-1], fast)]
    assert max(jumps[:2] + jumps[-2:]) <= 0.15 + 1e-9 and max(jumps) <= 0.22 + 1e-9
    assert np.allclose(fast[-1], b)
    short = ik.Path([a, a + 0.2], "swing")
    assert len(ik.actions_for(short, V2)) == len(ik.actions_for(short, V1))


def test_descent_to_the_lowest_height_is_not_refused():
    """Pointing down, 1.5 cm above the table is the prompt's lowest grasp point; in R0 about 4 in 10
    straight descents to it were refused ("runs into the table 100% of the way")."""
    checked = 0
    for arm, sx in (("left", -1), ("right", 1)):
        chain = arm_chain(arm)
        for x in (0.05, 0.15, 0.25):
            for y in (-0.20, -0.05, 0.05):
                for yaw in (0.0, 45.0, 90.0):
                    start = homogeneous(rotation_of(90.0, yaw, 0.0), (sx * x, y, TABLE_Z + 0.10))
                    goal = homogeneous(rotation_of(90.0, yaw, 0.0), (sx * x, y, TABLE_Z + 0.015))
                    s0, _ = ik.solve(chain, start, np.zeros(6))
                    s1, _ = ik.solve(chain, goal, s0.q)
                    if not (s0.reached and s1.reached):
                        continue  # out of this arm's reach
                    assert s1.clear, (arm, x, y, yaw)
                    path, why = ik.line_path(chain, s0.q, s1.q, chain.grasp_pose(s1.q), V1)
                    assert path is not None, (arm, x, y, yaw, why)
                    checked += 1
    assert checked >= 30


def test_too_low_says_how_low_it_can_go():
    driver = _Arms().driver()
    # more than 5 mm too low (the lowest clear height pointing down is 0.780): refused
    low = driver.goto("left", (-0.15, -0.15, TABLE_Z + 0.008), pitch=90, yaw=0, roll=0)
    assert not low.moved and "fingertips into the table" in str(low), str(low)
    assert "z 0.780 (the lowest the fingertips clear the table" in str(low), str(low)
    assert driver.goto("left", (-0.15, -0.15, 0.780), pitch=90, yaw=0, roll=0).moved


def test_blocked_note_names_the_other_arm():
    from hexashell.motion.chain import link_points
    from hexashell.motion.driver import NUDGE_M, OTHER_PUSH_M, blocked_note

    chains = {arm: arm_chain(arm) for arm in ("left", "right")}
    start = {arm: np.zeros(6) for arm in chains}
    assert "touching an object or the table?" in blocked_note(chains, "left", start, start, 0.02)
    # both grasp points over the middle of the table, 4 cm apart; the right arm a little off its command
    close = {"left": _down(chains["left"], -0.02, -0.2, 0.14)[0], "right": _down(chains["right"], 0.02, -0.2, 0.14)[0]}
    nudged = {**close, "right": close["right"] + np.array([0.006, 0.0, 0.0, 0.0, 0.0, 0.0])}
    off = np.max(np.linalg.norm(link_points(chains["right"], close["right"])
                                - link_points(chains["right"], nudged["right"]), axis=1))
    assert NUDGE_M < off < OTHER_PUSH_M, off
    note = blocked_note(chains, "left", close, nudged, 0.02)
    assert note.startswith("the arm lags its command by 20 mm") and "perhaps the right arm" in note, note
    # right next to it, but exactly where it was sent: it is not what blocks the left arm
    note = blocked_note(chains, "left", close, close, 0.02)
    assert "touching an object or the table? not the right arm: it has not moved" in note, note
    held_back = {**close, "right": close["right"] + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])}
    assert "it is pushing the right arm" in blocked_note(chains, "left", close, held_back, 0.02)
    # pushed but far away (it holds something against the table): not blamed on this arm
    apart = {"left": _down(chains["left"], -0.35, -0.2, 0.14)[0], "right": close["right"]}
    assert "touching an object" in blocked_note(chains, "left", apart, {**apart, "right": held_back["right"]}, 0.02)


def test_blocked_note_names_a_measured_thing_at_the_hand():
    """In a hand-over the arms are always within NEAR_ARM_M, and in R1 3 of 5 blocks against a lock said
    "perhaps the other arm" while it had not moved: a measured Thing at the hand is named first, unless
    the other arm is pushed."""
    from hexashell.motion.driver import Solid, blocked_note

    chains = {arm: arm_chain(arm) for arm in ("left", "right")}
    close = {"left": _down(chains["left"], -0.02, -0.2, 0.14)[0], "right": _down(chains["right"], 0.02, -0.2, 0.14)[0]}
    lock = Solid(id=3, label="lock", center=(-0.02, -0.2), half=(0.03, 0.03), heading=0.0, round=False,
                 bottom=TABLE_Z, top=TABLE_Z + 0.12, views=2)
    note = blocked_note(chains, "left", close, close, 0.02, touch=lock)
    assert note == "the arm lags its command by 20 mm: something is in the way - touching Thing#3 'lock'?", note
    held_back = {**close, "right": close["right"] + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0])}
    assert "it is pushing the right arm" in blocked_note(chains, "left", close, held_back, 0.02, touch=lock)


def test_a_block_names_what_the_hand_touches():
    from hexashell.motion.driver import TOUCH_M, Solid

    def box(top, x=-0.25, id=4, label="box"):
        return Solid(id=id, label=label, center=(x, -0.15), half=(0.04, 0.03), heading=0.0, round=False,
                     bottom=TABLE_Z, top=TABLE_Z + top, views=2)

    def descend(*things):
        arms = _Floor(TABLE_Z + 0.10)  # something stops the grasp point 10 cm above the table
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.22), pitch=90, yaw=0, roll=0).ok
        move = driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.03), things=list(things))
        assert move.blocked, str(move)
        return str(move)

    # the fingertips end 1.3 cm below the grasp point, at 8.7 cm
    assert "touching Thing#4 'box'?" in descend(box(0.09), box(0.05, x=-0.05, id=5, label="cup"))
    assert "touching an object or the table?" in descend(box(0.087 - TOUCH_M - 0.005))  # too far below
    # what the hand holds is not what it touches
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.10), pitch=90, yaw=0, roll=0).ok
    around = box(0.12)
    assert driver._touching("left", [around]) is around
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding
    assert driver._touching("left", [around]) is None  # the close caught it
    driver.closes.clear()  # as if measured again since, in the hand
    assert driver._standing([around]) == [around] and driver._touching("left", [around]) is None


def test_a_shift_in_the_fingers_is_noted():
    """What the hand holds sliding or turning in the fingers changes the width it holds: a goto says so
    once that is more than HELD_SHIFT_M off the width at the close (R1: a key slid ~4 cm up the
    fingers, 1.0 to 0.6 cm wide, over four moves of 1-2 mm each, unseen), and counts on from there.
    Back within HELD_SHIFT_M of the width at the close it says nothing: it was squeezed, not moved
    (R3, hang_mugs: a mug pressed on a peg read 7.2 -> 6.5 cm and back, 20 of 44 notes)."""
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.10), pitch=90, yaw=0, roll=0).ok

    def up(floor):
        arms.grip_floor = floor  # what the fingers squeeze is this wide now
        return str(driver.goto("left", by=(0.0, 0.0, 0.02)))

    arms.grip_floor = 0.10
    assert "1.3 cm wide (holding)" in str(driver.gripper("left", "close"))
    assert "shifted" not in up(0.085)  # 1.6 mm narrower
    note = up(0.07)  # 3.3 mm narrower than at the close
    assert ("what it holds has shifted in the fingers (grip 1.3 cm wide, now 1.0 cm) - where how it sits matters"
            in note), note
    assert "shifted" not in up(0.07)
    assert "shifted" not in up(0.105)  # 3.8 mm wider again: back within 0.6 mm of the close
    assert "(grip 1.4 cm wide, now 1.0 cm)" in up(0.07)  # counted from there
    assert "(grip 1.0 cm wide, now 0.7 cm)" in up(0.04)  # sliding on
    assert "(grip 0.7 cm wide, now 1.8 cm)" in up(0.14)  # and wider than at the close
    # a new close counts from its own width; an open hand says nothing
    assert driver.gripper("left", 1.0).opening > 0.9
    assert "shifted" not in up(0.0)
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding
    assert "shifted" not in up(0.38)


class _Arms:
    """Two arms without cameras: joints go 70 % of the way to their command each action, a gripper
    at most 0.2 of its range, no further open than ``grip_cap`` (fingers blocked) and no further
    closed than ``grip_floor`` (something between the fingers); ``later`` changes ``grip_floor`` from
    a given action number on."""

    track = 0.7  # how far the joints go to their command each action

    def __init__(self):
        self.chains = {arm: arm_chain(arm) for arm in ("left", "right")}
        self.q = {arm: np.zeros(6) for arm in self.chains}
        self.grip = {arm: 1.0 for arm in self.chains}
        self.grip_cap = 1.0
        self.grip_floor = 0.0
        self.later: dict[int, float] = {}
        self.seq = 0

    def obs(self):
        import fakes

        state = {}
        for arm, q in self.q.items():
            state[f"{arm}_arm_joint_state"] = q.tolist()
            state[f"{arm}_ee_pose"] = fakes.pose_to_xyzwxyz(self.chains[arm].flange(q))
            state[f"{arm}_ee_joint_state"] = [self.grip[arm]]
        return {"state": state}

    def act(self, action):
        from hexashell.state import parse_obs

        self.grip_floor = self.later.get(self.seq + 1, self.grip_floor)
        for arm in self.q:
            self.q[arm] = self.q[arm] + self.track * (np.asarray(action[f"{arm}_arm_joint_state"]) - self.q[arm])
            goal = float(action[f"{arm}_ee_joint_state"][0])
            moved = self.grip[arm] + float(np.clip(goal - self.grip[arm], -0.2, 0.2))
            self.grip[arm] = min(self.grip_cap, max(self.grip_floor, moved))
        self.seq += 1
        return parse_obs(self.obs(), self.seq)

    def driver(self):
        from hexashell.motion.driver import Driver
        from hexashell.state import parse_obs

        return Driver(self, parse_obs(self.obs(), 0))


class _Floor(_Arms):
    """_Arms whose left grasp point stops at height ``floor`` on the way down (something under it)."""

    def __init__(self, floor):
        super().__init__()
        self.floor = floor

    def act(self, action):
        from hexashell.state import parse_obs

        before = self.q["left"].copy()
        super().act(action)
        after, chain = self.q["left"], self.chains["left"]
        if chain.grasp_pose(after)[2, 3] < self.floor <= chain.grasp_pose(before)[2, 3] + 1e-9:
            lo, hi = 0.0, 1.0  # how far along this action it gets
            for _ in range(30):
                mid = (lo + hi) / 2
                if chain.grasp_pose(before + mid * (after - before))[2, 3] >= self.floor:
                    lo = mid
                else:
                    hi = mid
            self.q["left"] = before + lo * (after - before)
        return parse_obs(self.obs(), self.seq)


def test_dry_run_warns_next_to_the_other_arm():
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("right", (0.02, -0.2, TABLE_Z + 0.14), pitch=90, yaw=0).ok
    near = driver.goto("left", (-0.02, -0.2, TABLE_Z + 0.14), pitch=90, yaw=0, dry_run=True)
    far = driver.goto("left", (-0.30, -0.2, TABLE_Z + 0.14), pitch=90, yaw=0, dry_run=True)
    assert near.reached and "comes right next to the right arm" in str(near), str(near)
    assert far.reached and "right arm" not in str(far), str(far)


def test_open_that_stops_short_says_why():
    """Near the table the fingers are pressed on it: lift a little.  Higher up (a release on a cradle or
    hook) something beside them holds them, and lifting pulled the object off (R1, 6 runs): back off."""
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.02), pitch=90, yaw=0, roll=0).ok
    closed = driver.gripper("left", "close")
    assert closed.goal == 0.0 and "note" not in str(closed)
    arms.grip_cap = 0.39  # something presses on the fingers
    short = driver.gripper("left", "open")
    assert str(short).endswith("note: opened only to 0.39: the fingers are blocked (pressed on the table or against "
                               "something beside them) - lift a little, then open again"), str(short)
    arms.grip_cap = 1.0
    assert "note" not in str(driver.gripper("left", "open"))
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.20), pitch=45, yaw=0, roll=0).ok
    driver.gripper("left", "close")
    arms.grip_cap = 0.39
    high = driver.gripper("left", "open")
    assert str(high).endswith("note: opened only to 0.39: something beside the fingers holds them - back off a little "
                              "the way the hand came in, then open again"), str(high)


def test_status_names_its_arms():
    import pickle

    from hexashell.robot.results import ArmStatus, Status

    arm = {a: ArmStatus(arm=a, at=(0.0, 0.0, 0.9), angles=(0.0, 0.0, 0.0), gripper=1.0, goal=1.0, holding=False,
                        joints=(0.0,) * 6, home_dist_m=0.0, home_turn_deg=0.0) for a in ("left", "right")}
    status = pickle.loads(pickle.dumps(Status(step=3, step_limit=10, arms=arm)))  # it travels to the cell
    assert status.left is status["left"] and status.right.arm == "right" and status.steps_left == 7


def test_left_out_angles_keep_the_asked_ones():
    """R0 filled left-out angles from the pose read back, so a twist from a contact (or a closest
    reachable pose) was carried into every later motion; now they are the angles last asked for."""
    from hexashell.motion.driver import display_angles
    from hexashell.state import parse_obs

    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.20, -0.20, TABLE_Z + 0.12), pitch=90, yaw=30, roll=0).moved
    arms.q["left"] = arms.q["left"] + np.array([0.0, 0.0, 0.0, 0.0, 0.0, math.radians(8)])  # twisted
    driver.now = parse_obs(arms.obs(), arms.seq)
    assert abs(display_angles(driver.grasp_pose("left")[:3, :3])[1] - 22.0) < 1.0
    lower = driver.goto("left", by=(0.0, 0.0, -0.03))
    assert lower.target_angles == (90.0, 30.0, 0.0) and abs(lower.angles[1] - 30.0) < 1.0, str(lower)


def test_half_turn_only_pointing_straight_down(monkeypatch):
    """An open, empty gripper may arrive with its fingers half a turn round only when pointing about
    straight down, and the yaw it arrives at is kept; R0 also turned camera hovers at pitch 60, which
    then saw the table upside down."""
    from hexashell.motion import driver as motion

    where = (-0.20, -0.20, TABLE_Z + 0.15)
    driver = _Arms().driver()
    driver.goto("left", where, pitch=90, yaw=0, roll=0)
    turned = driver.goto("left", where, pitch=90, yaw=150)
    assert "pass yaw=-30 from now on - once the gripper closes, yaw=150 is taken as yaw=-30" in str(turned), str(turned)
    assert driver.asked["left"] == (90.0, -30.0, 0.0)
    assert driver.goto("left", by=(0.0, 0.0, -0.02)).target_angles == (90.0, -30.0, 0.0)

    def hover(gate):
        monkeypatch.setattr(motion, "FLIP_PITCH", gate)
        driver = _Arms().driver()
        driver.goto("left", where, pitch=60, yaw=0, roll=0)
        return driver.goto("left", where, pitch=60, yaw=-120)

    assert "half a turn" in str(hover(0.0))  # R0's rule
    kept = hover(85.0)
    assert kept.moved and "half a turn" not in str(kept) and abs(kept.angles[2]) < 5.0, str(kept)


@pytest.mark.parametrize("pitch", [85.0, 86.0, 88.0, 90.0])
def test_after_a_half_turn_no_new_angles_keep_the_exact_rotation(pitch):
    """After a half-turn the angles print folded, (pitch, yaw + 180, 0); below pitch 90 they name a hand
    tilted the other way, 2 x (90 - pitch) degrees off.  3fe2a88 kept the folded angles, so a later
    motion that left the angles out turned the hand 9.7 degrees at pitch 85.  Now a motion that asks
    for no other angles (none, or the printed ones) keeps the exact rotation."""
    from hexashell.geometry import angle_between
    from hexashell.motion.pose import finger_flip

    where = (-0.20, -0.20, 0.915)
    exact = finger_flip(rotation_of(pitch, 150.0, 0.0))  # where the half-turn went
    for request in ({}, {"pitch": pitch}, {"pitch": pitch, "yaw": -30.0, "roll": 0.0}):
        driver = _Arms().driver()
        driver.goto("left", where, pitch=pitch, yaw=0, roll=0)
        turned = driver.goto("left", where, pitch=pitch, yaw=150)
        assert "pass yaw=-30 from now on - once the gripper closes, yaw=150 is taken as yaw=-30" in str(turned), (
            str(turned))
        before = driver.grasp_pose("left")[:3, :3]
        lower = driver.goto("left", by=(0.0, 0.0, -0.02), **request)
        after = driver.grasp_pose("left")[:3, :3]
        assert lower.moved and "half a turn" not in str(lower), str(lower)
        assert lower.target_angles == (pitch, -30.0, 0.0)
        assert math.degrees(angle_between(after, before)) < 1.0, request
        assert math.degrees(angle_between(after, exact)) < 1.0, request
        assert np.allclose(driver.aim["left"], exact, atol=1e-9), request  # the full rotation planned


def test_after_a_half_turn_new_angles_fill_in_from_the_printed_ones():
    """A motion that asks for other angles takes the ones it leaves out from those printed (the
    note's yaw), and turns to exactly those."""
    from hexashell.geometry import angle_between

    where = (-0.20, -0.20, 0.915)
    driver = _Arms().driver()
    driver.goto("left", where, pitch=85, yaw=0, roll=0)
    driver.goto("left", where, pitch=85, yaw=150)
    hover = driver.goto("left", by=(0.0, 0.0, 0.02), pitch=60)
    assert hover.target_angles == (60.0, -30.0, 0.0)
    assert np.allclose(driver.aim["left"], rotation_of(60.0, -30.0, 0.0), atol=1e-9)
    assert math.degrees(angle_between(driver.grasp_pose("left")[:3, :3], rotation_of(60.0, -30.0, 0.0))) < 1.0


def test_angles_print_folded_from_80_degrees():
    """Pointing within 10 degrees of vertical, yaw and roll turn about nearly the same axis: results
    print the fingers' heading as yaw and roll 0 (R0 folded only from 87)."""
    from hexashell.motion.driver import display_angles
    from hexashell.motion.pose import angles_of, closing_axis_heading

    assert display_angles(rotation_of(79.0, 30.0, 40.0)) == angles_of(rotation_of(79.0, 30.0, 40.0))
    for pitch in (80.0, 84.0, 89.0):
        rot = rotation_of(pitch, 30.0, 40.0)
        shown = display_angles(rot)
        assert shown[0] == pitch and shown[2] == 0.0, shown
        assert abs(closing_axis_heading(rotation_of(*shown)) - closing_axis_heading(rot)) < 0.05, shown


def test_auto_swing_far_off_the_line_is_refused():
    """R0's 'auto' swung the joints whenever the straight line failed; such a swing can carry the grasp
    point far off the line (sweeping objects there).  Now 'auto' only swings close to the line, and
    path='swing' still takes the curve, saying how far off it went."""
    target = dict(xyz=(-0.05, 0.0, TABLE_Z + 0.05), pitch=90, yaw=0, roll=0)
    refused = _Arms().driver().goto("left", **target, dry_run=True)
    assert not refused.ok and "joint swing would carry the grasp point" in str(refused), str(refused)
    assert "path='swing'" in str(refused) and "waypoint" in str(refused), str(refused)
    swung = _Arms().driver().goto("left", **target, path="swing")
    assert swung.moved and "the grasp point moves on a curve" in str(swung), str(swung)


def test_close_that_stays_open_is_not_holding():
    """R0 printed 'closed on something ~8.8 cm wide (holding)' when the fingers never moved (pressed
    on the table, mostly right after a BLOCKED move): now .holding is None, with a note."""
    arms = _Arms()
    driver = arms.driver()
    arms.grip_floor = 0.8
    jammed = driver.gripper("left", "close")
    assert jammed.holding is None and "the fingers stayed about fully open" in str(jammed), str(jammed)
    from hexashell.robot.results import ArmStatus

    status = ArmStatus(arm="left", at=(0.0, 0.0, 0.9), angles=(90.0, 0.0, 0.0), gripper=0.8, goal=0.0,
                       holding=None, joints=(0.0,) * 6, home_dist_m=0.0, home_turn_deg=0.0)
    assert "the fingers stayed about fully open (blocked?)" in str(status), str(status)
    driver.gripper("left", "open")
    arms.grip_floor = 0.4
    held = driver.gripper("left", "close")
    assert held.holding is True and "note" not in str(held), str(held)


def test_drop_on_the_way_is_noted():
    """R0's Move and Home lines said nothing when what the gripper held slipped out on the way."""
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.03), pitch=90, yaw=0, roll=0).moved
    arms.grip_floor = 0.4
    assert driver.gripper("left", "close").holding is True
    kept = driver.goto("left", by=(0.0, 0.0, 0.05))
    assert kept.moved and "slipped out" not in str(kept), str(kept)
    arms.grip_floor = 0.0  # it slips out
    lost = driver.goto("left", by=(0.0, 0.0, 0.05))
    assert "what it held has slipped out" in str(lost), str(lost)
    assert lost.dropped and not lost.ok, str(lost)
    arms.grip_floor = 0.4
    driver.gripper("left", "open")
    assert driver.gripper("left", "close").holding is True
    arms.grip_floor = 0.0
    home = driver.home("left")
    assert "left: the gripper closed fully on the way" in str(home) and home.dropped and not home.ok, str(home)
    assert driver.home("left").ok  # reported once


def test_slipped_out_only_for_what_a_close_caught():
    """R1 printed "slipped out" after 19 presses and empty closes: a closed, empty gripper that a press
    pries apart reads like a hold.  Only what a close caught (gripper() or goto(grip=...)) slips out."""
    arms = _Arms()
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.06), pitch=90, yaw=0, roll=0).moved
    assert driver.gripper("left", "close").holding is False  # shut on nothing
    arms.grip_floor = 0.3  # a press pries the closed fingers apart: it reads like a hold
    assert driver.goto("left", by=(0.0, 0.0, -0.02)).moved and driver.holding("left") is True
    arms.grip_floor = 0.0
    lifted = driver.goto("left", by=(0.0, 0.0, 0.03))
    assert lifted.ok and "slipped out" not in str(lifted), str(lifted)
    # closing on the way: shut fully, then pried apart in the same move (pressing with closed fingers)
    driver.gripper("left", "open")
    arms.later = {arms.seq + 6: 0.3}
    driver.goto("left", by=(0.0, 0.0, -0.02), grip="close")
    assert driver.holding("left") is True and not driver.carrying("left")
    arms.grip_floor = 0.0
    assert "slipped out" not in str(driver.goto("left", by=(0.0, 0.0, 0.03)))
    # closing on the way on something: a grasp
    driver.gripper("left", "open")
    arms.grip_floor = 0.4
    caught = driver.goto("left", by=(0.0, 0.0, -0.02), grip="close")
    assert caught.ok and driver.carrying("left"), str(caught)
    arms.grip_floor = 0.0
    lost = driver.goto("left", by=(0.0, 0.0, 0.03))
    assert lost.dropped and "what it held has slipped out" in str(lost), str(lost)


def test_close_right_after_a_block_on_something_wide_says_so():
    """R1: 17 of 23 closes that stayed fully open came right after a BLOCKED descent, the fingertips on
    top of something as wide as the open gripper, and agents tried the same approach 4-7 times."""
    arms = _Floor(TABLE_Z + 0.08)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.14), pitch=90, yaw=0, roll=0).ok
    assert driver.goto("left", by=(0.0, 0.0, -0.10)).blocked
    arms.grip_floor = 0.8
    wide = driver.gripper("left", "close")
    assert wide.holding is None and "rest on top of something as wide as the open gripper or wider" in str(wide)
    assert "grasp lower from the side, by a narrower part, or another way" in str(wide), str(wide)
    # pressed on the table itself: the note as before
    arms = _Floor(TABLE_Z + 0.021)
    driver = arms.driver()
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.10), pitch=90, yaw=0, roll=0).ok
    assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.015)).blocked
    arms.grip_floor = 0.8
    table = driver.gripper("left", "close")
    assert table.holding is None and "pressed on the table or against something" in str(table), str(table)
    assert "rest on top" not in str(table)


def test_blocked_while_holding_says_what_opening_does():
    """R1: after holding moves BLOCKED by 14-55 mm, 4 of 4 opens dropped what was held (after 6-9 mm, 2 of
    5 left it hanging).  Held up from below, it rests there (R4; sideways or held back:
    tests/test_r4_motion.py)."""
    note = "what it holds rests on something below it: opening here leaves it there"
    for grip_floor, by, said in ((0.4, -0.10, True), (0.4, -0.088, False), (0.0, -0.10, False)):
        arms = _Floor(TABLE_Z + 0.06)
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.15, TABLE_Z + 0.14), pitch=90, yaw=0, roll=0).ok
        arms.grip_floor = grip_floor  # 0: an empty close
        driver.gripper("left", "close")
        move = driver.goto("left", by=(0.0, 0.0, by))
        assert move.blocked and (note in str(move)) == said, (grip_floor, by, str(move))


def test_refusal_offers_another_yaw_and_rises_only_up():
    """R0 never offered another yaw (organize_table: 27 refusals said 'nothing close' while yaw 0 was
    reachable), and on the way up offered heights below the hand."""
    other_yaw = _Arms().driver().goto("left", (-0.45, -0.25, TABLE_Z + 0.15), pitch=0, yaw=-90, roll=0,
                                      dry_run=True)
    assert not other_yaw.ok and "yaw 0 at the same point and pitch" in other_yaw.alternatives, str(other_yaw)
    driver = _Arms().driver()
    assert driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.10), pitch=90, yaw=0, roll=0).moved
    rise = driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.25), pitch=90, yaw=0, roll=0, dry_run=True)
    heights = [a for a in rise.alternatives if a.startswith("z ")]
    assert heights and "about as high as it reaches there" in heights[0], str(rise)
    assert float(heights[0].split()[1]) > TABLE_Z + 0.11, str(rise)


def test_offered_heights_are_ones_goto_lands_on():
    """R1 offered "z 0.834 (... about as high as it reaches there)" for refused lifts, and the next cells
    reached 0.900-0.910: only exact solutions counted, while goto also takes near misses.  The height is
    now halved down to within 3 mm of the highest one goto lands on, and while holding it says how much
    that tilts what is held (R1: tilted lifts dropped a cup 3 of 3)."""
    for hold in (False, True):
        arms = _Arms()
        driver = arms.driver()
        assert driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.10), pitch=90, yaw=0, roll=0).moved
        if hold:
            arms.grip_floor = 0.4
            assert driver.gripper("left", "close").holding is True
        rise = driver.goto("left", (-0.30, -0.10, TABLE_Z + 0.25), pitch=90, yaw=0, roll=0, dry_run=True)
        height = next(a for a in rise.alternatives if a.startswith("z "))
        assert "about as high as it reaches there" in height, str(rise)
        assert ("it tilts what it holds by" in height) == hold, str(rise)
        z = float(height.split()[1])
        higher = homogeneous(rotation_of(90.0, 0.0, 0.0), (-0.30, -0.10, z + 0.004))
        assert driver._lands("left", higher, driver.now.arms["left"].joints) is None, z
        up = driver.goto("left", (-0.30, -0.10, z))
        assert up.ok and ("it tilts what it holds by" in str(up)) == hold, str(up)


def test_pointing_down_offers_a_lower_height_first():
    """R1: 155 of 183 refusals at pitch >= 80 offered no height, top-down grasps refused 38-49 cm above
    the table worked 17-25 cm above it, and "pitch 0" put a laptop in its slot on its side edge.  Now a
    steep refusal also tries heights down to 25 cm lower (below the hand too) and lists one first."""
    driver = _Arms().driver()
    assert driver.goto("left", (-0.30, -0.15, TABLE_Z + 0.15), pitch=90, yaw=0, roll=0).moved
    far = driver.goto("left", (-0.55, 0.0, TABLE_Z + 0.20), pitch=90, yaw=0, roll=0, dry_run=True)
    assert far.alternatives[0].startswith("z ") and "pitch 60 at the same point" in far.alternatives, str(far)
    z = float(far.alternatives[0].split()[1])
    assert z < TABLE_Z + 0.15, str(far)  # below the hand
    assert driver.goto("left", (-0.55, 0.0, z)).ok


def test_holding_low_pitch_and_lower_heights():
    """Held things slipped 10 of 31 times on moves asked for pitch 15-30 (R1, against 4.6 % of all holding
    moves): such a move says so.  And while holding, a lower height comes last and says how much lower
    the held thing goes."""
    slip = "held things slipped 1 time in 3 at pitch 15-30"
    for hold, pitch, said in ((True, 20, True), (True, 30, False), (True, 90, False), (False, 20, False)):
        arms = _Arms()
        driver = arms.driver()
        assert driver.goto("left", (-0.25, -0.20, TABLE_Z + 0.15), pitch=90, yaw=0, roll=0).moved
        arms.grip_floor = 0.4 if hold else 0.0
        driver.gripper("left", "close")
        move = driver.goto("left", (-0.25, -0.20, TABLE_Z + 0.15), pitch=pitch)
        assert move.moved and (slip in str(move)) == said, (hold, pitch, str(move))
    for hold in (False, True):
        arms = _Arms()
        driver = arms.driver()
        assert driver.goto("left", (-0.30, -0.20, TABLE_Z + 0.25), pitch=90, yaw=0, roll=0).moved
        if hold:
            arms.grip_floor = 0.4
            driver.gripper("left", "close")
        far = driver.goto("left", (-0.50, -0.10, TABLE_Z + 0.25), pitch=90, yaw=0, roll=0, dry_run=True)
        lower = [i for i, a in enumerate(far.alternatives) if a.startswith("z ")]
        assert len(lower) == 1, str(far)
        text = far.alternatives[lower[0]]
        assert float(text.split()[1]) < TABLE_Z + 0.25, str(far)
        if hold:
            assert lower[0] == len(far.alternatives) - 1 and "what it holds goes" in text, str(far)
        else:
            assert lower[0] == 0 and "what it holds" not in text, str(far)


def test_home_without_a_straight_lift_says_how_it_went():
    """R0's home note said only that it could not lift straight up (23 times, 7-13 cm above the
    table): now it says whether the swing home went lower on the way."""
    driver = _Arms().driver()
    assert driver.goto("right", (0.044, -0.059, 0.889), pitch=88, yaw=32, roll=32).moved
    home = str(driver.home("right"))
    assert "could not lift straight up first" in home and "without going lower" in home, home


def test_gentle_moves_at_the_end_pace_all_the_way():
    """R1: what the arm held slipped out in 1.00 % of the actions that moved it 3 cm or more, 0.39 % of
    the slower ones.  gentle=True is v1's pace for that one call; the driver's speed stays."""

    def carry(speed, **kw):
        arms = _Arms()
        driver = arms.driver()
        driver.speed = speed
        assert driver.goto("left", (-0.30, -0.15, TABLE_Z + 0.15), pitch=90, yaw=0, roll=0).ok
        move = driver.goto("left", by=(0.25, 0.0, 0.0), **kw)  # a 25 cm carry
        assert move.ok and driver.speed is speed, str(move)
        return move.steps

    v1, v2 = ik.SPEEDS["v1"], ik.SPEEDS["v2"]
    fast, gentle, slow = carry(v2), carry(v2, gentle=True), carry(v1)
    assert gentle == slow > fast, (fast, gentle, slow)
    assert carry(v1, gentle=True) == slow and gentle - fast == 4  # 03_api.md says so
