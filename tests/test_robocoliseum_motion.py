"""RoboColiseum Season II: the arm models, the chunked motions, the wire format and the verdict, offline.

The start joints and the reported end poses are those of recorded requests (the G2's ctek90d build in
phone_packaging, its omnipicker build, the G1, the ARX and the Agilex in pour_50ml_into_graduated_cylinder).
"""

from __future__ import annotations

import dataclasses
import math

import msgpack
import numpy as np
import pytest

from hexashell.benches.robocoliseum import budget, kinematics, link as L, mask as M, motion
from hexashell.benches.pose import shown_angles
from hexashell.benches.robocoliseum import obs as O
from hexashell.benches.robocoliseum import protocol as P
from hexashell.errors import Refused
from hexashell.geometry import angle_between

RECORDED = {
    "g2_90d": {
        "joints": [0.529873, -0.51672, -1.094525, -1.604456, -1.148206, 0.335738, -1.155124,
                   -0.519539, -0.518637, 1.084056, -1.58947, 1.153654, 0.338162, 1.192614],
        "effectors": [0.0, 0.0],
        "arm_base": [[0.935873, -0.352336, 0.000228, 0.249592], [-0.00025, -1.7e-05, 1.0, 8e-06],
                     [-0.352336, -0.935873, -0.000104, 1.112385], [0.0, 0.0, 0.0, 1.0]],
        "ends": {"left": ([0.385358, -0.080761, 0.378914], [-0.591112, 0.802591, -0.050759, 0.062109]),
                 "right": ([0.385191, -0.080451, -0.380275], [0.051077, -0.039544, 0.80047, -0.595881])},
    },
    "g2_omnipicker": {
        "joints": [0.737568, -0.716728, -1.525038, -1.534711, 0.27766, -0.938561, -0.840952,
                   -0.737677, -0.716654, 1.525251, -1.534863, -0.277506, -0.9382, 0.841131],
        "effectors": [-0.431758, -0.431796],
        "arm_base": [[0.885484, -0.464671, 8e-06, 0.189929], [-9e-06, 0.0, 1.0, -0.0],
                     [-0.464671, -0.885484, -4e-06, 1.143414], [0.0, 0.0, 0.0, 1.0]],
        "ends": {"left": ([0.436836, -0.093447, 0.401253], [0.180501, 0.932528, 0.082508, 0.301668]),
                 "right": ([0.436794, -0.093487, -0.401293], [0.301801, 0.082464, 0.932505, 0.180417])},
    },
    "g1": {
        "joints": [-1.0751, 0.6109, 0.2793, -1.2846, 0.7295, 1.4957, -0.1868,
                   1.0734, -0.6109, -0.2793, 1.2846, -0.7313, -1.4957, 0.1868],
        "effectors": [-0.785, -0.785], "waist": [0.8727, 0.42], "head": [0.0, 0.4363],
        "arm_base": [[0.64276, 0.0, 0.766067, 0.364651], [0.0, 1.0, -1e-06, 0.0],
                     [-0.766067, 1e-06, 0.64276, 1.264542], [0.0, 0.0, 0.0, 1.0]],
        "ends": {"left": ([0.343988, 0.299943, 0.039036], [0.10263, 0.704337, -0.658578, 0.244237]),
                 "right": ([0.344253, -0.302089, 0.038201], [0.705077, 0.101793, 0.24389, -0.658044])},
    },
    "arx": {
        "joints": [-0.000171, 0.959189, 0.852873, -0.58037, 0.00011, 0.004324,
                   -0.000233, 0.95919, 0.852876, -0.58037, -0.000494, 0.004267],
        "effectors": [-3.476779, -3.477815], "waist": [], "head": [],
        "arm_base": np.eye(4).tolist(),
        "ends": {"left": ([0.176738, 0.249964, 0.246024], [0.002046, 0.336636, -0.00086, 0.941632]),
                 "right": ([0.176738, -0.250031, 0.246024], [0.002131, 0.336636, -0.000595, 0.941632])},
    },
    "agilex": {
        "joints": [-0.180014, 1.350221, -0.839026, -0.000658, 2e-06, -3.9e-05,
                   0.179986, 1.350221, -0.839026, -0.000658, 2e-06, -3.9e-05],
        "effectors": [-3.499934, -3.49993], "waist": [], "head": [],
        "arm_base": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -0.15], [0.0, 0.0, 0.0, 1.0]],
        "ends": {"left": ([0.520901, 0.250091, 1.060453], [0.644804, 0.538431, 0.347431, 0.416666]),
                 "right": ([0.520396, -0.247315, 1.060453], [0.537992, 0.645171, 0.416414, 0.347734])},
    },
}
G2_ROBOTS = ("g2_90d", "g2_omnipicker")
#: the hands' start angles (pitch, yaw, roll) in the recorded requests, with the tool frames of kinematics
START_ANGLES = {"g1": {"left": (9.7, -82.8, -20.8), "right": (9.8, 82.8, 20.9)},
                "arx": {"left": (39.3, 0.0, 0.2), "right": (39.3, 0.0, 0.3)},
                "agilex": {"left": (24.3, -10.3, 0.0), "right": (24.3, 10.3, 0.0)}}


def _quat_xyzw(r: np.ndarray) -> list[float]:
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return [x, y, z, w]


def _end_pose(robot: str, joints: np.ndarray, arm_base: np.ndarray) -> dict:
    models = kinematics.arm_models(robot)
    out = {"base_link": {}, "arm_base_link": {}}
    for k, arm in enumerate(kinematics.ARMS):
        n = models[arm].n
        e_ab = models[arm].end_pose(joints[n * k:n * k + n])
        e_b = arm_base @ e_ab
        out["base_link"][f"{arm}_arm"] = {"position": list(e_b[:3, 3]), "orientation": _quat_xyzw(e_b[:3, :3])}
        out["arm_base_link"][f"{arm}_arm"] = {"position": list(e_ab[:3, 3]), "orientation": _quat_xyzw(e_ab[:3, :3])}
    return out


def _frame(robot: str) -> O.Frame:
    rec = RECORDED[robot]
    joints = np.array(rec["joints"], float)
    arm_base = np.array(rec["arm_base"], float)
    waist = np.array(rec.get("waist", np.zeros(5)), float)
    head = np.array(rec.get("head", np.zeros(3)), float)
    return O.Frame(seq=1, t=0.0, task_name="test", episode_idx=0, prompt="test", done=False, segments=[],
                   robot=robot, joints=joints, effectors=np.array(rec["effectors"], float),
                   waist=waist if waist.size else None, head=head if head.size else None, chassis=None,
                   end_pose=_end_pose(robot, joints, arm_base), arm_base=arm_base)


class FakeLink:
    """The arms follow the last row exactly (``lag``: joints that stay behind); a gripper closing on an object
    ``held`` m wide stops there."""

    def __init__(self, robot: str, *, held=None, lag=None):
        self.first = _frame(robot)
        self.robot = robot
        self.prompt = self.first.prompt
        self.command = L.start_command(self.first)
        self._frame = self.first
        self.chunks: list[list[dict]] = []
        self.held = dict(held or {})
        self.lag = dict(lag or {})
        self.over = False
        self.verdict = None

    @property
    def dispatches(self) -> int:
        return len(self.chunks)

    def latest(self) -> O.Frame:
        return self._frame

    def act(self, rows, need_depth):
        assert rows and need_depth
        self.chunks.append(rows)
        self.command = L.copy_row(rows[-1])
        f = self._frame
        joints, eff = f.joints.copy(), f.effectors.copy()
        models = kinematics.arm_models(self.robot)
        for k, arm in enumerate(kinematics.ARMS):
            n = models[arm].n
            joints[n * k:n * k + n] = np.asarray(rows[-1][arm], float) - self.lag.get(arm, 0.0)
            value = float(np.reshape(rows[-1]["grip"][arm], -1)[0])
            hand = models[arm].hand
            width = self.held.get(arm)
            if width is not None and hand.gap_m(value) < width:
                lo, hi = hand.open_value, hand.closed_value
                for _ in range(50):
                    mid = 0.5 * (lo + hi)
                    lo, hi = (mid, hi) if hand.gap_m(mid) > width else (lo, mid)
                value = lo
            eff[k] = value
        self._frame = dataclasses.replace(f, seq=f.seq + 1, joints=joints, effectors=eff,
                                          end_pose=_end_pose(self.robot, joints, f.arm_base))
        return self._frame


# -- the models ------------------------------------------------------------------------------------
@pytest.mark.parametrize("robot", sorted(RECORDED))
def test_forward_kinematics_matches_the_reported_end_poses(robot):
    rec = RECORDED[robot]
    models = kinematics.arm_models(robot)
    for k, arm in enumerate(kinematics.ARMS):
        n = models[arm].n
        got = models[arm].end_pose(np.array(rec["joints"][n * k:n * k + n]))
        want = kinematics.pose_of(*rec["ends"][arm])
        assert np.linalg.norm(got[:3, 3] - want[:3, 3]) < 1e-3
        assert math.degrees(angle_between(got[:3, :3], want[:3, :3])) < 0.2


@pytest.mark.parametrize("robot", ["g2_90d", "g1", "agilex"])
def test_arm_base_from_the_reported_poses(robot):
    f = _frame(robot)
    assert np.allclose(kinematics.arm_base_from_obs(f.end_pose), f.arm_base, atol=1e-5)


@pytest.mark.parametrize("hand", [kinematics.OMNIPICKER, kinematics.OMNIPICKER_G1, kinematics.CTEK90D,
                                  kinematics.ARX_GRIPPER, kinematics.AGILEX_GRIPPER], ids=lambda h: h.name)
def test_hand_openings(hand):
    assert hand.gap_m(hand.open_value) == pytest.approx(hand.open_gap, abs=1e-3)
    assert hand.gap_m(hand.closed_value) == pytest.approx(0.0, abs=1e-6)
    for g in (0.0, 0.3, 0.7, 1.0):
        assert hand.opening(hand.command(g)) == pytest.approx(g)
    gaps = [hand.gap_m(hand.command(g)) for g in np.linspace(0, 1, 11)]
    assert all(b > a for a, b in zip(gaps, gaps[1:]))


def test_ctek90d_pads_move_forward_as_they_close():
    h = kinematics.CTEK90D
    assert h.forward_m(h.open_value) == pytest.approx(0.0, abs=1e-9)
    assert 0.015 < h.forward_m(h.closed_value) < 0.02
    assert kinematics.OMNIPICKER.forward_m(kinematics.OMNIPICKER.closed_value) == 0.0


def test_start_angles_near_roll_zero_on_both_g2_builds():
    for robot in G2_ROBOTS:
        d = motion.RobotDriver(FakeLink(robot))
        for arm in kinematics.ARMS:
            assert abs(d.start_angles[arm][2]) < 10.0, (robot, arm, d.start_angles[arm])


@pytest.mark.parametrize("robot", sorted(START_ANGLES))
def test_start_angles_of_the_other_robots(robot):
    f = _frame(robot)
    models = kinematics.arm_models(robot)
    for arm, want in START_ANGLES[robot].items():
        got = shown_angles(models[arm].tool_rotation(f.end_link(arm)[:3, :3]))
        assert np.allclose(got, want, atol=1.0), (robot, arm, got)


def test_reach_is_about_the_stretched_arm():
    # the farthest the grasp point gets from the shoulder (joint samples: G1 0.826, ARX 0.800 with
    # the centre at 150 mm, Agilex 0.766 with it at 140 mm)
    assert kinematics.arm_models("g1")["right"].reach == pytest.approx(0.836, abs=0.01)
    assert kinematics.arm_models("arx")["right"].reach == pytest.approx(0.79, abs=0.015)
    assert kinematics.arm_models("agilex")["right"].reach == pytest.approx(0.75, abs=0.015)
    g2 = kinematics.arm_models("g2_90d")["right"]
    assert 0.86 < g2.reach < 0.91


# -- pacing ----------------------------------------------------------------------------------------
def test_profile_ramps_and_ends_at_the_target():
    rows = motion.profile([10.0, 10.0, 10.0])
    assert len(rows) == math.ceil(30.0 + motion.RAMP_ROWS)
    assert rows[-1] == (2, 1.0)
    done = [i + f for i, f in rows]
    assert all(b >= a - 1e-9 for a, b in zip(done, done[1:]))
    short = motion.profile([1.0])
    assert len(short) == math.ceil(2.0 * math.sqrt(motion.RAMP_ROWS)) and short[-1] == (0, 1.0)
    assert motion.profile([0.0]) == []


def test_grip_rows_scale_with_the_stroke_and_the_physics_rate():
    h = kinematics.CTEK90D
    # at 120 Hz (a row is 2 physics steps, 16.7 ms): 40 rows for a full stroke, at least 10
    assert motion.grip_rows(h, h.open_value, h.closed_value) == 40
    assert motion.grip_rows(h, h.command(0.5), h.closed_value) == 20
    assert motion.grip_rows(h, h.command(0.95), h.open_value) == 10
    assert motion.grip_rows(h, h.open_value, h.closed_value, 2.0 / 240) == 80
    assert motion.rows_of_time(motion.GRIP_S, 2.0 / 60) == 20


def _follow(driver, arm, rows, start, tau=None, row_s=None):
    """The joints of an arm after each row, following the rows' targets like the simulator's drives (first
    order, tau; each target held a row of row_s), from ``start``."""
    m = driver.models[arm]
    a = math.exp(-(row_s or driver.row_s) / (tau or m.tau_s))
    y = np.asarray(start, float)
    out = []
    for r in rows:
        y = a * y + (1.0 - a) * np.asarray(r[arm], float)
        out.append(y.copy())
    return out


@pytest.mark.parametrize("robot,hz", [("g1", 240), ("g1", 120), ("g2_omnipicker", 120), ("g2_omnipicker", 240),
                                      ("arx", 240), ("agilex", 120)])
def test_the_rows_lead_the_path_so_the_joints_keep_to_it(robot, hz):
    fake = FakeLink(robot)
    d = motion.RobotDriver(fake, physics_hz=hz)
    assert d.row_s == pytest.approx(2.0 / hz)
    q0 = fake.first.arm_joints("right")
    m = d.goto("right", by=(0.08, 0.05, -0.10))
    assert m.reached and fake.dispatches == 1
    rows = fake.chunks[0]
    model = d.models["right"]
    q_end = np.asarray(rows[-1]["right"], float)
    # the path's end is held, without lead, by the last rows
    assert all(np.allclose(r["right"], q_end) for r in rows[-motion.END_ROWS:])
    # the drives keep to the path: within a few thousandths of a radian of the end after the chunk, never past it
    got = _follow(d, "right", rows, q0)
    assert float(np.max(np.abs(got[-1] - q_end))) < 2e-3
    span = np.abs(q_end - q0)
    assert np.all(np.abs(got[-1] - q0) <= span + 2e-3)
    # each joint within its share of its velocity limit: the start of a row is its fastest
    path = [np.asarray(q0, float)] + got
    steps = np.abs(np.diff(np.array(path), axis=0))
    assert np.all(steps <= d.row_cap["right"] + 1e-4)
    peak = steps / (model.tau_s * (1.0 - math.exp(-d.row_s / model.tau_s)))
    assert float(np.max(peak / model.velocity)) <= motion.PACE_SHARE + 1e-3
    # the path itself as the targets: the joints would lag behind it, most at the end of the move
    n = len(rows) - motion.END_ROWS
    lagged = _follow(d, "right", [{"right": q} for q in got[:n]], q0)
    assert float(np.max(np.abs(lagged[-1] - got[n - 1]))) > 5 * float(np.max(np.abs(got[n - 1] - got[n - 1])) + 1e-3)


def test_a_slower_physics_rate_takes_fewer_rows_for_the_same_move():
    rows = {}
    for hz in (240, 120, 60):
        fake = FakeLink("g2_omnipicker")
        d = motion.RobotDriver(fake, physics_hz=hz)
        d.goto("right", by=(0.0, 0.0, -0.10))
        rows[hz] = len(fake.chunks[0])
    assert rows[240] > 1.6 * rows[120] and rows[120] > 1.6 * rows[60]


# -- the wire --------------------------------------------------------------------------------------
def test_pack_writes_one_chunk_and_the_conclusion():
    f = _frame("g2_90d")
    row = L.start_command(f)
    reply = msgpack.unpackb(L.pack([row, row], True), raw=False)["result"]
    assert len(reply["left_arm"]["values"]) == 2 and reply["left_arm"]["kind"] == "JOINT_ABS"
    assert reply["left_effector"] == [[0.0], [0.0]] and reply["need_depth"] is True
    assert "episode_done" not in reply and "waist" not in reply
    done = msgpack.unpackb(L.pack([row] * L.CONCLUDE_ROWS, False, episode_done=True), raw=False)["result"]
    assert done["episode_done"] is True and len(done["right_arm"]["values"]) == L.CONCLUDE_ROWS


def test_verdict_of_reads_the_official_score():
    entry = {"policy_dispatch_count": 9, "duration": 41.5, "task_instruction": "insert the block",
             "result": {"code": 0, "msg": "ok", "step": 2, "scores": {
                 "STEPS": [{"step": 0, "name": "PickUpOnGripper", "score": 1.0}, {"step": 1, "name": "InBBox", "score": 0.0}],
                 "E2E": 0.0, "POLICY_BUDGET_WEIGHTED": 0.16,
                 "POLICY_BUDGET_STEPS": [
                     {"name": "PickUpOnGripper", "score": 1.0, "step_out": 600, "points": 20, "earned_points": 20,
                      "baseline": 16, "count": 5, "over_budget": False, "factor": 1.0},
                     {"name": "InBBox", "score": 0.0, "step_out": 900, "points": 80, "earned_points": 0,
                      "baseline": 24, "count": 4, "over_budget": False, "factor": 1.0}]}}}
    v = L.verdict_of(entry)
    assert v["valid"] and not v["success"] and v["score"] == 0.16 and v["progress"] == 0.2
    assert v["dispatches"] == 9 and [s["name"] for s in v["sub_steps"]] == ["PickUpOnGripper", "InBBox"]
    assert budget.baseline(600) == 16 and budget.baseline(900) == 24


# -- motions ---------------------------------------------------------------------------------------
def test_goto_is_one_chunk_and_reaches():
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    m = d.goto("right", by=(0.0, 0.0, -0.05))
    assert m.moved and m.reached and not m.blocked and m.path == "line"
    assert fake.dispatches == 1 and m.steps == len(fake.chunks[0]) == d.rows
    assert m.err_mm < 2.0
    # the hand's path ramps: its first steps are shorter than the middle ones, none faster than HAND_M_S
    model = d.models["right"]
    q0 = fake.first.arm_joints("right")
    pts = [model.grasp_pose(q)[:3, 3] for q in [q0] + _follow(d, "right", fake.chunks[0], q0)]
    steps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert steps[0] < steps.max() and steps.max() <= motion.HAND_M_S * d.row_s + 1e-3


def test_a_refused_goto_sends_nothing():
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    m = d.goto("right", (2.0, -0.2, 1.0))
    assert not m.moved and fake.dispatches == 0
    assert "beyond the arm's reach" in " ".join(m.notes)


def test_a_tilted_hand_out_of_reach_is_offered_the_other_side():
    # insert_coin_into_money_box learn L05 t12: pitch 45 at the yaw the fingers had turned to did not reach;
    # tilted from the other side (the same line of the fingers seen from above) it did
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    m = d.goto("left", (0.722, 0.319, 0.855), pitch=45, yaw=-124, dry_run=True)
    assert not m.reached and m.alternatives[0].startswith("yaw 56 at the same point and pitch")
    assert d.goto("left", (0.722, 0.319, 0.855), pitch=45, yaw=56, dry_run=True).reached


def test_a_pose_without_a_line_is_offered_yaws_that_have_one(monkeypatch):
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    model = d.models["right"]
    asked = (90.0, 0.0, 0.0)
    blocked = [model.end_rotation(motion.rotation_of(*a)) for a in (asked, motion.half_turn(asked))]
    line = d._line

    def no_line_the_way_asked(arm, cur, target, q0, arm_base, **kw):
        if any(np.allclose(target[:3, :3], r, atol=1e-9) for r in blocked):
            return None
        return line(arm, cur, target, q0, arm_base, **kw)

    monkeypatch.setattr(d, "_line", no_line_the_way_asked)
    monkeypatch.setattr(d, "_off_line", lambda *a, **kw: (1.0, 0.0))  # and no swing near the line either
    m = d.goto("right", (0.60, -0.20, 0.95), pitch=90, yaw=0, dry_run=True)
    assert not m.reached and fake.dispatches == 0
    assert m.alternatives and all(a.endswith("at the same point (a straight line leads there)") for a in m.alternatives)
    assert m.alternatives[0].startswith(("yaw 30 ", "yaw -30 "))
    assert len(m.alternatives) <= motion.LINE_ALTERNATIVES


def test_yaws_offered_to_a_holding_hand_warn_of_the_turn(monkeypatch):
    # plug_charging_cable_into_phone learn L15 t14: the yaw offered for a refused lift turned the hand about 60
    # degrees on the way and the plug fell out (a 30-degree one kept it, L10 t16)
    fake = FakeLink("g2_90d", held={"right": 0.03})
    d = motion.RobotDriver(fake)
    assert d.gripper("right", "close").holding is True
    model = d.models["right"]
    asked = (90.0, 0.0, 0.0)
    blocked = model.end_rotation(motion.rotation_of(*asked))
    line = d._line

    def no_line_the_way_asked(arm, cur, target, q0, arm_base, **kw):
        if np.allclose(target[:3, :3], blocked, atol=1e-9):
            return None
        return line(arm, cur, target, q0, arm_base, **kw)

    monkeypatch.setattr(d, "_line", no_line_the_way_asked)
    monkeypatch.setattr(d, "_off_line", lambda *a, **kw: (1.0, 0.0))
    m = d.goto("right", (0.60, -0.20, 0.95), pitch=90, yaw=0, dry_run=True)
    assert not m.reached and m.alternatives
    for a in m.alternatives:
        assert "a straight line leads there; the hand turns" in a and "shake out what the gripper holds" in a


def test_dry_run_sends_nothing():
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    m = d.goto("left", by=(0.05, 0.0, -0.05), dry_run=True)
    assert m.dry_run and m.reached and m.steps > 0 and fake.dispatches == 0


def test_the_move_after_a_dry_run_follows_its_plan():
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    assert all(np.array_equal(a, b) for a, b in zip(d._random_seeds("left", 3), d._random_seeds("left", 3)))
    dry = d.goto("left", (0.60, 0.25, 0.95), pitch=60, yaw=-30, dry_run=True)
    assert dry.reached and "left" in d._dry

    def no_planning(*args, **kwargs):
        raise AssertionError("planned again")

    d._plan, d._solve = no_planning, no_planning
    m = d.goto("left", (0.60, 0.25, 0.95), pitch=60, yaw=-30)
    assert m.moved and m.steps == dry.steps and not d._dry
    with pytest.raises(AssertionError, match="planned again"):
        d.goto("left", (0.60, 0.25, 0.95), pitch=60, yaw=-30, dry_run=True)


def test_gripper_close_on_an_object_holds_it():
    fake = FakeLink("g2_90d", held={"right": 0.03})
    d = motion.RobotDriver(fake)
    g = d.gripper("right", "close")
    assert g.holding is True and g.width_m == pytest.approx(0.03, abs=2e-3)
    assert g.steps == motion.rows_of_time(motion.GRIP_S, d.row_s) == 40 and fake.dispatches == 1
    empty = d.gripper("left", "close")
    assert empty.holding is False


def test_a_hold_only_millimetres_wide_gets_a_note():
    d = motion.RobotDriver(FakeLink("g2_90d", held={"right": 0.004, "left": 0.03}))
    thin = d.gripper("right", "close")
    assert thin.holding is True and any("on top of it" in n for n in thin.notes)
    wide = d.gripper("left", "close")
    assert wide.holding is True and not wide.notes


def test_the_slow_waist_is_held_until_it_gets_there():
    chunks = {}
    for robot in ("g2_90d", "g2_omnipicker"):
        fake = FakeLink(robot)
        d = motion.RobotDriver(fake)
        d.waist(by=20)
        chunks[robot] = len(fake.chunks[-1])
    turn = math.radians(20.0)
    held = motion.rows_of_time(motion.SLOW_WAIST_SETTLE_S + motion.SLOW_WAIST_SETTLE_S_PER_RAD * turn, d.row_s)
    assert chunks["g2_90d"] - chunks["g2_omnipicker"] == held - motion.rows_of_time(motion.WAIST_SETTLE_S, d.row_s)
    assert chunks["g2_90d"] >= 75  # the slow waist read 97-100 % of a 20-degree turn about 1.4 s after it was sent


def test_blocked_when_the_arm_stays_behind():
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    fake.lag = {"right": np.array([0.0, 0.08, 0.0, 0.0, 0.0, 0.0, 0.0])}
    m = d.goto("right", by=(0.10, 0.0, 0.0))
    assert m.moved and m.blocked and m.halt == "Blocked" and m.lag_mm > 20.0
    pressed = d.goto("right", by=(0.02, 0.0, 0.0), contact=True)
    assert not pressed.blocked


def test_a_contact_push_stopped_short_says_so():
    # insert_ethernet_cable learn L10 t11, t14: pushes the socket stopped 11-21 mm short read as plain "short of
    # target", which the API text called "without pressing on anything"
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    fake.lag = {"right": np.array([0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0])}
    m = d.goto("right", by=(0.02, 0.0, 0.0), contact=True)
    assert not m.blocked and 1000 * motion.CONTACT_STOP_M < m.lag_mm < 1000 * motion.LAG_M
    assert any("something stopped it" in n for n in m.notes)
    fake.lag = {}
    free = d.goto("right", by=(0.02, 0.0, 0.0), contact=True)
    assert not any("stopped it" in n for n in free.notes)


def test_dropped_when_the_fingers_close_further():
    fake = FakeLink("g2_omnipicker", held={"left": 0.04})
    d = motion.RobotDriver(fake)
    assert d.gripper("left", "close").holding
    fake.held = {}
    m = d.goto("left", by=(0.0, 0.0, 0.05))
    assert m.dropped and m.halt == "Dropped" and d.holding("left") is False


def test_home_wait_and_waist_are_one_chunk_each():
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    d.goto("left", by=(0.05, 0.0, -0.08))
    n = fake.dispatches
    h = d.home()
    assert fake.dispatches == n + 1 and all(v[0] < 2.0 for v in h.arms.values())
    assert d.wait(25) == 25 and len(fake.chunks[-1]) == 25
    text = d.waist(by=10)
    assert "Waist" in text and fake.dispatches == n + 3
    assert all(r["waist"] is not None for r in fake.chunks[-1])
    assert fake.chunks[-1][-1]["waist"][motion.WAIST_YAW] == pytest.approx(math.radians(10.0), abs=1e-6)


def test_both_hands_move_in_one_chunk():
    fake = FakeLink("g2_omnipicker")
    d = motion.RobotDriver(fake)
    moves = d.goto("both", by=(0.0, 0.0, 0.04))
    assert len(moves) == 2 and all(m.reached for m in moves) and fake.dispatches == 1


@pytest.mark.parametrize("robot", ["arx", "agilex"])
def test_arx_and_agilex_move_both_arms_without_a_waist(robot):
    fake = FakeLink(robot, held={"left": 0.03})
    d = motion.RobotDriver(fake)
    assert d.arms == ("left", "right") and not d.turns_waist
    with pytest.raises(Refused, match="does not turn"):
        d.waist(by=10)
    m = d.goto("left", by=(0.0, 0.0, -0.04))
    assert m.moved and m.reached and not m.blocked and fake.dispatches == 1 and m.err_mm < 2.0
    right = fake.first.arm_joints("right")
    assert all(np.allclose(r["right"], right) for r in fake.chunks[0])
    reply = msgpack.unpackb(L.pack(fake.chunks[0], True), raw=False)["result"]
    assert len(reply["left_arm"]["values"][0]) == 6 and len(reply["right_arm"]["values"][0]) == 6
    assert "waist" not in reply
    g = d.gripper("left", "close")
    assert g.holding is True and g.width_m == pytest.approx(0.03, abs=2e-3)
    moves = d.goto("both", by=(0.0, 0.0, 0.03))
    assert all(mv.reached for mv in moves)
    h = d.home()
    assert set(h.arms) == {"left", "right"} and set(d.start_info()) == {"left", "right"}


def test_a_robot_that_moves_one_arm_refuses_the_other(monkeypatch):
    # no Season II robot moves one arm only; the driver and the card still allow it (a robot whose other arm
    # turns out not to move)
    monkeypatch.setitem(P.ROBOTS, "arx", dataclasses.replace(P.ROBOTS["arx"], arms=("right",)))
    fake = FakeLink("arx")
    d = motion.RobotDriver(fake)
    assert d.arms == ("right",)
    for call in (lambda: d.goto("left", by=(0.0, 0.0, 0.02)), lambda: d.goto("both", by=(0.0, 0.0, 0.02)),
                 lambda: d.gripper("left", "close"), lambda: d.gripper("both", "close"), lambda: d.home("left")):
        with pytest.raises(Refused, match=r"moves only its right arm \(the left one"):
            call()
    with pytest.raises(Refused, match="must be 'right'"):
        d.gripper("middle", "close")
    m = d.goto("right", by=(0.0, 0.0, -0.04))
    assert m.reached and all(np.allclose(r["left"], fake.first.arm_joints("left")) for r in fake.chunks[0])
    assert set(d.home().arms) == {"right"} and set(d.start_info()) == {"right"}
    from hexashell.benches.robocoliseum import session

    lines = session.robot_lines(fake.first, d)
    assert any(line.startswith("- Shoulder of the right arm") for line in lines)


def test_g1_moves_both_arms_and_keeps_its_waist():
    fake = FakeLink("g1")
    d = motion.RobotDriver(fake)
    assert d.arms == ("left", "right") and not d.turns_waist
    m = d.goto("right", by=(0.03, 0.0, -0.03))
    assert m.moved and m.reached and m.err_mm < 2.0
    with pytest.raises(Refused, match="does not turn"):
        d.waist(by=10)
    rows = fake.chunks[-1]
    assert all(r.get("waist") is None for r in rows)
    reply = msgpack.unpackb(L.pack(rows, True), raw=False)["result"]
    assert "waist" not in reply and len(reply["right_arm"]["values"][0]) == 7
    moves = d.goto("both", by=(0.0, 0.0, 0.03))
    assert all(mv.reached for mv in moves)
    for robot in ("g1", "g2_omnipicker"):  # two arms: 'both' is refused only where a call does not take it
        with pytest.raises(Refused, match="'left' or 'right'"):
            motion.RobotDriver(FakeLink(robot)).gripper("both", "close")


# -- the robot's own pixels ------------------------------------------------------------------------
@pytest.mark.parametrize("robot", sorted(RECORDED))
def test_mask_marks_the_gripper_but_not_the_gap_between_the_fingers(robot):
    f = _frame(robot)
    arms = M.frame_arms(f)
    shape = arms["right"]
    tool = shape.end_link[:3, :3] @ shape.model.tool_in_end
    origin = shape.end_link[:3, 3]

    def at(approach, across, pads=0.0):
        return origin + tool @ np.array([approach, across, pads])

    hand = shape.model.hand
    gap = hand.gap_m(shape.reading)
    body = at(0.02, 0.0)
    finger = at(hand.grasp_reach, gap / 2.0 + 0.006)
    between = at(hand.grasp_reach, 0.0)
    beyond = at(hand.tip_reach + 0.05, 0.0)
    inside = M.inside_arm(np.array([body, finger, between, beyond]), shape)
    assert inside.tolist() == [True, True, False, False]


# -- the task card's robot lines --------------------------------------------------------------------
@pytest.mark.parametrize("robot", sorted(RECORDED))
def test_card_describes_each_robot(robot):
    from hexashell.benches.robocoliseum import session

    fake = FakeLink(robot)
    d = motion.RobotDriver(fake)
    lines = session.robot_lines(fake.first, d)
    text = "\n".join(lines)
    assert lines[0].startswith("Robot: ") and "{" not in text
    shoulders = [line for line in lines if line.startswith("- Shoulder")]
    assert len(shoulders) == 1
    assert ("left (" in shoulders[0]) == (len(d.arms) == 2)
    assert ("waist facing forward" in shoulders[0]) == d.turns_waist
    assert ("fixed base" in lines[0]) == (robot in ("arx", "agilex")) and "both" not in lines[0]
    assert any(line.startswith("- Heights") for line in lines) == (robot in ("arx", "agilex"))
    assert "- Time: one step (one command row) is about 17 ms of simulated time in this scene (its physics runs at " \
           "120 Hz)." in lines


# -- the chassis (the WBC board) --------------------------------------------------------------------
def test_drive_is_one_chunk_with_the_pose_target_in_every_row():
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake, mobile=True)
    start = L.copy_row(fake.command)
    text = d.drive(forward=0.3)
    rows = fake.chunks[-1]
    assert fake.dispatches == 1 and "Drove: forward 0.30 m" in text and "stale" in text
    assert len(rows) == motion.rows_of_time(0.3 / motion.DRIVE_M_S, motion.DRIVE_ROW_S) == 60
    assert all(np.allclose(r["wheels"], [0.3, 0.0, 0.0]) for r in rows)
    assert all(np.allclose(r["left"], start["left"]) and np.allclose(r["right"], start["right"]) for r in rows)
    assert "wheels" not in fake.command  # never carried into the next chunk
    assert d.drives == 1 and d.drive_end == d.rows and np.allclose(d.moved_by[:3, 3], [0.3, 0.0, 0.0])
    d.drive(turn=-30)
    assert len(fake.chunks[-1]) == motion.rows_of_time(motion.DRIVE_MIN_S, motion.DRIVE_ROW_S)
    assert np.allclose(fake.chunks[-1][0]["wheels"], [0.0, 0.0, math.radians(-30.0)])
    assert math.degrees(math.atan2(d.moved_by[1, 0], d.moved_by[0, 0])) == pytest.approx(-30.0)


def test_drive_refuses_a_fixed_base_and_odd_targets():
    fake = FakeLink("g2_90d")
    with pytest.raises(Refused, match="does not drive"):
        motion.RobotDriver(fake).drive(forward=0.2)
    d = motion.RobotDriver(fake, mobile=True)
    for kwargs, words in ((dict(forward=1.2, left=1.2), "at most"), (dict(turn=200), "at most"),
                          (dict(), "nothing to drive"), (dict(forward=True), "number"),
                          (dict(left="far"), "number"), (dict(turn=float("nan")), "number")):
        with pytest.raises(Refused, match=words):
            d.drive(**kwargs)
    assert fake.dispatches == 0


def test_a_hand_pushed_on_a_drive_gets_a_note():
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake, mobile=True)
    fake.lag = {"right": np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])}
    text = d.drive(forward=0.5)
    assert "right hand was pushed" in text and "left hand" not in text


def test_pack_sends_the_chassis_target_only_with_wheels():
    f = _frame("g2_90d")
    row = L.start_command(f)
    assert "wheels" not in msgpack.unpackb(L.pack([row, row], True), raw=False)["result"]
    moving = dict(L.copy_row(row), wheels=np.array([0.3, -0.1, 0.2]))
    reply = msgpack.unpackb(L.pack([moving, moving], True), raw=False)["result"]
    assert reply["wheels"]["kind"] == "POSE_DELTA"
    assert np.allclose(reply["wheels"]["values"], [[0.3, -0.1, 0.2]] * 2)
    with pytest.raises(ValueError, match="all of its rows"):
        L.pack([moving, row], True)


def test_a_drive_chunk_reaches_the_reply_with_its_wheels_and_only_that_chunk():
    """act() once copied the rows without "wheels", so no drive reached the simulator."""
    import threading
    first = _frame("g2_90d")
    ep = L.EpisodeLink(0, first, hold_s=60)
    moving = dict(L.copy_row(ep.command), wheels=np.array([0.3, 0.0, 0.0]))
    t = threading.Thread(target=ep.act, args=([moving] * 3, False), daemon=True)
    t.start()
    rows, need, kind = ep.serve(dataclasses.replace(first, seq=2), spent_s=0.0)
    assert kind == "action" and len(rows) == 3
    reply = msgpack.unpackb(L.pack(rows, need), raw=False)["result"]
    assert np.allclose(reply["wheels"]["values"], [[0.3, 0.0, 0.0]] * 3)
    ep.serve(dataclasses.replace(first, seq=3), spent_s=0.0)  # the request after the chunk releases act()
    t.join(5)
    assert not t.is_alive()
    assert "wheels" not in ep.command
    rows, need, kind = ep.serve(dataclasses.replace(first, seq=4), spent_s=L.SIM_REPLY_S)
    assert kind == "hold" and "wheels" not in msgpack.unpackb(L.pack(rows, need), raw=False)["result"]


class _FakeScene:
    """What the host touches of a DepthScene on a drive."""

    def __init__(self):
        self.records, self.carried, self.masks = {}, {}, {1: None}

    def bind(self, views, hands=None):
        self.hands = hands


def test_a_drive_rebases_what_perception_knows_and_stale_things_are_refused():
    from hexashell.benches.robocoliseum.host import ColiseumHost
    from hexashell.robot import results as R
    from hexashell.vision.rgbd import _Record
    fake = FakeLink("g2_90d")
    scene = _FakeScene()
    host = ColiseumHost(fake, scene=scene, mobile=True)
    cup = R.Thing(id=1, label="cup", found=True, center_xyz=(1.0, 0.5, 0.8), top=0.85, size=(0.08, 0.08, 0.1),
                  grip_yaw=80.0, step=0)
    held = R.Thing(id=2, label="block", found=True, center_xyz=(0.5, 0.0, 0.9), top=0.92, size=(0.04, 0.04, 0.04),
                   step=0)
    scene.records = {1: _Record(cup, np.array([[1.0, 0.5, 0.8]]), "cup"),
                     2: _Record(held, np.array([[0.5, 0.0, 0.9]]), "block")}
    scene.carried = {2: np.eye(4)}
    host.dispatch("drive", (), dict(forward=0.4, turn=90))
    # the base went 0.4 m forward and turned left a quarter turn: the cup, 0.6 m ahead and 0.5 m left of
    # where the base is now, lies 0.5 m to its right and 0.6 m ahead
    assert scene.records[1].thing.center_xyz == pytest.approx((0.5, -0.6, 0.8), abs=1e-4)
    assert np.allclose(scene.records[1].points, [[0.5, -0.6, 0.8]])
    assert scene.records[1].thing.grip_yaw == pytest.approx(-10.0)
    assert scene.records[2].thing.center_xyz == (0.5, 0.0, 0.9)  # held: it rode along
    assert scene.masks == {} and host.motions == 1
    with pytest.raises(Refused, match="before the robot drove"):
        host._goto("left", cup)
    fresh = R.Thing(id=3, label="cup", found=True, center_xyz=(0.5, -0.6, 0.8), step=host.driver.rows)
    assert not host._before_drive(fresh)
    # a pixel of a picture seen before the drive is warned of, one of a picture seen after it is not
    host.shown["head"], host.shown_drives["head"] = np.eye(4), 0
    view = type("View", (), {"camera": type("Camera", (), {"pose": np.eye(4)})()})()
    assert "drove since" in host._moved_since_shown("head", view)
    host.looked, host.looked_drives = {"head": np.eye(4)}, {"head": host.driver.drives}
    host.pictures_shown()
    assert host._moved_since_shown("head", view) is None


# -- the body and the head (the WBC board's G2_crsB) ---------------------------------------------------
from hexashell.benches.robocoliseum import body as B  # noqa: E402
from hexashell.benches.robocoliseum import session as S  # noqa: E402

#: a recorded probe (pick_up_trash i0): body joints read and arm_base_link reported with them
PROBE_BODY = [
    ([-1.0019625, 2.3416076, -0.1009158, -5.1e-06, -2.673e-04],
     [[0.326, -0.94537, 8e-05, 0.60109], [-0.00027, -1e-05, 1.0, 0.0], [-0.94537, -0.326, -0.00025, 0.57411]]),
    ([-1.0020880, 2.3413801, -0.4981698, -3.44e-05, -2.724e-04],
     [[0.6666, -0.74542, 0.00016, 0.50956], [-0.00027, -3e-05, 1.0, 2e-05], [-0.74542, -0.6666, -0.00023, 0.73003]]),
    ([-1.0026373, 2.3412926, -1.2461183, -8.01e-05, -2.471e-04],
     [[0.99571, -0.09254, 0.00024, 0.2107], [-0.00025, -8e-05, 1.0, 4e-05], [-0.09254, -0.99571, -0.0001, 0.88067]]),
]
#: pick_up_trash_wbc's start: body joints and head
TRASH_BODY = np.array([-0.81594, 1.51390, -0.14662, 0.0, 0.0])
TRASH_HEAD = np.array([0.0, 0.0, 0.45138])


class BodyLink(FakeLink):
    """FakeLink with a G2_crsB body: the last row's waist and head are where they end, arm_base_link with them."""

    def __init__(self, **kw):
        super().__init__("g2_90d", **kw)
        f = self.first
        ab = B.arm_base(TRASH_BODY)
        self.first = dataclasses.replace(f, waist=TRASH_BODY.copy(), head=TRASH_HEAD.copy(), arm_base=ab,
                                         end_pose=_end_pose("g2_90d", f.joints, ab))
        self.command = L.start_command(self.first)
        self._frame = self.first

    def act(self, rows, need_depth):
        frame = super().act(rows, need_depth)
        last = rows[-1]
        waist = frame.waist if last.get("waist") is None else np.asarray(last["waist"], float)
        head = frame.head if last.get("head") is None else np.asarray(last["head"], float)
        ab = B.arm_base(waist)
        self._frame = dataclasses.replace(frame, waist=waist, head=head, arm_base=ab,
                                          end_pose=_end_pose(self.robot, frame.joints, ab))
        return self._frame


def _body_driver(**kw):
    fake = BodyLink(**kw)
    return fake, motion.RobotDriver(fake, mobile=True, physics_hz=120)


def test_body_forward_kinematics_matches_the_probe():
    for q, ab in PROBE_BODY:
        got = B.arm_base(q)
        want = np.array(ab + [[0.0, 0.0, 0.0, 1.0]])
        assert np.linalg.norm(got[:3, 3] - want[:3, 3]) < 1e-3
        assert math.degrees(angle_between(got[:3, :3], want[:3, :3])) < 0.2


def test_body_solve_finds_the_posture_asked_within_the_limits():
    for lean, z in ((19.2, 1.245), (45.0, 0.8), (70.0, 0.575), (5.0, 0.882), (0.0, 1.0)):
        q = B.solve(math.radians(lean), z, TRASH_BODY)
        assert q is not None and B.within(q)
        got = B.posture(q)
        assert abs(math.degrees(got.lean) - lean) < 1e-6 and abs(got.shoulder_z - z) < 1e-6
        assert B.HIP_X[0] - 1e-9 <= got.hip[0] <= B.HIP_X[1] + 1e-9 and got.hip[1] >= B.HIP_MIN_Z - 1e-9
    assert B.solve(0.0, 0.85, TRASH_BODY) is None  # upright, the shoulders go no lower than about 0.86
    lo, hi = B.heights(math.radians(70.0), TRASH_BODY)
    assert 0.5 < lo < 0.6 and 1.0 < hi < 1.1


def test_body_paces_the_pitch_joints_and_keeps_the_turn():
    fake, d = _body_driver()
    text = d.body(lean=70, height=0.58)
    assert len(fake.chunks) == 1
    rows = fake.chunks[0]
    w = np.array([r["waist"] for r in rows])
    assert w.shape[1] == 5 and np.allclose(w[:, 3:], 0.0)
    assert float(np.max(np.abs(np.diff(w[:, :3], axis=0)))) <= motion.BODY_RAD_S * d.row_s * 1.001
    got = B.posture(fake.latest().waist)
    assert abs(math.degrees(got.lean) - 70.0) < 1e-3 and abs(got.shoulder_z - 0.58) < 1e-4
    assert text.startswith("Body: torso leaning 70° forward, shoulders at z 0.58")
    # the hands rode along: their angles now are the ones goto keeps
    assert all(np.allclose(d.asked[a], d.angles_now(a)) for a in d.arms)


def test_body_refuses_other_robots_and_postures_it_has_not():
    with pytest.raises(Refused, match="no body"):
        motion.RobotDriver(BodyLink(), mobile=False).body(lean=30)
    fake, d = _body_driver()
    with pytest.raises(Refused, match="leans 0 to 75"):
        d.body(lean=80)
    with pytest.raises(Refused, match="from z 0.86 to 1.34"):
        d.body(lean=0, height=0.85)
    with pytest.raises(Refused, match="degrees"):
        d.body(lean="far")
    assert not fake.chunks


def test_body_dry_run_and_reading_send_nothing():
    fake, d = _body_driver()
    assert "dry run" in d.body(lean=45, height=0.8, dry_run=True)
    assert d.body().startswith("Body: torso leaning 32° forward, shoulders at z 1.07")
    assert not fake.chunks


def test_body_keeps_the_arms_off_the_floor(monkeypatch):
    fake, d = _body_driver()
    held = {a: np.asarray(fake.command[a], float) for a in d.arms}
    z0, _ = d._lowest(TRASH_BODY, held)
    z1, _ = d._lowest(B.solve(math.radians(70.0), 0.58, TRASH_BODY), held)
    assert z1 < z0 - 0.2
    monkeypatch.setattr(motion, "FLOOR_Z", 0.5 * (z0 + z1) - motion.FLOOR_CLEAR_M)
    with pytest.raises(Refused, match="would come down"):
        d.body(lean=70, height=0.58)
    assert not fake.chunks
    d.body(lean=19.2, height=1.245)  # up and back: the arms rise
    assert len(fake.chunks) == 1


def test_home_both_restores_the_start_posture_at_the_body_pace():
    fake, d = _body_driver()
    d.body(lean=70, height=0.58)
    d.home("both")
    w = np.array([r["waist"] for r in fake.chunks[-1]])
    assert np.allclose(w[-1], TRASH_BODY)
    assert float(np.max(np.abs(np.diff(w[:, :3], axis=0)))) <= motion.BODY_RAD_S * d.row_s * 1.001


def test_head_turns_and_tips_in_one_chunk_within_its_range():
    fake, d = _body_driver()
    d.head(yaw=20, pitch=10)
    rows = fake.chunks[-1]
    assert all(r["head"] is not None and len(r["head"]) == 3 for r in rows)
    assert np.allclose(fake.latest().head, [math.radians(20.0), 0.0, math.radians(10.0)])
    reply = msgpack.unpackb(L.pack(rows, True), raw=False)["result"]
    assert reply["head"]["kind"] == "JOINT_ABS" and len(reply["head"]["values"]) == len(rows)
    text = d.head(pitch=40)
    assert "40° became 29°" in text
    assert np.allclose(fake.latest().head[2], math.radians(29.0))
    with pytest.raises(Refused, match="no head"):
        motion.RobotDriver(BodyLink(), mobile=False).head(pitch=10)


def test_pack_sends_the_head_only_once_it_moved():
    f = _frame("g2_90d")
    row = L.start_command(f)
    assert "head" not in msgpack.unpackb(L.pack([row, row], True), raw=False)["result"]


def test_head_camera_rides_on_the_head_joints():
    fake, d = _body_driver()
    f1 = fake.first
    mount = np.eye(4)
    mount[:3, 3] = (0.05, 0.0, 0.08)
    pose1 = f1.arm_base @ B.head_in_arm_base(f1.head) @ mount
    cal = O.Calibrations()
    cal.seen("head", O.Calib(k=np.eye(3), pose=pose1, width=640, height=480), f1)
    f2 = dataclasses.replace(f1, head=np.array([0.3, 0.0, 0.1]))
    got = cal.carried("head", f2, 640, 480).pose
    assert np.allclose(got, f2.arm_base @ B.head_in_arm_base(f2.head) @ mount, atol=1e-9)


def test_card_lists_the_torso_and_the_head_only_where_they_move():
    fake, d = _body_driver()
    lines = S.body_lines(fake.first, d)
    assert len(lines) == 2
    assert "leans 0 to 75 degrees" in lines[0] and "leans 32 degrees with the shoulders at z 1.07" in lines[0]
    assert "tipped 26 degrees down" in lines[1]
    assert S.body_lines(fake.first, motion.RobotDriver(BodyLink(), mobile=False)) == []


# -- goto(speed=): the write task's pen strokes (told only in the task's skill) ----------------------------------
def _grasp_steps(d, fake, arm="right"):
    model, q0 = d.models[arm], fake.first.arm_joints(arm)
    pts = [model.grasp_pose(q)[:3, 3] for q in [q0] + _follow(d, arm, fake.chunks[0], q0)]
    return np.linalg.norm(np.diff(pts, axis=0), axis=1)


def test_goto_speed_caps_the_grasp_points_pace_for_that_move():
    fast, slow = FakeLink("g2_90d"), FakeLink("g2_90d")
    df, ds = motion.RobotDriver(fast), motion.RobotDriver(slow)
    mf = df.goto("right", by=(0.0, 0.06, 0.0), contact=True)
    ms = ds.goto("right", by=(0.0, 0.06, 0.0), contact=True, speed=0.05)
    assert mf.reached and ms.reached and slow.dispatches == 1 and ms.steps == len(slow.chunks[0])
    assert ms.steps > 3 * mf.steps
    assert _grasp_steps(ds, slow).max() <= 0.05 * ds.row_s * 1.05
    assert any("capped at 0.05 m/s" in n for n in ms.notes) and not any("m/s" in n for n in mf.notes)
    # the next move without it goes at the full pace again
    again = ds.goto("right", by=(0.0, -0.06, 0.0))
    assert again.steps == mf.steps


@pytest.mark.parametrize("speed", [0.0, 0.005, 2.0, "fast", float("nan"), True, (0.05,)])
def test_goto_refuses_a_speed_it_cannot_keep(speed):
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    with pytest.raises(Refused, match="top speed in m/s"):
        d.goto("right", by=(0.0, 0.0, -0.02), speed=speed)
    with pytest.raises(Refused, match="top speed in m/s"):
        d.goto("both", by=(0.0, 0.0, -0.02), speed=speed)
    assert fake.dispatches == 0


def test_a_capped_move_too_long_for_one_motion_is_refused_before_anything_moves(monkeypatch):
    monkeypatch.setattr(motion, "SPEED_MAX_ROWS", 50)
    fake = FakeLink("g2_90d")
    d = motion.RobotDriver(fake)
    for dry_run in (True, False):
        with pytest.raises(Refused, match="would take 80 steps, more than 50"):
            d.goto("right", by=(0.0, 0.06, 0.0), speed=0.05, dry_run=dry_run)
    with pytest.raises(Refused, match="more than 50"):
        d.goto("both", by=(0.0, 0.0, 0.04), speed=0.02)
    assert fake.dispatches == 0
    assert d.goto("right", by=(0.0, 0.06, 0.0)).reached  # the same move at the full pace is not capped


def test_both_hands_keep_a_speed_too():
    fast, slow = FakeLink("g2_omnipicker"), FakeLink("g2_omnipicker")
    motion.RobotDriver(fast).goto("both", by=(0.0, 0.0, 0.04))
    moves = motion.RobotDriver(slow).goto("both", by=(0.0, 0.0, 0.04), speed=0.05)
    assert len(moves) == 2 and all(m.reached for m in moves) and slow.dispatches == 1
    assert len(slow.chunks[0]) > 3 * len(fast.chunks[0])


def test_the_host_hands_goto_speed_to_the_driver_and_help_does_not_show_it():
    import inspect
    from hexashell.benches.robocoliseum.host import ColiseumHost
    from hexashell.benches.robocoliseum.proxy import Robot
    fast = ColiseumHost(FakeLink("g2_90d"), scene=_FakeScene()).dispatch(
        "goto", ("right",), dict(by=(0.0, 0.06, 0.0), dry_run=True))
    slow = ColiseumHost(FakeLink("g2_90d"), scene=_FakeScene()).dispatch(
        "goto", ("right",), dict(by=(0.0, 0.06, 0.0), dry_run=True, speed=0.05))
    assert slow.steps > 3 * fast.steps
    assert "speed" in inspect.signature(Robot.goto).parameters
    assert "speed" not in (Robot.goto.__doc__ or "")
