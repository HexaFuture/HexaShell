"""RoboColiseum Season II, the dexmanip board's G2 with Wuji five-finger hands: the hands' kinematics and shapes,
robot.hand and its holds, gripper and goto(grip=) on a hand, a press kept by a relative move, the fisheye hand
cameras, the self mask and the task card, offline.

The start state is that of a recorded request (pick_up_a_package, the first request of an episode).
"""

from __future__ import annotations

import dataclasses
import math

import cv2
import numpy as np
import pytest

from hexashell.benches.robocoliseum import dexterous as D
from hexashell.benches.robocoliseum import kinematics, motion, wuji
from hexashell.benches.robocoliseum import link as L
from hexashell.benches.robocoliseum import mask as M
from hexashell.benches.robocoliseum import obs as O
from hexashell.errors import Refused

JOINTS = [1.73779, -1.157128, -1.594264, -1.788436, -0.421014, 0.010765, 0.000866,
          -1.733886, -1.153122, 1.596391, -1.787571, 0.422871, 0.00533, -0.000711]
EFFECTORS = [0.0531, 0.0168, 0.0059, 0.0007, 0.0139, 0.0574, 0.0052, 0.0002, -0.0008, 0.059, -0.0, -0.0001,
             -0.0035, 0.0589, -0.001, -0.0001, -0.0105, 0.0564, -0.0034, -0.0003,
             0.053, 0.0163, 0.0058, 0.0006, 0.014, -0.0574, 0.0052, 0.0003, -0.0007, -0.0591, 0.0, -0.0001,
             -0.0035, -0.0591, -0.001, -0.0001, -0.0105, -0.0566, -0.0033, -0.0003]
WAIST = [-0.697541, 1.379489, -0.627112, 1.9e-05, -9e-06]
ARM_BASE = [[0.998497, -0.054809, 1e-05, 0.135351], [-9e-06, 1.9e-05, 1.0, -8e-06],
            [-0.054809, -0.998497, 1.8e-05, 1.171737], [0.0, 0.0, 0.0, 1.0]]
ENDS = {"left": ([0.44604, 0.357188, 0.857623], [0.715188, -0.018833, 0.698152, 0.027119]),
        "right": ([0.447411, -0.358995, 0.85979], [0.71198, 0.019233, 0.701462, -0.025806])}


def _end_pose(joints: np.ndarray) -> dict:
    out = {"base_link": {}, "arm_base_link": {}}
    models = kinematics.arm_models("g2_wuji")
    arm_base = np.array(ARM_BASE, float)
    for k, arm in enumerate(kinematics.ARMS):
        e_b = arm_base @ models[arm].end_pose(joints[7 * k:7 * k + 7])
        out["base_link"][f"{arm}_arm"] = {"position": list(e_b[:3, 3]),
                                          "orientation": kinematics.quat_xyzw_of(e_b[:3, :3])}
    return out


def _frame() -> O.Frame:
    joints = np.array(JOINTS, float)
    return O.Frame(seq=1, t=0.0, task_name="pick_up_a_package", episode_idx=0, prompt="test", done=False, segments=[],
                   robot="g2_wuji", joints=joints, effectors=np.array(EFFECTORS, float), waist=np.array(WAIST),
                   head=None, chassis=None, end_pose=_end_pose(joints), arm_base=np.array(ARM_BASE, float))


def _pinch_opening_at(side: str, width: float) -> float:
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if wuji.pinch_gap(side, wuji.PINCH.joints(side, mid)) > width else (mid, hi)
    return 0.5 * (lo + hi)


class WujiLink:
    """The arms follow the last row (``lag``: joints that stay behind, per arm); the hands follow theirs, except
    a pinch's thumb and index closing on something ``pinch[arm]`` m wide, and a wrap's four fingers on a bar that
    stops them at opening ``bar[arm]``."""

    def __init__(self, *, pinch=None, bar=None, lag=None):
        self.first = _frame()
        self.robot = "g2_wuji"
        self.prompt = self.first.prompt
        self.command = L.start_command(self.first)
        self._frame = self.first
        self.chunks: list[list[dict]] = []
        self.pinch = dict(pinch or {})
        self.bar = dict(bar or {})
        self.lag = dict(lag or {})
        self.over = False
        self.verdict = None

    @property
    def dispatches(self) -> int:
        return len(self.chunks)

    def latest(self) -> O.Frame:
        return self._frame

    def act(self, rows, need_depth):
        assert rows
        self.chunks.append(rows)
        self.command = L.copy_row(rows[-1])
        f = self._frame
        joints, eff = f.joints.copy(), f.effectors.copy()
        for k, arm in enumerate(kinematics.ARMS):
            joints[7 * k:7 * k + 7] = np.asarray(rows[-1][arm], float) - self.lag.get(arm, 0.0)
            q = np.asarray(rows[-1]["grip"][arm], float).reshape(20).copy()
            width = self.pinch.get(arm)
            if width is not None and wuji.pinch_gap(arm, q) < width:
                stop = wuji.PINCH.joints(arm, _pinch_opening_at(arm, width))
                for finger in ("thumb", "index"):
                    s = slice(wuji.FIRST[finger], wuji.FIRST[finger] + 4)
                    q[s] = stop[s]
            g = self.bar.get(arm)
            if g is not None:
                stop = wuji.WRAP.joints(arm, g)
                for finger in ("index", "middle", "ring", "pinky"):
                    s = slice(wuji.FIRST[finger], wuji.FIRST[finger] + 4)
                    if q[s][0] > stop[s][0]:  # curled further than the bar lets it
                        q[s] = stop[s]
            eff[20 * k:20 * k + 20] = q
        self._frame = dataclasses.replace(f, seq=f.seq + 1, joints=joints, effectors=eff,
                                          end_pose=_end_pose(joints))
        return self._frame


def _driver(**kw) -> tuple[WujiLink, motion.RobotDriver]:
    link = WujiLink(**kw)
    return link, motion.RobotDriver(link)


# -- the hand's kinematics and shapes ------------------------------------------------------------------
def test_the_arm_chain_reproduces_the_reported_end_poses():
    joints = np.array(JOINTS, float)
    pose = _end_pose(joints)["base_link"]
    for arm, (p, quat) in ENDS.items():
        assert np.allclose(pose[f"{arm}_arm"]["position"], p, atol=1e-4)
        got = kinematics.pose_of(p, quat)[:3, :3]
        want = kinematics.pose_of(pose[f"{arm}_arm"]["position"], pose[f"{arm}_arm"]["orientation"])[:3, :3]
        assert np.allclose(got, want, atol=1e-3)


def test_the_left_hand_is_the_right_one_mirrored():
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = wuji.clamp(rng.uniform(wuji.LIMITS[:, 0], wuji.LIMITS[:, 1]))
        right = wuji.points("right", q)
        left = wuji.points("left", wuji.mirror_q(q))
        for name in wuji.FINGERS:
            assert np.allclose(left[name], right[name] * [1, -1, 1], atol=1e-4)


def test_the_pinch_gap_is_about_three_centimetres_times_the_opening():
    for side in ("left", "right"):
        for g in (0.0, 0.25, 0.5, 0.75, 1.0):
            gap = wuji.pinch_gap(side, wuji.PINCH.joints(side, g))
            assert abs(gap - 0.0302 * g) < 0.0012


@pytest.mark.parametrize("shape", sorted(wuji.SHAPES))
def test_each_shape_has_its_grasp_point_and_a_right_handed_tool_frame(shape):
    models = kinematics.arm_models("g2_wuji", shape)
    for arm in kinematics.ARMS:
        m = models[arm]
        assert m.hand.dexterous and m.hand.shape == shape
        assert np.allclose(m.grasp_offset[:3, 3], wuji.SHAPES[shape].point(arm))
        t = m.tool_in_end
        assert np.allclose(t.T @ t, np.eye(3), atol=1e-9) and np.linalg.det(t) > 0.999
        assert np.allclose(t[:, 0], [0, 0, 1])
    # pointing straight down at yaw 0, roll 0, both hands close along y (mirror images)
    yl = models["left"].tool_in_end[:, 1]
    yr = models["right"].tool_in_end[:, 1]
    assert np.allclose(yl * [1, -1, 1], -yr, atol=1e-9)


def test_the_flat_start_has_both_hands_pointing_forward_near_roll_zero():
    _, d = _driver()
    for arm in d.arms:
        p, y, r = d.start_angles[arm]
        assert abs(p) < 5 and abs(y) < 8 and abs(r) < 5
    assert d.dexterous and all(d.hands[a].shape == "flat" for a in d.arms)


# -- robot.hand ---------------------------------------------------------------------------------------------
def test_a_change_of_shape_is_one_paced_chunk_and_moves_the_grasp_point():
    link, d = _driver()
    before = d.grasp_pose("right")[:3, 3].copy()
    end = d.frame.end_link("right")
    res = d.hand("right", "pinch")
    assert link.dispatches == 1 and res.steps == len(link.chunks[0])
    rows = link.chunks[0]
    q = [np.asarray(r["grip"]["right"], float) for r in rows]
    step = max(float(np.max(np.abs(b - a))) for a, b in zip(q, q[1:]))
    assert step <= D.HAND_RAD_S * d.row_s + 1e-9
    target = D.targets("right", wuji.PINCH, 1.0, {})
    assert np.allclose(q[-1], target)
    settle = math.ceil(D.HAND_SETTLE_S / d.row_s - 1e-9)
    assert all(np.allclose(v, target) for v in q[-settle:])
    # the left hand and the arms held
    assert all(np.allclose(r["left"], link.first.arm_joints("left")) for r in rows)
    assert all(np.allclose(r["grip"]["left"], link.first.effector("left")) for r in rows)
    after = d.grasp_pose("right")[:3, 3]
    assert np.allclose(after, (end @ np.r_[wuji.PINCH.point("right"), 1.0])[:3], atol=1e-6)
    assert np.linalg.norm(after - before) > 0.04
    # a change of shape opens the new one: an open hand, not a missed close
    assert res.was == "flat" and res.shape == "pinch" and res.opening == 1.0 and res.holding is None and res.opened
    assert "grasp point now" in str(res) and "where the pads meet" in str(res)


def test_the_angles_last_asked_carry_over_into_the_new_shape():
    _, d = _driver()
    d.goto("right", by=(0.0, 0.0, 0.05), pitch=90, yaw=0, roll=0)
    rot_before = d.models["right"].end_rotation(motion.rotation_of(*d.asked["right"]))
    d.hand("right", "pinch")
    rot_after = d.models["right"].end_rotation(motion.rotation_of(*d.asked["right"]))
    assert np.allclose(rot_before, rot_after, atol=1e-3)  # the angles are kept rounded
    # pointing down, the pinch's yaw differs from the flat hand's, the wrist being the same
    assert abs(d.asked["right"][0] - 90.0) < 1.0


def test_a_pinch_on_something_two_centimetres_wide_holds_it():
    link, d = _driver(pinch={"right": 0.02})
    d.hand("right", "pinch")
    res = d.hand("right", opening=0.0)
    assert res.holding is True and set(res.holders) == {"thumb", "index"} and d.catches["right"] == 1
    assert abs(res.gap_m - 0.02) < 0.002 and "holding" in str(res)
    assert d.squeezing("right")
    status = str(d.arm_status("right"))
    assert "hand pinch, opening 0.00, holding" in status
    # opening wide lets go: the hand is open, not a close that missed
    link.pinch = {}
    res = d.hand("right", opening=1.0)
    assert res.holding is None and res.opened and not d.squeezing("right")
    assert "open: nothing between the thumb and index" in str(res) and "closed without stopping" not in str(res)


def test_a_pinch_that_closes_on_nothing_is_empty():
    _, d = _driver()
    d.hand("right", "pinch")
    res = d.gripper("right", "close")
    assert isinstance(res, D.Hand) and res.holding is False and d.catches["right"] == 0
    assert "empty" in str(res)


def test_a_wrap_holds_with_two_fingers_stopped():
    _, d = _driver(bar={"left": 0.6})
    d.hand("left", "wrap")
    res = d.hand("left", opening=0.0)
    assert res.holding is True and len(res.holders) == 4


def test_flat_and_point_have_no_opening():
    _, d = _driver()
    with pytest.raises(Refused, match="no opening"):
        d.gripper("right", "close")
    with pytest.raises(Refused, match="no opening"):
        d.goto("right", by=(0, 0, 0.02), grip=0.0)
    with pytest.raises(Refused, match="hand shape"):
        d.hand("right", "claw")
    res = d.hand("right", "point")
    assert res.holding is None and res.opening is None
    # a shape asked with an opening it does not have: the shape, without the opening (it raised, and the rest
    # of that reply's code did not run)
    res = d.hand("right", "flat", 0.0)
    assert d.hands["right"].shape == "flat" and res.opening is None and "the opening was left out" in str(res)
    # an opening alone, for a hand that has none, is still refused
    with pytest.raises(Refused, match="no opening"):
        d.hand("right", opening=0.5)



def test_a_change_to_an_open_shape_says_open_not_a_missed_close():
    _, d = _driver()
    res = d.hand("right", "pinch")
    assert res.opened and res.holding is None
    assert "open: nothing between the thumb and index" in str(res) and "empty" not in str(res)
    res = d.hand("left", "wrap", 1.0)
    assert res.opened and "open: nothing between the index, middle, ring and pinky" in str(res)
    # a close that stops on nothing is still empty
    res = d.gripper("right", "close")
    assert res.holding is False and not res.opened and "closed without stopping on anything" in str(res)
    # wider than it was is open too
    d.hand("right", opening=0.2)
    res = d.hand("right", opening=0.6)
    assert res.opened and res.holding is None

def test_fingers_set_on_their_own_and_dropped_on_a_change_of_shape():
    link, d = _driver()
    d.hand("right", "flat", fingers={"thumb": 1.0, "pinky": (60, 0, 30, 20)})
    q = np.asarray(link.command["grip"]["right"], float)
    assert np.allclose(q[0:4], wuji.clamp(np.r_[wuji.THUMB_TUCKED, np.zeros(16)])[0:4])
    assert np.allclose(q[16:20], np.radians([60, 0, 30, 20]))
    assert d.hands["right"].fingers.keys() == {"thumb", "pinky"}
    d.hand("right", "wrap")
    assert d.hands["right"].fingers == {}
    with pytest.raises(Refused):
        d.hand("right", fingers={"sixth": 0.5})


def test_the_same_shape_again_sends_nothing_and_a_dry_run_says_where_the_point_would_be():
    link, d = _driver()
    res = d.hand("right", "flat")
    assert link.dispatches == 1  # the start readings are not quite flat: one motion
    res = d.hand("right", "flat")
    assert link.dispatches == 1 and res.steps == 0
    dry = d.hand("left", "point", dry_run=True)
    assert link.dispatches == 1 and dry.dry_run and dry.steps > 0
    end = d.frame.end_link("left")
    assert np.allclose(dry.at, (end @ np.r_[wuji.POINT.point("left"), 1.0])[:3], atol=1e-4)
    assert d.hands["left"].shape == "flat"


# -- goto with a hand ---------------------------------------------------------------------------------------
def test_goto_grip_moves_the_fingers_along_and_never_turns_the_hand_half_round():
    link, d = _driver(pinch={"right": 0.02})
    d.hand("right", "pinch")
    n = link.dispatches
    move = d.goto("right", by=(0.0, 0.0, -0.01), grip=0.0)
    assert link.dispatches == n + 1 and move.reached
    rows = link.chunks[-1]
    hand_rows = D.path(link.chunks[-2][-1]["grip"]["right"], D.targets("right", wuji.PINCH, 0.0, {}), d.row_s)
    assert len(rows) >= len(hand_rows)
    assert d.holding("right") is True and d.catches["right"] == 1
    # an approach with the hand open: the fingers arrive as asked, never half a turn round
    d.hand("right", opening=1.0)
    move = d.goto("right", by=(0.05, 0.0, -0.05), pitch=90, yaw=0)
    assert "half a turn" not in str(move)


def test_a_hold_lost_on_the_way_is_dropped():
    link, d = _driver(pinch={"right": 0.02})
    d.hand("right", "pinch")
    d.hand("right", opening=0.0)
    assert d.holding("right") is True
    link.pinch = {}  # it slips out: the fingers reach their targets
    move = d.goto("right", by=(0.0, 0.0, 0.05))
    assert move.dropped and move.halt == "Dropped" and d.holding("right") is False


def test_a_relative_move_of_pressing_hands_keeps_the_press():
    link, d = _driver()
    base = np.array(ARM_BASE, float)
    stays_behind = np.r_[0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    for arm, dy in (("left", -0.03), ("right", 0.03)):
        # the palm closes on a box's side and stops there: from now on its joints stay behind their commands
        link.lag[arm] = stays_behind
        move = d.goto(arm, by=(0.0, dy, 0.0), contact=True)
        assert not move.blocked and "keeps pressing" in str(move)
        assert d.pressing[arm] > motion.CONTACT_STOP_M
    sent = {a: base @ d.models[a].grasp_pose(np.asarray(link.command[a], float)) for a in d.arms}
    moves = d.goto("both", by=(0.0, 0.0, 0.08))
    for m in moves:
        assert not m.blocked and m.reached and "keeps pressing on what stopped it" in str(m)
    for a in d.arms:
        now = base @ d.models[a].grasp_pose(np.asarray(link.command[a], float))
        assert np.allclose(now[:3, 3] - sent[a][:3, 3], [0, 0, 0.08], atol=0.003)
        assert d.pressing[a] > motion.CONTACT_STOP_M
    # home lets go of the press
    d.home()
    assert all(d.pressing[a] == 0.0 for a in d.arms)



def test_a_contact_move_that_reaches_its_target_says_it_presses_on_nothing():
    link, d = _driver()
    move = d.goto("right", by=(0.0, 0.03, 0.0), contact=True)
    assert move.reached and "presses on nothing" in str(move)
    # stopped by something: it keeps pressing, and says that instead
    link.lag["right"] = np.r_[0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    move = d.goto("right", by=(0.0, 0.03, 0.0), contact=True)
    assert "keeps pressing" in str(move) and "presses on nothing" not in str(move)
    # without contact=True nothing is said
    link.lag = {}
    move = d.goto("left", by=(0.0, -0.03, 0.0))
    assert "presses on nothing" not in str(move)

# -- the hand cameras, the mask, the card ---------------------------------------------------------------
def test_the_fisheye_pictures_become_a_pinhole_with_the_mount_pose():
    frame = _frame()
    w, h = wuji.CAMERA_SIZE
    rgb = np.full((h, w, 3), 90, np.uint8)
    fx, fy, cx, cy = wuji.CAMERA_K
    cv2.circle(rgb, (int(round(cx)), int(round(cy))), 30, (255, 0, 0), -1)
    depth = np.full((h, w), 0.5, np.float32)
    out_rgb, out_depth, cal = O._wuji_hand("right_wrist", frame, rgb, depth)
    assert out_rgb.shape == (wuji.PINHOLE_SIZE[1], wuji.PINHOLE_SIZE[0], 3) and out_depth.shape == out_rgb.shape[:2]
    py, px = int(round(cal.k[1, 2])), int(round(cal.k[0, 2]))
    assert out_rgb[py, px, 0] > 200 and out_rgb[py, px, 2] < 50  # the optical axis stays in the middle
    assert np.allclose(out_depth[py, px], 0.5)
    assert np.allclose(cal.pose, frame.end_link("right") @ wuji.camera_mount("right"))
    # the camera looks along the fingers: the index fingertip is ahead of it, in the picture
    end = frame.end_link("right")
    tip = (end @ np.r_[wuji.points("right", frame.effector("right"))["index"], 1.0])[:3]
    local = np.linalg.inv(cal.pose) @ np.r_[tip, 1.0]
    assert local[2] > 0.1
    u = cal.k[0, 0] * local[0] / local[2] + cal.k[0, 2]
    v = cal.k[1, 1] * local[1] / local[2] + cal.k[1, 2]
    assert 0 < u < wuji.PINHOLE_SIZE[0] and 0 < v < wuji.PINHOLE_SIZE[1]


def test_the_mask_marks_the_fingers_and_the_palm_but_not_what_is_ahead():
    frame = _frame()
    arms = M.frame_arms(frame)
    for arm in kinematics.ARMS:
        end = frame.end_link(arm)
        pts = wuji.points(arm, frame.effector(arm))
        inside = [(end @ np.r_[pts[n], 1.0])[:3] for n in ("index", "pinky", "palm", "index_link4")]
        ahead = [(end @ np.r_[pts["index"] + [0.0, 0.0, 0.06], 1.0])[:3]]
        got = M.inside_arm(np.array(inside + ahead), arms[arm])
        assert got[:4].all() and not got[4]


def test_the_card_describes_the_hands():
    from hexashell.benches.robocoliseum import session

    link, d = _driver()
    lines = session.robot_lines(link.first, d)
    text = "\n".join(lines)
    assert lines[0].startswith("Robot: AgiBot G2") and "Wuji five-finger hands" in lines[0] and "{" not in text
    assert any(line.startswith("- Hands (robot.hand") for line in lines)
    assert "1 -> 3.0 cm" in text and "Gripper:" not in text
    assert any(line.startswith("- Wrist cameras: each sits at the heel of its palm") for line in lines)


def test_even_extents_count_a_crowded_near_end_by_its_area():
    from hexashell.benches.robocoliseum.footprint import even_extent
    from hexashell.vision.rgbd import closing_yaw_robocasa

    rng = np.random.default_rng(1)
    # a 35 x 20 x 22 cm carton on a table at z 0.76, its length along x: the head sees its top evenly; a wrist
    # camera beside its near end sees the first 15 cm of a side face, a hundred times more densely
    top = np.c_[rng.uniform(0.60, 0.95, 4000), rng.uniform(-0.16, 0.04, 4000), np.full(4000, 0.98)]
    side = np.c_[rng.uniform(0.60, 0.75, 60000), np.full(60000, -0.16), rng.uniform(0.76, 0.98, 60000)]
    pts = np.r_[top, side]
    crowded = np.percentile(pts[:, 0], [3, 97])
    assert crowded[1] - crowded[0] < 0.2  # what the shared fit's percentiles see
    ext = even_extent(pts, closing_yaw_robocasa)
    assert 0.31 < ext.size[0] < 0.36 and 0.17 < ext.size[1] < 0.21 and abs(ext.size[2] - 0.22) < 0.025
    assert abs(ext.center[0] - 0.775) < 0.02 and abs(ext.bottom - 0.76) < 0.02 and abs(ext.top - 0.98) < 0.005
    assert not ext.round and abs(ext.grip_yaw) < 5  # the fingers close across its width, along y


def test_hands_get_their_own_parts_of_the_system_prompt():
    from types import SimpleNamespace

    from hexashell.benches.robocoliseum import session
    from hexashell.runtime import card

    def variant(dexterous):
        fake = SimpleNamespace(host=SimpleNamespace(driver=SimpleNamespace(dexterous=dexterous)))
        return session.ColiseumSession.prompt_variant(fake)

    assert variant(True) == "hands" and variant(False) is None
    hands, grippers = (" ".join(t.split()) for t in (card.system_prompt("robocoliseum", variant="hands"),
                                                       card.system_prompt("robocoliseum")))
    assert "a five-finger hand on each arm" in hands and "two-finger gripper" not in hands
    assert "two-finger gripper on each arm" in grippers and "five-finger hand on each arm" not in grippers
    assert "Turn a hand in the open" in hands and "An open, empty gripper is symmetric" not in hands
    for part in ("04_seeing", "03_api"):  # the parts without a variant are the shared ones
        text = " ".join((card.PROMPTS / "robocoliseum" / f"{part}.md").read_text(encoding="utf-8").split())
        assert text in hands and text in grippers
