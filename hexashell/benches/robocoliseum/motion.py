"""The robots' motions behind the robot calls in Season II: the arms that move (``protocol.Robot.arms``: both, on
every robot), their grippers and the G2's waist turn.  Each robot motion is one chunk of rows: one reply to the
benchmark, one dispatch (:mod:`.budget`).

Frames: the world of the robot calls is ``base_link`` (x forward, y left, z up; it moves only when the chassis
does: ``drive``, on the WBC board).  An arm is solved in its ``arm_base_link`` (:mod:`.kinematics`), which the waist
carries (the G2's turn, the G1's lift and pitch); where it is comes with every observation.

Rows: one row is one step.  The simulator plays a chunk one row per step: it sets the row's JOINT_ABS targets
as they are (no smoothing) and reads the state, each served at a physics step, so a row lasts about
``ROW_SUBSTEPS`` physics steps of simulated time (8.3 ms at 240 Hz, 16.7 ms at 120 Hz), not 1/30 s.  After the
last row it waits up to a second of wall time for the arms to arrive before it sends the next request - where
physics runs slowly (liquid; a busy GPU: a few steps a second) the arms are still on the way then; the grippers
are not waited for, and physics goes on while the model thinks.  A joint follows its target with the lag of its
drive (``Build.tau_s``) and no faster than its velocity limit, so a motion's rows are paced within what the
joints follow - none faster than ``PACE_SHARE`` of its limit, the grasp point at most ``HAND_M_S``
(``NEAR_M_S`` in the last ``NEAR_END_M`` of a move, where it may meet something), the hand's turn at most
``TURN_DEG_S``, ramping up and down over ``RAMP_S`` at the ends - and each row's targets run ahead of the path
by the drive's lag, so that the joints end each row where the path is; the path's end is held ``END_ROWS``
rows more.  A motion runs to its end - nothing stops it halfway - and is judged from the request after its
chunk: reached; blocked (the grasp point ended ``LAG_M`` or more behind its command: it is pressing on
something, and keeps pressing until the next motion); dropped (what the gripper held slipped out: the fingers
closed ``DROP_GAP_M`` further).

- ``goto``: a straight line of the grasp point (waypoints every ``STEP_M`` and ``STEP_DEG``, each solved from
  the one before with its spare freedom leaning toward the joints that reach the target; a joint jumping more
  than ``JUMP_RAD`` between two waypoints gets waypoints between them, up to ``SPLITS`` halvings, and still
  jumping it breaks the line; the turn is spread evenly, else in the line's first or second half,
  ``SCHEDULES``) or a swing of the joints (all joints in proportion).  'auto' is the line, else a swing that
  keeps the grasp point within ``SWING_OFF_M`` of it and at most ``SWING_DIP_M`` below its lower end.  An
  open, empty gripper with roll left out may arrive with its fingers half a turn round (the same grasp): when
  the asked way has no line, or pointing at least ``FLIP_PITCH`` down when that saves ``FLIP_SAVE_ROWS`` and
  ``FLIP_SAVE_SHARE`` of the rows; the result's note says so, and after a steep one a closed gripper keeps that
  heading (``ALIAS_DEG``).  ``speed`` (m/s, ``SPEED_MIN_M_S`` up to ``HAND_M_S``) caps the grasp point's pace
  for that one move, the slow end included (the write task's pen strokes; only its skill tells it); a
  capped move of more than ``SPEED_MAX_ROWS`` rows is refused.
- ``gripper``: the hand's command for an opening (:class:`.kinematics.Hand`), ``GRIP_S`` of rows for a full
  stroke (less for part of one, at least ``GRIP_MIN_S``).  A close that ends with the pads ``HOLD_GAP_M`` or
  more apart holds something (under ``THIN_HOLD_M`` with a note: maybe only the fingertips on its top).
- ``hand`` (five-finger hands: the dexmanip board's Wuji, :mod:`.dexterous`): a hand's shape and opening, which
  set the arm's grasp point (the shape's working point); ``gripper`` and ``goto(grip=)`` set a pinch's or a wrap's
  opening, and no approach arrives half a turn round (the hand is not symmetric).  A relative move (``by=``) of an
  arm left pressing by a contact motion starts from where it was sent, not where it stopped, so it keeps pressing
  (a box squeezed between the palms is lifted with ``goto('both', by=...)``).
- ``home``: lift a low hand, then swing the arms to their start joints (both: the waist too, and on the WBC
  board the body's posture), in one chunk; the head stays as it is.
- ``wait``: hold everything for n rows.
- ``waist``: turn the G2's torso about the vertical (the arms turn with it), then hold ``WAIST_SETTLE_S`` of
  rows (the simulator does not wait for the waist; the G2_crsB build's slower one: ``SLOW_WAIST_SETTLE_S`` plus
  ``SLOW_WAIST_SETTLE_S_PER_RAD`` a radian).
- ``drive`` (a robot whose chassis drives: the WBC board): one pose target for the base - forward, left (m) and
  a turn, from where it stands - in every row of one chunk, paced by ``DRIVE_M_S`` and ``DRIVE_RAD_S`` (see
  there); the arms, grippers and waist hold.  The simulator reports the chassis as zero at every request, so the
  base's move is read from the head pictures before and after the drive (:mod:`.odometry`): the result gives
  what was asked and what the pictures read (or that they cannot tell: after a large turn, or without a depth
  picture from before), a note when the base hardly moved or ended away from where it was sent, and a hand
  pushed off its pose on the way.
- ``body`` (the WBC board's G2: :mod:`.body`): the torso's lean and the shoulders' height, on its three pitch
  joints (the waist's turn and roll kept), paced by ``BODY_RAD_S``, then ``BODY_SETTLE_S`` held; refused when an
  arm, held as it is, would come within ``FLOOR_CLEAR_M`` of the floor on the way.  The simulator waits for a
  free-base robot's whole body after a chunk, so the posture is read when it has arrived.
- ``head`` (the same robot): turn and tip the head on the torso, paced by ``HEAD_RAD_S``; its camera moves with it.

The other arm, the grippers and the waist hold what was last sent; an arm that does not move (none in Season
II) holds its start.
"""

from __future__ import annotations

import math

import numpy as np

from ...errors import Refused
from ...geometry import angle_between, homogeneous, invert, rot_z, slerp, wrap_deg
from ...robot import results as R
from ..pose import rotation_of, shown_angles, shown_near, turn_deg
from . import body as body_model
from . import dexterous as D
from . import kinematics
from . import odometry
from . import wuji
from . import protocol as P
from .link import NATIVE, EpisodeLink, copy_row

# -- planning ------------------------------------------------------------------------------------------
#: line waypoints: at most this far (m) and this much turn (degrees) apart
STEP_M = 0.01
STEP_DEG = 4.0
#: no joint moves more than this (rad) between two waypoints of a line: a bigger jump is halved (the
#: waypoint between solved) up to SPLITS times, and a line that still jumps is no line (the wrist would flip)
JUMP_RAD = 0.25
SPLITS = 3
#: how the hand's turn is spread along a line (the grasp point always moves evenly along it): with
#: the move, in its first half, in its second half - tried in this order
SCHEDULES = ("even", "turn_first", "turn_last")
#: path='auto' swings the joints only when no line leads there and the grasp point then stays this
#: close (m) to the straight line and dips at most SWING_DIP_M below its lower end
SWING_OFF_M = 0.03
SWING_DIP_M = 0.01
#: a swing's waypoints (for its pace and the check above): at most this far apart in every joint (rad)
SWING_SAMPLE_RAD = 0.05
#: the fingers may arrive half a turn round (the same grasp for an open gripper; roll left out):
#: pointing at least this steeply, whichever way is shorter; less steeply, only when the asked way has
#: no line (the wrist camera then sees upside down, which misleads)
FLIP_PITCH = 85.0
#: ... and there, the turned way is taken when its chunk is shorter by at least both of these
FLIP_SAVE_ROWS = 15
FLIP_SAVE_SHARE = 0.25
#: results print a hand pointing at least this steeply with its heading as the yaw (roll 0)
FOLD_PITCH = 80.0
#: after such a turned approach, a closed gripper asked for the yaw first asked (within this) keeps the
#: fingers' heading: the turn would take what it holds half a turn round
ALIAS_DEG = 20.0
#: a refused pose's suggestions: a tilted hand (at least this pitch) that does not reach is offered the yaw
#: half a turn round (tilted from the other side, the open fingers' line seen from above the same); a hand
#: pointing down that reaches its pose but has no straight line there is offered the nearest of these yaw
#: offsets that has one, at most LINE_ALTERNATIVES of them
OTHER_SIDE_PITCH = 20.0
LINE_ALTERNATIVE_YAWS = (30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 180.0)
LINE_ALTERNATIVES = 2
#: a two-hand move refused for a hand's line (native runtimes): how far both lines lead together is found by
#: halving the offset this many times, and offered when it is at least BOTH_LEAST_M
BOTH_HALVINGS = 5
BOTH_LEAST_M = 0.01
#: random IK seeds tried when the joints now, the last command and the start do not reach a pose (the same
#: seeds every time, so that a dry run and the move after it plan alike)
RANDOM_SEEDS = 12
#: a move asked right after its dry run, with the arm's joints within this (rad) of where they were and the
#: waist where it was, follows the dry run's plan
DRY_KEEP_RAD = 0.01

# -- pace -----------------------------------------------------------------------------------------------
#: a row's simulated time: ROW_SUBSTEPS physics steps (genie_sim 3.6 pi_env.step serves the row's targets and
#: the state read each at a physics step; measured: 8.3-9.4 ms at 240 Hz, 16.7 ms at 120 Hz, 33 ms at
#: 60 Hz); PHYSICS_HZ when the configuration gives no rate
ROW_SUBSTEPS = 2
PHYSICS_HZ = 120
#: a stage's step limit counts rows (one action row is one step; the wait for the arm after a chunk is not
#: counted), so motions go briskly, within what the drives follow: each joint at most PACE_SHARE of its
#: velocity limit (at its fastest, the start of a row), the grasp point HAND_M_S (NEAR_M_S in the last
#: NEAR_END_M), the hand's turn TURN_DEG_S (per simulated second: at 120 Hz 2 cm, 6 mm and 6 degrees a row);
#: the pace ramps up and down over RAMP_S at the ends
PACE_SHARE = 0.7
HAND_M_S = 1.2
NEAR_M_S = 0.36
NEAR_END_M = 0.03
TURN_DEG_S = 360.0
#: goto(speed=): the grasp point's top speed for one move (m/s), from SPEED_MIN_M_S up to HAND_M_S (it caps
#: NEAR_M_S too); a move it stretches past SPEED_MAX_ROWS rows is refused rather than sent (a stage's steps are
#: rows)
SPEED_MIN_M_S = 0.01
SPEED_MAX_ROWS = 1200
RAMP_S = 0.1
#: the default ramp of profile(), in rows (RAMP_S at PHYSICS_HZ)
RAMP_ROWS = RAMP_S * PHYSICS_HZ / ROW_SUBSTEPS
END_ROWS = 2
#: the share of the drives' lag the targets run ahead by (RobotDriver._lead)
LEAD_GAIN = 1.0

# -- judging --------------------------------------------------------------------------------------------
#: blocked: the grasp point this far (m) behind its command after the chunk
LAG_M = 0.02
#: a contact push (no Blocked) that ends this far (m) behind its command met something: it says so (of the moves
#: that raised nothing in the learning runs, contact pushes included, 85% ended within 3 mm of their command and
#: 94% within 8 mm; pushes stopped by a socket ended 11-21 mm behind and read as plain "short of target")
CONTACT_STOP_M = 0.008
#: five-finger hands: an opening this wide is the hand open (an empty hand there is open, not a missed close)
OPEN_HAND = 0.999
#: a pinch "holding" with its pads this much (m) farther apart than its opening's gap is held apart, not a grasp
WIDER_THAN_OPENING_M = 0.01
#: a move within these of its target reached it
REACHED_MM = 10.0
REACHED_DEG = 3.0
#: a pose the arm misses by at most these is accepted: it goes to the closest pose it reaches (noted)
NEAR_MM = REACHED_MM
NEAR_DEG = REACHED_DEG

# -- the gripper, waiting, the waist, home ----------------------------------------------------------------
#: simulated seconds of a full stroke of the gripper (open to closed takes about 35 rows at 120 Hz on both G2
#: grippers: 40 rows there), and of the shortest
GRIP_S = 2.0 / 3.0
GRIP_MIN_S = 1.0 / 6.0
#: a close that ends with the pads at least this far apart (m) holds something
HOLD_GAP_M = 0.003
#: a hold with the pads less than this far apart (m) gets a note: something thin, or the fingertips came down on
#: top of the object instead of around it (the ctek90d read about 3 mm on a phone's top, and the lift left the
#: phone on the table)
THIN_HOLD_M = 0.006
#: (no note for a wide hold that barely closed: the ctek90d read 7.3 cm on a can measured 6.3 cm that it held,
#: lifted and carried, and 7.6 cm on a 3 cm plug that it did not hold; only the object's size tells them apart)
#: holding: the pads closing this much further (m) means it slipped out
DROP_GAP_M = 0.004
WAIT_MAX = 150
#: the waist: its yaw joint (body joint 5, about the vertical), how fast it turns (rad per simulated second:
#: 0.04 a row at 120 Hz), the most (deg), the simulated seconds held after a turn (20 rows at 120 Hz)
WAIST_YAW = 4
WAIST_RAD_S = 2.4
WAIST_LIMIT_DEG = 170.0
WAIST_SETTLE_S = 1.0 / 3.0
#: the G2_crsB build's waist (finemanip, WBC; the Wuji build has the same body) follows its command slowly: 35
#: rows after a 20-degree turn was sent it read 60-72 % of the way, 1.3-1.5 s after the start 97-100 % (35
#: degrees: 99 % at 1.7 s); the G2 omnipicker's read 99 % after 37 rows.  These robots hold this long after a turn (s), plus
#: this much per radian turned
SLOW_WAIST_ROBOTS = ("g2_90d", "g2_wuji")
SLOW_WAIST_SETTLE_S = 0.9
SLOW_WAIST_SETTLE_S_PER_RAD = 0.8
#: home lifts a hand first when it is this much (m) below its start height
HOME_LIFT_M = 0.05
#: the chassis (the WBC board's robot).  The simulator's chassis servo takes a chunk's rows as DRIVE_ROW_S each,
#: whatever a row's physics time, and steers the base to the chunk's pose target by then (at most 1 m/s and 1.57
#: rad/s); after the chunk it holds that target and waits for the base to stop before its next request.  A drive's
#: rows pace it at DRIVE_M_S and DRIVE_RAD_S of that time (at least DRIVE_MIN_S): in a probe (G2,
#: open_the_door) 0.3 m forward and 0.2 m left in 60 rows each ended 1 mm off, +30 and -30 degrees in 60 rows
#: 0.2 degrees off; 0.6 m in 30 rows ran the right hand into the door frame.  One drive goes at most DRIVE_MAX_M
#: and DRIVE_MAX_DEG; a hand that ends DRIVE_PUSHED_M or more off its commanded pose met something on the way
DRIVE_ROW_S = 1.0 / 30.0
DRIVE_M_S = 0.15
DRIVE_RAD_S = math.radians(30.0)
DRIVE_MIN_S = 1.0
DRIVE_MAX_M = 1.5
DRIVE_MAX_DEG = 180.0
#: a drive whose move, as the head pictures give it (:mod:`.odometry`), ends this far (m, and per metre asked) or
#: this many degrees (and per degree asked) from what was asked gets a note; one that moved less than
#: DRIVE_STILL_M and DRIVE_STILL_DEG did not move; the move read stands for the asked one in what perception
#: knows (the world's shift) when at least DRIVE_TRUST matched points agree with it
DRIVE_OFF_M, DRIVE_OFF_PER_M = 0.08, 0.2
DRIVE_OFF_DEG, DRIVE_OFF_PER_DEG = 6.0, 0.2
DRIVE_STILL_M, DRIVE_STILL_DEG = 0.03, 2.0
DRIVE_TRUST = 25
DRIVE_PUSHED_M = 0.02
#: the body (robot.body / robot.head: the WBC board's G2_crsB, whose base is free).  Its pitch joints go at most
#: 1.21 rad/s (URDF); in a probe (pick_up_trash) ramps of 0.27-0.77 rad/s to leans of 19-71
#: degrees (shoulders 0.58-1.25 m high) ended within 0.01 rad by the next request and held still, a 71-degree
#: lean for 120 rows too.  A posture moves the pitch joints at BODY_RAD_S, then holds BODY_SETTLE_S; leans are
#: asked within BODY_LEAN_DEG (the probe's range, rounded); a posture that reads more than BODY_OFF_RAD off its
#: command after the move gets a note.  The floor is FLOOR_Z in base_link (the wheels' axles 0.03 m up, 0.07 m
#: in radius); an arm's points (its joints from the third on, the grasp point, the fingertips) keep FLOOR_CLEAR_M
#: above it while the body moves, unless one starts lower (then it may not go lower).  The head turns (+-0.781
#: rad) and tips down (-0.018..0.523 rad) at HEAD_RAD_S (its joints: 2.5 rad/s), then holds HEAD_SETTLE_S
BODY_ROBOTS = ("g2_90d",)
BODY_RAD_S = 0.6
BODY_SETTLE_S = 0.2
BODY_LEAN_DEG = (0.0, 75.0)
BODY_OFF_RAD = 0.03
FLOOR_Z = -0.04
FLOOR_CLEAR_M = 0.03
HEAD_RAD_S = 1.0
HEAD_SETTLE_S = 0.15
HEAD_YAW_DEG = 44.0
HEAD_PITCH_DEG = (0.0, 29.0)


def check_arm(arm, arms=kinematics.ARMS) -> str:
    """``arm`` when it is one of the arms that move (``arms``)."""
    if arm in arms:
        return arm
    if len(arms) == 1:
        if arm in kinematics.ARMS or arm == "both":
            other = next(a for a in kinematics.ARMS if a != arms[0])
            raise Refused(f"this robot moves only its {arms[0]} arm (the {other} one stays as it is): "
                          f"use arm='{arms[0]}'")
        raise Refused(f"arm must be '{arms[0]}' (the only arm that moves), not {arm!r}")
    raise Refused(f"arm must be 'left' or 'right' (or 'both' where a call allows it), not {arm!r}")


def opening_of(value) -> float:
    words = {"open": 1.0, "close": 0.0, "closed": 0.0}
    if isinstance(value, str):
        if value.lower() not in words:
            raise Refused("an opening is 'open', 'close' or a number 0..1")
        return words[value.lower()]
    try:
        g = float(value)
    except (TypeError, ValueError):
        raise Refused("an opening is 'open', 'close' or a number 0..1") from None
    if not (0.0 <= g <= 1.0):
        raise Refused("an opening is 'open', 'close' or a number 0..1")
    return g


def vector_of(value, name: str = "by") -> np.ndarray:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres") from None
    if v.size != 3 or not np.all(np.isfinite(v)):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres")
    return v


def top_speed(value) -> float | None:
    """goto(speed=): None, or the grasp point's top speed in m/s, SPEED_MIN_M_S..HAND_M_S."""
    if value is None:
        return None
    text = f"speed is the grasp point's top speed in m/s, from {SPEED_MIN_M_S:g} to {HAND_M_S:g}"
    if isinstance(value, (bool, str)):
        raise Refused(text)
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise Refused(text) from None
    if not math.isfinite(v) or not SPEED_MIN_M_S <= v <= HAND_M_S:
        raise Refused(text)
    return v


def check_slow_rows(rows: int, top: float) -> None:
    """A move goto(speed=top) stretches past SPEED_MAX_ROWS rows (the steps a call reports) is refused."""
    if rows > SPEED_MAX_ROWS:
        raise Refused(f"at most {top:g} m/s that move would take {rows} steps, more than {SPEED_MAX_ROWS}: "
                      f"split it into shorter moves, or go faster")


def _xyz(p) -> str:
    return "(" + ", ".join(f"{float(v):.3f}" for v in p) + ")"


def _tup(p, nd: int = 4) -> tuple:
    return tuple(round(float(v), nd) for v in p)


def _miss(sol) -> float:
    return sol.pos_err_m + 0.01 * sol.rot_err_deg


def _tidy(angles) -> tuple[float, float, float]:
    """Angles that would print as -180 print as 180."""
    return tuple(float(v) + 360.0 if -180.5 < float(v) <= -179.5 else float(v) for v in angles)


def _share(schedule: str, t: float) -> float:
    """How much of the turn is done at ``t`` (0..1) of the way along a line."""
    if schedule == "turn_first":
        return min(1.0, 2.0 * t)
    if schedule == "turn_last":
        return max(0.0, 2.0 * t - 1.0)
    return t


def half_turn(angles) -> tuple[float, float, float]:
    """The same hand with its fingers half a turn round about the way they point:
    Rz(y) Ry(p) Rx(r + 180) = Rz(y + 180) Ry(180 - p) Rx(r) - pointing steeply, written with the yaw
    turned (as results print it), otherwise with the roll."""
    p, y, r = (float(v) for v in angles)
    if abs(p) >= FOLD_PITCH:
        return (round(wrap_deg(180.0 - p), 2), round(wrap_deg(y + 180.0), 2), r)
    return (p, y, round(wrap_deg(r + 180.0), 2))


def profile(costs, ramp: float = RAMP_ROWS) -> list[tuple[int, float]]:
    """The rows of a path of segments that would take ``costs`` rows each at full pace, the pace ramping up and
    down over ``ramp`` rows at the ends: per row, (segment index, share of that segment done)."""
    c = np.asarray(costs, float).reshape(-1)
    total = float(c.sum())
    if total <= 1e-9:
        return []
    if total >= ramp:
        duration = total + ramp

        def done(t: float) -> float:
            if t < ramp:
                return t * t / (2.0 * ramp)
            if t > duration - ramp:
                return total - (duration - t) ** 2 / (2.0 * ramp)
            return ramp / 2.0 + (t - ramp)
    else:
        duration = 2.0 * math.sqrt(total * ramp)

        def done(t: float) -> float:
            if t <= duration / 2.0:
                return t * t / (2.0 * ramp)
            return total - (duration - t) ** 2 / (2.0 * ramp)

    n = max(1, math.ceil(duration - 1e-9))
    cum = np.concatenate([[0.0], np.cumsum(c)])
    out = []
    for k in range(1, n + 1):
        s = min(total, max(0.0, done(duration * k / n)))
        i = min(max(int(np.searchsorted(cum, s, side="right")) - 1, 0), len(c) - 1)
        frac = 1.0 if c[i] <= 1e-12 else min(1.0, max(0.0, (s - cum[i]) / c[i]))
        out.append((i, frac))
    out[-1] = (len(c) - 1, 1.0)
    return out


def rows_of_time(seconds: float, row_s: float) -> int:
    """Rows that last at least ``seconds`` of simulated time."""
    return max(1, math.ceil(seconds / row_s - 1e-9))


def grip_rows(hand: kinematics.Hand, before: float, after: float, row_s: float = ROW_SUBSTEPS / PHYSICS_HZ) -> int:
    """Rows (of ``row_s`` each) for the gripper to go from one command to another."""
    stroke = abs(float(after) - float(before)) / abs(hand.open_value - hand.closed_value)
    return max(rows_of_time(GRIP_MIN_S, row_s), rows_of_time(GRIP_S * min(stroke, 1.0), row_s))


class RobotDriver:
    """Moves a robot (``protocol.ROBOTS`` key; its arms that move: ``protocol.Robot.arms``) through an
    :class:`~.link.EpisodeLink`.  ``on_frame(frame, rows)``: after every chunk.  ``physics_hz``: the scene's
    physics rate (the configuration's; None: PHYSICS_HZ), which sets a row's simulated time.  ``mobile``: the
    chassis drives (the WBC board)."""

    def __init__(self, link: EpisodeLink, *, log=None, on_frame=None, seed: int = 0, physics_hz: int | None = None,
                 mobile: bool = False):
        self.link = link
        self.mobile = bool(mobile)
        #: drives so far, the last one's pose of the base in the frame it started from (4x4), and the rows sent
        #: when it ended (the pictures before then were taken from elsewhere)
        self.drives = 0
        self.moved_by: np.ndarray | None = None
        self.drive_end = 0
        self.robot = link.robot
        #: the benchmark's physics runtime; on the native ones (dexmanip on libuipc, Newton, MuJoCo) the robot's body
        #: is fixed: libuipc ignores a waist target, Newton takes only its start value, MuJoCo makes any an invalid
        #: episode (link.pack never sends one there)
        self.runtime = getattr(link, "runtime", "isaac")
        self.fixed_body = self.runtime in NATIVE
        #: whether a pinch's pads, read, tell that something holds them apart: not on the native runtimes, whose finger
        #: drives lag (an empty close reads its pads 1.0-1.2 cm apart on libuipc and Newton, against 0.0-0.04 cm
        #: in our model and on Isaac)
        self.gap_tells = self.runtime not in NATIVE
        #: whether the hands' readings tell touching and holding at all: not on libuipc (the cloth tasks), whose hand
        #: model has no cloth and whose soft drives lag, so the readings come from the lag alone ("holding" on
        #: air, 20 of 20 hanger closes, 9 of 9 bag wraps), nor on Newton (the wok: 15 of 15 closes
        #: read "empty", the one that lifted the pan too, and a pinch opened in the air read "holding", 2 of 2: the
        #: finger errors were the same at every close); there the hand's results say that it cannot be told
        self.hands_feel = self.runtime not in ("libuipc", "newton")
        #: whether a two-hand move refused for a hand's line also says which hand could go alone and how far both
        #: could go together: on the native runtimes (there, three such refusals in two fold episodes
        #: were each followed by one hand moved alone); Isaac keeps its words
        self.both_tells_which = self.runtime in NATIVE
        #: the arms that move, and whether the torso turns (the G2's waist)
        self.arms = tuple(P.ROBOTS[self.robot].arms)
        self.turns_waist = P.ROBOTS[self.robot].waist > WAIST_YAW and not self.fixed_body
        # per arm (a five-finger hand's shape changes its arm's grasp point: _set_shape)
        self.models = dict(kinematics.arm_models(self.robot))
        self.hand_model = self.models[self.arms[0]].hand
        self.dexterous = bool(self.hand_model.dexterous)
        #: a row's simulated time (s), the ramp in rows, and per arm each joint's most per row (rad) and the
        #: lead of its targets (see _lead)
        self.physics_hz = int(physics_hz or PHYSICS_HZ)
        self.row_s = ROW_SUBSTEPS / self.physics_hz
        self.ramp_rows = RAMP_S / self.row_s
        self.row_cap: dict[str, np.ndarray] = {}
        self.lead: dict[str, float] = {}
        for arm, m in self.models.items():
            a = math.exp(-self.row_s / m.tau_s)
            # within a row a joint is fastest at its start, (target - q) / tau = (its step in the row) / (tau (1 - a))
            self.row_cap[arm] = PACE_SHARE * m.velocity * m.tau_s * (1.0 - a)
            self.lead[arm] = LEAD_GAIN * a / (1.0 - a)
        first = link.first
        self.frame = link.latest()
        self.log = log or (lambda event: None)
        self.on_frame = on_frame or (lambda frame, rows: None)
        self.seed = int(seed)
        #: the last successful dry run of each arm's goto: its question, the joints then and its plan
        self._dry: dict[str, dict] = {}
        self.start_q = {arm: first.arm_joints(arm) for arm in self.arms}
        self.start_end = {arm: first.end_link(arm) for arm in self.arms}
        self.start_pose = {arm: self.grasp_of(first, arm) for arm in self.arms}
        self.start_angles = {arm: shown_angles(self.models[arm].tool_rotation(self.start_pose[arm][:3, :3]))
                             for arm in self.arms}
        self.start_grippers = {arm: 1.0 if self.dexterous else round(self.hand_model.opening(first.gripper(arm)), 3)
                               for arm in self.arms}
        #: the angles goto keeps when some are left out: the last asked for
        self.asked = dict(self.start_angles)
        self.grip_goal = {arm: self.start_grippers[arm] for arm in self.arms}
        #: whether each gripper holds something: True after a close that stopped on it, False after one that
        #: closed fully or lost it, None after an open (and at the start: unknown)
        self._holding: dict[str, bool | None] = {arm: None for arm in self.arms}
        self.held_gap: dict[str, float | None] = {arm: None for arm in self.arms}
        #: closes that caught something, per arm (the perception hears of each)
        self.catches = {arm: 0 for arm in self.arms}
        #: per arm, after an approach whose fingers arrived half a turn round pointing steeply: (the yaw
        #: asked, the yaw it has), until the gripper opens again (see ALIAS_DEG)
        self.alias: dict[str, tuple[float, float] | None] = {arm: None for arm in self.arms}
        #: five-finger hands: each one's shape and hold (dexterous.HandState)
        self.hands = {arm: D.HandState(target=first.effector(arm)) for arm in self.arms} if self.dexterous else {}
        #: five-finger hands: per arm, how far (m) a contact motion left its grasp point behind its command (0: it
        #: is not pressing); a relative move starts from the command then (see goto)
        self.pressing = {arm: 0.0 for arm in self.arms}
        #: rows and chunks sent by the robot calls in this episode
        self.rows = 0
        self.chunks = 0
        #: whether every chunk asks for the depth pictures (and so the calibration) in the next request
        self.motion_depth = True

    # -- reading -----------------------------------------------------------------------------------
    def grasp_of(self, frame, arm: str) -> np.ndarray:
        """The grasp point's pose (end link orientation) in base_link, as read."""
        return frame.end_link(arm) @ self.models[arm].grasp_offset

    def grasp_pose(self, arm: str) -> np.ndarray:
        return self.grasp_of(self.frame, arm)

    def holding(self, arm: str) -> bool | None:
        return self._holding[arm]

    def squeezing(self, arm: str) -> bool:
        """Whether the hand is closed (a gripper's goal closed; a five-finger hand holding, or a pinch or wrap
        closed fully)."""
        if not self.dexterous:
            return self.grip_goal[arm] <= 0.05
        st = self.hands[arm]
        return self._holding[arm] is True or (st.opening is not None and st.opening <= 0.05)

    def opening_read(self, reading: float) -> float:
        return round(self.hand_model.opening(reading), 3)

    def home_offset(self, arm: str) -> tuple[float, float]:
        now, start = self.grasp_pose(arm), self.start_pose[arm]
        return (round(float(np.linalg.norm(now[:3, 3] - start[:3, 3])), 4),
                round(turn_deg(now[:3, :3], start[:3, :3]), 1))

    def angles_now(self, arm: str, asked=None) -> tuple[float, float, float]:
        rot = self.models[arm].tool_rotation(self.grasp_pose(arm)[:3, :3])
        return _tidy(shown_near(rot, asked) if asked is not None else shown_angles(rot))

    def refresh(self) -> None:
        """Read the latest request."""
        self.frame = self.link.latest()

    # -- sending -----------------------------------------------------------------------------------
    def _row(self, **arms) -> dict:
        row = copy_row(self.link.command)
        for arm, q in arms.items():
            row[arm] = np.asarray(q, float).reshape(self.models[arm].n)
        return row

    def _grip_value(self, opening: float) -> np.ndarray:
        return np.array([self.hand_model.command(opening)], float)

    def _send(self, rows: list[dict], *, need_depth: bool | None = None):
        """One chunk (one dispatch); the frame after it."""
        need = self.motion_depth if need_depth is None else bool(need_depth)
        frame = self.link.act(rows, need)
        self.frame = frame
        self._dry.clear()
        self.rows += len(rows)
        self.chunks += 1
        self.on_frame(frame, rows)
        return frame

    def hold(self, rows: int = 1, *, need_depth: bool = True):
        """Rows that hold everything (one dispatch)."""
        return self._send([self._row() for _ in range(max(1, int(rows)))], need_depth=need_depth)

    def hold_for_depth(self):
        return self.hold(1, need_depth=True)

    # -- pacing ------------------------------------------------------------------------------------
    def _costs(self, paths: dict[str, list[np.ndarray]], *, slow_end: bool = True, waist=None,
               top_m_s: float | None = None) -> np.ndarray:
        """Rows each segment of these joint paths (equal lengths) takes at full pace (``top_m_s``: the grasp
        point at most that fast, goto(speed=))."""
        k = len(next(iter(paths.values()))) - 1
        costs = np.zeros(max(k, 0))
        hand_m_s, near_m_s = HAND_M_S, NEAR_M_S
        if top_m_s is not None:
            hand_m_s, near_m_s = min(HAND_M_S, top_m_s), min(NEAR_M_S, top_m_s)
        hand_row, near_row, turn_row = hand_m_s * self.row_s, near_m_s * self.row_s, TURN_DEG_S * self.row_s
        for arm, path in paths.items():
            model = self.models[arm]
            cap = self.row_cap[arm]
            poses = [model.grasp_pose(q) for q in path]
            end = poses[-1][:3, 3]
            for i in range(k):
                dq = float(np.max(np.abs(path[i + 1] - path[i]) / cap))
                near = slow_end and float(np.linalg.norm(poses[i + 1][:3, 3] - end)) < NEAR_END_M
                dp = float(np.linalg.norm(poses[i + 1][:3, 3] - poses[i][:3, 3])) / (near_row if near else hand_row)
                dr = math.degrees(angle_between(poses[i][:3, :3], poses[i + 1][:3, :3])) / turn_row
                costs[i] = max(costs[i], dq, dp, dr)
        if waist is not None:
            speed = np.full(len(waist[0]), WAIST_RAD_S)
            if len(speed) > WAIST_YAW:
                speed[:3] = BODY_RAD_S  # the G2's pitch joints (the WBC body's lean and height)
            for i in range(k):
                step = np.abs(np.asarray(waist[i + 1], float) - np.asarray(waist[i], float)) / (speed * self.row_s)
                costs[i] = max(costs[i], float(np.max(step)))
        return costs

    def _timed(self, paths: dict[str, list[np.ndarray]], *, slow_end: bool = True, waist=None,
               top_m_s: float | None = None) -> list[dict]:
        """The paths (joint waypoints from the joints now, equal lengths; ``waist``: the waist's, or None) as
        paced rows: per row {arm: joints, 'waist': values or absent}."""
        costs = self._costs(paths, slow_end=slow_end, waist=waist, top_m_s=top_m_s)
        out = []
        for i, f in profile(costs, ramp=self.ramp_rows):
            row = {arm: path[i] + f * (path[i + 1] - path[i]) for arm, path in paths.items()}
            if waist is not None:
                row["waist"] = waist[i] + f * (waist[i + 1] - waist[i])
            out.append(row)
        return out

    def _lead(self, arm: str, path: list[np.ndarray], prev=None) -> list[np.ndarray]:
        """The targets of the rows that bring an arm's joints along ``path`` (one point per row, from the joints
        now).  A joint follows its target like dq/dt = (target - q) / tau, and a row holds its target for
        ``row_s``, so the target that takes a joint from q[k-1] to q[k] in a row is q[k] + L (q[k] - q[k-1]),
        L = a / (1 - a), a = exp(-row_s / tau) (``LEAD_GAIN`` of that); within the joint's range."""
        m = self.models[arm]
        lo, hi = m.lower - kinematics.LIMIT_MARGIN, m.upper + kinematics.LIMIT_MARGIN
        lead = self.lead[arm]
        prev = np.asarray(self.frame.arm_joints(arm) if prev is None else prev, float)
        out = []
        for q in path:
            q = np.asarray(q, float)
            out.append(np.clip(q + lead * (q - prev), lo, hi))
            prev = q
        return out

    def _chunk(self, timed: list[dict], *, grip: dict | None = None, end_rows: int = END_ROWS,
               start: dict | None = None) -> list[dict]:
        """Full rows of a timed plan (the arms not in it hold): the arms' targets lead their paths (``_lead``; from
        ``start[arm]`` instead of the joints read, for an arm that is pressing), and the paths' ends are held
        ``end_rows`` more (at least one: the last row sent is where they end).  ``grip``: per arm a value for every
        row, or a list of one per row (the last one kept)."""
        start = start or {}
        led = {arm: self._lead(arm, [t[arm] for t in timed], start.get(arm)) for arm in kinematics.ARMS
               if timed and arm in timed[0]}
        rows = []
        for k, t in enumerate(timed):
            row = self._row(**{arm: q[k] for arm, q in led.items()})
            if "waist" in t:
                row["waist"] = np.asarray(t["waist"], float)
            for arm, value in (grip or {}).items():
                row["grip"][arm] = value[min(k, len(value) - 1)] if isinstance(value, list) else value
            rows.append(row)
        if rows:
            end = copy_row(rows[-1])
            for arm in led:
                end[arm] = np.asarray(timed[-1][arm], float).reshape(self.models[arm].n)
            rows += [copy_row(end) for _ in range(max(1, end_rows))]
        return rows

    @staticmethod
    def _pad(path: list[np.ndarray], n: int) -> list[np.ndarray]:
        return path + [path[-1]] * (n - len(path))

    # -- solving -----------------------------------------------------------------------------------
    def _random_seeds(self, arm: str, n: int) -> list[np.ndarray]:
        m = self.models[arm]
        rng = np.random.default_rng([self.seed, kinematics.ARMS.index(arm)])
        return [rng.uniform(m.lower, m.upper) for _ in range(n)]

    def _solve(self, arm: str, goal_ab: np.ndarray, q_now: np.ndarray, *, quick: bool = False) -> kinematics.Solution:
        model = self.models[arm]
        seeds = [q_now, self.link.command[arm], self.start_q[arm]]
        if model.beyond_reach(goal_ab[:3, 3]):
            return model.solve(goal_ab, q_now, iterations=60)  # how near it gets, for the message
        sol = model.ik(goal_ab, seeds, iterations=120 if quick else 200)
        if sol.reached or quick:
            return sol
        more = model.ik(goal_ab, [q_now] + self._random_seeds(arm, RANDOM_SEEDS))
        return more if more.reached or _miss(more) < _miss(sol) else sol

    def _line(self, arm: str, cur: np.ndarray, target: np.ndarray, q0: np.ndarray, arm_base: np.ndarray, *,
              q_goal: np.ndarray | None = None, schedule: str = "even"):
        """Joint waypoints (after ``q0``) along the straight line of the grasp point from ``cur`` to ``target``
        (base_link), the turn spread by ``schedule``; each waypoint is solved from the one before, its spare
        freedom leaning toward the joints that reach the target (``q_goal``).  None: no such line."""
        model = self.models[arm]
        inv = invert(arm_base)
        p0, p1 = cur[:3, 3], target[:3, 3]
        r0, r1 = cur[:3, :3], target[:3, :3]
        dist = float(np.linalg.norm(p1 - p0))
        ang = math.degrees(angle_between(r0, r1))
        if dist < 0.001 and ang < 0.5:
            return []
        if schedule != "even" and (ang < 2 * STEP_DEG or dist < 2 * STEP_M):
            return None  # about the even line: nothing new to try
        n = max(1, math.ceil(dist / STEP_M), math.ceil((ang if schedule == "even" else 2.0 * ang) / STEP_DEG))
        q = np.asarray(q0, float)
        lean = None if q_goal is None else np.asarray(q_goal, float) - q
        prev, rest_prev = cur, (None if lean is None else q.copy())
        out: list[np.ndarray] = []
        for i in range(1, n + 1):
            t = i / n
            s = _share(schedule, t)
            pose = homogeneous(slerp(r0, r1, s), p0 + t * (p1 - p0))
            rest = None if lean is None else np.asarray(q0, float) + s * lean
            sol = model.solve(inv @ pose, q, iterations=100, rest=rest)
            if not sol.reached:
                sol = model.solve(inv @ pose, q, iterations=300, rest=rest)
                if not sol.reached:
                    return None
            piece = self._split(model, inv, prev, q, pose, sol.q, rest_prev, rest, SPLITS)
            if piece is None:
                return None
            out += piece
            q, prev, rest_prev = sol.q, pose, rest
        return out

    def _split(self, model, inv, pa, qa, pb, qb, ra, rb, depth: int):
        """The joints from ``qa`` (at pose ``pa``) to ``qb`` (at ``pb``), with waypoints between them
        while a joint jumps more than JUMP_RAD; None when it still jumps after ``depth`` halvings."""
        if float(np.max(np.abs(qb - qa))) <= JUMP_RAD:
            return [qb]
        if depth <= 0:
            return None
        mid = homogeneous(slerp(pa[:3, :3], pb[:3, :3], 0.5), 0.5 * (pa[:3, 3] + pb[:3, 3]))
        rm = None if ra is None or rb is None else 0.5 * (ra + rb)
        sol = model.solve(inv @ mid, qa, iterations=150, rest=rm)
        if not sol.reached:
            return None
        first = self._split(model, inv, pa, qa, mid, sol.q, ra, rm, depth - 1)
        if first is None:
            return None
        second = self._split(model, inv, mid, sol.q, pb, qb, rm, rb, depth - 1)
        return None if second is None else first + second

    @staticmethod
    def _swing(q0: np.ndarray, q1: np.ndarray) -> list[np.ndarray]:
        """Joint waypoints (after ``q0``) of a swing: every joint in proportion."""
        dq = np.asarray(q1, float) - np.asarray(q0, float)
        big = float(np.max(np.abs(dq)))
        if big < 1e-4:
            return []
        n = max(1, math.ceil(big / SWING_SAMPLE_RAD))
        return [np.asarray(q0, float) + dq * (i / n) for i in range(1, n + 1)]

    def _off_line(self, arm: str, plan, p0: np.ndarray, p1: np.ndarray, arm_base: np.ndarray) -> tuple[float, float]:
        """How far (m) the grasp point leaves the straight line from p0 to p1 along a plan, and how far it
        dips below the line's lower end."""
        model = self.models[arm]
        d = p1 - p0
        dd = float(d @ d)
        low = min(float(p0[2]), float(p1[2]))
        off = dip = 0.0
        for q in plan:
            p = (arm_base @ model.grasp_pose(q))[:3, 3]
            t = 0.0 if dd < 1e-12 else min(max(float((p - p0) @ d) / dd, 0.0), 1.0)
            off = max(off, float(np.linalg.norm(p - (p0 + t * d))))
            dip = max(dip, low - float(p[2]))
        return off, dip

    def _rows_of(self, arm: str, q_now, plan) -> int:
        return len(profile(self._costs({arm: [q_now] + list(plan)}), ramp=self.ramp_rows)) + END_ROWS if plan else 0

    def _plan(self, arm, cur, options, q_now, path, arm_base, *, shortest: bool):
        """The motion to the first of ``options`` ((label, angles, target, solution), each reachable, the
        asked one first) that has one: (joint waypoints or None, 'line' / 'swing', notes, the option).
        ``shortest``: a later option whose chunk is clearly shorter (FLIP_SAVE_*) wins over the first."""
        if path in ("auto", "line"):
            found = []
            for opt in options:
                for schedule in SCHEDULES:
                    line = self._line(arm, cur, opt[2], q_now, arm_base, q_goal=opt[3].q, schedule=schedule)
                    if line is not None:
                        found.append((line, opt))
                        break
                if found and not shortest:
                    break
            if found:
                line, opt = found[0]
                rows = self._rows_of(arm, q_now, line)
                for other, o in found[1:]:
                    save = rows - self._rows_of(arm, q_now, other)
                    if save >= FLIP_SAVE_ROWS and save >= FLIP_SAVE_SHARE * rows:
                        line, opt = other, o
                return line, "line", [], opt
            if path == "line":
                return None, "line", ["no straight line of the grasp point leads there from here (a joint would "
                                      "have to turn over on the way); path='swing' swings the joints instead"], None
        if path == "swing":
            opt = options[0]
            return self._swing(q_now, opt[3].q), "swing", [], opt
        worst = None
        for opt in options:
            swing = self._swing(q_now, opt[3].q)
            off, dip = self._off_line(arm, swing, cur[:3, 3], opt[2][:3, 3], arm_base)
            if off <= SWING_OFF_M and dip <= SWING_DIP_M:
                return swing, "swing", [f"no straight line from here: the joints swung (the grasp point left the line "
                                        f"by up to {100 * off:.0f} cm)"], opt
            if worst is None:
                worst = (off, dip)
        off, dip = worst
        below = f", {100 * dip:.0f} cm below its lower end" if dip > SWING_DIP_M else ""
        return None, "swing", [f"no straight line leads there from here, and a swing of the joints would take the "
                               f"grasp point up to {100 * off:.0f} cm off it{below}: path='swing' allows that, or go "
                               "by a point in between (a higher one, or these angles first where the hand is)"], None

    def _alternatives(self, arm, cur, p, angles, q_now, arm_base) -> list[str]:
        inv = invert(arm_base)
        model = self.models[arm]
        out = []
        far = model.beyond_reach((inv @ np.r_[p, 1.0])[:3])

        def reaches(a) -> bool:
            goal = homogeneous(model.end_rotation(rotation_of(*a)), p)
            return self._solve(arm, inv @ goal, q_now, quick=True).reached

        if not far and abs(angles[0]) >= FOLD_PITCH:
            # pointing down: the fingers a quarter turn round, or tilted toward the point (the yaw its heading
            # from the arm's base, the roll that keeps the fingers' line seen from above within 45°)
            quarter = (angles[0], round(wrap_deg(angles[1] + 90.0)), 0.0)
            if reaches(quarter):
                out.append(f"yaw {quarter[1]:.0f} at the same point (the fingers a quarter turn round)")
            o = arm_base[:3, 3]
            heading = float(round(math.degrees(math.atan2(p[1] - o[1], p[0] - o[0]))))
            off = (angles[1] - heading + 90.0) % 180.0 - 90.0
            roll = 0.0 if abs(off) <= 45.0 else 90.0
            for pitch in (75.0, 60.0, 45.0):
                if reaches((pitch, heading, roll)):
                    out.append(f"pitch {pitch:.0f} yaw {heading:.0f} roll {roll:.0f} at the same point (tilted "
                               "toward it; seen from above the fingers close along about the same line)")
                    break
        else:
            if not far and abs(angles[0]) >= OTHER_SIDE_PITCH and not self.dexterous:
                # tilted the other way round: seen from above the (open, symmetric) fingers close along the same
                # line
                other = (angles[0], round(wrap_deg(angles[1] + 180.0)), angles[2])
                goal = homogeneous(model.end_rotation(rotation_of(*other)), p)
                if self._solve(arm, inv @ goal, q_now).reached:
                    out.append(f"yaw {other[1]:.0f} at the same point and pitch (tilted from the other side; seen "
                               "from above the fingers close along the same line)")
            for dp in () if far else (15.0, -15.0, 30.0, -30.0):
                a = (angles[0] + dp, angles[1], angles[2])
                if reaches(a):
                    out.append(f"pitch {a[0]:.0f} at the same point")
                    if len(out) >= 3:
                        break
        rot = model.end_rotation(rotation_of(*angles))
        p0 = cur[:3, 3]
        lo, hi = 0.0, 1.0
        if self._solve(arm, inv @ homogeneous(rot, p0), q_now, quick=True).reached:
            for _ in range(5):
                mid = 0.5 * (lo + hi)
                if self._solve(arm, inv @ homogeneous(rot, p0 + mid * (p - p0)), q_now, quick=True).reached:
                    lo = mid
                else:
                    hi = mid
            if lo > 0.05:
                q = p0 + lo * (p - p0)
                out.append(f"{_xyz(q)} with these angles ({100 * (1 - lo) * float(np.linalg.norm(p - p0)):.0f} cm "
                           "short, on the way there)")
        return out

    def _line_alternatives(self, arm, cur, p, angles, q_now, arm_base) -> list[str]:
        """Pointing down at a pose the arm reaches but no straight line leads to: other yaws at the same point
        that one does lead to, the nearest to the yaw asked first (at most ``LINE_ALTERNATIVES``)."""
        if abs(angles[0]) < FOLD_PITCH:
            return []
        inv = invert(arm_base)
        model = self.models[arm]
        out = []
        # a held part: the wrist turning on the way can shake it out (a 60-degree turn lost a plug, 30 kept it)
        held = self._holding[arm] is True
        for d in LINE_ALTERNATIVE_YAWS:
            a = (angles[0], round(wrap_deg(angles[1] + d)), angles[2])
            goal = homogeneous(model.end_rotation(rotation_of(*a)), p)
            sol = self._solve(arm, inv @ goal, q_now, quick=True)
            if sol.reached and self._line(arm, cur, goal, q_now, arm_base, q_goal=sol.q) is not None:
                if held:
                    turn = turn_deg(model.tool_rotation(cur[:3, :3]), rotation_of(*a))
                    out.append(f"yaw {a[1]:.0f} at the same point (a straight line leads there; the hand turns "
                               f"{turn:.0f}° on the way, which can shake out what the gripper holds)")
                else:
                    out.append(f"yaw {a[1]:.0f} at the same point (a straight line leads there)")
                if len(out) >= LINE_ALTERNATIVES:
                    break
        return out

    def _lag(self, arm: str, frame, q_cmd) -> float:
        model = self.models[arm]
        a = model.grasp_pose(frame.arm_joints(arm))[:3, 3]
        b = model.grasp_pose(q_cmd)[:3, 3]
        return float(np.linalg.norm(a - b))

    # -- the gripper's state ---------------------------------------------------------------------------
    def _slipped(self, arm: str, frame) -> bool:
        if self.dexterous:
            return self._hand_slipped(arm, frame)
        if self._holding[arm] is not True or self.held_gap[arm] is None:
            return False
        gap = self.hand_model.gap_m(frame.gripper(arm))
        if gap < self.held_gap[arm] - DROP_GAP_M:
            self.log({"kind": "dropped", "arm": arm, "reading": round(frame.gripper(arm), 4),
                      "gap_mm": round(1000 * gap, 1), "held_gap_mm": round(1000 * self.held_gap[arm], 1)})
            self._holding[arm] = False
            self.held_gap[arm] = None
            return True
        return False

    def _closed(self, arm: str, reading: float) -> bool:
        """After a close: whether the fingers stopped apart (they hold something)."""
        gap = self.hand_model.gap_m(reading)
        holding = gap >= HOLD_GAP_M
        if holding and self._holding[arm] is not True:
            self.catches[arm] += 1
        self._holding[arm] = holding
        self.held_gap[arm] = gap if holding else None
        return holding

    # -- the five-finger hands' state (dexmanip) -------------------------------------------------------
    def _hand_slipped(self, arm: str, frame) -> bool:
        st = self.hands[arm]
        if self._holding[arm] is not True or st.held is None:
            return False
        if D.still_holds(wuji.SHAPES[st.shape], arm, st, frame.effector(arm)):
            return False
        self.log({"kind": "dropped", "arm": arm, "hand": st.shape, "holders": list(st.holders),
                  "reading": [round(float(v), 3) for v in frame.effector(arm)]})
        self._holding[arm] = False
        st.holders, st.held = (), None
        self.held_gap[arm] = None
        return True

    def _judge(self, arm: str, frame) -> None:
        """After a five-finger hand's motion: which fingers stopped on something, and whether it holds (neither, where
        the readings cannot tell: ``hands_feel``)."""
        st = self.hands[arm]
        reading = frame.effector(arm)
        if not self.hands_feel:
            st.touching, st.holders, st.held = (), (), None
            self._holding[arm] = None
            self.held_gap[arm] = None
            return
        st.touching = D.stopped(st.target, reading)
        holding, holders = D.holds(wuji.SHAPES[st.shape], arm, st.touching, reading)
        if holding and self._holding[arm] is not True:
            self.catches[arm] += 1
        self._holding[arm] = holding
        st.holders = holders
        st.held = reading.copy() if holding else None
        self.held_gap[arm] = wuji.pinch_gap(arm, reading) if holding and st.shape == "pinch" else None

    def _set_shape(self, arm: str, name: str) -> None:
        """A five-finger hand takes another shape: its arm's grasp point and tool frame become that shape's (the
        angles last asked carried over into the new frame)."""
        old = self.models[arm]
        new = kinematics.arm_models(self.robot, name)[arm]
        end_rot = old.end_rotation(rotation_of(*self.asked[arm]))
        self.models[arm] = new
        self.start_pose[arm] = self.start_end[arm] @ new.grasp_offset
        self.start_angles[arm] = shown_angles(new.tool_rotation(self.start_pose[arm][:3, :3]))
        self.asked[arm] = _tidy(shown_near(new.tool_rotation(end_rot), self.asked[arm]))
        self.alias[arm] = None
        self._dry.pop(arm, None)

    def _hand_target(self, arm: str, *, opening: float | None) -> np.ndarray:
        st = self.hands[arm]
        return D.targets(arm, wuji.SHAPES[st.shape], opening, st.fingers)

    def _check_opening(self, arm: str) -> None:
        st = self.hands[arm]
        if wuji.SHAPES[st.shape].fixed:
            raise Refused(f"the {arm} hand is {st.shape}, which has no opening: an opening (gripper, goto's grip=) "
                          f"is a pinch's or a wrap's - hand('{arm}', 'pinch') or hand('{arm}', 'wrap') first")

    def _grip_asked(self, arm: str, g: float) -> None:
        if g > 0.05 and self.grip_goal[arm] <= 0.05:
            self.alias[arm] = None  # it lets go: a later approach starts afresh
        self.grip_goal[arm] = g
        if g > 0.05 and self._holding[arm] is not None:
            self._holding[arm] = None
            self.held_gap[arm] = None

    # -- goto --------------------------------------------------------------------------------------
    def goto(self, arm, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, path="auto",
             dry_run=False, contact=False, speed=None):
        if path not in ("auto", "line", "swing"):
            raise Refused("path is 'auto', 'line' or 'swing'")
        top = top_speed(speed)
        if arm == "both" and len(self.arms) > 1:
            return self._goto_both(xyz, by=by, pitch=pitch, yaw=yaw, roll=roll, grip=grip, path=path,
                                   dry_run=dry_run, contact=contact, speed=top)
        arm = check_arm(arm, self.arms)
        frame = self.frame
        model = self.models[arm]
        # a five-finger hand's arm left pressing by a contact motion: a relative move starts from its command
        press = self.pressing[arm] if (self.dexterous and by is not None and xyz is None) else 0.0
        q_now = np.asarray(self.link.command[arm], float) if press else frame.arm_joints(arm)
        cur = frame.arm_base @ model.grasp_pose(q_now) if press else self.grasp_of(frame, arm)
        asked = list(self.asked[arm])
        for i, v in enumerate((pitch, yaw, roll)):
            if v is not None:
                asked[i] = float(v)
        if yaw is not None and roll is None and abs(asked[0]) >= FOLD_PITCH:
            asked[2] = 0.0  # pointing steeply, the yaw asked is the fingers' heading
        angles = tuple(asked)
        p = cur[:3, 3].copy() if xyz is None else np.asarray(xyz, float).reshape(3)
        if by is not None:
            p = p + vector_of(by)
        g_open = None if grip is None else opening_of(grip)
        if g_open is not None and self.dexterous:
            self._check_opening(arm)
        open_after = self.grip_goal[arm] if grip is None else g_open
        notes: list[str] = []
        # a closed gripper whose fingers arrived half a turn from the yaw asked keeps their heading when
        # asked for that yaw again (see ALIAS_DEG)
        alias = self.alias[arm]
        if (alias is not None and yaw is not None and roll is None and open_after <= 0.05
                and abs(angles[0]) >= FLIP_PITCH and abs(wrap_deg(float(yaw) - alias[0])) <= ALIAS_DEG):
            kept = float(yaw)
            angles = (angles[0], round(wrap_deg(alias[1] + kept - alias[0]), 2), angles[2])
            notes.append(f"kept the fingers' heading, yaw {angles[1]:.0f}: yaw {kept:.0f} names the same grasp half "
                         "a turn round, and turning there would take what it holds half a turn round (with roll "
                         "given too, it turns)")
        at_now, ang_now = _tup(cur[:3, 3]), self.angles_now(arm, angles)

        def refused(why, alternatives=()):
            return R.Move(arm=arm, reached=False, moved=False, target=_tup(p), target_angles=angles, at=at_now,
                          angles=ang_now, dry_run=bool(dry_run), notes=notes + list(why),
                          alternatives=list(alternatives))

        # the same grasp with the fingers half a turn round, for an open, empty gripper with roll left out (not a
        # five-finger hand: it is not the same the other way round)
        choices = [("", angles)]
        if not self.dexterous and roll is None and open_after >= 0.9 and self._holding[arm] is not True:
            choices.append(("half a turn", half_turn(angles)))
        question = (tuple(np.round(p, 4)), tuple(np.round(angles, 2)), [c[1] for c in choices], path)
        kept = self._dry.get(arm)
        if (kept is not None and kept["question"] == question
                and float(np.max(np.abs(q_now - kept["q"]))) <= DRY_KEEP_RAD
                and np.allclose(frame.arm_base, kept["arm_base"], atol=1e-4)):
            plan, how, option = kept["plan"], kept["how"], kept["option"]
            notes += kept["notes"]
        else:
            inv = invert(frame.arm_base)
            p_ab = (inv @ np.r_[p, 1.0])[:3]
            far = model.beyond_reach(p_ab)
            options, near, found = [], None, []
            for label, a in choices:
                goal = homogeneous(model.end_rotation(rotation_of(*a)), p)
                sol = self._solve(arm, inv @ goal, q_now)
                if sol.reached:
                    options.append((label, a, goal, sol))
                elif near is None or _miss(sol) < _miss(near[3]):
                    near = (label, a, goal, sol)
            if not options and near is not None and not far and 1000 * near[3].pos_err_m <= NEAR_MM \
                    and near[3].rot_err_deg <= NEAR_DEG:
                label, a, _, sol = near
                closest = frame.arm_base @ model.grasp_pose(sol.q)
                options.append((label, a, closest, kinematics.Solution(q=sol.q, reached=True, pos_err_m=0.0,
                                                               rot_err_deg=0.0)))
                found.append(f"the arm reaches that pose only to within {1000 * sol.pos_err_m:.0f} mm / "
                             f"{sol.rot_err_deg:.0f}°, so the closest pose it reaches is the target")
            if not options:
                if far:
                    past = float(np.linalg.norm(p_ab - model.shoulder)) - model.reach
                    why = f"the {arm} arm cannot reach that point: it lies {100 * past:.0f} cm beyond the arm's reach"
                else:
                    why = (f"the {arm} arm cannot reach that pose (at best {1000 * near[3].pos_err_m:.0f} mm / "
                           f"{near[3].rot_err_deg:.0f}° off)")
                return refused([why], self._alternatives(arm, cur, p, angles, q_now, frame.arm_base))
            plan, how, more, option = self._plan(arm, cur, options, q_now, path, frame.arm_base,
                                                 shortest=abs(angles[0]) >= FLIP_PITCH)
            found += more
            notes += found
            if plan is not None and dry_run:
                self._dry[arm] = {"question": question, "q": q_now.copy(), "arm_base": frame.arm_base.copy(),
                                  "plan": plan, "how": how, "option": option, "notes": found}
        if plan is None:
            return refused([], self._line_alternatives(arm, cur, p, angles, q_now, frame.arm_base))
        label, used, target, _ = option
        heading = _tidy(shown_near(rotation_of(*used), used))[1]
        if label:
            if abs(angles[0]) >= FOLD_PITCH:
                notes.append(f"the fingers arrive half a turn round, at yaw {heading:.0f} (the same grasp for an open "
                             f"gripper), so the {arm}_wrist picture is upside down from before: pass yaw={heading:.0f} "
                             f"from now on - once the gripper closes, yaw={angles[1]:.0f} is taken as "
                             f"yaw={heading:.0f} (with roll given too, it turns the wrist half a turn)")
            else:
                notes.append(f"the fingers arrive half a turn round, at roll {used[2]:.0f} (the same grasp for an open "
                             f"gripper; with roll {angles[2]:.0f} no path leads there), so the {arm}_wrist picture is "
                             f"upside down from before; roll {used[2]:.0f} is kept for the next moves")
        timed = self._timed({arm: [q_now] + plan}, top_m_s=top) if plan else []
        if top is not None and timed:
            notes.append(f"the grasp point's pace was capped at {top:g} m/s")
        if not timed and g_open is None:
            # already there: nothing to send
            return R.Move(arm=arm, reached=True, moved=not dry_run, target=_tup(p), target_angles=used, at=at_now,
                          angles=ang_now, steps=0, path=how, dry_run=bool(dry_run), notes=notes + ["already there"])
        hand_to = None
        if g_open is None:
            grip_value = None
        elif self.dexterous:
            # the fingers' rows from the motion's start; the arm holds its end while they need longer
            hand_to = self._hand_target(arm, opening=g_open)
            grip_value = D.path(self.link.command["grip"][arm], hand_to, self.row_s)
            if len(timed) + END_ROWS < len(grip_value):
                timed = timed + [{arm: timed[-1][arm] if timed else q_now}] * (len(grip_value) - END_ROWS - len(timed))
        else:
            grip_value = self._grip_value(g_open)
            if not timed:
                before = float(self.link.command["grip"][arm][0])
                timed = [{arm: q_now}] * grip_rows(self.hand_model, before, float(grip_value[0]), self.row_s)
        rows = self._chunk(timed, grip=None if grip_value is None else {arm: grip_value},
                           start={arm: q_now} if press else None)
        if top is not None:
            check_slow_rows(len(rows), top)
        if dry_run:
            return R.Move(arm=arm, reached=True, moved=False, target=_tup(p), target_angles=used, at=at_now,
                          angles=ang_now, steps=len(rows), path=how, dry_run=True, notes=notes)
        if g_open is not None and self.dexterous:
            st = self.hands[arm]
            st.opening, st.target = g_open, hand_to
            self.grip_goal[arm] = g_open
        elif g_open is not None:
            self._grip_asked(arm, g_open)
        if label and abs(angles[0]) >= FOLD_PITCH:
            self.alias[arm] = (float(angles[1]), float(heading))
        elif not label and open_after >= 0.9:
            self.alias[arm] = None
        angles = used
        self.asked[arm] = angles
        start = self.rows
        q_end = plan[-1] if plan else q_now
        frame = self._send(rows)
        lag = self._lag(arm, frame, q_end)
        # a five-finger hand that changed its opening on the way is judged anew (an opening lets go on purpose)
        dropped = False if (g_open is not None and self.dexterous) else self._slipped(arm, frame)
        blocked = not contact and lag - press > LAG_M
        if g_open is not None and self.dexterous:
            self._judge(arm, frame)
        elif g_open is not None and g_open <= 0.05:
            self._closed(arm, frame.gripper(arm))
        if self.dexterous:
            self.pressing[arm] = lag if (contact or press) and lag > CONTACT_STOP_M else 0.0
        move = self._move_result(arm, frame, p, angles, start, how, notes, blocked=blocked, dropped=dropped,
                                 lag=lag, contact=contact, aim=target, press=press)
        self.log({"kind": "goto", "arm": arm, "target": _tup(p), "angles": angles, "path": how, "steps": move.steps,
                  "err_mm": move.err_mm, "lag_mm": move.lag_mm, "halt": move.halt})
        return move

    def _move_result(self, arm, frame, p, angles, start, how, notes, *, blocked, dropped, lag, contact,
                     aim=None, press: float = 0.0) -> R.Move:
        """The errors are from the pose asked; reached is judged at the pose the plan went to (``aim``: the
        closest pose the arm reaches, when it reaches the one asked only nearly), within the press it kept
        (``press``, m: a relative move of a pressing arm, sent from its command)."""
        now = self.grasp_of(frame, arm)
        target_rot = rotation_of(*angles)
        err_mm = 1000.0 * float(np.linalg.norm(now[:3, 3] - p))
        err_deg = turn_deg(self.models[arm].tool_rotation(now[:3, :3]), target_rot)
        slack = 1000.0 * press
        if aim is None:
            reached = err_mm <= REACHED_MM + slack and err_deg <= REACHED_DEG
        else:
            reached = (1000.0 * float(np.linalg.norm(now[:3, 3] - aim[:3, 3])) <= REACHED_MM + slack
                       and turn_deg(now[:3, :3], aim[:3, :3]) <= REACHED_DEG)
        notes = list(notes)
        if press:
            notes.append(f"it moved from where it was sent, {slack:.0f} mm past where it had stopped, so it keeps "
                         "pressing on what stopped it")
        halt = "Dropped" if dropped else "Blocked" if blocked else ""
        if dropped:
            notes.insert(0, "what the gripper held slipped out on the way")
        if blocked:
            notes.append(f"the grasp point ended {1000 * lag:.0f} mm short of its command: something is in the way, "
                         "and the arm keeps pressing on it until the next motion")
        elif contact and lag > CONTACT_STOP_M:
            notes.append(f"the grasp point ended {1000 * lag:.0f} mm behind its command: something stopped it, and "
                         "the arm keeps pressing on it until the next motion")
        elif contact and self.dexterous and not press and not dropped:
            # (pick_up_a_package: palms sent "a little into" a box that ended at their targets were not pressing on
            # it, and nothing said so)
            notes.append("contact=True, but nothing stopped the hand: it is at its command and presses on nothing "
                         "(what was in its way, if anything, moved along)")
        return R.Move(arm=arm, reached=reached, moved=True, target=_tup(p),
                      target_angles=angles, at=_tup(now[:3, 3]), angles=self.angles_now(arm, angles),
                      err_mm=round(err_mm, 1), err_deg=round(err_deg, 1), steps=self.rows - start, path=how,
                      blocked=blocked, dropped=dropped, lag_mm=round(1000 * lag, 1), skipped=0, halt=halt,
                      notes=notes)

    def _goto_both(self, xyz, *, by, pitch, yaw, roll, grip, path, dry_run, contact, speed=None):
        if xyz is not None or by is None or any(v is not None for v in (pitch, yaw, roll)) or grip is not None:
            raise Refused("goto('both', by=(dx, dy, dz)): both hands move together by the same offset along a straight "
                          "line, keeping their angles; nothing else is given with 'both'")
        if path == "swing":
            raise Refused("goto('both', ...) moves along a straight line only")
        d = vector_of(by)
        frame = self.frame
        plans, targets, curs = {}, {}, {}
        inv = invert(frame.arm_base)
        # five-finger hands left pressing (a box squeezed between them) move from their commands: they keep pressing
        press = {arm: self.pressing[arm] if self.dexterous else 0.0 for arm in self.arms}
        q_now = {arm: np.asarray(self.link.command[arm], float) if press[arm] else frame.arm_joints(arm)
                 for arm in self.arms}
        for arm in self.arms:
            cur = frame.arm_base @ self.models[arm].grasp_pose(q_now[arm]) if press[arm] else self.grasp_of(frame, arm)
            target = cur.copy()
            target[:3, 3] = cur[:3, 3] + d
            curs[arm], targets[arm] = cur, target
            sol = self._solve(arm, inv @ target, q_now[arm], quick=True)
            plans[arm] = self._line(arm, cur, target, q_now[arm], frame.arm_base,
                                    q_goal=sol.q if sol.reached else None)
        angles = {arm: self.angles_now(arm, self.asked[arm]) for arm in self.arms}
        bad = [arm for arm in self.arms if plans[arm] is None]
        rows: list[dict] = []
        if not bad:
            n = max(len(plans[a]) for a in self.arms)
            paths = {a: self._pad([q_now[a]] + plans[a], n + 1) for a in self.arms}
            timed = self._timed(paths, top_m_s=speed) if n else []
            rows = self._chunk(timed, start={a: q_now[a] for a in self.arms if press[a]}) if n else []
            if speed is not None:
                check_slow_rows(len(rows), speed)
        if bad or dry_run:
            told = self._both_refused(bad, curs, q_now, frame.arm_base, d, contact) if bad and self.both_tells_which \
                else {}
            out = []
            for arm in self.arms:
                ok = plans[arm] is not None
                notes = [] if ok else [f"the {arm} hand cannot move along that straight line from here"]
                notes += told.get(arm, [])
                out.append(R.Move(arm=arm, reached=ok and not bad, moved=False, target=_tup(targets[arm][:3, 3]),
                                  target_angles=angles[arm], at=_tup(curs[arm][:3, 3]), angles=angles[arm],
                                  steps=len(rows), path="line", dry_run=bool(dry_run), notes=notes))
            return out
        start = self.rows
        if rows:
            frame = self._send(rows)
        out = []
        for arm in self.arms:
            q_end = plans[arm][-1] if plans[arm] else q_now[arm]
            lag = self._lag(arm, frame, q_end)
            dropped = self._slipped(arm, frame)
            m = self._move_result(arm, frame, targets[arm][:3, 3], self.asked[arm], start, "line", [],
                                  blocked=not contact and lag - press[arm] > LAG_M, dropped=dropped, lag=lag,
                                  contact=contact, press=press[arm])
            if self.dexterous:
                self.pressing[arm] = lag if (contact or press[arm]) and lag > CONTACT_STOP_M else 0.0
            out.append(m)
        self.log({"kind": "goto", "arm": "both", "by": _tup(d), "steps": self.rows - start,
                  "halt": [m.halt for m in out]})
        return out

    def _both_refused(self, bad, curs, q_now, arm_base, d, contact) -> dict[str, list[str]]:
        """The notes of a two-hand move by ``d`` refused because the lines of the hands in ``bad`` fail (native
        runtimes): each other hand's line is free, so it can go alone; and, on the last hand refused, how far both
        lines lead together (an offset checked as offered, in whole mm toward zero)."""
        inv = invert(arm_base)
        extra = ", contact=True" if contact else ""

        def together(v) -> bool:
            for arm in sorted(self.arms, key=lambda a: a not in bad):  # the refused hands first: they fail first
                target = curs[arm].copy()
                target[:3, 3] = curs[arm][:3, 3] + v
                sol = self._solve(arm, inv @ target, q_now[arm], quick=True)
                if self._line(arm, curs[arm], target, q_now[arm], arm_base,
                              q_goal=sol.q if sol.reached else None) is None:
                    return False
            return True

        lo, hi = 0.0, 1.0
        for _ in range(BOTH_HALVINGS):
            mid = 0.5 * (lo + hi)
            if together(mid * d):
                lo = mid
            else:
                hi = mid
        told = {arm: [(f"the {arm} hand's own line is free: goto('{arm}', by={_xyz(d)}{extra}) moves it alone "
                       "('both' moves both hands or neither)")] for arm in self.arms if arm not in bad}
        offer = np.trunc(lo * d * 1000.0) / 1000.0 + 0.0  # + 0.0: no "-0.000"
        dist, share = float(np.linalg.norm(d)), float(np.linalg.norm(offer))
        if share >= BOTH_LEAST_M and together(offer):
            last = (f"together both hands can go {100 * share:.0f} of the {100 * dist:.0f} cm: "
                    f"goto('both', by={_xyz(offer)}{extra})")
        else:
            last = f"together both hands cannot go even {100 * BOTH_LEAST_M:.0f} cm of the way"
        told.setdefault(bad[-1], []).append(last)
        return told

    # -- gripper -----------------------------------------------------------------------------------
    def gripper(self, arm, opening):
        arm = check_arm(arm, self.arms)
        if self.dexterous:
            self._check_opening(arm)
            return self.hand(arm, opening=opening)
        g = opening_of(opening)
        value = self._grip_value(g)
        was_holding = self._holding[arm]
        before = float(self.link.command["grip"][arm][0])
        self._grip_asked(arm, g)
        start = self.rows
        n = grip_rows(self.hand_model, before, float(value[0]), self.row_s)
        rows = [self._row() for _ in range(n)]
        for row in rows:
            row["grip"][arm] = value
        frame = self._send(rows)
        r = frame.gripper(arm)
        notes = []
        if g <= 0.05:
            holding = self._closed(arm, r)
            gap = self.hand_model.gap_m(r)
            if holding and gap < THIN_HOLD_M:
                notes.append(f"only ~{100 * gap:.1f} cm between the pads: something thin, or the fingertips came "
                             "down on top of it instead of around it - look at the wrist picture before lifting")
        else:
            holding = None
            if was_holding:
                notes.append("let go of what it held")
        grip = R.Grip(arm=arm, opening=self.opening_read(r), width_m=round(self.hand_model.gap_m(r), 4), holding=holding,
                      steps=self.rows - start, goal=g, notes=notes)
        self.log({"kind": "gripper", "arm": arm, "goal": g, "reading": round(r, 4), "holding": holding,
                  "steps": grip.steps})
        return grip

    # -- a five-finger hand ------------------------------------------------------------------------------
    def hand(self, arm, shape=None, opening=None, *, fingers=None, dry_run=False) -> D.Hand:
        """A five-finger hand's shape (:mod:`.wuji`: 'flat', 'pinch', 'wrap', 'point'; None: as it is), its opening
        (a pinch's or a wrap's, 0..1; None: as it was, open on a change of shape) and fingers set on their own
        ({finger: a curl 0..1 or four joint angles in degrees}; a change of shape drops them), in one motion
        (:mod:`.dexterous`).  The arm stays; its grasp point becomes the new shape's working point."""
        if not self.dexterous:
            raise Refused("this robot's hands are two-finger grippers: gripper(arm, opening) opens and closes them")
        arm = check_arm(arm, self.arms)
        if shape is None and opening is None and fingers is None:
            raise Refused("hand(arm, shape, opening, fingers=...): give a shape ('flat', 'pinch', 'wrap' or 'point'), "
                          "an opening (0..1: a pinch's or a wrap's) or fingers")
        st = self.hands[arm]
        name = st.shape if shape is None else D.shape_name(shape)
        sh = wuji.SHAPES[name]
        notes = []
        if opening is not None and sh.fixed:
            if shape is None:
                raise Refused(f"a {name} hand has no opening (it is one pose): 'pinch' and 'wrap' open and close")
            # the shape asked is clear: take it, without the opening (raising here would leave the rest
            # of that reply's code unrun)
            notes.append(f"a {name} hand has no opening (it is one pose): the opening was left out")
            opening = None
        changed = name != st.shape
        opened_from = None if changed else st.opening
        if sh.fixed:
            g = None
        elif opening is not None:
            g = opening_of(opening)
        else:
            g = 1.0 if changed or st.opening is None else st.opening
        own = {} if changed else dict(st.fingers)
        if fingers is not None:
            own.update(D.finger_targets(fingers, arm))
        target = D.targets(arm, sh, g, own)
        before = np.asarray(self.link.command["grip"][arm], float)
        model = kinematics.arm_models(self.robot, name)[arm] if changed else self.models[arm]
        if dry_run or (not changed and float(np.max(np.abs(target - before))) < 1e-4):
            pose = self.frame.end_link(arm) @ model.grasp_offset
            angles = _tidy(shown_angles(model.tool_rotation(pose[:3, :3])))
            seq = D.path(before, target, self.row_s)
            notes += [] if dry_run else ["already so: nothing was sent"]
            return D.Hand(arm=arm, shape=name, opening=g, steps=len(seq) if dry_run else 0,
                          was=st.shape if changed else "", fingers=tuple(own), touching=st.touching,
                          holding=self._holding[arm], holders=st.holders, at=_tup(pose[:3, 3]), angles=angles,
                          dry_run=bool(dry_run), notes=notes, untold=not self.hands_feel)
        seq = D.path(before, target, self.row_s)
        rows = [self._row() for _ in seq]
        for row, q in zip(rows, seq):
            row["grip"][arm] = q
        start = self.rows
        was = st.shape
        st.shape, st.opening, st.fingers, st.target = name, g, own, target
        self.grip_goal[arm] = 1.0 if g is None else g
        if changed:
            self._set_shape(arm, name)
        frame = self._send(rows)
        dropped = [a for a in self.arms if a != arm and self._slipped(a, frame)]
        self._judge(arm, frame)
        pose = self.grasp_pose(arm)
        if changed:
            notes.append(f"the grasp point is now {D.WORKING[name]} (goto places that point, and its pitch, yaw "
                         f"and roll are this shape's); the wrist did not move")
        if dropped:
            notes.insert(0, f"what the {dropped[0]} hand held slipped out meanwhile")
        reading = frame.effector(arm)
        holding = self._holding[arm]
        gap = round(wuji.pinch_gap(arm, reading), 4) if name == "pinch" else None
        # opened (fully, or wider than it was): an empty hand is open, not a close that missed
        opened = g is not None and (g >= OPEN_HAND or (opened_from is not None and g > opened_from + 1e-3))
        if opened and holding is False:
            holding = None
        if (holding and gap is not None and self.gap_tells
                and gap > wuji.pinch_gap(arm, target) + WIDER_THAN_OPENING_M):
            # (take_the_book: "holding" with the pads 10.4 cm apart, the fingers held back by a book)
            notes.append(f"the pads are {100 * gap:.1f} cm apart, wider than this opening's "
                         f"{100 * wuji.pinch_gap(arm, target):.1f} cm: something holds the fingers apart, which is "
                         "not a grasp")
        result = D.Hand(arm=arm, shape=name, opening=g, steps=self.rows - start, was=was if changed else "",
                        fingers=tuple(own), touching=st.touching, holding=holding, holders=st.holders,
                        gap_m=gap, at=_tup(pose[:3, 3]), angles=self.angles_now(arm, self.asked[arm]), notes=notes,
                        halt="Dropped" if dropped else "", opened=opened, untold=not self.hands_feel)
        self.log({"kind": "hand", "arm": arm, "shape": name, "was": was, "opening": g, "fingers": sorted(own),
                  "steps": result.steps, "touching": list(st.touching), "holding": self._holding[arm],
                  "error": [round(float(v), 3) for v in reading - target]})
        return result

    # -- home --------------------------------------------------------------------------------------
    def home(self, arm="both") -> R.Homing:
        arms = self.arms if arm == "both" else (check_arm(arm, self.arms),)
        start = self.rows
        notes = []
        frame = self.frame
        q_now = {a: frame.arm_joints(a) for a in arms}
        # first: lift the hands that are low, along straight lines
        lifts = {}
        for a in arms:
            cur = self.grasp_of(frame, a)
            if cur[2, 3] < self.start_pose[a][2, 3] - HOME_LIFT_M:
                up = cur.copy()
                up[2, 3] += HOME_LIFT_M
                plan = self._line(a, cur, up, q_now[a], frame.arm_base)
                if plan is None:
                    notes.append(f"the {a} hand could not lift straight up first")
                else:
                    lifts[a] = plan
        timed: list[dict] = []
        q_mid = dict(q_now)
        if lifts:
            n = max(len(p) for p in lifts.values())
            paths = {a: self._pad([q_now[a]] + lifts.get(a, []), n + 1) for a in arms}
            timed += self._timed(paths, slow_end=False)
            q_mid = {a: paths[a][-1] for a in arms}
        # then: swing back to the start joints (both arms: the waist too)
        swings = {a: self._swing(q_mid[a], self.start_q[a]) for a in arms}
        waist_path = None
        w, w0 = frame.waist, self.link.first.waist
        if arm == "both" and not self.fixed_body and w is not None and w0 is not None and len(w) > WAIST_YAW:
            w_from = np.asarray(self.link.command["waist"] if self.link.command.get("waist") is not None else w, float)
            w_to = np.asarray(w0, float)
            if float(np.max(np.abs(w_to - w_from))) >= 1e-3:
                waist_path = (w_from, w_to)
        n = max([len(s) for s in swings.values()] + [0])
        if waist_path is not None:
            n = max(n, math.ceil(float(np.max(np.abs(waist_path[1] - waist_path[0]))) / SWING_SAMPLE_RAD))
        if n:
            paths = {a: self._pad([q_mid[a]] + swings[a], n + 1) if swings[a] else [q_mid[a]] * (n + 1)
                     for a in arms}
            if any(swings[a] for a in arms):
                # the arms in proportion (the shorter one stretched over the same rows)
                for a in arms:
                    if swings[a]:
                        paths[a] = [q_mid[a] + (self.start_q[a] - q_mid[a]) * (i / n) for i in range(n + 1)]
            waist = None
            if waist_path is not None:
                waist = [waist_path[0] + (waist_path[1] - waist_path[0]) * (i / n) for i in range(n + 1)]
            timed += self._timed(paths, slow_end=False, waist=waist)
        dropped = False
        if timed:
            hint = ("home('both') raises the torso with the arms" if arm != "both"
                    else "lift the hand first (goto with by=), or raise the torso (body)")
            self._floor_check("home", [t.get("waist") for t in timed], {a: [t[a] for t in timed] for a in arms},
                              hint=hint)
            rows = self._chunk(timed)
            if waist_path is not None:
                turn = float(waist_path[1][WAIST_YAW] - waist_path[0][WAIST_YAW])
                settle = self._waist_settle_rows(turn)
                if float(np.max(np.abs(waist_path[1][:3] - waist_path[0][:3]))) >= 1e-3:
                    settle = max(settle, rows_of_time(BODY_SETTLE_S, self.row_s))  # the body's posture changed
                rows += [copy_row(rows[-1]) for _ in range(settle)]
            frame = self._send(rows)
            for a in arms:
                dropped |= self._slipped(a, frame)
        for a in arms:
            self.asked[a] = self.angles_now(a, self.start_angles[a])
            self.pressing[a] = 0.0
        out = {}
        for a in arms:
            dist, turn = self.home_offset(a)
            out[a] = (round(1000 * dist, 1), turn)
        if dropped:
            notes.insert(0, "what a gripper held slipped out on the way")
        self.log({"kind": "home", "arms": list(arms), "steps": self.rows - start, "off": out})
        return R.Homing(arms=out, steps=self.rows - start, notes=notes, dropped=dropped)

    # -- wait --------------------------------------------------------------------------------------
    def wait(self, n: int) -> int:
        if isinstance(n, bool) or not isinstance(n, (int, np.integer)) or not 1 <= n <= WAIT_MAX:
            raise Refused(f"wait(n): n is a whole number of steps, 1..{WAIT_MAX}")
        n = int(n)
        frame = self._send([self._row() for _ in range(n)])
        for a in self.arms:
            self._slipped(a, frame)
        return n

    # -- waist -------------------------------------------------------------------------------------
    def waist(self, yaw=None, *, by=None) -> str:
        if self.fixed_body:
            raise Refused("the torso does not turn in this task: there is no waist to move")
        if not self.turns_waist:
            raise Refused("this robot's torso does not turn: there is no waist to move")
        w = self.frame.waist
        if w is None or len(w) <= WAIST_YAW:
            raise Refused("the waist's readings are not available in this episode")
        if (yaw is None) == (by is None):
            raise Refused("waist(yaw=degrees) turns the torso to that heading (0 = the start's), waist(by=degrees) "
                          "turns it by that much; positive turns left")
        try:
            value = float(yaw if yaw is not None else by)
        except (TypeError, ValueError):
            raise Refused("waist: the angle is a number of degrees") from None
        if not math.isfinite(value):
            raise Refused("waist: the angle is a number of degrees")
        base = np.asarray(self.link.command["waist"] if self.link.command.get("waist") is not None else w, float).copy()
        now = float(w[WAIST_YAW])
        start_yaw = float(self.link.first.waist[WAIST_YAW]) if self.link.first.waist is not None else 0.0
        goal = start_yaw + math.radians(value) if yaw is not None else now + math.radians(value)
        lim = math.radians(WAIST_LIMIT_DEG)
        clipped = abs(goal) > lim
        goal = max(-lim, min(lim, goal))
        start = self.rows
        dropped = False
        if abs(goal - now) > 1e-4:
            to = base.copy()
            to[WAIST_YAW] = goal
            frm = base.copy()
            frm[WAIST_YAW] = now
            costs = [abs(goal - now) / (WAIST_RAD_S * self.row_s)]
            rows = []
            for _, f in profile(costs, ramp=self.ramp_rows):
                row = self._row()
                row["waist"] = frm + f * (to - frm)
                rows.append(row)
            rows += [copy_row(rows[-1]) for _ in range(self._waist_settle_rows(goal - now))]
            frame = self._send(rows)
            for a in self.arms:
                dropped |= self._slipped(a, frame)
        for a in self.arms:  # the hands turned with the torso: keep the angles they have now
            self.asked[a] = self.angles_now(a)
        got = self._waist_yaw(goal)
        hands = "; ".join(f"{a} grasp point now {_xyz(self.grasp_pose(a)[:3, 3])}" for a in self.arms)
        text = (f"Waist: yaw {math.degrees(got - start_yaw):.0f}° from the start's ({math.degrees(got - now):+.0f}°), "
                f"{self.rows - start} steps; the arms turned with it: {hands}")
        if clipped:
            text += f"\n  note: the waist turns at most {WAIST_LIMIT_DEG:.0f}° either way"
        if abs(goal - got) > math.radians(2.0):
            text += (f"\n  note: the waist reads {math.degrees(abs(goal - got)):.0f}° short of the heading asked: "
                     "something may be in the way of an arm or the torso, or it is still turning (then the arms "
                     "move on with it) - look before a precise move")
        if dropped:
            text += "\n  note: what a gripper held slipped out on the way"
        self.log({"kind": "waist", "goal_rad": round(goal, 4), "read_rad": round(got, 4), "steps": self.rows - start})
        return text

    def _waist_settle_rows(self, turn_rad: float) -> int:
        """The rows held after a waist turn of ``turn_rad`` was sent, for the waist to get there."""
        s = WAIST_SETTLE_S
        if self.robot in SLOW_WAIST_ROBOTS:
            s = SLOW_WAIST_SETTLE_S + SLOW_WAIST_SETTLE_S_PER_RAD * abs(float(turn_rad))
        return rows_of_time(s, self.row_s)

    def _waist_yaw(self, default: float) -> float:
        w = self.frame.waist
        return float(w[WAIST_YAW]) if w is not None and len(w) > WAIST_YAW else default

    # -- the chassis -------------------------------------------------------------------------------
    def drive(self, forward=0.0, left=0.0, turn=0.0) -> str:
        if not self.mobile:
            raise Refused("this robot's base does not drive in this task: there is no chassis to move")
        asked = {}
        for name, value in (("forward", forward), ("left", left), ("turn", turn)):
            try:
                v = float(value)
            except (TypeError, ValueError):
                v = math.nan
            if isinstance(value, bool) or not math.isfinite(v):
                raise Refused(f"drive: {name} is a number of {'degrees' if name == 'turn' else 'metres'}")
            asked[name] = v
        dist, turn_deg = math.hypot(asked["forward"], asked["left"]), asked["turn"]
        if dist > DRIVE_MAX_M + 1e-9 or abs(turn_deg) > DRIVE_MAX_DEG + 1e-9:
            raise Refused(f"drive: at most {DRIVE_MAX_M:g} m and {DRIVE_MAX_DEG:g} degrees in one call - drive a "
                          "longer way in parts and look in between")
        if dist < 0.001 and abs(turn_deg) < 0.1:
            raise Refused("drive(forward=m, left=m, turn=degrees): nothing to drive")
        turn_rad = math.radians(turn_deg)
        target = np.array([asked["forward"], asked["left"], turn_rad], float)
        seconds = max(dist / DRIVE_M_S, abs(turn_rad) / DRIVE_RAD_S, DRIVE_MIN_S)
        rows = []
        for _ in range(rows_of_time(seconds, DRIVE_ROW_S)):
            row = self._row()
            row["wheels"] = target.copy()
            rows.append(row)
        start = self.rows
        before = self.frame
        frame = self._send(rows)
        dropped = [a for a in self.arms if self._slipped(a, frame)]
        pushed = {a: self._lag(a, frame, self.link.command[a]) for a in self.arms}
        pushed = {a: m for a, m in pushed.items() if m >= DRIVE_PUSHED_M}
        odo: dict = {}
        try:
            seen = odometry.base_move(before, frame, info=odo)
        except Exception as exc:  # noqa: BLE001 - a drive never fails for want of an estimate
            seen = None
            odo["error"] = f"{type(exc).__name__}: {exc}"[:300]
        self.drives += 1
        odometry.dump(before, frame, self.drives, odo)
        self.moved_by = homogeneous(rot_z(turn_rad), (asked["forward"], asked["left"], 0.0))
        if seen is not None and seen.agree >= DRIVE_TRUST:
            self.moved_by = homogeneous(rot_z(math.radians(seen.turn_deg)), (seen.forward, seen.left, 0.0))
        self.drive_end = self.rows
        asked_text = f"forward {asked['forward']:.2f} m, left {asked['left']:.2f} m, turned {turn_deg:+.0f}°"
        if seen is None:
            got = (f"Drove: {asked_text} (as asked: the head pictures before and after could not tell where the "
                   "base got to - look)")
        else:
            got = f"Drove: asked {asked_text}; the head pictures before and after read {seen.text()}"
        text = (f"{got}, {self.rows - start} steps. The world of the robot calls moved with the base (x forward, "
                "y left from where it stands now): points, Spots and Things from before the drive are stale - "
                "look, find and measure again before reaching")
        if seen is not None:
            off_m = math.hypot(seen.forward - asked["forward"], seen.left - asked["left"])
            off_deg = abs(wrap_deg(seen.turn_deg - turn_deg))
            if math.hypot(seen.forward, seen.left) < DRIVE_STILL_M and abs(seen.turn_deg) < DRIVE_STILL_DEG:
                text += ("\n  note: the base hardly moved: something holds it (a wall or furniture against it, a hand "
                         "pressing on something) - look, and free it before driving again")
            elif off_m > DRIVE_OFF_M + DRIVE_OFF_PER_M * dist or off_deg > DRIVE_OFF_DEG + DRIVE_OFF_PER_DEG * abs(turn_deg):
                text += (f"\n  note: the base ended {off_m:.2f} m and {off_deg:.0f}° from where it was sent: it met "
                         "something or turned on the way - look before going on")
        for arm, miss in pushed.items():
            text += (f"\n  note: the {arm} hand was pushed {100 * miss:.1f} cm off its pose: it met something on the "
                     "way, and the base may have stopped short or turned - look before going on")
        for arm in dropped:
            text += f"\n  note: what the {arm} gripper held slipped out on the way"
        self.log({"kind": "drive", "asked": [round(float(v), 4) for v in target], "steps": self.rows - start,
                  "pushed_mm": {a: round(1000 * m, 1) for a, m in pushed.items()}, "dropped": dropped,
                  "seen": None if seen is None else [round(seen.forward, 4), round(seen.left, 4),
                                                     round(seen.turn_deg, 2), seen.agree, seen.matched],
                  "odometry": odo})
        return text

    # -- the body and the head (the WBC board's G2) ----------------------------------------------------
    def _body_joints(self) -> np.ndarray:
        """The body's five joints as read (refused for a robot without robot.body)."""
        if not (self.mobile and self.robot in BODY_ROBOTS):
            raise Refused("this robot's torso keeps its posture in this task: there is no body to move")
        w = self.frame.waist
        if w is None or len(w) < 5:
            raise Refused("the body's readings are not available in this episode")
        return np.asarray(w, float)[:5].copy()

    def _arm_points(self, arm: str, q) -> np.ndarray:
        """An arm's points that may meet the floor, in arm_base_link: its joints from the third on, the grasp
        point and the fingertips."""
        m = self.models[arm]
        fr = m.frames(q)
        end = fr[-1] @ m.end
        if self.dexterous:  # the fingertips and the palm as last sent
            q_hand = self.hands[arm].target
            hand = [(end @ np.r_[v, 1.0])[:3] for v in wuji.points(arm, q_hand).values()]
            return np.array([f[:3, 3] for f in fr[2:]] + [(fr[-1] @ m.tool)[:3, 3]] + hand)
        tip = end[:3, 3] + end[:3, :3] @ (np.asarray(m.hand.approach_in_end, float) * m.hand.tip_reach)
        return np.array([f[:3, 3] for f in fr[2:]] + [(fr[-1] @ m.tool)[:3, 3], tip])

    def _lowest(self, body_q, arm_q: dict) -> tuple[float, str]:
        """The lowest of the arms' points (base_link z) with these body joints and arm joints, and its arm."""
        ab = body_model.arm_base(body_q)
        low, who = math.inf, ""
        for arm, q in arm_q.items():
            z = float(np.min(self._arm_points(arm, q) @ ab[2, :3] + ab[2, 3]))
            if z < low:
                low, who = z, arm
        return low, who

    def _floor_check(self, what: str, body_path, arm_paths: dict, *, hint: str) -> None:
        """Refuse a motion (``what``) whose arms would come within FLOOR_CLEAR_M of the floor on the way, or lower
        than they start where they start lower (the WBC body).  ``body_path``: the body's joints per row (None:
        as they are); ``arm_paths``: per arm its joints per row (the last one held after its end)."""
        if not (self.mobile and self.robot in BODY_ROBOTS):
            return
        w = self.frame.waist
        if w is None or len(w) < 5:
            return
        now = np.asarray(w, float)[:5]
        body_path = list(body_path or [None])
        n = max([len(body_path)] + [len(p) for p in arm_paths.values()])

        def at(k):
            qb = body_path[min(k, len(body_path) - 1)]
            qb = now if qb is None else np.asarray(qb, float)[:5]
            return self._lowest(qb, {a: p[min(k, len(p) - 1)] for a, p in arm_paths.items()})

        start, _ = at(0)
        limit = min(FLOOR_Z + FLOOR_CLEAR_M, start - 0.01)
        for k in range(n):
            z, arm = at(k)
            if z < limit:
                raise Refused(f"{what}: on the way the {arm} arm would come down to z {z:.2f}, "
                              f"{100 * (z - FLOOR_Z):.0f} cm above the floor (z {FLOOR_Z:.2f}): {hint}")

    def _head_view(self, frame) -> str:
        """Where the head camera looks (from its calibration), or ''."""
        calib = (getattr(frame, "calib", None) or {}).get("head")
        if calib is None:
            return ""
        pos, axis = calib.pose[:3, 3], calib.pose[:3, 2]
        down = math.degrees(math.asin(max(-1.0, min(1.0, -float(axis[2])))))
        if down >= 0.5:
            text = f"looks about {down:.0f}° down"
        else:
            text = f"looks about {-down:.0f}° up"
        if axis[0] < -0.05:
            text += " and back toward the base"
        if axis[2] < -0.05:
            t = (FLOOR_Z - float(pos[2])) / float(axis[2])
            spot = pos + t * axis
            text += f", the middle of its picture on the floor at about ({spot[0]:.2f}, {spot[1]:.2f})"
        return text

    def posture_text(self) -> str:
        """The body's posture and the head's as read (robot.body() without a posture)."""
        w = self._body_joints()
        now = body_model.posture(w)
        text = (f"Body: torso leaning {round(math.degrees(now.lean))}° forward, shoulders at z {now.shoulder_z:.2f}, "
                f"{now.shoulder_x:.2f} m ahead of the base's centre")
        h = self.frame.head
        if h is not None and len(h) >= 3:
            text += (f"; head turned {round(math.degrees(float(h[body_model.HEAD_TURN]))):+d}°, tipped "
                     f"{round(math.degrees(float(h[body_model.HEAD_TIP])))}° down")
        view = self._head_view(self.frame)
        if view:
            text += f"; the head camera {view}"
        return text

    def _no_posture(self, lean_deg: float, z: float, ref: np.ndarray, asked: dict) -> str:
        parts = []
        hs = body_model.heights(math.radians(lean_deg), ref)
        if hs is not None:
            parts.append(f"at a {lean_deg:.0f}° lean the shoulders go from z {hs[0]:.2f} to {hs[1]:.2f}")
        if "height" in asked:
            lo, hi = BODY_LEAN_DEG
            ls = body_model.leans(z, ref, lo, hi)
            if ls is not None:
                parts.append(f"with the shoulders at z {z:.2f} the torso leans {math.degrees(ls[0]):.0f} to "
                             f"{math.degrees(ls[1]):.0f}°")
            else:
                parts.append(f"the shoulders reach z {z:.2f} at no lean from {lo:.0f} to {hi:.0f}°")
        return f"body: no posture leans {lean_deg:.0f}° with the shoulders at z {z:.2f}: " + "; ".join(parts)

    def body(self, lean=None, height=None, *, dry_run=False) -> str:
        w = self._body_joints()
        if lean is None and height is None:
            if dry_run:
                raise Refused("body(lean=degrees, height=m, dry_run=True): name the posture to try")
            return self.posture_text()
        asked = {}
        for name, value in (("lean", lean), ("height", height)):
            if value is None:
                continue
            try:
                v = float(value)
            except (TypeError, ValueError):
                v = math.nan
            if isinstance(value, bool) or not math.isfinite(v):
                raise Refused(f"body: {name} is a number of {'degrees' if name == 'lean' else 'metres (z)'}")
            asked[name] = v
        now = body_model.posture(w)
        lean_deg = asked.get("lean", math.degrees(now.lean))
        lo, hi = BODY_LEAN_DEG
        if not lo - 1e-9 <= lean_deg <= hi + 1e-9:
            raise Refused(f"body: the torso leans {lo:.0f} to {hi:.0f} degrees forward (now "
                          f"{math.degrees(now.lean):.0f})")
        z = asked.get("height", now.shoulder_z)
        command = self.link.command.get("waist")
        base = np.asarray(command if command is not None else w, float).copy()
        ref = np.r_[w[:3], base[3:5]]
        q = body_model.solve(math.radians(lean_deg), z, ref)
        if q is None:
            raise Refused(self._no_posture(lean_deg, z, ref, asked))
        frm, to = base.copy(), base.copy()
        frm[:3], to[:3] = w[:3], q[:3]
        move = float(np.max(np.abs(to[:3] - frm[:3])))
        path = [frm + f * (to - frm) for _, f in profile([move / (BODY_RAD_S * self.row_s)], ramp=self.ramp_rows)]
        held = {a: [np.asarray(self.link.command[a], float)] for a in self.arms}
        self._floor_check("body", path, held, hint="lift the hand first (goto with by=), or lean less or keep the "
                                                   "shoulders higher")
        settle = rows_of_time(BODY_SETTLE_S, self.row_s)
        if dry_run:
            ab = body_model.arm_base(to)
            ends = "; ".join(f"{a} grasp point {_xyz((ab @ self.models[a].grasp_pose(held[a][0]))[:3, 3])}"
                             for a in self.arms)
            return (f"Body (dry run, nothing moved): torso leaning {lean_deg:.0f}° forward, shoulders at z {z:.2f}, in "
                    f"{len(path) + settle} steps; the arms, held as they are, would end at: {ends}")
        if move < 1e-4:
            return self.posture_text()
        rows = []
        for qb in path:
            row = self._row()
            row["waist"] = qb
            rows.append(row)
        rows += [copy_row(rows[-1]) for _ in range(settle)]
        start = self.rows
        frame = self._send(rows)
        dropped = [a for a in self.arms if self._slipped(a, frame)]
        pushed = {a: self._lag(a, frame, self.link.command[a]) for a in self.arms}
        pushed = {a: m for a, m in pushed.items() if m >= DRIVE_PUSHED_M}
        for a in self.arms:  # the hands tipped with the torso: keep the angles they have now
            self.asked[a] = self.angles_now(a)
        got_q = np.asarray(frame.waist, float)[:5] if frame.waist is not None and len(frame.waist) >= 5 else to[:5]
        got = body_model.posture(got_q)
        hands = "; ".join(f"{a} grasp point now {_xyz(self.grasp_pose(a)[:3, 3])}" for a in self.arms)
        text = (f"Body: torso leaning {round(math.degrees(got.lean))}° forward, shoulders at z {got.shoulder_z:.2f} "
                f"(asked {lean_deg:.0f}°, z {z:.2f}), {self.rows - start} steps; the arms moved with it: {hands}")
        view = self._head_view(frame)
        if view:
            text += f"\n  the head camera {view}"
        off = float(np.max(np.abs(got_q[:3] - to[:3])))
        if off > BODY_OFF_RAD:
            text += (f"\n  note: a body joint reads {math.degrees(off):.0f}° off the posture asked: something may hold a "
                     "hand or the torso, or it is still moving - look before a precise move")
        for arm, miss in pushed.items():
            text += (f"\n  note: the {arm} hand was pushed {100 * miss:.1f} cm off its pose: it met something on the "
                     "way - look before going on")
        for arm in dropped:
            text += f"\n  note: what the {arm} gripper held slipped out on the way"
        self.log({"kind": "body", "asked": [round(lean_deg, 2), round(z, 4)], "goal": [round(float(v), 4) for v in to[:3]],
                  "read": [round(float(v), 4) for v in got_q[:3]], "steps": self.rows - start,
                  "pushed_mm": {a: round(1000 * m, 1) for a, m in pushed.items()}, "dropped": dropped})
        return text

    def head(self, yaw=None, pitch=None) -> str:
        if not (self.mobile and self.robot in BODY_ROBOTS):
            raise Refused("this robot's head keeps its pose in this task: there is no head to move")
        h = self.frame.head
        if h is None or len(h) < 3:
            raise Refused("the head's readings are not available in this episode")
        lo, hi = HEAD_PITCH_DEG
        if yaw is None and pitch is None:
            raise Refused(f"head(yaw=degrees, pitch=degrees): yaw turns the head left (+) or right (-) on the torso, up "
                          f"to {HEAD_YAW_DEG:.0f}°; pitch tips it down from the torso's line, {lo:.0f} to {hi:.0f}°")
        h = np.asarray(h, float)[:3]
        command = self.link.command.get("head")
        to = np.asarray(command if command is not None else h, float)[:3].copy()
        notes = []
        for name, value, j, (a, b) in (("yaw", yaw, body_model.HEAD_TURN, (-HEAD_YAW_DEG, HEAD_YAW_DEG)),
                                       ("pitch", pitch, body_model.HEAD_TIP, (lo, hi))):
            if value is None:
                continue
            try:
                v = float(value)
            except (TypeError, ValueError):
                v = math.nan
            if isinstance(value, bool) or not math.isfinite(v):
                raise Refused(f"head: {name} is a number of degrees")
            if not a <= v <= b:
                notes.append(f"the head's {name} goes from {a:.0f} to {b:.0f}°: {v:.0f}° became "
                             f"{min(b, max(a, v)):.0f}°")
                v = min(b, max(a, v))
            to[j] = math.radians(v)
        frm = h.copy()
        move = float(np.max(np.abs(to - frm)))
        start = self.rows
        frame = self.frame
        if move >= 1e-4:
            rows = []
            for _, f in profile([move / (HEAD_RAD_S * self.row_s)], ramp=self.ramp_rows):
                row = self._row()
                row["head"] = frm + f * (to - frm)
                rows.append(row)
            rows += [copy_row(rows[-1]) for _ in range(rows_of_time(HEAD_SETTLE_S, self.row_s))]
            frame = self._send(rows)
            for a in self.arms:
                if self._slipped(a, frame):
                    notes.append(f"what the {a} gripper held slipped out meanwhile")
        got = np.asarray(frame.head, float)[:3] if frame.head is not None and len(frame.head) >= 3 else to
        text = (f"Head: turned {round(math.degrees(float(got[body_model.HEAD_TURN]))):+d}°, tipped "
                f"{round(math.degrees(float(got[body_model.HEAD_TIP])))}° down on the torso, {self.rows - start} steps")
        view = self._head_view(frame)
        if view:
            text += f"; its camera {view}"
        if float(np.max(np.abs(got - to))) > math.radians(3.0):
            notes.append("the head reads more than 3° off the pose asked - look before relying on it")
        for note in notes:
            text += f"\n  note: {note}"
        self.log({"kind": "head", "goal": [round(float(v), 4) for v in to], "read": [round(float(v), 4) for v in got],
                  "steps": self.rows - start})
        return text

    # -- status --------------------------------------------------------------------------------------
    def arm_status(self, arm: str) -> R.ArmStatus:
        pose = self.grasp_pose(arm)
        dist, turn = self.home_offset(arm)
        if self.dexterous:
            st = self.hands[arm]
            gap = wuji.pinch_gap(arm, self.frame.effector(arm)) if st.shape == "pinch" else None
            return D.HandStatus(arm=arm, at=_tup(pose[:3, 3]), angles=self.angles_now(arm),
                                gripper=1.0 if st.opening is None else float(st.opening),
                                goal=float(self.grip_goal[arm]), holding=self._holding[arm],
                                joints=_tup(self.frame.arm_joints(arm)), home_dist_m=dist, home_turn_deg=turn,
                                hand=D.state_words(st, self._holding[arm], gap, untold=not self.hands_feel))
        return R.ArmStatus(arm=arm, at=_tup(pose[:3, 3]), angles=self.angles_now(arm),
                           gripper=self.opening_read(self.frame.gripper(arm)), goal=float(self.grip_goal[arm]),
                           holding=self._holding[arm], joints=_tup(self.frame.arm_joints(arm)),
                           home_dist_m=dist, home_turn_deg=turn)

    def start_info(self) -> dict:
        return {arm: (_tup(self.start_pose[arm][:3, 3], 3), self.start_angles[arm]) for arm in self.arms}


__all__ = ["RobotDriver", "check_arm", "opening_of", "profile", "vector_of"]
