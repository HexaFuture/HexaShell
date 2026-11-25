"""The five-finger hands of the robot calls (dexmanip: the G2's Wuji hands, :mod:`.wuji`): their shapes, how a
hand motion is paced, and what the readings after it say (touching, holding, lost).

A hand is in one of the shapes of :mod:`.wuji` (it starts flat).  ``robot.hand(arm, shape, opening, fingers=)``
moves its 20 joints there; ``gripper(arm, opening)`` and ``goto(grip=)`` set the opening of a pinch or a wrap.

- Pace: the joint targets move along a straight line (all joints in proportion) at up to ``HAND_RAD_S`` for the
  joint that moves most (at least ``HAND_MIN_S``), then ``HAND_SETTLE_S`` of rows hold the end: the simulator
  waits for the arms after a chunk, never for the hands, and the drives' lag (0.1 s) is spent in that hold.
- Contact: after a hand motion, a finger whose reading stays more than ``TOUCH_RAD`` from its target in some joint
  was stopped by something (and squeezes it: the drives are stiff, their torque capped).  A pinch holds when the
  thumb and the index finger both stopped with their pads ``HOLD_GAP_M`` or more apart, a wrap when at least
  ``WRAP_HOLDERS`` of its four fingers stopped; flat and point hold nothing (they push, press, lift on the palm).
- Lost: a holding finger that came ``SLIP_RAD`` or more nearer its target since it caught, or reached it, lets
  go; a hold whose fingers no longer stand (``still_holds``) is lost (the motion reports Dropped).
- The arm's grasp point is the shape's working point (:class:`.wuji.Shape`), so a change of shape moves it while
  the wrist stays: the driver's arm model changes with the shape (``kinematics.arm_models(robot, shape)``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ...errors import Refused
from ...robot import results as R
from ...robot.results import _deg, _xyz
from . import wuji

#: the pace of the joint that moves most (rad/s), the shortest hand motion and the hold after it (s)
HAND_RAD_S = 3.0
HAND_MIN_S = 1.0 / 6.0
HAND_SETTLE_S = 0.3
#: a joint this far (rad) from its target after a hand motion was stopped by something
TOUCH_RAD = 0.06
#: a holding finger this much (rad) nearer its target than when it caught has let go
SLIP_RAD = 0.1
#: the pinch's pads at least this far apart (m) when they stop: something between them
HOLD_GAP_M = 0.003
#: a wrap holds with at least this many of its four fingers stopped on what it holds
WRAP_HOLDERS = 2
#: what each shape's working point (the grasp point while in it) is, for the results
WORKING = {"flat": "the middle of the palm", "pinch": "where the pads meet",
           "wrap": "the middle of the curled fingers", "point": "the index fingertip"}


def shape_name(value) -> str:
    try:
        return wuji.shape_of(value).name
    except ValueError as exc:
        raise Refused(str(exc)) from None


def finger_targets(fingers, side: str) -> dict[str, np.ndarray]:
    """{finger: its four joint targets on this side} from ``fingers=`` ({finger: curl 0..1 or four degrees})."""
    if not isinstance(fingers, dict) or not fingers:
        raise Refused("fingers is a dict {finger: a curl 0..1, or its four joint angles in degrees}, the fingers being "
                      + ", ".join(wuji.FINGERS))
    out = {}
    for f, v in fingers.items():
        try:
            out[str(f)] = wuji.finger_joints(str(f), v, side)
        except ValueError as exc:
            raise Refused(str(exc)) from None
    return out


@dataclass
class HandState:
    """What a hand was last sent, and what holds."""

    shape: str = wuji.START_SHAPE
    opening: float | None = None  # a pinch's or a wrap's: 0 closed .. 1 open
    fingers: dict = field(default_factory=dict)  # fingers set on their own: this side's joints (rad)
    target: np.ndarray | None = None  # the 20 joint targets last sent
    touching: tuple = ()  # fingers stopped short of their targets after the last hand motion
    holders: tuple = ()  # the fingers that hold (while it holds)
    held: np.ndarray | None = None  # the readings when it caught


def targets(side: str, shape: wuji.Shape, opening: float | None, fingers: dict) -> np.ndarray:
    """The 20 joint targets of a shape at an opening (None: open), the fingers set on their own replaced."""
    q = np.array(shape.joints(side, 1.0 if opening is None else opening), float)
    for f, v in fingers.items():
        i = wuji.FIRST[f]
        q[i:i + 4] = v
    return wuji.clamp(q)


def path(before, after, row_s: float) -> list[np.ndarray]:
    """Per row, the joint targets from ``before`` to ``after``: a straight line at HAND_RAD_S (at least
    HAND_MIN_S), then HAND_SETTLE_S of the end."""
    a = np.asarray(before, float).reshape(20)
    b = np.asarray(after, float).reshape(20)
    span = float(np.max(np.abs(b - a)))
    n = max(1, math.ceil(HAND_MIN_S / row_s - 1e-9), math.ceil(span / (HAND_RAD_S * row_s) - 1e-9))
    settle = max(1, math.ceil(HAND_SETTLE_S / row_s - 1e-9))
    return [a + (b - a) * (k / n) for k in range(1, n + 1)] + [b.copy() for _ in range(settle)]


def stopped(target, reading) -> tuple[str, ...]:
    """The fingers with a joint more than TOUCH_RAD from its target."""
    err = np.abs(np.asarray(target, float).reshape(20) - np.asarray(reading, float).reshape(20))
    return tuple(f for f in wuji.FINGERS if float(np.max(err[wuji.FIRST[f]:wuji.FIRST[f] + 4])) > TOUCH_RAD)


def holds(shape: wuji.Shape, side: str, touching, reading) -> tuple[bool | None, tuple]:
    """(whether the hand holds something - None for a shape that holds nothing -, the fingers that hold it)."""
    if not shape.holders:
        return None, ()
    holders = tuple(f for f in shape.holders if f in touching)
    if shape.name == "pinch":
        ok = len(holders) == 2 and wuji.pinch_gap(side, reading) >= HOLD_GAP_M
    else:
        ok = len(holders) >= WRAP_HOLDERS
    return (True, holders) if ok else (False, ())


def still_holds(shape: wuji.Shape, side: str, state: HandState, reading) -> bool:
    """Whether a hold stands: enough of its fingers still stopped where they caught (none SLIP_RAD nearer its
    target), and a pinch's pads still apart."""
    t = np.asarray(state.target, float).reshape(20)
    h = np.asarray(state.held, float).reshape(20)
    r = np.asarray(reading, float).reshape(20)
    still = []
    for f in state.holders:
        s = slice(wuji.FIRST[f], wuji.FIRST[f] + 4)
        nearer = float(np.max(np.abs(t[s] - h[s]) - np.abs(t[s] - r[s])))
        if nearer < SLIP_RAD and float(np.max(np.abs(t[s] - r[s]))) > TOUCH_RAD:
            still.append(f)
    if shape.name == "pinch":
        return len(still) == 2 and wuji.pinch_gap(side, r) >= HOLD_GAP_M
    return len(still) >= WRAP_HOLDERS


def _and(names) -> str:
    names = list(names)
    if len(names) <= 1:
        return "".join(names)
    return ", ".join(names[:-1]) + " and " + names[-1]


#: a hand's result where its readings cannot tell what the fingers meet (libuipc's cloth tasks: its hand model has no
#: cloth and its soft drives lag, so the fingers' readings come from the lag alone; Newton's wok: the readings were
#: the same at every close, held or not)
UNTOLD = ("touching or holding cannot be told here: the fingers' readings do not show what they meet in this task, "
          "so judge a grasp from a picture")


def state_words(state: HandState, holding: bool | None, gap_m: float | None = None, *, untold: bool = False) -> str:
    """A hand's state in a few words (the status lines); ``untold``: the readings cannot tell touching or holding
    (:data:`UNTOLD`)."""
    out = state.shape if state.opening is None else f"{state.shape}, opening {state.opening:.2f}"
    if state.fingers:
        out += f" ({_and(state.fingers)} set on their own)"
    if untold:
        return out + ", touching or holding cannot be told here"
    if holding:
        out += f", holding (the {_and(state.holders)} stopped on it"
        out += f", pads {100 * gap_m:.1f} cm apart)" if gap_m is not None else ")"
    elif holding is False:
        out += ", empty"
    others = [f for f in state.touching if f not in (state.holders if holding else ())]
    if others:
        out += f", the {_and(others)} touching something"
    return out


@dataclass
class Hand:
    """What robot.hand() did (and gripper() on a five-finger hand)."""

    arm: str
    shape: str
    opening: float | None
    steps: int = 0
    was: str = ""  # the shape before, when it changed
    fingers: tuple = ()  # the fingers set on their own
    touching: tuple = ()
    holding: bool | None = None
    holders: tuple = ()
    gap_m: float | None = None  # a pinch's pads, as read
    at: tuple | None = None  # the grasp point after (base_link)
    angles: tuple | None = None
    dry_run: bool = False
    notes: list[str] = field(default_factory=list)
    halt: str = ""  # 'Dropped': the other hand lost what it held on the way
    opened: bool = False  # the motion opened the hand (fully, or wider than it was): nothing held is "open"
    steps_left: int | None = None  # the native runtimes: the steps left of the episode's span (as the shared results)
    untold: bool = False  # the readings cannot tell touching or holding (libuipc: UNTOLD in place of those lines)

    def __repr__(self) -> str:
        what = self.shape if self.opening is None else f"{self.shape}, opening {self.opening:.2f}"
        if self.fingers:
            what += f" ({_and(self.fingers)} set on their own)"
        if self.was:
            what += f" (it was {self.was})"
        head = f"Hand {self.arm}: {what}"
        left = "" if self.steps_left is None else f"  [{self.steps_left} steps left]"
        lines = [f"{head}  (dry run: nothing moved, it would take {self.steps} steps)" if self.dry_run
                 else f"{head}  {self.steps} steps{left}"]
        if not self.dry_run and self.untold:
            lines.append(f"  {UNTOLD}")
        elif not self.dry_run:
            gap = "" if self.gap_m is None else f" (pads {100 * self.gap_m:.1f} cm apart)"
            if self.holding:
                lines.append(f"  holding: the {_and(self.holders)} stopped on something{gap}")
            elif self.holding is False:
                lines.append(f"  empty: the {_and(wuji.SHAPES[self.shape].holders)} closed without stopping on "
                             f"anything{gap}")
            elif self.opened:
                lines.append(f"  open: nothing between the {_and(wuji.SHAPES[self.shape].holders)}{gap}")
            others = [f for f in self.touching if f not in (self.holders if self.holding else ())]
            if others:
                lines.append(f"  touching: the {_and(others)} stopped short of where they were sent (something is "
                             "in the way, and they press on it)")
        if self.at is not None:
            p, y, r = self.angles
            moved = " now" if self.was else ""
            lines.append(f"  grasp point{moved} {_xyz(self.at)} pitch {_deg(p)} yaw {_deg(y)} roll {_deg(r)} "
                         f"({WORKING[self.shape]})")
        lines += [f"  note: {n}" for n in self.notes]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass(repr=False)
class HandStatus(R.ArmStatus):
    """An arm's state with its five-finger hand's (``hand``: :func:`state_words`)."""

    hand: str = ""

    def __repr__(self) -> str:
        p, y, r = self.angles
        home = "home" if self.home_dist_m < 0.01 and self.home_turn_deg < 3 else \
            f"{100 * self.home_dist_m:.0f} cm / {self.home_turn_deg:.0f}° from home"
        return (f"{self.arm}: {self.point} {_xyz(self.at)} pitch {_deg(p)} yaw {_deg(y)} roll {_deg(r)}  "
                f"hand {self.hand}  ({home})")

    __str__ = __repr__


# -- the task card --------------------------------------------------------------------------------------------
#: the fingers' thickness (m, about) for the reach past a grasp point
_FINGER_R = 0.010
#: the openings the card gives a pinch's gap of
CARD_OPENINGS = (1.0, 0.8, 0.6, 0.4, 0.2)


def _ahead_cm(shape: wuji.Shape, opening: float) -> float:
    """How far (cm) the hand reaches past the shape's working point along the way it points (right hand)."""
    tcp = shape.point("right")
    approach = shape.tool_in_end("right")[:, 0]
    pts = [p for seg in wuji.skeleton("right", shape.joints("right", opening)) for p in seg]
    return 100.0 * (max(float((p - tcp) @ approach) for p in pts) + _FINGER_R)


def card_lines() -> list[str]:
    """The task card's lines on the five-finger hands."""
    pinch, wrap, flat, point = (wuji.SHAPES[n] for n in ("pinch", "wrap", "flat", "point"))
    gaps = ", ".join(f"{g:g} -> {100 * wuji.pinch_gap('right', pinch.joints('right', g)):.1f} cm" for g in CARD_OPENINGS)
    return [
        "- Hands (robot.hand(arm, shape, opening)): each hand takes one of four shapes - 'flat' (every finger "
        "straight, the thumb beside the index: how both start), 'pinch' (the thumb's pad against the index finger's, "
        f"the other three half curled; the gap between the pads at an opening g: {gaps}), 'wrap' (the four fingers "
        "curled round a bar lying across the palm, the thumb along the index finger; opening 1 fits a bar about "
        f"{100 * (wrap.open_gap_m or 0.0):.0f} cm thick, 0 is a fist) and 'point' (the index finger straight, the "
        "others curled). gripper(arm, opening) and goto's grip= set a pinch's or a wrap's opening.",
        "- Grasp point (the point goto places) - each shape's own, so a change of shape moves it while the wrist "
        f"stays: flat, the middle of the palm ({100 * flat.point('right')[2]:.1f} cm beyond the wrist's end link; "
        f"the fingertips reach {_ahead_cm(flat, 1.0):.0f} cm past it); pinch, where the pads meet (the hand reaches "
        f"{_ahead_cm(pinch, 1.0):.1f} cm past it); wrap, the middle of the curled fingers (they reach "
        f"{_ahead_cm(wrap, 1.0):.1f} cm past it open, {_ahead_cm(wrap, 0.0):.1f} cm closed); point, the index "
        "fingertip (nothing else of the hand within 8 cm behind it).",
        "- Angles: each shape has its own pitch, yaw and roll. Pointing straight down at yaw 0 and roll 0, every "
        "shape closes along y (left-right): the palms face the robot's middle, a pinch's pads meet along y, a "
        "wrap's bar lies along x. At the start both hands point forward, their palms facing each other.",
        "- Holding: fingers stopped by something squeeze it and keep squeezing until changed; a pinch holds what its "
        "thumb and index stop on, a wrap what at least two of its fingers stop on; flat and point hold nothing "
        "(they push, press and lift from below). Between two palms a box is carried by pressing both on its sides "
        "(goto with contact=True, a little into it) and then moving them together with goto('both', by=...), which "
        "keeps them pressing.",
    ]


__all__ = ["Hand", "HandState", "HandStatus", "card_lines", "finger_targets", "holds", "path", "shape_name",
           "state_words", "still_holds", "stopped", "targets"]
