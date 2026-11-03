"""The host side of ``robot.*`` for a one-arm benchmark: checks a cell's calls, sends motions to the
simulator server (:mod:`.armserver`) and answers perception from its RGB-D pictures.

``dispatch`` is what the kernel calls for each robot call.  Errors the model should see come back
as :class:`~hexashell.kernel.host.CallFailed`; once the episode is over (success, the step budget,
a simulator failure) the call answers :class:`~hexashell.kernel.host.Stop` and so does every later
one.  Poses cross to the server as 4x4 tool poses in the episode's frame (x forward, y left, z up;
the angles of :mod:`.pose`).

A benchmark subclasses :class:`ArmHost` with its cameras, finger stroke, control period and the
advice its notes give, and adds what its robot has besides the arm (RoboCasa: the base).
"""

from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

from ..errors import Refused
from ..kernel.host import CallFailed, Stop
from ..robot import results as RR
from ..vision import overlay
from ..vision.cameras import Camera
from ..vision.rgbd import DepthView, spot
from . import results as R
from .pose import angles_of, nearest, rotation_of, shown_angles, shown_near, turn_deg
from .session import SimError, SimRefused

#: what every arm benchmark's robot answers (method ``_<name>``)
ARM_CALLS = ("goto", "arc", "gripper", "home", "wait", "find", "measure", "to_world", "to_pixel", "task", "state",
             "image", "depth", "look", "camera", "things")
#: the fingers stop closing farther apart than this when they hold something (m)
HOLD_MIN_M = 0.004
#: in a light hold (``light_hold``), a gap within this much (m) of the one the fingers are set at: they hold nothing
EASED_EMPTY_M = 0.002
#: a change of the gap this large (m) while holding: what the fingers hold turned or slid
#: a close that ends this share of the stroke apart or wider: the fingers hardly moved
NEARLY_OPEN = 0.9
HELD_SHIFT_M = 0.003
#: a close that holds with the grasp point less than TOP_GRASP_M under the measured top of a thing less than
#: LOW_THING_M high and no wider than the open fingers, or over that top (the thing: the one measured whose centre
#: is nearest the grasp point and within TOP_GRASP_XY_M of it across): closed this near the top of a low thing
#: (see ``top_grasp_note``)
LOW_THING_M = 0.06
TOP_GRASP_M = 0.005
TOP_GRASP_XY_M = 0.06
#: under this height (m), a low thing is best gripped at the height it stands on
FLAT_THING_M = 0.04
#: a grasp point less than this (m) over what a thing stands on is at the surface: pointing down, the fingertips
#: stop on it about 9 mm under the grasp point
SURFACE_M = 0.01
#: a move down with the fingers open that stopped short with the grasp point from STOP_OVER_ON_M over what a
#: thing measured under the hand stands on (the one nearest across, within TOP_GRASP_XY_M) to TOP_GRASP_OVER_M
#: over its top: the fingers rest on it or on something beside it (see ``top_stop_note``)
STOP_OVER_ON_M = 0.012
TOP_GRASP_OVER_M = 0.03
GRIP_WORDS = {"open": 1.0, "close": 0.0, "closed": 0.0}
WAIT_MAX = 400
PRESSING = "the hand stopped behind its plan: it is pressing on something - look before going on"
#: how far the arm pulls the hand back toward the shoulder when it gives way (see arm.GIVE_WAY_M)
GIVE_WAY_CM = 5
#: the speeds (m/s) a goto or arc may ask for: down to arm.MIN_SPEED, at most the arm's own (arm.ARM_SPEED)
SPEEDS = (0.01, 0.25)
#: a hold (during a grip or a wait) strained with the arm stretched out and gave way
GAVE_WAY = ("meanwhile the arm pressed on something at full force with the elbow open wider than a right angle, "
            f"so it pulled the hand {GIVE_WAY_CM} cm back toward the shoulder (pressed on, a stretched arm snaps "
            "straight): the hand moved - look before going on")


def gave_way_how(res: dict) -> str:
    """What the arm did when it gave way as its elbow straightened (see arm.GIVE_WAY_BEND): went back
    by the joints toward where it came from (the server's ``gave_way_back``, the hand moving
    ``gave_way_m``), or pulled the hand in."""
    if res.get("gave_way_back"):
        cm = 100.0 * float(res.get("gave_way_m") or 0.0)
        return f"went back part of the way it came, which bends the elbow (the hand moved {cm:.0f} cm)"
    return f"pulled the hand {GIVE_WAY_CM} cm back toward the shoulder"


def gave_way_notes(res: dict) -> list[str]:
    """The note of a grip or a wait whose hold gave way (the server's ``gave_way`` times, ``stretched``
    of them because the elbow straightened)."""
    if not res.get("gave_way"):
        return []
    if int(res["gave_way"]) > int(res.get("stretched") or 0):
        return [GAVE_WAY]
    return ["meanwhile the arm, held far out, crept on until its elbow was nearly straight (a straight arm gets "
            f"stuck), so it {gave_way_how(res)}: look before going on, and work nearer the arm"]


#: the hand ended this much (m) farther from the target in the start frame than relative to the base:
#: the base was pushed during the motion (the arm holds its pose relative to the base)
SHOVED_M = 0.01
#: the Panda's joints, from the shoulder out
JOINT_NAMES = ("the shoulder's turn", "the shoulder", "the upper arm's twist", "the elbow", "the forearm's twist",
               "the wrist", "the hand's twist")


def limit_note(at_limit) -> str | None:
    """The note of a motion that stopped short with joints at an end of their travel (the server's
    ``at_limit``: [joint number, "low" or "high"])."""
    if not at_limit:
        return None
    joints, elbow = [], None
    ks = {int(k) for k, _ in at_limit}
    for k, side in at_limit:
        k = int(k)
        if k == 4:
            elbow = "straight" if side == "high" else "fully bent"
            joints.append(f"joint 4 (the elbow) {elbow}")
        else:
            name = JOINT_NAMES[k - 1] if 1 <= k <= len(JOINT_NAMES) else "a joint"
            joints.append(f"joint {k} ({name}) at the end of its travel")
    if elbow == "straight":
        advice = "the target is about as far as the arm reaches from here: bring the base closer"
    elif elbow == "fully bent":
        advice = "the target is too close in for the arm: move the base back, or the target out"
    elif 6 in ks:
        advice = ("ask for other angles: another pitch (pointing near level: more down; pointing down close to the "
                  "base: less), or the roll 180 degrees off (the fingers are symmetric); or move the base")
    elif ks & {5, 7}:
        advice = ("ask for other angles: the fingers are symmetric, so the same grip with the roll (pointing down: "
                  "the yaw) 180 degrees off turns the hand the other way; or move the base")
    else:
        advice = "ask for other angles (nearer those of the start pose), or move the base"
    return f"the arm stopped with {' and '.join(joints)}: {advice}"


def camera_checker(cams, aliases=None) -> Callable[[Any], str]:
    """``check_cam(cam)``: the camera's own name (an alias allowed), or :class:`Refused`."""
    cams, aliases = tuple(cams), dict(aliases or {})

    def check_cam(cam) -> str:
        name = aliases.get(cam, cam) if isinstance(cam, str) else cam
        if name not in cams:
            raise Refused(f"camera must be one of {', '.join(cams)}, not {cam!r}")
        return name

    return check_cam


def describe(status: dict, *, finished: bool = False, failure: str | None = None) -> str:
    if failure:
        return f"the simulator failed ({failure}); the episode is over"
    if status.get("success"):
        return "the environment reports SUCCESS: the task is complete"
    step, horizon = status.get("step"), status.get("horizon")
    if step is not None and horizon is not None and step >= horizon:
        return f"the step budget is used up ({step} of {horizon}); the episode is over"
    if finished:
        return "the episode was ended"
    return "the episode is over"


class ArmHost:
    public: tuple[str, ...] = ARM_CALLS
    cams: tuple[str, ...] = ()
    aliases: dict[str, str] = {}
    #: the gap between the finger pads when fully open (m); an opening g asks for g times it
    finger_stroke_m = 0.08
    #: control period (s)
    step_s = 0.05
    #: the note of a goto out of reach
    unreachable_note = "from here the arm cannot get there: ask for another pitch or height"
    #: the note of a goto out of reach from the arm's pose now that the arm gets to from home
    home_note = "robot.home() first, then the same move: from home the arm gets there with the base where it is"
    #: what the note of a motion that stopped short suggests when nothing is in the way
    stalled_advice = "ask for another pitch"
    #: what the note of a motion stopped with the arm stretched suggests after going home
    stretched_advice = "ask for another pitch or height"
    #: what the note of a motion that pressed on something with the arm stretched out suggests
    jammed_advice = "to get there, come from another side or pitch"
    #: whether a motion during which what the hand held slipped out stops the cell (RobotError)
    stop_on_drop = False
    #: with the gap between the fingers this much (m) narrower than after the first motion of the grasp (what
    #: they hold settles in the fingers then), what they hold is slipping: a drive or a home stops the cell
    #: before it starts, once per grasp (RobotError; None: never); a call that grasps in its own way may set
    #: another for that grasp (slip_limit)
    stop_on_slip: float | None = None
    #: gripper('light'): after a close that stops on something, the fingers are set (in steps of 0.1) at least
    #: this much (of the opening) under where they stand, a light hold (None: no 'light'; a close keeps squeezing)
    light_hold: float | None = None
    #: whether a close that holds says so when the grasp point is at the top of a low thing measured under it
    top_grasp_note = False
    #: whether a move down with the fingers open that stops short over a thing measured under the hand says so
    top_stop_note = False
    #: (m, m/s, deg): an arc asked for no speed, with the fingers holding something at least that far from its line (a
    #: door's handle), goes at that speed while the earlier arcs about the same line that kept hold of it have turned
    #: it less than that far (the start of a door's swing; None: at the arm's own)
    held_arc: tuple[float, float, float] | None = None

    def __init__(self, session, opened: dict, *, scene=None, log: Callable[[dict], None] | None = None,
                 attach: Callable[[str, np.ndarray, str], None] | None = None,
                 on_rows: Callable[[np.ndarray], None] | None = None):
        self.session = session  # .call(op, **args), .step, .step_limit, .over, .fail(), .over_reason()
        self.log = log or (lambda event: None)
        self.attach = attach or (lambda kind, image, caption: None)
        self.on_rows = on_rows or (lambda rows: None)
        self.check_cam = camera_checker(self.cams, self.aliases)
        self.facts = dict(opened.get("facts") or {})
        start = opened["start"]
        self.proprio = start
        self.start_tcp = np.asarray(start["tcp"], float)
        # angles a motion leaves out keep the last asked ones; roll starts at 0 (the start pose has a few
        # degrees of roll, which would turn a grasp from above away from the yaw asked for; near vertical
        # the roll is folded into the yaw first, so that a hand pointing down is not turned half round)
        self.asked = no_roll(shown_angles(self.start_tcp[:3, :3]))
        self.grip_goal = round((1.0 - float(opened.get("level", -1.0))) / 2.0, 2)
        self.holding: bool | None = False
        self.held_width: float | None = None  # the gap when the grip was last checked
        self.grasp_width: float | None = None  # the gap when the fingers closed on what they hold
        self.carry_width: float | None = None  # ... and after the first motion with it
        self.slip_stopped = False  # stop_on_slip stopped a cell during this grasp
        self.slip_limit: float | None = None  # stop_on_slip for this grasp, set by the call that made it
        self.hold_gap: float | None = None  # in a light hold: the gap the fingers are set at
        self.dropped = False  # what the hand held slipped out during the current call
        self.swung: list[list] = []  # [a point on a line, its direction, degrees arcs holding on turned about it]
        self.scene = scene
        self._views: tuple[int | None, dict[str, DepthView]] | None = None
        if scene is not None:
            scene.bind(self.views, hand=self._tcp)

    # -- dispatch ----------------------------------------------------------------------------------
    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        if self.session.over:
            raise Stop(self.session.over_reason())
        if name not in self.public:
            raise CallFailed("RobotError", f"robot has no '{name}'")
        method = getattr(self, "_" + name)
        self.dropped = False
        try:
            out = method(*args, **kwargs)
        except RR.NotFound as exc:
            raise CallFailed("NotFound", str(exc)) from None
        except (Refused, SimRefused) as exc:
            raise CallFailed("ValueError", str(exc)) from None
        except SimError as exc:
            self.session.fail(str(exc).splitlines()[0])
            raise Stop(self.session.over_reason()) from None
        except TypeError as exc:
            if "argument" in str(exc):
                raise CallFailed("TypeError", f"robot.{name}: {exc}") from None
            raise
        if self.session.over:
            raise Stop(self.session.over_reason())
        if self.dropped and self.stop_on_drop:
            # the rest of the cell was written for the object in hand
            raise CallFailed("RobotError", f"{out}\nwhat the hand held slipped out on the way, so the cell stops here")
        return out

    def _call(self, op: str, **args) -> dict:
        reply = self.session.call(op, **args)
        if reply.get("proprio") is not None:
            self.proprio = reply["proprio"]
        rows = reply.get("log")
        if rows is not None and len(rows):
            self.on_rows(np.asarray(rows))
        return reply

    def _left(self) -> int | None:
        step, limit = self.session.step, self.session.step_limit
        return None if step is None or limit is None else max(0, int(limit) - int(step))

    def _tcp(self) -> np.ndarray:
        return np.asarray(self.proprio["tcp"], float)

    def _width(self) -> float:
        return float(self.proprio["width"])

    # -- the arm -----------------------------------------------------------------------------------
    def _goto(self, xyz=None, *, by=None, pitch=None, yaw=None, roll=None, grip=None, speed=None, dry_run=False):
        tcp = self._tcp()
        p = tcp[:3, 3].copy() if xyz is None else point_of(xyz)
        if by is not None:
            p = p + vector_of(by, "by")
        given = (number(pitch, "pitch"), number(yaw, "yaw"), number(roll, "roll"))
        angles = tuple(self.asked[i] if v is None else float(v) for i, v in enumerate(given))
        opening = None if grip is None else opening_of(grip)
        pose = np.eye(4)
        pose[:3, :3] = rotation_of(*angles)
        pose[:3, 3] = p
        v = speed_of(speed)
        res = self._call("goto", pose=pose, grip=opening, dry_run=bool(dry_run), speed=v)["result"]
        move = self._move(res, p, angles, bool(dry_run))
        self.log({"kind": "goto", "target": p, "angles": angles, "grip": opening, "speed": v,
                  "dry_run": bool(dry_run), "result": brief(res)})
        if res.get("moved"):
            self.asked = angles
            if opening is not None:
                self._grip_asked(opening, tcp)
            if self.top_stop_note and not dry_run and not move.reached:
                move.notes += self._top_stop_notes(p, angles)
        elif not dry_run:
            # refused: the cell stops here (unless it catches NotExecuted), so that its next motions
            # do not run from the wrong place
            raise CallFailed("NotExecuted", str(move))
        return move

    def _arc(self, center, angle, *, axis=(0.0, 0.0, 1.0), grip=None, speed=None, dry_run=False):
        c = point_of(center)
        a = axis_of(axis)
        turn = number(angle, "angle")
        if turn is None:
            raise Refused("arc: angle is a number of degrees")
        if turn == 0 or abs(turn) > 180.0:
            raise Refused("arc: the angle is between -180 and 180 degrees (not 0); turn further in a second arc")
        opening = None if grip is None else opening_of(grip)
        start = self._tcp()
        asked = speed_of(speed)
        v = asked
        line = self._swung_line(c, a)
        swung = 0.0 if line is None else abs(line[2])
        holding = bool(self.holding)
        if asked is None and holding and self.held_arc is not None and swung < self.held_arc[2]:
            off = self._tcp()[:3, 3] - c
            if float(np.linalg.norm(off - float(off @ a) * a)) >= self.held_arc[0]:
                v = self.held_arc[1]
        res = self._call("arc", center=c, axis=a, angle=math.radians(turn), grip=opening,
                         dry_run=bool(dry_run), speed=v)["result"]
        end = np.asarray(res["end"], float)
        target_angles = nearest(angles_of(end[:3, :3]), self.asked)
        move = self._move(res, end[:3, 3], target_angles, bool(dry_run), rot=end[:3, :3])
        self.log({"kind": "arc", "center": c, "axis": a, "angle": turn, "grip": opening, "speed": v,
                  **({"held_slow": True} if asked is None and v is not None else {}), "swung": round(swung, 1),
                  "dry_run": bool(dry_run), "result": brief(res)})
        if res.get("moved") and not dry_run and holding and opening is None and not self.dropped:
            # what the hand holds turned with it (a slip moved a door by an unknown part: not counted)
            if line is None:
                line = [c, a, 0.0]
                self.swung.append(line)
            line[2] += turn if float(line[1] @ a) > 0 else -turn
        if res.get("moved"):
            self.asked = target_angles
            if opening is not None:
                self._grip_asked(opening, start)
        elif not dry_run:
            raise CallFailed("NotExecuted", str(move))
        return move

    def _swung_line(self, c: np.ndarray, a: np.ndarray) -> list | None:
        """The entry of ``swung`` for the line through c along a: within 6 cm and 10 degrees of it (one hinge)."""
        for line in self.swung:
            off = c - line[0]
            if (abs(float(line[1] @ a)) > math.cos(math.radians(10.0))
                    and float(np.linalg.norm(off - float(off @ line[1]) * line[1])) < 0.06):
                return line
        return None

    def _push_arc(self, center, angle, *, axis=(0.0, 0.0, 1.0), push=None, press=0.03, tilt=25.0, dry_run=False):
        c = point_of(center)
        a = axis_of(axis)
        turn = number(angle, "angle")
        if turn is None:
            raise Refused("push_arc: angle is a number of degrees")
        if turn == 0 or abs(turn) > 180.0:
            raise Refused("push_arc: the angle is between -180 and 180 degrees (not 0)")
        depth = number(press, "press")
        if depth is None or not 0.01 <= depth <= 0.05:
            raise Refused("push_arc: press is how far past the hand each push aims, 0.01 to 0.05 (m)")
        lean = number(tilt, "tilt")
        if lean is None or not 0.0 <= lean <= 60.0:
            raise Refused("push_arc: tilt is 0 to 60 degrees")
        if push is None:
            # along the hand's circle about the hinge, the way the angle turns
            arm = self._tcp()[:3, 3] - c
            u = np.cross(a, arm - a * float(arm @ a)) * math.copysign(1.0, turn)
            if float(np.linalg.norm(u)) < 0.02:
                raise Refused("push_arc: the hand is on the hinge line; center is a point on the hinge, away "
                              "from the hand")
        else:
            u = vector_of(push, "push")
        u = u / float(np.linalg.norm(u))
        res = self._call("push_arc", center=c, axis=a, angle=math.radians(turn), push=u, press=float(depth),
                         tilt=math.radians(lean), dry_run=bool(dry_run))["result"]
        self.log({"kind": "push_arc", "center": c, "axis": a, "angle": turn, "push": u, "press": float(depth),
                  "tilt": lean, "dry_run": bool(dry_run), "result": brief(res)})
        reach = res.get("reach") or {}
        tcp = self._tcp()
        moved = bool(res.get("moved"))
        why = str(reach.get("why") or ("" if reach.get("ok") else "out of reach"))
        notes = []
        if not reach.get("ok") and not reach.get("stretched_now"):
            notes.append(self.unreachable_note)
        stop = str(res.get("stop") or "")
        turned = math.degrees(float(res.get("angle") or 0.0))
        if moved:
            self.asked = nearest(angles_of(tcp[:3, :3]), self.asked)
            if stop == "no touch":
                notes.append(f"the hand went {100.0 * float(res.get('touch_m') or 0.0):.0f} cm along the push "
                             "without meeting anything: look where the part is, and start nearer it")
            elif stop == "slid":
                drift = 100.0 * float(res.get("drift_m") or 0.0)
                notes.append(f"after {abs(turned):.0f} deg the hand came {abs(drift):.0f} cm "
                             f"{'nearer to' if drift < 0 else 'farther from'} the hinge, as a hand that slid off "
                             "the part (or touched nothing) does (on the part it keeps its distance): look where "
                             "the hand is and whether the part turned")
            elif stop == "stalled":
                notes.append(f"the part stopped turning at {abs(turned):.0f} deg though the hand pressed harder: "
                             "it is at the end of its travel, or the hand pushes where it does not turn it "
                             "(near the hinge, or on the wrong face): look")
            elif stop == "jammed":
                notes.append("the arm pressed at full force with the elbow open wider than a right angle, so it "
                             f"stopped and pulled the hand {GIVE_WAY_CM} cm back toward the shoulder: look what it "
                             f"touched; {self.jammed_advice}")
            elif stop == "stretched":
                notes.append(f"the arm's elbow came nearly straight, so it stopped and {gave_way_how(res)}: "
                             f"{self.stretched_advice}")
            elif stop == "cycles":
                notes.append("the push ran for its longest without getting there: look")
            shift = float(res.get("hinge_shift_m") or 0.0)
            if shift > 0.02:
                notes.append(f"the hand's track turns about a line {100.0 * shift:.0f} cm from the center given")
            limited = limit_note(res.get("at_limit")) if stop != "angle" else None
            if limited and stop not in ("jammed", "stretched"):
                notes.append(limited)
        notes += self._slipped(moved and not dry_run)
        out = R.Push(moved=moved, turned=round(turned, 1), angle=round(turn, 1), stop=stop,
                     at=tuple(round(float(v), 4) for v in tcp[:3, 3]),
                     angles=shown_near(tcp[:3, :3], self.asked), steps=int(res.get("steps", 0)),
                     dry_run=bool(dry_run), reachable=bool(reach.get("ok")), why=why, notes=notes,
                     steps_left=self._left())
        if not moved and not dry_run:
            raise CallFailed("NotExecuted", str(out))
        return out

    def _move(self, res: dict, target, angles, dry_run: bool, rot=None) -> R.Move:
        reach = res.get("reach") or {}
        tcp = self._tcp()
        moved = bool(res.get("moved"))
        reached = bool(reach.get("ok")) if dry_run or not moved else bool(res.get("reached"))
        err_m = float(res.get("err_m", reach.get("err_m", 0.0)) or 0.0)
        err_deg = float(res.get("err_deg", reach.get("err_deg", 0.0)) or 0.0)
        own_m, shoved = err_m, False
        if moved and not dry_run:
            # the arm's own error is relative to the base, which a push moves: give the hand's distance
            # from the target in the scene, the figure the printed positions show
            off = float(np.linalg.norm(tcp[:3, 3] - np.asarray(target, float)))
            goal_rot = rotation_of(*angles) if rot is None else np.asarray(rot, float)
            shoved = off > own_m + SHOVED_M
            err_m, err_deg = off, turn_deg(tcp[:3, :3], goal_rot)
            reached = reached and not shoved
        notes = []
        why = str(reach.get("why") or ("" if reach.get("ok") else "out of reach"))
        if reach.get("stretched_now"):
            pass  # the why says what to do first
        elif reach.get("from_home"):
            notes.append(self.home_note)
        elif not reach.get("ok"):
            notes.append(self.unreachable_note)
        elif reach.get("limit_margin") is not None and float(reach["limit_margin"]) < math.radians(5.0):
            notes.append("the arm is near a joint limit there")
        limited = limit_note(res.get("at_limit")) if moved and not res.get("reached") else None
        if moved and res.get("jammed"):
            notes.append("the arm pressed on something at full force with the elbow open wider than a right angle, "
                         f"so it stopped and pulled the hand {GIVE_WAY_CM} cm back toward the shoulder (pressed on, a "
                         "stretched arm snaps straight and gets stuck). Look what it touched (the hand, the wrist or "
                         f"the forearm); {self.jammed_advice}")
        elif moved and res.get("stretched"):
            notes.append("the arm's elbow came nearly straight on the way (a straight arm gets stuck), so it stopped "
                         f"short and {gave_way_how(res)}: {self.stretched_advice}")
        elif limited:
            notes.append(limited)
        elif moved and res.get("blocked"):
            notes.append(PRESSING)
        elif moved and res.get("stalled"):
            notes.append(f"the hand stopped {1000.0 * err_m:.0f} mm and {err_deg:.0f} deg from the target and was "
                         "not getting closer (it touches something, or the arm is near the edge of what it can do "
                         f"there): look; with nothing in the way, {self.stalled_advice}")
        notes += self._slipped(moved and not dry_run)
        if shoved:
            notes.append(f"the hand ended {100.0 * err_m:.1f} cm from the target, though relative to the base it "
                         f"got within {100.0 * own_m:.1f} cm: the base was pushed during the move (the hand or the "
                         "arm pressed on something) - look")
        return R.Move(reached=reached, moved=moved, target=tuple(round(float(v), 4) for v in target),
                      target_angles=tuple(round(float(v), 1) for v in angles),
                      at=tuple(round(float(v), 4) for v in tcp[:3, 3]), angles=shown_near(tcp[:3, :3], angles),
                      err_mm=round(1000.0 * err_m, 1), err_deg=round(err_deg, 1),
                      steps=int(res.get("steps", 0)), blocked=bool(res.get("blocked")), dry_run=dry_run, why=why,
                      notes=notes, steps_left=self._left())

    def _slipped(self, moved: bool = True) -> list[str]:
        width = self._width()
        if self.holding and width < (HOLD_MIN_M if self.hold_gap is None else self.hold_gap + EASED_EMPTY_M):
            self.holding = False
            self.held_width = self.grasp_width = self.carry_width = None
            self.dropped = True
            if self.scene is not None:
                self.scene.released(self._tcp())
            how = "fully" if self.hold_gap is None else "in to where they were set"
            self.hold_gap = None
            return [f"the fingers closed {how}: what they held slipped out - look where it went"]
        if moved and self.holding and self.grasp_width is not None and self.carry_width is None:
            self.carry_width = width
        if self.holding and self.held_width is not None and abs(width - self.held_width) > HELD_SHIFT_M:
            before, self.held_width = self.held_width, width
            return [f"the gap between the fingers changed from {100 * before:.1f} to {100 * width:.1f} cm: what "
                    "they hold turned or slid in them (it may have hit something) - look"]
        return []

    def _check_slip(self, motion: str) -> None:
        """Before a drive or a home: stop the cell if what the hand holds is slipping (``stop_on_slip``)."""
        if self.stop_on_slip is None or not self.holding or self.carry_width is None or self.slip_stopped:
            return
        width = self._width()
        if self.carry_width - width < (self.stop_on_slip if self.slip_limit is None else self.slip_limit):
            return
        self.slip_stopped = True
        raise CallFailed("RobotError", f"{motion} did not start: the gap between the fingers is "
                         f"{100 * width:.1f} cm, {100 * self.carry_width:.1f} cm after the first move with it. What "
                         "they hold is slipping out, and things carried like this tend to fall on the way. Set it "
                         "down, open and grasp it again (deeper, or where it is widest) before carrying it. This "
                         f"stops a cell once per grasp: {motion} again carries it as it is")

    def _thing_under(self, p, low: float) -> RR.Thing | None:
        """The thing measured whose centre is nearest the grasp point ``p`` across (within TOP_GRASP_XY_M), with
        ``p`` from ``low`` over what it stands on to TOP_GRASP_OVER_M over its top; things wider than the open
        fingers (they hold a part of it) and things last measured in a hand (what they stood on is not known) left
        out."""
        near = []
        for thing in self.scene.things():
            c = thing.center_xyz
            if not thing.found or c is None or thing.top is None or thing.on_z is None or thing.size is None:
                continue
            if thing.on.startswith("held") or thing.size[1] > self.finger_stroke_m:
                continue
            d = math.hypot(c[0] - p[0], c[1] - p[1])
            if d < TOP_GRASP_XY_M and thing.on_z + low < p[2] < thing.top + TOP_GRASP_OVER_M:
                near.append((d, thing))
        return min(near, key=lambda dt: dt[0])[1] if near else None

    def _top_stop_notes(self, target, angles) -> list[str]:
        """After a move down, pointing down with the fingers open, that stopped short: the grasp point at the top
        of a thing measured under it, or lower down but above the surface it stands on."""
        if self.scene is None or self.holding is not False or self.grip_goal <= 0.3 or angles[0] < 60.0:
            return []
        p = self._tcp()[:3, 3]
        if float(target[2]) > p[2] - 0.005:
            return []
        thing = self._thing_under(p, STOP_OVER_ON_M)
        if thing is None:
            return []
        under = thing.top - p[2]
        where = f"{100 * under:.1f} cm under" if under >= 0 else f"{100 * -under:.1f} cm over"
        held = "1 time in 3" if under >= 0.01 else "1 time in 5"
        return [f"the hand stopped {100 * (p[2] - float(target[2])):.1f} cm above the target with the grasp point "
                f"{where} the top of Thing#{thing.id} '{thing.label}' as last measured, "
                f"{100 * (p[2] - thing.on_z):.1f} cm over what it stands on: the fingers rest on it or on something "
                f"beside it, not around it. Closed after a stop like this, they held about {held} (4 in 5 after moves "
                "that got 1 cm or more under the top). Rise 2 cm, look at the wrist picture and come down where both "
                "fingers clear it: the grasps that held after such stops were about 2.5 cm to one side and 2 cm lower"]

    def _top_grasp_notes(self) -> list[str]:
        """After a full close that holds: the grasp point at, or over, the top of a low thing measured under it."""
        if self.scene is None:
            return []
        p = self._tcp()[:3, 3]
        thing = self._thing_under(p, -0.02)
        if thing is None:
            return []
        high, under = thing.top - thing.on_z, thing.top - p[2]
        if high >= LOW_THING_M or under >= TOP_GRASP_M:
            return []
        if high < FLAT_THING_M and p[2] - thing.on_z < SURFACE_M:  # at the surface: nothing lower to go to
            return []
        where = f"{100 * under:.1f} cm under" if under >= 0 else f"{100 * -under:.1f} cm over"
        if high < FLAT_THING_M:
            lower = (f"down to the height it stands on (z {thing.on_z:.3f}; a move that stops short within 1 cm "
                     "over it, on the surface, is fine)")
        else:
            lower = f"down to about half its height (z {thing.on_z + high / 2:.3f})"
        return [f"the grasp point is {where} the top of Thing#{thing.id} '{thing.label}' as last measured "
                f"({100 * high:.1f} cm high): closed this near their top, low things slip out as the hand rises about "
                "half the time. Unless it has to stick out under the fingers (a sponge to scrub with) or is a part "
                f"of a fixture (a knob, a lever, a faucet handle), open, go {lower} and close again"]

    def _grip_asked(self, opening: float, start: np.ndarray) -> None:
        """A motion with ``grip`` that began at grasp pose ``start``."""
        self.grip_goal = opening
        self.hold_gap = None
        self.holding = None if opening <= 0.05 else False
        self.held_width = self.grasp_width = self.carry_width = self.slip_limit = None
        if opening > 0.05 and self.scene is not None:
            self.scene.released(start)  # the fingers open as the motion starts

    def _gripper(self, opening):
        light = self.light_hold is not None and isinstance(opening, str) and opening.lower() == "light"
        g = 0.0 if light else opening_of(opening)
        before = self._width()
        res = self._call("gripper", opening=g)["result"]
        self.grip_goal = g
        self.hold_gap = None
        width = float(res["width"])
        done = abs(float(res["level"]) - float(res["goal"])) < 1e-6
        stroke = self.finger_stroke_m
        # the gap of the opening actually set (the gripper moves in steps of 0.1)
        asked = (1.0 - float(res["goal"])) / 2.0 * stroke
        held_back = False
        if not done:
            holding = None
        elif g <= 0.05:
            holding = width > HOLD_MIN_M
        else:
            # a partial close that stops wider than asked holds what is between the fingers; an
            # opening that stops narrower than asked is held back from outside and holds nothing
            holding = before > asked + 0.002 and width > asked + 0.005
            held_back = before < asked - 0.002 and width < asked - 0.005
        steps, shown, eased = int(res["steps"]), g, None
        if light and holding:
            ease = math.floor((width / stroke - self.light_hold) * 10.0 + 1e-6) / 10.0
            if ease >= 0.1:
                eased = self._call("gripper", opening=ease)["result"]
                steps += int(eased["steps"])
                width = float(eased["width"])
                gap = (1.0 - float(eased["goal"])) / 2.0 * stroke
                self.grip_goal = shown = ease
                if width < gap + EASED_EMPTY_M:
                    holding = False  # it came out as the fingers eased
                else:
                    self.hold_gap = gap
        self.holding = holding
        if self.scene is not None:  # what the fingers hold moves with the hand (measure looks for it there)
            if holding:
                self.scene.grasped(self._tcp())
            else:
                self.scene.released(self._tcp())
        self.held_width = self.grasp_width = width if holding else None
        self.carry_width = self.slip_limit = None
        self.slip_stopped = False
        notes = []
        if self.hold_gap is not None:
            notes.append(f"then eased to a light hold: the fingers are set at {100 * self.hold_gap:.1f} cm")
        elif light and holding:
            notes.append("too thin to ease: it is held with the full squeeze")
        elif eased is not None:
            notes.append("it came out as the fingers eased - look where it is")
        if g > 0.05 and holding:
            notes.append(f"the fingers stopped at {100 * width:.1f} cm, short of the {100 * asked:.1f} "
                         "cm asked: something is between them")
        elif held_back:
            notes.append(f"the fingers opened only to {100 * width:.1f} cm of the {100 * asked:.1f} cm asked: "
                         "something beside them (a rim or wall around the fingers) stops them from outside. "
                         "They hold nothing between them; move the hand out the way it came in")
        elif holding and width >= NEARLY_OPEN * stroke:
            notes.append(f"the fingers closed only to {100 * width:.1f} cm, nearly fully open: they may rest on "
                         "something or press on it from outside rather than hold it - look at the wrist picture "
                         "before lifting")
        if holding and g <= 0.05 and self.top_grasp_note:
            notes += self._top_grasp_notes()
        notes += gave_way_notes(res)
        # one event for both (the analyses take a gripper call to end a hold): the steps and the gap at the end
        self.log({"kind": "gripper", "opening": g, "result": {**brief(res), "steps": steps, "width": width},
                  **({"light": True} if light else {}), **({} if eased is None else {"eased": shown})})
        return R.Grip(opening=round(shown, 2), width_m=round(width, 4), holding=holding, goal=shown, steps=steps,
                      steps_left=self._left(), notes=notes)

    def _home(self):
        self._check_slip("robot.home()")
        res = self._call("home")["result"]
        goal = np.asarray(res["goal"], float)
        self.asked = no_roll(shown_angles(goal[:3, :3]))
        notes = self._slipped()
        if res.get("jammed"):
            notes.append("on the way home the arm pressed on something at full force with the elbow open wider than a "
                         f"right angle, so it stopped and pulled the hand {GIVE_WAY_CM} cm back toward the shoulder: "
                         "look what it touched (the hand, the wrist or the forearm), move the hand clear of it, then "
                         "home again")
        elif res.get("blocked"):
            notes.append(limit_note(res.get("at_limit")) or PRESSING)
        self.log({"kind": "home", "result": brief(res)})
        return R.Homing(home_mm=round(1000.0 * float(res["home_m"]), 1), home_deg=round(float(res["home_deg"]), 1),
                        steps=int(res["steps"]), blocked=bool(res.get("blocked")), steps_left=self._left(), notes=notes)

    def _wait(self, n):
        try:
            n = int(n)
        except (TypeError, ValueError):
            raise Refused(f"wait(n): n is a whole number of steps, 1..{WAIT_MAX}") from None
        if not 0 < n <= WAIT_MAX:
            raise Refused(f"wait(n): n is a whole number of steps, 1..{WAIT_MAX}")
        res = self._call("wait", n=n)["result"]
        notes = gave_way_notes(res)
        return R.Waited(steps=int(res["steps"]), step_s=self.step_s, steps_left=self._left(), notes=notes)

    # -- pictures ----------------------------------------------------------------------------------
    def views(self, cams=None) -> dict[str, DepthView]:
        """The cameras at this step (all rendered once per step; ``cams`` is for hosts whose pictures
        are dear)."""
        step = self.session.step
        if self._views is None or self._views[0] != step:
            obs = self._call("observe", cams=list(self.cams), robot_mask=True)["obs"]
            views = {}
            for cam, image in obs["images"].items():
                c = obs["cameras"][cam]
                camera = Camera(cam, float(c["focal"]), np.asarray(c["pose"], float), int(c["width"]),
                                int(c["height"]))
                robot = obs["robot"].get(cam)
                if robot is None:
                    robot = np.zeros(image.shape[:2], bool)
                views[cam] = DepthView(cam, image, obs["depth"][cam], camera, np.asarray(robot, bool),
                                       step=int(obs["step"]))
            self._views = (step, views)
        return self._views[1]

    def pictures(self) -> dict[str, np.ndarray]:
        return {cam: v.image for cam, v in self.views().items()}

    def _need_scene(self):
        if self.scene is None:
            raise Refused("find and measure are not available in this run (no SAM3 service); to_world and the "
                          "pictures work")
        return self.scene

    def _find(self, cam, text=None, *, at=None, points=None, labels=None, box=None):
        return self._need_scene().find(self.check_cam(cam), text, at=at, points=points, labels=labels, box=box)

    def _measure(self, target, *, cams="auto"):
        return self._need_scene().measure(target, cams=cams)

    def _to_world(self, cam, xy, *, z=None, pair=None):
        return spot(self.views(), self.check_cam(cam), xy, z=z, pair=pair, check_cam=self.check_cam)

    def _to_pixel(self, xyz, cam):
        px = self.views()[self.check_cam(cam)].camera.pixel(point_of(xyz))
        return None if px is None else (round(px[0], 1), round(px[1], 1))

    def _things(self):
        return RR.ThingList(self.scene.things() if self.scene is not None else [])

    def _image(self, cam):
        return self.views()[self.check_cam(cam)].image.copy()

    def _depth(self, cam):
        return self.views()[self.check_cam(cam)].depth.copy()

    def _camera(self, cam):
        return self.views()[self.check_cam(cam)].camera.info()

    def _look(self, cams="all", *, grid=False, marks=None, masks=None, crop=None, zoom=None, tile=False, note=None):
        names = list(self.cams) if cams in ("all", None) else [cams] if isinstance(cams, str) else list(cams)
        names = [self.check_cam(c) for c in names]
        if not names:
            raise Refused("look: name at least one camera")
        if crop is not None and len(names) > 1:
            raise Refused("look: crop applies to one camera at a time")
        views = self.views()
        rendered, notes = [], []
        for cam in names:
            view = views.get(cam)
            if view is None:
                notes.append(f"no {cam} picture")
                continue
            camera = view.camera
            img, more = overlay.render(view.image, cam=cam, grid=bool(grid), marks=marks,
                                       outlines=self._outlines(masks, cam), crop=crop, zoom=zoom,
                                       project=lambda p, c=camera: c.pixel(point_of(p)))
            rendered.append((cam, img))
            notes += more
        if not rendered:
            raise Refused("look: none of those pictures is available")
        extras = [w for w, on in (("grid", grid), ("marks", marks), ("outlines", masks), ("crop", crop)) if on]
        what = ", ".join(c for c, _ in rendered) + (f" ({', '.join(extras)})" if extras else "")
        caption = f"look: {what}" + (f" — {note}" if note else "")
        if tile and len(rendered) > 1:
            self.attach("look", overlay.side_by_side([im for _, im in rendered]), caption + " (side by side)")
        else:
            for cam, img in rendered:
                self.attach("look", img, f"look: {cam}" + (f" ({', '.join(extras)})" if extras else "")
                            + (f" — {note}" if note else ""))
        return RR.Picture(cams=[c for c, _ in rendered],
                          what=what + (", side by side" if tile and len(rendered) > 1 else ""), notes=notes)

    def _outlines(self, masks, cam: str) -> list[tuple[np.ndarray, str]]:
        if masks is None:
            return []
        items = RR.mask_items(masks)
        out = []
        for item in items:
            if isinstance(item, RR.Mask):
                if item.cam == cam:
                    out.append((item.pixels, f"#{item.id} {item.label}"))
            elif isinstance(item, RR.Thing):
                m = item.masks.get(cam)
                if m is not None:
                    out.append((m.pixels, f"T{item.id} {item.label}"))
            else:
                raise Refused("look(masks=...): give Masks (from find), Things (from measure) or a Thing's .masks")
        return out

    # -- reading -----------------------------------------------------------------------------------
    def _task(self):
        return R.TaskInfo(instruction=str(self.facts.get("instruction") or ""), step=self.session.step,
                          step_limit=self.session.step_limit,
                          start_at=tuple(round(float(v), 3) for v in self.start_tcp[:3, 3]),
                          start_angles=shown_angles(self.start_tcp[:3, :3]), task_name=self.facts.get("task"))

    def home_pose(self) -> np.ndarray:
        """The tool pose ``home`` returns to: the start pose (a subclass may move it with its base)."""
        return self.start_tcp

    def home_offset(self) -> tuple[float, float]:
        """(mm, degrees) of the hand from the home pose."""
        goal = self.home_pose()
        tcp = self._tcp()
        return (round(1000.0 * float(np.linalg.norm(tcp[:3, 3] - goal[:3, 3])), 1),
                round(turn_deg(tcp[:3, :3], goal[:3, :3]), 1))

    def status_extra(self) -> dict:
        """More fields of :class:`~hexashell.benches.results.Status` (the base, if any)."""
        return {}

    def _state(self):
        tcp = self._tcp()
        width = self._width()
        home_mm, home_deg = self.home_offset()
        return R.Status(step=self.session.step, step_limit=self.session.step_limit,
                        at=tuple(round(float(v), 4) for v in tcp[:3, 3]), angles=shown_near(tcp[:3, :3], self.asked),
                        gripper=round(min(1.0, max(0.0, width / self.finger_stroke_m)), 2), width_m=round(width, 4),
                        goal=self.grip_goal, holding=self.holding, home_mm=home_mm, home_deg=home_deg,
                        grasp_m=None if self.grasp_width is None else round(self.grasp_width, 4),
                        **self.status_extra())


# -- arguments ---------------------------------------------------------------------------------------
def point_of(value) -> np.ndarray:
    """A point (episode frame) from a tuple, an array, a Spot, a Thing (its centre) or a Move / Status (the hand)."""
    if isinstance(value, RR.Thing):
        value = value.center
    elif isinstance(value, RR.Spot):
        if value.xyz is None:
            raise RR.NotFound(f"the Spot has no world point: {value.warning}")
        value = value.xyz
    elif isinstance(value, (R.Move, R.Status)):
        value = value.at
    try:
        p = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused("a point is three numbers (x, y, z) in metres") from None
    if p.size != 3 or not np.all(np.isfinite(p)):
        raise Refused("a point is three numbers (x, y, z) in metres")
    return p


def vector_of(value, name: str) -> np.ndarray:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres") from None
    if v.size != 3 or not np.all(np.isfinite(v)):
        raise Refused(f"{name} is three numbers (dx, dy, dz) in metres")
    return v


def axis_of(value) -> np.ndarray:
    if isinstance(value, str):
        named = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
        if value.lower().lstrip("+-") not in named:
            raise Refused("arc(axis=...): 'x', 'y', 'z' or a direction (ax, ay, az)")
        v = np.array(named[value.lower().lstrip("+-")])
        return -v if value.startswith("-") else v
    v = vector_of(value, "axis")
    n = float(np.linalg.norm(v))
    if n < 1e-6:
        raise Refused("arc(axis=...): the direction must not be zero")
    return v / n


def opening_of(value) -> float:
    if isinstance(value, str):
        if value.lower() not in GRIP_WORDS:
            raise Refused("an opening is 'open', 'close' or a number 0..1")
        return GRIP_WORDS[value.lower()]
    try:
        g = float(value)
    except (TypeError, ValueError):
        raise Refused("an opening is 'open', 'close' or a number 0..1") from None
    if not (0.0 <= g <= 1.0):
        raise Refused("an opening is 'open', 'close' or a number 0..1")
    return g


def speed_of(value) -> float | None:
    """A motion's speed asked for (m/s), or :class:`Refused`; None: the arm's own."""
    v = number(value, "speed")
    if v is not None and not (SPEEDS[0] <= v <= SPEEDS[1]):
        raise Refused(f"speed is in m/s, {SPEEDS[0]:g} to {SPEEDS[1]:g} (the most, and what a motion uses "
                      "without it)")
    return v


def number(v, name: str):
    """``v`` as a finite float (None stays None), or :class:`Refused` naming the argument."""
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a number") from None
    if not math.isfinite(f):
        raise Refused(f"{name} is a number")
    return f


def no_roll(angles) -> tuple[float, float, float]:
    pitch, yaw, _ = angles
    return (pitch, yaw, 0.0)


def brief(res: dict) -> dict:
    """A motion result for the event log, without the joint arrays."""
    out = {}
    for k, v in res.items():
        if k == "reach" and isinstance(v, dict):
            out[k] = {kk: vv for kk, vv in v.items() if kk not in ("joints",)}
        elif isinstance(v, np.ndarray) and v.size > 16:
            continue
        else:
            out[k] = v
    return out
