"""The host side of ``robot.*`` on DexJoCo-X: Franka Panda arms with Wuji hands (five fingers, 20 joints), the
hand, finger, arc, home and wait commands, and the perception of :class:`~hexashell.benches.armhost.ArmHost` on
the scene camera's and the wrist cameras' RGB-D pictures.

Everything crosses to the server (:mod:`.server`) in the scene frame (metres, z up, the robot faces +x), angles
in degrees.  The hands have no touch sensors: what a hand presses on or holds comes from its fingers (a finger
that something outside the hand pushes on presses on something; a hand whose thumb and one more finger do holds
something), as the server reads it.  Perception keeps measured things on the hand that holds them
(:class:`~hexashell.vision.rgbd.DepthScene`, one hand per side).
"""

from __future__ import annotations

import math

import numpy as np

from ...errors import Refused
from .. import results as R
from ..armhost import JOINT_NAMES, ArmHost, axis_of, camera_checker, number, vector_of
from ..armhost import point_of as arm_point_of
from . import protocol as P

PUBLIC = ("hand", "arc", "fingers", "home", "wait", "find", "measure", "to_world", "to_pixel", "task", "state",
          "image", "depth", "look", "camera", "things")
#: farthest one hand motion goes (m)
HAND_MAX_M = 1.2
#: longest approach (m)
APPROACH_MAX_M = 0.25
#: a wrist camera sent to look at a point stays at least this far from it (m)
LOOK_MIN_M = 0.10
#: the slowest hand motion (m/s) and arc (deg/s) asked for, and the fastest hand motion (the task's arm may
#: follow less: protocol.speeds)
HAND_SPEED_MIN = 0.01
HAND_SPEED_MAX = 0.25
TURN_SPEED_MIN = 1.0
#: a finger setting's shortest and longest time (s)
FINGER_SECONDS = (0.1, 3.0)
#: the server's operations that run steps (the proprioception before one is kept for its notes)
MOTIONS = ("hand", "arc", "home", "wait", "fingers")
#: a dry run is reachable within this much (m, deg), and at the edge of reach within the second (a motion's
#: "reached"); checked against real motions, see control.NULL_PULL
DRY_RUN_M = 0.005
DRY_RUN_DEG = 2.0
DRY_RUN_EDGE_M = 0.010
DRY_RUN_EDGE_DEG = 5.0


def point_of(value) -> np.ndarray:
    """A scene-frame point from three numbers, a Spot, a Thing (its centre), a DexterousMove (where its point
    ended) or a DexterousHand (its palm)."""
    if isinstance(value, R.DexterousMove):
        value = value.at
    elif isinstance(value, R.DexterousHand):
        value = value.palm
    return arm_point_of(value)


def unit_of(value, name: str) -> np.ndarray:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a direction: three numbers (x, y, z)") from None
    n = float(np.linalg.norm(v)) if v.size == 3 and np.all(np.isfinite(v)) else 0.0
    if n < 1e-6:
        raise Refused(f"{name} is a direction: three numbers (x, y, z), not all zero")
    return v / n


def rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle_rad) * K + (1.0 - math.cos(angle_rad)) * (K @ K)


def joint_names(at_limit) -> str:
    return " and ".join(f"joint {int(k)} ({JOINT_NAMES[int(k) - 1]})" for k in at_limit
                        if 1 <= int(k) <= len(JOINT_NAMES))


def far_note(at_limit) -> str:
    joints = f"the arm would end with {joint_names(at_limit)} at the end of its travel: " if at_limit else ""
    return (f"{joints}that pose is out of the arm's reach with the hand held this way - another orientation of the "
            "hand, or a point nearer the arm's base, reaches easier")


def lost_note(limits) -> str:
    there = f"there the arm would have {joint_names(limits)} at the end of its travel" if limits else \
        "the arm cannot follow that path"
    return (f"{there}, and a real motion would stop on the way - another orientation of the hand, or a point nearer "
            "the arm's base, reaches easier")


def edge_note(at_limit) -> str:
    joints = f" (with {joint_names(at_limit)} at the end of its travel)" if at_limit else ""
    return (f"that pose is at the edge of what the arm reaches{joints}: a real motion may stop short of it - another "
            "orientation of the hand, or a point nearer the arm's base, reaches surely")


def _xyz(v) -> str:
    return "(" + ", ".join(f"{float(a):.3f}" for a in v) + ")"


def direction_words(v) -> str:
    """A scene-frame direction in words: the axis it runs along, or its numbers when it runs along none."""
    v = np.asarray(v, float)
    names = (("+x (away from the robot)", "-x (toward the robot)"), ("+y (the robot's left)", "-y (the robot's right)"),
             ("up", "down"))
    k = int(np.argmax(np.abs(v)))
    if abs(float(v[k])) >= 0.8:
        return names[k][0 if v[k] > 0 else 1]
    return "(" + ", ".join(f"{float(a):.2f}" for a in v) + ")"


def stopped_note(res: dict, what: str = "motion") -> str:
    """Why a motion stopped short: the arm at the end of a joint's travel, or something holding the hand up."""
    where = f" ({res['stage']})" if res.get("stage") else ""
    if res.get("at_limit"):
        return (f"the {what} stopped there{where}: the arm has {joint_names(res['at_limit'])} at the end of its "
                "travel - this way of holding the hand gets no farther; another orientation, or a point nearer the "
                "arm's base, reaches easier")
    if res.get("beyond_reach_m") is not None:
        return (f"the {what} stopped there{where}: the arm cannot get to the goal with the hand held this way - "
                "another orientation, or a point nearer the arm's base, reaches easier")
    if what == "arc":
        return "the arc stopped there: what the hand turns (or something in its way) holds it up - look before going on"
    return (f"the motion stopped there{where}: something is in the hand's way or it presses on something - look "
            "before going on")


class DexterousHost(ArmHost):
    public = PUBLIC
    aliases = P.CAMERA_ALIASES
    step_s = P.STEP_S

    def __init__(self, session, opened: dict, *, scene=None, log=None, attach=None, on_rows=None):
        # ArmHost's own __init__ is built round one gripper's tool pose; the parts of it the shared calls use
        self.session = session
        self.log = log or (lambda event: None)
        self.attach = attach or (lambda kind, image, caption: None)
        self.on_rows = on_rows or (lambda rows: None)
        self.facts = dict(opened.get("facts") or {})
        self.task = task = str(self.facts.get("task"))
        self.sides = tuple(self.facts.get("arms") or P.arms(task))
        self.cams = P.cameras(task)
        self.check_cam = camera_checker(self.cams, self.aliases)
        self.proprio = opened["start"]
        self.start = opened["start"]
        self.before = None
        self.dropped = False
        self.holding = {s: False for s in self.sides}
        self.scene = scene
        self._views = None
        if scene is not None:
            scene.bind(self.views, hands={s: (lambda s=s: self.grasp_pose(s)) for s in self.sides})

    # -- readings --------------------------------------------------------------------------------
    def _hand_now(self, side: str) -> dict:
        return self.proprio["hands"][side]

    def grasp_pose(self, side: str) -> np.ndarray:
        """The pose things a hand holds are carried at: x along the fingers, z out of the palm, at the grasp
        point."""
        h = self._hand_now(side)
        x = unit_of(h["finger"], "finger")
        z = np.asarray(h["normal"], float)
        z = z - x * float(x @ z)
        z = z / max(1e-9, float(np.linalg.norm(z)))
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
        T[:3, 3] = np.asarray(h["grasp"], float)
        return T

    def _point_now(self, side: str, point: str) -> np.ndarray:
        h = self._hand_now(side)
        if point in ("palm", "grasp", "camera"):
            return np.asarray(h[point], float)
        return np.asarray(h["tips"][point], float)

    def _side(self, side) -> str:
        if side not in self.sides:
            if side in P.SIDES:
                raise Refused(f"this task has only the {' and '.join(self.sides)} arm")
            raise Refused(f"side is {' or '.join(repr(s) for s in self.sides)}, not {side!r}")
        return side

    def _call(self, op: str, **args) -> dict:
        if op in MOTIONS:
            self.before = self.proprio
        return super()._call(op, **args)

    def _motion_notes(self) -> list[str]:
        """After a command, for a hand that holds nothing: its fingers press on something now (and did not
        before the command), or something pushed on them on the way and let go (it touched something: it may
        have pushed or tipped it); then what slipped out of a hand that held it (``_grip_notes``)."""
        notes = []
        pressed = self.proprio.get("pressed") or {}
        for side in self.sides:
            if self.holding[side]:
                continue
            now = list(self._hand_now(side).get("pressing") or [])
            was = list(((self.before or {}).get("hands") or {}).get(side, {}).get("pressing") or [])
            fingers = pressed.get(side) or []
            if now and set(now) != set(was):
                notes.append(f"the {side} hand presses on something now ({', '.join(now)}); look before going on")
            elif fingers and not now:
                notes.append(f"on the way something pushed on the {side} hand's fingers ({', '.join(fingers)}): the "
                             "hand touched something - it may have pushed or tipped it; look before going on")
        return notes + self._grip_notes()

    def _grip_notes(self) -> list[str]:
        """After a command: a hand that held something and whose thumb and fingers no longer both press on
        something (what it held slipped out)."""
        notes = []
        for side in self.sides:
            if not self.holding[side] or self._hand_now(side).get("holding"):
                continue
            self.holding[side] = False
            self.dropped = True
            if self.scene is not None:
                self.scene.released(self.grasp_pose(side), hand=side)
            pressing = self._hand_now(side).get("pressing") or []
            now = ", ".join(pressing) if pressing else "none"
            notes.append(f"the {side} hand's thumb and fingers no longer both press on something (pressing now: "
                         f"{now}): what it held has slipped out or is slipping - look before going on")
        return notes

    def _to_pixel(self, xyz, cam):
        px = self.views()[self.check_cam(cam)].camera.pixel(point_of(xyz))
        return None if px is None else (round(px[0], 1), round(px[1], 1))

    # -- hands -----------------------------------------------------------------------------------
    def _speed(self, speed, what: str) -> float | None:
        """A hand motion's speed (m/s) or an arc's (deg/s) asked for, within the task's; None: the default."""
        v = number(speed, "speed")
        if v is None:
            return None
        default, most = P.speeds(self.task)["hand" if what == "hand" else "turn"]
        if what == "hand":
            most = min(most, HAND_SPEED_MAX)
            if not HAND_SPEED_MIN <= v <= most:
                raise Refused(f"hand(speed=...): m/s, {HAND_SPEED_MIN:g} to {most:g} in this task (without it "
                              f"{default:g})")
        elif not TURN_SPEED_MIN <= v <= most:
            raise Refused(f"arc(speed=...): degrees per second, {TURN_SPEED_MIN:g} to {most:g} in this task "
                          f"(without it {default:g})")
        return v

    def _orientation(self, side: str, facing, normal, finger, along) -> dict:
        n = f = a = None
        if facing is not None:
            if facing not in P.PALM_FACING:
                raise Refused(f"hand(facing=...): one of {', '.join(P.PALM_FACING)}")
            rn, rf = P.palm_axes(facing, side)
            n, f = np.array(rn), np.array(rf)
        if normal is not None:
            n = unit_of(normal, "normal")
        if finger is not None:
            f = unit_of(finger, "finger")
        if along is not None:
            if finger is not None:
                raise Refused("hand: along (a handle's direction, for a power grip) or finger, not both")
            a = unit_of(along, "along")
            f = None  # facing gives the palm's way only
        if n is not None and f is not None and abs(float(n @ f)) > math.cos(math.radians(60.0)):
            raise Refused("hand: the palm's normal and the fingers' direction must be at least 60 degrees apart "
                          "(they are at right angles on the hand)")
        return {"normal": None if n is None else [float(v) for v in n],
                "finger": None if f is None else [float(v) for v in f],
                "along": None if a is None else [float(v) for v in a]}

    def _hand(self, side, xyz=None, *, by=None, point="palm", facing=None, normal=None, finger=None, along=None,
              look_at=None, approach=None, speed=None, dry_run=False):
        side = self._side(side)
        if point not in P.HAND_POINTS:
            raise Refused(f"hand(point=...): one of {', '.join(P.HAND_POINTS)}")
        now = self._point_now(side, point)
        target = now.copy() if xyz is None else point_of(xyz)
        if by is not None:
            target = target + vector_of(by, "hand(by=...)")
        seen = None
        if look_at is not None:
            if point != "camera":
                raise Refused("hand(look_at=...) turns the hand so that its wrist camera looks at that point: give "
                              "point='camera' (xyz is then where the camera goes)")
            if any(v is not None for v in (facing, normal, finger, along)):
                raise Refused("hand: look_at sets the hand's orientation; leave out facing, normal, finger and along")
            seen = point_of(look_at)
            if float(np.linalg.norm(seen - target)) < LOOK_MIN_M:
                raise Refused(f"hand(look_at=...): the camera goes at least {LOOK_MIN_M:g} m from what it looks at")
        orient = self._orientation(side, facing, normal, finger, along)
        if xyz is None and by is None and seen is None and all(v is None for v in orient.values()):
            raise Refused("hand: give xyz (where the point goes), by=(dx, dy, dz), or an orientation (facing, "
                          "normal, finger, along, look_at)")
        if float(np.linalg.norm(target - now)) > HAND_MAX_M:
            raise Refused(f"hand: at most {HAND_MAX_M:g} m in one call")
        sp = self._speed(speed, "hand")
        ap = number(approach, "approach")
        if ap is not None and not 0.0 <= ap <= APPROACH_MAX_M:
            raise Refused(f"hand(approach=...): the distance (m, 0 to {APPROACH_MAX_M:g}) the hand comes in along its "
                          "palm's normal at the end")
        extra = {} if seen is None else {"look_at": [float(v) for v in seen]}
        res = self._call("hand", side=side, point=point, xyz=[float(v) for v in target], speed=sp,
                         approach=ap or None, dry_run=bool(dry_run), **orient, **extra)["result"]
        tgt = tuple(round(float(v), 3) for v in target)
        if dry_run:
            lost = res.get("lost_at")
            e_m, e_deg = float(res["error_m"]), float(res["error_deg"])
            ok = e_m <= DRY_RUN_M and e_deg <= DRY_RUN_DEG and lost is None
            edge = not ok and lost is None and e_m <= DRY_RUN_EDGE_M and e_deg <= DRY_RUN_EDGE_DEG
            if lost is not None:
                note = lost_note(res.get("lost_limits"))
            elif ok:
                note = None
            elif edge:
                note = edge_note(res.get("at_limit"))
            else:
                note = far_note(res.get("at_limit"))
            return R.DexterousMove(side=side, point=point, reached=ok, edge=edge, at=tgt, target=tgt,
                                   err_mm=1000.0 * float(res["error_m"]), err_deg=float(res["error_deg"]),
                                   dry_run=True, at_limit=[int(k) for k in res.get("at_limit") or []],
                                   lost_at=None if lost is None else float(lost),
                                   lost_mm=1000.0 * float(res.get("lost_m") or 0.0),
                                   lost_deg=float(res.get("lost_deg") or 0.0), notes=[note] if note else [])
        notes = [stopped_note(res)] if res.get("blocked") else []
        if seen is not None:
            h = self._hand_now(side)
            gap = float(np.linalg.norm(seen - np.asarray(h["camera"], float)))
            notes.append(f"the {side}_wrist camera looks at {_xyz(seen)} from {gap:.2f} m; the top of its picture is "
                         f"toward {direction_words(h['picture_up'])}")
        notes += self._motion_notes()
        return R.DexterousMove(side=side, point=point, reached=bool(res["reached"]), at=tuple(res["at"]), target=tgt,
                               err_mm=1000.0 * float(res["error_m"]), err_deg=float(res["error_deg"]),
                               blocked=res.get("blocked"), steps=int(res["steps"]), steps_left=self._left(),
                               notes=notes)

    def _arc(self, side, center, angle, *, axis=(0.0, 0.0, 1.0), speed=None):
        side = self._side(side)
        c = point_of(center)
        a = axis_of(axis)
        ang = number(angle, "angle")
        if ang is None or not 0.0 < abs(ang) <= 180.0:
            raise Refused("arc(side, center, angle): angle in degrees, -180 to 180 and not 0 (positive: "
                          "counter-clockwise seen from the axis's tip, the right-hand rule)")
        sp = self._speed(speed, "arc")
        palm = self._point_now(side, "palm")
        goal = rotation(a, math.radians(ang)) @ (palm - c) + c
        res = self._call("arc", side=side, center=[float(v) for v in c], axis=[float(v) for v in a], angle=ang,
                         speed=sp)["result"]
        at = np.asarray(res["palm"], float)
        notes = [stopped_note(res, "arc")] if res.get("blocked") else []
        notes += self._motion_notes()
        return R.DexterousMove(side=side, point="palm", reached=bool(res["reached"]), at=tuple(round(float(v), 3)
                                                                                              for v in at),
                               target=tuple(round(float(v), 3) for v in goal),
                               err_mm=1000.0 * float(np.linalg.norm(at - goal)), blocked=res.get("blocked"),
                               turned=float(res["turned_deg"]), asked_turn=float(res["asked_deg"]),
                               steps=int(res["steps"]), steps_left=self._left(), notes=notes)

    def _home(self, side=None):
        if side is not None:
            side = self._side(side)
        res = self._call("home", side=side)["result"]
        return R.DexterousHome(sides={s: v["blocked"] for s, v in res["sides"].items()}, steps=int(res["steps"]),
                               steps_left=self._left(), notes=self._motion_notes())

    # -- fingers ---------------------------------------------------------------------------------
    def _fingers(self, side, shape=None, *, amount=1.0, thumb=None, index=None, middle=None, ring=None, little=None,
                 oppose=None, spread=None, seconds=0.5):
        side = self._side(side)
        if shape is not None and shape not in P.SHAPES:
            raise Refused(f"fingers(side, shape): a shape is one of {', '.join(P.SHAPES)}")
        amt = number(amount, "amount")
        if amt is None or not 0.0 <= amt <= 1.0:
            raise Refused("fingers(amount=...): 0 (open) to 1 (the whole shape)")
        curl = {}
        for name, v in (("index", index), ("middle", middle), ("ring", ring), ("little", little)):
            c = number(v, name)
            if c is not None:
                if not 0.0 <= c <= 1.0:
                    raise Refused(f"fingers({name}=...): a finger's curl is 0 (straight) to 1 (curled in)")
                curl[name] = c
        t, o, s = number(thumb, "thumb"), number(oppose, "oppose"), number(spread, "spread")
        for v, name, lo, hi, what in ((t, "thumb", 0.0, 1.0, "the thumb's curl"),
                                      (o, "oppose", 0.0, 1.0, "how far the thumb swings across the palm"),
                                      (s, "spread", -1.0, 1.0, "the fingers' spread (negative: together)")):
            if v is not None and not lo <= v <= hi:
                raise Refused(f"fingers({name}=...): {what}, {lo:g} to {hi:g}")
        sec = number(seconds, "seconds")
        if sec is None or not FINGER_SECONDS[0] <= sec <= FINGER_SECONDS[1]:
            raise Refused(f"fingers(seconds=...): {FINGER_SECONDS[0]:g} to {FINGER_SECONDS[1]:g} s")
        if shape is None and not curl and t is None and o is None and s is None:
            raise Refused("fingers: give a shape and/or settings (thumb, index, middle, ring, little, oppose, spread)")
        asked_before = dict(self._hand_now(side).get("asked") or {})
        held_before = self.holding[side]
        res = self._call("fingers", side=side, shape=shape, amount=amt, curl=curl or None, thumb=t, oppose=o,
                         spread=s, seconds=sec)["result"]
        holding = bool(res["holding"])
        pressing = list(res.get("pressing") or [])
        notes = []
        if held_before and not holding and self.scene is not None:
            self.scene.released(self.grasp_pose(side), hand=side)
        if holding and not held_before and self.scene is not None:
            self.scene.grasped(self.grasp_pose(side), hand=side)
        self.holding[side] = holding
        closing = sum(res["asked"].values()) > sum(asked_before.values()) + 0.05 if asked_before else False
        if closing and not holding:
            if pressing:
                notes.append(f"{', '.join(pressing)} pressing on something, but a hold needs the thumb and one more "
                             "finger pressing: the hand holds nothing")
            else:
                notes.append("the fingers closed without pressing on anything: the hand holds nothing")
        notes += self._grip_notes()
        return R.DexterousFingers(side=side, shape=str(res["shape"]), curl=dict(res["curl"]), asked=dict(res["asked"]),
                                  pressing=pressing, holding=holding, steps=int(res["steps"]), steps_left=self._left(),
                                  notes=notes)

    def _wait(self, n):
        try:
            n = int(n)
        except (TypeError, ValueError):
            raise Refused(f"wait(n): n is a whole number of steps, 1..{P.WAIT_MAX_STEPS}") from None
        if not 0 < n <= P.WAIT_MAX_STEPS:
            raise Refused(f"wait(n): n is a whole number of steps, 1..{P.WAIT_MAX_STEPS}")
        res = self._call("wait", n=n)["result"]
        return R.Waited(steps=int(res["steps"]), step_s=self.step_s, steps_left=self._left(),
                        notes=self._motion_notes())

    # -- reading ---------------------------------------------------------------------------------
    def _hand_state(self, side: str) -> R.DexterousHand:
        h = self._hand_now(side)
        return R.DexterousHand(side=side, palm=tuple(h["palm"]), normal=tuple(h["normal"]), finger=tuple(h["finger"]),
                               grasp=tuple(h["grasp"]), tips={f: tuple(v) for f, v in h["tips"].items()},
                               shape=str(h["shape"]), curl=dict(h["curl"]), pressing=list(h.get("pressing") or []),
                               holding=bool(h.get("holding")), camera=tuple(h["camera"]), view=tuple(h["view"]))

    def _task(self):
        return R.DexterousTask(instruction=str(self.facts.get("instruction") or ""), step=self.session.step,
                               step_limit=self.session.step_limit, arms=self.sides, task_name=self.facts.get("task"))

    def _state(self):
        return R.DexterousStatus(step=self.session.step, step_limit=self.session.step_limit,
                                 hands={s: self._hand_state(s) for s in self.sides})
