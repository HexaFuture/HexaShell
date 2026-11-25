"""The host side of ``robot.*`` on SIMPLE's Unitree G1: HITCH's body, hand, finger, walk and gaze commands, and
the perception of :class:`~hexashell.benches.armhost.ArmHost` on the gaze camera's RGB-D pictures.

Everything crosses to the server (:mod:`.server`) in the start frame (protocol): points in metres, angles in
degrees.  The fingers have two settings, open and closed (closing curls them until they touch and then
squeezes); what they touch comes from the hand's contacts (the Dex3's touch sensors), and a closed hand holds
something when its thumb and one more part touch it.  Perception keeps measured things on the hand that holds
them (:class:`~hexashell.vision.rgbd.DepthScene`, one hand per side).
"""

from __future__ import annotations

import math

import numpy as np

from ...errors import Refused
from ...geometry import wrap_deg
from ...robot import results as RR
from ...vision.rgbd import MOVED as MOVED_M
from .. import results as R
from ..armhost import ArmHost, camera_checker, number
from ..armhost import point_of as arm_point_of
from . import protocol as P

PUBLIC = ("body", "walk", "hand", "fingers", "gaze", "wait", "find", "measure", "to_world", "to_pixel", "task",
          "state", "image", "depth", "look", "camera", "things")
#: what a hand command's hold may be (control.HOLD_MODES)
HOLDS = ("torso", "world", "pelvis")
#: farthest one walk goes (m)
WALK_MAX_M = 4.0
#: a hand move within this much of its target (m) reached it
HAND_REACHED_M = 0.02
#: the point a hand carries things at: this far (m) out from the palm along its normal and ahead of the palm's
#: centre along the fingers, between the thumb and the curled fingers (the Dex3's closed shape)
GRASP_OFFSET_M = 0.04
GRASP_AHEAD_M = 0.03
#: the parts of a hand that touch: a closed hand holds when the thumb and one more touch
PARTS = ("thumb", "index", "middle", "palm")
#: a held palm this far (m) from where it is kept, after a body command: say so
HELD_OFF_M = 0.02


def holds(touching) -> bool:
    parts = set(touching or ())
    return "thumb" in parts and len(parts) >= 2


def unit_of(value, name: str) -> np.ndarray:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"{name} is a direction: three numbers (x, y, z)") from None
    n = float(np.linalg.norm(v)) if v.size == 3 and np.all(np.isfinite(v)) else 0.0
    if n < 1e-6:
        raise Refused(f"{name} is a direction: three numbers (x, y, z), not all zero")
    return v / n


def point_of(value) -> np.ndarray:
    """A start-frame point from three numbers, a Spot, a Thing (its centre), a HandMove or a PalmState (the palm)."""
    if isinstance(value, (R.HandMove, R.PalmState)):
        value = value.at if isinstance(value, R.HandMove) else value.point
    return arm_point_of(value)


def rot_z_deg(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class SimpleHost(ArmHost):
    public = PUBLIC
    cams = P.CAMERAS
    aliases = P.CAMERA_ALIASES
    step_s = P.STEP_S

    def __init__(self, session, opened: dict, *, scene=None, log=None, attach=None, on_rows=None):
        # ArmHost's own __init__ is built round one arm's tool pose; the parts of it the shared calls use
        self.session = session
        self.log = log or (lambda event: None)
        self.attach = attach or (lambda kind, image, caption: None)
        self.on_rows = on_rows or (lambda rows: None)
        self.check_cam = camera_checker(self.cams, self.aliases)
        self.facts = dict(opened.get("facts") or {})
        self.proprio = opened["start"]
        self.start = opened["start"]
        self.dropped = False
        self.holding = {s: False for s in P.SIDES}
        self.scene = scene
        self._views = None
        if scene is not None:
            scene.bind(self.views, hands={s: (lambda s=s: self.grasp_pose(s)) for s in P.SIDES})

    # -- readings --------------------------------------------------------------------------------
    def grasp_pose(self, side: str) -> np.ndarray:
        """The pose things a hand holds are carried at: x the fingers' direction, y the palm's normal, between
        the thumb and the curled fingers."""
        palm = self.proprio["palms"][side]
        x = unit_of(palm["finger"], "finger")
        y = np.asarray(palm["normal"], float)
        y = y - x * float(x @ y)
        y = y / max(1e-9, float(np.linalg.norm(y)))
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, y, np.cross(x, y)
        T[:3, 3] = np.asarray(palm["point"], float) + GRASP_OFFSET_M * y + GRASP_AHEAD_M * x
        return T

    def _heading(self) -> float:
        return float(self.proprio["heading"])

    def _palm(self, side: str) -> R.PalmState:
        p = self.proprio
        palm = p["palms"][side]
        touching = list(p.get("touching", {}).get(side, []))
        fingers = p.get("fingers", {}).get(side, "open")
        closed = fingers == "closed"
        return R.PalmState(point=tuple(palm["point"]), normal=tuple(round(v, 3) for v in palm["normal"]),
                           finger=tuple(round(v, 3) for v in palm["finger"]),
                           fingers=fingers if fingers in ("closed", "wide") else "open", touching=touching,
                           holding=self.holding[side] if closed else None, hold=p.get("holds", {}).get(side, "torso"))

    def _steps(self, res: dict) -> int:
        return int(round(float(res.get("seconds") or 0.0) / P.STEP_S))

    def _motion_notes(self) -> list[str]:
        """After a motion: what a hand that holds nothing touched (now, or only on the way: it may have
        pushed something), then what slipped out of a hand that held it (``_grip_notes``)."""
        notes = []
        for side in P.SIDES:
            if self.holding[side]:
                continue
            now = self.proprio.get("touching", {}).get(side, [])
            on_the_way = self.proprio.get("touched", {}).get(side, [])
            if now:
                notes.append(f"the {side} hand touches something ({', '.join(now)})")
            elif on_the_way:
                notes.append(f"the {side} hand touched something on the way ({', '.join(on_the_way)}): it may have "
                             "pushed or tipped it - look before going on")
        return notes + self._grip_notes()

    def _grip_notes(self) -> list[str]:
        """After a motion: a hand that held something and now does not (what it held slipped out)."""
        notes = []
        for side in P.SIDES:
            if not self.holding[side]:
                continue
            touching = self.proprio.get("touching", {}).get(side, [])
            if holds(touching):
                continue
            self.holding[side] = False
            self.dropped = True
            if self.scene is not None:
                self.scene.released(self.grasp_pose(side), hand=side)
            now = ", ".join(touching) if touching else "nothing"
            notes.append(f"the {side} hand no longer holds with its thumb and one more part (touching: {now}): what "
                         "it held may have slipped out - look before going on")
        return notes

    @staticmethod
    def _side(side) -> str:
        if side not in P.SIDES:
            raise Refused(f"side is 'left' or 'right', not {side!r}")
        return side

    # -- body ------------------------------------------------------------------------------------
    def _body(self, height=None, *, pitch=None, yaw=None):
        h, p, y = number(height, "height"), number(pitch, "pitch"), number(yaw, "yaw")
        if h is None and p is None and y is None:
            raise Refused("body: give height (m), pitch or yaw (degrees)")
        for v, (lo, hi), what in ((h, P.PELVIS_HEIGHT_RANGE_M, "height is the pelvis's height above the ground, "),
                                  (p, P.TORSO_PITCH_RANGE_DEG, "pitch is the torso's forward lean, "),
                                  (y, P.TORSO_YAW_RANGE_DEG, "yaw is the torso's turn left of the pelvis, ")):
            if v is not None and not lo <= v <= hi:
                raise Refused(f"body: {what}{lo:g} to {hi:g}")
        res = self._call("body", height=h, pitch=p, yaw=y)["result"]
        if "seconds" not in res:
            return None  # the episode ended during it (dispatch says so)
        notes = []
        reached_h = float(res["height_m"]["reached"])
        if "height command at its limit" in res.get("limits", []) and h is not None and abs(reached_h - h) > 0.01:
            notes.append(f"the controller's height command is at its limit: the pelvis goes no {'lower' if h < reached_h else 'higher'} "
                         f"than about {reached_h:.2f} m")
        if "pitch command at its limit" in res.get("limits", []):
            notes.append("the controller's torso pitch command is at its limit")
        for side, held in (res.get("hands") or {}).items():
            if held.get("position_error_m", 0.0) > HELD_OFF_M:
                notes.append(f"the {side} palm (hold {held['hold']}) is {100 * held['position_error_m']:.0f} cm from "
                             "where it is kept: the arm cannot keep it there from this body pose")
        notes += self._motion_notes()
        return R.BodyMove(reached=bool(res["reached"]), height=reached_h, pitch=float(res["pitch_deg"]["reached"]),
                          yaw=float(res["yaw_deg"]["reached"]), asked_height=h, asked_pitch=p, asked_yaw=y,
                          steps=self._steps(res), steps_left=self._left(), notes=notes)

    # -- walking ---------------------------------------------------------------------------------
    def _walk(self, x=None, y=None, *, heading=None, by=None, turn=None):
        px, py = (float(v) for v in self.proprio["pelvis"][:2])
        h0 = self._heading()
        x, y, heading, turn = number(x, "x"), number(y, "y"), number(heading, "heading"), number(turn, "turn")
        if (x is None) != (y is None):
            raise Refused("walk: give both x and y (a point of the start frame), or by=(forward, left)")
        if by is not None:
            try:
                fwd, left = (float(v) for v in by)
            except (TypeError, ValueError):
                raise Refused("walk(by=...): (forward, left) in metres, from the pelvis now") from None
            bx, by_ = (px, py) if x is None else (x, y)
            d = rot_z_deg(h0)[:2, :2] @ np.array([fwd, left])
            x, y = bx + float(d[0]), by_ + float(d[1])
        if x is None:
            x, y = px, py
        if turn is not None:
            if heading is not None:
                raise Refused("walk: heading (in the start frame) or turn (from the heading now), not both")
            heading = wrap_deg(h0 + turn)
        if math.hypot(x - px, y - py) > WALK_MAX_M:
            raise Refused(f"walk: at most {WALK_MAX_M:g} m in one call")
        res = self._call("walk", x=x, y=y, heading=heading)["result"]
        if "seconds" not in res:
            return None
        ax, ay, ah = res["reached_pose"]
        notes = []
        if res.get("blocked"):
            notes.append(f"the walk stopped: {res['blocked']} - something is in the way (the table's edge, or a leg "
                         "against it); look before going on")
        elif res.get("near_stopped"):
            notes.append("these legs get no nearer than this: another walk this short only sways in place - make up "
                         "the rest with the hands (or the torso's lean and yaw)")
        elif not res["reached"]:
            notes.append("the pelvis stopped short: walk again for what is left")
        notes += self._motion_notes()
        return R.Walk(reached=bool(res["reached"]), blocked=bool(res.get("blocked")), at=(ax, ay), heading=ah,
                      target=(round(x, 3), round(y, 3)), target_heading=float(res["asked"][2]),
                      err_mm=1000.0 * float(res["position_error_m"]), err_deg=abs(float(res["heading_error_deg"])),
                      steps=self._steps(res), steps_left=self._left(), notes=notes)

    # -- hands -----------------------------------------------------------------------------------
    def _orientation(self, side: str, facing, normal, finger):
        n = f = None
        if facing is not None:
            if facing not in P.PALM_FACING:
                raise Refused(f"hand(facing=...): one of {', '.join(P.PALM_FACING)}")
            rn, rf = P.palm_axes(facing, side)
            Rz = rot_z_deg(self._heading())
            n, f = Rz @ np.array(rn), Rz @ np.array(rf)
        if normal is not None:
            n = unit_of(normal, "normal")
        if finger is not None:
            f = unit_of(finger, "finger")
        if n is not None and f is not None and abs(float(n @ f)) > math.cos(math.radians(60.0)):
            raise Refused("hand: the palm's normal and the fingers' direction must be at least 60 degrees apart "
                          "(they are at right angles on the hand)")
        return n, f

    def _hand(self, side, xyz=None, *, by=None, facing=None, normal=None, finger=None, seconds=1.0, via=None,
              hold="torso", dry_run=False):
        side = self._side(side)
        if hold not in HOLDS:
            raise Refused(f"hand(hold=...): one of {', '.join(HOLDS)}")
        now = np.asarray(self.proprio["palms"][side]["point"], float)
        target = now if xyz is None else point_of(xyz)
        if by is not None:
            try:
                d = np.asarray(by, float).reshape(3)
            except (TypeError, ValueError):
                raise Refused("hand(by=...): (dx, dy, dz) in metres, start frame") from None
            target = target + d
        n, f = self._orientation(side, facing, normal, finger)
        s = number(seconds, "seconds")
        lo, hi = P.HAND_SEGMENT_S
        if s is None or not lo <= s <= hi:
            raise Refused(f"hand(seconds=...): each segment takes {lo:g} to {hi:g} s")
        points = [point_of(v) for v in (via or [])]
        if len(points) + 1 > P.HAND_WAYPOINTS_MAX:
            raise Refused(f"hand: at most {P.HAND_WAYPOINTS_MAX - 1} via points")
        waypoints = [{"point": [float(v) for v in p], "seconds": s} for p in points]
        waypoints.append({"point": [float(v) for v in target], "seconds": s,
                          "normal": None if n is None else [float(v) for v in n],
                          "finger": None if f is None else [float(v) for v in f]})
        res = self._call("hand", side=side, waypoints=waypoints, hold=hold, dry_run=bool(dry_run))["result"]
        tgt = tuple(round(float(v), 3) for v in target)
        if dry_run:
            err = float(res["position_error_m"])
            notes = []
            if res.get("at_limit"):
                notes.append(f"the arm would be at a joint limit ({', '.join(res['at_limit'])})")
            return R.HandMove(side=side, reached=err <= HAND_REACHED_M / 2, at=tuple(res["reached_point"]), target=tgt,
                              err_mm=1000.0 * err, normal_err_deg=res.get("normal_error_deg"),
                              finger_err_deg=res.get("finger_error_deg"), hold=hold, dry_run=True, notes=notes)
        if "seconds" not in res:
            return None
        err = float(res["position_error_m"])
        notes = []
        if res.get("at_limit"):
            notes.append(f"the arm is at a joint limit ({', '.join(res['at_limit'])}): from this body pose the palm "
                         "gets no farther - walk closer, or change the body's height, lean or turn")
        if res.get("slowed_by_speed_limit_s"):
            notes.append(f"the arm's speed limit made the move {res['slowed_by_speed_limit_s']:.1f} s longer")
        notes += self._motion_notes()
        return R.HandMove(side=side, reached=err <= HAND_REACHED_M, at=tuple(res["reached_point"]), target=tgt,
                          err_mm=1000.0 * err, normal_err_deg=res.get("normal_error_deg"),
                          finger_err_deg=res.get("finger_error_deg"), hold=hold, steps=self._steps(res),
                          steps_left=self._left(), notes=notes)

    def _fingers(self, side, closed):
        side = self._side(side)
        words = {"open": False, "close": True, "closed": True, "wide": "wide"}
        how = "fingers(side, closed): True / 'close' to close, False / 'open' to open, 'wide' to open with the thumb back"
        if isinstance(closed, str):
            if closed.lower() not in words:
                raise Refused(how)
            closed = words[closed.lower()]
        elif not isinstance(closed, (bool, np.bool_)):
            raise Refused(how)
        wide = closed == "wide"
        closed = False if wide else bool(closed)
        held_before = self.holding[side]
        res = self._call("fingers", side=side, closed="wide" if wide else closed)["result"]
        if "seconds" not in res:
            return None
        touching = list(res.get("touching", []))
        notes = []
        if closed:
            self.holding[side] = holds(touching)
            if self.holding[side] and self.scene is not None:
                self.scene.grasped(self.grasp_pose(side), hand=side)
            if not self.holding[side]:
                notes.append("the fingers closed on nothing they can hold (a hand holds with the thumb and one more "
                             "part touching)")
                on_the_way = self.proprio.get("touched", {}).get(side, [])
                if on_the_way and not touching:
                    notes.append(f"while closing they touched something ({', '.join(on_the_way)}) and nothing now: "
                                 "they may have pushed or tipped it - look before going on")
        else:
            if held_before and self.scene is not None:
                self.scene.released(self.grasp_pose(side), hand=side)
            self.holding[side] = False
        notes += self._grip_notes()
        return R.Fingers(side=side, closed=closed, touching=touching, holding=self.holding[side] if closed else None,
                         steps=self._steps(res), steps_left=self._left(), notes=notes, wide=wide)

    # -- the gaze camera -------------------------------------------------------------------------
    def _gaze(self, yaw=None, pitch=None, *, at=None):
        yaw, pitch = number(yaw, "yaw"), number(pitch, "pitch")
        notes = []
        if at is not None:
            if yaw is not None or pitch is not None:
                raise Refused("gaze: yaw and pitch, or at=(x, y, z), not both")
            point = [float(v) for v in point_of(at)]
            res = self._call("gaze", at=point)["result"]
        else:
            if yaw is None and pitch is None:
                raise Refused("gaze: give yaw and/or pitch (degrees), or at=(x, y, z)")
            for v, (lo, hi), what in ((yaw, P.GAZE_YAW_RANGE_DEG, "yaw (left of the torso's forward)"),
                                      (pitch, P.GAZE_PITCH_RANGE_DEG, "pitch (down from the torso's horizontal)")):
                if v is not None and not lo <= v <= hi:
                    raise Refused(f"gaze: {what} is {lo:g} to {hi:g} degrees")
            res = self._call("gaze", yaw=yaw, pitch=pitch)["result"]
        if "seconds" not in res:
            return None
        (gy, gp), (ay, ap) = res["gaze_deg"], res["asked_deg"]
        if at is not None and (abs(gy - ay) > 0.5 or abs(gp - ap) > 0.5):
            ways = []
            if abs(gy - ay) > 0.5:
                ways.append("turn the torso (body yaw)")
            if ap > gp + 0.5:
                ways.append("lean forward (body pitch) or step back")
            elif ap < gp - 0.5:
                ways.append("stand up straighter or step back")
            notes.append(f"that point is at yaw {ay:.0f} pitch {ap:.0f} from the camera, outside its range: "
                         f"{' or '.join(ways) or 'walk'} to bring it in view")
        notes += self._grip_notes()
        return R.Gaze(yaw=gy, pitch=gp, asked_yaw=ay, asked_pitch=ap, steps=self._steps(res), steps_left=self._left(),
                      notes=notes)

    # -- perception ------------------------------------------------------------------------------
    def _measure(self, target, *, cams="auto"):
        """As the scene measures; a re-measured Thing's "moved" note judged by its top and ground place.

        The scene compares the centres of what the camera saw; with the one gaze camera that centre moves
        with where it looks from (from near and high only the top is in view: the centre rises by up to
        half the object's height), so a still object read as moved.  Here the note stays only when the top
        or the centre's place on the ground moved more than the scene's MOVED."""
        thing = super()._measure(target, cams=cams)
        notes = getattr(thing, "notes", None)
        if (not isinstance(target, RR.Thing) or not getattr(thing, "found", False) or not notes
                or not notes[0].startswith("moved ") or target.center_xyz is None or thing.center_xyz is None
                or target.top is None or thing.top is None):
            return thing
        a, b = np.asarray(target.center_xyz, float), np.asarray(thing.center_xyz, float)
        moved = max(float(np.linalg.norm(b[:2] - a[:2])), abs(float(thing.top) - float(target.top)))
        notes.pop(0)
        if moved > MOVED_M:
            notes.insert(0, f"moved {100 * moved:.1f} cm since it was measured")
        return thing

    def _wait(self, n):
        try:
            n = int(n)
        except (TypeError, ValueError):
            raise Refused(f"wait(n): n is a whole number of steps, 1..{P.WAIT_MAX_STEPS}") from None
        if not 0 < n <= P.WAIT_MAX_STEPS:
            raise Refused(f"wait(n): n is a whole number of steps, 1..{P.WAIT_MAX_STEPS}")
        res = self._call("wait", n=n)["result"]
        if "steps" not in res:
            return None
        return R.Waited(steps=int(res["steps"]), step_s=self.step_s, steps_left=self._left(), notes=self._motion_notes())

    # -- reading ---------------------------------------------------------------------------------
    def _task(self):
        return R.HumanoidTask(instruction=str(self.facts.get("instruction") or ""), step=self.session.step,
                              step_limit=self.session.step_limit, official_steps=self.facts.get("official_steps"),
                              task_name=self.facts.get("task"))

    def _state(self):
        p = self.proprio
        return R.HumanoidStatus(step=self.session.step, step_limit=self.session.step_limit,
                                official_steps=self.facts.get("official_steps"), pelvis=tuple(p["pelvis"]),
                                heading=float(p["heading"]), pelvis_height=float(p["pelvis_height"]),
                                torso_pitch=float(p["torso_pitch"]), torso_yaw=float(p["torso_yaw"]),
                                left=self._palm("left"), right=self._palm("right"), gaze=tuple(p["gaze"]))
