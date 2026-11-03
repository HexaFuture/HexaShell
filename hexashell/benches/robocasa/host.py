"""The host side of ``robot.*`` in RoboCasa: the arm, gripper and cameras of
:class:`~hexashell.benches.armhost.ArmHost`, and the base (``drive``).

Poses cross to the server as 4x4 tool poses in the start frame (see :mod:`.sim`).  ``home``
returns to the start pose relative to the base, where the base is now.
"""

from __future__ import annotations

import math

import numpy as np

from ...errors import Refused
from ...geometry import wrap_deg
from ...kernel.host import CallFailed
from ...robot import results as RR
from ...vision.rim import rim_of
from .. import results as R
from ..arm import planar_pose
from ..armhost import ARM_CALLS, GIVE_WAY_CM, ArmHost, brief, camera_checker, describe, gave_way_how, number, point_of  # noqa: F401  (describe: for the session)
from . import protocol as P

PUBLIC = ARM_CALLS + ("drive", "push_arc", "rim_pick")
CAMS = tuple(P.CAMERAS)
ALIASES = {"agentview_left": "left", "robot0_agentview_left": "left", "agentview_right": "right",
           "robot0_agentview_right": "right", "eye_in_hand": "wrist", "robot0_eye_in_hand": "wrist", "hand": "wrist"}
check_cam = camera_checker(CAMS, ALIASES)
#: the gap between the finger pads when fully open (m)
FINGER_STROKE_M = 0.08
#: farthest the base drives in one call (m)
DRIVE_MAX_M = 4.0
#: rim_pick: the hand comes down ABOVE_M from over the pinch point at DOWN_SPEED (m/s); a close under EMPTY_M
#: (m) holds nothing (replays on three bowls: pinches that held closed on 1.5-2.6 cm, empty ones on 0.2-0.4 cm),
#: and then it tries once more RETRY_OUT_M nearer the rim's edge (pinches 2.3-3.9 cm in from it closed empty);
#: the dish within FACE_DEG of the base's heading; the hint puts its middle STAND_M ahead of the base
RIM_ABOVE_M = 0.10
RIM_DOWN_SPEED = 0.05
RIM_EMPTY_M = 0.008
RIM_RETRY_OUT_M = 0.007
RIM_FACE_DEG = 50.0
RIM_STAND_M = 0.6
#: a dish held by its rim settles on in the fingers: in scripted carries the gap narrowed up to 8 mm after the lift
#: (in 6 of 22 that hit nothing 4 mm or more) and none fell; stop_on_slip for such a grasp
RIM_SLIP_M = 0.012
#: depth points this far across (m) from the dish's point go to rim_of
RIM_CLOUD_M = 0.45


class CasaHost(ArmHost):
    public = PUBLIC
    cams = CAMS
    aliases = ALIASES
    finger_stroke_m = FINGER_STROKE_M
    step_s = P.STEP_S
    unreachable_note = ("from here the arm cannot get there: drive the base closer (robot.drive), or ask for another "
                        "pitch or height")
    stalled_advice = "drive the base closer or ask for another pitch"
    stretched_advice = "drive the base closer"
    jammed_advice = ("if it touched something on the way (a shelf above, a tap, a door), come from another side "
                     "or at another angle; if nothing was in the way, it reached too far: drive the base closer "
                     "first, so that the arm reaches with the elbow well bent")
    #: a drop on the way stops the cell: carrying on with an empty hand wastes the steps that bind here
    stop_on_drop = True
    #: casa_dev4: of 44 things that fell while the base drove or the arm went home, 21 had slipped 4 mm or
    #: more since the first move of the grasp (4 more fell later in arm motions); of the ~690 grasps that did
    #: not fall, 17 would have stopped too.  Measured from the close, 31 and 56: things settle in the fingers
    #: on the first move (a straw turns, a bowl hangs from its rim) and mostly hold after that
    stop_on_slip = 0.004
    #: casa_val1-4 and a smoke batch, 1219 closes matched to the thing measured under them: of 209 on things under
    #: 6 cm high with the grasp point less than 5 mm under their top (or over it), 51% slipped out within 15 steps
    #: (before the hand had risen); of 462 closed 1-3 cm under it, 8%.  Things under 4 cm held through the carry
    #: after 91% of 153 closes within 3 mm of the height they stand on, 56% of 128 closes 1-1.5 cm over it
    top_grasp_note = True
    #: casa_dev4, casa_val1-6 and doorsmoke1, moves down pointing down with the fingers open that stopped short of
    #: a target under the hand: of 145 closes right after one with the grasp point at the top of a thing measured
    #: under it (0.5 cm under to 3 cm over its top, 1.2 cm or more over what it stands on), 17% held; of 66 after
    #: one 1 cm or more under the top, 35%; after moves that reached 1 cm or more under the top, 79% of 575.
    #: Replayed, 37 of those stops whose close slipped out at once held 1 or 2 times with the hand pressed 0.5 or
    #: 1 cm lower, or 0.5 cm higher, before closing: where the fingers are, not how hard they press.  Of 61 later
    #: grasps that held after such a close slipped out, the median was 2.5 cm across and 1.9 cm lower
    top_stop_note = True
    #: replays of casa_val1-5 holds, the fingers eased at once as gripper('light') does: of carries that fell while
    #: squeezed, small soft food (vegetables, meat, strawberries, sausages, buns, kebab, ice) held through 43 of 66,
    #: and of 77 that held squeezed 4 fell; mugs, plates, blender jars, knives, lemon wedges and baguettes held
    #: through 7 of 74 and lost 30 of 105 (they turn and slide out on the lift); door handles: 59 of 125 pulls lost
    light_hold = 0.05
    #: casa_val1, casa_val9 and casa_val12, arcs holding something, nearly all at the arm's speed: of 701 with the
    #: hand 0.25 m or more from the line (doors' handles, mostly 0.33-0.6 m), 23% slipped out; of 234 nearer (taps,
    #: knobs), 6%.  Replayed, 11 door arcs that had slipped held 7 times at 0.06-0.12 m/s and 4 times at 0.15; 4 that
    #: had held still held at 0.06.  The door's panel rubs its cabinet as it opens; at speed the squeeze does not keep
    #: the bar in.  In the same hours (casa_arcA at the arm's speed, casa_arcB at 0.1; OpenCabinet, ArrangeBreadBasket,
    #: WaffleReheat x seeds 10-19), the first arcs of a swing slipped 27 of 61 times against 15 of 54, and arcs after
    #: the door had turned 30 degrees or more with the hand on it 4 of 59 against 6 of 77, at twice the steps.
    #: Replayed, the arm's speed from 22 degrees on lost 3 of 11 doors that a slow arc kept; in casa_val1/9/12, at the
    #: arm's speed, arcs from 15-29 degrees slipped 7 of 45 times, from 29-35 degrees 4 of 109
    held_arc = (0.25, 0.1, 30.0)

    def __init__(self, session, opened: dict, **kwargs):
        super().__init__(session, opened, **kwargs)
        self.start_base = np.asarray(opened["start"]["base"], float)
        self.start_tcp_base = np.linalg.inv(planar_pose(*self.start_base)) @ self.start_tcp

    def _base(self) -> tuple[float, float, float]:
        x, y, yaw = (float(v) for v in self.proprio["base"])
        return x, y, yaw

    def home_pose(self) -> np.ndarray:
        return planar_pose(*self._base()) @ self.start_tcp_base

    def status_extra(self) -> dict:
        x, y, yaw = self._base()
        return {"base": (round(x, 4), round(y, 4)), "heading": round(wrap_deg(math.degrees(yaw)), 1)}

    # -- the base ----------------------------------------------------------------------------------
    def _drive(self, forward=0.0, left=0.0, turn=0.0, *, to=None, heading=None, about=None, dry_run=False):
        bx, by, byaw = self._base()
        relative = [number(v, name) or 0.0 for v, name in ((forward, "forward"), (left, "left"), (turn, "turn"))]
        if about is not None:
            if to is not None or heading is not None or relative[0] or relative[1]:
                raise Refused("drive(about=...): give only turn= with it (degrees, + to the left)")
            if not 0.0 < abs(relative[2]) < 180.0:
                raise Refused("drive(about=...): turn= between -180 and 180 degrees, not 0")
            ax, ay = xy_of(about, "about")
            t = math.radians(relative[2])
            c, s = math.cos(t), math.sin(t)
            tx, ty = ax + c * (bx - ax) - s * (by - ay), ay + s * (bx - ax) + c * (by - ay)
            th = byaw + t
        elif to is not None or heading is not None:
            if any(relative):
                raise Refused("drive: give forward/left/turn (from where the base is) or to=/heading= (start "
                              "frame), not both")
            tx, ty = (bx, by) if to is None else xy_of(to, "to")
            th = byaw if heading is None else math.radians(number(heading, "heading"))
        else:
            f, lft, t = relative
            tx = bx + f * math.cos(byaw) - lft * math.sin(byaw)
            ty = by + f * math.sin(byaw) + lft * math.cos(byaw)
            th = byaw + math.radians(t)
        if math.hypot(tx - bx, ty - by) > DRIVE_MAX_M:
            raise Refused(f"drive: at most {DRIVE_MAX_M:.0f} m in one call")
        way, shuts = ("", False) if about is None else self._hand_way((ax, ay), relative[2])
        if dry_run:
            hand = planar_pose(tx, ty, th) @ np.linalg.inv(planar_pose(bx, by, byaw)) @ self._tcp()
            self.log({"kind": "drive", "target": [tx, ty, th], **({} if about is None else {"about": [ax, ay]}),
                      "dry_run": True})
            return R.Drive(reached=True, blocked=False, at=(round(bx, 4), round(by, 4)),
                           heading=round(wrap_deg(math.degrees(byaw)), 1), target=(round(tx, 4), round(ty, 4)),
                           target_heading=round(wrap_deg(math.degrees(th)), 1), steps_left=self._left(),
                           notes=[way] if way else [], dry_run=True,
                           hand=tuple(round(float(v), 3) for v in hand[:3, 3]))
        self._check_slip("robot.drive()")
        res = self._call("drive", target=[tx, ty, th], **({} if about is None else {"about": [ax, ay]}))["result"]
        pose = [float(v) for v in res["pose"]]
        turned = math.degrees(pose[2] - byaw)
        self.asked = (self.asked[0], round(wrap_deg(self.asked[1] + turned), 2), self.asked[2])
        notes = self._slipped()
        if res.get("jammed") and res.get("stretched"):
            notes.append("the base stopped: the arm, held far out, was straightening its elbow (a straight arm gets "
                         f"stuck), so it {gave_way_how(res)}. Look whether the hand or what it holds catches on "
                         "something; home() before driving on")
        elif res.get("jammed"):
            notes.append("the base stopped: the arm, held out with the elbow open wider than a right angle, pressed on "
                         "something at full force, and driving on would have snapped it straight; it pulled the hand "
                         f"{GIVE_WAY_CM} cm back toward the shoulder. Look what it touched (the hand, the wrist or the "
                         "forearm) and move it clear, or home(), before driving on")
        elif res.get("off"):
            notes.append(f"the base was held off its path and stopped ({off_words(res['off'])} of where it should "
                         "have been): it touches something (a counter, an open door) - look before going on")
        elif res.get("blocked"):
            notes.append("the base stopped short: it is pushing against something (a counter, an open door) - "
                         "look before going on")
        if shuts and (res.get("blocked") or not res.get("reached")):
            notes.append(f"the hand went forward (away from the base) as the base turned: on a door's handle with "
                         f"`about` at its hinge that pushes the door shut. A door opens toward the base: "
                         f"turn={-relative[2]:g}")
        self.log({"kind": "drive", "target": [tx, ty, th], **({} if about is None else {"about": [ax, ay]}),
                  "result": brief(res)})
        return R.Drive(reached=bool(res.get("reached")), blocked=bool(res.get("blocked")),
                       at=(round(pose[0], 4), round(pose[1], 4)), heading=round(wrap_deg(math.degrees(pose[2])), 1),
                       target=(round(tx, 4), round(ty, 4)), target_heading=round(wrap_deg(math.degrees(th)), 1),
                       err_mm=round(1000.0 * float(res["err_m"]), 1), err_deg=round(float(res["err_deg"]), 1),
                       steps=int(res["steps"]), steps_left=self._left(), notes=notes)

    def _hand_way(self, about, turn: float) -> tuple[str, bool]:
        """Which way the grasp point first goes as the base turns ``turn`` degrees about the floor point
        ``about``, and whether that is mostly forward, away from the base, with something held (on a
        door's handle, about its hinge: pushing the door shut)."""
        _, _, byaw = self._base()
        dx, dy = (float(v) for v in self._tcp()[:2, 3] - np.asarray(about, float))
        c, s = math.cos(byaw), math.sin(byaw)
        ahead, aside = dx * c + dy * s, dy * c - dx * s  # the hand from the point, in the base's frame
        sign = 1.0 if turn > 0 else -1.0
        fwd, lft = -sign * aside, sign * ahead  # its first direction
        r = math.hypot(fwd, lft)
        if r < 0.02:
            return f"the hand stays within {100 * r:.0f} cm of the point: it turns in place", False
        parts = []
        if abs(fwd) >= 0.3 * r:
            parts.append("forward (away from the base)" if fwd > 0 else "back (toward the base)")
        if abs(lft) >= 0.3 * r:
            parts.append("to the left" if lft > 0 else "to the right")
        shuts = bool(self.holding) and fwd > 0.7 * r
        way = f"the hand goes round the point {r:.2f} m from it, first {' and '.join(parts)}"
        if shuts:
            way += (f". On a door's handle with the point at its hinge, that pushes the door shut; a door opens "
                    f"toward the base: turn={-turn:g}")
        return way, shuts

    # -- a dish by its rim -------------------------------------------------------------------------
    def _rim_pick(self, target, *, pitch=45.0, inset=0.012, under=0.015, lift=0.10, outer=False, dry_run=False):
        tilt, ins, down, up = (number(v, n) for v, n in ((pitch, "pitch"), (inset, "inset"), (under, "under"),
                                                         (lift, "lift")))
        if tilt is None or not 30.0 <= tilt <= 60.0:
            raise Refused("rim_pick: pitch is 30 to 60 degrees (the hand keeps it while it holds the dish)")
        if ins is None or not 0.004 <= ins <= 0.03:
            raise Refused("rim_pick: inset is how far in from the rim's outer edge the grasp point goes, 0.004 to "
                          "0.03 (m)")
        if down is None or not 0.005 <= down <= 0.03:
            raise Refused("rim_pick: under is how far under the rim's top the grasp point goes, 0.005 to 0.03 (m)")
        if up is None or not 0.0 <= up <= 0.25:
            raise Refused("rim_pick: lift is 0 to 0.25 (m)")
        if self.holding and not dry_run:
            raise Refused("rim_pick: the hand holds something: set it down and open first")
        p = self._dish_point(target)
        bx, by, byaw = self._base()
        for toward in (p[:2] - (bx, by), None):
            if toward is None:
                rim, why = rim_of(self._cloud(p), p, (bx, by), outer=bool(outer))
                if rim is None:
                    raise Refused(f"rim_pick: {why}")
                toward = rim.toward
            rel = wrap_deg(math.degrees(math.atan2(toward[1], toward[0]) - byaw))
            if abs(rel) > RIM_FACE_DEG:
                raise Refused(f"rim_pick: the dish is {abs(rel):.0f} degrees to the {'left' if rel > 0 else 'right'} "
                              f"of the base's heading: turn the base toward it first (robot.drive(turn={rel:.0f}))")
        u = np.array(rim.toward)
        angles = (tilt, round(math.degrees(math.atan2(u[1], u[0])), 1), 90.0)

        def pinch_at(i: float) -> np.ndarray:
            return np.array([rim.edge[0] + u[0] * i, rim.edge[1] + u[1] * i, rim.top - down])

        def result(**kw) -> R.RimPick:
            tcp = self._tcp()
            base = dict(center=tuple(round(v, 4) for v in rim.center), radius=round(rim.radius, 4),
                        top=round(rim.top, 4), on_z=round(rim.on_z, 4), angles=angles,
                        at=tuple(round(float(v), 4) for v in tcp[:3, 3]), steps_left=self._left())
            return R.RimPick(**{**base, **kw})

        g = pinch_at(ins)
        plan = [g + (0.0, 0.0, RIM_ABOVE_M), g, g + (0.0, 0.0, up)]
        checks = [self._goto(q, pitch=angles[0], yaw=angles[1], roll=angles[2], dry_run=True) for q in plan]
        bad = next((m for m in checks if not m.reached), None)
        self.log({"kind": "rim_pick", "rim": {"center": rim.center, "radius": rim.radius, "top": rim.top,
                                              "on_z": rim.on_z, "edge": rim.edge, "points": rim.points},
                  "angles": angles, "pinch": g, "outer": bool(outer), "reachable": bad is None,
                  "dry_run": bool(dry_run)})
        pinch = tuple(round(float(v), 4) for v in g)
        if bad is not None:
            d = np.asarray(rim.center) - np.array([bx, by])
            ahead = float(d @ np.array([math.cos(byaw), math.sin(byaw)]))
            aside = float(d @ np.array([-math.sin(byaw), math.cos(byaw)]))
            hint = (f"its middle is {ahead:.2f} m ahead of the base and {abs(aside):.2f} m to the "
                    f"{'left' if aside > 0 else 'right'}: robot.drive(forward={ahead - RIM_STAND_M:.2f}, "
                    f"left={aside:.2f}) puts it {RIM_STAND_M:g} m straight ahead")
            out = result(holding=False, pinch=pinch, dry_run=bool(dry_run), executed=False,
                         why=f"{bad.why or 'out of reach'} at {tuple(round(v, 3) for v in bad.target)}", notes=[hint])
            if dry_run:
                return out
            raise CallFailed("NotExecuted", str(out))
        if dry_run:
            return result(holding=False, pinch=pinch, dry_run=True)
        steps, notes, tries, width = 0, [], 0, None
        insets = [ins] + ([ins - RIM_RETRY_OUT_M] if ins - RIM_RETRY_OUT_M >= 0.004 else [])
        why = ""
        for i in insets:
            g = pinch_at(i)
            pinch = tuple(round(float(v), 4) for v in g)
            tries += 1
            m = self._goto(g + (0.0, 0.0, RIM_ABOVE_M), pitch=angles[0], yaw=angles[1], roll=angles[2], grip="open")
            steps += m.steps
            if self._width() < 0.9 * self.finger_stroke_m:
                steps += self._gripper("open").steps
            m = self._goto(g, speed=RIM_DOWN_SPEED)
            steps += m.steps
            if m.blocked and m.err_mm > 15.0:
                steps += self._goto(None, by=(0.0, 0.0, RIM_ABOVE_M)).steps
                why = (f"the fingers met something {m.err_mm / 10.0:.1f} cm short of the pinch point on the way "
                       "down (the rim's top, the contents or a neighbour): look, then try another inset")
                break
            grip = self._gripper("close")
            steps += grip.steps
            if grip.holding and grip.width_m >= RIM_EMPTY_M:
                width = grip.width_m
                why = ""
                break
            why = (f"the fingers closed on {100.0 * grip.width_m:.1f} cm, nothing between them, at "
                   f"{100.0 * i:.1f} cm in from the rim's edge")
            steps += self._gripper("open").steps
            steps += self._goto(None, by=(0.0, 0.0, RIM_ABOVE_M)).steps
        if width is None:
            if tries > 1 and not why.startswith("the fingers met"):
                why += f" (and {100.0 * insets[0]:.1f} cm in before)"
            return result(holding=False, pinch=pinch, tries=tries, steps=steps, why=why)
        self.slip_limit = RIM_SLIP_M
        if tries > 1:
            notes.append(f"the first pinch, {100.0 * insets[0]:.1f} cm in from the rim's edge, closed on nothing; "
                         f"this one is {100.0 * insets[tries - 1]:.1f} cm in")
        if width > 0.045:
            notes.append(f"the fingers closed on {100.0 * width:.1f} cm, more than a dish's wall: they may hold "
                         "what is in it or the wall and its contents - look")
        if up > 0:
            m = self._goto(None, by=(0.0, 0.0, up))
            steps += m.steps
            notes += m.notes
        holding = bool(self.holding)
        return result(holding=holding, width_m=round(self._width(), 4) if holding else None, pinch=pinch,
                      tries=tries, steps=steps, notes=notes,
                      why="" if holding else "it slipped out on the lift - look where it is")

    def _dish_point(self, target) -> np.ndarray:
        """A point on the dish (x, y, z): a measured centre at its top, or the point given."""
        if isinstance(target, (RR.Thing, RR.Mask, str)) or (
                isinstance(target, (tuple, list)) and target and isinstance(target[0], (str, RR.Mask))):
            thing = self._measure(target)
            c = thing.center
            return np.array([c[0], c[1], thing.top if thing.top is not None else c[2]], float)
        return point_of(target)

    def _cloud(self, p) -> np.ndarray:
        """Every camera's depth points (the robot left out) within RIM_CLOUD_M across of ``p``."""
        out = []
        for v in self.views().values():
            ys, xs = np.nonzero(v.valid() & ~v.robot)
            q = v.back_project(xs.astype(float), ys.astype(float), v.depth[ys, xs].astype(float))
            out.append(q[np.hypot(q[:, 0] - p[0], q[:, 1] - p[1]) < RIM_CLOUD_M])
        return np.concatenate(out) if out else np.zeros((0, 3))


def off_words(off) -> str:
    """'4 cm behind and 2 cm to the right, turned 3 degrees left' from (ahead, left, turn degrees)."""
    ahead, left, turn = (float(v) for v in off)
    parts = []
    if abs(ahead) >= 0.005:
        parts.append(f"{100 * abs(ahead):.0f} cm {'ahead' if ahead > 0 else 'behind'}")
    if abs(left) >= 0.005:
        parts.append(f"{100 * abs(left):.0f} cm to the {'left' if left > 0 else 'right'}")
    words = " and ".join(parts) or "at the place"
    if abs(turn) >= 0.5:
        words += f", turned {abs(turn):.0f} degrees {'left' if turn > 0 else 'right'}"
    return words


def xy_of(value, name: str) -> tuple[float, float]:
    try:
        v = np.asarray(value, float).reshape(-1)
    except (TypeError, ValueError):
        raise Refused(f"drive({name}=...): a floor point (x, y) in metres") from None
    if v.size not in (2, 3) or not np.all(np.isfinite(v)):
        raise Refused(f"drive({name}=...): a floor point (x, y) in metres")
    return float(v[0]), float(v[1])
