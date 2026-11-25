"""Result types of the benchmark robots (one arm, on a mobile base or fixed in place; a humanoid with two hands).

Like :mod:`hexashell.robot.results` they are plain data that cross from the host to the sandboxed
worker and print as a line or a few, most important first.  Perception results (Mask, Thing, Spot,
CameraInfo, Picture) are those of :mod:`hexashell.robot.results`.
"""

from __future__ import annotations

from dataclasses import dataclass, field


def _xyz(v) -> str:
    return "(" + ", ".join(f"{float(a):.3f}" for a in v) + ")"


def _deg(v) -> str:
    return str(round(float(v)))  # an int: never "-0"


def _left(steps_left) -> str:
    return "" if steps_left is None else f"  [{steps_left} steps left]"


@dataclass
class Move:
    reached: bool
    moved: bool
    target: tuple[float, float, float]
    target_angles: tuple[float, float, float]
    at: tuple[float, float, float]
    angles: tuple[float, float, float]
    err_mm: float = 0.0
    err_deg: float = 0.0
    steps: int = 0
    blocked: bool = False
    dry_run: bool = False
    why: str = ""
    notes: list[str] = field(default_factory=list)
    steps_left: int | None = None

    @property
    def ok(self) -> bool:
        return self.reached and not self.blocked

    def __repr__(self) -> str:
        p, y, r = self.angles
        pose = f"{_xyz(self.at)} pitch {_deg(p)} yaw {_deg(y)} roll {_deg(r)}"
        tp, ty, tr = self.target_angles
        goal = f"{_xyz(self.target)} pitch {_deg(tp)} yaw {_deg(ty)} roll {_deg(tr)}"
        if self.dry_run:
            lines = [f"Move (dry run): {'reachable' if self.reached else 'NOT reachable: ' + self.why}  target {goal}"]
        elif not self.moved:
            lines = [f"Move: NOT executed - {self.why or 'out of reach'}; target {goal}"]
        elif self.ok:
            lines = [f"Move: at {pose}  ±{self.err_mm:.0f} mm {self.err_deg:.0f}°  {self.steps} steps"
                     f"{_left(self.steps_left)}"]
        else:
            what = "BLOCKED" if self.blocked else "short of target"
            lines = [f"Move: {what} - at {pose}, {self.err_mm:.0f} mm / {self.err_deg:.0f}° from {goal}  "
                     f"{self.steps} steps{_left(self.steps_left)}"]
        lines += [f"  note: {n}" for n in self.notes]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass
class Push:
    """A push of a hinged part about its hinge (``robot.push_arc``)."""

    moved: bool
    turned: float
    angle: float
    stop: str
    at: tuple[float, float, float]
    angles: tuple[float, float, float]
    steps: int = 0
    dry_run: bool = False
    reachable: bool = True
    why: str = ""
    notes: list[str] = field(default_factory=list)
    steps_left: int | None = None

    @property
    def ok(self) -> bool:
        return self.moved and self.stop == "angle"

    def __repr__(self) -> str:
        if self.dry_run:
            head = f"Push (dry run): {'reachable' if self.reachable else 'NOT reachable: ' + self.why}"
        elif not self.moved:
            head = f"Push: NOT executed - {self.why or 'out of reach'}"
        else:
            p, y, r = self.angles
            head = (f"Push: the hand went {_deg(self.turned)} of {_deg(self.angle)} deg around the hinge "
                    f"({PUSH_STOPS.get(self.stop, self.stop)}) - at {_xyz(self.at)} pitch {_deg(p)} yaw {_deg(y)} "
                    f"roll {_deg(r)}  {self.steps} steps{_left(self.steps_left)}")
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class RimPick:
    """A round open dish picked by the near side of its rim (``robot.rim_pick``)."""

    holding: bool
    center: tuple[float, float]
    radius: float
    top: float
    on_z: float
    pinch: tuple[float, float, float]
    angles: tuple[float, float, float]
    at: tuple[float, float, float]
    width_m: float | None = None
    tries: int = 0
    steps: int = 0
    dry_run: bool = False
    executed: bool = True
    why: str = ""
    notes: list[str] = field(default_factory=list)
    steps_left: int | None = None

    @property
    def ok(self) -> bool:
        return self.holding and not self.dry_run

    def __repr__(self) -> str:
        p, y, r = self.angles
        dish = (f"rim centre ({self.center[0]:.3f}, {self.center[1]:.3f}) radius {self.radius:.3f}, near top "
                f"{self.top:.3f}, on z {self.on_z:.3f}")
        pinch = f"pinch at {_xyz(self.pinch)} pitch {_deg(p)} yaw {_deg(y)} roll {_deg(r)}"
        if self.dry_run:
            head = (f"RimPick (dry run): {'reachable' if self.executed else 'NOT reachable: ' + self.why} - {dish}; "
                    f"{pinch}")
        elif not self.executed:
            head = f"RimPick: NOT executed - {self.why}; {dish}; {pinch}"
        elif self.holding:
            head = (f"RimPick: holding it by the near rim (closed on ~{100 * (self.width_m or 0.0):.1f} cm) and "
                    f"lifted - at {_xyz(self.at)}; {dish}; {pinch}  {self.steps} steps{_left(self.steps_left)}")
        else:
            head = (f"RimPick: NOT holding - {self.why}; {dish}; last {pinch}  {self.steps} steps"
                    f"{_left(self.steps_left)}")
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


#: why a push stopped, as its line says it
PUSH_STOPS = {"angle": "all the way", "stalled": "the part stopped turning", "slid": "the hand slid off",
              "no touch": "touched nothing", "jammed": "the arm strained", "stretched": "the elbow came straight",
              "cycles": "the longest push", "over": "the episode ended"}


@dataclass
class Grip:
    opening: float
    width_m: float
    holding: bool | None
    goal: float
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        if self.goal <= 0.05 or self.holding:  # (a light hold sets the fingers a little under what they hold)
            if self.holding:
                state = f"closed on something ~{100 * self.width_m:.1f} cm wide (holding)"
            elif self.holding is None:
                state = f"closing, the fingers ~{100 * self.width_m:.1f} cm apart"
            else:
                state = "closed fully (empty - unless it is something thin: check a picture)"
        else:
            state = f"open {self.opening:.2f} (gap ~{100 * self.width_m:.1f} cm)"
        lines = [f"Grip: {state}  {self.steps} steps{_left(self.steps_left)}"]
        lines += [f"  note: {n}" for n in self.notes]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass
class Homing:
    home_mm: float
    home_deg: float
    steps: int = 0
    blocked: bool = False
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.home_mm <= 10.0 and self.home_deg <= 3.0

    def __repr__(self) -> str:
        where = "home" if self.ok else f"{self.home_mm:.0f} mm / {self.home_deg:.0f}° from home"
        if self.blocked:
            where = "BLOCKED, " + where
        lines = [f"Home: arm {where}  {self.steps} steps{_left(self.steps_left)}"]
        lines += [f"  note: {n}" for n in self.notes]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass
class Drive:
    """A base motion."""

    reached: bool
    blocked: bool
    at: tuple[float, float]
    heading: float
    target: tuple[float, float]
    target_heading: float
    err_mm: float = 0.0
    err_deg: float = 0.0
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)
    dry_run: bool = False
    #: a dry run: where the grasp point would end
    hand: tuple[float, float, float] | None = None

    def __repr__(self) -> str:
        at = f"({self.at[0]:.3f}, {self.at[1]:.3f}) heading {self.heading:.0f}"
        if self.dry_run:
            head = (f"Drive (dry run): the base would end at ({self.target[0]:.3f}, {self.target[1]:.3f}) heading "
                    f"{self.target_heading:.0f}" + ("" if self.hand is None else f", the hand at {_xyz(self.hand)}"))
        elif self.reached and not self.blocked:
            head = f"Drive: base at {at}  {self.steps} steps{_left(self.steps_left)}"
        else:
            what = "BLOCKED" if self.blocked else "short of target"
            head = (f"Drive: {what} - base at {at}, {self.err_mm:.0f} mm / {self.err_deg:.0f}° from "
                    f"({self.target[0]:.3f}, {self.target[1]:.3f}) heading {self.target_heading:.0f}  "
                    f"{self.steps} steps{_left(self.steps_left)}")
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class Waited:
    steps: int
    step_s: float
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        head = f"Waited {self.steps} steps ({self.step_s * self.steps:.2f} s){_left(self.steps_left)}"
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class Status:
    step: int | None
    step_limit: int | None
    at: tuple[float, float, float]
    angles: tuple[float, float, float]
    gripper: float
    width_m: float
    goal: float
    holding: bool | None
    home_mm: float
    home_deg: float
    #: the mobile base (none for an arm fixed in place)
    base: tuple[float, float] | None = None
    heading: float | None = None
    #: the gap between the fingers when they closed on what they hold
    grasp_m: float | None = None

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    @property
    def xyz(self):
        return self.at

    def __repr__(self) -> str:
        p, y, r = self.angles
        if self.goal <= 0.05 or self.holding:
            grip = f"closed, holding ({100 * self.width_m:.1f} cm" if self.holding else \
                "closed fully (empty, or something thin)" if self.holding is not None else \
                f"closing ({100 * self.width_m:.1f} cm)"
            if self.holding:
                slid = self.grasp_m is not None and self.grasp_m - self.width_m >= 0.002
                grip += f"; {100 * self.grasp_m:.1f} cm when it closed)" if slid else ")"
        else:
            grip = f"open {self.gripper:.2f}"
        home = "home" if self.home_mm <= 10 and self.home_deg <= 3 else \
            f"{self.home_mm / 10:.0f} cm / {self.home_deg:.0f}° from home"
        budget = "" if self.step is None else f"step {self.step} of {self.step_limit} ({self.steps_left} left)\n"
        text = (f"{budget}arm: grasp point {_xyz(self.at)} pitch {_deg(p)} yaw {_deg(y)} roll {_deg(r)}  gripper "
                f"{grip}  ({home})")
        if self.base is not None:
            text += f"\nbase: ({self.base[0]:.3f}, {self.base[1]:.3f}) heading {self.heading:.0f}"
        return text

    __str__ = __repr__


@dataclass
class TaskInfo:
    instruction: str
    step: int | None
    step_limit: int | None
    start_at: tuple[float, float, float]
    start_angles: tuple[float, float, float]
    task_name: str | None = None

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    def __repr__(self) -> str:
        lines = [f"Task: {self.instruction}"]
        if self.step_limit is not None:
            lines.append(f"Budget: {self.step_limit} steps; used {self.step}, {self.steps_left} left")
        p, y, r = self.start_angles
        lines.append(f"Start pose of the arm: grasp point {_xyz(self.start_at)} pitch {_deg(p)} yaw {_deg(y)} "
                     f"roll {_deg(r)}")
        return "\n".join(lines)

    __str__ = __repr__


# -- a humanoid (SIMPLE's Unitree G1): body, walking, two hands, a gaze camera ------------------------------
def _palm(v) -> str:
    return "none" if v is None else _xyz(v)


@dataclass
class BodyMove:
    """Pelvis height, torso pitch and yaw (``robot.body``)."""

    reached: bool
    height: float
    pitch: float
    yaw: float
    asked_height: float | None = None
    asked_pitch: float | None = None
    asked_yaw: float | None = None
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        now = f"pelvis {self.height:.3f} m high, torso pitch {_deg(self.pitch)} yaw {_deg(self.yaw)}"
        if self.reached:
            head = f"Body: {now}  {self.steps} steps{_left(self.steps_left)}"
        else:
            asked = ", ".join(f"{k} {v}" for k, v in (("height", self.asked_height), ("pitch", self.asked_pitch),
                                                      ("yaw", self.asked_yaw)) if v is not None)
            head = f"Body: short of target ({asked}) - {now}  {self.steps} steps{_left(self.steps_left)}"
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class Walk:
    """A walk of the pelvis over the ground (``robot.walk``)."""

    reached: bool
    blocked: bool
    at: tuple[float, float]
    heading: float
    target: tuple[float, float]
    target_heading: float
    err_mm: float = 0.0
    err_deg: float = 0.0
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        at = f"({self.at[0]:.3f}, {self.at[1]:.3f}) heading {self.heading:.0f}"
        if self.reached and not self.blocked:
            off = f", {self.err_mm:.0f} mm from the target" if self.err_mm >= 10 else ""
            head = f"Walk: pelvis at {at}{off}  {self.steps} steps{_left(self.steps_left)}"
        else:
            what = "BLOCKED" if self.blocked else "short of target"
            head = (f"Walk: {what} - pelvis at {at}, {self.err_mm:.0f} mm / {self.err_deg:.0f}° from "
                    f"({self.target[0]:.3f}, {self.target[1]:.3f}) heading {self.target_heading:.0f}  "
                    f"{self.steps} steps{_left(self.steps_left)}")
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class HandMove:
    """A palm moved through waypoints (``robot.hand``)."""

    side: str
    reached: bool
    at: tuple[float, float, float]
    target: tuple[float, float, float]
    err_mm: float = 0.0
    normal_err_deg: float | None = None
    finger_err_deg: float | None = None
    hold: str = "torso"
    steps: int = 0
    dry_run: bool = False
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.reached

    def __repr__(self) -> str:
        turn = [f"{w} {e:.0f}°" for w, e in (("normal", self.normal_err_deg), ("fingers", self.finger_err_deg))
                if e is not None]
        off = f"{self.err_mm:.0f} mm" + (f", {', '.join(turn)} off" if turn else "")
        if self.dry_run:
            head = (f"Hand {self.side} (dry run): {'reachable' if self.reached else 'NOT reachable'} - the palm "
                    f"would end at {_xyz(self.at)}, {off} from {_xyz(self.target)}")
        elif self.reached:
            head = (f"Hand {self.side}: palm at {_xyz(self.at)}  ±{off}  hold {self.hold}  {self.steps} steps"
                    f"{_left(self.steps_left)}")
        else:
            head = (f"Hand {self.side}: short of target - palm at {_xyz(self.at)}, {off} from {_xyz(self.target)}  "
                    f"{self.steps} steps{_left(self.steps_left)}")
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class Fingers:
    """One hand's fingers opened or closed (``robot.fingers``)."""

    side: str
    closed: bool
    touching: list[str]
    holding: bool | None
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)
    #: opened wide: the thumb swung back from the fingers
    wide: bool = False

    def __repr__(self) -> str:
        touch = ", ".join(self.touching) if self.touching else "nothing"
        if self.closed:
            what = "closed, holding" if self.holding else "closed, holding nothing"
        else:
            what = "open wide (thumb back)" if self.wide else "open"
        head = f"Fingers {self.side}: {what} (touching: {touch})  {self.steps} steps{_left(self.steps_left)}"
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class Gaze:
    """The gaze camera turned (``robot.gaze``)."""

    yaw: float
    pitch: float
    asked_yaw: float
    asked_pitch: float
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        head = f"Gaze: yaw {self.yaw:.0f} pitch {self.pitch:.0f}"
        if abs(self.yaw - self.asked_yaw) > 0.5 or abs(self.pitch - self.asked_pitch) > 0.5:
            head += f" (asked yaw {self.asked_yaw:.0f} pitch {self.asked_pitch:.0f}: at its limit)"
        head += f"  {self.steps} steps{_left(self.steps_left)}"
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class PalmState:
    point: tuple[float, float, float]
    normal: tuple[float, float, float]
    finger: tuple[float, float, float]
    fingers: str
    touching: list[str]
    holding: bool | None
    hold: str

    def __repr__(self) -> str:
        grip = self.fingers + (", holding" if self.holding else ", holding nothing" if self.fingers == "closed" else "")
        touch = ", ".join(self.touching) if self.touching else "nothing"
        return (f"palm {_xyz(self.point)} facing {_xyz(self.normal)} fingers along {_xyz(self.finger)}; {grip} "
                f"(touching: {touch}); hold {self.hold}")

    __str__ = __repr__


@dataclass
class HumanoidStatus:
    step: int | None
    step_limit: int | None
    official_steps: int | None
    pelvis: tuple[float, float, float]
    heading: float
    pelvis_height: float
    torso_pitch: float
    torso_yaw: float
    left: PalmState
    right: PalmState
    gaze: tuple[float, float]

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    def __repr__(self) -> str:
        lines = []
        if self.step is not None:
            lines.append(f"step {self.step} of {self.step_limit} ({self.steps_left} left; the official limit is "
                         f"{self.official_steps})")
        lines += [f"pelvis: ({self.pelvis[0]:.3f}, {self.pelvis[1]:.3f}) heading {self.heading:.0f}, "
                  f"{self.pelvis_height:.3f} m high; torso pitch {_deg(self.torso_pitch)} yaw {_deg(self.torso_yaw)}",
                  f"left hand: {self.left!r}", f"right hand: {self.right!r}",
                  f"gaze: yaw {_deg(self.gaze[0])} pitch {_deg(self.gaze[1])}"]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass
class HumanoidTask:
    instruction: str
    step: int | None
    step_limit: int | None
    official_steps: int | None
    task_name: str | None = None

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    def __repr__(self) -> str:
        lines = [f"Task: {self.instruction}"]
        if self.step_limit is not None:
            lines.append(f"Budget: {self.step_limit} steps; used {self.step}, {self.steps_left} left (the official "
                         f"limit is {self.official_steps} steps)")
        lines.append("Start frame: origin on the ground under the pelvis at the start, x its heading then, y to its "
                     "left, z up")
        return "\n".join(lines)

    __str__ = __repr__


# -- arms with dexterous hands (DexJoCo-X: Franka Panda arms, Wuji hands) ----------------------------------------


def _fingers_of(curl) -> str:
    return " ".join(f"{f} {float(v):.2f}" for f, v in (curl or {}).items())


@dataclass
class DexterousMove:
    """A hand moved (``robot.hand``, ``robot.home``) or turned about a line (``robot.arc``)."""

    side: str
    point: str
    reached: bool
    at: tuple[float, float, float]
    target: tuple[float, float, float]
    err_mm: float = 0.0
    err_deg: float = 0.0
    #: why the motion stopped before its end (something held the hand up), or None
    blocked: str | None = None
    steps: int = 0
    dry_run: bool = False
    #: arcs: degrees turned and asked for
    turned: float | None = None
    asked_turn: float | None = None
    #: dry runs: the arm's joints (1-7, shoulder to wrist) that would be at their limits
    at_limit: list[int] = field(default_factory=list)
    #: dry runs: the pose at the edge of what the arm reaches (a real motion may stop short of it)
    edge: bool = False
    #: dry runs: where along the way (0-1) the arm would first fall behind the point's path, and how far
    lost_at: float | None = None
    lost_mm: float = 0.0
    lost_deg: float = 0.0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.reached and not self.blocked

    def __repr__(self) -> str:
        what = f"Arc {self.side}" if self.turned is not None else f"Hand {self.side}"
        if self.dry_run and self.lost_at is not None:
            head = (f"{what} (dry run): NOT reachable - about {100 * self.lost_at:.0f}% of the way to "
                    f"{_xyz(self.target)} the arm would fall {self.lost_mm:.0f} mm / {self.lost_deg:.0f}° behind the "
                    f"{self.point}'s path")
        elif self.dry_run:
            verdict = "reachable" if self.reached else "at the edge of reach" if self.edge else "NOT reachable"
            head = (f"{what} (dry run): {verdict} - the {self.point} would end {self.err_mm:.0f} mm / "
                    f"{self.err_deg:.0f}° from {_xyz(self.target)}")
        elif self.turned is not None:
            state = "turned" if self.ok else "BLOCKED after" if self.blocked else "short:"
            head = (f"{what}: {state} {self.turned:.0f}° of {self.asked_turn:.0f}°, {self.point} at {_xyz(self.at)}  "
                    f"{self.steps} steps{_left(self.steps_left)}")
        elif self.ok:
            head = (f"{what}: {self.point} at {_xyz(self.at)}  ±{self.err_mm:.0f} mm {self.err_deg:.0f}°  "
                    f"{self.steps} steps{_left(self.steps_left)}")
        else:
            state = "BLOCKED" if self.blocked else "short of target"
            head = (f"{what}: {state} - {self.point} at {_xyz(self.at)}, {self.err_mm:.0f} mm / {self.err_deg:.0f}° "
                    f"from {_xyz(self.target)}  {self.steps} steps{_left(self.steps_left)}")
        lines = [head]
        if self.blocked:
            lines.append(f"  blocked: {self.blocked}")
        return "\n".join(lines + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class DexterousHome:
    """Hands back at their start poses (``robot.home``)."""

    #: per hand: None (home) or why it stopped on the way
    sides: dict[str, str | None]
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(v is None for v in self.sides.values())

    def __repr__(self) -> str:
        parts = [f"{s} {'home' if why is None else 'BLOCKED (' + why + ')'}" for s, why in self.sides.items()]
        head = f"Home: {', '.join(parts)}  {self.steps} steps{_left(self.steps_left)}"
        return "\n".join([head] + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class DexterousFingers:
    """One hand's fingers set (``robot.fingers``): a named shape and/or each finger's curl."""

    side: str
    shape: str
    #: each finger's curl, measured and asked for (0 straight .. 1 curled in)
    curl: dict[str, float]
    asked: dict[str, float]
    #: the fingers pressing on something (something outside the hand pushes on them)
    pressing: list[str]
    #: the thumb and one more finger pressing: the hand holds something
    holding: bool
    steps: int = 0
    steps_left: int | None = None
    notes: list[str] = field(default_factory=list)

    def __repr__(self) -> str:
        on = ", ".join(self.pressing) if self.pressing else "none"
        grip = "holding" if self.holding else "holding nothing"
        head = (f"Fingers {self.side}: {self.shape}, {grip} (pressing on something: {on})  {self.steps} steps"
                f"{_left(self.steps_left)}")
        lines = [head, f"  curl: {_fingers_of(self.curl)}"]
        return "\n".join(lines + [f"  note: {n}" for n in self.notes])

    __str__ = __repr__


@dataclass
class DexterousHand:
    """One hand now: its palm, grasp point and fingertips (scene frame), its fingers."""

    side: str
    palm: tuple[float, float, float]
    normal: tuple[float, float, float]
    finger: tuple[float, float, float]
    grasp: tuple[float, float, float]
    tips: dict[str, tuple[float, float, float]]
    shape: str
    curl: dict[str, float]
    pressing: list[str]
    holding: bool
    #: the wrist camera: where it is and its line of sight
    camera: tuple[float, float, float] | None = None
    view: tuple[float, float, float] | None = None

    def __repr__(self) -> str:
        on = ", ".join(self.pressing) if self.pressing else "none"
        grip = "holding" if self.holding else "holding nothing"
        tips = "; ".join(f"{f} {_xyz(p)}" for f, p in self.tips.items())
        text = (f"palm {_xyz(self.palm)} facing {_xyz(self.normal)} fingers along {_xyz(self.finger)}; grasp point "
                f"{_xyz(self.grasp)}\n    fingers: {self.shape}, {grip} (pressing on something: {on}); curl "
                f"{_fingers_of(self.curl)}\n    tips: {tips}")
        if self.camera is not None and self.view is not None:
            text += f"\n    wrist camera at {_xyz(self.camera)} looking along {_xyz(self.view)}"
        return text

    __str__ = __repr__


@dataclass
class DexterousStatus:
    step: int | None
    step_limit: int | None
    hands: dict[str, DexterousHand]

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    @property
    def right(self) -> DexterousHand | None:
        return self.hands.get("right")

    @property
    def left(self) -> DexterousHand | None:
        return self.hands.get("left")

    def __repr__(self) -> str:
        lines = []
        if self.step is not None:
            lines.append(f"step {self.step} of {self.step_limit} ({self.steps_left} left)")
        lines += [f"{side} hand: {hand!r}" for side, hand in self.hands.items()]
        return "\n".join(lines)

    __str__ = __repr__


@dataclass
class DexterousTask:
    instruction: str
    step: int | None
    step_limit: int | None
    arms: tuple[str, ...]
    task_name: str | None = None

    @property
    def steps_left(self) -> int | None:
        if self.step is None or self.step_limit is None:
            return None
        return max(0, self.step_limit - self.step)

    def __repr__(self) -> str:
        lines = [f"Task: {self.instruction}"]
        if self.step_limit is not None:
            lines.append(f"Budget: {self.step_limit} steps; used {self.step}, {self.steps_left} left")
        lines.append(f"Arms: {', '.join(self.arms)}. Scene frame: metres, z up, the robot faces +x, y to its left")
        return "\n".join(lines)

    __str__ = __repr__
