"""Result types of the benchmark robots (one arm, on a mobile base or fixed in place).

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
