"""A RoboColiseum Season II episode: one episode of a Genie Sim 3.6 benchmark run, played through :mod:`.link`.

The benchmark (``geniesim benchmark run`` in its container, :mod:`.sim`) asks our WebSocket server for actions
(:class:`.link.Server`); every episode it starts becomes an :class:`.link.EpisodeLink`, and
:class:`ColiseumSession` is what :class:`~hexashell.runtime.episode.Episode` needs of it: the robot calls
(:class:`.host.ColiseumHost`), the task card, the start pictures and the verdict, which the benchmark writes to its
evaluate_ret file when the episode ends.

What the model is told: the instruction and the benchmark's stage texts (``segments``) as they come, the
task's name, the robot and its grippers, the start pose of each hand it moves; how the benchmark scores (each motion
is one reply, each stage allows so much simulated time and so many replies) without its numbers, and how many
motions it has made so far.

On the native runtimes (dexmanip on libuipc, Newton, MuJoCo) the facts of the runtime instead of the stages'
allowances: the time of one step, the episode's one span of steps for the whole task and the steps left of it
(the cell's status line, robot.task, every motion's result), whether the scene stands still while the model thinks,
a torso that does not turn, and the ``<part>.hands_native.md`` prompt parts (no depth, no waist, no stages in order);
never the dispatch budget.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np

from ...runtime.session import Attach, Session
from . import body as body_model
from . import dexterous as D
from . import link as L
from . import obs as O
from . import protocol as P
from .host import (EARLY_END_WORDS, NATIVE_STEPS_WORDS, SPAN_WORDS, STEP_WORDS, ColiseumHost, check_cam, describe_native_end,
                   describe_verdict, stage_texts)
from .link import NATIVE
from .motion import BODY_LEAN_DEG, BODY_ROBOTS, HEAD_PITCH_DEG, HEAD_YAW_DEG

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "robocoliseum"
#: the longest the end of an episode may take once the model is done: the benchmark plays the closing hold
#: chunk, then judges the episode and writes its verdict
VERDICT_CAP_S = 900.0

CAMERA_WORDS = {"head": "on the head, looking down at the work area in front of the robot",
                "left_wrist": "on the left gripper, looking along its fingers",
                "right_wrist": "on the right gripper, looking along its fingers"}
#: what the card says of each robot's grippers: their kind, how far the fingertips reach past the grasp point
HAND_WORDS = {
    "omnipicker": ("omnipicker grippers (two parallel pads)",
                   "the fingertips reach about 1 cm past the grasp point, open or closed"),
    "ctek90d": ("ctek90d grippers (two pads on swinging links)",
                "the fingertips reach 2.2 cm past the grasp point open and up to 4 cm closed: the pads swing "
                "forward as they close, less when they stop on something wide (closed on 8 cm the fingertips "
                "reach about 2.7 cm, on 6 cm 3.3 cm, on 4 cm 3.7 cm). Pointing down, they hold only what lies "
                "within that reach: close with the grasp point at most about 2 cm above the top of a low "
                "object (a phone, a card), lower for a wide one; the fingertips may then meet the surface under "
                "it, and the grasp still holds"),
    "arx_gripper": ("parallel grippers (two fingers that slide together)",
                    "the fingertips reach about 2.5 cm past the grasp point, open or closed"),
    "agilex_gripper": ("parallel grippers (two fingers that slide together)",
                       "the fingertips reach about 2.5 cm past the grasp point, open or closed"),
    # the hands' own lines come from dexterous.card_lines
    "wuji": ("Wuji five-finger hands (20 joints each, set by robot.hand)", ""),
}
#: what a wrist camera sees of its own gripper (the recorded first pictures of each robot)
WRIST_WORDS = {
    "omnipicker": ("each sits just behind its gripper and looks along the fingers: the gripper fills the lower "
                   "half of the picture, a finger on each side of the middle, and what is between the fingers "
                   "shows just above the gripper, in the middle"),
    "arx_gripper": ("each sits on its wrist and looks along the fingers: the two fingers show as dark shapes in "
                    "the lower corners of the picture, one on each side, and what is between them shows in the "
                    "lower middle"),
}
WRIST_WORDS["wuji"] = ("each sits at the heel of its palm, on the little finger's side, and looks along the fingers: "
                       "the hand's own fingers fill the right part of the right camera's picture and the left part "
                       "of the left's, the thumb at the top, and what lies ahead of the fingertips shows in the "
                       "middle; the pictures are undistorted from fisheye lenses and wide (about 114 degrees "
                       "across)")
WRIST_WORDS["ctek90d"] = WRIST_WORDS["omnipicker"]
WRIST_WORDS["agilex_gripper"] = WRIST_WORDS["arx_gripper"]


class RobotWords(NamedTuple):
    #: the card's first line (``{kind}``: the grippers, from HAND_WORDS)
    what: str
    #: where the head camera sits
    head: str = CAMERA_WORDS["head"]
    #: the height of the robot's base frame (the world frame of the robot calls) above the floor, m, where the
    #: robot itself fixes it (its model's base_link above its footprint)
    base_z: float = 0.0
    #: the base frame sits at the arms' mounts, at a height the scene sets: the card says so with no number
    base_on_mount: bool = False


_FIXED_HEAD = "fixed to the robot's body, looking at the work area in front of the robot"
#: the robots the bench plays (protocol.ROBOTS keys)
ROBOT_WORDS = {
    "g2_omnipicker": RobotWords("AgiBot G2, two seven-joint arms on a torso that turns at the waist, with {kind}"),
    "g2_90d": RobotWords("AgiBot G2, two seven-joint arms on a torso that turns at the waist, with {kind}"),
    "g2_wuji": RobotWords("AgiBot G2, two seven-joint arms on a torso that turns at the waist, with {kind}"),
    "g1": RobotWords("AgiBot G1, two seven-joint arms on a torso that does not turn, with {kind}"),
    # the base frame of these two is not on the floor: ARX's sits at its arms' mounts, whose height a scene
    # sets (no number on the card: the heights are measured in each episode); AgileX's base_link is 0.15 m
    # above its footprint (its model)
    "arx": RobotWords("ARX, two six-joint arms on a fixed base, with {kind}", _FIXED_HEAD, base_on_mount=True),
    "agilex": RobotWords("AgileX, two six-joint arms on a fixed base, with {kind}", _FIXED_HEAD, 0.15),
}
#: the openings the card gives the gap of
CARD_OPENINGS = (1.0, 0.8, 0.6, 0.4, 0.2)


def _xyz(p) -> str:
    return f"({p[0]:.2f}, {p[1]:+.2f}, {p[2]:.2f})"


#: the leans (degrees) at which the card gives the shoulders' heights
CARD_LEANS = (20.0, 45.0, 70.0)


def body_lines(frame, driver) -> list[str]:
    """The card's lines on the torso's posture and the head (robot.body, robot.head), where they move."""
    w = frame.waist
    if not driver.mobile or driver.robot not in BODY_ROBOTS or w is None or len(w) < 5:
        return []
    now = body_model.posture(np.asarray(w, float)[:5])
    spans = []
    for lean in CARD_LEANS:
        h = body_model.heights(math.radians(lean), np.zeros(5))
        if h is not None:
            spans.append(f"z {h[0]:.2f}-{h[1]:.2f} at a {lean:.0f}-degree lean")
    lines = [f"- Torso (robot.body): it leans {BODY_LEAN_DEG[0]:.0f} to {BODY_LEAN_DEG[1]:.0f} degrees forward and "
             f"lowers or raises the shoulders ({', '.join(spans)}), carrying both arms and the head; at the start it "
             f"leans {math.degrees(now.lean):.0f} degrees with the shoulders at z {now.shoulder_z:.2f}."]
    h = frame.head
    if h is not None and len(h) >= 3:
        lines.append(f"- Head (robot.head): it turns up to {HEAD_YAW_DEG:.0f} degrees either way and tips "
                     f"{HEAD_PITCH_DEG[0]:.0f} to {HEAD_PITCH_DEG[1]:.0f} degrees down on the torso; at the start it "
                     f"is turned {math.degrees(float(h[body_model.HEAD_TURN])):.0f} and tipped "
                     f"{math.degrees(float(h[body_model.HEAD_TIP])):.0f} degrees down.")
    return lines


def native_time_line(runtime: str) -> str:
    """The card's line on time on a native runtime: a step, the episode's one span, the pause (or not) while the
    model thinks."""
    step = f"one step (one command row) is {STEP_WORDS[runtime]} of simulated time in this task's simulator"
    if runtime == "mujoco":
        step += ", longer when a joint would otherwise move faster than its motor allows"
        left = "how many steps of 1/15 s are left"
    else:
        left = "how many are left"
    pause = ("The simulation stands still while you think." if P.PAUSES[runtime] else
             "The simulation does not stand still while you think: it runs on, the robot holding its pose.")
    return (f"- Time: {step}. The episode is one span of motion for the whole task, {SPAN_WORDS[runtime]}: it "
            f"ends when the span is used up, wherever the task stands; every motion's result and the line after "
            f"each cell say {left}. {pause}")


def robot_lines(frame, driver) -> list[str]:
    """The card's facts of this robot: the grippers, the wrist cameras, the shoulders and reach, the head
    camera, and the floor when the base frame is not on it."""
    hand = driver.hand_model
    words = ROBOT_WORDS[driver.robot]
    kind, tips = HAND_WORDS[hand.name]
    wrist = WRIST_WORDS[hand.name]
    runtime = getattr(driver, "runtime", "isaac")
    native = runtime in NATIVE
    what = words.what
    if native:
        # the robot's body is fixed there (motion.RobotDriver.fixed_body); MuJoCo's wrist lenses are pinholes
        what = what.replace("a torso that turns at the waist", "a torso that does not turn in this task")
        lens = O.NATIVE_LENSES.get(runtime, {}).get("left_wrist")
        if lens is not None and lens.model != "fisheye":
            wrist = wrist.replace(" from fisheye lenses", "")
    if len(driver.arms) == 1:
        other = next(a for a in ("left", "right") if a not in driver.arms)
        wrist += f"; the {other} arm does not move, so its camera keeps one view"
    sizes = sorted({(img.shape[1], img.shape[0]) for cam, img in frame.images.items() if cam != "head"})
    if sizes:
        wrist += "; their pictures are " + " and ".join(f"{w} x {h}" for w, h in sizes)
    lines = [f"Robot: {what.format(kind=kind)}."]
    if hand.dexterous:
        lines += D.card_lines()
    else:
        gaps = ", ".join(f"{g:g} -> {100 * hand.gap_m(hand.command(g)):.1f} cm" for g in CARD_OPENINGS)
        lines.append(f"- Gripper: the gap between the pads at an opening g: {gaps}; nothing wider than about "
                     f"{100 * hand.open_gap - 0.3:.1f} cm fits between them. The grasp point (between the pads) is "
                     f"{100 * hand.grasp_reach:.1f} cm beyond the wrist's end link; {tips}.")
    lines.append(f"- Wrist cameras: {wrist}.")
    shoulders = {arm: (frame.arm_base @ np.r_[driver.models[arm].shoulder, 1.0])[:3] for arm in driver.arms}
    reach = driver.models[driver.arms[0]].reach
    facing = " (the waist facing forward)" if driver.turns_waist else ""
    if len(shoulders) > 1:
        lines.append(f"- Shoulders{facing}: {', '.join(f'{a} {_xyz(p)}' for a, p in shoulders.items())}; each "
                     f"grasp point reaches up to about {reach:.2f} m from its shoulder, less in many directions.")
    else:
        (arm, p), = shoulders.items()
        lines.append(f"- Shoulder of the {arm} arm: {_xyz(p)}; its grasp point reaches up to about {reach:.2f} m "
                     f"from it, less in many directions.")
    head = frame.calib.get("head")
    if head is not None:
        pos, axis = head.pose[:3, 3], head.pose[:3, 2]
        down = float(np.degrees(np.arcsin(np.clip(-axis[2], -1.0, 1.0))))
        tilt = f"about {down:.0f} degrees down" if down >= 0.5 else f"about {-down:.0f} degrees up"
        lines.append(f"- Head camera at about {_xyz(pos)}, looking forward and {tilt}; its pictures are "
                     f"{head.width} x {head.height}.")
    if driver.mobile:
        lines.append("- Base: wheels that drive it any way on the floor and turn it on the spot (robot.drive). The "
                     "world of the robot calls is the base's own and moves with it; nothing reads where the base "
                     "got to - judge it from the pictures.")
        lines += body_lines(frame, driver)
    if native:
        lines.append(native_time_line(runtime))
        if runtime in EARLY_END_WORDS:
            lines.append(f"- Besides the span, the episode ends at once on: {EARLY_END_WORDS[runtime]}.")
    else:
        lines.append(f"- Time: one step (one command row) is about {1000 * driver.row_s:.0f} ms of simulated time "
                     f"in this scene (its physics runs at {driver.physics_hz} Hz).")
    if words.base_z:
        lines.append(f"- Heights: z 0 is this robot's base, {words.base_z:.2f} m above the floor (the floor is at "
                     f"z {-words.base_z:.2f}); measure the table's height, do not assume it.")
    elif words.base_on_mount:
        lines.append("- Heights: z 0 is this robot's base at its arms' mounts, not the floor, and the scene sets "
                     "how high that is: a table or an object may read below z 0. Measure the table's height, do "
                     "not assume it.")
    return lines


def notes_folder(task: P.Task) -> Path:
    """Where this task's skills are read from: ``skills/robocoliseum/`` itself, or the subfolder whose ``tasks.txt``
    names the task (one task name a line, ``#`` starts a comment).  Such a task reads all its skills (common, robot,
    board, task, config) from that folder only: one missing there is none, even if ``skills/robocoliseum/`` has
    it."""
    for folder in sorted(p for p in SKILLS.iterdir() if p.is_dir()):
        listed = folder / "tasks.txt"
        if listed.exists():
            names = {line.split("#", 1)[0].strip() for line in listed.read_text(encoding="utf-8").splitlines()}
            if task.task in names:
                return folder
    return SKILLS


def skill(name: str, folder: Path | None = None) -> str | None:
    """``<folder>/<name>.md`` (a robot, a board, a task or ``common``; ``folder`` from :func:`notes_folder`, by
    default ``skills/robocoliseum/``), or None."""
    path = (SKILLS if folder is None else folder) / f"{name}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


def note_names(task: P.Task) -> list[tuple[str, str]]:
    """The notes a card may get, in order, with their titles: ``common``, the robot's, the board's, the task's
    and the task's on this robot (a cross-embodiment task is played by several robots)."""
    return [("common", "Notes"), (f"robot_{task.robot}", "Notes for this robot"),
            (f"board_{task.board}", "Notes for tasks of this kind"), (task.task, "Notes for this task"),
            (task.config, "Notes for this task on this robot")]


def _gripper_words(opening: float, holding: bool | None) -> str:
    if holding:
        return "closed on something it holds at the start"
    return f"open {opening:.2f}" if opening >= 0.05 else "closed"


class ColiseumSession(Session):
    bench = "robocoliseum"
    proxy = "hexashell.benches.robocoliseum.proxy"

    def __init__(self, link, info: dict[str, Any]):
        t = P.task(info["config"])
        instance = int(info["instance"])
        super().__init__({**info, "task_name": t.task, "board": t.board, "eval_seed": info.get("seed")})
        self.link = link
        self.t = t
        self.config = t.config
        self.task = t.task
        self.instance = instance
        #: the split part (run.py checked it; our own wok_flip and Rubik's cube cases are not numbered by it)
        self.part = info.get("part") or P.part_of(instance)
        #: the k-th episode of this benchmark run (0 = the first)
        self.k = int(info.get("episode", link.index))
        self.rec = None
        self.host: ColiseumHost | None = None
        self.first: dict[str, np.ndarray] = {}
        self._noted = 0

    # -- names -----------------------------------------------------------------------------------
    def prompt_variant(self) -> str | None:
        """Five-finger hands: the ``<part>.hands.md`` parts in place of the gripper's (the shared parts speak of
        two-finger grippers, their grasp advice and their step costs); on a native runtime (dexmanip on libuipc,
        Newton, MuJoCo) the ``<part>.hands_native.md`` parts: no depth, no waist, one span of steps for the
        episode, no stages in order.  The shared prompt takes a variant's part where there is one and the plain part
        otherwise, so every part has a ``hands_native`` one (tests/test_robocoliseum_native.py)."""
        if self.host is None or not self.host.driver.dexterous:
            return None
        return "hands_native" if getattr(self.host, "native", False) else "hands"

    def record_name(self) -> str | None:
        own = "".join(f"_{p}" for p in (self.info.get("archive") and str(self.info["archive"]).removesuffix(".zip"),
                                         self.info.get("seed") is not None and f"seed{self.info['seed']}") if p)
        return f"{self.config}{own}_i{self.instance:02d}_e{self.k}"

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.info.get("seed"), "layout": self.instance, "config": self.config,
                "episode": self.k}

    # -- lifetime ------------------------------------------------------------------------------
    def open(self, settings, rec, attach: Attach) -> None:
        self.rec = rec
        first = self.link.first
        if first.robot not in ROBOT_WORDS:
            raise RuntimeError(f"robot {first.robot} has no robot calls yet (only {', '.join(ROBOT_WORDS)})")
        if not settings.depth:
            self.errors.append("depth off is not supported on RoboColiseum: perception reads the depth pictures")
        scene = self._scene(settings)
        self.host = ColiseumHost(self.link, task_name=self.task, scene=scene, log=rec.event, attach=attach, rec=rec,
                                 seed=self.instance * 1000 + self.k, physics_hz=self.t.physics_hz,
                                 mobile=self.t.mobile)
        self.perception = scene is not None
        self.first = {cam: img for cam, img in first.images.items() if cam in O.CAMERAS}
        rec.event({"kind": "episode_open", "config": self.config, "instance": self.instance, "part": self.part,
                   "episode": self.k, "robot": first.robot, "instruction": first.prompt, "segments": first.segments,
                   "first": O.summary(first), "start": self.host.driver.start_info()})

    def _scene(self, settings):
        if not settings.sam3_url:
            return None
        try:
            from ...vision.rgbd import DepthScene, closing_yaw_robocasa

            # closing_yaw_robocasa: a yaw in (-90, 90], at yaw 0 the fingers close along world y - ours
            return DepthScene(settings.sam3_url, cams=list(O.CAMERAS), check_cam=check_cam, log=self.rec.event,
                              closing_yaw=closing_yaw_robocasa)
        except Exception as exc:  # noqa: BLE001 - run without perception, and say so
            self.errors.append(f"perception unavailable: {exc}")
            self.rec.event({"kind": "perception_unavailable", "error": str(exc)})
            return None

    def conclude(self, timeout_s: float) -> dict | None:
        """End the episode (a held chunk that tells the benchmark the policy is done), then its verdict
        (``timeout_s`` is the shared runner's minute; the cap is VERDICT_CAP_S)."""
        self.link.conclude()
        # a libuipc episode's scorer may still be catching up (link.VERDICT_RUNTIME_S)
        v = self.link.wait_verdict(max(VERDICT_CAP_S, L.verdict_wait_s(self.link.runtime) + 30.0))
        if v is None:
            return None
        v = dict(v)
        v.setdefault("step", v.get("dispatches"))
        return v

    def close(self) -> None:
        if self.host is not None and self.host.scene is not None:
            try:
                self.host.scene.close()
            except Exception:  # noqa: BLE001
                pass

    # -- during the episode ----------------------------------------------------------------------
    @property
    def over(self) -> bool:
        return self.link.over

    @property
    def verdict(self) -> dict | None:
        return self.link.verdict

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        return self.host.dispatch(name, args, kwargs)

    @property
    def step(self) -> int | None:
        # Isaac: the benchmark's allowances are per stage and not told; native: the steps used of the episode's span
        span = self.host.span() if self.host is not None else None
        return None if span is None else span[0]

    @property
    def step_limit(self) -> int | None:
        span = self.host.span() if self.host is not None else None
        return None if span is None else span[1]

    @property
    def instruction(self) -> str | None:
        return self.link.prompt

    def frames(self) -> dict[str, np.ndarray]:
        try:
            return {cam: img for cam, img in self.link.latest().images.items() if cam in O.CAMERAS}
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            if self.rec is not None:
                self.rec.event({"kind": "frames_failed", "error": str(exc)})
            return {}

    def over_text(self) -> str:
        runtime = getattr(self.link, "runtime", "isaac")
        if self.host is not None:
            reason = self.host.over_reason()
        elif runtime in NATIVE:
            reason = describe_native_end(self.link.verdict, runtime)
        else:
            reason = describe_verdict(self.link.verdict)
        return f"The episode is over: {reason}."

    def start_pictures(self) -> list[tuple[str, np.ndarray]]:
        return [(cam, self.first[cam]) for cam in O.CAMERAS if cam in self.first]

    def card(self, settings) -> tuple[str, dict]:
        first = self.link.first
        driver = self.host.driver
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {self.instruction!r}",
                 f"Task: {self.task}"]
        lines += robot_lines(first, driver)
        stages = stage_texts(first.segments)
        if stages and stages != [str(self.instruction).strip()]:
            # e.g. plug_charging_cable_into_phone lists six steps and judges three stages; cover_cup_lid's step 1
            # is judged by no stage
            if self.host.native:  # the native prompts: a task judged in parts, in no order
                lines.append(f"The benchmark's own description of the task, step by step ({NATIVE_STEPS_WORDS}; "
                             "the instruction says what the task is):")
            else:
                lines.append("The benchmark's own description of the task, step by step, in order (a description, "
                             "not the stages it judges: several steps can make one stage, and a step may not be "
                             "judged at all; the instruction says what the task is):")
            lines += [f"{i}. {s}" for i, s in enumerate(stages, 1)]
        lines.append("Start pose of each hand (robot.home() returns to it):")
        for arm, (xyz, angles) in driver.start_info().items():
            p, y, r = (round(v) + 0 for v in angles)  # + 0: no "-0"
            grip = (f"hand {driver.hands[arm].shape}" if driver.dexterous
                    else f"gripper {_gripper_words(driver.start_grippers[arm], driver.holding(arm))}")
            lines.append(f"- {arm}: grasp point ({', '.join(f'{v:.3f}' for v in xyz)}), pitch {p} yaw {y} roll {r}, "
                         f"{grip}")
        shown = [c for c in O.CAMERAS if c in self.first]
        cam_words = {**CAMERA_WORDS, "head": ROBOT_WORDS[driver.robot].head}
        if driver.dexterous:
            cam_words.update({f"{a}_wrist": f"on the {a} hand, looking along its fingers" for a in ("left", "right")})
        words = "; ".join(f"{c} ({cam_words[c]})" for c in shown)
        lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "config": self.config, "instance": self.instance, "part": self.part,
                "skill": False, "notes": [], "stages": len(stages)}
        if settings.skills:
            folder = notes_folder(self.t)
            used["notes_folder"] = "." if folder == SKILLS else folder.name
            for name, title in note_names(self.t):
                text = skill(name, folder)
                if text:
                    lines += ["", f"## {title}", "", text]
                    used["notes"].append(name)
            used["skill"] = bool(used["notes"])
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        first = self.link.first
        v = verdict or {}
        return {"task": self.task, "config": self.config, "board": self.t.board, "robot": self.t.robot,
                "instance": self.instance, "part": self.part, "seed": self.info.get("seed"), "episode": self.k,
                "instruction": self.instruction, "segments": first.segments, "result_index": self.link.result_index,
                "score": v.get("score"), "progress": v.get("progress"), "valid": v.get("valid"),
                "dispatches": v.get("dispatches")}

    def report(self, finish: dict | None) -> dict[str, Any]:
        host = self.host
        out: dict[str, Any] = {"link": dict(self.link.stats)}
        if host is not None:
            out.update({"rows_sent": host.driver.rows, "motions": host.motions, "catches": dict(host.driver.catches)})
        return out

    def timing_note(self) -> str:
        names = ["goto", "gripper", "home", "wait", "waist", "drive"]
        if self.host is not None and self.host.driver.dexterous:
            names.insert(2, "hand")
        if self.host is not None and not self.host.driver.turns_waist:
            names.remove("waist")
        if self.host is None or not self.host.driver.mobile:
            names.remove("drive")
        elif self.host.driver.robot in BODY_ROBOTS:
            names += ["body", "head"]
        calls = ", ".join(names)
        runtime = getattr(self.link, "runtime", "isaac")
        if self.host is not None and runtime in NATIVE:
            pause = (", and the simulation stands still meanwhile." if P.PAUSES[runtime] else
                     ", but the simulation runs on meanwhile: the robot holds its pose, and whatever is still moving "
                     "goes on moving.")
            return (f"only motions use steps. Every motion call ({calls}) is one reply to the benchmark and uses "
                    f"steps of the episode's one span of {SPAN_WORDS[runtime]}, which is all the task gets. "
                    f"Thinking and looking use none{pause}")
        return (f"only motions use steps. Every motion call ({calls}) is one reply to the benchmark and "
                "takes simulated time; each stage of the task allows only so much of both, and a stage done in "
                "fewer motions scores more. Thinking, looking and measuring cost nothing, but the scene does not "
                "freeze meanwhile: the robot holds its last pose, and whatever is still moving goes on moving.")

    def cell_note(self) -> str | None:
        n = self.link.dispatches
        if n == self._noted:
            return None
        self._noted = n
        return f"robot motions so far in this episode: {n}"
