"""A DexJoCo-X episode: the simulator server, the robot calls, the task card and the verdict.

The server (:mod:`.server`) runs in DexJoCo-X's Python environment (bench-dexjoco-x, ``$DEXJOCO_X_ROOT`` the
release); :class:`DexterousSession` starts it, opens the official episode of the task's scene seed (a learning
or validation scene of the Wuji demonstrations, or for a test seed an official evaluation scene, which the run
must allow: protocol.OFFICIAL_TEST_VARIABLE=1) and answers the cells' robot calls through
:class:`.host.DexterousHost`.  The episode runs to the official step limit and ends at the first step at which
the environment reports success, or when the environment ends it.  Records: ``server.log`` (the server's
output) next to the usual files; ``states.jsonl`` has one line per control step.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ..session import ArmSession
from . import protocol as P
from .host import DexterousHost

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "dexjoco_x"

#: what the scene camera sees (protocol.Task.scene_camera)
SCENE_CAMERA_WORDS = {
    "across": "fixed across the table, facing the robot (the robot's right is on the picture's left)",
    "behind": "fixed behind and above the robot, looking over it at the table (the robot's right is on the "
              "picture's right)",
}
WRIST_WORDS = "on the {side} wrist, looking past the fingers (along them, tilted 30 degrees to the palm's side); " \
              "it moves with the hand"

#: what the environment checks, per task (from the release's code: benchmark6/environments.py, ipad_task.py,
#: single_hand_hanoi.py and the task environments in sim/envs)
TASK_CHECKS: dict[str, str] = {
    "hammer_nail":
        "the nail is driven 4 cm into the block. Only a blow of the hammer's metal part (its face, head, neck or "
        "claw; not the handle, not the hand) on the nail drives it: by 0.8 cm per 2 cm/s of the head's downward "
        "speed just before the blow, at most 2.4 cm per blow (from 6 cm/s on), so it takes two blows or more. How "
        "the hammer is held is not checked.",
    "pick_bucket":
        "at one step, the boxed object's centre is within the basket's bounds and the basket is lifted at least "
        "15 cm (four reference corners of the basket, on its wall toward -y, are all 15 cm or more above where they "
        "started). How the basket is held is not checked.",
    "bimanual_microwave_cook":
        "at one step, the food's centre is inside the microwave, the door is shut (within 0.6 degrees of closed) "
        "and something touches the start button. The door starts shut. It is 46 cm wide, hinged at the front of its "
        "left (+y) edge (the hinge line about 3.5 cm inside the cabinet's left side), and opens toward the robot, up "
        "to 92 degrees; its handle stands upright on its outer face near the right edge, 43 cm from the hinge line.",
    "bimanual_photograph":
        "at one step: the camera's lens points within 18 degrees of +x (at the target disc; at the start it points "
        "90 degrees away from it); a point on the lens's axis 1.4 cm in front of the lens (the lens stands 9 cm out "
        "of the camera's front, so the point is 13.5 cm in front of the middle of the camera's body) is 30 to 50 cm "
        "in front of the disc (in x) and within 16 cm of the disc's centre in y and z; and something touches the "
        "shutter button on the camera's "
        "top (a right fingertip on the camera's top within about 2 cm of the button counts too). How the camera is "
        "held is not checked. The button is a small round cap, 3 cm across, at one end of the camera's top, near "
        "its back: with the lens toward +x and the top up, at the right (-y) end. At the start the camera leans "
        "with its lens toward +y and down and its top toward +x, the button at its lower (-y) end, on the side "
        "away from the robot.",
    "bimanual_unlock_ipad":
        "first the left hand holds the tablet: the middle of its screen at least 5 cm above where it started, the "
        "tablet at least 5 mm clear of everything but the hands (the stand, the table, the arms), gripped by two "
        "parts of the hand (two fingers, or a finger and the palm) pressing from opposite sides, and still (under "
        "8 cm/s and 0.5 rad/s) for 0.5 s on end; then right fingertips press 1, 2 and 3 in that order, each press "
        "a new touch (between presses no right fingertip touches the tablet). It succeeds 10 steps after 3 is "
        "pressed. The episode ends at once without success if the right hand touches the tablet before the left "
        "hand holds it so, if after that the left hand's hold is lost (or the screen's middle sinks below 4 cm "
        "above its start, or the tablet comes within 5 mm of anything but the hands), or if a right fingertip "
        "presses another digit. At the start the tablet, about 42 cm wide (along y) and 27 cm tall, leans on its "
        "stand with the screen toward the robot and up; the stand, 18 cm wide, is under its middle.",
    "single_hand_hanoi":
        "the posts A, B and C stand in a row along y, about 21 cm apart, C at the -y end. At the start the large "
        "disk is on C, the medium on B and the small on A. Success: post C holds large, medium and small from the "
        "bottom up and A and B are empty, each disk within 4 cm of C's axis and seated at its height in the stack, "
        "all still, for 20 steps on end, and then for 20 more steps no disk touches the right hand. The episode "
        "ends at once without success when for 10 steps on end everything is still and a larger disk sits on a "
        "smaller one, or a disk lies on the table (off every post and low). The posts are 1.6 cm thick and stand "
        "18 cm above the base, their tops 13 cm above the top of a disk lying on the base. The disks are square "
        "frames 5 cm tall, 5, 6.6 and 10 cm wide (small, medium, large), with a square hole through the middle "
        "2.2, 2.9 and 3.5 cm across: the small disk's hole leaves 3 mm on each side of a post.",
}


def skill(task: str) -> str | None:
    path = SKILLS / f"{task}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


def _xyz(v) -> str:
    return "(" + ", ".join(f"{float(a):.3f}" for a in v) + ")"


def camera_words(task: str, cam: str) -> str:
    if cam == "scene":
        return SCENE_CAMERA_WORDS[P.TASKS[task].scene_camera]
    return WRIST_WORDS.format(side=cam.split("_")[0])


class DexterousSession(ArmSession):
    bench = "dexjoco_x"
    proxy = "hexashell.benches.dexjoco_x.proxy"
    server_module = "hexashell.benches.dexjoco_x.server"
    host_class = DexterousHost

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=P.IMAGE_SIZE)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])
        P.check_task(self.task)
        #: a test seed (an official evaluation scene) is opened only when the run allows it
        self.official_test = P.split_of(self.task, self.seed) == "test" and P.official_test_allowed()
        P.check_seed(self.task, self.seed, official_test=self.official_test)

    # -- the task's cameras ------------------------------------------------------------------------
    @property
    def cams(self) -> tuple[str, ...]:
        return P.cameras(self.task)

    def check_cam(self, cam) -> str:
        from ..armhost import camera_checker

        return camera_checker(self.cams, P.CAMERA_ALIASES)(cam)

    def open_args(self) -> dict[str, Any]:
        args = {"task": self.task, "seed": self.seed}
        if self.official_test:
            args["official_test"] = True
        return args

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.task}_s{self.seed}"

    def labels(self) -> dict[str, Any]:
        split = P.split_of(self.task, self.seed)
        labels = {"task": self.task, "seed": self.seed, "split": split, "hand": P.HAND}
        if split == "test":
            labels["official_episode"] = self.seed - P.test_seed(self.task, 0)
        return labels

    def over_reason(self) -> str:
        ended = self.status.get("failure")
        if ended and not self.failure and not self.status.get("success"):
            return f"the environment ended the episode without success: {ended}"
        return super().over_reason()

    def card(self, settings) -> tuple[str, dict]:
        f, host = self.facts, self.host
        spec = P.TASKS[self.task]
        limit = int(f.get("steps") or spec.steps)
        hold = int(f.get("start_hold_steps") or 0)
        hand_speed, turn_speed = P.speeds(self.task)["hand"], P.speeds(self.task)["turn"]
        arms = " and ".join(spec.arms)
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {f.get('instruction')!r}",
                 f"Task: {self.task} (DexJoCo-X: Franka Panda {'arms' if len(spec.arms) > 1 else 'arm'} with "
                 f"five-fingered Wuji {'hands' if len(spec.arms) > 1 else 'hand'}; the {arms} "
                 f"{'arms' if len(spec.arms) > 1 else 'arm'}, fixed in place)",
                 f"Step budget: {limit} steps ({limit * P.STEP_S:.0f} s of simulated time; one step is {P.STEP_S:g} s) "
                 f"for the whole episode (used so far: {self.step or 0}: the robot held still for the first {hold} "
                 "while the objects settled). The episode ends by itself the moment the environment reports success."]
        checks = TASK_CHECKS.get(self.task)
        if checks:
            lines.append(f"What the environment checks: {checks}")
        lines += ["Scene frame: metres, z up, the robot faces +x, y to its left; it is fixed.",
                  f"Hand motions go at {hand_speed[0]:g} m/s unless asked otherwise (at most "
                  f"{min(hand_speed[1], 0.25):g}); arcs at {turn_speed[0]:g} degrees per second (at most "
                  f"{turn_speed[1]:g}).",
                  "Start pose of each hand (robot.home() returns to it); the fingers open:"]
        for side in spec.arms:
            h = host.start["hands"][side]
            lines.append(f"- {side}: palm {_xyz(h['palm'])}, facing {_xyz(h['normal'])}, fingers along "
                         f"{_xyz(h['finger'])}; grasp point {_xyz(h['grasp'])}")
        shown = [c for c in self.cams if c in self.first]
        if shown:
            words = "; ".join(f"{c} ({camera_words(self.task, c)})" for c in shown)
            lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "skill": False}
        if settings.skills:
            text = skill(self.task)
            if text:
                lines += ["", "## Notes for this task", "", text]
                used["skill"] = True
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        s = self.status
        return {"task": self.task, "seed": self.seed, "split": P.split_of(self.task, self.seed), "hand": P.HAND,
                "instruction": self.instruction, "environment_ended": s.get("failure"),
                "terminated": bool(s.get("terminated")), "truncated": bool(s.get("truncated"))}

    # -- records ---------------------------------------------------------------------------------
    def _rows(self, rows: np.ndarray) -> None:
        """server.LOG_COLUMNS per step: step, right and left palm points, success, and whether the physics has
        each hand holding something (for the records only)."""
        for row in np.asarray(rows, float):
            rec = {"step": int(row[0]), "success": bool(row[7] > 0.5)}
            for k, side in ((1, "right"), (4, "left")):
                if np.all(np.isfinite(row[k:k + 3])):
                    rec[f"{side}_palm"] = [round(float(v), 4) for v in row[k:k + 3]]
            for k, side in ((8, "right"), (9, "left")):
                if len(row) > k and np.isfinite(row[k]):
                    rec[f"{side}_holding_physics"] = bool(row[k] > 0.5)
            self.rec.state(rec)
