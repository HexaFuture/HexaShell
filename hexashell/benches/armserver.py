"""The simulator server of a one-arm benchmark: the requests every such benchmark answers.

A benchmark's server module (``python -m hexashell.benches.<bench>.server <fd>``) subclasses
:class:`ArmServer` with ``open`` (build and reset an episode: sets ``sim``, ``robot`` and the home
pose) and its own operations, and calls :func:`main`.  Requests ``{"op": ..., "args": {...}}`` (see
:mod:`hexashell.benches.wire`) are answered one at a time; every reply has ``ok``, the episode
``status`` (step, horizon, success, over) and, after a motion, ``log``: one row per step (step,
grasp point xyz, hand rotation vector, finger gap, then what the benchmark adds).

Shared operations: observe, state, reach, goto, arc, push_arc, home, gripper, wait, close.  Poses are 4x4 in
the episode's frame; the hand's are of the tool axes (see :mod:`hexashell.benches.arm`).  The result
of a motion lists the arm's joints that ended at the end of their travel (``at_limit``).
"""

from __future__ import annotations

import math
import os
import socket
import time
import traceback

import numpy as np

from . import arm as A
from . import wire


class Refused(Exception):
    """A request the server will not carry out (the message says why); nothing moved."""


#: a joint this close (rad) to an end of its travel is at its limit
AT_LIMIT = math.radians(1.0)
#: a pose is refused when the arm would hold it with the elbow bent less than this (rad) short of
#: straight (arm.Elbow.room), and so is a goto or an arc that would on its way: the controller does
#: not bend a stretched arm back (it turns the wrist instead); it pushes the elbow on through straight
#: to the end of its travel, and the arm stays stuck there.  Arriving far out, the hand overshoots
#: outward by up to about 2 cm at the wrist (up to 12 deg less bend than the goal's near 35 deg)
ELBOW_ROOM = math.radians(35.0)
#: a goto or an arc is refused while the elbow is bent less than this (rad) short of straight (after a
#: motion stopped stretched): from there the controller pushes the elbow on through straight; the way
#: home (by the joints) bends it back
START_ROOM = math.radians(20.0)
#: share of the arm's speeds of the way home: the hand lags behind while the elbow is nearly straight
HOME_SPEED = 0.6
#: the way home starts with the arm giving way when the elbow is bent less than this (rad) short of
#: straight (and not past it)
HOME_ROOM = math.radians(30.0)
#: the arm is at home when each of its joints is this close (rad) to home's
HOME_NEAR = 0.05


def bend_words(bend: float) -> str:
    """The elbow's bend (rad short of straight, see arm.Elbow.room) in words: past straight it is bent
    backwards, which a count "short of straight" would show as a negative number."""
    deg = math.degrees(bend)
    if deg >= 0.5:
        return f"the elbow {deg:.0f} deg short of straight"
    if deg > -0.5:
        return "the elbow straight"
    return f"the elbow bent {-deg:.0f} deg backwards past straight"


class ArmServer:
    #: lowest the grasp point may be asked to go (z in the episode frame)
    floor_z = 0.0
    site_to_tool = A.PANDA_SITE_TO_TOOL
    #: the least room (rad) a pose or the way to it must leave the elbow (see ELBOW_ROOM)
    elbow_room = ELBOW_ROOM

    def __init__(self):
        self.sim = None
        self.robot: A.Arm | None = None
        #: the arm's joints at the start of the episode (``open`` sets them): ``home`` goes back to them
        self.home_joints: np.ndarray | None = None
        #: a warm server (:mod:`.daemon`) keeps the task's environment for its next episode
        self.warm = False

    # -- requests ----------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        if op == "open":
            return self.open(**args)
        if op == "ping":
            return {"pid": os.getpid()}
        if self.sim is None or self.robot is None:
            raise Refused("no episode: send open first")
        method = getattr(self, "op_" + str(op), None)
        if method is None:
            raise Refused(f"unknown operation {op!r}")
        return method(**args)

    def status(self) -> dict:
        return self.sim.status() if self.sim is not None else {}

    def open(self, **args) -> dict:
        raise NotImplementedError

    def home_goal(self) -> np.ndarray:
        """The tool pose ``home`` returns to (episode frame)."""
        raise NotImplementedError

    # -- reading -----------------------------------------------------------------------------------
    def op_observe(self, cams=None, robot_mask: bool = True) -> dict:
        return {"obs": self.sim.observe(cams, robot_mask=bool(robot_mask))}

    def op_state(self) -> dict:
        return {"proprio": self.sim.proprio(), "level": self.robot.level(), "grip_goal": self.robot.grip_goal}

    def op_reach(self, pose) -> dict:
        return {"reach": self._check(np.asarray(pose, float))}

    # -- motion ------------------------------------------------------------------------------------
    def _site_goal(self, tool_pose: np.ndarray) -> np.ndarray:
        """Grip site pose of a tool pose (both in the episode frame)."""
        goal = np.array(tool_pose, float)
        goal[:3, :3] = goal[:3, :3] @ self.site_to_tool.T
        return goal

    def _link0_goal(self, tool_pose: np.ndarray) -> np.ndarray:
        return np.linalg.inv(self.sim.link0()) @ self._site_goal(tool_pose)

    def _check(self, tool_pose: np.ndarray) -> dict:
        if tool_pose.shape != (4, 4) or not np.all(np.isfinite(tool_pose)):
            raise Refused("a pose is a finite 4x4 matrix")
        p = tool_pose[:3, 3]
        shoulder = self.sim.shoulder()
        out = {"shoulder": shoulder, "distance": float(np.linalg.norm(p - shoulder)), "floor": p[2] < self.floor_z}
        if out["floor"]:
            out.update(ok=False, why="below the floor limit")
            return out
        if out["distance"] > A.REACH_M:
            out.update(ok=False, why=f"{out['distance']:.2f} m from the shoulder (the arm reaches {A.REACH_M} m)")
            return out
        out.update(A.reach(self.sim, self._site_goal(tool_pose)))
        if out["ok"]:
            # the controller keeps the elbow as bent as the pose allows (arm.Elbow.room)
            bend = self.robot.elbow.room(self._link0_goal(tool_pose))
            out["elbow_deg"] = math.degrees(bend)
            if bend < self.elbow_room:
                out.update(ok=False, stretched=True,
                           why=f"there the arm would be stretched almost straight ({bend_words(bend)}; it needs "
                               f"{math.degrees(self.elbow_room):.0f}): a stretched arm gets stuck. "
                               f"That is about as far as the arm reaches " + self.reach_where)
        elif out["progress"] < 0.1:  # the search found nothing nearer than the hand is now
            out["why"] = "no pose of the arm was found for it " + self.reach_where
        else:
            out["why"] = (f"the hand gets within {100 * out['err_m']:.1f} cm and {out['err_deg']:.0f} deg of it "
                          + self.reach_where)
        return out

    #: how a failed reach check ends ("... of it" + this)
    reach_where = "from where the arm is"
    #: how a failed reach check ends when the arm gets there from home
    home_where = "from the arm's pose now; from home (robot.home()) it gets there"

    def _from_home(self, tool_pose: np.ndarray) -> bool:
        """Whether a goto to ``tool_pose`` that the reach check refused passes it from home (``home``'s
        joints, everything else where it is), the way there included: a search from the joints now can
        stall in a pose the arm has turned into (a joint at a limit, the wrist turned over)."""
        if self.home_joints is None or float(np.abs(self.sim.arm_joints() - self.home_joints).max()) < HOME_NEAR:
            return False
        if not A.reach(self.sim, self._site_goal(tool_pose), start=self.home_joints)["ok"]:
            return False
        goal, start = self._link0_goal(tool_pose), self._link0_goal(self.home_goal())
        if self.robot.elbow.room(goal) < self.elbow_room:
            return False
        return self._check_way(A.hand_path(start, goal, self.robot.elbow.room, A.ARM_SPEED), start, "") is None

    def _check_way(self, path: list[np.ndarray], start: np.ndarray, advice: str) -> dict | None:
        """The verdict on a plan (grip-site poses, frame of robot0_link0) that on its way leaves the elbow
        less room than ``elbow_room`` and less than it has now (arm.Elbow.room), ending ``advice``.  None
        when the way is fine."""
        if not path:
            return None
        rooms = np.array([self.robot.elbow.room(p) for p in path])
        k = int(np.argmin(rooms))
        if rooms[k] >= min(self.elbow_room, self.robot.elbow.room(start) - math.radians(1.0)):
            return None
        share = (k + 1) / len(path)
        return {"ok": False, "stretched": True, "elbow_deg": math.degrees(float(rooms[k])), "way_share": share,
                "why": f"on the way ({100 * share:.0f} % along) the arm would be stretched almost straight "
                       f"({bend_words(float(rooms[k]))}; it needs {math.degrees(self.elbow_room):.0f}): "
                       f"a stretched arm gets stuck. {advice}"}

    #: the advice of a goto whose straight line runs too far out (see :meth:`_check_way`)
    way_advice = ("A long way out while the hand turns a lot swings the wrist out: go in two moves, turning the hand "
                  "in the one nearer the arm")

    def _reply(self, result: dict) -> dict:
        return {"result": result, "proprio": self.sim.proprio(), "log": self.robot.take_history()}

    def at_limit(self) -> list[list]:
        """The arm's joints at an end of their travel: [joint number (1 at the shoulder), "low" or "high"]."""
        q = self.sim.arm_joints()
        low, high = self.sim.arm_limits[:, 0], self.sim.arm_limits[:, 1]
        out = []
        for k in range(len(q)):
            if q[k] - low[k] < AT_LIMIT:
                out.append([k + 1, "low"])
            elif high[k] - q[k] < AT_LIMIT:
                out.append([k + 1, "high"])
        return out

    def _stretched_now(self) -> dict | None:
        """The verdict on any goto or arc while the arm is stretched almost straight (START_ROOM)."""
        bend = self.robot.bend()
        if bend >= START_ROOM:
            return None
        return {"ok": False, "stretched_now": True, "elbow_now_deg": math.degrees(bend),
                "why": f"the arm is stretched almost straight now ({bend_words(bend)}): from there it gets stuck. "
                       "robot.home() first: it bends the elbow back"}

    def op_goto(self, pose, grip=None, dry_run: bool = False, check: bool = True, speed=None) -> dict:
        tool = np.asarray(pose, float)
        verdict = self._stretched_now() or (self._check(tool) if check else {"ok": True})
        if not verdict["ok"] and "progress" in verdict and not verdict.get("stretched") and self._from_home(tool):
            verdict.update(from_home=True, why=verdict["why"].removesuffix(self.reach_where) + self.home_where)
        path, start = None, None
        if verdict["ok"]:
            _, start = self.robot.site_now()
            path = A.hand_path(start, self._link0_goal(tool), self.robot.elbow.room,
                               A.ARM_SPEED if speed is None else float(speed))
            if check:
                verdict = self._check_way(path, start, self.way_advice) or verdict
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict})
        if self.sim.over:
            return self._reply({"moved": False, "reach": verdict, "over": True})
        if grip is not None:
            self.robot.set_grip(float(grip))
        result = self.robot.follow(path, start)
        return self._reply({"moved": True, "reach": verdict, **result, "at_limit": self.at_limit()})

    def op_arc(self, center, axis, angle, grip=None, dry_run: bool = False, speed=None) -> dict:
        """Turn the hand by ``angle`` (rad) about the line through ``center`` along ``axis``, at most
        ``speed`` (m/s; None: the arm's speed)."""
        center = np.asarray(center, float).reshape(-1)
        axis = np.asarray(axis, float).reshape(-1)
        angle = float(angle)
        if center.size != 3 or axis.size != 3 or not np.all(np.isfinite(np.r_[center, axis, angle])) \
                or float(np.linalg.norm(axis)) < 1e-6:
            raise Refused("arc: center and axis are 3 numbers each (the axis not zero), the angle a number")
        if abs(angle) > math.radians(180.0):
            raise Refused("arc: at most 180 degrees at a time")
        link0 = self.sim.link0()
        to_link0 = np.linalg.inv(link0)
        _, start = self.robot.site_now()
        path = A.arc_path(start, (to_link0 @ np.r_[center, 1.0])[:3], link0[:3, :3].T @ axis, angle,
                          room=self.robot.elbow.room, speed=A.ARM_SPEED if speed is None else float(speed))
        verdict = self._stretched_now() or {"ok": True}
        for k in sorted({len(path) // 2, len(path) - 1}) if path and verdict["ok"] else []:
            pose = link0 @ path[k]
            pose[:3, :3] = pose[:3, :3] @ self.site_to_tool
            verdict = self._check(pose)
            if not verdict["ok"]:
                verdict["why"] = ("the end of the arc: " if k == len(path) - 1 else "half way: ") + verdict["why"]
                break
        if verdict["ok"]:
            verdict = self._check_way(path, start, "The arc runs too far from the arm " + self.reach_where) or verdict
        end = link0 @ path[-1] if path else link0 @ start
        end[:3, :3] = end[:3, :3] @ self.site_to_tool
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict, "end": end})
        if self.sim.over:
            return self._reply({"moved": False, "reach": verdict, "over": True, "end": end})
        if grip is not None:
            self.robot.set_grip(float(grip))
        result = self.robot.follow(path, start)
        return self._reply({"moved": True, "reach": verdict, "end": end, **result, "at_limit": self.at_limit()})

    def op_push_arc(self, center, axis, angle, push, press=0.03, tilt=0.0, dry_run: bool = False) -> dict:
        """Push the hinged part the hand touches by up to ``angle`` (rad) about the line through ``center``
        along ``axis``, pressing ``press`` (m) along ``push`` leaned ``tilt`` (rad) toward the way the part
        turns (episode frame; see arm.Arm.push_arc).  A dry run checks the hand's pose half way and at the
        end of that turn as an arc would make it."""
        center, axis, push = (np.asarray(v, float).reshape(-1) for v in (center, axis, push))
        angle, press, tilt = float(angle), float(press), float(tilt)
        if center.size != 3 or axis.size != 3 or push.size != 3 \
                or not np.all(np.isfinite(np.r_[center, axis, push, angle, press, tilt])) \
                or float(np.linalg.norm(axis)) < 1e-6 or float(np.linalg.norm(push)) < 1e-6:
            raise Refused("push_arc: center, axis and push are 3 numbers each (axis and push not zero), "
                          "the angle a number")
        if not 0.0 < abs(angle) <= math.radians(180.0):
            raise Refused("push_arc: the angle is between -180 and 180 degrees (not 0)")
        if not A.PRESS_RANGE[0] <= press <= A.PRESS_RANGE[1]:
            raise Refused(f"push_arc: press is {A.PRESS_RANGE[0]:g} to {A.PRESS_RANGE[1]:g} m")
        if not 0.0 <= tilt <= math.radians(60.0):
            raise Refused("push_arc: tilt is 0 to 60 degrees")
        link0 = self.sim.link0()
        c = (np.linalg.inv(link0) @ np.r_[center, 1.0])[:3]
        a, u = link0[:3, :3].T @ axis, link0[:3, :3].T @ push
        _, start = self.robot.site_now()
        off = start[:3, 3] - c
        if float(np.linalg.norm(off - a * float(off @ a) / float(a @ a))) < 0.02:
            raise Refused("push_arc: the hand is on the hinge line; center is a point on the hinge, away from the hand")
        verdict = self._stretched_now() or {"ok": True}
        if verdict["ok"] and dry_run:
            path = A.arc_path(start, c, a, angle, room=self.robot.elbow.room)
            for k in sorted({len(path) // 2, len(path) - 1}) if path else []:
                pose = link0 @ path[k]
                pose[:3, :3] = pose[:3, :3] @ self.site_to_tool
                verdict = self._check(pose)
                if not verdict["ok"]:
                    verdict["why"] = ("at the end of the turn: " if k == len(path) - 1 else "half way: ") \
                        + verdict["why"]
                    break
        if not verdict["ok"] or dry_run:
            return self._reply({"moved": False, "dry_run": bool(dry_run), "reach": verdict})
        if self.sim.over:
            return self._reply({"moved": False, "reach": verdict, "over": True})
        result = self.robot.push_arc(c, a, angle, u, press, tilt)
        return self._reply({"moved": True, "reach": verdict, **result, "at_limit": self.at_limit()})

    def op_home(self) -> dict:
        """Back to the home pose: first straight up to its height when well below it (if the arm can),
        then the arm's joints straight back to where they started (without them, the hand's straight
        line), at HOME_SPEED.  A straight line of the hand from a pose far from home, turned a lot, can
        run a joint (mostly the elbow) into its limit; the joints' line cannot.  The joints' way back
        bends a stretched elbow, so it is not stopped for stretching; but from an elbow nearly straight
        (HOME_ROOM) it can push it on through straight, so the arm first gives way (arm.Arm.give_way).
        An elbow bent backwards past straight would stay so: the hand first goes out (arm.Arm.unbend)."""
        goal = self.home_goal()
        parts = []
        if self.home_joints is not None and not self.sim.over and self.robot.bend() < 0.0:
            parts.append({"steps": self.robot.unbend()["steps"], "blocked": False})
        if self.home_joints is not None and 0.0 < self.robot.bend() < HOME_ROOM:
            steps, _ = self.robot.give_way()
            parts.append({"steps": steps, "blocked": False, "gave_way": steps})
        now = self.sim.tcp()
        if not self.sim.over and now[2, 3] < goal[2, 3] - 0.05:
            up = now.copy()
            up[2, 3] = goal[2, 3]
            if self._check(up)["ok"]:
                parts.append(self.robot.move(self._link0_goal(up)))
        if not self.sim.over and not (parts and parts[-1]["blocked"]):
            if self.home_joints is None:
                parts.append(self.robot.move(self._link0_goal(goal)))
            else:
                _, start = self.robot.site_now()
                parts.append(self.robot.follow(A.joint_path(self.sim, self.home_joints, speed=HOME_SPEED), start,
                                               guard=False))
        tcp = self.sim.tcp()
        dist = float(np.linalg.norm(tcp[:3, 3] - goal[:3, 3]))
        turn = math.degrees(A.turn_between(tcp[:3, :3], goal[:3, :3]))
        return self._reply({"moved": bool(parts), "steps": sum(p["steps"] for p in parts),
                            "blocked": any(p["blocked"] for p in parts), "jammed": any(p.get("jammed") for p in parts),
                            "home_m": dist, "home_deg": turn,
                            "goal": goal, "at_limit": self.at_limit()})

    def op_gripper(self, opening) -> dict:
        return self._reply(self.robot.grip(float(opening)))

    def op_wait(self, n) -> dict:
        n = int(n)
        if not 0 < n <= 400:
            raise Refused("wait: 1..400 steps")
        return self._reply(self.robot.wait(n))

    def op_close(self) -> dict:
        """The episode is over.  The environment is freed, or, in a warm server, kept for the next
        episode (``open`` builds a new one only for another task)."""
        self.robot = None
        if not self.warm:
            sim, self.sim = self.sim, None
            if sim is not None:
                sim.close()
        return {}


def serve(sock: socket.socket, server) -> bool:
    """Answer requests until ``close`` (returns True) or until the other side goes away (False)."""
    while True:
        try:
            request = wire.recv(sock)
        except wire.WireError:
            return False
        op = str(request.get("op", ""))
        args = request.get("args") or {}
        t0 = time.time()
        try:
            reply = {"ok": True, **server.handle(op, args)}
        except Refused as exc:
            reply = {"ok": False, "kind": "refused", "error": str(exc)}
        except Exception as exc:  # noqa: BLE001 - reported to the episode side
            reply = {"ok": False, "kind": "crash", "error": f"{type(exc).__name__}: {exc}",
                     "trace": traceback.format_exc()[-6000:]}
        # a failed open leaves a warm server's previous scene in place: its status is not this episode's
        reply["status"] = {} if op == "open" and not reply["ok"] else server.status()
        reply["seconds"] = round(time.time() - t0, 3)
        try:
            wire.send(sock, reply)
        except wire.WireError:
            return op == "close"
        if op == "close":
            return True


def prepare() -> None:
    """Before the simulator is imported: render on ``$HEXA_EGL_DEVICE``."""
    from . import egl

    egl.pin(os.environ.get("HEXA_EGL_DEVICE"))


def main(argv: list[str], server: ArmServer) -> None:
    """Serve on the socket whose descriptor is ``argv[0]``, rendering on ``$HEXA_EGL_DEVICE``."""
    prepare()
    sock = socket.socket(fileno=int(argv[0]))
    try:
        serve(sock, server)
    finally:
        sock.close()
