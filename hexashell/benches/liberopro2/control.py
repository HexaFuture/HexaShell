"""Moving LIBERO-Pro 2.0's Panda through its official controller (server side).

Every step sends the official action (libero_action_v1): robosuite 1.4's OSC_POSE in delta mode, translation and
axis-angle rotation in MuJoCo's world frame, then the gripper's sign (negative opens)::

    arm[0:6] (OSC_POSE deltas) | gripper[6] (sign)

The arm and the gripper move as :mod:`hexashell.benches.arm` describes; robosuite 1.4's gripper moves its level
0.25 a step (:data:`GRIP_STEP`), so openings are set in steps of 0.125.

A perturbation may delay the readings or the commands (:mod:`.timing`, the *lag*): the arm then steers the hand as
predicted ahead of the readings (``sim.data``: where the commands already sent will leave it), and what only the
readings can tell waits for them.  Whether the hand keeps up with its plan is judged from the readings against the
planned pose they show (the one sent ``lag`` steps before; not before they show the motion); when the lag changes,
the plan goes on from its pose nearest the hand; a grip takes the fingers as still once the readings show the steps
since the level last moved (:meth:`Robot.follow`, :meth:`Robot.grip`).  With no lag the arm moves as the shared
one's, step for step.
"""

from __future__ import annotations

import math

import numpy as np

from ..arm import (ARM_SPEED, ARM_TURN, ARRIVED, BEHIND, BLOCKED_STEPS, CLOSING, DT, FAR_BEHIND, FINGERS_STILL,
                   GRIP_STEPS_MAX, KEEP_UP, SETTLE_STEPS, STALL_SHARE, VERY_FAR, Arm, rotvec, turn_between)
from . import protocol as P
from . import sim as S

if abs(DT - P.STEP_S) > 1e-9:
    raise RuntimeError("the arm control assumes LIBERO's control period")

#: lowest the grasp point may be asked to go (arm frame: the table or floor the robot works on is near z = 0 in
#: every LIBERO scene)
FLOOR_Z = -0.05
GRIP_STEP = S.GRIP_SPEED * S.SUBSTEPS
#: a motion's plan takes at most twice its steps and PLAN_EXTRA more (when the lag changes it goes on from its pose
#: nearest the hand, perhaps back along it)
PLAN_EXTRA = 40


class OSC:
    """robosuite 1.4's OSC_POSE controller as :class:`~hexashell.benches.arm.Arm` reads robosuite 1.5's: its
    deltas are in MuJoCo's world frame (``origin_ori``: no turn)."""

    name = "OSC_POSE"
    input_type = "delta"

    def __init__(self, controller):
        if controller.name != "OSC_POSE" or not controller.use_delta:
            raise RuntimeError(f"unexpected arm controller: {controller.name} (deltas: {controller.use_delta})")
        self.input_max, self.input_min = (np.asarray(v, float) for v in (controller.input_max, controller.input_min))
        self.output_max, self.output_min = (np.asarray(v, float)
                                            for v in (controller.output_max, controller.output_min))
        self.kp, self.kd = np.asarray(controller.kp, float), np.asarray(controller.kd, float)
        self.origin_ori = np.eye(3)


class Robot(Arm):
    """The arm and gripper of one episode.  Rebuild after every reset (robosuite builds a new controller)."""

    grip_step = GRIP_STEP

    def __init__(self, sim, log=None):
        if int(sim.rs.action_dim) != 7:
            raise RuntimeError(f"unexpected action dim {sim.rs.action_dim}")
        super().__init__(sim, OSC(sim.rs.robots[0].controller), sim.gripper, log=log)
        #: the step the gripper's level last moved; whether the last step changed the lag (the hand as predicted then
        #: moved with the lag, not by the step)
        self.level_moved = int(sim.steps)
        self.lag_changed = False

    def action(self, arm6: np.ndarray, grip: float, **extra) -> np.ndarray:
        out = np.zeros(7)
        out[0:6] = arm6
        out[6] = grip
        return out

    def step(self, arm6: np.ndarray, **extra) -> bool:
        level, changes = self.level(), len(self.sim.timing.changes)
        alive = super().step(arm6, **extra)
        if self.level() != level:
            self.level_moved = int(self.sim.steps)
        self.lag_changed = len(self.sim.timing.changes) != changes
        return alive

    def _stretching(self, margin_before: float) -> tuple[bool, float]:
        """As the arm's, but not after a step that changed the lag (the margin then taken afresh)."""
        if self.lag_changed:
            return False, self.elbow.margin(self.bend())
        return super()._stretching(margin_before)

    # -- the readings, where they lag the hand as predicted ----------------------------------------
    def _seen_lag(self, target: np.ndarray) -> tuple[float, float]:
        """:meth:`_lag` of the hand as read."""
        now = np.linalg.inv(self.sim.link0()) @ self.sim.site_pose(self.sim.seen)
        return float(np.linalg.norm(target[:3, 3] - now[:3, 3])), turn_between(now[:3, :3], target[:3, :3])

    def _seen_velocity(self) -> tuple[np.ndarray, np.ndarray]:
        """:meth:`velocity` of the hand as read."""
        import mujoco

        m, d, sim = self.sim.model, self.sim.seen, self.sim
        site, body = np.zeros(6), np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_SITE, sim.site_id, site, 0)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, sim.link0_id, body, 0)
        rot = d.xmat[sim.link0_id].reshape(3, 3)
        arm = d.site_xpos[sim.site_id] - d.xpos[sim.link0_id]
        linear = site[3:] - body[3:] - np.cross(body[:3], arm)
        return rot.T @ linear, rot.T @ (site[:3] - body[:3])

    def _not_keeping_up(self, lag_m, lag_r, before, after, velocity=None) -> bool:
        """As the arm's, with the hand's ``velocity`` (linear, angular; None: as predicted)."""
        move = after[:3, 3] - before[:3, 3]
        turn = rotvec(after[:3, :3] @ before[:3, :3].T)
        want_v = float(np.linalg.norm(move)) / (2.0 * DT)
        want_w = float(np.linalg.norm(turn)) / (2.0 * DT)
        v, w = self.velocity() if velocity is None else velocity
        slow_v = want_v < 1e-6 or float(v @ move) / (2.0 * DT * want_v) < KEEP_UP * want_v
        slow_w = want_w < 1e-6 or float(w @ turn) / (2.0 * DT * want_w) < KEEP_UP * want_w
        return (lag_m > BEHIND[0] and slow_v) or (lag_r > BEHIND[1] and slow_w)

    def _nearest(self, path: list[np.ndarray]) -> int:
        """The index after the plan's pose nearest the hand as predicted (a turn of a rad counting as ARM_SPEED /
        ARM_TURN m)."""
        _, now = self.site_now()
        per_rad = ARM_SPEED / ARM_TURN
        far = [float(np.linalg.norm(p[:3, 3] - now[:3, 3])) + per_rad * turn_between(now[:3, :3], p[:3, :3])
               for p in path]
        return int(np.argmin(far)) + 1

    # -- arm and gripper -------------------------------------------------------------------------
    def follow(self, path: list[np.ndarray], start: np.ndarray, guard: bool = True, strain: bool = True) -> dict:
        """As the arm's (:meth:`hexashell.benches.arm.Arm.follow`), the readings perhaps lagging (see the module):
        whether the hand keeps up with the plan is judged from the readings, against the plan's pose sent ``lag``
        steps before (once they show this motion); when the lag changes, the plan goes on from its pose nearest the
        hand (PLAN_EXTRA) and the settling starts its counts afresh (and the elbow's watch: :meth:`_stretching`)."""
        goal_site = path[-1] if path else start
        self.came_from = self.sim.arm_joints()
        self.integral[:] = 0.0
        steps, behind, blocked, alive = 0, 0, False, not self.sim.over
        stretched, jammed, margin = False, False, self.elbow.margin(self.bend())
        sent: list = []  # each step's planned poses: (before, the step's, after)
        i = 0
        while i < len(path) and steps < 2 * len(path) + PLAN_EXTRA:
            if not alive:
                break
            pose = path[i]
            before = path[i - 1] if i > 0 else start
            after = path[i + 1] if i + 1 < len(path) else pose
            alive = self.step(self.arm_command(before, pose, after))
            steps += 1
            i += 1
            sent.append((before, pose, after))
            if strain and self.straining():
                jammed = True
                break
            stretched, margin = self._stretching(margin)
            if guard and stretched:
                break
            stretched = False
            if self.lag_changed:
                i, behind = self._nearest(path), 0
                self.integral[:] = 0.0
                continue
            lag = self.sim.lag
            if lag == 0:
                lag_m, lag_r = self._lag(pose)
                slow = self._not_keeping_up(lag_m, lag_r, before, after)
            elif len(sent) > lag:
                was_before, was, was_after = sent[-1 - lag]
                lag_m, lag_r = self._seen_lag(was)
                slow = self._not_keeping_up(lag_m, lag_r, was_before, was_after, self._seen_velocity())
            else:
                continue  # the readings do not show this motion yet
            if lag_m > VERY_FAR[0] or lag_r > VERY_FAR[1] or slow:
                behind += 1
            else:
                behind = 0
            if behind >= BLOCKED_STEPS:
                blocked = True
                break
        settled, stalled = 0, False
        if not blocked and not stretched and not jammed:
            scores: list[float] = []
            tol, stall = self.settle_tol, self.stall_steps
            far_before = None
            for _ in range(SETTLE_STEPS):
                if not alive:
                    break
                if self.lag_changed:
                    scores, far_before, behind = [], None, 0
                lag_m, lag_r = self._lag(goal_site)
                if lag_m < tol[0] and lag_r < tol[1] and (not self.settle_rest or self.resting()):
                    break
                # still far from the goal after the plan and not closing in (something holds the hand
                # back): blocked, the count going on from the plan's last steps
                far = lag_m / FAR_BEHIND[0] + lag_r / FAR_BEHIND[1]
                if (lag_m > FAR_BEHIND[0] or lag_r > FAR_BEHIND[1]) \
                        and not (far_before is not None and far < CLOSING * far_before):
                    behind += 1
                    if behind >= BLOCKED_STEPS:
                        blocked = True
                        break
                else:
                    behind = 0
                far_before = far
                # the lag in units of the settle tolerance; no longer shrinking: stop waiting
                scores.append(lag_m / tol[0] + lag_r / tol[1])
                if len(scores) > stall and scores[-1] > STALL_SHARE * scores[-1 - stall]:
                    stalled = True
                    break
                alive = self.step(self.arm_command(goal_site, goal_site, goal_site))
                steps += 1
                settled += 1
                if strain and self.straining():
                    jammed = True
                    break
                stretched, margin = self._stretching(margin)
                if guard and stretched:
                    break
                stretched = False
        gave_way, (back, moved) = 0, (False, 0.0)
        if jammed or stretched:
            gave_way, alive = self.give_way(None if jammed else self.came_from)
            back, moved = self.last_give_way
            steps += gave_way
        err_m, err_r = self._lag(goal_site)
        reached = not blocked and not stretched and not jammed and err_m < ARRIVED[0] and err_r < ARRIVED[1]
        _, now = self.site_now()
        self.hold = goal_site.copy() if reached else now
        if not reached:
            self.integral[:] = 0.0
        return {"steps": steps, "planned": len(path), "settle": settled, "blocked": blocked or jammed,
                "jammed": jammed, "gave_way": gave_way, "gave_way_back": back, "gave_way_m": moved,
                "reached": reached, "stalled": stalled and not reached,
                "stretched": stretched, "bend_deg": math.degrees(self.bend()), "err_m": err_m,
                "err_deg": math.degrees(err_r), "over": self.sim.over}

    def grip(self, opening: float) -> dict:
        """As the arm's (:meth:`hexashell.benches.arm.Arm.grip`), the fingers taken as still only once the readings
        show two steps since the level last moved (the lag and two), GRIP_STEPS_MAX and the lag at most."""
        self.set_grip(opening)
        steps, alive = 0, not self.sim.over
        self.count_anew()
        while alive and steps < GRIP_STEPS_MAX + self.sim.lag:
            lag = self.sim.lag
            at_goal = abs(self.level() - self.grip_goal) < 1e-6
            shown = steps >= 2 and (lag == 0 or self.sim.steps - self.level_moved >= lag + 2)
            if at_goal and shown and self.sim.finger_speed() < FINGERS_STILL:
                break
            n, alive = self.hold_step()
            steps += n
        return {"steps": steps, "width": self.sim.finger_width(), "level": self.level(), "goal": self.grip_goal,
                **self.gave_way_counts(), "over": self.sim.over}
