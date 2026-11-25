"""The FR3 and its hand through franky: the only module that imports it.

:class:`FrankyDriver` holds the FCI connection for as long as the service runs.  It follows the planner's joint
waypoints with franky's ``JointWaypointMotion``, each waypoint carrying its velocity and its leg's minimum time so
the arm passes through instead of stopping.  It never recovers from a reflex on its own: a fault ends the motion
and stays reported until an operator asks for :meth:`recover`.

The gripper's rules are those of the Franka Hand's ``grasp``: it closes until blocked and reports success only if
the width lies within the epsilons, so a closing command takes any width from the target up to the hand's maximum.
Opening is ``move``.  Waiting for the hand has a deadline: a future that never ends would hold the service.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from .planner import TimedPath

#: One gripper command may take this long (s)
GRIPPER_DEADLINE_S = 10.0
#: What the service accepts for the hand's speed (m/s) and grasping force (N)
GRIPPER_SPEED_LIMIT_M_S = 0.05
GRIPPER_FORCE_LIMIT_N = 40.0
#: After a motion every joint must be this close to its goal (rad)
GOAL_TOLERANCE_RAD = 0.02
POLL_S = 0.01


class RobotFault(RuntimeError):
    """The robot could not finish a command; the message is franky's or says what was found."""


def fault_types(franky) -> tuple[type, ...]:
    """What franky raises: GripperException stands beside franky.Exception, not under it."""
    return franky.Exception, franky.GripperException


def joint_waypoint_motion(franky, path: TimedPath, *, relative_dynamics: float = 1.0):
    """franky's motion for a planned path (its first waypoint is where the arm already is)."""
    legs_ms = np.diff(path.t) * 1000.0
    waypoints = [franky.JointWaypoint(franky.JointState(q.tolist(), dq.tolist()),
                                      minimum_time=franky.Duration(max(1, int(round(leg)))))
                 for q, dq, leg in zip(path.q[1:], path.dq[1:], legs_ms)]
    return franky.JointWaypointMotion(waypoints,
                                      relative_dynamics_factor=franky.RelativeDynamicsFactor(relative_dynamics))


def grip_command(width_m: float, current_m: float, max_width_m: float, *, speed_m_s: float, force_n: float) -> dict:
    """What the hand does for an opening: ``move`` to open, ``grasp`` (any block up to its maximum) to close."""
    if not 0.0 <= width_m <= max_width_m:
        raise ValueError(f"gripper width {width_m} m is outside 0..{max_width_m}")
    if not 0.0 < speed_m_s <= GRIPPER_SPEED_LIMIT_M_S:
        raise ValueError(f"gripper speed {speed_m_s} m/s is outside (0, {GRIPPER_SPEED_LIMIT_M_S}]")
    if not 0.0 < force_n <= GRIPPER_FORCE_LIMIT_N:
        raise ValueError(f"grasp force {force_n} N is outside (0, {GRIPPER_FORCE_LIMIT_N}]")
    if width_m >= current_m:
        return {"kind": "move", "width": width_m, "speed": speed_m_s}
    return {"kind": "grasp", "width": width_m, "speed": speed_m_s, "force": force_n,
            "epsilon_inner": width_m, "epsilon_outer": max_width_m - width_m}


class FrankyDriver:
    def __init__(self, robot_ip: str, *, realtime: bool = True, dynamics_factor: float = 0.1,
                 gripper_speed_m_s: float = 0.05, grasp_force_n: float = 20.0):
        import franky

        self.franky = franky
        self.faults = fault_types(franky)
        self.name = "Franka FR3"
        self.mode = "real"
        self.robot = franky.Robot(
            robot_ip, relative_dynamics_factor=franky.RelativeDynamicsFactor(dynamics_factor),
            realtime_config=franky.RealtimeConfig.Enforce if realtime else franky.RealtimeConfig.Ignore)
        self.gripper = franky.Gripper(robot_ip)
        self.max_width_m = float(self.gripper.max_width)
        self.gripper_speed_m_s, self.grasp_force_n = float(gripper_speed_m_s), float(grasp_force_n)

    # -- reading ---------------------------------------------------------------------------------
    def joints(self) -> np.ndarray:
        return np.asarray(self.robot.current_joint_state.position, float)

    def gripper_width_m(self) -> float:
        return float(self.gripper.width)

    def health(self) -> dict:
        state = self.robot.state
        mode = getattr(state.robot_mode, "name", str(state.robot_mode).split(".")[-1])
        return {"robot_mode": mode, "has_errors": bool(self.robot.has_errors), "errors": str(state.current_errors),
                "control_command_success_rate": float(state.control_command_success_rate)}

    # -- acting ----------------------------------------------------------------------------------
    def follow(self, path: TimedPath, cancel: threading.Event) -> None:
        """Run a path; returns when the arm has stopped, at its goal unless cancelled."""
        motion = joint_waypoint_motion(self.franky, path)
        try:
            self.robot.move(motion, asynchronous=True)
            while self.robot.poll_motion():
                if cancel.is_set():
                    self.robot.stop()
                    self.robot.join_motion()
                    return
                time.sleep(POLL_S)
            self.robot.join_motion()
        except self.faults as exc:
            raise RobotFault(str(exc)) from exc
        error = np.abs(self.joints() - path.q[-1])
        if float(error.max()) > GOAL_TOLERANCE_RAD:
            raise RobotFault(f"joint {int(error.argmax()) + 1} ended {float(error.max()):.3f} rad from its goal")

    def grip(self, width_m: float, cancel: threading.Event) -> None:
        """Change the opening; closing on something is not a fault, the measured width tells what happened."""
        command = grip_command(width_m, self.gripper_width_m(), self.max_width_m, speed_m_s=self.gripper_speed_m_s,
                               force_n=self.grasp_force_n)
        kind = command.pop("kind")
        try:
            future = getattr(self.gripper, f"{kind}_async")(**command)
            deadline = time.monotonic() + GRIPPER_DEADLINE_S
            while not future.wait(0.05):
                if cancel.is_set():
                    self.gripper.stop()
                    return
                if time.monotonic() > deadline:
                    self.gripper.stop()
                    raise RobotFault(f"the hand did not finish its {kind} in {GRIPPER_DEADLINE_S:.0f} s")
            finished = bool(future.get())
        except self.faults as exc:
            raise RobotFault(str(exc)) from exc
        if kind == "move" and not finished:
            raise RobotFault(f"the hand did not reach {width_m * 1000:.1f} mm")

    def recover(self) -> bool:
        return bool(self.robot.recover_from_errors())

    def stop(self) -> None:
        self.robot.stop()

