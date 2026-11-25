"""GR00T's decoupled whole-body controller (SIMPLE's teleoperation tasks) on simulated time.

SIMPLE's teleoperation agents (simple/agents/sonic_decoupled_wbc_agent.py, and the model agent
simple/baselines/psi0_decoupled_wbc.py) build the controller and give it one goal per 0.02 s step, stamped
with the wall clock (time.monotonic()):

- the upper body's 31 joint targets (waist yaw, roll, pitch; per side the arm's 7 joints and the Dex3 hand's
  7), the base height command and the walking command pass through an interpolator.  A goal given at time t
  has target time t + 0.02 s; the controller is read at t; waypoints older than t - 0.04 s are dropped.  Its
  speed limit (the WBC config's upper_body_max_joint_speed, 1000 rad/s) never binds.  So the reading depends
  on how far apart the steps are in time: 0.02 s apart, it is exactly the previous step's goal; more than
  0.06 s apart (the official evaluation with Isaac Sim makes about 1.6 steps a second), it is two thirds of
  the way from the previous goal to this one; in between, something in between.
- the lower body (an RL policy: two ONNX networks, standing and walking) counts steps: its gait phase moves
  0.02 x its frequency per observation, and it never reads the time.  The waist targets reach it as a torso
  orientation command; the walking command is (vx, vy, turn flag, target heading), where the heading is
  absolute (the pelvis's yaw in the world) and the turn rate is the heading error / 0.5 s, at most 1 rad/s,
  while |flag| >= 0.1.
- a goal older than 1 s (wall clock) is replaced by a stop.
- the stand before the policy (the evaluation's stabilisation loop) sleeps to keep its steps 0.02 s apart.

Here every time is simulated: the k-th goal since the controller's reset is stamped k x 0.02 s (the stand's
goals included), and its age is measured on the same clock, so it is never stale.  Nothing waits.  The
reading is therefore always the previous step's goal, whatever the speed of the process, which is the
controller as designed (real-time 50 Hz teleoperation).  Against the official evaluation's slow steps
(whose reading lags the step's goal by a third of its change) that is two thirds of a step of extra lag:
each step's reading is 2/3 of that step's goal change behind theirs
(``scripts/simple_executor/check_wbc_clock.py`` measures what the clock does to the robot).
"""

from __future__ import annotations

import numpy as np

#: SIMPLE's DEFAULT_NAV_CMD (decoupled_wbc.control.main.constants): stand still, heading 0
STAND_NAVIGATE = (0.0, 0.0, 0.0, 0.0)
#: SIMPLE's DEFAULT_BASE_HEIGHT (m)
DEFAULT_BASE_HEIGHT = 0.74
#: the official stabilisation loop: at most this many steps, until the robot reports it stands still
STABILIZE_MAX_STEPS = 300
#: window of the first stabilisation goal (s), SIMPLE's "2-second ramp" to the controller's default pose
FIRST_GOAL_WINDOW_S = 2.0


class SimClockWbc:
    """The controller SIMPLE's teleoperation agents build (same construction, same configuration), driven
    one goal per step on simulated time.  ``robot`` is the task's G1Sonic, ``sonic_config`` the environment's.

    A step's goal is ``upper`` (target per upper-body joint name of the WBC, ``upper_names``; set it by the
    joints driven with ``set_targets``, as the hands' names differ), ``navigate``
    (vx m/s, vy m/s, turn flag, target heading rad) and ``height`` (base height command, m); they hold until
    changed.  ``action()`` returns the step's command to the robot (SIMPLE's ActionCmd "decoupled_wbc": PD
    targets of the 29 body joints and the two hands) and moves the clock one step."""

    def __init__(self, robot, sonic_config: dict):
        from simple.agents.sonic_decoupled_wbc_agent import SonicDecoupledWbcAgent

        agent = SonicDecoupledWbcAgent(robot, sonic_config=sonic_config)
        self._agent = agent
        self.robot = robot
        self.policy = agent._wbc_policy
        self.model = agent._dwbc_robot_model
        self.dt = 1.0 / float(agent._control_frequency)
        indices = [int(i) for i in self.model.get_joint_group_indices("upper_body")]
        by_index = {int(i): n for n, i in self.model.joint_to_dof_index.items()}
        #: upper-body joint names in the order the controller takes its targets
        self.upper_names = [by_index[i] for i in indices]
        self.default_upper = dict(zip(self.upper_names,
                                      np.asarray(self.model.get_initial_upper_body_pose(), dtype=float)))
        self._by_index = by_index
        self._motor_kp = [float(v) for v in sonic_config["MOTOR_KP"]]
        #: MuJoCo joint -> name of the WBC target that drives it (identity outside the hands; set at reset)
        self.wbc_name: dict[str, str] = {}
        self._driven: dict[str, str] = {}
        #: PD stiffness (N m / rad) G1Sonic applies to each of the 29 body joints, by MuJoCo joint name
        #: (set at reset)
        self.body_kp: dict[str, float] = {}
        self.k = 0
        self._first = True
        self.upper = dict(self.default_upper)
        self.navigate = np.array(STAND_NAVIGATE, dtype=float)
        self.height = DEFAULT_BASE_HEIGHT

    @property
    def t(self) -> float:
        """Simulated time of the next goal (s since the reset)."""
        return self.k * self.dt

    def set_targets(self, targets: dict) -> None:
        """Upper-body targets (rad) named by the MuJoCo joint they are for (waist, arms, fingers); the
        others keep theirs."""
        if not self.wbc_name:
            raise RuntimeError("the hands' joint names are known after reset()")
        for j, v in targets.items():
            self.upper[self.wbc_name.get(j, j)] = float(v)

    def targets(self) -> dict:
        """The upper body's current targets, named by the MuJoCo joint each drives."""
        return {self._driven.get(n, n): v for n, v in self.upper.items()}

    def reset(self) -> None:
        """A new episode, as SIMPLE's agent reset before the stand: the interpolator starts again at time 0,
        the lower body's state is cleared, and the goal is the controller's default."""
        self.k = 0
        self._first = True
        self.policy.reset(init_time=0.0)
        self.upper = dict(self.default_upper)
        self.navigate = np.array(STAND_NAVIGATE, dtype=float)
        self.height = DEFAULT_BASE_HEIGHT
        self._joint_names()

    def _joint_names(self) -> None:
        """Which MuJoCo joint each of the WBC's targets drives.  G1Sonic applies the WBC's 29 body targets
        and its PD gains (MOTOR_KP) to the body joints in its MJCF's order, which is the WBC's order (checked
        here).  The WBC gives each hand's 7 targets in the order (thumb_0..2, index_0..1, middle_0..1) and
        G1Sonic applies them in its MJCF's order (thumb_0..2, middle_0..1, index_0..1), so the WBC's
        "index" targets drive the middle finger and its "middle" targets the index finger
        (scripts/simple_executor/check_wbc_fingers.py).  SIMPLE's observation of the hands goes through the
        same two orders, so SIMPLE is consistent with itself; here targets are named by the joint they
        drive.  (The robot knows its joints once the environment is reset.)"""
        robot, by_index = self.robot, self._by_index
        everything = np.arange(len(by_index))
        body = [robot.mjModel.joint(int(i)).name for i in robot.body_joint_index]
        wbc_body = [by_index[int(i)] for i in self.model.get_body_actuated_joints(everything)]
        if body != wbc_body:
            raise RuntimeError(f"the WBC's body joint order {wbc_body} is not G1Sonic's {body}")
        self.body_kp = dict(zip(body, self._motor_kp))
        names: dict[str, str] = {}
        for side in ("left", "right"):
            wbc_order = [by_index[int(i)] for i in self.model.get_hand_actuated_joints(everything, side=side)]
            driven = [robot.mjModel.joint(int(i)).name for i in getattr(robot, f"{side}_hand_index")]
            names.update(zip(driven, wbc_order))
        self.wbc_name = names
        self._driven = {v: k for k, v in names.items()}

    def engage(self) -> None:
        """Before the stand: the official evaluation engages the lower body's RL policy right away."""
        self.policy.lower_body_policy.use_policy_action = True

    def start_policy(self) -> None:
        """After the stand, before the first step of the episode: the official evaluation restarts the gait
        phase."""
        import torch

        self.policy.lower_body_policy.gait_indices = torch.zeros((1), dtype=torch.float32)

    def stabilize_action(self):
        """SIMPLE's get_stabilize_action on simulated time: the controller's default upper-body pose, stand
        still, default height; the first goal after a reset has a 2 s window."""
        window = FIRST_GOAL_WINDOW_S if self._first else self.dt
        upper = np.array([self.default_upper[n] for n in self.upper_names], dtype=float)
        return self._act(upper, np.asarray(STAND_NAVIGATE, dtype=float),
                         np.atleast_1d(np.asarray(DEFAULT_BASE_HEIGHT)), window)

    def action(self):
        """This step's command for the current goal (what Psi0DecoupledWbcAgent.get_action does with a
        model's action, on simulated time)."""
        upper = np.array([self.upper[n] for n in self.upper_names], dtype=float)
        return self._act(upper, np.array(self.navigate, dtype=float), np.atleast_1d(float(self.height)), self.dt)

    def _act(self, upper: np.ndarray, navigate: np.ndarray, height: np.ndarray, window: float):
        from simple.core.action import ActionCmd

        t = self.t
        self.policy.set_observation(self._agent._build_wbc_observation(self.robot.prepare_obs()))
        self.policy.set_goal({
            "target_upper_body_pose": upper,
            "navigate_cmd": navigate,
            "base_height_command": height,
            "target_time": t + window,
            "interpolation_garbage_collection_time": t - 2 * self.dt,
            "timestamp": t,
        })
        # set_goal stamps the goal with the wall clock for its 1 s staleness check; the check compares
        # with the time the controller is read at, which is simulated here
        self.policy.last_goal_time = t
        q = self.policy.get_action(time=t)["q"]
        self._first = False
        self.k += 1
        m = self.model
        return ActionCmd("decoupled_wbc", target_q=m.get_body_actuated_joints(q),
                         left_hand_q=m.get_hand_actuated_joints(q, side="left"),
                         right_hand_q=m.get_hand_actuated_joints(q, side="right"))
