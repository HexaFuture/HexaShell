"""How late the readings are, and the arm ahead of them (LIBERO-Pro 2.0's server only).

A perturbation may delay what the robot reports (its readings and pictures then show a step some steps back)
or what it does (a command then takes effect some steps after it was sent).  Either way the readings at step k
show the arm as the commands up to step k - ``lag`` left it, ``lag`` 0 when nothing is late.  :class:`Timing`
follows that lag from the commands sent and the readings that come back alone, and from them predicts the arm
``lag`` steps ahead of its readings: where the commands already sent will leave it (a Smith predictor), the arm
the controller then steers (:mod:`.sim`, :mod:`.control`).

The prediction runs :class:`Replica`: the scene's nominal model under robosuite 1.4's OSC_POSE as the
environment runs it (each step's goal from the command and the hand's pose as the step starts, the goal
orientation kept by a command that turns by exactly nothing, each physics step's torques clipped to the motors'
limits, the controller's pull toward its nullspace pose), without contacts and with the fingers held as read.
Free of contacts it puts the grip site of a reading's next step within 0.3 mm of the reading (median 0.02 mm),
14 steps ahead within 1 mm (median 0.4 mm).

The lag changes in three ways, each told by the readings alone (see :meth:`Timing.update`):

* the readings repeat an earlier step's exactly (they stopped, or went back): the lag grows by the steps between;
* the readings jump ahead to where the commands since the last reading take the arm: it shrinks by those steps;
* the arm answers a command from further back (or less far back) than the lag says: a search over lags 0 to
  LAG_MAX finds the one that explains the readings, taken only when it explains them well, the lag before does
  not, and the steps that tell them apart (the two lags' predictions differ) side with it (EVIDENCE).  A touch,
  a load or a perturbed arm put the readings off every lag's prediction: no lag explains them, and none is taken.

With lag 0 the prediction is the readings themselves: then nothing is predicted, only checked a step ahead.
"""

from __future__ import annotations

import copy

import numpy as np

from ..arm import DT

#: the longest lag searched for (steps); readings that keep repeating may take it further, to LAG_CAP
LAG_MAX = 30
LAG_CAP = 60
#: how far apart two arm states are (:func:`gap`): the grip site's displacement (m) plus its turn (rad) times
#: TURN_M, and DT times the same of their velocities
TURN_M = 0.1
#: a reading further than CHECK_M from the lag's prediction calls for a look at the lag
CHECK_M = 5e-4
#: a lag's prediction matches a reading within MATCH_M; two predictions differing by INFO_M are told apart by it
MATCH_M = 5e-4
INFO_M = 1e-3
#: a reading jumped ahead to a predicted state within JUMP_M, and within JUMP_SHARE of the check's error
JUMP_M = 2e-3
JUMP_SHARE = 0.25
#: the states ahead are run again from the reading when it is further than ANCHOR_M from their first, or after
#: ANCHOR_STEPS steps; else they go on from the step before's, one command further
ANCHOR_M = 5e-4
ANCHOR_STEPS = 10
#: the search runs a lag's command only if it differs from every one run that step by SAME_M or more (the OSC's
#: deltas: m, and rad times TURN_M); else it takes that one's prediction
SAME_M = 1e-4
#: the search: over the last WINDOW steps, the best lag explains them within GOOD_M (root mean square), at most
#: 1/9 of the lag before's squared error (plus MATCH_M a step), 1/4 of every other lag's (plus MARGIN_M a step),
#: and at least EVIDENCE of them tell it from the lag before and match it; it ends when the lag before explains
#: QUIET steps in a row, or after SEARCH_STEPS, and none starts for COOL_STEPS after one that found nothing
WINDOW = 10
GOOD_M = 5e-4
MARGIN_M = 2e-4
EVIDENCE = 3
QUIET = 5
SEARCH_STEPS = 40
COOL_STEPS = 20


def gap(a, b, jac: np.ndarray) -> float:
    """How far apart two arm states (joint positions, joint velocities) are, to first order about the reading
    whose grip-site Jacobian is ``jac`` (6 x joints, linear rows first): see TURN_M."""
    p = jac @ (np.asarray(a[0], float) - np.asarray(b[0], float))
    v = jac @ (np.asarray(a[1], float) - np.asarray(b[1], float))
    return float(np.linalg.norm(p[:3]) + TURN_M * np.linalg.norm(p[3:])
                 + DT * (np.linalg.norm(v[:3]) + TURN_M * np.linalg.norm(v[3:])))


def _key(state) -> bytes:
    return np.asarray(state[0], np.float64).tobytes() + np.asarray(state[1], np.float64).tobytes()


def _inverse(a: np.ndarray) -> np.ndarray:
    """The OSC's pseudo-inverse, as the inverse where that is finite (it is but at a singular pose)."""
    try:
        b = np.linalg.inv(a)
        if np.isfinite(b).all():
            return b
    except np.linalg.LinAlgError:
        pass
    return np.linalg.pinv(a)


class Replica:
    """robosuite 1.4's OSC_POSE moving the scene's nominal arm, contacts off (see the module's docstring).

    ``model``: the nominal model (copied); ``rest``: the qpos of everything but the arm (and the fingers, set as
    ``fingers`` says); ``controller``: the environment's arm controller (gains, limits, nullspace pose);
    ``torque_limits``: the robot's (low, high); ``motors``: the arm's actuators in joint order."""

    def __init__(self, model, rest, controller, torque_limits, site_id: int, arm_qpos, arm_dof, finger_qpos,
                 finger_dof, motors, substeps: int):
        import mujoco

        c = controller
        if c.name != "OSC_POSE" or not c.use_delta or not c.use_ori or c.impedance_mode != "fixed" \
                or not c.uncoupling or c.interpolator_pos is not None or c.interpolator_ori is not None \
                or np.array(c.position_limits).any() or np.array(c.orientation_limits).any():
            raise RuntimeError("the arm's controller is not the OSC_POSE the replica copies")
        self.model = copy.deepcopy(model)
        if int(self.model.opt.integrator) == int(mujoco.mjtIntegrator.mjINT_RK4):
            raise RuntimeError("the replica steps in two halves: not with the RK4 integrator")
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data = mujoco.MjData(self.model)
        self.rest = np.array(rest, float)
        self.site = int(site_id)
        self.qpos, self.dof = np.array(arm_qpos, int), np.array(arm_dof, int)
        self.finger_qpos, self.finger_dof = np.array(finger_qpos, int), np.array(finger_dof, int)
        self.fingers = self.rest[self.finger_qpos].copy()
        self.motors = np.array(motors, int)
        self.substeps = int(substeps)
        self.kp, self.kd = np.asarray(c.kp, float), np.asarray(c.kd, float)
        self.in_min, self.in_max = np.asarray(c.input_min, float), np.asarray(c.input_max, float)
        out_min, out_max = np.asarray(c.output_min, float), np.asarray(c.output_max, float)
        self.scale = np.abs(out_max - out_min) / np.abs(self.in_max - self.in_min)
        self.out_mid, self.in_mid = (out_max + out_min) / 2.0, (self.in_max + self.in_min) / 2.0
        self.home = np.asarray(c.initial_joint, float)
        self.low, self.high = (np.asarray(v, float) for v in torque_limits)
        nv = self.model.nv
        self._jp, self._jr, self._m = np.zeros((3, nv)), np.zeros((3, nv)), np.zeros((nv, nv))
        self._ix, self._eye = np.ix_(self.dof, self.dof), np.eye(len(self.dof))
        self.steps = 0  # control steps run, for the cost

    def scaled(self, command) -> np.ndarray:
        """The OSC's delta of a command (its scale_action)."""
        return (np.clip(np.asarray(command, float), self.in_min, self.in_max) - self.in_mid) * self.scale \
            + self.out_mid

    def run(self, state, commands) -> list:
        """The arm states (joint positions, velocities) after each of ``commands`` (the arm's six numbers as
        sent), from ``state``."""
        import mujoco
        import robosuite.utils.transform_utils as T

        m, d = self.model, self.data
        d.qpos[:] = self.rest
        d.qvel[:] = 0.0
        d.qacc_warmstart[:] = 0.0
        d.qpos[self.qpos] = state[0]
        d.qvel[self.dof] = state[1]
        d.ctrl[:] = 0.0
        dof, site, jp, jr, full, ix, eye = self.dof, self.site, self._jp, self._jr, self._m, self._ix, self._eye
        kp_j = 10.0
        kv_j = 2.0 * np.sqrt(kp_j)
        out, goal_r = [], None
        for command in commands:
            delta = self.scaled(command)
            for sub in range(self.substeps):
                d.qpos[self.finger_qpos] = self.fingers
                d.qvel[self.finger_dof] = 0.0
                mujoco.mj_step1(m, d)
                ee, rot = d.site_xpos[site], d.site_xmat[site].reshape(3, 3)
                if sub == 0:
                    goal_p = ee + delta[:3]
                    if np.any(delta[3:] != 0.0):
                        goal_r = T.quat2mat(T.axisangle2quat(delta[3:])) @ rot
                    elif goal_r is None:  # kept from before the replica's first step: taken as the hand's
                        goal_r = rot.copy()
                mujoco.mj_jacSite(m, d, jp, jr, site)
                jac = np.concatenate([jp[:, dof], jr[:, dof]])
                qd = d.qvel[dof]
                mujoco.mj_fullM(m, full, d.qM)
                mass = full[ix]
                # robosuite's orientation_error: half the sum of the columns' cross products, (D C^T - C D^T) / 2
                s = goal_r @ rot.T - rot @ goal_r.T
                error = np.array([goal_p[0] - ee[0], goal_p[1] - ee[1], goal_p[2] - ee[2],
                                  0.5 * s[2, 1], 0.5 * s[0, 2], 0.5 * s[1, 0]])
                wrench = self.kp * error - self.kd * (jac @ qd)
                inv = np.linalg.inv(mass)
                lam_inv = jac @ inv @ jac.T
                lam_full = _inverse(lam_inv)
                lam_p, lam_r = _inverse(lam_inv[:3, :3]), _inverse(lam_inv[3:, 3:])
                null = eye - inv @ jac.T @ lam_full @ jac
                tau = jac.T @ np.concatenate([lam_p @ wrench[:3], lam_r @ wrench[3:]]) + d.qfrc_bias[dof]
                tau = tau + null.T @ (mass @ (kp_j * (self.home - d.qpos[self.qpos]) - kv_j * qd))
                d.ctrl[self.motors] = np.clip(tau, self.low, self.high)
                mujoco.mj_step2(m, d)
            self.steps += 1
            out.append((d.qpos[self.qpos].copy(), d.qvel[dof].copy()))
        return out


class Timing:
    """The lag of the readings behind the commands, and the arm ahead of the readings (see the module).

    ``run(state, commands)``: the arm states after each command from a state (:meth:`Replica.run`); ``q``, ``qd``:
    the first readings (the arm's joint positions and velocities), before any command; ``scaled``: the OSC's delta
    of a command (:meth:`Replica.scaled`).  Each step,
    :meth:`update` takes the command sent and the readings that came back; :meth:`predicted` is the arm then.
    Commands before the first count as nothing sent (zeros)."""

    def __init__(self, run, q, qd, zero=None, scaled=None):
        self.run = run
        #: the OSC's delta of a command (:meth:`Replica.scaled`): commands whose deltas are alike share a prediction
        self.scaled = scaled or (lambda c: np.array(c, float))
        self.lag = 0
        self.zero = np.zeros(6) if zero is None else np.asarray(zero, float)
        #: the commands sent (step k's at k - 1), the readings (step k's at k), the lag each reading had
        self.commands: list = []
        self.readings: list = [(np.array(q, float), np.array(qd, float))]
        self.lags: list = [0]
        self.seen: dict = {_key(self.readings[0]): 0}
        #: the states after each command since the latest reading took effect, from it: the last is the prediction
        self.ahead: list = []
        self._ahead_from = (None, 0)  # (the reading's key, the step of the first command) the states start from
        self._anchored = 0  # the step the states ahead were last run from a reading
        self.search: dict | None = None
        self.cool = 0
        #: (step, lag before, lag after, how: "repeat", "ahead" or "search") of each change
        self.changes: list = []
        #: each step's error of the lag's prediction a step ahead (m, see :func:`gap`)
        self.checks: list = []

    @property
    def step(self) -> int:
        return len(self.commands)

    def command(self, j: int) -> np.ndarray:
        return self.commands[j - 1] if j >= 1 else self.zero

    def predicted(self):
        """(joint positions, joint velocities) of the arm once the commands sent have taken effect."""
        return self.ahead[-1] if self.ahead else self.readings[-1]

    def update(self, command, q, qd, jac: np.ndarray) -> bool:
        """Step k: ``command`` was sent, the readings ``q``, ``qd`` came back (``jac``: the grip site's Jacobian at
        them, see :func:`gap`).  Returns whether the lag changed."""
        self.commands.append(np.array(command, float).reshape(-1)[:6])
        k = self.step
        last, now = self.readings[-1], (np.array(q, float), np.array(qd, float))
        lag = self.lag
        # the lag's prediction of the reading a step on from the reading before (the states ahead's first, when
        # they were run from it), and whether the states ahead, if they went on from older ones, still hold
        expect = self.ahead[0] if lag and self._anchored == k - 1 else self.run(last, [self.command(k - lag)])[0]
        check = gap(now, expect, jac)
        near = bool(lag) and len(self.ahead) == lag and gap(now, self.ahead[0], jac) <= ANCHOR_M
        self.checks.append(check)
        new, how = lag, None
        if check > CHECK_M:
            j = self.seen.get(_key(now))
            if j is not None:
                new, how = min(LAG_CAP, self.lags[j] + k - j), "repeat"
            elif lag:
                options = self.ahead[1:] + self.run(self.ahead[-1], [self.commands[-1]])
                errors = [gap(now, s, jac) for s in options]
                i = int(np.argmin(errors))
                if errors[i] < JUMP_M and errors[i] < JUMP_SHARE * check:
                    new, how = lag - i - 1, "ahead"  # options[i]: the state i + 2 commands on
            if how is None and self.search is None and k >= self.cool:
                self.search = {"rows": [], "steps": 0, "quiet": 0}
        if how is not None:
            self.search = None
        elif self.search is not None:
            found = self._search(k, last, now, jac, lag, expect, check)
            if found is not None:
                new, how = found, "search"
        self.readings.append(now)
        self.lags.append(new)
        self.seen[_key(now)] = k
        if how is not None and new != lag:
            self.changes.append((k, lag, new, how))
        self.lag = new
        self._roll(k, now, new == lag and near)
        return new != lag

    def _roll(self, k: int, now, near: bool) -> None:
        """The states after the commands not yet in the readings (k - lag + 1 .. k), from the reading ``now``;
        those of the step before kept where they start from the same reading (the readings repeat), or, ``near``
        (the lag the same and the reading within ANCHOR_M of their first), taken on a step, the states then
        going on from older readings (for ANCHOR_STEPS at most)."""
        first = k - self.lag + 1
        key = _key(now)
        if self.lag == 0:
            self.ahead, self._ahead_from, self._anchored = [], (key, first), k
            return
        if self._ahead_from == (key, first):
            kept = self.ahead[:self.lag]
        elif near and k - self._anchored < ANCHOR_STEPS:
            kept = self.ahead[1:]
        else:
            kept, self._anchored = [], k
        start = kept[-1] if kept else now
        rest = [self.command(j) for j in range(first + len(kept), k + 1)]
        self.ahead = kept + (self.run(start, rest) if rest else [])
        self._ahead_from = (key, first)

    def _search(self, k: int, last, now, jac, lag: int, expect, check: float):
        """A step of the search over lags (see the module): the lag found, or None.  ``expect``: the lag's
        prediction of the reading ``now`` (``check`` off it), from the reading before, ``last``."""
        s = self.search
        preds, done = [], []  # done: (the OSC's delta, rotation times TURN_M, of a command run, its prediction)
        for h in range(LAG_MAX + 1):
            c = self.command(k - h)
            delta = self.scaled(c)
            delta[3:] *= TURN_M
            for other, pred in done:
                if float(np.abs(delta - other).max()) < SAME_M:
                    break
            else:
                pred = self.run(last, [c])[0]
                done.append((delta, pred))
            preds.append(pred)
        errors = np.array([gap(now, p, jac) for p in preds])
        apart = np.array([gap(p, expect, jac) for p in preds])
        s["rows"] = (s["rows"] + [(errors, apart, check)])[-WINDOW:]
        s["steps"] += 1
        s["quiet"] = s["quiet"] + 1 if check <= MATCH_M else 0
        rows = s["rows"]
        n = len(rows)
        sse = sum(e ** 2 for e, _, _ in rows)
        lag_sse = sum(c ** 2 for _, _, c in rows)
        evidence = sum((e < MATCH_M) & (a > INFO_M) for e, a, _ in rows)
        best = int(np.argmin(sse))
        others = np.delete(sse, best)
        if best != lag and evidence[best] >= EVIDENCE and sse[best] <= n * GOOD_M ** 2 \
                and lag_sse >= 9.0 * sse[best] + n * MATCH_M ** 2 \
                and float(others.min()) >= 4.0 * sse[best] + n * MARGIN_M ** 2:
            self.search = None
            return best
        if s["quiet"] >= QUIET:
            self.search = None
        elif s["steps"] >= SEARCH_STEPS:
            self.search, self.cool = None, k + COOL_STEPS
        return None
