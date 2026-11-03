"""XPolicyLab entry point of the HexaShell policy (runs inside the policy server)."""

from __future__ import annotations

import sys
from pathlib import Path

# this directory is a symlink into the HexaShell checkout: make its package importable
_HEXASHELL_ROOT = Path(__file__).resolve().parents[2]
if str(_HEXASHELL_ROOT) not in sys.path:
    sys.path.insert(0, str(_HEXASHELL_ROOT))

from XPolicyLab.model_template import ModelTemplate  # noqa: E402

from hexashell.xpl.adapter import PolicyHost  # noqa: E402


class Model(ModelTemplate):
    def __init__(self, deploy_cfg):
        super().__init__()
        self.host = PolicyHost(dict(deploy_cfg))
        self.model = self.host

    def reset(self):
        return self.host.reset()

    def update_obs(self, obs):
        return self.host.update_obs(obs)

    def get_action(self):
        return self.host.get_action()

    def on_trial_end(self, result=None):
        return self.host.on_trial_end(result)

    def update_obs_batch(self, obs_list):
        raise NotImplementedError("HexaShell drives one environment per policy server (eval_batch: false)")

    def get_action_batch(self, env_idx_list=None):
        raise NotImplementedError("HexaShell drives one environment per policy server (eval_batch: false)")
