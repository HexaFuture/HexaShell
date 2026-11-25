"""HexaShell on a Franka Research 3 with a Franka Hand.

Two processes, as with URAI.  The robot's machine runs the service (:mod:`.server`), the one owner of the
FCI connection and the cameras: it plans with the FR3's own kinematics (:mod:`.arm`, :mod:`.planner`), runs
the motions through franky (:mod:`.driver`) and answers URAI's harness HTTP interface.  Anywhere else,
``python -m hexashell.real.franka --url ... --task ...`` runs an episode against it with
:mod:`hexashell.real.urai`'s client and host, a one-arm ``robot`` and its own prompts (``prompts/franka``).

    python -m hexashell.real.franka.server --sim --port 7861                  # kinematic simulator
    python -m hexashell.real.franka.server --rig rig.json --port 7861         # read-only: plans, never moves
    python -m hexashell.real.franka.server --rig rig.json --allow-motion      # the robot moves

The service binds to loopback unless told otherwise; another address needs a token.  The FCI takes one
controlling client, so nothing else may hold it while the service runs.
"""
