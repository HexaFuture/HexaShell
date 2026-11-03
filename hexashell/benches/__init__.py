"""Benchmarks besides RoboDojo: LIBERO-PRO, RoboTwin 2.0, RoboCasa365.

Each runs its simulator in a server process of its own Python environment (their Python and
numpy versions differ from HexaShell's).  The episode side talks to it through :mod:`.wire`.

Keep this package's ``__init__`` files free of imports: the simulator servers import modules
from here with nothing but numpy and the simulator installed.
"""
