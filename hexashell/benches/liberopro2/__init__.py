"""LIBERO-Pro 2.0 with LIBERO's Panda arm, through the benchmark's own environment and its perturbations.

The simulator runs in a server process of the LIBERO-Pro 2.0 environment (Python 3.8, robosuite 1.4:
:mod:`.server`); the episode side is :mod:`.session` and :mod:`.host`; cells see :mod:`.proxy`.  The
test protocol is in :mod:`.protocol`.  Keep this file free of imports (the server loads modules from here).
"""
