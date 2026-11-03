"""A robot call's error reaches the cell as the class the prompt names (Blocked, Dropped, NotExecuted).

Run: ``python tests/test_cell_errors.py`` in the policy environment.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hexashell.kernel.host import CallFailed, Kernel  # noqa: E402

KINDS = ["Blocked", "Dropped", "NotExecuted", "Blocked"]


def test_robot_errors_reach_the_cell():
    calls = []

    def dispatch(name, args, kwargs):
        calls.append(name)
        kind = KINDS[len(calls) - 1]
        raise CallFailed(kind, f"Move left: at (0.100, 0.000, 0.800) ({kind.lower()})")

    work = Path(tempfile.mkdtemp(prefix="hexashell-errors-")) / "workspace"
    kernel = Kernel(work, dispatch, cpu_s=20, idle_s=30)
    kernel.start()
    try:
        r = kernel.run(
            "for cls in (Blocked, Dropped, NotExecuted):\n"
            "    try:\n"
            "        robot.goto('left', (0.1, 0.0, 0.8))\n"
            "    except cls as e:\n"
            "        print('caught', type(e).__name__, issubclass(cls, RobotError), e)\n")
        assert r.error is None, r.error
        assert r.output.count("caught") == 3 and "False" not in r.output, r.output
        assert "caught Blocked True Move left: at (0.100, 0.000, 0.800) (blocked)" in r.output, r.output
        assert "caught Dropped True" in r.output and "caught NotExecuted True" in r.output, r.output
        # not caught: the rest of the cell does not run, and the error names the class
        r = kernel.run("robot.goto('left', (0.1, 0.0, 0.8))\nprint('went on')")
        assert "went on" not in r.output, r.output
        assert r.error.splitlines()[-1].startswith("Blocked: Move left: at"), r.error
    finally:
        kernel.close()


if __name__ == "__main__":
    test_robot_errors_reach_the_cell()
    print("ok")
