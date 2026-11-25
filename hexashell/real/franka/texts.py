"""What the model is told on the FR3 besides the system prompt and the docstrings: URAI's sentences, except those
that name its backend."""

from ..urai.texts import BUDGET, CARD, FINISH, FINISH_STATUS, FINISHED, NUDGE, OVER, SWING_REFUSED, SWING_USED  # noqa: F401

TIMING = ("The FR3 service runs on wall-clock time and does not pause while you think. Action steps come from the "
          "actual execution time, 0.04 s per step.")
CONTACT = "contact=True adds no stall check; the robot's own collision reflexes still apply"
CANCELLED = "The execution was cancelled"
