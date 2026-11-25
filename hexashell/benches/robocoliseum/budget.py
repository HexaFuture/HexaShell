"""The Season II dispatch budget, as the benchmark scores it (``plugins/output_system/policy_budget.py`` of
genie_sim 3.6.0, rule "policy-dispatch-budget").

A dispatch is one of our replies that carries actions (a chunk of rows); a hold row sent while the model
thinks is a dispatch too.  Each stage of a task has a step limit S (StepOut); its baseline is
``max(1, (S // 30) * 4 // 5)`` dispatches.  A stage that took n dispatches keeps the share
``max(0, min(b, 3b - 2n)) / b`` of its points: all of them up to b, nothing from 1.5 b on.  Episode score =
(sum of stage points x stage score x share) / 125 + 25 / 125 when every stage is met (E2E) and no stage went
over its baseline; the stage points of a task add up to 100.  The native runtimes (libuipc, Newton, MuJoCo)
apply one budget to the whole episode instead (``episode_policy_budget_breakdown``).
"""

from __future__ import annotations

PROCESS_POINTS = 100.0
COMPLETION_BONUS_POINTS = 25.0
TOTAL_POINTS = PROCESS_POINTS + COMPLETION_BONUS_POINTS
STEPS_PER_DISPATCH = 30


def baseline(step_out: int) -> int:
    """Dispatches a stage with this StepOut may take without losing points."""
    if type(step_out) is not int or step_out < STEPS_PER_DISPATCH:
        raise ValueError(f"StepOut must be an integer of at least {STEPS_PER_DISPATCH}, not {step_out!r}")
    return max(1, (step_out // STEPS_PER_DISPATCH) * 4 // 5)


def share(step_out: int, count: int) -> float:
    """The share of a stage's points kept after ``count`` dispatches."""
    b = baseline(step_out)
    if type(count) is not int or count < 0:
        raise ValueError(f"dispatch count must be a nonnegative integer, not {count!r}")
    return max(0, min(b, 3 * b - 2 * count)) / b


def last_free(step_out: int) -> int:
    """The most dispatches a stage may take and still count for the completion bonus."""
    return baseline(step_out)


def last_scoring(step_out: int) -> int:
    """The most dispatches after which a stage still keeps some of its points."""
    b = baseline(step_out)
    return (3 * b - 1) // 2


def episode_score(stages: list[dict]) -> float:
    """Score of an episode from its stages: dicts with ``points`` (of 100), ``score`` (the checker's 0..1),
    ``step_out`` and ``count`` (None when the stage never ran)."""
    process, complete = 0.0, True
    for s in stages:
        if s.get("count") is None:
            complete = False
            continue
        process += s["points"] * s["score"] * share(s["step_out"], s["count"])
        complete = complete and s["score"] >= 1.0 and s["count"] <= baseline(s["step_out"])
    return (process + (COMPLETION_BONUS_POINTS if complete and stages else 0.0)) / TOTAL_POINTS
