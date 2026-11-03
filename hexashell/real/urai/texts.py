"""What the model is told in a URAI episode besides the system prompt and the docstrings.

The task card's last line starts with the episode's sentence on replies and minutes and ends with
:data:`TIMING`.
"""

#: the finish tool: what it does, and what its statuses mean ("done" is the real robot's own)
FINISH = ("End the session and record your report on how far the task got; this is not an automatic evaluation. "
          "Write the summary in English.")
FINISH_STATUS = "done: you believe the task is complete; the other statuses give the reason for stopping."
#: the answer to finish
FINISHED = "The session has ended and your report is recorded."
#: after a reply without a tool call
NUDGE = ("Keep observing and acting with python; when the task is done or you decide to stop, report what you "
         "observed in English with finish.")
CARD = ("# Task card\n\n{instruction}\n\nRobot: {name}. World frame, TCP, bases, table height, maximum opening and "
        "cameras:\n{model}\n\n{task}\nThere is no automatic grader; when done, call finish and report truthfully what "
        "you observed. Write all observations and reports in English.\n")
OVER = "Session over: action budget {step}/{limit}; the task result has not been evaluated."
TIMING = ("The URAI backend runs on wall-clock time and does not pause while you think. Action steps come from the "
          "actual server-side execution time, 0.04 s per step.")
#: a robot call once the budget is spent
BUDGET = "The action budget is used up: {step}/{limit}"
#: goto(path="auto"): the joint-space path URAI planned after the straight line failed
SWING_REFUSED = ("The joint path leaves the straight line by {off:.1f} mm and dips {dip:.1f} mm below the ends; give "
                 'an intermediate point or an explicit path="swing"')
SWING_USED = "Using the joint path: {off:.1f} mm off the straight line, {dip:.1f} mm below the ends"
CONTACT = "contact=True adds no harness stall check; URAI's own collision and tracking checks still apply"
