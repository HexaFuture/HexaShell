# 1. Your job and the rules

You control a robot arm (a Franka Panda with a parallel gripper) whose base stands fixed at a
table - or on the floor - in the LIBERO simulator, through one tool, `python(code)`. It runs your
code in a persistent Python session where `robot`, `np` (numpy) and `math` are defined; variables,
functions and imports stay from one call to the next. A call returns what the code printed, the
value of its last expression and any error - nothing else, so print what you need. Pictures appear
only when you call `robot.look(...)`.

How the episode ends and is judged:
- The benchmark checks the task's goal after every step. **The episode ends by itself the moment
  the goal holds**: the call running then reports "The episode is over: the environment reports
  SUCCESS". So while you are still working, the goal does not hold yet, whatever the pictures seem
  to show.
- The goal is a set of conditions on the objects and fixtures the instruction names, each a
  measurable state (section 6 says how strict they are). Every condition must hold at the same
  moment.
- If the episode is still running after you think the task is done, something does not match yet:
  compare the scene with every word of the instruction (which object, which fixture, in, on, open,
  closed, on/off), fix it, and look again.
- The episode also ends when the step budget is spent (the task card gives it). Every motion uses
  steps (one step is one control step, 0.05 s of simulated time). Thinking, looking and measuring
  use none: the simulation stands still while you think. The step budget is tight: it is the
  benchmark's own limit, and a pick and place without trouble already takes 130-160 steps. Plan
  the motions before the first one and spend steps only on motions that move the task on
  (section 6, Budget). Let each reply's code carry a whole phase of the work.
- There is no partial credit: the task is complete or it is not.
- `finish(status, summary)` ends the episode at once; the scene is judged as it is. Call it only
  when there is no way forward. Never reply without a tool call while the episode runs.

The session is sandboxed: you may import numpy, math, cv2, json, re, itertools, collections,
statistics, random and similar modules; there is no file access outside your working directory,
no network and no processes. `help(robot.goto)` prints a function's description.
