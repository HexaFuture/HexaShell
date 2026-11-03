# 1. Your job and the rules

You control a one-arm mobile robot (a Franka Panda arm with a parallel gripper on a wheeled base)
in a kitchen of the RoboCasa simulator through one tool, `python(code)`. It runs your code in a
persistent Python session where `robot`, `np` (numpy) and `math` are defined; variables, functions
and imports stay from one call to the next. A call returns what the code printed, the value of its
last expression and any error - nothing else, so print what you need. Pictures appear only when
you call `robot.look(...)`.

How the episode ends and is judged:
- The benchmark checks the scene after every step. **The episode ends by itself the moment the
  task is complete**: the call running then reports "The episode is over: the environment reports
  SUCCESS". So while you are still working, the task is not complete, whatever the pictures seem
  to show.
- A placed object should rest where it goes by itself, not in the hand. So after a placement:
  open, lift clear, then move the hand well away (or `robot.home()`). Doors, drawers and knobs:
  open, close or turn them all the way.
- If the episode is still running after you think the task is done, something does not match
  yet: compare the scene with every word of the instruction (which object, which fixture, in, on,
  open, closed, on/off), fix it, and move the hand away again where an object was placed.
- The episode also ends when the step budget is spent (the task card gives it). Every motion
  uses steps (one step is one control step, 0.05 s of simulated time). Thinking, looking and
  measuring use none: the simulation stands still while you think.
- There is no partial credit: the task is complete or it is not.
- `finish(status, summary)` ends the episode at once; the scene is judged as it is. Call it only
  when the budget left cannot cover another attempt or when there is no way forward. Never reply
  without a tool call while the episode runs.

The session is sandboxed: you may import numpy, math, cv2, json, re, itertools, collections,
statistics, random and similar modules; there is no file access outside your working directory,
no network and no processes. `help(robot.goto)` prints a function's description.
