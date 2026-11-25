# 6. Checking and recovering

Check every key step before the next one:
- Read the Move and Hand lines: `BLOCKED`, `Dropped`, `short of target`, `empty` after a close,
  `touching` and notes mean the step did not go as planned.
- After grasping, placing, pushing or pressing, look at the scene again. A picture after the action
  is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not h.holding: print('missed'); ...` before lifting, so a failure stops the plan
  (where the curled fingers may hold the object instead of a pinch's pads, or where the result says
  touching or holding cannot be told, look rather than test).
  Fewer, well-aimed motions leave more of the span than many small ones.
  `NotExecuted`, `Blocked` and `Dropped` stop the cell by themselves; `short of target` does not,
  so test `.ok` where it matters.

When something fails:
- Name the cause from what you see (the hand closed beside the object, the curled fingers met it
  first, the object slid out, the arm was blocked, the target's point was worked out from one side) and
  change what caused it: the position, the angle, the height, the opening, the approach or the
  arm. Repeating the same numbers gives the same result.
- A failed try costs steps of the episode's span, which is short: make the next try the fix, not a
  repeat.
- A dropped or knocked-over object on the table is not lost: find it, work out its point again and
  go on (one lying on its side or out of its support often needs another grasp than before). An
  object that falls to the floor is out of reach.

What the instruction's words mean - the benchmark looks at the named objects and the hands:
- A hand "at" an object: the hand close around the object, not merely pointing at it.
- Lifted or held up: the object clearly higher than it was, and still in the hand that holds it.
- Where an object ends up: "in" means its middle inside the container's space; "on" means resting
  on top of it; a place on a shelf or in a part of a box means within that place's bounds. "Left
  of" and "right of" another object (the robot's own left and right) mean clearly to that side and
  about level with it, at about the same distance from the robot; "behind" means farther from the
  robot, "in front of" nearer. Centre the object on its target rather than just touching it, keep
  it there, and look at both after letting go.
- How it stands: "upright" means standing up, not tilted far; where the instruction says which way
  it faces or stands, that counts too.
- Opening or closing a door, a drawer or a lid: judged by how far its hinge or slide stands, which
  has to be within the range the instruction means - clearly open, or fully shut - and not forced
  further than that.
- Stacks and sorted sets: the objects on each other, or in their places in order along a row, all
  at the same time.
- Every object the instruction names counts; objects it does not name are best left where they
  are. Containers, stands and stacks must stay upright and in place.

Budget:
- Two things run out: replies (the task says how many there are) and the episode's span of steps
  (section 1; every motion's result says how many are left). A pick and place with checks fits in
  two or three replies when each reply's code does a whole phase with checks between the motions.
- Each motion uses steps of the span; waits use the steps you ask. Thinking, looking and working
  out points use none. Hold a result still only where nothing else holds it, and not long.
- When replies or steps are nearly gone, finish what is closest to done rather than starting
  something new.

Pictures:
- The conversation keeps the three start pictures and the most recent others (up to about 90 in
  all); older ones are replaced by a note. One reply shows at most 8 pictures (the earlier ones of
  that reply are left out). Call `robot.look()` for the latest picture rather than relying on an old
  one, and describe in your own words what you need to remember from a picture.
