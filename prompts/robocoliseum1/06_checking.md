# 6. Checking and recovering

Check every key step before the next one:
- Read the Move and Grip lines: `BLOCKED`, `Dropped`, `short of target`, `closed fully` (for
  anything but thin things) and notes mean the step did not go as planned.
- After grasping, placing, pushing or pressing, look at the scene or measure again. A picture after
  the action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not g.holding: print('missed'); ...` before lifting, so a failure stops the plan.
  `NotExecuted`, `Blocked` and `Dropped` stop the cell by themselves; `short of target` does not,
  so test `.ok` where it matters.

When something fails:
- Name the cause from what you see (the gripper closed beside the object, the fingers met its
  corners, the object slid out, the arm was blocked, the target was measured from one side) and
  change what caused it: the position, the angle, the height, the opening, the approach or the
  arm. Repeating the same numbers gives the same result.
- Each stage has its own step allowance, so a failed try costs the stage's steps: make the next
  try the fix, not a repeat.
- A dropped or knocked-over object on the table is not lost: find it, measure it again and go on.
  An object that falls to the floor or off the work area ends the episode.

How the benchmark judges a stage - it looks at the named objects and the hands:
- A hand "at" an object: the point between the fingers close around the object, not merely
  pointing at it.
- Lifted or held up: the object higher than it was at the look that first saw the hand at it, and
  still near the hand that holds it.
- Where an object ends up: "in" means its middle inside the container's space; "on" means resting
  on top of it; a place on a shelf or in a part of a box means within that place's bounds. "Left
  of" and "right of" another object (the robot's own left and right) mean clearly to that side and
  about level with it, at about the same distance from the robot; "behind" means farther from the
  robot, "in front of" nearer. Centre the object on its target rather than just touching it, keep
  it there over more than one look, and look at both after letting go.
- How it stands: "upright" means standing up, not tilted far; where the instruction says which way
  it faces or stands, that counts too.
- Opening or closing a door, a drawer or a lid: judged by how far its hinge or slide stands, which
  has to be within the range the instruction means - clearly open, or fully shut - and not forced
  further than that.
- Stacks and sorted sets: the objects on each other, or in their places in order along a row, all
  at the same looks.
- Every object the instruction names counts; objects it does not name are best left where they
  are. Containers, stands and stacks must stay upright and in place.
- A stage once seen done stays done; the next one begins at that same look, and the stages are
  judged in order.

Budget:
- Replies are the scarcer resource (the task says how many there are). A pick and place with
  checks fits in two or three replies when each reply's code does a whole phase with checks
  between the motions.
- Steps count per stage (section 1). Thinking and measuring cost none; motions cost few; waits
  cost what you ask: hold long enough for the benchmark to look, but not much longer.
- When replies are nearly gone, finish what is closest to done rather than starting something new.

Pictures:
- The conversation keeps the three start pictures and the most recent others (up to about 90 in
  all); older ones are replaced by a note. One reply shows at most 8 pictures (the earlier ones of
  that reply are left out). Call `robot.look()` for a current picture rather than relying on an old
  one, and describe in your own words what you need to remember from a picture.
