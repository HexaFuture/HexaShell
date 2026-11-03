# 6. Checking and recovering

Check every key step before the next one:
- Read the Move and Grip lines: `BLOCKED`, `short of target`, `closed fully` (for anything but
  thin things) and notes mean the step did not go as planned.
- After grasping, placing, pushing or pressing, look at the scene or measure again. A picture after
  the action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not g.holding: print('missed'); ...` before lifting, so a failure stops the plan.
  A goto that cannot be done stops the cell by itself (`NotExecuted`); `BLOCKED` and `short of
  target` do not, so test `.ok` where it matters.

When something fails:
- Name the cause from what you see (the gripper closed beside the object, the object slid out,
  the arm was blocked, the target was measured from one side) and change what caused it: the
  position, the angle, the height, the opening, the approach or the arm. Repeating the same
  numbers gives the same result.
- A dropped or knocked-over object is not lost: find it, measure it again and go on. Put back
  anything that was moved by accident if it matters for the task.

How strict the benchmark is - it judges the named objects' positions and poses:
- Positions are judged against the instruction's words: "on" (resting on it, centred on it), "in"
  (inside the walls, resting there), "next to", "left of", "behind" (a band of distance on that
  side, roughly in line) each mean a measurable arrangement.
  Centre the object on its target rather than just touching it, and look at both after letting go.
- Which way an object faces or stands can count too ("upright", "facing the robot", "head
  first"): turn it in the hand before setting it down, and check its pose in a picture afterwards.
- Heights count for things to be lifted or held up: well clear of the table, as the instruction
  says, and held still there.
- Every object the instruction names counts, and objects it does not name are best left where
  they are. Containers, stands and stacks must stay upright and in place.
- With several parts ("stack the three blocks", "put both ..."), every earlier part must still
  hold when the last one is done: do not knock the first object off while doing the second, and
  look at all of them at the end.
- Open the grippers fully at the end of a placement (section 1).

Budget:
- Replies are the scarce resource (the prompt says how many there are). A pick and place with
  checks fits in two or three replies when each reply's code does a whole phase; the step budget
  is rarely the limit.
- When replies are nearly gone, finish what is closest to done rather than starting something
  new.

Pictures:
- The conversation keeps the three start pictures and the most recent others (at most 20 in all);
  older ones are replaced by a note. Call `robot.look()` for a current picture rather than relying
  on an old one, and describe in your own words what you need to remember from a picture.
