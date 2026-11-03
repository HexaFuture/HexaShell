# 6. Checking and recovering

Check every key step before the next one:
- Read the Move and Grip lines: `BLOCKED`, `short of target`, `closed fully` (for anything but
  thin things) and notes mean the step did not go as planned.
- After grasping, placing or pushing, look at the scene or measure again. A picture after the
  action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not g.holding: print('missed'); ...` before lifting, so a failure stops the plan;
  where the part's width is known, compare `g.width_m` with it too (much narrower: an edge or rim).
  A goto that cannot be done stops the cell by itself (`NotExecuted`), and so does one that ran
  but stalled or ended BLOCKED more than 25 mm short (`Blocked`), or lost what it lifted or
  carried (`Dropped`), and a refused close (`Blocked`); a smaller `BLOCKED` and `short of target`
  do not, so test `.ok` where it matters.
- For "in" (a hole, cup, box or slot): check that the object is down inside, not lying across
  the opening or on its rim, before covering it, closing a lid or calling `finish`; when unsure,
  look from a second camera.

When something fails:
- Name the cause from what you see (the gripper closed beside the object, the object slid out,
  the arm was blocked, the target was measured from one view) and change what caused it: the
  position, the angle, the height, the opening, or the arm. Repeating the same numbers gives the
  same result: before a repeat, say what you will do differently and why.
- A dropped or knocked-over object is not lost: find it, measure it again and go on. When two or
  three different ways of picking it up again have failed, weigh what it still adds against the
  steps the rest of the task needs. Put back anything that was moved by accident if it matters
  for the task.

How strict the benchmark is:
- Positions are checked to about a centimetre or two, and angles to some degrees, against the
  instruction's words: "in", "on", "next to", "aligned", "in order" each mean a measurable
  arrangement. Every object the instruction names counts, and objects it does not name are best
  left where they are.
- The score table also shows whether the way of doing it is checked. Where its rows name only
  where things must end up, a way that is slow or keeps failing can give way to a surer one (such
  as carrying the objects one by one instead of moving them with a tool); where a row checks the
  way (an object that must not be lifted, a number of presses), keep to exactly what it checks.
- Containers, fixtures and stacks must stay upright and in place.
- After a pour, the container usually needs to be turned back upright. Before that it should be
  close to empty, with what left it inside the target (not splashed or spilled). Some liquid in
  the target does not make it done.
- After the task's arrangement is made, both arms go home with open grippers when the score
  table's top row asks for a return to origin (section 1). If the episode is still running after
  that, something does not match yet: compare carefully, fix, home again where needed.

Budget:
- Before each phase, compare the steps it needs with the steps left. A pick and place takes about
  60-80 steps, more with careful checks. When the score table's top row asks for a return to
  origin, keep about 40 for the final release and `home`; otherwise no final `home` is needed.
- When the budget is nearly gone, secure what partial credit the score table gives rather than
  starting something that cannot be finished.

Pictures:
- The conversation keeps the three start pictures and the most recent others (at most 20 in all);
  older ones are replaced by a note. Call `robot.look()` for a current picture rather than relying
  on an old one, and describe in your own words what you need to remember from a picture.
