# 6. Checking and recovering

Check every key step before the next one:
- Read the Move, Grip and Drive lines: `BLOCKED`, `short of target`, `closed fully` and notes mean
  the step did not go as planned.
- After grasping, placing, opening or turning something, look at the scene or measure again. A
  picture after the action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not g.holding: print('missed'); ...` before lifting, so a failure stops the plan.
  A motion that cannot be done stops the cell by itself (`NotExecuted`), and so does one during
  which what the hand held slipped out (`RobotError`, with the motion's line); `BLOCKED` and
  `short of target` do not, so test `.ok` where it matters.

When something fails:
- Name the cause from what you see (the gripper closed beside the object, the object slid out,
  the hand was blocked by the counter, the handle slipped out of the fingers, the target was
  measured from one side) and change what caused it: the position, the angle, the height, the
  opening, or where the base stands. Repeating the same numbers gives the same result.
- A dropped or knocked-over object is not lost: find it, measure it again and go on. Only one
  on the floor is out of reach (the grasp point stays 5 cm above it: "below the floor limit").

How strict the benchmark is:
- It checks the instruction's words: "in", "on", "open", "closed", "on/off", "next to" each mean a
  measurable state, and every object and fixture the instruction names counts. "Open" usually
  means open wide, not ajar; "closed" means shut.
- Objects placed in or on something must rest there by themselves, the gripper open and away from
  them (section 1).

Budget:
- Before each phase, compare the steps it needs with the steps left. A pick and place with a short
  drive takes about 200-300 steps, more with careful checks.
- When the budget is nearly gone, finish what is closest to done rather than starting something
  new.

Pictures:
- The conversation keeps the three start pictures and the most recent others (at most 20 in all);
  older ones are replaced by a note. Call `robot.look()` for a current picture rather than relying
  on an old one, and describe in your own words what you need to remember from a picture.
