# 6. Checking and recovering

Check every key step before the next one:
- Read the Body, Walk, Hand and Fingers lines: `BLOCKED`, `short of target`, `holding nothing` and
  notes mean the step did not go as planned.
- After grasping, lifting, placing or handing over, check the hand's `.holding` and look at the
  scene: a picture after the action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for example
  `if not f.holding: print('missed'); ...` before lifting, so a failure stops the plan. `BLOCKED` and
  `short of target` do not stop a cell by themselves: test `.ok` / `.reached` where it matters.

When something fails:
- Name the cause from what you see (the palm closed beside the object, the object slid out, the walk
  stopped at the table, the arm was at a joint limit, the object was measured from one side) and
  change what caused it: where the robot stands, the body's height or lean, the palm's point or
  orientation. Repeating the same numbers gives the same result.
- A dropped or knocked-over object is not lost: find it, measure it again and go on.
- If the robot stands crooked or too close after a blocked walk, walk back a little before trying
  again.

How strict the benchmark is:
- It checks the instruction's words: the object it names, lifted, held by which hand, placed on or in
  what. Objects placed must rest there by themselves, the fingers open.

Budget:
- Before each phase, compare the steps it needs with the steps left and with the official limit.
- When the budget is nearly gone, finish what is closest to done rather than starting something new.

Pictures:
- The conversation keeps the start picture and the most recent others (at most 20 in all); older ones
  are replaced by a note. Call `robot.look()` for a current picture rather than relying on an old one,
  and describe in your own words what you need to remember from a picture.
