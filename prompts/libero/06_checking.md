# 6. Checking and recovering

Check every key step before the next one:
- Read the Move and Grip lines: `BLOCKED`, `short of target`, `closed fully` and notes mean the
  step did not go as planned.
- After grasping, placing, opening or turning something, look at the scene or measure again. A
  picture after the action is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for
  example `if not g.holding: print('missed'); ...` before lifting, so a failure stops the plan.
  A motion that cannot be done stops the cell by itself (`NotExecuted`); `BLOCKED` and `short of
  target` do not, so test `.ok` where it matters.

When something fails:
- Name the cause from what you see (the gripper closed beside the object, the object slid out,
  the hand was blocked by the table or a cabinet, the handle slipped out of the fingers, the
  target was measured from one side) and change what caused it: the position, the angle, the
  height, the opening or the approach. Repeating the same numbers gives the same result.
- A dropped or knocked-over object is not lost: find it, measure it again and go on.

How strict the benchmark is - it judges where the objects are and how the fixtures stand:
- "on" an object (a plate, a bowl, a box): resting on it and centred on it, seen from above.
  Anywhere on the plate is not enough: centre it, and measure both again after letting go.
- "on" a part of a fixture (the cooking area of a stove, the top of a cabinet, a rack) or on an
  area of the table named by a relation ("the front of the stove", "the right of the plate"): its
  centre over that part or area, resting on it (not held high above it). An area named by a
  relation is a small patch on that side of the named thing, roughly in line with its centre.
- "in" (a basket, a drawer, a compartment, a microwave): its centre inside the space within the
  walls, below the rim.
- "open": nearly as far as it goes - a drawer pulled out almost fully, a door swung wide.
  "closed": fully shut - a drawer pushed in until it stops, a door pressed flat; nearly shut does
  not count, so push once more and look. "turn on": the knob turned clearly away from off.
- With two parts ("put both ...", "... and close it"), the first part must still hold when the
  second is done: do not knock the first object off while doing the second, and look at both at
  the end.

Budget:
- Steps run out long before replies do: the budget leaves room for one or two retries at most.
  Looking, measuring and `dry_run` use no steps; use them to get each motion right the first time
  rather than trying and correcting.
- Spend steps only on motions that do work:
  - keep the hand pointing down unless the task needs another angle, and when it does, turn
    during a move that travels anyway;
  - open the gripper with the approach move (`grip=`), not with a call of its own;
  - go at full speed, except for the last few centimetres onto or into something;
  - measure from close by at the point above a grasp you pass anyway, not with an extra move;
  - skip `home()` unless the arm is stuck.
- A pick and place with checks fits in two or three replies when each reply's code does a whole
  phase.
- When the steps left are few, finish what is closest to done by the shortest way rather than
  starting something new; there is no partial credit, and stopping early earns nothing.

Pictures:
- The conversation keeps the two start pictures and the most recent others (at most 20 in all);
  older ones are replaced by a note. Call `robot.look()` for a current picture rather than relying
  on an old one, and describe in your own words what you need to remember from a picture.
