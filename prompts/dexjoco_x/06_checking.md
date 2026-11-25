# 6. Checking and recovering

Check every key step before the next one:
- Read the Hand, Arc and Fingers lines: `BLOCKED`, `short of target`, `holding nothing` and notes
  mean the step did not go as planned. A touch note ("presses on something now", "touched something
  on the way", "slipped out") is all the robot feels: something may have been pushed, tipped or
  dropped - look before going on.
- After grasping, lifting, placing, pressing or turning, check the hand's `.holding` and look at
  the scene (a wrist camera close by shows more than the scene camera): a picture after the action
  is the only proof of where things ended up.
- Several motions in one cell are fine when the code checks the results between them, for example
  `if not f.holding: print('missed'); ...` before lifting, so a failure stops the plan. `BLOCKED`
  and `short of target` do not stop a cell by themselves: test `.ok` / `.reached` / `.holding` where
  it matters.

When something fails:
- Name the cause from what you see and read (the fingers closed above the handle, the thumb hit the
  object on the way down, the object slid out, the arm was at a joint limit, the object was measured
  from one side only) and change what caused it: the grasp point, the orientation, the approach, the
  shape. Repeating the same numbers gives the same result.
- A knocked-over or dropped object is not lost: find it, measure it again and go on, if the arm
  still reaches it.
- A hand that stopped against something stays there pressing: move it back out (`by=` away from
  what it pressed) before the next motion.

Rules that end the episode at once (the task card lists the task's own):
- Some tasks end without success when a rule is broken: in the tablet task a right-hand touch on the
  tablet before the left hand holds it as required, a lost hold afterwards, or a wrong digit; in the
  Tower of Hanoi a larger disk on a smaller one, or a disk left on the table. Plan every motion near
  such things with that in mind, and check (`dry_run`, `robot.to_pixel`, a close look) before
  moving.

How strict the benchmark is:
- It checks exactly what the task card says it checks, at one step: everything the card names must
  hold at the same time. Parts it does not check (how a thing is held, which finger presses) are
  yours to choose.

Budget:
- Before each phase, compare the steps it needs with the steps left.
- When the budget is nearly gone, finish what is closest to done rather than starting something new.

Pictures:
- The conversation keeps the start pictures and the most recent others; when they grow too many
  (several dozen), the oldest are replaced by a note. Call `robot.look()` for a current picture
  rather than relying on an old one, and describe in your own words what you need to remember from
  a picture.
