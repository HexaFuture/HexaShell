# 5. Moving and handling objects

How motions run:
- `goto` moves the grasp point in a straight line when it can (`path='line'` insists on it).
  Otherwise it swings the joints, but only when the grasp point stays within about 3 cm of that
  line and dips at most about 1 cm below its lower end; a wider curve can sweep through objects,
  so that move is not executed and the result says how far off it would go: go via a waypoint
  (each leg a straight line), or pass `path='swing'` to accept the curve. Near objects, use
  straight lines; to cross the table, lift first.
- A pose that cannot be reached is not executed: the result says why and what the arm reaches
  instead (the fingers a quarter turn round, the hand tilted toward the point, a point on the way
  there). Change the plan rather than resending the same numbers. A pose the arm misses by at most
  about 1 cm and 3 degrees is accepted: it goes to the closest pose it reaches, and the result
  says so.
- `BLOCKED` means the arm is pressing on something - an object, the table or the other arm. Look
  before going on: something may have been pushed or knocked over. `Dropped` means what the hand
  held slipped out on the way: look where it went.
- Only one arm moves at a time (except `goto('both', by=...)`); the other holds its pose, and a
  closed gripper keeps squeezing.
- Motions are quick: a 10 cm straight move takes 4-6 steps, from the start pose to above an object
  11-22, closing the gripper 5-17, `home` about 23, a 20-degree waist turn about 5. Every result
  prints the steps it used. The steps that count are the benchmark's per stage (section 1): a
  direct approach and short holds leave plenty; tries, detours and long waits use them up.

The gripper:
- An opening g (0..1) leaves a gap between the pads of about: 1 → 6.8 cm, 0.8 → 5.6 cm,
  0.6 → 4.4 cm, 0.5 → 3.7 cm, 0.4 → 3.0 cm. Nothing wider than about 6.5 cm fits between them.
- It is position-controlled: `'close'` squeezes whatever is between the pads and keeps squeezing
  while the arm moves. An opening wider than the object does not hold it. Carry things closed.
- After closing, `holding` says what is in the fingers. True: the fingers stopped on something - a
  grasp; its width reads lower than the object's own size (up to about 1.5 cm), so compare the two
  only roughly. False (`closed fully`): a miss, or something very thin - look at a wrist picture to
  tell. A Move that raises `Dropped`, or a Homing that says what it held slipped out, means it fell
  on the way.
- Letting go: `robot.gripper(arm, 'open')` opens fully.

Picking something up from above:
1. Go about 10 cm above it pointing straight down (`pitch=90`) with `yaw=thing.grip_yaw` (when it
   is None any yaw holds it; 0 is the easiest), opening the gripper on the way (`grip=1`), and
   measure it again from there: `t = robot.measure(thing)`. The wrist camera now looks down at its
   top. If the new center or `grip_yaw` differs from the first measurement (more than about 1 cm,
   or 10 degrees), move over the new center with the new yaw before going down: fingers closing
   across a block's corners slip off it.
2. Descend in a straight line to the grasp height: about 2 cm below its top for a small object,
   about half its height for a taller one, but no lower than about 1 cm above the surface it
   stands on (the fingertips reach 1 cm below the grasp point).
3. Close the gripper there and check the Grip line, then keep still a little before lifting: the
   look that first sees the hand at the object also notes how high the object is, and the lift is
   measured from that (section 1). Lifting too soon can have that look come with the object
   already on its way up.
4. Lift straight up 8-10 cm, keep it there still while the benchmark looks, and confirm with a
   picture (or `robot.state()`: holding) that the object came up.

Other grasps:
- Tall or thin things standing up (bottles, cans, tools): from the side, the hand level or
  tilted, the grasp point at half to two thirds of the height, approaching along the pointing
  direction from about 10 cm away with the gripper open.
- Long things: near the middle or by a handle, across the long side (`grip_yaw` gives that).
- Hollow things (cups, bowls, baskets, pots): by the wall or the rim, one finger inside and one
  outside, or by a handle.
- Farther than a hand pointing down reaches (about 0.72-0.78 m out, less across the middle): tilt
  the hand toward it - `pitch` 45-60, `yaw` its heading from that arm's shoulder - or turn the
  waist toward it (section 2). Ask `dry_run=True` when unsure.
- Something too big for one hand: one hand on each side, both closed on it, then
  `robot.goto('both', by=...)` to lift and carry it; check that it stays level and held.

Putting down:
- Carry high enough to clear everything on the way: lift, move across, then descend.
- Lower in a straight line until the object's bottom is about 1 cm above the surface it goes on,
  open, then lift straight up before moving sideways. Dropping from higher makes things bounce,
  roll or tip over; pressing it down drags it out of place.
- Onto or into another object (a plate, a box, a basket, a shelf, a stack): over its centre,
  measured from close by just before. A held object hangs where the fingers hold it, so place by
  the object, not by the gripper: if you held it off-centre, shift the target by the same amount.
- Stacking: aim at the lower object's centre - where you released it, if you placed it yourself
  (measuring a stack again can merge the outlines).
- After letting go, keep the hand clear of the object and still for a moment, so that the
  benchmark sees it resting where it goes.

Pushing, pulling, pressing, turning:
- Push with the gripper closed, its lowest point 1-2 cm above the surface, in a straight line, a
  few centimetres at a time, `contact=True`; check after each push. Pushing higher tips objects
  over.
- Doors, drawers and lids: grip the handle (or push on the flat part) and move along the way it
  opens - straight out for a drawer; for a door, along its arc about the hinge, in short straight
  legs, with `contact=True`. Re-measure the handle after each leg.
- Press (buttons and the like): close the gripper, point down above the centre, go straight down
  until the fingertips are 1-2 cm below where they touch the top, then straight back up at least
  3 cm.
- To turn a held object, change yaw (or roll) with the arm high above everything; to pour, hold
  the container above the target and tilt it with `roll` (or `pitch`) in a few steps, then turn it
  back. `dry_run=True` shows whether a turn is possible before you commit.

The two arms:
- Each arm reaches its own side best. For something in the middle, check both with `dry_run`.
  An instruction that names the arm to use means that arm.
- Keep the arm that is not working clear of the other arm's path, the camera views you need and
  the objects: lifted and toward its own side is enough.
- A hand-over: one arm holds the object still and high in the middle; the other grips a free part
  of it; the first then opens and backs away along its pointing direction.
