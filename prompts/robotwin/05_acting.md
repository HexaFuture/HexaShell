# 5. Moving and handling objects

How motions run:
- `goto` moves the grasp point in a straight line when it can (`path='line'` insists on it).
  Otherwise it swings the joints, but only when the grasp point stays within about 3 cm of that
  line; a wider curve can sweep through objects, so that move is not executed and the result says
  how far off it would go: go via a waypoint (each leg a straight line), or pass `path='swing'`
  to accept the curve. Near objects, use straight lines; to cross the table, lift first.
- A pose that cannot be reached is not executed: the result says why and what is reachable
  instead (another pitch, another height, the other arm). Change the plan rather than resending
  the same numbers. A pose within 2 cm and 8 degrees of reach is accepted with a note.
- `BLOCKED` means the arm is pressing on something - an object, the table or the other arm. Look
  before going on: something may have been pushed or knocked over.
- Only one arm moves at a time; the other holds its pose, and a closed gripper keeps squeezing.
- Steps are plentiful (a 10 cm straight move takes 5-7, from the start pose to above an object
  12-18, `home` 15-20; every Move, Grip and Homing prints what it used and what is left); replies
  are not. Let one cell carry a whole phase - approach, grasp, check, lift, carry, place, check -
  with tests in the code between the motions (section 6).

The gripper:
- An opening g (0..1) leaves a gap of about 11·g - 2 cm between the pads: 1 → 9 cm, 0.8 → 6.8 cm,
  0.6 → 4.6 cm, 0.5 → 3.5 cm. The pads touch at about 0.18.
- It is position-controlled: `'close'` squeezes whatever is between the pads and keeps squeezing
  while the arm moves. An opening wider than the object does not hold it. Carry things closed.
- After closing, `holding` and the width say what is in the fingers: holding with a width near the
  object's size is a grasp; `closed fully` is a miss - except on thin things, which the fingers
  close on completely: look at a wrist picture to tell. `holding` None means the fingers stayed
  about fully open: they are blocked (pressed on the table or against something) - look before
  lifting. A Move or Homing that says what it held slipped out means it fell on the way: look where
  it went.
- Letting go: `robot.gripper(arm, 'open')` opens fully. After a placement leave the gripper fully
  open, not part-open (section 1).

Picking something up from above:
1. Go 8-10 cm above it pointing down with `yaw=thing.grip_yaw`, opening the gripper on the way
   (`grip=`, a gap at least 2 cm wider than the object), and measure it again from there.
2. Descend in a straight line to the grasp height (from the new measurement): 1.5-2 cm below its
   top for a small object, about half its height for a taller one, but no lower than 1.5 cm above
   the surface it stands on.
3. Close, check the Grip line, lift straight up 8-10 cm, and confirm with a picture or
   `robot.measure(thing)` that the object came up with the gripper.

Other grasps:
- Tall or thin things (bottles, cans, standing tools) are steadier gripped from the side: level or
  tilted pitch, grasp point at half to two thirds of the height, approaching along the pointing
  direction from about 10 cm away with the gripper open.
- Long things: grip near the middle or by a handle, across the long side (`grip_yaw` gives that).
- Hollow things (cups, bowls, baskets) can be gripped by the wall or the rim: one finger inside, one
  outside; a mug also by its handle.
- Something too big for one hand takes both hands, one on each side, lifted together in small
  steps: move one arm a few centimetres, then the other, and check after each pair that it stays
  level and held.

Putting down:
- Carry high enough to clear everything on the way (lift, move across, then descend).
- Lower in a straight line until the object's bottom is about 1 cm above the surface it goes
  on, open, then lift straight up before moving sideways. Dropping from higher makes things
  bounce, roll or tip; pressing it down drags it out of place.
- Onto or into another object (a pad, a plate, a basket, a stand): over its centre, measured from
  close by just before. A held object hangs where the fingers hold it, so place by the object, not
  by the gripper: if you held it off-centre, shift the target by the same amount.
- Stacking: aim at the lower object's centre - where you released it, if you placed it yourself
  (measuring a stack again can merge the outlines).

Pushing, pressing, turning:
- Push with the gripper closed, its lowest point 1-2 cm above the table, in a straight line, a
  few centimetres at a time; check after each push. Pushing higher tips objects over.
- Press (buttons and the like): close the gripper, point down above the centre, go straight
  down until the fingertips are 1-2 cm below where they touch the top, then straight back up at
  least 3 cm. Count every press.
- To turn a held object, change yaw (or roll) with the arm high above everything. The wrist turns
  only about 178 degrees either way from its middle; `dry_run=True` shows whether a turn is
  possible before you commit.

The two arms:
- Each arm reaches its own side best. For something in the middle, check both with `dry_run`.
  An instruction that names the arm to use means that arm.
- Keep the arm that is not working clear of the other arm's path, the camera views you need and
  the objects: lifted and toward its own side is enough.
- A hand-over: one arm holds the object still and high in the middle; the other grips a free part of
  it; the first then opens and backs away along its pointing direction.
