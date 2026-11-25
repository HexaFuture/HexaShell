# 5. Moving and handling objects

How motions run:
- `goto` moves the grasp point in a straight line when it can (`path='line'` insists on it).
  Otherwise it swings the joints, but only when the grasp point stays within about 3 cm of that
  line and dips at most about 1 cm below its lower end; a wider curve can sweep through objects,
  so that move is not executed and the result says how far off it would go: go via a waypoint
  (each leg a straight line), or pass `path='swing'` to accept the curve. Near objects, use
  straight lines; to cross the table, lift first.
- A pose that cannot be reached is not executed: the result says why and what the arm reaches
  instead (the fingers a quarter turn round, the hand tilted toward the point or from the other
  side, another yaw that a straight line leads to, a point on the way there). Take one of those
  rather than resending the same numbers or trying angles one by one. A pose the arm misses by at
  most about 1 cm and 3 degrees is accepted: it goes to the closest pose it reaches, and the
  result says so.
- A motion is sent whole and runs to its end; nothing stops it halfway. `BLOCKED` means the arm
  ended pressing on something - an object, the table, the other arm or the robot's own body - and
  keeps pressing until the next motion. Look before going on: something may have been pushed or knocked over. `Dropped`
  means what the hand held slipped out on the way: look where it went.
- Only one arm moves at a time (except `goto('both', by=...)`); the other holds its pose, and a
  closed gripper keeps squeezing.
- Motions are brisk and slow down over their last few centimetres: a 10 cm straight move takes about
  15 steps, from the start pose to above an object 20-60, a full close or open of the gripper 40,
  `home` 40-55, a 20-degree waist turn (where the torso turns) about 30. Every result prints the
  steps it used. What counts is the benchmark's allowance per stage, of steps and of motions
  (section 1): a direct approach leaves plenty; tries, detours, small corrections and long waits use
  them up.
- `goto(..., grip=...)` moves the gripper along with the arm in the same motion: open on the
  approach that way. Close and let go with `gripper()` once there.

The gripper:
- The task card gives the gap between the pads at each opening g (0..1) and the widest thing that
  fits between them.
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
   about half its height for a taller one, but with the fingertips clear of the surface it stands
   on (the task card says how far they reach past the grasp point, open and closed, and where
   grippers that swing forward close on a low object).
3. Close the gripper there and check the Grip line. No wait is needed before lifting: the lift is
   measured from where the object was when its stage began (section 1).
4. Lift straight up 8-10 cm and confirm with a picture (or `robot.state()`: holding) that the object
   came up. Carrying it on at that height keeps it up for the looks that follow.

Other grasps:
- Tall or thin things standing up (bottles, cans, tools): from the side, the hand level or
  tilted, the grasp point at half to two thirds of the height, approaching along the pointing
  direction from about 10 cm away with the gripper open.
- Long things: near the middle or by a handle, across the long side (`grip_yaw` gives that).
- Hollow things (cups, bowls, baskets, pots): by the wall or the rim, one finger inside and one
  outside, or by a handle.
- Farther than a hand pointing down reaches: tilt the hand toward it - `pitch` 45-60, `yaw` its
  heading from that arm's shoulder - or, where the torso turns, turn the waist toward it (section
  2). Ask `dry_run=True` when unsure.
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
- After letting go, keep the hand clear of the object, so that the benchmark sees it resting where
  it goes: the opening itself takes long enough for a look; lifting the hand away is fine.

Pushing, pulling, pressing, turning:
- Push with the gripper closed, its lowest point 1-2 cm above the surface, in a straight line,
  `contact=True`; check after each push. Push the whole way in one motion when you can see that
  the path is clear, in two or three when the object might turn. Pushing higher tips objects
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
