# 5. Moving and handling things

How motions run:
- `goto` moves the grasp point in a straight line at up to 25 cm/s (or the `speed` asked for),
  turning the hand on the way. A straight line can sweep through things: near objects and fixtures
  keep moves short, and to cross over something, lift first (each leg a straight line). `home()`
  lifts first when low, then swings the arm's joints back to where they started (the hand's way
  curves): near an open door or drawer, first move the hand clear of it, or it may push it shut.
- The arm cannot work stretched out straight. A pose it would hold with the elbow nearly straight,
  or a straight line passing through such poses, is refused as out of reach (a long way out while
  the hand turns a lot swings the wrist out: go in two moves). A Move, a grip or a wait in which
  the elbow straightens that far anyway stops, and the arm goes back part of the way it came, which
  bends the elbow, with a note: aim nearer the arm or at another pitch. While the arm is stretched
  that far, only `home()` moves it.
- Pressing hard on something with the elbow open wider than a right angle (the hand, the wrist
  or the forearm against a fixture) would snap the elbow straight: such a motion stops at once and
  the arm pulls the hand 5 cm back toward the shoulder, with a note. Look what it touched, and come
  from another side or pitch.
- A pose that cannot be reached is not executed: the result says why (too far from the shoulder,
  or how close the hand would get). The base cannot move closer, so ask for another pitch, height
  or approach direction rather than resending the same numbers.
- `BLOCKED` means the hand is pressing on something - an object, the table, a cabinet, a door.
  Look before going on: something may have been pushed or knocked over. A Move that stopped short
  "and was not getting closer" touches something too, or the arm is at the edge of what it can do
  there.
- Low and far with a level hand is often out of reach; pointing down, or a pitch near 45 degrees,
  reaches lower. Far from the shoulder the hand is less exact.
- Rough costs: a 10 cm move about 15-20 steps (at `speed=0.05` about 45-50), 30 cm about 30-40,
  turning the hand 90 degrees about 45, a gripper change about 15, `home` about 70. A move that also
  turns the hand costs about the larger of the two, not their sum. Every Move, Grip and Homing
  prints what it used and what is left.
- The two fingers are alike, so roll r and roll r + 180 grip the same, but one of them may turn the
  hand much less. From pointing down at yaw 0 (the start), levelling the hand with the fingers
  closing up-down: pointing right (yaw -90), roll -90 turns it 90 degrees and roll 90 about 180
  (some 30 steps more); pointing left (yaw 90), the other way round.
- Steps are scarce, replies much less so. Let one cell carry a whole phase - approach, grasp,
  check, lift, carry, place, check - with tests in the code between the motions (section 6).

The gripper:
- An opening g (0..1) leaves a gap of about g × 8 cm between the pads.
- It is position-controlled: `'close'` squeezes whatever is between the pads and keeps squeezing
  while the arm moves. An opening wider than the object does not hold it. Carry things closed.
- After closing, `holding` and the width say what is in the fingers: holding with a width near the
  object's size is a grasp; `closed fully` is a miss (unless it is very thin - look at the wrist
  picture). A Move or Homing that says what it held slipped out means it fell on the way: look
  where it went; one that says the gap between the fingers changed means it turned or slid in
  them.
- A held object turns with the hand. Something long gripped from above lies across the fingers,
  along the hand's up-down direction: levelling the hand at roll 0 stands it upright; at roll 90 it
  stays lying, across the pointing direction.

Picking something up from above:
1. Go 8-10 cm above it pointing down with `yaw=thing.grip_yaw`, opening the gripper on the way
   (`grip=`, a gap at least 2 cm wider than the object), and measure it again from there.
2. Descend in a straight line to the grasp height (from the new measurement): 1.5-2 cm below its
   top for a small object, about half its height for a taller one, but no lower than 1 cm above
   the surface it stands on.
3. Close, check the Grip line, lift straight up 8-10 cm, and confirm with a picture that the
   object came up with the gripper.

Other grasps:
- Tall things (bottles, cans, cartons) are often easier from the side: level pitch, the grasp
  point at about half the height, approaching along the pointing direction from 10-15 cm away,
  the gripper open. A bottle can also be taken from above by its neck.
- Long or flat things (a book, a box lying down): grip near the middle, across the narrow side
  (`grip_yaw` gives that); something thin is gripped across its thickness at an edge.
- Hollow things (bowls, mugs) can be gripped by the wall or the rim: one finger inside, one
  outside. A mug can also be gripped by its handle.

Putting down and putting in:
- Carry high enough to clear everything on the way: lift, move across, then descend.
- Lower in a straight line until the object's bottom is about 1-2 cm above the surface it goes on,
  open, then lift straight up (or back out along the way in) before moving sideways.
- Onto another object (a plate, a bowl, a box): over the other object's centre, measured from close
  by just before - section 6 says how exactly. A held object hangs where the fingers hold it, so
  place by the object, not by the gripper: if you held it off-centre, shift by the same amount.
- Into a basket, a bowl, a compartment: release with the object over the middle of the opening and
  low enough that it ends up inside the walls, below the rim.
- Into a drawer or a microwave: open it wide first (and leave it open); come in from above or
  level through the opening with the held object clear of the front and the edges, set it down
  inside, open, back out the way you came.

Pushing:
- Flat or awkward things (a plate) can be slid along the table: lower the closed gripper to 1-2 cm
  above the table just beside it, on the side away from where it must go, and push in short
  straight legs through its centre (off-centre pushes turn it), looking and measuring after each.

Fixtures:
- Drawers slide straight out along the direction their front faces. Grip the handle (or hook the
  fingers behind the front's top edge) and pull with `robot.goto(by=...)`; a drawer is stiff, it
  moves only while pulled, and the grip can slide off the handle: pull in legs of 4-6 cm and look.
  To close one, push its front straight back in with the closed gripper until it stops.
- Doors with a hinge (a microwave) turn about it. With the grasp point on the handle or the free
  edge, `robot.arc(hinge, angle, axis='z')` moves the hand along the door's circle about a vertical
  hinge line through `hinge`. Check the direction first: `dry_run=True` prints where the arc ends.
  Swing in steps of 20-40 degrees and look after each; re-grip when the hand lags behind the door.
  To close a door, push its outer face near the free edge along its circle until it is flat shut.
- Knobs turn about their own axis. For an upright axis: point down onto the knob, close on it and
  change `yaw` (`robot.goto(yaw=...)` keeps the position); for a level axis: point along it and
  change `roll`. Small steps, looking after each. A knob with a lever or a pointer can also be
  pushed round by it with the closed gripper.
- Buttons and switches: closed gripper, straight in until 1-2 cm past where it touches, then
  straight back out.
