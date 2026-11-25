# 5. Moving and handling objects

How motions run:
- `goto` moves the grasp point in a straight line when it can (`path='line'` insists on it).
  Otherwise it swings the joints, but only when the grasp point stays within about 3 cm of that
  line and dips at most about 1 cm below its lower end; a wider curve can sweep through objects,
  so that move is not executed and the result says how far off it would go: go via a waypoint
  (each leg a straight line), or pass `path='swing'` to accept the curve. Near objects, use
  straight lines; to cross the table, lift first.
- A pose that cannot be reached is not executed: the result says why and what the arm reaches
  instead (the hand tilted toward the point or from the other side, another yaw that a straight
  line leads to, a point on the way there). Take one of those rather than resending the same
  numbers or trying angles one by one, and dry-run it first: it can need a swing of its own. A
  pose the arm misses by at most about 1 cm and 3 degrees is accepted: it goes to the closest pose
  it reaches, and the result says so.
- **Turn a hand in the open.** Both hands start pointing forward with the palms facing each other,
  and most grasps need a hand turned far from there (pointing down is a turn of 150 degrees or
  more). Turn it at a waypoint 15-20 cm clear of everything, then go in on a straight line with
  the same angles: a motion that turns the hand on its way sweeps the fingers through whatever is
  near its end, and can end BLOCKED there. A dry run says whether a pose is reachable, not whether
  the hand's way to it is clear. Change the angles the same way later: back out, turn, come back
  in - not with the fingers among the objects.
- A motion is sent whole and runs to its end; nothing stops it halfway. `BLOCKED` means the arm
  ended pressing on something - an object, the table, the other arm or the robot's own body - and
  keeps pressing until the next motion. Look before going on: something may have been pushed or
  knocked over. To get out, back straight out along the fingers with the same angles: a motion
  that turns the hand while the fingers are among objects drags them along. `Dropped` means what
  the hand held slipped out on the way: look where it went.
- `contact=True` is for a press, push or pull meant to meet resistance: it raises no `Blocked`,
  and the result says whether something stopped the hand (it then keeps pressing on it) or nothing
  did (it presses on nothing, and a light object in its way moved along).
- Only one arm moves at a time (except `goto('both', by=...)`); the other holds its pose, and
  fingers stopped on something keep squeezing.
- Motions are brisk and slow down over their last few centimetres; how long one takes depends
  most on how far the hand turns. On this robot a short straight move (10 cm, the angles kept)
  takes about 15-30 steps, a move that also turns the hand far (the first one from the start pose)
  80-135, a swing 40-130, a change of a hand's shape or opening 28-45, `home` 55-70, a waist turn
  of 10-25 degrees (where the torso turns) 80-95. Every result prints the steps it used. What
  counts is the benchmark's allowance per stage, of steps and of motions (section 1): a direct
  approach leaves plenty; tries, detours, small corrections and long waits use them up.
- `goto(..., grip=...)` sets a pinch's or a wrap's opening in the same motion, from the start of
  the move: open on the approach that way, never while the hand holds what it should keep. Close
  with `gripper()` or `hand()` once there.

The hands:
- The task card gives the four shapes, each shape's grasp point (the point `goto` places) and how
  far the hand reaches past it, the pinch's gap at each opening and the wrap's bar.
- Set the shape before the approach, with the hand clear of everything: a change of shape moves
  the fingers, and they push what they sweep through (at the start the fingers can already touch
  an object beside them: lift the hand first). It also moves the grasp point by several cm and
  changes the angle readings, while the wrist stays: plan the next `goto` from what the result
  (or `robot.state()`) says. `hand(arm, 'flat')` and `hand(arm, 'point')` take no opening.
- What holds:
  - a pinch holds what its thumb and index pads stop on. Its other three fingers hang half curled
    on the palm's side and reach past the pads (the task card says how far): they meet what lies
    under or beside the pads first, so turn the palm so that they pass outside the object, not
    over or into it;
  - a wrap holds what at least two of its fingers stop on: a bar, a handle, a rim. Open, its
    fingers reach far past its grasp point, so a wrap moved along its fingers toward a light
    object pushes it ahead: bring the open hand to the object from the side the palm faces;
  - flat and point hold nothing: they push, press, tilt and lift from below. Two flat palms carry
    a box pressed between them (the task card says how).
- After a close, `holding` says that the holding fingers stopped on something. Compare the pad
  gap with the object's size, and confirm with a picture after the lift before carrying: holding
  has been reported while the object slid away. A pinch counts only its thumb and index: what the
  curled fingers alone hold reads as empty.
- Letting go: `robot.gripper(arm, 'open')` opens a pinch or a wrap fully (as a change of shape
  without an opening does); `robot.hand(arm, 'flat')` straightens every finger.

Picking something up:
1. Measure it from the head picture. Set the shape (a pinch for small things and rims, a wrap for
   bars and handles) and go to about 10 cm from it, on the side you will grasp from, with the hand
   open; measure it again from there (`t = robot.measure(thing)`): the wrist camera now looks at
   it. If the new centre or `grip_yaw` differs from the first (more than about 1 cm, or 10
   degrees), aim at the new one: light objects slide or tip at the first touch, and a miss pushes
   them away.
2. Go in on a straight line until the holding fingers are round the part to hold. Pointing down,
   a pinch's pads stay as high above the surface under the hand as its curled fingers reach past
   them (the task card): from above it takes the top of something taller than that - a rim, a
   handle, an edge that stands up -, not something lying flat.
3. Close there and check the Hand line. No wait is needed before lifting: the lift is measured
   from where the object was when its stage began (section 1).
4. Lift straight up 8-10 cm and confirm with a picture that the object came up. Carrying it on at
   that height keeps it up for the looks that follow.

Other grasps:
- Something lying flat and too low for a pinch from above (a lid, a card, a tool on the table):
  take a part of it that stands up; ways to try besides: slide it until an edge overhangs the
  table's edge and take that edge, or press it between two flat palms.
- Long things: near the middle or by a handle, across the long side (`grip_yaw` gives that).
- Farther than a hand pointing down reaches: tilt the hand toward it - `pitch` 45-60, `yaw` its
  heading from that arm's shoulder - or, where the torso turns, turn the waist toward it (section
  2). Ask `dry_run=True` when unsure. A hand that has crossed the robot's middle is refused low
  poses there: take things on each side with that side's hand.
- Something too big for one hand: a flat palm on each side, each sent a little into it with
  `contact=True` (the result says whether it presses), then `robot.goto('both', by=...)` to lift
  and carry it; check that it stays level and held.

Putting down:
- Carry high enough to clear everything on the way: lift, move across, then descend.
- Lower in a straight line until the object's bottom is about 1 cm above the surface it goes on,
  open the hand, then lift straight up before moving sideways. Dropping from higher makes things
  bounce, roll or tip over; pressing it down drags it out of place.
- Onto or into another object (a plate, a box, a basket, a shelf, a stack): over its centre,
  measured from close by just before. A held object hangs where the fingers hold it, so place by
  the object, not by the hand: if you held it off-centre, shift the target by the same amount.
- Stacking: aim at the lower object's centre - where you released it, if you placed it yourself
  (measuring a stack again can merge the outlines).
- After letting go, keep the hand clear of the object, so that the benchmark sees it resting where
  it goes: the opening itself takes long enough for a look; lifting the hand away is fine. A box
  carried between two palms: lower it until it rests, then move the palms apart.

Pushing, pulling, pressing, tilting, turning:
- Push with a fist (`hand(arm, 'wrap', 0)`) or a flat palm, its lowest point 1-2 cm above the
  surface, in a straight line, `contact=True`; check after each push. Push the whole way in one
  motion when you can see that the path is clear, in two or three when the object might turn.
  Pushing higher tips objects over.
- Doors, drawers and lids: grip the handle (a wrap or a pinch) or push on the flat part with the
  palm, and move along the way it opens - straight out for a drawer; for a door, along its arc
  about the hinge, in short straight legs, with `contact=True`. Re-measure the handle after each
  leg.
- Press (buttons and the like): the point shape, its fingertip above the centre, straight down
  until it is 1-2 cm below where it touches the top, then straight back up at least 3 cm.
- Tilting something that stands (out of a row, onto its side): press or pull at its top edge with
  the point shape's fingertip. Past its balance it keeps going while you think: put the other hand
  (a fist or a flat palm) where it would go before you tilt it, and never leave it leaning on
  nothing.
- To turn a held object, change yaw (or roll) with the arm high above everything; to pour, hold
  the container above the target and tilt it with `roll` (or `pitch`) in a few steps, then turn it
  back. `dry_run=True` shows whether a turn is possible before you commit.

The two arms:
- Each arm reaches its own side best. For something in the middle, check both with `dry_run`.
  An instruction that names the arm to use means that arm.
- Keep the arm that is not working clear of the other arm's path, the camera views you need and
  the objects: lifted and toward its own side is enough. It can also hold still as a stop or a
  support - a flat palm or a fist where a tipping or sliding object would go.
- A hand-over: one arm holds the object still and high in the middle; the other grips a free part
  of it; the first then opens and backs away along its pointing direction.
