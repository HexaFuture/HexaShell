# 5. Acting with arms and five-fingered hands

The arms stay where they are mounted: plan which hand does what from where things are, then each
motion's path, then the grasp.

Which hand, and from where:
- In the one-arm tasks the right hand does everything. In the two-arm tasks each hand works on its
  own side (section 2); a hand that holds or steadies something keeps it while the other works.
  Hold a thing the other hand must then work on (press a button on it, touch its screen) away from
  that part, so that the other hand can reach it.
- Check a doubtful pose with `dry_run=True` before moving. A joint-limit note means: change the
  hand's orientation (tilt the palm, turn the fingers) or the point, not the speed.

Moving without knocking things over:
- A hand's point goes in a straight line from where it is, and the whole hand sweeps along: the
  fingers 12-14 cm ahead of the palm, the thumb 12 cm to its side, the wrist and the arm behind it.
  Between two places near objects, go up first, then across, then down (separate calls), and do
  large turns of the hand in free space, not next to what you will grasp.
- Lowering an open hand palm down: the palm's face is its lowest part, the thumb level with it.
- Near objects, move slower (`speed=0.06`-`0.08`).

Grasping:
1. **Choose the grasp and the shape.** A handle or a bar: point 'grasp' on the handle's axis,
   `along` = the handle's direction (toward the end that should be on the thumb's side), palm down or
   as the handle lies, then 'power'. A basket's or a bag's handle: 'hook', the fingers under the
   handle. A box or a can standing up: 'wrap', with the palm against one side and the fingers round
   it. A small or low thing on a surface (a block, a ring, food): as the benchmark's demonstrations
   take it - the hand tilted 40-45 degrees, the palm down and back toward the robot, the fingers
   forward and down and turned 10-30 degrees toward the robot's middle (right hand
   `normal=(-0.6, -0.2, -0.78), finger=(0.77, 0.14, -0.62)`; left hand `normal=(-0.6, 0.2, -0.78),
   finger=(0.77, -0.14, -0.62)`) - and the grasp point 1.5-2 cm nearer the robot than the thing's
   middle, at its middle height or a little lower: the thumb closes on the near side, the index and
   middle fingers on the far side. Then 'tripod' ('pinch' for a thin rim). The turn matters for
   'tripod': not turned, or turned the other way, the fingers did not grip a piece of food that the
   turned hand lifted. These vectors are for a thing whose long side lies along y (`grip_yaw` about
   90 or -90). For one turned from that by d = (`grip_yaw` mod 180) - 90 degrees (positive d is
   counter-clockwise seen from above), turn both vectors about the vertical by d as well:
   (x, y) -> (x cos d - y sin d, x sin d + y cos d). With the vectors as given, food turned 16-17
   degrees stopped the hand on the way in and was not held (two seeds); with the vectors turned by
   d, food turned 3-17 degrees was held on all four seeds tried; turned by -d, on one of the four.
   With the palm straight down the fingers tend to close above a low thing or push it away. Reach
   for this tilt is in section 2. A box-like thing longer than the hand that leans on one end (the
   camera): 'wrap' at its upper end, the end away from any part the other hand must reach, not at
   its middle, where the fingers closed on nothing or it slipped out. The grasp
   point 4 cm from its measured centre toward that end (along the table), 1.5 cm nearer the robot
   and 2.5 cm below its measured top, the hand as the demonstrations hold the camera (left hand
   `normal=(-0.72, -0.26, -0.64), finger=(0.67, -0.5, -0.55)`; right hand
   `normal=(-0.72, 0.26, -0.64), finger=(0.67, 0.5, -0.55)`) or tilted 45 degrees and not turned
   (`normal=(-0.71, 0, -0.71), finger=(0.71, 0, -0.71)`). Both held through a 13 cm lift on every
   seed tried, also with the point 1 cm lower or to either side (1-1.5 cm higher, the 45-degree hand
   lost it); in the food orientation above the camera slipped out on the lift although all five
   fingers pressed. Gripped so and turned to aim along +x, the point that the environment checks in
   front of the lens (task card) was 13-14 cm in front of (+x of) the grasp point on five of six
   seeds (5 cm on the sixth): aim by that point, not by the grasp point. With the grasp point about
   52 cm in front of the target the point is well inside the checked range (30-50 cm); with the
   grasp point 42 cm in front, the middle of that range, the point was 28 cm in front of the
   target, 2 cm out of the range (two seeds). A wide flat thing leaning on a stand (a tablet): grip
   it by one side edge, the palm toward the edge (left hand at the +y edge: `facing='inward'`, or
   `normal=(0, -1, 0),
   finger=(0.86, 0, -0.5)`), the grasp point 4-6 cm in from the edge, at the face toward the robot
   and a little above the middle of the edge's height, then 'wrap'.
2. **Measure from close by** (section 4) and put the grasp point within about 1 cm of where step 1
   says: a handle's axis is about half its thickness below its measured top surface. A grasp 1.5 cm
   too high closes on nothing.
3. **Come in along the palm's normal** with the fingers open: `approach=0.08` to `0.10` on the grasp
   motion, so the fingers do not sweep into the object on the way. Before coming in, the hand goes in
   a straight line from where it is to the approach's start (the grasp point backed off along the
   palm's normal), turning on the way: from a look above the thing, first move it across to above
   that start, clear of the thing, then make the grasp motion. Sent straight from a look above a
   tablet's middle to a grasp at its edge, a hand hit the tablet on the way and knocked it off its
   stand. The same holds for a point the hand goes to before a grasp. Next to something tall (a post, a
   wall, another object) the open thumb catches on it: shape the fingers part of the way first
   (`robot.fingers(side, 'tripod', amount=0.5)`), bring the grasp point 8 cm straight above its goal,
   and come straight down (`by=(0, 0, -0.08)`, `speed=0.05`). A ring on a post that stands above
   it: the fingers turned further, about 22 degrees toward the robot's middle (right hand
   `normal=(-0.66, 0.11, -0.74), finger=(0.73, 0.3, -0.61)`, as the demonstrations take the rings),
   the grasp point 2 cm nearer the robot than the post, half the ring's width plus 1 cm to the hand's
   own side of the post, and 1 cm below the ring's middle height; shape the fingers part of the way
   and come straight down as above. Rings 5 and 6.6 cm wide were lifted this way; with the fingers
   turned only 10 degrees the hand stopped on the post or closed beside a 5 cm ring.
4. **Close** with the shape (`robot.fingers(side, 'tripod', seconds=0.6)`) and check `.holding`. If
   it is False, open (`robot.fingers(side, 'open')`), move back up, look, measure again and change what
   made it miss (1-2 cm lower, deeper, more to the middle; another orientation), then try again.
5. **Lift a little first**, slowly (`robot.hand(side, by=(0, 0, 0.05), speed=0.08)`), and check
   `.holding` again before going on: a grip held by two fingers only may slip. A wide flat thing
   held by one side swings down on its far side as it comes off its support, and its middle rises
   less than the hand: gripped 4-6 cm in from its edge, a tablet came clear of its stand on each of
   ten seeds; gripped 1-3 cm in, it hung more steeply, and on three seeds its far side still rested
   on the stand after a 12 cm lift (on one even after 30 cm). Look that the far side is clear before
   going on.

Pressing buttons and keys:
- Set 'point' first (the index straight, the middle, ring and little fingers curled, the thumb
  open), bring the index tip a couple of centimetres in front of the button (point='index', the
  finger along the press: at right angles to the face, `finger` into it), then press with a short
  slow move a few millimetres past the button's face (`by=`, `speed=0.03`-`0.05`): the move reports
  BLOCKED when the fingertip meets it - that is the press. Move back out before the next press: a
  new press needs a new touch.
- The open thumb's tip is about 9 cm from the index tip, back along the finger and out on the
  palm's side. With the finger at right angles to the face the thumb stays about 6.5 cm off it;
  with the hand tilted 40-45 degrees over a flat face it hangs level with the index tip and can
  touch the face. On a held tablet it so touched another digit, which ended the episode (one seed;
  at the first press in 22 of 27 episodes on held tablets the thumb's tip was within 2 cm of the
  index tip's height; with the finger at right angles to the face, 5-8.5 cm above it in ten of ten).

Turning things about a hinge (doors, lids):
- Measure the hinge line (where the door turns: at the door's own edge, inside the cabinet's outer
  corner) and the handle. Hold or push the door with the hand, then
  `robot.arc(side, hinge_point, angle, axis='z')` swings the hand round the hinge, the hand turning
  with it. A door swings open toward its hinge's side: open it with the hand on that side. A door
  hinged on its left (+y) edge turns clockwise, seen from above, as it opens toward the robot: a
  negative angle (two of ten episodes first turned such a door the other way).
- An upright handle: grip it with the palm on the hinge's side of the handle, facing along the door
  toward it (a door hinged on the left: `normal=(0, -1, 0)`, `along=(0, 0, 1)`), then 'power'. Come
  to the grasp from in front of the door - first to a point about 8 cm in front of the handle and
  8 cm toward the hinge, then straight to the grasp point (the hand keeps its orientation) - not
  with `approach=`, which brought the fingers against the door's face. So held, a microwave's door
  swung open 45-68 degrees on each of ten seeds (also with the arc about a point 3 cm off the hinge
  line); held with the palm toward the door, the arc stopped after 12-15 degrees with the hand's
  twist at the end of its travel.
- To shut a door, swing it back with the arc while the hand still holds the handle (the arc stops
  where the door meets its frame), or let go, look where the door is, and push its outer face with
  the fingertips: palm down, the fingers toward the door (left hand `normal=(-0.4, -0.23, -0.89),
  finger=(0.85, -0.46, -0.26)`, as the demonstrations push), the middle fingertip on the face about
  9 cm in from the free edge and in the upper half, then the arc about the hinge to a few degrees past
  shut. A door shut by pushing needs the hand near its free edge, not near the hinge.

Placing and letting go:
- Move what is held above the spot, lower it until it is a centimetre or two over the surface, open
  the fingers (`robot.fingers(side, 'open')`), then move the hand up and away along the palm's
  normal.
- A ring onto a post: bring its hole over the post's axis (within a few millimetres; look from
  above) with its bottom a centimetre or two above the post's top, then lower it slowly
  (`by=(0, 0, -0.10)`, `speed=0.04`). Held as in step 3, a ring hangs tilted about 15-20 degrees
  in the fingers; with a tight hole (the small Hanoi disk: 3 mm each side of the post) it slides
  2-3 cm onto the post and jams there, and the motion stops BLOCKED. Let go there (open the
  fingers, then move the hand straight up): the ring slides down and seats. To tell this from a
  ring that stopped on the post's top, note `robot.state().right.palm[2]` when the fingers close
  on the ring: held so, the ring's bottom rises about 1 cm more than the palm. With the ring
  gripped lying on the base (18 cm below the post's top), on each of fifteen seeds where the hand
  let go at the first stop the palm had risen 13.4-16.1 cm since the grip, and the ring seated; on
  the one seed where the ring had stopped on the post's top the palm had risen 17.3 cm, and let go
  there the ring fell to the table. So at 16 cm or less let go; from 17 cm on, lift it 1 cm, look
  from above, move it a few millimetres toward the post's axis and lower again. Do not lift a ring
  that is partly on: lifted, it came off the post, and on four of nine seeds the next tries missed
  the post by a centimetre or so and let the ring go beside it, where it fell to the table and
  ended the episode.

Using the budget well:
- Thinking, looking and measuring cost nothing; motions do. Put several motions in one cell when the
  code checks each result before the next (`if not m.ok: ...`), and avoid motions that only undo
  each other.
- Typical costs (one-arm tasks; the two-arm tasks' softer arms take about a third longer): a 30 cm
  move 45-60 steps; a grasp motion with an approach 100-140 steps; closing or opening the fingers
  12-20 steps; a 12 cm lift at 0.1 m/s about 26 steps; a wrist camera view 50-80 steps; home 50-70
  steps. The budget (1000-2000 steps) holds some fifteen to thirty such motions: there is room for a
  second attempt, not for many.
