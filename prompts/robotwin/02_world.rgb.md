# 2. The world and the robot

Units are metres and degrees. One world frame for everything:
- x runs across the table to the robot's right, y away from the robot, z up.
- The table is about 1.2 m wide and 0.7 m deep; its near edge runs along y = -0.35, just in front
  of the arms. Its height changes by a few centimetres from one episode to the next: the task card
  gives the table top's height, measured in the start pictures (about z = 0.71 to 0.74).
- The two arms stand side by side behind the table's near edge, their bases at about
  (-0.30, -0.42) (left arm) and (+0.31, -0.42) (right arm), at z = 0.78 - a few centimetres above
  the table top.
- The head camera is fixed at (-0.03, -0.45, 1.35), between and above the arm bases, looking
  forward and 53 degrees down. In its picture +x is to the right and +y is up the picture
  (farther away); the left arm is on the left. It sees the table from about y = -0.25 to the far
  edge; the hands at their start pose are just below the picture.
- Each arm carries a wrist camera on top of its hand, looking along the fingers and a little down,
  past their tips. Pointing straight down at yaw 0, its picture is turned like the head camera's
  (+x to the right, +y up the picture) and shows what lies under the fingers.
- Pictures are 320 x 240 pixels. Pixels are (x, y) = (column, row) from the top-left corner. Every
  tool that takes or gives pixels uses this order. There are no depth pictures: positions come from
  the colour pictures and the camera models (section 4).

The grasp point and the gripper's angles:
- Motions drive an arm's **grasp point**: the middle between the finger pads, 0.143 m beyond the
  wrist flange. The fingertips reach 1.3 cm beyond it, so pointing straight down the grasp point
  cannot go lower than about 1.5 cm above the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down. `yaw`: the heading of
  the pointing direction, 0 away from the robot (+y), +90 towards the robot's left (-x). `roll`
  turns the fingers about the pointing direction, + clockwise seen from behind the hand: pointing
  level, +roll tips the top of an upright held object to the hand's right (at yaw 0: towards +x).
  Both arms start about level and pointing away, with the grippers open (the task card gives the
  start poses).
- Pointing straight down, `yaw` alone sets how the fingers lie: they close along the line at `yaw`
  degrees from +x, counter-clockwise seen from above (yaw 0: they close along x; yaw 90: along y).
  Within 10 degrees of straight down, results print the angles this way: heading as `yaw`, `roll` 0.
- An open, empty gripper is symmetric: pointing about straight down (pitch 85 or more) with `roll`
  left out, a move may arrive at yaw + 180 instead, the same grasp. Its result then says so and
  gives the new yaw, which later motions keep (the wrist camera then sees upside down). Otherwise
  the hand turns to exactly the angles asked.
- Angles you leave out of a motion keep the ones you last asked for (after `home()`, the start
  pose's), not where a contact or a reach limit left the hand.

Reach, pointing away from the robot (the left arm mirrors the right):

| pointing | the grasp point reaches forward to about, 6 / 16 / 26 cm above the table top |
|---|---|
| straight down (pitch 90) | y = 0.10 / 0.06 / -0.05 |
| pitch 60 | y = 0.23 / 0.20 / 0.14 |
| pitch 45 | y = 0.29 / 0.26 / 0.22 |
| level (pitch 0) | y = 0.35 at any of these heights |

Across, pointing down 5 cm above the table, the right arm reaches x = -0.20 to the table's right
edge at y = -0.30, -0.15 to the edge at y = -0.15, 0.0 to 0.6 at y = 0.0 and 0.2 to 0.4 at
y = 0.10. Each arm works best on its own side; things beyond these reaches (far side of the
table) are out of reach pointing down - tilt the hand, or push or hand the object nearer.
`robot.goto(..., dry_run=True)` says whether a pose is reachable without moving anything.
