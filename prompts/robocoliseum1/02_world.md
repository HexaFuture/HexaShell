# 2. The world and the robot

Units are metres and degrees. One world frame for everything, fixed to the robot's base (the base
does not move during an episode):
- x points forward, away from the robot; y to the robot's left; z up from the floor.
- Most tasks are at a table in front of the robot, its top at about z = 0.73, the objects about
  0.5 to 0.9 m in front of the robot. Other scenes (a kitchen, a shop shelf, a warehouse, a door, a
  laboratory bench) have their own heights and distances: measure them.
- The robot's torso turns at the waist about the vertical (`robot.waist`). It carries both arms and
  the head camera; at the start it faces forward. The world frame does not turn with it.
- The arms hang from shoulders at about (0.19, +0.24, 1.14) (left) and (0.19, -0.24, 1.14) (right)
  while the waist faces forward. Each arm's grasp point reaches up to about 0.84 m from its
  shoulder, less in many directions (the table below).
- The head camera sits at about (0.44, 0.02, 1.40), looking forward and about 59 degrees down: it
  sees the work area in front of the robot from above. In its picture the robot's left (+y) is on
  the left and farther things (+x) are higher up the picture.
- Each gripper carries a wrist camera on the back of the hand, looking along the fingers. Pointing
  straight down at yaw 0, it sits about 9 cm ahead of the grasp point and 8 cm above it, and its
  picture is turned like the head camera's (forward is up the picture, the robot's left on the
  left): it shows what lies under and just ahead of the fingers.
- Pictures are 640 x 400 pixels (head) and 640 x 528 (wrists). Pixels are (x, y) = (column, row)
  from the top-left corner; every tool that takes or gives pixels uses this order. Every camera
  also gives a depth picture (section 4).

The grasp point and the gripper's angles:
- Motions drive an arm's **grasp point**: the middle between the finger pads, 0.148 m beyond the
  wrist's end link. The fingertips reach about 1 cm beyond it, so pointing straight down the grasp
  point cannot go lower than about 1 cm above the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down. `yaw`: the heading of the
  pointing direction, 0 forward (+x), +90 to the robot's left (+y). `roll` turns the fingers about
  the pointing direction. Both hands start a little in front of the shoulders, pointing forward,
  inward and a little down, the grippers open (the task card gives the start poses).
- Pointing straight down, `yaw` alone sets how the fingers lie: at yaw 0 they close along y (left
  and right), at yaw 90 along x (forward and back); in general across the heading. Within 10
  degrees of straight down, results print the angles this way: heading as `yaw`, `roll` 0.
- An open, empty gripper is symmetric: pointing about straight down (pitch 85 or more) with `roll`
  left out, a move may arrive at yaw + 180 instead, the same grasp, when that way is shorter. Its
  result then says so and gives the new yaw; pass that yaw from then on (the wrist picture is then
  upside down). Once that gripper has closed, the yaw first asked is taken as the new one, so what
  it holds is not turned half a turn. Otherwise the hand turns to exactly the angles asked.
- Angles you leave out of a motion keep the ones you last asked for (after `home()`, the start
  pose's), not where a contact or a reach limit left the hand.

Reach of the right arm with the waist facing forward, the grasp point 6 to 26 cm above a table top
at z = 0.73 (the left arm mirrors it: change the sign of y). The farthest x it reaches:

| pointing | y = +0.20 | y = 0 | y = -0.20 | y = -0.40 |
|---|---|---|---|---|
| straight down (pitch 90) | 0.55 | 0.72 | 0.78 | 0.76 |
| pitch 60, heading forward (yaw 0) | - | 0.84 | 0.90 | 0.88 |
| pitch 45, heading forward | - | 0.88 | 0.94 | 0.92 |
| level (pitch 0), heading forward | - | 0.90 | 0.96 | 0.94 |

Pointing straight down it reaches in to about x = 0.20 at y = 0, 0.40 at y = -0.20, 0.32 at
y = -0.40 and 0.30 at y = +0.20; tilted at y = -0.20 only from about x = 0.50 (pitch 60) or 0.56
(pitch 45) outward - nearer points need a steeper pitch. Each arm works best on its own side;
across the middle it reaches less far, and a tilted hand heading forward does not reach the other
side at all. For things farther away, tilt the hand toward them (`yaw` their heading from the
shoulder), or turn the waist. `robot.goto(..., dry_run=True)` says whether a pose is reachable
without moving anything.
