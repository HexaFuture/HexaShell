# 2. The world and the robot

Units are metres and degrees. One world frame for everything, fixed to the robot's base (on most
tasks the base does not move; where it drives, see below):
- x points forward, away from the robot; y to the robot's left; z up from the floor (on a robot
  whose base is not on the floor, z 0 is at the base: the task card says how high).
- Most tasks are at a table in front of the robot, the objects about 0.4 to 0.9 m in front of it.
  Heights and distances differ from scene to scene: measure them.
- On some robots the torso turns at the waist about the vertical (`robot.waist`; the task card says
  whether it does). It carries both arms and the head camera; at the start it faces forward. The
  world frame does not turn with it.
- Where the base drives on wheels (`robot.drive`; the task card says whether it does), the world
  frame moves with the base: after a drive, every point and Thing from before is off by the drive -
  look, find and measure again before reaching. Nothing reads where the base got to: judge it from
  the pictures. The arms ride along as they are, so a hand held out meets walls, door frames and
  furniture first.
- Where the task card lists the torso and the head (`robot.body`, `robot.head`), the torso leans
  forward and goes down or up, carrying both arms and the head camera: the hands move with it, down
  and forward, so lift a low hand before leaning further. The space right under a far-leaning torso
  is the robot's own: a hand brought back under it meets the body or the legs, so reach the floor
  ahead of the shoulders or out to the side. The head turns and tips on the torso; the more the
  torso leans, the more steeply the head camera looks down (past straight down it looks back toward
  the base).
- The task card gives where the shoulders are (the arms hang from them), how far each arm's grasp
  point reaches from its shoulder (less in many directions), and where the head camera sits and how
  far down it looks. The head camera sees the work area in front of the robot from above: in its
  picture the robot's left (+y) is on the left and farther things (+x) are higher up the picture.
- Each gripper carries a wrist camera on the back of the hand, looking along the fingers: it shows
  what lies under and just ahead of the fingers when the hand points down, the fingers themselves
  at its edge (the card says how much of it they fill).
- Picture sizes differ between robots and cameras: the task card gives them, and `robot.image` gives
  a picture's size. Pixels are (x, y) = (column, row) from the top-left corner; every tool that
  takes or gives pixels uses this order. Every camera also gives a depth picture (section 4).

The grasp point and the gripper's angles:
- Motions drive an arm's **grasp point**: the middle between the finger pads. The fingertips reach
  beyond it (the task card says how far for this robot's grippers), so pointing straight down the
  grasp point cannot go lower than that above the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down. `yaw`: the heading of the
  pointing direction, 0 forward (+x), +90 to the robot's left (+y). `roll` turns the fingers about
  the pointing direction. The hands start a little in front of the shoulders, pointing forward,
  inward or sideways (the task card gives the start poses).
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

Reach: each arm works best on its own side and in front of its shoulder; across the middle it
reaches less far. Pointing straight down it reaches less far out than tilted toward the point
(`pitch` 45-60, `yaw` the point's heading from the shoulder), and tilted it does not reach in close.
For things farther away, tilt the hand toward them or, where the torso turns, turn the waist.
`robot.goto(..., dry_run=True)` says whether a pose is reachable and how many steps it takes,
without moving anything (it is not a motion).
