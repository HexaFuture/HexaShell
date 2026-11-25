# 2. The world and the robot

Units are metres and degrees. One world frame for everything, fixed to the robot's base (the base
does not move in these tasks):
- x points forward, away from the robot; y to the robot's left; z up from the floor (on a robot
  whose base is not on the floor, z 0 is at the base: the task card says how high).
- Most tasks are at a table in front of the robot, the objects about 0.4 to 0.9 m in front of it.
  Heights and distances differ from scene to scene: work them out (section 4).
- The torso and the head do not move in these tasks: the shoulders and the head camera stay where
  they are.
- The task card gives where the shoulders are (the arms hang from them), how far each arm's grasp
  point reaches from its shoulder (less in many directions), and where the head camera sits and how
  far down it looks. The head camera sees the work area in front of the robot from above: in its
  picture the robot's left (+y) is on the left and farther things (+x) are higher up the picture.
- Each hand carries a wrist camera at the heel of its palm, looking along the fingers: it shows what
  lies ahead of the fingertips, and under them when the hand points down, the hand's own fingers
  filling one side of the picture (the card says which). It turns with the hand's roll, so the
  scene can appear tilted in it.
- Picture sizes differ between robots and cameras: the task card gives them, and `robot.image` gives
  a picture's size. Pixels are (x, y) = (column, row) from the top-left corner; every tool that
  takes or gives pixels uses this order. The cameras give no depth pictures in these tasks: a pixel
  gives a direction, and a height you know or a second picture gives the point (section 4).

The grasp point and the hand's angles:
- Motions drive an arm's **grasp point**: the working point of the hand's shape - the middle of the
  palm, where a pinch's pads meet, the middle of a wrap's curled fingers, the index fingertip (the
  task card gives each). The fingers reach beyond it (the task card says how far), so pointing
  straight down the grasp point cannot go lower than that above the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down. `yaw`: the heading of the
  pointing direction, 0 forward (+x), +90 to the robot's left (+y). `roll` turns the fingers about
  the pointing direction. The hands start a little in front of the shoulders, pointing forward,
  inward or sideways (the task card gives the start poses).
- Pointing straight down, `yaw` alone sets how the fingers lie: at yaw 0 every shape closes along
  y (left and right) - the left palm facing the robot's right, the right palm its left -, at yaw 90
  along x (forward and back); in general across the heading. Within 10 degrees of straight down,
  results print the angles this way: heading as `yaw`, `roll` 0.
- A hand is not symmetric (the thumb on one side, the curled fingers on the palm's side): it turns
  to exactly the angles asked, and yaw + 180 is the other grasp, the palm facing the other way.
- Angles you leave out of a motion keep the ones you last asked for (after `home()`, the start
  pose's), not where a contact or a reach limit left the hand.

Reach: each arm works best on its own side and in front of its shoulder; across the middle it
reaches less far. Pointing straight down it reaches less far out than tilted toward the point
(`pitch` 45-60, `yaw` the point's heading from the shoulder), and tilted it does not reach in close.
For things farther away, tilt the hand toward them.
`robot.goto(..., dry_run=True)` says whether a pose is reachable and how many steps it takes,
without moving anything (it is not a motion).
