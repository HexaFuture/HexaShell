# 2. The world and the robot

Units are metres and degrees. One frame for everything, the **arm frame**: origin at the arm's
base plate, x forward (from the arm toward the workspace), y to its left, z up. The robot does not
move, so the frame holds for the whole episode. The table (or the floor) the robot works on lies
close to z = 0; measure it rather than assume it.

The robot:
- One arm with a parallel gripper on a fixed base: nothing drives, and what the arm cannot reach
  from where it stands stays out of reach.
- Two cameras. **agentview** stands across the workspace from the robot, facing it and looking down
  at the table: the top of its picture is the robot's side, and the robot's left (+y) is on the
  picture's right. The arm hides what is behind it in that view. **wrist** sits on the hand,
  looking where the fingers point; with the hand pointing straight down at yaw 0, up in its picture
  is +x and right is -y. Pictures are 512 x 512. Both cameras also give a depth picture, so
  outlines measure in 3-D directly (section 4).
- Pixels are (x, y) = (column, row) from the top-left corner. Every tool that takes or gives
  pixels uses this order.
- The instructions say left, right, front and back as a person sees the scene from the agentview
  camera, facing the robot: "left" is the left of the agentview picture, which is the robot's right
  (-y); "right" is +y; "front" is the side nearer that camera (+x), "back" the side nearer the
  robot (-x).

The grasp point and the gripper's angles:
- Motions drive the arm's **grasp point**: the middle between the finger pads. The fingertips
  reach about 1 cm beyond it, so pointing straight down it cannot go lower than about 1 cm above
  the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down, -90 straight up. `yaw`:
  the heading of the pointing direction, 0 along +x (away from the arm), +90 along +y (to the
  left). `roll` turns the fingers about the pointing direction, + clockwise seen from behind the
  hand. At roll 0 the fingers close along the horizontal line across the pointing direction.
- Pointing straight down, `yaw` alone sets how the fingers lie: at yaw 0 they close along y, at
  yaw 90 along x; in general across the heading `yaw`. Within 10 degrees of straight down, results
  print the angles this way: heading as `yaw`, `roll` 0.
- A pitch past 90 tips the pointing direction on over, back toward where the yaw comes from (pitch
  120 = 30 degrees past straight down). The same hand can be written two ways (pitch p, yaw y, roll
  r is pitch 180 - p, yaw y + 180, roll r + 180): results print the way nearer what you asked.
- The task card gives the start pose. Angles you leave out of a motion keep the ones you last
  asked for (after `home()`, the start pose's pitch and yaw), not where a contact or a reach limit
  left the hand. `roll` stays 0 until you ask for another.

Reach: the arm reaches from its shoulder, about a third of a metre above the base plate. Far
corners of the workspace, points close to the base and low points far away are hard to reach, and
far from the shoulder the hand is less exact. `robot.goto(..., dry_run=True)` says whether a pose
is reachable, without moving anything; when one is not, ask for another pitch, height or approach
direction.
