# 2. The world and the robot

Units are metres and degrees. One frame for everything, the **start frame**: the base's place at
the start of the episode - origin on the floor under the base, x forward (the way the base faces at
the start), y to its left, z up. It stays fixed while the base drives; `robot.state()` gives where
the base is now and its heading (0 = facing +x, 90 = facing +y).

The robot:
- A wheeled base that drives forward, sideways and turns on the spot (`robot.drive`). The arm
  stands on its front part, on a fixed column, and moves with it. Seen from above the base is a
  box reaching 0.15 m ahead of its position (the point `robot.state()` gives), 0.55 m behind it
  and 0.25 m to each side. It turns about a point 0.21 m behind its position, so a turn swings the
  rear sideways: turning left swings the rear corners to the right, turning right to the left.
- Three cameras: **left** and **right** ride on the base, high up behind it on its left and right,
  looking ahead and down over the arm at what is in front of the robot (they move with the base);
  the arm hides part of their view. **wrist** sits on the hand, looking where the fingers point.
  Pictures are 512 x 512. Every camera also gives a depth picture, so outlines measure in 3-D
  directly (section 4).
- Pixels are (x, y) = (column, row) from the top-left corner. Every tool that takes or gives
  pixels uses this order.

The grasp point and the gripper's angles:
- Motions drive the arm's **grasp point**: the middle between the finger pads. The fingertips
  reach about 1 cm beyond it, so pointing straight down it cannot go lower than about 1 cm above
  the surface under it.
- Orientation is three angles. `pitch`: 0 points level, 90 straight down, -90 straight up. `yaw`:
  the heading of the pointing direction, 0 along +x (forward at the start), +90 along +y (to the
  left). `roll` turns the fingers about the pointing direction, + clockwise seen from behind the
  hand. At roll 0 the fingers close along the horizontal line across the pointing direction.
- Pointing straight down, `yaw` alone sets how the fingers lie: at yaw 0 they close along y, at
  yaw 90 along x; in general across the heading `yaw`. Within 10 degrees of straight down, results
  print the angles this way: heading as `yaw`, `roll` 0.
- A pitch past 90 tips the pointing direction on over, back toward where the yaw comes from (pitch
  120 = 30 degrees past straight down). The same hand can be written two ways (pitch p, yaw y, roll
  r is pitch 180 - p, yaw y + 180, roll r + 180): results print the way nearer what you asked.
- The task card gives the start pose. Angles you leave out of a motion keep the ones you last
  asked for (after `home()`, the start pose's pitch and yaw; after a turn of the base, turned with
  it), not where a contact or a reach limit left the hand. `roll` stays 0 until you ask for another.

Reach: the arm reaches from its shoulder, which rides on the base. What it can reach depends on
where the base stands: when a target is out of reach, drive the base closer (or turn it to face
the target) rather than stretching. `robot.goto(..., dry_run=True)` says whether a pose is
reachable from where the base is, without moving anything.
