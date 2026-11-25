# bimanual_photograph

## Judging
- The card's three conditions must hold at one step. Done as below, the checked point was well
  inside its range and the lens about 7-8 degrees off +x. The episode ended during the press move
  (L1-L10).
- What decides the episode is the left hand's first grasp of the camera. With the four-digit hold
  of step 3 the camera stayed in the hand to the end in every run; with weaker holds it slipped out
  on the lift, the aim, or even later while both hands were still.

## Flow
1. Use the start pictures only; no wrist views were needed (L1-L10).
   - `cam = robot.measure('camera')`.
   - The disc is the only near-black thing in the upper part of the scene picture. The median of the
     pixels with R+G+B below 150 in rows 30-170 gave its middle, and `robot.to_world('scene', (u, v))`
     there gave its centre (dx, dy, dz) (L1-L10).
   - Measured from the start pictures, the camera's centre reads about 1.3 cm nearer the robot and
     1.3 cm further toward -y than measured from close by. The offsets below are for the
     start-picture measure.
2. Grasp with the left hand in the demonstrations' camera hold, `normal=(-0.72, -0.26, -0.64),
   finger=(0.67, -0.5, -0.55)`. The grasp point is `g = (cx - 0.005, cy + 0.04, cam.top - 0.025)`:
   0.5 cm nearer the robot than the measured centre, 4 cm toward +y, 2.5 cm below the top.
   - First move the grasp point 18 cm straight above g in that orientation (`speed=0.18`).
   - Then `robot.hand('left', g, point='grasp', normal=..., finger=..., approach=0.08)`.
   - Then `robot.fingers('left', 'wrap', seconds=0.6)`.
   On L1-L10 the wrap pressed with thumb, middle, ring and little, and the camera stayed in the
   hand to the end.
3. Before lifting, check `robot.state().left.pressing`. It should list thumb, middle, ring and
   little. Across the scripted runs on the learning seeds (82 episodes, many of them repeats of the
   same grasp; the fingers left at 'wrap'):
   - every hold with those four stayed in the hand to the end (57 of 57);
   - holds with fewer or other digits stayed in only 7 of 25.
4. Lift 13 cm: `robot.hand('left', by=(0, 0, 0.13), speed=0.08)`.
5. Aim in one move. Send the grasp point to `(dx - 0.52, dy + 0.06, dz + 0.05)`, 52 cm in front of
   the disc and 5 cm above its centre, with the demonstrations' aiming hold
   `normal=(-0.006, -0.763, -0.646), finger=(-0.05, -0.645, 0.763)` and `speed=0.10`.
   - With the point at the disc's own height, the move stopped BLOCKED at the hand's twist limit
     (joint 7) on the two seeds with the lowest discs, 11 degrees short of the turn (L9, L10).
   - 5 cm higher it was reached on all ten.
6. Press with the right hand.
   - `robot.fingers('right', 'point')`.
   - Find the shutter from the left hand's state, without looking: from `palm`, about 13 cm along
     `normal` (n), 19 cm along `finger` (f) and 1.5 cm along n x f. As code:
     `cap = palm + 0.13*n + 0.19*f + 0.015*np.cross(n, f)`. With 12 cm and 2 cm instead, the press
     missed the shutter on L6.
   - Move the index (`point='index'`) to 4 cm above `cap`, palm down and fingers forward as the
     demonstrations press: `normal=(0.06, 0.073, -0.995), finger=(0.907, 0.413, 0.085)`,
     `speed=0.10`.
   - Then press with `by=(0, 0, -0.05)` at `speed=0.04`. The fingertip meets the camera's top and
     the episode ended during this move (L1-L10).

## Verified parameters
These were measured on L6, L8 and L10 (and all ten where noted), with the start-picture measure:
- 1 cm lower than step 2's point: the camera slipped out on the lift (3/3).
- 1 cm toward -y (3 cm toward +y instead of 4): slipped on the lift or the aim, or the fingers
  closed on nothing (3/3).
- At the prompt's offsets taken from the start-picture measure (1.5 cm nearer the robot than the
  centre): 9 of 10. On L10 the camera, held by thumb, middle and little only, slipped out about 45
  steps after the aim, with both hands still.
- At the point the prompt's offsets give from a close-by measure (start centre + 2.7 cm in y, as the
  models grasped): 3 of 4 slipped on the lift (L6, L8, L10; L9 held).
- 1 cm higher: held, but on one of three the press point of step 6 missed the shutter (L8). The
  shutter sits differently in a hand that grips higher.
- Aim speed 0.05 instead of 0.10 made no difference, and turning the hand in place before the aim
  cost steps and gained nothing (L1-L10, with the older grasp point).

## Common failures and fixes
- Squeezing harder after the wrap (curl 0.9 on each finger, thumb 0.6) pushed the camera out of the
  hand on all ten seeds (L1-L10). Leave the fingers at 'wrap'.
- A weak first grasp (no ring finger pressing) already shifts and tips the camera: measured again it
  had moved about 2 cm. Opening, backing off 8 cm along the palm's normal, rising 8 cm, measuring
  again and grasping at step 2's point gave four digits on 3 of 6 seeds and success on 2 of 6. A
  second grasp is a poor rescue, so make the first one as in step 2.
- If the camera slips out after all, it falls onto the table. Look before trying again; the steps
  left after a slip were 650 or more.

## Final checks
- Before the press: `robot.state().left.holding` is True and the four digits still press.
- Nothing else is needed: the episode ends by itself on success.

## Typical step costs
- Grasp 108-109 (two moves and the wrap), lift 34-36, aim 70-77, 'point' 15, the right hand's move
  64-81, the press about 25-30. Done at steps 321-345 of the 1000 (L1-L10).
- The models' own successful runs took 372-661 steps.
