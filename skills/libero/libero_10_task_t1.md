# libero_10_task_t1

## Judging
- What counted: with both of these in the basket the episode ended with success (7 eps).
  "alphabet soup": the blue-label can in the left part of the agentview picture (-y), from above
  about 0.39-0.44 ahead, 0.13-0.17 to -y, 6 cm wide, top 0.096; the tall orange bottle stands on
  its -x side (toward the robot), the milk carton on its +x side (toward the camera). "butter":
  the small flat orange-and-white box about 0.54-0.58 ahead, 0.03-0.07 to +y, 7.2 × 3.4 cm, top
  0.035, on the camera side (+x) of the red-and-green can.
- The red-and-green can a little right of the picture's centre (+y; about 0.40-0.45 ahead) never
  counted as the soup: 3 episodes put it and the box in the basket and none ended there.
- The words 'alphabet soup' measured the milk carton (10 eps) and 'butter' found nothing (10 eps):
  point at both.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` over the cans to read the
   labels; `s = robot.measure(('agentview', (x, y)))` on the blue can, `b = ...` on the box. From
   above the can measured about 0.39-0.44 ahead, 0.13-0.17 to -y (8 eps); the box 0.54-0.58
   ahead, 0.03-0.07 to +y, grip_yaw 0 (10 eps).
2. Can: `robot.goto((s.center[0], s.center[1], 0.29), pitch=90, yaw=0, grip='open')`,
   `s = robot.measure(s)`, straight down to about 0.07, close: 6.3 cm, holding; straight up to
   0.26-0.29 (7 eps).
3. Across at 0.27-0.30 to about 0.47-0.52 ahead, 0.25-0.28 to +y, lower to 0.12-0.135, open,
   straight up (7 eps).
4. Box: `robot.goto((b.center[0], b.center[1], 0.22), pitch=90, yaw=0)`, down to 0.03-0.04, close:
   3.9 cm (4 eps); up to 0.23-0.28, across, lower over the basket: the episode ended
   with success during that lowering or the open (7 eps).

## Verified parameters
- Basket floor from above: about 0.49-0.52 ahead, 0.26-0.27 to +y (5 eps, 2 in other variants).
- Either order works: can first in 4 episodes, box first in 3 (7 eps).
- Both cans close at 6.2-6.3 cm: the width does not tell them apart (10 eps).

## Common failures and fixes
- The red-and-green can taken as the soup: in the basket at 138-168, the box at 299-348, and the
  steps ran out before the blue can was in (3 eps).
- Asking for 0.020-0.022 on the box left the hand at 0.026 and the close printed "~8.0 cm" with a
  warning; the lift then read 3.9 cm and carried it (6 eps). Close at 0.03-0.04 instead: 3.9 cm at
  once (4 eps).

## Typical step costs
- Horizon 520. Successes 321-365 (7 eps). Can first: approach 23, down 31, close 10, up 27,
  across 40, down 22, open 13, up 23, to the box 31, down 31, close 10, up 28, across 29, lower:
  332 (1 ep).
