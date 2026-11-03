# libero_10_swap_t7

## Judging
- What counted: with both in the basket the episode ended with success (1 ep, 4 probes).
  "alphabet soup": the can with the blue label and yellow print in the left part of the
  agentview picture (-y), at mid-height: about 0.30-0.33 ahead, 0.13-0.15 to -y, 6 cm wide, top
  0.096; the tall orange bottle stands between it and the camera (+x of it). "cream cheese box":
  the flat blue-and-white box at the lower left of the picture (+x, -y), 7.3 × 3.8 cm, top 0.035.
- The red-and-green can a little right of the picture's centre (+y; about 0.34-0.38 ahead) never
  counted as the soup: 9 episodes put it and the box in the basket, and none ended there.
- 'alphabet soup' found nothing (10 eps): point at the blue can. 'cream cheese box' found the box.

## Flow
1. Before any motion (free): `s = robot.measure(('agentview', (x, y)))` on the blue can: about
   0.30-0.33 ahead, 0.13-0.15 to -y (3 eps). `b = robot.measure('cream cheese box')`: 0.60-0.64
   ahead, 0.17-0.22 to -y, grip_yaw 0 (10 eps).
2. Can, with the hand tipped back toward the robot: `robot.goto((s.center[0], s.center[1], 0.16),
   pitch=120, yaw=0, grip='open')`, straight down to 0.065 at pitch 120, close, straight up to 0.25
   at pitch 120 (2 probes).
3. Across at 0.25 with pitch 90 to the basket's measured centre, down to 0.13, open, up to 0.25
   (2 probes). A whole basket measured about 14-17 × 12-16 cm, centred near 0.50-0.52 ahead
   (6 eps); one only about 8 cm deep, centred near 0.54-0.56 ahead, covers just its camera side
   (8 eps): then use the floor measured from above (below).
4. Box: across at 0.25 to above it, straight down to 0.035 at yaw 0, close, up to 0.25, across at
   0.25, lower toward 0.15: the episode ended with success during that lowering (2 probes).

## Verified parameters
- The box closes at 4.2-4.3 cm with the hand at 0.03 (10 eps).
- Basket floor from above: about 0.49-0.52 ahead, 0.26-0.27 to +y (5 eps, 4 in other variants).

## Common failures and fixes
- The red-and-green can placed as the soup (9 eps); 2 of them reached the blue can only at 468-477,
  too late.
- At pitch 90 the descent to the blue can ended "short", 1.3-1.7 cm nearer the robot than asked,
  and every lift with it stopped BLOCKED at 0.09-0.14 (1 ep, 3 probes), "too close in for the
  arm" (1 ep). The one success got free by a move at pitch 65, then home, and dropped it in at
  503 (1 ep). Use pitch 120 (2 probes).
- A low straight move from the start to 0.12 over the box ended "short" at 0.157, and the
  descent from there stopped BLOCKED at 0.149 (probe). Go over at 0.25 first, then straight down
  (2 probes).

## Typical step costs
- Horizon 520. Probed: can 30-31, down 21, close 10, up 28, across 44-46, down 19-23, open 12-13,
  up 18-20, to the box 45-47, down 28, close 10, up 29, across 44-47, down 11: 351-362
  (2 probes). The one success took 503 (1 ep).
