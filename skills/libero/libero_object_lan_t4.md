# libero_object_lan_t4

## Judging
- The bottle that counted: the tall orange bottle with the light grey cap (label "Tomato
  Ketchup") in the left part of the agentview picture (-y), at mid-height: about 0.50 ahead,
  0.24 to -y from agentview, 14-15 cm tall; the blue-label can stands between it and the camera
  (+x of it). Lowered into the basket, it ended the episode with success (3 probes).
- The words 'ketchup' measured the small dark red-brown bottle a little left of and below the
  picture's centre (about 0.65-0.67 ahead, 0.09-0.11 to -y, top 0.112) first (9 eps). It never
  counted: all 10 episodes placed it and none ended there.
- The orange bottle showed up only as another 'ketchup' seen, at about 0.50 ahead, 0.24 to -y,
  score 0.42-0.54 (3 eps): choose it by its mask or pixel, not by the best match.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` on the left part of the
   picture (-y) to read the label (untested); `m = robot.find('agentview', at=(x, y))[0]` on the
   orange bottle, check `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`: about
   0.50 ahead, 0.24 to -y (10 eps). From agentview its top read only 0.083-0.085 (10 eps); from
   above this bottle measures 0.147 (23 eps in other variants).
2. `robot.goto((c.center[0], c.center[1], 0.22), pitch=90, yaw=0, grip='open')`, straight down
   to 0.12 (by its neck), close (probe); aiming 1.5 cm to -x of that centre worked too (probe).
3. Straight up to 0.30, across at 0.30 to above the basket's measured centre, straight down
   toward 0.16: the episode ended with success during that lowering, at about 0.18 (2 probes).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.59-0.62 ahead, 0.25-0.27 to +y
  (10 eps).
- Grasped by its neck from above at 0.11-0.13, this bottle closes at 3.4 cm (10 eps in the swap
  variant).
- From above, this bottle's centre measured 0.7-1.7 cm farther to -x (toward the robot) than
  from agentview alone (23 eps in other variants); the neck grasp still held (2 probes).

## Common failures and fixes
- Small dark bottle placed first: in the basket at 151-167 steps, no success (10 eps). The late
  tries at the orange bottle went in one move from the basket to its agentview centre, 53-64
  steps, and ended at 253-280 with no hold (8 eps). Hover 0.22 above it, then straight down
  (2 probes).

## Typical step costs
- Horizon 280. Probed path 149: approach 29, down 17, close 10, up 25, across 55, down 13
  (2 probes).
