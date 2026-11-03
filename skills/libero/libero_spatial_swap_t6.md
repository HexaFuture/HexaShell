# libero_spatial_swap_t6

## Judging
- In our episodes the bowl that counted measured x 0.78-0.80, y -0.06 to -0.085, top 0.037
  (8 eps): "next to the cookies box" here meant the box's -y side, a little to +x (box near
  y +0.02 to +0.04, 3 eps). Another black bowl stood near x 0.70-0.72, y +0.20 to +0.215
  (4 eps); no run moved it.
- The plate lies on the stove: x 0.38-0.41, y -0.13 to -0.16, its top 0.036 (7 eps). Success
  came with the bowl set down at the plate's centre (8 eps).

## Flow
1. Measure bowl and plate (free). Measure the bowl again once the wrist sees it: the first
   measure's x read up to 1.4 cm too small (10 eps).
2. The -x rim (nearer the robot), turning and opening on the way, low: `robot.goto((c[0] - 0.04,
   c[1], 0.07), pitch=90, yaw=90, grip='open')`, c = bowl centre (54-61 steps) (2 probes).
3. Straight down to 0.024 (15-16 steps from 0.07), close (12-13), lift 12 cm with
   `robot.goto(by=(0, 0, 0.12))` (24-31) (7 eps, 2 probes).
4. Carry level at 0.15 to the plate's centre with x - 0.04 (the grasp's offset) (42-49 steps),
   then straight down toward 0.066. Success came during that descent (6 eps); the bowl sat on
   the plate with the hand stopped near 0.079, "short of target" (2 probes).

## Verified parameters
- Yaw 90 closes the fingers along x on the bowl's -x wall: hand 3.8-4.3 cm to -x of its centre,
  at the centre's y, 0.024-0.031 up; closes at about 0.8 cm, holding (7 eps).
- The -y rim at yaw 0 is at the edge of reach: refused as stretched at pitch 90 (5 eps).
- During the carry a move may note "the fingers closed fully: what they held slipped out" while
  the bowl still hangs in the hand: go on with the place (4 eps, all successes).

## Common failures and fixes
- One slanting move from the lift (16 cm) down to the low place pose (43 steps), then open: no
  success (1 ep). A bowl hanging low on a slanting path can catch the plate's rim. Carry level
  at 0.15, then come straight down (2 probes, 163 and 173 steps).
- The -y rim at pitch 60-70: BLOCKED 16 cm up (2 eps). Yaw-90 tries from that stuck pose were
  BLOCKED too; the bowl was grasped with 38 steps to go, too few for the carry (1 ep). Take the
  -x rim at yaw 90 from the start (2 probes).

## Typical step costs
- Horizon 220. Probes 163-173: approach 54-61, down 15-16, close 12-13, lift 28-31, carry 43-45,
  down 9 (2 probes). Learning-round successes 165-203 (8 eps).
