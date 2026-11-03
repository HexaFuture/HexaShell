# libero_spatial_task_t1

## Judging
- In our episodes the bowl that counted measured x 0.77-0.805, y -0.06 to -0.08, top 0.037
  (8 eps); the plate x 0.70-0.73, y +0.19 to +0.21 (10 eps). Success came with that bowl set
  down at the plate's centre (8 eps).

## Flow
1. Measure bowl and plate (free). Measure the bowl again once the wrist sees it: the first
   measure's x read up to 1.5 cm too small (10 eps).
2. The -x rim (nearer the robot), turning and opening on the way, low: `robot.goto((c[0] - 0.04,
   c[1], 0.07), pitch=90, yaw=90, grip='open')`, c = bowl centre (54-56 steps) (2 probes).
3. Straight down to 0.024 (11-16 steps from 0.07), close (12), lift 11 cm with
   `robot.goto(by=(0, 0, 0.11))` (18-31) (7 eps, 2 probes).
4. Carry 13 cm up to the plate's centre with x - 0.04 (the grasp's offset), then straight down
   to 0.04 and open; success came at the end of that descent or within 2 steps of opening
   (6 eps, 2 probes).

## Verified parameters
- Yaw 90 closes the fingers along x on the bowl's -x wall: hand 3.7-4.1 cm to -x of its centre,
  at the centre's y, 0.025-0.030 up; closes at about 0.8 cm, holding (7 eps).
- A rim at yaw 0 is at the edge of reach: the -y rim was refused as stretched at pitch 90
  (2 eps), the +y rim too with the bowl at x 0.80 (1 ep).

## Common failures and fixes
- The -y rim at pitch 60-65, or a slanting approach at 90: BLOCKED 15-16 cm up (3 eps). After
  that, moves at pitch 35-65 to the -y or +y rim kept being BLOCKED 16-19 cm up: 37 and 146
  steps lost, one run out of steps with the bowl lifted, one ended with it on the table
  (2 eps). Take the -x rim at yaw 90 from the start (2 probes, 154 and 163 steps).
- Approaching 15-20 cm above the bowl and descending from there cost 29-48 steps down instead
  of 11-16 (6 eps, 2 probes).

## Typical step costs
- Horizon 220. Probes 154-163: approach 54-56, down 11-16, close 12-13, lift 22-26, carry 34-35,
  down 17, open 2 (2 probes). Learning-round successes 165-206 (8 eps).
