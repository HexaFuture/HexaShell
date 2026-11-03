# libero_spatial_swap_t3

## Judging
- In our episodes the bowl that counted sat on the cookies box: x 0.715-0.74, y +0.015 to
  +0.035, top 0.056 (5 eps). The plate lies at x 0.675-0.705, y -0.25 to -0.285 (10 eps).
  Success came with that bowl set down at the plate's centre (5 eps).
- Another black bowl sits on the wooden cabinet's top, near y +0.205, top 0.265. Once the box
  looked empty; that cabinet bowl, set on the plate's centre, did not count (1 ep).

## Flow
1. Measure bowl and plate from the start pictures (free), then the bowl again from above it
   (9 eps).
2. One move to 10 cm above the bowl's -y rim, opening on the way (36-38 steps):
   `robot.goto((c[0], c[1] - 0.042, 0.15), pitch=90, yaw=0, grip='open')`, c = bowl centre
   (9 probes).
3. Straight down to 1.3 cm below its top, about 0.043 (18-21 steps); close (12); lift 8 cm with
   `robot.goto(by=(0, 0, 0.08))` (16-19) (9 probes).
4. Carry 12 cm up to the plate's centre with y - 0.045 (the grasp's offset), then straight down
   to 0.045 and open; success came during that descent, before opening (9 probes).

## Verified parameters
- From the start pose the bowl measured 8.8-9.3 cm wide, not round, its x 1.4-1.9 cm too small;
  from above it, round 9.6 cm. Use the measure from above (9 eps).
- Grasp: hand 4.2 cm to -y of the bowl's centre, 1.3 cm below its top, pointing down at yaw 0;
  the place pose keeps that offset (9 probes).

## Common failures and fixes
- A slanting move from 18-21 cm above the bowl down to its rim was BLOCKED after 9-24 steps,
  14-15 cm from its target and still 16-18 cm up (6 eps). Retries at pitch 55-60 worked twice,
  but only 13-18 steps remained (2 eps). Come straight down above the rim (2 eps, 9 probes).
- Yaw 90 on the -x rim (nearer the robot): the hand stopped 1.2-2.3 cm high, "short of target",
  and the grip read empty (5 eps); 4 of those 5 runs failed. Use yaw 0 on the -y rim (9 probes).

## Typical step costs
- Horizon 220. Probed path 129-160: approach 36-38, down 18-21, close 12, lift 16-19, carry
  34-58, down 10-17 (9 probes). Learning-round successes took 156-209 (5 eps).
