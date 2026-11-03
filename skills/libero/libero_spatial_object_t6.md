# libero_spatial_object_t6

## Judging
- In our episodes the bowl that counted stood beside the cookies box, far out: x 0.775-0.805,
  y -0.055 to -0.085, top 0.037 (20 eps). The plate lies at x 0.70-0.73, y +0.19 to +0.215
  (20 eps). Success came with that bowl lowered onto the plate (18 eps). Another black bowl stood
  on the stove; no run moved it.

## Flow
1. Measure bowl and plate (free) (20 eps).
2. Turn the hand during the first, long move and stop low over the -x rim:
   `robot.goto((c[0] - 0.04, c[1], 0.07), pitch=90, yaw=90, grip='open')`, c = bowl centre
   (50-63 steps) (2 probes; 30 eps with this bowl in the lan, task and swap scenes).
3. Measure the bowl again (free; its x came out 0.2-0.7 cm larger) (20 eps), straight down to
   0.024 (16 steps; 10-18 in those scenes), close (13) (2 probes, 30 eps).
4. Lift 11 cm with `robot.goto(by=(0, 0, 0.11))` (27-30 steps), carry 13 cm up to the plate's
   centre with x - 0.04 (35-38), straight down to 0.04 (16), open: success came within 3 steps
   (2 probes).

## Verified parameters
- Yaw 90 put the hand 3.7-4.3 cm to -x of the bowl's centre, at its y, 0.021-0.030 up, holding
  (18 eps, 2 probes).

## Common failures and fixes
- Yaw 0 on the first move and the turn to yaw 90 on the way down: that descent cost 49-56 steps
  with the grasp at x 0.735-0.74 and 67-82 at x 0.74-0.76 (7 eps). Once the steps then ran out
  with the bowl held over the plate (1 ep). With yaw 90 set in the first move the descent cost
  24-37 from 0.145-0.19 up (11 eps), 16 from 0.07 (2 probes).
- Yaw 0 grasps: one at the bowl's centre was refused as stretched, pitch-75 rims closed empty or
  stopped short, a pitch-65 rim held too late and the steps ran out (1 ep); a pitch-60 +y rim
  held, done on step 215 (1 ep). Take the -x rim at yaw 90 as in step 2 (18 eps, 2 probes).

## Typical step costs
- Horizon 220. Probes 165 and 179: approach 56-63, down 16, close 13, lift 27-30, carry 35-38,
  down 16, open 2-3 (2 probes).
- Yaw 90 from the first move: 164-192 in all (11 eps); turned on the way down: 163-209, and one
  ran out (7 eps). Steps 2-4 with this bowl in the other scenes: 140-177 (30 eps).
