# libero_object_t1

## Judging
- The cream cheese that counted is the small flat box, white and blue (green in some starts), low
  left of the middle of the agentview picture, about (195, 313): 8.1-8.2 x 4.1-4.2 x 1.5-1.6 cm,
  top 0.015-0.018, grip_yaw 0. 'cream cheese' by name found nothing (130 of 131 calls).
- SUCCESS came while held, 6-24 steps into the lowering over the basket. Not it: the orange flat
  box nearer the camera. No white and blue box, a red box 14-15 cm tall there: take it (untested).

## Flow
1. Free: `c = robot.measure(robot.find('agentview', at=(195, 313))[0])`, check its size;
   `b = robot.measure('basket')`: warned (199 of 214 eps).
2. Skip the look-down b's warning offers: as good, 40 steps sooner (72 vs 70 of 129 probes).
   Look only if "agree only roughly" or x under 0.45.
3. `robot.goto((c.center[0], c.center[1], 0.18), pitch=75, yaw=0, grip='open')`, `c =
   robot.measure(c)`, then straight down, `robot.goto((c.center[0], c.center[1], 0.012))`, and
   `robot.gripper('close')`: holding 4.2 cm (11 of 11 probes; at 0.13 first: 79 of 90 probes).
4. Up to 0.23 at pitch 90, across to `(b.center[0], b.center[1] - 0.19, 0.23)`, `b2 =
   robot.measure(b)`: unwarned and at most 22 cm long, use it, going to its stop (longer: a plate
   joined in, keep b). Look (free). No plate: above b.center, down toward 0.10: SUCCESS on the way
   (72 probes; 141 steps without the stop, 151 with it, 3 probes each).

## Verified parameters
- Straight down at pitch 90 it often stopped "short" at 0.03-0.05 (80 eps); closes there empty.

## Common failures and fixes
- A white round plate on the basket, new since the start: lowering BLOCKED at 0.17 (17 eps);
  moves from on top stayed on it at 0.17-0.19 (12 eps). From the stop down to 0.165, one move +y
  at 0.165 to 0.10 past b.center, up to 0.23 above it, down toward 0.10: SUCCESS at 205-218 (7 of
  8 probes); swept on the line of the 30-33 cm reading instead: 3 of 8 (probes).
- Sent low over the cheese at once from the start: BLOCKED 10 cm past it, then NOT executed (5 eps).
- First reading under x 0.45, top 0.17: from 8 cm nearer the robot at 0.38-0.43 the basket read
  top 0.26-0.28; carried at 0.33-0.34, lowered to 0.215-0.235: SUCCESS (8 eps).
- Lowering BLOCKED at 0.158, 7 cm off: the basket had moved 9 cm. Measured again, or its agentview
  spot at z 0.14; up, across, open at 0.15-0.22: SUCCESS at 243-258 (5 eps).
- Re-read with nothing touched, "moved" 10-11 cm, "on something at z 0.09": off (3 eps).
- "closed on ~7.8 cm ... nearly fully open" (holding True): empty; closing again 30-35 steps on
  read ~8 cm again (3 eps). Up, back down to 0.014-0.022, close: 4.2 cm, SUCCESS at 219-269 (10
  eps). Staying down, `robot.wait(40)`, then the close: 20 steps sooner (untested).
- Read "agree only roughly", 3 x 3 x 4 cm: by its wrist spot; stopped at 0.05, closing held (3 eps).
- The red box: at its grip_yaw, 3 cm under its top: held 4.4-4.5 cm; counted for the pudding (1 ep).

## Typical step costs
- Horizon 280. Above 35-51, down 26-27, close 8, up 28, across 36-50, 6-24: at 148-172 (72 probes).
  With the old note 156-274, median 194 (200 of 215 eps); no note 174-280 (98 eps).
