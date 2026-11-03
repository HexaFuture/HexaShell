# libero_object_lan_t7

## Judging
- The milk carton: about 0.48-0.50 ahead, 0.23-0.24 to -y, about 5 cm square, top 0.128-0.134.
  Lowered into the basket while still held, it ended the episode with SUCCESS (19 eps, 1 probe).
- Let go from 0.177 over the basket, it did not count (1 ep).

## Flow
1. Free: `m = robot.measure('milk carton')` or point at it, `b = robot.measure('basket')`: top
   0.140, about 15 cm across, 0.60-0.62 ahead, 0.25-0.27 to +y (19 eps).
2. Yaw: the agentview-only measure's grip_yaw, -74 to -76 (11 eps); yaw 0 or 15 also held
   (6 eps). The two-view measure from the start pose said -39 or -40 in 5 runs; measured again
   from above, the same carton said -74 to -76 (5 eps).
3. `robot.goto((m.center[0], m.center[1], 0.23), pitch=90, yaw=-75, grip='open')`, down to
   0.095, close: reached 0.095-0.105, held 5.3-5.9 cm (17 eps, 1 probe). Straight up to 0.29.
4. Over the basket's measured centre at 0.28-0.32, then down to 0.14: SUCCESS came during the
   lowering, before any opening (19 eps, 1 probe).

## Verified parameters
- Yaw -75, 0 or 15: the descent reached 0.095-0.105 and the carton came along (17 eps, 1 probe).
- Yaw -39/-40: the descent stopped short at 0.115-0.116 and closed on 6.5-7.2 cm; the carton
  came along in 2 of 3 (3 eps).

## Common failures and fixes
- Gripped at yaw -40: the carton slipped out on the lift; the regrasp at yaw 0 held 5.5 cm but
  86 steps were gone; let go from 0.177 over the basket: no SUCCESS, steps ran out (1 ep). Grip
  at -75 or 0 (17 eps, 1 probe) and lower it, held, to 0.14 (19 eps, 1 probe).

## Typical step costs
- Horizon 280. Above 28-41, down 21-30, close 10, up 23-28, over the basket 53-57 (the longest
  leg), down 8-25: 152-184 (19 eps); 175 at yaw -75 (1 probe).
