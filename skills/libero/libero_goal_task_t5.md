# libero_goal_task_t5

## Judging
- The cream cheese: the flat box right of and below the middle of the agentview picture. Pointed
  at, two views: 0.59-0.63 ahead, 0.11-0.15 to +y, 7.5-7.9 x 3.8-3.9 cm, long side along x, top
  0.007 (20 eps). 'cream cheese' found nothing in all 20 episodes: point at it.
- `robot.measure('stove')`: round, top 0.019, 0.40-0.42 ahead, 0.20-0.22 to +y (20 eps).
- SUCCESS came while the closed hand, on the cheese's -y side, pushed it toward +y (18 eps,
  2 probes).
- Pushed toward -x, the cheese re-measured 0.56 ahead and 0.15 to +y, then 0.55 ahead and 0.20
  to +y: neither counted; a push from its -x side toward +x then gave SUCCESS (1 ep).

## Flow
1. Free: point at the cheese, `c = robot.measure(('agentview', (x, y)))` (20 eps).
2. `robot.goto((c.center[0], c.center[1] - 0.055, 0.09), pitch=90, yaw=0, grip='close')`, then
   `robot.goto((c.center[0], c.center[1] - 0.055, 0.004))`: it reached 0.005-0.013 (13 eps,
   2 probes).
3. `robot.goto((c.center[0], c.center[1] + 0.05, 0.004))`: SUCCESS came during this push
   (13 eps, 2 probes), with the hand 0.017-0.026 past the cheese's centre (2 probes).

## Verified parameters
- Hand 0.047-0.061 to -y of the cheese's centre and within 0.012 of it in x, grasp point at
  0.005-0.013; push ends 0.004-0.056 past the centre in y: SUCCESS every time (13 eps,
  2 probes).
- Yaw -45, 35 or 90, or a push toward +y and a little toward -x from the cheese's +x end, also
  gave SUCCESS (5 eps).

## Common failures and fixes
- Pushes with the closed hand at 0.018-0.023, the cheese's top being 0.007, went over it; one
  began on its +y side and moved away from it; by its re-measures it never moved; ran out at
  300 (1 ep). Go down to 0.004 beside it first (13 eps, 2 probes).
- A first push from the cheese's +x/-y corner at 0.013 turned it; pushes toward -x and back
  toward +x followed: SUCCESS only at 243 (1 ep). Push straight toward +y (13 eps).

## Typical step costs
- Horizon 300. Above 29-35, down 12-27, push 7-10 at the default speed: 55-65 (4 eps, 2 probes);
  pushes at speed 0.05-0.1 took 18-31: 66-81 (9 eps).
