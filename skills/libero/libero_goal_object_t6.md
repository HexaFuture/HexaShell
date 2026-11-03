# libero_goal_object_t6

## Judging
- "The cream cheese", the white box with a red label 10-16 cm +y of the bowl (agentview right),
  dropped into the bowl ended the episode with SUCCESS 5-9 steps into the open (14 probes, 5 eps).
- A box dropped into a bowl knocked onto a lean (its measured top 0.092-0.108) did not count (3
  eps, 2 probes). In a level bowl, also one pushed 2.8-6.4 cm aside, it did (4 eps).

## Flow
1. `bowl = robot.measure('bowl')`: 0.144 x 0.145 m, top 0.062-0.064, centre 0.55-0.59 m ahead,
   within 2.6 cm of y 0 (10 eps).
2. `robot.measure('cream cheese')` found nothing (10 eps). Point at it in the agentview picture:
   `c = robot.measure(('agentview', (x, y)))`: 7.5-7.9 x 3.5-3.9 cm, top 0.007, 0.59-0.63 m
   ahead, 10-16 cm +y of the bowl's centre (10 eps).
3. Take it at yaw 45, 3 cm +x of its centre, so the hand stays off the bowl:
   `robot.goto((c.center[0] + 0.03, c.center[1], 0.12), pitch=90, yaw=45, grip='open')`, then
   `robot.goto((c.center[0] + 0.03, c.center[1], 0.004), speed=0.08)`, `robot.gripper('close')`,
   `robot.goto(by=(0, 0, 0.13))`: the box came up and the bowl stayed put (10 probes).
4. `robot.goto((bowl.center[0] + 0.035, bowl.center[1] + 0.015, 0.15))`, down to z 0.07, then
   `robot.gripper('open')`: SUCCESS on the open (2 probes; with +0.03 and +0, 8 probes).

## Verified parameters
- Yaw 45, the grasp point 3 cm +x of the box's centre, closed at z 0.004: it held and the bowl
  stayed put every time (10 probes). Over the box's centre one of 3 descents stopped "short" 1.7
  cm up and closed empty; at yaw -45 the fingers took the bowl's rim (1 probe).
- With the box over the bowl's centre, opening with the hand at z 0.06-0.105 counted (14
  probes); the episodes' successes opened at z 0.052-0.083 (5 eps).

## Common failures and fixes
- At yaw 0 the hand came down on the bowl's +y rim: "short" 1.1-1.7 cm up, too high to hold (4
  eps), BLOCKED 8 cm up (1 ep), a hold that slipped on the lift (1 ep), or no sign at all. Measured
  again, the bowl had moved 2.8-6.4 cm, mostly to -y (6 eps), or leaned, top 0.076-0.108 (4 eps).
  Second descents at speed 0.05-0.08 (twice grip 0.65, once pitch 75) held (5 eps), 40-100 steps.
- After a knock: top 0.058-0.064, release over the new centre: SUCCESS (3 eps); top 0.080-0.084
  also counted (1 ep). Top 0.09 or more: no SUCCESS with the box inside (3 eps); the closed hand
  lowered onto the bowl's -x rim, 7 cm from its centre, gave SUCCESS as it met the rim (1 ep).
- Over a bowl pushed against the cabinet the move down was BLOCKED 18 cm up; opening there
  dropped the box in: SUCCESS (1 ep).
- With the box set down again by the bowl's rim, a full open put a finger under the rim and the
  lift carried the bowl off (1 ep). Beside the bowl open only to about 0.65 (untested).

## Typical step costs
- Horizon 300. Yaw-45 route: pre-pose 34-38, down 35, close 10, lift 22, carry 18-25, lower 15,
  open 5-7: SUCCESS at 140-151 (10 probes). At yaw 0 the successes took 131-268 (7 eps); the
  failures ran out at 300 (3 eps).
