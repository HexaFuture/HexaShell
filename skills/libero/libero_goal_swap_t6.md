# libero_goal_swap_t6

## Judging
- The cream cheese: the flat box lying near the robot on its +y side (upper right of the
  agentview picture): 0.24-0.27 ahead, 0.19-0.23 to +y, about 7-7.6 x 3.7 cm, long side along x,
  top 0.007. 'cream cheese' found nothing in all 20 episodes: point at it.
- The bowl: `robot.measure('bowl')`, 0.44-0.48 ahead, 0.04-0.07 to -y, top 0.037 (20 eps).
- SUCCESS came on letting go over the bowl from 0.05-0.08, or during the lowering to 0.052 with
  the cheese held (18 eps, 2 probes).

## Flow
1. Free: point at the cheese, `c = robot.measure(('agentview', (x, y)))`; `b =
   robot.measure('bowl')` (20 eps).
2. `robot.goto((c.center[0], c.center[1], 0.15), pitch=90, yaw=30, grip='open')`, down to
   0.002: it ends "short" by about 2 cm with the hand turned to yaw 47; close there (2 probes).
3. `robot.goto(by=(0, 0, 0.12))`: it stopped short near 0.09, fine (2 probes). Over the bowl's
   centre at 0.16, down to 0.052, `robot.gripper('open')`: SUCCESS (2 probes).

## Verified parameters
- Yaw 30 from the start, cheese 0.22-0.23 to +y: SUCCESS at 156 and 165 (2 probes). Yaw 20 or
  30 as a second try held 4.9-5.5 cm (2 eps).
- Yaw 0 down to 0.002 ends "short" by 10-28 mm. With the cheese at 0.19-0.21 to +y, closing
  there held 4.2-4.3 cm (9 eps). At 0.20-0.23 to +y the result said the hand's twist reached its
  end of travel (9 eps); closing at once still held 4.2 cm (1 ep).
- The grasp must get down to 0.002: closes at 0.010-0.014 got 0.4-1.1 cm and lost it (3 eps).

## Common failures and fixes
- After the twist note, the yaw 180 retry cost 75-93 steps for the turn alone: finished at
  269-299 or ran out (4 eps). Retry at yaw 20-30 instead: 26-29 steps (2 eps), or start there
  (2 probes).
- Aimed at 0.017 first: short, nothing held; the retry ended at 0.011 and closed on 1.1 cm; the
  yaw 180 turn took 93; held 4.2 cm at 245, the lift took 51, finish at 296 (1 ep).
- Carries begun from 0.09-0.10 up were BLOCKED on the way to the bowl in 5 eps, though not in 2
  probes at yaw 30; begun from 0.13-0.16 up, never (8 eps).

## Typical step costs
- Horizon 300. Above 36-41, down 24-33, close 10, up 25-36, over the bowl 34-42, down 14-25,
  open 7: 156-165 (2 probes); yaw 0 runs that closed at once: 168-202 (10 eps).
