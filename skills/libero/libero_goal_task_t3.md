# libero_goal_task_t3

## Judging
- The dark cabinet left in the agentview picture (to -y) faces +y with three drawers. Pulling
  its top handle 15 cm toward +y opened the top drawer; the blue box lowered into that drawer
  ended the episode with SUCCESS, before letting go (20 probes).
- The cream cheese is that small blue box, at the right of the agentview picture (to +y), 25-29
  cm to +y of the top handle (10 eps). The words 'cream cheese' found nothing in all 10
  episodes: point at it instead.
- Open, pick and place fit in the 300 steps only on a lean route: about 245 as below.

## Flow
1. Free, before any motion: `h = robot.measure('top drawer handle')` (an 8 cm bar, top 0.185);
   `c = robot.measure(('agentview', (u, v)))` on the blue box: 7.9 x 3.8 cm, top 0.007,
   grip_yaw about 0 (10 eps). Keep hx, hy = h.center[:2] and x0, y0 = c.center[:2].
2. `robot.goto(h.center, pitch=0, yaw=-90, roll=-90, grip='open')`, `robot.gripper('close')`,
   `robot.goto(by=(0, 0.15, 0))`: this one pull opens it (80 probes).
3. Letting go of the handle on the way: `robot.goto((x0, y0, 0.14), pitch=90, yaw=0, roll=0,
   grip='open')`, then `robot.goto((x0, y0, 0.004))` (it stops a few mm higher, the
   fingertips on the table) and `robot.gripper('close')` (20 probes).
4. Straight up: `robot.goto((x0, y0, 0.27))`; across: `robot.goto((hx, hy + 0.03, 0.27))`;
   then `robot.goto(by=(0, 0, -0.10))`: SUCCESS came 7-8 steps into this lowering (probes,
   20 of 20). If not: open there, lift, look (untested).

## Verified parameters
- Roll -90 at the handle: 51-53 steps from the start pose, and the pull never slipped (80
  probes). With roll 90 the same reach took 122-123 steps (2 eps).
- Start the turn to pointing down high: from the pulled handle to 0.10 above the box, opening
  on the way, the hand was BLOCKED near the handle in 3 of 10; to 0.14, never (probes).
- Gripped across its 3.8 cm side at yaw 0, the box came along every time (20 probes).

## Common failures and fixes
- Pointing down onto the handle: BLOCKED or short, 13-150 steps lost (8 eps). Grip it level.
- Roll 90 at the handle: afterwards pointing down at yaw 0 was refused as not reachable (1
  ep). Use roll -90 as in step 2 (probes).
- A down-pointing grip on the handle at yaw 90 slipped off during the pull (1 ep).
- Boxes grasped only at 254-274, after a slow opening, ran out of steps in the carry (2 eps).

## Typical step costs
- Horizon 300. Route above 235-260: handle 51-53, close 10, pull 24-31, above the box 46-48,
  down 20-43, close 10, up 33, across 33-37, lower 7-8 (20 probes).
- learn1 had the drawer open in 2 eps only at 193-198 and placed nothing; 250-300 used (10 eps).
