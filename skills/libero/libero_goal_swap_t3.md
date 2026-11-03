# libero_goal_swap_t3

## Judging
- The dark cabinet left in the agentview picture (to -y) faces +y with three drawers. Pulling
  its top handle 15 cm toward +y opened the top drawer; the bowl lowered into that drawer ended
  the episode with SUCCESS, before letting go (20 probes).
- The bowl (9.5 cm across) stands 26-30 cm to +y of the top handle (10 eps), farther than the
  open drawer reaches, so it is free from above (20 probes).
- Open, pick and place fit in the 300 steps only on a lean route: about 240 as below.

## Flow
1. Free, before any motion: `h = robot.measure('top drawer handle')` (an 8 cm bar, top 0.185)
   and `b = robot.measure('bowl')` (9.5 cm across, top 0.037); keep hx, hy = h.center[:2] and
   x0, y0 = b.center[:2] (10 eps).
2. `robot.goto(h.center, pitch=0, yaw=-90, roll=-90, grip='open')`, `robot.gripper('close')`,
   `robot.goto(by=(0, 0.15, 0))`: this one pull opens it (80 probes).
3. Letting go of the handle on the way: `robot.goto((x0, y0 + 0.045, 0.12), pitch=90, yaw=0,
   roll=0, grip='open')`, then `robot.goto((x0, y0 + 0.045, 0.025))`, the fingers astride
   the bowl's +y wall; `robot.gripper('close')` (20 probes).
4. Straight up: `robot.goto((x0, y0 + 0.045, 0.30))`; across: `robot.goto((hx, hy + 0.08,
   0.30))`; then `robot.goto(by=(0, 0, -0.10))`: SUCCESS came 6-7 steps into this lowering
   (probes, 20 of 20). If not: open, lift 5 cm, look (untested).

## Verified parameters
- Roll -90 at the handle: 51-53 steps from the start pose, and the pull never slipped (80
  probes). With roll 90 the same reach took 119-124 steps (2 eps).
- After that grip and release, every down-pointing yaw stayed reachable (probes).
- Carry at 0.30, as for this bowl in the other top-drawer tasks: at 0.27 it hit the open
  drawer's +y panel and pushed the drawer shut (probes).

## Common failures and fixes
- Pointing down onto the handle: BLOCKED or short, 8-100 steps lost (9 eps). Grip it level.
- Roll 90 at the handle: afterwards pointing down at yaw 0 was refused as not reachable (1
  ep). Use roll -90 as in step 2 (probes).
- Grasps on the bowl that closed on 0.3-0.7 cm did not hold it (2 eps); astride its +y wall,
  1.2 cm below its top, as in step 3, it came along every time (20 probes).
- A diagonal lift from the bowl toward the cabinet (to -y) was BLOCKED 13.6 cm short (1 ep):
  go straight up to 0.30 first (probes).

## Typical step costs
- Horizon 300. Route above 230-247: handle 51-53, close 10, pull 24-31, above the bowl 46-49,
  down 17, close 12, up 33-34, across 30-37, lower 6-7 (20 probes).
- learn1 had the drawer open in 2 eps only at 174-195 and placed nothing; 252-300 used (10 eps).
