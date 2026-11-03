# libero_goal_object_t3

## Judging
- The dark cabinet left in the agentview picture (to -y) faces +y with three drawers. Pulling
  its top handle 15 cm toward +y opened the top drawer; the big bowl lowered into that drawer
  ended the episode with SUCCESS, before letting go (20 probes).
- The big bowl (14.4 cm across) stands 13-17 cm to +y of the top handle (10 eps), so the open
  drawer reaches over its -y part: a move down to it was BLOCKED by the drawer (1 ep).
- Open, pick and place fit in the 300 steps only on a lean route: about 270 as below.

## Flow
1. Free, before any motion: `h = robot.measure('top drawer handle')` (an 8 cm bar, top 0.185)
   and `b = robot.measure('bowl')` (14.4 cm across, top 0.062, seen by both cameras); keep
   hx, hy = h.center[:2] and x0, y0 = b.center[:2] (9 eps).
2. `robot.goto(h.center, pitch=0, yaw=-90, roll=-90, grip='open')`, `robot.gripper('close')`,
   `robot.goto(by=(0, 0.15, 0))`: this one pull opens it (80 probes).
3. The bowl's +y rim stays clear of the open drawer. Letting go of the handle on the way:
   `robot.goto((x0, y0 + 0.155, 0.24), pitch=90, yaw=0, roll=0, grip='open')`, then
   `robot.goto((x0, y0 + 0.065, 0.048))`, fingers astride that rim; close (20 probes).
4. Drag it to +y, out from under the drawer, then lift: `robot.goto(by=(0, 0.13, 0.02))`,
   `robot.goto((x0, y0 + 0.195, 0.33))` (20 probes).
5. `robot.goto((hx, hy + 0.12, 0.33))`, then `robot.goto(by=(0, 0, -0.12))`: SUCCESS came 7-8
   steps into this lowering (probes, 20 of 20). If not: open, lift 5 cm, look (untested).

## Verified parameters
- Roll -90 at the handle: 51-53 steps from the start pose, and the pull never slipped (80
  probes). With roll 90 the same reach took 118 steps (1 ep).
- After that grip and release, every down-pointing yaw stayed reachable (probes).
- The rim descent may push the drawer a few cm to -y, partly shut; SUCCESS still came (probes).
- Carry at 0.33, higher than for a small bowl. A bowl centre measured in one view only also
  worked (1 probe).

## Common failures and fixes
- Pointing down onto the handle: BLOCKED or short, 28-83 steps lost (9 eps). Grip it level.
- Roll 90 at the handle: afterwards pointing down at yaw 0 was refused as not reachable (1
  ep). Use roll -90 as in step 2 (probes).
- Once the drawer was open, a move down to the bowl was BLOCKED 13 cm short by the drawer
  (1 ep), and a pinch on the bowl's rim closed on 0.9 cm (1 ep). Use steps 3-4 (probes).

## Typical step costs
- Horizon 300. Route above 261-279: handle 51-53, close 10-11, pull 24-30, above the bowl
  46-48, down 27-45, close 11-12, drag 19-23, up 32, across 32-37, lower 7-8 (20 probes).
- learn1 had the drawer open in 3 eps only at 163-250 and placed nothing; 268-300 used (10 eps).
