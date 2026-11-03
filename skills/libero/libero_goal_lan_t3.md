# libero_goal_lan_t3

## Judging
- The dark cabinet left in the agentview picture (to -y) faces +y with three drawers. Pulling
  its top handle 15 cm toward +y opened the top drawer; the bowl lowered into that drawer ended
  the episode with SUCCESS, before letting go (20 probes).
- The bowl (9.5 cm across) stands 12-17 cm to +y of the top handle (10 eps), so the open
  drawer reaches over its -y part: moves down to it were BLOCKED by the drawer (3 eps).
- Open, pick and place fit in the 300 steps only on a lean route: about 260 as below.

## Flow
1. Free, before any motion: `h = robot.measure('top drawer handle')` (an 8 cm bar, top 0.185)
   and `b = robot.measure('bowl')` (9.5 cm across, top 0.037); keep hx, hy = h.center[:2] and
   x0, y0 = b.center[:2] (10 eps).
2. `robot.goto(h.center, pitch=0, yaw=-90, roll=-90, grip='open')`, `robot.gripper('close')`,
   `robot.goto(by=(0, 0.15, 0))`: this one pull opens it (80 probes).
3. The bowl's +y wall stays clear of the open drawer. Letting go of the handle on the way:
   `robot.goto((x0, y0 + 0.135, 0.22), pitch=90, yaw=0, roll=0, grip='open')`, then
   `robot.goto((x0, y0 + 0.045, 0.025))`, fingers astride that wall; close (30 probes).
4. Drag it to +y, out from under the drawer, then lift: `robot.goto(by=(0, 0.09, 0.02))`,
   `robot.goto((x0, y0 + 0.135, 0.30))` (30 probes).
5. `robot.goto((hx, hy + 0.08, 0.30))`, then `robot.goto(by=(0, 0, -0.10))`: SUCCESS came 6-8
   steps into this lowering (probes, 20 of 20). If not: open, lift 5 cm, look (untested).

## Verified parameters
- Roll -90 at the handle: 51-53 steps from the start pose, and the pull never slipped (80
  probes). With roll 90 the same reach took 124-125 steps (2 eps).
- After that grip and release, every down-pointing yaw stayed reachable (probes).
- The descent to the bowl may push the drawer a few cm to -y; SUCCESS still came (probes).
- Carry at 0.30: at 0.27 the hanging bowl hit the open drawer's +y panel and pushed the drawer
  shut, 4 failures in 10 (probes).

## Common failures and fixes
- Pointing down onto the handle: BLOCKED or short, 10-84 steps lost (8 eps). Grip it level.
- Roll 90 at the handle: afterwards pointing down at yaw 0 was refused as not reachable (2
  eps). Use roll -90 as in step 2 (probes).
- Once the drawer was open, moves down or level to the bowl were BLOCKED 7-13 cm short by the
  drawer (3 eps), and pinches on the bowl's rim closed on 0.3-0.9 cm (3 eps). Use steps 3-4.
- A diagonal descent onto the bowl's rim, a level move under the drawer and a pitch-60
  approach hit the bowl's rim or pushed the drawer shut (probes).

## Typical step costs
- Horizon 300. Route above 257-268: handle 51-53, close 10, pull 24-31, above the bowl 46-48,
  down 27-30, close 12, drag 16, up 31-32, across 30-34, lower 6-8 (20 probes).
- learn1 had the drawer open in 4 eps only at 180-273 and placed nothing; 247-300 used (10 eps).
