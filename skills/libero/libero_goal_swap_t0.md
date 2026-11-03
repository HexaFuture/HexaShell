# libero_goal_swap_t0

## Judging
- The dark cabinet left in the agentview picture (to -y) has three drawers facing +y. One
  level grip on its middle bar and a pull of about 15 cm toward +y ended the episode with
  SUCCESS (10 probes).
- The middle bar: 6-8 cm long along x, top 0.112, centre near 0.108 (8 eps).
- One reach, one close and one pull fit in about 100 steps; learn1 needed 159-281 (8 eps).

## Flow
1. Free, before any motion: `h = robot.measure(('agentview', (u, v)))` on the middle bar
   (8 eps); keep hx, hy, hz = h.center.
2. Level, pointing to -y, the open fingers closing vertically: first a stop 6 cm to +y of the
   bar, `robot.goto((hx, hy + 0.06, hz), pitch=0, yaw=-90, roll=-90, grip='open')`, then
   `robot.goto(h.center)` and `robot.gripper('close')` (10 probes).
3. `robot.goto(by=(0, 0.15, 0))`: SUCCESS came during it in 7 of 10. Otherwise it ended after 25
   steps, short of target or ok; then `robot.goto(by=(0, 0.04, -0.015))`: SUCCESS came 2-10
   steps in (10 probes).

## Verified parameters
- The stop 6 cm to +y: going straight to h.center, 8 of 10 opened; one reach was BLOCKED
  before the bar and one arrived slow and short (probes). Via the stop all 10 arrived ok.
- Roll -90 at the bar: 47-62 steps to the stop (10 probes). learn1 came at roll 90: 54-99
  steps, and the hand had to be pushed into the bar afterwards (10 eps).
- The extra move 1.5 cm down after a pull with no SUCCESS: SUCCESS 11 of 12 times (probes); a
  level 3 cm extra pull did nothing (2 probes).

## Common failures and fixes
- Roll 90 at the bar (10 eps): 4 reaches BLOCKED, jammed or stalled 2-12 cm short; then a
  push to -y into the bar ended short, stalled or BLOCKED (8 eps). Use roll -90 as in step 2.
- Grips that closed on 0.6-1.1 cm, or half-closed at 3.5-4.3 cm, slipped off during the pull
  or held nothing (2 eps).
- Pulls in legs of 5-7 cm cost 33-80 steps (8 eps); the one 15 cm pull took 17-25 (probes).
- 2 eps ran out at 286-300 steps after repeated regrips (2 eps).

## Typical step costs
- Horizon 300. Route above 87-106: to the stop 47-62, in 13-15, close 10-11, pull 17-25,
  extra 2-10 when needed (10 probes).
- learn1 successes took 159-281 (8 eps).
