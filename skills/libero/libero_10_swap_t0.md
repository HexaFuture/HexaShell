# libero_10_swap_t0

## Judging
- What counted: with both cans in the basket the episode ended with success (17 eps, 4 probes).
  "alphabet soup": the blue-label can left of the agentview picture's centre (-y), just above
  mid-height: from above about 0.24-0.29 ahead, 0.13-0.18 to -y, 6 cm wide, top 0.096; the tall
  orange bottle between it and the camera (+x of it) hides most of it. "tomato sauce": the
  red-and-green can at the far left of the picture, 0.22-0.28 to -y.
- The words 'alphabet soup' measured the milk carton (6 eps) or the orange-juice carton (4 eps),
  and 'tomato sauce' found nothing (10 eps): point at both cans.

## Flow
1. Before any motion (free): `s = robot.measure(('agentview', (x, y)))` on the visible top of the
   blue can, `t = ...` on the red-and-green can: about 0.49-0.55 ahead, 0.22-0.28 to -y (20 eps).
2. Above `s.center` at 0.25, pitch 90, yaw 0, `grip='open'`, then `s = robot.measure(s)`: from
   above the blue can measured 0.24-0.29 ahead, 0.13-0.18 to -y (19 eps).
3. Descend with the hand tipped back toward the robot: to 0.16 over `s.center` at pitch 120, yaw
   0, straight down to 0.065, close, straight up to 0.25, all at pitch 120 (10 eps, 4 probes).
4. Across at 0.25 with pitch 90 to the middle of the basket, about 0.50-0.53 ahead, 0.24-0.26 to
   +y, lower toward 0.13, open, up to 0.25 (10 eps, 4 probes).
5. Red-and-green can: above it at 0.25, straight down to 0.065 at pitch 90, close, up to 0.25,
   across to the same middle as the first can, not beside it, lower toward 0.13: the episode
   ended with success during that lowering (8 eps, 4 probes).

## Verified parameters
- If the agentview measure of the blue can gives one side under 4.5 cm (half hidden), its centre
  moved 0.6-1.5 cm when measured again from above (8 eps). Descending on such a first centre
  stopped 2-4 cm high on the can, and once the can never reached the basket (2 probes); on the
  re-measured centre the descent was clean and held (3 probes).
- Basket floor from above: about 0.49-0.52 ahead, 0.26-0.27 to +y (5 eps, 4 in other variants).

## Common failures and fixes
- Grasped at pitch 90, every lift with the blue can stopped BLOCKED low, "too close in for the
  arm" (10 eps); home() with it held got free at 70-111 steps (7 eps); grasp to release took
  172-214 (8 eps), 2 episodes ran out. Pitch 120 as above lifted it at once (10 eps, 4 probes).
- Where the held can stopped at 0.16-0.21 over the basket, opening there dropped it in (5 eps,
  probe), with the hand 0.46-0.57 ahead. Aimed 0.57 ahead to keep off the first can, the second
  stopped BLOCKED at 0.20 with the hand pushed on to 0.58 ahead; opened there, it fell over the
  camera-side (+x) wall onto the table and the episode ran out (1 ep). Fix: step 5 (8 eps).

## Typical step costs
- Horizon 520. With the flow above, successes 391-464 (9 eps) and 356-400 (4 probes), e.g.
  hover 37, to 0.16 29, down 21, close 10, up 29, across 46, down 19, open 13, up 20, to the
  other can 48, down 25, close 10, up 26, across 48, down 10: 391 (1 ep). Lifting the blue can
  the pitch-90 way, 461-519 (8 eps).
