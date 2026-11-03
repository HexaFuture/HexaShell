# libero_object_object_t4

## Judging
- The bottle that counted: the plain blue bottle in the left part of the agentview picture (-y),
  at mid-height: about 0.50 ahead, 0.24 to -y from agentview, 14-15 cm tall (top 0.147); nothing
  on it reads ketchup. Lowered into the basket, it ended the episode with success (3 probes).
- The words 'ketchup' measured the small dark red-brown bottle a little left of and below the
  picture's centre (about 0.65-0.67 ahead, 0.09-0.11 to -y, top 0.112) in all 10 episodes; all 10
  placed it and none ended there.
- The words 'blue bottle' found the blue one (3 eps).

## Flow
1. Before any motion (free): `m = robot.find('agentview', at=(x, y))[0]` on the blue bottle (or
   'blue bottle'), check `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`: about
   0.50 ahead, 0.24 to -y, top 0.147 (6 eps).
2. `robot.goto((c.center[0], c.center[1], 0.22), pitch=90, yaw=0, grip='open')`, straight down
   to 0.12 (by its neck), close (probe); aiming 1.5 cm to -x of that centre worked too (probe).
3. Straight up to 0.30, across at 0.30 to above the basket's measured centre, straight down
   toward 0.16: the episode ended with success during that lowering, at about 0.18 (2 probes).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).
- A bottle of this shape measured 0.7-1.7 cm farther to -x from above than from agentview alone
  (23 eps in other variants); the neck grasp still held from the agentview centre (probe).

## Common failures and fixes
- Small dark bottle placed first: in the basket at 156-178 steps, no success (10 eps).
- The late tries at the blue bottle went in one move from the basket down to 0.12-0.13 on its
  agentview centre, 51-59 steps; the closes read 0.9-1.0 cm or nothing, or the steps ran out
  (4 eps). Hover 0.22 above it, then straight down over its neck (2 probes).

## Typical step costs
- Horizon 280. Probed path 149: approach 29, down 17, close 10, up 25, across 55, down 13
  (2 probes).
