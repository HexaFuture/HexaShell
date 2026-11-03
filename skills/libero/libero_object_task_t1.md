# libero_object_task_t1

## Judging
- The can that counted: the blue-label can in the left part of the agentview picture (-y), at
  mid-height: about 0.48 ahead, 0.24 to -y, top 0.080. Lowered into the basket, it ended the
  episode with success (2 probes).
- The red-and-green can between it and the camera (+x of it: about 0.70 ahead, 0.20 to -y) never
  did: all 10 episodes placed it first and none ended there (10 eps).
- The words 'alphabet soup' measured the orange-juice carton a little left of the picture's
  centre (about 0.41 ahead, 0.07 to -y) in 6 episodes, or found nothing (4): a carton with top
  0.137 is not a can. Point at the blue can instead.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` on the two cans in the
   left part of the agentview picture (-y); the one farther from the camera (-x) has the blue
   label. `m = robot.find('agentview', at=(x, y))[0]`, `robot.look('agentview', masks=[m])`,
   `c = robot.measure(m)`: about 0.48 ahead, 0.24 to -y, top 0.080 (8 eps).
2. `robot.goto((c.center[0], c.center[1], 0.15), pitch=90, yaw=0, grip='open')`, then straight
   down to 0.055 and close (probe); a can of this size closed at 6.2 cm, holding (10 eps).
3. Straight up to 0.27, across at 0.27 to above the basket's measured centre, straight down
   toward 0.13: the episode ended with success during that lowering, at about 0.15 (probe).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.24-0.27 to +y
  (10 eps).
- Coming down from 0.15 above the blue can, the red-and-green can on its +x side is no obstacle
  (2 probes).

## Common failures and fixes
- Red-and-green can first: in the basket at 175-186 (10 eps), 7 of them after a zoomed look at it
  (7 eps); the blue can was tried only at 242-272, too late (6 eps). Take the blue-label can on
  its -x side (2 probes).
- One straight move from above the basket to grasp height at the blue can: BLOCKED after 47-49
  steps, 4.4-4.8 cm short, "pressing on something"; then the close found nothing or the steps
  ran out (5 eps). Go to 0.15 above it first, then straight down (2 probes).

## Typical step costs
- Horizon 280. Probed path 156-158: approach 31, down 17, close 10, up 29, across 56-58, down 13
  (2 probes). The red-and-green can alone had used 175-186 by its release (10 eps).
