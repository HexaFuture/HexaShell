# libero_object_object_t3

## Judging
- The bottle that counted: the small dark-green bottle a little left of and below the centre of the
  agentview picture (-y): from above about 0.64-0.66 ahead, 0.09-0.11 to -y, about 10 cm tall (top
  0.112). Lowered into the basket, it ended the episode with success (5 eps).
- The tall orange bottle with the light grey cap, a little right of the picture's centre (+y)
  at mid-height, about 0.45-0.47 ahead, 0.06 to +y, top 0.147, never did: 5 episodes placed it
  first and none ended there (5 eps).
- The words 'bbq sauce' found the green bottle in 5 episodes and nothing in the other 5: point
  at it when they fail.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` to see the small green
   bottle; `m = robot.find('agentview', at=(x, y))[0]`, check `robot.look('agentview',
   masks=[m])`, `c = robot.measure(m)`. From above it measured about 0.64-0.66 ahead, 0.09-0.11
   to -y, top 0.112 (6 eps).
2. `robot.goto((c.center[0], c.center[1], 0.23), pitch=90, yaw=0, grip='open')`, measure again,
   straight down to 0.08-0.10, close: 2.7-3.6 cm, holding (5 eps).
3. Straight up to 0.25-0.27, across at 0.27-0.29 to above the basket's measured centre, straight
   down toward 0.14: the episode ended with success during that lowering (5 eps).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).
- From above the green bottle read top 0.112 (6 eps), the orange one 0.147 (1 ep); pointed at
  from agentview the orange one read only 0.084-0.085 (7 eps), so check from above.

## Common failures and fixes
- Tall orange bottle placed first: in the basket at 133-138, no success (5 eps).
- The late grabs went in one move from above the basket to grasp height at the green bottle:
  "short of target" at 0.105-0.111, on its top, then closed on 0.5-1.6 cm or on nothing (4 eps).
  Hover 0.23 above it first, then straight down, as the successes did (5 eps).

## Typical step costs
- Horizon 280. Successes 140-149: approach 36-39, down 20-24, close 10, up 24-26, across 38-40,
  down 11-13 (5 eps).
