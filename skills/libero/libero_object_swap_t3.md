# libero_object_swap_t3

## Judging
- The bottle that counted: the small dark red-brown bottle with a dark cap in the left part of
  the agentview picture (-y), at mid-height: about 0.48-0.50 ahead, 0.23-0.25 to -y, about 10 cm
  tall (top 0.112); the white bottle with the green cap stands between it and the camera (+x of
  it) and partly hides it. Lowered into the basket, it ended the episode with success (6 eps).
- The taller orange bottle with the light grey cap (label "Tomato Ketchup"), a little right of
  the picture's centre (+y) at mid-height, about 0.45-0.47 ahead, 0.06 to +y, top 0.147, never
  did: 4 episodes placed it and none ended there (4 eps).
- The words 'bbq sauce' found nothing in all 10 episodes: point at the small dark bottle.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` on the left part of the
   picture (-y); `m = robot.find('agentview', at=(x, y))[0]` on the small dark bottle, check
   `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`: about 0.48-0.50 ahead,
   0.23-0.25 to -y, top 0.112 (7 eps).
2. `robot.goto((c.center[0], c.center[1], 0.25), pitch=90, yaw=0, grip='open')`, measure again,
   straight down to 0.09-0.10, close: 2.7-3.5 cm, holding (6 eps).
3. Straight up to 0.26-0.27, across at 0.25-0.30 to above the basket's measured centre,
   straight down toward 0.14: the episode ended with success during that lowering (6 eps).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).

## Common failures and fixes
- The orange bottle lifted first (9 eps): 4 put it in the basket at 129-139 and failed; 5 saw
  its label after the lift, set it down at 95-101 and still won at 263-275 (9 eps).
- 7 episodes zoomed on the orange bottle before moving and still lifted it (7 eps). The one
  direct success zoomed over both, pointed at the small dark bottle, measured top 0.112 and took
  it: done at 158 (1 ep).

## Typical step costs
- Horizon 280. Direct path 158: approach 29, down 23, close 10, up 26, across 56, down 14 (1 ep).
  With the orange bottle lifted and set down first: 263-275 (5 eps).
