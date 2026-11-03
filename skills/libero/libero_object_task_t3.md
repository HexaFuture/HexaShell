# libero_object_task_t3

## Judging
- The bottle that counted: the tall orange bottle with the light grey cap (label "Tomato
  Ketchup"), a little right of the centre of the agentview picture (+y) at mid-height: from above
  about 0.45 ahead, 0.06 to +y, 14-15 cm tall (top 0.147). Lowered into the basket, it ended the
  episode with success (2 eps).
- The words 'ketchup' measure the small dark red-brown bottle a little left of and below the
  picture's centre (about 0.65-0.67 ahead, 0.09-0.11 to -y, top 0.112) first, in all 10 episodes. It
  never counted: 8 episodes placed it and none ended there.
- `robot.measure('ketchup')` listed the tall bottle only as another 'ketchup' seen, at about
  0.47 ahead, 0.06 to +y (3 eps): choose that one, by its mask or pixel.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` over both bottles to
   read the label; `m = robot.find('agentview', at=(x, y))[0]` on the tall one, check
   `robot.look('agentview', masks=[m])`, `c = robot.measure(m)` (1 ep). Pointed at from
   agentview its top read only 0.084-0.086 (9 eps); from above it measured about 0.45 ahead,
   0.06 to +y, top 0.147 (2 eps).
2. `robot.goto((c.center[0], c.center[1], 0.27), pitch=90, yaw=0, grip='open')`, measure again,
   straight down to 0.11-0.125 (by its neck), close: 3.3-3.8 cm, holding (9 eps).
3. Straight up to 0.31, across at 0.32-0.33 to above the basket's measured centre, straight down
   toward 0.17: the episode ended with success during that lowering (2 eps).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).

## Common failures and fixes
- The small dark bottle placed first: in the basket at 157-175, no success (8 eps); 7 of them
  grasped the tall bottle at 232-258, too late (7 eps).
- One episode set the small bottle down off the basket at step 133 and still won at 267 (1 ep).

## Typical step costs
- Horizon 280. Direct path 122: approach 13, down 21, close 10, up 25, across 39, down 14 (1 ep);
  267 with the wrong bottle set down first (1 ep).
