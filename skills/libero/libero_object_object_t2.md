# libero_object_object_t2

## Judging
- The bottle that counted: the red-cap bottle a little left of and below the centre of the agentview
  picture (-y): from above about 0.65 ahead, 0.10 to -y, top 0.146. Taken first, it ended the
  episode with success (4 eps); taken after the wrong bottle was set down, too (1 ep).
- The orange bottle with the light grey cap (label "Tomato Ketchup") in the left part of the
  picture (-y), at mid-height, about 0.48 ahead, 0.24 to -y, never did: 5 episodes placed it
  first and none ended there (5 eps).
- Both are 14-15 cm tall (top 0.146-0.147), so height does not tell them apart; the cap does.
- The words 'salad dressing' found nothing in 9 of 11 tries: point at the red-cap bottle.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` over both bottles to
   see the caps; `m = robot.find('agentview', at=(x, y))[0]` on the red-cap one, check
   `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`. From above it measured about
   0.65 ahead, 0.10 to -y, top 0.146 (5 eps).
2. `robot.goto((c.center[0], c.center[1], 0.25), pitch=90, yaw=0, grip='open')`, measure again,
   straight down to 0.12 (by its neck), close: 3.7 cm, holding (5 eps).
3. Straight up to 0.30, across at 0.32 to above the basket's measured centre, straight down
   toward 0.16: the episode ended with success during that lowering (4 eps).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).
- The grey-cap bottle closes at 3.3-3.4 cm, the red-cap one at 3.7 cm (10 eps).

## Common failures and fixes
- Grey-cap bottle placed first: in the basket at 144-186, no success, then out of steps (5 eps).
- Lifted by mistake, it can still be undone: one episode set it down off the basket at step 110,
  then took the red-cap bottle and succeeded at 260 (1 ep). Set it down at once, low, where it
  stood (untested).

## Typical step costs
- Horizon 280. Successes 141-148: approach 31-38, down 19-22, close 10, up 24-27, across 40-41,
  down 14-15 (4 eps); 260 with the wrong bottle set down first (1 ep).
