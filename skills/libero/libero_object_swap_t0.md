# libero_object_swap_t0

## Judging
- The can that counted: the blue-label can a little left of the centre of the agentview picture
  (-y), at mid-height: about 0.40 ahead, 0.08 to -y, top 0.080; the white bottle with the green
  cap stands between it and the camera (+x of it). Every episode that took it first ended with
  success (7 eps).
- The red-and-green can in the lower middle of the agentview picture (+x, nearest the camera:
  about 0.75 ahead, y about +0.03) never did: each failed episode placed it first (3 eps).
- The words 'alphabet soup' found nothing in all 10 episodes: point at the blue can.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` on that can shows the
   blue label; `m = robot.find('agentview', at=(x, y))[0]`, `robot.look('agentview',
   masks=[m])`, `c = robot.measure(m)`. It measured about 0.40 ahead, 0.08 to -y, top 0.080,
   6 cm wide (8 eps).
2. `robot.goto((c.center[0], c.center[1], 0.18), pitch=90, yaw=0, grip='open')`, measure again
   from there, straight down to 0.05-0.06, close: 6.3 cm, holding (7 eps).
3. Straight up to 0.23-0.25, across at 0.27-0.29 to above the basket's measured centre, straight
   down toward 0.11: the episode ended with success during that lowering (7 eps).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.26 to +y
  (10 eps).

## Common failures and fixes
- Red-and-green can first: in the basket at 170-174 steps; the late move from there to the blue
  can at grasp height stopped "short" on its top, the close read 2.1-2.6 cm, out of steps (3 eps).
  Every success settled on the blue can before the first motion, 5 of them after a zoom that
  showed it (7 eps).

## Typical step costs
- Horizon 280. Successes 136-148: approach 18-21, down 23-28, close 10, up 25-27, across 44-50,
  down 13-22 (7 eps).
