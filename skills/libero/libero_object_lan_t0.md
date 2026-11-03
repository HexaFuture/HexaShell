# libero_object_lan_t0

## Judging
- The can that counted: the blue-label can in the left part of the agentview picture (-y), at
  mid-height: about 0.48 ahead, 0.24 to -y, 6 cm wide, top 0.080; the milk carton stands between
  it and the camera (+x of it). Lowered into the basket, it ended the episode with success
  (2 probes).
- The red-and-green can in the lower middle of the agentview picture (+x, nearest the camera:
  about 0.75 ahead, y about +0.03) never did: all 10 episodes placed it first, and the steps
  left were then too few for the blue can (10 eps).
- The words 'alphabet soup' found nothing in all 10 episodes: point at the blue can instead.

## Flow
1. Before any motion (free): `robot.look('agentview', crop=..., zoom=3)` around that can to see
   the blue label (untested); `m = robot.find('agentview', at=(x, y))[0]`,
   `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`. Pointed at, it measured about
   0.48 ahead, 0.24 to -y, top 0.080, 6 cm wide (9 eps).
2. `robot.goto((c.center[0], c.center[1], 0.15), pitch=90, yaw=0, grip='open')`, then straight
   down to 0.055 and close (probe); a can of this size closed at 6.2-6.3 cm, holding (10 eps).
3. Straight up to 0.27, across at 0.27 to above the basket's measured centre, straight down
   toward 0.13: the episode ended with success during that lowering, at about 0.15 (probe).

## Verified parameters
- Basket from agentview: top 0.140, 15-16 cm across, about 0.60-0.62 ahead, 0.25-0.27 to +y
  (10 eps).
- Can grasp: pointing down at yaw 0, grasp point 2.5 cm below its top (2 probes).

## Common failures and fixes
- The red-and-green can taken for the soup: in the basket at 169-212 steps, no success, then
  out of steps (10 eps). 6 had zoomed on it first and kept it (6 eps): a zoom does not settle
  it, the blue label does. Take the blue-label can (2 probes).
- One straight move from above the basket to grasp height at the blue can: BLOCKED after 49
  steps, 4.4-4.6 cm short, "pressing on something"; the close then found nothing (3 eps). Go to
  0.15 above it first, then straight down (2 probes).

## Typical step costs
- Horizon 280. Probed path 154-155: approach 31, down 17, close 10, up 29, across 54-55, down 13
  (2 probes). The red-and-green can alone had used 169-212 by its release (10 eps).
