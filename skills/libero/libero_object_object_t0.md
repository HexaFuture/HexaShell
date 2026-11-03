# libero_object_object_t0

## Judging
- The can that counted: the big blue can in the left part of the agentview picture (-y), at
  mid-height: about 0.47-0.50 ahead, 0.23-0.25 to -y, about 9 cm across and 12 cm tall; the milk
  carton stands between it and the camera (+x of it). Lowered into the basket, it ended the
  episode with success (9 eps, 7 probes).
- The red-and-green can in the lower middle of the agentview picture (+x, nearest the camera:
  about 0.75 ahead, y about +0.03) never did: all 10 learn1 episodes placed it first, and none
  got the big can in afterwards (10 eps).
- The words 'alphabet soup' found nothing in all 10 learn1 episodes: point at the big can.
- It is wider than the open gripper (gap about 7.7 cm), so it cannot be gripped across its middle.

## Flow
1. Before any motion (free): point at it, `m = robot.find('agentview', at=(x, y))[0]`, check
   `robot.look('agentview', masks=[m])`, `c = robot.measure(m)`: agentview only, 0.47-0.50
   ahead, 0.23-0.25 to -y, top 0.119, 8.6-8.9 cm wide (15 eps). Keep this `c` for the grip.
2. Grip its edge on the camera side (+x): `robot.goto((c.center[0] + 0.035, c.center[1], 0.18),
   pitch=90, yaw=0, grip='open')`, straight down to 0.085, close, straight up to 0.27 (9 eps,
   2 probes): held 6.1-7.6 cm (9 eps).
3. Carry keeping the offset: hand at the basket's measured centre + 0.035 in x, at 0.27; down to
   0.17, then open: SUCCESS came on the opening (9 eps, 2 probes).

## Verified parameters
- From the start measure, + 0.035 in x at yaw 0 gripped and lifted it (9 eps, 2 probes).
- Measured again from above (two views), the can read 0.017 farther to +x (2 eps). From the
  start measure, + 0.052 closed on nothing and + 0.041 held 2.5 cm and slipped (1 ep).
- Basket from agentview: top 0.140, 15-16 cm across, about 0.59-0.62 ahead, 0.24-0.27 to +y
  (20 eps).

## Common failures and fixes
- Red-and-green can first: in the basket at 174-219 steps, no success (10 eps).
- Re-measured from above, then + 0.035 from that: closed on nothing; the next grips held 2.5 and
  0.4 cm and slipped; the fourth held 6.1 cm with 64 steps left and the carry ran out (1 ep).
  Grip from the start measure; one run re-measured but kept it and succeeded (9 eps).
- Centred over the big can, the open hand came down on its top and stopped "short" at
  0.12-0.13 (3 eps): use the edge grasp (9 eps, 2 probes).

## Typical step costs
- Horizon 280. Approach 30-32, down 16-17, close 10-11, up 26, across 54-57, down 18, open 7-10:
  163-168 (9 eps); 165-166 (2 probes).
