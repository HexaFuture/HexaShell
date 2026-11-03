# libero_spatial_object_t4

## Judging
- "In the top layer of the wooden cabinet", going by outcomes: the bowl in the open top drawer
  (top ~0.20, x 0.73-0.75, y -0.13..-0.15) set on the plate (x 0.70-0.73, y +0.19..+0.21) ended
  the episode (2 probes; 1 probe and 2 eps in the swap scene). The bowl on the cabinet's top
  (top ~0.265, y -0.26..-0.30), let go over the plate, never did (7 eps; 6 in the swap scene).

## Flow
1. Free: `robot.find('agentview', 'black bowl')`, measure both bowls and the plate (2 views);
   take the bowl with top ~0.20 (10 eps).
2. The -x rim: `robot.goto((b.x - 0.04, b.y, 0.30), pitch=90, yaw=90, grip='open')`, straight
   down to 0.188, `robot.gripper('close')`: ~0.8-0.9 cm, holding (4 probes, 6 eps).
3. Up to 0.30, level to (p.x - 0.04, p.y, 0.30), open there: it drops onto the plate and the
   episode ends within 7 steps (3 probes; 8 eps in this scene opened 0.20-0.33 up over it).

## Verified parameters
- The agentview-only centre of the drawer bowl was within 0.4 cm of the 2-view one (12 eps).
- Hand 1.2 cm below that bowl's top and 4 cm to -x of its centre, the fingers closing along x
  on its -x wall (4 probes, 6 eps).

## Common failures and fixes
- All 10 took the bowl on the cabinet's top and none ended the episode (10 eps): take the drawer
  bowl (2 probes).
- Slanted descents from the cabinet toward the plate were BLOCKED ~0.22 up at y -0.05..+0.05;
  2 of them opened there and missed the plate (5 eps). Carry level at 0.30 instead (3 probes).
- A 'slipped out' note after the lift or carry was wrong: the bowl still hung in the hand
  (9 eps). Measure it (free) before reacting.

## Typical step costs
- Horizon 220. Probes 134-149: approach 45-46, down 23, close 12, up 20-22, carry 26-40, open
  6-7 (3 probes). Lowering to 0.05 before opening cost ~30 more (1 probe).
- One descent to 0.188 took 56 steps instead of 23 (1 probe): budget for it.
