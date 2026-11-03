# libero_spatial_swap_t4

## Judging
- "In the top layer of the wooden cabinet", going by outcomes: the bowl in the open top drawer
  (top ~0.20, x 0.73-0.75, y -0.13..-0.15) set on the plate (here x 0.71-0.74, y +0.02..+0.045)
  ended the episode (3 probes, 10 eps; 2 probes in the object scene). The bowl on the cabinet's top
  (top ~0.265, y -0.26..-0.30), let go over the plate, never did (6 eps; 7 in the object scene).

## Flow
1. Free: `robot.find('agentview', 'black bowl')`, measure both bowls and the plate (2 views);
   take the bowl with top ~0.20 (10 eps).
2. The -x rim: `robot.goto((b.x - 0.04, b.y, 0.30), pitch=90, yaw=90, grip='open')`, straight
   down to 0.188, `robot.gripper('close')`: ~0.8-0.9 cm, holding (6 probes, 16 eps).
3. Up to 0.29-0.30, level to (p.x - 0.04, p.y, 0.29), then straight down toward 0.06 and open
   there: the episode ended at the end of that descent or within 4 steps of opening, once after
   the descent was BLOCKED ~0.11 up (2 probes, 2 eps).

## Verified parameters
- The agentview-only centre of the drawer bowl was within 0.4 cm of the 2-view one (12 eps).

## Common failures and fixes
- Opened 0.30 up over the plate: 8 times the episode ended within 9 steps, but twice the bowl
  came to rest 2-3 cm to +y of the plate's centre and no success came; nudging it ran out of
  steps (10 eps). Lower it first, as in step 3 (2 probes).
- 8 of 10 took the bowl on the cabinet's top and none of them ended the episode (8 eps).
- In the other scenes, slanted descents from the cabinet were BLOCKED ~0.22 up at y -0.05..+0.05
  and 2 opened there, off their plate (5 eps). Carry level at 0.29-0.30 first (3 probes).
- A 'slipped out' note after the lift or carry was wrong: the bowl still hung in the hand
  (9 eps). Measure it (free) before reacting.

## Typical step costs
- Horizon 220. Probes 161-163: approach 45, down 23-33, close 12, up 21, carry 25-28, down 23-29,
  open 3-4 (2 probes). Opening 0.30 up instead: 129-164 when it counted (1 probe, 8 eps).
- The same straight descent to 0.188 cost 20-26 steps, but 52-56 in 3 of 10 runs (10 eps).
