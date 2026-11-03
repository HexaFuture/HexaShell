# libero_spatial_task_t4

## Judging
- The bowl standing on the cabinet's top (top ~0.265, x 0.64-0.72, y -0.28..-0.31): set on the
  plate (x 0.70-0.73, y +0.19..+0.22) it ended the episode (6 eps, 2 probes). The bowl in the
  open drawer (top ~0.20) was never tried here.

## Flow
1. Free: measure the bowl and the plate (2 views). The bowl's agentview-only centre sat
   0.2-2.2 cm to -x and 0.5-1.6 cm to +y of its 2-view one (10 eps): measure again after step 2.
2. The -x rim: `robot.goto((b.x - 0.04, b.y, 0.31), pitch=90, yaw=90, grip='open')`, measure
   the bowl again (the wrist sees it) and use that centre; straight down to (b.x - 0.04, b.y,
   0.25), `robot.gripper('close')` (2 probes).
3. Up to 0.31, level to (p.x - 0.04, p.y, 0.31), open there: it drops onto the plate and the
   episode ends within 7 steps (2 probes; 8 eps in this scene opened 0.20-0.33 up over it).

## Verified parameters
- Hand 1.5 cm below the bowl's top and 4 cm to -x of its 2-view centre, yaw 90 (2 probes).

## Common failures and fixes
- All 10 grasped a y-side rim at yaw 0 (9 the +y one): approaches 40-62, carries 61-84, most
  then a descent over the plate before opening; the 6 successes ended at 194-219 of 220 (10 eps).
- Slanted carry-descents toward the plate were BLOCKED ~0.22 up at y -0.03..+0.02 (2 eps); a
  long carry plus a slow descent ran out of steps (2 eps). Carry level at 0.31 (2 probes).
- With the yaw-0 grasp, the hand over the plate at p.y + 0.04, 0.30 up or higher, was refused
  as stretched (3 eps); the -x rim grasp needs no y offset (2 probes).
- A 'slipped out' note after the carry was wrong: the bowl still hung in the hand (9 eps).
  Measure it (free) before reacting.

## Typical step costs
- Horizon 220. Probes 147-150: approach 44, down 18-19, close 12, up 15, carry 51-54, open 6-7
  (2 probes). Learning successes 194-219 (6 eps).
