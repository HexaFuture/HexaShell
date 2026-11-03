# libero_spatial_lan_t4

## Judging
- The bowl in the open top drawer (top ~0.20, x 0.73-0.76, y -0.125..-0.155): set on the plate
  (x 0.70-0.72, y +0.19..+0.21) it ended the episode (6 eps, 1 probe). The bowl on the cabinet's
  top (top ~0.265) was never tried here.

## Flow
1. Free: `robot.find('agentview', 'black bowl')`, measure the drawer bowl and the plate (2 views)
   (10 eps).
2. The -x rim: `robot.goto((b.x - 0.04, b.y, 0.30), pitch=90, yaw=90, grip='open')`, straight
   down to 0.188, `robot.gripper('close')`: ~0.8-0.9 cm, holding (1 probe, 4 eps).
3. Up to 0.30, level to (p.x - 0.04, p.y, 0.30), open there: it drops onto the plate and the
   episode ends within 7 steps (1 probe; 8 eps in this scene opened 0.20-0.33 up over it).

## Verified parameters
- The agentview-only centre of the drawer bowl was within 0.4 cm of the 2-view one (10 eps).
- The -x rim at yaw 90: successes at 177-184 steps (4 eps); the +y rim at yaw 0: 210-211 (2 eps).

## Common failures and fixes
- A yaw-0 grasp on the +y rim: descents of 62-67 steps (4 eps), or stopped ~0.22 up above the
  bowl and closed empty (2 eps). Use the -x rim at yaw 90 (1 probe, 4 eps).
- The +x rim at yaw 90 was refused as stretched (1 ep).
- Slanted carry-descents toward the plate were BLOCKED ~0.22 up at y -0.05..+0.03; one opened
  there and lost the bowl off the plate (2 eps). Carry level at 0.30 instead (1 probe).
- A 'slipped out' note after the lift or carry was wrong: the bowl still hung in the hand
  (9 eps). Measure it (free) before reacting.

## Typical step costs
- Horizon 220. Probe 147: approach 45, down 23, close 12, up 22, carry 39, open 6 (1 probe).
- Failures: 62-67-step yaw-0 descents plus 26-58-step slanted carries, or 1-4 empty closes,
  used up the 220 (4 eps).
