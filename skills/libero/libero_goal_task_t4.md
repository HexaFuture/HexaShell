# libero_goal_task_t4

## Judging
- Put the plate (the flat disc, 11.7 cm across, to +y of the dark cabinet that stands left in
  the agentview picture, to -y) on top of that cabinet. Carried over the cabinet and lowered,
  it ended the episode with SUCCESS while still held (10 probes, 6 eps).
- Grips on the plate's -x rim, the fingers closing along x, caught the black bowl that stands
  there instead of the plate (2 eps).

## Flow
1. Free, before any motion: `p = robot.measure('plate')` (11.7 cm across, top 0.005, 10 eps)
   and `c = robot.measure('cabinet')` (about 23 x 16 cm, top 0.215; 'drawer cabinet' gave the
   same centre) (13 eps). Keep px, py = p.center[:2] and cx, cy = c.center[:2].
2. Pointing down, fingers closing along y: `robot.goto((px, py + 0.05, 0.10), pitch=90, yaw=0,
   roll=0, grip='open')`, then `robot.goto((px, py + 0.05, 0.007))`, the fingers astride the
   +y rim (the hand stops near 0.010); `robot.gripper('close')` (10 probes).
3. Straight up: `robot.goto((px, py + 0.05, 0.37))`; across: `robot.goto((cx, cy + 0.05,
   0.37))`, the plate centre then over the cabinet centre (10 probes).
4. `robot.goto(by=(0, 0, -0.08))`: SUCCESS came 6-8 steps in, about 2 cm down, before letting
   go (10 probes). If not: open there and look (untested).

## Verified parameters
- Grasp height: asked 0.010 (the hand stopped at 0.013) the plate stayed on the table, 10 of
  10; asked 0.006-0.009 it came along every time (12 probes).
- The +y rim at yaw 0 closed on 1.5-1.7 cm and held through the carry (5 eps).
- Carry at 0.37: learn1 carries at 0.37-0.40 reached the cabinet (7 eps).

## Common failures and fixes
- The plate's -x rim at yaw 90: closed empty on 0.3 cm (1 ep) or caught the bowl; putting the
  bowl back and regripping cost 100-130 steps (2 eps). Use the +y rim as in step 2.
- Pointing down at the plate's -y rim, toward the cabinet: jammed 11-13 cm short (2 eps).
- Carry targets far out to -y (near y -0.27) were refused as stretched (2 eps); cy + 0.05 was
  always reachable (10 probes).
- Level approaches to the rim at pitch 20-35: BLOCKED 4.6-10.5 cm short (1 ep).

## Typical step costs
- Horizon 300. Route above 162-171: above the rim 42-45, down 16, close 11, up 42-45, across
  40-50, lower 6-8 (10 probes).
- learn1 successes took 183-278 (7 eps); the 3 failures ran out at 272-300 (3 eps).
