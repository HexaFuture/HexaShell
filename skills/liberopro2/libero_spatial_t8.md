# libero_spatial_t8

## Judging
- The bowl that counted stood 11-12 cm +y and 3-10 cm -x of the plate; the other black bowl was
  25 cm away (95 eps). In the agentview picture it is right of the plate, about (468, 328).
- Its first reading is 1 view, "height guessed", up to 6 cm off (95 eps). SUCCESS came with the
  bowl still held, lowered onto the plate (44 eps), or 2-3 steps into the open (47 eps).

## Flow
1. Free: `p = robot.measure('plate')`; `b = robot.measure(robot.find('agentview', at=(468,
   328))[0])`: warned, with a pose to look from (117 of 126 probes). Go there 2-4 cm higher
   ("at" in 117 of 119 eps), `b = robot.measure(b)`: unwarned (116 of 117 eps); keep if within 6 cm.
2. `robot.goto((b.center[0], b.center[1] - 0.044, 0.12), pitch=90, yaw=0, roll=0, grip='open')`
   (21-38), down to 0.025 on the -y rim (17-33), `robot.gripper('close')`: holding 0.9-1.1 cm
   (93 of 116 closes, probes).
3. `robot.goto(by=(0, 0, 0.12))` (21-24), `p = robot.measure(p)`, `robot.goto((p.center[0],
   p.center[1] - 0.044, 0.13))` (15-39), down to 0.05 (15-22), `robot.gripper('open')`,
   `robot.goto(by=(0, 0, 0.08))` (SUCCESS in 83 of 126 probes).

## Verified parameters
- Held: yaw 0 on the -y rim (48 eps); yaw 27-28 with the hand 2.0 cm +x and 3.8 cm -y of the
  centre (36 eps); yaw 90 on the -x rim (17 eps). Hand 0.9-1.9 cm below the measured top, holding
  0.7-1.3 cm; released with the offset kept at hand z 0.047-0.065 (95 eps).

## Common failures and fixes
- "slipped out" at the end of the carry: SUCCESS at once (12 of 20 eps); else the open stopped at
  6.2 cm: press with the open hand 2.5-3.7 cm nearer the plate's centre, z 0.022-0.046 (5 of 8 eps).
- "closed fully (empty)": bowl ~8 cm further out; -y rim, yaw 0, pitch 70-76: SUCCESS (2 of 3 eps).
- Readings disagreeing after a camera-check line (the bowl "resting" 5-19 cm up, 5-14 cm jumps):
  descents stopped 2-3 cm up; closed there, held 1.4-3.1 cm: SUCCESS at 162-211 (11 of 15 eps).
- The plate read about 16 cm up: yaw-0 descents by the bowl BLOCKED at z 0.08-0.15 (13 eps); yaw 90
  from z 0.16-0.30 down to the -x rim held (16 eps), but lifted straight up it "slipped out" (14 of
  16 eps); 7 cm further -x a lift held (4 eps). So `robot.goto(by=(-0.07, 0, 0.04))`, up to 0.25,
  across to 4.5 cm -x of the plate's centre, down to 0.20 (SUCCESS 172, 1 ep; 0 of 30 probes).
- A dark board in the pictures but no move BLOCKED: keep the Flow: SUCCESS at 136-185 (12 eps); +y
  rim and a 0.14-0.30 lift: 201-219 (8 of 12 eps), the rest out of steps.
- A board reading top about 0.31 by the bowl's -y side: moves near it BLOCKED (33 eps); +y rim held,
  carried at 0.03-0.13 BLOCKED (15 eps); up 0.23, across at 0.25 over the plate: SUCCESS 217 (1 ep).
- A new box on the plate after the lift (89-103 steps left): swept -y or set on it, none (20 eps).
  At once `robot.goto((p.center[0] - 0.12, p.center[1] - 0.044, 0.06))`, +x to p.center[0] + 0.03
  rising to 0.09, asking at most x 0.75 at pitch 90; open (untested as one route; 1 of 3 eps).

## Typical step costs
- Horizon 220. Probed: look 43-51, hover 21-38, down 17-33, close 9-11, up 21-24, across 15-39,
  down 15-22: 141-193 (83 probes). This note 125-215, median 151 (180 of 210 eps).
