# libero_goal_swap_t5

## Judging
- "The front of the stove" (agentview front, +x of the stove): the plate, lifted by its rim and
  carried there, ended the episode with SUCCESS while still hanging from the fingers, the grasp
  point 16-18 cm +x and 0.5-2 cm -y of the stove's measured centre (3 probes).
- The words say push, but the lifted and carried plate counted as well (3 probes); so did a
  push in three legs (1 ep).

## Flow
1. `plate = robot.measure('plate')`, `stove = robot.measure('stove')`: plate 0.116 m round, 1 cm
   thin, top 0.005, centre 0.43-0.47 m ahead, 4-6.5 cm to -y; stove centre 0.40-0.42 ahead,
   0.20-0.22 to +y (10 eps). So the plate must go about 17 cm +x and 24 cm +y. The wooden rack
   stands close beside its -y side.
2. Take the plate by its +y rim, away from the rack. With px, py the plate's centre:
   `robot.goto((px, py + 0.05, 0.11), pitch=90, yaw=0, grip='open')` (not grip_yaw), straight
   down to z 0.004, `robot.gripper('close')` (3 probes).
3. `robot.goto(by=(0, 0, 0.10))`: the plate tips and hangs from its rim; it holds (3 probes).
4. With sx, sy the stove's centre: `robot.goto((sx + 0.19, sy + 0.02, 0.14))`. SUCCESS came
   21-23 steps into this move, before it ended, nothing hit on the way (3 probes).
5. No SUCCESS when that move ends: lower until the plate touches the table, open, measure it and
   push it on in short legs (untested).

## Verified parameters
- Grasp point on the plate's centre in x, 5 cm +y of it, closed at z 0.004-0.005; carried at
  z 0.13-0.14 it came along every time (3 probes).
- The one success pushed in three legs, the grasp point at z 0.005-0.013: 15 cm +x from the -x
  rim at yaw 0, 18 cm +y from the -y rim at yaw 90, then 4 cm more +y (1 ep).

## Common failures and fixes
- Descents beside the plate's -y or -x rim, 5-9 cm from its centre, were BLOCKED 5-12 cm up by
  the rack, often after a yaw turn of 50-180 degrees (9 eps). Its +y rim is clear (3 probes).
- The other pushes moved the plate only 4-13 cm before the steps ran out (6 eps). Two that moved
  it 12-19 cm ended with it tilted, measured 2.4-6.5 cm tall, and no SUCCESS (2 eps).

## Typical step costs
- Horizon 300. Lift route: pre-pose 23, down 18-19, close 13-14, lift 17, carry 21-23: SUCCESS
  at 92-96 (3 probes). The push success took 259 (1 ep); the failures ran out at 272-300 (9 eps).
