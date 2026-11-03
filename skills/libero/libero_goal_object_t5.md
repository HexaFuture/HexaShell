# libero_goal_object_t5

## Judging
- "The front of the stove" (agentview front, +x of the stove): the plate, lifted by its rim and
  carried there, ended the episode with SUCCESS while still hanging from the fingers, the grasp
  point 19-22 cm +x and 1-3 cm -y of the stove's measured centre (6 probes, 3 eps).
- The words say push, but the lifted and carried plate counted as well (6 probes, 3 eps); so
  did a push (1 ep, 1 probe).

## Flow
1. `plate = robot.measure('plate')` (a yellow plate), `stove = robot.measure('stove')`: plate
   0.117 x 0.10 m, 1 cm thin, top 0.005, centre 0.69-0.72 m ahead, 1-3.5 cm to -y; stove centre
   0.40-0.42 ahead, 0.20-0.22 to +y (20 eps). A cabinet stands close beside the plate's -y side.
2. Take the plate by its +y rim, away from the cabinet. With px, py the plate's centre:
   `robot.goto((px, py + 0.05, 0.11), pitch=90, yaw=0, grip='open')` (not grip_yaw), straight
   down to z 0.004, `robot.gripper('close')`: holding at 2.4-3.1 cm (4 eps; 6 probes).
3. `robot.goto(by=(0, 0, 0.10))`: the plate tips and hangs from its rim, and the gap drops to
   0.6-1.2 cm with a note that it changed. It holds: go on (4 eps; 6 probes).
4. With sx, sy the stove's centre: `robot.goto((sx + 0.19, sy + 0.02, 0.14))`. SUCCESS came
   19-20 steps into this move, before it ended (6 probes).
5. No SUCCESS when that move ends: lower until the plate touches the table, open, measure it and
   push it on in short legs (untested).

## Verified parameters
- Grasp point on the plate's centre in x, 5 cm +y of it, closed at z 0.003-0.007; carried 10-15
  cm up it came along every time (6 probes, 3 eps).
- Pushing at yaw 90 with the grasp point at z 0.000-0.005 moved the plate 70-98% as far as the
  hand went (4 probes). Other pushes, higher (z 0.007-0.015), tipped (pitch 45-80 or 125) or
  at yaw 0, mostly slid over its rim: it moved under 1 cm (6 eps, 1 probe), 1.8-4.4 cm (4 eps),
  16 cm once (1 ep).

## Common failures and fixes
- The first move from the start pose to low over the plate's -y half hit the cabinet halfway,
  0.54-0.61 m ahead and 12-20 cm up: BLOCKED, 31-43 steps lost (15 eps). The +y rim is clear.
- Coming down over the plate's -y edge, the cabinet's handles stopped the hand: at yaw 0 18-19 cm
  up (7 eps); at yaw 90 8-9 cm up, 6-9 cm -y of the plate's centre (6 eps).
- Poses 0.77-0.81 m ahead, beyond the plate, were refused as a stretched arm (10 eps).
- The one success here pushed: yaw 90, the closed fingers at the plate's -y rim at z 0.000, one
  diagonal leg toward the stove's +x side at speed 0.20 (1 ep, 1 probe). The same push elsewhere
  moved the plate 19-21 cm with no SUCCESS (2 probes): lift instead.

## Typical step costs
- Horizon 300. Lift route: pre-pose 38-43, down 18, close 12-13, lift 19, carry 19-20: SUCCESS
  at 106-112 (6 probes). The push success took 240, 105 when replayed without its detours (1 ep,
  1 probe); the failures ran out at 289-300 (9 eps).
