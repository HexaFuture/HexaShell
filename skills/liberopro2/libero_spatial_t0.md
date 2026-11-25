# libero_spatial_t0

## Judging
- The bowl that counted lay between the ramekin and the plate, the plate 11-17 cm +x of it; the
  other black bowl stood about 12 cm nearer the robot and 14 cm to +y (100 eps). In the agentview
  picture it is at about (383, 290): 11 x 11 cm, round, top 0.035-0.037 (88 of 120 probes).
- SUCCESS came held over or on the plate (11 probes, 47 eps), or 2-3 steps into the open (49 eps).

## Flow
1. Free: `p = robot.measure('plate')` ('ramekin' found nothing, 29 calls); `b = robot.measure(
   robot.find('agentview', at=(383, 290))[0])`: often "rests on something 34 mm" up (67 eps).
2. `robot.goto((b.center[0], b.center[1] - 0.044, 0.17), pitch=90, yaw=0, roll=0, grip='open')`
   (27-35 steps), `b = robot.measure(b)`: on the table (120 probes).
3. On its -y rim: `robot.goto((b.center[0], b.center[1] - 0.044, 0.026))` (21-33 steps),
   `robot.gripper('close')`: holding 0.8-0.9 cm (101 of 122 closes, probes).
4. `robot.goto(by=(0, 0, 0.12))` (20-26), `p = robot.measure(p)`, then `robot.goto((p.center[0],
   p.center[1] - 0.044, 0.14))` (20-35), down to 0.05 (17-24), `robot.gripper('open')`,
   `robot.goto(by=(0, 0, 0.08))` (SUCCESS in 77 of 126 probes).

## Verified parameters
- Held at yaw 0 on the -y rim (55 eps) or the +y rim (36 eps), or yaw 90 on the +x rim (7 eps):
  hand 4.0-4.7 cm out, 0.7-1.7 cm below the measured top, holding 0.7-0.9 cm; released over the
  plate's centre, offset kept, at hand z 0.048-0.071 (100 eps); a raised plate: z 0.102 (2 eps).

## Common failures and fixes
- A descent BLOCKED and pulled 5 cm back, with a note that the pictures lag the commands: up 1-6
  cm, measured again, down again: held, SUCCESS at 164-181 (5 eps).
- A descent "short of target" with the hand 2.9 cm below the asked height: up to 0.052, down to
  0.028, close: held 0.8-1.0 cm, SUCCESS at 152-155 (3 eps); closed elsewhere: 177-180 (2 eps).
- Readings disagreeing after a camera-check line (the bowl "resting" 4-21 cm up, 2-8 cm jumps):
  descents stopped 3.5-4.6 cm up; closed there, held 1.2-2.0 cm: SUCCESS at 166-214 (4 of 5 eps).
- "closed fully (empty)" after a normal descent: measured again, the bowl had moved 7-12 cm; its
  new -y rim at 0.026, yaw 0: held, SUCCESS at 177-181 (5 eps).
- "slipped out" on the lift or the carry (18 eps): the bowl still hung in the fingers, SUCCESS on
  carrying on (4 eps), or fell onto the plate, SUCCESS at once (7 eps); fallen elsewhere, a descent
  and open chained on `.ok` still ran (16-29 steps) and none recovered (3 eps): look first.
- A tall board between bowl and plate: lifted to 0.27-0.30 and carried over it: SUCCESS at 145-162
  (5 eps); at 0.15 BLOCKED and the bowl fell (1 ep).
- A new box on the plate's centre after the lift: swept +y, the plate went along, none (11 eps);
  bowl on its centre line, pushed +x 9-13 cm at hand z 0.035-0.072: SUCCESS at 209-214 (3 of 4 eps).

## Typical step costs
- Horizon 220. Probed route: hover 27-35, down 21-33, close 8-12, up 20-26, across 20-35, down
  17-24: SUCCESS at 102-145 (77 probes). With this note 111-215, median 127 (201 of 210 eps); no
  note 109-217, median 149 (107 of 126 eps). A second close: SUCCESS at 177-215 (8 of 11 eps).
