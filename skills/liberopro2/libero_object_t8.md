# libero_object_t8

## Judging
- The chocolate pudding that counted is the small brown box low in the left part of the agentview
  picture, about (134, 250). From the start it read 8.2 x 5.1 cm, top 0.057-0.059; from above 8.0
  x 4.7 cm, top 0.026-0.027 (119 probes). 'chocolate pudding' by name found nothing (133 calls).
- SUCCESS came while held, 6-16 steps into the lowering over the basket. Not it: the orange-juice
  carton, 13 cm tall. No brown box, a red box 14.5-15 cm tall at that spot: it counted.

## Flow
1. Free: `p = robot.measure(robot.find('agentview', at=(134, 250))[0])`; `b = robot.measure(
   'basket')`: warned, with a pose to look from (190 of 215 eps).
2. Go to that pose (34-44 steps), `b2 = robot.measure(b)`: use it if unwarned and 9 cm tall or
   more, else the first reading's "may stand ... at (x, y)" (74 of 129 probes; not looking 68).
3. `robot.goto((p.center[0], p.center[1], 0.18), pitch=90, yaw=0, grip='open')` (46-56 steps),
   `p = robot.measure(p)`: top 0.026-0.027 (119 probes).
4. `robot.goto((p.center[0], p.center[1], 0.018))`: reached 0.009-0.023 in 24-27 steps (98 of 124
   probes); `robot.gripper('close')`: holding 4.6 cm (105 of 124 probes).
5. Up to 0.24. With step 2's basket centre (x, y): `robot.goto((x, y - 0.19, 0.175))`, `robot.goto(
   (x, y, 0.175))`, down toward 0.10: SUCCESS on the way (63 probes; no case lost against across
   at 0.24, 3 steps more). Not read again on the way: 1-2 cm off, no SUCCESS there (6 eps).

## Verified parameters
- A look-down "agree only roughly", 17 x 6 cm, "on something": lowering there BLOCKED (12 probes).

## Common failures and fixes
- A white round plate falls on the basket as the hand nears, not seen from 19 cm off. Across at
  0.24 and down: BLOCKED at 0.17-0.20 (38 eps); moves from on top stayed on it (34 eps). Step 5's
  way in at 0.175 pushed it off: SUCCESS 221-238 (7 of 7 probes).
- The move to the look pose BLOCKED at 0.26-0.28 by a thin panel (10 eps): straight to 0.18 over the
  box, take it, up to 0.43, across at 0.43, down to 0.14: SUCCESS at 196-231 (8 of 8 probes).
- First basket reading under x 0.40, raised: from 50 cm ahead, 30 cm to +y, 46 cm up, yaw 0 it
  read top 0.32-0.35; carried at 0.37-0.40, opened 6 cm nearer than its centre: SUCCESS (7 eps).
- The close found nothing, the box moved 8 cm: found again, at its grip_yaw (7 eps). "closed on ~7.8
  cm ... nearly fully open" (holding True): empty. Read again: ran out (4 eps); same x, y (11 eps).
- Over the basket the lowering stopped "short" at 0.130, 3.5 cm off: it had moved; re-readings were
  off (7 eps). `robot.goto((x - 0.033, y + 0.066, 0.175))`, open: SUCCESS 261-262 (5 of 5 probes).
- A move down BLOCKED after 9-19 steps, the hand not moved: measured, sent again, held (5 eps). A
  1-view start reading of 1 x 1 x 7 cm is off (6 probes); measure from above, or to_world (8 eps).
- Start reading "agree only roughly", 6 x 4 x 2.5 cm: by wrist spots; closed at 0.038, held (5 eps).
- The red box: at its grip_yaw, 3 cm under its top: 4.4 cm (13 eps). Not step 5's way: up to 0.34,
  across to (x + 0.03, y - 0.01), down to 0.15: SUCCESS 247-254 (5 of 5 eps); at (x, y) BLOCKED (5).

## Typical step costs
- Horizon 280. Look 34-44, above 46-56, down 24-27, close 8, up 29, in 37 + 25, down 7 (63 probes).
