# libero_goal_t3

## Judging
- The dark cabinet stands to -y (left in agentview), 3 drawers facing +y; the top handle is highest
  there. Pulled 15 cm to +y it opened; the bowl lowered into that drawer: SUCCESS (164 eps).
- The bowl stood under the open drawer's edge (probes). Route: 270-295 of 300 steps (146 eps).

## Flow
1. Free, before any motion: `b = robot.measure('bowl')`: round, 11-12 cm, top 0.036-0.040, keep
   x0, y0 = b.center[:2]; not round or 6 cm+ tall: odd, see step 4. `ms = robot.find('agentview',
   'drawer handle')`, `top = min(ms, key=lambda m: m.centroid[1])`, `h = robot.measure(top)`,
   `sp = robot.to_world('agentview', top.centroid, z=0.18)`; h is far off yet (126 probes).
2. `robot.goto((sp[0], sp[1] + 0.09, 0.20), pitch=0, yaw=-90, roll=-90, grip='open')`, `h =
   robot.measure(h)`: 2 views ±1 mm, 8 x 3 x 1.6 cm, top 0.185; keep hx, hy, hz = h.center
   (probes). Taller, or NOT executed: see below (17 eps). Pointing down: BLOCKED (92 of 126 eps).
3. `robot.goto(h.center)`, close (holding 1.7-2.9 cm), `robot.goto(by=(0, 0.15, 0))` (88 probes).
4. `robot.goto((x0, y0 + 0.135, 0.22), pitch=90, yaw=0, roll=0, grip='open')`; an odd bowl: `b2 =
   robot.measure(b)` here, if unwarned, round, at most 6 cm tall: x0, y0 = b2.center[:2], go above
   it: SUCCESS 284-293 (3 of 3 probes; kept b: empty, 5 of 5; b again from the start: same, 5).
   `m = robot.goto((x0, y0 + 0.045, 0.025))`, astride the +y wall; if m.ok, close: 0.8 (88 probes).
5. Out `robot.goto(by=(0, 0.09, 0.02))`, up to (x0, y0 + 0.135, 0.30), across to (hx, hy + 0.08,
   0.30), `print(robot.measure('drawer'))` (no steps; see below), then `by=(0, 0, -0.10)`: SUCCESS
   on the way, hand ~0.26 (146 eps; 273-275, 10 of 10 probes).

## Verified parameters
- Keep x0, y0: re-measured with the drawer open the bowl read 1-3 cm off, 3-5 cm high (115 probes).
- "What they held slipped out" with the bowl held: usually false (119 of 164 eps); check the wrist.
- Grips of 3-4 cm missed (SUCCESS 4 of 15 probes; 'open' 12). Across at hz + 0.07: none (27 probes).

## Common failures and fixes
- Step 2 read 19-38 cm tall: three handles as one. Point at it in both pictures, `robot.to_world(
  'agentview', px, pair=('wrist', px2))`: 0.276-0.279 = hz; held 1.7, pulled (5 of 13 eps). Lift
  the bowl to 0.39-0.40, cross at 0.34-0.39: SUCCESS at 281-293 (2 of 3 eps).
- Step 2 NOT executed, sp[0] 0.79-0.80: pitch 0, yaw -60, x 2 cm less: held 1.7, pulled (7 eps).
  Over the drawer 0.30 out of reach: x 3-4 cm less, across at 0.26, to 0.19: SUCCESS ~282 (2 eps).
- Down onto the bowl BLOCKED at z 0.09-0.10: no close (8 cm, empty, 3 eps), no out and back (bowl
  pushed, closed empty, 7 eps). The same goto, 2-3 times: held 0.8, SUCCESS 296-297 (5 of 5 probes).
- Down onto the bowl "short of target" 29 mm BELOW it: holding 6.3-7.8 cm, all ran out (10 eps).
  If m.at z < 0.015: the same goto again, then close: SUCCESS at 296-299 (10 of 10 probes).
- Over there 'drawer' read x at least hx + 0.09, top 0.20-0.24: it moved; lowered first: ran out (9
  eps). `robot.goto((hx + 0.10, hy, 0.23), pitch=60, yaw=0)`, open: SUCCESS 279-281 (5 of 5 probes).

## Typical step costs
- (146 eps): level 44-54, handle 17-20, close 9-11, pull 24-28, above bowl 46-48, down 27-28,
  close 9-11, out 16, up 31-32, across 33-39, lower 6-9; a 90-degree turn = 37 cm of travel.
