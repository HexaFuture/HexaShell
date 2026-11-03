# libero_10_task_t3

## Judging
- The bottle (~14.5 cm tall, ~3.5 cm wide) stands ~13 cm to -x of the bottom drawer's low bar
  (~4 cm up); the drawer starts open ~15 cm, its box to +y of the bar. Done = the bottle lying
  inside and the drawer pushed shut to +y: in our runs the episode ended during that push.
  Standing, leaning or propped on a wall, it stuck out and the drawer never closed.

## Flow
1. Free: `robot.measure('bottle')` (x 0.50-0.54, y +0.025..+0.065, top ~0.145) (10 eps).
   The bar h: point at the lowest bar in the agentview picture, `robot.measure(('agentview',
   (u, v)))`; expect it ~4 cm up, x 0.65-0.66, y about +0.05 (3 probes).
2. Lay it down first: `robot.goto((b.x + 0.05, b.y, 0.20), pitch=90, yaw=0, grip='close')`,
   down to 0.11, `robot.goto(by=(-0.13, 0, 0))`, `robot.wait(15)`: it falls toward -x and lies
   along x, its foot about where it stood; look to check (4 probes).
3. Open; `robot.goto((b.x - 0.078, b.y, 0.012), pitch=90, yaw=0)`, close across it (3 probes).
4. Up to 0.20, level to (h.x + 0.03, h.y + 0.08, 0.20), down toward 0.07: BLOCKED at 0.15-0.18
   is normal; open, `robot.wait(10)`: it lies flat in the drawer (3 probes).
5. Push fingertips first, pointing to +y (yaw 90), tilted down: closed hand, pitch=30, yaw=90:
   (h.x, h.y - 0.09, 0.16) (may end short), then (h.x, h.y - 0.04, 0.045), then
   (h.x, h.y + 0.19, 0.045): it stops ~13 cm to +y of h and the episode ends (3 probes).

## Verified parameters
- The one learning success: the bottle fell over by chance, was grasped lying, dropped flat and
  pushed in with a tilted hand (1 ep).

## Common failures and fixes
- Tilting the held bottle to lay it in (pitch 0-45 at the drawer): not reachable, BLOCKED,
  short or refused, in all 10 (10 eps). Lay it on the table first instead (3 probes).
- Released upright, leaning or propped (tops 0.08-0.17 up): the drawer never closed (9 eps).
- `robot.measure('bottom drawer')` gave a box well off the table in 7 of 7 tries (7 eps).

## Typical step costs
- Horizon 520. Probes 318-322: knock 70-74, open 14, down 18-20, close 10, up 26, carry 33-35,
  lower 10-13, open 14, wait 10, push points 49-50 + 41, push 18-19 (3 probes).
- The one learning success took 502 (1 ep).
