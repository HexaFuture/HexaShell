# libero_10_object_t8

## Judging
- Two gold moka pots. Pointed at, one in the middle of the agentview picture (x about 275-312,
  y 332-351) measured 0.70-0.74 ahead, 0.01-0.06 to +y; one right in that picture (x 401-439,
  y 284-304) 0.59-0.64 ahead, 0.21-0.26 to +y; both top 0.124-0.125, about 0.11 x 0.05
  (19 eps; one more measured them by name, the same).
- `robot.measure('stove')`: 0.60-0.63 ahead, 0.18-0.21 to -y, about 0.18 across, top 0.019
  (20 eps). Done when both pots stand on it: SUCCESS came as the second one came down onto it,
  still held, or was let go there (13 eps, 4 probes).

## Flow
1. Free: `p1 = robot.measure(('agentview', (x, y)))` on the middle pot, `p2` the same on the
   other, `s = robot.measure('stove')` (20 eps).
2. Knob of p1: `k = (p1.center[0] - 0.006, p1.center[1] + 0.017)`. One move turns and travels:
   `robot.goto((k[0], k[1], 0.25), pitch=90, yaw=90, grip='open')`; then straight down to
   `(k[0], k[1], 0.134)`, no lower; `robot.gripper('close')` (about 1.2 cm); up to 0.25 (4 probes).
3. Across at 0.25 to over `(s.center[0] + 0.04, s.center[1])`, down to 0.165, open, up to 0.25
   (4 probes).
4. p2 the same way, then over `(s.center[0] - 0.05, s.center[1])`, down to 0.165, open: SUCCESS
   came on that open (4 probes).

## Verified parameters
- Knob tops seen from above lay 0.001-0.009 to -x and 0.012-0.026 to +y of the pointed centre
  (35 pots, 20 eps). Every descent onto a pot aimed at 0.131-0.139 got there and closed on the
  knob at 1.1-1.5 cm, once 2.9 (8 eps; 4 probes). In libero_10_lan_t8, pots measuring the same,
  20 of 20 such grasps held at 1.1-1.2 cm, once 2.4 (10 eps).

## Common failures and fixes
- Descents aimed 1.5-5 cm below 0.134 stopped where the fingers touched (29 descents, 20 eps):
  on the lid at 0.128-0.131, closing on the knob or lid at 1.2-5.1 cm or in a 0.7-0.9 cm pinch
  that slipped (14 descents, 13 eps), or past the lid at 0.107-0.126, closing at 7.5-8.1 cm, the
  fingers nearly fully open (15 descents, 12 eps).
- Let go over the stove, such a pot stayed wedged between the open fingers and rose with the
  hand in 9 of 14 (12 eps). Freeing it took 84-146 steps (4 eps); carried over or turned beside
  the other pot, it knocked that pot over or off the stove (3 eps). Stop at 0.134 (4 probes).
- Turning toward yaw 90 on the way down closed at 7.5-8.1 cm in 9 of 10 first grasps; turned
  above it first, then sent as low, 3 of 9 (19 eps). Turn on the move to 0.25 above (4 probes).
- At yaw 0 the close found nothing in 7 of 9 descents (7 eps); yaw 90 held the knob (4 probes).

## Typical step costs
- Horizon 520. First pot: over it with the turn 50-56, down 19, close 11, up 21, across 30-32,
  down 16, open 14, up 17; second pot 43-47, 19, 11, 20-21, across 42-46, down 16, open 2:
  337-342 (4 probes). The 13 successes took 345-510 (13 eps).
