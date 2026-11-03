# libero_goal_task_t1

## Judging
- `robot.measure('plate')`: round, 11.6-11.8 cm across, top 0.005, 0.69-0.72 ahead, within
  3.5 cm of y 0. It also noted another 'plate' 11-15 cm to -x (10 eps): a dark bowl standing
  against the plate's -x side (start picture).
- `robot.measure('stove')`: 0.40-0.42 ahead, 0.20-0.215 to +y, about 0.18 across (10 eps).
- SUCCESS came as the plate, still held, came down over the stove (3 eps, 4 probes) or was let
  go just above it (1 ep). A plate that slipped out on the way and landed leaning on the stove's
  edge did not count (1 ep, picture).

## Flow
1. Free: `pl = robot.measure('plate')`, `st = robot.measure('stove')` (10 eps).
2. Pinch the +y rim, away from the bowl: `robot.goto((pl.center[0], pl.center[1] + 0.05, 0.10),
   pitch=90, yaw=0, grip='open')`, down to 0.006 with speed=0.1, `robot.gripper('close')`: held
   1.6-2.4 cm (3 eps) and carried (4 probes). Then `robot.goto(by=(0, 0, 0.13))`.
3. Across to over `st.center` at 0.175, then down toward 0.135: it stopped short and SUCCESS
   came (4 probes). Keep holding until then.

## Verified parameters
- After the lift the fingers read 0.6-1.0 cm, or "closed fully: slipped out", while the plate was
  still held and reached the stove (4 eps): look before letting go or regrasping.

## Common failures and fixes
- The yaw 90 pinch on the -x rim, about 5 cm to -x of the plate's centre, once carried the bowl
  (1 ep, picture); 3 runs let go believing so and lost 85-95 steps each (3 eps); 2 runs carried
  the plate (2 eps). The +y rim pinch carried the plate (2 eps, 4 probes).
- Yaw 0 pinches aimed at the -y rim never got there: the hand stopped 3-4.5 cm short of it,
  0.08-0.15 up, short or BLOCKED (5 eps). Use the +y rim.
- Regrasping again and again after the notes above: finish with 14-29 steps left (3 eps).
- A +x-rim pinch at 1.8 cm slipped out during the carry at 0.12 up; the plate landed leaning on
  the stove's edge and no SUCCESS came (1 ep, picture): lower it while held (4 probes).

## Typical step costs
- Horizon 300. Over the rim 38-45, down 24-25, close 10-11, up 23, across 40-41, down 5: 141-148
  (4 probes). The four learn1 successes took 226-276 (4 eps).
