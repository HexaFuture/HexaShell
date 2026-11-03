# libero_10_swap_t9

## Judging
- `robot.measure('yellow and white mug')` found the mug 0.63-0.69 ahead, within 3.5 cm of y 0,
  top 0.09 (10 eps). Put inside with the door shut, it ended the episode with SUCCESS (4 probes).
- The microwave stands at -y (left in the agentview picture), top about 0.195 up. Its opening
  faces -y; the door hangs on the opening's -x edge (nearer the robot) and swings out toward -y.
  The words 'microwave' found nothing (9 eps).
- SUCCESS came as the closing hand stopped against the shut door, the mug inside (4 probes).
  Every learn1 run ran out of steps, or stopped with 10-48 left (10 eps): waste no move.

## Flow
1. Free first: `m = robot.measure('yellow and white mug')`. Hinge hx, hy: to_world spots on the
   edge where the open door meets the box, below the box's top, lay 0.37-0.40 ahead, 0.34-0.36
   to -y (8 eps). A spot on the box's top corner read 1.7 cm more to -x than one lower down, and
   from it the mug was BLOCKED on the way in (2 probes): use the lower spot.
2. `robot.goto((m.center[0] - 0.04, m.center[1], 0.16), pitch=90, yaw=90, grip='open')`, down to
   0.075, `robot.gripper('close')` on the mug's -x wall, straight up to 0.32 (4 probes).
3. Across high: `robot.goto((hx + 0.087, hy - 0.10, 0.32), pitch=90, yaw=90)`, down to 0.20
   (4 probes). Low and straight from the mug, or from the door, it was BLOCKED (2 eps, probe).
4. Turn and go in with one move at the default speed: `robot.goto((hx + 0.113, hy + 0.004,
   0.125), pitch=0, yaw=60, roll=90)`, 89-117 steps (4 probes); `robot.gripper(0.6)`.
5. Out: same pitch, yaw and roll to `(hx + 0.073, hy - 0.065, 0.125)` (4 probes).
6. Close as for a door at 120 (4 probes): `t = math.radians(-120)`; `P = (hx + 0.20*math.cos(t)
   + 0.05*math.sin(t), hy + 0.20*math.sin(t) - 0.05*math.cos(t))`; from the exit straight over P
   at 0.26 with pitch=90, yaw=-30, grip='close'; down to 0.15; `robot.arc((hx, hy, 0.15), 130,
   axis='z')`: it ended short or BLOCKED as the door shut, with SUCCESS (4 probes).

## Verified parameters
- Held by its -x wall, the mug turns with the hand: at pitch 0, yaw 60, roll 90 it went in
  (4 probes); a level hand at yaw 90 there was out of reach or BLOCKED (10 eps, probes).
- Turning to pitch 0 at 0.30-0.32 up was refused (arm stretched straight), or the descent after
  it was BLOCKED (3 probes): turn on the way in, as in step 4.

## Common failures and fixes
- Level-hand approaches toward the opening refused or BLOCKED again and again, no arc tried
  (10 eps): use steps 3-6 (4 probes).
- At speed 0.10-0.15 the step-4 move took 165-233 steps and the run ran out (4 probes).
- A door less than square open at the start (its free edge to +x of the hinge; not measured,
  untested): the mug was BLOCKED on the way in (probe); a push about the hinge took 100 steps
  and the next move to the mug was BLOCKED (probe). No working flow is known for it.

## Typical step costs
- Horizon 520. Mug 45+15+10, up 31, across 52-53, down 19, in 89-117, open 10, out 19-52, over P
  51-52, down 18-19, arc 44-46: 407-466 (4 probes).
