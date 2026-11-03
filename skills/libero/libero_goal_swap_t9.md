# libero_goal_swap_t9

## Judging
- "The rack": the bottle lowered onto the big slanted wooden rack, measured 20-22 cm to -y with
  its top at 0.29-0.30, ended the episode with SUCCESS, lying tilted as the hand held it: while
  still held, as it met the rack (5 probes, 1 ep), or on the open just after (2 probes, 3 eps).

## Flow
1. `b = robot.measure('wine bottle')`: about 0.04 x 0.03 x 0.14 m, top 0.145, centre 0.56-0.58
   m ahead, within 1.5 cm of y 0 (9 eps).
2. The rack: `ms = robot.find('agentview', 'wooden rack')`, `rack = robot.measure(ms[0])`, and
   check it is about 0.25 x 0.14 x 0.19 m: centre 0.69-0.71 ahead, 20-22 cm to -y (6 eps).
   rx, ry its centre.
3. Neck from above: `robot.goto((b.center[0], b.center[1], 0.22), pitch=90, yaw=0,
   grip='open')`, down to z 0.115, close: holding at 1.2-1.5 cm (10 eps; 8 probes). Then
   `robot.goto(by=(0, 0, 0.21))`.
4. Turn high, near the bottle: `robot.goto((rx - 0.17, ry + 0.11, 0.38), pitch=60, yaw=90)`
   (4 probes).
5. `robot.goto((rx - 0.08, ry - 0.02, 0.36))`, then `robot.goto(by=(0, 0, -0.06))`: SUCCESS came
   2-9 steps into the lowering, the hand at z 0.33-0.36 (4 probes). No SUCCESS when it stops:
   open, then lift straight up (untested).

## Verified parameters
- Pitch 60, yaw 90, the hand 6.7-8.1 cm -x and 2.1-3.0 cm -y of the rack's measured centre at
  z 0.365, then down: SUCCESS (4 probes).
- Pitch 30, yaw 90, the hand 8.4-10 cm -x and 5-7.5 cm -y of the rack's measured centre at z
  0.316-0.33 (stopped there or lowered until BLOCKED), then open: SUCCESS (1 probe, 1 ep; similar
  poses in 1 probe, 2 eps). Once refused as a stretched arm; pitch 60 then worked (1 probe).

## Common failures and fixes
- `robot.measure('rack')` found nothing (6 eps) or only a small part, 11 cm long with its top at
  0.12 (3 eps, all failures). Find the rack's mask and check its size first.
- Reach-arounds at pitch -20 to -30 or level, from the -y side, were BLOCKED, 60-62 steps lost
  (2 eps).
- Upright (pitch 90) releases with the bottle over the rack: it fell over or off, no SUCCESS
  (2 eps); once an upright bottle counted while still being lowered onto it (1 ep).

## Typical step costs
- Horizon 300. Pitch-60 route: pre-pose 19-21, down 18, close 10, lift 28, turn 45, over the
  rack 28-32, lower 2-9: SUCCESS at 155-161 (4 probes). Pitch-30 route 191 (2 probes). The
  successes took 146-232 (6 eps); the failures ran out at 285-300 (4 eps).
