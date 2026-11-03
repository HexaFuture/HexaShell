# libero_goal_task_t0

## Judging
- The dark cabinet left in the agentview picture (to -y) has three drawers facing +y. Hooking
  its lowest bar and pulling it about 14 cm toward +y ended the episode with SUCCESS (probes).
- That bar sits low, near the table: 8.7 cm long along x, top 0.043 (5 eps). Open fingers did
  not get around it: every grip closed empty or stopped short, the drawer never moved (10 eps).
  Hook it with closed fingers instead, as below; nothing else had to be moved first (probes).

## Flow
1. Free, before any motion: `h = robot.measure(('agentview', (u, v)))` with (u, v) on the
   middle of the lowest bar; keep hx, hy, hz = h.center (5 eps). Prefer this centre to a
   to_world spot on the bar's top: aims from such spots needed a retry in 2 of 5 (probes).
2. Closed fingers, pointing to -y and tilted down: `robot.goto((hx, hy + 0.04, 0.10),
   pitch=60, yaw=-90, roll=-15, grip='close')`, then `robot.goto((hx, hy + 0.03, 0.09),
   pitch=45, yaw=-90, roll=-15)` (18 probes).
3. Hook: `robot.goto((hx, hy - 0.02, hz), pitch=40, yaw=-90, roll=-15)`. It stops short of
   target after about 19 steps, 1.6 cm above the bar's centre, the fingertips past the bar on
   its -y side (14 probes).
4. Pull: `robot.goto(by=(0, 0.05, -0.01))`, up to 5 times. Each arrives short of target in
   22-24 steps, the hand 4.5-4.7 cm further to +y at the same height; SUCCESS came 3-5 steps
   into the 4th (14 probes).

## Verified parameters
- Roll -15, not 0: at roll 0 the hand read roll -3 or -4 at the bar and, after 2-7 cm, the
  pulls ended BLOCKED with the hand thrown up and to -x off the bar; 1 opening in 29 probes.
  At -15 the roll held and no pull was BLOCKED (31 probes).
- Pitch 60, then 45, then 40: 60 to 40 in one move, or pitch 30-35 at the bar, was BLOCKED;
  at 45 or 60 on the bar the hook slipped off (probes).
- Aim off the measured centre: 1.5 cm along x or 0.8 cm along y still opened it (6 probes);
  1.5 cm along y missed both ways (2 probes).

## Common failures and fixes
- Step 3 ends BLOCKED after about 21 steps, the hand thrown up and to -x: the aim was too far
  to -y. Redo step 2's second move and step 3 with hy 1 cm larger, then pull (3 probes).
- The first pull arrives ok in about 13 steps and the hand sinks 1 cm: the fingertips stayed
  on the +y side of the bar. Redo steps 2-3 with hy 1 cm smaller, then pull (1 probe).
- Open-finger grips, level at roll 90 or -90: BLOCKED, jammed or stalled 1.5-18 cm short, or
  arrived and closed empty on 0.3 cm (10 eps). Pointing down: BLOCKED 6-23 cm short (7 eps).
- Level approaches from +y, the arm low and far out, were refused as stretched (7 eps).

## Typical step costs
- Horizon 300. Route above 147-152: tilt 43-46, pitch-45 move 12, hook 19, pulls 22-24 each,
  SUCCESS 3-5 steps into the 4th (14 probes). With either retry 185-212 (4 probes).
- learn1 used 279-300 steps and never moved the drawer (10 eps).
