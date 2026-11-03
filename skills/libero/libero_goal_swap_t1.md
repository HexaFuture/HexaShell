# libero_goal_swap_t1

## Judging
- `robot.measure('bowl')`: round, 9.4-9.5 cm across, top 0.037, 0.44-0.47 ahead, 0.04-0.07 to -y
  (20 eps).
- `robot.measure('stove')`: 0.56-0.57 ahead, 0.23-0.25 to -y, top 0.019, about 0.18 along x
  (20 eps). A wooden rack stands on its -x side (start picture): the straight carry toward the
  stove at 0.15-0.17 up was BLOCKED or stopped short in every run (10 eps).
- SUCCESS came as the held bowl came down on the stove or slid toward -x on it, on letting go
  there, or as the hand pushed it there (15 eps, 13 probes). A bowl let go 5 cm to +x of the
  stove's middle, by its own measure afterwards, did not count (1 ep).

## Flow
1. Free: `b = robot.measure('bowl')`, `st = robot.measure('stove')` (20 eps).
2. The +x rim: `robot.goto((b.center[0] + 0.038, b.center[1], 0.14), pitch=90, yaw=90,
   grip='open')`, down to 0.024, `robot.gripper('close')`: held 0.9 cm (13 eps) and carried
   (17 probes). Then `robot.goto(by=(0, 0, 0.13))`.
3. Around the rack, never straight: `robot.goto((st.center[0] + 0.095, st.center[1] + 0.11,
   0.15))`, then `robot.goto((st.center[0] + 0.095, st.center[1], 0.12))` (12 eps, 17 probes).
4. Down to 0.053 with speed=0.1: SUCCESS came there (3 eps, 5 probes). If not, on toward -x to
   `(st.center[0] + 0.03, st.center[1], 0.053)` with speed=0.1: SUCCESS came during the slide
   (3 eps, 2 probes), or it stopped short and SUCCESS came on `robot.gripper('open')` at once
   (5 eps, 3 probes).

## Verified parameters
- The +x-rim hold kept the bowl all the way (13 eps, 17 probes). The +y rim at yaw 0, z
  0.022-0.025, held 0.8 cm too (5 eps), but the bowl turned in the fingers on the carry (3 eps).
- Opened where the slide stopped, the hand 0.068-0.078 to +x of `st.center[0]`: SUCCESS (3 eps,
  3 probes); at 0.086-0.096 to +x, no SUCCESS, also after a lowering aimed at + 0.08 (4 probes).

## Common failures and fixes
- Slides that ended BLOCKED with the hand risen to 0.13 and tipped to pitch 113-114, 0.085-0.09
  to +x of `st.center[0]`: opening did not count, every later move and home was BLOCKED (2 eps).
  Both aimed at + 0.03, which worked in the other 6 (6 eps, 4 probes); one at - 0.05 jammed too
  (1 ep). Slides to + 0.06 never did, but gave SUCCESS in 3 of 7 (7 probes). No sure fix.
- Let go just after the lowering with the hand only 0.06 to +x of the stove's middle, holding
  the +y rim: no SUCCESS; a regrasp and slide was BLOCKED, the run ran out (1 ep). Slide first,
  then open (6 eps, 3 probes).
- After the straight carry was BLOCKED the wrist stayed twisted; the next move around was
  BLOCKED twice (1 ep). Go around from the start (12 eps, 17 probes).

## Typical step costs
- Horizon 300. Over the rim 45, down 19, close 12, up 20-21, around 26-28 and 19, down 27-41,
  slide 27-28 (43 when jammed): 156-160 with SUCCESS on the way down (2 eps, 3 probes), 178-211
  with the slide (6 eps, 3 probes). The seven learn1 successes took 185-275 (7 eps).
