# libero_10_lan_t8

## Judging
- Two moka pots, both top 0.124 when pointed at: one in the middle of the agentview picture,
  0.70-0.74 ahead, 0.01-0.06 to +y; one right in the agentview picture, 0.60-0.64 ahead,
  0.21-0.25 to +y (10 eps).
- `robot.measure('stove')`: 0.61-0.63 ahead, 0.18-0.21 to -y, about 0.18 across, top 0.019
  (9 eps). Done when both pots stand on it: SUCCESS came as the second one came down onto it or
  was let go there (6 eps, 2 probes).

## Flow
1. Free: `p1 = robot.measure(('agentview', (x, y)))` on the middle pot, `p2` the same on the
   other, `s = robot.measure('stove')` (10 eps).
2. Middle pot by its lid knob: `robot.goto((p1.center[0] - 0.005, p1.center[1] + 0.015, 0.25),
   pitch=90, yaw=90, grip='open')`, straight down to 0.13, `robot.gripper('close')`, up to 0.25
   (2 probes).
3. Across at 0.25 to over `(s.center[0] + 0.04, s.center[1])`, down to 0.16, open, up to 0.25
   (2 probes).
4. The other pot the same way from `p2`, then over `(s.center[0] - 0.05, s.center[1])`, down to
   0.16 (SUCCESS came there or on the open after it) (2 probes).

## Verified parameters
- Knob grasps held 1.1-1.5 cm (18 eps in the three pot variants). Set 9 cm apart along x on the
  stove, both pots stood (2 probes).

## Common failures and fixes
- Grasps across the body closed at 8.0-8.1 cm; the pot slipped, or wedged and tilted, and the
  run ran out (3 eps). Close on the lid knob instead (2 probes).
- Knob grasps missed again and again until no steps were left (1 ep): re-measure the pot after a
  miss rather than repeating the same move (untested).

## Typical step costs
- Horizon 520. First pot 51-55, 19, 12-13, 21, across 28-33, down 16-17, open 13-14, up 17-18;
  second pot 46-47, 19, 12-13, 21, across 46, down 16, open 0-2: 339-352 (2 probes). The six
  learn1 successes took 364-507 (6 eps).
