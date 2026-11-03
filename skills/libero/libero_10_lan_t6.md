# libero_10_lan_t6

## Judging
- "to right" is to +y of the plate (the right of the agentview picture, the robot's left), in
  line with the plate's centre in x: every success left the pudding there (20 eps).
- The pudding counted with its centre about 10 cm to +y of the plate's measured centre (+10 to
  +12 cm counted wherever the plate measured). At +14 to +17 cm it counted in some runs and not
  in others: the first placement missed in 12 of 20.
- SUCCESS came during the pudding's descent, still held, the grasp point at height 0.09-0.10,
  with the mug upright and centred on the plate.

## Flow
1. `p = robot.measure('plate')`, `m = robot.measure('white mug')` (found in 19 of 20 eps). By
   words the pudding was never found: point at it in the agentview picture (20 eps).
2. Mug by its -y wall: `robot.goto((m.center[0], m.center[1] - 0.04, 0.25), pitch=90, yaw=0,
   grip='open')`, straight down to 0.11, `robot.gripper('close')`, up to 0.25 (18 eps).
3. Across at 0.25 until the hand is 3.5-4 cm to -y of `p.center`, down to 0.12, open, up to
   0.25: the mug stood centred on the plate (20 eps).
4. Pudding from above at its measured centre, yaw 0: down to 0.035, close, up to 0.21 (20 eps).
5. Across at 0.22 to `(p.center[0], p.center[1] + 0.10)`, then straight down toward 0.045:
   SUCCESS came at height 0.09-0.10, still held (1 ep here, 3 in sibling variants, 2 probes).

## Verified parameters
- Plate measured 11.6 cm across, top 0.034; white mug top 0.128 (20 eps). The wall grip held at
  a width of 0.6-1.7 cm (20 eps).
- Pudding measured about 7.6 by 4.5 cm, top 0.045, 0.44-0.49 ahead and 0.08-0.12 to +y; gripped
  at its centre it closed at 4.6 cm and stayed at the hand (20 eps).

## Common failures and fixes
- Pudding set 14-17 cm to +y of the plate's centre: not counted (12 eps). Moved to +10 to +13.5
  cm, in line in x, it counted (7 eps); runs with that second placement ended at 409-474 (10 eps).
- Lowering the pudding at yaw 0 with the hand 14-15 cm to +y of the mug: below a grasp height of
  about 0.065 the hand's housing pressed the mug's rim, the move stopped short and the mug tilted
  or slid off centre (2 eps here, 4 in sibling variants). At +10 cm SUCCESS came higher, at
  0.09-0.10 (4 eps, 2 probes).
- The second try then went sideways at height 0.09, or down at pitch 65 (blocked): the mug was
  pushed about 3 cm off centre and the steps ran out at 520 (2 eps). Rise to 0.22 before moving
  across and come down straight, as the direct successes did (18 eps of the four variants).
- Let go from 0.164 over the +10 cm spot with the mug centred, the pudding counted (1 ep).

## Typical step costs
- Horizon 520. Mug: over 23-30, down 19-21, close 13-14, up 20-23, across 37-47, down 17-20,
  open 14, up 20-23: set by step 132-157 (18 eps of the four variants).
- Pudding: over 35-45, down 25-30, close 10, up 22-27, across 42-45, down 13-27 to SUCCESS:
  289-300 in all (4 eps).
