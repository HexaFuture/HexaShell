# libero_10_task_t6

## Judging
- "to the right of the plate" is +y of the plate (the right of the agentview picture, the robot's
  left), in line with the plate's centre in x: every success left the pudding there (20 eps).
- The pudding counted with its centre about 10 cm to +y of the plate's measured centre; at +13 to
  +16 cm it counted in some runs and not in others (the first placement missed in 5 of 20).
- SUCCESS came during the pudding's descent, still held, the grasp point at height 0.09-0.10,
  with the red mug upright and centred on the plate.

## Flow
1. `p = robot.measure('plate')`, `m = robot.measure('red mug')` (found in 20 of 20 eps). By
   words the pudding was never found: point at it in the agentview picture (20 eps).
2. Red mug by its +y wall: `robot.goto((m.center[0], m.center[1] + 0.05, 0.25), pitch=90, yaw=0,
   grip='open')`, straight down to 0.133, `robot.gripper('close')`, up to 0.25 (10 eps).
3. Across at 0.25 until the hand is 3.5-4 cm to +y of `p.center`, down to 0.145, open, up
   to 0.25: the mug stood centred on the plate (10 eps).
4. Pudding from above at its measured centre, yaw 0: down to 0.035, close, up to 0.21 (20 eps).
5. Across at 0.22 to `(p.center[0], p.center[1] + 0.10)`, then straight down toward 0.045:
   SUCCESS came at height 0.09-0.10, still held (4 eps in sibling variants, 2 probes).

## Verified parameters
- Red mug measured 8 by 9.5 cm, top 0.151-0.153, 0.29-0.33 ahead near y 0 (20 eps). Its +y wall
  grip at 4.8-5.4 cm from the centre held at a width of 1.0-1.1 cm (10 eps). The +x wall, the
  hand 2.5-3.8 cm to +x of the centre at height 0.13-0.137, held at 1.0-1.7 cm (6 eps).
- Plate measured 11.6 cm across, top 0.034 (20 eps). Pudding about 7.6 by 4.5 cm, top 0.045,
  0.44-0.49 ahead and 0.07-0.12 to +y; it closed at 4.6 cm and stayed at the hand (20 eps).

## Common failures and fixes
- Coming down at yaw 90 over the red mug's middle was blocked; closing 4.8 cm to +x of its centre
  caught nothing (width 0.6 cm) and knocked the mug over; it lay on the plate by about step 370,
  the pudding then missed and the run ran out at 520 (1 ep). Grip a wall as in Flow (16 eps).
- Pudding set 13-21 cm to +y of the plate's centre: not counted (5 eps). Moved closer, in line in
  x, it counted (2 eps); runs with that second placement ended at 428-486 (4 eps). Set at +12
  cm first, it counted (1 ep).

## Typical step costs
- Horizon 520. Red mug: over, down and close 58-64; let go on the plate by step 139-155
  (7 eps). Pudding: over 35-45, down 25-30, close 10, up 22-27, across 42-45,
  down 13-27 to SUCCESS: 285-309 in all (5 eps).
