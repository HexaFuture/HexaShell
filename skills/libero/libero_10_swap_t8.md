# libero_10_swap_t8

## Judging
- The stove is the one 0.77-0.94 ahead: to_world spots on its plate lay from 0.06 to -y to 0.16
  to +y, top about 0.02 (10 eps). Both pots set there ended the episode with SUCCESS as the
  second was let go (7 eps, 2 probes). `robot.measure('stove')` returned a box at x -0.85 every
  time (10 eps): use the spots.
- One pot stands at -y, 0.46-0.48 ahead, 0.19-0.22 to -y; the other at +y, 0.59-0.63 ahead,
  0.21-0.25 to +y (10 eps).

## Flow
1. Free: measure both pots; plate middle bx, by = middle of to_world spots on its -x and +x
   edges and its -y and +y sides (10 eps).
2. Each pot by its lid knob: `robot.goto((p.center[0] - 0.005, p.center[1] + 0.015, 0.25),
   pitch=90, yaw=0, grip='open')`, down to 0.13, `robot.gripper('close')`, up to 0.25 (2 probes).
3. Over the plate pitch 90 was refused 0.80-0.87 ahead (5 eps), pitch 75 at 0.83-0.85 (2 eps);
   pitch 60 with yaw 0 reached it (5 eps, 2 probes). The -y pot: to `(bx - 0.08, by - 0.05,
   0.26)` with pitch=60, yaw=0, then `(bx - 0.03, by - 0.05, 0.165)`, open; then to bx - 0.10 at
   0.27 (2 probes).
4. The +y pot: to `(bx - 0.09, by + 0.055, 0.26)`, then `(bx - 0.05, by + 0.055, 0.165)`, open:
   SUCCESS (2 probes).

## Verified parameters
- The successful runs let go with the hand 0.80-0.85 ahead at 0.13-0.165, pitch 60-75, yaw 0:
  first pot within 2.5 cm of y 0, second 0.08-0.11 to +y (6 eps, 2 probes).
- Knob grasps held 1.1-1.5 cm (18 eps in the three pot variants).

## Common failures and fixes
- Repeated grasps across the body, a pot pushed over or dropped beside the plate, a release
  lowering BLOCKED with 22 steps left (3 eps): knob grasps and the two drop points (2 probes).
- A second pot let go only 8 cm to +y of the first was BLOCKED and fell (1 ep); the probes' two
  drop points were 10.5 cm apart across (2 probes).

## Typical step costs
- Horizon 520. The -y pot 24-25, 19, 11-13, 20-21, across 40-42, down 21-25, open 14, away 24-27;
  the +y pot 35-36, 19, 11-12, 20, across 31, down 19, open 6: 315-328 (2 probes). The seven
  learn1 successes took 407-491 (7 eps).
