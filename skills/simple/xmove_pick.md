# xmove_pick

## Judging
- One stage: the box picked up off the table in the right hand. It counted while the hand rose
  12-14 cm straight up after the grasp (4/4 runs); nothing else was needed.

## Flow
1. `robot.measure('cracker box')` from the start: it stands upright on a low table about 0.6 m
   ahead, its long side across (grip_yaw near 90 or -90; 4/4 runs).
2. `robot.walk` straight ahead so that the pelvis ends 0.38-0.41 m behind the box's measured
   centre, heading kept (0.19-0.22 m, about 106 steps; 4/4 runs). Measure the box again.
3. `robot.body(0.74, pitch=25)` to `pitch=30`: standing upright or at 20 degrees the palm point
   was NOT reachable, 2-8 cm short (4 runs).
4. End grip on the box's right end: `robot.fingers('right', 'wide')`, then the right palm
   `facing='inward'` (or `normal=(0, 1, 0), finger=(1, 0, 0)`) 4-5 cm right of the grip point
   and 2-4 cm higher, then in to the grip point (`seconds` 0.6-0.7): the palm point 9-10 cm right
   of the measured centre, level with it front to back, 4-6 cm below the top (4/4 runs).
5. `robot.fingers('right', True)`: holding with thumb, index and middle (4/4 runs).
6. `robot.hand('right', by=(0, 0, 0.12))` to `0.14` with `seconds` 0.7-0.8: it counted on the way
   (4/4 runs).

## Verified parameters
- Check the grip point with `dry_run=True` after the lean; it was reachable within 0-2 cm at 25-30
  degrees (4 runs).
- A `via` point above and behind the box's right end on the way in worked as well (1 run).

## Common failures and fixes
- None seen: 4/4 runs succeeded without these notes, the slowest at step 464.

## Final checks
- None needed: the episode ends by itself on success (4/4 runs).

## Typical step costs
- Walk about 106, lean 32-71, wide open 50, two hand moves 80-170, close 50, lift 26-36; done
  at steps 354-464 of the official 800 (4 runs).
