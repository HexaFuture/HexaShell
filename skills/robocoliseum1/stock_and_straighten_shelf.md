# stock_and_straighten_shelf

## Judging
- Stages in order: the right hand at the juice in the basket, that juice held up, it standing on
  the shelf, the hand at the overturned juice, that juice upright (L0, L5; 2/2).
- Grasps counted at the close or the lift. Placements counted at or shortly after the release,
  hand backed off (L0, L5); once only during the next turn's move (L5).
- Counted (2/2): the juice from the basket standing cap up on the shelf top near its front, in
  front of the row of the same juice; the overturned juice standing upright on the shelf, set down
  5-7 cm nearer the robot than where it was grasped.

## Flow
1. Measure both juices at head pixels: by words the basket one matched a shelf bottle and the
   overturned one was NOT FOUND (L5).
2. Juice in the basket, right arm: `pitch=90`, `yaw` = grip_yaw, grasp point at its centre height;
   close, `robot.wait(12)`, lift 16-23 cm (2/2). If a move prints "half a turn round", use
   grip_yaw + 180 from then on: the grip_yaw itself was then NotExecuted (L5).
3. Turn level, `pitch=0, yaw=0, roll=0`, about 20 cm above the shelf top in front of it: the juice
   hangs cap up (2/2). Carrying at `pitch=90` was NOT reachable (L0, L5). Look; if the cap is
   down, turn `roll` by 180 (L5).
4. Shelf top by `robot.to_world` on shelf pixels. Lower over the shelf's front until the move
   stops short (L5) or the grasp point is 8-8.5 cm above the shelf top (L0). Open; back off
   `by=(-0.13, 0, 0.025)` to `(-0.13, 0, 0.05)`; wait (2/2).
5. Overturned juice: `robot.to_world` on its top and bottom ends gives the lean (34-39°, top to the
   robot's left; 2/2). Start level 11-14 cm in front of its centre at its centre height with
   `roll` = minus the lean (-25 to -35); line in to 1-2 cm past the measured centre at `roll` -32
   to -35; close; wait (2/2). For a top leaning to the right, a positive roll (untested).
6. Back 8 cm and up 10-12 cm keeping the roll, then `roll=0`: the juice hangs upright (2/2).
7. Lower 5-7 cm nearer the robot than the grasp until the move stops short; open; back off 13-14 cm
   level and up 2.5-4 cm; wait (2/2).

## Common failures and fixes
- Turns lost (L5, 10 of 25): words for both juices; yaw = grip_yaw arrived half a turn round and
  the next grip_yaw was NotExecuted; a `pitch=90` carry NotExecuted, then four dry runs; `roll=180`
  hung the juice cap down, `roll=0` fixed it. L0 took the plain route in 15 turns.

## Final checks
- After the last release, wait and look: the end message came at the start of a wait (2/2).
