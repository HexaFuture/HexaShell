# sort_number

## Judging
- One stage: the blocks in one row, ordered by the digit on each, not by colour (one colour carried
  different digits: L0, L5, L10). Smallest on the robot's left (+y, the left of the head picture),
  largest on its right, counted (L0, L5, L20, L25; 4/4); the other direction is untested.
- A straight row across the table, all about the same distance from the robot, with clear gaps,
  counted (4/4). A digit left turned a quarter round on the table still counted (L20; 1/1).
- `finish` with a misread digit, so the row out of order, scored 0 (L10, L15; 2/2).
- Success came by itself during a later move (L5, L20, L25; 3/3). In L0 it had not come through two
  waits, and `finish` then counted (L0; 1/1).

## Flow
1. Measure each block by its head pixel, `robot.measure(('head', (u, v)))` (6/6). Word searches
   ('digit block', 'number block', a digit, a colour) found nothing (L10, L15), and
   `robot.measure('digit')` gave another block or NOT FOUND (L0).
2. Read every digit large. Head crops at `zoom=4` read all right (L20, L25; 2/2); the head picture
   or `zoom=3` misread one (L0, L10, L15; L5 read right). A wrist view 10 cm above the blocks
   (`pitch=90, yaw=0, roll=0`, `robot.look('<arm>_wrist')`) shows the digits large and sharp (L5;
   L0, L10, L15 frames): check there any digit that could be another, 1/7 or 5/9 (untested).
3. Plan the row: keep blocks already in order where they are and move as few as you can (L25: one
   move); neighbours about 10-15 cm apart (4/4); place only on a free spot.
4. Grasp at `pitch=90` with the `grip_yaw` a re-measure from 10 cm above gives (L5, L25), `roll=0`
   (it kept the asked yaw on every move, L20), grasp point 1-2 cm above the measure's `on z`;
   close; wait (all held grasps, 6/6).
5. "Closed on something" 0.8-1.8 cm wide held 16/16 (all six); 2.0-2.4 cm slipped 4/4 (L5): open,
   re-measure from 10 cm above for a new `grip_yaw`, and grasp a little lower (L5; 2/2 held).
6. Lift 10 cm, carry, and set down with the grasp point at the grasp height or up to 1 cm higher
   (6/6); open, lift 10 cm, and home that arm or move it well aside. Then wait.

## Common failures and fixes
- Misread digits: a 9 read as 5 (L10), a 1 as 7 (L15). Both models were sure, never looked closer,
  and finished with the row out of order; L15 judged the start already in order and only
  straightened it. Fix: step 2 and the final check.
- A block taken to a spot still occupied landed beside it, and a later move set one block on
  another (L10). Clear the spot first or use a free one.
- A block set down 5 cm from one still to move merged their outlines in the next measure; grips
  then went wide and slipped (L5). Keep the gaps.
- An empty close on a thin block bent the torso forward; the next lift was BLOCKED and
  `robot.home()` did not bring the arm back. The other arm finished, `roll=0` on every move (L20).
- A wrist picture taken half a turn round (yaw near 180) is upside down: 6 and 9 swap in it.

## Final checks
- Before `finish`, read the final row again from close up (zoom 4 or a wrist view; untested as a
  check): the digits must increase from the robot's left to its right.
