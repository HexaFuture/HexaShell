# clean_the_desktop

## Judging
- Five stages in order: pen into the holder, laptop shut, tissue into the bin, mouse right of the
  laptop, pencil box upright. Each counted during the moves or waits after its release; success
  came during a later call, a wait or home (L0, L0b, L5a, L5b; 4/4): end with `robot.wait`.
- A pen lying against the holder's side counted, and re-grasping it used up the laptop's time:
  each stage has its own (L5; 1/1). Place the pen right the first time, then go on.

## Flow
1. Measure the tissue, the pencil box and the pen holder at head pixels, not by words: the box was
   not found (L0, L5, L0a; 3/3); 'pen holder' also matched the bin (L0, L5, L0a, L5a, L5b; 5/5).
2. Pen, left arm: `pitch=90`, `yaw` = grip_yaw, grasp point 1-2 cm above the table, lift 16-17 cm;
   high over the holder turn to `pitch=0, yaw=0, roll=0` (L0, L5, L0a, L0b, L5a, L5b; 6/6).
3. Aim at the middle of the rim-height spots from `robot.to_world('head', ...)` on rim pixels
   (some read the table through the mesh), not at the holder's measured box: too wide or too narrow
   (L0, L5, L0a, L0b, L5a, L5b; 6/6). Lower until the move stops, open, `by=(-0.09, 0, 0.10)`, wait
   (L0, L0a, L0b, L5a, L5b; 5/5); with no stop the pen ended beside or on the holder (L5; 2/2).
4. Laptop: take the lid's top edge at three head pixels by `robot.to_world`, and again after it
   turns (L0, L0a, L0b, L5a, L5b; 5/5).
   a. Turn it (as found the lid's back was out of reach): right arm, `grip=0.1, pitch=40-65`, in
      beside the lid's right end 6 cm below its edge, behind it to 3-12 cm right of the middle, then
      toward the robot at pitch 55, 65, 75; it turned, left end nearer (L0a, L0b, L5a, L5b; 4/4).
   b. Close with the arm on the nearer end's side (left in L0b, L5a, L5b; right in L0, turned the
      other way by a left-arm side push), `grip=0.1`: `contact=True` down near the lid's middle from
      5-7 cm behind and 5-10 cm above its edge at `pitch=60-70` (stops 2-4 cm below it, "pressing",
      were fine), or in low beside the nearer end and along behind it; then legs down the arc toward
      the robot at pitch 55, 65, 75 to just above the shut lid; open, lift (L0, L0b, L5a, L5b; 4/4).
5. Tissue, left arm: `pitch=90`, grasp point 1.5-2 cm above the table, lift 18 cm; open over
   the bin's middle about 9 cm above its top at `pitch=30`, yaw toward it (L0, L0b, L5a, L5b; 4/4).
6. Mouse, left arm: same grasp, lift 15 cm; at `pitch=60` lower it to 1-1.5 cm above its grasp
   height, 30-33 cm right of the shut laptop's middle, level front to back (L0, L0b, L5a, L5b; 4/4).
7. Box, right arm: `pitch=90`, `yaw` = grip_yaw, grasp point 1 cm below its top, lift 19 cm, turn
   to `pitch=0, yaw=0, roll=0`; open with its lower end just off the table (L0, L0b, L5a, L5b; 4/4).

## Common failures and fixes
- Right arm at the lid as found: `pitch=70` behind it NOT reachable; at other pitches the arm met
  the shelf over the desk (short, "pressing", BLOCKED); waist turns failed too (L0a, L0b, L5b; 3/3).
  7-13 turns each, no gain (L0a t13-t19, L0b t8-t18, L5a t11-t23, L5b t8-t15; 4/4): do 4a.
- Once the turn slid the laptop to the table's front edge; the right arm, reaching across, went down
  behind its left part and pushed level toward the robot below the edge: the laptop fell off the
  table and nothing later counted (L0a t21-t41; 1/1). Close by 4b (untested with it at the edge).
- Over or in front of the edge the lid stayed open or was taken between the fingers (L0, L5; 2/2).
- Pen lying by the holder: `pitch=65, yaw=0, roll=90` by `path='swing'` at its centre held, then
  level via `pitch=30, roll=45`; `pitch=90` NOT reachable, 75-85 empty or Dropped (L5 t10-t37; 1/1).
