# insert_building_block
## Judging
- Stages in order: the named arm picks up the purple star (counted at the first look after the
  lift, 16/16); the star drops through the star-shaped hole into the box (counted in the wait
  after the release). A star on the hole's mouth does not count; each retry cost 3-6 motions.
## Flow
1. The star is the purple piece on the named arm's side; the other one, farther out, is taller
   (L0-L15, V1-V16; 16/16). Measure its height from a wrist camera 10 cm above (the head read
   0.6-0.9 cm more): about 2-2.6 cm, it lies on its side (V1-V16); 3.5, face up (L5, L15).
2. Pick up: `pitch=90`, open, 10 cm above it. Lying star: in that wrist picture its ridges run
   between its two star-shaped ends; set the yaw so the pads close across the ridges, on its
   points (V1, V6; untested in L). Lower until the grasp point is 0.5-1.7 cm above its top;
   close (3.6-4.2 cm); lift 14-16 cm (L0-L15, V1-V16; 16/16).
3. Lying star: one line to a level hand, `pitch=0, roll=0`, left `yaw=-90`, right `yaw=90`, near
   the hole (V1 t6, V6 t6, V16 t6, L0, L10). In a zoomed head picture a face must be up: pads on
   its points did that (V1, V6; 2/2), pads on its ends left a side up (V6 ns, V11, V16, V16 ns;
   4/4; then lower it onto the box beside the hole, open, grasp again as in 2: V16 t22-t27;
   untested in L).
4. Face up: in one zoomed head picture match its tips to the hole's, turning the yaw by the
   difference (V16 t27: -10°). Lower until the grasp point is 2.5-3 cm above the box top; open;
   back off 5 cm and up 5 cm; `robot.wait(30)`. In 3/8 (V16 t27, V6 ns t30, L0); 5 stuck.
5. On the mouth (top about 4 cm over the box top) or standing face up: grasp from above, grasp
   point 0.7-1.2 cm above its top; lift 10-12 cm; keep pointing down (V6 t13-t20, L5, L10, L15):
   a. grasp point 10 cm above the box top over the hole; in one picture of the holding hand's
      wrist camera take the hole's five tips with `to_world('<arm>_wrist', px, z=<box top>)`;
   b. take the star's five tips there too; turn the yaw by the angle difference (72° steps);
   c. lower to a grasp point about 4 cm above the box top; check the tips; open; lift 10 cm;
      `robot.wait(30)`. In 4/5 (V6 t20, L5, L10, L15); an angle from one tip stuck (V11 t28).
## Verified parameters
- Pointing down over the hole, yaws reached: left -45 to -150, right 98 to 180; refused: left -24
  to 60, right 216 (L0-L15, V1, V11).
## Common failures and fixes
- `roll=90` to bring a face up: the lower finger stayed under the star; it stuck or fell off the
  box (V16 t15-t19, V11 t14, V1 ns t16; 0/5; untested in L). Use 3.
- Setting it on the table to turn it cost 10+ motions and the episode (V1 t18-t29, V11 t38;
  untested in L). Doubting which piece is the star lost V11 ns; words found the other (L10).
- `measure` at a star pixel returned the table, about 90 x 55 cm (V1 t29, V6 t2, V11 t18; untested
  in L): measure again at its middle or from a wrist camera.
- Tilted releases stayed on the mouth (L5 twice, L0 t41, L10 t28, V16 ns roll -43; 0/5).
## Final checks
- Before opening pointing down: the star's tips over the hole's within about 2 mm and 2° in the
  wrist picture (L10, L15, V6 t20); 5-12 mm or 13° off, it stuck (L5, L15).
- After the wait, zoom the head picture on the hole: standing on the mouth, top about 4 cm over
  the box top: step 5 (L0 t18, L10 t14, V6 t10); lying on the box: step 3 (V16 t22).
