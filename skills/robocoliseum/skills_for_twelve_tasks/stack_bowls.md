# stack_bowls
## Judging
- Two stages: a bowl lifted near a hand, then all three nested in one stack ("1 of 2 stages", L0
  r1, L5 r2). No close ever held a bowl (0/29, both rounds); the first stage passed only when a
  descending hand knocked a bowl up (L0, L5, L10 r1; L5, L10, L15 r2).
- A bowl reached the floor in seven of eight episodes. The one bowl really lifted was squeezed
  between two level flat palms (L5 t31 r2): use that.
## Flow
1. Measure each bowl by colour or a head pixel; use a centre seen by two or three cameras
   (L0-L15, both rounds); one-view measures read 5-11 cm in some size (L5 t1, t13, L15 t6 r2).
2. Lift each hand straight up 15 cm before anything else (L0-L15 r2; 4/4, nothing touched).
3. Base: a bowl left where it stands, the one with the shortest carries from the other two, not
   at an edge (untested). L5 r2 spent its last ten replies moving a bowl to the middle (t24-t33).
4. Hands flat, as they start; turn both level, pitch 0, yaw 0, roll 0 (palms facing each other,
   fingers forward), about 20 cm above the table and away from the bowls; then straight down
   beside the bowl (L5 t27-t28 r2; 2/2). Dry-run each pose.
5. Grasp points at the bowl's centre front to back, at half its height above the table, each 3
   cm outside its measured side (L5 t31 r2; 1/1). Palms 8 cm nearer the robot than the centre
   pressed nothing and slid the bowl 5 cm (L5 t29 r2).
6. contact=True each palm to 1-2 cm inside its measured side: one stops and presses, the other may
   push the bowl to it; then goto('both', by=(0, 0, 0.13)) lifted the bowl 13 cm; look: the
   bowl between the palms (L5 t31-t32 r2; 1/1).
7. Carry with goto('both', by=...): up until the bottom is 3 cm above the base's rim, then over
   the base in moves of about 5 cm, looking after each (untested). One 10 cm sideways move
   stopped 3 cm short for both hands and turned the bowl on its side (L5 t33 r2).
8. Over the base's measured centre lower until the bottom is 1 cm above its rim; move each palm
   3 cm straight out, one then the other; lift both; look (untested).
9. The third bowl the same way onto the stack (untested).
## Verified parameters
- Three-view measures of a bowl agreed within 1 cm (L0-L15 r2); a tilted bowl read 2-6 cm
  taller than upright (L5 t22-t27 r2).
- Costs: level pose by line 24-81 steps, swing 61-105, home 62-92, contact move 12-16, the lift
  of both 24-26 (L5, L15 r2). Both hands start flat: the squeeze needs no change of shape.
## Common failures and fixes
- Pinches on a wall closed empty, the palm toward the bowl (0/11 r1) or away from it (L0 t6, t9,
  L5 t9, t12, t16, t18, t23, L10 t4, L15 t4, t7, t8, t11, t13 r2; 0/13): do not pinch a bowl.
- Each pinch moved the bowl: a descent threw one 28 cm (L5 t5 r2); one try moved it 10-13 cm
  (L0 t5-t7, L10 t4-t5, L15 t4-t5 r2); misses walked bowls off edges (L0 t9, L10 t6 r2; r1).
- Level, the fingers reach 16 cm past the grasp point: a level hand sent down and in along a
  line swept a bowl off the far edge (L15 t17 r2). Come down beside the bowl, then move sideways.
- A fingertip push sent a bowl 42 cm across the table (L5 t24-t26 r2).
- "holding" with the pads held apart came while a bowl went off the table (L0 t9, L10 t6 r2).
## Final checks
- Before letting go: the held bowl right above the base in the head picture; at the end three
  bowls in one column on the table, the hands away (untested).
