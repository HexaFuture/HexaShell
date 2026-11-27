# clean_the_blackboard

## Judging
- Stages: eraser held in the right hand; robot in front of the whiteboard; all writing wiped off.
- Credit grows with the share wiped, but the last stage needs every mark gone (L0: one speck left,
  0.80). Extra motions cost credit: L5, L15 wiped 77%, 87% in ~160 motions and scored 0.44, 0.40.

## Flow
1. measure('eraser') from the start pose: its top, size and grip_yaw on the desk (L0-L15).
2. Right hand at the offered pitch 75 over its middle, fingers across the narrow side, grasp point
   1 cm above the measured top; close (~5 cm, holding); lift 16 cm (L0-L15; 4/4).
3. home, drive(turn=180) (the instruction), body(lean=0, height=1.30). to_world on the board face
   by the writing: drive till it is 0.68-0.71 m ahead and the writing's right end ~0.40 m right,
   measuring after each (untested). Hand pitch -15, yaw 0, roll 0: the pad faces the board.
4. Map the writing: grid, to_world on its row ends. A stance reaches ~0.40 m right to ~0.15 m left
   at 1.3-1.55 m up, ~0.10 m left near 1.05 m (L0-L15); dry-run a row's left end (L0 t10, L10).
5. Probe once at a written row: contact=True to 1 cm past where the pad meets the face; where the
   hand stops, 3.6-4 cm in front of the face, is the wiping depth (L0-L15 t8-t9; 4/4).
6. A row is 3 motions, each contact=True: to the row's start 2-3 cm off the board at its height,
   in to the wiping depth, one sideways stroke past the row's far end. Rows ~8 cm apart, only
   where writing is (L10 t11-t22: all wiped in 30 motions; L0 t12-t26: 93% in 45).
7. Once the stance's reach is clean: hand back to ~0.5 m ahead, drive left by 0.40 at most (to put
   the writing left within the reach), look with grid, to_world it, wipe it (L0 t19, L5, L10).
8. Leftovers: to_world each mark and run one stroke through it (L10 t23-t24 finished).

## Verified parameters
- The eraser held long side up on the board; measure the writing, its rows and ends (L0-L15).

## Common failures and fixes
- Pressing ~2 cm deeper than the probed depth (L5 t23; L15 t22, t26) or stroking upward with the
  pad on the board (L5 t34, t56; L15 t23) stopped strokes 9-34 cm short, hand twisted 9-31°.
  Move between rows with the pad off the board; downward pressed strokes ran (L15 t51-t61).
- After such stalls the eraser sat tilted in the fingers (right_wrist showed its dark pad edge);
  full strokes at pitch -15 wiped ~1%, at pitch -45 they wiped again (L15 t35-t43; learn1 L0 t44).
- Writing beyond the reach: L15 spent ~55 motions for 17% before driving left (L15 t9-t33); L5
  tried ~20 pitch, yaw and roll poses (~20 refused) for +9% in ~120 motions, never drove left
  again and ran out of time (L5 t20-t84). Wipe what a stance reaches once, then drive left.
- 'Dropped' and 'closed fully' after the probe were wrong: right_wrist showed it held (L10 t9-t11).
- Re-wiping whole areas for the last specks took ~55 motions for +0.7% and left one (L0 t30-t46).
- Swings and roll changes dropped the eraser to the floor, which ended it (learn1 L5 t34, L10 t42).
- measure('eraser') from the wrist just above it found nothing (L0 t3; learn1 L0, L10, L15).

## Final checks
- After a stall: right_wrist shows only the eraser's grey back, no dark pad edge (L15 t22-t31).
- Before finish(): the head picture with grid shows no mark anywhere (L0 finished with one left).
