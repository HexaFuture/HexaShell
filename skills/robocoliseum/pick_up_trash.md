# pick_up_trash

## Judging
- In order: near a piece; it lifted; near both; the other lifted; near the red bin; the bin
  opened (by its foot pedal; not in the task text); both pieces inside (L0-L15).
- No message says when a stage passes: a lift replied "Dropped" counted (L0 t6); "near both"
  passed at the second lift (L5, L10, L15).

## Flow
1. head(pitch=0); a wall ahead: turn 180, head yaw ±44 (L0-L15; 4/4). Trash: a crushed red drink
   can and a white paper ball, ~6 cm, on the floor; the bin: red, ~13 cm wide, ~24 cm tall.
2. Drive so the piece is 0.94-0.99 m ahead of the base's centre, 0.20-0.24 m to the side of the
   hand that takes it; body(lean=70, height=0.58) (L0-L15; 7/7). Nearer ones: back up (L5 t37).
3. pitch 90, grip 1, grasp point ~12 cm above the piece; measure again; down to 0.2-0.9 cm under
   its measured top; close; lift 18 cm (L0-L15; 7/7 at the first try).
4. Left hand first, then the right, as the card says (order untested). Crouched, measure the other
   piece, drive or turn so it lies alike for the right hand, take it (L5, L10, L15 t6-t10; 3/3).
5. Stand to lean 20, find the red bin, drive till its lid's middle is 0.90-0.95 m straight ahead,
   crouch, measure the lid, drive the rest. At 1.05 m 'near' never counted (L5, L10; L15 t14).
6. The pedal faced the robot, its middle ~10 cm nearer than the lid's, its top ~6 cm up: measure
   it from a wrist picture (L5 t16, L10 t18, L15 t15; 3/3).
7. Press at once: closed right hand, piece in it, pitch 90, at the pedal's middle, 0-2 cm under
   its top, contact=True; it stops on the pedal, the lid stands up; stay (L5, L10, L15; 3/3).
8. Left piece: raise it, then pitch 45-60, yaw -60 to -90, grasp point over the lid's middle,
   8-10 cm above the rim; open; move the hand out (L5 t26-t27, L10 t33-t34; 2/2).
9. Hold the pedal with the empty left hand the same way; right piece over the middle at pitch 45
   yaw 90 (L10 t43), else pitch 0 yaw 65 (L5 t52, where pitch 30-60 were blocked); open.

## Verified parameters
- Crouched pitch-90 closes held 0.92-0.99 m ahead, 0.20-0.24 m aside, reading 6-8 cm (L0-L15; 9/9)

## Common failures and fixes
- "Dropped" or "closed fully (empty)" with the piece wedged at the fingertips (L0 t6-t7, L10
  t21): look at the wrist picture before redoing anything.
- Pitch-90 moves to over the bin's middle were refused or blocked 6-14 cm short, 10 times (L5,
  L10, L15): tilt the hand as in Flow 8-9.
- The piece in the pressing hand slipped out (L5 t18, L10 t19; 2/3): when the left piece is in,
  take it from the floor by the pedal with the right hand (L5 t37-t38, L10 t35-t37; 2/2).
- Crouched beside a chair the floor read 0.6 m low and the reach was refused; standing, backing
  0.2 m and crouching again fixed it but lost the reach and the held can (L0 t10-t12; 1/1).
- 'trash can' found a black basket; the red bin was taken for trash (earlier runs L0, L5, L10).

## Final checks
- Before driving to the bin: both hands 'holding', each piece in its wrist picture (L5-L15).
- Before each release: the lid up in the head picture, the hand over the opening (L5, L10).
