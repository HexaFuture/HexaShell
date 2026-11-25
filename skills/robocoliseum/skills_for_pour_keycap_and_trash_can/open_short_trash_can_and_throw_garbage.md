# open_short_trash_can_and_throw_garbage

## Judging
- Three judged stages (the episode end says "N of 3 stages"): the left gripper presses the
  pedal and keeps it down; the right gripper lifts the ball; the ball ends inside the can.
- The lid must stay up until the ball is in: a lid that shut after a later motion cost the
  stage or the drop (agilex L0 t3-t6; g1 L0, V1 t30-t44). Fully open = upright behind the can.
- A ball already held up when the press stage passed was not counted as lifted (arx V1 t6-t12).

## Flow
1. Measure by pointing at pixels (words found the ball 2 of 32 times; L0-L15, V1-V16): can =
   red cylinder; pedal = the dark plate at its foot on your side; ball = white lump.
2. Left hand closed: first over the plate higher than the can's top, then straight down to 4-6
   cm above it (g1 L5, L10, V1-V16). Low lines and swings from the start pose hit the can and
   pushed it 3-24 cm (g2op L0, L10, L15, arx L0; g1 V1, V6, V11 without notes).
3. Press straight down with contact=True, the fingertips (not the grasp point) on the plate's
   middle, commanded 2-3 cm under its top. Passing presses had the fingertips 6-10 cm (most
   7.5-9.5) in front of the can's measured centre (L0-L15, V1-V16). Pitch: robot note.
4. Look at the lid after the press and after each right-arm motion, the lift too. Upright: keep
   the left arm exactly there until the ball is in. Shut: measure the plate again and press once
   at its middle (agilex L10 t9, L15 t7; it shut in agilex L0 t3-t6, g1 L0 t24-t31, V1 t20-t30).
5. Right hand above the ball, straight down, fingertips 4.5-6 cm under the ball's measured top
   (its lower half), close, check "holding", lift 8-12 cm straight up. Such closes held 26 of
   29, higher ones 11 of 41 (agilex, g1, g2op L0-L15, V1-V16). Grasp point, yaw: robot note.
6. Dry-run the drop pose: over the can's middle, grasp point 6-9 cm above its top. Carry in one
   line above can-top height, clear of the left arm. Opens at 5.7-13 cm fell in 16 of 17
   (V1-V16; 6/6 in L5, L10). Then open and move the hand straight up (BLOCKED else, agilex L5 t8).

## Verified parameters
- Ball: soft, about 6 cm wide, top 6.5-7 cm above the table. Two-view measures: 3.3-4.5 cm tall,
  bottom 2-3 cm up, centre 1-2 cm above its real middle; go by its top (L0-L15, V1-V16).
- Can: 13-15 cm wide, top about 15.5 cm above the table (measured 11.5-14 cm tall: its bottom
  reads 1.5-3.7 cm high). Plate top 4-5 cm above the table (L0-L15, V1-V16).

## Common failures and fixes
- Where the approach or a press moved the can 2-24 cm, the lid stayed shut or rose partly
  (agilex L0, L10, L15, arx L0, L10, g1 L0, L15, V1, V6, V11); pressing the old spot did
  nothing (g2op L0, L10, L15). Each further press slid it 2-4 cm (arx L10, g1 V1 t34-t42).
- Carry slips ended 7 of 9 runs one stage short (agilex V1, V11, V16; arx V6, V11, V16, and V6
  without notes); slips over the can fell in (arx L5 t14, V1 t7, V11 t8). After a slip point at
  the ball anew in a new picture: an old Thing or pixel gave the table (arx L5, g1 L10, agilex V1).

## Final checks
- Before opening: lid upright in the head picture, ball between the fingers in the right-wrist
  picture, grasp point over the can's middle; opening over a half-shut lid lost it (g1 L0, V1).
