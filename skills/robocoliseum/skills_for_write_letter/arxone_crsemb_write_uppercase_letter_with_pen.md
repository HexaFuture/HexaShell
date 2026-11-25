# arxone_crsemb_write_uppercase_letter_with_pen
- Before the pick, park the left hand at pitch 20 yaw -60 about 13 cm from the sheet's centre
  toward the robot, 12 cm to its left and 13 cm above it: its wrist camera sees the whole sheet at
  0.6-0.75 mm per pixel (L0, V1, L5, V6 r6 8/8; probe L0, L5 r6 12/12; probe L0, V1, L5 r9).
- Pick (L0, V1, L5, V6 r6, r8 16/16; probe L0, V1, L5 r9 7/7): hand level, pitch 0 yaw 0, 9 cm
  behind the grasp point, straight in to 3 cm below the pen's top, close, lift 10 cm straight up.
- Grasp point: the pen's measured centre put 5 mm further in, away from the robot; it closed at
  1.0-1.1 cm (L0, V1, L5, V6 r8 8/8; probe L0, V1, L5 r9). The centre reads 4.5-6 mm toward the
  robot, by words or from a head pixel (L0, V1, L5, V6 r6). Holds of 1.2-1.3 cm left the first
  pass of the A and the B partial (L0, V1, V6 r6, 6 runs).
- Write at pitch 20 yaw 0, the tip 2.5 mm above the surface: it is about 3.9 cm toward the robot
  and 10.8 cm below the grasp point (probe L0, L5 r5, r6, r9). Moves on the paper ended 1.0-2.1 mm
  below the command (probe L0, V1, L5 r6, r9); keep the 2.5 mm anyway.
- Tip check, a 4 mm tick (5 moves): hover 12 mm over the first point, straight down to the
  writing height, 4 mm along the first stroke, lift straight up 6 cm, look; new offset = the
  tick's middle minus the mean .at of its two moves; go back down at the first point. It read
  0.2-1.1 mm in probes (probe L0, V1, L5 r6, r9) and moved the offset 1.75-2.5 mm to the robot's
  left in every model run (L0, V1, L5, V6 r8 8/8). A tick seen 0.3 mm long came before a bare A
  (probe L0 r6).
- Without a check the A passed 2/2 in 23 moves and the T 2/2 in 18 when the grasp sat where the
  probes' grasp sat (probe L0, L5 r6); a grasp 1-1.5 mm from it moved the ink 1.0-1.4 mm to one
  side, left gaps, and the A failed (probe L0 r6). Keep the check.
- Each move stopped 0.7-0.8 mm to the robot's left of its target (probe L0, L5 r5, r6): aim each
  later target back by the mean miss so far (probe L0, L5 r6, r9).
- A and T: speed=0.05 in moves of up to 30 mm, 74-90 steps each: the A passed 2/2 in 13 motions
  after the pick and the T 2/2 in 11 (probe L0, L5 r9; r7 too). The T's bar and stem as one move
  each left no ink 5-25 mm into both and failed (probe L5 r7).
- B: stem and bars in moves of 18-20 mm, 5 chords of 13-15 mm per bowl, 20 moves, speed=0.07:
  36-49 steps a move; it passed in 24 motions after the pick (probe V1 r9 1/1). At 0.05, 44-63
  steps a move, it passed too but took far longer (probe V1 r9 2/2).
- B points from this view sat 0.1-0.3 mm off the band's middle (probe V1 r9 3/3); the bowls' ink
  lay near the band in every model run (V1, V6 r8).
- Without speed: 9-11 mm apart, 14-16 steps each; 12-30 mm moves and an 85 mm leg fell short
  (probe L0, L5 r6).
- After a partial first pass, lowering the tip 1-3 mm drew beside the band (L0 t14-t16, V6
  t12-t14 r6); new tip readings from one picture moved the offset 4-10 cm (L0 t21, V1 t20 r6).
