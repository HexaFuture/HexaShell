# agilex_crsemb_write_uppercase_letter_with_pen
- Before the pick, park the left hand at pitch 30 yaw -45 about 8 cm from the sheet's centre
  toward the robot, 7-9 cm to its left and 11 cm above it: its wrist camera sees the whole sheet
  at 0.4-0.65 mm per pixel (L0, V1, L5, V6 r6 8/8; probe L0, L5 r6 9/9; probe L0, L5 r9 4/4).
- For a B that view cut off its ends; parked instead with path='swing' about 11 cm toward the
  robot, 15 cm to the left and 17 cm above the centre, same angles, it held the B at 0.63-0.81 mm
  per pixel (probe V1 r9 4/4). Its B points lay 1.1-1.4 mm off the band's middle on average, up
  to 2.9 mm, the same way each time; the dot, seen in the same view, takes that out (probe V1 r9).
- Pick at pitch 20 yaw 40, the grasp point 3.4 cm below the pen's top and 1.5 cm back along the
  fingers from its axis (probe L0, L5 r5, r6, r9). First path='swing' to 7 cm back along the fingers
  and 10 cm above the grasp point; then path='line' straight down at that same place, still 7 cm
  back, to 3 cm above the grasp height; then in to the grasp point with path='swing'; close; lift
  10 cm straight up (probe L0, L5 r6, 8 of 9 held; probe L0, V1, L5 r9 8/8).
- A line from there to 3 cm over the grasp point itself was not executed (L0, V1, L5, V6 t2 r6,
  8/8), nor was a straight line in from 7 cm back (probe L0, L5 r6, 9/9; probe L0, V1, L5 r9).
- Holds read 1.4-1.7 cm (L0, V1, L5, V6 r6, r8 16/16; probe L0, V1, L5 r9 8/8). A swing in that
  ended 2 cm from the axis and 3.8 cm below the top closed at 1.2 cm, and the pen was gone after
  the lift (probe L5 r6): below about 1.4 cm open, go back up and grasp again (untested).
- Write in the grasp pose, pitch 20 yaw 40, the tip 3 mm above the surface: it is 2.5-3.2 cm
  ahead of the grasp point along the fingers and 11 cm below it (probe L0, L5 r6, r9; r8 7/8).
- The tip sat 0.7-6 mm from that offset, different after each grasp although the grasp point
  repeated within 0.4 mm (probe L0, V1, L5 r6, r9); once 21 mm (V6 r8): check it every time.
- Tip check, a dot (4 moves): hover 12 mm over the first point, straight down to the writing
  height, lift straight up 6 cm, look; new offset = the dot minus the .at of the move down; go
  back down at the first point. Dots were 8-41 pixels (probe L0, V1, L5 r6, r9).
- Take the darkest blob near where the tip was meant, whatever its size. A 41-pixel dot 6 mm away
  passed over for its size, and a faint 2-pixel blob taken for being nearest beside an 11-pixel
  dark dot 3 mm away, each left the ink 1.5-3.5 mm off and the B failed (probe V1 r9). In all 8
  logged reads the darkest blob was the dot (probe L0, V1, L5 r9); the largest once was not (L0 r8).
- Each move stopped 1.0-1.9 mm off its target, forward and to the robot's right (probe L0, L5 r5,
  r6, r9): aim each later target back by the mean miss so far (probe L0, L5 r6, r9).
- A and T: speed=0.05 in moves of up to 30 mm, 64-90 steps each: the A passed 2/2 in 12-13
  motions after the pick and the T 2/2 in 10 (probe L0, L5 r9; r7 too). The T's bar and stem as
  one move each inked whole but lay 1-1.4 mm off, one aim correction a stroke (probe L5 r7).
- B: stem and bars in moves of 18-20 mm, 5 chords of 13-15 mm per bowl, 20 moves, speed=0.07:
  35-50 steps a move; it passed in 22 motions after the pick with the dot read right and failed
  on the faint blob above (probe V1 r9). At 0.05, 43-64 steps a move, 1 of 2 passed (probe V1 r9).
- Without speed: 9.5-11 mm apart, 13-17 steps each; 12-30 mm moves fell short (probe L0, L5 r6).
- The B with 9 mm chords on the bars too took 37-38 motions and failed 2/2 after dots read 1-6.5
  mm off (V1 r8); lifts, waits and second passes made runs of 60-68 moves (V1 r6, 2 runs).
