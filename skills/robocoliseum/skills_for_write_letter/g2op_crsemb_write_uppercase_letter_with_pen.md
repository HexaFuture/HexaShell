# g2op_crsemb_write_uppercase_letter_with_pen
- Turn the waist to yaw 20 first (probe L0, V1, L5 r5, r6, r9, r10, r12 69/69). Then park the left
  hand at pitch 30 yaw -90 level with the sheet's centre, 14 cm to its left and 7 cm above it: its
  wrist camera sees the whole sheet at 0.88-0.97 mm per pixel (probe L0, V1, L5 r5, r6, r9, r10).
- Pick from above, pitch 90 yaw 0: from about 15 cm above the grasp point straight down to 4.5 cm
  below the pen's top, close (a thin hold, 0.6-0.8 cm), lift 10 cm straight up (L0, V1, L5, V6 r8
  8/8; probe L0, V1, L5 r9 15 of 16; probe L0, L5 r10 4/4; probe L0, V1, L5 r12 16/16). Once the
  close came up empty ("closed fully"): open, go back up and grasp again (probe L0 r9; untested).
- A side grasp (hand level) held no wider and was blocked 2 of 5 times (probe L0, L5 r10).
- Every pen measure had its holder, 10-12 x 3 x 15 cm, top about 2 mm low (L0, V1, L5, V6 r6, r8).
- Write pointing straight down, pitch 90 yaw 0, the tip 3 mm above the surface; the tip is about
  10 cm below the grasp point (probe L0, V1, L5 r6, r7, r9, r10). After each grasp it sat elsewhere:
  up to 25 mm from the usual offset (L0, V1, L5, V6 r8), 2-11 mm (probe L0, V1, L5 r9, r10).
- So read the held tip first, at the hover: to_world on pixels around where the tip should be in
  the left_wrist and the right_wrist views, the 3 points farthest down the pen; forward from the
  left_wrist view, sideways from the right_wrist view, the height from the grasp. The mark then fell
  0.6-4 mm from where it was meant (probe L0, V1, L5 r9 12/12; probe L0, L5 r10 4/4).
- Tip mark, an 8 mm line along the first stroke (6 moves): straight down to the writing height,
  8 mm along, lift 1 cm, then up to 4 cm and 4 cm to the robot's right, look; new offset = the
  line's middle minus the mean .at of its two moves; back over the first point, then straight
  down (probe L0, V1, L5 r7, r9, r10, r12). An L mark instead did worse (probe L0, V1, L5 r9).
- The pen still moves in its thin hold while it writes: over a letter the needed offset changed
  by 0.5-5.2 mm, up to 4.6 mm at one look (probe L0, L5 r10). One correction is not enough.
- So look while the pen stays down; looks need no motion. Look after the first move of each
  straight piece and at each corner, not after a move back over the one before it, nor after a
  bowl's chords (probe V1 r12). Read the ink of the moves since the last look across the planned
  line every mm; leave out the pen's own image, the mark's ink and points within 3.5 mm of a
  crossing piece; drop readings over 1.5 mm from their median. Fit one offset to all looks so far,
  each across its own line against the offset it was drawn with, each older look weighing half the
  next; move every later target by the whole change (probe L0, L5 r10; probe L0, V1, L5 r12).
- With the looks the A passed 4 of 4, 15-16 motions after the pick (probe L0 r10, r12), the B 1 of
  2, 19-24 motions; in the failed B the first move up the stem lay 2.9 mm across (probe V1 r12).
- The T: the bar, one move back along it to its middle, then a stop: the stem's first 14 mm, lift
  1 cm, up and aside as after the mark, look, fit; over the stem's top, straight down, one move
  over that piece, the rest in equal moves, the next look after the first: 5 of 6, 19-22 motions
  (probe L5 r12). Without the stop the stem's first move lay up to 4.8 mm right; the T passed 2 of
  6 (lifting off the bar's end or past its middle no better; probe L5 r10, r12), 1 of 3 (L5 r11).
- Failed model runs went wrong after a sharp turn, then worse with repairs: a refit after every
  move, readings against the offset after the mark, offsets from tip clicks or by hand, changes cut
  or clipped (L0 run 3 t12-t16, V1 runs 2-3, V6 runs 1, 3 r11). Keep the one fit; no clicks.
- Each move stopped 1.1-1.9 mm off its target, to the robot's right (probe L0, L5 r5, r6, r9, r10):
  aim each later target back by the mean miss so far (probe L0, V1, L5 r6, r9, r10, r12).
- Along the paper speed=0.05 in moves of up to 30 mm, 17-24 steps each; the B in 15 moves, each
  straight piece one move (probe L0, V1, L5 r7, r9, r10, r12); 15-20 mm moves fell short (L0 r6).
- Never aim lower mid-letter: 3 mm lower drew off the band (V1 run 2 t28-t29 r6; L0 run 3 t9 r11).
