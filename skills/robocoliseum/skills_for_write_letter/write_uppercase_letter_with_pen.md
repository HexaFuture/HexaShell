# write_uppercase_letter_with_pen
## Judging
- Two stages: lift the pen out of its holder with the right hand, then write the gray letter
  along the middle of its band, end to end on every stroke; no dots, drags or lines beside it.
## Flow
1. Park the camera hand (this robot's note) and leave it there; pick and lift the pen (this
   robot's note) (L0, V1, L5, V6 r8; probe L0, V1, L5 r9).
2. Measure the pen: 1.3-2.1 cm across with its top within 1-2 mm is the pen alone (agilex V1,
   V6, g1 r8); 9-13 x 3 x 15 cm, top 2-5 mm low, is the pen with its holder: on g1 measure again
   by words or the Thing itself, on g2op pick anyway (g1, g2op L0, V1, L5, V6 r8 8/8 each).
3. Measure the paper: its centre height is the surface (probe L0, V1, L5 r9; 0.2-1.1 mm above it).
4. Letter from the parked wrist view: click each stroke's ends, follow the middle of the gray
   band, fit straight pieces meeting at the corners; to_world on the plane at the surface height
   (probe L0, L5 r6, r9). B: fit the stem and the three bars as lines, their crossings are the
   corners; take 5-6 points on the middle of each bowl's band (probe V1 r9).
5. Plan one pen-down path, no lifts between strokes (probe L0, L5 r7, r9). A: from the left foot
   up to the apex, down the right leg to the bar, along the bar, one move back along it, then the
   rest of the right leg. T: along the bar, one move back to its middle, down the stem. B, nothing
   drawn twice: from the stem's middle up the stem, along the top bar, round the upper bowl to
   where the bowls meet, along the middle bar to the stem, down the stem, along the bottom bar,
   round the lower bowl to where the bowls meet (probe V1 r9; g1 V1 r8). Robot notes may add lifts.
6. Points: every end and corner; straight pieces as this robot's note says. Each bowl is a
   half-ellipse, its ends on the two bars' lines, its depth fitted to the band points; trace it
   as 5 chords at equal angles, the 4 inner points set 0.7 mm outward (probe V1 r9).
7. Tip: this robot's offset and tip mark, made at the first point from 12 mm above it, then back
   down at the first point (this robot's note) and trace from there (probe L0, V1, L5 r9).
8. Trace: one goto per point, path='line', contact=True, same angles and height, each target aimed
   back by the mean miss so far; speed= (m/s, the grasp point's pace, one move) on moves along the
   paper as this robot's note says, not down, up or in the air. Last point: robot.wait(35), keep
   holding still, then finish; never lower the tip or redraw (probe L0, V1, L5 r9; arx L0 r6).
## Verified parameters
- Tip 1.5-3 mm above the surface inked end to end without pressing (probe L0, V1, L5 r6, r9).
- The B in 15-20 moves along the paper, 16-26 motions after the pick: it passed 7 of 9 on agilex,
  arx and g1, 1 of 2 on g2op (probe V1 r9).
- The A in 9 moves along the paper and the T in 5, 10-15 motions after the pick: on agilex, arx
  and g1 the A passed 6/6 and the T 7 of 8; on g2op 1 of 7 and 2 of 7 (probe L0, L5 r9).
## Common failures and fixes
- The B with the middle bar drawn twice, some with 8-10 mm chords on the bars too, took 32-44
  motions after the pick and failed 5 of 9, the ink 1-1.4 mm off after the tip mark (V1, V6 r8).
- A grasp from the pen-with-holder measure wrote nothing for 30 moves (g1 L0 r6, two runs).
- A misread tip mark put every stroke 1.4-3.5 mm off (probe V1, L5 r9): see this robot's note.
- A dry run said reachable, then the move was refused (g1 L5 t9, t13 r5): check each .at.
## Final checks
- Before the trace: "holding" with no warning in the grip note, the tip mark seen, every point
  dry-run (probe L0, L5 r6; g2op V6 r8 slipped after "fingertips came down on top of it").
