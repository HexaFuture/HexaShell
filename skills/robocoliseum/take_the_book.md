# take_the_book
## Judging
- In order (the instruction): the named hand tilts a book in the row, then the named hand takes it
  out: the book held in that hand's own grip, clear of the row. Keep every other book on the board:
  books fell in 5 of 12 model episodes (V1, V6, V16, L5 r3).
## Flow
1. Measure with to_world on head pixels along one row at the spines' mid-height: fronts, tops, the
   board and its edge (probe L0-L15: within 0.5 cm). Packed spines read as one: aim inside one.
   The chosen book's thickness: to_world on its spine's two side edges.
2. Choose a book on the named hand's side within 15 cm of the robot's middle, no taller neighbour
   toward the middle; of those the shorter (a 23 cm book did not turn, probe L0; 27 cm out: L10 r3).
3. Named hand 'point', the other 'wrap' at 0, its fist at pitch, yaw, roll 0: swing 30 cm to its
   side; line to 28 cm in front of the spine, 3 cm to the named side (BLOCKED: go on); down to the
   board; line in to 15 cm in front of the spine, 3.5 cm under the board's top (probe L0-L15). The
   fist only supports: it catches the tilted book, the named hand takes it (step 8).
4. Point, pitch 30, yaw, roll 0: swing 21 cm in front of the spine, 17 cm aside, 7 cm over the top;
   line 4 cm behind the spine, 4.5 cm over it; contact down to 1.5 cm under it (probe L0-L15).
5. Tilt: the top's front edge turns about the spine's bottom edge. Contact lines to its path at 20,
   40 and 55 degrees from upright, 2, 3 and 3 cm under it (probe L0-L15: 31/35).
6. Keep both hands still 7 s (wait 150, three times): the book comes down onto the fist (probe L5,
   L15; 5/5; L10 r3).
7. Release: goto(by=(-0.06, 0, 0.10)) once a dry run says by line (probe L0-L15: 12/13; V1-V16,
   L0-L15 r3: 10/11); never by swing near the book: it threw it off (V1 r3). Measure the lying
   book's front end in its own plane and its thickness there (probe L0-L15).
8. Take it out with the named hand (untested): 'pinch' set clear of the book, its opening the
   book's thickness + 2 cm (the card's gaps); 8 cm over the book's front end, 4 cm in from it,
   pads closing across its thickness; down to it, contact=True; close: "holding". Lift 6 cm
   straight up, then 10 cm toward the robot, past the board's edge; the fist 6 cm down and aside.
9. Hands still 3 s; the book stays in the pinch, clear of the board: done. Slipped back onto the
   fist or the board: open, rise 8 cm, step 8 once more, 2 cm further in (untested).
## Verified parameters
- A pressing fist stopped with its grasp point 3.3-3.6 cm over what it pressed (probe L0-L15).
  Point to wrap moves the grasp point 14-16 cm toward the robot, 7 cm sideways: read the pose.
## Common failures and fixes
- Later motion of the fist with the book on it lost the book (L10 r3; 3/3): take it with the
  named hand first, then move the fist.
- Presses and drags on the lying book laid it flat, moved it 3-5 cm out, back or aside or
  dropped it (V6, V11, V16, L5 r3; probe L0, L10, L15): grip it, do not push it.
- A standing book the tilt pulled out of the row stops a fist at its top, above a lying book's
  depth; pressed, dragged or tilted, it fell (V6 r3): leave it (untested).
## Final checks
- The book in the named hand's pinch ("holding"), clear of the board and the row; the other hand
  off it; no other book fallen (untested).
