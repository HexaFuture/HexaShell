# take_the_book
## Judging
- In order: the named hand tilts a book in the row, then takes it out onto the other hand's fist,
  its middle 3-6.5 cm in front of the board's edge; keep both hands still there (probe L0-L15; 17
  times). Keep every book on the board: books fell in 5 of 12 model episodes (V1, V6, V16, L5 r3).
## Flow
1. Measure with to_world on head pixels along one row at the spines' mid-height: fronts, tops, the
   board and its edge (probe L0-L15: within 0.5 cm). Packed spines read as one: aim inside one.
2. Choose a book 20-22 cm tall on the named hand's side within 15 cm of the robot's middle, no
   taller neighbour toward the middle (probe L0-L15; 23 cm: no turn, probe L0; 27 cm out: L10 r3).
3. Named hand 'point', the other 'wrap' at 0, its fist at pitch, yaw, roll 0: swing 30 cm to its
   side; line to 28 cm in front of the spine, 3 cm to the named side (BLOCKED: go on); down to the
   board; line in to 15 cm in front of the spine, 3.5 cm under the board's top (probe L0-L15).
4. Point, pitch 30, yaw, roll 0: swing 21 cm in front of the spine, 17 cm aside, 7 cm over the top;
   line 4 cm behind the spine, 4.5 cm over it; contact down to 1.5 cm under it (probe L0-L15).
5. Tilt: the top's front edge turns about the spine's bottom edge. Contact lines to its path at 20,
   40 and 55 degrees from upright, 2, 3 and 3 cm under it (probe L0-L15: 31/35).
6. Keep both hands still 7 s (wait 150, three times). Book then on the fist 3-6.5 cm in front of the
   edge: move nothing more (probe L5, L15; 5/5; L10 r3; later motion lost it 3/3).
7. Else release: goto(by=(-0.06, 0, 0.10)) once a dry run says by line (probe L0-L15: 12/13; V1-V16,
   L0-L15 r3: 10/11); never by swing near the book: it threw it off (V1 r3). Measure the lying
   book's front end in its own plane; middle = front end + half its height (probe L0-L15).
8. Middle less than 4.5 cm in front of the edge: named hand 'wrap' at 0, over the book 8 cm in front
   of its middle, centred on it. It ends BLOCKED short: send it again, contact (probe L0-L15: 6/6).
   Contact steps of 2.5 cm down; stop at the first stop: more moved the book 3-5 cm out, back or
   aside (V11, L5 r3; probe L0, L10, L15). Dry-run goto('both', by=(-d, 0, 0), contact=True), d =
   4/3 of the shortfall, at most 7 cm, halved when refused; all refused: the named fist alone, at
   most 3.5 cm (probe L0-L15: placed 12/13).
9. Hands still 5 s. Middle still less than 3 cm out: Flow 8 again before lifting; lifts let short
   books slide back 1-4 cm (probe L0; L0 r3; fix untested). Then the named fist straight up 10 cm (a
   lift toward the robot dropped it: L5 r3), 6.5 cm to its side, down to 12 cm in front of the
   spine, 11 cm over the board; hands still (probe L0-L15: 9/9; V1, V11, V16, L0, L15 r3).
## Verified parameters
- Books 20-24 cm tall, 14-19 cm deep; spines 2-6 cm behind the board's edge. A pressing fist stopped
  with its grasp point 3.3-3.6 cm over what it pressed (probe L0-L15). Point to wrap moves the grasp
  point 14-16 cm toward the robot, 7 cm sideways: read the pose.
## Common failures and fixes
- Three stops and a 6.5 cm drag laid the book flat; one stop and a 6 cm drag placed it (V11 r3).
- Landed far back, front end near the edge (V6, V16 r3): aimed from the spine, the fist met only its
  front edge; four press-and-drag rounds, the last on a book already 3 cm out, dropped it (V16 r3).
  Aim from the middle; measure before each drag (untested).
- A standing book the tilt pulled out of the row stops a fist at its top, above a lying book's
  depth; pressed, dragged or tilted, it fell (V6 r3): leave it (untested).
## Final checks
- Book on the fist, its middle 3-6.5 cm out, named fist beside it, both hands still (probe L0-L15).
