# BreadSelection

## Judging
- Done when the croissant and the jam both lie on the cutting board near its middle and the hand
  is well away from the croissant, all at once. The other pastry does not matter.
- Still running after the jam's release: the hand is still near the croissant (back off).

## Flow
1. Priors: the board at the counter's front, the pastries on plates beside it, maybe one more
   object at the back of the counter. The jam's cabinet starts OPEN; the jar stood at the front
   of a shelf 0.5-0.6 m above the counter, never on the counter (10 eps): in front, over the board
   (6 eps), or 0.5-0.9 m to the right of the base (3 eps), once on the side wall past a corner.
2. 'croissant', 'cutting board' measured by name (10 eps); 'jam', 'jar of jam' NOT FOUND (2 eps):
   point at the jar (small, orange or red, white lid): `robot.measure(('left', (x, y)))` (9 eps).
3. Doors over the board shut: not the jam's cabinet; never open them (2 eps lost 530 and 1090
   steps, nothing inside). Look for the open one on both sides at shelf height: the hand at shelf
   height, yaw ~-95, `robot.look('wrist')` found it (1 ep); so did backing off and turning (1 ep).
4. Croissant: from above, yaw = its grip_yaw (9 eps). Place it 6.5 cm from the board's centre
   toward one end, the grasp point 3.3-3.5 cm above the board's top; open (9 eps).
5. Then up ~5 cm; pull the hand back to ~0.2 m ahead of the base; rise there; turn level (6 eps).
   Straight lifts of 0.3 m over a board 0.45 m+ ahead, back-and-up moves, and direct moves from
   the board to the jar were refused or BLOCKED (8 eps).
6. Jam: level (pitch 0, yaw 0) 12-13 cm in front of its centre, at its centre height; in to its
   measured centre; close; up 7-9 cm; straight out 0.25 m (6 eps). Level stopped short or BLOCKED
   (a higher shelf, a jar to the side: 9 tries, 2 eps): pitch 15-45 to its centre held (3 eps).
7. To the board's other half, ~7 cm from its centre, away from the croissant; `dry_run=True` on
   the level pose first. Not reachable over a board 0.45 m+ ahead (3 eps): pitch 30-35 down to a
   grasp point 5-6.5 cm above the board's top (2 eps). Board under 0.4 m ahead and the elbow fully
   bent: drive back 0.2 m first, then pitch 45 (1 ep). Open; back off 0.13 m and up (9 eps).

## Verified parameters
- Croissant 10-11.6 x 4-6.5 x 2.5-3.6 cm: closed 4.0-6.5 cm, held (9 eps). A re-measure from above
  that came back far smaller (one view) was wrong: the grasp on it dropped (1 ep).
- Jar 5-5.6 x 2-4.6 x 5 cm: level grips 5.3-6.1 cm (6 eps), pitch 15-45 grips 6.5-6.7 cm
  (3 eps); the gap shrank 8-13 mm on the carry and held (4 eps). Dropped 5-12 cm onto the board:
  counted (3 eps).

## Common failures and fixes
- Shut doors opened, then a pink box on the counter (soap) checked: out of steps with the
  croissant placed (1 ep); jar found with 14 steps left (1 ep). Do step 3 instead.

## Typical step costs
- Horizon 1950. Jar in front: 352-560 in all (6 eps): croissant ~150, jam ~250. Jar to the side:
  969-1936 (3 eps). No jar in the start pictures: search sideways first, before anything else.
