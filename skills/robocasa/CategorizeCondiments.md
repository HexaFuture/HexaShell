# CategorizeCondiments

## Judging
- Done when each counter item (a shaker; a condiment, ketchup or syrup bottle) is on a shelf of
  the target cabinet, wholly inside, close beside its counterpart there (the bottle by the cabinet's
  bottle, the shaker by its shaker), and the hand is well away from both items, at once.
- Upright does not matter (items let go lying counted, 8 eps); nor do the shelf or the doors.
- The target cabinet stood open or had no doors (9 eps); opening a closed one cost ~790 steps
  and it was empty (1 ep): leave closed doors.

## Flow
1. Priors: the counterparts stood at shelf fronts in the open wall cabinet right above the items
   (9 eps). Measure all four from the start pose (6 eps); words that worked: 'black shaker',
   'white shaker', 'syrup bottle', 'red bottle', 'salad dressing bottle' (8 eps). No open cabinet
   in view: look around for one before anything else (untested).
2. Bottle within ~10 cm of the shaker: take the bottle first (untested). Taking the shaker first,
   the hand pressed on the bottle at yaw 0, then tipped it over at yaw 90 (2 eps).
3. From above: pitch 90, yaw = grip_yaw re-measured from ~12 cm above, grip=0.8; grasp point
   2-2.6 cm below the shaker's top, 2.5-3.5 cm below a standing bottle's top; close; up 12 cm
   (8 eps). Stopped short ~0.3 m ahead: drive back 0.15-0.2 m first (2 eps).
4. Under the cabinet's front edge pitch 90 was BLOCKED 8-16 cm high (2 eps): pitch 60 (1 ep) or
   25 (1 ep), yaw 0, in from 10-15 cm nearer the robot, grasp point ~4 cm below its top.
5. Level via a point ~0.3 m ahead at the shelf's height + 10-14 cm (pitch 0, yaw 0; 6 eps), then
   straight in (diagonal moves at the cabinet were BLOCKED or stopped short, 4 eps) to ~8 cm
   beside the counterpart, the whole item behind the shelf's front. Lower the grasp point to
   2.5-4 cm above the shelf (it often stops short there, touching: fine); open; straight back
   0.33 m, rising 8 cm. Then the other item; success came on that back-off (8 eps).

## Verified parameters
- Holds: shaker 3.5-4.2 cm, standing bottle 2.3-4.5 cm (8 eps); gaps that shrank 0.4-0.7 cm on
  the level move still held (4 eps). Let go 7-12 cm beside the counterpart: counted (8 eps).
- Tipped bottle: pitch 90, yaw across it, at its narrower part: 6.6 cm held; carried at the
  shelf's height + ~10 cm (higher was BLOCKED twice); counted (1 ep).

## Common failures and fixes
- Tipped bottle (3 eps): 7.2-7.5 cm holds on its wide part slipped on the level move (2 eps) or
  stuck in the opened fingers and came back out (1 ep). Pushes in the cabinet never counted (2
  eps, 650-780 steps) and knocked placed items over (1 ep): regrasp and set down again (untested).
- Merged measures, e.g. a cabinet bottle 0.20-0.31 m wide, the counter pair 0.25 m (3 eps):
  measure again closer, or point at each item, before choosing the spot.

## Typical step costs
- Horizon 1650. Clean successes 379-453 (6 eps), ~200 per item: grasp ~70, level 40-53, in and
  down 30-60, open and back ~50. Lost: a closed cabinet ~790, a tipped bottle ~600 (1 ep each).
