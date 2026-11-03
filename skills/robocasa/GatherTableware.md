# GatherTableware
## Judging
- Done when the mugs stand together and the bowl is clearly apart from them (well beyond their
  own spread), the hand away from the mugs. Where they stand and the doors do not matter, nor
  does letting go of the bowl. It ended during the bowl's carry (6 eps) or as the hand backed off
  what it had set (8 eps).
## Flow
1. Priors: you face an open wall cabinet, shelves about 1.0-1.5 m up, the bowl and two mugs on
   its shelves. The third mug is in another open wall cabinet, 0.4-5.8 m
   away; in 12 of 22 eps on the same wall, 0.4-3.1 m to the side. Measure the near three first.
2. The first turn in place was BLOCKED (7 eps); backing off 0.2-0.65 m first let it turn (21 eps).
   Look along the wall to both sides first, then turn 90 degrees at a time with robot.look('all')
   and robot.find(cam, 'mug'): it was near the top of 'left' or 'right', 1.0-4.5 m away (15 eps).
   From below or over ~30 degrees off square, an open cabinet's doors hid its shelves (3 eps:
   found at step 1247-1488, or never). Look into each square-on (within ~25 degrees) from 1.3-2.2 m
   (3 eps); robot.to_world on a door pixel locates a cabinet seen from the side (prior tool).
3. On measured centres (`math.dist`): the mugs as close together as you can, the bowl as far from
   them as you can. Third mug ~3 m or less from the others: carry the bowl straight away from it,
   hand close to the base (it passed 0.03-1.54 m into the carry, 5 eps). Farther: fetch it, set it
   by the near mugs off the bowl's side (passed at once, 4 eps); if still running, move the bowl
   0.2-0.5 m or a near mug to them (4 eps).
4. Mug: 8-9 cm tall; partly hidden it reads 5-6.5 cm (10 eps). Level, pointing at it, grasp point
   at `.on_z` + 4 cm, within 3 cm of its measured centre (13 eps); if empty, measure again (1 ep).
5. Bowl: base square to it (heading = approach yaw), at most ~0.17 m to the side; from 3-9 cm
   before its front rim go in at roll 90, grasp point on the rim (half its measured width before
   its centre) ~1 cm below its top, pitch 45: it held 3 of 3 (3 eps; BLOCKED reaches, 3 eps). Level
   held 3 of 7, pitch 25-35 1 of 5: the rest slipped on lifts or in drives (7 eps). Back 0.2-0.3 m.
6. Mug in hand: up 6-10 cm, hand back 0.25-0.4 m (8 eps); back the base straight out till the mug
   is ~0.9 m from where it stood before any drive across or turn; come in square (prior tool).
   Out 0.24-0.69 m, 3 of 8 drives lost it on a door or cabinet (8 eps); 0.89-3 m, 4 held (4 eps).
## Verified parameters
- Handle closes of 0.9-1.7 cm held 12 of 17 times (13 eps), the grasp point 1.3-5 cm under the
  mug's measured top; two less than 1 cm under it slipped as the hand rose (2 eps).
- Far shelf, base stopped by the counter: level reaches BLOCKED, empty or out of reach (12 of 20
  eps); what held there: level on the handle (8 eps), pitch 25 (3 eps). Two misses: switch (2 eps).
## Common failures and fixes
- Mug closes of 2.8-8 cm from the side (across the body, or "nearly fully open") slipped or fell
  6 of 10 times (5 eps); from above, ~8 cm closes held on a counter (1 ep). Take the handle.
- A mug on the floor was out of reach (4 eps): leave it there and group the rest around it.
- Bowl tries lost 400-900 steps (5 eps): pitch 90 was out of reach at shelf height (6 eps), deep
  level reaches came back BLOCKED or empty (6 eps). Use step 5; drive 0.3 m closer first (1 ep).
## Typical step costs
- Limit 2250; successes used 637-2129 (14 eps): search 100-620 (20 eps), fetch and set the far
  mug 470-990 (8 eps), bowl route 270-330 (3 eps), each near mug carried over ~950 (1 ep); a
  failed far-shelf series 160-380 (4 eps). Plan ~700 (bowl), ~1100 (fetch), ~1900 (near mugs).
