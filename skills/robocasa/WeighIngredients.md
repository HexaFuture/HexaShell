# WeighIngredients
## Judging
- Done when the FRONT item stands upright on the scale near its middle, EVERY door is fully shut,
  and the hand is well away, at once.
## Flow
1. Priors: the item at the FRONT of a shelf ~0.5 m above the counter, 0.5-0.7 m ahead (10 eps); a
   closer drive stops within 8 cm (3 eps). The scale at the counter front ~0.3 m ahead (9 eps) or
   behind the robot (1 ep). One door (3 eps) or two (7 eps). NOT FOUND: 'cereal', 'yogurt' (3 eps),
   'honey bottle' 1 of 2; found: 'cereal box', 'blue can', 'bottle' (4 eps), pointing (5 eps).
2. Low item (can, cup, bar), (x, y, z) its centre: level at (x-0.14, y, top+0.09), yaw 0, open;
   pitch 45 5 cm in front, 3 cm over its top; then at (x, y, z); close; back 10, up 6 cm (4 eps).
   Bottle: level, yaw 0, from 11-13 cm out straight in at mid height; close (2 eps). Its measure
   once came out 22-26 cm wide (merged with things near it): aim by `robot.to_world` (1 ep).
3. Tall thin box (2.5-3.4 cm thick, 13-15 cm tall, turned 30-55 degrees: 4 eps): pitch 45 is NOT
   reachable (3 eps). Level or pitch 15, yaw = its grip_yaw, from 10 cm out straight in to its
   centre; close (2 eps). Closes under 3 cm slipped out within 15 steps (1 ep, 3 of 3).
4. In hand: pull it out level till clear of the shelf front, `robot.drive(forward=-0.2)` (1 ep;
   prior tool; probe, 10); lowered or turned over the shelf, it was lost (2 eps). Over
   `robot.to_world` of the scale's top centre. Low item: pitch 45, open with the grasp point
   2.5-6.5 cm above its top (4 eps). Box or bottle: upright, hand level (box: pitch 30 or 90 too),
   lower until touching, open (5 eps); a tall box level or at pitch 10 stopped BLOCKED high, at
   pitch 30, same yaw, it stood on the scale (probe, 5 and 2). NOT reachable: drive back
   0.13-0.22 m (4 eps); level at yaw 0 can stay NOT reachable: yaw +-45 to 90 was, or the base
   0.18 m sideways (2 eps).
5. Doors last, one at a time. Hinge = the open door's end at the cabinet front (`robot.to_world`
   along its edge). Outside push (3 eps, 4 doors): base in line with the hinge, 0.35-0.65 m behind
   it; hand closed, level, yaw toward the middle (-90 left hinge, +90 right), 10-15 cm outside the
   door in front of its free edge; along it to 0.3-0.4 m from the hinge; in to touch its outer face;
   `robot.arc(hinge, 25, axis='z')`, then 30, 30 (minus: right hinge). Or grip the free edge, hand
   level pointing along the door, fingers either side; arcs 25-35 (2 eps, 4 doors).
6. Arcs stop a few degrees short of shut: finish each door with the closed hand level, yaw 0,
   0.2-0.35 m from the hinge, straight in to the cabinet front until BLOCKED (7 eps).
## Verified parameters
- Holds: cans and cups 5.7-6.5 cm, a bar 4.7 cm (4 eps); boxes 3.3-4.1 cm (3 eps); a box from above
  across its thin side 3.1-3.3 cm (3 eps); a bottle 6.9-8.0 (2 eps); door edges 2.8-6.0 (2 eps).
## Common failures and fixes
- A box closed under 3 cm slipped and the misses pushed it out of reach: episode lost (1 ep). Tilted
  to pitch 30-47 or let go while BLOCKED, the item fell (4 eps). Lying item: from above across it,
  lift, level the hand (it stands up), lower until touching, open (4 eps; 3 tries in 1: 1700 steps).
- Door edge grips: arcs 1-6 cm short, re-grips 150-200 steps, 5 of 20 slipped off (7 eps). A door
  boxed in by a fixture stayed open, square grips slipping (2 eps): try step 5's grip (untested).
## Typical step costs
- Horizon 3000; successes 1168-2642 (8 eps). Item and place 204-405 (5 eps), 577-2012 after a fall
  (4 eps). Doors: outside push 300-640 each (3 eps), edge grips 215-1475 (6 eps). Plan ~1000.
