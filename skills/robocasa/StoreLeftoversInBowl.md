# StoreLeftoversInBowl

## Judging
- Done when the foods are in the bowl and the bowl stands on a shelf of the fridge's main part
  (not a drawer, not the freezer); it fired as the hand left the bowl (11 eps).

## Flow
1. Find the fridge (tall, doors open, 2-5 m away) in the start pictures, else after step 6 by base
   turns of 90 deg, a look after each; go there by straight legs over floor seen free (prior tool).
2. Foods at pitch 90: drumstick grasp point within 1 cm of its `on_z`, others ~1 cm below the top
   (14 eps held); a drumstick measured 7+ cm wide (others 2-5.5): across its thin bone end (prior
   tool; 2 eps slipped mid-way). Let go 6-10 cm above the rim, centred (5 eps). Re-measure the bowl.
3. Each food: lift 10 cm, bring it in by goto at the grasp angles, no home() or hand turn (1 ep:
   they narrowed or dropped it); print(robot.state()). "... cm when it closed": it slides (17 eps:
   steady 10/10 reached the bowl, sliding 2/9). Then no drive, even after one refused: by arm alone
   if the release passes dry_run (3 eps held), else set it back, open, grasp again, check again.
4. Bowl by its near wall, fingers square to the flared wall (7 eps none fell; prior tool, 48/50):
   its centre c 0.62 m ahead of the base, square (dry_run first); u = unit vector to c, h its
   heading, R = (size[0] + size[1]) / 4; grasp point g = c - u*(R - 0.006) at top - 0.022. goto
   12 cm back and 18 cm above g, pitch=90, yaw=h+90, grip='open'; there pitch=45, yaw=h, roll=-90
   (not +90: joint 7's stop, 2 eps); goto 7 cm back and 7 cm above g, then g; gripper('close').
5. Grip over 3.6 cm: food in the fingers too: open, back out, 40 deg round, still pointing at c.
   Lift 10 cm, robot.wait(6); 1.2 cm under the Grip width: it turned: set back, same (prior tool).
6. Keep these angles to the release. Carry: grasp point 0.30 m ahead of the base, 15 cm higher
   (7 eps none fell); back off 0.4 m before the first turn or sideways leg (6 eps).
7. Line up square, base 1.15 m out from the shelf's front edge, the bowl midway in the clear gap at
   its height, between the doors' free edges and their bins, not the walls inside (2 eps). Shelf:
   lowest above the drawers, edge 0.95-1.3 m (prior tool); next edge 18+ cm over g (3 eps).
8. Grasp point z = edge + bowl height + 0.023. Drive in by 0.25 m legs, robot.state() after each leg
   or arm move: BLOCKED, the hand 2.5 cm off or the gap 0.6 cm changed: back the base 0.15-0.3 m,
   arm still (7/7 kept it, 5 eps; arm moves back dropped 3 in 8), re-centre. The gap back within
   0.3 cm of before: go on (2 eps); 1+ cm off: it hangs turned, set it down, grasp again (untested).
9. Base 0.33-0.47 m out (5 eps): arm in to the bowl centre ~12 cm behind the edge; lower to edge +
   bowl height - 0.022; open, robot.wait(8), up 4.5 cm, robot.drive(forward=-0.35) (prior tool).

## Verified parameters
- Lift narrowing: square pinch <=0.21 cm (7 eps); rim pinch at pitch 90 0.15-2.1, 8 drops (10 eps).

## Common failures and fixes
- Slipped on the lift after a grip under 2 cm, a descent `short of target` 1.5+ cm high or across a
  7+ cm drumstick (7 of 8 lifts, 4 eps): measure, grasp again. Never open over the floor (1 ep).
- The doors start open about to their stops: widening one dropped the bowl (1 ep).

## Typical step costs
- Foods 346-497 (13 eps); a food drop cost 150-1700 more (4 eps); successes ended 1057-2119 (7 eps).
