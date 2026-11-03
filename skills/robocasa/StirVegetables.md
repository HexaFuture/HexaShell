# StirVegetables

## Judging
- Done after a few moments of real stirring; they add up. Stirring counts while the spatula,
  held in the closed hand, touches both vegetables and both move a little, both lying in the pot
  near its middle, the pot left on its lit burner (do not shove it).

## Flow
1. Priors: the pot on the lit burner; vegetables and spatula on the counter beside the stove,
   either side. 'spatula' was not found in 3 eps: name its look ('metal spatula',
   'wooden utensil') or point at it (3 eps).
2. Vegetables first, over the pot's centre, the second next to the first (untested).
3. Spatula: pitch 90, yaw = grip_yaw or grip_yaw+180, whichever is nearer 0; grasp point 1.2-1.8
   cm above the counter (5 eps) and ~2-3 cm behind its centre along that heading, so the longer
   part points forward once the yaw is 0 (4 eps). Lift 10-15 cm and check the Grip width.
4. Turn the yaw to 0 over the counter, then carry by drive or in arm legs of 20 cm or less
   (3 eps). Stand about 0.5-0.7 m from the pot's centre (5 eps).
5. Stir from the robot's side: turn to pitch 135, yaw 0, the hand 13-25 cm nearer the robot than
   the pot's centre and 10-14 cm above the rim; the end ahead of the hand goes down and forward
   (5 eps). Then lower 2-4 cm at a time to rim height (up to 4 cm under it in a deep pot), with a
   3-6 cm stroke forward and back or sideways after each lower, the hand 3-13 cm nearer than the
   centre. Success fired 2-10 moves after the turn (5 eps).

## Verified parameters
- Pitch 45 from the far side puts the end BEHIND the hand down and back: success in a 9 cm deep
  pot (1 ep); in a 14 cm pot the 5 cm short end went down and never reached them (1 ep).
- Vegetable heights read about half: tomato, lemon, onion measured 2.4-2.6 cm tall with their tops
  5-6 cm above the counter (3 eps). Take the top minus the counter as the true height.

## Common failures and fixes
- Vegetables slipped out on the lift 13 times (8 eps, ~100 steps each); grasps within 1.4 cm of
  the measured top slipped 6 times (4 eps). Grasp at about half the true height (untested).
- Thin flat spatula (measured 1.4 cm tall or less): top-down closes slipped 15 times (2 eps,
  ~1200 steps). After two slips change the grasp: pressed down at pitch 130 it held (1 ep); one
  lying in or leaning on the pot was grasped there from above and held (2 eps).
- A held spatula slipped out in one fast 0.48 m arm move, twice, then lay near the floor; ~900
  steps there gave no hold (1 ep). Keep arm legs short.
- Pitch 135 refused (stretched) or BLOCKED: drive 10 cm closer or back, then retry (untested).

## Typical step costs
- Horizon 2400. Successes took 748-1961 steps (7 eps, median 903). Vegetables in by 370-770;
  spatula held by 500-770 (6 eps), 1656 once; hold to success 60-260 at pitch 125-140 (5 eps).
- The big costs: retried spatula closes (~1200, 2 eps), slipped vegetables (~100 each), BLOCKED
  poses and drives (~500, 3 eps). Not held by ~1000: use the fixes above, not the same close.
