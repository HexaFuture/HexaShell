# RecycleBottlesByType

## Judging
- Done when each of the 3 middle bottles stands next to the samples of its own type, all 7 on
  the dining counter (one let go lying counted, 1 ep), the hand well away from all 7, at once.
  Don't knock the samples. Plastic = bottled water; glass = beer, wine, liquor.
- Sort each middle bottle by its look; which end group is which differs (10 eps).

## Flow
1. Priors: the robot stands behind the middle stool, facing the dining counter; the bottles
   stand along the counter's near edge, a sample may be only a few cm from it; a sample pair
   in front of each end stool, one to each side (10 eps).
2. Types by look: water bottles clear, gray or blue with blue caps; glass green, brown, black or
   red, long necks. Measure by colour words (10 eps); type words scored every bottle alike.
3. Pick: `robot.measure('gray bottle')` (its colour); pitch 90, yaw = grip_yaw (0 if any), grasp
   point 2.5-3.5 cm below its measured top, under the cap (glass: the neck); close; lift 0.2.
4. Drive along the counter in relative legs (`robot.drive(left=-0.50)`, 10 eps). Pitch 90 reaches
   ~0.60 m 0.3 m above the counter, ~0.68 m 0.1 m above; else `robot.drive(forward=0.08)` (5 eps).
5. **Release beside or behind the samples, never nearer the robot**: grasp point at least as far
   ahead as the nearest sample's centre, 10-20 cm from every sample, 13-15 cm above the counter
   for a 15-17 cm bottle; open; lift 0.2 (13 releases, 7 eps, none fell). Releases 6-22 cm
   nearer the robot than the samples: 2 of 12 fell off the near edge (2 eps).
6. Last release: pull the hand toward the base, away from all 7 (a 0.25 m lift NotExecuted, 1 ep).

## Verified parameters
- Grasps 2.5-3.5 cm below the measured top: 30 of 31 holds kept the bottle to the release,
  2.3-5.4 cm wide, creeping under 7 mm through drives of 0.3-0.8 m (10 eps). A descent stopped
  "short of target" (fingertips on a neighbour): closing there held (2 holds, 1 ep).
- A bottle lying on its side: grasped across at yaw = grip_yaw ~1 cm below its top (5.4 cm),
  released lying 4 cm above the counter; it counted (1 ep).

## Common failures and fixes
- Released 7-13 cm nearer the robot than the samples, the bottle fell off the near edge (2 eps):
  once onto a stool, re-picked there for ~175 steps; once to the floor, out of reach: lost.
- A 1.4 cm grip on a tall black bottle was its cap: it fell in the lift; 2 cm lower held (1 ep).
  Under 2 cm, open and grasp 2 cm lower before lifting.
- A glass bottle went to the plastic group first: ~220 steps (1 ep). Releasing on a sample's
  spot was BLOCKED or toppled samples (1 ep). Measure in the wrist picture: left/right pointing
  measures merged bottles with the counter (1 ep).

## Typical step costs
- Horizon 2850. 612-972 (9 eps); clean 612-685 (6 eps). Per bottle: pick ~60, carry 35-65 per
  0.5-0.8 m leg, release 60-75, return 40-60. Extras: driving to both ends to look before the
  first pick ~200 (2 eps; 8 eps picked within 45 steps); a bottle off the edge ~175 (1 ep).
