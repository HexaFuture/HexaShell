# libero_spatial_object_t9

## Judging
- In our episodes the bowl that counted sat on top of the wooden cabinet: x 0.65-0.72, y -0.25
  to -0.31, top 0.265 (19 eps). The plate lies at x 0.70-0.735, y +0.19 to +0.215 (20 eps).
  Success came with it set down on the plate (18 eps); no run moved the bowl on the stove.

## Flow
1. Measure bowl and plate (free). The first measure from the start pose had its x 0.5-2.9 cm
   too small (16 eps): measure again once the wrist sees the bowl.
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.30),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (38-43 steps; 58-63 for a bowl at x 0.71-0.72)
   (4 probes, 10 eps).
3. Straight down to 1.5 cm below its top, 0.249 (13-27 steps), close (11-12) (10 eps, 4 probes).
4. Lift 8 cm with `robot.goto(by=(0, 0, 0.08))` (17 steps) (2 probes). With the grasp at x 0.69
   or more, pull in while lifting instead: `robot.goto((g[0] - 0.045, g[1], 0.32))`,
   g = grasp pose (20-28 steps) (8 eps, 2 probes).
5. One straight move to the plate's centre with y + 0.04 (the grasp's offset), 0.10 up (64-66
   steps), straight down to 0.045 (16), open: success at most 2 steps after (2 probes; 1 more in
   the swap scene). With under 100 steps left after the lift, move straight to 0.045 in one go
   instead (62-77 steps): success came at its end (9 eps, 2 probes).

## Verified parameters
- Grasp on the +y rim: hand 3.6-4.3 cm to +y of the bowl's centre, 0.245-0.255 up; it closes
  at about 0.8-1.0 cm, holding (20 eps, 4 probes).
- Refused as stretched: a lift straight up from a grasp at x 0.71 (1 ep, 1 probe), and a carry
  over the plate 32-34 cm up (2 eps). The low place pose ran (11 eps, 4 probes).
- During the carry a move may note "the fingers closed fully: what they held slipped out" while
  the bowl still hangs in the hand: go on with the place (8 eps).

## Common failures and fixes
- A slanted move all the way down to 0.045 worked 9 times in 10; once it noted the finger gap
  widening to 1.7 cm and, opened, the bowl lay 2.5 cm to -y of the plate's centre: no success
  (10 eps, plus 1 in the swap scene). Come down the last 5-6 cm straight, as step 5 (2 probes).
- Lift to 0.34, level carry 34 cm up (70 steps), then down: out of steps with the bowl held 6 cm
  up (1 ep); such runs took 187-220 (8 eps). One straight move: 149-206 (10 eps, 2 probes).
- A straight move from the grasp pose to the place pose, with no lift first, was BLOCKED 23 cm
  up (1 probe). Lift or pull in first (2 probes).
- The -x rim at yaw 90 and an 8 cm lift: the straight move was BLOCKED after 29 steps, 24 cm up
  and 34 cm from the place pose (2 probes). After an 11 cm lift it worked: 175 steps (1 probe).

## Typical step costs
- Horizon 220. One straight move, probes 151-192: approach 38-63, down 15-27, close 12, lift or
  pull-in 17-27, move 63-66, open 3 (2 probes); runs 149-182 (9 eps). Two legs: 163, and 213
  with the bowl at x 0.71 (2 probes).
