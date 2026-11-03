# libero_spatial_swap_t9

## Judging
- In our episodes the bowl that counted sat on top of the wooden cabinet: x 0.65-0.73, y -0.255
  to -0.30, top 0.263-0.266 (19 eps). The plate lies at x 0.445-0.475, y +0.18 to +0.22 (20 eps).
  Success came with the bowl set down at the plate's centre (18 eps); let go 5 cm up and landing
  2.8 cm off-centre, it did not count (1 ep). Another black bowl stood on the stove, near
  x 0.41, y -0.15, top 0.067 (1 ep); no run moved it.

## Flow
1. Measure bowl and plate (free). The first measure from the start pose had its x 0.7-2.7 cm
   too small (11 eps): measure again once the wrist sees the bowl.
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.30),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (38-46 steps, up to 58 for a bowl at x 0.71-0.73)
   (3 probes, 10 eps).
3. Straight down to 1.5 cm below its top, about 0.249 (12-22 steps), close (11-12)
   (10 eps, 3 probes).
4. Lift 8 cm with `robot.goto(by=(0, 0, 0.08))` (17 steps) (2 probes). With the grasp at x 0.69
   or more, pull in while lifting instead: `robot.goto((g[0] - 0.04, g[1], 0.33))`,
   g = grasp pose (16-30 steps) (12 eps, 1 probe).
5. One straight move to the plate's centre with y + 0.04 (the grasp's offset), 0.10 up (55
   steps), then straight down to 0.045 (12): success came on that descent (1 probe; 2 more in
   the object scene, within 2 steps of opening).

## Verified parameters
- Grasp on the +y rim: hand 3.4-4.8 cm to +y of the bowl's centre, 0.246-0.253 up; it closes at
  about 0.8-1.0 cm, holding (19 eps, 3 probes).
- A lift straight up from a grasp at x 0.71 was refused as stretched (2 eps).
- During the carry a move may note "the fingers closed fully: what they held slipped out" while
  the bowl still hangs in the hand: go on with the place (8 eps).

## Common failures and fixes
- One slanted move all the way down to 0.045 ended the episode 6 times in 10; twice BLOCKED
  midway 24-25 cm up (done on steps 180 and 199); once the bowl truly slipped out on the way;
  once the finger gap widened to 1.7 cm and, opened, the bowl lay 2.6 cm to -y of the plate's
  centre, re-grasped and done on step 210 (10 eps). Come down the last 5-6 cm straight (2 probes).
- Over the plate, measure('plate') also noted "other 'plate' seen" 27 cm up: that is the held
  bowl, read 2-3 cm off. Re-aiming from it moved the hand 3 cm off the planned place pose; let
  go 5 cm up, the bowl landed 2.8 cm off-centre and no success came (1 ep). Place at the
  plate's centre plus the grasp's offset (2 probes).

## Typical step costs
- Horizon 220. Probes 145-160: approach 39-46, down 15-21, close 11-12, lift or pull-in 17-19,
  straight move 57, open 5-6 (2 probes); two legs 151 (1 probe). Runs 141-160 (6 eps).
