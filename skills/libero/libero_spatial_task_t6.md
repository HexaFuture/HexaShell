# libero_spatial_task_t6

## Judging
- In our episodes the bowl that counted stood on the stove: x 0.39-0.42, y -0.12 to -0.155,
  top 0.068 (9 eps); a bowl on the table measures about 0.037. The plate lies at x 0.70-0.73,
  y +0.19 to +0.215 (10 eps). Success came with the bowl set down at the plate's centre
  (9 eps); let go 5.7 cm up and landing 2.5 cm off-centre, it did not count (1 ep).

## Flow
1. Measure bowl and plate (free). A first measure from one view only, not round, had its x
   2.8 cm too small: measure again once the wrist sees the bowl (1 ep).
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.10),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (23-28 steps) (6 eps, 1 probe). The -y rim
   (c[1] - 0.04) works the same (3 eps).
3. Straight down to 1.5 cm below the bowl's top, about 0.052 (12 steps from 0.10, 18-29 from
   13-18 cm up), close (12), lift 12 cm with `robot.goto(by=(0, 0, 0.12))` (20-23)
   (9 eps, 1 probe).
4. Carry to the plate's centre with the same y offset (47-69 steps), then straight down to
   0.045 and open; success came during that descent or within 2 steps of opening
   (9 eps, 1 probe).

## Verified parameters
- Grasp at yaw 0 on the +y or -y rim: hand 3.5-4.2 cm to that side of the bowl's centre,
  0.051-0.054 up; it closes at about 0.8-0.9 cm, holding (9 eps, 1 probe).
- On the way to the plate a move may note "the fingers closed fully: what they held slipped
  out" while the bowl still hangs in the hand: go on with the place (4 eps).

## Common failures and fixes
- Over the plate, measure('plate') also noted "other 'plate' seen" about 15 cm up: that is the
  held bowl, read 2-3 cm off. Re-aiming from it moved the hand 2.6 cm off the planned place
  pose; let go 5.7 cm up, the bowl landed about 2.5 cm off-centre and no success came (1 ep).
  Place at the plate's centre plus the grasp's offset and go down to 0.045 before opening
  (7 eps, 1 probe: 143 steps).

## Typical step costs
- Horizon 220. Probe 143: approach 27, down 12, close 12, lift 20, carry 55, down 17 (1 probe).
  Learning-round successes 144-177 (9 eps).
