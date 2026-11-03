# libero_spatial_swap_t7

## Judging
- In our episodes the bowl that counted stood on the stove: x 0.38-0.415, y -0.13 to -0.15,
  top 0.068 (9 eps); a bowl on the table measures about 0.037. The plate lies at x 0.445-0.475,
  y +0.19 to +0.22 (10 eps), a little to +x of the bowl. Success came with the bowl set down
  at the plate's centre (9 eps); let go 7.7 cm up and landing 2 cm off-centre, it did not
  count (1 ep).

## Flow
1. Measure bowl and plate (free). A first measure that was not round, under 9 cm across, had
   its x 2-2.8 cm too small: measure again once the wrist sees the bowl (2 eps).
2. Above its +y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.10),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (22-26 steps) (8 eps). The -y rim
   (c[1] - 0.04) works the same (1 ep, 1 probe).
3. Straight down to 1.5 cm below the bowl's top, about 0.052 (17-25 steps), close (12), lift
   12 cm with `robot.goto(by=(0, 0, 0.12))` (20-21) (9 eps, 1 probe).
4. Carry to the plate's centre with the same y offset (36-39 steps), then straight down to
   0.045 and open; success came within 3 steps of opening, or during the descent
   (8 eps, 1 probe).

## Verified parameters
- Grasp at yaw 0 on the +y or -y rim: hand 3.6-4.3 cm to that side of the bowl's centre,
  0.049-0.054 up; it closes at about 0.8-0.9 cm, holding (9 eps, 1 probe).

## Common failures and fixes
- Over the plate, measure('plate') also noted "other 'plate' seen" about 11 cm up: that is the
  held bowl, read 2-3 cm off. Re-aiming from it moved the hand 1.7 cm off the planned place
  pose; let go 7.7 cm up, the bowl landed 2 cm off-centre and no success came (1 ep). Place at
  the plate's centre plus the grasp's offset and go down to 0.045 before opening
  (8 eps, 1 probe: 130 steps).

## Typical step costs
- Horizon 220. Probe 130: approach 31, down 17, close 12, lift 21, carry 39, down 10 (1 probe).
  Learning-round successes 132-155 (9 eps).
