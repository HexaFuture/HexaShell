# libero_spatial_lan_t5

## Judging
- In our episodes the bowl that counted sat on the ramekin: x 0.44-0.475, y +0.185 to +0.215,
  top 0.080-0.083 (20 eps). The plate lies at x 0.70-0.74, y +0.18 to +0.22 (20 eps). Success
  came with that bowl lowered onto the plate (18 eps).

## Flow
1. Measure bowl and plate (free) (20 eps).
2. Over a rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] + 0.04, 0.18), pitch=90,
   yaw=0, grip='open')`, c = bowl centre; the -y rim works alike (24-30 steps) (17 eps, 1 probe).
3. Straight down to about 0.067, 1.4 cm below its top (18-24 steps), close (12-14), holding
   (17 eps, 1 probe).
4. Up to about 0.18 (19-22 steps), level to the plate's centre plus the grasp's offset (33-48),
   straight down to 0.05-0.07 (20-36) and open: success came during that descent or within
   3 steps of opening (15 eps, 1 probe).

## Verified parameters
- The rim grasp put the hand 3.6-4.1 cm to +y or -y of the bowl's centre, 0.064-0.071 up
  (17 eps); let go over the plate, the bowl landed 3.5-3.7 cm from the hand on that side (2 eps).

## Common failures and fixes
- While holding, measure('plate') also noted "other 'plate' seen" about 15 cm up, 5-6 cm from the
  hand: that is the held bowl, read 2 cm farther out than the grasp's offset. Re-aiming from it
  put the bowl 2.6-3.1 cm off the plate's centre and no success came (2 eps). Place by the
  grasp's offset: 11 runs saw that note, ignored it and succeeded (11 eps).

## Typical step costs
- Horizon 220. Probe 161: approach 30, down 19, close 14, lift 20, carry 46, down 30, open 2
  (1 probe). Rim at yaw 0: successes 131-162 (15 eps); at yaw 90: 155-211 (3 eps).
