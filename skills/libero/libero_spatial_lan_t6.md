# libero_spatial_lan_t6

## Judging
- In our episodes the bowl that counted measured x 0.77-0.805, y -0.055 to -0.085, top about
  0.036 (9 eps); the plate x 0.70-0.73, y +0.19 to +0.21 (10 eps). Success came with that bowl
  set down at the plate's centre (9 eps); landing 2-3 cm off-centre, it did not count (1 ep).

## Flow
1. Measure bowl and plate (free). Measure the bowl again once the wrist sees it: the first
   measure's x read up to 1.3 cm too small (10 eps).
2. The -x rim (nearer the robot), turning and opening on the way, low: `robot.goto((c[0] - 0.04,
   c[1], 0.07), pitch=90, yaw=90, grip='open')`, c = bowl centre (48-50 steps) (1 ep, 1 probe).
3. Straight down to 0.024 (10-15 steps from 0.07), close (12), lift 11 cm with
   `robot.goto(by=(0, 0, 0.11))` (22-32) (9 eps, 1 probe).
4. Carry 13-15 cm up to the plate's centre with x - 0.04 (the grasp's offset), then straight
   down to 0.04 and open; success came at the end of that descent or within 3 steps of opening
   (7 eps, 1 probe).

## Verified parameters
- Yaw 90 closes the fingers along x on the bowl's -x wall: hand 3.4-4.1 cm to -x of its centre,
  at the centre's y, 0.025-0.028 up; closes at about 0.8 cm, holding (9 eps).
- The -y rim at yaw 0 is at the edge of reach: refused as stretched at pitch 90 (3 eps).

## Common failures and fixes
- The -y rim at pitch 60 after that refusal: the hand stopped 1.4 cm from its target, "short of
  target", and 122 steps were gone by the lift (1 ep). The -x rim at yaw 90: 154 steps in all
  (1 probe).
- Re-aiming from an agentview spot of the hanging bowl moved the hand about 5 cm off the planned
  place pose; let go 6 cm up, the bowl landed 2-3 cm off-centre and no success came (1 ep). Keep
  the grasp's offset and go down to 0.04 first (9 eps, 1 probe).

## Typical step costs
- Horizon 220. Probe 154: approach 50, down 15, close 12, lift 25, carry 32, down 17, open 3
  (1 probe). Learning-round successes 161-203; 8 of them approached 16-20 cm up first and spent
  22-78 steps coming down (9 eps).
