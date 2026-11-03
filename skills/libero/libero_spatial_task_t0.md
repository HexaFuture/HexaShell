# libero_spatial_task_t0

## Judging
- In our episodes the bowl that counted was the nearer one (smaller x), to +y of the ramekin:
  right of it in the agentview picture (9 eps). It measured x 0.41-0.50, y +0.30 to +0.38
  (9 eps). The farther one, set on the plate's centre, did not count (1 ep).
- The ramekin is the small round grey cup; measure('ramekin') found nothing (1 ep). The other
  bowl stood between the ramekin and the plate (7 eps), beside the ramekin on its +y side (1 ep)
  or leaning against it, top about 0.09, not round (2 eps). In the agentview picture it is left
  of (-y) and lower than (+x) the one that counted (10 eps); where both were measured, it stood
  5-7 cm to +x and 8-13 cm to -y of it (2 eps).

## Flow
1. Find both bowls and measure them and the plate (free): the plate measured x 0.70-0.745,
   y +0.11 to +0.30 (10 eps); the other bowl x 0.52-0.57, y +0.10 to +0.26 (3 eps).
2. Above its -y rim at yaw 0, opening on the way: `robot.goto((c[0], c[1] - 0.04, 0.07),
   pitch=90, yaw=0, grip='open')`, c = bowl centre (36-37 steps) (1 ep, 1 probe). The +y rim
   (c[1] + 0.04) works too (7 eps), but see the plate far to +y below.
3. Straight down to 0.024 (14-28 steps), close (12), lift 10 cm with
   `robot.goto(by=(0, 0, 0.10))` (18-23) (8 eps, 1 probe).
4. Carry 12-18 cm up to the plate's centre with the same y offset (36-49 steps), then straight
   down to 0.04-0.045 and open; success came during that descent or within 3 steps of opening
   (9 eps, 1 probe).

## Verified parameters
- Grasp at yaw 0 on the +y or -y rim: hand 3.4-4.0 cm to that side of the bowl's centre,
  0.023-0.029 up; it closes at about 0.8-0.9 cm, holding (8 eps, 1 probe).

## Common failures and fixes
- Took the farther bowl, leaning against the ramekin's -y side at the start (measured top
  0.091, x 0.52, y +0.10): set down on the plate's centre, no success came (1 ep). The nearer
  bowl with the same flow: success at 148 (1 probe).
- Grasp on the +y rim with the plate far to +y (centre y +0.30): the place pose at y +0.34 was
  refused as stretched at pitch 90; pitch 80 worked (1 ep). The -y rim keeps the hand at -y of
  the plate's centre (1 ep, 1 probe).

## Typical step costs
- Horizon 220. Probe 148: approach 36, down 28, close 12, lift 18, carry 39, down 15 (1 probe).
  Learning-round successes 148-198 (9 eps).
