# libero_goal_object_t8

## Judging
- Carried by its rim and lowered onto the plate, the bowl ended the episode with SUCCESS as it met
  the plate, the fingers still closed, the hand at z 0.074-0.094 (9 eps, 2 probes).

## Flow
1. `b = robot.measure('bowl')`, `p = robot.measure('plate')`: bowl about 0.144 x 0.145 x 0.064 m,
   top 0.062-0.068, centre 0.55-0.64 m ahead, -2 to +9 cm in y; plate 0.116-0.12 x 0.093-0.105
   m, top 0.004-0.005, centre 0.68-0.72 ahead, 1-3.5 cm to -y (10 eps). Each was also named
   "other" in the other's measure: go by the size (9 eps). A cabinet is by the plate's -y side.
2. A bowl read taller than about 0.07: measure it again with the hand above it (1 ep: 0.137 x
   0.119 x 0.119 became 0.146 x 0.138 x 0.064, 4.6 cm away).
3. Take a rim away from the cabinet. The +y rim at yaw 0: `robot.goto((b.center[0],
   b.center[1] + 0.061, 0.15), pitch=90, yaw=0, grip='open')`; or the -x rim at yaw 90:
   `(b.center[0] - 0.063, b.center[1], 0.15)` with yaw=90. Straight down to z 0.045-0.058,
   close: holding at 0.8-1.1 cm (7 eps).
4. `robot.goto(by=(0, 0, 0.12))`: the bowl tips on its rim and the gap drops to 0.5-0.6 cm
   with a note that it changed. It holds: go on (10 eps).
5. Carry at z 0.165-0.175, the hand at the plate's centre plus the same offset:
   `(p.center[0], p.center[1] + 0.061, 0.168)` or `(p.center[0] - 0.063, p.center[1], 0.168)`
   (7 eps, 2 probes). A second plate measure moved under 1.5 cm (7 eps).
6. Straight down to z 0.065-0.07: SUCCESS 11-15 steps into the lowering, 18-20 at speed
   0.10-0.12 (7 eps, 2 probes). If the move ends first, open: SUCCESS on the open (2 eps).

## Verified parameters
- Grasp point on the rim 5.8-6.7 cm from the bowl's measured centre, at z 0.045-0.058: the
  bowl came along through the lift and the carry every time (10 eps, 2 probes).
- The hand 5.8-6.6 cm from the plate's centre, on the side it holds the bowl, lowered to z
  0.074-0.094: SUCCESS (9 eps, 2 probes).

## Common failures and fixes
- The -y rim at yaw 0 puts the hand 6-7 cm -y of the plate's centre, over the cabinet's edge:
  the carry stopped "short" 2.2-3.6 cm off at z 0.157-0.163, and moves down from there, also at
  pitch 70-80, were BLOCKED (3 eps). Released from z 0.18, the bowl landed tilted against the
  cabinet, half on the plate, and the steps ran out (1 ep). A turn to yaw -90 up at z
  0.18-0.205, clear of the cabinet (44-45 steps), swings the bowl +x of the hand; the hand then
  went 6 cm -x of the plate's centre and down: SUCCESS at 204 (2 eps). From the same start as
  the failure, the +y or -x rim gave SUCCESS at 114 and 131 (2 probes).

## Typical step costs
- Horizon 300. +y rim at yaw 0: pre-pose 24-27, down 17-19, close 11, lift 20-21, carry 24-26,
  lower 11-20: SUCCESS at 109-120 (3 eps, 1 probe). The -x rim at yaw 90 adds about 20 to the
  pre-pose for the turn: 131-147 (4 eps, 1 probe). The -y rim with the yaw -90 turn took 204
  (2 eps); the failure ran out at 300 (1 ep).
