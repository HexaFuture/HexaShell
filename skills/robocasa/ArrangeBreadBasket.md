# ArrangeBreadBasket
## Judging
- Done when the bread lies well inside the basket, the basket stands on the dining counter (by
  the stools, two objects in its middle) and the hand has moved well away from the basket. The
  door, how the bread lies and the order do not matter.
## Flow
1. Priors (10 eps): wall cabinet ahead (one door opened 4 eps, both 5, no door 1); the bread on
   its bottom shelf ~0.5 m above the counter, 0.5-0.7 m ahead; basket 14-36 cm across below.
2. Door: level (pitch 0, yaw 0) 10 cm in front of the bar's measured centre, in to it, close.
   Hinge = the door's outer edge at handle height; right-hinged opens with +, left-hinged with -.
   `robot.arc(hinge, 30, axis='z')` two or three times per door (7 eps).
3. Let go; leave along the door's outward face (right hinge at ~60: `robot.goto(by=(-0.15, -0.12,
   0.02), yaw=50)`), then down below the cabinet at pitch 60, then home (it ran in 5 of 16 eps).
4. Bread: point and measure. Up to ~0.6 m ahead: level 20 cm in front at its mid-height, in to
   1-2 cm past its centre, close, up 7 cm, straight out ~0.3 m (6 eps). Further: a waypoint 8-9 cm
   in front and 3-8 cm above, pitch 15-30, then in to the centre (3 eps).
5. Pitch 55-65 over the basket's centre, grasp point 2-3.5 cm above its top; open; up 0.3 m
   (4 eps). A small low basket (sides <=15 cm, top near 1.0 m): drive back first until its
   centre is 0.45-0.5 m ahead (2 eps), then pitch 65 (1 ep); nearer, that pose was refused or
   stopped short and the loaf fell outside the basket (4 eps).
6. Basket: pitch 90, yaw across one wall's top wire (wrist `to_world` on the plane z = rim top),
   grasp point 1-2 cm below the rim's top (8 eps); close; if `holding`, `robot.goto(by=(-0.17, 0,
   0.02))` or shorter, the hand ending >=0.1 m ahead of the base (15 eps); lift 0.17-0.19 (8 eps).
7. Drive sideways first, then back, then turn: nothing behind the base is in any picture.
   `robot.measure('dining counter')` worked (6 eps); drive until its near edge is ~0.35 m ahead.
8. Pitch 90; lower until the basket's bottom is 2.5-7 cm above the counter (3 eps); open; lift.
## Verified parameters
- Door bars (round or flat, ~21 cm tall): closes 1.3-3.8 cm (9 eps).
- Bread 5.7-9.1 x 2.4-4.6 x 5.7-8.9 cm: holds of 5.0-7.8 cm across the loaf carried, despite the
  "nearly fully open" note (7 eps); 1.7 cm carried (1 ep); 1.5 and 2.2 cm slipped on the lift.
- Basket rim wire: 1.1-3.9 cm held through slide, lift and drive (8 eps); success came on the lift.
## Common failures and fixes
- A loaf 8-9 cm long lying across (grip_yaw beyond about +-60) is wider than the 7.7 cm opening at
  yaw 0: level reaches stopped short at it and closed empty, ~20 times in the failure (2 eps).
  Yaw 25 toward grip_yaw with pitch 15, base 6.5 cm closer, held 7.3 cm (1 ep).
- Small basket (15 x 11 cm) with a wire handle over its middle: bread let go 10 cm above hit the
  rim and shoved it; a second drop tipped it; failure (1 ep). Go low, beside the handle (untested).
- A 7.1 cm hold slid out while tilting to pitch 55 on the way (1 ep): tilt over the basket.
- The bar slipped out in 13 arcs (4 eps); 12 grasps cost ~1400 steps (1 ep). Backing off toward
  the cabinet's middle pushed the door to ~20 degrees: ~400 steps to re-open (1 ep).
## Typical step costs
- Horizon 4350. Successes 1201-2898 (8 eps): doors 430-1700, bread 110-770, basket 90-240, drives
  240-920. Both failures: doors 1400-2300, then 880-1700 on blocked reaches. Aim for ~1200.
