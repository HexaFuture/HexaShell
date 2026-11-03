# WashFruitColander
## Judging
- Done when EVERY fruit named (listed with commas, no "and") lies in the colander, the
  colander stands under the stream (its middle near the stream, below the outlet) and the water
  runs, at once. The hand need not let go or move away; the colander need not touch the basin.
- Water runs only with the lever stick tipped well over to the robot's RIGHT; toward or away
  from the robot only sets the temperature.
- Still running: no stream in the pictures = the lever is short of its stop; with the stream on,
  a fruit is out or the colander is off the stream (a bump on the spout swings it).
## Flow
1. Priors: the robot is centred on the sink; the colander and each fruit stand at the counter's
   front edge just left or right of it, each on either side. Water starts off.
2. Lever first, basin empty, from the start pose, OPEN hand (scripted: water on in 10 of 10 eps, by
   step 182-230): a thin upright stick (~6 mm) just right of the spout, at the back (1 ep). Its
   top (x, y, z): to_world in the right AND left camera; 6-9 cm further hit the wall: the nearer
   (7 eps, faucet task). Do not drive forward. Stick more than 0.20 m right or 0.15 m left of the
   hand: `robot.drive(left=...)` so that it is 0.10 m right of the hand; (x, y, z) stay. pitch=0,
   yaw=20, roll=180, grip='open': `robot.goto((x - 0.096, y - 0.058, z - 0.013), ...)`, then
   `robot.goto((x + 0.017, y - 0.017, z - 0.013), ..., speed=0.08)`; `robot.push_arc((x, y +
   0.01, z - 0.07), 60, axis='x', push=(0.34, -0.94, 0), press=0.05, tilt=0)`: 'jammed', 'angle',
   'stalled' stops are normal (scripted: 10 eps). Back out `robot.goto(by=(-0.06, -0.03, 0))`,
   `robot.home()`; zoom on the stream.
3. Colander: measure it by pointing; hand above it; `robot.to_world('wrist', ...)` on the near
   rim; pitch 90, yaw 90 (across the rim wall), 2 cm below the rim top; close; lift 0.20; drive.
   Its CENTRE under the spout's outlet (to_world on it); it turns in the fingers: re-measure (1 ep).
4. Each fruit named (names may repeat): hand 0.15-0.17 m above it, pitch 90; point at it in the
   wrist picture; at its centre, yaw = grip_yaw (0 if 'any'), 2-2.7 cm below its top; close;
   lift 0.18; drive at that height (no home); open 3.5-7 cm above the colander's rim top (1 ep).
## Verified parameters
- Colander ~23 x 19 x 9 cm, grasped 2.1 cm below the rim top: holding 3.3 cm; the gap fell to 2.3
  on the lift and 1.8 at the release; it held through a 0.51 m drive. Released, its centre was
  only ~4 cm beyond the grasp point, ~14 cm in front of the outlet (it counted; aim closer, 1 ep).
- Three apples 4-7 cm: 2.0-2.7 cm below their tops, holding 5.2-5.9 cm; drives of 0.43-0.50 m
  with the hand lifted out held (2/2); all three dropped in from 3.5-7 cm above the rim (1 ep).
## Common failures and fixes
- 'mango' was NOT FOUND and 'apple' matched all three fruits. Pointing at the wrist picture's
  centre (hand over the fruit) and `find(at=)` from home returned the counter: point at the
  fruit itself, seen from 0.15-0.17 m above it (1 ep).
- Homing two or three times per fruit cost ~50 steps each (~160 for one fruit, 1 ep).
- The lever was never moved: 'faucet handle' once matched a cabinet handle below the counter;
  from the start base pitch 90 11 cm above the stick was out of reach (1 ep). Use step 2.
## Typical step costs
- Horizon 3150. Successes 1384-2829 (9 eps). In 1 ep: colander in by 405, fruits 270-435 steps
  each, all in by 1472. Plan ~1300: lever ~230 (scripted: 10 eps), colander 300-400, fruits ~250.
