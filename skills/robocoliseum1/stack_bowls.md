# stack_bowls

## Judging
- One stage: the three bowls nested in one upright stack on the table. Two bowls nested into the
  bowl nearest the robot, left where it stood, counted (L0, L5, L10, L15, L25; 5/5).
- Success came by itself during a wait after the last release, both hands home; no `finish` (5/5).
- A bowl knocked off the table ended the episode at once, the stage not done (L20; 1/1).

## Flow
1. `robot.find('head','bowl')` gave all three (L0, L15, L20, L25; 4/4); or measure each bowl by its
   head pixel, `robot.measure(('head',(u,v)))` (L5, L10).
2. Base: the bowl nearest the robot, never moved (5/5). Carry each other bowl with the arm on its
   side (10/10), one at a time; home that arm before the other starts (5/5).
3. Grasp the rim, never the centre: grasp point about 6 cm from the bowl's centre (half the
   measured width less 1 cm), 1.5 cm below its top (10/10). Far bowl: the rim facing the robot;
   side bowl: either side rim (poses below).
4. Hover open 10 cm above the bowl's top over that spot, re-measure the bowl, go straight down
   (`path='line'`), `robot.gripper(arm,'close')`, `robot.wait(12)`: "closed on something" 0.7-1.1
   cm (holding), no slips (10/10). Lift 13-19 cm with `robot.goto(arm, by=(0,0,...))` (10/10).
5. Re-measure the stack by head pixel (5/5). Aim the grasp point at its centre plus the offset the
   grasp had from the carried bowl's centre (10/10).
6. Go down until the grasp point is about 3.5-7 cm above the top of the stack below; open; lift
   12-15 cm; `robot.home(arm)` (10/10). Each nested bowl raised the top about 5 cm (5/5).

## Verified parameters
- Rim grasps that held: `pitch=90, yaw=0, roll=0`, side rim (L5, L15 x2, L25; 4/4); `pitch=90,
  yaw=90`, rim facing the robot (L10, L25; 2/2; the left hand arrived at yaw -90, the same grasp);
  `pitch=75, yaw=0` or `15, roll=90`, rim facing the robot (L0 x2, L5; 3/3); `pitch=60, yaw=0,
  roll=0`, side rim of a far side bowl (L10; 1/1).
- A refused rim pose: the dry run's "reachable instead" (L5) or a lower pitch (L0, L10) worked.

## Common failures and fixes
- Aimed at a far bowl's centre, the line down was NOT executed ("a joint would have to turn over");
  `path='swing'` to rim height then swung the hand into that bowl and knocked it off the table
  (L20). Aim at the rim facing the robot: at the same far spot, `pitch=90, yaw=90` went by line
  (L25; 1/1). Never swing low near the bowls; if a line is refused, change rim, pitch or yaw, or go
  by a higher point, dry-running first. Swings high above the bowls went fine (L0, L10; 2/2).
- A far side bowl was out of reach at pitch 90 and 80, and it moved 4 cm while poses near it were
  tried. Re-measure, then take its side rim at pitch 60 (L10).
- Carries far across the robot's middle with a tilted hand (pitch 75, 60) took dry runs, high
  waypoints and several turns (L0, L10); at pitch 90, carries crossing it by less than 10 cm went
  in one line move (L5, L15, L25; 3/3). A first bowl moved to the middle (demonstrated): untested.

## Final checks
- Head look: three bowls in one upright stack, both hands home; then `robot.wait(20)` (5/5).
