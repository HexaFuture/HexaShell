# ArrangeTea

## Judging
- Done when the kettle and the mug both stand on the tray near its middle, BOTH cabinet doors are
  fully shut (flush), and the hand is well away from the mug, all at once.

## Flow
1. Priors: the doors start wide open; the mug stands at a shelf's front, near its middle.
   Kettle and tray: in reach (7 eps), 0.45-0.75 m to one side (2 eps), ~1 m behind (1 ep).
2. Kettle by its top handle: pitch 90, yaw = its grip_yaw, grasp point 0.5-1 cm below its top (6
   eps); close only if the hand got there (2 eps); up 10 cm, BLOCKED lifts held (5 eps): if so,
   draw it back 10 cm toward the robot at that height, then up; still BLOCKED: 32 cm ran (2 eps).
3. Kettle centre 8-12 cm in front of the tray's centre (6 eps); open with its bottom at the tray's
   top: bottom = grasp point - (pick-up grasp point - the tray's `.on_z`), -1 to +0.8 cm (4 eps).
4. Mug: level, yaw 0, 1.5 cm past its centre at mid-height, close, up 7 cm, out 0.27 m (7 eps).
5. Mug at pitch 45: to a point in front of the kettle, then across at that height to 5-8 cm behind
   the tray's centre, the grasp point 4+ cm above the kettle's top (1 ep; take its height from the
   pick-up, later measures read 3-6 cm low, 3 eps); down to 7-8.5 cm above the tray's top (3 eps).
   Slanted moves down over or past the kettle: 1 fell (~700 steps), 4 BLOCKED (70-300) (5 eps).
6. Doors, each from OUTSIDE (its outer face). Hinge = its outer edge at the cabinet front, by
   robot.to_world. Home; base square, 0.65-0.85 m back from it, 0.1 m out to 0.2 m in (7 eps).
7. Never straight to the press point (~13 BLOCKED, 8 eps): closed, level, yaw -90 (left door) or 75
   (right), 6-16 cm above the mug's shelf; 0.6-0.7 m out, past the door's plane, in to 0.4-0.5 m
   out, sweep 15-30 cm to the middle; `robot.arc(hinge, 35, axis='z')` (-35 right) (7 eps).
8. 2nd arcs of 25-35 degrees were refused or stopped 8 of 13 times, 15-20 ran 2 of 3 (8 eps):
   dry-run one; if refused, leave it: the last push closed doors left 12-40 degrees open (4 eps).
9. Last: home; base 0.45-0.65 m back from the doors' middle; closed, level, yaw 0, same height: to
   10-23 cm out, then straight in to 1-5 cm past their front. Success fired on it (6 eps).

## Verified parameters
- Grips that held: kettle handle 1.2-2.8 cm (9 eps), mug 0.6-1.6 cm (7 eps).

## Common failures and fixes
- Kettle: the descent stopped 2-3 cm short, grips of 0.6-1.0 cm fell on the lift 4 of 4 (2 eps).
  Re-aim with robot.to_world on the handle in the wrist picture, descend again: 1.7 cm held (1 ep).
- Mug approach stopped short and closed anyway: 13 cm short fell on the pull-out (1 ep); 3 cm short
  slipped in a 1.1 m drive to the floor (1 ep). Open and re-approach to full depth: held (1 ep).
- Wide cup, handle to the side: level grips closed empty 5 times (2 eps); pitch 35 down onto its
  middle closed 7.5 cm and held (1 ep), pitch 30 BLOCKED (1 ep). Go to pitch 35 first (untested).
- A door in an inside corner (a counter and a tall unit on its outer side): 22 drives toward that
  side were held off (616 steps), 13 arm moves there BLOCKED; it stayed open, no fix found (1 ep).

## Typical step costs
- Horizon 2250. Successes 1295-2107 (6 eps): kettle 150-650, mug 320-630, doors 770-1035 (most).
