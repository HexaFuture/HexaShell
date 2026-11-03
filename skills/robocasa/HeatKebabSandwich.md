# HeatKebabSandwich
## Judging
- Done when kebab AND baguette both lie on a toaster oven rack (its tray if none), the door is
  lifted off flat, and the timer is on (a small turn will do). The rack's
  slide, the hand, other knobs do not matter. On the oven floor, the door or another item: no.
## Flow
1. Priors: the door lies open flat toward the robot, hinged at its bottom; racks in; the
   plate with both items beside the oven. Successes: timer, rack out, items, rack in, door (4 eps).
2. Timer first (a revisit cost 240-300 steps, 3 eps). Turn both end knobs of the column or row,
   top/right first: the lowest was not the timer in some kitchens, the top in others (prior tool).
   Pitch 10 for top-row knobs (5 eps), else 30-35 (7 eps); close; roll +55 once (3 eps). After a
   slip, again: two tries a knob, then retry after the door (prior tool; one ate 1650 steps, 1 ep).
3. Rack out if it has room. Front wire: `robot.to_world` down a pixel column of a zoomed crop, the
   spot much nearer than its neighbours (5 eps); next near spot up the column: the rack above or the
   opening top. Under ~12 cm free, step 6 (7-10 cm: the hand stopped 2-7 cm short, 4 eps); 18 cm:
   out (2 eps). Base ~0.5 m behind the wire (0.44-0.59, 5 eps); pitch 35-45, roll 90/-90 (0/180
   slipped, 3 eps); close; `robot.goto(by=(-0.16,0,0))` or `robot.drive(forward=-0.17)` holding (9
   eps); look (a "holding" pull moved nothing, 1 ep). Two short tries: step 6 (4 eps lost 800-1500).
4. Each item: pitch 90, yaw=grip_yaw, lift; stay at pitch 90 to the release, turning only the yaw,
   over the plate (3 eps). Drives held it at pitch 90 5 of 6 times, at roll 90/-90 1 of 4 (6 eps).
   Baguette, 0.43 m+ drives: end grip at yaw 0 lost 6/6, middle grip at yaw 90 held 10/10 (15 eps).
5. Rack out: let go only above its wires; at the opening of a rack in or half out, items fell (3
   eps). Point it in, turned clear of the oven; open over the wires: 10 of 10 landed (8 eps). Yaw 90
   stops 10-13 cm before the face: 11 of 27 landed, 16 fell short (20 eps). Last resort, turned
   over the plate, no drive: pitch 25-35, roll 90/-90, item along the front, 2-3 cm above (4 eps).
6. Rack stays in (prior tool, 10/30): grip 2 cm inside the thicker end (not the middle: the hand
   stays out); over the plate turn the yaw (and the item) till the free end points into the oven, 90
   at most a move. Base 0.50-0.62 m behind the face, item 6 cm left or right of the rack middle.
   Grasp point 4 cm before the face, opening top - 4 cm (at least rack + 5 cm); lower to rack + 3.2
   cm, open, up 6 cm, close; push the near end in at pitch 45, yaw into the oven, rack + 2.5 cm,
   from 10 cm before the face to 2.2 cm in. Then never nudge it.
7. Rack in: closed fingers on its front wire, pitch 35-60, roll 0 or -90, 16-22 cm in (5 eps).
8. Door last, lifted from below; edge grips slip (10+, 4 eps). Base ~0.49 m behind its free edge (3
   eps; nearer: elbow limit, 2 eps). Fingers closed, pitch 45-55, 5-6 cm before the edge, 4-5 cm
   under its top, in to a stop, up 10-14 cm: SUCCESS in the lift (3 eps); from above: on it (2 eps).
## Verified parameters
- Kebab closes 2.4-3.1 cm lifted it (9 eps), under 2 cm slipped (4 eps); baguette 4.7-6.6 cm,
  1.4-1.8 cm below its top (9 eps); on the rack wire 1-3.6 cm or "closed fully" pulled (9 eps).
- Door: edge 26-30 cm from the hinge, top 5-7 cm above the counter (1 ep); level or pitch 20
  there out of reach (3 eps); `robot.home()` under it: lifted (1 ep).
## Common failures and fixes
- Roll-90 pushes into a low opening: BLOCKED ~10 cm short, the hand is taller (2 eps): step 6.
## Typical step costs
- Horizon 2700. Successes 1751-2103: timer 160-330 a knob, rack 100-180, item 200-800, rack in
  50-330, door 70-470 (4 eps). Stay under 450, 250, 500, 250, 250: failures overran (16 eps).
