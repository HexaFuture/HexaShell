# PackIdenticalLunches
## Judging
- Done when each box holds exactly one named vegetable and one named meat, both well inside it
  (on another item or across both boxes fails), all let go and the hand well away, at once.
- An item on the floor is lost (12 eps). The fridge doors start open.
## Flow
1. Priors: items at the front of fridge shelves about 1.0-1.5 m up; the boxes side by side
   at the front of the nearest counter, 0.5-2.7 m aside, behind or round a corner (7 eps).
2. Doors: one pushed shut cost 570-3040 steps (4 eps). Turn and slide only 0.6+ m behind the
   start pose; nearer, only straight back or in, base within 0.12 m sideways of it (prior tool,
   15/50). Diagonal `drive(to=...)` returns from 0.8-0.9 m aside swung a door nearly shut 3 of 3;
   slide-then-in legs 0 of ~25 (10 eps). Slides over 0.8 m: stop 0.7 m short, look (prior tool).
3. Survey from 0.6 m back, turning or sliding along the counter till both boxes show (88-490
   steps, 10 eps). 'tupperware' finds none: point at a rim (7 eps); clear ones measure flat (9 eps).
4. Shelf height: a neighbour hiding an item's foot makes its `.on_z` read 2-2.5 cm high and its
   height low (2 eps): use the lowest `.on_z` on that shelf, or `robot.to_world` on bare shelf.
5. Stand at heading 0, the base 0.58 m behind the grasp point (0.55 above 1.42 m), in line with
   it, at most 0.12 m sideways of the start pose (prior tool); holds came from 0.44-0.61 m (6 eps).
6. Grasp at pitch 45 (30 above 1.42 m), the point under `.center` at `.on_z` + max(half its
   height, 1.1 cm); yaw `grip_yaw` if within 45 of 0 (None: 0); beyond, 0 if under 7 cm long, else
   45-60 toward it after `dry_run=True`. Open; turn the hand 0.33 m ahead of the base, 6 cm above
   the point; go to 12 cm back from it along the hand, then 3 cm past it (2 if under 3 cm tall):
   BLOCKED there is expected, 3.5+ cm short is not (prior tool, 15/50; 22 of 28 in vs 17 unpushed).
7. Close, then at once ease the hold: `robot.gripper(max(0, int(g.width_m / 0.008) / 10 - 0.2))`, g
   the close's Grip: still a squeeze, lighter (replays); empty, the gap reads the asked one. Lift
   2.5 cm, back out 12 cm along the hand; 0.12+ m sideways of the base, `robot.goto(by=(0, dy, 0))`
   in front of it (prior tool; with it 2 of 2 drove clear; without, from 0.2+ m 7 of 7 BLOCKED, 4
   fell, 8 eps); `robot.drive(forward=-0.25)` to clear the fridge, `robot.home()` (prior tool).
8. A gap 3+ mm under the close before that drive: set it back down, open, regrasp; 3 of 4 floor
   drops had shrunk 4-6 mm on the back-out, then went home at the shelf (4 eps). Refused for
   slipping before a turn or slide: drive back in, do so (untested; 6 of 13 sent on fell, 12 eps).
9. At a box: base 0.47-0.6 m behind its centre (a leg BLOCKED at the counter jolted 2 of 4 items
   out, 4 eps); release at pitch 75-90, grasp point 7-20 cm above the counter: 27 of 27 in (11 eps).
## Verified parameters
- Held to a box (pitch, closed cm): tomato 15, 6.9; sweet potato 15, 5.0-5.3; eggplant 35, 5.6;
  thin steak 15, 5.8; mushroom 20, 5.5; potato 15-63, 5.3-6.8; lemon 20-60, 5.0-5.7; drumstick
  15-74, 2.0-6.5; broccoli 15-90, 3.3-5.7; pepper 90, 5.5-6.5; fish 20 or 89-90, 2.2-4.8 (11 eps).
## Common failures and fixes
- Pitch 90 under a shelf close above is BLOCKED (7 eps); 0 of 3 such items got in, 1090-1370 steps
  each, two pushed 14-19 cm deeper (3 eps): take them last, at pitch 30 or 15 (untested).
- A 7.3-7.8 cm close on a 3-5 cm item rests on top of it: all 3 fell on the lift (3 eps).
## Typical step costs
- Horizon 3900; successes ended at 2497-3705 (4 eps). A clean item 350-660 (8 eps), a failed grasp
  30-110, the survey 88-490 (10 eps); prior tool 600-800 an item. Plan: 400 + 4 x 650 = ~3000.
