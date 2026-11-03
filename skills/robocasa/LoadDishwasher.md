# LoadDishwasher

## Judging
- Done when both dishes rest on the upper rack (not the door, the tub floor or only on the other
  dish) and the door is shut; the hand may stay near. Tilted on the wires counts (1 ep). The
  rack starts out over the open door; the door or the base pushes it in (7 eps): no rack push.

## Flow
1. Cup: pitch 90, yaw 0, hand 2.5-3 cm left or right of its middle (one finger inside), 0.5-1.5
   cm below the rim (6 eps). robot.home(); robot.drive(forward=-0.3); sideways till square to the
   rack; forward 0.14 (18 eps). Rack front wall x and top: `robot.to_world`, many pixels (13 eps).
2. Each dish in two legs (15 eps): toward the robot till all of it is 2+ cm in front of the
   counter's edge; down there till its bottom is 2+ cm above the wall's top (1 ep), often 15-25 cm
   under the counter top (the cup's `.on_z`); along +x till its middle is 1.5 cm + half its width
   past the wall (prior tool); down till its bottom is 1-4 cm above the wires. **Open only if that
   move arrived and the wrist picture shows wires under the dish** (13 eps). Back 8-10 cm, up.
3. Bowl: pitch 90, yaw 0, hand 4-5 cm (half its width less 1 cm) left or right of its middle,
   1-2 cm below its top: all 7 held and went in (7 eps). Yaw 90 slipped or jammed (2 eps). Hand
   BLOCKED going down (something tall just behind): the same grip at pitch 45-55 (1 ep, prior tool).
4. Door, straight after the bowl: robot.home(); sideways till the base faces the door's middle;
   cells of `for i in range(4): print(robot.drive(forward=0.3))` and a look. Each drive stops
   BLOCKED in 8-14 steps, 0.5-8 cm further; the door shut after 2-15 drives (4 eps). Go on while
   they gain 3+ mm, even 7-12 mm for 4-6 in a row (2 eps). Never back off: it falls open (2 eps).
5. Two drives in a row under 3 mm (untested): arm push. Door top edge by `robot.to_world`; hand
   closed, pitch 55, yaw 0, 6 cm in front of and 5 cm below it, base 0.6 m behind that (2 eps).
   Legs `robot.goto(by=(0.05, 0, 0.03), speed=0.05)`, at most 7 cm up, then `by=(0.05, 0, 0)`;
   out of reach: robot.drive(forward=0.13), hand on the door at pitch 45-55; shut so (2 eps).

## Verified parameters
- Cup grips 0.6-1.3 cm held unless knocked or opened (27 eps); round the whole cup (6.9 cm) it
  slipped (1 ep). Bowl wall grips 1.6-3.5 cm lost 30-75% of the gap on the lift, held (7 eps).

## Common failures and fixes
- Down over the spot from above the counter: the dish hit the counter's edge and slipped (6 eps).
- Leg toward the robot refused, went down over the counter instead: lost (1 ep). Drive back 0.15
  and redo the leg (untested). `goto(grip='open')` while holding opens mid-move: dropped (1 ep).
- In-move BLOCKED, front past the wall: 4-7 cm lower there, on (2 eps). Short of it (dish under
  the wall's top): lower tries BLOCKED, slipped (1 ep); back 4 cm, 4+ cm up, pitch 65 (1 ep).
- Opening after a short or BLOCKED move: the dish fell on the door (4 eps).
- Base push given up after 1-6 drives: shut 730-920 steps later (3 eps) or never (2 eps). Arm
  first: 270-900 steps finding a reachable start (6 eps); pitch 80-90 legs turned the hand (6 eps).

## Typical step costs
- Horizon 1800. Both dishes in by 626-855 (8 eps). Door by base drives 205-265 once begun (3
  eps), by the arm 246-348 (2 eps); 945-1120 where the arm start was hunted (5 eps).
