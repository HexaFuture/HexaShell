# TurnOnSinkFaucet
## Judging
- On = the lever stick tipped well over to the robot's RIGHT, near its stop, in every sink seen;
  its fore-aft tilt is the temperature and does not count. It stays where a push leaves it.
  Success fired during a push or the open right after (3 eps).
## Flow
1. Priors (scripted: 20 eps): a thin stick, 3-9 cm tall, on a hub just right of the spout's
   upright tube; the spout arches or runs forward ABOVE and in front of it. Its top is 1.01-1.11 m
   high, 0.67-0.85 m ahead of the start, 0.32 m right to 0.06 m left. Do not drive forward.
2. Free: zoom the right AND the left camera on the stick; `robot.to_world` its top in each, before
   any move. A spot 6-9 cm further than the other hit the wall behind: take the nearer (7 eps: 6
   failed on it, 1 was BLOCKED at the stick and redone). On the stick: within 1 cm (28 eps).
3. Stick more than 0.20 m right or 0.15 m left of the hand: `robot.drive(left=...)` so that it is
   0.10 m right of the hand; (x, y, z) stay as they are (scripted: 5 of 20 eps).
4. Open level hand, pitch=0, yaw=20, roll=180, grip='open': `robot.goto((x - 0.096, y - 0.058,
   z - 0.013), ...)`, then `robot.goto((x + 0.017, y - 0.017, z - 0.013), ..., speed=0.08)`: the
   stick in the hand, 1.5 cm from the left finger, 1 cm past the pads' middle (scripted: 20 eps).
5. `robot.push_arc((x, y + 0.01, z - 0.07), 60, axis='x', push=(0.34, -0.94, 0), press=0.05,
   tilt=0)`: on during the push, after 36-46 steps (scripted: 20 of 20 eps); 137-162 steps from
   the start (9 eps). The stick measured 1.5 cm off in x, 1.2 in y, 1 in z: the same (scripted).
6. Not on: `.stop` 'no touch' = the fingers passed the stick: measure it with the wrist camera
   (~0.8 mm a pixel) and come 1.5 cm further in. 'angle' or 'stalled' with the top moved under
   2 cm right = it pushed nothing. Back out `robot.goto(by=(-0.06, -0.03, 0))`; again (untested).
7. Second way, grasp (10 eps): pitch 45, yaw 0, roll 0, open, 12-13 cm in front of the stick,
   4-6 cm to its RIGHT, 9 cm above its top; ONE straight move to its x and y, 1-2 cm below the
   top; close (holding 0.9-1.3 cm); `robot.goto(by=(0, -0.05, -0.015))`: "short of target" after
   2-4 cm is normal; still off, `robot.goto(by=(0, -0.03, -0.01))` from the same grasp (2 eps).
## Verified parameters
- Roll 180: at roll 0 the right finger hit the spout or reach failed (scripted: 5 of 20 eps).
- A hinge 7 cm below the top, 1 cm left of the stick, did as well as the stick's own pivot, a few
  cm below the top (scripted: 19 of 20 eps each). 'faucet handle' found the hub or the tap (5 eps).
- Grasps that turned it: pitch 30-45, yaw 0, roll 0, 1-2.5 cm below the top (2 eps); pitch 45,
  yaw 90, roll 90, 1.6 cm below the top: on at the first push (1 ep). Both close along the push.
## Common failures and fixes
- Pitch 70-90 from above the stick: BLOCKED by the spout or the stretched arm, or short (~25
  tries, 9 eps); none led to water. Do not try it.
- Grasp pre-poses within 3 cm of the stick's line or left of it: the hand ran onto the spout
  (BLOCKED, 8 eps; in 1 it wedged and all 20 later arm moves and homes were BLOCKED).
- Fingers closing fore-aft (yaw 40-90 at roll 0, or pitch 90): the stick slid out of the
  fingertips in 8 of 9 pushes (4 eps). Close them left-right, as in step 7.
- An open level hand pushed with `robot.goto(by=...)`: most pushes ran their full 9 cm within 1-3
  mm, past the stick, or were BLOCKED on the tube; on in 3 of 10 eps (10 eps). Use step 5.
## Typical step costs
- Horizon 600. push_arc: 137-162 in all, 222 with one redo (10 eps). Grasp successes used 304-472,
  100-200 on blocked tries (3 eps); failures spent it all on 6-20 blocked or slipped tries (7 eps).
