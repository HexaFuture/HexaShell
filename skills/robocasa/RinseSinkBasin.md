# RinseSinkBasin
## Judging
- Done once, with the water running, the spout has been straight, swung well to the robot's left
  and well to its right (swing it far, 40 degrees or so each way), in any order. Keep the water
  running through the swings. Still running: a side not far enough, or water off.
- Water on = the lever stick tipped well over to the robot's RIGHT; it stays on through the
  swings, and a swing through straight counts as straight too (4 eps).
## Flow
1. Priors (12 eps): the lever is a thin stick a few cm tall, just right of the spout's upright
   tube, ~0.7-0.8 m ahead of the start; the spout arches ~0.3-0.4 m above the counter toward the
   robot, its nozzle ~0.2 m in front of the tube. Do not drive forward first (scripted: 20 eps).
2. **Water first**, open hand (grasps held the stick only after 230-1150 steps, 6 eps). Its top
   (x, y, z): to_world in the right AND left camera; 6-9 cm further = the wall: the nearer (6 eps).
3. Stick more than 0.20 m right or 0.15 m left of the hand: `robot.drive(left=...)` so that it is
   0.10 m right of the hand; (x, y, z) stay. pitch=0, yaw=20, roll=180, grip='open': `robot.goto(
   (x - 0.096, y - 0.058, z - 0.013), ...)`, then `robot.goto((x + 0.017, y - 0.017, z - 0.013),
   ..., speed=0.08)`; `robot.push_arc((x, y + 0.01, z - 0.07), 60, axis='x', push=(0.34, -0.94,
   0), press=0.05, tilt=0)`: on at the first push in 10 of 10, done by step 155-231 (10 eps; and
   scripted: 20 of 20 eps). Back out `robot.goto(by=(-0.06, -0.03, 0))`, then `robot.home()` (~145
   steps): from the push's end, pitch 45-90 moves were NOT executed (6 eps) or BLOCKED (1 ep).
4. No stream ('jammed', 'angle', 'stalled' stops at the stick's end are normal; scripted: 20 eps):
   to_world the top; moved under 2 cm right = too little (3 eps); 'no touch': wrist camera, 1.5 cm
   further in (untested). Then pinch: open 0.5, via 8-12 cm behind-right, pitch 60, grasp ~1 cm
   below the top (2 eps); `robot.goto(by=(0, -0.055, -0.02))`, open, `robot.goto(by=(-0.10, 0,
   0.13))`, look. "short of target" or BLOCKED descents hit the arch or top: back out, never close
   there (25 empty, 5 eps).
5. Spout, from above (2 eps): pitch 90, yaw so the fingers close across the arch (0 when it points
   at the robot), 3-6 cm behind the nozzle; close 2.9-3.3 cm; `robot.arc(tube, 20, axis='z')` one
   at a time (+ right, - left), `robot.wait(30)`, look for the stream: 12 arcs held (1 ep).
## Verified parameters
- Lever pinches by distance: pitch 90 reached it only ~0.70 m ahead (2 of 3 eps); at 0.73 m or
  more BLOCKED or "stretched" (6 eps). Pitch 0-15, yaw 0, 0.73-0.74 m ahead: 4 pinch pushes
  turned without water, a 5th (roll 90) slipped and turned it on (2 eps).
- Spout grips at pitch 35-60 (2.2-4.6 cm): arcs of 15-20 degrees lost the grip 4 of 4 (4 eps);
  5-6 cm legs held in 2 eps, slipped 5 times in 2. Pitch 90, yaw 0, legs: 1 of 2 slipped (1 ep).
## Common failures and fixes
- Water never on (5 eps, 16-30 lever tries, 1222-1350 steps): pinches of 0.5-3.5 cm slipped or
  turned in pushes without water 24 times (9 eps). Water on at ~780, then pitch 35-60 spout grips
  came up empty 4 times and slipped in an arc and 2 legs (1 ep): step 5.
- An open level hand pushed with `robot.goto(by=...)`: BLOCKED on the tube or ran past the stick;
  success in 3 of 10 eps (10 eps). Use push_arc, step 3.
- Pinches at yaw 65-90 either way, pitch 45-90: BLOCKED 25 times (5 eps).
## Typical step costs
- Horizon 1350. push_arc successes 511-683, the spout 304-494 of it, home included (9 eps); 1255
  after a BLOCKED arm (1 ep). Pinch successes 742-1347, lever 310-950, spout 230-680 (4 eps).
