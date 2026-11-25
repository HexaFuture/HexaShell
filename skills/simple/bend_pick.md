# bend_pick

## Judging
- One stage: the box lifted well clear of the table in the right hand. Standing up straight
  with the box held (`robot.body(0.76, pitch=0)`) lifted it enough; it counted on the way up
  (scripted 20/20, and 1/1 run of the same steps through `robot`).
- The official limit is short (400 steps); the scripted runs finished at 350-390 (scripted).

## Flow
1. `robot.gaze(0, 55)`, look, `robot.measure('cracker box')`: it stands upright on a low table,
   its top about 0.62 m high, 0.37-0.42 m ahead, its long side across (6/6 runs).
2. Get the hands clear before going down (one cell from here to step 6): the left palm about
   0.25 m ahead, 0.2 m left and 0.95 m high, facing in; the right palm facing down about 1.0 m
   high, about 0.28 m ahead (short of the box, not over it) and about 23 cm right of the box's
   centre, `hold='pelvis'`: 47-58 steps (V2, V3; scripted). Over the centre of a box 0.40 m or
   more ahead it took 98 steps or stopped short, and the box counted at 412-415, past 400 (L8,
   L9, V4). A hand left hanging low goes into the box on the next move (L3).
3. `robot.body(0.54, pitch=25)`: at 0.60 m and 20 degrees the grip point is out of reach
   (scripted).
4. Top claw: palm down (`normal=(0, 0, -1)`), the fingers pointing away from the robot across the
   top's short side (`finger=S`; S = the horizontal direction across the top, square to grip_yaw,
   the way that points away from the robot). The palm point 2.5 cm above the top, 3.6 cm from the
   top's centre toward the robot and 2 cm to its right: the open thumb then hangs behind the near
   face. Path: `via=[above_back, back]`, where back = the grip point 14 cm toward the robot and
   above_back = back 8 cm higher, `seconds=0.5` (scripted 20/20).
5. `robot.fingers('right', True)`: holding (scripted 20/20).
6. `robot.body(0.76, pitch=0)`, then `robot.wait(25)` (scripted 20/20).

## Verified parameters
- The box tips 19-21 degrees in the hand as it rises; it still counted (scripted 20/20).
- Palm point 1.6 cm above the top instead of 2.5: one box in 18 was knocked over on the way in
  (scripted).
- An end grip as in xmove_pick also worked (L3, done at 386; scripted 5/6): `body(0.58,
  pitch=30)`, `'wide'`, the right palm `facing='inward'` via a point up and to the right, 6 cm
  right of the grip point, then in: 1 cm outside the right end, 2 cm nearer than the centre,
  4.4 cm below the top; close, `by=(0, 0, 0.12)`. Do step 2 first: the scripted miss was the
  hanging right hand pushing a near box during the lean (scripted).

## Common failures and fixes
- Palm down over the top's centre: 0/6 runs (L3 twice, L4 twice, L5, L6). The thumb, out 10 cm
  along the palm's normal, lands on the top: it closed on nothing, and the hand coming down or
  the closing fingers knocked the box over, sooner or later in all 6. Lying down it was never
  picked up again (6/6 runs). Use step 4 (or the end grip), and look before closing again.

## Final checks
- None needed: the episode ends by itself on success (scripted).

## Typical step costs
- Hands clear about 100, down to 0.54 m and 25 degrees about 80, the claw path about 165, close
  25, standing up about 20 until it counted: done at 350-390 of 400 (scripted). Looking and
  measuring cost nothing; there is room for about one extra motion (30-100 steps).
- Through `robot` the same flow was done at 369-415 (8 runs), past 400 only with the right hand
  over the box (3 runs; step 2). Nothing else fits: keep the whole flow in one cell.
