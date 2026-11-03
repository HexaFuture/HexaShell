# CloseToasterOvenDoor

## Judging
- Shut = pushed fully closed; the oven then holds it and the episode ends.
- Hinged at its bottom front edge a few cm above the counter, it starts open, nearly flat, its free
  edge (dark bar) low. Only the frame is solid in depth, not the glass (prior tool).
- Do not grip the edge: grips on it slipped out in 24 of 41, and each of the 3 failures at the
  current stance began so (20 eps). Lift it on closed fingers from below (prior tool, 49/50 and
  10/10). Let go at 60-75 degrees up it stays; at 40-57 it falls (6 eps).

## Flow
1. Free: `robot.to_world` in left and right on the free edge's dark front bar (ends and middle;
   take the nearest, lowest), the counter top beside the door, and the oven's front face 12-36 cm
   above the edge, straight ahead of it (front). The door is ~0.2-0.3 m long (prior tool).
2. ONE drive puts the edge 0.52 m ahead and centred: `robot.drive(forward=edge_x - 0.52,
   left=edge_y)` (prior tool, 59/60). Then `dry_run=True` on the contact pose (step 3) and the
   arc's top (step 4): top refused, `drive(forward=0.04)`; contact refused, -0.04 (untested).
3. `robot.gripper('close')` (empty). Hand at pitch 45, yaw 0, roll 0: 7 cm short of the edge and
   10-14 cm above it; down to counter + 4 cm, then counter + 1.5 cm (speed=0.05, untested); level
   to 1 cm past the edge. A stop 1-4 cm short against the bar, below it, is contact (prior tool).
4. Lift, one cell: P = (front_x - 0.03, edge_y, counter z + 0.038); from the reached point h,
   r = P_x - h_x, a0 = atan2(h_z - P_z, r). Five `robot.goto` legs to P + r*(-cos a, 0, sin a),
   a = a0 + (90 - a0)*i/5, pitch 45, yaw 0, roll 0 (not `robot.arc`: it turns the hand), then
   4 cm level legs to 8 cm past the front. Go on after BLOCKED; NotExecuted: `drive(forward=0.05)`,
   redo the leg (untested). It shut on the last arc leg or the first push (prior tool, 10/10).
5. Door up (60 degrees or more) but open: closed hand at pitch 90, yaw 0, roll 0, lowered 3-5 cm
   on the robot side of the door and 4-7 cm below its top edge; `robot.goto(by=(0.04, 0, 0.005))`
   legs, a look after each (9 eps). Door back down: measure the edge again, redo 3-4 (untested).

## Verified parameters
- Stance: edge 0.528 m ahead, within 1 cm sideways (59 of 60 runs); 0.44-0.47 m ahead or 9-17 cm
  sideways failed the entry. Pivot 3.8 cm above the counter (about the hinges' height) and 3 cm
  short of the front: a pivot past the hinge lets the fingers ride off the free edge (prior tool).
- Arc radius ~0.1-0.25 m; the arc's top was ~0.65-0.8 m ahead of the base, ~0.15-0.3 m above the
  counter (prior tool, 59 runs). At pitch 90, 2-4 pushes of 3.5-4.5 cm shut it (9 eps).

## Common failures and fixes
- Grip on the bar at roll 90: slipped on the first or second lift leg, the door fell flat;
  re-grasps cost 50-140 steps, five ran out of steps (20 eps).
- Full-force stops on the door pull the hand 5 cm back: the edge tore out of the fingers or the
  door fell (5 eps). Door down again: redo steps 3-4 from where the base is (untested).

## Typical step costs
- Horizon 450. Drive, entry, lift and push took 217-351 steps, median ~250 (prior tool, 60 runs);
  push finish 9-20 steps a leg (9 eps); a redone entry and lift ~150 (untested).
