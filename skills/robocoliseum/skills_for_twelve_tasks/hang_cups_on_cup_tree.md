# hang_cups_on_cup_tree
## Judging
- Card: the named gripper (the arm on the cup's side) lifts the cup by its handle, which then
  hangs on a branch of the wooden cup tree, the gripper open and away, the cup still.
- 'cup tree' is not found by words. Ends sit at two or three heights, each branch rising to it.
## Flow
1. Measure 'cup handle' and 'cup' ('mug handle', 'mug' on arx) from the head. The line from the
   cup's centre to the handle's centre was 0-5 degrees off the handle 26/29; grip_yaw 0-9 off,
   on g2op 8-37 (probe L0-L15). Yaw source, pitch, depth: notes for this task on this robot.
2. Measure again 10 cm above the handle and use it; down to the depth, close, lift 8-15 cm (L0-L15).
3. Look with the holding wrist: the cup must hang upright. A tilted cup or one on a finger hung
   1 of 9 times (agilex V1, V6, V16, arx L15, V1, V11): open, grasp again (agilex L10, g1 V11).
4. Branches: to_world on head pixels at the top of each end and 2-3.5 cm in; heading = inner to
   end (5 mm 21/28, 5 degrees 24/28); a hidden end read 22-37 cm off: compare heights (probe L0).
5. Pose by dry runs, yaw = heading + 90 or - 90: for each pitch in the notes for this task on this
   robot and each branch (low ones last: 0/4, agilex V6, V16 r4; probe L15), dry-run 3 cm above the
   approach point, it and the inside point; take the first where all go; else skew 15, then 30.
6. Turn in a move of its own at the lift point (the cup stayed 19/19), then one line to 3 cm
   above the approach point, all angles fixed from here (probe L0-L15).
7. Approach point 2.5 cm beyond the end on its line, 2.3 cm above the end's measured top; wait(60)
   there; along the branch to 2.5 cm inside the end, same height; lower 1.8 cm with contact=True;
   wait(30). Without the waits g2op hung 0/4, with one or both 6/10 (probe L5, L10; L0-L15).
8. Final checks below; open in a cell of its own and look; back out 4.5 cm along the branch and
   up 3 cm with grip=1, home that arm, wait(120): hung cups stayed (probe L0-L15; all 10 r4 hangs).
## Verified parameters
- The branch passes under the fingers, which close along it, and through the opening (probe L0-L15).
- At the open of the hangs: 1.7-3.0 cm in; the lowering had stopped 1.2-1.6 cm above the end's
  top (g1 V6, V11, V16 r4), on the L0/L10 cup 1.4-2.3 (L0, L10 r4; probe L0).
## Common failures and fixes
- The lowering went its full way: nothing under the handle; the open dropped the cup (agilex L5,
  arx L10; probe L0; 3/3). Do not open: back to the approach point, look, redo.
- The move inside stopped short (compare the arrival with the inside point): the handle met the
  end (g1 L5, V11, arx V16, agilex V1 t8; probe L5). Back out; second entries, mostly higher, never
  hung (g1 L15, arx V1, V6, V11, V16, g2op V11, V16 r4; 0/8): take the next pose (untested).
- 'Dropped' at the branch: the cup was on it 2 of 7 (arx L10, g1 V6 r4), fell 5 (g1 L10, L15, g2op
  V1, agilex V6 r4; probe L5): look with the holding wrist before anything else.
- Lost on the way: moves at pitch 60 (probe L0; 3/4); agilex carries, also in 3 cm steps with
  waits (L0, L10, L15, V1, V16 r4; probe L0, L10); the other arm homed through the tree (arx L5).
- Shifts by the holding wrist's measure put g2op 2.8 cm and arx 0.9-1.8 cm off the branch line
  (V1, V6, V11; L0-L15, V1-V16 r4). A fallen cup was never hung again (L0-L15, V1-V16; 0/34).
## Final checks
- Before opening: the lowering stopped and the holding wrist shows the branch under the fingers in
  the handle's opening. Stops 1.8-2.6 cm above the end's top hung 4/16, 3 on the L0/L10 cup (L0,
  L10, V6, V16, L5 r4; probe L5, L15): if it is not plainly inside, back out, next pose (untested).
