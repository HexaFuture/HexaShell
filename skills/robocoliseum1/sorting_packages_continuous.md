# sorting_packages_continuous

## Judging
- Four rounds in order, any package first: on the scanning table with its waybill (the white sheet
  with the bars) as the top face, then into the blue bin. Waybill on top counted (L5, L0a, L0b, L5b;
  12/12); another face up did not, and then no later package counted (L0, L5, L5a). Success came
  only once the last package was in the bin (L5b: up from t35, success t48; L0b t44). A stage that
  takes too long ends the episode: L0 and L5a ended with none counted.

## Flow
1. Before the first grasp, find each package's waybill face (zoomed head crop; learn2 4/4). A white
   strip on an edge is a side label (L5a t35-t36); on a white package look for the bars (L5a t27).
2. Grasp with `roll=0` (held 16/16; `roll=90` grasps slipped 3/5: L0, L5), so that one turn of
   the hand while holding brings the waybill up:
   - On top: top grasp, `pitch=90`, `yaw=` its `grip_yaw`, 2 cm below its top; the top stays the
     top (L0, L5, L0b t11, t23; 4/4). If NOT executed, `path='swing'` (L0, L5, L0b t5).
   - Toward the robot: hand level (`pitch=0`) pointing at that face 10 cm out, move in to its
     centre, close, lift 15-17 cm, `pitch=90`: the palm face comes up (L0a t5-t6, L5b t4, t16,
     t21; 4/4). If NOT executed, lift higher at `pitch=45`, then `path='swing'` (L5b t21-t26).
   - Facing away: top grasp with `yaw=` pointing from its centre at that face, then `pitch=0`
     while holding: the face ahead of the hand comes up (L0b packages 1, 3, 4; 3/3). Pointing
     elsewhere sent another face up (L5 package 2, L5a t26-t27, t35-t36; 0/3).
   - At a side: hold it level as for a face toward the robot, then roll it: `roll=90` brought the
     left face up (L5b t34-t35); L0b looked after each roll, set it down at 270 (t26-t30).
   - Do not turn it about the vertical (`yaw` while holding): the top face stays; the two packages
     L0a turned so took 20 and 43 turns (L0a t15-t34, t35-t77).
3. Turn right to the scanning table (`robot.waist(yaw=-75)` to `-90`, learn2 4/4); measure it.
4. Set it down at the table's middle (on its printed cross) or just short of it, at `pitch=75`, or
   level after a `pitch=0` turn (L0b t14, t37, t41); open; back off at `pitch=60` (L0a, L0b, L5b).
   Farther was NOT reachable or NOT executed (L0a t8-t9, L0b t8-t10, L5a t9, L5b t7); "short of
   target" there still counted (L5b t8).
5. Look from above (zoomed head crop): waybill on top. If not, grasp it there and use step 2
   (L0b t13-t14, L0a t32, t77); a level approach there once pushed it away (L5a t16-t23).
6. Re-grasp at `pitch=75` (`60` or `45` if refused: L0b t16, t32, t38), lift, turn the waist to
   about `-170`, release 10-15 cm inside the bin's near rim, the hand 1-5 cm above the rim, at
   `pitch=15` or `0`, `yaw=-90` (`pitch=30` to `45` NOT executed: L0a t13, L0b t18); open, rise,
   `robot.waist(yaw=0)`. This binned all four in L0b and L5b (L0b despite drops on the way, t32).
7. Few cells: L0b did grasp, waist turn and set-down in one cell (t37, t41), 6 turns per package.

## Common failures and fixes
- Far package: turn the waist toward it first (`robot.waist(yaw=20)`: L0b t34, t39; 2/2).
- A tall package laid on its side counted (L5b t35), but `pitch=75` and level re-grasps were NOT
  reachable or BLOCKED; after `robot.home`, `pitch=90` from above held (L5b t36-t46, 11 turns).
  Set down tilted (`pitch=29`), one fell over (L5).
- Grasping a package inside the bin: NOT executed, BLOCKED, empty (L0 t61-t64).
