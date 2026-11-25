# rubik_cube_solve
## Judging
- In order: lift the cube off the table held between both hands, never on one fingertip (L0 r2
  t15-t19); turn one outer layer a quarter turn so it sits square again; solve the cube. Set the
  cube down again before turning a layer (Flow 5-6).
## Flow
1. With no motion: pair= on the top's corners (head with a wrist picture), keep those that agree
   (V3 t1-t6: four within 3 mm); the top's middle with z= at that height (V2 t4, V3 t7: 2 mm).
2. Both hands hand(.., 'wrap', 1), pointing down (pitch 90, roll 0), each 21 cm above the top and
   7.5 cm to its side of the middle: first at yaw -30, then turn to yaw 0 in place, both by line
   (probe L0, L1; 19/19 hands; V3 left). Straight to yaw 0 had no line (probe L1; 0/2).
3. Straight down to 3.2 cm above the top (2.8 cm bent a finger on the table, V2 t6); the left in
   to 5.9 cm from the middle. Press in with contact=True, the right then the left, each aimed 3.3
   cm from the middle: they stop 1.4-2.7 cm behind, at the faces (probe L0, L1; 9/9; V3 t15).
4. goto('both', by=(0, 0, 0.06)), wait 25-30 steps: the cube comes up between the open hands,
   both reading empty: judge the hold in the head picture (probe L0, L1; 11/11; V3 t17).
5. Set it down: goto('both', by=(0, 0, -0.048)); ease the left 1.8 cm out, then the right; each
   4.5 cm out with contact=True. It stays square, 1-1.3 cm nearer the robot: measure its middle
   again as in Flow 1 (probe L0, L1; 4/4). Then home('both') (probe L0; 1/1).
6. Last, try the turn: hand('left', 'flat'), palm face down (pitch 0, roll 90) at yaw -45, 4 cm
   above that middle; down with contact=True aimed 1.5 cm below the top (it stops at the top);
   then yaw -30, -15, 0, 15, 30, 45 there, each contact=True, path='line': the top layer went
   round and settled a quarter turn on (probe L0; 1/1 from the start; L1 0/5; after a lift 0/4).
## Verified parameters
- A scrambled cube about 6 cm on a side, alone on a plain table left of the robot's middle (L0-V3).
- A palm pressed on the top took the top layer 29-46 degrees round for a 75-135 degree turn (probe
  L0, L1; 8 runs); once it went on to a quarter turn and settled, else it went back or stuck.
## Common failures and fixes
- One hand alone never held the cube: empty or on a fingertip (L0 r1, r2, L1 r1, r2; 0/6).
- pair= can match two different points on one line of sight, with or without a warning: tops 12
  cm low (L0 r2 t4; V2 t3, rays 4.9 mm) and 8 cm high (L1 r2 t3). Drop heights far from the rest.
- Never let the arms touch, as with the right moving in while the left held the cube (L0 r2 t19),
  the left opening beside the right (L1 r2 t12), the right coming in while the left palm was on
  the top, the palm passing a right hand left 4.5 cm out (probe L0, L1; 4/4). Clear the path.
- Letting one hand go while the other still pressed slid the cube 2.2-2.5 cm; the palm then met
  its edge and broke it apart (probe L0, L1; 2/2). Ease both off first and measure again.
- After the lift, a palm not sent home first had no straight line past yaw 15 (probe L0; 2/2).
- Turning: a palm aimed 2-4 cm below the top broke the cube apart (probe L0, L1; 0/5), 0.8 cm
  turned the whole cube (probe L0; 0/1), a start at yaw -90 took the layer 29 degrees (probe L1;
  0/1), a swing while pressing broke it (probe L1; 0/1); twisting one of two holding hands turned
  the whole cube (L1 r2; probe L0, L1; 0/3); a wrap closed over the top held nothing (probe L1).
- The palm at yaw 45 could not be lifted: it slid 5-7 cm with the cube (probe L0, L1; 4/4).
## Final checks
- Before pressing in: the hands on opposite sides, fingertips below the top (L1 r2 t9, V3 t14);
  lifted: the cube clear of the table (V3 t17); before the palm: both arms home (probe L0).
