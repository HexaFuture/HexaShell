# cloth_hanger
## Judging
- Card: the left hand lifts the shirt by its collar and holds it open; the right hand puts the
  hanger in through the collar; both hang the shirt on the rack's rod. The end counts 5 parts.
- L0, L5: no part; the shirt never came up, the hanger went off the table (L0 a2, L5 r2).
  V1, V6: the shirt came up by a strip pinched past the table's near edge: one part (2/2).
## Flow
1. Locate from the head picture: to_world('head', px, z=height) or pair= with one wrist view
   put the hanger's ends within 1-2 cm (L0 a1, a2, L5 r2). Pairs between the two wrists read
   its right end 1.5-2.6 cm too high (L0 a4, L5 a1, a2 r1): check a pair's height.
2. Scene (L0, L5 r1, r2): the shirt flat on the table's left half, collar toward the robot; the
   hanger flat right of it, hook toward the robot, bar on the far side, its points 2-2.5 cm
   above the table top by pinch stops (L0 a2 t3, L5 t8 r2); its right end sticks out past the
   table's right edge, with nothing under it.
3. The shirt first, as the card says, with the left hand: a closed wrap pointing down, pressed
   on the collar edge with contact=True and dragged 12-19 cm toward the robot, left a strip
   hanging past the table's near edge (V1 t20, V6 t10-t11). Then a pinch, level (pitch 0, yaw
   0, roll 90), lowered there to the table top's height or a little below, closed and lifted
   23-25 cm: the shirt came up (V1 t21-t23, V6 t14-t15; 2/2). Keep the left hand still there.
4. Never press on the part past the edge: a pinch pointing down that stopped on the right end
   tipped the hanger over the edge, at once (L5 t8 r2) or at the second press after the first
   turned it (L0 a2 t3, t5 r2): 3/3 presses failed, 2/2 hangers ended on the floor.
5. Take the hanger where the table is under it: right pinch open, pointing down, its pads across
   the bar or the neck by the hook; look from 10 cm above in the wrist picture, go down with
   contact=True to the table top, close, look in the head picture, then lift 15 cm (untested).
6. With the hanger up: lower its left end into the neck opening and push it in under the
   shirt's front toward the left sleeve, as the card says (untested).
## Verified parameters
- A level pinch sent down stopped on its curled fingers 6-8 cm above the shirt (L0, L5 r1;
  13/13) and 4-5 cm above the hanger (V6 t21, t23); at roll 180 it got lower on the shirt but
  closed on nothing (L5 t3-t5, t17-t23 r2).
- Left hand pointing down over the shirt: lifts of 18 cm at yaw 0 and 27 cm at yaw -30 were
  refused; 18-24 cm at yaw -30 and -60 ran (L0 a4 t20-t29, L5 a2 t14 r1).
## Common failures and fixes
- No close lifted the shirt where the table was under it: pinches and wraps pointing down,
  level, at roll 180, a ridge pushed up between two closed wraps (L0, L5, V1 t18-t19; 0/48).
- Closed wraps dragged toward the robot slid the shirt off the table's near edge (L5 t24-t31
  r2): stop each drag while the shirt is on the table, and look after each.
- With the shirt up, a move 16-22 cm forward and 13-14 cm up let it go and it slid off the
  near edge (V1 t25-t26, V6 t18-t20; 2/2). Nothing that fell off came back (L0, L5, V1, V6).
- The hanger was off the table at the first look after a close on it and two more moves, one
  with grip='open' (V6 t24-t26): look right after each close, before any move.
## Final checks
- Hanger: off the table, hanging from the right hand, in the head picture (untested).
- Shirt: off the table under the hand, in the head and wrist pictures (V1 t23-t24, V6 t15).
