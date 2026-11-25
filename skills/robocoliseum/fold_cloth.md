# fold_cloth
## Judging
- Card: both hands fold the near side (its hem corner and sleeve) away from the robot, then the
  far side toward it; then the left hand lays the hem end onto the collar end.
- Start (L0, L5, V1, V6): the shirt flat, collar to the right, hem to the left, one sleeve toward
  the robot and one away; the far edge lies at the arms' reach. The end message counts three parts.
- The near side counted in L0, L5 r2 and V6, not in V1; no other part counted in any unit.
## Flow
1. Table top: a pinch pointing down, sent down with contact=True, stops on it (L0 t12, L5 t8
   r2). The point shape's stop read 3-9 cm below it (L0 t4-t5, L5 t3-t4 r2): do not use it.
   Shirt points: to_world('head', px, z=table top) (L0, L5 r2).
2. Near side: both hands wrap, closed (opening 0), pointing down; the left on the near hem
   corner, the right on the near shoulder where the sleeve joins. Down with contact=True: they
   stop on the cloth 3.5-4 cm above the table top (L0 t16, L5 t14 r2, V1 t3, V6 t3).
3. Then the left wrist picture. Where the near side counted, that press folded the corner: a
   crease ran from the fist into the shirt (V6 t3). In V1 the corner lay flat under the fist
   and came back out in the slide (V1 t3-t4). If flat: lift 3 cm, press 2 cm farther in (untested).
4. Slide the left 14-18 cm straight away from the robot with contact=True, then the right about
   10 cm: the near side counted as the right moved (L0 t17, L5 t14 r2, V6 t4; 3/3 with the
   corner folded). Stop there and look.
5. Far side: both fists tilted 25-35, pressed on the far edge with contact=True, dragged 11-14
   cm toward the robot, then 8-9 cm more: the middle heaped, nothing counted after (V1 t9-t10,
   V6 t8-t10; 0/2). Untried: one leg, the other hand pressing the middle flat (untested).
6. Hem onto collar, after both sides: a closed wrap pressed on the hem end and slid along the
   shirt toward the collar end in short legs, looking after each (untested).
## Verified parameters
- Table top (example): 0.718-0.722 from pinch stops (L0, L5 r2); 0.715-0.721 by pairs (L0 r1).
- A closed wrap pointing down stopped on the cloth 3.5-5 cm above the table top and carried
  the cloth when slid (L0 t15-t17, L5 t11-t14 r2, V1 t3-t4, V6 t3-t4; 6/6).
- Pointing down at yaw 0 was refused over the shirt; the yaw the refusal offered ran (L0, L5
  r1; 2/2). Pointing down was refused 6-8 cm past the middle line (L0 t16, L5 t13 r2).
## Common failures and fixes
- A heaped middle never spread out again, and nothing counted after it (L0, L5 r2, V1, V6).
  It came from drags of 20-40 cm: the right hand's after the near side (L0 t17, L5 t14 r2),
  the far side's second leg (V1 t10, V6 t10). Keep each leg short and look after each.
- No close lifted the cloth: pinches and wraps, pointing down, level or tilted; level pinches
  stopped 4-9 cm above the table on their curled fingers (L0, L5 r1, r2; 0/51). Move the cloth
  by pressing and sliding, never by lifting it.
- Pinches tilted 25-40 at the far edge stopped 4-5 cm above the table and moved it at most
  2 cm (L0 t24, L5 t18, t27 r2; 0/3). Pinch drags from the hem toward the collar bunched the
  shirt (L0 t26, L5 t23-t25 r2; 0/2).
## Final checks
- After the press, a crease from the left fist into the shirt (V6 t3); after the near side, the
  middle as wide as at the start, in the head picture (L0, L5 r2, V6 t4).
- At the end: the hem end on the collar end, both hands off the shirt (untested).
