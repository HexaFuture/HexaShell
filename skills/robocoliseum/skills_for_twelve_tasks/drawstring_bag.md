# drawstring_bag
## Judging
- Card: the right hand puts the bag into the bin by its opening; the left puts each can into
  the bag; both hook the cords and tighten the mouth. The end message counts five parts.
- In L0 r2, V1 and V6 both cans went into the empty bin and counted as two of the five parts;
  the bag never came up (L0, L5, V1, V6).
## Flow
1. The start picture shows the bag puffed up; after the first motion it lies flat (L0, L5 r1,
   r2). Aim from a picture taken after a motion.
2. Cans first. Locate each from the head picture: to_world('head', px, z=its top's height, as
   measured) on the middle of its top put both within 3 mm (L0 r2). Heights 4-10 cm off put the
   points 4-6 cm off (L5 t17, L0 t10 r2). The bin's middle the same way, with z=its measured rim
   height (L0 t20 r2).
3. Left hand, wrap set open (opening 1), level, yaw 90, roll 0: above a point about 9 cm in
   front of the can, down to about 6 cm above the table (lower, its fingers met the table),
   forward with contact=True to the can's middle ("nothing stopped" is right here), close,
   lift straight up 30-35 cm above the table (L0 r2 t17-t19, t28, V1 t4, t9, V6 t5, t8; 6/6).
4. A level hand over the bin was refused: swing (path='swing') to pitch 60, yaw 0 above the
   bin's middle, lower the grasp point to about 5 cm above the rim, open: the can fell in
   (L0 r2 t22-t25, t29, V1 t7, t10, V6 t6, t9; 6/6).
5. Rise straight up before swinging back: a low swing back dipped into the bin and stopped
   (L0 t26 r2). Then the other can the same way (L0 t27-t29 r2).
6. Then the bag: no close lifted it (below). Untried: drag its near edge toward the robot with
   a closed wrap until a flap hangs past the table's near edge, then pinch the flap (untested).
## Verified parameters
- The bin's rim and the cans: measure them in the episode (L0, L5 r1-r2).
- Table top: a pinch pointing down stops on it; the point shape's stop read 3-10 cm below it
  or was pushed aside (L0 t3-t4, t8-t9, L5 t2-t7 r2): do not use it.
- The bag lies flat beside the bin, its mouth a slit across its near third, a white cord
  loop at the mouth's far end (L0 r1).
- Lifts keeping a tilted or level grasp at the bag were refused past 11-23 cm above the table;
  pointing down the hand went to 40 cm (L0 a2, a3 r1).
## Common failures and fixes
- No close lifted the bag: pinches and wraps pointing down, tilted, level from the side at
  its near lip, on the white cord loop, and a level wrap into the mouth (L0, L5, V1 t15-t29,
  V6 t12-t29; 0/49).
- A level wrap that missed pushed both cans 3-7 cm (L5 t18 r2), and the next try aimed at the
  old place (L5 t24 r2): locate both cans again after any touch.
- A wrap pointing down on a can lifted it tilted and it slipped out, or pushed it away (L5 t27,
  t31 r2; 0/2): come from the side, level, as in Flow 3.
## Final checks
- After each can: inside the bin and the hand clear of the rim, in the head picture (L0 r2).
- After the bag lift: its whole body off the table, hanging from the hand (untested).
