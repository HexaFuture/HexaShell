# plug_charging_cable_into_phone
## Judging
- Three stages: the left gripper lifts the phone, the right lifts the plug, the plug goes into the
  phone. With this flow both lifts counted 7/8 (V1-V16, L0, L5, L15); grasps from above on the
  phone in the holder dropped it or were blocked 4/4 (V1-V16 without notes).
- The insertion decides the score: in once, after 6 motions (V1); V6, V11, V16, L0, L5 and L15 ran
  out of steps after 25-33 motions of pushes and re-posing.
## Flow
1. `robot.measure('phone')` found it in the holder (L0-L15, V1-V16; 8/8): about 15 cm long,
   `grip_yaw` along its long axis, on a long side, leaning back, screen up and to the robot's right.
2. Phone: level left hand, `pitch=0`, `yaw=grip_yaw`, rolled to the screen (`roll` -43 to -45);
   grasp point 2-3 cm in from the near end at the phone's measured centre height; stop about 6 cm
   short, slide in, close (1.3-1.5 cm, screen and back), lift 14-15 cm (V1-V16, L0, L5, L15; 7/7).
3. One line to left `pitch=0, yaw=-90, roll=0` (37-60 steps): the screen stands upright and the
   free end, where the slot is, lies about 13 cm to the robot's right of the grasp point (V1-V16;
   4/4; untested in L).
4. Plug: it lies on its narrow edge (L0-L15). Right hand pointing down, yaw along its body by the
   half turn (`grip_yaw` ±180); lower until the grasp point is about 1.2 cm above its top; close
   (0.6-0.8 cm); lift 15 cm (V1, V6, V16; L5, L10 t15).
5. Right hand to `pitch=90, yaw=90, roll=0` beside the free end (V1 t10, V6 t8, V11 t13, V16 t8):
   the tip points at the end, its flat sides face the screen and the back. In the right wrist
   picture the tip lay 1.5-2.4 cm ahead of and 2.3-3.4 cm below the grasp point (V1 t11, V6 t9,
   V16 t10; untested in L).
6. The slot is at the left grasp point's depth (the pads centre the phone) and close to its
   height. Put the tip there, 1-2 cm out from the end face; push toward the robot's left with
   `contact=True`; keep holding; look. V1: a push with the tip about 0.5 cm above that height met
   the end face; 0.9 cm lower it went in, the right grasp point 2.5 cm above the left one and
   within 2 mm of its depth (t14-t17; untested in L).
## Common failures and fixes
- Pushes with the tip up to 2 cm above the left grasp point's height (right grasp point 2.6-4.6
  cm above the left one), or 3-6 mm off its depth, met the end face: none of about 30 went in (V6
  t12-t26, V11 t16-t35, V16 t11-t29; L0, L5, L15). `pitch` 45-80 did not help (V6, V11, V16).
- After a miss: back off 1 cm, lower the tip 0.5-1 cm and push again before moving sideways (V1
  t16: in; untested in L).
- Crossed plug: the phone lay flat (left `roll=-90`, L0 t8; held across its width from above, L10
  t12) while the plug's flat sides stood upright (L0 t27, L10 t23; 2/2): out.
- Wrong end: L5 pushed at the end beside the left hand (t12-t32); use the free end (L0, L5, V1).
- Re-posing: left `yaw=90` with the phone was NOT reachable (L0 t28-t34, L5 t12, L15 t8); swings
  of 85-158 steps (L0 t22, L10 t19, t27).
- A 0.3 cm close on the phone was on an edge and it fell flat (L10 t6).
- "Dropped" after a 2.7 cm close on the plug, yet it was held and the lift counted (V11 t9-t12);
  "holding" at 7.6 cm was the fingertips on the table (L0 t11): look.
## Final checks
- Before each push: the plug's flat sides face the screen and the back, its tip on the phone's
  long axis at the left grasp point's height (V1; crossed and out: L0 t27, L10 t23).
