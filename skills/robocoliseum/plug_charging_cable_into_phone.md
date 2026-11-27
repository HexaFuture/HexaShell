# plug_charging_cable_into_phone
## Judging
- Three stages: the left gripper lifts the phone, the right lifts the plug, the plug goes into the
  phone. With this flow both lifts counted 7/8 (V1-V16, L0, L5, L15); grasps from above on the
  phone in the holder dropped it or were blocked 4/4 (V1-V16 without notes).
- The insertion decides the score: in once, after 6 motions (V1); V6, V11, V16, L0, L5 and L15 ran
  out of steps after 25-33 motions of pushes and re-posing.
## Flow
1. `robot.measure('phone')` found it in the holder (L0-L15, V1-V16; 8/8): read its centre, size and
   `grip_yaw` (along its long axis); see in the head picture which way its screen faces.
2. Phone: level left hand, `pitch=0`, `yaw=grip_yaw`, rolled to the screen (`roll` -43 to -45);
   grasp point 2-3 cm in from the near end at the phone's measured centre height; stop about 6 cm
   short, slide in, close (1.3-1.5 cm, screen and back), lift 14-15 cm (V1-V16, L0, L5, L15; 7/7).
3. One line to left `pitch=0, yaw=-90, roll=0` (37-60 steps): the screen stands upright and the
   free end, where the port is, points to the robot's right (V1-V16; 4/4; untested in L).
4. Plug: it lies on its narrow edge (L0-L15). Right hand pointing down, yaw along its body by the
   half turn (`grip_yaw` ±180); lower until the grasp point is about 1.2 cm above its top; close
   (0.6-0.8 cm); lift 15 cm (V1, V6, V16; L5, L10 t15).
5. Port (untested): zoom the head picture on the phone's free end (to_pixel of its far end); the
   port is the dark slot in the middle of the end face. to_world on the slot, and on the end face
   right beside it for its depth: the port P and the face's line.
6. Right hand to `pitch=90, yaw=90, roll=0` 6-8 cm out from P (V1 t10, V6 t8, V11 t13, V16 t8):
   the tip points at the end, its flat sides face the screen and the back. In the right wrist
   picture to_world on the tip: its offset from the grasp point (it keeps it while the pitch stays).
7. Grasp point = P - that offset, 1-2 cm out from the end face; look with the right wrist: tip and
   slot on one line, else move by= their difference (to_world on both). Push toward the robot's
   left with `contact=True`; keep holding; look (V1: in at t14-t17; untested in this form).
## Common failures and fixes
- Pushes aimed by the left grasp point's height and depth, not at the seen port, met the end face:
  none of about 30 went in (V6 t12-t26, V11 t16-t35, V16 t11-t29; L0, L5, L15). `pitch` 45-80 did
  not help (V6, V11, V16).
- After a miss: back off 1 cm, look with the right wrist, to_world on the tip and the slot, move by
  their difference, push again before any re-posing (a miss 0.5 cm high, 0.9 cm lower went in: V1).
- Crossed plug: the phone lay flat (left `roll=-90`, L0 t8; held across its width from above, L10
  t12) while the plug's flat sides stood upright (L0 t27, L10 t23; 2/2): out.
- Wrong end: L5 pushed at the end beside the left hand (t12-t32); use the free end (L0, L5, V1).
- Re-posing: left `yaw=90` with the phone was NOT reachable (L0 t28-t34, L5 t12, L15 t8); swings
  of 85-158 steps (L0 t22, L10 t19, t27).
- A 0.3 cm close on the phone was on an edge and it fell flat (L10 t6).
- "Dropped" after a 2.7 cm close on the plug, yet it was held and the lift counted (V11 t9-t12);
  "holding" at 7.6 cm was the fingertips on the table (L0 t11): look.
## Final checks
- Before each push: the plug's flat sides face the screen and the back; in the right wrist picture
  its tip and the port's slot on one line, within about 2 mm (untested).
