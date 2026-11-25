# insert_straw_into_milk_tea_cup
## Judging
- Two stages: the named hand lifts the straw (counted at the first look after the lift, 16/16);
  the straw goes through the lid's hole into the cup (counted during the press or the wait after).
- A press beside the hole meets the lid, stops 1-5 cm short and can push the cup (12 cm over five
  presses in L5). In at the first press 2/12 (L0, V6); each miss cost about 3 motions.
## Flow
1. Point at the straw and the cup in the head picture (words find neither). Measure the lid at a
   lid pixel. The hole is the black dot 1-1.4 cm to the robot's right of the lid's measured centre
   and 0.3-0.8 cm farther out (L0-L15, V1-V16; 8/8). Zoom the head picture on the lid, take the
   dot with `to_world('head', px, z=<lid top>)` and check it against that offset (V11 took a
   point 1.4 cm off it and met the lid five times: t7-t21).
2. Hand pointing down, open, about 10 cm above the straw's middle, fingers across it; asked for yaw
   0 it may arrive at 180: keep it. Lower until the grasp point is 2.5-2.7 cm above the straw's
   top; close (1.2-1.4 cm); lift straight up 22-24 cm (L0-L15, V1-V16; 16/16).
3. One line to a level hand, `pitch=0, roll=0`: left hand `yaw=-90`, grasp point about 5 cm to the
   robot's left of the hole; right hand `yaw=90`, 5 cm to its right; 16 cm above the lid (left:
   L0, V1 t5, V11 t6, V11 ns t10, 36-44 steps; right: V6 t7, V16 t6, 53 steps).
4. The straw's lower end hangs 3.5-4.2 cm ahead of and 6.7-7.5 cm below the grasp point (L0, L15,
   V1-V16). Check in the other hand's wrist picture that it hangs straight (two points 5 cm apart
   agree), then move its lower end about 1 cm above the hole (L0, L15, V6 t10).
5. One `contact=True` descent of about 4.5 cm; `robot.wait(30)`; a second wait before any lift
   (L0 t9-t10, V6 t11-t13).
6. Stopped short: up 2 cm; zoom the head picture on the hole and move toward the part of the hole
   left uncovered by about its width; take the hole again from the head picture (the cup moves);
   press (V1 t10-t23, V16 t10-t22, V11 ns t15-t23: in at the third or fourth press; L15 0/5).
## Verified parameters
- A 1.1 cm close slipped out on the lift (L5, L15 earlier runs): reopen, 5 mm lower, close.
- Presses that went in reached their target or stopped 1 cm short (L0, V1 t23, V6 t11, V6 ns t17);
  every press that stopped 2 cm or more short was on the lid (V1-V16, L5-L15).
## Common failures and fixes
- Holding the straw level at the pick yaw (38°) or at -45 to -60 instead of ∓90: the hand hid the
  lid and the straw hung off vertical; 2 of 5 went in, after 3-4 presses (V6 ns, V16 ns; V1 ns 0/7,
  L5, L10).
- `yaw=-90` had no straight line from the lift pose (L5, L10); L15 went by `roll=180`, then
  `roll=0` (194 steps). In V1, V11 and V11 ns the one line worked (36-38 steps).
- Hole readings from a wrist camera on the lid plane (3.9 mm per pixel) sent the presses 1.5 cm
  the wrong way (V11 t15-t21); a wrist reading of the straw's end within 1 mm of the hole still met
  the lid five times (L15 t10-t20): after a miss, correct by the head picture.
- `pitch=-7` to move the end along the heading: in once (L10 earlier run), 0/3 (L5, L10, L15).
- A grasp or lift with the hand tilted left the straw about 30° off vertical (L5, L15 earlier);
  `path='swing'` near the cup knocked it over (L5 earlier run).
## Final checks
- Before the press: the two wrist points 5 cm apart agree within 3 mm (L0, L15), and the straw's
  lower end is over the black dot in the zoomed head picture (untested).
