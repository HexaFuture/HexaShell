# insert_coin_into_money_box
## Judging
- Stages in order: the named hand picks the upright coin out of the coin tray; the coin goes into
  the money box through its slot. The first close decided the episode: held, the coin reached the
  box (L0, L5, V11, V16); slipped out, it fell over or leaned and was never carried again (L10,
  L15, V1, V6, and all four runs without notes). An open costs about 40 steps, a close 10-16.
## Flow
1. Point at the coin (words found nothing). It stands on edge in a cell of the tray, its top about
   1 cm above the cell walls. Take its centre with a wrist camera in view: centres from the head
   alone were 6-9 mm off (V1, V6; untested in L). The box is the blue cylinder; the slot is the
   dark line on its top: note its heading (L0-L15, V11, V16).
2. One move to about 10 cm above the coin: `pitch=90`, `grip=0.2`, yaw = `thing.grip_yaw` ±180
   (the same closing line; `grip_yaw` itself was refused: L0-L15).
3. Lower toward a grasp point 1 cm above the coin's top; close. At `grip=0.2` the fingers went
   down between the cell walls to the target (V16 t7, V16 ns t11; untested on a first close); at
   0.35 they stopped about 1 cm short and closed on the coin's top edge (L0-L15, V1, V6, V16).
4. Lift 12-15 cm only after a close of 0.4 cm or less, or "closed fully" with the coin between the
   pads in the wrist picture: held 4/4 (L0, V11, V16, V16 ns). Closes of 0.8 cm or more slipped
   out on the lift (L10, L15, V1, V6, V16, runs without notes; 0/14).
5. Carry pointing down, heading along the slot (yaw 180 for a slot along x: L0; -90 along y: L5;
   V11 right hand yaw 198 for a slot at 18°), about 10 cm above the box top over the slot. In the
   holding hand's wrist picture take the slot's two ends with `to_world('<hand>_wrist', px,
   z=<box top>)`; move over their middle (L0, L5, V11).
6. Lower until the coin's lower edge is about 1 cm above the box top (V11: grasp point 5.7 cm
   above it; L5: the coin hung about 6 cm below the grasp point); open; lift 10 cm; wait. In
   3/3 when the coin was seen over the slot (L0 t15, L5 t18, V11 t12).
## Verified parameters
- With the box farther out, `pitch=90` reached about 9 cm above its top, not 13 (L0 t8-t9).
- Left hand to the farther box: a direct move was refused, a waist turn did not help, a swing
  ended 11 cm short; first a point 10 cm nearer at the same angles, then a line, got there (V16
  t9-t16; untested in L).
## Common failures and fixes
- Fully open fingers closed on the cell walls, about 5 cm, and lifted the tray; the coin then fell
  over or leaned and was never carried (V1 ns t4, V6 ns t3, V11 ns t4; L0-L15 earlier runs).
- After a slip every later grasp failed: descents stopped short, closed wide or knocked it flat;
  tilted pitches were blocked; opening and closing 2-3 cm higher knocked it flat (L0-L15, V1, V6).
- "Dropped" and "closed fully (empty)" were reported with the coin held (L5 t10, V11 t6, V16 t7),
  "holding" with nothing between the pads (L15 t26): judge by the wrist picture.
- Released with the coin hidden behind the fingers, it landed on the lid about 4 cm off the slot
  (V16 t17-t21; untested in L). One run opened the hand on the held coin over the tray (V16 ns t13).
- A measure at a pixel on the flat coin returned the tray, about 23 x 11 cm (L15 t30).
## Final checks
- After the lift: the coin under the fingers in the wrist picture (L0 t7, L5 t11, V11 t7).
- Over the slot: the coin's edge along the slot's dark line in the wrist picture (L0, L5, V11);
  if the fingers hide it, zoom the head picture on the box before opening (untested).
