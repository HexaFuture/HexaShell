# put_the_plates_on_the_shelf

## Judging
- In order: a plate lifted by the left hand; the other by the right; the robot at the dish rack;
  each plate in the rack, counted on its own (L0, L15).
- No message says when a stage passes (L0, L15): each plate is to end standing in the rack.

## Flow
1. measure('plate'): two plates on the island ahead, one flat, one deeper; read each one's centre,
   width and top (L0-L15; 4/4).
2. Left hand, deeper plate: pitch 30, yaw 0, roll 90, grip 0.65, grasp point ~1.5 cm outside its
   near rim, 2.5 cm above its top; close; lift 12 cm moving 4 cm back (L0-L15; 4/4 first try).
3. Right hand, flat plate: drive sideways till it lies ahead. pitch 45-60, grasp point ~2.5 cm past
   its far rim, ~3 cm above its top; drag it straight back with contact=True till about a third
   of it sticks out past the island's edge; look (L0 t20-t21, L15 t25; 2/2). Dragged further, it
   tipped over the edge and was lost (L5 t25).
4. Hand up, back up 0.15-0.25 m; pinch the overhang at pitch 30, roll ±90, grip 0.5-0.65, 1-3 cm
   outside its near rim, ~1 cm above its top; close; lift (L0 t31, L15 t33; 2/2).
5. Grasp points ~1.3 m up, then body(lean=5, height=1.30) (L0 t37, L15 t37; 2/2). The rack: a
   small dark wire dish rack; find it (head yaw up to 44, else turn by 90 and look), measure it.
6. Drive to it in legs of at most 1.5 m, around what is in the way, measuring it after each (L0
   t39-t42, L15 t37-t46: round the island's end). Stop with the rack's middle 0.8-0.9 m straight
   ahead (L0 t43, L15 t46; 2/2). Finding it and the legs: untested in this form.
7. Right plate over the rack's middle, lowered till it stops in a slot (L0 t48, L15 t51; 2/2);
   open, raise the hand; check that the plate stands in the rack (untested).
8. Put the left plate in at once the same way, at the same lean (untested).

## Verified parameters
- Plates lowered into the rack: grasp point stopped 4 cm above the rack's top at pitch 30, roll 90
  (L0 t48); 15 cm above at pitch 90, yaw 0, roll 0, the plate hanging under it (L15 t51).

## Common failures and fixes
- On the island the right hand lifted no plate in 16 closes at pitch 30-90, roll 0 or ±90
  (L0-L15; 0/16): drag first (Flow 3).
- The gripper's words were wrong: "Dropped" with the plate pinched by its rim, 'closed fully
  (empty)' while holding (L0 t33, t43, L15 t33, t41): judge by the wrist picture.
- Held plates fell out: a lean from 5 to 30 took the hand 0.38 m down onto the counter (L0 t54);
  a crouched drive (L10 t47); a drive that touched a tall cabinet (L15 t39-t40). Keep them high.
- After a lean to 30 the wire rack read flat or 5-6 cm low (L0 t54, L15 t46): measure it at lean 5.
- A plate on the floor or the counter took 23-49 motions and was never lifted again (L0 t50-t84,
  L10 t40-t62, L15 t56-t85): go on with the other plate.

## Final checks
- Before driving: both plates in their wrist pictures, well above the island (L0, L15).
- At the rack: each plate in a slot in the head picture, both hands raised (untested).
