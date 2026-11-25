# sorting_packages

## Judging
- Stages in order: Follow (each episode used only the right arm; 6/6), PickUpOnGripper, Inside (on
  the scanning table), Upright (barcode label up), PickUpOnGripper, Inside (in the blue bin).
- On the table it counted on its near part, short of the printed cross (L0a, L0b, L5a, L5b; 4/4).
  Label up counted only with the whole top face the white waybill (L5, L0b, L5a, L5b; 4/4); with
  it on a side face nothing after it counted, though it went into the bin twice (L0, L0a; 0/2),
  and L0a was ended at t49 with that stage open. Success came during or after the closing home.

## Flow
1. Measure the package; find its waybill face in a zoomed head crop (on top: L5, L5a, L5b; on a
   side face: L0, L0a, L0b).
2. Grasp from above: `pitch=90`, `yaw=` its `grip_yaw` (label on a side: step 3), `roll=0`, about
   2 cm below the top; `wait(12)`; lift about 15 cm (13/13 `roll=0` grasps held). If NOT reachable,
   use `path='swing'` (L5a t3, L5b t5). If no yaw is reachable, turn the waist 10-20° and dry-run
   again: left made it reachable, right worse (L0a t5-t13, L0b t5-t7; other places untested). A
   swing said to go 12 cm below its lower end was BLOCKED: go by a higher point (L0a t6-t13).
3. Label on a side face: grasp with the hand heading toward it, so that in the wrist picture after
   the lift the waybill fills the face beyond the package's far edge (L0b t11, picture 11). At the
   table, `goto(..., pitch=0)` keeping the hand's yaw: that face comes on top (L0b t14-t17; 1/1).
   A narrow white strip on that face is the edge of a label on the next one: it stayed on its side
   (L0a t14-t25; again t45; 0/2): set it down, grasp again heading toward the waybill (untested).
   Label toward the robot: level hand, palm on it, close, lift, `pitch=90`: it comes on top
   (continuous task L5 t13, L5b t16; untested here).
4. `robot.waist(yaw=-65)` (dry runs over it at -85 to -90 refused: L5); measure the table at a
   pixel on its dark top (a higher one gave the scanner post: L0a t16); `pitch=90` reached its near
   part (L5a, L5b). Lower until the package meets the table ("short of target" there was fine),
   open, rise (L5a, L5b; level hand: L0b t16).
5. Look from above: the whole top face must be the white waybill with its bars (L5 picture 10,
   L0b picture 17); a white strip along one edge is a side label (L0a).
6. Grasp again (`pitch=90`, or `pitch=75` where 90 is NOT reachable: L0b t18-t20), lift 15 cm,
   `robot.waist(yaw=-140)` (-125 went deeper: L5a t26-t31; at -160 `pitch=30` NOT reachable: L0a).
7. Measure the bin. Release at `pitch=30, yaw=-90` (`pitch=45` NOT reachable: L0b t22, L5a t13,
   L5b t14), as far in and as deep as the dry run allows; open, rise (L5, L0b, L5b). L5a's package
   came to rest by the near wall; it counted when released again further in and deeper (t15, t31).
8. `robot.home('both')`, `robot.wait(20)`; if success has not come, look into the bin (L5a).

## Common failures and fixes
- Tilted, rolled grasps (pitch 30-60, `roll=90`) turned or slipped (L0 t8-t14; continuous task 3/5).
- Label taken as up from an edge strip (L0a t25-t26) or with no look from above (L0): do step 5.
- Bin descent BLOCKED below the rim: opening there still dropped it in (continuous task L0 t41-t42).

## Final checks
- Waybill seen on top from above before the second grasp; hand open and raised in the bin; home.
