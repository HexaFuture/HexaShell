# phone_packaging
## Judging
- Six stages in order, all with the right arm: phone up, phone into the base, box up, box onto the
  phone, lid up, lid onto the base. Picks counted after 13-15 cm lifts (L0-L15, V).
- A part counts only flat and square inside the base, the lid fully down over it; each has only a
  few mm to spare. A lift counts only after the stage before it has: a box laid on a phone that
  had not counted pressed the phone in, but the box's own lift then never counted (V6 ns).
## Flow
1. Measure all four from head pixels (words found the phone, none of the box parts: 0/4); the lid
   with `find('head', box=...)` around all of it. Measure the empty base's top now (L0-L15, V).
2. Phone: 15 cm above it, `pitch=90`, `yaw=grip_yaw`; pass the yaw the result gives. Measure it
   again from the wrist; grasp point 1.7-1.8 cm above that top; close; lift 14 cm (L0-L15, V).
3. Into the base: over its centre, `yaw` = base grip_yaw + 180; lower the grasp point to 2.5 cm
   above the base's top (it may stop 1-2 cm short as the phone lands); open; lift 13 cm (L5,
   L10, L15, V; 9/12 counted at once).
4. Box: the same way, grasp point 1.5 cm above its wrist-measured top, lift 15 cm (L0-L15, V).
5. Onto the phone: over the base's centre at the base's yaw, grasp point 3.5-4 cm above the base's
   top (L5, V11 sk, V16; 4/10 counted at once). Before opening, look down with the right wrist: box
   edges parallel to the walls, else turn and shift by what you see (untested).
6. Lid: `pitch=90` low over it was refused 5 times: turn the waist about 20 degrees toward it first
   (L0, L10, V; 7/7 reached), then find it again in the right wrist picture (`find` with a box; it
   may have moved). Grasp at that centre, grasp point 1 cm below its head-measured top (about its
   wrist-measured top); close (8.0 cm); lift 14 cm (L0-L15, V1 ns, V6 ns, V16 ns; 7/7 clean).
7. Onto the base, waist still turned: hand straight over the base's centre at the base's yaw, no
   offset; lower the grasp point to 2.5 cm above the base's top; open; lift (V16 ns; untested on L).
## Verified parameters
- Closes on: phone 5.9-6.0 cm, box 6.6 cm, lid 8.0 cm (L0-L15, V).
- Releases that counted at once, above the base's top: phone 2.5 cm (L5-L15, V), box 3.5-4 cm
  (L5, V), lid 2.4 cm (V16 ns; untested on L).
## Common failures and fixes
- Grasping the phone or the box again inside the base: `Dropped` 8/8, closes of 2.9-7.6 cm (V1 sk
  t13-t23, V16 sk t9-t13; untested on L). Do not.
- A box left high: a lid lowered over the base pressed it in once (V1 ns t13), not in V6 sk (t22);
  the lid then had to be lifted off the base again for its own lift to count (V1 ns t18; untested
  on L). Untested: press the box's high end down with the closed hand, then look.
- Lid grasp points 2.3-2.5 cm below its head top (to_world corner points minus 1 cm) were pushed
  1.5-2 cm sideways; one closed on 5.5 cm and slipped, the lid fell on its side (V11 sk t16-t21;
  V16 sk t22; untested on L).
- A lid placed by a grasp offset landed 1.4 cm off, on the rim, 3.6 cm high (V16 sk t24-t26;
  untested on L). A lid on the rim stayed high after closed-hand presses (L5 t14, t19; L15 t16).
## Final checks
- After each placement look straight down with the right wrist: the whole outline inside the
  walls, a dark gap on every side (L0, L10, L15, V). Never grasp a part inside the base again.
- Lid down: its top is back near its height on the table; on the rim it was 4 cm higher (L5,
  L10, L15, V16 sk).
