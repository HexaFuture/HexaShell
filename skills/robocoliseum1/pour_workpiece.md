# pour_workpiece

## Judging
- Pour the pieces of the open box in front of the right arm into the open box on its left (L15,
  L0a, L5a, L5b, L10b, L15b; 6/6); a back-row bin counts nothing (L5, L10; 0/2).
- Four "Inside" stages, one per piece as far as seen: a piece outside the receiving box kept one
  at 0 (L10a, L15a; 2/2). Success came a few moves after the last piece lay inside (5/5); the
  failures ended while single pieces were being freed or fetched (L0b, L10a, L15a).

## Flow
1. `robot.measure` both large boxes: centre, top (rim height), size (L0, L5, L10, L15; learn2 8/8).
2. Left hand up and out first, 30-40 cm above the rims beyond the receiving box's far end: the
   carry was BLOCKED until then (L15 t9-t11; learn2 8/8, none BLOCKED).
3. Right arm open, `pitch=90`, `yaw=90`, above the middle of the near wall (the long wall facing
   the robot) of the box in front: half its smaller size from its centre toward the robot, or
   `robot.to_world` on that rim (L5b t4-t6, L15b t5-t6); half the long size closed on nothing
   (L10b t4-t13). Lower the grasp point to 2-6 cm below the rim, close, `robot.wait(12)`: Grip
   0.4-1.0 cm (holding) (L0, L5, L15; learn2 19/20). Lift 18-25 cm; on Dropped, close again and
   lift with `contact=True` (L15b t7-t9).
4. Carry at `pitch=90` only to above the receiving box's near end (farther in it was NOT
   reachable, 8/8), else at `pitch=60`. Pour point: `pitch=15`, hand 20-25 cm above the rim, over
   the receiving box's half nearer the box in front, 1-4 cm beyond its near wall line (none went
   past the box: L5b, L10a, L10b, L15a, L15b); 6 cm beyond, one landed on its far rim or in a
   back-row bin (L5a t9, L0b t12).
5. Tip in steps, `robot.wait(12)` to `(20)` after each: `pitch=15`, `-15`, `-50`, then `-80`.
   Ending at -50 left a piece stuck 5/7 (L5a, L10a, L10b, L15a, L15b). Retips at -50 to -70 freed
   it in 1 of 5 (L10b t22), -80 or -85 2/4 (L5a t16-t19, L15b t25; not L0b t12, t18), the flip
   below emptied the box 2/2 (L0 t12-t16, L15a t29-t34). If `dry_run=True` finds -80 NOT reachable
   (L15a t19), raise the hand 34-39 cm above the rim at the near wall line: -85 was (L15b t20-t25).
6. Look (head): the box is empty, no piece on a rim or beside either box (L0a, L5a, L10b, L15a had
   one there). `pitch=0`, `pitch=90` over the box's place, lower to the grasp height, open, rise
   20 cm, `robot.wait(20)` (L15, L5b, L15b: success during this).

## Common failures and fixes
- A piece left in the box fell onto the table on the way back (L0b t19-t20, L10b t19, L15a t8-t10).
  Do not reach into the box in front for one: the hand closed on its wall and lifted its end (L15a
  t22-t25); side-wall pinches found nothing or let the box fall (L10 t51-t62).
- A pinch about 10 cm off the wall's middle let the tipped box shove the receiving box away (L10a
  t15-t18); moving it back took 14 turns (L10a t24-t37), and the last piece never got in.
- Loose piece: `pitch=90` (or `75`), `yaw=0`, 1-2 cm below its top: held 3/6 (L0a t19, L0b t35,
  L10b t27); let go over the receiving box at `pitch=75` (counted 2/2: L0a t21, L10b t29; at 90 it
  lay inside, never counted: L0b t37). On the far rim: closed-hand sweep, `contact=True` (L5a t24).
- Flip: `yaw=180`, `pitch=150` over the receiving box's near half, 22-26 cm above its rim (if NOT
  executed, first `path='swing'` to `yaw=180`, `pitch=90` above the box's place: L15a t26-t29),
  then `170`, `185`, `190`; no further (a swing to `pitch=230` dropped the box: L0 t22).
