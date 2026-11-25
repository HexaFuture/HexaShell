# match_and_pick_from_conveyor
## Judging
- In order: a red box held up off the belt; the robot at the nearest shelf, facing it; the box on
  that shelf's third board from the floor. Boxes ride right to left, about 4.4 cm/s.
- Take one of the first red boxes to arrive, not a later one (L0 t23, L10 t24 r2).
## Flow
1. Reply 1, one cell, no look first: t0 = time.time(); left hand open, pitch 0, yaw -20, 0.80 m
   ahead, 0.06 m left, 0.665 m up, contact=True: a fence across the belt; boxes stop at it and
   slide back along it (probe L0-L15; 26/26 in place). It stays there until step 6.
2. Replies show no clock (L0 t5 r3): count time from t0. Pass time with time.sleep in a cell: the
   belt runs, the robot stays put; wait(n) passes only n/60 s (probe L0-L15; 20/20).
3. Reply 2 is ONE cell running steps 3-5 as a loop; do not end it to see pictures before the down.
   First the right hand open, pitch 90, yaw 90 to 0.55 m ahead, 0.20 m right, 0.80 m up, if a dry
   run allows (probe L5; 2/2). Sleep to t0 + 40 s; every 5 s: wait(1); find 'red box' (head, right
   wrist), 'box' (head); measure each. Red: 10 % or more of robot.image(cam)[mask.pixels] with
   R > 1.5 G, B > 0.8 G; skip a size under 1 cm. Look again 3 s later; a red that moved under 3 cm
   lies still (probe L0-L15; first still red 48-138 s in, 23/24; the loop lifted a red 5/8).
4. At once: right hand 9 cm above the point 2 cm under the red's top, pitch 90, yaw 90; down there
   with contact=True; close; lift 8 cm (probe L0-L15; mid-queue downs begun by 62 s held 8/10).
5. Not "holding" about 6 cm: open, rise 6 cm, loop on; keep the left hand in place (probe L0-L15).
6. Left hand up 15 cm, home; right hand 0.45 ahead, 0.22 right, 0.92 m up (probe L5-L15; 16/16).
7. drive(forward=-0.30), drive(turn=90), drive(forward=1.17, left=0.655) twice, then
   drive(forward=0.10): the nearest shelf stands in front (probe L5-L15; 16/16, the last 14/14).
8. body(lean=20, height=1.30); right hand pitch 45, yaw 90 (give both: left out, the place was
   refused, probe L5) to 0.52 m ahead, 0.12 m right, 1.36 m up, then 0.74 m ahead, 1.34 m up; down
   to 1.285 m with contact=True; open; back 12 cm, up 6 cm (probe L5-L15; 12/12).
## Verified parameters
- Belt top 0.60 m, box tops 0.644-0.651 m, boxes about 9 x 6 x 5 cm; still reds 0.52-0.70 m
  ahead, 0.07 m right to 0.07 m left; red masks 11-98 % red pixels, others 0-9 % (probe L0-L15).
- Right hand pitch 90, yaw 90 at box height was refused from about 0.72 m ahead (probe L0-L15).
## Common failures and fixes
- Late downs: looks, measures and moves in separate replies put the first down 90-233 s in, on a
  spot seen a reply before: 12/12 stopped 41-108 mm short (L0-L15, V1-V16 r3). First looks at
  85-115 s found the red already at the near end (probe L5, L10, L15; 3/3). Keep to step 3.
- After a miss, looping on (probe L0-L15; 0/7) and lifting or moving the fence (L0-L15, V1-V16 r3;
  0/11) lifted no red; tilted hands and yaws other than 90 held 0/16 (L5-L15, V1-V16 r3).
- find('red box') found nothing with reds there (L0-L15, V1-V16 r3): test 'box' masks as in step
  3. Pointing at the head picture measured flat tops, the belt, two boxes as one (L5, L15, V11 r3).
- A red at the near end, 0.50-0.56 m ahead beside the left forearm, was taken 4/12 (probe L0 0/5,
  L15 3/3; L0, V1, V6, V16 r3; 1/4): take it anyway; it was the first still red (probe L0; 7/8).
## Final checks
- After the lift: "holding" about 6 cm and the red between the pads in the right wrist picture (a
  brown box gave 5.4 cm, V1 t18 r3); 0.4-3.7 cm was an edge or the left arm (probe L0, L5; V16 r3).
  After the carry, "Dropped" and "closed fully (empty)" came with the box held (probe L5; 2/2):
  check that picture, go on. After opening: the box on the board, hand out (probe L5-L15; 12/12).
