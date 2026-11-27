# match_and_pick_from_conveyor
## Judging
- In order: a red box held up off the belt; the robot at the nearest shelf, facing it; the box on
  that shelf's third board from the floor. Take one of the first reds to arrive (L0 t23, L10 t24).
## Flow
1. Reply 1 is one cell: t0 = time.time(); b = measure('conveyor belt'): its top (a box's on_z is
   the belt top too) and far edge (b.center x + its width / 2); find 'box' (head) twice, 2 s apart:
   the y change gives the boxes' direction. Left hand open, pitch 0, yaw -20, contact=True, grasp
   point over the far edge, 6.5 cm above the belt top, 6 cm left of the robot's middle: a fence;
   boxes stop at it and slide back along it (probe L0-L15, boxes riding left; 26/26). The measures
   before it are untested; boxes riding right: the mirror (untested). It stays until step 6.
2. Replies show no clock (L0 t5 r3): count time from t0. Pass time with time.sleep in a cell: the
   belt runs, the robot stays put; wait(n) passes only n/60 s (probe L0-L15; 20/20).
3. Reply 2 is ONE cell running steps 3-5 as a loop; do not end it to see pictures before the down.
   First the right hand open, pitch 90, yaw 90, over the belt's middle, 20 cm right of the robot's
   middle, 20 cm above the belt top, if a dry run allows (probe L5; 2/2). Sleep to t0 + 40 s; every
   5 s: wait(1); find 'red box' (head, right wrist), 'box' (head); measure each. Red: 10 % or more
   of robot.image(cam)[mask.pixels] with R > 1.5 G, B > 0.8 G (reds 11-98 %, others 0-9 %); skip a
   size under 1 cm. Look again 3 s later; one that moved under 3 cm lies still (probe L0-L15).
4. At once: right hand 9 cm above the point 2 cm under the red's top, pitch 90, yaw 90 (refused
   beyond about 0.72 m ahead); down there, contact=True; close; lift 8 cm (probe L0-L15; 8/10).
5. Not "holding" about 6 cm: open, rise 6 cm, loop on; keep the left hand in place (probe L0-L15).
6. Left hand up 15 cm, home; right hand up to 32 cm above the belt top, then in to 0.45 m ahead,
   0.22 m right: a carry pose by the body (probe L5-L15; 16/16).
7. The nearest shelf (untested): drive(forward=-0.30); turn the base by 90 at a time, each time
   find 'shelf' in the head picture (head yaw up to 44) and measure; the goal is the one nearest
   where the red was taken. Face it; drive at it in legs of at most 1.5 m, around what is in the
   way, measuring it after each, until it is straight ahead, its front 0.45-0.55 m from the base.
8. Count its boards from the floor (look, grid); the third: to_world on its top by the front edge.
   body(lean=20, height=that top + 0.05), dry_run first; right hand pitch 45, yaw 90 (give both,
   probe L5) to 10 cm above it outside the front edge, then 20 cm in at 8 cm above; down with
   contact=True to its top + the red's height - 2 cm; open; back 12 cm, up 6 cm (untested).
## Common failures and fixes
- Late downs: looks, measures and moves in separate replies put the first down 90-233 s in, on a
  spot seen a reply before: 12/12 stopped 41-108 mm short (L0-L15, V1-V16 r3). Keep to step 3.
- After a miss, looping on (probe L0-L15; 0/7) and lifting or moving the fence (L0-L15, V1-V16 r3;
  0/11) lifted no red; tilted hands and yaws other than 90 held 0/16 (L5-L15, V1-V16 r3).
- find('red box') found nothing with reds there (L0-L15, V1-V16 r3): test 'box' masks as in step
  3. Pointing at the head picture measured flat tops, the belt, two boxes as one (L5, L15, V11 r3).
- A red at the fence's near end, beside the left forearm, was taken 4/12 (probe L0 0/5, L15 3/3;
  L0, V1, V6, V16 r3; 1/4): take it anyway; it was the first still red (probe L0; 7/8).
## Final checks
- After the lift: "holding" about 6 cm and the red between the pads in the right wrist picture;
  0.4-3.7 cm was an edge or the left arm (probe L0, L5; V16 r3). "Dropped" or "closed fully" after
  the carry came with the box held (probe L5; 2/2): check that picture. After opening: box on board.
