# take_the_scissors
## Judging
- Lift them with the hand the task names and keep holding; at once the other hand puts its thumb
  through one ring and its index through the other and opens and closes the blades.
## Flow
1. s = measure('scissors'); keep s.top. Axis = grip_yaw; m = find('head', 'scissors')[0]; the
   blade end is the end third of m (along the axis) with fewer pixels. P = s.center moved 3 cm
   toward the blade tips. (probe L0-L15; P 0.6-2.4 cm from the pivot; blade end 26/26)
2. First hand, tweezer setting (middle, ring, pinky = 1): thumb target T and index target I 2.9 cm
   either side of the axis at P, tips at s.top - 3.7 cm (under the pivot); pose by the rule.
   hand(arm, 'flat'); dry-run 8 cm above and the pose, T and I both ways, else the steep tweezer
   from 5 cm above (probe L15; 7/7); go there by straight moves; open fingers. (probe L0-L15)
3. goto(pose, contact=True); closed fingers; goto(arm, by=(0, 0, 0.05)), no wait; the head picture
   shows the blade end up on the two fingers (probe L0-L15; 25/25), else scoop again (V11 r3).
4. Rings, first head picture after the lift: 'finger hole' outlines inside m, else holes of m:
   areas of ~m.pixels m encloses (cv2.connectedComponentsWithStats, 100 px or more), centroids
   (probe L0, L10: within 0.5 cm); a ring with no hole there (open, under a finger): its centre by
   eye in look('head', crop=..., zoom=4) (probe L5, L15; 2/2). Rings' height R (untested): median
   to_world without z on 4-6 rim pixels (measure now reads the raised blades); to_world(..., z=R).
5. At once, the other hand: narrow-ring setting if the centres are under 6.6 cm apart, else wide;
   thumb target one ring, index the other; tips at R - 1.7 cm; moves as in 2 (probe L0-L15; 16/18).
6. Down with contact=True; wait(15) three times; fingers 0.5, 1, 0.5, 0 of the way from thread to
   spread, wait(15) after each; then by=(0, 0, -0.01), contact=True, waits, cycle, twice; a 4th;
   up 1.5 cm, a 5th (probe: opened L5 4/4, L15 wide 2/3, L0 1/3, L10 0/3; L15 r3; V16 r3 1/2).
## Verified parameters
- Rule, right hand: h = heading I to T; yaw = h + dyaw; grasp point = middle of T, I + along on
  h + left to its left, z tips + up. Left: yaw = h - dyaw, roll, left negated. (probe L0-L15)
- Tweezer: pitch 25, roll -102, dyaw 168.5, along 5.1, left 1.6, up 8.9 cm; open thumb [79,-4,-8,-2]
  index [27,15,40,17]; closed thumb [72,2,-6,16] index [37,15,63,-9]. (probe L0-L10)
- Steep tweezer: pitch 32, roll -111.5, dyaw -173, along 5.5, left 4.0, up 8.6 cm; open thumb
  [52,12,0,25] index [1,8,70,-13]; closed thumb [55,18,5,13] index [3,21,80,6]. (probe L15)
- Narrow rings: pitch 25, roll -110, dyaw -147, along 3.0, left 5.3, up 8.7 cm; thread thumb
  [63,9,-8,-3] index [45,6,-12,68]; spread thumb [37,-7,2,-6] index [42,-13,15,5]. (probe L0-L15)
- Wide rings: pitch 34, roll -115, dyaw -161.5, along 4.0, left 4.9, up 8.8 cm; thread thumb
  [47,10,-14,41] index [3,2,44,34]; spread thumb [46,-7,-20,2] index [30,-15,4,-9]. (probe L10, L15)
## Common failures and fixes
- path='swing' to the tweezer's 8 cm point swept the scissors 6-7 cm (V11 r3; 2/2; V6 r3). Flow 2.
- One hole mirrored across the axis read rings 7.2 cm apart as 5.8-6.0 (probe L15), by eye 6.45
  (V6 r3): narrow, never opened (0/4); wide there opened (probe L15 2/3; L15 r3). Flow 4.
- Both rings by eye, one 0.8 cm off across the line between them: the hand turned 7 degrees and
  nothing threaded (probe L5; 0/2); a hidden hole's visible middle: never opened (probe L15).
- Past cycle 5, wider spreads, moved hands or a new scoop never opened them (L0-L10, V6, V16 r3).
- Rings 84-89 cm ahead: the thumb stood on a ring at every depth (V1 r3; threaded 1/2, late).
## Final checks
- Threaded: the wrist picture shows a finger standing inside each ring (probe L0, L10).
