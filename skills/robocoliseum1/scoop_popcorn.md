# scoop_popcorn

## Judging
- A picture-based judge looks at the scene from time to time and rates the whole task at once.
  The episode ends when the judge is satisfied or when the task has taken too long.
- Counted: popcorn inside the bucket and none on the table or on the rim (L0, L5, L15; 3/3). A few
  kernels were enough. It counted after one trip (L0, L5), or only after the fourth though popcorn
  was inside from the second (L15); each time the scoop was away from the bucket, at or on its way
  to the pile.
- Not counted: a well-filled bucket with one kernel on the table beside it and one on its rim
  (L10). Five more pours did not change that and the episode ran out. Do not spill.

## Flow
1. The scoop starts in the right hand: keep that gripper closed all episode (4/4).
2. Measure 'popcorn pile' and 'popcorn bucket'. If a word is NOT FOUND or seen by one wrist
   camera only, measure at a head pixel (L10, L5).
3. Scoop, right arm, `contact=True`, hand level (`pitch` 0-9). The scoop's middle sits about 10 cm
   ahead of the grasp point (L10): aim the grasp point 11-16 cm short of the pile's middle along
   the hand's yaw (4/4). 3-5 cm above the pile's centre height took a few kernels (L0, L5, L15),
   enough to count (L0, L5); down to 2 cm below it took more (L10, L15). Wait; lift 15-19 cm (4/4).
   If NotExecuted, take the "reachable instead" point or a smaller `yaw` (L0, L15).
4. Carry in two legs: straight back toward the robot until 10-17 cm nearer than the bucket's
   centre, then across to it (4/4). The direct move from the pile was BLOCKED (L5, L10 twice,
   L15).
5. Pour by pitch: grasp point 4-10 cm nearer the robot and 5-9 cm right of the bucket's centre,
   12-16 cm above its rim, `yaw` 35-45 so the bowl is over the bucket; tilt to `pitch` 50-65; wait
   (4/4). Return by the same two legs; repeat until popcorn is seen inside.

## Common failures and fixes
- Roll pours (`roll` ±85-160) cost NotExecuted turns (L5, L10) and a `path='swing'` (L10). One
  left a kernel on the rim (L10). A first trip with roll pours put nothing in (L15, small load).
  Pour by pitch alone: pitch pours put popcorn in (L0, L10, L15).
- A heaped scoop dropped a kernel on the table beside the bucket in the carry that followed two
  BLOCKED direct moves (L10). No other carry left popcorn on the table (4/4). Carry in two legs
  from the start; a few kernels per trip were enough (L0, L5).
- `pitch` 75-80 at the pour point was NotExecuted or NOT reachable (L5, L10, L15); 50-65 worked at
  the points in step 5 (4/4).
- No episode tried to clear spilled popcorn; picking it up with the free left hand (untested).

## Final checks
- Look into the bucket with the left_wrist camera (from the head it looked empty in L5) and at the
  table and rim around it with the head camera.
- Do not call finish while the episode runs; make another trip the same way. Three finished runs
  with popcorn inside and the table clear stayed uncounted (V6a, V6b, V11a); runs that kept making
  trips were counted at a later look (L15, L10b, L15a, V16b).
