# OpenCabinet

## Judging
- Every door swung right open, about square to the cabinet front; "doors" = a pair: both. No
  release needed; a door stays where it is left, but a hand or arm passing it swung it back (2
  eps). An upper wall cabinet; the doors open toward the robot about their outer vertical edges.
  Running: a door is short.

## Flow
1. Free: measure each handle by pointing (`robot.measure(('left', (x, y)))`): vertical bars
   0.15-0.22 m tall; a pair's two handles sit side by side at the seam (10 eps). `robot.to_world`
   on each hinge edge. Hinge = (handle x + 2-4 cm, hinge edge y, handle height); doors 0.38-0.57 m
   wide (8 eps). Right hinge: + opens; left: -.
2. Grip from where the base starts: pre-grasp 12 cm in front, pitch 0, yaw 0, roll 0, open
   (31-36 steps); onto the bar's centre (15-19); close (10-11): 1.3-2.9 cm wide (9 eps).
3. `robot.arc(hinge, 30, axis='z')` twice (21-44 steps each): ~60 degrees (9 eps). Dry-run a third:
   reachable once, with the hinge 0.14 m off the base's centreline (90 degrees, done at 159 steps,
   1 ep); refused with the hinge 0.33-0.55 m to the side (6 eps). Then push (step 4).
4. Push the inner face (7 eps); the other door the same, mirrored:
   - open (14 steps); down 0.30-0.32 m FIRST (29-40), then 10 cm out along the door's outward
     normal (14): out first was refused in 3 eps (elbow nearly straight);
   - closed, pitch 20-40 (pitch 0 was NotExecuted, 3 eps), 0.30 m below the handle, on the INNER
     side ~0.4 m in from the door line (2 eps; from 0.21-0.24 m: BLOCKED going up 3 of 4); up to
     3-5 cm below the handle height at pitch 0 (50-80 steps for both moves);
   - yaw along the door toward the hinge (a right door at 60 degrees: -30; a left one: +30);
     move toward the door until short (touching), 0.30-0.38 m from the hinge (22-38 steps, 4 eps);
   - `robot.arc(hinge, 40, axis='z')` (same sign), then 20-35 more, until BLOCKED at its stop.

## Verified parameters
- Push arcs: 40, then 20-35 reached the stop (7 eps, 4-50 steps each). Grip arcs of 20-25 held
  after a slip (3 arcs, 1 ep). A 4.7 cm close held through two arcs but crept 33 mm (1 ep).

## Common failures and fixes
- A 30-degree grip arc slipped off the bar 5 times (3 eps; grips 1.6-4.0 cm, 9-21 steps into the
  arc); the door then lay 5-20 degrees open. Measure the handle again by pointing, grip it there
  (~55 steps) and go on with arcs of 20-25 (1 ep). One ep slipped twice, then its moves below the
  door shut it: ~370 steps before a grip held (1 ep).
- A hinge 0.5+ m to the side: the push stalled short of the stop, BLOCKED at full force with
  the elbow open (2 of 4 eps; one failed near 70 degrees after 250 steps of tries). Before that
  push, `robot.drive(left=...)` 0.20 m toward that hinge: both pushes then reached the stop (1 ep).
- Moves below a half-open door shut it (1 ep), `robot.home()` near one too (2 eps): look; no home.

## Typical step costs
- Horizon 1050; one door 159-371 (3 eps), a pair 594-937 (5 eps): grip 55-60, two arcs 50-70,
  open/down/out 55-60, set-up 60-120 (150-300 if BLOCKED), push 4-50. Failures lost 250-370 (2 eps).
