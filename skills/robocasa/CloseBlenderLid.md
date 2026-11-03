# CloseBlenderLid
## Judging
- Closed = the lid seated on the jar's mouth, sitting level, the hand moved away from it. The
  stop depth does not tell (14 eps): seated after stops 2.5 cm under to 1 cm over Z (step 2);
  a few mm off the mouth's centre it can rest on the rim, slightly tipped (prior tool).
  Check the tilt (step 6).

## Flow
1. Jar: clear or dark; depth sees through it, `measure` and `find` miss it (prior tool; 3 eps).
   Mouth about 0.3-0.4 m ahead of the base's start, a little to a side, rim 0.3-0.5 m over the
   counter (13 eps). Triangulate the rim's left, right and front edges, `robot.to_world('left',
   (x, y), pair=('right', (x2, y2)))`: C = midway left-right, front x + half that width; height
   z_r (16 eps). Arm hiding the rim in a view: move the hand aside (1 ep). No scipy: numpy (2 eps).
2. Grasp: pitch 90, yaw 0, open, 10 cm over the knob, down to its top - 0.008, close, then
   `robot.goto(by=(0, 0, 0.03))`, read `Grip` (17 eps). Square knob: yaw = `.grip_yaw`; off-centre
   knob: close along the line to the lid's centre (prior tool; 1 ep). Seated hand height Z = z_r +
   hand z at the close - counter z by the lid (not a knob's `on z`: 6 cm low, 1 ep); 2-6 cm (6 eps).
3. Knob wider than the open fingers (they stop on its top; central closes slipped, 11 tries, 2
   eps): pinch 2.3-2.6 cm off its centre, square to the closing line (prior tool, 3 of 4; 2 eps).
4. Carry: lift to Z + 0.04 or more before moving toward C: straight up; if refused, up and toward
   the base, never up toward the jar. Lower moves near the jar hit it (6 eps). Drive so the mouth
   is 0.28-0.38 m ahead, nearer for a high carry (11 eps). Then one level move over C (plus the
   knob's offset from the lid's centre), then only straight down (6 eps). Lid held close in
   drives: driven held far out, it was lost (1 ep).
5. Lower straight down to Z - 0.045 at `speed=0.02` from the carry height: it stops at contact,
   `short of target` or BLOCKED (6 eps); full-speed legs below the carry hit the rim (3 eps).
   1.2 cm+ under Z: `robot.goto(by=(0, 0, 0.01))`, else 0.005; open; rise 4-6 cm, `dry_run=True`
   first (15 eps); if refused, `robot.goto(by=(-0.18, 0, 0.025))` (1 ep).
6. Tilt: `robot.to_world('wrist', ...)` on the lid's edge at front, back, left, right; height change
   over distance above 0.1 = tilted (prior tool; 3 eps). Level: `robot.goto(by=(-0.20, 0, 0))`.
7. Tilted (at most twice, ~140 steps each): grasp as in 2, lift 3 cm at `speed=0.03`, move 6 mm
   toward the low side (12 mm for 16 degrees+), lower as in 5, up 0.005, open, check (prior tool:
   5 of 9; 4 eps). Tipped 20 degrees+: grasp along the lid's axis, from a 3-point wrist fit (1 ep).
## Verified parameters
- Knob closes of 3.7-7.6 cm at pitch 90 held first lifts at full speed (17 eps).

## Common failures and fixes
- Re-grasps on the jar: 3.4-4.7 cm closes or pitch 15-30 slipped on full-speed lifts (8 tries, 3
  eps), 5.1-7.4 cm at pitch 90 held (3 eps): lift as in 7. Pressing it down seated 2 of 16 (7 eps).
- Gap growing while lowering = caught on the rim; opened, it tipped 40 degrees, then 7 closed it
  (1 ep). `robot.home()` near the jar: BLOCKED or threw the lid down (4 eps); use `robot.goto`.

## Typical step costs
- Successes 286-564 of 900 steps, 9-33 of 40 replies (6 eps); two failures ran out of replies
  (2 eps): one stage per cell. Slow lowering costs 1 step per mm plus ~30 (3 eps).
