# pick_up_a_package
## Judging
- Stage 1 passed in all eight episodes once the palms raised the box, even tipped (L0, L10).
- Stage 2 is judged when the box first touches the table again, and the episode ends there (the
  message does not say so). Credit grew with the sideways distance from where it was lifted:
  25 cm (L5 r1), 31 cm (L15), 33 cm (L5 r2: just short of full), 49 cm (L10 r2: full, success).
- A box left tilted on an edge after the lift counts as set down: no carry (L0 both rounds, L10 r1).
## Flow
1. measure the box before touching it: centre, size, grip_yaw. (L0-L15 r2; 4/4)
2. Dry-run both flat hands at yaw = grip_yaw, pitch 0, roll 0, each 2-3 cm outside its side face,
   over the box's middle front to back, 2-3 cm above its measured centre. Box far to one side:
   turn the waist 20 degrees toward it, then give both hands one pitch, the first of 0, 15 and 25
   that both reach. (L10 r2: pitch 25, 2 cm nearer the robot; L5, L15 r2 at pitch 0; 3/3 level)
3. Press each hand 1-2 cm inside its face with contact=True. The first press pushed the box and
   ended "at" its target: send that hand 1-2 cm further in. Lift only when the last move of each
   hand said "short of target ... something stopped it". (L0-L15 r2; 4/4)
4. Lift: goto('both', by=(0, 0, 0.12)). A pressing hand may stop the cell with Blocked though the
   box rose: look at the head picture, box level and clear. (L0, L5 r2)
5. Carry by turning the waist 40 degrees toward the named side: waist(by=40) for left,
   waist(by=-40) for right. The hands keep their grip and the box swings with them; the crossing
   hand's reach no longer limits the carry. (L10 r2: 39 degrees, -19 to +19, moved it 49 cm; 1/1)
6. Dry-run, then lower both straight down by the lift height: goto('both', by=(0, 0, -0.12)).
   (L10 r2, after pulling both 5 cm toward the robot; 1/1)
## Verified parameters
- Grasp within 2 cm of the box's middle front to back, both hands at one pitch: it stayed level
  (L5, L15 r2 at pitch 0; L10 r2 at pitch 25, 2.5 cm above its measured centre).
- Contact targets 1-2 cm inside the faces: the repeated presses stopped 14-46 mm short and held
  the box through lift and carry (L0-L15 r2).
- Lift 12-13 cm: stage 1 passed every time (L0-L15, both rounds; 8/8).
- Arm-only carries: the crossing hand was refused low poses farther than 28-30 cm to the side,
  so the box went 25-33 cm (L5, L15 both rounds; 4/4).
## Common failures and fixes
- The large carton (L0, measured 33 x 20 x 21 cm) tipped forward on the lift in both rounds,
  palms 3 cm nearer than its middle (r1) or at its measured middle and centre (r2). Fix: grasp
  4-5 cm above its measured centre, at its middle, each hand pressing 2-3 cm in (untested).
- Hands pitched 10 and 15, 5 cm nearer than the middle: the box stood on an edge (L10 r1).
- Diagonal carry, 16 cm toward the robot and 29 cm to the side: the box slid toward the leading
  palm, pushed it 6 cm out and turned (L5 r2 t13). Fix: Flow 5.
- One palm "at" its target when lifting: the palms rose without the box (L0 r1 t4) or it slid
  out after the carry (L15 r1). Fix: Flow 3.
## Final checks
- Before lifting: the last contact move of each hand said "short of target" (L0-L15 r2).
- Before lowering: head picture shows the whole box level, over the table on the named side,
  about 40 cm or more from where it was lifted (L10 r2).
