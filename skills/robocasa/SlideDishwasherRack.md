# SlideDishwasherRack

## Judging
- Only the TOP rack counts: "out" = pulled right out toward the robot, to its stop; "in" =
  pushed right in, to its stop. It stays where left; the hand may stay on it.
- "out" starts about half out: pull it the rest of the way (some 10-20 cm). "in" starts fully
  out: push it all the way (some 20-40 cm), into its stop. Still running: keep going along x.
- The door is open, lying low in front. The rack's front wall is wire, ~15 cm tall; its top wire
  was some 20-30 cm below the tub's top front edge (4 eps). `robot.to_world` both.

## Flow
1. Find the open door lying ~0.12 m above the floor in the start pictures: the dishwasher is
   about half a metre to that side (right 7 eps, left 3 eps). `robot.drive(forward=-0.25)` (23
   steps), then sideways until in front of it (33-35 steps), no turn (10 eps).
2. "out", base 0.55-0.57 m behind the wire (2 eps), the closed hook: grip='close', pitch 70, yaw 0,
   ~6 cm in front of the wire and 7-13 cm above its top (39-54 steps); to 2 cm past the wire and
   3 cm above its top (15-16); down to 1-3 cm below its top (10); `robot.goto(by=(-0.08, 0, 0))`
   (12), then by=(-0.08 to -0.10, 0, 0): success during the second (2 eps).
3. "in": `robot.drive(forward=0.08)` to 0.12, so the base is 0.31-0.39 m behind the wire (6 eps).
   Closed, pitch 90, yaw 0, 2-3 cm in front of the wire, 8-10 cm above it (45-62 steps); down to
   1-1.5 cm below its top (12-25); `robot.goto(by=(0.14, 0, 0))` (17-23 steps, 6 eps).
4. Push on at pitch 90 until short, ~0.51 m ahead of the base (2 eps; one ep succeeded in this leg
   at 0.60). Then pitch 70, yaw 0, 2-3 cm back, at the wire's top height (11-12 steps); then pitch
   65, 1-1.5 cm below its top, on to 0.60-0.64 m ahead of the base: success (3 eps, 4-15 steps).

## Verified parameters
- The closed hook needs no grip: the fingertips behind the wire pulled it 0.13-0.16 m (2 eps).
- The open door stopped the base 0.10-0.16 m after the back-off ("held off its path", 3 eps).

## Common failures and fixes
- An open pinch from above (pitch 90, yaw +-90): short of the wire or BLOCKED on the edge each
  time, 62-170 steps lost (3 eps). Use the closed hook.
- "out", wire only 19-20 cm below the edge: the pitch-70 hook stopped 2 cm above the wire's top at
  the same place on every try, also from 0.10 m closer: the hand meets the edge (2 eps, both
  failed). What reached the wire there, after 380+ steps: the hook with roll 180, 2.6 cm below its
  top (1 of 2 eps); open, pitch 70, roll 90, fingers above and below the wire (1 ep). Use them at
  the first such stop. Pull in 0.08 legs (a 0.18 leg was refused, 1 ep), or drive back (untested).
- "in": driving the base closer after a push made pitch 60-65 stop with joint 6 at its end
  0.48-0.52 m ahead (2 eps). Push from the first place, 0.31-0.39 m behind the wire's start.
- "in" stuck at pitch 65, 0.55-0.62 m ahead (2 eps): roll 180, yaw 25, pitch 50-85 were short or
  BLOCKED, or pushed the base (175-190 steps each). One ended in success on a small move later.

## Typical step costs
- Horizon 450, 40 replies. "out" 157-245 in all: drives 56, hook 64-80, pulls 20-21 (2 eps). "in"
  191-265: drives 56-71, pre-pose 45-62, down 12-25, pushes 23-60 (6 eps). Wrong way first: ~57 (3).
