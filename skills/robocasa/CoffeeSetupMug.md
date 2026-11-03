# CoffeeSetupMug
## Judging
- Done when the mug stands under the outlet, centred to a few cm, and the hand has moved well
  away from it. The spot: about 8 cm in front of the back wall under the outlet (prior tool). It
  fires on the move away (29 eps).
## Flow
1. Find S before driving (looks cost no steps). The outlet is under the round holder that a handle
   points at you from, or at the spout tips under the head; not a gauge, a ring under one, a screen
   or a button (beside the holder the mug hit its handle's end 9-13 cm short: 3 eps). robot.to_world
   on the back wall under it and on it (part words gave nothing or the whole machine: 4 eps); S =
   7.7 cm in front of that wall (prior tool; 12 eps), within 2 cm when taken at the holder (3 eps).
2. A jug or other part 2+ cm over the tray within 10 cm left of S: at yaw -60 the palm's left
   corner, 7 cm deeper and 9 cm left of the mug's axis, hit it (2 eps). There: drive 0.25 m back,
   grasp at yaw -45, turn to yaw -15 on the way to the staging point, slide (prior tool, 4/4 there).
3. m = robot.measure('mug'); w = min(m.size[0], m.size[1]), r = w/2; the axis is m.center moved
   (larger - w)/2 away from the handle, which heads -85..-135 (prior tool; 5 eps). Then
   robot.drive(forward=-0.15) (step 2: -0.25). Level side grasp (prior tool, 48/50; 29 eps): pitch
   0, yaw = the handle's heading + 60 in -70..-45 (-60 if unsure), u = (cos yaw, sin yaw); g =
   axis - d*u, d = r - 0.021, z = m.on_z + 0.046; w 8+ cm: d = r - 0.023, z = max(z, m.top - 0.02).
   Any grasp: close 2-5 cm under the top: 0 of 27 slipped at the lift, 3 of 12 nearer it (28 eps).
4. Roll 0 only for a mug 7+ cm left of the base's line, else 180: roll 0 nearer stopped at joint 7
   in 5 of 6 (62-104 steps to redo each); 180 never did, and held as the redo in 5 of 5 (20 eps).
5. Hand 8-18 cm above p = g - 0.09*u, grip='open'; down to p; along u to g; robot.gripper('close')
   (6.1-7.7 cm held: 29 eps); lift till the bottom is 3 cm over the tray top (prior tool). Pitch 0
   short at a joint limit: pitch 15 (2 eps). Refused: yaw -45 or -70, base 0.05 back (prior tool).
6. A = S moved 2 cm left, away from the handle; hand target = A - d*u at the slide's yaw. Axis on
   or right of S: a tube or steam wand right of the outlet met the rim (2 eps); 1.7-3.5 cm left won
   (3 eps). From the target less 12 cm in x (rise only there: 3 eps), bottom 1.2 cm over the tray
   top (3 cm let a wand in: 1 ep), in at speed 0.08, then down at 0.05 till it stops (19 eps).
7. Open only with the mug under the outlet in the wrist picture (else it tipped: 3 eps); then
   `goto(by=...)` 6 cm along -u, 15 along -x, 20 up (19 eps), not a drive or rise first (4 eps).
## Verified parameters
- Base 0.15 m back, g 0.39-0.47 m ahead of it (29 eps; prior tool 0.10-0.30); level at yaw 0
  0.42-0.63 m ahead: refused (5 eps). Holding, a 0.08 drive forward carried it in (5 eps).
## Common failures and fixes
- Up to 4.5 cm short over the tray: lower it there at 0.05 and release as in 7 (6 eps; prior tool).
- BLOCKED farther or off the tray: turning in the bay placed 0 of 4; backing off at the pressed
  height pulled the mug out in 3 of 9 (5 eps). Rise 0.8 cm, back 4 cm along -x, go to the staging
  point, fix the cause (the handle's end: step 1; palm on a part left: step 2), slide (prior tool).
- Released, not fired: measure it once, keep S, regrasp at roll 180, move it by the miss (2 eps,
  188-239 steps). A new outlet pick there sent it 9.5 cm off (1 ep).
- No level pose: from above at pitch 30-90, w under 7.6 cm, 4+ cm free over the rim (10 eps).
## Typical step costs
- Horizon 600, 40 replies. Clean: 204-262 steps, 7-12 replies (10 eps). All 7 losses ran out of
  steps on detours (7 eps): joint-7 redo 62-104, robot.home() 51-116 (none won), re-placing 188-239.
